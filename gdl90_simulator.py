#!/usr/bin/env python3
#
# Copyright 2026 Clyde Stubbs
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
GDL90 / SkyEcho simulator.

A stand-alone, dependency-free port of the Kotlin ``Gdl90Simulator`` to Python 3.
It broadcasts simulated GDL90 traffic (heartbeat, ownship, status, ForeFlight ID
and 1090ES traffic reports) over UDP, and emulates a SkyEcho device by exposing a
WebSocket interface (a ``/`` status stream and a ``/stats`` stream).

The GDL90 wire format, message layout, CRC and byte-stuffing all match the Kotlin
implementation in ``common`` so that the two produce byte-identical output for the
same inputs.

Runs on macOS and Linux with a stock Python 3 (>= 3.8); no third-party packages.

Usage::

    python3 gdl90_simulator.py                     # discover clients, WS on :7070
    python3 gdl90_simulator.py --target 127.0.0.1  # unicast to a fixed host
    python3 gdl90_simulator.py --broadcast         # subnet-broadcast the GDL90 data

The ``GDL90_UNICAST`` environment variable is honoured for parity with the Kotlin
app (comma separated ``host[:ignored]`` targets).
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import hashlib
import json
import math
import os
import random
import socket
import struct
import time
from dataclasses import dataclass, field

# ---------------------------------------------------------------------------
# Constants (from the Kotlin app / common module)
# ---------------------------------------------------------------------------

MAX_PACKET_SIZE = 508          # max size of a UDP data packet
WS_PORT = 7070                 # websocket port
OWN_ADDRESS = 0x7C0000         # our ICAO address
GDL90_PORT = 4000              # port clients listen on for GDL90 data
DISCOVERY_PORTS = (47578, 63093)

EARTH_RADIUS = 6371.01 * 1000  # metres (spherical model used for translate)

INVALID_DATA = -65536.0
VS_NO_DATA = -2048  # 0x800 as a 12-bit two's complement value: GDL90 vertical-velocity "no data" sentinel

# Unit ratios (value_in_base = value * ratio); base units are metres, m/s, etc.
FOOT = 0.3048
KNOT = 1852.0 / 3600
FPM = 0.3048 / 60
NM = 1852.0
ATM = 101325.0                 # pascals


def feet_to_m(ft: float) -> float:
    return ft * FOOT


def m_to_feet(m: float) -> float:
    return m / FOOT


def knots_to_mps(kt: float) -> float:
    return kt * KNOT


def fpm_to_mps(fpm: float) -> float:
    return fpm * FPM


def altitude_to_pressure_hpa(altitude_m: float) -> float:
    """Pressure altitude (m) -> pressure (hPa). Mirrors PressureSource.toPressure."""
    if altitude_m == INVALID_DATA:
        return INVALID_DATA
    pascals = (1 - altitude_m / 44307.6939) ** 5.2553026 * ATM
    return pascals / 100.0


# ---------------------------------------------------------------------------
# Bit / byte helpers (mirror com.controlj.utility.Bits)
# ---------------------------------------------------------------------------

def byte_of(value: int, n: int) -> int:
    """The n-th byte (0 = least significant) of an int, as an unsigned 0..255.

    Matches Kotlin ``Int.byte(n)`` semantics for negative values too, because
    ``(v >> 8n) & 0xFF`` yields the two's-complement byte in Python.
    """
    return (value >> (8 * n)) & 0xFF


def nibble_of(value: int, pos: int, size: int = 4) -> int:
    return (value >> (pos * size)) & ((1 << size) - 1)


def nibbles_to_byte(nibble0: int, nibble1: int = 0) -> int:
    return (nibble0 + (nibble1 << 4)) & 0xFF


def bits_to_byte(*bits: bool) -> int:
    """Assemble a byte from bits given LSB first."""
    result = 0
    for pos, b in enumerate(bits):
        if b:
            result |= 1 << pos
    return result & 0xFF


# ---------------------------------------------------------------------------
# CRC-16 and GDL90 framing (mirror com.controlj.network.CRC16 / Stratux)
# ---------------------------------------------------------------------------

def _build_crc_table() -> list[int]:
    table = []
    for i in range(256):
        crc = i << 8
        for _ in range(8):
            crc = (crc << 1) ^ (0x1021 if (crc & 0x8000) else 0)
        table.append(crc & 0xFFFF)
    return table


_CRC_TABLE = _build_crc_table()

FLAG_BYTE = 0x7E
CONTROL_BYTE = 0x7D


def crc16(data: bytes) -> int:
    acc = 0
    for b in data:
        acc = (_CRC_TABLE[acc >> 8] ^ ((acc << 8) & 0xFFFF) ^ (b & 0xFF)) & 0xFFFF
    return acc


def add_crc16(data: bytes) -> bytes:
    """Append the little-endian CRC-16 to the message body."""
    crc = crc16(data)
    return data + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def encode(databytes: bytes) -> bytes:
    """Wrap a message body (CRC added here) with flag bytes and byte-stuffing."""
    out = bytearray([FLAG_BYTE])
    for b in add_crc16(databytes):
        if b in (FLAG_BYTE, CONTROL_BYTE):
            out.append(CONTROL_BYTE)
            out.append(b ^ 0x20)
        else:
            out.append(b)
    out.append(FLAG_BYTE)
    return bytes(out)


# ---------------------------------------------------------------------------
# Geometry (mirror com.controlj.data.Position)
# ---------------------------------------------------------------------------

def translate(lat: float, lon: float, bearing: float, distance: float) -> tuple[float, float]:
    """Spherical great-circle translation. Returns (lat, lon) in degrees."""
    o1 = math.radians(lat)
    l1 = math.radians(lon)
    t = math.radians(bearing)
    d = distance / EARTH_RADIUS

    o2 = math.asin(math.sin(o1) * math.cos(d) + math.cos(o1) * math.sin(d) * math.cos(t))
    l2 = l1 + math.atan2(
        math.sin(t) * math.sin(d) * math.cos(o1),
        math.cos(d) - math.sin(o1) * math.sin(o2),
    )
    l2h = (l2 + 3 * math.pi) % (2 * math.pi) - math.pi
    return math.degrees(o2), math.degrees(l2h)


def distance_to(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Vincenty inverse solution (metres), matching Position.getDistanceBearing."""
    l1 = math.radians(lon1)
    l2 = math.radians(lon2)
    o1 = math.radians(lat1)
    o2 = math.radians(lat2)

    a = 6378137.0
    b = 6_356_752.314245
    f = 1 / 298.257223563

    big_l = l2 - l1
    tan_u1 = (1 - f) * math.tan(o1)
    cos_u1 = 1 / math.sqrt(1 + tan_u1 * tan_u1)
    sin_u1 = tan_u1 * cos_u1
    tan_u2 = (1 - f) * math.tan(o2)
    cos_u2 = 1 / math.sqrt(1 + tan_u2 * tan_u2)
    sin_u2 = tan_u2 * cos_u2

    lam = big_l
    iteration_limit = 100
    cos_sq_alpha = 0.0
    sin_rho = 0.0
    cos_rho = 0.0
    rho = 0.0
    cos2_alpha_m = 0.0
    while True:
        sin_l = math.sin(lam)
        cos_l = math.cos(lam)
        sin_sq_rho = (cos_u2 * sin_l) ** 2 + (cos_u1 * sin_u2 - sin_u1 * cos_u2 * cos_l) ** 2
        sin_rho = math.sqrt(sin_sq_rho)
        if sin_rho == 0.0:
            return 0.0
        cos_rho = sin_u1 * sin_u2 + cos_u1 * cos_u2 * cos_l
        rho = math.atan2(sin_rho, cos_rho)
        sin_alpha = cos_u1 * cos_u2 * sin_l / sin_rho
        cos_sq_alpha = 1 - sin_alpha * sin_alpha
        cos2_alpha_m = cos_rho - 2.0 * sin_u1 * sin_u2 / cos_sq_alpha if cos_sq_alpha != 0 else 0.0
        if math.isnan(cos2_alpha_m):
            cos2_alpha_m = 0.0
        c = f / 16 * cos_sq_alpha * (4 + f * (4 - 3 * cos_sq_alpha))
        lam_prev = lam
        lam = big_l + (1 - c) * f * sin_alpha * (
            rho + c * sin_rho * (cos2_alpha_m + c * cos_rho * (-1 + 2.0 * cos2_alpha_m * cos2_alpha_m))
        )
        iteration_limit -= 1
        if abs(lam - lam_prev) <= 1e-12 or iteration_limit <= 0:
            break

    u_sq = cos_sq_alpha * (a * a - b * b) / (b * b)
    big_a = 1 + u_sq / 16384 * (4096 + u_sq * (-768 + u_sq * (320 - 175 * u_sq)))
    big_b = u_sq / 1024 * (256 + u_sq * (-128 + u_sq * (74 - 47 * u_sq)))
    delta_a = big_b * sin_rho * (
        cos2_alpha_m + big_b / 4 * (
            cos_rho * (-1 + 2.0 * cos2_alpha_m * cos2_alpha_m)
            - big_b / 6 * cos2_alpha_m * (-3 + 4.0 * sin_rho * sin_rho) * (-3 + 4.0 * cos2_alpha_m * cos2_alpha_m)
        )
    )
    return b * big_a * (rho - delta_a)


# ---------------------------------------------------------------------------
# VH callsign codes (mirror com.controlj.aviation.VHCodes)
# ---------------------------------------------------------------------------

VH_BASE = 0x7C0000
_CODE_MAP = [chr(c) for c in range(ord('A'), ord('Z') + 1)] + [str(d) for d in range(10)]


def call_from(code: int) -> str:
    chars = []
    c = code - VH_BASE
    for _ in range(3):
        chars.append(_CODE_MAP[c % 36])
        c //= 36
    return "".join(reversed(chars))


# ---------------------------------------------------------------------------
# Emitter / address enumerations (mirror TrafficTarget)
# ---------------------------------------------------------------------------

# AddressType ordinals
ADDR_ADSB_ICAO = 0
ADDR_ADSB_SELF = 1

# EmitterCategory ordinals
EC_NONE = 0
EC_LIGHT = 1
EC_LARGE = 3
EC_HEAVY = 5
EC_FIGHTER = 6
EC_ROTORCRAFT = 7
EC_GLIDER = 9
EC_UAV = 14

TRAFFIC_CATEGORIES = [EC_GLIDER, EC_LARGE, EC_LIGHT, EC_ROTORCRAFT, EC_HEAVY, EC_FIGHTER, EC_UAV]

# NACp accuracy table (metres), mirroring PositionReport.accuracyTable
_ACCURACY_TABLE = [
    INVALID_DATA,
    10.0 * NM, 4.0 * NM, 2.0 * NM, 1.0 * NM, 0.5 * NM, 0.3 * NM, 0.1 * NM, 0.05 * NM,
    30.0, 10.0, 3.0,
]


def accuracy_to_nacp(accuracy: float) -> int:
    """Mirror PositionReport.accuracyToNacP (including its indexOf quirk)."""
    min_diff = min(abs(accuracy - v) for v in _ACCURACY_TABLE)
    return _ACCURACY_TABLE.index(min_diff)


# ---------------------------------------------------------------------------
# Simulation state
# ---------------------------------------------------------------------------

@dataclass
class LocationState:
    latitude: float
    longitude: float
    altitude: float          # metres
    speed: float             # m/s
    track: float             # degrees true
    accuracy_h: float = 6.0
    accuracy_v: float = 9.0

    def extrapolate(self, time_secs: float) -> "LocationState":
        lat, lon = translate(self.latitude, self.longitude, self.track, self.speed * time_secs)
        return LocationState(lat, lon, self.altitude, self.speed, self.track,
                             self.accuracy_h, self.accuracy_v)


@dataclass
class Target:
    latitude: float
    longitude: float
    altitude: float          # metres
    speed: float             # m/s
    track: float             # degrees
    vertical_speed: float    # m/s
    address: int
    category: int
    call_sign: str
    accuracy_h: float
    accuracy_v: float
    timestamp: float         # epoch seconds
    is_extrapolated: bool = False
    turn_rate: float = 0.0   # degrees per second, positive to the right

    def distance(self, own: LocationState) -> float:
        return distance_to(self.latitude, self.longitude, own.latitude, own.longitude)

    def extrapolated(self, at: float) -> "Target":
        duration = at - self.timestamp
        lat, lon, track = fly(self.latitude, self.longitude, self.track, self.turn_rate,
                              self.speed, duration)
        altitude = self.altitude
        if -1000.0 < self.vertical_speed < 1000.0:
            altitude = self.altitude + self.vertical_speed * duration
        return Target(lat, lon, altitude, self.speed, track, self.vertical_speed,
                      self.address, self.category, self.call_sign, self.accuracy_h,
                      self.accuracy_v, at, is_extrapolated=True, turn_rate=self.turn_rate)


FLY_STEP = 0.5  # seconds


def fly(lat: float, lon: float, track: float, turn_rate: float, speed: float,
        duration: float) -> tuple[float, float, float]:
    """Fly at a constant speed and turn rate for duration seconds, backwards if it is
    negative. Returns (lat, lon, track)."""
    direction = 1.0 if duration >= 0 else -1.0
    remaining = abs(duration)
    while remaining > 0:
        dt = min(FLY_STEP, remaining) * direction
        # each step is a straight line along the track half way through it
        middle = track + turn_rate * dt / 2
        lat, lon = translate(lat, lon, middle % 360.0, speed * dt)
        track = (track + turn_rate * dt) % 360.0
        remaining -= abs(dt)
    return lat, lon, track


MAX_DISTANCE = 20000.0
MAX_TARGETS = 8


def new_target(origin: LocationState, now: float) -> Target:
    our_track = round(origin.track)
    bearing = float(random.randrange(our_track - 80, our_track + 80))
    track = (bearing + random.randrange(-10, 10) + 540) % 360.0
    distance = random.uniform(2000.0, MAX_DISTANCE)
    altitude = feet_to_m(random.randrange(-10, 50) * 100.0) + origin.altitude
    lat, lon = translate(origin.latitude, origin.longitude, bearing, distance)
    speed = knots_to_mps(random.randrange(10, 20) * 10.0)
    v_speed = fpm_to_mps(random.randrange(-10, 10) * 100.0)
    code = random.randrange(0x7C0000, 0x7CB640)
    return Target(
        latitude=lat, longitude=lon, altitude=altitude, speed=speed, track=track,
        vertical_speed=v_speed, address=code, category=random.choice(TRAFFIC_CATEGORIES),
        call_sign=call_from(code), accuracy_h=20.0, accuracy_v=20.0, timestamp=now,
    )


THREAT_ADDRESS = 0x7CFFF0   # of the first scenario; the others follow
THREAT_SPEED = 30.0        # m/s, a glider
THREAT_PERIOD = 90.0       # seconds between runs
THREAT_TIME = 45.0         # seconds from the start of a run to the collision


@dataclass
class Scenario:
    """A target that reaches the ownship THREAT_TIME seconds after the start of a run."""
    name: str
    description: str
    # its track when it reaches the ownship, relative to the ownship's track
    final_track: float
    turn_rate: float = 0.0       # degrees per second, positive to the right
    vertical_speed: float = 0.0  # m/s; it starts the climb or descent away from the ownship's height
    # instead of a collision: circle at turn_rate, this far to the right of where the ownship will be
    circle_offset: float | None = None


THREAT_SCENARIOS = [
    Scenario("HEADON", "level, head-on", 180.0),
    Scenario("TURNING", "turning right at 3 degrees per second onto a head-on course", 180.0, turn_rate=3.0),
    Scenario("CLIMB", "head-on, climbing at 2.5 m/s from 112 m below", 180.0, vertical_speed=2.5),
    Scenario("DESCEND", "crossing from the right, sinking at 3 m/s from 135 m above", 270.0,
             vertical_speed=-3.0),
    Scenario("CIRCLING", "circling at 18 degrees per second, 400 m to the right of the ownship's path; "
             "no collision", 0.0, turn_rate=18.0, circle_offset=400.0),
]


def new_threat(origin: LocationState, now: float, index: int) -> Target:
    """The scenario's target, placed by flying it backwards from the point where the ownship will be.
    Each scenario has its own address, and a call sign that is not an instruction."""
    scenario = THREAT_SCENARIOS[index]
    meet = origin.extrapolate(THREAT_TIME)
    if scenario.circle_offset is not None:
        # start on the left of the circle, flying the ownship's way, and turn right round it
        radius = THREAT_SPEED / math.radians(scenario.turn_rate)
        lat, lon = translate(meet.latitude, meet.longitude, (origin.track + 90.0) % 360.0,
                             scenario.circle_offset - radius)
        track = origin.track
    else:
        final_track = (origin.track + scenario.final_track) % 360.0
        lat, lon, track = fly(meet.latitude, meet.longitude, final_track, scenario.turn_rate,
                              THREAT_SPEED, -THREAT_TIME)
    return Target(
        latitude=lat, longitude=lon, altitude=origin.altitude - scenario.vertical_speed * THREAT_TIME,
        speed=THREAT_SPEED, track=track, vertical_speed=scenario.vertical_speed,
        address=THREAT_ADDRESS + index, category=EC_GLIDER, call_sign=f"THREAT{index + 1}",
        accuracy_h=20.0, accuracy_v=20.0, timestamp=now, turn_rate=scenario.turn_rate,
    )


def is_threat(target: Target) -> bool:
    return THREAT_ADDRESS <= target.address < THREAT_ADDRESS + len(THREAT_SCENARIOS)


class Simulation:
    def __init__(self, scenarios: list[int] | None = None, ownship_speed: float = knots_to_mps(150.0)) -> None:
        # the indexes of the threat scenarios to run in turn; none for no threat
        self.scenarios = scenarios or []
        self.threat_runs = 0
        self.threat_started = 0.0
        self.initial = LocationState(
            latitude=-28.149444444444445,
            longitude=151.94305555555556,
            altitude=feet_to_m(2500.0),
            speed=ownship_speed,
            track=0.0,
            accuracy_h=6.0,
            accuracy_v=9.0,
        )
        self.current = self.initial
        self.targets: list[Target] = []

    def tick_location(self, elapsed_secs: float) -> None:
        self.current = self.initial.extrapolate(elapsed_secs)

    def update_traffic(self, now: float) -> None:
        valid = [t.extrapolated(now) for t in self.targets
                 if t.distance(self.current) < MAX_DISTANCE and not is_threat(t)]
        if self.scenarios:
            if now - self.threat_started >= THREAT_PERIOD:
                self.threat_started = now
                index = self.scenarios[self.threat_runs % len(self.scenarios)]
                self.threat_runs += 1
                scenario = THREAT_SCENARIOS[index]
                print(f"Threat run started: THREAT{index + 1} {scenario.name}, {scenario.description}")
                threat = new_threat(self.current, now, index)
            else:
                threat = next(t for t in self.targets if is_threat(t)).extrapolated(now)
            valid.insert(0, threat)
        while len(valid) < MAX_TARGETS:
            valid.append(new_target(self.current, now))
        self.targets = valid


# ---------------------------------------------------------------------------
# GDL90 message encoders (bodies only; framing done by encode())
# ---------------------------------------------------------------------------

def heartbeat_body(timestamp_secs: int, gps_valid: bool = True, utc_ok: bool = True) -> bytes:
    return bytes([
        0x00,
        bits_to_byte(False, False, False, False, False, False, False, gps_valid),
        bits_to_byte(utc_ok, False, False, False, False, False, False, bool(timestamp_secs & 0x10000)),
        byte_of(timestamp_secs, 0),
        byte_of(timestamp_secs, 1),
        0x00,  # (uplinkRecvd << 3) + high bits of totalRecvd; both zero here
        0x00,  # totalRecvd low byte
    ])


def _position_report_body(
    report_type: int,
    lat: float, lon: float, altitude_m: float, speed_mps: float, track: float,
    addr_type: int, address: int, category: int, call_sign: str,
    nac_p: int, nic: int, vertical_speed_mps: float,
    is_heading: bool, is_extrapolated: bool, is_in_flight: bool,
    priority_code: int = 0, alert_status: bool = False,
) -> bytes:
    lat_int = int(lat / 360.0 * 0x1000000)
    lon_int = int(lon / 360.0 * 0x1000000)
    alt_int = round((m_to_feet(altitude_m) + 1000.0) / 25)
    track_valid = track != INVALID_DATA
    m = bits_to_byte(track_valid, is_heading, is_extrapolated, is_in_flight)
    speed_int = round(speed_mps / KNOT)
    vs_int = round((vertical_speed_mps / FPM) / 64) if vertical_speed_mps != INVALID_DATA else VS_NO_DATA

    call_bytes = call_sign[:8].ljust(8).encode("ascii", "replace")
    return bytes([
        report_type,
        nibbles_to_byte(addr_type, 1 if alert_status else 0),
        byte_of(address, 2), byte_of(address, 1), byte_of(address, 0),
        byte_of(lat_int, 2), byte_of(lat_int, 1), byte_of(lat_int, 0),
        byte_of(lon_int, 2), byte_of(lon_int, 1), byte_of(lon_int, 0),
        (alt_int >> 4) & 0xFF, nibbles_to_byte(m, alt_int & 0xF),
        nibbles_to_byte(nac_p, nic),
        (speed_int >> 4) & 0xFF,
        nibbles_to_byte(nibble_of(vs_int, 2), nibble_of(speed_int, 0)),
        byte_of(vs_int, 0),
        int(track / 360 * 256) & 0xFF,
        category & 0xFF,
    ]) + call_bytes + bytes([nibbles_to_byte(priority_code, 0)])


def ownship_report_body(own: LocationState) -> bytes:
    nac_p = accuracy_to_nacp(own.accuracy_h)
    return _position_report_body(
        report_type=10,
        lat=own.latitude, lon=own.longitude, altitude_m=own.altitude,
        speed_mps=own.speed, track=own.track,
        addr_type=ADDR_ADSB_SELF, address=OWN_ADDRESS, category=EC_LIGHT,
        call_sign="SIM", nac_p=nac_p, nic=nac_p,
        vertical_speed_mps=INVALID_DATA,
        is_heading=True, is_extrapolated=False, is_in_flight=True,
    )


def traffic_report_body(t: Target) -> bytes:
    nac_p = accuracy_to_nacp(t.accuracy_h)
    return _position_report_body(
        report_type=20,
        lat=t.latitude, lon=t.longitude, altitude_m=t.altitude,
        speed_mps=t.speed, track=t.track,
        addr_type=ADDR_ADSB_ICAO, address=t.address, category=t.category,
        call_sign=t.call_sign, nac_p=nac_p, nic=nac_p,
        vertical_speed_mps=t.vertical_speed,
        is_heading=False, is_extrapolated=t.is_extrapolated, is_in_flight=True,
    )


def stratux_status_body(num_targets: int) -> bytes:
    version = 1
    rev_code = 0
    gps_fix_ordinal = 2  # ThreeD
    # byte13 flags: gpsFix bit0, gpsFix bit1, ahrs, pressure, cpuTempValid, e978, e1090, gps
    byte13 = bits_to_byte(
        bool(gps_fix_ordinal & 1),  # gpsFix bit0
        bool(gps_fix_ordinal & 2),  # gpsFix bit1
        False,                      # ahrsValid
        True,                       # pressureValid
        False,                      # cpuTempValid
        False,                      # enable978
        True,                       # enable1090
        True,                       # enableGps
    )
    radio_count = 1
    satellites_locked = 7
    satellites_tracked = 10
    cpu_temp_int = round(39.0 * 10.0)
    return bytes([
        83,
        ord('X'),
        1,
        1,  # VERSION
        byte_of(version, 0), byte_of(version, 1), byte_of(version, 2), byte_of(version, 3),
        byte_of(rev_code, 0), byte_of(rev_code, 1), byte_of(rev_code, 2), byte_of(rev_code, 3),
        0,  # enableAhrs
        byte13,
        0,  # byte14
        (radio_count & 3),  # + rY835AI*4 (false)
        satellites_locked & 0xFF,
        satellites_tracked & 0xFF,
        0, 0,                                    # num978
        (num_targets >> 8) & 0xFF, num_targets & 0xFF,   # num1090
        0, 0,                                    # rate978
        (num_targets >> 8) & 0xFF, num_targets & 0xFF,   # rate1090
        (cpu_temp_int >> 8) & 0xFF, cpu_temp_int & 0xFF,
        0,  # numTowers
    ])


def stratux_heartbeat_body(protocol_version: int = 1) -> bytes:
    byte1 = (protocol_version << 2) | 0x02  # gpsEnabled
    return bytes([204, byte1 & 0xFF])


def foreflight_id_body(ws_port: int) -> bytes:
    serial_number = 1234
    device_name = f"Sim:{ws_port}"[:8].ljust(8).encode("ascii", "replace")
    long_name = f"SkyEcho Simulator; websocket port {ws_port}"[:16].ljust(16).encode("ascii", "replace")
    # The Kotlin message allocates a 39-byte buffer but writes only 36 bytes, so the
    # wire form carries three trailing zero-pad bytes after the usesMSL flag.
    return (bytes([101, 0, 1]) + struct.pack("!Q", serial_number)
            + device_name + long_name + bytes([0]) + bytes(3))


# ---------------------------------------------------------------------------
# GPS time helper (seconds since 1980, no leap seconds) — mirror gpsTime()
# ---------------------------------------------------------------------------

def gps_time() -> int:
    return int(time.time()) - 315964800 + 18


# ---------------------------------------------------------------------------
# UDP: client discovery + broadcasting
# ---------------------------------------------------------------------------

class ClientRegistry:
    def __init__(self) -> None:
        self.addresses: set[str] = set()

    def add(self, host: str, note: str = "client") -> None:
        if host not in self.addresses:
            self.addresses.add(host)
            print(f"Added {note} {host}")


class _DiscoveryProtocol(asyncio.DatagramProtocol):
    def __init__(self, registry: ClientRegistry) -> None:
        self.registry = registry

    def datagram_received(self, data: bytes, addr) -> None:
        self.registry.add(addr[0])


async def start_discovery(registry: ClientRegistry) -> None:
    loop = asyncio.get_running_loop()
    for port in DISCOVERY_PORTS:
        try:
            await loop.create_datagram_endpoint(
                lambda: _DiscoveryProtocol(registry),
                local_addr=("0.0.0.0", port),
                reuse_port=hasattr(socket, "SO_REUSEPORT"),
            )
        except OSError as ex:
            print(f"Warning: could not listen for clients on UDP {port}: {ex}")


def build_packets(messages: list[bytes]) -> list[bytes]:
    """Pack framed messages into datagrams of at most MAX_PACKET_SIZE bytes."""
    packets: list[bytes] = []
    buf = bytearray()
    for msg in messages:
        if len(msg) > MAX_PACKET_SIZE:
            continue
        if len(buf) + len(msg) > MAX_PACKET_SIZE:
            packets.append(bytes(buf))
            buf = bytearray()
        buf += msg
    if buf:
        packets.append(bytes(buf))
    return packets


class Broadcaster:
    def __init__(self, gdl90_port: int, broadcast_addr: str | None) -> None:
        self.port = gdl90_port
        self.broadcast_addr = broadcast_addr
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    def send(self, packet: bytes, targets: set[str]) -> None:
        for host in targets:
            try:
                self.sock.sendto(packet, (host, self.port))
            except OSError as ex:
                print(f"Send to {host} failed: {ex}")
        if self.broadcast_addr:
            try:
                self.sock.sendto(packet, (self.broadcast_addr, self.port))
            except OSError as ex:
                print(f"Broadcast to {self.broadcast_addr} failed: {ex}")


async def gdl90_broadcast_loop(sim: Simulation, registry: ClientRegistry,
                               broadcaster: Broadcaster, ws_port: int) -> None:
    ff_id = foreflight_id_body(ws_port)
    stx_hb = stratux_heartbeat_body()
    tick = 0
    while True:
        now = time.time()
        sim.tick_location(float(tick))
        utc_seconds = int(now) % 86400

        messages = [
            encode(heartbeat_body(utc_seconds)),
            encode(ownship_report_body(sim.current)),
        ]
        sim.update_traffic(now)
        messages.append(encode(stratux_status_body(len(sim.targets))))
        messages.append(encode(ff_id))
        messages.append(encode(stx_hb))
        for t in sim.targets:
            messages.append(encode(traffic_report_body(t)))

        targets = set(registry.addresses)
        if targets or broadcaster.broadcast_addr:
            for packet in build_packets(messages):
                broadcaster.send(packet, targets)

        tick += 1
        await asyncio.sleep(1.0)


# ---------------------------------------------------------------------------
# Minimal WebSocket server (SkyEcho emulation) — stdlib only
# ---------------------------------------------------------------------------

_WS_MAGIC = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


def _ws_text_frame(text: str) -> bytes:
    payload = text.encode("utf-8")
    header = bytearray([0x81])  # FIN + text opcode
    n = len(payload)
    if n < 126:
        header.append(n)
    elif n < 65536:
        header.append(126)
        header += struct.pack("!H", n)
    else:
        header.append(127)
        header += struct.pack("!Q", n)
    return bytes(header) + payload


def _ws_close_frame() -> bytes:
    return bytes([0x88, 0x00])


async def _read_http_request(reader: asyncio.StreamReader) -> dict[str, str] | None:
    try:
        raw = await asyncio.wait_for(reader.readuntil(b"\r\n\r\n"), timeout=10)
    except (asyncio.IncompleteReadError, asyncio.TimeoutError, ConnectionError):
        return None
    lines = raw.decode("latin-1").split("\r\n")
    if not lines or not lines[0].startswith("GET"):
        return None
    parts = lines[0].split(" ")
    headers = {"__path__": parts[1] if len(parts) > 1 else "/"}
    for line in lines[1:]:
        if ":" in line:
            k, v = line.split(":", 1)
            headers[k.strip().lower()] = v.strip()
    return headers


def _accept_key(key: str) -> str:
    digest = hashlib.sha1((key + _WS_MAGIC).encode("ascii")).digest()
    return base64.b64encode(digest).decode("ascii")


def _status_message(sim: Simulation, client_count: int) -> str:
    loc = sim.current
    return json.dumps({
        "clientCount": 1,
        "status": {
            "NACp": 8,
            "NIC": client_count,
            "baroAltitude": loc.altitude,
            "callsign": "SIM",
            "gnssAltitude": loc.altitude,
            "gpsFix": 3,
            "gpsSatellites": 7,
            "icaoAddress": OWN_ADDRESS,
            "latitude": loc.latitude,
            "longitude": loc.longitude,
        },
    })


def _stats_message(sim: Simulation) -> str:
    loc = sim.current
    return json.dumps({
        "batteryCapacity_%": 75,
        "host": {
            "GPS": {
                "FixType": 3,
                "GnssAlt": int(loc.altitude * 1000),
                "NumSats": 7,
                "UTCTime": gps_time(),
                "positionAccuracy_mm": int(loc.accuracy_h * 1000),
                "velocityAccuracy_mmps": int(loc.accuracy_v * 1000),
                "verticalAccuracy_mm": int(loc.accuracy_v * 1000),
            },
        },
        "sensor": {
            "baro": {
                "altitude_m": loc.altitude,
                "pressure_mbar": altitude_to_pressure_hpa(loc.altitude),
                "temp_c": 25.0,
            },
        },
    })


def make_ws_handler(sim: Simulation, registry: ClientRegistry):
    async def handler(reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        peer = writer.get_extra_info("peername")
        headers = await _read_http_request(reader)
        if headers is None or "sec-websocket-key" not in headers:
            writer.close()
            return
        path = headers["__path__"]
        accept = _accept_key(headers["sec-websocket-key"])
        response = (
            "HTTP/1.1 101 Switching Protocols\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Accept: {accept}\r\n\r\n"
        )
        writer.write(response.encode("ascii"))
        await writer.drain()
        print(f"WebSocket connected: {peer} path={path}")

        # Drain anything the client sends (incl. close) so we notice disconnects.
        async def drain_client() -> None:
            try:
                while await reader.read(1024):
                    pass
            except (ConnectionError, asyncio.IncompleteReadError):
                pass

        drain_task = asyncio.ensure_future(drain_client())
        try:
            build = _stats_message if path == "/stats" else _status_message
            while not writer.is_closing() and not drain_task.done():
                if build is _stats_message:
                    text = build(sim)
                else:
                    text = build(sim, len(registry.addresses))
                writer.write(_ws_text_frame(text))
                await writer.drain()
                await asyncio.sleep(1.0)
        except (ConnectionError, asyncio.CancelledError):
            pass
        finally:
            drain_task.cancel()
            try:
                writer.write(_ws_close_frame())
                await writer.drain()
            except (ConnectionError, RuntimeError):
                pass
            writer.close()
            print(f"WebSocket closed: {peer}")

    return handler


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

async def async_main(args: argparse.Namespace) -> None:
    scenarios = []
    if args.scenario:
        scenarios = [next(i for i, s in enumerate(THREAT_SCENARIOS) if s.name == args.scenario.upper())]
    elif args.threat:
        scenarios = list(range(len(THREAT_SCENARIOS)))
    sim = Simulation(scenarios, args.ownship_speed / 3.6)
    registry = ClientRegistry()

    # Explicit unicast targets: CLI and GDL90_UNICAST env (parity with Kotlin app).
    for source in (args.target, os.environ.get("GDL90_UNICAST")):
        if source:
            for entry in source.split(","):
                host = entry.strip().split(":")[0]
                if host:
                    try:
                        registry.add(socket.gethostbyname(host), "unicast GDL90 target")
                    except OSError as ex:
                        print(f"Could not resolve target {host}: {ex}")

    broadcast_addr = None
    if args.broadcast is not None:
        broadcast_addr = args.broadcast or "255.255.255.255"
        print(f"Broadcasting GDL90 to {broadcast_addr}:{args.gdl90_port}")

    broadcaster = Broadcaster(args.gdl90_port, broadcast_addr)

    await start_discovery(registry)

    ws_handler = make_ws_handler(sim, registry)
    ws_server = await asyncio.start_server(ws_handler, "0.0.0.0", args.ws_port)
    print(f"SkyEcho WebSocket server on ws://0.0.0.0:{args.ws_port}/ (and /stats)")
    print(f"GDL90 output on UDP port {args.gdl90_port}; discovery on {DISCOVERY_PORTS}")

    async with ws_server:
        await gdl90_broadcast_loop(sim, registry, broadcaster, args.ws_port)


def main() -> None:
    parser = argparse.ArgumentParser(description="GDL90 / SkyEcho simulator")
    parser.add_argument("--gdl90-port", type=int, default=GDL90_PORT,
                        help=f"UDP port to send GDL90 data to (default {GDL90_PORT})")
    parser.add_argument("--ws-port", type=int, default=WS_PORT,
                        help=f"WebSocket port for SkyEcho emulation (default {WS_PORT})")
    parser.add_argument("--target", metavar="HOST[,HOST...]", default=None,
                        help="Explicit unicast target host(s) instead of/along with discovery")
    parser.add_argument("--broadcast", nargs="?", const="", default=None, metavar="ADDR",
                        help="Also subnet-broadcast GDL90 (default 255.255.255.255)")
    parser.add_argument("--threat", action="store_true",
                        help=f"Every {THREAT_PERIOD:.0f} s, add a glider at {THREAT_SPEED:.0f} m/s that reaches the "
                             f"ownship {THREAT_TIME:.0f} s later, taking the scenarios in turn (call signs THREAT1 "
                             "and on): " + "; ".join(f"{s.name} ({s.description})" for s in THREAT_SCENARIOS))
    parser.add_argument("--scenario", choices=[s.name.lower() for s in THREAT_SCENARIOS],
                        type=str.lower, default=None,
                        help="Run only this threat scenario (implies --threat)")
    parser.add_argument("--ownship-speed", type=float, default=knots_to_mps(150.0) * 3.6, metavar="KMH",
                        help="Ownship ground speed in km/h (default 278, 150 kt); about 100 for a glider")
    args = parser.parse_args()

    try:
        asyncio.run(async_main(args))
    except KeyboardInterrupt:
        print("\nShutting down.")


if __name__ == "__main__":
    main()
