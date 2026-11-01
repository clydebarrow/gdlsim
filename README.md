# GDL90 / SkyEcho Simulator (Python)

A stand-alone, dependency-free Python 3 implementation of a GDL90 simulator. It
broadcasts simulated [GDL90](https://www.faa.gov/sites/faa.gov/files/air_traffic/technology/adsb/archival/GDL90_Public_ICD_RevA.PDF)
traffic over UDP and emulates a uAvionix **SkyEcho** device by exposing a
WebSocket interface.

## Requirements

- Python **3.8+** (uses only the standard library)
- macOS or Linux

No `pip install` step is needed.

## Running

```bash
python3 gdl90_simulator.py
```

This starts:

- **GDL90 broadcast** on UDP port `4000`, sent once per second to any client that
  has been discovered (see below).
- **SkyEcho WebSocket server** on port `7070`, with two streams:
  - `ws://<host>:7070/` — status message (position, callsign, GPS fix), 1 Hz
  - `ws://<host>:7070/stats` — SkyEcho GPS/baro stats, 1 Hz

### Client discovery

Like the original, the simulator listens on UDP ports `47578` and `63093`. Any
device that sends a packet to one of those ports is recorded as a client and
receives the GDL90 stream on port `4000`.

### Sending to a fixed host

When broadcasts are not visible (e.g. an Android emulator behind NAT), target a
host explicitly:

```bash
python3 gdl90_simulator.py --target 127.0.0.1
```

The `GDL90_UNICAST` environment variable is also honoured.

```bash
GDL90_UNICAST=127.0.0.1 python3 gdl90_simulator.py
```

### Subnet broadcast

To spray the GDL90 stream onto the local subnet without discovery:

```bash
python3 gdl90_simulator.py --broadcast                 # 255.255.255.255
python3 gdl90_simulator.py --broadcast 192.168.1.255   # a specific broadcast address
```

## Feeding an Android emulator

An emulator sits behind a NAT and does not see UDP broadcasts, so:

1. Forward a host UDP port to the emulator's GDL90 listen port (4000):
   ```bash
   adb emu redir add udp:4000:4000
   ```
2. Run the simulator unicasting to the redirect:
   ```bash
   python3 gdl90_simulator.py --target 127.0.0.1
   ```

## Options

| Flag | Default | Description |
|------|---------|-------------|
| `--gdl90-port` | `4000` | UDP port the GDL90 data is sent to |
| `--ws-port` | `7070` | WebSocket port for SkyEcho emulation |
| `--target HOST[,HOST...]` | — | Explicit unicast target(s) |
| `--broadcast [ADDR]` | — | Also subnet-broadcast (default `255.255.255.255`) |
| `--threat` | — | For testing collision alarms: every 90 s, add a glider at 30 m/s that reaches the ownship 45 s later, taking the scenarios in turn: `HEADON` (level, head-on), `TURNING` (turning right at 3 degrees per second onto a head-on course), `CLIMB` (head-on, climbing at 2.5 m/s from 112 m below), `DESCEND` (crossing from the right, sinking at 3 m/s from 135 m above) and `CIRCLING` (circling 400 m to the right of the ownship's path, which should raise no alarm). Each has its own address and the call sign `THREAT1` to `THREAT5` |
| `--scenario NAME` | — | Run only this threat scenario (`headon`, `turning`, `climb`, `descend` or `circling`) |
| `--ownship-speed KMH` | `278` | Ownship ground speed (150 kt); about 100 for a glider |

## What is simulated

- **Ownship**: starts at −28.1494°, 151.9431° (SE Queensland), 2500 ft, 150 kt,
  tracking north, moving along a great circle each second.
- **Traffic**: up to 8 targets generated around the ownship (bearing ±80° of our
  track, 2–20 km away, ±5000 ft), extrapolated each second and regenerated as
  they leave a 20 km radius. Callsigns are derived from the Australian VH ICAO
  address block.

Each broadcast contains a Heartbeat, Ownship report, Stratux status, ForeFlight
ID, Stratux heartbeat and one 1090ES traffic report per target, packed into UDP
datagrams of at most 508 bytes.

## License

Licensed under the [Apache License, Version 2.0](LICENSE).
