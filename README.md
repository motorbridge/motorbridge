# motorbridge

[![Rust](https://img.shields.io/badge/Rust-2021-orange.svg)](https://www.rust-lang.org/)
[![Python](https://img.shields.io/badge/Python-3.10--3.14-blue.svg)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Platforms](https://img.shields.io/badge/Platforms-Linux%20%7C%20Windows%20%7C%20macOS-6f42c1.svg)](README.md#release-and-installation-overview-full-matrix)
[![GitHub Release](https://img.shields.io/github/v/release/tianrking/motorbridge)](https://github.com/tianrking/motorbridge/releases)

Unified CAN motor control stack with a vendor-agnostic Rust core, stable C ABI, and Python/C++ bindings.

> Chinese version: [README.zh-CN.md](README.zh-CN.md)

## Transport Legend

- `[STD-CAN]`: classic CAN path (`socketcan`/`pcan`)
- `[CAN-FD]`: dedicated FD path (`socketcanfd`)
- `[DM-SERIAL]`: Damiao serial-bridge path (`dm-serial`)

Current status:
- `[CAN-FD]` has been integrated as an independent transport path.
- No motor model is officially marked as CAN-FD validated in this repository yet.

## Current Vendor Support

- Damiao:
  - models: `3507`, `4310`, `4310P`, `4340`, `4340P`, `6006`, `8006`, `8009`, `10010L`, `10010`, `H3510`, `G6215`, `H6220`, `JH11`, `6248P`
  - modes: `scan`, `enable`, `disable`, `MIT`, `POS_VEL`, `VEL`, `FORCE_POS`, `set-id`, `set-zero`
- RobStride:
  - models: `rs-00`, `rs-01`, `rs-02`, `rs-03`, `rs-04`, `rs-05`, `rs-06`
  - modes: `scan`, `ping`, `enable`, `disable`, `MIT`, `POS_VEL`, `VEL`, parameter read/write, `set-id`, `zero`
  - host/feedback default: `0xFD` (with `0xFF/0xFE` fallback probing)
  - note: torque/current control is currently parameter-level only (`write-param` on `iq_ref`/limits), not a first-class unified mode
- MyActuator:
  - models: `X8` (runtime string; protocol is ID-based)
  - modes: `scan`, `enable`, `disable`, `stop`, `set-zero`, `status`, `current`, `vel`, `pos`, `version`, `mode-query`
- HighTorque:
  - models: `hightorque` (runtime string; native `ht_can v1.5.5`)
  - modes: `scan`, `read`, `mit`, `pos-vel`, `vel`, `stop`, `brake`, `rezero`
- Hexfellow:
  - models: `hexfellow` (runtime string; CANopen profile)
  - modes: `scan`, `status`, `enable`, `disable`, `pos-vel`, `mit` (via `socketcanfd`)

## Update (2026-04): Damiao / RobStride Capability Convergence

- Damiao production baseline now covers: `scan / enable / disable / MIT / POS_VEL / VEL / FORCE_POS / set-id / set-zero`.
- RobStride production baseline now covers: `scan / ping / enable / disable / MIT / POS_VEL / VEL / parameter read-write / set-id / zero`.
- RobStride default host/feedback path is `0xFD` (with internal fallback probing to `0xFF/0xFE`).
- In RobStride `pos-vel`, `--vel/--kd/--tau` are intentionally ignored and reported as warnings (no hard error).

## Architecture

### Layered Runtime View

```mermaid
flowchart TB
  APP["User Apps (Rust/C/C++/Python/ROS2/WS)"] --> SURFACE["CLI / ABI / SDK / Integrations"]
  SURFACE --> CORE["motor_core (CoreController / bus / model / traits)"]
  CORE --> RX["Background RX worker (default)"]
  CORE --> MANUAL["poll_feedback_once() (manual-compatible)"]
  RX --> CACHE["Latest state cache per motor"]
  MANUAL --> CACHE
  CORE --> DAMIAO["motor_vendors/damiao"]
  CORE --> ROBSTRIDE["motor_vendors/robstride"]
  CORE --> MYACT["motor_vendors/myactuator"]
  CORE --> HIGHTORQUE["motor_vendors/hightorque"]
  CORE --> HEXFELLOW["motor_vendors/hexfellow"]
  CORE --> TEMPLATE["motor_vendors/template (onboarding scaffold)"]
  DAMIAO --> CAN["CAN bus backend"]
  ROBSTRIDE --> CAN
  MYACT --> CAN
  HIGHTORQUE --> CAN
  HEXFELLOW --> CAN
  CAN --> LNX["Linux: SocketCAN"]
  CAN --> WIN["Windows (experimental): PEAK PCAN"]
  CAN --> HW["Physical motors"]
```

### Workspace Topology (Latest)

```mermaid
flowchart LR
  ROOT["motorbridge workspace"] --> CORE["motor_core"]
  ROOT --> VENDORS["motor_vendors/*"]
  ROOT --> CLI["motor_cli"]
  ROOT --> ABI["motor_abi"]
  ROOT --> TOOLS["tools/factory_calib_ui"]
  ROOT --> INTS["integrations/*"]
  ROOT --> BIND["bindings/*"]
  VENDORS --> VD["damiao"]
  VENDORS --> VH["hexfellow"]
  VENDORS --> VHT["hightorque"]
  VENDORS --> VR["robstride"]
  VENDORS --> VM["myactuator"]
  VENDORS --> VT["template"]
  INTS --> ROS["ros2_bridge"]
  INTS --> WS["ws_gateway"]
  BIND --> PY["python"]
  BIND --> CPP["cpp"]
```

### Python Binding Surface (v0.1.7+)

```mermaid
flowchart TB
  PYAPP["Python App"] --> CTL["Controller(...) / from_dm_serial(...) / from_socketcanfd(...)"]
  CTL --> ADD["add_damiao / add_robstride / add_myactuator / add_hightorque / add_hexfellow"]
  ADD --> MOTOR["MotorHandle"]
  MOTOR --> CTRL1["send_mit / send_pos_vel / send_vel / send_force_pos"]
  MOTOR --> CTRL2["ensure_mode / enable / disable / set_zero / stop / clear_error"]
  MOTOR --> FB1["request_feedback()"]
  CTL --> FB2["poll_feedback_once() (backward-compatible)"]
  FB1 --> STATE["get_state() latest cached feedback"]
  FB2 --> STATE
```

- [`motor_core`](motor_core): vendor-agnostic controller, routing, CAN bus layer (Linux SocketCAN / Windows experimental PCAN)
- [`motor_vendors/damiao`](motor_vendors/damiao): Damiao protocol / models / registers
- [`motor_vendors/hexfellow`](motor_vendors/hexfellow): Hexfellow CANopen-over-CAN-FD implementation
- [`motor_vendors/hightorque`](motor_vendors/hightorque): HighTorque native ht_can protocol implementation
- [`motor_vendors/robstride`](motor_vendors/robstride): RobStride extended CAN protocol / models / parameters
- [`motor_vendors/myactuator`](motor_vendors/myactuator): MyActuator CAN protocol implementation
- [`motor_cli`](motor_cli): unified Rust CLI
  - full parameters (English): [`motor_cli/README.md`](motor_cli/README.md)
  - full parameters (Chinese): [`motor_cli/README.zh-CN.md`](motor_cli/README.zh-CN.md)
  - Damiao command/register guide: [`motor_cli/DAMIAO_API.md`](motor_cli/DAMIAO_API.md), [`motor_cli/DAMIAO_API.zh-CN.md`](motor_cli/DAMIAO_API.zh-CN.md)
  - RobStride command/parameter guide: [`motor_cli/ROBSTRIDE_API.md`](motor_cli/ROBSTRIDE_API.md), [`motor_cli/ROBSTRIDE_API.zh-CN.md`](motor_cli/ROBSTRIDE_API.zh-CN.md)
  - MyActuator command/mode guide: [`motor_cli/MYACTUATOR_API.md`](motor_cli/MYACTUATOR_API.md), [`motor_cli/MYACTUATOR_API.zh-CN.md`](motor_cli/MYACTUATOR_API.zh-CN.md)
- [`motor_abi`](motor_abi): stable C ABI
- [`bindings/python`](bindings/python): Python SDK + `motorbridge-cli`
- [`bindings/cpp`](bindings/cpp): C++ RAII wrapper

## Quick Start

Build:

```bash
cargo build
```

Bring up CAN:

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up
ip -details link show can0
```

Quick CAN restart (Linux):

```bash
# default: can0 / 1Mbps / restart-ms=100 / loopback off
IF=can0; BITRATE=1000000; RESTART_MS=100; LOOPBACK=off
sudo ip link set "$IF" down 2>/dev/null || true
if [ "$LOOPBACK" = "on" ]; then
  sudo ip link set "$IF" type can bitrate "$BITRATE" restart-ms "$RESTART_MS" loopback on
else
  sudo ip link set "$IF" type can bitrate "$BITRATE" restart-ms "$RESTART_MS" loopback off
fi
sudo ip link set "$IF" up
ip -details link show "$IF"
```

Damiao CLI:

```bash
cargo run -p motor_cli --release -- \
  --vendor damiao --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 \
  --mode mit --pos 0 --vel 0 --kp 20 --kd 1 --tau 0 --loop 50 --dt-ms 20
```
`[STD-CAN]`

Hexfellow CLI:

```bash
cargo run -p motor_cli --release -- \
  --vendor hexfellow --transport socketcanfd --channel can0 \
  --model hexfellow --motor-id 1 --feedback-id 0 \
  --mode status
```
`[CAN-FD]`

RobStride CLI:

```bash
cargo run -p motor_cli --release -- \
  --vendor robstride --channel can0 --model rs-00 --motor-id 127 \
  --mode vel --vel 0.3 --loop 40 --dt-ms 50
```

HighTorque CLI (native ht_can v1.5.5):

```bash
cargo run -p motor_cli --release -- \
  --vendor hightorque --channel can0 --model hightorque --motor-id 1 \
  --mode read
```

RobStride CLI parameter read:

```bash
cargo run -p motor_cli --release -- \
  --vendor robstride --channel can0 --model rs-00 --motor-id 127 \
  --mode read-param --param-id 0x7019
```

MyActuator CLI:

```bash
cargo run -p motor_cli --release -- \
  --vendor myactuator --channel can0 --model X8 --motor-id 1 --feedback-id 0x241 \
  --mode status --loop 20 --dt-ms 50
```

Unified scan (all vendors):

```bash
cargo run -p motor_cli --release -- \
  --vendor all --channel can0 --mode scan --start-id 1 --end-id 255
```

## Experimental Windows Support (PCAN-USB)

Linux remains the primary target. Windows support is experimental and currently backed by PEAK PCAN (`PCANBasic.dll`).

- Install PEAK PCAN driver + PCAN-Basic runtime on Windows.
- Channel mapping:
  - `can0` -> `PCAN_USBBUS1`
  - `can1` -> `PCAN_USBBUS2`
- Optional bitrate suffix: `@<bitrate>` (for example `can0@1000000`).

Validation commands on Windows:

```bash
# Scan Damiao IDs
cargo run -p motor_cli --release -- --vendor damiao --channel can0@1000000 --model 4340P --motor-id 0x01 --feedback-id 0x11 --mode scan --start-id 1 --end-id 16

# Move motor #1 (4340P) to +pi rad (~180 deg)
cargo run -p motor_cli --release -- --vendor damiao --channel can0@1000000 --model 4340P --motor-id 0x01 --feedback-id 0x11 --mode pos-vel --pos 3.1416 --vlim 2.0 --loop 1 --dt-ms 20

# Move motor #7 (4310) to +pi rad (~180 deg)
cargo run -p motor_cli --release -- --vendor damiao --channel can0@1000000 --model 4310 --motor-id 0x07 --feedback-id 0x17 --mode pos-vel --pos 3.1416 --vlim 2.0 --loop 1 --dt-ms 20
```

## macOS PCAN Runtime (PCBUSB)

This project supports PCAN on macOS via MacCAN's `PCBUSB` runtime.
On macOS, `PCANBasic.dll` is not used.

### 1. Prerequisites

- A PEAK-compatible USB-CAN adapter recognized by macOS.
- `motorbridge` source built on macOS.
- Bundled archive in this repo: `third_party/pcan/macos/macOS_Library_for_PCANUSB_v0.13.tar.gz`.
- Or download directly from GitHub:
  - <https://github.com/tianrking/motorbridge/blob/main/third_party/pcan/macos/macOS_Library_for_PCANUSB_v0.13.tar.gz>

### 2. Quick install from bundled archive (recommended)

Use the helper script from repo root:

```bash
# user-local install (no sudo, recommended)
./scripts/setup_pcbusb_macos.sh --user-local

# system install (uses package install.sh, requires sudo)
./scripts/setup_pcbusb_macos.sh --system
```

If you use `--user-local`, run `motor_cli` with:

```bash
DYLD_LIBRARY_PATH=$HOME/.local/lib ./target/release/motor_cli ...
```

### 3. Manual install PCBUSB (system-wide)

If you want to download manually first:

```bash
mkdir -p /tmp/motorbridge-pcan && cd /tmp/motorbridge-pcan
curl -L -o macOS_Library_for_PCANUSB_v0.13.tar.gz \
  https://raw.githubusercontent.com/tianrking/motorbridge/main/third_party/pcan/macos/macOS_Library_for_PCANUSB_v0.13.tar.gz
```

Then install:

```bash
tar -xzf macOS_Library_for_PCANUSB_v0.13.tar.gz
cd PCBUSB
sudo ./install.sh
```

The installer places:

- `libPCBUSB.dylib` into `/usr/local/lib`
- `PCBUSB.h` into `/usr/local/include`

### 4. Optional user-local install (no sudo)

If your user cannot write to `/usr/local`, use a local runtime path:

```bash
mkdir -p ~/.local/lib ~/.local/include
cp PCBUSB/libPCBUSB.0.13.dylib ~/.local/lib/
ln -sf ~/.local/lib/libPCBUSB.0.13.dylib ~/.local/lib/libPCBUSB.dylib
cp PCBUSB/PCBUSB.h ~/.local/include/
```

Then run `motor_cli` with:

```bash
DYLD_LIBRARY_PATH=$HOME/.local/lib ./target/release/motor_cli ...
```

### 5. Verify runtime loading

```bash
python3 - <<'PY'
from can.interfaces.pcan.basic import PCANBasic
PCANBasic()
print("PCBUSB load OK")
PY
```

If using user-local install:

```bash
DYLD_LIBRARY_PATH=$HOME/.local/lib python3 - <<'PY'
from can.interfaces.pcan.basic import PCANBasic
PCANBasic()
print("PCBUSB load OK")
PY
```

### 6. Build motorbridge CLI

```bash
cargo build -p motor_cli --release
```

### 7. Channel mapping on macOS (PCAN backend)

- `can0` maps to `PCAN_USBBUS1`
- `can1` maps to `PCAN_USBBUS2`
- Optional bitrate suffix is supported (example: `can0@1000000`)

### 8. Scan motors (Damiao)

```bash
./target/release/motor_cli \
  --vendor damiao --channel can0 --mode scan --start-id 1 --end-id 16
```

If using user-local `PCBUSB`:

```bash
DYLD_LIBRARY_PATH=$HOME/.local/lib ./target/release/motor_cli \
  --vendor damiao --channel can0 --mode scan --start-id 1 --end-id 16
```

### 9. Control example (Damiao MIT)

Replace `motor-id` and `feedback-id` with your scan hits.

```bash
./target/release/motor_cli \
  --vendor damiao --channel can0 --model 4310 \
  --motor-id 0x02 --feedback-id 0x12 \
  --mode mit --pos 0 --vel 0 --kp 20 --kd 1 --tau 0 \
  --loop 50 --dt-ms 20
```

### 10. Troubleshooting

- `load PCBUSB failed ...`:
  - Install PCBUSB with `install.sh`, or export `DYLD_LIBRARY_PATH` for local install.
- `No CAN backend for current platform`:
  - Use a build that includes the macOS PCAN backend.
- `hits=0` on scan:
  - Check wiring, power, termination resistor, and CAN bitrate.


## Linux USB-CAN (`slcan`) Quick Guide

Linux uses SocketCAN interface names directly (for example `can0`, `slcan0`).
Do not pass bitrate suffix in Linux channel names (for example `can0@1000000` is invalid on Linux SocketCAN).

Bring up an `slcan` adapter as `slcan0`:

```bash
sudo slcand -o -c -s8 /dev/ttyUSB0 slcan0
sudo ip link set slcan0 up
ip -details link show slcan0
```

Then use `slcan0` as CLI channel:

```bash
cargo run -p motor_cli --release -- --vendor damiao --channel slcan0 --mode scan --start-id 1 --end-id 255
```

## Damiao Dedicated CAN-FD Transport (`socketcanfd`)

Use this Linux-only transport when you want an independent CAN-FD path without changing existing classic CAN or `dm-serial` behavior.

```bash
# Bring up CAN-FD interface first
scripts/canfd_restart.sh can0

# Damiao over dedicated socketcanfd transport
cargo run -p motor_cli --release -- --vendor damiao \
  --transport socketcanfd --channel can0 \
  --model 4310 --motor-id 0x04 --feedback-id 0x14 \
  --mode mit --verify-model 0 --ensure-mode 0 \
  --pos 0.5 --vel 0 --kp 20 --kd 1 --tau 0 --loop 80 --dt-ms 20
```
`[CAN-FD]` (transport integrated; motor verification matrix pending)

## Damiao Serial Bridge Quick Guide (`dm-serial`)

Use this path only when your Damiao adapter exposes a serial bridge (for example `/dev/ttyACM1`) and you want to run Damiao through that private transport:

```bash
# Damiao scan over serial bridge
cargo run -p motor_cli --release -- --vendor damiao \
  --transport dm-serial --serial-port /dev/ttyACM1 --serial-baud 921600 \
  --model 4310 --mode scan --start-id 1 --end-id 16

# Damiao MIT over serial bridge
cargo run -p motor_cli --release -- --vendor damiao \
  --transport dm-serial --serial-port /dev/ttyACM1 --serial-baud 921600 \
  --model 4310 --motor-id 0x04 --feedback-id 0x14 \
  --mode mit --verify-model 0 --ensure-mode 0 \
  --pos 0.5 --vel 0 --kp 20 --kd 1 --tau 0 --loop 80 --dt-ms 20
```
`[DM-SERIAL]`

## CAN Debugging (Professional Playbook)

For deterministic troubleshooting of Linux `slcan` and Windows `pcan`, use:

- [`docs/en/can_debugging.md`](docs/en/can_debugging.md)
- [`docs/zh/can_debugging.md`](docs/zh/can_debugging.md)

Interpretation:

- `vendor=damiao id=<n>` means one Damiao motor is online at motor ID `<n>`.
- `vendor=robstride id=<n> responder_id=<m>` means one RobStride motor responded.
- `vendor=hightorque ... [hit] id=<n> ...` means one HighTorque motor responded via native ht_can v1.5.5.
- `vendor=myactuator id=<n>` means one MyActuator motor responded.
- `hits=<k>` at the end of each scan block is the count of discovered devices.

## ABI and Bindings

- C ABI:
  - `motor_controller_new_socketcan(channel)`
  - `motor_controller_new_dm_serial(serial_port, baud)` (Damiao-only serial bridge; cross-platform, e.g. `/dev/ttyACM0` or `COM3`)
  - Damiao: `motor_controller_add_damiao_motor(...)`
  - Hexfellow: `motor_controller_add_hexfellow_motor(...)` (CAN-FD path via `socketcanfd`)
  - RobStride: `motor_controller_add_robstride_motor(...)`
  - MyActuator: `motor_controller_add_myactuator_motor(...)`
  - HighTorque: `motor_controller_add_hightorque_motor(...)`
- Python:
  - `Controller(channel="can0")`
  - `Controller.from_dm_serial("/dev/ttyACM0", 921600)` (Damiao-only)
  - `Controller.add_damiao_motor(...)`
  - `Controller.add_hexfellow_motor(...)`
  - `Controller.add_robstride_motor(...)`
  - `Controller.add_myactuator_motor(...)`
  - `Controller.add_hightorque_motor(...)`
- C++:
  - `Controller("can0")`
  - `Controller::from_dm_serial("/dev/ttyACM0", 921600)` (Damiao-only)
  - `Controller::add_damiao_motor(...)`
  - `Controller::add_hexfellow_motor(...)`
  - `Controller::add_robstride_motor(...)`
  - `Controller::add_myactuator_motor(...)`
  - `Controller::add_hightorque_motor(...)`

Unified mode IDs for ABI/Bindings (`ensure_mode`):

- `1 = MIT`
- `2 = POS_VEL`
- `3 = VEL`
- `4 = FORCE_POS`

Unified control units:

- position: `rad`
- velocity: `rad/s`
- torque: `Nm`

Vendor-specific protocol naming/mapping and unsupported operations are documented in:

- [`docs/en/abi.md`](docs/en/abi.md)
- [`docs/zh/abi.md`](docs/zh/abi.md)

RobStride-specific ABI/binding helpers include:

- `robstride_ping`
- `robstride_get_param_*`
- `robstride_write_param_*`

## Example Entry Points

- Cross-language index: `examples/README.md`
- C ABI demo: `examples/c/c_abi_demo.c`
- C++ ABI demo: `examples/cpp/cpp_abi_demo.cpp`
- Python ctypes demo: `examples/python/python_ctypes_demo.py`
- Python SDK docs: `bindings/python/README.md`
- C++ binding docs: `bindings/cpp/README.md`

## Release and Package Matrix

### A) GitHub Releases (binary assets)

| Asset | Install / Usage | Platform | Primary Audience | Included Capability |
|---|---|---|---|---|
| `motorbridge-abi-<tag>-linux-x86_64.deb` | `sudo apt install ./motorbridge-abi-<tag>-linux-x86_64.deb` | Linux x86_64 | C/C++ users (Ubuntu/Debian) | `libmotor_abi` + headers + CMake config |
| `motorbridge-abi-<tag>-linux-*.tar.gz` | extract and link manually | Linux x86_64/aarch64 | C/C++ users (non-deb or cross env) | Same ABI payload as `.deb` |
| `motorbridge-abi-<tag>-windows-x86_64.zip` | extract and link/import | Windows x86_64 | C/C++ users | `motor_abi.dll/.lib` + headers + CMake config |
| `motor-cli-<tag>-<platform>.tar.gz/.zip` | run `bin/motor_cli` | Linux/Windows | Field debug / production tooling | Full unified CLI (`scan`, mode control, id ops, etc.) |
| `motorbridge-*.whl`, `motorbridge-*.tar.gz` | `pip install ./...` | depends on wheel tag | Offline Python install from Release assets | Python SDK + `motorbridge-cli` |

### B) PyPI / TestPyPI (Python package channel)

| Channel | Publish Trigger | Python Versions | Platform Matrix | Package Type |
|---|---|---|---|---|
| TestPyPI | `Actions -> Python Publish -> repository=testpypi` | 3.10 / 3.11 / 3.12 / 3.13 / 3.14 | Linux (x86_64, aarch64), Windows (x86_64), macOS (arm64) | wheel + sdist |
| PyPI | tag `vX.Y.Z` or manual `repository=pypi` | 3.10 / 3.11 / 3.12 / 3.13 / 3.14 | Linux (x86_64, aarch64), Windows (x86_64), macOS (arm64) | wheel + sdist |

Install from PyPI:

```bash
pip install motorbridge
```

Fallback source install:

```bash
pip install --no-binary motorbridge motorbridge
```

### C) Functional Scope by Distribution Type

| Distribution Type | Typical Use | What You Can Do |
|---|---|---|
| ABI package (`.deb/.tar.gz/.zip`) | C/C++ integration | Call stable C ABI, use C++ RAII wrapper, embed into native robotics stack |
| Python package (wheel/sdist) | Python app/tooling | Use `Controller/Motor/Mode` APIs and `motorbridge-cli` |
| `motor_cli` binary package | Ops / factory / debugging | Direct CAN operations without Python runtime |

### D) Additional Automated Distribution Channel

| Channel | CI Workflow | Output |
|---|---|---|
| APT repository (GitHub Pages) | `.github/workflows/apt-repo-publish.yml` | `https://<owner>.github.io/<repo>/apt` |

### E) Arch Linux

```bash
yay -Syu libmotorbridge
yay -Syu python-motorbridge
yay -Syu motorbridge
```

[![Packaging status](https://repology.org/badge/vertical-allrepos/motorbridge.svg)](https://repology.org/project/motorbridge/versions)

Notes:
- `.deb` is currently Linux x86_64 oriented; other Linux targets should use ABI `.tar.gz`.
- macOS x86_64 wheels are intentionally not produced in current matrix.
- Device matrix reference: `docs/en/devices.md`.
- Distribution channel automation guide: `docs/en/distribution_channels.md`.
