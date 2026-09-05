# motorbridge Python SDK

<!-- channel-compat-note -->
## Channel Compatibility (PCAN + CANable candleLight/gs_usb + CAN-FD + Damiao Serial Bridge + DM_Device)

- Linux SocketCAN uses prepared interfaces directly: `can0`, `can1`. For CANable, use candleLight/gs_usb firmware so it appears as a SocketCAN interface such as `can0`.
- Use PCAN or CANable candleLight/gs_usb for standard CAN.
- CAN-FD transport is available both in CLI (`--transport socketcanfd`) and Python SDK (`Controller.from_socketcanfd(...)`), and is required for Hexfellow.
- Damiao-only adapter transports are available in CLI: serial bridge (`--transport dm-serial --serial-port /dev/ttyACM0 --serial-baud 921600`) and DM_Device SDK (`--transport dm-device --dm-device-type usb2canfd|usb2canfd-dual|linkx4c --dm-channel 0|1|2|3`; Damiao motors only; adapter must be in USB mode).
- Damiao-only DM_Device SDK transport is available in CLI
  (`--transport dm-device --dm-device-type usb2canfd|usb2canfd-dual|linkx4c`)
  and Python SDK (`Controller.from_dm_device(...)`). The adapter must be in USB
  mode. Channel mapping: `usb2canfd` => `0`, `usb2canfd-dual` =>
  `0`/`1`, `linkx4c` => SDK channels `0..3`. Linux x86_64
  USB2CANFD_DUAL channel 0/1 and LINKX4C channel `0..3` scans are verified.
- Full Damiao serial-bridge interface list and command patterns are documented in `motor_cli/README.md` (section `3.6` in `motor_cli/README.zh-CN.md`).
- On Linux SocketCAN, do not append bitrate in `--channel` (for example `can0@1000000` is invalid).
- On Windows (PCAN backend), `can0/can1` map to `PCAN_USBBUS1/2`; optional `@bitrate` suffix is supported.


Python binding layer on top of `motor_abi`.

> Chinese version: [README.zh-CN.md](README.zh-CN.md)

## README Navigation (What Each One Is For)

If this is your first time in this folder, read in this order:

1. This file: [README.md](README.md)  
   Purpose: Python binding overview (install, API scope, common commands).
2. [examples/README.md](examples/README.md) (English) / [examples/READMEzh_cn.md](examples/READMEzh_cn.md) (Chinese)  
   Purpose: practical demo index and run instructions (from simplest to advanced).
2.5. [`../motorbridge-docs`](../../../motorbridge-docs)
   Purpose: canonical Mintlify documentation site (tutorial + API style docs).
3. [get_started/README.md](get_started/README.md) / [get_started/README.zh-CN.md](get_started/README.zh-CN.md)  
   Purpose: pip-first onboarding path (install -> scan -> run).
4. [DAMIAO_PYTHON_REFERENCE.zh-CN.md](DAMIAO_PYTHON_REFERENCE.zh-CN.md)  
   Purpose: Damiao Python interface reference (parameter lookup style).
5. [DAMIAO_binding.md](DAMIAO_binding.md)  
   Purpose: Damiao binding implementation notes (design/internal behavior).
6. [README.zh-CN.md](README.zh-CN.md)  
   Purpose: Chinese overview for Chinese-speaking teammates.

Notes:
- If your goal is "run something now", start with `Start Here (Simplest 2 Examples)` in `examples/README.md`.
- If your goal is CLI parameter lookup, see `../../motor_cli/README.md`.

## Scope
Packaging note:

- Current package target version: `0.5.3`.
- Published wheel includes `motor_abi` shared library and `ws_gateway` binary for that platform.
- Published wheels do not bundle the DaMiao DM_Device SDK runtime. When
  `Controller.from_dm_device(...)`, Python CLI `--transport dm-device`, or
  `motorbridge-gateway --transport dm-device` is used, motorbridge resolves the
  current OS/arch runtime. If the runtime is missing, it prints the required
  file name, GitHub download URL, and valid install paths.
  This keeps Linux manylinux wheels compatible with `auditwheel` while keeping
  runtime setup explicit.
- DM_Device runtime controls:
  - Show the required runtime and install paths: `motorbridge-install-dm-device`
  - Download explicitly into the user cache: `motorbridge-install-dm-device --download`
  - Print the resolved path: `motorbridge-install-dm-device --print-path`
  - Use a manually installed SDK runtime: `MOTOR_DM_DEVICE_LIB=/path/to/libdm_device.so`
  - Use the source-tree location: `third_party/dm_device/v1.1.0/<platform>/<arch>/<runtime>`
  - Use an internal mirror for explicit downloads: `MOTOR_DM_DEVICE_DOWNLOAD_BASE_URL=https://.../third_party/dm_device/v1.1.0`
  - Override the cache directory: `MOTOR_DM_DEVICE_CACHE_DIR=/path/to/cache`
- DM_Device runtime support matrix:

| Platform / Arch | Published Python Wheel | DM_Device Runtime Available | Runtime File | OS/runtime ABI notes | Hardware Verified |
| --- | --- | --- | --- | --- | --- |
| Linux x86_64 | yes | yes | `linux/x86_64/libdm_device.so` | needs `libusb-1.0.so.0`, `libstdc++.so.6` with `GLIBCXX_3.4.32`, `GLIBC_2.14+` | yes, USB2CANFD_DUAL channel 0/1 and LINKX4C channel `0..3` scan |
| Linux aarch64 | yes | yes | `linux/arm64/libdm_device.so` | needs `libusb-1.0.so.0`, `GLIBC_2.17+`, `GLIBCXX_3.4.22+` | pending host validation |
| Windows x86_64 | yes | yes | `windows/msvc/dm_device.dll` | needs libusb runtime/driver and Microsoft Visual C++ runtime (`MSVCP140*.dll`, `VCRUNTIME140*.dll`) | pending host validation |
| macOS arm64 | yes | yes | `macos/arm64/libdm_device.dylib` | links system `libc++`, `libSystem`, `libobjc`; final OS floor pending macOS host validation | pending host validation |
| macOS x86_64 | no official wheel | source/manual install only | `macos/x86_64/libdm_device.dylib` | links system `libc++`, `libSystem`, `libobjc`; final OS floor pending macOS host validation | pending host validation |
| Other arch/OS | no | no | none vendored | unsupported | unsupported |

- ABI metadata helpers:
  - `motorbridge.abi_version()` returns the loaded ABI library version.
  - `motorbridge.abi_capabilities()` returns the loaded ABI capability JSON as a Python `dict`.
- `0.4.5` improves Damiao `ensure_mode` readback verification by retrying the
  mode-register write after failed verification attempts. It also documents the
  experimental RobStride `robstride_cia402` and `robstride_mit` CLI paths; those
  paths are for testing/protocol bring-up and are not production-ready yet.
- `0.4.4` adds Damiao `dm-device` transport, Python
  `Controller.from_dm_device(...)`, Python CLI `--transport dm-device`, and
  explicit runtime resolution for `libdm_device.so`/`.dylib`/`.dll` when the
  target platform has a vendored SDK runtime.
- For Python CLI scans with `--transport dm-device`, omit `--dm-channel` to scan
  every channel for the selected adapter: `0` on `usb2canfd`, `0|1` on
  `usb2canfd-dual`, or `0|1|2|3` on `linkx4c`. Pass `--dm-channel ...` to scan
  one physical channel.
- `0.4.2` optimizes Damiao `dm-serial` multi-motor control by making
  `recv(0ms)` non-blocking when no serial bytes are pending and by reducing the
  bounded serial read timeout to 1 ms.
- `0.4.1` adds ABI capability/version discovery and aligns C++ RobStride
  wrapper parity with the Python SDK. Binding parity is tracked in
  `bindings/api_surface.json`.
- After `pip install motorbridge`, gateway binary path is typically:
  `.../site-packages/motorbridge/bin/ws_gateway` (or `ws_gateway.exe` on Windows).
- `0.4.1` includes Damiao `dm-serial` whole-arm scan/session handling in the
  bundled gateway and adds `damiao_state_many` for multi-joint browser
  telemetry.
- `0.3.9` RobStride `request_feedback()` semantics and `0.3.8` PP/CSP-specific
  position-control entrypoints remain available while keeping the public Python
  binding API backward compatible.
- RobStride parameter writes default to no status-ack wait; set
  `MOTORBRIDGE_ROBSTRIDE_WRITE_ACK_TIMEOUT_MS` to restore synchronous waiting.
- The Python CLI is now implemented as a `motorbridge.cli` package, but
  `motorbridge-cli`, `python -m motorbridge.cli`, `python -m motorbridge`,
  `from motorbridge.cli import main`, and legacy flat run arguments remain
  supported.

- Gateway launch command (added to PATH by pip):
  - `motorbridge-gateway -- --bind 127.0.0.1:9002 ...`
  - This launcher executes the packaged Rust `ws_gateway` binary, so WS JSON
    ops such as `state_stream`, `damiao_state_many`, `param_stream`,
    `damiao_param_stream`, and `robstride_param_stream` are supported by the
    bundled gateway version.
- Security note:
  - keep loopback bind (`127.0.0.1`) for local usage.
  - if you bind to non-loopback addresses (`0.0.0.0` or host IP), export `MOTORBRIDGE_WS_TOKEN` before launch.
  - clients must pass the token in `x-motorbridge-token` or `Authorization: Bearer ...`.
- macOS runtime note (only if you see dynamic library load errors):
  - Resolve binary path generically:
    `GW="$(python3 -c "import motorbridge, pathlib; print(pathlib.Path(motorbridge.__file__).resolve().parent/'bin'/'ws_gateway')")"`
  - Use package-local `lib` directory (no machine-specific absolute path):
    `PKG_DIR="$(python3 -c "import motorbridge, pathlib; print(pathlib.Path(motorbridge.__file__).resolve().parent)")"`
    `DYLD_LIBRARY_PATH="$PKG_DIR/lib:${DYLD_LIBRARY_PATH:-}" "$GW" --bind 127.0.0.1:9002 --vendor damiao --channel can0 --model auto --motor-id 0x01 --feedback-id 0x11 --dt-ms 20`


- High-level API: `Controller`, `Motor`, `Mode`
- CLI: `motorbridge-cli`
- Controller constructors:
  - `Controller(channel="can0")` (SocketCAN/PCAN path)
  - `Controller.from_socketcanfd(channel="can0")` (CAN-FD path, required by Hexfellow)
  - `Controller.from_dm_serial(serial_port="/dev/ttyACM0", baud=921600)` (Damiao-only serial bridge)
  - `Controller.from_dm_device(dm_device_type="usb2canfd-dual", dm_channel="0")` / `Controller.from_dm_device(dm_device_type="linkx4c", dm_channel="0")` (Damiao-only DM_Device SDK transport)
- Vendors:
  - Damiao: `add_damiao_motor(...)`
  - Hexfellow: `add_hexfellow_motor(...)`
  - MyActuator: `add_myactuator_motor(...)`
  - RobStride: `add_robstride_motor(...)`
  - HighTorque: `add_hightorque_motor(...)`
- Unified state-query pattern:
  - Recommended flow: `request_feedback() -> poll_feedback_once() -> get_state()`.
  - RobStride has no single-shot private-protocol status request; `request_feedback()` is a non-blocking no-op for RobStride. Use `robstride_ping()` for connectivity, active report for streaming state, or typed parameter reads for fresh position/velocity values.

## Unified Mode Mapping Summary (Top-Level -> Vendor Native)

| Unified Mode | Damiao | RobStride | Hexfellow | MyActuator | HighTorque |
| --- | --- | --- | --- | --- | --- |
| `Mode.MIT` | native MIT | native MIT | native MIT (mode 5) | unsupported | maps to native pos+vel+tqe |
| `Mode.POS_VEL` | native POS_VEL | maps to native Position (`run_mode=1` + `vel_max(0x7024)` + `loc_ref(0x7016)`) | native POS_VEL (mode 1) | Position setpoint flow | maps to native pos+vel+tqe |
| `Mode.VEL` | native VEL | native Velocity | unsupported | native velocity setpoint flow | native velocity command |
| `Mode.FORCE_POS` | native FORCE_POS | unsupported | unsupported | unsupported | maps to native pos+vel+tqe |

Note:

- RobStride unified high-level control currently covers `MIT` / `POS_VEL` / `VEL`.
- Torque/current is parameter-level only for RobStride (`robstride_write_param_*`), not a dedicated unified mode.
- RobStride feedback/host default should use `0xFD`; scan tries `0xFD,0xFF,0xFE,0x00,0xAA` by default.
- RobStride `feedback_id` / `host_id` is not the motor `device_id`; scan hits report the motor ID as `probe` / `device_id`.

## Quick Start

```python
from motorbridge import Controller, Mode

with Controller("can0") as ctrl:
    motor = ctrl.add_damiao_motor(0x01, 0x11, "4340P")
    ctrl.enable_all()
    motor.ensure_mode(Mode.MIT, 1000)
    motor.send_mit(0.0, 0.0, 20.0, 1.0, 0.0)
    print(motor.get_state())
    motor.close()
```

Damiao over serial bridge:

```python
from motorbridge import Controller, Mode

with Controller.from_dm_serial("/dev/ttyACM1", 921600) as ctrl:
    motor = ctrl.add_damiao_motor(0x04, 0x14, "4310")
    ctrl.enable_all()
    motor.ensure_mode(Mode.MIT, 1000)
    motor.send_mit(0.5, 0.0, 20.0, 1.0, 0.0)
    motor.close()
```

RobStride quick use:

```python
from motorbridge import Controller

with Controller("can0") as ctrl:
motor = ctrl.add_robstride_motor(127, 0xFD, "rs-00")
    print(motor.robstride_ping())
    print(motor.robstride_get_param_f32(0x7019))
    motor.close()
```

MyActuator quick use:

```python
from motorbridge import Controller, Mode

with Controller("can0") as ctrl:
    motor = ctrl.add_myactuator_motor(1, 0x241, "X8")
    ctrl.enable_all()
    motor.ensure_mode(Mode.POS_VEL, 1000)
    motor.send_pos_vel(3.1416, 2.0)  # rad / rad/s
    print(motor.get_state())
    motor.close()
```

Hexfellow quick use (CAN-FD only):

```python
from motorbridge import Controller, Mode

with Controller.from_socketcanfd("can0") as ctrl:
    motor = ctrl.add_hexfellow_motor(1, 0x00, "hexfellow")
    ctrl.enable_all()
    motor.ensure_mode(Mode.MIT, 1000)      # Hexfellow supports MIT / POS_VEL
    motor.send_mit(0.8, 1.0, 30.0, 1.0, 0.1)
    print(motor.get_state())
    motor.close()
```

## CLI Examples

Damiao:

```bash
motorbridge-cli run \
  --vendor damiao --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 \
  --mode mit --pos 0 --vel 0 --kp 20 --kd 1 --tau 0 --loop 50 --dt-ms 20

# Rust CLI style ID update is also accepted:
motorbridge-cli run \
  --vendor damiao --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 \
  --set-motor-id 0x02 --set-feedback-id 0x12 --store 1 --verify-id 1
```

RobStride:

```bash
motorbridge-cli run \
  --vendor robstride --channel can0 --model rs-00 --motor-id 127 \
  --mode ping
```

RobStride MIT quick check:

```bash
motorbridge-cli run \
  --vendor robstride --channel can0 --model rs-00 \
  --motor-id 2 --feedback-id 0xFD \
  --mode mit --ensure-strict 1 \
  --pos 0.5 --vel 0 --kp 20.0 --kd 0.5 --tau 0 \
  --loop 100 --dt-ms 20
```

RobStride position target, aligned with the WS gateway native register path
(`vel_max` `0x7024`, `loc_kp` `0x701E`, `loc_ref` `0x7016`):

```bash
motorbridge-cli run \
  --vendor robstride --channel can0 --model rs-00 \
  --motor-id 2 --feedback-id 0xFD \
  --mode pos-vel \
  --pos 1.5 --vlim 1.0 --loc-kp 5.0 \
  --loop 1 --dt-ms 20

motorbridge-cli run \
  --vendor robstride --channel can0 --model rs-00 \
  --motor-id 2 --feedback-id 0xFD \
  --mode pos-vel \
  --pos -1.5 --vlim 1.0 --loc-kp 5.0 \
  --loop 1 --dt-ms 20
```

RobStride parameter read:

```bash
motorbridge-cli robstride-read-param \
  --channel can0 --model rs-00 --motor-id 127 --param-id 0x7019 --type f32

# Rust CLI style is also accepted by Python CLI:
motorbridge-cli run \
  --vendor robstride --channel can0 --model rs-00 --motor-id 127 --feedback-id 0xFD \
  --mode read-param --param-id 0x7019 --param-type f32
```

Damiao parameter read/write:

```bash
motorbridge-cli damiao-read-param \
  --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 \
  --param-id 21 --type f32

motorbridge-cli damiao-write-param \
  --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 \
  --param-id 8 --type u32 --value 0x01 --verify 1
```

Unified scan (all vendors):

```bash
motorbridge-cli scan --vendor all --channel can0 --start-id 0x01 --end-id 0xFF
```

RobStride-focused scan and ID update:

```bash
motorbridge-cli scan \
  --vendor robstride --channel can0 --start-id 1 --end-id 127 \
  --feedback-ids 0xFD,0xFF,0xFE,0x00,0xAA

motorbridge-cli id-set \
  --vendor robstride --channel can0 \
  --motor-id 127 --feedback-id 0xFD \
  --new-motor-id 126 --store 1 --verify 1
```

HighTorque via binding:

```python
from motorbridge import Controller

with Controller("can0") as ctrl:
    motor = ctrl.add_hightorque_motor(1, 0x01, "hightorque")
    motor.send_mit(3.1416, 0.8, 0.0, 0.0, 0.8)  # kp/kd are accepted but ignored by protocol
    motor.request_feedback()
    print(motor.get_state())
    motor.close()
```

HighTorque via Rust CLI:

```bash
cargo run -p motor_cli --release -- \
  --vendor hightorque --channel can0 --motor-id 1 --mode read
```

## Experimental Windows Support (PCAN-USB)

Linux remains the primary target. Windows support is experimental and currently uses PEAK PCAN.

- Install PEAK PCAN driver + PCAN-Basic runtime (`PCANBasic.dll`).
- Use `channel` as `can0@1000000` (maps to `PCAN_USBBUS1` at 1Mbps).

Recommended quick validation with Rust CLI on Windows:

```bash
cargo run -p motor_cli --release -- --vendor damiao --channel can0@1000000 --model 4340P --motor-id 0x01 --feedback-id 0x11 --mode scan --start-id 1 --end-id 16
cargo run -p motor_cli --release -- --vendor damiao --channel can0@1000000 --model 4340P --motor-id 0x01 --feedback-id 0x11 --mode pos-vel --pos 3.1416 --vlim 2.0 --loop 1 --dt-ms 20
cargo run -p motor_cli --release -- --vendor damiao --channel can0@1000000 --model 4310 --motor-id 0x07 --feedback-id 0x17 --mode pos-vel --pos 3.1416 --vlim 2.0 --loop 1 --dt-ms 20
```

Local wheel build (Windows):

```bash
python -m pip install --user wheel
set MOTORBRIDGE_LIB=%CD%\\target\\release\\motor_abi.dll
set MOTORBRIDGE_WS_GATEWAY_BIN=%CD%\\target\\release\\ws_gateway.exe
python -m pip wheel --no-build-isolation bindings/python -w bindings/python/dist
python -m pip install bindings/python/dist/motorbridge-*.whl
```

## Example Programs

- Damiao wrapper demo: `examples/python_wrapper_demo.py`
- Hexfellow CAN-FD demo: `examples/hexfellow_canfd_demo.py` (MIT / POS_VEL only)
- Damiao maintenance demo: `examples/damiao_maintenance_demo.py`
- Damiao register rw demo: `examples/damiao_register_rw_demo.py`
- Damiao dm-serial demo: `examples/damiao_dm_serial_demo.py`
- RobStride wrapper demo: `examples/robstride_wrapper_demo.py`
- Full Damiao mode demo: `examples/full_modes_demo.py`
- Damiao scan / tune / position helpers:
  - `examples/scan_ids_demo.py`
  - `examples/pid_register_tune_demo.py`
  - `examples/pos_ctrl_demo.py`
  - `examples/pos_repl_demo.py`

See [examples/README.md](examples/README.md) (English) or [examples/READMEzh_cn.md](examples/READMEzh_cn.md) (Chinese).

## Damiao Full-Coverage Status

Damiao usage in Python examples is now covered end-to-end:

- control modes: `mit` / `pos-vel` / `vel` / `force-pos`
- transport paths: SocketCAN/PCAN constructor + `from_socketcanfd(...)` + `from_dm_serial(...)` + `from_dm_device(...)`
- maintenance ops: `clear_error`, `set_zero_position`, `set_can_timeout_ms`, `request_feedback`
  - project guard for Damiao set-zero: call `disable()` before `set_zero_position()`
  - no user-facing `ms` parameter for set-zero; core applies fixed `20ms` settle
- register APIs: `get/write f32`, `get/write u32`, `store_parameters`

## RobStride Maintenance Notes

- `clear_error()` is supported through the same unified motor method as Damiao.
- `robstride_set_active_report(True/False)` toggles RobStride comm_type `24` active status reporting.
- With active reporting enabled, background polling can update `get_state()` from incoming status frames without a fresh query command; `request_feedback()` remains available as a compatibility/manual-refresh helper.

## CLI Run-Mode Argument Map

`motorbridge-cli run` keeps one unified command surface, but each vendor/mode
uses only the arguments that its native protocol understands.

| Vendor | Mode | Effective arguments | Notes |
| --- | --- | --- | --- |
| Damiao | `mit` | `--pos --vel --kp --kd --tau` | native MIT frame |
| Damiao | `pos-vel` | `--pos --vlim` | native position-speed frame |
| Damiao | `vel` | `--vel` | native velocity frame |
| Damiao | `force-pos` | `--pos --vlim --ratio` | native force-position frame |
| RobStride | `mit` | `--pos --vel --kp --kd --tau` | native MIT frame |
| RobStride | `pos-vel` | `--pos --vlim --loc-kp` | maps to native Position mode; `--kp` is accepted as a `--loc-kp` fallback |
| RobStride | `vel` | `--vel` | native speed mode |
| HighTorque | `mit` | `--pos --vel --tau` | `--kp/--kd` are accepted for unified signature but ignored by `ht_can v1.5.5` |
| Hexfellow | `mit` | `--pos --vel --kp --kd --tau` | CAN-FD path |
| Hexfellow | `pos-vel` | `--pos --vlim` | CAN-FD path |

For RobStride `pos-vel`, `--vel`, `--kd`, and `--tau` are intentionally
ignored because the firmware path is parameter-based (`limit_spd`, `loc_kp`,
`loc_ref`). The Rust CLI and Python CLI warn when those ignored arguments are
explicitly provided.

## End-to-End Demo Commands

```bash
# Build ABI once
cargo build -p motor_abi --release
export PYTHONPATH=bindings/python/src
export LD_LIBRARY_PATH=$PWD/target/release:${LD_LIBRARY_PATH}

# Damiao wrapper demo
python3 bindings/python/examples/python_wrapper_demo.py \
  --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 \
  --pos 0 --vel 0 --kp 20 --kd 1 --tau 0 --loop 20 --dt-ms 20

# RobStride wrapper demo: ping
python3 bindings/python/examples/robstride_wrapper_demo.py \
  --channel can0 --model rs-00 --motor-id 2 --feedback-id 0xFD --mode ping

# RobStride wrapper demo: clear fault
python3 bindings/python/examples/robstride_wrapper_demo.py \
  --channel can0 --model rs-00 --motor-id 2 --feedback-id 0xFD --mode clear-error

# RobStride wrapper demo: active-report bring-up
python3 bindings/python/examples/robstride_wrapper_demo.py \
  --channel can0 --model rs-00 --motor-id 2 --feedback-id 0xFD \
  --mode write-param --param-id 0x7026 --param-type u16 --param-value 3

python3 bindings/python/examples/robstride_wrapper_demo.py \
  --channel can0 --model rs-00 --motor-id 2 --feedback-id 0xFD \
  --mode active-report --active-report 1

# RobStride wrapper demo: position command
python3 bindings/python/examples/robstride_wrapper_demo.py \
  --channel can0 --model rs-00 --motor-id 2 --feedback-id 0xFD \
  --mode pos-vel --pos 1.5 --vlim 1.0 --loc-kp 5.0 --loop 1 --dt-ms 20

# RobStride wrapper demo: velocity
python3 bindings/python/examples/robstride_wrapper_demo.py \
  --channel can0 --model rs-00 --motor-id 2 --feedback-id 0xFD \
  --mode vel --vel 0.3 --loop 40 --dt-ms 50
```

## Notes

- `id-dump` is a Damiao workflow; `id-set` supports Damiao and RobStride; `scan` supports `damiao|hexfellow|myactuator|robstride|hightorque|all`.
- For RobStride `id-set`, `--new-motor-id` changes `device_id`; `--feedback-id` remains the host-side ID.
- RobStride `motor_id` / `device_id` is validated as `1..255`; `feedback_id` / `host_id` is validated as `0..255` to prevent silent `ctypes` truncation.
- RobStride scan probes each `--feedback-ids` host_id exactly through host-id-specific ABI helpers; invalid host IDs are rejected instead of silently falling back.
- Python CLI and Rust CLI are aligned for the production Damiao and RobStride workflows: scan, enable/disable, control, ID update, parameter read/write, RobStride clear-error, and RobStride active-report. Rust CLI still exposes deeper vendor-specific surfaces for HighTorque/MyActuator/Hexfellow debugging.
- `Mode.MIT` and `send_force_pos` are not available for MyActuator in ABI wrapper.
- Hexfellow supports `MIT` and `POS_VEL` through ABI wrapper; `VEL` and `FORCE_POS` return unsupported.
- Full Damiao tuning reference stays in:
  - [DAMIAO_API.md](DAMIAO_API.md)
  - [DAMIAO_API.zh-CN.md](DAMIAO_API.zh-CN.md)

## PyPI Auto Publish (GitHub Actions)

This repository includes `.github/workflows/pypi-publish.yml`.

- Tag publish policy:
  - push `vX.Y.Z` -> publish the same artifacts to both TestPyPI and PyPI
- Manual publish is still available via workflow `Python Publish`:
  - `testpypi` (only TestPyPI)
  - `pypi` (only PyPI)

### One-time setup (token mode)

1. Create API token on PyPI and add repository secret `PYPI_API_TOKEN`.
2. Create API token on TestPyPI and add repository secret `TEST_PYPI_API_TOKEN`.
3. Keep package version unique for every upload (for example `0.1.6`, `0.1.7`).
