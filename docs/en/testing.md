# Testing Guide

<!-- channel-compat-note -->
## Channel Compatibility (PCAN + CANable candleLight/gs_usb + MCU Serial Bridge + Damiao Serial Bridge + DM_Device)

- Linux SocketCAN uses prepared interfaces directly: `can0`, `can1`. For CANable, use candleLight/gs_usb firmware so it appears as a SocketCAN interface such as `can0`.
- Use PCAN or CANable candleLight/gs_usb for standard CAN.
- Damiao-only adapter transports are available in CLI: serial bridge (`--transport dm-serial --serial-port /dev/ttyACM0 --serial-baud 921600`) and DM_Device SDK (`--transport dm-device --dm-device-type usb2canfd|usb2canfd-dual|linkx4c --dm-channel 0|1|2|3`; Damiao motors only; adapter must be in USB mode).
- Generic MCU UART-to-CAN bridge: `--transport mcu-serial --serial-port /dev/ttyACM0 --serial-baud 921600`. Note this is a different protocol from the Damiao-only `dm-serial` bridge: `mcu-serial` relays classic CAN frames through the bridge firmware, while `dm-serial` carries the Damiao serial protocol. It covers all classic-CAN vendors (damiao / robstride / myactuator / hightorque; hexfellow is CAN-FD-only and not supported); the bridge firmware proactively reports structured link status on bus-off / TX failure / RX drop, and all four frontends (motor_abi / Python / motor_cli / ws_gateway) support it.
- On Linux SocketCAN, do not append bitrate in `--channel` (for example `can0@1000000` is invalid).
- On Windows (PCAN backend), `can0/can1` map to `PCAN_USBBUS1/2`; optional `@bitrate` suffix is supported.


This project currently focuses on deterministic unit tests for protocol and parsing logic, plus workspace-level compilation checks.

## What Is Covered

- `motor_core`:
  - Windows PCAN channel/bitrate parsing and validation
  - `CoreController` integration tests with fake `CanBus`:
    - duplicate device-id rejection
    - frame routing
    - enable/disable fan-out
    - shutdown lifecycle behavior
- `motor_vendor_damiao`:
  - protocol encode/decode primitives
  - model matching/suggestion logic
- `motor_vendor_robstride`:
  - extended CAN ID build/parse
  - ping/parameter encoding and validation
- `motor_cli`:
  - input parsing helpers and RobStride parameter value parsing

## Run All Tests

```bash
cargo test --workspace --all-targets
```

## Release Test Notes

Every release should add a repeatable release test note that records core, Rust CLI, Python binding/CLI,
hardware-in-the-loop commands, and dangerous command boundaries.

- Current version: [`release_test_notes/0.6.0.md`](../../release_test_notes/0.6.0.md)

## Recommended Local Quality Gate

```bash
cargo check --workspace
cargo test --workspace --all-targets
```

## Hardware-in-the-loop (manual)

Automated tests avoid real CAN hardware. For hardware validation, run:

1. vendor scan
2. enable/disable
3. control mode command
4. feedback/state readback

Use the commands in root `README.md` (Linux) and Windows experimental section (`can0@1000000`) for repeatable checks.

Reliability helper scripts:

- [`tools/reliability/README.md`](../../tools/reliability/README.md)
- `tools/reliability/reliability_runner.py`

## Next Step Improvements

- Expand long-run HIL matrix (different adapters and bus loads)
- Add periodic cross-platform compare-scan jobs with explicit tolerance policy
