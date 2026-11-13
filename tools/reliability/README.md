# Reliability Validation (Minimum Loop)

<!-- channel-compat-note -->
## Channel Compatibility (PCAN + CANable candleLight/gs_usb + MCU Serial Bridge + Damiao Serial Bridge)

- Linux SocketCAN uses prepared interfaces directly: `can0`, `can1`. For CANable, use candleLight/gs_usb firmware so it appears as a SocketCAN interface such as `can0`.
- Use PCAN or CANable candleLight/gs_usb for standard CAN.
- Damiao-only serial bridge transport is also available in CLI (`--transport dm-serial --serial-port /dev/ttyACM0 --serial-baud 921600`).
- Generic MCU UART-to-CAN bridge: `--transport mcu-serial --serial-port /dev/ttyACM0 --serial-baud 921600`. Note this is a different protocol from the Damiao-only `dm-serial` bridge: `mcu-serial` relays classic CAN frames through the bridge firmware, while `dm-serial` carries the Damiao serial protocol. It covers all classic-CAN vendors (damiao / robstride / myactuator / hightorque / huayi; hexfellow is CAN-FD-only and not supported); the bridge firmware proactively reports structured link status on bus-off / TX failure / RX drop, and all four frontends (motor_abi / Python / motor_cli / ws_gateway) support it.
- Full Damiao serial-bridge interface list and command patterns are documented in `motor_cli/README.md` (section `3.6` in `motor_cli/README.zh-CN.md`).
- On Linux SocketCAN, do not append bitrate in `--channel` (for example `can0@1000000` is invalid).
- On Windows (PCAN backend), `can0/can1` map to `PCAN_USBBUS1/2`; optional `@bitrate` suffix is supported.


This folder provides practical checks for:

- endurance (long-run control/read loop)
- error/timeout behavior (automated unit tests + manual HIL checks)
- disconnect/recovery (manual HIL steps)
- cross-platform scan consistency (Linux vs Windows)

## 1) Endurance

Run one command repeatedly and generate a JSON report:

Windows (PCAN) example:

```bash
python tools/reliability/reliability_runner.py endurance \
  --command "cargo run -p motor_cli --release -- --vendor damiao --channel can0@1000000 --model 4340P --motor-id 0x01 --feedback-id 0x11 --mode pos-vel --pos 3.1416 --vlim 2.0 --loop 1 --dt-ms 20" \
  --duration-sec 1800 \
  --interval-sec 0.5 \
  --report tools/reliability/reports/windows_endurance_4340p.json
```

Linux (SocketCAN) example (`can0` or `can1`):

```bash
python tools/reliability/reliability_runner.py endurance \
  --command "cargo run -p motor_cli --release -- --vendor damiao --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 --mode pos-vel --pos 3.1416 --vlim 2.0 --loop 1 --dt-ms 20" \
  --duration-sec 1800 \
  --interval-sec 0.5 \
  --report tools/reliability/reports/linux_endurance_4340p.json
```

Template-driven CANable/PCAN regression:

```bash
python tools/reliability/reliability_runner.py endurance \
  --template tools/reliability/templates/linux_socketcan_endurance_4340p.json
```

Available templates:

- `tools/reliability/templates/linux_socketcan_endurance_4340p.json`
- `tools/reliability/templates/linux_socketcan_endurance_rs00_vel.json`

Pass criteria:

- `fail == 0`
- `success_rate == 1.0`

Template thresholds are evaluated automatically:

- `thresholds.max_fail`
- `thresholds.min_success_rate`

## 2) Error/Timeout Injection

Automated coverage is in `cargo test --workspace --all-targets`:

- bus read error path (`CoreController`)
- Damiao register timeout
- RobStride parameter timeout

## 3) Disconnect / Recovery (HIL)

Manual steps:

1. Start a short control loop (`pos-vel` or `vel`).
2. Unplug PCAN-USB (or down Linux CAN interface).
3. Verify command pauses, then auto-reconnect/retry is attempted.
4. Re-plug / recover bus.
5. Verify loop resumes without restarting the process, or re-run scan/control to confirm recovery.

## 4) Cross-Platform Consistency (Scan)

Save scan stdout logs on Linux and Windows, then compare:

```bash
python tools/reliability/reliability_runner.py compare-scan \
  --left-log tools/reliability/reports/linux_scan.log \
  --right-log tools/reliability/reports/windows_scan.log \
  --vendors damiao,robstride \
  --allow-hit-delta 1 \
  --id-mode intersect-nonempty
```

Comparison checks:

- per-vendor `hits` with optional tolerance (`--allow-hit-delta`)
- vendor subset filtering (`--vendors`)
- configurable `id` matching mode (`--id-mode`):
  - `exact`
  - `left-subset`
  - `right-subset`
  - `intersect-nonempty`
