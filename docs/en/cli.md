# CLI Guide (`motor_cli`)

<!-- channel-compat-note -->
## Channel Compatibility (PCAN + CANable candleLight/gs_usb + Damiao Serial Bridge + DM_Device)

- Linux SocketCAN uses prepared interfaces directly: `can0`, `can1`. For CANable, use candleLight/gs_usb firmware so it appears as a SocketCAN interface such as `can0`.
- Use PCAN or CANable candleLight/gs_usb for standard CAN.
- Damiao-only CAN-FD transport is available in CLI (`--transport socketcanfd`), independent from classic `socketcan`.
- Damiao-only adapter transports are available in CLI: serial bridge (`--transport dm-serial --serial-port /dev/ttyACM0 --serial-baud 921600`) and DM_Device SDK (`--transport dm-device --dm-device-type usb2canfd|usb2canfd-dual|linkx4c --dm-channel 0|1|2|3`; Damiao motors only; adapter must be in USB mode).
- On Linux SocketCAN, do not append bitrate in `--channel` (for example `can0@1000000` is invalid).
- On Windows (PCAN backend), `can0/can1` map to `PCAN_USBBUS1/2`; optional `@bitrate` suffix is supported.

Transport legend:
- `[STD-CAN]` => `--transport auto|socketcan`
- `[CAN-FD]` => `--transport socketcanfd`
- `[DM-SERIAL]` => `--transport dm-serial`
- `[DM-DEVICE]` => `--transport dm-device`

`[CAN-FD]` note: integrated transport path, but motor validation matrix is not declared yet.

## Debugging Guide

- For deterministic PCAN + CANable candleLight/gs_usb troubleshooting, see [can_debugging.md](can_debugging.md).

## Build

```bash
cargo build -p motor_cli --release
```

## Common

- `--vendor damiao|robstride|hightorque|myactuator|hexfellow|all`
- `--transport auto|socketcan|socketcanfd|dm-serial|dm-device` (`dm-serial`/`dm-device` are Damiao-only; `socketcanfd` required for Hexfellow)
- `--channel can0`
- `--serial-port /dev/ttyACM0 --serial-baud 921600` (used with `--transport dm-serial`)
- `--dm-device-type usb2canfd|usb2canfd-dual|linkx4c --dm-channel 0|1|2|3` (used with `--transport dm-device`; in scan mode, omit `--dm-channel` to scan all channels on the selected adapter)
- `--motor-id <id>`
- `--loop <n> --dt-ms <ms>`

## Damiao

```bash
cargo run -p motor_cli --release -- \
  --vendor damiao --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 \
  --mode mit --pos 0 --vel 0 --kp 20 --kd 1 --tau 0 --loop 50 --dt-ms 20
```
`[STD-CAN]`

```bash
# Damiao over serial bridge
cargo run -p motor_cli --release -- \
  --vendor damiao --transport dm-serial --serial-port /dev/ttyACM1 --serial-baud 921600 \
  --model 4310 --motor-id 0x04 --feedback-id 0x14 \
  --mode mit --verify-model 0 --ensure-mode 0 \
  --pos 0.5 --vel 0 --kp 20 --kd 1 --tau 0 --loop 80 --dt-ms 20
```
`[DM-SERIAL]`

```bash
# Damiao over dedicated CAN-FD transport
cargo run -p motor_cli --release -- \
  --vendor damiao --transport socketcanfd --channel can0 \
  --model 4310 --motor-id 0x04 --feedback-id 0x14 \
  --mode mit --verify-model 0 --ensure-mode 0 \
  --pos 0.5 --vel 0 --kp 20 --kd 1 --tau 0 --loop 80 --dt-ms 20
```
`[CAN-FD]`

## Hexfellow

```bash
# Hexfellow scan (CAN-FD path)
cargo run -p motor_cli --release -- \
  --vendor hexfellow --transport socketcanfd --channel can0 \
  --mode scan --start-id 1 --end-id 32
```
`[CAN-FD]`

```bash
# Hexfellow status query
cargo run -p motor_cli --release -- \
  --vendor hexfellow --transport socketcanfd --channel can0 \
  --model hexfellow --motor-id 1 --feedback-id 0 \
  --mode status
```
`[CAN-FD]`

## RobStride

```bash
cargo run -p motor_cli --release -- \
  --vendor robstride --channel can0 --model rs-00 --motor-id 127 --mode ping
```

```bash
cargo run -p motor_cli --release -- \
  --vendor robstride --channel can0 --model rs-00 --motor-id 127 \
  --mode mit --ensure-strict 1 --pos 0.5 --vel 0 --kp 20.0 --kd 0.5 --tau 0 --loop 100 --dt-ms 20
```

```bash
cargo run -p motor_cli --release -- \
  --vendor robstride --channel can0 --model rs-00 --motor-id 127 \
  --mode pos-vel --pos 1.5 --vlim 1.0 --loc-kp 5.0 --loop 1 --dt-ms 20
```

## HighTorque (native `ht_can` v2.0.0)

Supported modes:

- `scan`
- `read` / `ping`
- `mit` (unified interface)
- `pos` / `vel` / `tqe` (torque mode `05 13`: `--tau <Nm>` physical units, model torque-coeff compensated)
- `pos-vel-tqe`
- `pos-vel-acc` (trapezoid `07 2d`, firmware v4.6.0+): `--pos <rad> --vlim <rad/s> --acc <rad/s^2>`
- `volt` (DQ voltage mode `01 00 08 05 1B`): `--volt <V>` (0.1 V/LSB, Q component only per protocol)
- `cur` (DQ current mode `01 00 09 05 1C`): `--cur <A>` (0.1 A/LSB)
- `set-mode` (run mode switch `01 00 <mode>`): `--run-mode <0-17>` (table 2; 0 = stop & clear error, 8 = DQ voltage, 9 = DQ current, 10 = position, 15 = brake)
- `read-reg` (generic register read, `17` family): `--addr <hex|dec>` required, `--count` (1-3, default 1); since 2026-10-22 reads are **fixed Int16** (`--ty` removed, passing it errors out) and return **converted physical values** (pos/vel/acc in rad-family units, torque in N·m via model k, current/voltage/temperature/PWM/PID per their scales; unclassified registers return raw 1:1); each output line carries that row's register name and unit; the address must be in the parameter table (the 43 xlsx-listed registers within 0x00-0x45) — missing (缺号) and out-of-table addresses are rejected for both read and write
- `write-reg` (generic register write `05 <addr> <val16>`): `--addr` required (must also be in the parameter table; missing/out-of-table addresses are rejected), `--value <int16>`, `--config` (0/1; 1 = config class via extended frame, default 0 via the control channel)
- `periodic-report` (periodic status report `05 B4 02 <t_ms>`): `--period-ms` (>=1 enables the motor-initiated status stream, 0 stops; `timed-read` remains a legacy alias)
- `stop` / `brake` (run mode 15, all phases shorted damping brake) / `rezero` / `conf-write`

Unit interface (aligned with other vendors):

- `--pos` in `rad`
- `--vel` in `rad/s`
- `--tau` in `Nm`
- `--kp`, `--kd` are packed into the v2.0.0 MIT bit-packed frame (firmware v4.6.0+; ranges kp ±400, kd ±100; values beyond range saturate)

Raw interface (debug, position/velocity only):

- `--raw-pos`, `--raw-vel` (torque/voltage/current are physical-only `--tau`/`--volt`/`--cur`; encoding is vendor-side)

Examples:

```bash
# Scan IDs
cargo run -p motor_cli --release -- \
  --vendor hightorque --channel can0 --mode scan --start-id 1 --end-id 32
```

```bash
# Read status (prints pos_rad / vel_rad_s)
cargo run -p motor_cli --release -- \
  --vendor hightorque --channel can0 --motor-id 1 --mode read
```

```bash
# Move to +180 deg (pi rad), with velocity/torque limits
cargo run -p motor_cli --release -- \
  --vendor hightorque --channel can0 --motor-id 1 \
  --mode mit --pos 3.1415926 --vel 0.8 --tau 0.8
```

```bash
# Stop
cargo run -p motor_cli --release -- \
  --vendor hightorque --channel can0 --motor-id 1 --mode stop
```

```bash
# Trapezoid move to +90 deg, vlim 1 rad/s, acc 2 rad/s^2
cargo run -p motor_cli --release -- \
  --vendor hightorque --channel can0 --motor-id 1 \
  --mode pos-vel-acc --pos 1.5708 --vlim 1.0 --acc 2.0
```

```bash
# Read 3 consecutive registers (pos/vel/torque; fixed Int16, returns converted physical values rad/rad·s⁻¹/N·m)
cargo run -p motor_cli --release -- \
  --vendor hightorque --channel can0 --motor-id 1 \
  --mode read-reg --addr 0x01 --count 3
```

## MyActuator

```bash
cargo run -p motor_cli --release -- \
  --vendor myactuator --channel can0 --model X8 --motor-id 1 --feedback-id 0x241 \
  --mode status --loop 20 --dt-ms 50
```

```bash
# Set current position as zero (power-cycle actuator to apply persistently)
cargo run -p motor_cli --release -- \
  --vendor myactuator --channel can0 --model X8 --motor-id 1 --feedback-id 0x241 \
  --mode set-zero --loop 1
```

## Unified scan

```bash
cargo run -p motor_cli --release -- \
  --vendor all --channel can0 --mode scan --start-id 1 --end-id 255
```

RobStride focused scan:

```bash
cargo run -p motor_cli --release -- \
  scan --vendor robstride --channel can0 --start-id 1 --end-id 127 \
  --feedback-ids 0xFD,0xFF,0xFE,0x00,0xAA
```

For RobStride, `probe` / `device_id` is the motor ID. `feedback_id` / `host_id` (for example `0xFD`) is the host-side ID, not the motor ID.
RobStride `motor_id` / `device_id` values are validated as `1..255`; `feedback_id` / `host_id` values are validated as `0..255`.
