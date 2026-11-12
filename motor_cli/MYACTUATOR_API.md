# MyActuator API and Mode Reference (Complete)

<!-- channel-compat-note -->
## Channel Compatibility (PCAN + CANable candleLight/gs_usb + MCU Serial Bridge + Damiao Serial Bridge + DM_Device)

- Linux SocketCAN uses prepared interfaces directly: `can0`, `can1`. For CANable, use candleLight/gs_usb firmware so it appears as a SocketCAN interface such as `can0`.
- Use PCAN or CANable candleLight/gs_usb for standard CAN.
- Damiao-only adapter transports are available in CLI: serial bridge (`--transport dm-serial --serial-port /dev/ttyACM0 --serial-baud 921600`) and DM_Device SDK (`--transport dm-device --dm-device-type usb2canfd|usb2canfd-dual|linkx4c --dm-channel 0|1|2|3`; Damiao motors only; adapter must be in USB mode).
- Generic MCU UART-to-CAN bridge: `--transport mcu-serial --serial-port /dev/ttyACM0 --serial-baud 921600`. Note this is a different protocol from the Damiao-only `dm-serial` bridge: `mcu-serial` relays classic CAN frames through the bridge firmware, while `dm-serial` carries the Damiao serial protocol. It covers all classic-CAN vendors (damiao / robstride / myactuator / hightorque; hexfellow is CAN-FD-only and not supported); the bridge firmware proactively reports structured link status on bus-off / TX failure / RX drop, and all four frontends (motor_abi / Python / motor_cli / ws_gateway) support it.
- On Linux SocketCAN, do not append bitrate in `--channel` (for example `can0@1000000` is invalid).
- On Windows (PCAN backend), `can0/can1` map to `PCAN_USBBUS1/2`; optional `@bitrate` suffix is supported.


Practical reference for `motor_cli` MyActuator control in `motorbridge`.

## 1) Basics

- Vendor name: `myactuator` (backward-compatible )
- CAN IDs:
  - Tx command: `0x140 + motor_id`
  - Rx feedback: `0x240 + motor_id`
- Typical ID range: `motor_id in [1, 32]`
- Common default for ID=1:
  - `--motor-id 1`
  - `--feedback-id 0x241`

## 2) Supported `motor_cli` Modes

- `scan`: probe IDs in 1..32
- `enable`: release brake (`0x77`)
- `disable`: shutdown motor (`0x80`)
- `stop`: stop closed loop (`0x81`)
- `set-zero`: set current position as encoder zero (`0x64`, persistent after power-cycle)
- `status`: request status-2 (`0x9C`)
- `current`: current closed loop (`0xA1`)
- `vel`: speed closed loop (`0xA2`)
- `pos`: absolute position closed loop (`0xA4`)
- `version`: query version date (`0xB2`)
- `mode-query`: query system operating mode (`0x70`)

## 3) Arguments and Units

| Argument | Type | Default | Used in | Unit |
|---|---|---|---|---|
| `--start-id` | u16 | `1` | `scan` | id |
| `--end-id` | u16 | `32` | `scan` | id |
| `--current` | f32 | `0.0` | `current` | A |
| `--vel` | f32 | `0.0` | `vel` | rad/s |
| `--pos` | f32 | `0.0` | `pos` | rad |
| `--max-speed` | f32 | `8.726646` | `pos` | rad/s |
| `--loop` | u64 | `1` | all | cycles |
| `--dt-ms` | u64 | `20` | all | ms |

## 4) Encoding Details (Current CLI)

CLI input is unified in radians/rad-s, then converted internally to MyActuator protocol units (deg/deg-s).

- `A1` current mode:
  - payload `[4..5] = int16(current / 0.01)`
- `A2` velocity mode:
  - payload `[4..7] = int32((vel_rad_s.to_degrees()) * 100)`
- `A4` absolute position mode:
  - payload `[2..3] = uint16(max_speed_rad_s.to_degrees())`
  - payload `[4..7] = int32(pos_rad.to_degrees() * 100)`

## 5) Feedback Decoding (status/command reply)

For response commands `0x9C`, `0xA1`, `0xA2`, `0xA4`:

- `data[1]`: temperature (int8, °C)
- `data[2..3]`: current (int16 * 0.01 A)
- `data[4..5]`: speed (int16, deg/s)
- `data[6..7]`: shaft angle (int16 * 0.01 deg, near-turn angle)

For `0x92` multi-turn angle response:

- `data[4..7]`: multi-turn angle (int32 * 0.01 deg)

CLI status output now prints both:

- `angle=...` from status-2 (`0x9C`)
- `mt_angle=...` from multi-turn angle (`0x92`) for absolute-position judgement

## 6) Common Command Examples

```bash
# Scan MyActuator IDs
motor_cli --vendor myactuator --channel can0 --mode scan --start-id 1 --end-id 32

# Read status continuously
motor_cli --vendor myactuator --channel can0 --model X8 --motor-id 1 --feedback-id 0x241 \
  --mode status --loop 40 --dt-ms 50

# Velocity control (+0.5236 rad/s ~= +30 deg/s)
motor_cli --vendor myactuator --channel can0 --model X8 --motor-id 1 --feedback-id 0x241 \
  --mode vel --vel 0.5236 --loop 100 --dt-ms 20

# Absolute position control (pi rad = 180 deg, max 5.236 rad/s ~= 300 deg/s)
motor_cli --vendor myactuator --channel can0 --model X8 --motor-id 1 --feedback-id 0x241 \
  --mode pos --pos 3.1416 --max-speed 5.236 --loop 1

# Set current position as zero (requires power-cycle to apply persistently)
motor_cli --vendor myactuator --channel can0 --model X8 --motor-id 1 --feedback-id 0x241 \
  --mode set-zero --loop 1
```

## 7) Troubleshooting

If motor replies but does not move:

1. Query status-1 (`0x9A`) externally and check error code.
2. If error code is `0x0004`, this means **low voltage** protection.
3. Recover supply voltage, then reset (`0x76`) and release brake (`0x77`).

Typical status-1 decode:

- `data[3]`: brake released flag
- `data[4..5]`: bus voltage (`uint16 * 0.1 V`)
- `data[6..7]`: error code
