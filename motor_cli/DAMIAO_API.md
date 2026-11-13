# Damiao API and Tuning Reference (Complete)

<!-- channel-compat-note -->
## Channel Compatibility (PCAN + CANable candleLight/gs_usb + MCU Serial Bridge + Damiao Serial Bridge + DM_Device)

- Linux SocketCAN uses prepared interfaces directly: `can0`, `can1`. For CANable, use candleLight/gs_usb firmware so it appears as a SocketCAN interface such as `can0`.
- Use PCAN or CANable candleLight/gs_usb for standard CAN.
- Damiao-only adapter transports are available in CLI: serial bridge (`--transport dm-serial --serial-port /dev/ttyACM0 --serial-baud 921600`) and DM_Device SDK (`--transport dm-device --dm-device-type usb2canfd|usb2canfd-dual|linkx4c --dm-channel 0|1|2|3`; Damiao motors only; adapter must be in USB mode).
- Generic MCU UART-to-CAN bridge: `--transport mcu-serial --serial-port /dev/ttyACM0 --serial-baud 921600`. Note this is a different protocol from the Damiao-only `dm-serial` bridge: `mcu-serial` relays classic CAN frames through the bridge firmware, while `dm-serial` carries the Damiao serial protocol. It covers all classic-CAN vendors (damiao / robstride / myactuator / hightorque / huayi; hexfellow is CAN-FD-only and not supported); the bridge firmware proactively reports structured link status on bus-off / TX failure / RX drop, and all four frontends (motor_abi / Python / motor_cli / ws_gateway) support it.
- On Linux SocketCAN, do not append bitrate in `--channel` (for example `can0@1000000` is invalid).
- On Windows (PCAN backend), `can0/can1` map to `PCAN_USBBUS1/2`; optional `@bitrate` suffix is supported.


This page is a practical reference for all commonly adjustable control/configuration parameters currently available in `motorbridge` for Damiao motors.

> Chinese version: [DAMIAO_API.zh-CN.md](DAMIAO_API.zh-CN.md)

## 1) Common Device Parameters

| Parameter | Meaning | Typical value |
|---|---|---|
| `channel` | CAN interface name | `can0` |
| `model` | Motor model string (`4310`, `4340P`, etc.) | matches real device |
| `motor-id` | Command ID (`ESC_ID`) | e.g. `0x01` |
| `feedback-id` | Feedback ID (`MST_ID`) | e.g. `0x11` |
| `loop` | Number of send cycles | `100` |
| `dt-ms` | Send interval per cycle (ms) | `20` |

## 2) Real-time Control Mode Parameters

## 2.1 MIT Mode

Command fields:

- `pos`: target position
- `vel`: target velocity
- `kp`: position stiffness gain
- `kd`: velocity damping gain
- `tau`: feedforward torque

Example (`motor_cli`):

```bash
motor_cli \
  --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 \
  --mode mit --pos 0 --vel 0 --kp 20 --kd 1 --tau 0 --loop 200 --dt-ms 20
```

## 2.2 POS_VEL Mode

Command fields:

- `pos`: target position
- `vlim`: velocity limit

Example:

```bash
motor_cli \
  --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 \
  --mode pos-vel --pos 3.10 --vlim 1.50 --loop 300 --dt-ms 20
```

## 2.3 VEL Mode

Command field:

- `vel`: target velocity

Example:

```bash
motor_cli \
  --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 \
  --mode vel --vel 0.5 --loop 100 --dt-ms 20
```

## 2.4 FORCE_POS Mode

Command fields:

- `pos`: target position
- `vlim`: velocity limit
- `ratio`: torque limit ratio

Example:

```bash
motor_cli \
  --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 \
  --mode force-pos --pos 0.8 --vlim 2.0 --ratio 0.3 --loop 100 --dt-ms 20
```

## 2.5 Mode Selection (`CTRL_MODE`)

Register:

- `rid=10` (`CTRL_MODE`), values:
  - `1=MIT`
  - `2=POS_VEL`
  - `3=VEL`
  - `4=FORCE_POS`

## 3) High-impact Registers (Priority)

These have major control impact and should be tuned carefully.

| RID | Name | Type | Meaning |
|---|---|---|---|
| `21` | `PMAX` | `f32` | Position mapping range |
| `22` | `VMAX` | `f32` | Velocity mapping range |
| `23` | `TMAX` | `f32` | Torque mapping range |
| `25` | `KP_ASR` | `f32` | Speed loop Kp |
| `26` | `KI_ASR` | `f32` | Speed loop Ki |
| `27` | `KP_APR` | `f32` | Position loop Kp |
| `28` | `KI_APR` | `f32` | Position loop Ki |
| `4` | `ACC` | `f32` | Acceleration |
| `5` | `DEC` | `f32` | Deceleration |
| `6` | `MAX_SPD` | `f32` | Maximum speed |
| `9` | `TIMEOUT` | `u32` | Communication timeout register |

## 4) Protection Registers

| RID | Name | Type | Meaning |
|---|---|---|---|
| `0` | `UV_Value` | `f32` | Under-voltage threshold |
| `2` | `OT_Value` | `f32` | Over-temperature threshold |
| `3` | `OC_Value` | `f32` | Over-current threshold |
| `29` | `OV_Value` | `f32` | Over-voltage threshold |

## 5) How to Read/Write Parameters

Recommended write workflow:

1. `get_register` read old value
2. `write_register` write new value
3. `get_register` read back
4. `store_parameters` persist

Python SDK API example:

```python
from motorbridge import Controller

with Controller("can0") as ctrl:
    m = ctrl.add_damiao_motor(0x01, 0x11, "4340P")
    old = m.get_register_f32(22, 1000)
    print("old VMAX", old)
    m.write_register_f32(22, old * 0.9)
    new = m.get_register_f32(22, 1000)
    print("new VMAX", new)
    m.store_parameters()
    m.close()
```

WS gateway command examples:

```json
{"op":"get_register_f32","rid":22,"timeout_ms":1000}
{"op":"write_register_f32","rid":22,"value":25.0}
{"op":"store_parameters"}
```

## 6) ID and Calibration Parameters

- `rid=8` (`ESC_ID`): command ID
- `rid=7` (`MST_ID`): feedback ID

Tools:

- Rust CLI: `motor_cli --vendor damiao --mode scan ...` and `--set-motor-id/--set-feedback-id --store --verify-id`
- Python: `motorbridge-cli scan/id-set/id-dump`
- WS: `scan`, `set_id`, `verify`

## 7) Safety Notes

- Tune one parameter group at a time.
- Use small steps, verify each change.
- Keep safe mechanical load and emergency stop ready.
- For protection thresholds (`0/2/3/29`), avoid aggressive changes without hardware margin analysis.

## 8) Gripper Motor Calibration

Damiao motors use a single-turn encoder (position range approx. ±PMAX rad). **Zero position is NOT retained across power cycles.** When a motor is used as a gripper, calibrate the zero position after each power-on:

1. Use MIT mode with low `kp` and `tau=0` to move toward the mechanical limit — the motor stops softly on contact.
2. Wait until feedback shows `vel ≈ 0` and `pos` is stable, then run `--mode set-zero`.
3. After calibration, use MIT mode for closing (force-limited, safe) and pos-vel mode for opening (fast, precise).

For full command examples, see the [Operation Manual](../docs/zh/operation_manual.md) Section 9.
