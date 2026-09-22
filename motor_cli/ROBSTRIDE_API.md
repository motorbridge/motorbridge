# RobStride API and Parameter Reference (Complete)

<!-- channel-compat-note -->
## Channel Compatibility (PCAN + CANable candleLight/gs_usb + Damiao Serial Bridge + DM_Device)

- Linux SocketCAN uses prepared interfaces directly: `can0`, `can1`. For CANable, use candleLight/gs_usb firmware so it appears as a SocketCAN interface such as `can0`.
- Use PCAN or CANable candleLight/gs_usb for standard CAN.
- Damiao-only adapter transports are available in CLI: serial bridge (`--transport dm-serial --serial-port /dev/ttyACM0 --serial-baud 921600`) and DM_Device SDK (`--transport dm-device --dm-device-type usb2canfd|usb2canfd-dual|linkx4c --dm-channel 0|1|2|3`; Damiao motors only; adapter must be in USB mode).
- On Linux SocketCAN, do not append bitrate in `--channel` (for example `can0@1000000` is invalid).
- On Windows (PCAN backend), `can0/can1` map to `PCAN_USBBUS1/2`; optional `@bitrate` suffix is supported.


Practical and complete reference for RobStride control, parameter access, and current capability boundaries in `motorbridge`.

> Chinese version: [ROBSTRIDE_API.zh-CN.md](ROBSTRIDE_API.zh-CN.md)

## 1) Common Device Parameters

| Parameter | Meaning | Typical value |
|---|---|---|
| `channel` | CAN interface name | `can0` |
| `model` | RobStride model string | `rs-00`, `rs-06` |
| `motor-id` | Device ID | e.g. `127` |
| `feedback-id` | Host/feedback ID used in command frame | usually `0xFD` |
| `loop` | Send cycles for periodic control | `20`~`100` |
| `dt-ms` | Send interval per cycle | `20`~`50` |

## 2) `motor_cli` RobStride Modes

Supported now:

- `ping`
- `scan`
- `enable`
- `disable`
- `mit`
- `pos-vel`
- `vel`
- `read-param`
- `write-param`

Unified "big-four" mapping status:

| Unified capability | RobStride status | Notes |
|---|---|---|
| `MIT` | supported | native operation-control frame |
| `POS_VEL` | supported | mapped to `run_mode=1` + `0x7017/0x7016` |
| `VEL` | supported | mapped to `run_mode=2` + `0x700A` |
| `TORQUE/CURRENT` | parameter-level only | no first-class high-level mode yet; use `write-param` (`iq_ref`, limits) |

### 2.1 Ping

```bash
motor_cli \
  --vendor robstride --channel can0 --model rs-06 --motor-id 127 --feedback-id 0xFD \
  --mode ping
```

### 2.2 MIT

```bash
motor_cli \
  --vendor robstride --channel can0 --model rs-06 --motor-id 127 --feedback-id 0xFD \
  --mode mit --ensure-strict 1 --pos 0.5 --vel 0 --kp 20.0 --kd 0.5 --tau 0 --loop 100 --dt-ms 20
```

MIT mapping details (unified -> native):

- Effective inputs: `--pos`, `--vel`, `--kp`, `--kd`, `--tau` (all are used).
- Units:
  - `--pos`: `rad`
  - `--vel`: `rad/s`
  - `--tau`: `Nm`
  - `--kp`, `--kd`: MIT loop gains

### 2.3 Position (unified `pos-vel` mapping)

```bash
motor_cli \
  --vendor robstride --channel can0 --model rs-06 --motor-id 127 --feedback-id 0xFD \
  --mode pos-vel --pos 1.5 --vlim 1.0 --loc-kp 5.0 --loop 1 --dt-ms 20
```

Notes:

- Unified `pos-vel` maps to native RobStride Position path:
  - `run_mode=1` (Position)
  - write `0x7017` (`limit_spd`) from `--vlim`
  - optional write `0x701E` (`loc_kp`) from `--loc-kp` or `--kp`
  - write `0x7016` (`loc_ref`) from `--pos`
- `--vel`, `--kd`, and `--tau` do not belong to native Position mode and are ignored in `--mode pos-vel`.
- This legacy `pos-vel` mapping is kept for compatibility.

#### RobStride-Specific PP / CSP

`pos-vel-pp` follows the manual's PP position mode sequence:

- write `0x7005` (`run_mode`) = `1`
- send enable frame (communication type 3)
- write `0x7024` (`vel_max`) from `--vlim`
- write `0x7025` (`acc_set`) from `--acc`
- write `0x7016` (`loc_ref`) from `--pos`

```bash
motor_cli \
  --vendor robstride --channel can0 --model rs-06 --motor-id 127 --feedback-id 0xFD \
  --mode pos-vel-pp --pos 1.0 --vlim 1.5 --acc 10.0
```

`pos-vel-csp` follows the manual's CSP position mode sequence:

- write `0x7005` (`run_mode`) = `5`
- send enable frame (communication type 3)
- write `0x7017` (`limit_spd`) from `--vlim`
- write `0x7016` (`loc_ref`) from `--pos`

```bash
motor_cli \
  --vendor robstride --channel can0 --model rs-06 --motor-id 127 --feedback-id 0xFD \
  --mode pos-vel-csp --pos 1.0 --vlim 1.5
```

RobStride parameter writes do not wait for status ack by default, so `pos-vel` / PP / CSP target updates can run close to the outer loop cadence. To restore conservative synchronous waiting, set:

```bash
export MOTORBRIDGE_ROBSTRIDE_WRITE_ACK_TIMEOUT_MS=260
```

This environment variable only affects communication type 18 parameter-write waits; `enable`, `disable`, `set-zero`, and similar control frames keep their dedicated wait behavior.

Python binding code can call the two dedicated interfaces directly:

```python
from motorbridge import Controller

with Controller("can0") as ctrl:
    motor = ctrl.add_robstride_motor(1, 0xFD, "rs-00")

    # PP: run_mode=1 -> enable -> vel_max(0x7024) -> acc_set(0x7025) -> loc_ref(0x7016)
    motor.robstride_send_pos_vel_pp(pos=0.0, vel_max=1.0, acc_set=10.0)

    # CSP: run_mode=5 -> enable -> limit_spd(0x7017) -> loc_ref(0x7016)
    motor.robstride_send_pos_vel_csp(pos=0.0, vlim=1.0)
```

Those two calls are full manual-sequence wrappers. For a high-rate position loop, prepare the mode and limits once, then write only `loc_ref(0x7016)` cyclically:

```python
from motorbridge import Controller

with Controller("can0") as ctrl:
    motor = ctrl.add_robstride_motor(1, 0xFD, "rs-00")

    # CSP high-rate path: prepare once.
    motor.robstride_write_param_i8(0x7005, 5)       # run_mode = CSP
    motor.enable()
    motor.robstride_write_param_f32(0x7017, 1.0)   # limit_spd

    # Cyclic loop: loc_ref only.
    for pos in [0.0, 0.1, 0.2, 0.1, 0.0]:
        motor.robstride_write_param_f32(0x7016, pos)
```

PP high-rate code follows the same shape:

```python
with Controller("can0") as ctrl:
    motor = ctrl.add_robstride_motor(1, 0xFD, "rs-00")

    motor.robstride_write_param_i8(0x7005, 1)        # run_mode = PP
    motor.enable()
    motor.robstride_write_param_f32(0x7024, 1.0)    # vel_max
    motor.robstride_write_param_f32(0x7025, 10.0)   # acc_set

    for pos in [0.0, 0.1, 0.2, 0.1, 0.0]:
        motor.robstride_write_param_f32(0x7016, pos)
```

### 2.4 Velocity

```bash
motor_cli \
  --vendor robstride --channel can0 --model rs-06 --motor-id 127 --feedback-id 0xFD \
  --mode vel --vel 0.3 --loop 40 --dt-ms 50
```

### 2.5 Two Usage Paths (Unified Wrapper / Native Params)

- Unified wrapper path (recommended for app-layer control):
  - `--mode mit`
  - `--mode pos-vel` (already mapped to native Position)
  - `--mode vel`
- Native path (debug/protocol-level verification):
  - `--mode read-param --param-id ...`
  - `--mode write-param --param-id ... --param-value ...`
  - Typical sequence: write `run_mode(0x7005)` first, then write target params (`loc_ref/spd_ref`, etc.)

## 3) Scan and ID Update

### 3.1 Scan

```bash
motor_cli \
  scan --vendor robstride --channel can0 --model rs-06 \
  --start-id 1 --end-id 255 \
  --feedback-ids 0xFD,0xFF,0xFE,0x00,0xAA
```

Notes:

- Fast pass: ping + query-parameter probe.
- `probe` / `device_id` is the motor ID.
- `feedback_id` / `host_id` (for example `0xFD`) is the host-side ID, not the motor ID.
- `--feedback-ids` is a comma-separated list of host IDs to try during scan.
- RobStride `motor_id` / `device_id` must be `1..255`; `feedback_id` / `host_id` must be `0..255`.
- During scan, each listed `--feedback-ids` entry is probed exactly; invalid host IDs are rejected instead of silently falling back.
- If no ping replies in full range, CLI auto-falls back to blind pulse probing:
  - `--manual-vel` (default `0.2`)
  - `--manual-ms` (default `200`)
  - `--manual-gap-ms` (default `200`)

### 3.2 Update Device ID

```bash
motor_cli \
  --vendor robstride --channel can0 --model rs-06 \
  --motor-id 127 --feedback-id 0xFD --set-motor-id 126 --store 1
```

Python CLI equivalent:

```bash
motorbridge-cli id-set \
  --vendor robstride --channel can0 --model rs-06 \
  --motor-id 127 --feedback-id 0xFD \
  --new-motor-id 126 --store 1 --verify 1
```

Raw protocol alignment (with official upper software):

- Set-ID frame uses `comm_type=7`.
- This changes the RobStride `device_id` only; it does not change `feedback_id` / `host_id`.
- `--set-motor-id` / `--new-motor-id` is validated as `1..255`; out-of-range values are rejected instead of being truncated.
- Extended ID format in this command path is:
  - `0x07 [new_id] [host_id] [old_id]`
  - example (`old_id=1`, `new_id=11`, `host_id=0xFD`): `0x070BFD01`
- Data payload uses the latest ping UUID token when available (fallback to zeros if ping token is unavailable).

## 4) Frequently Used Parameter IDs

| Param ID | Name | Type | Meaning |
|---|---|---|---|
| `0x7005` | `run_mode` | `i8` | control mode selector |
| `0x700A` | `spd_ref` | `f32` | target velocity |
| `0x7019` | `mechPos` | `f32` | mechanical position |
| `0x701B` | `mechVel` | `f32` | mechanical velocity |
| `0x701C` | `VBUS` | `f32` | bus voltage |

## 5) Parameter Read/Write

Read parameter:

```bash
motor_cli \
  --vendor robstride --channel can0 --model rs-06 --motor-id 127 --feedback-id 0xFD \
  --mode read-param --param-id 0x7019
```

Write parameter:

```bash
motor_cli \
  --vendor robstride --channel can0 --model rs-06 --motor-id 127 --feedback-id 0xFD \
  --mode write-param --param-id 0x700A --param-value 0.3
```

Python binding sample:

```python
from motorbridge import Controller

with Controller("can0") as ctrl:
    m = ctrl.add_robstride_motor(127, 0xFD, "rs-06")
    print(m.robstride_ping())
    print(m.robstride_get_param_f32(0x7019, 500))
    m.robstride_write_param_f32(0x700A, 0.3)
    m.close()
```

## 6) Protocol Communication Coverage

`motorbridge` currently exposes or uses these RobStride protocol communication types:

- In use directly: `0(GET_DEVICE_ID)`, `1(OPERATION_CONTROL)`, `3(ENABLE)`, `4(DISABLE/CLEAR_ERROR)`, `6(SET_ZERO_POSITION)`, `7(SET_DEVICE_ID)`, `17(READ_PARAMETER)`, `18(WRITE_PARAMETER)`, `22(SAVE_PARAMETERS)`, `24(ACTIVE_REPORT)`
- Receive/parse path: `2(OPERATION_STATUS)`, `21(FAULT_REPORT)`; fault/warning raw values and documented fault bits are exposed in state diagnostics.
- Present in protocol constants but intentionally not first-class high-level APIs yet: `23(SET_BAUDRATE)`, `25(SET_PROTOCOL)` because both can make the device unreachable if used incorrectly.

## 7) Gap Summary and Next Improvements

Current status: core control is production-usable (`scan/ping/mit/pos-vel/vel/read/write/set-id/set-zero/store`).

Known issues (observed in field tests):

1. `pos-vel` parameter effectiveness can be inconsistent on some firmware:
   - `--vlim` (`0x7017`) and `--kp`/`loc_kp` (`0x701E`) may read back as written but show weak/no visible effect.
   - `MIT` path is currently more reliable.
2. RobStride zero calibration is still inconsistent:
   - experimental `zero` sequence may complete transport-level send/ack but device-side `zero_sta`/`mechPos` verification can fail.
   - treat zero calibration as unresolved until firmware-specific sequence is fully matched.

Main improvement opportunities:

1. Add semantic CLI mode for current/torque control (today still done via write-param, less ergonomic).
2. Add multi feedback-host candidate support in scan CLI.
3. Decide whether `SET_BAUDRATE / SET_PROTOCOL` should be exposed behind an explicit danger gate.

## 8) WS Gateway JSON Examples

```json
{"op":"set_target","vendor":"robstride","channel":"can0","model":"rs-06","motor_id":127,"feedback_id":253}
{"op":"robstride_ping","timeout_ms":200}
{"op":"robstride_read_param","param_id":28697,"type":"f32","timeout_ms":200}
{"op":"robstride_write_param","param_id":28682,"type":"f32","value":0.3,"verify":true}
{"op":"vel","vel":0.3,"continuous":true}
{"op":"mit","pos":0.0,"vel":0.0,"kp":0.5,"kd":0.2,"tau":0.0,"continuous":true}
{"op":"scan","vendor":"robstride","start_id":1,"end_id":255,"feedback_ids":"0xFD,0xFF,0xFE","timeout_ms":120}
```

## 9) Safety Notes

- Start with small velocity and short loop count.
- Confirm CAN wiring/termination and interface state before stress tests.
- Prefer ping/read-param verification before long periodic control.
- Keep emergency stop path available.
