# FashionStar Python API

> 中文：[FASHIONSTAR_API.zh-CN.md](FASHIONSTAR_API.zh-CN.md)

Use the existing `Controller`/`Motor` API for FashionStar CAN servos. Protocol coverage, units, PCAN channel setup and Rust CLI commands are in the [FashionStar reference](../../motor_cli/FASHIONSTAR_API.md).

## SDK

```python
from motorbridge import Controller

ctrl = Controller("can0")
try:
    motor = ctrl.add_fashionstar_motor(0, 0x321, "fashionstar")
    try:
        motor.send_pos_vel(1.5707963, 1.0)  # rad, rad/s
        motor.request_feedback()
        print(motor.get_state_snapshot())
    finally:
        motor.close()
finally:
    try:
        ctrl.close_bus()
    finally:
        ctrl.close()
```

This cleanup closes the connection. The existing `with Controller(...)` also calls `shutdown()` on exit, releasing servo output. Position queried after `request_feedback()` may still be in transit; it does not confirm arrival.

- `add_fashionstar_motor(motor_id, feedback_id, model)`: ServoID 0..254; model is currently ignored.
- Use existing `enable()`, `disable()`, `stop()`, `set_zero_position()`, `send_pos_vel(pos, vlim)`, `get_register_u32(rid, timeout_ms)`, `write_register_u32(rid, value)`.
- `ensure_mode(Mode.POS_VEL)` is available. VEL, MIT/quantitative torque and FORCE_POS are currently unsupported; operations without a protocol mapping return errors.
- `vel` is estimated from consecutive position samples; `torq` carries current in A (`0x11` in mA divided by 1000), following MyActuator's mapping; `t_mos` is servo temperature in °C using the official CAN tool's default conversion. The first velocity sample, invalid temperature readings and rotor temperature are `NaN`. `torq` does not represent torque in N·m.
- Configuration requires a power cycle. After writing `0x81`, power-cycle, create a connection with the new ID, then verify; the live handle retains its old ID.

## Python CLI

```bash
motorbridge scan --vendor fashionstar --channel can0 --start-id 0 --end-id 16
motorbridge run --vendor fashionstar --channel can0 --motor-id 0 --mode ping
motorbridge run --vendor fashionstar --channel can0 --motor-id 0 --mode pos-vel --pos 1.5707963 --vlim 1
motorbridge run --vendor fashionstar --channel can0 --motor-id 0 --mode read-param --param-id 0x81 --param-type u32
motorbridge id-set --vendor fashionstar --channel can0 --motor-id 0 --new-motor-id 2
```

Python CLI modes: `enable`, `disable`, `pos-vel`, `zero`, `set-zero`, `ping`, `read-param`, `write-param`. Use existing `scan` and `id-set` subcommands; `run --set-motor-id` reuses the ID update flow. CLI cleanup closes the connection without an additional disable command. Transports: `auto|socketcan|mcu-serial`; the UC-06 serial protocol is different.
