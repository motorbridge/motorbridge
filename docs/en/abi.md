# ABI Guide (`motor_abi`)

## Build

```bash
cargo build -p motor_abi --release
```

Artifacts:

- Linux: `target/release/libmotor_abi.so`, `libmotor_abi.a`
- Windows: `target/release/motor_abi.dll`, `motor_abi.lib`
- Header: `motor_abi/include/motor_abi.h`

## Unified Surface

The ABI keeps one unified control surface across vendors.

## ABI Metadata

- `motor_abi_version()` returns the loaded ABI library version string.
- `motor_abi_capabilities_json()` returns a stable JSON capability document
  describing supported transports, vendors, controller lifecycle operations,
  control modes, and vendor-specific features.
- Python exposes the same information through `motorbridge.abi_version()` and
  `motorbridge.abi_capabilities()`.
- C++ exposes the same information through `motorbridge::abi_version()` and
  `motorbridge::abi_capabilities_json()`.
- Binding parity is tracked in `bindings/api_surface.json`.

Unified mode IDs (`motor_handle_ensure_mode`):

- `1 = MIT`
- `2 = POS_VEL`
- `3 = VEL`
- `4 = FORCE_POS`

Unified control units:

- position: `rad`
- velocity: `rad/s`
- torque: `Nm`

Common control/state APIs:

- `motor_handle_enable`
- `motor_handle_disable`
- `motor_handle_clear_error`
- `motor_handle_set_zero_position`
- `motor_handle_ensure_mode`
- `motor_handle_send_mit`
- `motor_handle_send_pos_vel`
- `motor_handle_send_vel`
- `motor_handle_send_force_pos`
- `motor_handle_request_feedback`
- `motor_handle_set_can_timeout_ms`
- `motor_handle_store_parameters`
- `motor_handle_get_state`
- `motor_handle_get_state_snapshot`

`motor_handle_get_state_snapshot` returns the same state as
`motor_handle_get_state` plus its age in nanoseconds and a per-motor sequence
number. The age uses the host monotonic clock. The sequence advances only when
MotorBridge processes a complete state update; parameter, ping, and fault-only
replies do not make cached feedback appear fresh.

## Vendor Entry Points

- Damiao: `motor_controller_add_damiao_motor(...)`
- Hexfellow: `motor_controller_add_hexfellow_motor(...)` (CAN-FD path via `socketcanfd`)
- RobStride: `motor_controller_add_robstride_motor(...)`
- MyActuator: `motor_controller_add_myactuator_motor(...)`
- HighTorque: `motor_controller_add_hightorque_motor(...)`
- Huayi: `motor_controller_add_huayi_motor(controller, motor_id, feedback_id, model)` — CANopen/CiA402 over classic CAN; `motor_id` is the node id (1..=127), `feedback_id=0` reuses the node id; `model` is a label only, limits are read at runtime (peak torque `6076h`, compressed-MIT ranges `2004h`).

## Unified Mode to Native Protocol Mapping

| Unified mode | Damiao native | Hexfellow native | RobStride native | MyActuator native | HighTorque native | Huayi native |
|---|---|---|---|---|---|---|
| `MIT` | `Mit` | mode `5` | `Mit` | not available | mapped to native pos+vel+tqe | **`6060h = 5` (MIT)** |
| `POS_VEL` | `PosVel` | mode `1` | mapped to `Position` (`run_mode=1`, `limit_spd=0x7017`, `loc_ref=0x7016`) | `Position` setpoint flow | mapped to native pos+vel+tqe | `6060h = 1` (PP: `6081h` + `607Ah` + CW bit4 handshake) |
| `VEL` | `Vel` | not available | `Velocity` | `Velocity` setpoint flow | mapped to native velocity command | `6060h = 3` (PV: `60FFh`) |
| `FORCE_POS` | `ForcePos` | not available | not available | not available | mapped to native pos+vel+tqe | not available (no matching 6060h mode) |

> **Mode-number trap (Huayi 5=MIT)**: Huayi's `6060h` values follow CiA402 numbering — `5` is MIT. In the RobStride **proprietary protocol**, `5` is CSP. Do not mix the two. The unified IDs (1/2/3/4) are mapped by the ABI layer; user code never touches raw `6060h` values.

Behavior rule:

- Unsupported calls return non-zero and a readable message via `motor_last_error_message()`.
- Signatures stay stable even when a vendor ignores part of a signature.
- Example: HighTorque accepts `send_mit(pos, vel, kp, kd, tau)` for interface consistency, but native protocol does not use `kp/kd`.
- Hexfellow ABI path supports MIT and POS_VEL, and reports `VEL` / `FORCE_POS` as unsupported.
- RobStride supports MIT / POS_VEL / VEL on unified APIs; torque/current remains parameter-level (`robstride_write_param_*`), not a unified mode.
- Damiao set-zero sequence rule: call `motor_handle_disable` before `motor_handle_set_zero_position`; otherwise set-zero is rejected by core guard.
- Damiao set-zero settle rule: core applies an internal fixed settle (`~20ms`) after successful `set_zero_position` (no extra ABI parameter).
- Huayi set-zero mirrors the Damiao rule: the 3001h position preset requires Switch On Disabled — call `motor_handle_disable` before `motor_handle_set_zero_position`; persist with `motor_handle_store_parameters` (1010h "save").
- Huayi `motor_handle_get_state` mapping: `status_code` = `6061h` mode display, `fault_code` = `603Fh`/EMCY low byte (full u16 via `motor_handle_huayi_get_error_code`); state comes from the TPDO stream (TPDO2 anchors the seq, TPDO1/TPDO3 patch in place).
- Huayi `motor_handle_request_feedback`: a no-op when the TPDO stream is active (the background receiver thread keeps the cache fresh); otherwise it auto-activates the three default TPDOs (10ms event timer). `motor_handle_set_can_timeout_ms` is rejected for Huayi — the manual provides no watchdog object equivalent to `6099h`.

## Vendor-Specific Extensions

Damiao register APIs:

- `motor_handle_write_register_f32`
- `motor_handle_write_register_u32`
- `motor_handle_get_register_f32`
- `motor_handle_get_register_u32`

RobStride extensions:

- `motor_handle_robstride_ping`
- `motor_handle_robstride_ping_host_id`
- `motor_handle_robstride_set_device_id`
- `motor_handle_robstride_set_active_report`
- `motor_handle_robstride_get_fault_report`
- `motor_handle_robstride_get_param_f32_host_id`
- `motor_handle_robstride_write_param_i8/u8/u16/u32/f32`
- `motor_handle_robstride_get_param_i8/u8/u16/u32/f32`

Huayi extensions (typed SDO access + private fast paths; the generic u8-rid register APIs cannot address 16-bit CANopen indices and are always rejected for Huayi motors in favor of this set):

- `motor_handle_huayi_sdo_read_u16/i16/i32/f32(index, sub, timeout_ms, out)`
- `motor_handle_huayi_sdo_write_u16/i16/i32/f32(index, sub, value)`
- `motor_handle_huayi_send_torque(torque_nm)` — torque mode (`6060h=4`, `6071h` in per-mille of peak)
- `motor_handle_huayi_send_mit_compressed(pos, vel, kp, kd, tau)` — compressed-MIT realtime channel (RPDO4, one 8-byte frame per command to `0x500+node`; the slow path is unified `motor_handle_send_mit` via `2003h`)
- `motor_handle_huayi_position_preset(position_rev)` — 3001h position preset (disable first; ±0.5 Rev of the output shaft)
- `motor_handle_huayi_configure_periodic_report(period_ms)` — TPDO stream on/off (0 stops; plays the role of RobStride `set_active_report`)
- `motor_handle_huayi_get_error_code(out)` — cached error code (u16, no bus traffic; EMCY codes are visible even before a TPDO2 anchor)
- `motor_handle_huayi_get_error_history(out_codes, out_count)` — 1003h error-history walk
- `motor_handle_huayi_set_node_id(new_id)` — 2001h node-id change (store + power cycle to apply)
- `motor_handle_huayi_nmt(command, node)` — NMT master command (0x01/0x02/0x80/0x81/0x82; node 0 broadcasts)
- `motor_controller_huayi_scan_ids(controller, start, end, timeout_ms, out_nodes, out_count)` — node scan (NMT start broadcast + 6041h/6061h/603Fh probes); the direct probes race the core receive thread, so this returns an error when any motor is already registered on the controller — scan before `add_huayi_motor`, or use a separate controller on the same bus

## Typical Call Flow

1. Transport constructor:
   - `motor_controller_new_socketcan(channel)` (general path)
   - `motor_controller_new_socketcanfd(channel)` (CAN-FD path; required by Hexfellow)
   - `motor_controller_new_dm_serial(serial_port, baud)` (Damiao-only serial bridge; cross-platform, e.g. `/dev/ttyACM0` or `COM3`)
   - `motor_controller_new_dm_device(dm_device_type, dm_channel)` (Damiao-only DM_Device SDK path; e.g. `usb2canfd` + `0`, `usb2canfd-dual` + `0|1`, or `linkx4c` + `0|1|2|3`)
2. `motor_controller_add_<vendor>_motor`
3. optional: `motor_controller_enable_all`
4. optional: `motor_handle_ensure_mode`
5. send control commands / read state / vendor-specific operations
6. `motor_controller_shutdown`
   - or `motor_controller_close_bus` when only closing the local session/bus
7. `motor_handle_free`
8. `motor_controller_free`

## Examples

- C ABI demo: `examples/c/c_abi_demo.c`
- C++ ABI demo: `examples/cpp/cpp_abi_demo.cpp`
- Python ctypes demo: `examples/python/python_ctypes_demo.py`
