# Python Practical Demos

<!-- channel-compat-note -->
## Channel Compatibility (PCAN + CANable candleLight/gs_usb + CAN-FD + Damiao Serial Bridge + DM_Device)

- Linux SocketCAN uses prepared interfaces directly: `can0`, `can1`. For CANable, use candleLight/gs_usb firmware so it appears as a SocketCAN interface such as `can0`.
- Use PCAN or CANable candleLight/gs_usb for standard CAN.
- Hexfellow examples require CAN-FD path (`Controller.from_socketcanfd(...)` / CLI `--transport socketcanfd`).
- Damiao-only adapter transports are available in CLI: serial bridge (`--transport dm-serial --serial-port /dev/ttyACM0 --serial-baud 921600`) and DM_Device SDK (`--transport dm-device --dm-device-type usb2canfd|usb2canfd-dual|linkx4c --dm-channel 0|1|2|3`; Damiao motors only; adapter must be in USB mode).
- Full Damiao serial-bridge interface list and command patterns are documented in `motor_cli/README.md` (section `3.6` in `motor_cli/README.zh-CN.md`).
- On Linux SocketCAN, do not append bitrate in `--channel` (for example `can0@1000000` is invalid).
- On Windows (PCAN backend), `can0/can1` map to `PCAN_USBBUS1/2`; optional `@bitrate` suffix is supported.


Examples built on the Python SDK.

> Chinese version (examples): [READMEzh_cn.md](READMEzh_cn.md)  
> Chinese version (Python binding overview): [../README.zh-CN.md](../README.zh-CN.md)

## Start Here (Simplest 2 Examples)

If you installed from pip and want a cleaner onboarding path, start with:

- `../get_started/README.md` (English)
- `../get_started/README.zh-CN.md` (Chinese)

If you are new to this repository, start with these two files first:

- `simple_01_motor_control.py`: single-motor minimal template (default Damiao, other vendors as commented snippets).
- `simple_02_quad_motor_control.py`: minimal 4-motor multi-vendor swing demo (2 Damiao + 1 MyActuator + 1 RobStride).

Recommended order:

1. Run `simple_01_motor_control.py` to verify your CAN channel and one motor first.
2. Then run `simple_02_quad_motor_control.py` to verify multi-controller / multi-vendor behavior.

Quick start commands:

```bash
# 1) single motor (safest first step)
PYTHONPATH=bindings/python/src LD_LIBRARY_PATH=$PWD/target/release:${LD_LIBRARY_PATH} \
python3 bindings/python/examples/simple_01_motor_control.py \
  --channel can0 --loop 120 --dt-ms 20 --pos 1.0 --vlim 1.0

# 2) quad motor swing
PYTHONPATH=bindings/python/src LD_LIBRARY_PATH=$PWD/target/release:${LD_LIBRARY_PATH} \
python3 bindings/python/examples/simple_02_quad_motor_control.py \
  --channel can0 --pos 1.0 --loop 240 --dt-ms 20 --swing-loop 60
```

### Parameter Cheat Sheet (for the two simplest scripts)

`simple_01_motor_control.py`
- `--channel`: CAN interface name, e.g. `can0`, `can1`.
- `--loop`: number of control iterations.
- `--dt-ms`: control period (ms). Start with `20`; if bus is busy, use `30`/`50`.
- `--pos`: target position (radians).
- `--vlim`: speed limit for POS_VEL.

`simple_02_quad_motor_control.py`
- `--channel`: CAN interface name.
- `--loop`: total loop count.
- `--dt-ms`: control period (ms), same tuning rule as above.
- `--pos`: swing amplitude (radians), used as `+pos/-pos`.
- `--swing-loop`: switch sign every N loops.
- `--rs-dir-sign`: `-1` means RobStride direction opposite to others; `+1` means same.
- `--dm-vlim`: Damiao speed limit in POS_VEL.
- `--my-vlim`: MyActuator speed limit in POS_VEL.
- `--rs-kp` / `--rs-kd`: RobStride MIT gains.

### What to Read Next (recommended order)

1. [../README.zh-CN.md](../README.zh-CN.md) or [../README.md](../README.md)  
   Start here for Python binding installation and base API concepts.
2. [../../motor_cli/README.zh-CN.md](../../motor_cli/README.zh-CN.md) or [../../motor_cli/README.md](../../motor_cli/README.md)  
   Use this for vendor scan commands and ID confirmation.
3. [../../examples/README.zh-CN.md](../../examples/README.zh-CN.md) or [../../examples/README.md](../../examples/README.md)  
   Broader demo index (web/WS/other integration examples).

### WS Demo Quick Guide (`quad_vendor_binding_ws_demo.py`)

This demo is a Python WS bridge for 4 motors (`dm1`, `dm2`, `my`, `rs`) with a matching web UI.

Backend start:

```bash
cd /path/to/rust_dm
python3 -m venv .venv-ws
source .venv-ws/bin/activate
pip install websockets
export PYTHONPATH=$PWD/bindings/python/src
export LD_LIBRARY_PATH=$PWD/target/release:${LD_LIBRARY_PATH}
python3 bindings/python/examples/quad_vendor_binding_ws_demo.py \
  --bind 127.0.0.1 --port 9010 --channel can0 --dt-ms 20
```

Frontend start (new terminal):

```bash
cd /path/to/rust_dm/bindings/python/examples
python3 -m http.server 8080
```

Open browser:
- `http://127.0.0.1:8080/quad_vendor_binding_ws_demo.html`
- WS URL: `ws://127.0.0.1:9010`

If your IDs are different, pass them on startup:

```bash
python3 bindings/python/examples/quad_vendor_binding_ws_demo.py \
  --channel can0 \
  --dm1-id 0x01 --dm1-fid 0x11 --dm1-model 4340P \
  --dm2-id 0x07 --dm2-fid 0x17 --dm2-model 4310 \
  --my-id 1 --my-fid 0x241 --my-model X8 \
  --rs-id 127 --rs-fid 0xFE --rs-model rs-00
```

### Key Points Beginners Must Know

- One `Controller` instance cannot mix vendors; use separate controllers for mixed-vendor control.
- `os error 105` usually means send rate is too high or another process is also writing to CAN.
- Streaming control needs periodic sends; single-shot command is often not enough for stable holding.
- Confirm motor IDs/feedback IDs with scan before running these demos.

## Files

- `simple_01_motor_control.py`: simplest single-motor template (default Damiao; includes commented vendor switch snippets)
- `simple_02_quad_motor_control.py`: simplest 4-motor multi-vendor demo with fixed-rate loop and swing target
- `python_wrapper_demo.py`: minimal Damiao MIT loop
- `damiao_maintenance_demo.py`: Damiao maintenance flow (`clear_error` / `set_zero_position` / `set_can_timeout_ms` / `request_feedback`)
- `damiao_register_rw_demo.py`: Damiao register read/write (`f32` + `u32` + optional `store_parameters`)
- `damiao_dm_serial_demo.py`: Damiao serial-bridge transport demo (`Controller.from_dm_serial`)
- `dm_serial_01_calibration_demo.py`: SOP-01 dm-serial maintenance/calibration flow
- `dm_serial_02_control_modes_demo.py`: SOP-02 dm-serial normal control loop (4 modes, no calibration/config writes)
- `dm_serial_03_status_demo.py`: SOP-03 dm-serial status-only monitor
- `dm_serial_04_enable_setzero_no_delay_demo.py`: SOP-04 dm-serial stress repro (`set_zero` then immediate control)
- `dm_serial_05_setzero_timing_ab_test.py`: SOP-05 dm-serial set-zero settle-time A/B test
- `dm_serial_06_recover_no_reboot_demo.py`: SOP-06 dm-serial software recovery (no host reboot)
- `dm_serial_07_enable_setzero_enable_rotate_demo.py`: SOP-07 dm-serial robust sequence (`disable -> set_zero -> enable -> control`)
- `dm_serial_08_negative_enable_setzero_guard_demo.py`: SOP-08 dm-serial negative test (`enable` state `set_zero` should be rejected by core guard)
- `dm_serial_leader_monitor_demo.py`: Damiao dm-serial leader monitor (enable-all + selected-ID full state stream)
- `robstride_wrapper_demo.py`: RobStride ping / clear-error / read-param / write-param / active-report / pos-vel / mit / vel demo
- `robstride_posvel_timing_demo.py`: RobStride legacy `pos-vel` / manual PP / manual CSP timing comparison
- `quad_vendor_pos_binding_demo.py`: 4-motor sync position demo via Python binding (no ws_gateway)
- `quad_vendor_binding_ws_demo.py`: Python binding WS bridge backend (for web UI control)
- `quad_vendor_binding_ws_demo.html`: simple web UI for `quad_vendor_binding_ws_demo.py`
- `hexfellow_canfd_demo.py`: Hexfellow CAN-FD demo (`mit` / `pos-vel` only)
- `full_modes_demo.py`: Damiao full-mode demo
- `pid_register_tune_demo.py`: Damiao register tuning
- `scan_ids_demo.py`: Damiao fast scan (legacy helper)
- `pos_ctrl_demo.py`: Damiao one-shot target position
- `multi_motor_ctrl_demo.py`: Damiao multi-motor control with `-id` / `-pos` one-to-one mapping
- `mit_pos_switch_demo.py`: two-phase mode switch demo (MIT then POS_VEL) for multi-motor targets
- `pos_repl_demo.py`: Damiao interactive position console

## Quick Run

Damiao:

```bash
PYTHONPATH=bindings/python/src python3 bindings/python/examples/python_wrapper_demo.py \
  --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 \
  --pos 0 --vel 0 --kp 20 --kd 1 --tau 0 --loop 20 --dt-ms 20
```

Damiao maintenance:

```bash
PYTHONPATH=bindings/python/src python3 bindings/python/examples/damiao_maintenance_demo.py \
  --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 \
  --can-timeout-ms 1000 --set-zero 0
```

Damiao register rw:

```bash
PYTHONPATH=bindings/python/src python3 bindings/python/examples/damiao_register_rw_demo.py \
  --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 \
  --read-f32-rid 21 --read-u32-rid 10 --store 0
```

Damiao dm-serial:

```bash
PYTHONPATH=bindings/python/src python3 bindings/python/examples/damiao_dm_serial_demo.py \
  --serial-port /dev/ttyACM0 --serial-baud 921600 --model 4310 \
  --motor-id 0x01 --feedback-id 0x11 --mode mit --loop 40 --dt-ms 20
```

SOP-01 dm-serial calibration:

```bash
PYTHONPATH=bindings/python/src python3 bindings/python/examples/dm_serial_01_calibration_demo.py \
  --serial-port /dev/ttyACM0 --serial-baud 921600 --model 4310 \
  --motor-id 0x04 --feedback-id 0x14 --set-zero 0 --can-timeout-ms 1000
```

SOP-02 dm-serial normal control (no calibration/config writes):

```bash
PYTHONPATH=bindings/python/src python3 bindings/python/examples/dm_serial_02_control_modes_demo.py \
  --serial-port /dev/ttyACM0 --serial-baud 921600 --model 4310 \
  --motor-id 0x04 --feedback-id 0x14 --mode pos-vel --pos 0.8 --vlim 1.0 --loop 100 --dt-ms 20
```

SOP-03 dm-serial status monitor:

```bash
PYTHONPATH=bindings/python/src python3 bindings/python/examples/dm_serial_03_status_demo.py \
  --serial-port /dev/ttyACM0 --serial-baud 921600 --model 4310 \
  --motor-id 0x04 --feedback-id 0x14 --loop 100 --dt-ms 50
```

SOP-05 dm-serial set-zero settle-time A/B test:

```bash
PYTHONPATH=bindings/python/src python3 bindings/python/examples/dm_serial_05_setzero_timing_ab_test.py \
  --serial-port /dev/ttyACM0 --serial-baud 921600 --model 4310 \
  --motor-id 0x04 --feedback-id 0x14 --settle-list-ms 0,50,100,200 --rounds 10 --ensure-timeout-ms 500
```

SOP-06 dm-serial software recovery without reboot:

```bash
PYTHONPATH=bindings/python/src python3 bindings/python/examples/dm_serial_06_recover_no_reboot_demo.py \
  --serial-port /dev/ttyACM0 --serial-baud 921600 --model 4310 \
  --motor-id 0x04 --feedback-id 0x14 --attempts 6 --timeout-ms 800
```

SOP-07 dm-serial robust set-zero + control sequence:

```bash
PYTHONPATH=bindings/python/src python3 bindings/python/examples/dm_serial_07_enable_setzero_enable_rotate_demo.py \
  --serial-port /dev/ttyACM0 --serial-baud 921600 --model 4310 \
  --motor-id 0x04 --feedback-id 0x14 --target-pos 3.0 --vlim 1.0 \
  --loop 50 --dt-ms 20 --ensure-timeout-ms 800 \
  --post-setzero-ms 0
```

SOP-08 dm-serial negative test (`enable` then `set_zero`, expect guard reject):

```bash
PYTHONPATH=bindings/python/src python3 bindings/python/examples/dm_serial_08_negative_enable_setzero_guard_demo.py \
  --serial-port /dev/ttyACM0 --serial-baud 921600 --model 4310 \
  --motor-id 0x04 --feedback-id 0x14 --ensure-timeout-ms 800
```

## dm-serial Timing Notes (Set-Zero Sequence)

- Observed issue: running `set_zero_position()` and immediately calling `ensure_mode(...)` can trigger `register 10 not received` timeouts.
- Root cause pattern: this is typically a dm-serial timing window (bridge latency + motor internal state switch), not a normal control-mode logic bug.
- Core guard rule: `set_zero_position()` is accepted only after `disable()`.
- Core settle rule: after `set_zero_position()`, a fixed `20 ms` settle is applied in core (not exposed as Python argument).
- Recommended sequence for robust control:
  1. `disable` (or `disable_all`)
  2. `set_zero_position`
  3. `enable` (or `enable_all`)
  4. `ensure_mode`
  5. run control loop
- If the timeout state is triggered, try software recovery first (`disable -> clear_error -> enable -> retry`) before rebooting host/device.

Damiao dm-serial leader monitor (selected IDs full-state stream):

```bash
PYTHONPATH=bindings/python/src python3 bindings/python/examples/dm_serial_leader_monitor_demo.py \
  --serial-port /dev/ttyACM0 --serial-baud 921600 --model 4310 \
  -id 0x04 0x07 --loop 10000 --dt-ms 20 --hold-mode mit-zero
```

RobStride:

```bash
PYTHONPATH=bindings/python/src python3 bindings/python/examples/robstride_wrapper_demo.py \
  --channel can0 --model rs-00 --motor-id 2 --feedback-id 0xFD --mode ping
```

RobStride active-report bring-up:

```bash
PYTHONPATH=bindings/python/src python3 bindings/python/examples/robstride_wrapper_demo.py \
  --channel can0 --model rs-00 --motor-id 2 --feedback-id 0xFD \
  --mode write-param --param-id 0x7026 --param-type u16 --param-value 3

PYTHONPATH=bindings/python/src python3 bindings/python/examples/robstride_wrapper_demo.py \
  --channel can0 --model rs-00 --motor-id 2 --feedback-id 0xFD \
  --mode active-report --active-report 1
```

RobStride position command:

```bash
PYTHONPATH=bindings/python/src python3 bindings/python/examples/robstride_wrapper_demo.py \
  --channel can0 --model rs-00 --motor-id 2 --feedback-id 0xFD \
  --mode pos-vel --pos 1.5 --vlim 1.0 --loc-kp 5.0 --loop 1 --dt-ms 20
```

RobStride POS_VEL timing comparison (legacy / PP / CSP):

```bash
# Default ack behavior: no wait for RobStride parameter writes.
PYTHONPATH=bindings/python/src python3 bindings/python/examples/robstride_posvel_timing_demo.py \
  --channel can0 --model rs-00 --motor-id 2 --feedback-id 0xFD \
  --mode all --pos 0.0 --vlim 1.0 --acc 10.0 --loop 100 --dt-ms 10

# Optional comparison: restore conservative parameter-write ack wait.
PYTHONPATH=bindings/python/src python3 bindings/python/examples/robstride_posvel_timing_demo.py \
  --channel can0 --model rs-00 --motor-id 2 --feedback-id 0xFD \
  --mode legacy --pos 0.0 --vlim 1.0 --loop 20 --dt-ms 10 --ack-timeout-ms 260
```

The demo prints per-mode call time and effective loop frequency. `pp` follows
`run_mode=1 -> enable -> vel_max(0x7024) -> acc_set(0x7025) -> loc_ref(0x7016)`;
`csp` follows `run_mode=5 -> enable -> limit_spd(0x7017) -> loc_ref(0x7016)`.

Python code can use the two dedicated RobStride interfaces directly:

```python
from motorbridge import Controller

with Controller("can0") as ctrl:
    motor = ctrl.add_robstride_motor(1, 0xFD, "rs-00")

    motor.robstride_send_pos_vel_pp(0.0, 1.0, 10.0)  # pos, vel_max, acc_set
    motor.robstride_send_pos_vel_csp(0.0, 1.0)       # pos, limit_spd
```

For high-rate loops, prepare PP/CSP once and write only `loc_ref(0x7016)` in the loop:

```python
from motorbridge import Controller

with Controller("can0") as ctrl:
    motor = ctrl.add_robstride_motor(1, 0xFD, "rs-00")

    motor.robstride_write_param_i8(0x7005, 5)      # run_mode = CSP
    motor.enable()
    motor.robstride_write_param_f32(0x7017, 1.0)   # limit_spd

    for pos in positions:
        motor.robstride_write_param_f32(0x7016, pos)
```

Hexfellow (CAN-FD only):

```bash
PYTHONPATH=bindings/python/src python3 bindings/python/examples/hexfellow_canfd_demo.py \
  --channel can0 --motor-id 0x01 --feedback-id 0x00 --mode mit --loop 20 --dt-ms 50
```

Unified vendor scan via CLI:

```bash
PYTHONPATH=bindings/python/src python3 -m motorbridge.cli scan \
  --vendor all --channel can0 --start-id 0x01 --end-id 0xFF
```

Multi-motor `pos-vel` (example: motor `4` and `7`, positions mapped by order):

```bash
PYTHONPATH=bindings/python/src python3 bindings/python/examples/multi_motor_ctrl_demo.py \
  --channel can0 --model 4310 --mode pos-vel \
  -id 4 7 -pos 0.8 -0.6 -vlim 1.2 1.2 --loop 200 --dt-ms 20
```

MIT/POS_VEL switch demo (POS_VEL-only run for motor `4` and `7`, target `-3`):

```bash
PYTHONPATH=bindings/python/src python3 bindings/python/examples/mit_pos_switch_demo.py \
  --channel can0 --model 4310 -id 4 7 \
  --trajectory -3 \
  --mit-hold-loops 0 --pos-hold-loops 50 \
  --dt-ms 20 --print-state 1
```

## Damiao Coverage Note

Damiao examples now cover the full high-level SDK usage surface:

- Control modes: `mit` / `pos-vel` / `vel` / `force-pos`
- Transport paths: `Controller(channel)` + `Controller.from_socketcanfd(...)` + `Controller.from_dm_serial(...)` + `Controller.from_dm_device(...)`
- Maintenance ops: `clear_error`, `set_zero_position`, `set_can_timeout_ms`, `request_feedback`
- Register APIs: `get/write f32`, `get/write u32`, `store_parameters`
- Scan helper and tuning workflows

## Scope Notes

- Recent multi-motor demo updates are mainly in `examples/*` and WS bridge scripts.
- `motor_core` behavior was not changed in this batch; WS-related tuning is done in `integrations/ws_gateway`.
