# Python Binding Quickstart (pip-first)

This folder is a fresh, beginner-first quickstart for users who install from pip.

Goal: install package -> run scan -> run a motor example in minutes.

## 1) Install

```bash
python3 -m pip install motorbridge
```

If you want to test pre-release package from TestPyPI:

```bash
python3 -m pip install -i https://test.pypi.org/simple/ motorbridge==<version>
```

## 2) Hardware and Channel

- Linux SocketCAN channel examples: `can0`, `can1`
- Windows PCAN channel examples: `can0@1000000`, `can1@1000000`
- Ensure only one sender is writing to the bus while testing.

### Which transport should I use?

- `TRANSPORT = "auto"` or `"socketcan"`:
  use normal CAN path (`CHANNEL` is used).
- `TRANSPORT = "mcu-serial"`:
  use a generic MCU UART-to-CAN bridge (`SERIAL_PORT` / `SERIAL_BAUD` are used)
  for classic-CAN vendors (damiao / robstride / myactuator / hightorque).
  Note this is a different protocol from the Damiao-only `dm-serial` bridge.
- `TRANSPORT = "dm-serial"`:
  use Damiao serial bridge (`SERIAL_PORT` / `SERIAL_BAUD` are used), Damiao only.
- `TRANSPORT = "dm-device"`:
  use DaMiao DM_Device SDK (`DM_DEVICE_TYPE` / `DM_CHANNEL` are used), Damiao only.
  The adapter must be in USB mode. In scan scripts, set `DM_CHANNEL = None` to
  scan every channel for the selected adapter: `usb2canfd` => `0`,
  `usb2canfd-dual` => `0`/`1`, `linkx4c` => SDK channels `0..3`.
  Set `DM_CHANNEL` when you want one physical channel.

## 3) Quick Commands (no source checkout required)

```bash
# Unified scan from installed CLI
motorbridge-cli scan --vendor all --channel can0 --start-id 1 --end-id 255

# Single Damiao control (example IDs)
motorbridge-cli run --vendor damiao --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 \
  --mode pos-vel --pos 1.0 --vlim 1.0 --loop 60 --dt-ms 20
```

## 4) Course scripts in this folder

All runnable scripts in `get_started` now live under `courses/`.
For beginners, this is easier to follow and avoids duplicate entry points.

### Config constants (simple meaning)

- `TRANSPORT`: `auto/socketcan/socketcanfd/mcu-serial/dm-serial/dm-device`
- `CHANNEL`: CAN interface name (`can0`, `can1`, `can0@1000000`, ...)
- `VENDOR`: scan target vendor (`all` is most common)
- `MOTOR_ID` / `FEEDBACK_ID`: motor command/feedback IDs
- `MODEL`: motor model string
- `TARGET_POS` / `POS`: target position in radians
- `V_LIMIT`: velocity limit for position mode
- `LOOP`: send loop count
- `DT_MS`: control period in ms
- `SERIAL_PORT` / `SERIAL_BAUD`: used for `mcu-serial` and `dm-serial`
- `DM_DEVICE_TYPE` / `DM_CHANNEL`: used only for `dm-device`; supported
  device types are `usb2canfd`, `usb2canfd-dual`, and `linkx4c`
  (`DM_CHANNEL=None` means all channels for scan)

## 5) Course Series (workflow-first)

If you want a strict real-world workflow course, use `courses/`:

- `courses/00-enable-and-status.py`
- `courses/01-scan.py`
- `courses/02-register-rw.py`
- `courses/03-mode-switch-method.py`
- `courses/04-mode-mit.py`
- `courses/05-mode-pos-vel.py`
- `courses/06-mode-vel.py`
- `courses/07-mode-force-pos.py`
- `courses/08-mode-mixed-switch.py`
- `courses/09-multi-motor.py`

Example:

```bash
python3 bindings/python/get_started/courses/00-enable-and-status.py
python3 bindings/python/get_started/courses/01-scan.py
python3 bindings/python/get_started/courses/03-mode-switch-method.py
```

## 6) Run now (courses only)

```bash
python3 bindings/python/get_started/courses/00-enable-and-status.py
python3 bindings/python/get_started/courses/01-scan.py
python3 bindings/python/get_started/courses/09-multi-motor.py
```

## 7) Common Issues

- `os error 105`: bus TX is too fast or another process is sending; increase `--dt-ms` to 30/50.
- no motor response: verify CAN wiring, bitrate, motor/feedback IDs.
- CANable users: initialize the candleLight/gs_usb interface before running examples.

## 8) `poll_feedback_once()` version note

- `<= v0.1.6`: keep manual `poll_feedback_once()` in status-query scripts.
- `v0.1.7+`: background polling is enabled by default, so manual poll is usually optional.
- Course scripts keep `poll_feedback_once()` for backward-compatible teaching style.

## 9) What to read next

- Python API overview: `bindings/python/README.md`
- Full examples catalog: `bindings/python/examples/README.md`
- CLI full docs: `motor_cli/README.md`
