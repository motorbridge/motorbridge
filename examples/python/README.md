# Python ctypes Examples

<!-- channel-compat-note -->
## Channel Compatibility (PCAN + CANable candleLight/gs_usb + MCU Serial Bridge + Damiao Serial Bridge)

- Linux SocketCAN uses prepared interfaces directly: `can0`, `can1`. For CANable, use candleLight/gs_usb firmware so it appears as a SocketCAN interface such as `can0`.
- Use PCAN or CANable candleLight/gs_usb for standard CAN.
- Damiao-only serial bridge transport is also available in CLI (`--transport dm-serial --serial-port /dev/ttyACM0 --serial-baud 921600`).
- Generic MCU UART-to-CAN bridge: `--transport mcu-serial --serial-port /dev/ttyACM0 --serial-baud 921600`. Note this is a different protocol from the Damiao-only `dm-serial` bridge: `mcu-serial` relays classic CAN frames through the bridge firmware, while `dm-serial` carries the Damiao serial protocol. It covers all classic-CAN vendors (damiao / robstride / myactuator / hightorque; hexfellow is CAN-FD-only and not supported); the bridge firmware proactively reports structured link status on bus-off / TX failure / RX drop, and all four frontends (motor_abi / Python / motor_cli / ws_gateway) support it.
- Full Damiao serial-bridge interface list and command patterns are documented in `motor_cli/README.md` (section `3.6` in `motor_cli/README.zh-CN.md`).
- On Linux SocketCAN, do not append bitrate in `--channel` (for example `can0@1000000` is invalid).
- On Windows (PCAN backend), `can0/can1` map to `PCAN_USBBUS1/2`; optional `@bitrate` suffix is supported.


Python demos that call the Rust ABI directly through `ctypes`.

> Chinese version: [README.zh-CN.md](README.zh-CN.md)

## File

- `python_ctypes_demo.py`: unified two-vendor demo
- `four_vendor_pos_sync.py`: concurrent multi-vendor position sync helper

Vendor coverage:

- Damiao: `enable`, `disable`, `mit`, `pos-vel`, `vel`, `force-pos`
- RobStride: `ping`, `enable`, `disable`, `mit`, `pos-vel`, `vel`, `read-param`, `write-param`
  - note: torque/current is parameter-level only (`write-param` on `iq_ref`/limits), not a unified high-level mode

## Build and Run

```bash
cargo build -p motor_abi --release
python3 examples/python/python_ctypes_demo.py --help
```

## Examples

Damiao MIT:

```bash
python3 examples/python/python_ctypes_demo.py \
  --vendor damiao --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 \
  --mode mit --pos 0 --vel 0 --kp 20 --kd 1 --tau 0 --loop 50 --dt-ms 20
```

RobStride ping:

```bash
python3 examples/python/python_ctypes_demo.py \
  --vendor robstride --channel can0 --model rs-00 --motor-id 127 --mode ping
```

RobStride read parameter:

```bash
python3 examples/python/python_ctypes_demo.py \
  --vendor robstride --channel can0 --model rs-00 --motor-id 127 \
  --mode read-param --param-id 0x7019 --param-type f32
```

RobStride write parameter:

```bash
python3 examples/python/python_ctypes_demo.py \
  --vendor robstride --channel can0 --model rs-00 --motor-id 127 \
  --mode write-param --param-id 0x700A --param-type f32 --param-value 0.2
```

Multi-vendor position sync (Damiao x2 + MyActuator + HighTorque):

```bash
python3 examples/python/four_vendor_pos_sync.py \
  damiao 0x01 damiao 0x07 myactuator 1 hightorque 1 \
  --pos 1.57 \
  --damiao-model-by-id "0x01=4340P,0x07=4310" \
  --stagger-ms 50
```

Dry-run (print generated `motor_cli` commands only):

```bash
python3 examples/python/four_vendor_pos_sync.py \
  damiao:0x01 damiao:0x07 myactuator:1 hightorque:1 \
  --pos 1.57 --dry-run
```
