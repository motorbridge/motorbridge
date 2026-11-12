# Cross-language Examples

<!-- channel-compat-note -->
## Channel Compatibility (PCAN + CANable candleLight/gs_usb + MCU Serial Bridge + Damiao Serial Bridge + DM_Device)

- Linux SocketCAN uses prepared interfaces directly: `can0`, `can1`. For CANable, use candleLight/gs_usb firmware so it appears as a SocketCAN interface such as `can0`.
- Use PCAN or CANable candleLight/gs_usb for standard CAN.
- Damiao-only adapter transports are available in CLI: serial bridge (`--transport dm-serial --serial-port /dev/ttyACM0 --serial-baud 921600`) and DM_Device SDK (`--transport dm-device --dm-device-type usb2canfd|usb2canfd-dual|linkx4c --dm-channel 0|1|2|3`; Damiao motors only; adapter must be in USB mode).
- Generic MCU UART-to-CAN bridge: `--transport mcu-serial --serial-port /dev/ttyACM0 --serial-baud 921600`. Note this is a different protocol from the Damiao-only `dm-serial` bridge: `mcu-serial` relays classic CAN frames through the bridge firmware, while `dm-serial` carries the Damiao serial protocol. It covers all classic-CAN vendors (damiao / robstride / myactuator / hightorque; hexfellow is CAN-FD-only and not supported); the bridge firmware proactively reports structured link status on bus-off / TX failure / RX drop, and all four frontends (motor_abi / Python / motor_cli / ws_gateway) support it.
- On Linux SocketCAN, do not append bitrate in `--channel` (for example `can0@1000000` is invalid).
- On Windows (PCAN backend), `can0/can1` map to `PCAN_USBBUS1/2`; optional `@bitrate` suffix is supported.


## Index

- Rust CLI: `motor_cli/src/main.rs`
- C ABI: `examples/c/c_abi_demo.c`
- C++ ABI: `examples/cpp/cpp_abi_demo.cpp`
- Python ctypes: `examples/python/python_ctypes_demo.py`
- Multi-vendor position sync script: `examples/python/four_vendor_pos_sync.py`
- WS quad sync HMI: `examples/web/ws_quad_sync_hmi.html`
- Python SDK: `bindings/python/examples/*`
- C++ wrapper: `bindings/cpp/examples/*`

## Quick Commands

```bash
cargo build -p motor_abi --release
```

Damiao Python ctypes:

```bash
python3 examples/python/python_ctypes_demo.py --vendor damiao --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 --mode mit
```

RobStride Python ctypes:

```bash
python3 examples/python/python_ctypes_demo.py --vendor robstride --channel can0 --model rs-00 --motor-id 127 --mode ping
```

RobStride C ABI:

```bash
cc examples/c/c_abi_demo.c -I motor_abi/include -L target/release -lmotor_abi -o c_abi_demo
LD_LIBRARY_PATH=target/release ./c_abi_demo --vendor robstride --channel can0 --model rs-00 --motor-id 127 --mode read-param --param-id 0x7019 --param-type f32
```

Multi-vendor position sync helper:

```bash
python3 examples/python/four_vendor_pos_sync.py \
  damiao 0x01 damiao 0x07 myactuator 1 hightorque 1 \
  --pos 1.57 \
  --damiao-model-by-id "0x01=4340P,0x07=4310" \
  --stagger-ms 50
```

Web HMI (single slider controls 4 motors to the same angle):

```bash
cargo run -p ws_gateway --release -- --bind 127.0.0.1:9002 --vendor damiao --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 --dt-ms 20
python3 -m http.server 18080
# open http://127.0.0.1:18080/examples/web/ws_quad_sync_hmi.html
```
