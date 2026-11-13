# Examples Index

<!-- channel-compat-note -->
## Channel Compatibility (PCAN + CANable candleLight/gs_usb + MCU Serial Bridge + Damiao Serial Bridge)

- Linux SocketCAN uses prepared interfaces directly: `can0`, `can1`. For CANable, use candleLight/gs_usb firmware so it appears as a SocketCAN interface such as `can0`.
- Use PCAN or CANable candleLight/gs_usb for standard CAN.
- Damiao-only serial bridge transport is also available in CLI (`--transport dm-serial --serial-port /dev/ttyACM0 --serial-baud 921600`).
- Generic MCU UART-to-CAN bridge: `--transport mcu-serial --serial-port /dev/ttyACM0 --serial-baud 921600`. Note this is a different protocol from the Damiao-only `dm-serial` bridge: `mcu-serial` relays classic CAN frames through the bridge firmware, while `dm-serial` carries the Damiao serial protocol. It covers all classic-CAN vendors (damiao / robstride / myactuator / hightorque / huayi; hexfellow is CAN-FD-only and not supported); the bridge firmware proactively reports structured link status on bus-off / TX failure / RX drop, and all four frontends (motor_abi / Python / motor_cli / ws_gateway) support it.
- Full Damiao serial-bridge interface list and command patterns are documented in `motor_cli/README.md` (section `3.6` in `motor_cli/README.zh-CN.md`).
- On Linux SocketCAN, do not append bitrate in `--channel` (for example `can0@1000000` is invalid).
- On Windows (PCAN backend), `can0/can1` map to `PCAN_USBBUS1/2`; optional `@bitrate` suffix is supported.


Cross-language example entry for the current `motorbridge` stack.

> Chinese version: [README.zh-CN.md](README.zh-CN.md)

## Coverage

- Rust CLI: `motor_cli/src/main.rs`
- C ABI demo: `examples/c/c_abi_demo.c`
- C++ ABI demo: `examples/cpp/cpp_abi_demo.cpp`
- Python ctypes demo: `examples/python/python_ctypes_demo.py`
- Multi-vendor position sync script: `examples/python/four_vendor_pos_sync.py`
- WS quad sync HMI: `examples/web/ws_quad_sync_hmi.html`
- WS quad independent-slider HMI: `examples/http_quad_control_demo/README.zh-CN.md`
- Python SDK demos: `bindings/python/examples/*`
- C++ wrapper demos: `bindings/cpp/examples/*`
- Damiao tuning reference:
  - `../motor_cli/DAMIAO_API.md`
  - `../motor_cli/DAMIAO_API.zh-CN.md`
- RobStride API/parameter reference:
  - `../motor_cli/ROBSTRIDE_API.md`
  - `../motor_cli/ROBSTRIDE_API.zh-CN.md`
- MyActuator command/mode reference:
  - `../motor_cli/MYACTUATOR_API.md`
  - `../motor_cli/MYACTUATOR_API.zh-CN.md`

## Vendor Support in Examples

- Damiao:
  - modes: `enable`, `disable`, `mit`, `pos-vel`, `vel`, `force-pos`
  - register / ID workflows remain available through CLI, Python SDK, and calibration tools
- RobStride:
  - modes: `ping`, `enable`, `disable`, `mit`, `vel`, `read-param`, `write-param`
  - parameter examples use the RobStride ABI and binding helpers
- MyActuator:
  - modes: `scan`, `enable`, `disable`, `stop`, `status`, `current`, `vel`, `pos`, `version`, `mode-query`
  - CLI input uses radians/rad-s for `pos`/`vel` (`motor_cli` converts to protocol degrees internally)

## CAN Setup

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up
ip -details link show can0
```

## Experimental Windows Support (PCAN-USB)

Linux remains the primary target. Windows support is experimental and currently uses PEAK PCAN.

- Install PEAK PCAN driver + PCAN-Basic runtime (`PCANBasic.dll`).
- Use `can0@1000000` as the Windows channel form.

Windows quick validation commands:

```bash
cargo run -p motor_cli --release -- --vendor damiao --channel can0@1000000 --model 4340P --motor-id 0x01 --feedback-id 0x11 --mode scan --start-id 1 --end-id 16
cargo run -p motor_cli --release -- --vendor damiao --channel can0@1000000 --model 4340P --motor-id 0x01 --feedback-id 0x11 --mode pos-vel --pos 3.1416 --vlim 2.0 --loop 1 --dt-ms 20
cargo run -p motor_cli --release -- --vendor damiao --channel can0@1000000 --model 4310 --motor-id 0x07 --feedback-id 0x17 --mode pos-vel --pos 3.1416 --vlim 2.0 --loop 1 --dt-ms 20
```

## Quick Start

Damiao with Rust CLI:

```bash
cargo run -p motor_cli --release -- \
  --vendor damiao --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 \
  --mode mit --pos 0 --vel 0 --kp 20 --kd 1 --tau 0 --loop 50 --dt-ms 20
```

RobStride with Rust CLI:

```bash
cargo run -p motor_cli --release -- \
  --vendor robstride --channel can0 --model rs-00 --motor-id 127 \
  --mode ping
```

RobStride parameter read:

```bash
cargo run -p motor_cli --release -- \
  --vendor robstride --channel can0 --model rs-00 --motor-id 127 \
  --mode read-param --param-id 0x7019
```

MyActuator with Rust CLI:

```bash
cargo run -p motor_cli --release -- \
  --vendor myactuator --channel can0 --model X8 --motor-id 1 --feedback-id 0x241 \
  --mode pos --pos 3.1416 --max-speed 5.236 --loop 1 --dt-ms 50
```

## Cross-language ABI Demos

Python ctypes:

```bash
cargo build -p motor_abi --release
python3 examples/python/python_ctypes_demo.py --vendor robstride --channel can0 --model rs-00 --motor-id 127 --mode ping
python3 examples/python/python_ctypes_demo.py --vendor damiao --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 --mode mit
```

C:

```bash
cargo build -p motor_abi --release
cc examples/c/c_abi_demo.c -I motor_abi/include -L target/release -lmotor_abi -o c_abi_demo
LD_LIBRARY_PATH=target/release ./c_abi_demo --vendor robstride --channel can0 --model rs-00 --motor-id 127 --mode ping
LD_LIBRARY_PATH=target/release ./c_abi_demo --vendor damiao --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 --mode mit
```

C++:

```bash
cargo build -p motor_abi --release
g++ -std=c++17 examples/cpp/cpp_abi_demo.cpp -I motor_abi/include -L target/release -lmotor_abi -o cpp_abi_demo
LD_LIBRARY_PATH=target/release ./cpp_abi_demo --vendor robstride --channel can0 --model rs-00 --motor-id 127 --mode ping
LD_LIBRARY_PATH=target/release ./cpp_abi_demo --vendor damiao --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 --mode mit
```

## Recommended Higher-level Examples

- Python SDK:
  - `bindings/python/examples/python_wrapper_demo.py`
  - `bindings/python/examples/robstride_wrapper_demo.py`
- C++ wrapper:
  - `bindings/cpp/examples/cpp_wrapper_demo.cpp`
  - `bindings/cpp/examples/robstride_wrapper_demo.cpp`

## Validation Checklist (suggested order)

1. Scan all vendors on the same bus.
2. Verify Damiao control path (MIT or velocity).
3. Verify RobStride control path (ping/read-param/velocity).
4. Verify MyActuator control path (position or velocity).
5. Verify Python binding demos (Damiao + RobStride).
6. Verify C++ binding demos (Damiao + RobStride).

Quick commands:

```bash
# 1) Unified scan
cargo run -p motor_cli --release -- --vendor all --channel can0 --mode scan --start-id 1 --end-id 255

# 2) Damiao quick velocity
cargo run -p motor_cli --release -- --vendor damiao --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 --mode vel --vel 0.5 --loop 40 --dt-ms 50

# 3) RobStride ping + velocity
cargo run -p motor_cli --release -- --vendor robstride --channel can0 --model rs-06 --motor-id 127 --feedback-id 0xFD --mode ping
cargo run -p motor_cli --release -- --vendor robstride --channel can0 --model rs-06 --motor-id 127 --feedback-id 0xFD --mode vel --vel 0.3 --loop 40 --dt-ms 50

# 4) MyActuator position (radians)
cargo run -p motor_cli --release -- --vendor myactuator --channel can0 --model X8 --motor-id 1 --feedback-id 0x241 --mode pos --pos 3.1416 --max-speed 5.236 --loop 1 --dt-ms 50

# 5) Multi-vendor position sync helper (Damiao x2 + MyActuator + HighTorque)
python3 examples/python/four_vendor_pos_sync.py \
  damiao 0x01 damiao 0x07 myactuator 1 hightorque 1 \
  --pos 1.57 --damiao-model-by-id "0x01=4340P,0x07=4310" --stagger-ms 50

# 6) Web HMI (one slider drives 4 motors to same angle via ws_gateway)
cargo run -p ws_gateway --release -- --bind 127.0.0.1:9002 --vendor damiao --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 --dt-ms 20
python3 -m http.server 18080
# open: http://127.0.0.1:18080/examples/web/ws_quad_sync_hmi.html
```

## Notes

- `id-dump` is a Damiao-oriented workflow; `id-set` supports Damiao and RobStride device ID updates. Unified `scan` is available in Rust CLI (`--vendor all`) and Python SDK CLI (`motorbridge.cli scan --vendor all`).
- RobStride examples focus on ping, parameter access, MIT, and velocity control.
