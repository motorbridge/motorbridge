# C ABI Examples

<!-- channel-compat-note -->
## Channel Compatibility (PCAN + CANable candleLight/gs_usb + CAN-FD + MCU Serial Bridge + Damiao Serial Bridge)

- Linux SocketCAN uses prepared interfaces directly: `can0`, `can1`. For CANable, use candleLight/gs_usb firmware so it appears as a SocketCAN interface such as `can0`.
- Use PCAN or CANable candleLight/gs_usb for standard CAN.
- Hexfellow examples require CAN-FD path (`motor_controller_new_socketcanfd(...)` / CLI `--transport socketcanfd`).
- Damiao-only serial bridge transport is also available in CLI (`--transport dm-serial --serial-port /dev/ttyACM0 --serial-baud 921600`).
- Generic MCU UART-to-CAN bridge: `--transport mcu-serial --serial-port /dev/ttyACM0 --serial-baud 921600`. Note this is a different protocol from the Damiao-only `dm-serial` bridge: `mcu-serial` relays classic CAN frames through the bridge firmware, while `dm-serial` carries the Damiao serial protocol. It covers all classic-CAN vendors (damiao / robstride / myactuator / hightorque / huayi; hexfellow is CAN-FD-only and not supported); the bridge firmware proactively reports structured link status on bus-off / TX failure / RX drop, and all four frontends (motor_abi / Python / motor_cli / ws_gateway) support it.
- Full Damiao serial-bridge interface list and command patterns are documented in `motor_cli/README.md` (section `3.6` in `motor_cli/README.zh-CN.md`).
- On Linux SocketCAN, do not append bitrate in `--channel` (for example `can0@1000000` is invalid).
- On Windows (PCAN backend), `can0/can1` map to `PCAN_USBBUS1/2`; optional `@bitrate` suffix is supported.


Direct C demos for `motor_abi`.

> Chinese version: [README.zh-CN.md](README.zh-CN.md)

## Files

- `c_abi_demo.c`: unified demo for Damiao + RobStride
- `hexfellow_canfd_demo.c`: Hexfellow CAN-FD demo (`mit` / `pos-vel` only)

Vendor coverage:

- Damiao: `enable`, `disable`, `mit`, `pos-vel`, `vel`, `force-pos`
- RobStride: `ping`, `enable`, `disable`, `mit`, `pos-vel`, `vel`, `read-param`, `write-param`
  - note: torque/current is parameter-level only (`write-param` on `iq_ref`/limits), not a unified high-level mode
- Hexfellow (CAN-FD only): `mit`, `pos-vel`

## Build

```bash
cargo build -p motor_abi --release
cc examples/c/c_abi_demo.c -I motor_abi/include -L target/release -lmotor_abi -o c_abi_demo
cc examples/c/hexfellow_canfd_demo.c -I motor_abi/include -L target/release -lmotor_abi -o hexfellow_canfd_demo
LD_LIBRARY_PATH=target/release ./c_abi_demo --help
```

## Examples

Damiao MIT:

```bash
LD_LIBRARY_PATH=target/release ./c_abi_demo \
  --vendor damiao --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 \
  --mode mit --pos 0 --vel 0 --kp 20 --kd 1 --tau 0 --loop 50 --dt-ms 20
```

RobStride ping:

```bash
LD_LIBRARY_PATH=target/release ./c_abi_demo \
  --vendor robstride --channel can0 --model rs-00 --motor-id 127 --mode ping
```

RobStride read position parameter:

```bash
LD_LIBRARY_PATH=target/release ./c_abi_demo \
  --vendor robstride --channel can0 --model rs-00 --motor-id 127 \
  --mode read-param --param-id 0x7019 --param-type f32
```

RobStride low-gain MIT:

```bash
LD_LIBRARY_PATH=target/release ./c_abi_demo \
  --vendor robstride --channel can0 --model rs-00 --motor-id 127 \
  --mode mit --ensure-strict 1 --pos 0.5 --vel 0 --kp 20.0 --kd 0.5 --tau 0 --loop 100 --dt-ms 20
```

Hexfellow (CAN-FD only):

```bash
LD_LIBRARY_PATH=target/release ./hexfellow_canfd_demo \
  --channel can0 --motor-id 0x01 --feedback-id 0x00 --mode mit --loop 20 --dt-ms 50
```
