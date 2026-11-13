# C++ Example Programs

<!-- channel-compat-note -->
## Channel Compatibility (PCAN + CANable candleLight/gs_usb + CAN-FD + MCU Serial Bridge + Damiao Serial Bridge + DM_Device)

- Linux SocketCAN uses prepared interfaces directly: `can0`, `can1`. For CANable, use candleLight/gs_usb firmware so it appears as a SocketCAN interface such as `can0`.
- Use PCAN or CANable candleLight/gs_usb for standard CAN.
- Hexfellow examples require CAN-FD path (`Controller::from_socketcanfd(...)` / CLI `--transport socketcanfd`).
- Damiao-only adapter transports are available in CLI: serial bridge (`--transport dm-serial --serial-port /dev/ttyACM0 --serial-baud 921600`) and DM_Device SDK (`--transport dm-device --dm-device-type usb2canfd|usb2canfd-dual|linkx4c --dm-channel 0|1|2|3`; Damiao motors only; adapter must be in USB mode).
- Generic MCU UART-to-CAN bridge: `--transport mcu-serial --serial-port /dev/ttyACM0 --serial-baud 921600`. Note this is a different protocol from the Damiao-only `dm-serial` bridge: `mcu-serial` relays classic CAN frames through the bridge firmware, while `dm-serial` carries the Damiao serial protocol. It covers all classic-CAN vendors (damiao / robstride / myactuator / hightorque / huayi; hexfellow is CAN-FD-only and not supported); the bridge firmware proactively reports structured link status on bus-off / TX failure / RX drop, and all four frontends (motor_abi / Python / motor_cli / ws_gateway) support it.
- Full Damiao serial-bridge interface list and command patterns are documented in `motor_cli/README.md` (section `3.6` in `motor_cli/README.zh-CN.md`).
- On Linux SocketCAN, do not append bitrate in `--channel` (for example `can0@1000000` is invalid).
- On Windows (PCAN backend), `can0/can1` map to `PCAN_USBBUS1/2`; optional `@bitrate` suffix is supported.

## Damiao Set-Zero Sequence Note

- Recommended order: `disable -> set_zero_position -> enable -> ensure_mode -> control`.
- `set_zero_position` is guarded in core and requires disabled state.


Build from repo root:

```bash
cargo build -p motor_abi --release
cmake -S bindings/cpp -B bindings/cpp/build \
  -DMOTORBRIDGE_ABI_LIBRARY=$PWD/target/release/libmotor_abi.so
cmake --build bindings/cpp/build -j
```

Files:

- `cpp_wrapper_demo.cpp`: Damiao MIT loop
- `robstride_wrapper_demo.cpp`: RobStride ping / read-param / mit / vel demo
- `hexfellow_canfd_demo.cpp`: Hexfellow CAN-FD demo (`mit` / `pos-vel` only)
- `full_modes_demo.cpp`: Damiao full-mode control
- `pid_register_tune_demo.cpp`: Damiao tuning
- `scan_ids_demo.cpp`: Damiao scan (legacy helper)
- `pos_ctrl_demo.cpp`: Damiao position target
- `pos_repl_demo.cpp`: Damiao interactive position console

Unified scan via Rust CLI:

```bash
cargo run -p motor_cli --release -- \
  --vendor all --channel can0 --mode scan --start-id 1 --end-id 255
```
