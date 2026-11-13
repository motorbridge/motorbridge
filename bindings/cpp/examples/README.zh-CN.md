# C++ 示例程序

<!-- channel-compat-note -->
## 通道兼容说明（PCAN + CANable candleLight/gs_usb + CAN-FD + MCU 串口转 CAN 桥 + Damiao 串口桥 + DM_Device）

- Linux SocketCAN 直接使用已初始化的接口名：`can0`、`can1`。CANable 请刷 candleLight/gs_usb 固件，让系统识别为 `can0` 这类 SocketCAN 接口。
- 标准 CAN 推荐 PCAN 或 CANable candleLight/gs_usb。
- Hexfellow 示例需使用 CAN-FD 路径（`Controller::from_socketcanfd(...)` / CLI `--transport socketcanfd`）。
- 仅 Damiao 可选两类适配器链路：串口桥 `--transport dm-serial --serial-port /dev/ttyACM0 --serial-baud 921600`，以及 DM_Device SDK `--transport dm-device --dm-device-type usb2canfd|usb2canfd-dual|linkx4c --dm-channel 0|1|2|3`。DM_Device 链路当前只配 Damiao 电机协议使用，适配器需处于 USB 模式。
- 通用 MCU 串口转 CAN 桥：`--transport mcu-serial --serial-port /dev/ttyACM0 --serial-baud 921600`。注意与 Damiao 专用串口桥 `dm-serial` 是两种协议：`mcu-serial` 由桥固件透传经典 CAN 帧，`dm-serial` 承载 Damiao 串口协议。适配经典 CAN 全品牌（damiao / robstride / myactuator / hightorque / huayi；hexfellow 仅 CAN-FD 不适用）；桥固件在 bus-off / 发送失败 / 接收丢弃时会主动上报结构化链路状态；四个前端（motor_abi / Python / motor_cli / ws_gateway）均支持。
- Damiao 串口桥完整接口与命令模板见 `motor_cli/README.zh-CN.md` 第 `3.6` 节（英文见 `motor_cli/README.md`）。
- Linux SocketCAN 下 `--channel` 不要带 `@bitrate`（例如 `can0@1000000` 无效）。
- Windows（PCAN 后端）中，`can0/can1` 映射 `PCAN_USBBUS1/2`，可选 `@bitrate` 后缀。

## Damiao 置零顺序说明

- 推荐顺序：`disable -> set_zero_position -> enable -> ensure_mode -> control`。
- `set_zero_position` 由核心层防护，要求电机处于失能状态。


在仓库根目录构建:

```bash
cargo build -p motor_abi --release
cmake -S bindings/cpp -B bindings/cpp/build \
  -DMOTORBRIDGE_ABI_LIBRARY=$PWD/target/release/libmotor_abi.so
cmake --build bindings/cpp/build -j
```

文件说明:

- `cpp_wrapper_demo.cpp`: Damiao MIT 循环
- `robstride_wrapper_demo.cpp`: RobStride 的 ping / read-param / mit / vel 示例
- `hexfellow_canfd_demo.cpp`: Hexfellow CAN-FD 示例（仅 `mit` / `pos-vel`）
- `full_modes_demo.cpp`: Damiao 全模式控制
- `pid_register_tune_demo.cpp`: Damiao 调参
- `scan_ids_demo.cpp`: Damiao 扫描（历史辅助）
- `pos_ctrl_demo.cpp`: Damiao 目标位置
- `pos_repl_demo.cpp`: Damiao 交互式位置控制台

通过 Rust CLI 统一扫描:

```bash
cargo run -p motor_cli --release -- \
  --vendor all --channel can0 --mode scan --start-id 1 --end-id 255
```
