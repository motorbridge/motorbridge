# HighTorque 协议说明（2026-03 更新）

> **2026-09 更新**：vendor 层已迁移到 `ht_can` **v2.0.0**（MIT 走 `0x18000|id` 位打包帧，
> 固件 v4.6.0+，`kp/kd` 打包进帧；清错=停止帧 模式 0）。CLI/ws_gateway 的 MIT 路径也已
> 切 v2.0.0 并复用 vendor 编码。本文下述 v1.5.5 时代的帧族描述为历史记录，
> 现行状态以 [hightorque_fullchain_audit.md](hightorque_fullchain_audit.md) 为准。

<!-- channel-compat-note -->
## 通道兼容说明（PCAN + CANable candleLight/gs_usb + Damiao 串口桥 + DM_Device）

- Linux SocketCAN 直接使用已初始化的接口名：`can0`、`can1`。CANable 请刷 candleLight/gs_usb 固件，让系统识别为 `can0` 这类 SocketCAN 接口。
- 标准 CAN 推荐 PCAN 或 CANable candleLight/gs_usb。
- 仅 Damiao 可选两类适配器链路：串口桥 `--transport dm-serial --serial-port /dev/ttyACM0 --serial-baud 921600`，以及 DM_Device SDK `--transport dm-device --dm-device-type usb2canfd|usb2canfd-dual|linkx4c --dm-channel 0|1|2|3`。DM_Device 链路当前只配 Damiao 电机协议使用，适配器需处于 USB 模式。
- Linux SocketCAN 下 `--channel` 不要带 `@bitrate`（例如 `can0@1000000` 无效）。
- Windows（PCAN 后端）中，`can0/can1` 映射 `PCAN_USBBUS1/2`，可选 `@bitrate` 后缀。


本文用于统一说明 `rust_dm` 当前 HighTorque 支持状态，避免与旧版本“兼容层”描述混淆。

## 1. 当前实现状态

`rust_dm` 现在已经实现 HighTorque 的原生 `ht_can v1.5.5` 直连 CAN 路径，包含：

- Rust CLI：`motor_cli --vendor hightorque`
- C ABI：`motor_controller_add_hightorque_motor(...)`
- Python 绑定：`Controller.add_hightorque_motor(...)`
- C++ 绑定：`Controller::add_hightorque_motor(...)`

控制量纲统一为：

- 位置：`rad`
- 速度：`rad/s`
- 力矩：`Nm`

并在 HighTorque 后端按协议做缩放映射：

- `pos_raw = rad / (2π) * 10000`
- `vel_raw = rad/s / (2π) / 0.00025`
- `tqe_raw = Nm * 100`

## 2. 已验证的直连 CAN 指令族

当前实现按 `ht_can v1.5.5` 处理：

- 扫描/读取状态：`0x17 0x01`（8 字节）
- 反馈判定：`0x27 0x01`
- 位置/速度/力矩联合控制：`0x07 0x35`
- 速度控制：`0x07 0x07`
- 停机：`0x01 0x00 0x00`

说明：

- 扫描命中 ID 以回复帧 CAN ID 的高字节提取（例如 `0x100 -> id=1`）。
- 默认 `motor_id` 有效范围建议 `1..127`。

## 3. 与旧“兼容层”说明的关系

旧文档曾描述 `hightorque` 通过复用 RobStride 协议的兼容方案。该描述已过时。

现在仓库主线已切换为 HighTorque 原生协议实现，不再依赖 RobStride 协议兼容层来完成 HighTorque 控制。

## 4. 串口 CANboard 路径（独立链路）

HighTorque 官方生态中仍存在一条常见链路：

- 上位机 -> USB 虚拟串口 -> CANboard -> 电机 CAN/CAN-FD

该链路通常使用 `0xF7` 头的私有串口封包（CRC8/CRC16）。  
它与 `rust_dm` 当前的 SocketCAN 直连实现是两条不同传输层。

换言之：

- 现在你可以直接用 `can0` 跑原生 HighTorque 控制。
- 若未来需要“等价官方串口 SDK”能力，可再新增串口传输后端，而不影响现有 CAN 直连接口。

## 5. 推荐验证命令

```bash
# 1) 扫描
cargo run -p motor_cli -- --vendor hightorque --channel can0 --mode scan --start-id 1 --end-id 32

# 2) 读状态
cargo run -p motor_cli -- --vendor hightorque --channel can0 --motor-id 1 --mode read

# 3) 位置控制（180° = π rad）
cargo run -p motor_cli -- --vendor hightorque --channel can0 --motor-id 1 \
  --mode mit --pos 3.1415926 --vel 0.8 --tau 0.8 --loop 1
```
