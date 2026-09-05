# 测试指南

<!-- channel-compat-note -->
## 通道兼容说明（PCAN + CANable candleLight/gs_usb + Damiao 串口桥 + DM_Device）

- Linux SocketCAN 直接使用已初始化的接口名：`can0`、`can1`。CANable 请刷 candleLight/gs_usb 固件，让系统识别为 `can0` 这类 SocketCAN 接口。
- 标准 CAN 推荐 PCAN 或 CANable candleLight/gs_usb。
- 仅 Damiao 可选两类适配器链路：串口桥 `--transport dm-serial --serial-port /dev/ttyACM0 --serial-baud 921600`，以及 DM_Device SDK `--transport dm-device --dm-device-type usb2canfd|usb2canfd-dual|linkx4c --dm-channel 0|1|2|3`。DM_Device 链路当前只配 Damiao 电机协议使用，适配器需处于 USB 模式。
- Linux SocketCAN 下 `--channel` 不要带 `@bitrate`（例如 `can0@1000000` 无效）。
- Windows（PCAN 后端）中，`can0/can1` 映射 `PCAN_USBBUS1/2`，可选 `@bitrate` 后缀。


当前测试重点是可重复的单元测试（协议与参数解析），以及工作区级编译与测试检查。

## 当前覆盖范围

- `motor_core`：
  - Windows PCAN 通道/波特率解析与校验
  - `CoreController` + Fake `CanBus` 集成测试：
    - 重复设备 ID 拒绝
    - 反馈帧路由
    - enable/disable 批量下发
    - shutdown 生命周期行为
- `motor_vendor_damiao`：
  - 协议编解码基础逻辑
  - 型号匹配/推荐逻辑
- `motor_vendor_robstride`：
  - 扩展 CAN ID 组装/拆解
  - ping/参数编码与校验
- `motor_cli`：
  - 参数解析辅助函数
  - RobStride 参数值类型解析

## 运行全部测试

```bash
cargo test --workspace --all-targets
```

## 发布测试记录

每次准备发版时，都应该新增一份可重复执行的 release test note，记录 core、Rust CLI、Python binding/CLI、硬件在环命令和危险命令边界。

- 当前版本：[`release_test_notes/0.5.3.md`](../../release_test_notes/0.5.3.md)

## 本地建议质量门禁

```bash
cargo check --workspace
cargo test --workspace --all-targets
```

## 硬件在环（手动验证）

自动化测试默认不依赖真实 CAN 硬件。硬件验证建议固定流程：

1. 扫描设备
2. 使能/失能
3. 下发控制模式命令
4. 回读反馈状态

可直接复用根目录 `README` 的 Linux 命令，以及 Windows 实验章节中的 `can0@1000000` 命令。

可靠性辅助脚本：

- [`tools/reliability/README.zh-CN.md`](../../tools/reliability/README.zh-CN.md)
- `tools/reliability/reliability_runner.py`

## 下一步增强建议

- 扩展长时间硬件在环矩阵（不同适配器、不同总线负载）
- 增加跨平台 compare-scan 的周期性任务，并固定容差策略
