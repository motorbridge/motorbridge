# motorbridge 文档（中文）

本文档与当前 `main` 分支实现保持同步。

## 文档导航关系图

```mermaid
flowchart TD
  IDX["index.md"] --> ARCH["architecture.md"]
  IDX --> DEV["devices.md"]
  IDX --> CLI["cli.md"]
  IDX --> ABI["abi.md"]
  IDX --> EX["examples.md"]
  IDX --> EXT["extending.md"]
  IDX --> WIN["windows_distribution.md"]
  IDX --> TST["testing.md"]
  IDX --> CANDBG["can_debugging.md"]
  IDX --> PCAN["pcan_reinit_limit.md"]
  IDX --> DIST["distribution_channels.md"]
  IDX --> OPMAN["operation_manual.md"]
  IDX --> STUDIO["motorbridge-studio（独立仓库）"]
  IDX --> INT["integrations/README.md"]
```

## 快速入口

- 架构说明：[architecture.md](architecture.md)
- CLI 使用：[cli.md](cli.md)
- ABI 接口：[abi.md](abi.md)
- 多语言示例：[examples.md](examples.md)
- 支持设备：[devices.md](devices.md)
- 扩展开发：[extending.md](extending.md)
- Windows 分发：[windows_distribution.md](windows_distribution.md)
- 测试指南：[testing.md](testing.md)
- CAN 调试（PCAN + CANable candleLight/gs_usb）：[can_debugging.md](can_debugging.md)
- 平台差异（Windows PCAN 高频重初始化限制）：[pcan_reinit_limit.md](pcan_reinit_limit.md)
- 分发渠道（APT/Homebrew/Winget/Scoop/Choco）：[distribution_channels.md](distribution_channels.md)
- 最终用户操作手册（PCAN 主链路 + Damiao 串口桥备用链路）：[operation_manual.md](operation_manual.md)
- MotorBridge Studio：独立仓库 `motorbridge-studio`（由 `tools/factory_calib_ui_ws` 拆分）
- 集成目录：[`integrations/README.md`](../../integrations/README.md)
- WS 网关：[`integrations/ws_gateway/README.zh-CN.md`](../../integrations/ws_gateway/README.zh-CN.md)

## motorbridge 提供什么

- 与厂商无关的通用核心（`motor_core`）
- 厂商协议插件（`motor_vendors/*`）
- Rust CLI（`motor_cli`）
- 稳定 C ABI（`motor_abi`，供 C/C++/Python 等调用）
- Python SDK 包（`bindings/python`）
- C++ RAII 封装包（`bindings/cpp`）

## 建议阅读顺序

1. [architecture.md](architecture.md)
2. [devices.md](devices.md)
3. [cli.md](cli.md)
4. [abi.md](abi.md)
5. [examples.md](examples.md)
6. [extending.md](extending.md)
7. [windows_distribution.md](windows_distribution.md)
8. [can_debugging.md](can_debugging.md)
9. [pcan_reinit_limit.md](pcan_reinit_limit.md)
10. [distribution_channels.md](distribution_channels.md)
11. [operation_manual.md](operation_manual.md)
12. [testing.md](testing.md)
