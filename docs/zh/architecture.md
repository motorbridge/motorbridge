# 架构说明

## 分层视图

```mermaid
flowchart TB
  APP["上层应用（Rust/C/C++/Python/ROS2/WS）"] --> SURFACE["CLI / ABI / SDK / Integrations"]
  SURFACE --> CORE["motor_core（controller/bus/model/traits）"]
  CORE --> DAMIAO["motor_vendors/damiao"]
  CORE --> ROBSTRIDE["motor_vendors/robstride"]
  CORE --> MYACT["motor_vendors/myactuator"]
  CORE --> HIGHTORQUE["motor_vendors/hightorque"]
  CORE --> HEXFELLOW["motor_vendors/hexfellow"]
  CORE --> HUAYI["motor_vendors/huayi"]
  CORE --> TEMPLATE["motor_vendors/template（接入模板）"]
  HUAYI --> CANOPEN["motor_canopen（CANopen 公用框架）"]
  CANOPEN --> CAN["CAN 总线后端"]
  DAMIAO --> CAN
  ROBSTRIDE --> CAN
  MYACT --> CAN
  HIGHTORQUE --> CAN
  HEXFELLOW --> CAN
  CAN --> LNX["Linux：SocketCAN"]
  CAN --> WIN["Windows（实验）：PEAK PCAN"]
  CAN --> HW["真实电机硬件"]
```

## 运行时控制流

```mermaid
sequenceDiagram
  participant U as 用户工具
  participant C as CoreController
  participant V as Vendor Motor
  participant B as CAN 后端
  U->>C: 发送控制指令（MIT/POS_VEL/VEL/FORCE_POS）
  C->>V: 按电机句柄路由
  V->>B: 编码并发送帧
  B-->>V: 反馈帧
  V-->>C: 解析后的状态
  C-->>U: 状态缓存/回读结果
```

## 设计目标

`motorbridge` 的核心目标是把通用控制能力与厂商协议细节解耦。

- 核心层可跨品牌复用
- 厂商差异下沉到 vendor crate
- 通过 ABI 暴露稳定跨语言接口

## 仓库结构

```text
motorbridge/
├── motor_core/                  # 与厂商无关的运行时
├── motor_canopen/               # CANopen 阵营公用框架（SDO/CiA402/PDO/NMT/扫描）
├── motor_vendors/
│   ├── damiao/                  # 生产可用实现
│   ├── robstride/               # 生产可用实现（扩展 ID / 参数协议）
│   ├── myactuator/              # 生产可用实现（RMD 协议）
│   ├── hightorque/              # 生产可用实现（ht_can 协议）
│   ├── hexfellow/               # 生产可用实现（CANopen over CAN-FD）
│   ├── huayi/                   # 华翼关节驱动器（CANopen/CiA402，经典 CAN）
│   └── template/                # 新厂商模板
├── motor_cli/                   # Rust CLI
├── motor_abi/                   # C ABI (cdylib + staticlib)
├── integrations/
│   ├── ros2_bridge/             # ROS2 桥接
│   └── ws_gateway/              # Rust WebSocket 网关
├── bindings/
│   ├── python/                 # Python SDK 包 + CLI
│   └── cpp/                    # C++ RAII 封装 + CMake 包
├── examples/                    # C/C++/Python 示例
└── docs/
    ├── en/
    └── zh/

独立仓库：
- `motorbridge-studio/`          # 由 `tools/factory_calib_ui_ws` 拆出的 Web 控制台
```

## 分层说明

### 1) `motor_core`

- `bus.rs`：CAN 总线抽象
- `device.rs`：统一 `MotorDevice` 接口
- `controller.rs`：调度/路由/轮询
- `model.rs`：型号目录抽象
- `socketcan.rs`：Linux 经典 SocketCAN 后端
- `socketcanfd.rs`：Linux 独立 SocketCAN-FD 后端
- `pcan.rs`：Windows PEAK PCAN 后端（实验性）

### 2) `motor_canopen`

CANopen 阵营的公用协议层，收拢"怎么发帧、怎么握手"：

- SDO 读写（快速收发 + 分段）
- CiA402 状态机序列（enable/disable/quick-stop）
- PDO 重配置与 TPDO 流解析
- NMT 主站命令与节点扫描
- COB-ID 规则（`0x600+node` / `0x580+node` 等）

厂商 crate 只保留语义差异：对象字典条目、单位与缩放、`6060h` 模式编号映射。
Huayi 与 `robstride_cia402` 均组合这一层；一条经典 CAN 总线上两个 CANopen 厂商可共存。

### 3) `motor_vendors/*`

每个厂商 crate 负责：

- 帧编码/解码（CANopen 阵营除外，见 `motor_canopen`）
- 寄存器语义
- 型号限制参数
- 基于 `CoreController` 的控制器封装

### 4) `motor_abi`

- 导出 C 兼容句柄与函数
- Rust 错误转换为返回码 + `motor_last_error_message()`
- 供 C/C++/Python 等语言集成
- 同一个 `MotorController` 或 `MotorHandle` 上的调用会在 ABI 内部串行化；
  不同 handle 仍可在不同线程使用
- `motor_controller_free` / `motor_handle_free` 是独占所有权操作：
  不要与同一指针上的其它操作并发调用，释放后也不能继续使用该指针

### 5) SDK 与示例

- `motor_cli`：运维/调试命令行
- `bindings/python`：可直接集成的 Python 包
- `bindings/cpp`：可直接集成的 C++ RAII 封装
- `examples/*`：最小化跨语言 ABI 调用示例

## 生命周期策略

控制器生命周期显式管理：

- 需要明确停止/失能流程时：调用 `motor_controller_shutdown`
- 只想关闭本地会话/总线时：调用 `motor_controller_close_bus`
- `motor_controller_free` 仅负责释放对象
