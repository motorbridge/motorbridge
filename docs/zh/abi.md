# ABI 指南（`motor_abi`）

## 构建

```bash
cargo build -p motor_abi --release
```

产物：

- Linux：`target/release/libmotor_abi.so`、`libmotor_abi.a`
- Windows：`target/release/motor_abi.dll`、`motor_abi.lib`
- 头文件：`motor_abi/include/motor_abi.h`

## 统一接口面

ABI 对外保持一套统一控制接口。

## ABI 元数据

- `motor_abi_version()` 返回当前加载的 ABI 库版本字符串。
- `motor_abi_capabilities_json()` 返回稳定 JSON 能力文档，描述已支持的
  transports、vendors、controller lifecycle、control modes 和厂商专用 features。
- Python 中对应 `motorbridge.abi_version()` 和 `motorbridge.abi_capabilities()`。
- C++ 中对应 `motorbridge::abi_version()` 和 `motorbridge::abi_capabilities_json()`。
- `bindings/api_surface.json` 用于跟踪 ABI、Python、C++ 和文档的接口对齐。

统一模式 ID（`motor_handle_ensure_mode`）：

- `1 = MIT`
- `2 = POS_VEL`
- `3 = VEL`
- `4 = FORCE_POS`

统一控制单位：

- 位置：`rad`
- 速度：`rad/s`
- 力矩：`Nm`

通用控制/状态接口：

- `motor_handle_enable`
- `motor_handle_disable`
- `motor_handle_clear_error`
- `motor_handle_set_zero_position`
- `motor_handle_ensure_mode`
- `motor_handle_send_mit`
- `motor_handle_send_pos_vel`
- `motor_handle_send_vel`
- `motor_handle_send_force_pos`
- `motor_handle_request_feedback`
- `motor_handle_set_can_timeout_ms`
- `motor_handle_store_parameters`
- `motor_handle_get_state`

## 厂商入口

- Damiao：`motor_controller_add_damiao_motor(...)`
- Hexfellow：`motor_controller_add_hexfellow_motor(...)`（通过 `socketcanfd` 走 CAN-FD）
- RobStride：`motor_controller_add_robstride_motor(...)`
- MyActuator：`motor_controller_add_myactuator_motor(...)`
- HighTorque：`motor_controller_add_hightorque_motor(...)`
- Huayi：`motor_controller_add_huayi_motor(controller, motor_id, feedback_id, model)`——经典 CAN 上的 CANopen/CiA402；`motor_id` 即节点号（1..=127），`feedback_id=0` 沿用节点号；`model` 仅为标签，限值运行时自读（`6076h` 峰值力矩、`2004h` 压缩 MIT 量程）。

## 统一模式与厂商原生协议映射

| 统一模式 | Damiao 原生 | Hexfellow 原生 | RobStride 原生 | MyActuator 原生 | HighTorque 原生 | Huayi 原生 |
|---|---|---|---|---|---|---|
| `MIT` | `Mit` | 模式 `5` | `Mit` | 不支持 | 映射到原生 pos+vel+tqe | **`6060h = 5`（MIT）** |
| `POS_VEL` | `PosVel` | 模式 `1` | 映射到 `Position`（`run_mode=1`，`limit_spd=0x7017`，`loc_ref=0x7016`） | `Position` 设定流程 | 映射到原生 pos+vel+tqe | `6060h = 1`（PP：`6081h` + `607Ah` + CW bit4 握手） |
| `VEL` | `Vel` | 不支持 | `Velocity` | `Velocity` 设定流程 | 映射到原生速度命令 | `6060h = 3`（PV：`60FFh`） |
| `FORCE_POS` | `ForcePos` | 不支持 | 不支持 | 不支持 | 映射到原生 pos+vel+tqe | 不支持（无 6060h 对应模式） |

> **模式编号陷阱（Huayi 5=MIT）**：Huayi 的 `6060h` 取值采用 CiA402 标准编号——`5` 是 MIT。RobStride **私有协议**里 `5` 是 CSP，两者勿混淆。对外统一编号（1/2/3/4）由 ABI 层映射，用户代码不接触 `6060h` 原始值。

行为约定：

- 不支持的调用返回非 0，并可通过 `motor_last_error_message()` 获取清晰错误信息。
- 即使某厂商忽略部分参数，也保持统一函数签名不变。
- 例如：HighTorque 支持 `send_mit(pos, vel, kp, kd, tau)` 统一签名，但原生协议不使用 `kp/kd`。
- Hexfellow 的 ABI 路径支持 `MIT` 和 `POS_VEL`，`VEL` / `FORCE_POS` 会返回不支持。
- RobStride 的 ABI 路径支持 `POS_VEL`，语义映射为原生 Position：先设置 `run_mode=1`，再写 `limit_spd` 与 `loc_ref`。
- RobStride 的统一高层目前支持 `MIT/POS_VEL/VEL`；`TORQUE/CURRENT` 仍是参数级能力（通过 `robstride_write_param_*`），尚未开放统一模式。
- Damiao 置零顺序规则：先调用 `motor_handle_disable`，再调用 `motor_handle_set_zero_position`；否则会被核心防护拒绝。
- Damiao 置零稳定规则：`set_zero_position` 成功后，核心层内置固定稳定等待（约 `20ms`），ABI 不额外暴露等待参数。
- Huayi 置零规则与 Damiao 同构：3001h 位置预设要求 Switch On Disabled，须先 `motor_handle_disable` 再 `motor_handle_set_zero_position`；`motor_handle_store_parameters`（1010h "save"）落盘。
- Huayi `motor_handle_get_state` 映射：`status_code` = `6061h` 模式显示，`fault_code` = `603Fh`/EMCY 低字节（完整 u16 走 `motor_handle_huayi_get_error_code`）；状态来自 TPDO 流（TPDO2 锚定 seq，TPDO1/TPDO3 就地补丁）。
- Huayi `motor_handle_request_feedback` 语义：TPDO 流已激活时为 no-op（后台接收线程持续入缓存）；未激活时自动激活三组默认 TPDO（10ms 事件计时器）。`motor_handle_set_can_timeout_ms` 对 Huayi 拒绝——手册未提供 `6099h` 等价看门狗对象。

## 厂商扩展接口

Damiao 寄存器接口：

- `motor_handle_write_register_f32`
- `motor_handle_write_register_u32`
- `motor_handle_get_register_f32`
- `motor_handle_get_register_u32`

RobStride 扩展接口：

- `motor_handle_robstride_ping`
- `motor_handle_robstride_ping_host_id`
- `motor_handle_robstride_set_device_id`
- `motor_handle_robstride_set_active_report`
- `motor_handle_robstride_get_fault_report`
- `motor_handle_robstride_get_param_f32_host_id`
- `motor_handle_robstride_write_param_i8/u8/u16/u32/f32`
- `motor_handle_robstride_get_param_i8/u8/u16/u32/f32`

Huayi 扩展接口（类型化 SDO + 私有快速通道；u8 rid 的通用寄存器接口无法寻址 16 位 CANopen 索引，对 Huayi 电机一律拒绝并指向本组接口）：

- `motor_handle_huayi_sdo_read_u16/i16/i32/f32(index, sub, timeout_ms, out)`
- `motor_handle_huayi_sdo_write_u16/i16/i32/f32(index, sub, value)`
- `motor_handle_huayi_send_torque(torque_nm)`——力矩模式（`6060h=4`，`6071h` 峰值千分比）
- `motor_handle_huayi_send_mit_compressed(pos, vel, kp, kd, tau)`——压缩 MIT 实时通道（RPDO4，每命令一帧 8 字节到 `0x500+node`；慢速路径是统一 `motor_handle_send_mit` 走 `2003h`）
- `motor_handle_huayi_position_preset(position_rev)`——3001h 位置预设（须先 disable，±0.5 Rev 输出轴）
- `motor_handle_huayi_configure_periodic_report(period_ms)`——TPDO 流开/停（0 停止；扮演 RobStride `set_active_report` 的角色）
- `motor_handle_huayi_get_error_code(out)`——缓存错误码（u16，无总线流量；EMCY 码在 TPDO2 锚点到达前即可见）
- `motor_handle_huayi_get_error_history(out_codes, out_count)`——1003h 错误历史遍历
- `motor_handle_huayi_set_node_id(new_id)`——2001h 节点号变更（store + 断电后生效）
- `motor_handle_huayi_nmt(command, node)`——NMT 主站命令（0x01/0x02/0x80/0x81/0x82；node 0 广播）
- `motor_controller_huayi_scan_ids(controller, start, end, timeout_ms, out_nodes, out_count)`——节点扫描（NMT start 广播 + 6041h/6061h/603Fh 探测）；直连探测会与核心接收线程竞争，控制器上已有注册电机时返回错误——先扫描再 `add_huayi_motor`，或用同一总线上另一个控制器

## 典型调用顺序

1. 选择传输层构造器：
   - `motor_controller_new_socketcan(channel)`（通用路径）
   - `motor_controller_new_socketcanfd(channel)`（CAN-FD 路径；Hexfellow 必须使用）
   - `motor_controller_new_dm_serial(serial_port, baud)`（仅 Damiao 串口桥；跨平台，可用 `/dev/ttyACM0` 或 `COM3`）
2. `motor_controller_add_<vendor>_motor`
3. 可选：`motor_controller_enable_all`
4. 可选：`motor_handle_ensure_mode`
5. 发送控制命令 / 读取状态 / 调用厂商扩展接口
6. `motor_controller_shutdown`
   - 或在只关闭本地会话/总线时调用 `motor_controller_close_bus`
7. `motor_handle_free`
8. `motor_controller_free`

## 示例

- C ABI 示例：`examples/c/c_abi_demo.c`
- C++ ABI 示例：`examples/cpp/cpp_abi_demo.cpp`
- Python ctypes 示例：`examples/python/python_ctypes_demo.py`
