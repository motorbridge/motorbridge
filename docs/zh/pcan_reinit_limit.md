# 平台差异：PCAN 高频重初始化限制（Windows vs Linux）

记录一个由后端架构差异导致的平台特定行为：**同一段"对每个 (motor_id, feedback_id) 对重开 CAN 总线"的扫描逻辑，在 Linux 上正常、在 Windows 上会扫不到电机**。本文说明根因、代码位置与规避方式，供后续维护者避免重蹈。

## 现象

RobStride 扫描默认配置为 `startId=0x01, endId=0x10, feedbackId="0xFD,0xFF,0xFE"`（见 `motorbridge-studio/src/lib/constants.js`）。当**电机 ID 段长度 N 与 feedback_id 个数 M 都大于 1** 时：

- **Linux**：可正常扫到电机。
- **Windows**：扫不到电机（后期 `CAN_Initialize` 失败，对应 ID 全部 no hit）。
- 把 N 或 M 任一改为 1，Windows 立刻恢复正常。

## 根因：socket 开关 ≠ 控制器重置

两个平台用了不同的 CAN 后端：

| 平台 | 后端 | "开一次总线"实际做了什么 |
|---|---|---|
| Linux | SocketCAN（`PF_CAN` + `SOCK_RAW`） | 内核 socket 分配，**不碰 CAN 控制器**，`can0` 接口保持 UP |
| Windows | PCAN-Basic API | `CAN_Initialize` **重新编程控制器**（位时序/滤波），`CAN_Uninitialize` 停控制器 |

代码位置：

- 后端选择：`motor_core/src/bus`（`open_can_bus` 在 Linux 走 SocketCAN、Windows 走 `PcanBus`）。
- Windows PCAN：`motor_core/src/pcan.rs`（`PcanBus::open` → `CAN_Initialize`，`shutdown` → `CAN_Uninitialize`，整个模块 `#[cfg(target_os="windows")]`）。
- 网关扫描：`integrations/ws_gateway/src/commands/scan.rs` 的 `cmd_scan_robstride_with_progress` 按 `feedback_id` 循环，每个 `feedback_id` 调一次 `open_robstride_controller`。

### Linux 为何没事

SocketCAN 的 socket 生命周期与控制器硬件状态**解耦**：反复 `socket()`/`close()` 只是分配/释放内核 socket 结构，控制器从不重置、接口一直在线。所以 N×M=48 次"重开"在 Linux 上就是 48 次廉价 socket 操作，不会失败。即便插的是 PCAN-USB 硬件，Linux 下 PEAK 驱动的 netdev 模式也把它注册成标准 `can0` 接口、走同一条 SocketCAN 路径。

### Windows 为何失败

PEAK 驱动**释放通道不够快**：`CAN_Uninitialize` 之后立刻 `CAN_Initialize`，通道还没真正释放，下一次返回 `PCAN_ERROR_INITIALIZE`（通道已占用/已初始化）。这是 PEAK 官方论坛反复确认的已知问题（[t=3053](https://forum.peak-system.com/viewtopic.php?t=3053)、[t=2893](https://forum.peak-system.com/viewtopic.php?t=2893)、[t=7416](https://forum.peak-system.com/viewtopic.php?t=7416)、[t=3809](https://forum.peak-system.com/viewtopic.php?t=3809)、[t=2544](https://forum.peak-system.com/viewtopic.php?t=2544)）。官方 [`CAN_Initialize` 文档](https://documentation.help/PCAN-Basic/CAN_Initialize.html) 也注明"硬件状态不允许时会失败"。社区通用 workaround 是 `Uninitialize` 与 `Initialize` 之间加 1~2 秒延迟，或**直接避免反复重初始化**。

在本项目里，`PcanBus::open`（`motor_core/src/pcan.rs:254-270`）在 `CAN_Initialize` 返回非 OK 时直接 `Err`，再被 `scan.rs:130` 的 `?` 冒上去，整条 scan 命令返回错误 → 该 ID 记失败、no hit。48 次突发重开后，后续 `open_robstride_controller` 纷纷失败 → 扫不到。

## 触发条件总结

该症状只在 **N>1 且 M>1** 时出现：

| N（电机ID数） | M（feedback_id 数） | 总线重开次数 | Windows |
|---|---|---|---|
| 16 | 3 | 48 | ❌ 后期 `CAN_Initialize` 失败 |
| 16 | 1 | 16 | ✅ |
| 1 | 3 | 3 | ✅ |
| 1 | 1 | 1 | ✅ |

任一维度降为 1，重开次数就落到 PCAN 能容忍的范围内。

## 规避方式

**原则：不要在 Windows 上反复 `CAN_Initialize`/`CAN_Uninitialize`。**

### 已采用的修复（motorbridge-studio 前端）

RobStride 扫描从"逐 ID 发 N 条 ws 命令"改为**整段范围一条命令**，并带 `scan_all_feedback_ids: true`：

- 之前：前端 `for (probe = startId..endId) sendCmd('scan', {start_id:probe, end_id:probe})`，网关每条命令内为每个 feedback_id 开一次总线 → N×M 次重开。
- 之后：前端发一条 `{start_id, end_id, scan_all_feedback_ids:true}`，网关复用同一个 controller 跨整段 ID → 仅 M 次重开（默认 48 → 3）。
- 增量卡片更新不丢：网关流式吐 `scan_progress` 的 `phase:"hit"` 事件，前端通过 `onProgress` 即时 `applyHits`。

代码：`motorbridge-studio/src/lib/motorScanOps.js`（robstride 分支）、`motorbridge-studio/src/wsGatewayClient.js`（进度事件路由）、`motorbridge-studio/src/hooks/useGatewayBridge.js`（`sendCmd` 透传 `onProgress`）。**仅前端改动，未动 Rust。**

### 写新扫描/控制代码时的注意

1. **复用 controller**：在网关侧，一个 `feedback_id` 一个 controller，跨整个 ID 段复用，不要每个 ID 重开。`cmd_scan_robstride_with_progress` 已是此结构。
2. **不要逐 ID 拆包**：前端不要为了"增量更新"把整段范围拆成 N 条命令——网关的复用优化会被抵消。增量改用流式进度事件。
3. **跨命令也需间隔**：即便单条命令内已复用，若不得不连续多次 `open`/`close`，`scan.rs:232-233` 的 `#[cfg(target_os="windows")] sleep(20ms)` 是最低限度的 settle，跨命令目前没有间隔——能复用就尽量复用，别堆命令。
4. **DM 链路不受影响**：Damiao 扫描在网关侧本就只开一次 controller（`damiao_ws.rs:84`），前端也是整段范围一条命令，不存在此问题。但 DM 有自己的、同源不同的隐患（`add_device` 对同一 motor_id 去重导致每 ID 只试第一组 `(feedback_id, model)` 候选），属 Rust 侧、平时被"出厂值=第一候选"掩盖，不在本修复范围。

## 参考

- [SocketCAN 内核文档](https://docs.kernel.org/networking/can.html)
- [`CAN_Initialize` | PCAN-Basic 文档](https://documentation.help/PCAN-Basic/CAN_Initialize.html)
- PEAK 论坛：[Need to reboot to initialize PCAN-USB](https://forum.peak-system.com/viewtopic.php?t=3053)、[Reset initialized state by software](https://forum.peak-system.com/viewtopic.php?t=2893)、[CAN_Initialize sometimes fails](https://forum.peak-system.com/viewtopic.php?t=7416)、[PCAN_ERROR_INITIALIZE always](https://forum.peak-system.com/viewtopic.php?t=3809)、[CAN_initialize failing after Stop Debugging](https://forum.peak-system.com/viewtopic.php?t=2544)
- [PCAN-Linux 驱动 netdev vs chardev 模式](https://forum.peak-system.com/viewtopic.php?t=2083)
- [Linux CAN 驱动对比（Sojka & Píša）](https://rtime.ciirc.cvut.cz/~hanzalek/publications/Hanzalek10_168894.pdf)
