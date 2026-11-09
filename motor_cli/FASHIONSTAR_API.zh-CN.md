# FashionStar CAN API 与模式参考

> English: [FASHIONSTAR_API.md](FASHIONSTAR_API.md)

本文介绍 motorbridge 的 FashionStar CAN 控制接口。

## 1）基础信息

- 厂商标识：`fashionstar`。型号参数默认 `fashionstar`，当前实现忽略该参数，不区分型号。
- 经典 CAN，收发均使用标准 11-bit 仲裁 ID **0x321**。`motor_id` 是数据区的 ServoID，范围 **0..254**；不接受 255 作为单电机 ID。
- `feedback_id` 固定为 `0x321`（801），与 ServoID 无关。
- 链路：`auto`、`socketcan`、`mcu-serial`。MCU 固件必须使用 motorbridge 已有串口协议，不能直接使用官方网页工具的 UC-06 串口封装。
- 主机与舵机波特率必须一致，官方默认值为 500 kbit/s。Linux 先配置 `can0`；PCAN 可指定 `can0@500000`。

## 2）`motor_cli` 支持模式

| 模式 | 行为 |
|---|---|
| `scan` | 读取型号寄存器 `0x01`，默认扫描 0..254 |
| `ping` | 查询型号码，不运动 |
| `status`、`read` | 查询位置、状态、温度和电流，并根据相邻位置采样估算速度 |
| `enable`、`stop` | 当前位置锁力（`0x7B=0x11`） |
| `disable` | 卸力（`0x7B=0x10`） |
| `zero`、`set-zero` | 先卸力，等待 50 ms，再设当前位置为零点 |
| `pos`、`pos-vel` | 带正速度限制的绝对位置运动，映射原生模式 2 |
| `read-param`、`write-param` | 复用整数寄存器原始值接口 |

连续速度（VEL）、MIT/定量力矩及 FORCE_POS 在当前接入中**暂不支持**，调用返回错误。扭力开关不能替代定量力矩控制。

当前协议没有原生连续速度指令。需要长行程旋转时，可通过 `POS_VEL` 在 ±1024 圈范围内设置较远的绝对目标角度和正速度限制；舵机到达目标后停止。

## 3）参数与单位

| 参数 | 默认值 | 含义 |
|---|---|---|
| `--motor-id` | 1 | ServoID，0..254 |
| `--feedback-id` | `0x321` | 固定仲裁 ID |
| `--pos` | 0 | 绝对目标位置，rad，范围 ±1024 圈 |
| `--vlim` | 1 | 正速度限制，rad/s，取整后为 1..65535 个 0.1°/s |
| `--timeout-ms` | 500 | 等待回包超时；保护区解锁等待另计 |
| `--param-id` | `0x14`（读）、`0`（写） | 寄存器地址 |
| `--param-type` | `u32` | `u16` 或 `u32`，均传原始整数位值 |
| `--param-value` | 写入时必填 | 十进制或 `0x` 十六进制无符号原始值 |
| `--loop`、`--dt-ms` | 1、20 | 命令次数与间隔；重复 POS_VEL 仍然有角度目标 |

## 4）常用命令

```bash
motor_cli --vendor fashionstar --channel can0 --mode scan --start-id 0 --end-id 16
motor_cli --vendor fashionstar --channel can0 --motor-id 0 --mode status
motor_cli --vendor fashionstar --channel can0 --motor-id 0 --mode pos-vel --pos 1.5707963 --vlim 1
motor_cli --vendor fashionstar --channel can0 --motor-id 0 --mode stop
motor_cli --vendor fashionstar --channel can0 --motor-id 0 --mode read-param --param-id 0x81 --param-type u32
motor_cli --vendor fashionstar --channel can0 --motor-id 0 --set-motor-id 2
# 重新上电后，使用新 ServoID 验证：
motor_cli --vendor fashionstar --channel can0 --motor-id 2 --mode ping
```

## 5）寄存器、状态与配置

POS_VEL 依次写入模式 `0x52=2`、有符号目标角度 `0x53/0x54`、速度限制 `0x58`，最后写 `0x59=1` 执行。角度单位为 signed32 的 0.1°，保留设备原有功率和加减速配置。

u32 寄存器接口在低字地址 `0x03`、`0x15`、`0x50`、`0x53` 自动组合低高两个寄存器，其他已支持地址传输 16-bit 原始值。有符号值按对应寄存器位宽的补码传入。发送前检查地址权限和文档明确的范围。PID 参数 `0xC8..0xD5` 自动解锁、访问并重新上锁；解锁稳定等待共 400 ms，之后开始等待回包。

统一状态包含位置、状态码、估算速度（`vel`，rad/s）、电流（`torq`，A）和舵机温度（`t_mos`，℃）。速度是相邻完整位置采样之间的平均角速度；首次采样、查询失败或零点/圈数重置后的首次采样没有速度估算。电流寄存器 `0x11` 按官方 CAN SDK 和网页工具的 mA 单位除以 1000 换算为 A；与 MyActuator 一样，`torq` 表示电流，不表示 N·m 力矩。温度采用官方 CAN 工具默认 NTC 换算（B=3435、T0=298.15 K、ADC 满量程 4096），1..4095 之外的 ADC 值无效。单一舵机温度沿用项目其他单温度厂商的约定映射到 `t_mos`；`t_rotor` 仍无有效值。无效值在 C ABI/Python/C++ 中为 `NaN`，在 WS JSON 中为 `null`；完整位置/状态/温度/电流查询成功前没有有效快照。`stop` 停止锁力，`disable` 卸力。

**配置修改需要重新上电后生效。** 写入成功表示请求已发送。修改 ID 寄存器 `0x81` 后，当前句柄仍使用旧 ID；重新上电后，以新 ID 重新创建控制器和舵机句柄再验证。波特率修改同样需重新上电，并同步主机波特率。运行时目标、执行、停止指令不需要重新上电。当前协议接入没有独立保存命令。

Python SDK/CLI 示例见 [FASHIONSTAR_API.zh-CN.md](../bindings/python/FASHIONSTAR_API.zh-CN.md)。WS 复用已有 `set_target`、`pos_vel`、`state_once`、`get_register_u32`、`write_register_u32`、`scan`、`set_id`、`verify`；`set_id` 返回 `verified:false`、`requires_power_cycle:true`。默认 `param_stream` 读取电压（`0x10`）、电流（`0x11`）、功率（`0x12`）、温度（`0x13`）、状态（`0x14`）、多圈位置（`0x15/0x16`）、圈数（`0x17`）和单圈角度（`0x18`），均返回寄存器原始值。

## 6）协议来源

- [官方 CAN 协议](https://fashionstar.com.cn/wiki/protocols/servo/can-bus/)
- [官方各语言 SDK](https://fashionstar.com.cn/wiki/sdk/servo/)
- [官方 CAN 网页配置工具](https://fashionstar.com.cn/wiki/software/servo/canbus-web-config-tool/)
