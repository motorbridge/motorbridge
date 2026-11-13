# Damiao API 与调参总表（完整版）

<!-- channel-compat-note -->
## 通道兼容说明（PCAN + CANable candleLight/gs_usb + MCU 串口转 CAN 桥 + Damiao 串口桥 + DM_Device）

- Linux SocketCAN 直接使用已初始化的接口名：`can0`、`can1`。CANable 请刷 candleLight/gs_usb 固件，让系统识别为 `can0` 这类 SocketCAN 接口。
- 标准 CAN 推荐 PCAN 或 CANable candleLight/gs_usb。
- 仅 Damiao 可选两类适配器链路：串口桥 `--transport dm-serial --serial-port /dev/ttyACM0 --serial-baud 921600`，以及 DM_Device SDK `--transport dm-device --dm-device-type usb2canfd|usb2canfd-dual|linkx4c --dm-channel 0|1|2|3`。DM_Device 链路当前只配 Damiao 电机协议使用，适配器需处于 USB 模式。
- 通用 MCU 串口转 CAN 桥：`--transport mcu-serial --serial-port /dev/ttyACM0 --serial-baud 921600`。注意与 Damiao 专用串口桥 `dm-serial` 是两种协议：`mcu-serial` 由桥固件透传经典 CAN 帧，`dm-serial` 承载 Damiao 串口协议。适配经典 CAN 全品牌（damiao / robstride / myactuator / hightorque / huayi；hexfellow 仅 CAN-FD 不适用）；桥固件在 bus-off / 发送失败 / 接收丢弃时会主动上报结构化链路状态；四个前端（motor_abi / Python / motor_cli / ws_gateway）均支持。
- Linux SocketCAN 下 `--channel` 不要带 `@bitrate`（例如 `can0@1000000` 无效）。
- Windows（PCAN 后端）中，`can0/can1` 映射 `PCAN_USBBUS1/2`，可选 `@bitrate` 后缀。


本页是 `motorbridge` 当前 Damiao 可调控制/配置参数的实用总表。

> English version: [DAMIAO_API.md](DAMIAO_API.md)

## 1）通用设备参数

| 参数 | 含义 | 常用值 |
|---|---|---|
| `channel` | CAN 接口名 | `can0` |
| `model` | 电机型号字符串（`4310`、`4340P` 等） | 与实物一致 |
| `motor-id` | 命令 ID（`ESC_ID`） | 如 `0x01` |
| `feedback-id` | 反馈 ID（`MST_ID`） | 如 `0x11` |
| `loop` | 发送循环次数 | `100` |
| `dt-ms` | 每次发送间隔（毫秒） | `20` |

## 2）实时控制模式参数

## 2.1 MIT 模式

参数：

- `pos`：目标位置
- `vel`：目标速度
- `kp`：位置刚度增益
- `kd`：速度阻尼增益
- `tau`：前馈力矩

示例（`motor_cli`）：

```bash
motor_cli \
  --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 \
  --mode mit --pos 0 --vel 0 --kp 20 --kd 1 --tau 0 --loop 200 --dt-ms 20
```

## 2.2 POS_VEL 模式

参数：

- `pos`：目标位置
- `vlim`：速度限制

示例：

```bash
motor_cli \
  --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 \
  --mode pos-vel --pos 3.10 --vlim 1.50 --loop 300 --dt-ms 20
```

## 2.3 VEL 模式

参数：

- `vel`：目标速度

示例：

```bash
motor_cli \
  --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 \
  --mode vel --vel 0.5 --loop 100 --dt-ms 20
```

## 2.4 FORCE_POS 模式

参数：

- `pos`：目标位置
- `vlim`：速度限制
- `ratio`：力矩限制比例

示例：

```bash
motor_cli \
  --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 \
  --mode force-pos --pos 0.8 --vlim 2.0 --ratio 0.3 --loop 100 --dt-ms 20
```

## 2.5 模式寄存器（`CTRL_MODE`）

寄存器：

- `rid=10`（`CTRL_MODE`），取值：
  - `1=MIT`
  - `2=POS_VEL`
  - `3=VEL`
  - `4=FORCE_POS`

## 3）强影响寄存器（优先）

这些参数对控制效果影响很大，调参要谨慎。

| RID | 名称 | 类型 | 含义 |
|---|---|---|---|
| `21` | `PMAX` | `f32` | 位置映射范围 |
| `22` | `VMAX` | `f32` | 速度映射范围 |
| `23` | `TMAX` | `f32` | 力矩映射范围 |
| `25` | `KP_ASR` | `f32` | 速度环 Kp |
| `26` | `KI_ASR` | `f32` | 速度环 Ki |
| `27` | `KP_APR` | `f32` | 位置环 Kp |
| `28` | `KI_APR` | `f32` | 位置环 Ki |
| `4` | `ACC` | `f32` | 加速度 |
| `5` | `DEC` | `f32` | 减速度 |
| `6` | `MAX_SPD` | `f32` | 最大速度 |
| `9` | `TIMEOUT` | `u32` | 通信超时寄存器 |

## 4）保护相关寄存器

| RID | 名称 | 类型 | 含义 |
|---|---|---|---|
| `0` | `UV_Value` | `f32` | 欠压阈值 |
| `2` | `OT_Value` | `f32` | 过温阈值 |
| `3` | `OC_Value` | `f32` | 过流阈值 |
| `29` | `OV_Value` | `f32` | 过压阈值 |

## 5）参数读写方法

推荐流程：

1. `get_register` 读取旧值
2. `write_register` 写入新值
3. 再次 `get_register` 回读确认
4. `store_parameters` 持久化

Python SDK API 示例：

```python
from motorbridge import Controller

with Controller("can0") as ctrl:
    m = ctrl.add_damiao_motor(0x01, 0x11, "4340P")
    old = m.get_register_f32(22, 1000)
    print("old VMAX", old)
    m.write_register_f32(22, old * 0.9)
    new = m.get_register_f32(22, 1000)
    print("new VMAX", new)
    m.store_parameters()
    m.close()
```

WS 网关命令示例：

```json
{"op":"get_register_f32","rid":22,"timeout_ms":1000}
{"op":"write_register_f32","rid":22,"value":25.0}
{"op":"store_parameters"}
```

## 6）ID 与标定参数

- `rid=8`（`ESC_ID`）：命令 ID
- `rid=7`（`MST_ID`）：反馈 ID

工具入口：

- Rust CLI：`motor_cli --vendor damiao --mode scan ...` 以及 `--set-motor-id/--set-feedback-id --store --verify-id`
- Python：`motorbridge-cli scan/id-set/id-dump`
- WS：`scan`、`set_id`、`verify`

## 7）安全建议

- 每次只调一组参数。
- 小步进调整，每一步都回读确认。
- 保持机械安全和急停预案。
- 对保护阈值（`0/2/3/29`）不要盲目激进调整。

## 8）夹爪电机校准

Damiao 电机使用单圈编码器（位置范围约 ±PMAX rad），**断电后零点不保持**。当电机用于夹爪时，每次上电需先校准零位：

1. MIT 模式低力矩推向机械限位，`kp` 低、`tau=0`，碰到即软停。
2. 等反馈 `vel ≈ 0` 且 `pos` 稳定后，执行 `--mode set-zero`。
3. 之后闭合用 MIT（力控安全），张开用 pos-vel（快速精确）。

详细命令见 [操作手册](../docs/zh/operation_manual.md) 第 9 章。
