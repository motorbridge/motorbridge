# CLI 指南（`motor_cli`）

<!-- channel-compat-note -->
## 通道兼容说明（PCAN + CANable candleLight/gs_usb + Damiao 串口桥 + DM_Device）

- Linux SocketCAN 直接使用已初始化的接口名：`can0`、`can1`。CANable 请刷 candleLight/gs_usb 固件，让系统识别为 `can0` 这类 SocketCAN 接口。
- 标准 CAN 推荐 PCAN 或 CANable candleLight/gs_usb。
- 仅 Damiao 可选独立 CAN-FD 链路：`--transport socketcanfd`（与经典 `socketcan` 并存）。
- 仅 Damiao 可选两类适配器链路：串口桥 `--transport dm-serial --serial-port /dev/ttyACM0 --serial-baud 921600`，以及 DM_Device SDK `--transport dm-device --dm-device-type usb2canfd|usb2canfd-dual|linkx4c --dm-channel 0|1|2|3`。DM_Device 链路当前只配 Damiao 电机协议使用，适配器需处于 USB 模式。
- Linux SocketCAN 下 `--channel` 不要带 `@bitrate`（例如 `can0@1000000` 无效）。
- Windows（PCAN 后端）中，`can0/can1` 映射 `PCAN_USBBUS1/2`，可选 `@bitrate` 后缀。

传输标识：
- `[STD-CAN]` => `--transport auto|socketcan`
- `[CAN-FD]` => `--transport socketcanfd`
- `[DM-SERIAL]` => `--transport dm-serial`
- `[DM-DEVICE]` => `--transport dm-device`

`[CAN-FD]` 说明：目前是“链路已接入”，电机验证矩阵尚未声明完成。

## 调试入口

- PCAN + CANable candleLight/gs_usb 专业排障见：[can_debugging.md](can_debugging.md)。

## 构建

```bash
cargo build -p motor_cli --release
```

## 通用参数

- `--vendor damiao|robstride|hightorque|myactuator|hexfellow|all`
- `--transport auto|socketcan|socketcanfd|dm-serial|dm-device`（`dm-serial`/`dm-device` 仅 Damiao；`socketcanfd` 用于 Hexfellow）
- `--channel can0`
- `--serial-port /dev/ttyACM0 --serial-baud 921600`（配合 `--transport dm-serial`）
- `--dm-device-type usb2canfd|usb2canfd-dual|linkx4c --dm-channel 0|1|2|3`（配合 `--transport dm-device`；扫描模式不传 `--dm-channel` 会扫描所选适配器全部通道）
- `--motor-id <id>`
- `--loop <n> --dt-ms <ms>`

## Damiao 示例

```bash
cargo run -p motor_cli --release -- \
  --vendor damiao --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 \
  --mode mit --pos 0 --vel 0 --kp 20 --kd 1 --tau 0 --loop 50 --dt-ms 20
```
`[STD-CAN]`

```bash
# Damiao 串口桥链路
cargo run -p motor_cli --release -- \
  --vendor damiao --transport dm-serial --serial-port /dev/ttyACM1 --serial-baud 921600 \
  --model 4310 --motor-id 0x04 --feedback-id 0x14 \
  --mode mit --verify-model 0 --ensure-mode 0 \
  --pos 0.5 --vel 0 --kp 20 --kd 1 --tau 0 --loop 80 --dt-ms 20
```
`[DM-SERIAL]`

```bash
# Damiao 独立 CAN-FD 链路
cargo run -p motor_cli --release -- \
  --vendor damiao --transport socketcanfd --channel can0 \
  --model 4310 --motor-id 0x04 --feedback-id 0x14 \
  --mode mit --verify-model 0 --ensure-mode 0 \
  --pos 0.5 --vel 0 --kp 20 --kd 1 --tau 0 --loop 80 --dt-ms 20
```
`[CAN-FD]`

## Hexfellow 示例

```bash
# Hexfellow 扫描（CAN-FD 链路）
cargo run -p motor_cli --release -- \
  --vendor hexfellow --transport socketcanfd --channel can0 \
  --mode scan --start-id 1 --end-id 32
```
`[CAN-FD]`

```bash
# Hexfellow 状态查询
cargo run -p motor_cli --release -- \
  --vendor hexfellow --transport socketcanfd --channel can0 \
  --model hexfellow --motor-id 1 --feedback-id 0 \
  --mode status
```
`[CAN-FD]`

## RobStride 示例

```bash
cargo run -p motor_cli --release -- \
  --vendor robstride --channel can0 --model rs-00 --motor-id 127 --mode ping
```

```bash
cargo run -p motor_cli --release -- \
  --vendor robstride --channel can0 --model rs-00 --motor-id 127 \
  --mode mit --ensure-strict 1 --pos 0.5 --vel 0 --kp 20.0 --kd 0.5 --tau 0 --loop 100 --dt-ms 20
```

```bash
cargo run -p motor_cli --release -- \
  --vendor robstride --channel can0 --model rs-00 --motor-id 127 \
  --mode pos-vel --pos 1.5 --vlim 1.0 --loc-kp 5.0 --loop 1 --dt-ms 20
```

## HighTorque（原生 `ht_can` v2.0.0）

支持模式：

- `scan`
- `read` / `ping`
- `mit`（统一接口）
- `pos` / `vel` / `tqe`（力矩模式 `05 13`，`--tau <Nm>` 物理量，型号力矩系数补偿）
- `pos-vel-tqe`
- `pos-vel-acc`（梯形 `07 2d`，固件 v4.6.0+）：`--pos <rad> --vlim <rad/s> --acc <rad/s^2>`
- `volt`（DQ 电压模式 `01 00 08 05 1B`）：`--volt <V>`（0.1 V/LSB，协议只定义 Q 分量）
- `cur`（DQ 电流模式 `01 00 09 05 1C`）：`--cur <A>`（0.1 A/LSB）
- `set-mode`（运行模式切换 `01 00 <mode>`）：`--run-mode <0-17>`（表2；0=停止并清错，8=DQ 电压，9=DQ 电流，10=位置，15=刹车）
- `read-reg`（通用寄存器读 `17` 族）：`--addr <hex|dec>` 必填、`--count`（1-3，默认 1）；2026-10-22 起读取**固定按 Int16**（`--ty` 已删除，显式传入会报错），返回**换算后物理量**（位置/速度/加速度→rad 族、力矩→N·m 套型号 k、电流/电压/温度/PWM/PID 按各自 scale；无换算关系的寄存器 1:1），每行打印该行地址的寄存器名与单位；地址须在寄存器表（0x00-0x45 中 xlsx 列出的 43 个）内，缺号/表外地址读/写均报错
- `write-reg`（通用寄存器写 `05 <addr> <val16>`）：`--addr` 必填（同样须在寄存器表内，缺号/表外拒绝）、`--value <int16>`、`--config`（0/1，1=配置类走扩展帧，缺省 0 走控制通道）
- `periodic-report`（周期上报 `05 B4 02 <t_ms>`）：`--period-ms`（≥1 开启电机主动状态流，0 停止；`timed-read` 为旧名别名）
- `stop` / `brake`（模式 15，三相短接阻尼刹车）/ `rezero` / `conf-write`

统一单位接口（与其他电机保持一致）：

- `--pos`：弧度（rad）
- `--vel`：弧度每秒（rad/s）
- `--tau`：扭矩（Nm）
- `--kp`、`--kd`：打包进 v2.0.0 MIT 位打包帧（固件 v4.6.0+；量程 kp ±400、kd ±100，超界饱和）

底层原始接口（调试，仅位置/速度）：

- `--raw-pos`、`--raw-vel`（力矩/电压/电流已统一为物理量 `--tau`/`--volt`/`--cur`，由 vendor 侧编码）

示例：

```bash
# 扫描 ID
cargo run -p motor_cli --release -- \
  --vendor hightorque --channel can0 --mode scan --start-id 1 --end-id 32
```

```bash
# 读状态（输出包含 pos_rad / vel_rad_s）
cargo run -p motor_cli --release -- \
  --vendor hightorque --channel can0 --motor-id 1 --mode read
```

```bash
# 转到 +180 度（pi 弧度），并限制速度/力矩
cargo run -p motor_cli --release -- \
  --vendor hightorque --channel can0 --motor-id 1 \
  --mode mit --pos 3.1415926 --vel 0.8 --tau 0.8
```

```bash
# 停止
cargo run -p motor_cli --release -- \
  --vendor hightorque --channel can0 --motor-id 1 --mode stop
```

```bash
# 梯形运动到 +90 度，限速 1 rad/s、加速度 2 rad/s^2
cargo run -p motor_cli --release -- \
  --vendor hightorque --channel can0 --motor-id 1 \
  --mode pos-vel-acc --pos 1.5708 --vlim 1.0 --acc 2.0
```

```bash
# 读 3 个连续寄存器（位置/速度/转矩，固定 Int16，返回换算后物理量 rad/rad·s⁻¹/N·m）
cargo run -p motor_cli --release -- \
  --vendor hightorque --channel can0 --motor-id 1 \
  --mode read-reg --addr 0x01 --count 3
```

## MyActuator 示例

```bash
cargo run -p motor_cli --release -- \
  --vendor myactuator --channel can0 --model X8 --motor-id 1 --feedback-id 0x241 \
  --mode status --loop 20 --dt-ms 50
```

```bash
# 将当前位置设为零点（持久生效需断电重启）
cargo run -p motor_cli --release -- \
  --vendor myactuator --channel can0 --model X8 --motor-id 1 --feedback-id 0x241 \
  --mode set-zero --loop 1
```

## 全品牌扫描

```bash
cargo run -p motor_cli --release -- \
  --vendor all --channel can0 --mode scan --start-id 1 --end-id 255
```

RobStride 单独扫描：

```bash
cargo run -p motor_cli --release -- \
  scan --vendor robstride --channel can0 --start-id 1 --end-id 127 \
  --feedback-ids 0xFD,0xFF,0xFE,0x00,0xAA
```

RobStride 输出中，`probe` / `device_id` 是电机 ID；`feedback_id` / `host_id`（如 `0xFD`）是上位机侧 ID，不是电机 ID。
RobStride `motor_id` / `device_id` 会校验为 `1..255`；`feedback_id` / `host_id` 会校验为 `0..255`。
