# RobStride 三通道对照手册（Core CLI / Python CLI / Python SDK）

本文只聚焦 RobStride，并按“同一功能、三种入口”组织：
- Core CLI：`motor_cli`（Rust，基准实现）
- Python CLI：`motorbridge-cli` 或 `python -m motorbridge.cli`
- Python SDK：`from motorbridge import Controller, Mode`

目标：把**相同语义合并写**，只把差异点单独标注，便于快速查证。

## 通道说明（仅 SocketCAN）

- 本文只讨论 SocketCAN（`can0`、`can1`）。
- Linux 下 `--channel` 不要写 `@bitrate`（例如 `can0@1000000` 无效）。
- 排障参考：`../../docs/zh/can_debugging.md`。

## 0）前置

### 0.1 环境

```bash
cd motorbridge
cargo build -p motor_cli --release
CLI=./target/release/motor_cli
export LD_LIBRARY_PATH=$PWD/target/release:${LD_LIBRARY_PATH}
```

### 0.2 通用参数（示例）

```bash
CH=can0
MODEL=rs-06
MID=127
FID=0xFD
```

### 0.3 默认值与原厂协议关系（总览）

- 统一默认反馈 ID：`0xFD`（运行时可回退尝试 `0xFF/0xFE`）。
- 统一模式与原厂语义映射：
  - `mit` -> 原厂 MIT/阻抗控制帧（`pos/vel/kp/kd/tau` 全有效）。
  - `pos-vel` -> 原厂位置流程（PP）：`run_mode=1`，`vel_max(0x7024)`，`loc_ref(0x7016)`。
  - `vel` -> 原厂速度流程：`run_mode=2`，`spd_ref(0x700A)`。
  - `zero/set-zero` -> 原厂置零命令序列（需 `--zero-exp 1` 才真正下发）。
- 常用原厂参数：
  - `0x7005` `run_mode`
  - `0x7016` `loc_ref`
  - `0x7017` `limit_spd`
  - `0x700A` `spd_ref`
  - `0x7019` `mechPos`
  - `0x701B` `mechVel`

### 0.4 入口差异（你关心的“不同点”）

- 控制语义本身：Core CLI / Python CLI / Python SDK 一致。
- 主要差异只在入口：
  - CLI：命令行参数入口（`--mode ...`）。
  - SDK：先 `Controller(\"can0\")`，再 `add_robstride_motor(motor_id, feedback_id, model)`。
- 例外覆盖面：
- Python CLI 已支持 RobStride `id-set`，但只修改 `device_id`；
- `feedback_id` / `host_id` 是上位机侧 ID，不是电机 ID。
- ID 范围会显式校验：`device_id/motor_id/new_motor_id` 为 `1..255`，`feedback_id/host_id` 为 `0..255`，避免底层 `u8`/`ctypes` 静默截断。

## 1）扫描（scan）

共同语义：
- 在 ID 范围内探测在线设备。

参数有效性：
- 有效：`start-id/end-id/feedback-ids/param-timeout-ms`。

默认值：
- 默认完整回退列表：`feedback-ids=0xFD,0xFF,0xFE,0x00,0xAA`。
- 扫描输出中的 `probe` / `device_id` 是电机 ID；`feedback_id` / `host_id` 不是电机 ID。

原厂协议对应：
- 先走 ping 探测；必要时走参数读取探测。

### Core CLI

```bash
$CLI --vendor robstride --channel "$CH" --model "$MODEL" --mode scan --start-id 120 --end-id 130
```

### Python CLI

```bash
motorbridge-cli scan \
  --vendor robstride --channel "$CH" --model "$MODEL" \
  --start-id 120 --end-id 130 --feedback-ids 0xFD --param-timeout-ms 60
```

### Python SDK

```python
from motorbridge import Controller

found = []
with Controller("can0") as ctrl:
    for mid in range(120, 131):
        try:
            m = ctrl.add_robstride_motor(mid, 0xFD, "rs-06")
            try:
                print(mid, m.robstride_ping())
                found.append(mid)
            finally:
                m.close()
        except Exception:
            pass
print("found:", found)
```

## 2）连通性（ping）

共同语义：
- 验证某个 `motor_id + feedback_id` 是否可通信。

参数有效性：
- 有效：`motor-id/feedback-id`。

默认值：
- `feedback-id` 建议 `0xFD`。

原厂协议对应：
- 原厂 ping 帧。

### Core CLI

```bash
$CLI --vendor robstride --channel "$CH" --model "$MODEL" --motor-id "$MID" --feedback-id "$FID" --mode ping
```

### Python CLI

```bash
motorbridge-cli run \
  --vendor robstride --channel "$CH" --model "$MODEL" --motor-id "$MID" --feedback-id "$FID" \
  --mode ping
```

### Python SDK

```python
from motorbridge import Controller

with Controller("can0") as ctrl:
    m = ctrl.add_robstride_motor(127, 0xFD, "rs-06")
    print(m.robstride_ping())
    m.close()
```

## 3）使能（enable）/ 4）失能（disable）

共同语义：
- 开启或关闭驱动输出。

参数有效性：
- 有效：`mode=enable|disable`。

默认值：
- Python CLI 推荐 `loop=1`。

原厂协议对应：
- 原厂使能/失能控制命令。

### Core CLI

```bash
$CLI --vendor robstride --channel "$CH" --model "$MODEL" --motor-id "$MID" --feedback-id "$FID" --mode enable
$CLI --vendor robstride --channel "$CH" --model "$MODEL" --motor-id "$MID" --feedback-id "$FID" --mode disable
```

### Python CLI

```bash
motorbridge-cli run --vendor robstride --channel "$CH" --model "$MODEL" --motor-id "$MID" --feedback-id "$FID" --mode enable --loop 1 --dt-ms 20
motorbridge-cli run --vendor robstride --channel "$CH" --model "$MODEL" --motor-id "$MID" --feedback-id "$FID" --mode disable --loop 1 --dt-ms 20
```

### Python SDK

```python
from motorbridge import Controller

with Controller("can0") as ctrl:
    m = ctrl.add_robstride_motor(127, 0xFD, "rs-06")
    m.enable()
    m.disable()
    m.close()
```

## 5）MIT（统一阻抗）

共同语义：
- 五参数闭环控制：`pos/vel/kp/kd/tau`。

参数有效性：
- 有效：`pos`、`vel`、`kp`、`kd`、`tau`、`loop`、`dt-ms`。
- 无效：无。

默认值：
- 经验建议：`kp=3` 比 `kp=0.5` 更容易看见位置收敛效果。

原厂协议对应：
- 直接映射到 RobStride 原厂 MIT 控制帧。

### Core CLI

```bash
$CLI --vendor robstride --channel "$CH" --model "$MODEL" --motor-id "$MID" --feedback-id "$FID" \
  --mode mit --ensure-strict 1 --pos 0.5 --vel 0 --kp 20.0 --kd 0.5 --tau 0 --loop 100 --dt-ms 20
```

### Python CLI

```bash
motorbridge-cli run \
  --vendor robstride --channel "$CH" --model "$MODEL" --motor-id "$MID" --feedback-id "$FID" \
  --mode mit --ensure-strict 1 --pos 0.5 --vel 0 --kp 20.0 --kd 0.5 --tau 0 --loop 100 --dt-ms 20
```

### Python SDK

```python
from motorbridge import Controller, Mode

with Controller("can0") as ctrl:
    m = ctrl.add_robstride_motor(127, 0xFD, "rs-06")
    m.enable()
    m.ensure_mode(Mode.MIT, 1000)
    for _ in range(40):
        m.send_mit(0.5, 0.0, 20.0, 0.5, 0.0)
    m.close()
```

## 6）POS_VEL（统一位置速度）

共同语义：
- 发“目标位置 + 限速”命令。

参数有效性：
- 有效：`pos`、`vlim`（可选 `kp/loc_kp`）。
- 无效：`vel`、`kd`、`tau`（会被忽略）。

默认值：
- 建议先 `loop=1` 做到位命令，再观察反馈。

原厂协议对应：
- `run_mode=1`，`vel_max(0x7024)`，`loc_ref(0x7016)`。

### Core CLI

```bash
$CLI --vendor robstride --channel "$CH" --model "$MODEL" --motor-id "$MID" --feedback-id "$FID" \
  --mode pos-vel --pos 1.5 --vlim 1.0 --loc-kp 5.0 --loop 1 --dt-ms 20
```

### Python CLI

```bash
motorbridge-cli run \
  --vendor robstride --channel "$CH" --model "$MODEL" --motor-id "$MID" --feedback-id "$FID" \
  --mode pos-vel --pos 1.5 --vlim 1.0 --loc-kp 5.0 \
  --ensure-strict 1 --ensure-timeout-ms 1500 \
  --loop 1 --dt-ms 20
```

### Python SDK

```python
from motorbridge import Controller, Mode

with Controller("can0") as ctrl:
    m = ctrl.add_robstride_motor(127, 0xFD, "rs-06")
    m.enable()
    m.ensure_mode(Mode.POS_VEL, 1500)
    m.send_pos_vel(1.0, 0.8)
    m.close()
```

## 7）VEL（统一速度）

共同语义：
- 速度命令闭环。

参数有效性：
- 有效：`vel`、`loop`、`dt-ms`。
- 无效：`pos`、`kp`、`kd`、`tau`。

默认值：
- 示例：`vel=0.3`。

原厂协议对应：
- `run_mode=2`，`spd_ref(0x700A)`。

### Core CLI

```bash
$CLI --vendor robstride --channel "$CH" --model "$MODEL" --motor-id "$MID" --feedback-id "$FID" \
  --mode vel --vel 0.3 --loop 40 --dt-ms 50
```

### Python CLI

```bash
motorbridge-cli run \
  --vendor robstride --channel "$CH" --model "$MODEL" --motor-id "$MID" --feedback-id "$FID" \
  --mode vel --vel 0.3 --loop 40 --dt-ms 50
```

### Python SDK

```python
from motorbridge import Controller, Mode

with Controller("can0") as ctrl:
    m = ctrl.add_robstride_motor(127, 0xFD, "rs-06")
    m.enable()
    m.ensure_mode(Mode.VEL, 1000)
    for _ in range(40):
        m.send_vel(0.3)
    m.close()
```

## 8）读参数（read-param）

共同语义：
- 读取原厂参数寄存器。

参数有效性：
- 有效：`param-id`、`type`、`timeout-ms`。

默认值：
- 位置常读：`0x7019 (mechPos)`，类型 `f32`。

原厂协议对应：
- 原厂 29-bit 扩展帧参数读取通道。

### Core CLI

```bash
$CLI --vendor robstride --channel "$CH" --model "$MODEL" --motor-id "$MID" --feedback-id "$FID" \
  --mode read-param --param-id 0x7019
```

### Python CLI

```bash
motorbridge-cli run \
  --vendor robstride --channel "$CH" --model "$MODEL" --motor-id "$MID" --feedback-id "$FID" \
  --mode read-param --param-id 0x7019 --param-type f32 --timeout-ms 200

motorbridge-cli robstride-read-param \
  --channel "$CH" --model "$MODEL" --motor-id "$MID" --feedback-id "$FID" \
  --param-id 0x7019 --type f32 --timeout-ms 200
```

### Python SDK

```python
from motorbridge import Controller

with Controller("can0") as ctrl:
    m = ctrl.add_robstride_motor(127, 0xFD, "rs-06")
    print(m.robstride_get_param_f32(0x7019, 200))
    m.close()
```

## 9）写参数（write-param）

共同语义：
- 写入原厂参数，可回读验证。

参数有效性：
- 有效：`param-id`、`value`、`type`、`verify`。

默认值：
- 示例：`0x700A` 写 `f32=0.3`。

原厂协议对应：
- 原厂 29-bit 扩展帧参数写入通道。

### Core CLI

```bash
$CLI --vendor robstride --channel "$CH" --model "$MODEL" --motor-id "$MID" --feedback-id "$FID" \
  --mode write-param --param-id 0x700A --param-value 0.3
```

### Python CLI

```bash
motorbridge-cli run \
  --vendor robstride --channel "$CH" --model "$MODEL" --motor-id "$MID" --feedback-id "$FID" \
  --mode write-param --param-id 0x700A --param-type f32 --param-value 0.3 --timeout-ms 200

motorbridge-cli robstride-write-param \
  --channel "$CH" --model "$MODEL" --motor-id "$MID" --feedback-id "$FID" \
  --param-id 0x700A --type f32 --value 0.3 --verify 1 --timeout-ms 200
```

### Python SDK

```python
from motorbridge import Controller

with Controller("can0") as ctrl:
    m = ctrl.add_robstride_motor(127, 0xFD, "rs-06")
    m.robstride_write_param_f32(0x700A, 0.3)
    print(m.robstride_get_param_f32(0x700A, 200))
    m.close()
```

## 10）改 ID（set-motor-id）

共同语义：
- 修改设备 ID，并建议持久化。

参数有效性：
- Core CLI：`--set-motor-id`、`--store`。
- Python CLI：`id-set --vendor robstride --new-motor-id ... --store ... --verify ...`。
- Python SDK：`robstride_set_device_id()` + `store_parameters()`。

默认值：
- 示例改到 `126`。

原厂协议对应：
- 原厂 set-id 命令流程。

### Core CLI

```bash
$CLI --vendor robstride --channel "$CH" --model "$MODEL" --motor-id "$MID" --feedback-id "$FID" \
  --set-motor-id 126 --store 1
```

### Python CLI

```bash
motorbridge-cli id-set \
  --vendor robstride --channel "$CH" --model "$MODEL" \
  --motor-id "$MID" --feedback-id "$FID" \
  --new-motor-id 126 --store 1 --verify 1
```

### Python SDK

```python
from motorbridge import Controller

with Controller("can0") as ctrl:
    m = ctrl.add_robstride_motor(127, 0xFD, "rs-06")
    m.robstride_set_device_id(126)
    m.store_parameters()
    m.close()
```

## 11）写零点（zero / set-zero）

共同语义：
- 设置当前机械位置为零点；可选择持久化。

参数有效性：
- 有效：`mode=zero|set-zero`、`zero-exp`、`store`。
- 关键：`--zero-exp 1` 必须开启，否则不发送实验序列。

默认值：
- `store=1`（建议）。

原厂协议对应：
- `disable -> set_zero_position -> (optional) store_parameters`。

### Core CLI

```bash
$CLI --vendor robstride --channel "$CH" --model "$MODEL" --motor-id "$MID" --feedback-id "$FID" \
  --mode zero --zero-exp 1 --store 1
```

### Python CLI

```bash
motorbridge-cli run \
  --vendor robstride --channel "$CH" --model "$MODEL" --motor-id "$MID" --feedback-id "$FID" \
  --mode zero --zero-exp 1 --store 1

motorbridge-cli run \
  --vendor robstride --channel "$CH" --model "$MODEL" --motor-id "$MID" --feedback-id "$FID" \
  --mode set-zero --zero-exp 1 --store 1
```

### Python SDK

```python
from motorbridge import Controller

with Controller("can0") as ctrl:
    m = ctrl.add_robstride_motor(127, 0xFD, "rs-06")
    m.disable()
    m.set_zero_position()
    m.store_parameters()
    m.close()
```

验证建议：

```bash
motorbridge-cli robstride-read-param \
  --channel "$CH" --model "$MODEL" --motor-id "$MID" --feedback-id "$FID" \
  --param-id 0x7019 --type f32 --timeout-ms 200
```

## 12）最终口径（给查证者）

- 三通道在 RobStride 上的控制语义已对齐；主要差异在“入口形式”和少量命令覆盖面。
- Python CLI 已支持 RobStride `scan / ping(run) / read-param / write-param / id-set`。
- 参数“有效/无效”请以本手册各节为准，尤其：
  - `mit`：`pos/vel/kp/kd/tau` 全有效。
  - `pos-vel`：仅 `pos/vlim` 主有效；`vel/kd/tau` 无效。
  - `vel`：仅 `vel` 主有效。
- 若要做最严格复现实验，优先以 Core CLI 作为基准，再对照 Python CLI/SDK。
