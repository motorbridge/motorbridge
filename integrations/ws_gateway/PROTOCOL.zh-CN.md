# ws_gateway JSON 协议接口手册

本文面向浏览器/WebSocket/远程控制客户端，按 CLI 手册的粒度说明
`ws_gateway` 的 JSON 协议：每个 `op` 的作用、参数、默认值、适用厂商、
返回结构与使用示例。

源码入口：

- 网关主程序：`integrations/ws_gateway/src/main.rs`
- WS 路由：`integrations/ws_gateway/src/router/mod.rs`
- 连接/状态类操作：`integrations/ws_gateway/src/router/handlers/connection.rs`
- 统一控制类操作：`integrations/ws_gateway/src/router/handlers/control.rs`
- 厂商辅助控制：`integrations/ws_gateway/src/router/handlers/control_aux.rs`
- 寄存器/参数类操作：`integrations/ws_gateway/src/router/handlers/register.rs`
- scan / verify / set_id：`integrations/ws_gateway/src/commands/*.rs`

## 1. 基本模型

`ws_gateway` 是 Rust WebSocket 服务。客户端通过 WebSocket 发送 JSON 文本帧：

```json
{"op":"ping"}
```

网关返回 JSON 文本帧：

```json
{"ok":true,"op":"ping","data":{"pong":true,"vendor":"damiao"}}
```

失败时：

```json
{"ok":false,"op":"mit","error":"motor not connected"}
```

状态流推送不是普通响应，而是独立事件：

```json
{"type":"state","data":{"has_value":true,"pos":0.12,"vel":0.01,"torq":0.0,"status_code":1}}
```

## 2. 启动方式

### 2.1 源码启动

Damiao / SocketCAN：

```bash
cargo run -p ws_gateway --release -- --bind 127.0.0.1:9002 --vendor damiao --transport socketcan --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 --dt-ms 20
```

Damiao / 串口桥：

```bash
cargo run -p ws_gateway --release -- --bind 127.0.0.1:9002 --vendor damiao --transport dm-serial --serial-port /dev/ttyACM0 --serial-baud 921600 --model 4340P --motor-id 0x01 --feedback-id 0x11 --dt-ms 20
```

Damiao / DM_Device SDK USB2CANFD_DUAL：

```bash
cargo run -p ws_gateway --release -- --bind 127.0.0.1:9002 --vendor damiao --transport dm-device --dm-device-type usb2canfd-dual --dm-channel 1 --model 4310 --motor-id 0x04 --feedback-id 0x14 --dt-ms 20
```

Damiao / DM_Device SDK LINKX4C：

```bash
cargo run -p ws_gateway --release -- --bind 127.0.0.1:9002 --vendor damiao --transport dm-device --dm-device-type linkx4c --dm-channel 0 --model 4310 --motor-id 0x04 --feedback-id 0x14 --dt-ms 20
```

RobStride：

```bash
cargo run -p ws_gateway --release -- --bind 127.0.0.1:9002 --vendor robstride --transport socketcan --channel can0 --model rs-00 --motor-id 127 --feedback-id 0xFD --dt-ms 20
```

### 2.2 pip 包启动

安装后的 Python 包会包含或寻找 `ws_gateway` 可执行文件：

```bash
motorbridge-gateway -- --bind 127.0.0.1:9002 --vendor damiao --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 --dt-ms 20
```

若需要手动指定网关二进制：

```bash
export MOTORBRIDGE_WS_GATEWAY_BIN=/path/to/ws_gateway
motorbridge-gateway -- --bind 127.0.0.1:9002 --vendor damiao --channel can0
```

Windows PowerShell：

```powershell
$env:MOTORBRIDGE_WS_GATEWAY_BIN="C:\path\to\ws_gateway.exe"
motorbridge-gateway -- --bind 127.0.0.1:9002 --vendor robstride --channel can0@1000000 --model rs-00 --motor-id 127 --feedback-id 0xFD
```

## 3. 启动参数

| 参数 | 类型 | 默认值 | 作用 |
| --- | --- | --- | --- |
| `--bind` | string | `127.0.0.1:9002` | WebSocket 监听地址 |
| `--vendor` | enum | `damiao` | 默认厂商：`damiao` / `robstride` / `hexfellow` / `myactuator` / `hightorque` |
| `--transport` | enum | `auto` | 链路：`auto` / `socketcan` / `socketcanfd` / `dm-serial` / `dm-device` |
| `--channel` | string | `can0` | CAN 通道。Linux 通常 `can0`；Windows PCAN 可用 `can0@1000000` |
| `--serial-port` | string | `/dev/ttyACM0` | Damiao `dm-serial` 串口设备 |
| `--serial-baud` | u32 | `921600` | Damiao `dm-serial` 波特率 |
| `--dm-device-type` | string | `usb2canfd-dual` | Damiao `dm-device` 适配器类型：`usb2canfd` / `usb2canfd-dual` / `linkx4c` |
| `--dm-channel` | string | `0` | Damiao `dm-device` 控制通道：`usb2canfd` 用 `0`，`usb2canfd-dual` 用 `0` / `1`，`linkx4c` 用 SDK 通道 `0` / `1` / `2` / `3`；scan 消息不带 `dm_channel` 时会扫描所选适配器全部通道 |
| `--model` | string | 厂商默认 | 电机型号，例如 `4340P` / `rs-00` |
| `--motor-id` | u16 | `0x01` | 电机命令 ID / device ID |
| `--feedback-id` | u16 | 厂商默认 | 反馈 ID。RobStride 中它是 host_id，不是 motor_id |
| `--dt-ms` | u64 | `20` | 网关 tick 周期；影响 continuous 命令和 state stream |

## 4. 安全与鉴权

本机使用推荐绑定：

```bash
--bind 127.0.0.1:9002
```

如果绑定非回环地址，例如 `0.0.0.0:9002`，必须设置：

```bash
export MOTORBRIDGE_WS_TOKEN=your-token
MOTORBRIDGE_WS_TOKEN=your-token motorbridge-gateway -- --bind 0.0.0.0:9002
```

PowerShell：

```powershell
$env:MOTORBRIDGE_WS_TOKEN="your-token"
```

客户端握手可带 header：

```text
x-motorbridge-token: your-token
```

或：

```text
Authorization: Bearer your-token
```

浏览器客户端也可直接在 WS URL 上带 query：

```text
ws://host:9002/?motorbridge_ws_token=your-token
```

## 5. JSON 类型与通用规则

每个请求必须包含：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `op` | string | 操作名 |

ID 字段支持数字或十六进制字符串：

```json
{"motor_id":1}
{"motor_id":"0x01"}
```

常见通用字段：

| 字段 | 类型 | 默认值 | 作用 |
| --- | --- | --- | --- |
| `vendor` | string | 当前 target vendor | 临时指定厂商 |
| `transport` | string | 当前 target transport | 临时指定链路 |
| `channel` | string | 当前 target channel | CAN 通道 |
| `dm_device_type` | string | 当前 target dm_device_type | `dm-device` 适配器类型 |
| `dm_channel` | string | 当前 target dm_channel | `dm-device` 通道 |
| `model` | string | 当前 target model | 型号 |
| `motor_id` | u16 | 当前 target motor_id | 电机 ID |
| `feedback_id` | u16 | 当前 target feedback_id | 反馈 ID / RobStride host_id |
| `timeout_ms` | u64 | 操作默认值 | 等待硬件回复超时 |

## 6. 推荐调用流程

### 6.1 单电机本机控制

1. 先用 `motor_cli` 验证硬件。
2. 启动 `ws_gateway`。
3. WebSocket 连接 `ws://127.0.0.1:9002`。
4. 发送 `ping`。
5. 发送 `state_stream enabled=true`。
6. 发送 `enable`。
7. 发送 `pos_vel` / `mit` / `vel`。
8. 发送 `stop`。
9. 发送 `disable`。

示例：

```json
{"op":"ping"}
{"op":"state_stream","enabled":true}
{"op":"enable"}
{"op":"pos_vel","pos":0.5,"vlim":1.0,"continuous":true}
{"op":"stop"}
{"op":"disable"}
```

### 6.2 动态切换电机

`set_target` 会断开当前 session 内的 motor/controller，并切换默认目标。

```json
{"op":"set_target","vendor":"robstride","transport":"socketcan","channel":"can0","model":"rs-00","motor_id":127,"feedback_id":"0xFD"}
```

随后所有不带 `vendor/model/id` 的操作都作用于该目标。

## 7. 返回结构

普通成功响应：

```json
{"ok":true,"op":"enable","data":{"enabled":true}}
```

普通失败响应：

```json
{"ok":false,"op":"enable","error":"..."}
```

状态流：

```json
{"type":"state","data":{...}}
```

## 8. 连接与会话类 op

### 8.1 `ping`

作用：检查 WS 与当前目标基本可用性。RobStride 会真实发送 ping；其他厂商返回软件 pong。

参数：

| 字段 | 类型 | 默认值 | 作用 |
| --- | --- | --- | --- |
| `timeout_ms` | u64 | `200` | RobStride ping 超时 |

请求：

```json
{"op":"ping"}
```

RobStride 请求：

```json
{"op":"ping","timeout_ms":500}
```

返回：

```json
{"pong":true,"vendor":"damiao"}
```

RobStride 返回：

```json
{"pong":true,"vendor":"robstride","device_id":127,"responder_id":253}
```

### 8.2 `set_target`

作用：切换当前 WS session 的目标电机。会断开旧连接，清除 continuous active 命令。

参数：

| 字段 | 类型 | 默认值 | 作用 |
| --- | --- | --- | --- |
| `vendor` | string | 当前 vendor | 目标厂商 |
| `transport` | string | 当前 transport | 链路 |
| `channel` | string | 当前 channel | CAN 通道 |
| `serial_port` | string | 当前 serial_port | `dm-serial` 串口 |
| `serial_baud` | u32 | 当前 serial_baud | `dm-serial` 波特率 |
| `dm_device_type` | string | 当前 dm_device_type | `dm-device` 适配器类型；也接受 `dm-device-type` |
| `dm_channel` | string | 当前 dm_channel | `dm-device` 通道；也接受 `dm-channel` |
| `model` | string | 当前 model | 型号 |
| `motor_id` | u16/string | 当前 motor_id | 电机 ID |
| `feedback_id` | u16/string | 当前 feedback_id | 反馈 ID / host_id |

请求：

```json
{"op":"set_target","vendor":"damiao","transport":"dm-serial","serial_port":"/dev/ttyACM0","serial_baud":921600,"model":"4340P","motor_id":"0x01","feedback_id":"0x11"}
```

Damiao / DM_Device SDK：

```json
{"op":"set_target","vendor":"damiao","transport":"dm-device","dm_device_type":"usb2canfd-dual","dm_channel":"1","model":"4310","motor_id":"0x04","feedback_id":"0x14"}
```

```json
{"op":"set_target","vendor":"damiao","transport":"dm-device","dm_device_type":"linkx4c","dm_channel":"0","model":"4310","motor_id":"0x04","feedback_id":"0x14"}
```

RobStride：

```json
{"op":"set_target","vendor":"robstride","channel":"can0","model":"rs-00","motor_id":127,"feedback_id":"0xFD"}
```

返回：

```json
{"vendor":"robstride","transport":"socketcan","channel":"can0","serial_port":"/dev/ttyACM0","serial_baud":921600,"model":"rs-00","motor_id":127,"feedback_id":253}
```

### 8.3 `enable`

作用：使能当前 target 所在 controller 中的电机。

适用：

| 厂商 | 行为 |
| --- | --- |
| Damiao | `enable_all()` |
| RobStride | `enable_all()` |
| Hexfellow | `enable_all()` |
| MyActuator | `enable_all()` |
| HighTorque | 接受但 no-op |

请求：

```json
{"op":"enable"}
```

返回：

```json
{"enabled":true}
```

### 8.4 `disable`

作用：失能当前 target 所在 controller 中的电机，并清除 continuous active 命令。

请求：

```json
{"op":"disable"}
```

返回：

```json
{"disabled":true}
```

### 8.5 `stop`

作用：停止 continuous active 命令，并发送厂商对应的受控停止命令。`stop`
与 `disable` 语义不同：前者尽量保留闭环控制/保持力矩，后者取消电机力矩。

厂商行为：

| 厂商 | 行为 |
| --- | --- |
| Damiao | `send_cmd_vel(0.0)` |
| RobStride | 按实时读取的 `run_mode` 选择：MIT 保持当前位置（沿用最近一次 `kp/kd`，若无缓存或增益太小则使用默认安全增益）、PP 写 `vel_max=0`、Velocity 写 `spd_ref=0`、CSP 将 `loc_ref` 写为当前位置 |
| Hexfellow | 发送零 MIT |
| MyActuator | `stop_motor()` |
| HighTorque | 发送 stop raw frame |

请求：

```json
{"op":"stop"}
```

返回：

```json
{"stopped":true,"mode":"mit","strategy":"hold-current-position-mit"}
```

`stopped` 为兼容旧客户端保留的命令接受字段。RobStride 还返回 `mode` 与
`strategy`，便于客户端确认电机实际使用的停止策略。如果 MIT 会话没有缓存
到最近一次 `kp/kd`，或缓存的增益太小（如纯力矩模式 `kp=0, kd=0`），
网关会使用默认安全增益（`kp_max * 10%`, `kd_max * 10%`），并在 stderr
输出警告日志。默认增益足以提供位置保持，但可能不如用户自定义增益精确。

### 8.6 `state_once`

作用：读取当前缓存状态快照。不会主动请求所有厂商硬件状态，必要时先调用 `request_feedback` 或 `status`。

请求：

```json
{"op":"state_once"}
```

返回示例：

```json
{"state":{"has_value":true,"pos":0.12,"vel":0.0,"torq":0.0,"status_code":1,"status_name":"ENABLED"}}
```

### 8.7 `state_stream`

作用：开启/关闭周期状态推送。推送周期由网关启动参数 `--dt-ms` 决定。

参数：

| 字段 | 类型 | 默认值 | 作用 |
| --- | --- | --- | --- |
| `enabled` | bool | `false` | 是否开启状态流 |

开启：

```json
{"op":"state_stream","enabled":true}
```

关闭：

```json
{"op":"state_stream","enabled":false}
```

普通响应：

```json
{"enabled":true}
```

之后周期推送：

```json
{"type":"state","data":{"has_value":true,"pos":0.1,"vel":0.0,"torq":0.0}}
```

### 8.8 `damiao_state_many`

作用：Damiao 专用的多电机状态批量读取。适合 `dm-serial` 或 `dm-device` 整机机械臂扫描后，
浏览器上位机按扫描结果周期刷新多个关节的 `pos` / `vel` / `torq`。

请求：

```json
{
  "op": "damiao_state_many",
  "items": [
    {"motor_id": 1, "feedback_id": 17, "model": "4340P"},
    {"motor_id": 2, "feedback_id": 18, "model": "4340P"}
  ],
  "timeout_ms": 120
}
```

参数：

| 字段 | 类型 | 默认值 | 作用 |
| --- | --- | --- | --- |
| `items` / `motors` | array | 必填 | 要读取的 Damiao 电机列表 |
| `motor_id` / `esc_id` | u16 | 必填 | 电机命令 ID |
| `feedback_id` / `mst_id` | u16 | 必填 | 电机反馈 ID |
| `model` | string | 当前 target model | 电机型号 |
| `timeout_ms` | u64 | `120` | 单个电机等待新鲜反馈的超时 |

返回：

```json
{
  "vendor": "damiao",
  "states": [
    {"motor_id":1,"feedback_id":17,"model":"4340P","has_value":true,"pos":0.12,"vel":0.01,"torq":0.0,"status_code":1},
    {"motor_id":2,"feedback_id":18,"model":"4340P","has_value":false}
  ]
}
```

说明：

- 该 op 只支持 Damiao。当前 target 不是 Damiao 时返回 `ok:false`。
- 不在线或没有及时返回的关节会以 `has_value:false` 留在 `states` 中，不会导致整个请求失败。
- v0.4.1 起，扫描前 gateway 会释放 Damiao 活跃会话和 stream，避免 Windows `dm-serial` 整臂扫描时串口争用。

### 8.9 `param_stream` / `damiao_param_stream` / `robstride_param_stream`

作用：开启/关闭当前 WS session 的当前 target 电机观测参数流。前端切换 `set_target` 到哪个电机，之后推送的就是那个电机的参数。

适用：Damiao、RobStride。

- 推荐前端统一使用 `param_stream`，gateway 会按当前 vendor 自动选择 Damiao 或 RobStride。
- `damiao_param_stream` 是 Damiao 专用别名，当前 target 不是 Damiao 时返回错误。
- `robstride_param_stream` 是 RobStride 专用别名，当前 target 不是 RobStride 时返回错误。
- 参数流会周期性读取寄存器/参数，因此比 `state_stream` 更重；曲线 UI 推荐 `profile=realtime`，诊断时再用 `profile=full`。

参数：

| 字段 | 类型 | 默认值 | 作用 |
| --- | --- | --- | --- |
| `enabled` | bool | `false` | 是否开启参数流 |
| `profile` | string | `realtime` | `realtime` 读取常用实时观测；`full`/`all` 读取完整观测参数集 |
| `params` / `param_ids` | array/string | 无 | 自定义参数列表；RobStride 用参数 ID，Damiao 用 RID；支持数组或逗号字符串 |
| `interval_ms` | u64 | `1000` 左右 | 推送周期；会按 gateway `--dt-ms` 对齐 |
| `timeout_ms` | u64 | `80` | 单个参数读取超时，限制在 `20..1000` |

Damiao `realtime` profile 默认读取：

| RID | 类型 | 名称 | 作用 |
| --- | --- | --- | --- |
| `10` | `u32` | `CTRL_MODE` | 控制模式 |
| `21` | `f32` | `PMAX` | 位置映射范围 |
| `22` | `f32` | `VMAX` | 速度映射范围 |
| `23` | `f32` | `TMAX` | 扭矩映射范围 |
| `60` | `f32` | `VBus` | 电源电压 |
| `61` | `f32` | `Tpcb` | 驱动板温度 |
| `62` | `f32` | `Tmtr` | 电机温度 |
| `80` | `f32` | `p_m` | 电机位置 |
| `81` | `f32` | `xout` | 输出轴位置 |

RobStride `realtime` profile 默认读取：

| 参数 | 类型 | 名称 | 作用 |
| --- | --- | --- | --- |
| `0x7005` | `i8` | `run_mode` | 当前运行模式 |
| `0x7019` | `f32` | `mechPos` | 机械位置 |
| `0x701A` | `f32` | `iqf` | 滤波 Iq 电流 |
| `0x701B` | `f32` | `mechVel` | 机械速度 |
| `0x701C` | `f32` | `VBUS` | 母线电压 |
| `0x3025` | `f32` | `drv_temp` | 驱动温度 |
| `0x302B` | `f32` | `v_bus` | 母线电压反馈 |
| `0x302C` | `f32` | `torque_fdb` | 扭矩反馈 |

通用开启方式：

```json
{"op":"param_stream","enabled":true,"profile":"realtime","interval_ms":1000,"timeout_ms":80}
```

Damiao 专用开启方式：

```json
{"op":"damiao_param_stream","enabled":true,"profile":"realtime","interval_ms":1000,"timeout_ms":80}
```

RobStride 专用开启方式：

```json
{"op":"robstride_param_stream","enabled":true,"profile":"realtime","interval_ms":1000,"timeout_ms":80}
```

自定义曲线参数：

```json
{"op":"param_stream","enabled":true,"params":["0x7019","0x701A","0x701B","0x302C"],"interval_ms":500}
```

关闭：

```json
{"op":"param_stream","enabled":false}
```

普通响应：

```json
{"enabled":true,"vendor":"robstride","interval_ms":1000,"timeout_ms":80,"params":[28677,28697,28698,28699,28700,12325,12331,12332]}
```

RobStride 周期推送：

```json
{"type":"robstride_params","data":{"vendor":"robstride","motor_id":1,"feedback_id":253,"model":"rs-00","values":{"mechPos":-0.82,"iqf":0.31,"mechVel":0.02,"torque_fdb":0.08},"params":[{"param_id":28697,"name":"mechPos","type":"f32","value":-0.82,"ok":true}]}}
```

Damiao 周期推送：

```json
{"type":"damiao_params","data":{"vendor":"damiao","motor_id":1,"feedback_id":17,"model":"4340P","values":{"VBus":24.1,"Tpcb":36.0,"Tmtr":34.0,"p_m":0.12,"xout":0.12},"params":[{"rid":60,"name":"VBus","type":"f32","value":24.1,"ok":true}]}}
```

如果某个参数读取失败，该参数会在 `params[]` 中返回 `ok:false/error`；其它成功参数仍然推送。

### 8.10 `status`

作用：获取状态。MyActuator 会主动 request status + angle；其他厂商主要返回缓存状态。

请求：

```json
{"op":"status"}
```

返回：

```json
{"state":{"has_value":true,"pos":0.1,"vel":0.0,"torq":0.0}}
```

### 8.11 `poll_feedback_once`

作用：手动 drain 一次 CAN RX 队列，将反馈帧分发到对应 motor。

请求：

```json
{"op":"poll_feedback_once"}
```

返回：

```json
{"polled":true}
```

### 8.12 `shutdown`

作用：对当前 controller/bus 执行 shutdown，并清除 active 命令。

请求：

```json
{"op":"shutdown"}
```

返回：

```json
{"shutdown":true}
```

### 8.13 `close_bus`

作用：断开当前 session 的 controller/motor，不一定执行硬件 shutdown。

请求：

```json
{"op":"close_bus"}
```

返回：

```json
{"closed":true}
```

## 9. 统一控制类 op

统一控制 op 会自动 `ensure_connected()`。`continuous=true` 时，命令会成为 active command，并在每个网关 tick 重复发送；`stop` 或 `disable` 会清除 active command。

### 9.1 `mit`

作用：MIT 控制。

适用：

| 厂商 | 支持 | 说明 |
| --- | --- | --- |
| Damiao | 是 | 自动 ensure MIT mode |
| RobStride | 是 | 自动切 MIT mode 并 enable |
| Hexfellow | 是 | 转成 rev / rev/s |
| HighTorque | 是 | raw frame 映射，`kp/kd` 被协议忽略 |
| MyActuator | 否 | 返回 unsupported |

参数：

| 字段 | 类型 | 默认值 | 作用 |
| --- | --- | --- | --- |
| `pos` | f32 | `0.0` | 目标位置，rad |
| `vel` | f32 | `0.0` | 目标速度，rad/s |
| `kp` | f32 | `30.0` | 刚度 |
| `kd` | f32 | `1.0` | 阻尼 |
| `tau` | f32 | `0.0` | 前馈力矩，Nm |
| `continuous` | bool | `false` | 是否周期发送 |
| `ensure_timeout_ms` | u64 | `1000` | Damiao ensure mode 超时 |

请求：

```json
{"op":"mit","pos":0.0,"vel":0.0,"kp":20.0,"kd":0.5,"tau":0.0,"continuous":true}
```

返回：

```json
{"op":"mit","continuous":true}
```

### 9.2 `pos_vel` / `pos-vel`

作用：位置 + 速度限制控制。

适用：

| 厂商 | 支持 | 说明 |
| --- | --- | --- |
| Damiao | 是 | 原生 POS_VEL |
| RobStride | 是 | 切 Position mode (PP)，写 `0x7024 vel_max`、`0x701E loc_kp`、`0x7016 loc_ref` |
| Hexfellow | 是 | 转成 rev / rev/s |
| HighTorque | 当前 handler 返回不支持 | 后续可扩展 |
| MyActuator | 当前 handler 返回不支持 | 可用 `pos` 原生 op |

参数：

| 字段 | 类型 | 默认值 | 作用 |
| --- | --- | --- | --- |
| `pos` | f32 | `0.0` | 目标位置，rad |
| `vlim` | f32 | `1.0` | 速度限制，rad/s |
| `loc_kp` | f32 | 无 | RobStride 位置环 Kp；若无则尝试 `kp` |
| `kp` | f32 | 无 | RobStride `loc_kp` fallback |
| `continuous` | bool | `false` | 是否周期发送 |
| `ensure_timeout_ms` | u64 | `1000` | Damiao ensure mode 超时 |

请求：

```json
{"op":"pos_vel","pos":1.0,"vlim":1.5,"continuous":true}
```

RobStride：

```json
{"op":"pos_vel","pos":0.5,"vlim":1.0,"loc_kp":2.0,"continuous":true}
```

返回：

```json
{"op":"pos_vel","continuous":true}
```

### 9.3 `vel`

作用：速度控制。

适用：

| 厂商 | 支持 | 说明 |
| --- | --- | --- |
| Damiao | 是 | 原生 VEL |
| RobStride | 是 | 切 Velocity mode 并写 velocity target |
| MyActuator | 是 | rad/s 转 deg/s |
| HighTorque | 是 | raw velocity frame |
| Hexfellow | 否 | 用 `pos_vel` 或 `mit` |

参数：

| 字段 | 类型 | 默认值 | 作用 |
| --- | --- | --- | --- |
| `vel` | f32 | `0.0` | 目标速度，rad/s |
| `continuous` | bool | `false` | 是否周期发送 |
| `ensure_timeout_ms` | u64 | `1000` | Damiao ensure mode 超时 |

请求：

```json
{"op":"vel","vel":0.3,"continuous":true}
```

返回：

```json
{"op":"vel","continuous":true}
```

### 9.4 `force_pos` / `force-pos`

作用：位置 + 速度限制 + 力矩限幅比例控制。

适用：

| 厂商 | 支持 |
| --- | --- |
| Damiao | 是 |
| RobStride | 否 |
| Hexfellow | 否 |
| MyActuator | 否 |
| HighTorque | 当前 handler 返回不支持 |

参数：

| 字段 | 类型 | 默认值 | 作用 |
| --- | --- | --- | --- |
| `pos` | f32 | `0.0` | 目标位置，rad |
| `vlim` | f32 | `1.0` | 速度限制 |
| `ratio` | f32 | `0.3` | 力矩限幅比例，通常 `0.0..1.0` |
| `continuous` | bool | `false` | 是否周期发送 |
| `ensure_timeout_ms` | u64 | `1000` | Damiao ensure mode 超时 |

请求：

```json
{"op":"force_pos","pos":0.8,"vlim":2.0,"ratio":0.3,"continuous":true}
```

返回：

```json
{"op":"force_pos","continuous":true}
```

## 10. 厂商辅助控制 op

### 10.1 `clear_error`

作用：清故障。

适用：Damiao、RobStride。

请求：

```json
{"op":"clear_error"}
```

返回：

```json
{"cleared":true}
```

### 10.2 `set_zero_position`

作用：当前位置置零。

适用：

| 厂商 | 行为 |
| --- | --- |
| Damiao | `set_zero_position()`；核心层要求先 disable |
| RobStride | `set_zero_position()` |
| MyActuator | `set_current_position_as_zero()` |
| Hexfellow | 不支持 |
| HighTorque | 不支持 |

请求：

```json
{"op":"set_zero_position"}
```

返回：

```json
{"zero_set":true}
```

### 10.3 `ensure_mode`

作用：显式切换/确认控制模式。

参数：

| 字段 | 类型 | 默认值 | 作用 |
| --- | --- | --- | --- |
| `mode` | string/number | 必填 | 目标模式 |
| `timeout_ms` | u64 | `1000` | 校验超时 |

Damiao mode：

| 输入 | 含义 |
| --- | --- |
| `"mit"` 或 `1` | MIT |
| `"pos_vel"` / `"pos-vel"` 或 `2` | POS_VEL |
| `"vel"` 或 `3` | VEL |
| `"force_pos"` / `"force-pos"` 或 `4` | FORCE_POS |

RobStride mode：

| 输入 | 含义 |
| --- | --- |
| `"mit"` 或 `0` | MIT |
| `"position"` / `"pos"` 或 `1` | Position |
| `"vel"` / `"velocity"` 或 `2` | Velocity |

Hexfellow mode：

| 输入 | 含义 |
| --- | --- |
| `"mit"` / `1` | MIT |
| `"pos_vel"` / `"pos-vel"` / `2` | POS_VEL |

请求：

```json
{"op":"ensure_mode","mode":"pos_vel","timeout_ms":1000}
```

返回：

```json
{"ensured":true}
```

说明：Damiao 的 `ensure_mode` 以及 `mit` / `pos_vel` / `vel` / `force_pos`
会写入并回读 `RID 10`（`CTRL_MODE`）确认模式。若仅回读确认超时，WS 会返回
`ok:true`，并在 `data.warning` / `data.warnings` 中说明“确认超时但命令已继续”；
若读到明确不匹配的模式值，仍返回 `ok:false`。

### 10.4 `request_feedback`

作用：主动请求或轮询一次反馈。

厂商行为：

| 厂商 | 行为 |
| --- | --- |
| Damiao | 发送 feedback request |
| RobStride | poll feedback once |
| Hexfellow | poll feedback once |
| MyActuator | request status 后 poll |
| HighTorque | 发送 read raw frame |

请求：

```json
{"op":"request_feedback"}
```

返回：

```json
{"requested":true}
```

### 10.5 `set_active_report`

作用：开启/关闭 RobStride 主动上报。

适用：RobStride。

参数：

| 字段 | 类型 | 默认值 | 作用 |
| --- | --- | --- | --- |
| `enabled` | bool | `true` | 是否开启主动上报 |

请求：

```json
{"op":"set_active_report","enabled":true}
```

返回：

```json
{"active_report":true}
```

### 10.6 `store_parameters`

作用：保存/持久化当前参数。

适用：

| 厂商 | 行为 |
| --- | --- |
| Damiao | `store_parameters()` |
| RobStride | `save_parameters()` |
| 其他 | 不支持 |

请求：

```json
{"op":"store_parameters"}
```

返回：

```json
{"stored":true}
```

### 10.7 `set_can_timeout_ms`

作用：设置电机侧 CAN timeout。

适用：

| 厂商 | 行为 |
| --- | --- |
| Damiao | 写寄存器 `RID 9 = timeout_ms * 20` |
| RobStride | 写参数 `0x7028 canTimeout = timeout_ms` |
| 其他 | 不支持 |

请求：

```json
{"op":"set_can_timeout_ms","timeout_ms":1000}
```

Damiao 返回：

```json
{"timeout_ms":1000,"reg9_value":20000}
```

RobStride 返回：

```json
{"timeout_ms":1000,"param_id":"0x7028"}
```

## 11. Damiao 寄存器 op

这些 op 只适用于 Damiao。RobStride 请使用 `robstride_read_param` / `robstride_write_param`。

### 11.1 `write_register_u32`

作用：写 Damiao u32 寄存器。

参数：

| 字段 | 类型 | 默认值 | 作用 |
| --- | --- | --- | --- |
| `rid` | u8/u16/string | `0` | 寄存器 ID |
| `value` | u32 | `0` | 写入值 |
| `verify` | bool | `true` | 写入后回读并比较 |
| `verify_timeout_ms` / `timeout_ms` | u64 | `1000` | 单次回读确认超时 |
| `verify_attempts` | u64 | `2` | 回读确认次数，范围 `1..5` |

请求：

```json
{"op":"write_register_u32","rid":10,"value":2,"verify":true}
```

返回：

```json
{"rid":10,"value":2,"readback":2,"verified":true}
```

若写入已发出但回读确认没有收到，返回 `ok:true`，并在 `data.warning` /
`data.warnings` 中说明未完成确认；若回读值与写入值明确不一致，则返回
`ok:false`。

### 11.2 `write_register_f32`

作用：写 Damiao f32 寄存器。

参数同 `write_register_u32`，但 `value` 为 f32。回读确认使用浮点容差比较。

请求：

```json
{"op":"write_register_f32","rid":31,"value":5.0,"verify":true}
```

返回：

```json
{"rid":31,"value":5.0}
```

### 11.3 `get_register_u32`

作用：读 Damiao u32 寄存器。

参数：

| 字段 | 类型 | 默认值 | 作用 |
| --- | --- | --- | --- |
| `rid` | u8/u16/string | `0` | 寄存器 ID |
| `timeout_ms` | u64 | `1000` | 读超时 |

请求：

```json
{"op":"get_register_u32","rid":10,"timeout_ms":1000}
```

返回：

```json
{"rid":10,"value":2}
```

### 11.4 `get_register_f32`

作用：读 Damiao f32 寄存器。

请求：

```json
{"op":"get_register_f32","rid":21,"timeout_ms":1000}
```

返回：

```json
{"rid":21,"value":12.5}
```

## 12. RobStride 参数 op

### 12.1 `robstride_ping`

作用：发送 RobStride ping。

参数：

| 字段 | 类型 | 默认值 | 作用 |
| --- | --- | --- | --- |
| `timeout_ms` | u64 | `200` | ping 超时 |

请求：

```json
{"op":"robstride_ping","timeout_ms":500}
```

返回：

```json
{"device_id":127,"responder_id":253}
```

### 12.2 `robstride_read_param`

作用：读取 RobStride 参数。

参数：

| 字段 | 类型 | 默认值 | 作用 |
| --- | --- | --- | --- |
| `param_id` | u16/string | 必填/默认取实现路径 | 参数 ID |
| `type` | string | 参数表推断或调用路径默认 | `i8` / `u8` / `u16` / `u32` / `f32` |
| `timeout_ms` | u64 | 通常 `200..500` | 读取超时 |

常用参数：

| 参数 | 类型 | 读写 | 作用 |
| --- | --- | --- | --- |
| `0x7005` | `i8` | W/R | `run_mode` |
| `0x700A` | `f32` | W/R | `spd_ref` |
| `0x7017` | `f32` | W/R | `limit_spd` |
| `0x7019` | `f32` | R | `mechPos` |
| `0x701E` | `f32` | W/R | `loc_kp` |
| `0x7028` | `u32` | W | `canTimeout` |

请求：

```json
{"op":"robstride_read_param","param_id":"0x7019","type":"f32","timeout_ms":500}
```

返回示例：

```json
{"param_id":28697,"type":"f32","value":0.123}
```

### 12.3 `robstride_write_param`

作用：写 RobStride 参数，可选读回校验。

参数：

| 字段 | 类型 | 默认值 | 作用 |
| --- | --- | --- | --- |
| `param_id` | u16/string | 必填/默认取实现路径 | 参数 ID |
| `type` | string | 参数表推断或调用路径默认 | 值类型 |
| `value` | number | 必填 | 写入值 |
| `verify` | bool | `false` 或实现默认 | 是否写后读回 |
| `timeout_ms` | u64 | 通常 `200..500` | verify 读回超时 |

安全测试建议写 `0x7017 limit_spd`，不要写只读 `0x7019`。

请求：

```json
{"op":"robstride_write_param","param_id":"0x7017","type":"f32","value":1.0,"verify":true,"timeout_ms":500}
```

返回示例：

```json
{"param_id":28695,"type":"f32","value":1.0,"verify":1.0}
```

如需持久化，写成功并确认值安全后再调用：

```json
{"op":"store_parameters"}
```

## 13. MyActuator 原生 op

### 13.1 `current`

作用：MyActuator 电流控制。

参数：

| 字段 | 类型 | 默认值 | 作用 |
| --- | --- | --- | --- |
| `current` | f32 | `0.0` | 电流 A |

请求：

```json
{"op":"current","current":0.2}
```

返回：

```json
{"op":"current","current":0.2}
```

### 13.2 `pos`

作用：MyActuator 绝对位置控制。

参数：

| 字段 | 类型 | 默认值 | 作用 |
| --- | --- | --- | --- |
| `pos` | f32 | `0.0` | 目标位置，rad |
| `max_speed` | f32 | `8.726646` | 最大速度，rad/s；内部转 deg/s |

请求：

```json
{"op":"pos","pos":1.0,"max_speed":2.0}
```

返回：

```json
{"op":"pos","pos":1.0,"max_speed":2.0}
```

### 13.3 `version`

作用：读取 MyActuator 版本日期。

参数：

| 字段 | 类型 | 默认值 | 作用 |
| --- | --- | --- | --- |
| `timeout_ms` | u64 | `500` | 等待版本回复 |

请求：

```json
{"op":"version","timeout_ms":500}
```

返回：

```json
{"version":"..."}
```

### 13.4 `mode_query` / `mode-query`

作用：查询 MyActuator 控制模式。

请求：

```json
{"op":"mode_query"}
```

返回：

```json
{"mode":1}
```

## 14. HighTorque 原生 op

### 14.1 `read`

作用：读取 HighTorque 状态。

参数：

| 字段 | 类型 | 默认值 | 作用 |
| --- | --- | --- | --- |
| `timeout_ms` | u64 | `500` | 读取超时 |

请求：

```json
{"op":"read","timeout_ms":500}
```

返回：

```json
{"motor_id":1,"pos_raw":0,"vel_raw":0,"tqe_raw":0,"pos":0.0,"vel":0.0,"torq":0.0}
```

## 15. 扫描/校验/改 ID

这些操作不依赖当前 session 已连接的 motor，会按消息中的 `vendor/transport/channel/model/id` 或当前 target 创建临时 controller。

### 15.1 `scan`

作用：扫描电机。

通用参数：

| 字段 | 类型 | 默认值 | 作用 |
| --- | --- | --- | --- |
| `vendor` | string | 当前 vendor | 扫描厂商 |
| `transport` | string | 当前 transport | 链路 |
| `channel` | string | 当前 channel | CAN 通道 |
| `serial_port` | string | 当前 serial_port | `dm-serial` 串口 |
| `serial_baud` | u32 | 当前 serial_baud | `dm-serial` 波特率 |
| `dm_device_type` | string | 当前 dm_device_type | `dm-device` 适配器类型 |
| `dm_channel` | string | 不传则 all | `dm-device` 扫描通道；不传会扫描所选适配器全部通道：`usb2canfd` 为 `0`，`usb2canfd-dual` 为 `0|1`，`linkx4c` 为 `0|1|2|3`；传通道则只扫指定通道 |
| `start_id` | u16 | 厂商默认 | 起始 ID |
| `end_id` | u16 | 厂商默认 | 结束 ID |
| `timeout_ms` | u64 | 厂商默认 | 单 ID 超时 |

Damiao：

```json
{"op":"scan","vendor":"damiao","start_id":1,"end_id":16,"timeout_ms":100}
```

Damiao / DM_Device SDK：

```json
{"op":"scan","vendor":"damiao","transport":"dm-device","dm_device_type":"usb2canfd-dual","dm_channel":"1","model":"4310","start_id":1,"end_id":16,"feedback_base":16,"timeout_ms":80}
```

不带 `dm_channel` 时会扫描所选适配器全部通道。USB2CANFD_DUAL 示例：

```json
{"op":"scan","vendor":"damiao","transport":"dm-device","dm_device_type":"usb2canfd-dual","model":"4310","start_id":1,"end_id":16,"feedback_base":16,"timeout_ms":80}
```

LINKX4C 全四路扫描：

```json
{"op":"scan","vendor":"damiao","transport":"dm-device","dm_device_type":"linkx4c","model":"4310","start_id":1,"end_id":16,"feedback_base":16,"timeout_ms":80}
```

RobStride：

```json
{"op":"scan","vendor":"robstride","start_id":1,"end_id":127,"feedback_ids":"0xFD,0xFF,0xFE,0x00,0xAA","param_id":"0x7019","timeout_ms":120}
```

RobStride 参数：

| 字段 | 类型 | 默认值 | 作用 |
| --- | --- | --- | --- |
| `feedback_ids` | string/array | `0xFD,0xFF,0xFE` + 当前 `feedback_id` | host_id 候选列表；请求中传入的列表会追加并去重 |
| `param_id` | u16/string | `0x7019` | ping 失败时尝试读的参数 |

Hexfellow：

```json
{"op":"scan","vendor":"hexfellow","transport":"socketcanfd","start_id":1,"end_id":32,"timeout_ms":200}
```

MyActuator：

```json
{"op":"scan","vendor":"myactuator","start_id":1,"end_id":32,"timeout_ms":100}
```

HighTorque：

```json
{"op":"scan","vendor":"hightorque","start_id":1,"end_id":32,"timeout_ms":80}
```

返回：

```json
{"vendor":"robstride","transport":"socketcan","count":1,"start_id":1,"end_id":127,"hits":[{"probe":127,"via":"ping","feedback_id":253,"device_id":127,"responder_id":253}]}
```

### 15.2 `verify`

作用：校验某个 ID 是否能响应，并返回厂商相关确认信息。

通用参数：

| 字段 | 类型 | 默认值 | 作用 |
| --- | --- | --- | --- |
| `vendor` | string | 当前 vendor | 厂商 |
| `transport` | string | 当前 transport | 链路 |
| `motor_id` | u16/string | 当前 motor_id | 电机 ID |
| `feedback_id` | u16/string | 当前 feedback_id | 反馈 ID / host_id |
| `model` | string | 当前 model | 型号 |
| `timeout_ms` | u64 | `1000` | 超时 |

Damiao：

```json
{"op":"verify","vendor":"damiao","motor_id":"0x01","feedback_id":"0x11","model":"4340P","timeout_ms":1000}
```

RobStride：

```json
{"op":"verify","vendor":"robstride","motor_id":127,"feedback_id":"0xFD","timeout_ms":500}
```

返回：

```json
{"vendor":"damiao","transport":"socketcan","model_used":"4340P","motor_id":1,"feedback_id":17,"esc_id":1,"mst_id":17,"ok":true}
```

### 15.3 `set_id`

作用：修改电机 ID。

支持：

| 厂商 | 支持 | 说明 |
| --- | --- | --- |
| Damiao | 是 | 写 `MST_ID(RID 7)` 与 `ESC_ID(RID 8)` |
| RobStride | 是 | 修改 device_id；`feedback_id` 仍是 host_id |
| Hexfellow | 否 | 不支持 |
| MyActuator | 否 | 不支持 |
| HighTorque | 否 | 不支持 |

Damiao 参数：

| 字段 | 类型 | 默认值 | 作用 |
| --- | --- | --- | --- |
| `old_motor_id` | u16/string | 当前 motor_id | 旧 ESC_ID |
| `old_feedback_id` | u16/string | 当前 feedback_id | 旧 MST_ID |
| `new_motor_id` | u16/string | old_motor_id | 新 ESC_ID |
| `new_feedback_id` | u16/string | old_feedback_id | 新 MST_ID |
| `model` | string | 当前 model | 型号；`auto` 可尝试多个 Damiao 型号 |
| `store` | bool | `true` | 是否保存 |
| `verify` | bool | `true` | 是否改完后校验 |
| `timeout_ms` | u64 | `1000` | 校验超时 |

Damiao 请求：

```json
{"op":"set_id","vendor":"damiao","old_motor_id":2,"old_feedback_id":18,"new_motor_id":5,"new_feedback_id":21,"store":true,"verify":true,"timeout_ms":1000}
```

RobStride 参数：

| 字段 | 类型 | 默认值 | 作用 |
| --- | --- | --- | --- |
| `old_motor_id` | u16/string | 当前 motor_id | 旧 device_id，范围 `1..255` |
| `new_motor_id` | u16/string | old_motor_id | 新 device_id，范围 `1..255` |
| `feedback_id` | u16/string | 当前 feedback_id | host_id，范围 `0..255` |
| `verify` | bool | `true` | 是否改完后 ping 新 ID |
| `timeout_ms` | u64 | `1000` | 校验超时 |

RobStride 请求：

```json
{"op":"set_id","vendor":"robstride","old_motor_id":127,"new_motor_id":126,"feedback_id":"0xFD","verify":true,"timeout_ms":1000}
```

返回：

```json
{"vendor":"robstride","transport":"socketcan","old_motor_id":127,"new_motor_id":126,"feedback_id":253,"verify":{"vendor":"robstride","motor_id":126,"ok":true}}
## 16. capabilities

推荐客户端连接后先发：

```json
{"op":"capabilities"}
```

作用：获取网关声明的厂商、模式、操作能力。客户端 UI 应以此控制按钮可见性。

返回结构较大，核心字段：

```json
{
  "api_version":"v1",
  "default_vendor":"damiao",
  "vendors":{
    "damiao":{"transports":["auto","socketcan","socketcanfd","dm-serial","dm-device"],"modes":["mit","pos_vel","vel","force_pos"]},
    "robstride":{"transports":["auto","socketcan","socketcanfd"],"modes":["mit","pos_vel","vel"]}
  }
}
```

## 17. 浏览器 JS 最小示例

```html
<script>
const ws = new WebSocket("ws://127.0.0.1:9002");

ws.onopen = () => {
  ws.send(JSON.stringify({op: "ping"}));
  ws.send(JSON.stringify({op: "state_stream", enabled: true}));
};

ws.onmessage = (ev) => {
  const msg = JSON.parse(ev.data);
  console.log("ws", msg);
};

function enable() {
  ws.send(JSON.stringify({op: "enable"}));
}

function moveTo(pos) {
  ws.send(JSON.stringify({
    op: "pos_vel",
    pos,
    vlim: 1.0,
    continuous: true
  }));
}

function stop() {
  ws.send(JSON.stringify({op: "stop"}));
}
</script>
```

## 18. 调试建议

1. 先用 `motor_cli` 验证硬件，不要一上来调 WS。
2. 再启动 `ws_gateway`。
3. 先发 `ping` 和 `verify`。
4. 再发 `enable`。
5. 第一次控制建议 `continuous=false`，确认安全后再打开 continuous。
6. 浏览器 UI 调试优先看 `state_stream` 和普通响应中的 `ok:false/error`。
7. RobStride 的 `feedback_id` 是 host_id；常用 `0xFD`。
8. Damiao `dm-serial` 和 `dm-device` 只支持 Damiao，不支持 RobStride/MyActuator/Hexfellow/HighTorque。
9. 非本机绑定必须设置 `MOTORBRIDGE_WS_TOKEN`。

## 19. 常见错误

### `motor not connected`

说明当前 session 还没有成功连接到 target。通常是 CAN 未 up、ID/型号错误、或者上一条 `set_target` 切到了错误设备。

处理：

```json
{"op":"verify","vendor":"damiao","motor_id":"0x01","feedback_id":"0x11","timeout_ms":1000}
```

### `unsupported op`

说明 `op` 名字拼错，或该 op 尚未实现。

### `... is damiao-only`

说明你对非 Damiao 目标调用了 Damiao 寄存器接口。

RobStride 应改用：

```json
{"op":"robstride_read_param","param_id":"0x7019","type":"f32","timeout_ms":500}
```

### RobStride ping/read timeout

检查：

- `motor_id` 是否是 device_id。
- `feedback_id` 是否是 host_id，常用 `0xFD`。
- CAN 通道是否已经 up。
- 是否需要先 `scan` 尝试 `0xFD,0xFF,0xFE,0x00,0xAA`。

### 浏览器无法带 token

原生 `WebSocket` 不能设置自定义握手 header。远程访问时建议：

- 用本地 `127.0.0.1` 访问；或
- 使用受控反向代理做鉴权；或
- 使用非浏览器 WS 客户端传 header。
