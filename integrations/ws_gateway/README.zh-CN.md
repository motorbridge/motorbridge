# ws_gateway

<!-- channel-compat-note -->
## 通道兼容说明（PCAN + CANable candleLight/gs_usb + Damiao 串口桥 + DM_Device）

- Linux SocketCAN 直接使用已初始化的接口名：`can0`、`can1`。CANable 请刷 candleLight/gs_usb 固件，让系统识别为 `can0` 这类 SocketCAN 接口。
- 标准 CAN 推荐 PCAN 或 CANable candleLight/gs_usb。
- 仅 Damiao 可选两类适配器链路：串口桥 `--transport dm-serial --serial-port /dev/ttyACM0 --serial-baud 921600`，以及 DM_Device SDK `--transport dm-device --dm-device-type usb2canfd|usb2canfd-dual|linkx4c --dm-channel 0|1|2|3`。DM_Device 链路当前只配 Damiao 电机协议使用，适配器需处于 USB 模式。
- 仅 Damiao 可选 DM_Device SDK 链路：
  `--transport dm-device --dm-device-type usb2canfd|usb2canfd-dual|linkx4c`。
  适配器需要处于 USB 模式。Linux x86_64 下 USB2CANFD_DUAL 的通道 0/1
  和 LINKX4C 通道 `0..3` 扫描已实测通过。
- Damiao 串口桥完整接口与命令模板见 `motor_cli/README.zh-CN.md` 第 `3.6` 节（英文见 `motor_cli/README.md`）。
- Linux SocketCAN 下 `--channel` 不要带 `@bitrate`（例如 `can0@1000000` 无效）。
- Windows（PCAN 后端）中，`can0/can1` 映射 `PCAN_USBBUS1/2`，可选 `@bitrate` 后缀。


高性能 Rust WebSocket 网关（V1：JSON over WS）。

完整 JSON 协议、每个 `op` 的参数/默认值/返回值/示例见：
[`PROTOCOL.zh-CN.md`](./PROTOCOL.zh-CN.md)。

```mermaid
sequenceDiagram
  participant Client as WS 客户端
  participant GW as ws_gateway
  participant ABI as motor_abi 层
  participant HW as 电机/CAN
  Client->>GW: JSON 指令（enable/mit/pos-vel/...）
  GW->>ABI: 执行控制操作
  ABI->>HW: 下发 CAN 帧
  HW-->>ABI: 反馈状态
  ABI-->>GW: 解析后的状态
  GW-->>Client: 状态/事件 JSON
```

## 状态

WS API 主链路已实现。
内置网页上位机（`tools/ws_test_client.html`）仍在持续开发中。

## 传输

- 协议：WebSocket
- V1 载荷：JSON 文本帧
- 按 `--dt-ms` 周期推送状态

## 统一模式映射（草案）

目标：应用层优先使用统一操作集；厂商专属操作保留可用，但不作为默认推荐路径。

### 统一控制模式（应用层，固定基线）

| 统一模式 | 统一操作 | 核心参数 |
| --- | --- | --- |
| `mit` | `{"op":"mit", ...}` | `pos`, `vel`, `kp`, `kd`, `tau` |
| `pos_vel` | `{"op":"pos_vel", ...}` | `pos`, `vlim` |
| `vel` | `{"op":"vel", ...}` | `vel` |
| `force_pos` | `{"op":"force_pos", ...}` | `pos`, `vlim`, `ratio` |

若某厂商不支持这四种基线模式，网关统一返回 `unsupported`。
RobStride 另外显式提供 `pos_vel_pp`（`pos`、`vel_max`、`acc_set`）和
`pos_vel_csp`（`pos`、`limit_spd`），同时接受连字符形式的 op 别名。

### 厂商映射表（统一模式 -> 厂商原生）

| 厂商 | `mit` | `pos_vel` | `vel` | `force_pos` | 参数差异 | 备注 |
| --- | --- | --- | --- | --- | --- | --- |
| damiao | 原生 MIT | 原生 POS_VEL | 原生 VEL | 原生 FORCE_POS | 参数完整对齐 | 基线参考实现 |
| robstride | 原生 MIT | PP 兼容别名（`run_mode=1` + `vel_max` + 可选 `acc_set` + `loc_ref`） | 原生 Velocity 模式 | 不支持 | 显式 `pos_vel_pp` 使用 `vel_max`/`acc_set`；显式 `pos_vel_csp` 使用 `limit_spd` | 参数读写走 `robstride_*` |
| hexfellow | 原生 MIT | 原生 POS_VEL | 不支持 | 不支持 | `mit` 支持 `kp/kd/tau`，无独立 `vel` | CAN-FD 链路 |
| myactuator | 不支持 | Position 设定流程 | 原生速度设定 | 不支持 | `pos_vel` 通过 position setpoint 实现；基线里 `vel` 可用 | 强项是 current/position/version/mode-query |
| hightorque | 原生 MIT（ht_can 映射） | 映射到原生 pos+vel+tqe | 原生速度帧 | 映射到原生 pos+vel+tqe | `mit/vel` 为原生帧映射；`kp/kd` 保留但协议侧忽略；`pos_vel/force_pos` 映射到 pos+vel+tqe | 当前子集 scan/read/mit/vel/pos-vel/force-pos/stop；`enable/disable` 接受但为 no-op |

### 统一核心操作支持矩阵

| 厂商 | `scan` | `set_id` | `enable` | `disable` | `stop` | `state_once/status` |
| --- | --- | --- | --- | --- | --- | --- |
| damiao | 支持 | 支持 | 支持 | 支持 | 支持 | 支持 |
| robstride | 支持 | 支持 | 支持 | 支持 | 支持 | 支持 |
| hexfellow | 支持 | 不支持 | 支持 | 支持 | 支持 | 支持 |
| myactuator | 支持 | 不支持 | 支持 | 支持 | 支持 | 支持 |
| hightorque | 支持 | 不支持 | 接受（no-op） | 接受（no-op） | 支持 | 支持 |

### 模式参数差异说明

- `mit`：统一字段一致，但各厂商内部缩放/编码不同，由网关适配层处理。
  HighTorque 细节：当前协议路径会忽略 `kp/kd`。
- `pos_vel`：仅对具备等价模式的厂商可用。对 RobStride，它是已弃用的 PP
  兼容别名；应使用 `pos_vel_pp` 或 `pos_vel_csp` 明确原生模式和参数语义。
- `vel`：方向与量纲转换由厂商适配层内部处理。
- `force_pos`：Damiao 原生支持；HighTorque 映射到 pos+vel+tqe；其他厂商不支持。

## WS `capabilities` 响应结构（草案）

建议：客户端连接后先调用 `{"op":"capabilities"}`，根据返回能力矩阵自动适配 UI 与流程。

### 响应示例

```json
{
  "ok": true,
  "op": "capabilities",
  "data": {
    "api_version": "v1",
    "default_vendor": "damiao",
    "vendors": {
      "damiao": {
        "transports": ["auto", "socketcan", "socketcanfd", "dm-serial", "dm-device"],
        "modes": ["mit", "pos_vel", "vel", "force_pos"],
        "ops_unified": ["scan", "set_id", "enable", "disable", "stop", "state_once", "status", "verify"],
        "ops_vendor_native": ["write_register_u32", "write_register_f32", "get_register_u32", "get_register_f32", "damiao_state_many"]
      },
      "robstride": {
        "transports": ["auto", "socketcan", "socketcanfd"],
        "modes": ["mit", "pos_vel", "pos_vel_pp", "pos_vel_csp", "vel"],
        "ops_unified": ["scan", "set_id", "enable", "disable", "stop", "state_once", "status", "verify"],
        "ops_vendor_native": ["robstride_ping", "robstride_read_param", "robstride_write_param"]
      },
      "hexfellow": {
        "transports": ["auto", "socketcanfd"],
        "modes": ["mit", "pos_vel"],
        "ops_unified": ["scan", "enable", "disable", "stop", "state_once", "status", "verify"],
        "ops_vendor_native": []
      },
      "myactuator": {
        "transports": ["auto", "socketcan", "socketcanfd"],
        "modes": ["pos_vel", "vel"],
        "ops_unified": ["scan", "enable", "disable", "stop", "state_once", "status", "verify"],
        "ops_vendor_native": ["status", "version", "mode-query"]
      },
      "hightorque": {
        "transports": ["auto", "socketcan"],
        "modes": ["mit", "pos_vel", "vel", "force_pos"],
        "ops_unified": ["scan", "stop", "state_once", "status", "verify"],
        "ops_vendor_native": ["read"]
      }
    },
    "unsupported_behavior": "return {ok:false,error:'unsupported ...'}"
  }
}
```

## 构建

```bash
cargo build -p ws_gateway --release
```

## 运行

```bash
cargo run -p ws_gateway --release -- \
  --bind 127.0.0.1:9002 --vendor damiao --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 --dt-ms 20
```

Damiao 通过 DM_Device SDK / USB2CANFD_DUAL：

```bash
cargo run -p ws_gateway --release -- \
  --bind 127.0.0.1:9002 \
  --vendor damiao \
  --transport dm-device \
  --dm-device-type usb2canfd-dual \
  --dm-channel 1 \
  --model 4310 \
  --motor-id 0x04 \
  --feedback-id 0x14 \
  --dt-ms 20
```

Damiao 通过 DM_Device SDK / LINKX4C 通道 0：

```bash
cargo run -p ws_gateway --release -- \
  --bind 127.0.0.1:9002 \
  --vendor damiao \
  --transport dm-device \
  --dm-device-type linkx4c \
  --dm-channel 0 \
  --model 4310 \
  --motor-id 0x04 \
  --feedback-id 0x14 \
  --dt-ms 20
```

```bash
cargo run -p ws_gateway --release -- \
  --bind 127.0.0.1:9002 --vendor robstride --channel can0 --model rs-06 --motor-id 127 --feedback-id 0xFD --dt-ms 20
```

安全说明：

- 默认推荐使用 `127.0.0.1:9002`（本机回环）。
- 若绑定到非回环地址（例如 `0.0.0.0:9002`），必须设置环境变量 `MOTORBRIDGE_WS_TOKEN`。
- macOS/Linux 示例：`export MOTORBRIDGE_WS_TOKEN=your-token`，或 `MOTORBRIDGE_WS_TOKEN=your-token motorbridge-gateway -- --bind 0.0.0.0:9002`
- PowerShell 示例：`$env:MOTORBRIDGE_WS_TOKEN="your-token"`，然后再启动 `motorbridge-gateway -- --bind 0.0.0.0:9002`
- WS 客户端可在握手请求中带上 `x-motorbridge-token: <token>`、`Authorization: Bearer <token>`，浏览器客户端也可使用 query `?motorbridge_ws_token=<token>`。

## Damiao `dm-device` 扫描示例

```json
{
  "op": "scan",
  "vendor": "damiao",
  "transport": "dm-device",
  "dm_device_type": "usb2canfd-dual",
  "model": "4310",
  "start_id": 1,
  "end_id": 16,
  "feedback_base": 16,
  "timeout_ms": 80
}
```

说明：

- `dm_channel=0` 对应 SDK channel 0；`dm_channel=1` 对应 SDK channel 1。
- `dm_device_type=usb2canfd` 只有一路（`0` / SDK channel 0）。
- `dm_device_type=linkx4c` 使用 SDK 通道 `0`、`1`、`2`、`3`。
- scan 请求不带 `dm_channel` 时会扫描所选适配器全部通道：`usb2canfd` 为
  `0`，`usb2canfd-dual` 为 `0|1`，`linkx4c` 为 `0|1|2|3`；带
  `dm_channel` 时只扫描指定物理通道。
- 网关会在同一进程内复用已打开的 DM_Device SDK handle，避免 Linux 下 SDK/libusb
  反复 reopen 造成的打开失败。
- 同一个 DM_Device USB 适配器不要被两个独立进程同时打开。

## Windows 实验支持（PCAN-USB）

项目主线仍以 Linux 为主。Windows 支持为实验性能力，当前通过 PEAK PCAN 后端实现。

- 安装 PEAK 驱动与 PCAN-Basic 运行时（`PCANBasic.dll`）。
- Windows 启动网关时可使用 `can0@1000000`：

```bash
cargo run -p ws_gateway --release -- --bind 127.0.0.1:9002 --vendor damiao --channel can0@1000000 --model 4340P --motor-id 0x01 --feedback-id 0x11 --dt-ms 20
```

Windows 电机验证命令：

```bash
cargo run -p motor_cli --release -- --vendor damiao --channel can0@1000000 --model 4340P --motor-id 0x01 --feedback-id 0x11 --mode scan --start-id 1 --end-id 16
cargo run -p motor_cli --release -- --vendor damiao --channel can0@1000000 --model 4340P --motor-id 0x01 --feedback-id 0x11 --mode pos-vel --pos 3.1416 --vlim 2.0 --loop 1 --dt-ms 20
cargo run -p motor_cli --release -- --vendor damiao --channel can0@1000000 --model 4310 --motor-id 0x07 --feedback-id 0x17 --mode pos-vel --pos 3.1416 --vlim 2.0 --loop 1 --dt-ms 20
```

## 入站命令示例

```json
{"op":"ping"}
{"op":"enable"}
{"op":"disable"}
{"op":"set_target","vendor":"robstride","channel":"can0","model":"rs-06","motor_id":127,"feedback_id":255}
{"op":"mit","pos":0.0,"vel":0.0,"kp":20.0,"kd":1.0,"tau":0.0,"continuous":true}
{"op":"pos_vel","pos":3.1,"vlim":1.5,"continuous":true}
{"op":"pos_vel_pp","pos":0.5,"vel_max":0.02,"acc_set":0.05}
{"op":"pos_vel_csp","pos":0.5,"limit_spd":0.02,"continuous":true}
{"op":"vel","vel":0.5,"continuous":true}
{"op":"force_pos","pos":0.8,"vlim":2.0,"ratio":0.3,"continuous":true}
{"op":"stop"}
{"op":"state_once"}
{"op":"state_stream","enabled":true}
{"op":"damiao_state_many","items":[{"motor_id":1,"feedback_id":17,"model":"4340P"},{"motor_id":2,"feedback_id":18,"model":"4340P"}],"timeout_ms":120}
{"op":"clear_error"}
{"op":"set_zero_position"}
{"op":"ensure_mode","mode":"mit","timeout_ms":1000}
{"op":"request_feedback"}
{"op":"set_active_report","enabled":true}
{"op":"param_stream","enabled":true,"profile":"realtime","interval_ms":1000,"timeout_ms":80}
{"op":"damiao_param_stream","enabled":true,"profile":"realtime","interval_ms":1000,"timeout_ms":80}
{"op":"robstride_param_stream","enabled":true,"profile":"realtime","interval_ms":1000,"timeout_ms":80}
{"op":"store_parameters"}
{"op":"set_can_timeout_ms","timeout_ms":1000}
{"op":"write_register_u32","rid":10,"value":1,"verify":true}
{"op":"write_register_f32","rid":31,"value":5.0,"verify":true}
{"op":"get_register_u32","rid":7,"timeout_ms":1000}
{"op":"get_register_f32","rid":21,"timeout_ms":1000}
{"op":"robstride_ping","timeout_ms":200}
{"op":"robstride_read_param","param_id":28697,"type":"f32","timeout_ms":200}
{"op":"robstride_write_param","param_id":28682,"type":"f32","value":0.3,"verify":true}
{"op":"poll_feedback_once"}
{"op":"shutdown"}
{"op":"close_bus"}
{"op":"scan","start_id":1,"end_id":16,"feedback_base":16,"timeout_ms":100}
{"op":"scan","vendor":"robstride","start_id":120,"end_id":135,"feedback_ids":"0xFD,0xFF,0xFE,0x00,0xAA","timeout_ms":120}
{"op":"set_id","vendor":"damiao","old_motor_id":2,"old_feedback_id":18,"new_motor_id":5,"new_feedback_id":21,"store":true,"verify":true}
{"op":"set_id","vendor":"robstride","old_motor_id":127,"new_motor_id":126,"feedback_id":255,"verify":true}
{"op":"verify","motor_id":5,"feedback_id":21,"timeout_ms":1000}
{"op":"verify","vendor":"robstride","motor_id":127,"feedback_id":255,"timeout_ms":500}
```

Damiao 的 `ensure_mode` 和控制类 op 会回读 `RID 10`（`CTRL_MODE`）确认模式。
如果只是确认回读超时，WS 会返回 `ok:true`，并在 `data.warning` /
`data.warnings` 里说明“确认超时但命令已继续”；如果读到明确不匹配的模式值，
仍返回 `ok:false`。

## Damiao dm-serial arm telemetry

`v0.4.1` adds scan-safe Damiao session handling for Windows serial bridges. When
scan or batch_scan starts, the gateway stops state/parameter streams, releases
the active Damiao session, waits for a short Windows release gap, then probes
the serial bridge. This avoids serial-port contention during whole-arm scans.

Browser HMIs should use `damiao_state_many` after scan results are known. The
request accepts `items` with `motor_id`, `feedback_id`, and optional `model`.
Each returned state includes the same identity fields and `has_value`; missing
or offline joints return `has_value:false` instead of failing the whole request.

## 出站帧

成功响应：

```json
{"ok":true,"op":"vel","data":{"op":"vel","continuous":true}}
```

失败响应：

```json
{"ok":false,"op":"set_id","error":"..."}
```

状态流：

```json
{"type":"state","data":{"has_value":true,"pos":0.12,"vel":0.01,"torq":0.0,"status_code":1}}
```

参数流：

```json
{"type":"robstride_params","data":{"vendor":"robstride","motor_id":1,"feedback_id":253,"values":{"mechPos":0.12,"iqf":0.3,"mechVel":0.01,"torque_fdb":0.02}}}
```

## 说明

- `--vendor damiao|robstride|hexfellow|myactuator|hightorque` 用于设置会话默认厂商。
- `set_target` 可在单个会话中动态切换厂商/transport/通道/串口/型号/ID。
- `continuous=true` 会在每个 tick 持续发送该控制命令。
- `stop` 用于清除持续控制。
- `set_id` 按厂商处理：
  - Damiao：先写 `MST_ID`，再写 `ESC_ID`。
  - RobStride：使用 `SET_DEVICE_ID` 更新设备 ID。
- Damiao 专属操作：`write/get_register_*` 与 `dm-serial` transport。
- 参数流：`param_stream` 支持 Damiao 与 RobStride；`damiao_param_stream` / `robstride_param_stream` 是厂商专用别名。
- RobStride 专属操作：`robstride_ping`、`robstride_read_param`、`robstride_write_param`、`set_active_report`。
- MyActuator 专属操作：`current`、`pos`、`version`、`mode-query`。
- HighTorque 专属操作：`read`。
- 后续 V2 可升级为二进制帧，同时保留同一语义。

## 简易上位机（快速联调）

- 文件：`integrations/ws_gateway/tools/ws_test_client.html`
- 四电机同步专用示例：`examples/web/ws_quad_sync_hmi.html`
- 直接浏览器打开（双击或 `xdg-open`），连接 `ws://127.0.0.1:9002`。
- 当前状态：**开发中**（界面与交互会持续调整）。
- 若要稳定联调，建议优先使用 JSON 直连客户端（wscat/websocat/自定义客户端）。
- 动态设备工作流：
  - 同一页面扫描 Damiao 与 RobStride
  - 扫描结果进设备表（vendor + motor_id + feedback_id + model）
  - 可选择任意扫描到的设备作为当前目标，执行使能/失能/速度/MIT
  - 支持勾选批量操作：批量使能/停转/失能、批量 MIT 同步到角度
- 四电机同角度拖杆控制建议用本地静态服务打开：
  - `python3 -m http.server 18080`
  - 浏览器访问 `http://127.0.0.1:18080/examples/web/ws_quad_sync_hmi.html`
