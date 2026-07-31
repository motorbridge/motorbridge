# ws_gateway

<!-- channel-compat-note -->
## Channel Compatibility (PCAN + CANable candleLight/gs_usb + Damiao Serial Bridge + DM_Device)

- Linux SocketCAN uses prepared interfaces directly: `can0`, `can1`. For CANable, use candleLight/gs_usb firmware so it appears as a SocketCAN interface such as `can0`.
- Use PCAN or CANable candleLight/gs_usb for standard CAN.
- Damiao-only adapter transports are available in CLI: serial bridge (`--transport dm-serial --serial-port /dev/ttyACM0 --serial-baud 921600`) and DM_Device SDK (`--transport dm-device --dm-device-type usb2canfd|usb2canfd-dual|linkx4c --dm-channel 0|1|2|3`; Damiao motors only; adapter must be in USB mode).
- Damiao-only DM_Device SDK transport is available through
  `--transport dm-device --dm-device-type usb2canfd|usb2canfd-dual|linkx4c`.
  The adapter must be in USB mode. Linux x86_64 USB2CANFD_DUAL channel 0/1
  and LINKX4C channel `0..3` scans are verified.
- Full Damiao serial-bridge interface list and command patterns are documented in `motor_cli/README.md` (section `3.6` in `motor_cli/README.zh-CN.md`).
- On Linux SocketCAN, do not append bitrate in `--channel` (for example `can0@1000000` is invalid).
- On Windows (PCAN backend), `can0/can1` map to `PCAN_USBBUS1/2`; optional `@bitrate` suffix is supported.


High-performance Rust WebSocket gateway (V1: JSON over WS).

```mermaid
sequenceDiagram
  participant Client as WS Client
  participant GW as ws_gateway
  participant ABI as motor_abi layer
  participant HW as Motor/CAN
  Client->>GW: JSON command (enable/mit/pos-vel/...)
  GW->>ABI: execute control op
  ABI->>HW: send frame
  HW-->>ABI: feedback
  ABI-->>GW: parsed state
  GW-->>Client: state/event JSON
```

## Status

Core WS API is implemented.
The bundled web HMI (`tools/ws_test_client.html`) is still under active development.

## Transport

- Protocol: WebSocket
- V1 payload: JSON text frames
- Periodic state push on each `--dt-ms` tick

## Unified Mode Mapping (Draft)

Goal: application layer uses one unified operation set first; vendor-specific ops remain available but are not recommended as default.

### Unified Control Modes (app-facing, fixed baseline)

| Unified Mode | Unified Op | Core Parameters |
| --- | --- | --- |
| `mit` | `{"op":"mit", ...}` | `pos`, `vel`, `kp`, `kd`, `tau` |
| `pos_vel` | `{"op":"pos_vel", ...}` | `pos`, `vlim` |
| `vel` | `{"op":"vel", ...}` | `vel` |
| `force_pos` | `{"op":"force_pos", ...}` | `pos`, `vlim`, `ratio` |

If a vendor does not support one of these four baseline modes, gateway returns `unsupported`.
RobStride additionally exposes `pos_vel_pp` (`pos`, `vel_max`, `acc_set`) and
`pos_vel_csp` (`pos`, `limit_spd`); hyphenated operation aliases are accepted.

### Vendor Mapping Table (unified mode -> vendor-native)

| Vendor | `mit` | `pos_vel` | `vel` | `force_pos` | Parameter Differences | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| damiao | native MIT | native POS_VEL | native VEL | native FORCE_POS | full parameter match | baseline reference |
| robstride | native MIT | legacy alias for PP (`run_mode=1` + `vel_max` + optional `acc_set` + `loc_ref`) | native Velocity mode | unsupported | explicit `pos_vel_pp` uses `vel_max`/`acc_set`; explicit `pos_vel_csp` uses `limit_spd` | native param read/write via `robstride_*` |
| hexfellow | native MIT | native POS_VEL | unsupported | unsupported | `mit` supports `kp/kd/tau`; no standalone `vel` | CAN-FD path |
| myactuator | unsupported | Position setpoint flow | native velocity setpoint | unsupported | `pos_vel` via position setpoint; `vel` in baseline set | native strengths: current/position/version/mode-query |
| hightorque | native MIT (ht_can mapping) | maps to native pos+vel+tqe | native velocity frame | maps to native pos+vel+tqe | `mit/vel` are raw-frame mapped; `kp/kd` accepted but ignored by protocol; `pos_vel/force_pos` map to pos+vel+tqe | current subset: scan/read/mit/vel/pos-vel/force-pos/stop; `enable/disable` accepted as no-op |

### Unified Core Ops Support Matrix

| Vendor | `scan` | `set_id` | `enable` | `disable` | `stop` | `state_once/status` |
| --- | --- | --- | --- | --- | --- | --- |
| damiao | supported | supported | supported | supported | supported | supported |
| robstride | supported | supported | supported | supported | supported | supported |
| hexfellow | supported | unsupported | supported | supported | supported | supported |
| myactuator | supported | unsupported | supported | supported | supported | supported |
| hightorque | supported | unsupported | accepted (no-op) | accepted (no-op) | supported | supported |

### Parameter Notes by Mode

- `mit`: same unified fields, but vendor scaling differs internally (gateway adapter handles conversion).
  HighTorque detail: `kp/kd` are currently ignored by protocol path.
- `pos_vel`: only valid where vendor has equivalent mode. For RobStride it is a
  deprecated PP alias; use `pos_vel_pp` or `pos_vel_csp` to make the native
  mode and parameter semantics explicit.
- `vel`: sign/scale conversion is vendor-specific internally.
- `force_pos`: Damiao native; HighTorque maps to pos+vel+tqe; others unsupported.

## WS `capabilities` Response (Draft)

Recommended: client calls `{"op":"capabilities"}` on connect and adapts UI/flows by returned support matrix.

### Example response

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

## Build

```bash
cargo build -p ws_gateway --release
```

## Run

```bash
cargo run -p ws_gateway --release -- \
  --bind 127.0.0.1:9002 --vendor damiao --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 --dt-ms 20
```

Damiao over DM_Device SDK / USB2CANFD_DUAL:

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

Damiao over DM_Device SDK / LINKX4C channel 0:

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

Security note:

- `127.0.0.1:9002` is the default and recommended bind for local use.
- If you bind to non-loopback addresses (for example `0.0.0.0:9002`), you must set `MOTORBRIDGE_WS_TOKEN`.
- macOS/Linux example: `export MOTORBRIDGE_WS_TOKEN=your-token` or `MOTORBRIDGE_WS_TOKEN=your-token motorbridge-gateway -- --bind 0.0.0.0:9002`
- PowerShell example: `$env:MOTORBRIDGE_WS_TOKEN="your-token"` and then start `motorbridge-gateway -- --bind 0.0.0.0:9002`
- WebSocket clients may provide this token via header `x-motorbridge-token: <token>`, `Authorization: Bearer <token>`, or query `?motorbridge_ws_token=<token>` for browser clients.

## Damiao `dm-device` Scan Example

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

Notes:

- `dm_channel=0` maps to SDK channel 0; `dm_channel=1` maps to SDK channel 1.
- `dm_device_type=usb2canfd` has one channel (`0` / SDK channel 0).
- For `dm_device_type=linkx4c`, use SDK channels `0`, `1`, `2`, or `3`.
- Omit `dm_channel` in a scan request to scan all channels for the selected
  adapter: `0` on `usb2canfd`, `0|1` on `usb2canfd-dual`, or `0|1|2|3` on
  `linkx4c`.
- The gateway keeps the DM_Device SDK handle open and reuses it across scans in
  the same process, avoiding the SDK/libusb reopen issue observed on Linux.
- Do not open the same DM_Device USB adapter from two separate processes at the same time.

## Experimental Windows Support (PCAN-USB)

Linux remains the primary target. Windows support is experimental and currently uses PEAK PCAN.

- Install PEAK PCAN driver + PCAN-Basic runtime (`PCANBasic.dll`).
- Use `can0@1000000` as the channel value on Windows:

```bash
cargo run -p ws_gateway --release -- --bind 127.0.0.1:9002 --vendor damiao --channel can0@1000000 --model 4340P --motor-id 0x01 --feedback-id 0x11 --dt-ms 20
```

Quick Windows motor validation commands:

```bash
cargo run -p motor_cli --release -- --vendor damiao --channel can0@1000000 --model 4340P --motor-id 0x01 --feedback-id 0x11 --mode scan --start-id 1 --end-id 16
cargo run -p motor_cli --release -- --vendor damiao --channel can0@1000000 --model 4340P --motor-id 0x01 --feedback-id 0x11 --mode pos-vel --pos 3.1416 --vlim 2.0 --loop 1 --dt-ms 20
cargo run -p motor_cli --release -- --vendor damiao --channel can0@1000000 --model 4310 --motor-id 0x07 --feedback-id 0x17 --mode pos-vel --pos 3.1416 --vlim 2.0 --loop 1 --dt-ms 20
```

## Inbound command examples

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
{"op":"robstride_param_stream","enabled":true,"profile":"full","interval_ms":3000,"timeout_ms":80}
{"op":"robstride_param_stream","enabled":true,"params":["0x7019","0x701A","0x701B","0x302C"],"interval_ms":500}
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

For Damiao, `ensure_mode` and control ops verify `RID 10` (`CTRL_MODE`). A pure
verify-read timeout is returned as `ok:true` with `data.warning`/`data.warnings`
because the mode write may already have succeeded; an explicit mismatched mode
value remains `ok:false`.

## Damiao `dm-serial` Arm Telemetry

`v0.4.1` adds scan-safe Damiao session handling for Windows serial bridges. If a
scan or batch scan starts while the gateway already has a Damiao session open,
the gateway stops state/parameter streams, releases the active session, waits a
short Windows release gap, and then probes the serial bridge. This avoids serial
port contention during whole-arm scans.

For browser HMIs, prefer `damiao_state_many` after scan results are known. The
request accepts an `items` array with `motor_id`, `feedback_id`, and optional
`model`. Each returned state includes the same identity fields plus
`has_value`; missing/offline joints return `has_value:false` instead of breaking
the whole response.

## Outbound frames

Success response:

```json
{"ok":true,"op":"vel","data":{"op":"vel","continuous":true}}
```

Error response:

```json
{"ok":false,"op":"set_id","error":"..."}
```

State stream frame:

```json
{"type":"state","data":{"has_value":true,"pos":0.12,"vel":0.01,"torq":0.0,"status_code":1}}
```

RobStride parameter stream frame:

```json
{"type":"robstride_params","data":{"vendor":"robstride","motor_id":1,"feedback_id":253,"model":"rs-00","values":{"mechPos":0.12,"iqf":0.3,"mechVel":0.01,"torque_fdb":0.02},"params":[{"param_id":28697,"name":"mechPos","type":"f32","value":0.12,"ok":true}]}}
```

## Notes

- `--vendor damiao|robstride|hexfellow|myactuator|hightorque` controls default target vendor.
- `set_target` can switch vendor/transport/channel/serial/model/id on the fly per session.
- `continuous=true` keeps sending that control command every tick.
- `stop` clears continuous control.
- `set_id` is vendor-aware:
  - Damiao: write `MST_ID` first, then `ESC_ID`.
  - RobStride: device ID update via `SET_DEVICE_ID`.
- Damiao-only ops: `write/get_register_*` and `dm-serial` transport.
- Parameter streams: `param_stream` supports Damiao and RobStride; `damiao_param_stream` and `robstride_param_stream` are vendor-specific aliases.
- RobStride-only ops: `robstride_ping`, `robstride_read_param`, `robstride_write_param`, `set_active_report`.
- MyActuator-native ops: `current`, `pos`, `version`, `mode-query`.
- HighTorque-native op: `read`.
- V2 plan can switch to binary frames while preserving operation semantics.

## Simple HMI (for quick testing)

- File: `integrations/ws_gateway/tools/ws_test_client.html`
- Dedicated 4-motor sync example: `examples/web/ws_quad_sync_hmi.html`
- Open directly in browser (double-click or `xdg-open`), then connect to `ws://127.0.0.1:9002`.
- Current status: **in development** (UI/flow may change quickly).
- For stable validation, prefer sending JSON commands directly (wscat/websocat/custom client).
- Built-in dynamic device workflow:
  - scan Damiao and RobStride in one page
  - scan hits are shown in a device table (vendor + motor_id + feedback_id + model)
  - pick any scanned motor as current target and run enable/disable/vel/mit
  - checkbox batch ops: batch enable/stop/disable and batch MIT sync-to-position
- For the 4-motor same-angle slider workflow, start a local static server:
  - `python3 -m http.server 18080`
  - open `http://127.0.0.1:18080/examples/web/ws_quad_sync_hmi.html`
