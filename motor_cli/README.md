# motor_cli (English)

Complete parameter reference for the Rust `motor_cli` binary.

- Crate: `motor_cli`
- Recommended (release package): `./bin/motor_cli [ARGS...]`
- Optional (source build): `./target/release/motor_cli [ARGS...]`

## Release-first Usage

Download and extract the release package (GitHub Releases asset like `motor-cli-vX.Y.Z-linux-x86_64.tar.gz`), then run directly:

```bash
./bin/motor_cli -h
./bin/motor_cli --vendor damiao --mode scan --start-id 1 --end-id 16
```

If you want `motor_cli` as a plain command:

```bash
export PATH="$(pwd)/bin:$PATH"
motor_cli -h
```

## Additional Damiao Command/Register Reference

- Detailed Damiao command + register tuning doc (English): `DAMIAO_API.md`
- Chinese version (command/register reference): `DAMIAO_API.zh-CN.md`

## Additional RobStride Command/Parameter Reference

- Detailed RobStride command + parameter guide (English): `ROBSTRIDE_API.md`
- Chinese version (parameter/capability reference): `ROBSTRIDE_API.zh-CN.md`

## Additional MyActuator Command/Mode Reference

- Detailed MyActuator command + mode guide (English): `MYACTUATOR_API.md`
- Chinese version (command/mode reference): `MYACTUATOR_API.zh-CN.md`

## HighTorque Notes

- Protocol analysis (Chinese): `../docs/zh/hightorque_protocol_analysis.md`
- Current `vendor=hightorque` is a native ht_can v1.5.5-compat (v2.0.0 migration in progress) direct-CAN mode, not the official serial-CANboard transport.

## CAN Debugging Entry

- Professional PCAN + CANable candleLight/gs_usb troubleshooting: `../docs/en/can_debugging.md`
- Chinese troubleshooting guide: `../docs/zh/can_debugging.md`

## Transport Legend

- `[STD-CAN]` => `--transport auto|socketcan`
- `[CAN-FD]` => `--transport socketcanfd` (Linux-only; required by Hexfellow)
- `[DM-SERIAL]` => `--transport dm-serial` (Damiao-only)
- `[DM-DEVICE]` => `--transport dm-device` (Damiao-only DM_Device SDK path;
  enabled only on targets with a matching SDK runtime in `third_party/dm_device`;
  `usb2canfd`, `usb2canfd-dual`, and `linkx4c` are supported when the adapter
  is in USB mode; Linux x86_64 USB2CANFD_DUAL and LINKX4C scans are verified)

Current status:
- Hexfellow: validated on `socketcanfd` with unified `mit` / `pos-vel`.
- HighTorque: validated on standard CAN with unified `mit` / `vel` (`kp/kd` ignored by protocol).
- Damiao: baseline implementation for unified `mit` / `pos-vel` / `vel` / `force-pos`;
  `dm-device` scan verified on USB2CANFD_DUAL channel 0/1 and LINKX4C SDK
  channel `0..3`.

## Validated Capability Matrix (Damiao + RobStride, 2026-04)

| Capability | Damiao | RobStride |
|---|---|---|
| Scan | Yes | Yes |
| Ping / online probe | Yes (scan/register path) | Yes (`ping`) |
| Enable / Disable | Yes | Yes |
| MIT (`pos/vel/kp/kd/tau`) | Yes | Yes |
| POS_VEL unified mode | Yes | Yes (mapped to native Position path) |
| VEL unified mode | Yes | Yes |
| Parameter read/write | Yes | Yes |
| Set zero | Yes (disable first) | Yes (experimental sequence; firmware-dependent ack behavior) |
| Set motor ID | Yes (`--set-motor-id`) | Yes (`--set-motor-id`) |
| Set feedback ID | Yes (`--set-feedback-id`) | No (host id is configured by `--feedback-id`) |

Notes:
- RobStride default `--feedback-id` is `0xFD`; scan defaults to `--feedback-ids 0xFD,0xFF,0xFE,0x00,0xAA`.
- RobStride `feedback_id` / `host_id` is not the motor `device_id`; scan reports the motor ID as `probe` / `device_id`.
- RobStride `pos-vel` ignores `--vel/--kd/--tau` by design (warning only, no hard error).

## 1. Argument Parsing Rules

- Only `--key value` style options are parsed.
- A bare mode word, for example `motor_cli scan --vendor robstride ...`, is accepted as shorthand for `--mode scan`.
- A standalone flag (for example `--help`) is treated as value `1`.
- Numeric IDs accept decimal (`20`) and hex (`0x14`).
- Unknown keys are parsed but ignored unless used by code paths.

## 2. Top-Level Arguments (All Vendors)

| Argument | Type | Default | Notes |
|---|---|---|---|
| `--help` | flag | off | Prints CLI help and exits |
| `--vendor` | string | `damiao` | `damiao`, `robstride`, `robstride_cia402`, `robstride_mit`, `hightorque`, `myactuator`, `hexfellow`, `all` |
| `--transport` | string | `auto` | `auto`, `socketcan`, `socketcanfd`, `dm-serial`, `dm-device` (`socketcanfd` is Hexfellow-required path; `dm-serial`/`dm-device` are Damiao-only) |
| `--channel` | string | `can0` | Linux: SocketCAN interface name (`can0`); Windows (PCAN backend): `can0`/`can1` with optional `@bitrate` suffix (for example `can0@1000000`); macOS (PCBUSB backend): `can0`/`can1` |
| `--serial-port` | string | `/dev/ttyACM0` | Used when `--transport dm-serial` |
| `--serial-baud` | u64 | `921600` | Used when `--transport dm-serial` |
| `--dm-device-type` | string | `usb2canfd-dual` | Used when `--transport dm-device`; accepted values: `usb2canfd`, `usb2canfd-dual`, `linkx4c` |
| `--dm-channel` | string | control: `0`; scan: all if omitted | Used when `--transport dm-device`; `usb2canfd` accepts `0`, `usb2canfd-dual` accepts `0`/`1`, and `linkx4c` accepts `0`/`1`/`2`/`3`. In `--mode scan`, omit it to scan all channels for the selected adapter. |
| `--model` | string | vendor dependent | `4340` for Damiao, `rs-00` for RobStride / RobStride CiA402 / RobStride MIT, `hightorque` for HighTorque, `X8` for MyActuator |
| `--motor-id` | u16 (hex/dec) | `0x01` | Motor CAN ID |
| `--feedback-id` | u16 (hex/dec) | vendor dependent | Damiao `0x11`, RobStride `0xFD`, RobStride MIT host id `0xFD`, RobStride CiA402 unused/`0`, HighTorque `0x01`, MyActuator `0x241` (for motor-id `1`) |
| `--mode` | string | vendor dependent | Damiao `mit`, RobStride `ping`, RobStride CiA402 `status`, RobStride MIT `status`, HighTorque `read`, MyActuator `status`, `all` -> `scan` |
| `--loop` | u64 | `1` | Control loop cycles |
| `--dt-ms` | u64 | `20` | Loop interval in ms |
| `--ensure-mode` | `0/1` | `1` | Auto-switch mode before control |

### 2.1 Channel Quick Reference (`--channel`)

- Linux SocketCAN:
  - Use interface names directly: `can0`, `can1`.
  - Configure bitrate at interface setup time (`ip link` or the adapter helper script), not in `--channel`.
  - `can0@1000000` is invalid on Linux SocketCAN.
- Windows PCAN:
  - `can0` maps to `PCAN_USBBUS1`, `can1` maps to `PCAN_USBBUS2`.
  - Optional bitrate suffix is supported: `can0@1000000`.
- macOS PCBUSB (PCAN backend):
  - `can0` maps to `PCAN_USBBUS1`, `can1` maps to `PCAN_USBBUS2`.
  - Install `libPCBUSB.dylib` first (see root `README.md` macOS section).

### 2.2 Damiao Serial-Bridge Quick Reference (`--transport dm-serial`)

- This path is adapter-specific and intended for Damiao motors.
- Typical flags: `--transport dm-serial --serial-port /dev/ttyACM1 --serial-baud 921600`.
- In `dm-serial` mode, `--channel` is ignored by transport creation.

### 2.3 Damiao Dedicated CAN-FD Quick Reference (`--transport socketcanfd`)

- This path is Linux-only and independent from classic SocketCAN transport.
- Hexfellow must use this path (`--vendor hexfellow --transport socketcanfd`).
- Typical flags: `--transport socketcanfd --channel can0`.
- Ensure the interface is in FD mode first (`scripts/canfd_restart.sh can0`).
- Current status: Hexfellow validated; Damiao CAN-FD matrix can be validated per model.

### 2.4 Damiao DM_Device SDK Quick Reference (`--transport dm-device`)

- This path uses DaMiao `libdm_device` through motorbridge's `DmDeviceBus`.
- It is currently a Damiao motor protocol transport. The adapter must be in
  USB mode.
- Typical USB2CANFD_DUAL scan:

```bash
motor_cli \
  --vendor damiao \
  --transport dm-device \
  --dm-device-type usb2canfd-dual \
  --model 4310 \
  --mode scan \
  --start-id 1 \
  --end-id 16
```

- `usb2canfd` has one channel: `0` / SDK channel 0.
- `usb2canfd-dual` has two channels: `0` maps to SDK channel 0;
  `1` maps to SDK channel 1.
- In scan mode, omitting `--dm-channel` scans every physical channel for the
  selected adapter. Add `--dm-channel ...` to scan only one physical channel.
- LinkX4C is supported as `--dm-device-type linkx4c`; SDK channels `0..3`
  map to the four physical ports. In scan mode, omitting `--dm-channel` scans
  all four LinkX4C channels; use `--dm-channel 0`, `1`, `2`, or `3` to control
  or scan one port.
- Build support follows the vendored SDK runtime files under
  `third_party/dm_device/v1.1.0`; unsupported target architectures still build,
  but `--transport dm-device` returns an unsupported-platform error.
- Linux x86_64 USB2CANFD_DUAL channel 0/1 and LinkX4C channel 0..3 scans are
  verified.
- Do not open the same DM_Device USB adapter from two processes at the same time.

## 3. Vendor = `damiao`

### 3.1 Supported Modes

- `scan`
- `enable`
- `disable`
- `mit`
- `pos-vel`
- `vel`
- `force-pos`

### 3.2 Damiao Extra Arguments

| Argument | Type | Default | Used In | Notes |
|---|---|---|---|---|
| `--verify-model` | `0/1` | `1` | non-scan | Verify PMAX/VMAX/TMAX matches `--model` |
| `--verify-timeout-ms` | u64 | `500` | non-scan | Register read timeout for model handshake |
| `--verify-tol` | f32 | `0.2` | non-scan | Model limit tolerance |
| `--start-id` | u16 | `1` | scan | Scan start, must be 1..255 |
| `--end-id` | u16 | `255` | scan | Scan end, must be 1..255 |
| `--set-motor-id` | u16 opt | none | id-set flow | Write ESC_ID (RID 8) |
| `--set-feedback-id` | u16 opt | none | id-set flow | Write MST_ID (RID 7) |
| `--store` | `0/1` | `1` | id-set flow | Persist parameters |
| `--verify-id` | `0/1` | `1` | id-set flow | Re-read RID7/RID8 and verify |

### 3.3 Control Arguments by Mode

| Mode | Arguments | Defaults |
|---|---|---|
| `mit` | `--pos --vel --kp --kd --tau` | `0 0 2 1 0` |
| `pos-vel` | `--pos --vlim` | `0 1.0` |
| `vel` | `--vel` | `0` |
| `force-pos` | `--pos --vlim --ratio` | `0 1.0 0.1` |
| `enable`/`disable` | no extra required | n/a |

### 3.4 Scan Behavior Details

- The scanner is model-agnostic in practice: it internally tries a built-in model-hint list.
- For each candidate ID, it also tries multiple feedback-ID hints: inferred (`id+0x10`), user `--feedback-id`, `0x11`, `0x17`.
- Detection first attempts register reads (RID 21/22/23), then feedback fallback.

### 3.5 Damiao Examples

```bash
# Scan a range
motor_cli \
  --vendor damiao --channel can0 --mode scan --start-id 1 --end-id 16
# [STD-CAN]

# MIT control
motor_cli \
  --vendor damiao --channel can0 --model 4310 --motor-id 0x04 --feedback-id 0x14 \
  --mode mit --pos 1.57 --vel 2.0 --kp 35 --kd 1.2 --tau 0.3 --loop 120 --dt-ms 20
# [STD-CAN]

# MIT control via Damiao serial bridge
motor_cli \
  --vendor damiao --transport dm-serial --serial-port /dev/ttyACM1 --serial-baud 921600 \
  --model 4310 --motor-id 0x04 --feedback-id 0x14 \
  --mode mit --verify-model 0 --ensure-mode 0 \
  --pos 1.0 --vel 0 --kp 2 --kd 1 --tau 0 --loop 80 --dt-ms 20
# [DM-SERIAL]

# Position-velocity control
motor_cli \
  --vendor damiao --channel can0 --model 4310 --motor-id 0x04 --feedback-id 0x14 \
  --mode pos-vel --pos 3.14 --vlim 4.0 --loop 120 --dt-ms 20
# [STD-CAN]

# Update ID and persist
motor_cli \
  --vendor damiao --channel can0 --model 4310 --motor-id 0x01 --feedback-id 0x11 \
  --set-motor-id 0x04 --set-feedback-id 0x14 --store 1 --verify-id 1
```

## 4. Vendor = `robstride`

This path is the RobStride private extended-CAN protocol. Use `robstride_cia402` for RobStride motors that have been switched to CANopen/CiA402 mode, and `robstride_mit` for motors switched to F_CMD=2 MIT protocol.

### 4.0 RobStride Protocol Path Comparison

| Vendor | Protocol | CAN ID / frame | Data length | Better fit | Current status |
|---|---|---|---|---|---|
| `robstride` | private protocol, `F_CMD=0` | 29-bit extended CAN. The extended ID carries `comm_type`, host ID, and motor ID. | CAN 2.0, 8 bytes | Factory-style configuration, parameter read/write, ID changes, diagnostics, private MIT-like motion control | Most mature RobStride path |
| `robstride_cia402` | CANopen/CiA402, `F_CMD=1` | Mostly 11-bit standard CAN: NMT `0x000`, SDO `0x600+node` / `0x580+node`, heartbeat `0x700+node`. Protocol switching is the documented 29-bit extended frame `0xFFF`. | CAN 2.0, 8 bytes | CANopen master integration, standard state machine, object dictionary control | Experimental/incomplete for production: core CLI path exists, but EDS/PDO/SYNC, real-device validation, and `dm-device` transport support are not completed |
| `robstride_mit` | MIT protocol, `F_CMD=2` | 11-bit standard CAN. Control uses `motor_id`; typed commands use `(type << 8) \| motor_id`, for example position `0x100+id`, velocity `0x200+id`, parameter read `0x300+id`. | CAN 2.0, 8 bytes | High-rate joint control, `pos/vel/kp/kd/tau`, direct position/velocity commands | Experimental/incomplete for production: core CLI path exists, but high-rate loop ergonomics, real-device validation, and `dm-device` transport support are not completed |

After switching to the standard-frame paths, `robstride_cia402` and `robstride_mit` can be compatible with the DM Device SDK (`dm-device-sdk/C&C++`) at the CAN-adapter level. This is not wired in the CLI yet: today `--transport dm-device` is mainly for Damiao, so these RobStride standard-frame vendors should not be treated as ready to use through DM_Device SDK. The SDK can send and receive raw CAN 2.0 frames; RobStride protocol encoding still belongs in this repository's vendor backends.

### 4.0.1 RobStride ID Roles

In all three RobStride paths, `--motor-id` means "the motor to control". In the private and MIT protocols, `--feedback-id` is really the host/master ID. It is not another motor ID. CANopen/CiA402 does not use that host-ID reply convention; it uses standard CANopen COB-IDs derived from the node ID.

| Vendor | `--motor-id` means | `--feedback-id` means | Send IDs | Receive matching |
|---|---|---|---|---|
| `robstride` | Private-protocol target motor ID | Host/master ID, default `0xFD` | 29-bit extended ID `(comm_type << 24) | (extra_data << 8) | motor_id`; many commands put host ID in `extra_data` | Extended reply is accepted when decoded `device_id == motor_id` |
| `robstride_cia402` | CANopen node ID, normally `1..127` | Ignored/unused | NMT `0x000`; SDO request `0x600 + node`; protocol switch uses extended `0xFFF` | SDO reply `0x580 + node`; heartbeat `0x700 + node` |
| `robstride_mit` | MIT target motor CAN ID | Host/master feedback ID, default `0xFD` | Basic commands and packed MIT control use standard ID `motor_id`; typed commands use `0x100 + motor_id`, `0x200 + motor_id`, `0x300 + motor_id`, `0x400 + motor_id` | Feedback is accepted when standard ID equals `feedback_id` and `data[0] == motor_id`; parameter replies use typed reply ID |

### 4.1 Supported Modes

- `ping`
- `scan`
- `enable`
- `disable`
- `mit`
- `pos-vel`
- `vel`
- `read-param`
- `write-param`
- `get-protocol`
- `set-protocol`

### 4.2 RobStride Extra Arguments

| Argument | Type | Default | Used In | Notes |
|---|---|---|---|---|
| `--start-id` | u16 | `1` | scan | Scan start, 1..255 |
| `--end-id` | u16 | `255` | scan | Scan end, 1..255 |
| `--feedback-ids` | csv u16 | `0xFD,0xFF,0xFE,0x00,0xAA` | scan | RobStride host_id candidates, 0..255; not motor IDs |
| `--timeout-ms` | u64 | `80` | scan | Ping timeout |
| `--param-timeout-ms` | u64 | `120` | scan | Parameter fallback timeout |
| `--manual-vel` | f32 | `0.2` | scan fallback | Blind pulse velocity |
| `--manual-ms` | u64 | `200` | scan fallback | Pulse duration per ID |
| `--manual-gap-ms` | u64 | `200` | scan fallback | Gap between IDs |
| `--set-motor-id` | u16 opt | none | id-set flow | Set device ID, 1..255 |
| `--store` | `0/1` | `1` | id-set flow | Save parameters |
| `--param-id` | u16 | required for param modes | read/write param | Parameter ID |
| `--param-value` | typed | required for write | write-param | Parsed by parameter metadata |

### 4.3 Control Arguments by Mode

| Mode | Arguments | Defaults |
|---|---|---|
| `mit` | `--pos --vel --kp --kd --tau` | `0 0 8 0.2 0` |
| `pos-vel` | `--pos --vlim [--kp]` | `0 1.0 [none]` |
| `vel` | `--vel` | `0` |
| `enable`/`disable` | no extra required | n/a |

Notes:

- RobStride unified control currently supports `MIT` / `POS_VEL` / `VEL`.
- Torque/current is currently parameter-level only (via `write-param`, for example `iq_ref` and limit registers), not a first-class high-level mode.
- In RobStride `mit`, all five unified inputs are effective: `--pos`, `--vel`, `--kp`, `--kd`, `--tau`.
- RobStride `mit` units follow unified semantics: `pos` in `rad`, `vel` in `rad/s`, `tau` in `Nm` (`kp/kd` are MIT loop gains).
- In RobStride `pos-vel`, only `--pos`, `--vlim`, and optional `--kp`/`--loc-kp` are consumed; use `--loop 1` for a single position-target write.
- In RobStride `pos-vel`, `--vel`, `--kd`, and `--tau` are ignored (CLI prints a warning if provided).

### 4.4 Scan Behavior Details

- Fast pass: ping + query-parameter probe per ID.
- If no hits in full range: fallback to blind velocity pulses for manual movement observation.
- Fallback hit criteria includes state feedback presence.

### 4.5 RobStride Examples

```bash
# Ping
motor_cli \
  --vendor robstride --channel can0 --model rs-06 --motor-id 20 --feedback-id 0xFD --mode ping

# Scan
motor_cli \
  --vendor robstride --channel can0 --model rs-06 --mode scan --start-id 1 --end-id 255

# MIT control
motor_cli \
  --vendor robstride --channel can0 --model rs-06 --motor-id 20 --feedback-id 0xFD \
  --mode mit --ensure-strict 1 --pos 0.5 --vel 0 --kp 20.0 --kd 0.5 --tau 0 --loop 100 --dt-ms 20

# POS_VEL (mapped to native Position)
motor_cli \
  --vendor robstride --channel can0 --model rs-06 --motor-id 20 --feedback-id 0xFD \
  --mode pos-vel --pos 1.5 --vlim 1.0 --loc-kp 5.0 --loop 1 --dt-ms 20

# Velocity mode
motor_cli \
  --vendor robstride --channel can0 --model rs-06 --motor-id 20 --feedback-id 0xFD \
  --mode vel --vel 2.0 --loop 100 --dt-ms 20

# Read parameter
motor_cli \
  --vendor robstride --channel can0 --model rs-06 --motor-id 20 --feedback-id 0xFD \
  --mode read-param --param-id 0x7005

# Write parameter
motor_cli \
  --vendor robstride --channel can0 --model rs-06 --motor-id 20 --feedback-id 0xFD \
  --mode write-param --param-id 0x7005 --param-value 2

# Query / switch protocol flag in private protocol
motor_cli \
  --vendor robstride --channel can0 --model rs-00 --motor-id 1 --feedback-id 0xFD \
  --mode get-protocol

# Query current private-protocol control mode run_mode
motor_cli \
  --vendor robstride --channel can0 --model rs-00 --motor-id 1 --feedback-id 0xFD \
  --mode get-mode

motor_cli \
  --vendor robstride --channel can0 --model rs-00 --motor-id 1 --feedback-id 0xFD \
  --mode set-protocol --protocol mit

# Set motor ID (old 1 -> new 11) and persist
motor_cli \
  --vendor robstride --channel can0 --model rs-00 --motor-id 1 --feedback-id 0xFD \
  --set-motor-id 11 --store 1

# Python CLI equivalent for RobStride ID update
motorbridge-cli id-set \
  --vendor robstride --channel can0 --model rs-00 \
  --motor-id 1 --feedback-id 0xFD --new-motor-id 11 --store 1 --verify 1

# Zero (experimental sequence)
motor_cli \
  --vendor robstride --channel can0 --model rs-00 --motor-id 11 --feedback-id 0xFD \
  --mode zero --zero-exp 1 --store 1
```

## 5. Vendor = `robstride_cia402`

This path is RobStride CANopen/CiA402 over classic CAN. It keeps the same CLI surface as the other vendors, but internally uses CiA402 objects such as `6040` controlword, `6060` mode, `607A` target position, `60FF` target velocity, and `6071` target torque.

Status: experimental/incomplete. The command surface is present, but EDS/PDO/SYNC coverage, real-device validation, and `dm-device` transport support are not complete yet.

### 5.1 Supported Modes

- `scan`
- `status`
- `enable`
- `disable`
- `quick-stop`
- `clear-error`
- `zero`
- `watchdog`
- `set-protocol`
- `pos-vel` (CiA402 Profile Position, mode `1`)
- `vel` (CiA402 velocity mode, mode `3`)
- `torque` (CiA402 torque mode, mode `4`)
- `mit` (mapped to CSP, mode `5`; `kp`/`kd` are ignored)

### 5.2 RobStride CiA402 Examples

```bash
# Scan CANopen node IDs
motor_cli \
  --vendor robstride_cia402 --channel can0 --model rs-00 --mode scan --start-id 1 --end-id 127

# Read CiA402 status and feedback objects
motor_cli \
  --vendor robstride_cia402 --channel can0 --model rs-00 --motor-id 1 --mode status

# Enable drive
motor_cli \
  --vendor robstride_cia402 --channel can0 --model rs-00 --motor-id 1 --mode enable

# Switch motor protocol; power-cycle after this command
motor_cli \
  --vendor robstride_cia402 --channel can0 --mode set-protocol --protocol canopen

# Set current position as zero
motor_cli \
  --vendor robstride_cia402 --channel can0 --model rs-00 --motor-id 1 --mode zero

# Enable CAN watchdog; manual says raw 20000 means 1 second
motor_cli \
  --vendor robstride_cia402 --channel can0 --model rs-00 --motor-id 1 \
  --mode watchdog --watchdog-s 1.0

# Profile Position: pos(rad), vlim(rad/s), acc(rad/s^2)
motor_cli \
  --vendor robstride_cia402 --channel can0 --model rs-00 --motor-id 1 \
  --mode pos-vel --pos 1.57 --vlim 1.0 --acc 4.0 \
  --position-window 0.01 --position-window-time-ms 20 --loop 1

# Velocity mode: vel(rad/s)
motor_cli \
  --vendor robstride_cia402 --channel can0 --model rs-00 --motor-id 1 \
  --mode vel --vel 2.0 --loop 100 --dt-ms 20

# Torque mode: tau(Nm)
motor_cli \
  --vendor robstride_cia402 --channel can0 --model rs-00 --motor-id 1 \
  --mode torque --tau 0.2 --loop 100 --dt-ms 20
```

## 6. Vendor = `robstride_mit`

This path is RobStride F_CMD=2 MIT protocol over classic CAN standard frames. It is separate from `robstride --mode mit`: the old path is the private 29-bit extended-frame protocol, while this path uses the manual's chapter 6 standard-frame MIT protocol.

Status: experimental/incomplete. The command surface is present, but high-rate loop ergonomics, real-device validation, and `dm-device` transport support are not complete yet.

### 6.1 Supported Modes

- `scan`
- `status`
- `enable`
- `disable`
- `clear-error`
- `zero`
- `set-mode`
- `set-can-id`
- `set-host-id`
- `set-protocol`
- `save`
- `active-report`
- `mit` (packed `pos/vel/kp/kd/tau`)
- `pos-vel` (typed ID `(1<<8)|motor_id`, float `pos + vel`)
- `vel` (typed ID `(2<<8)|motor_id`, float `vel + current_limit`)
- `read-param`
- `write-param`

### 6.2 RobStride MIT Examples

```bash
# Scan MIT protocol motors; feedback-id is host id
motor_cli \
  --vendor robstride_mit --channel can0 --model rs-00 --feedback-id 0xFD \
  --mode scan --start-id 1 --end-id 127

# Switch a motor to MIT protocol; power-cycle after this command
motor_cli \
  --vendor robstride_mit --channel can0 --model rs-00 --motor-id 1 \
  --mode set-protocol --protocol mit

# MIT dynamic control
motor_cli \
  --vendor robstride_mit --channel can0 --model rs-00 --motor-id 1 --feedback-id 0xFD \
  --mode mit --pos 0.0 --vel 0.0 --kp 20.0 --kd 0.5 --tau 0.0 --loop 100 --dt-ms 20

# Position mode: float position(rad) + float velocity(rad/s)
motor_cli \
  --vendor robstride_mit --channel can0 --model rs-00 --motor-id 1 --feedback-id 0xFD \
  --mode pos-vel --pos 1.57 --vlim 1.0 --loop 1

# Velocity mode: float velocity(rad/s) + float current limit(A)
motor_cli \
  --vendor robstride_mit --channel can0 --model rs-00 --motor-id 1 --feedback-id 0xFD \
  --mode vel --vel 2.0 --current 2.0 --loop 100 --dt-ms 20
```

## 7. Vendor = `all`

`vendor=all` currently supports only `--mode scan`.

### 7.1 Additional Arguments for all-scan

| Argument | Default | Notes |
|---|---|---|
| `--damiao-model` | `4340P` | Model hint used when invoking Damiao scan path |
| `--robstride-model` | `rs-00` | Model hint used when invoking RobStride scan path |
| `--hightorque-model` | `hightorque` | Model hint used when invoking HighTorque scan path |
| `--myactuator-model` | `X8` | Model hint used when invoking MyActuator scan path |
| `--start-id` | `1` | Passed to all scans |
| `--end-id` | `255` | Passed to Damiao/RobStride; MyActuator path auto-clamps to `32` |

### 7.2 Example

```bash
motor_cli \
  --vendor all --channel can0 --mode scan --start-id 1 --end-id 255
```

## 8. Vendor = `hightorque` (native `ht_can` v1.5.5-compat (v2.0.0 migration in progress))

- This path uses native HighTorque `ht_can` v1.5.5-compat (v2.0.0 migration in progress) direct-CAN protocol.
- It is intended for setups where motors are exposed directly on SocketCAN (`can0` etc.).
- Official Panthera/HighTorque SDK serial chain (`USB serial -> CANboard -> motors`) is separate from this CLI direct-CAN path.
- Supported modes: `scan | read | ping | mit | pos | vel | tqe | pos-vel-tqe | volt | cur | stop | brake | rezero | conf-write | timed-read`.
- Unified unit interface:
  - `--pos` in `rad`
  - `--vel` in `rad/s`
  - `--tau` in `Nm`
  - `--kp`, `--kd` are accepted for MIT signature compatibility but ignored by `ht_can`.
  - Raw debug parameters: `--raw-pos`, `--raw-vel`, `--raw-tqe`.

## 9. Vendor = `myactuator`

### 9.1 Supported Modes

- `scan`
- `enable`
- `disable`
- `stop`
- `set-zero`
- `status`
- `current`
- `vel`
- `pos`
- `version`
- `mode-query`

### 9.2 MyActuator Extra Arguments

| Argument | Type | Default | Used In | Notes |
|---|---|---|---|---|
| `--start-id` | u16 | `1` | scan | Scan start, 1..32 |
| `--end-id` | u16 | `32` | scan | Scan end, 1..32 (input >32 will be clamped) |
| `--current` | f32 | `0.0` | current | Current setpoint in A |
| `--vel` | f32 | `0.0` | vel | Velocity setpoint in rad/s (converted to deg/s internally) |
| `--pos` | f32 | `0.0` | pos | Absolute position in rad (converted to deg internally) |
| `--max-speed` | f32 | `8.726646` | pos | Position move max speed in rad/s (converted internally) |

Status output note:

- `angle` comes from `0x9C` status-2 near-turn angle.
- `mt_angle` comes from `0x92` multi-turn angle and should be used for absolute-position judgement.

### 9.3 MyActuator Examples

```bash
# Scan IDs in MyActuator range
motor_cli \
  --vendor myactuator --channel can0 --mode scan --start-id 1 --end-id 32

# Query status repeatedly
motor_cli \
  --vendor myactuator --channel can0 --model X8 --motor-id 1 --feedback-id 0x241 \
  --mode status --loop 40 --dt-ms 50

# Velocity control
motor_cli \
  --vendor myactuator --channel can0 --model X8 --motor-id 1 --feedback-id 0x241 \
  --mode vel --vel 0.5236 --loop 100 --dt-ms 20

# Absolute position control
motor_cli \
  --vendor myactuator --channel can0 --model X8 --motor-id 1 --feedback-id 0x241 \
  --mode pos --pos 3.1416 --max-speed 5.236 --loop 1

# Set current position as zero (persistent after power-cycle)
motor_cli \
  --vendor myactuator --channel can0 --model X8 --motor-id 1 --feedback-id 0x241 \
  --mode set-zero --loop 1
```

## 10. Vendor = `hexfellow`

Transport constraint:
- Hexfellow is CAN-FD-only in this repository (`--transport socketcanfd`).
- Current support scope: scan / status / pos-vel / mit / enable / disable.
- Current status: transport integrated; model validation matrix pending.

### 10.1 Hexfellow Examples

```bash
# Scan IDs
motor_cli \
  --vendor hexfellow --transport socketcanfd --channel can0 \
  --mode scan --start-id 1 --end-id 32

# Status query
motor_cli \
  --vendor hexfellow --transport socketcanfd --channel can0 \
  --model hexfellow --motor-id 1 --feedback-id 0 \
  --mode status

# Position-velocity (pos in rad, vlim in rad/s)
motor_cli \
  --vendor hexfellow --transport socketcanfd --channel can0 \
  --model hexfellow --motor-id 1 --feedback-id 0 \
  --mode pos-vel --pos 3.1415926 --vlim 2.0

# MIT (pos/vel in rad / rad/s)
motor_cli \
  --vendor hexfellow --transport socketcanfd --channel can0 \
  --model hexfellow --motor-id 1 --feedback-id 0 \
  --mode mit --pos 0.0 --vel 0.0 --kp 1000 --kd 100 --tau 0
```

## 11. Practical Notes

- For Damiao ID updates, prefer keeping `--store 1 --verify-id 1`.
- If scan intermittently misses motors, retry after CAN restart.
- RobStride supports CLI `--mode pos-vel` (mapped to native Position); in this mode use `--pos/--vlim/[--kp|--loc-kp]`.
- MyActuator low-voltage protection returns error code `0x0004` in status-1 (`0x9A`) and blocks motion.
