# Changelog

All notable changes to this project will be documented in this file.

The format is based on Keep a Changelog, and this project adheres to Semantic
Versioning.

## [Unreleased]

## [0.5.2] - 2026-08-31

### Fixed

- Aligned RobStride `set_zero_position` frame with the vendor manual: the
  SET_ZERO_POSITION (comm type 6) data area is now a full 8-byte frame
  (`01 00 00 00 00 00 00 00`, DLC=8) instead of a 1-byte short frame,
  matching the manual's 8-byte data area and the other control frames
  (enable/disable/clear_error/save_parameters).

## [0.5.1] - 2026-08-06

### Added

- Added a mode-aware RobStride `controlled_stop` that zeroes the correct
  actuator per current mode instead of always zeroing velocity: Velocity writes
  `spd_ref=0`, Position (PP) writes `vel_max=0`, PositionCsp holds the measured
  position as `loc_ref`, and MIT holds position reusing the last `kp/kd` (or a
  10% fallback). Hardened `ws_gateway` `handle_stop` to clear the active ticker
  command unconditionally before dispatching, so a failed vendor stop no longer
  leaves the loop re-sending the previous motion command. Exposed the new
  `motor_handle_stop` FFI (separate from disable) and a `stop` mode in the
  RobStride CLI, and documented stop-vs-disable semantics per mode.

### Fixed

- Corrected the RobStride `pos_vel` register misdirection in PP mode: the
  velocity cap was written to `0x7017 limit_spd` (the CSP speed limit, ignored
  by `run_mode=1`) instead of `0x7024 vel_max`, so user-supplied `vlim` silently
  had no effect. The WS gateway (one-shot entry and the 50 Hz continuous
  ticker), the Python CLI `pos-vel` path, and the wrapper demo now write
  `0x7024 vel_max`; the gateway also accepts an optional `acc` field routed to
  `0x7025 acc_set`. The legacy `loc_kp` (`0x701E`) and `loc_ref` (`0x7016`)
  writes are preserved. Documentation and protocol tables were synced from
  `limit_spd(0x7017)` to `vel_max(0x7024)`. The timing demo's intentional
  legacy baseline is left untouched as a comparison fixture.

### Changed

- Replaced `tianrking` organization references with `motorbridge` across the
  repository.
- Python package version advanced to `0.5.1`.
- Rust workspace package version advanced to `0.5.1` for release/tag alignment.
- C++ package metadata advanced to `0.5.1`.

## [0.5.0] - 2026-07-22

### Added

- Added RobStride firmware version request and reply decoding so the host can
  read the motor firmware version over the parameter protocol.

### Fixed

- Corrected RobStride MIT velocity normalization: `rs-00` vmax `50 -> 33`,
  `rs-03` vmax `50 -> 20`, and `rs-06` vmax `20 -> 50` so MIT velocity
  encoding/decoding uses the vendor-specified denominator instead of a wrong
  shared range that distorted commands and feedback.
- Aligned `rs-05` velocity and torque limits with the RS05 manual so model
  specs, validation, and downstream tooling use the correct maximum ranges.

## [0.4.9] - 2026-07-06

### Fixed

- Aligned RobStride `rs-00` and `rs-06` torque-limit metadata with the
  vendor-provided values so model specs, validation, and downstream tooling use
  the correct maximum torque ranges.

## [0.4.8] - 2026-06-24

### Added

- Added RobStride motor status parsing and display support so CLI and related
  surfaces can expose richer fault and state details during bring-up.

### Fixed

- Scoped `ws_gateway` single-motor commands to the explicit requested target so
  one-motor operations no longer fan out across multiple discovered motors.

## [0.4.7] - 2026-06-16

### Fixed

- Hardened Damiao `ensure_control_mode()` so mode switching stays within one
  shared timeout budget instead of serially reusing the full timeout across
  every internal step. This reduces `ensure_mode` timeouts on slower joints and
  adapters.
- Improved Damiao mode-switch preparation and verification by reusing the hold
  position read, validating register writes through write ACKs, and making
  parameter persistence behave more reliably on real hardware.
- Raised the `dm-serial` transport timeout from `1 ms` to `10 ms` to reduce
  cross-platform serial write failures under tighter adapter/driver timing.

## [0.4.6] - 2026-06-12

### Fixed

- Reused existing Damiao motor handles during `damiao_state_many` polling so
  repeated browser telemetry no longer fails on duplicate motor registration.
- Allowed browser `ws_gateway` clients to send `MOTORBRIDGE_WS_TOKEN` in the
  WebSocket URL query as `?motorbridge_ws_token=...` while keeping existing
  header-based auth and non-loopback token requirements unchanged.

### Changed

- Python package version advanced to `0.4.6`.
- Rust workspace package version advanced to `0.4.6` for release/tag
  alignment.
- C++ package metadata advanced to `0.4.6`.

## [0.4.5] - 2026-06-05

### Added

- Added experimental RobStride standard-frame vendor paths:
  `robstride_cia402` for CANopen/CiA402 (`F_CMD=1`) and `robstride_mit` for
  MIT protocol (`F_CMD=2`). These paths expose core CLI entry points and
  documentation, but remain incomplete for production use; real-device
  validation matrices, high-rate/control ergonomics, EDS/PDO/SYNC coverage, and
  generic `dm-device` transport wiring are still pending.
- Added hardware gallery media and adapter documentation for common CAN and
  Damiao adapter setups.

### Fixed

- Improved Damiao `ensure_control_mode()` so failed mode-register verification
  attempts retry the `RID 10` write before the next readback attempt, using a
  conservative shared 20 ms retry gap. This keeps the successful path free of an
  unconditional post-write sleep while recovering from dropped or ignored mode
  writes more reliably.
- Fixed a strict Clippy warning in motor CLI defaults.

### Changed

- Documented RobStride protocol ID roles across the private, CANopen/CiA402,
  and MIT protocol paths.
- Python package version advanced to `0.4.5`.
- Rust workspace package version advanced to `0.4.5` for release/tag
  alignment.
- C++ package metadata advanced to `0.4.5`.

## [0.4.4] - 2026-06-04

### Added

- Added Damiao `dm-device` transport backed by DaMiao DM_Device SDK
  (`USB2CANFD`, `USB2CANFD_DUAL`, and `LINKX4C`). This transport is currently
  intended for Damiao motor protocol, and the adapter must be in USB mode.
- Added vendored DM_Device SDK runtime files under `third_party/dm_device`
  with platform-specific Linux/macOS/Windows libraries.
- Added `DmDeviceBus` in `motor_core` and a C++ SDK shim that exposes a stable
  C ABI for Rust while preserving the SDK's native frame/callback layout.
- Added Rust CLI flags `--transport dm-device`, `--dm-device-type`, and
  `--dm-channel`. `usb2canfd` uses `0`, `usb2canfd-dual` uses
  `0`/`1`, and `linkx4c` uses SDK channels `0..3`.
- Added ABI constructor `motor_controller_new_dm_device(...)`.
- Added Python `Controller.from_dm_device(...)` and Python CLI
  `--transport dm-device` support.
- Added Python DM_Device runtime resolver and
  `motorbridge-install-dm-device` helper. Python wheels do not embed the
  vendor runtime; if it is missing, the resolver prints the required file,
  GitHub download URL, cache/source-tree placement paths, and platform ABI
  requirements.
- Added WebSocket gateway `dm-device` support for Damiao, including
  `dm_device_type` and `dm_channel` target fields.

### Fixed

- Fixed repeated open behavior for long-running `dm-device` processes by
  reusing the already-open SDK handle inside the C++ shim instead of reopening
  the same USB adapter.
- Aligned Damiao `dm-device` scans in Rust CLI, Python CLI, and WebSocket
  gateway to use the stable feedback request path.
- Updated `dm-device` scan semantics so omitting `--dm-channel` / `dm_channel`
  scans every channel for the selected adapter; specifying a channel restricts
  the scan to one physical channel.
- Changed Python wheel packaging to omit `libdm_device.so`/`.dylib`/`.dll` by
  default. This avoids Linux manylinux `auditwheel` failures caused by the
  vendor x86_64 library requiring `GLIBCXX_3.4.32`.
- Added readback verification for WebSocket Damiao register writes. Confirmed
  readback returns `verified=true`; mismatches are hard errors; missing
  readback after a sent write returns `ok=true` with a warning instead of a
  misleading red failure.
- Softened Damiao WebSocket mode/register confirmation timeouts when the write
  was sent but no confirmation frame arrived within the timeout.
- Fixed Linux aarch64 CI builds by installing and selecting the matching C++
  cross compiler for the DM_Device C++ shim.
- Restricted Windows DM_Device runtime selection to supported x86_64 targets.

### Verified

- Verified Linux x86_64 wheel build includes `libmotor_abi.so` and
  `ws_gateway`, and does not embed `libdm_device.so`.
- Verified installed wheel can scan USB2CANFD_DUAL channel 0 and channel 1 Damiao
  motors on Linux x86_64 when the matching DM_Device runtime is available.
- Verified LINKX4C SDK channel `0..3` scan path on Linux x86_64, with Damiao
  motor feedback observed on channel `0`.
- Verified a single WebSocket gateway process can scan DM_Device channel 1 and then channel 0
  through `dm-device`.
- Documented runtime ABI requirements: Linux x86_64 requires
  `libusb-1.0.so.0`, `GLIBC_2.14+`, and `libstdc++.so.6` with
  `GLIBCXX_3.4.32`; Linux aarch64 requires `GLIBC_2.17+` and
  `GLIBCXX_3.4.22+`.
- Verified Windows GNU Rust `motor_core` cross-check. Windows/macOS runtime
  libraries are included and path-selected, but final runtime validation still
  requires those hosts.

### Changed

- Python package version advanced to `0.4.4`.
- Rust workspace package version advanced to `0.4.4` for release/tag
  alignment.
- C++ package metadata advanced to `0.4.4`.
- Documentation and API surface metadata updated for `dm-device`.

## [0.4.2] - 2026-06-03

### Fixed

- Optimized Damiao `dm-serial` background polling so `recv(0ms)` performs a
  true non-blocking drain instead of waiting on the serial-port read timeout
  when no bytes are pending.
- Reduced the Damiao serial bridge read timeout from 2 ms to 1 ms for bounded
  synchronous feedback/register reads while keeping compatibility fallbacks for
  serial backends that cannot report pending bytes.

### Changed

- Python package version advanced to `0.4.2`.
- Rust workspace package version advanced to `0.4.2` for release/tag
  alignment.
- C++ package metadata advanced to `0.4.2`.

## [0.4.1] - 2026-06-02

### Added

- Added ABI metadata entrypoints `motor_abi_version()` and
  `motor_abi_capabilities_json()` so tools, gateways, and language bindings can
  discover the loaded ABI version and supported transports/vendors/features.
- Added Python metadata helpers `motorbridge.abi_version()` and
  `motorbridge.abi_capabilities()`.
- Added C++ metadata helpers `motorbridge::abi_version()` and
  `motorbridge::abi_capabilities_json()`.
- Added `bindings/api_surface.json` as the canonical binding/API surface list
  used to keep ABI, Python, C++, and documentation aligned.
- Added API surface regression coverage for the binding parity metadata.

### Changed

- Completed C++ wrapper parity with Python for RobStride helper methods:
  `robstride_ping_host_id`, `robstride_get_param_f32_host_id`,
  `robstride_get_fault_report`, and `robstride_set_active_report`.
- Added C++ RobStride ID range validation to match Python binding behavior.
- Python package version advanced to `0.4.1`.
- Rust workspace package version advanced to `0.4.1` for release/tag
  alignment.
- C++ package metadata advanced to `0.4.1`.

## [0.4.0] - 2026-06-02

### Added

- Added WebSocket gateway `damiao_state_many` for Damiao multi-joint state
  reads, so browser HMIs can refresh every discovered joint in one logical
  request over `dm-serial`.
- Added Damiao state identity fields (`motor_id`, `feedback_id`, `model`) to
  gateway state snapshots, allowing clients to merge whole-arm telemetry by
  joint instead of treating all feedback as the active target.
- Added capability discovery for the Damiao multi-state operation.

### Fixed

- Fixed Windows Damiao `dm-serial` whole-arm scans in the WebSocket gateway by
  releasing active Damiao sessions and stopping state/parameter streams before
  scan probes reuse the serial port.
- Fixed stale Damiao state reads by requesting fresh feedback with a bounded
  timeout before returning gateway state snapshots.
- Fixed `param_stream enabled=false` so disabling the stream no longer opens or
  reopens hardware sessions.
- Fixed RobStride scan/session release handling to keep the same scan-safe
  behavior introduced for Windows PCAN.

### Changed

- Python package version advanced to `0.4.0`.
- Rust workspace package version advanced to `0.4.0` for release/tag alignment.
- C++ package metadata advanced to `0.4.0`.

## [0.3.9] - 2026-05-26

### Fixed

- Changed RobStride handling of the unified `request_feedback()` ABI call to a
  non-blocking no-op instead of issuing a blocking `ping`.
- Avoided misleading RobStride state semantics: `ping` replies do not synthesize
  `MotorState`, so `request_feedback() -> get_state()` no longer appears to
  refresh state when it cannot.
- Documented the correct RobStride state-query choices: use `robstride_ping()`
  for connectivity checks, active report for streaming state, or typed
  parameter reads for fresh position/velocity values.

### Changed

- Python package version advanced to `0.3.9`.
- Rust workspace package version advanced to `0.3.9` for release/tag alignment.
- C++ package metadata advanced to `0.3.9`.

## [0.3.8] - 2026-05-26

### Added

- Added RobStride-specific `pos-vel-pp` and `pos-vel-csp` control paths that
  follow the vendor manual sequences for PP and CSP position modes.
- Added C ABI and Python binding entrypoints for
  `robstride_send_pos_vel_pp()` and `robstride_send_pos_vel_csp()`.
- Added Python CLI and Rust CLI support for RobStride `pos-vel-pp` and
  `pos-vel-csp`.
- Added `robstride_posvel_timing_demo.py` to compare legacy `pos-vel`, PP, and
  CSP timing, including full-sequence and prepared high-rate profiles.
- Added documentation and Python code examples for RobStride PP/CSP usage and
  high-rate `loc_ref` loops.

### Fixed

- Removed the default 260 ms blocking wait from RobStride communication type 18
  parameter writes; set `MOTORBRIDGE_ROBSTRIDE_WRITE_ACK_TIMEOUT_MS` to restore
  conservative synchronous waiting.
- Clarified the difference between RobStride full manual PP/CSP sequences and
  high-rate prepared loops so users do not pay an `enable` ack wait every
  control cycle.

### Changed

- Python package version advanced to `0.3.8`.
- Rust workspace package version advanced to `0.3.8` for release/tag alignment.
- C++ package metadata advanced to `0.3.8`.

## [0.3.7] - 2026-05-21

### Fixed

- Fixed Python CLI RobStride scans on Windows PCAN by probing one
  host/feedback ID at a time, avoiding multiple active controllers and receive
  workers on the same CAN channel.
- Fixed Python CLI RobStride scan cleanup so unbound probe controllers are not
  asked to `close_bus()`, removing the misleading `controller has no motor`
  error after scan results.
- Fixed Python CLI RobStride single-host scans to print `hit` and `no reply`
  lines in probe order, so operators can see `0x01`, `0x02`, `0x03`, ... live
  instead of seeing missed IDs only after the full scan completes.
- Fixed WebSocket gateway RobStride scans to probe each requested host/feedback
  ID exactly and sequentially, use request-level `channel` and `model`, avoid
  appending unrelated default feedback IDs, and clamp default probe timing for
  Windows PCAN reliability.
- Fixed Windows WebSocket gateway RobStride repeated scans by releasing any
  active RobStride session before scan probes and adding a short PCAN release
  gap after scan `close_bus()`.
- Fixed PCAN error reporting to include names for `PCAN_ERROR_INITIALIZE` and
  `PCAN_ERROR_ILLHW`.

### Added

- Added Python scan regression coverage for the single-controller RobStride
  probing behavior.
- Added optional `MOTORBRIDGE_WS_DEBUG=1` connection logs for gateway TCP accept
  and WebSocket handshake diagnostics.
- Added the WebSocket gateway `capabilities` operation so MotorBridge Studio
  can discover batch scan support instead of falling back to legacy per-ID
  frontend probing.
- Added the WebSocket gateway `batch_scan` operation for Studio compatibility.
- Added live WebSocket `scan_progress` events for RobStride scans with
  `start`, `probe`, `hit`, `no_reply`, and `done` phases while preserving the
  final `scan` response.

### Changed

- Python package version advanced to `0.3.7`.
- Rust workspace package version advanced to `0.3.7` for release/tag alignment.
- C++ package metadata advanced to `0.3.7`.

## [0.3.6] - 2026-05-21

### Fixed

- Fixed Python CLI RobStride scans on Windows PCAN by probing one
  host/feedback ID at a time, avoiding multiple active controllers and receive
  workers on the same CAN channel.

## [0.3.5] - 2026-05-20

### Added

- Added `CoreController` drop-time polling cleanup so background receive
  workers stop even when callers forget to call `shutdown()` or `close_bus()`.
- Added RobStride MIT encoding regression tests proving all five unified MIT
  inputs (`pos`, `vel`, `kp`, `kd`, `tau`) are encoded into the native control
  frame.
- Added workspace lint inheritance for all Rust crates while keeping the active
  lint set aligned with strict CI.

### Fixed

- Hardened C ABI controller and motor handles with per-handle locking, removing
  same-handle concurrent-call undefined behavior while preserving the public ABI
  function names and signatures.
- Fixed Python binding closed-handle guards so motor methods consistently raise
  `CallError` instead of passing null pointers into the ABI.
- Fixed unbound controller operations to return a clear error instead of
  silently succeeding before any motor is added.
- Fixed Python RobStride scan efficiency by opening one controller per
  `feedback_id` candidate instead of reopening the CAN socket for every
  `(motor_id, feedback_id)` probe.
- Fixed Damiao register type error messages so `write_register_f32()` reports
  `expects float` and `write_register_u32()` reports `expects uint32`.
- Fixed CI compatibility with newer Clippy for the WebSocket handshake callback.

### Changed

- Split the Python CLI implementation from one large `cli.py` file into the
  `motorbridge.cli` package while preserving public entrypoints:
  `motorbridge.cli:main`, `python -m motorbridge.cli`, `python -m motorbridge`,
  and legacy flat run arguments.
- Clarified `open_can_bus()` as the cross-platform classic-CAN backend selector;
  `open_socketcan()` remains available as a compatibility alias.
- Python package version advanced to `0.3.5`.
- Rust workspace package version advanced to `0.3.5` for release/tag alignment.
- C++ package metadata advanced to `0.3.5`.

## [0.3.4] - 2026-05-20

### Added

- Added a detailed Chinese WebSocket gateway protocol manual covering every
  JSON `op`, parameters, defaults, vendor applicability, responses, and browser
  usage examples.
- Added RobStride Rust/Python CLI manual test commands for read/write/readback,
  optional store, active-report, clear-error, control smoke tests, and ID update
  workflows.
- Added regression coverage for RobStride parameter save responses that return
  a non-status device reply.

### Fixed

- Fixed `ws_gateway` `set_id` so Damiao `--transport dm-serial` is honored
  instead of accidentally opening the SocketCAN/PCAN path.
- Improved Damiao `ensure_control_mode()` so an initial register-10 read timeout
  no longer prevents writing the requested mode; write verification now retries.
- Fixed RobStride `save_parameters()` to accept valid device replies after
  communication type `22`, avoiding false `control ack timeout: comm_type=22`
  errors after a parameter was already written and read back successfully.
- Fixed strict Clippy failures across `motor_core`, `motor_cli`,
  `motor_vendor_robstride`, and `ws_gateway`.

### Changed

- Python package version advanced to `0.3.4`.
- Rust workspace package version advanced to `0.3.4` for release/tag alignment.

## [0.3.3] - 2026-05-19

### Added

- Added `-v` / `--version` output to the Rust CLI and Python CLI.
- Added Python binding version helpers: `motorbridge.__version__` and
  `motorbridge.get_version()`.
- Added `--store 1` to Python `robstride-write-param` and `damiao-write-param`
  subcommands for unified write, verify, and persist workflows.
- Added Rust CLI Damiao `read-param` / `write-param` support with matching
  `--type`, `--verify`, and `--store` semantics.

### Fixed

- Python CLI now disables argparse long-option abbreviation for the root parser,
  subcommands, and legacy run parser. Invalid options such as `--mode save` on
  `robstride-write-param` are rejected instead of being misparsed as `--model save`.
- RobStride `save_parameters()` now waits for the protocol status ACK after
  sending communication type `22`.

### Changed

- Python package version advanced to `0.3.3`.
- Rust workspace package version advanced to `0.3.3` for release/tag alignment.

## [0.3.2] - 2026-05-18

### Added

- Added RobStride fault-report diagnostics to the C ABI and Python SDK via
  `motor_handle_robstride_get_fault_report` / `Motor.robstride_get_fault_report()`.
- Python CLI state printing now includes non-zero RobStride `fault_raw` and
  `warning_raw` values.

### Fixed

- RobStride `FAULT_REPORT` frames no longer overwrite the latest motion state.
  Fault reports now update only the fault cache, so fault payloads are not
  exposed as bogus `-720 deg` / `-50 rad/s` / `0 C` feedback.
- RobStride `FAULT_REPORT` frames no longer advance the control status ACK
  sequence, avoiding false command acknowledgements.
- RobStride `clear_error()` clears the local cached fault report only after the
  device acknowledges the clear request.

### Changed

- Python package version advanced to `0.3.2`.
- Rust workspace package version advanced to `0.3.2` for release/tag alignment.

## [0.3.1] - 2026-05-15

### Fixed

- Python CLI RobStride `mit`, `pos-vel`, and `vel` now align their control
  startup sequence with the Rust CLI and WebSocket gateway: disable torque,
  set and verify `run_mode` via `0x7005`, re-enable torque, then send targets.
  This fixes cases where direct Python CLI `pos-vel` could enable the motor but
  fail to move until a Rust CLI or gateway scan/control path had prepared the mode.

### Changed

- Python package version advanced to `0.3.1`.
- Rust workspace package version advanced to `0.3.1` for release/tag alignment.

## [0.3.0] - 2026-05-15

### Added

- RobStride clear-fault support is now exposed through the unified clear-error path.
  `motor_handle_clear_error` sends RobStride communication type `4` with `data[0]=1`.
- RobStride active-report support is now exposed across Rust CLI, Python CLI/SDK,
  ABI, and WebSocket gateway.
- New ABI symbol: `motor_handle_robstride_set_active_report`.
- New Python SDK method: `Motor.robstride_set_active_report(enabled)`.
- New WS gateway operation: `{"op":"set_active_report","enabled":true}`.
- RobStride communication type `21` fault reports are decoded into raw fault/warning
  words plus documented fault and warning bits for diagnostics.

### Documentation

- Added RobStride bring-up notes for `EPScan_time(0x7026)`, including
  `EPScan_time=3` as the recommended initial 20 ms report interval for arm calibration.
- Added CLI, Python, ABI, and WS examples for RobStride clear-error and active-report.

## [0.2.9] - 2026-05-14

### Added

- Python CLI now exposes Damiao parameter/register read and write commands:
  `damiao-read-param` and `damiao-write-param`.
- Python CLI `run` now accepts Rust-style RobStride parameter modes:
  `--mode read-param`, `--mode write-param`, and `--mode save`.
- Python CLI `run` now accepts Rust-style ID update shortcuts:
  `--set-motor-id`, `--set-feedback-id`, `--verify-id`, and Damiao model verification options.

### Fixed

- Legacy Python CLI flat commands such as `motorbridge-cli --vendor robstride ...`
  are parsed as `run` commands instead of being rejected as invalid subcommands.
- Python CLI RobStride `pos-vel` now follows the same native register path as the
  WS gateway (`limit_spd` `0x7017`, `loc_kp` `0x701E`, `loc_ref` `0x7016`).
- RobStride and Damiao Python CLI documentation was aligned with the Rust CLI and binding behavior.

### Changed

- Python binding package version advanced to `0.2.9`.
- Rust workspace crates advanced to `0.2.9`.

## [0.2.8] - 2026-05-12

### Fixed

- Restored the MotorBridge tree to the v0.2.6-compatible unified RobStride interface shape.
- Completed the RobStride protocol section 4 runtime parameter list through `0x702E`,
  including `damper`, `add_offset`, `alveolous_open`, `iq_test`, and `dcc_set`.
- RobStride `set_zero_position()` now keeps the same upper-level command/API shape while
  writing `zero_sta(0x7029)=1` behind the scenes so zeroed motors use the `-pi..pi`
  startup coordinate range.
- RobStride parameter save now sends the official type-22 payload `01 02 03 04 05 06 07 08`.

### Changed

- Python binding package version advanced to `0.2.8`.
- Rust workspace crates advanced to `0.2.8`.

## [0.2.6] - 2026-05-09

### Added

- RobStride host-id-specific ABI helpers for exact scan probing from Python:
  `motor_handle_robstride_ping_host_id` and `motor_handle_robstride_get_param_f32_host_id`.
- Python SDK wrappers `robstride_ping_host_id(...)` and `robstride_get_param_f32_host_id(...)`.
- Release test note `release_test_notes/0.2.6.md` covering Rust core/CLI, Python binding/CLI,
  package smoke checks, and full RobStride/Damiao CLI command examples.

### Changed

- RobStride `motor_id` / `device_id` is now validated as `1..255`; `feedback_id` / `host_id` is
  validated as `0..255` across core, Rust CLI, Python SDK/CLI, and websocket gateway flows.
- Rust and Python RobStride scan now probe each listed `--feedback-ids` host ID exactly instead of
  silently falling back inside each candidate probe.
- RobStride parameter response filtering now requires the response `device_id` to match the target
  motor, reducing cross-talk risk on multi-motor buses.
- The embedded `bindings/python/mintlify` documentation copy was removed; canonical Mintlify docs now
  live in the sibling `motorbridge-docs` repository.

## [0.2.5] - 2026-05-09

### Added

- Python CLI `id-set --vendor robstride` now supports RobStride device ID updates with optional store and verify.
- Rust `motor_cli` accepts Python-style bare mode shorthand, for example `motor_cli scan --vendor robstride ...`.
- Rust RobStride scan now accepts `--feedback-ids`, `--timeout-ms`, `--param-id`, and `--param-timeout-ms`, matching the Python scan entrypoint.

### Changed

- RobStride scan output and documentation now consistently distinguish motor `device_id` / `probe` from host-side `feedback_id` / `host_id`.
- Python and Rust RobStride scan defaults are aligned around host ID candidates `0xFD,0xFF,0xFE,0x00,0xAA`.

## [0.2.3] - 2026-04-16

### Changed

- Refactored ABI FFI layers to reduce duplicated controller/motor dispatch boilerplate via shared
  macros and helpers.
- Consolidated vendor parameter FFI entrypoints (Hexfellow/HighTorque/MyActuator/RobStride) with
  shared macro-generated get/write wrappers.
- Aligned runtime/control-path robustness fixes across motor core, vendor controllers, Python
  bindings, and websocket gateway integration.

## [0.1.3] - 2026-03-24

### Added

- New practical Damiao guide:
  - `examples/damiao_controll_all_in_one.md`
  - includes one-page command bundles for:
    - CLI four core modes (`mit`, `pos-vel`, `vel`, `force-pos`)
    - C/C++ ABI examples
    - Python ctypes ABI examples
    - Python bindings examples
    - C++ bindings examples

### Changed

- Damiao CLI runtime output (`motor_cli/src/damiao_cli.rs`) now prints richer realtime fields:
  - `id`, `arbitration_id`, `status_name`
  - temperatures `t_mos`, `t_rotor`
  - mode-aware command/target context and tracking errors
    - MIT: `cmd_pos/cmd_vel/kp/kd/cmd_tau/e_pos/e_vel`
    - POS_VEL: `cmd_pos/vlim/e_pos`
    - VEL: `cmd_vel/e_vel`
    - FORCE_POS: `cmd_pos/vlim/ratio/e_pos`

## [0.1.2] - 2026-03-23

### Changed

- Release version bump from `0.1.1` to `0.1.2` for clean tag progression.
- Damiao `dm-serial` documentation rollout remains aligned across:
  - CLI README (full interface section)
  - root README
  - bindings/examples/integrations/tools related READMEs.

## [0.1.1] - 2026-03-23

### Added

- Damiao serial-bridge transport (`dm-serial`) for unix-like systems:
  - CLI transport selection: `--transport auto|socketcan|dm-serial`
  - Serial options: `--serial-port`, `--serial-baud`
  - Damiao controller serial constructor and transport runtime wiring.
- C ABI constructor for Damiao serial bridge:
  - `motor_controller_new_dm_serial(serial_port, baud)`
- SDK support for Damiao serial bridge:
  - Python: `Controller.from_dm_serial(...)`
  - C++: `Controller::from_dm_serial(...)`
- New Chinese operation manual for deployment/runtime usage:
  - `docs/zh/operation_manual.md`

### Changed

- README alignment across examples/bindings/integrations/tools:
  - All Damiao-related READMEs now mention `dm-serial` availability.
  - Added explicit pointer to complete interface/command section in
    `motor_cli/README.zh-CN.md` (`3.6`) and `motor_cli/README.md`.

## [0.1.0] - 2026-03-20

### Added

- Linux CANable candleLight/gs_usb quick guide in root README (EN/ZH), including candleLight/gs_usb setup and
  `--channel can0` usage examples.
- Channel quick reference in `motor_cli/README.md` and `motor_cli/README.zh-CN.md` covering:
  - Linux SocketCAN channels (`can0`, `can1`) and Linux rule "no `@bitrate` in channel name"
  - Windows PCAN channel mapping (`can0/can1`) with optional `@bitrate`

### Changed

- CLI startup summary now distinguishes scan semantics from control semantics:
  - `--mode scan` prints `model_hint`, `base_feedback_id`, and `scan_range`
  - defaults are explicitly tagged as `(default)` to reduce confusion

### Fixed

- RobStride frame filtering now only accepts status/fault frames from the target motor ID,
  preventing cross-device state pollution on shared CAN buses.
- Architecture Mermaid diagrams (EN/ZH) now include `myactuator` branch for consistency with
  workspace/runtime layout.

### Usage

- Linux CANable candleLight/gs_usb setup and examples:
  - `README.md` / `README.zh-CN.md` section: "Linux CANable candleLight/gs_usb Quick Guide"
- Channel compatibility and parameter rules:
  - `motor_cli/README.md` / `motor_cli/README.zh-CN.md` section: "Channel Quick Reference"
