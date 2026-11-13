# Huayi CANopen Integration — Real-Hardware Acceptance Record

Status: **PENDING HARDWARE** (template). All items below were validated against
the Huayi manual and pass simulated/offline checks (25 workspace test suites
green, clippy clean, C++ header compile-checked, api_surface parity green).
Real-hardware acceptance is blocked until a Huayi drive is on the bench.

Scope: `motor_canopen` shared framework, `motor_vendors/huayi`, `motor_abi`
wiring, bindings (Python/C++), `motor_cli`, `ws_gateway`.

Reference plan: `docs/zh/huayi_canopen_integration_plan.md` (§6 验收).

## Environment

- [ ] Adapter: PCAN / CANable candleLight / MCU serial bridge (`mcu-serial`)
- [ ] Bus: classic CAN, 1 Mbit/s, 120Ω termination ×2
- [ ] Drive: Huayi joint driver, node id 1..=127 (record actual node id and firmware)
- [ ] Power supply per manual spec

## 1. Node scan

- [ ] `motor_cli --vendor huayi --channel can0 --mode scan --start-id 1 --end-id 127`
      finds the drive; `statusword` / `mode_display` / `error_code` populated.
- [ ] `scan` works over `mcu-serial` too.
- [ ] `motor_controller_huayi_scan_ids` from C/Python returns the same hits.

## 2. Lifecycle (CiA402 state machine)

- [ ] `status`: after `enable`, 6041h statusword reflects the CiA402 state
      diagram (the gateway sends controlword `0x06→0x07→0x0F`, terminating in
      Operation Enabled); `disable` returns it toward Switch On Disabled.
- [ ] `enable` / `disable` / `stop` (controlled) / `quick-stop` (CW bit2) behave
      per manual; after quick-stop, error reset + enable recovers.
- [ ] `clear-error` clears a latched fault (trigger one via over-limit first).

## 3. Modes

- [ ] `mit` (unified `motor_handle_send_mit` → 2003h): physical rad/rad/s/Nm
      in, feedback consistent with `status`.
- [ ] `mit-compressed` (RPDO4, `0x500+node`, 8-byte packed): one frame per
      command at ≥100 Hz sustained; decode of 6076h peak-torque scaling correct.
- [ ] `pos-vel` (PP, 6060h=1): 6081h profile velocity honored, CW bit4
      handshake completes.
- [ ] `vel` (PV, 6060h=3): 60FFh target tracking at loop rate.
- [ ] `torque` (6060h=4): `--tau 0.5` on a torque meter / gravity load matches
      commanded value (6071h per-mille of 6076h peak).
- [ ] FORCE_POS correctly rejected (no matching 6060h mode).

## 4. Zeroing / persistence

- [ ] `zero` without prior disable is rejected by the guard (3001h requires
      Switch On Disabled); with `disable` first it succeeds (±0.5 Rev window).
- [ ] `save` (1010h) persists zero across power cycle; `restore` (1011h)
      rolls back.
- [ ] `set-node-id` (2001h): store + power cycle → drive answers on new id.

## 5. Feedback stream

- [ ] `report --period-ms 10`: TPDO1/TPDO2/TPDO3 stream; `status` reads come
      from cache with no SDO traffic on the bus (verify with candump).
- [ ] `report --period-ms 0` stops the stream.
- [ ] `request_feedback` semantics: no-op while stream active; auto-activates
      the 10ms stream when inactive.
- [ ] State mapping: `status_code` = 6061h display, `fault_code` = 603Fh/EMCY
      low byte; full u16 via `get_error_code`.

## 6. Error history / NMT

- [ ] `history` walks 1003h and prints `0x{:04X}` entries, then clears.
- [ ] `nmt` 0x80 (pre-op) stops TPDO traffic; 0x01 restarts it.

## 7. Mixed bus (the layering payoff)

- [ ] One `can0`, one Huayi + one RobStride (private) motor, both registered on
      a shared bus; each controlled through its own controller without frame
      cross-talk.
- [ ] Huayi + Damiao on `mcu-serial` bridge the same way; bus-off produces a
      structured `BusStatus` error, not a bare timeout.

## 8. Frontends

- [ ] Python: `bindings/python/examples/huayi_all_interfaces_demo.py can0 1`
      full walkthrough passes (scan → status → enable → mit → mit-compressed →
      pos-vel → vel → torque → zero → report → history → nmt → save).
- [ ] C++: compile + run the equivalent sequence against `motorbridge.hpp`.
- [ ] ws_gateway: connect/enable/mit/pos-vel/vel/torque/status/ensure_mode/
      request_feedback/set_active_report over WebSocket.
- [ ] CLI flags match this checklist (see `docs/{zh,en}/cli.md` Huayi section).

## Sign-off

| Item | Result | Date | Notes |
|---|---|---|---|
| Node scan |  |  |  |
| Lifecycle |  |  |  |
| Modes |  |  |  |
| Zero/persist |  |  |  |
| TPDO stream |  |  |  |
| Errors/NMT |  |  |  |
| Mixed bus |  |  |  |
| Frontends |  |  |  |

Pending vendor input (plan §9): per-model catalog — model naming, vmax, rated
torque (pmax/tmax are read at runtime from 2004h/6076h and need no catalog).
