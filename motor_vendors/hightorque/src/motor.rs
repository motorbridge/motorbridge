use crate::protocol::{
    amps_to_raw, decode_feedback, decode_register_reply, encode_mit_frame, encode_read,
    is_setting_ack, rad_to_pos_raw, radps2_to_acc_raw, radps_to_vel_raw, reply_motor_id,
    tqe_adjust_to_raw, volts_to_raw, HightorqueFeedbackState, MotorModel, ReadCmd, RegisterType,
    RegisterValue, RunMode, TorqueCoeff, MIT_ID_PREFIX,
};
use crate::registers;
use motor_core::bus::{CanBus, CanFrame};
use motor_core::device::MotorDevice;
use motor_core::error::{MotorError, Result};
use motor_core::feedback::{FeedbackCache, FeedbackSnapshot};
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

/// `read_registers` blocking-wait cache poll interval (aligned with DM's 2ms).
const REGISTER_POLL_INTERVAL_MS: u64 = 2;

pub struct HightorqueMotor {
    pub motor_id: u16,
    feedback_id: u16,
    model: String,
    /// Torque raw↔Nm compensation coefficient (parsed from `model`, mirrors
    /// firmware `motor_tqe_adj[]`, see P2-15).
    torque_coeff: TorqueCoeff,
    bus: Arc<dyn CanBus>,
    /// State cache (motor_core cross-vendor FeedbackCache): passively written
    /// by the background receive thread via `process_feedback_frame`, with its
    /// own write sequence and ingest timestamp (`sequence`/`age`);
    /// `request_fresh_state` judges reply arrival against the pre-send
    /// timestamp — no longer races the CoreController background receive
    /// thread for `bus.recv` on the same bus. Replaces the original
    /// `Mutex<Option<…>>` + manual `state_seq` scheme (the
    /// sequence-advance-as-success pattern is built into FeedbackCache).
    state: FeedbackCache<HightorqueFeedbackState>,
    /// Setting-ACK sequence: incremented by `process_feedback_frame` on each
    /// received ACK frame; `send_with_ack` polls this sequence to decide
    /// whether the ACK has arrived (mirrors the robstride crate).
    ack_seq: AtomicU64,
    /// Register cache (fed by the passive path): `(addr, ty)` → (latest
    /// value, reply ingest timestamp), keyed `(addr, ty)` (2026-10-21 A3 fix:
    /// same address with different types get separate entries, so an explicit
    /// Int8 read is not polluted by Int16 values from the passive 27 01
    /// stream). Replies are decoded by the CoreController background receive
    /// thread via `process_feedback_frame` → `decode_register_reply` before
    /// entering the cache; the active `read_registers` claims "replies after
    /// this request" via the `ts >= request_at` timestamp freshness check,
    /// no longer receiving frames from `bus.recv` — isomorphic to DM's
    /// register-reply cache and RS's param_state design (passively fed cache
    /// + active-side freshness claim).
    register_cache: Mutex<HashMap<(u8, RegisterType), (RegisterValue, Instant)>>,
    /// Control-frame frame-type switch (default off since 2026-10-09):
    /// whether control commands (pos_vel/vel/stop etc.) are sent as extended
    /// 29-bit frames `0x8000|motor_id` with bit15 reply master-switch set.
    /// Default (off): **standard 11-bit bare-id frame** — the v2.0.0
    /// reference firmware host's original behavior
    /// (`can_send(hfdcanx, id, ...)`, control-class rows in protocol §1.3).
    /// On: the `0x8000|id` extended form from the PDF §1.4 example. It was
    /// default-on on 2026-09-29, motivated by "every control frame returns
    /// a §3.2 status feedback frame"; falsified by the 2026-10-09 candump
    /// real-bus test: with bit15 set on a non-query command (e.g. 05 13),
    /// the motor replies only one DLC=0 empty frame (the protocol PDF §1.2
    /// says exactly this), with **no** status data — the only effect of
    /// enabling is a useless empty frame per control frame as bus overhead,
    /// so it reverted to standard frames. The switch is kept only for
    /// diagnostics/reproduction experiments (not exposed to ABI/Python).
    control_reply_enabled: AtomicBool,
}

impl HightorqueMotor {
    pub(crate) fn new(motor_id: u16, feedback_id: u16, model: &str, bus: Arc<dyn CanBus>) -> Self {
        let torque_coeff = MotorModel::from_hint(model)
            .unwrap_or(MotorModel::General)
            .coeff();
        Self {
            motor_id,
            feedback_id,
            model: model.to_string(),
            torque_coeff,
            bus,
            state: FeedbackCache::new(),
            ack_seq: AtomicU64::new(0),
            register_cache: Mutex::new(HashMap::new()),
            control_reply_enabled: AtomicBool::new(false),
        }
    }

    /// Current torque compensation coefficient (for upper layers/test diagnostics).
    pub fn torque_coeff(&self) -> TorqueCoeff {
        self.torque_coeff
    }

    /// Toggle the control-frame frame type (see the `control_reply_enabled`
    /// field comment). Default off: standard 11-bit bare id (the reference
    /// firmware host's original behavior). Set true to switch to extended
    /// `0x8000|id` + bit15 — note the motor replies to non-query commands
    /// with only one DLC=0 empty frame (verified on real hardware
    /// 2026-10-09), no status data; diagnostics/reproduction experiments
    /// only.
    pub fn set_control_reply(&self, enabled: bool) {
        self.control_reply_enabled.store(enabled, Ordering::Release);
    }

    pub fn latest_state(&self) -> Option<HightorqueFeedbackState> {
        self.latest_state_snapshot().map(|snapshot| snapshot.state)
    }

    pub fn latest_state_snapshot(&self) -> Option<FeedbackSnapshot<HightorqueFeedbackState>> {
        self.state.latest().ok().flatten()
    }

    /// Mapping of the unified enable contract onto HT: **semantic no-op
    /// (option B, user decision 2026-10-08), sends no frame**. The HT
    /// protocol has no separate enable frame; the control command's cmd
    /// byte determines the behavior and the motor executes it on receipt,
    /// so the enable contract's postcondition ("subsequent control
    /// commands will be executed") always holds — return Ok directly.
    ///
    /// Motivation: core `apply_bulk_device_op` aborts on error — an enable
    /// returning Err poisons mixed-bus `enable_all()` (motors after the HT
    /// one would never be enabled); the unified
    /// enable→command→disable flow also requires enable to be callable.
    /// Clearing errors/stopping remains explicit via
    /// `clear_error()`/`disable()`'s `01 00 00`, never piggybacked on
    /// enable, keeping this method side-effect free.
    pub fn enable(&self) -> Result<()> {
        Ok(())
    }

    pub fn disable(&self) -> Result<()> {
        self.send_stop()
    }

    /// clear_error ≡ stop (G3): table-2 mode 0's name is literally
    /// "stop, clear errors"; writing mode register 0x000=0 via the
    /// `0x01 0x00 0x00` frame performs both actions at once, same frame as
    /// `disable` (the protocol has no separate clear_error command).
    pub fn clear_error(&self) -> Result<()> {
        self.send_stop()
    }

    pub fn set_zero_position(&self) -> Result<()> {
        // Set the current position as zero (`0x40 0x01 0x04 0x64 0x20 0x63
        // 0x0A`, reference firmware `libelybot_can.c::rezero_pos`). This
        // command only changes the position offset in RAM, no mechanical
        // action (firmware comment: "this command only modifies in RAM");
        // fire-and-forget — no ACK wait, no sleep, followed immediately by
        // store_parameters to persist — aligned with the reference firmware,
        // whose rezero_pos/conf_write are already fire-and-forget (the 0x41
        // ACK timing is unreliable; the reference firmware never waits for
        // it either).
        self.send_query(&[0x40, 0x01, 0x04, 0x64, 0x20, 0x63, 0x0A], 7)?;
        // Short pause between the RAM offset write and the subsequent
        // conf_write, avoiding back-to-back extended frames.
        std::thread::sleep(Duration::from_millis(50));
        self.store_parameters()
    }

    pub fn ensure_control_mode(&self, mode: u32, _timeout: Duration) -> Result<()> {
        // Table-2 run modes: 0-15 defined, 16/17 reserved; reject beyond range.
        if mode <= 17 {
            Ok(())
        } else {
            Err(MotorError::InvalidArgument(format!(
                "HighTorque run mode must be 0-17 (表2), got {mode}"
            )))
        }
    }

    /// MIT mode (motor firmware v4.6.0+): `0x18000 | id` extended frame,
    /// bit-packed pos(16)/vel(12)/tqe(12)/kp(12)/kd(12). Output torque =
    /// (target position - current position)·kp + (target velocity -
    /// current velocity)·kd + feedforward torque.
    ///
    /// Ranges: pos ±3.2768 turns, vel ±2.0 turns/s, tqe ±10 Nm, kp ±400,
    /// kd ±100. Torque model adaptation (`tqe_adjust`) and PID model
    /// adaptation (`pid_adjust`) are documented in P2-15 and currently
    /// uncorrected.
    pub fn send_cmd_mit(
        &self,
        target_position: f32,
        target_velocity: f32,
        stiffness: f32,
        damping: f32,
        feedforward_torque: f32,
    ) -> Result<()> {
        let data = encode_mit_frame(
            target_position,
            target_velocity,
            feedforward_torque,
            stiffness,
            damping,
        );
        self.send_raw(MIT_ID_PREFIX | u32::from(self.motor_id), &data, 8, true)
    }

    /// Trapezoid mode (motor firmware v4.6.0+): `0x07 0x2d`,
    /// `[pos@2-3, vel@4-5, acc@6-7]` int16 little-endian. Moves to the
    /// target position at the target velocity/acceleration. `0x8000` means
    /// unlimited.
    pub fn send_cmd_pos_vel_acc(
        &self,
        target_position: f32,
        velocity_limit: f32,
        acceleration_limit: f32,
    ) -> Result<()> {
        let pos_raw = rad_to_pos_raw(target_position);
        let vel_raw = radps_to_vel_raw(velocity_limit);
        let acc_raw = radps2_to_acc_raw(acceleration_limit);
        let mut data = [0x07, 0x2d, 0, 0, 0, 0, 0, 0];
        data[2..4].copy_from_slice(&pos_raw.to_le_bytes());
        data[4..6].copy_from_slice(&vel_raw.to_le_bytes());
        data[6..8].copy_from_slice(&acc_raw.to_le_bytes());
        self.send_control(&data, 8)
    }

    pub fn send_cmd_pos_vel(&self, target_position: f32, velocity_limit: f32) -> Result<()> {
        let pos_raw = rad_to_pos_raw(target_position);
        let vel_raw = radps_to_vel_raw(velocity_limit);
        self.send_cmd_pos_vel_tqe(pos_raw, vel_raw, i16::MIN)
    }

    /// Classic position mode (§1.2.1 / reference firmware
    /// `motor_control_pos`): `0x07 0x07`, `[pos@2-3, 0x0000@4-5, tqe@6-7]`,
    /// 8 bytes, little-endian. Unlike coordinated mode `0x07 0x35` (the
    /// P1-2 path), this mode only takes target position + max torque; the
    /// velocity field is fixed 0.
    ///
    /// `torque_limit_nm = None` means torque unlimited (`0x8000`). The
    /// frame uses the standard 11-bit id = motor_id (P2-12/13). Torque is
    /// compensated per model (P2-15, `tqe_adjust`:
    /// `raw = ((tau - d) / k) * 100`); the model is parsed from `model`,
    /// unknown models fall back to General (k=0.5).
    pub fn send_cmd_pos_classic(
        &self,
        target_position: f32,
        torque_limit_nm: Option<f32>,
    ) -> Result<()> {
        let pos_raw = rad_to_pos_raw(target_position);
        let tqe_raw = match torque_limit_nm {
            Some(nm) => tqe_adjust_to_raw(nm, self.torque_coeff),
            None => i16::MIN, // 0x8000 = unlimited
        };
        let mut data = [0x07, 0x07, 0, 0, 0, 0, 0, 0];
        data[2..4].copy_from_slice(&pos_raw.to_le_bytes());
        // bytes 4-5 fixed 0x0000 (reference firmware `motor_control_pos` template; unused in pure position mode)
        data[6..8].copy_from_slice(&tqe_raw.to_le_bytes());
        self.send_control(&data, 8)
    }

    pub fn send_cmd_vel(&self, target_velocity: f32) -> Result<()> {
        let vel_raw = radps_to_vel_raw(target_velocity);
        let mut data = [0x07, 0x07, 0x00, 0x80, 0x20, 0x00, 0x80, 0x00];
        data[4..6].copy_from_slice(&vel_raw.to_le_bytes());
        data[6..8].copy_from_slice(&i16::MIN.to_le_bytes());
        self.send_control(&data, 8)
    }

    pub fn send_cmd_force_pos(
        &self,
        _target_position: f32,
        _velocity_limit: f32,
        _torque_limit_ratio: f32,
    ) -> Result<()> {
        Err(MotorError::InvalidArgument(
            "send_force_pos is not supported for HighTorque ABI; use send_mit/send_pos_vel"
                .to_string(),
        ))
    }

    /// Torque mode (protocol §1 command 5): `0x05 0x13 <tqe16>`, 4 bytes.
    /// Takes a physical quantity: raw = `tqe_adjust_to_raw(nm, coeff)`
    /// (0.01 N·m/LSB base + model torque coefficient compensation, same
    /// conversion as pos_classic's torque limit). No mode switch needed
    /// beforehand — this command itself is the torque target delivery.
    pub fn send_cmd_torque(&self, torque_nm: f32) -> Result<()> {
        let tqe_raw = tqe_adjust_to_raw(torque_nm, self.torque_coeff);
        let mut data = [0x05, 0x13, 0, 0];
        data[2..4].copy_from_slice(&tqe_raw.to_le_bytes());
        self.send_control(&data, 4)
    }

    /// Run-mode switch (protocol §1 command 6 / table 2): writes mode
    /// register 0x000, `0x01 0x00 <mode>`, 3 bytes. mode is 0-17 (0=stop
    /// and clear errors, 8=DQ voltage, 9=DQ current, 10=position, 15=brake).
    /// Unlike `ensure_control_mode` (validate only, MotorDevice trait
    /// semantics unchanged), this method actually sends a frame.
    pub fn send_run_mode(&self, mode: u8) -> Result<()> {
        if mode > 17 {
            return Err(MotorError::InvalidArgument(format!(
                "HighTorque run mode must be 0-17 (表2), got {mode}"
            )));
        }
        self.send_control(&[0x01, 0x00, mode], 3)
    }

    /// Brake (table-2 mode 15): `0x01 0x00 0x0F`, three-phase shorted
    /// damping brake.
    pub fn brake(&self) -> Result<()> {
        self.send_run_mode(RunMode::Brake as u8)
    }

    /// DQ voltage mode (protocol §1 command 6): mode write + register
    /// write packed into one frame, `0x01 0x00 0x08 0x05 0x1B <volt16>`,
    /// 7 bytes, 0.1 V/LSB. The protocol only defines the Q component
    /// (0x1B); there is no D register.
    pub fn send_cmd_dq_voltage(&self, voltage_v: f32) -> Result<()> {
        let volt_raw = volts_to_raw(voltage_v);
        let mut data = [0x01, 0x00, 0x08, 0x05, 0x1B, 0, 0];
        data[5..7].copy_from_slice(&volt_raw.to_le_bytes());
        self.send_control(&data, 7)
    }

    /// DQ current mode (protocol §1 command 6): mode write + register
    /// write packed into one frame, `0x01 0x00 0x09 0x05 0x1C <cur16>`,
    /// 7 bytes, 0.1 A/LSB.
    pub fn send_cmd_dq_current(&self, current_a: f32) -> Result<()> {
        let cur_raw = amps_to_raw(current_a);
        let mut data = [0x01, 0x00, 0x09, 0x05, 0x1C, 0, 0];
        data[5..7].copy_from_slice(&cur_raw.to_le_bytes());
        self.send_control(&data, 7)
    }

    /// Generic register write (derived from protocol §1 command 9):
    /// `0x05 <addr> <val16>`, 4 bytes. Frame type is chosen by register
    /// purpose: control-target registers (0x13/0x1B/0x1C and other hot-path
    /// registers) go through the control channel; configuration registers
    /// (storage, report config etc.) go through the extended `0x8000|id`
    /// query channel (reference firmware
    /// `can_send(hfdcanx, 0x8000 | id, ...)`).
    /// Mode register 0x000 does not use this family; it has the dedicated
    /// `send_run_mode` frame.
    pub fn write_register(&self, addr: u8, value: i16, config_class: bool) -> Result<()> {
        // 2026-10-22 follow-up decision: gap/out-of-table addresses are
        // rejected (writes are more dangerous than reads — never write a
        // register with unknown semantics). 0xB3/0xB4 are command
        // addresses, not registers; they are delivered via the dedicated
        // store_parameters/set_periodic_report methods.
        registers::ensure_known_span(addr, 1).map_err(MotorError::InvalidArgument)?;
        let mut data = [0x05, 0, 0, 0];
        data[1] = addr;
        data[2..4].copy_from_slice(&value.to_le_bytes());
        if config_class {
            self.send_query(&data, 4)
        } else {
            self.send_control(&data, 4)
        }
    }

    /// Periodic report config (protocol §1 command 10):
    /// `0x05 0xB4 0x02 <t_ms16>`, 5 bytes, extended `0x8000|id` frame.
    /// period_ms minimum 1; passing 0 stops reporting — afterwards the
    /// motor no longer actively sends `0x27 0x01` status frames.
    pub fn set_periodic_report(&self, period_ms: u16) -> Result<()> {
        let mut data = [0x05, 0xB4, 0x02, 0, 0];
        data[3..5].copy_from_slice(&period_ms.to_le_bytes());
        self.send_query(&data, 5)
    }

    pub fn store_parameters(&self) -> Result<()> {
        // Persist to flash (`0x05 0xB3 0x02 0x00 0x00`, reference firmware
        // `conf_write`). Fire-and-forget: the reference firmware does the
        // same, and the 0x41 ACK timing is unreliable (see the
        // set_zero_position comment); the firmware recommends power-cycling
        // after conf_write for the configuration to take effect.
        self.send_query(&[0x05, 0xB3, 0x02, 0x00, 0x00], 5)
    }

    /// Send a setting-class command and wait for the ACK (7-byte
    /// `0x41 0x01 0x04 OK\r\n`). The ACK frame increments `ack_seq` on the
    /// passive feedback path (`process_feedback_frame`); this method polls
    /// that sequence. Returns `MotorError::Timeout` if no ACK arrives in
    /// time (no retry).
    pub fn send_with_ack(&self, payload: &[u8], dlc: u8, timeout: Duration) -> Result<()> {
        let start_seq = self.ack_seq.load(Ordering::Acquire);
        self.send_query(payload, dlc)?;
        let deadline = Instant::now() + timeout;
        while Instant::now() < deadline {
            if self.ack_seq.load(Ordering::Acquire) > start_seq {
                return Ok(());
            }
            std::thread::sleep(Duration::from_millis(4));
        }
        Err(MotorError::Timeout(format!(
            "HighTorque setting ACK timeout on motor id {}",
            self.motor_id
        )))
    }

    /// Non-blocking state query (DM-aligned naming; at the 2026-10-21 B1
    /// fix this name was returned to the DM "send-and-return-immediately"
    /// semantics, and the old blocking method was deleted along with its
    /// 8-byte zero-padded frame).
    ///
    /// 2026-10-24 design (driven by real-bus testing): **sends two frames**
    /// of 2-byte canonical reads back-to-back —
    /// - `17 01`: 0x01-0x03 position/velocity/torque (command 8 is a
    ///   special case of command 7);
    /// - `16 0E`: 0x0E-0x0F temperature+fault code (the two addresses are
    ///   consecutive, one frame one reply; measured on a single socket,
    ///   two back-to-back frames occupy the bus pipeline for ~1ms, reply
    ///   latency ~0.5ms).
    ///
    /// Mode (0x00) is not in this frame group — it is separated from 0x01
    /// by gap addresses, and a standalone `15 00` read also patches
    /// status_code through the same receive bridge.
    ///
    /// Replies are handled by the passive path (background receive
    /// thread): the `27 01` anchor frame writes the state cache (sequence
    /// +1, i.e. one state count); `26 0E` patches `t_mos`/`fault_code`
    /// in place via the 2026-10-24 bridge (without advancing the
    /// sequence) — **the two reply frames count as one state count**, and
    /// `request_fresh_state` freshness is still defined solely by the
    /// anchor frame. Use `request_fresh_state` to block for fresh state,
    /// `latest_state`/`cached_register` for non-blocking readback.
    /// Isomorphic to DM's same-named method (send-and-return-immediately).
    pub fn request_motor_feedback(&self) -> Result<()> {
        self.request_registers(&[
            ReadCmd {
                addr: 0x01,
                count: 3,
            },
            ReadCmd {
                addr: 0x0E,
                count: 2,
            },
        ])
    }

    /// Blocking state query composition (DM-aligned naming, matching DM's
    /// same-named method): `request_motor_feedback` sends the frames +
    /// polls the state cache, returning once a state that "arrived after
    /// this request" (`received_at >= request_at`) is present; on timeout
    /// returns `MotorError::Timeout`. `request_at` is recorded **before**
    /// sending, so there is no missed-claim window of "reply arrives
    /// first, baseline recorded later". Replaces the deleted blocking
    /// `request_motor_feedback(timeout)` (whose wait_status bus.recv
    /// frame-stealing branch was removed with it — replies always go to
    /// the background receive thread, and the register cache is no longer
    /// bypassed and starved).
    ///
    /// 2026-10-24 two-frame note: `received_at` is advanced only by the
    /// anchor frame (`27 01`), the `26 0E` patch frame does not refresh
    /// it — what this method waits for is always "fresh pos/vel/tq".
    /// Real-bus replies arrive serially in request order (anchor frame
    /// first, patch frame ~250µs later), far below the 2ms poll interval,
    /// so by the time we return the temperature/fault patch has usually
    /// landed; in an extreme race (patch not yet arrived at the instant
    /// of return) t_mos/fault_code hold the previous cycle's values —
    /// absent fields are not overwritten.
    pub fn request_fresh_state(&self, timeout: Duration) -> Result<HightorqueFeedbackState> {
        let request_at = Instant::now();
        self.request_motor_feedback()?;
        let deadline = request_at + timeout;
        loop {
            if let Some(snapshot) = self.latest_state_snapshot() {
                if snapshot.received_at >= request_at {
                    return Ok(snapshot.state);
                }
            }
            if Instant::now() >= deadline {
                return Err(MotorError::Timeout(format!(
                    "HighTorque read timeout on motor id {}",
                    self.motor_id
                )));
            }
            std::thread::sleep(Duration::from_millis(REGISTER_POLL_INTERVAL_MS));
        }
    }

    /// Generic register read (protocol §1.3) — DM-style "request as
    /// send": just puts a set of `(cmd, addr)` read-request frames on the
    /// bus and returns immediately without waiting for replies.
    ///
    /// **Multiple requests are sent one frame per command** (2026-10-24
    /// real-bus finding: the firmware parses only the first `(cmd, addr)`
    /// pair in a frame, and multi-pair packed frames with non-zero
    /// trailing bytes are silently dropped whole — one pair per frame,
    /// each frame a 2-byte canonical dlc). The whole span is validated
    /// before sending: any out-of-range address rejects the whole group
    /// with zero frames sent; a bus error mid-send returns that error
    /// (earlier frames are already on the wire, no rollback).
    ///
    /// Replies are handled by the passive path (CoreController background
    /// receive thread: `process_feedback_frame` → `decode_register_reply`)
    /// writing into `register_cache` keyed by `(addr, ty)` (with ingest
    /// timestamp). Afterwards:
    /// - block for values: `read_registers` (send + poll fresh cache);
    /// - non-blocking poll: `cached_register` (latest value).
    pub fn request_registers(&self, cmds: &[ReadCmd]) -> Result<()> {
        // 2026-10-22 follow-up decision: gap/out-of-table addresses are
        // rejected — every address covered by count must be inside the
        // register table; any violation rejects the whole span with no
        // frames sent.
        for c in cmds {
            registers::ensure_known_span(c.addr, c.count).map_err(MotorError::InvalidArgument)?;
        }
        for c in cmds {
            let payload = encode_read(*c).ok_or_else(|| {
                MotorError::InvalidArgument(
                    "request_registers: count out of range (1..=3)".to_string(),
                )
            })?;
            self.send_query(&payload, 2)?;
        }
        Ok(())
    }

    /// Read the register cache's latest value (no frame sent, no wait):
    /// for the address, takes the entry with the latest ingest timestamp
    /// **across types** (after the A3 fix the cache is keyed `(addr, ty)`,
    /// and this interface has no ty parameter, so it takes the most
    /// recent one), then converts it to a physical value via
    /// `register_physical_value` according to that entry's actual type
    /// (2026-10-22 decision: reads always return converted physical
    /// values; registers without a conversion are 1:1). Returns `None`
    /// if no reply has ever been received for the address, **or the
    /// address is outside the register table** (gap/out-of-table access
    /// rejected, 2026-10-22 follow-up). Note: "fresh" vs "stale" cannot
    /// be distinguished (the value freezes at the last one after the
    /// motor goes offline); when freshness matters use `read_registers`,
    /// or first `request_registers` then read back one beat later and
    /// judge yourself.
    pub fn cached_register(&self, addr: u8) -> Option<f32> {
        if !registers::is_known_register(addr) {
            return None;
        }
        self.register_cache
            .lock()
            .ok()
            .and_then(|c| {
                c.iter()
                    .filter(|((a, _ty), _)| *a == addr)
                    .max_by_key(|(_k, (_v, ts))| *ts)
                    .map(|(_k, (v, _ts))| *v)
            })
            .map(|v| registers::register_physical_value(addr, v, self.torque_coeff))
    }

    /// Blocking composition of the generic register read (protocol §1.3):
    /// `request_registers` sends + polls `register_cache`, claiming
    /// "replies that arrived after this request" via the
    /// `ts >= request_at` timestamp freshness check.
    ///
    /// Compared to the old implementation (receiving frames directly via
    /// `bus.recv`): no longer races the CoreController background receive
    /// thread on the same bus queue; the timestamp is the transaction id,
    /// and replies of concurrent register-class queries (e.g. the `17 01`
    /// state query) enter their own cache keys by `(addr, ty)` — entries
    /// of a different type are never claimed by this request (A3 fix,
    /// 2026-10-21).
    ///
    /// 2026-10-22 decision: reads are **fixed Int16** (the claim key is
    /// always `(addr, Int16)`, so foreign Int8/I32 reply entries are never
    /// claimed), and the returned values are converted to physical
    /// quantities via `register_physical_value` (position in rad,
    /// velocity in rad/s, torque in N·m with the model k coefficient,
    /// temperature in °C etc.; registers without a conversion are 1:1).
    pub fn read_registers(&self, cmds: &[ReadCmd], timeout: Duration) -> Result<Vec<(u8, f32)>> {
        let request_at = Instant::now();
        self.request_registers(cmds)?;
        let deadline = request_at + timeout;
        loop {
            if let Some(vals) = self.fresh_registers(cmds, request_at) {
                return Ok(vals);
            }
            if Instant::now() >= deadline {
                let addrs = cmds
                    .iter()
                    .map(|c| format!("0x{:02X}", c.addr))
                    .collect::<Vec<_>>()
                    .join(",");
                return Err(MotorError::Timeout(format!(
                    "HighTorque register read timeout on motor id {} (addr {})",
                    self.motor_id, addrs
                )));
            }
            std::thread::sleep(Duration::from_millis(REGISTER_POLL_INTERVAL_MS));
        }
    }

    /// Take from the cache the values covering all requested addresses
    /// with ingest timestamp >= `request_at`; if any entry is missing, of
    /// the wrong type (Int16 key), or stale, returns `None` (keep
    /// waiting). Address advance is unified with the decode side as
    /// wrapping (2026-10-21 B3 fix: a consecutive read starting at 0xFF
    /// wraps past 0x100 — both sides must use the same arithmetic for
    /// the claim to succeed). Claimed entries are then converted to
    /// physical values one by one.
    fn fresh_registers(&self, cmds: &[ReadCmd], request_at: Instant) -> Option<Vec<(u8, f32)>> {
        let cache = self.register_cache.lock().ok()?;
        let mut out = Vec::new();
        for c in cmds {
            for i in 0..c.count {
                let addr = c.addr.wrapping_add(i);
                match cache.get(&(addr, RegisterType::Int16)) {
                    Some((v, ts)) if *ts >= request_at => {
                        let physical =
                            registers::register_physical_value(addr, *v, self.torque_coeff);
                        out.push((addr, physical));
                    }
                    _ => return None,
                }
            }
        }
        Some(out)
    }

    fn send_stop(&self) -> Result<()> {
        self.send_control(&[0x01, 0x00, 0x00], 3)
    }

    fn send_cmd_pos_vel_tqe(&self, pos_raw: i16, vel_raw: i16, tqe_raw: i16) -> Result<()> {
        let mut data = [0x07, 0x35, 0, 0, 0, 0, 0, 0];
        data[2..4].copy_from_slice(&vel_raw.to_le_bytes());
        data[4..6].copy_from_slice(&tqe_raw.to_le_bytes());
        data[6..8].copy_from_slice(&pos_raw.to_le_bytes());
        self.send_control(&data, 8)
    }

    /// Control-class commands (vel/pos_vel/pos_vel_acc/pos_classic/stop/
    /// brake).
    ///
    /// Default **standard 11-bit frame**, ID = bare `motor_id` — aligned
    /// with the v2.0.0 reference firmware `can_send(hfdcanx, id, ...)`
    /// (control-class rows in protocol §1.3). On 2026-09-29 the default
    /// was briefly extended `0x8000|id` + bit15, hoping every control
    /// frame would bring back a §3.2 status feedback frame; falsified by
    /// the 2026-10-09 candump real-bus test: for non-query commands the
    /// motor replies only one DLC=0 empty frame (PDF §1.2 behavior as
    /// written), no status data — hence the revert to standard bare-id
    /// frames. `set_control_reply(true)` switches back to the extended
    /// form (diagnostics/reproduction only).
    fn send_control(&self, payload: &[u8], dlc: u8) -> Result<()> {
        if self.control_reply_enabled.load(Ordering::Acquire) {
            self.send_raw(u32::from(0x8000u16 | self.motor_id), payload, dlc, true)
        } else {
            self.send_raw(u32::from(self.motor_id), payload, dlc, false)
        }
    }

    /// Query/config-class commands (read/rezero/store/conf_write):
    /// **extended 29-bit frame**, ID = `0x8000 | motor_id`, with the
    /// bit15 reply master-switch set (all these paths in the reference
    /// firmware use `can_send(hfdcanx, 0x8000 | id, ...)`, see
    /// P2-12/P2-13).
    fn send_query(&self, payload: &[u8], dlc: u8) -> Result<()> {
        self.send_raw(u32::from(0x8000u16 | self.motor_id), payload, dlc, true)
    }

    fn send_raw(
        &self,
        arbitration_id: u32,
        payload: &[u8],
        dlc: u8,
        is_extended: bool,
    ) -> Result<()> {
        let mut data = [0u8; 8];
        data[..payload.len().min(8)].copy_from_slice(&payload[..payload.len().min(8)]);
        self.bus.send(CanFrame {
            arbitration_id,
            data,
            dlc: dlc.min(8),
            is_extended,
            is_rx: false,
        })
    }
}

impl MotorDevice for HightorqueMotor {
    fn vendor(&self) -> &'static str {
        "hightorque"
    }

    fn model(&self) -> &str {
        &self.model
    }

    fn motor_id(&self) -> u16 {
        self.motor_id
    }

    fn feedback_id(&self) -> u16 {
        self.feedback_id
    }

    fn enable(&self) -> Result<()> {
        HightorqueMotor::enable(self)
    }

    fn disable(&self) -> Result<()> {
        HightorqueMotor::disable(self)
    }

    fn accepts_frame(&self, frame: &CanFrame) -> bool {
        if !frame.is_rx {
            return false;
        }
        if reply_motor_id(frame) != Some(self.motor_id as u8) {
            return false;
        }
        // 8 bytes: register read reply (0x27...) or the status feedback
        // attached to control commands (byte0=mode). 7 bytes: setting ACK
        // (0x41...). Other lengths: parseable register read replies
        // (int16 count1 = 4 bytes, float = 6 bytes, etc.) — these enter
        // the register cache; anything unmatched is not passive feedback,
        // and the frame is dropped.
        match frame.dlc {
            8 => true,
            7 => is_setting_ack(frame),
            _ => decode_register_reply(&frame.data[..frame.dlc as usize]).is_some(),
        }
    }

    fn process_feedback_frame(&self, frame: CanFrame) -> Result<()> {
        if reply_motor_id(&frame) != Some(self.motor_id as u8) {
            return Ok(());
        }
        if is_setting_ack(&frame) {
            self.ack_seq.fetch_add(1, Ordering::Release);
            return Ok(());
        }
        // Register read replies enter the cache keyed `(addr, ty)` (with
        // timestamp): the 0x01-0x03 of the `0x27 0x01` status reply also
        // enter (a state query is itself a register read, Int16); short
        // frames (4/6 bytes) only go through here. The type comes from
        // the decoded RegisterValue variant, consistent with the reply
        // frame's cmd-nibble type encoding. The timestamp serves the
        // freshness-claim check of read_registers.
        if let Some(vals) = decode_register_reply(&frame.data[..frame.dlc as usize]) {
            if let Ok(mut cache) = self.register_cache.lock() {
                let now = Instant::now();
                for (addr, v) in &vals {
                    cache.insert((*addr, v.register_type()), (*v, now));
                }
            }
            // 2026-10-24 bridge: mode/temperature/fault-code register
            // replies are patched in place into the state cache —
            // **without advancing the sequence or refreshing
            // received_at** (FeedbackCache::patch_latest), merging with
            // the anchor frame `27 01` into one state count, leaving the
            // counting logic unchanged. Real-bus timing guarantees the
            // anchor frame arrives first (request_motor_feedback sends
            // `17 01` first, the motor replies serially in request order,
            // ~250µs apart). Mapping:
            // - 0x00 (mode) → status_code, 1:1;
            // - 0x0E (temperature) → t_mos, ×0.1°C (same conversion as
            //   the register read);
            // - 0x0F (fault code) → fault_code, 1:1 (direct mapping to
            //   table 3, user-confirmed 2026-10-23).
            // Standalone register reads (`15 00`/`16 0E`) benefit too —
            // the patch applies indiscriminately. Before the first
            // (anchor) frame arrives, patch returns false and the patch
            // is silently dropped. The preceding `any` check keeps
            // non-bridge addresses (e.g. the 27 01 periodic stream) from
            // touching the state lock.
            let has_bridge = vals
                .iter()
                .any(|(addr, _)| matches!(*addr, 0x00 | 0x0E | 0x0F));
            if has_bridge {
                self.state.patch_latest(|state| {
                    for (addr, v) in &vals {
                        match *addr {
                            0x00 => {
                                if let Some(mode) = bridge_u8(*v) {
                                    state.status_code = mode;
                                }
                            }
                            0x0E => {
                                state.t_mos =
                                    registers::register_physical_value(0x0E, *v, self.torque_coeff);
                            }
                            0x0F => {
                                if let Some(fault) = bridge_u8(*v) {
                                    state.fault_code = fault;
                                }
                            }
                            _ => {}
                        }
                    }
                })?;
            }
        }
        if let Some(mut state) = decode_feedback(frame, self.torque_coeff) {
            if state.can_id as u16 == self.motor_id {
                // 2026-10-23 decision: fields not carried by the frame
                // are **not overwritten in the cache** (the old
                // implementation zeroed absent fields, so a single
                // 27 01 state query would wipe the fault code brought
                // by a control reply):
                // - the `27 01` read reply only has pos/vel/tqe →
                //   status/fault/temperature keep cached values;
                // - the control-command status frame has mode/fault but
                //   likewise no temperature → t_mos/t_rotor keep old
                //   values.
                // Fields actually present in the frame update as usual
                // (including received_at freshness advance;
                // request_fresh_state unaffected).
                let is_read_reply = frame.dlc >= 1 && frame.data[0] == 0x27;
                if let Ok(Some(prev)) = self.state.latest() {
                    if is_read_reply {
                        state.status_code = prev.state.status_code;
                        state.fault_code = prev.state.fault_code;
                    }
                    state.t_mos = prev.state.t_mos;
                    state.t_rotor = prev.state.t_rotor;
                }
                // Write into the FeedbackCache (which advances the
                // sequence and ingest timestamp internally, serving
                // request_fresh_state freshness checks and
                // latest_state_snapshot).
                self.state.update(state)?;
            }
        }
        Ok(())
    }
}

/// Bridge helper (2026-10-24): register raw value → u8, 1:1. Integer
/// values are only valid within 0..=255 (run modes 0-17 and table-3
/// fault codes are both in this domain); the float variant, or
/// out-of-range/negative values, returns `None` — the field keeps its
/// cached value (absent fields are not overwritten).
fn bridge_u8(v: RegisterValue) -> Option<u8> {
    let raw = match v {
        RegisterValue::I8(x) => i32::from(x),
        RegisterValue::I16(x) => i32::from(x),
        RegisterValue::I32(x) => x,
        RegisterValue::F32(_) => return None,
    };
    u8::try_from(raw).ok()
}

#[cfg(test)]
mod tests {
    use super::*;
    use motor_core::bus::CanFrame;
    use motor_core::device::MotorDevice;
    use motor_core::test_support::MockBus;
    use std::sync::Arc;
    use std::time::Duration;

    fn make_motor(id: u16) -> (Arc<MockBus>, Arc<HightorqueMotor>) {
        let bus: Arc<MockBus> = Arc::new(MockBus::new());
        let motor = Arc::new(HightorqueMotor::new(id, 0, "ht-test", bus.clone()));
        (bus, motor)
    }

    fn ack_frame(id: u16) -> CanFrame {
        CanFrame {
            arbitration_id: u32::from(id) << 8,
            data: [0x41, 0x01, 0x04, 0x4F, 0x4B, 0x0D, 0x0A, 0],
            dlc: 7,
            is_extended: true,
            is_rx: true,
        }
    }

    #[test]
    fn process_feedback_frame_bumps_ack_seq_on_setting_ack() {
        let (_bus, motor) = make_motor(1);
        let before = motor.ack_seq.load(Ordering::Acquire);
        motor.process_feedback_frame(ack_frame(1)).expect("process");
        assert_eq!(motor.ack_seq.load(Ordering::Acquire), before + 1);
    }

    #[test]
    fn process_feedback_frame_ignores_ack_for_other_motor() {
        let (_bus, motor) = make_motor(1);
        let before = motor.ack_seq.load(Ordering::Acquire);
        // An ACK for id=2 must not be consumed by the id=1 motor
        motor.process_feedback_frame(ack_frame(2)).expect("process");
        assert_eq!(motor.ack_seq.load(Ordering::Acquire), before);
    }

    #[test]
    fn send_with_ack_returns_ok_when_ack_arrives() {
        let (bus, motor) = make_motor(1);
        let m = motor.clone();
        // Simulate the passive feedback path incrementing the ACK
        // sequence after the command goes out
        let handle = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(15));
            m.process_feedback_frame(ack_frame(1)).expect("process");
        });
        let r = motor.send_with_ack(&[0x05, 0xB3, 0x02, 0, 0], 5, Duration::from_millis(500));
        handle.join().expect("thread join");
        assert!(r.is_ok(), "send_with_ack should succeed on ACK: {:?}", r);
        let sent = bus.sent.lock().expect("sent frames");
        assert_eq!(sent.len(), 1);
        assert_eq!(sent[0].arbitration_id, 0x8000 | 1);
        assert_eq!(&sent[0].data[..5], &[0x05, 0xB3, 0x02, 0, 0]);
    }

    #[test]
    fn send_with_ack_times_out_without_ack() {
        let (_bus, motor) = make_motor(1);
        let r = motor.send_with_ack(&[0x05, 0xB3, 0x02, 0, 0], 5, Duration::from_millis(30));
        assert!(
            matches!(r, Err(MotorError::Timeout(_))),
            "expected timeout, got {r:?}"
        );
    }

    /// Command 8 (read state) is a special case of command 7 (generic
    /// register read): the register-read primitive sends exactly the
    /// protocol doc's canonical `17 01` query frame (extended 29-bit
    /// 0x8000|id, dlc 2), and returns Ok immediately even with no reply —
    /// after request_motor_feedback_async was deleted, this path carries
    /// the "send-and-return" non-blocking state query.
    #[test]
    fn state_read_via_register_request_sends_canonical_frame() {
        let (bus, motor) = make_motor(1);
        // Push no reply frame: the blocking read_registers would time
        // out here; request (send-only) should return Ok immediately
        motor
            .request_registers(&[ReadCmd {
                addr: 0x01,
                count: 3,
            }])
            .expect("state read request send");
        let sent = bus.sent.lock().expect("sent frames");
        assert_eq!(sent.len(), 1);
        assert_eq!(sent[0].arbitration_id, 0x8000 | 1);
        assert!(sent[0].is_extended, "query frame must be extended 29-bit");
        assert_eq!(sent[0].dlc, 2);
        assert_eq!(&sent[0].data[..2], &[0x17, 0x01]);
    }

    /// Full closed loop of send-and-return + the background passive
    /// feedback path: after the register read request (i.e. the 17 01
    /// state query) goes out, the reply frame is fed through
    /// process_feedback_frame (the background polling thread's entry)
    /// into both the state cache and the register cache — latest_state
    /// and cached_register read the same frame.
    #[test]
    fn state_read_request_reply_feeds_both_caches_via_passive_path() {
        let (_bus, motor) = make_motor(1);
        assert!(motor.latest_state().is_none());
        motor
            .request_registers(&[ReadCmd {
                addr: 0x01,
                count: 3,
            }])
            .expect("state read request send");
        // Simulate the CoreController background thread receiving the
        // 27 01 reply (id<<8 arbitration ID)
        let reply = CanFrame {
            arbitration_id: 0x0100,
            data: [0x27, 0x01, 0x34, 0x12, 0x78, 0x56, 0x00, 0x0A],
            dlc: 8,
            is_extended: false,
            is_rx: true,
        };
        motor.process_feedback_frame(reply).expect("process");
        let st = motor.latest_state().expect("state should be cached");
        assert_eq!(st.can_id, 1);
        // pos_raw=0x1234=4660 → 4660*0.0001*2π
        assert!((st.pos - 0.4660 * std::f32::consts::TAU).abs() < 1e-4);
        // The same frame enters the register cache by address; reading
        // it back yields the converted physical value (same formula as
        // the state path)
        assert!(
            (motor.cached_register(0x01).unwrap() - 0x1234 as f32 * 0.0001 * std::f32::consts::TAU)
                .abs()
                < 1e-4
        );
        assert!(
            (motor.cached_register(0x02).unwrap() - 0x5678 as f32 / 4000.0 * std::f32::consts::TAU)
                .abs()
                < 1e-4
        );
        // tqe_raw=0x0A00=2560 → 2560*0.01*k (General k=0.5) = 12.8
        assert!((motor.cached_register(0x03).unwrap() - 12.8).abs() < 1e-4);
    }

    /// Convergence regression (2026-09-30): when the CoreController
    /// background receive thread feeds the 27 01 reply directly into
    /// process_feedback_frame (the bus frame is consumed by it, the
    /// active querying side never sees it), the blocking
    /// request_fresh_state must still return successfully.
    #[test]
    fn request_fresh_state_ok_when_worker_consumes_reply() {
        let (_bus, motor) = make_motor(1);
        let motor2 = Arc::clone(&motor);
        let worker = std::thread::spawn(move || {
            // Simulate the background thread receiving the reply while
            // the active query is waiting (the reply never enters
            // MockBus's rx queue, so the active side never receives it).
            std::thread::sleep(Duration::from_millis(30));
            let reply = CanFrame {
                arbitration_id: 0x0100,
                data: [0x27, 0x01, 0x34, 0x12, 0x78, 0x56, 0x00, 0x0A],
                dlc: 8,
                is_extended: false,
                is_rx: true,
            };
            motor2.process_feedback_frame(reply).expect("process");
        });
        let s = motor
            .request_fresh_state(Duration::from_millis(500))
            .expect("worker-consumed reply should still satisfy freshness wait");
        worker.join().expect("worker join");
        assert_eq!(s.can_id, 1);
    }

    /// B1 regression (2026-10-21): request_motor_feedback (DM-aligned
    /// non-blocking semantics) sends exactly the protocol-canonical
    /// 2-byte frame (extended 29-bit 0x8000|id) — the old blocking
    /// method used to send an 8-byte zero-padded frame, inconsistent
    /// with the doc.
    /// 2026-10-24: extended to **two frames sent back-to-back**
    /// (`17 01` pos/vel/tq + `16 0E` temperature/fault code) — real-bus
    /// testing showed the firmware parses one pair per frame and silently
    /// drops packed multi-pair frames.
    #[test]
    fn request_motor_feedback_sends_canonical_two_byte_frame() {
        let (bus, motor) = make_motor(1);
        motor.request_motor_feedback().expect("send");
        let sent = bus.sent.lock().expect("sent frames");
        assert_eq!(sent.len(), 2, "两帧:17 01 + 16 0E");
        for f in sent.iter() {
            assert_eq!(f.arbitration_id, 0x8000 | 1);
            assert!(f.is_extended, "查询帧应为扩展 29 位");
            assert_eq!(f.dlc, 2, "每帧 2 字节规范 dlc");
        }
        assert_eq!(&sent[0].data[..2], &[0x17, 0x01]);
        assert_eq!(&sent[1].data[..2], &[0x16, 0x0E]);
    }

    /// 2026-10-24: multiple ReadCmds are sent one frame per command (one
    /// pair per frame), with the whole span validated first.
    #[test]
    fn request_registers_sends_one_frame_per_cmd() {
        let (bus, motor) = make_motor(1);
        motor
            .request_registers(&[
                ReadCmd {
                    addr: 0x01,
                    count: 3,
                },
                ReadCmd {
                    addr: 0x0E,
                    count: 2,
                },
            ])
            .expect("send");
        let sent = bus.sent.lock().expect("sent frames");
        assert_eq!(sent.len(), 2, "两条命令两帧,不拼帧");
        assert_eq!(&sent[0].data[..2], &[0x17, 0x01]);
        assert_eq!(&sent[1].data[..2], &[0x16, 0x0E]);
    }

    /// 2026-10-24 two-frame state counting (core semantics): the
    /// `26 0E` reply frame **patches** temperature/fault code in place
    /// into the state cache — the sequence does not advance (one
    /// request_feedback cycle counts once), field values come from the
    /// patch frame; the register cache still ingests as usual
    /// (independent keys and timestamps, read_registers freshness
    /// claims unaffected).
    #[test]
    fn temp_error_reply_patches_state_without_bumping_count() {
        let (_bus, motor) = make_motor(1);
        // Anchor frame 27 01: pos/vel/tqe + one state count
        let anchor = CanFrame {
            arbitration_id: 0x0100,
            data: [0x27, 0x01, 0x34, 0x12, 0x78, 0x56, 0x00, 0x0A],
            dlc: 8,
            is_extended: false,
            is_rx: true,
        };
        motor
            .process_feedback_frame(anchor)
            .expect("process anchor");
        let after_anchor = motor.latest_state_snapshot().expect("snapshot");
        assert_eq!(after_anchor.sequence, 1);
        assert_eq!(after_anchor.state.t_mos, 0.0);
        assert_eq!(after_anchor.state.fault_code, 0);

        // Patch frame 26 0E: temperature 0x0142=322→32.2°C, fault code 5
        let patch = CanFrame {
            arbitration_id: 0x0100,
            data: [0x26, 0x0E, 0x42, 0x01, 0x05, 0x00, 0x00, 0x00],
            dlc: 6,
            is_extended: false,
            is_rx: true,
        };
        motor.process_feedback_frame(patch).expect("process patch");
        let after_patch = motor.latest_state_snapshot().expect("snapshot");
        // The two reply frames merge into one state count: sequence
        // stays put
        assert_eq!(after_patch.sequence, 1, "补丁不推进序号");
        assert_eq!(
            after_patch.received_at, after_anchor.received_at,
            "补丁不刷新时间戳(新鲜度由锚定帧定义)"
        );
        assert_eq!(after_patch.state.pos, after_anchor.state.pos);
        assert!(
            (after_patch.state.t_mos - 32.2).abs() < 1e-4,
            "0x0142 × 0.1°C"
        );
        assert_eq!(after_patch.state.fault_code, 5, "0x0F 1:1 桥接");
        // Register cache unchanged: temperature / fault code can be
        // read back independently (physical value / 1:1)
        assert!((motor.cached_register(0x0E).unwrap() - 32.2).abs() < 1e-4);
        assert_eq!(motor.cached_register(0x0F).unwrap(), 5.0);
    }

    /// 2026-10-24: a standalone mode read (`25 00` reply) also patches
    /// status_code via the bridge, without advancing the sequence; before
    /// the anchor frame arrives the patch is dropped (silently, no
    /// snapshot created).
    #[test]
    fn mode_reply_patches_status_code_without_bumping_count() {
        let (_bus, motor) = make_motor(1);
        // First (anchor) frame not yet arrived: the patch has nowhere to land
        let orphan = CanFrame {
            arbitration_id: 0x0100,
            data: [0x25, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00],
            dlc: 4,
            is_extended: false,
            is_rx: true,
        };
        motor
            .process_feedback_frame(orphan)
            .expect("process orphan");
        assert!(motor.latest_state().is_none(), "补丁不创建快照");

        // After the anchor frame, the patch takes effect
        let anchor = CanFrame {
            arbitration_id: 0x0100,
            data: [0x27, 0x01, 0, 0, 0, 0, 0, 0],
            dlc: 8,
            is_extended: false,
            is_rx: true,
        };
        motor
            .process_feedback_frame(anchor)
            .expect("process anchor");
        motor
            .process_feedback_frame(orphan)
            .expect("process mode patch");
        let st = motor.latest_state_snapshot().expect("snapshot");
        assert_eq!(st.sequence, 1, "补丁仍不推进序号");
        assert_eq!(st.state.status_code, 7, "0x00 模式 1:1 桥接");
    }

    #[test]
    fn accepts_frame_admits_status_register_and_ack() {
        let (_bus, motor) = make_motor(1);
        // 8-byte status frame
        let status = CanFrame {
            arbitration_id: 0x0100,
            data: [0x0A, 0x00, 0x34, 0x12, 0x09, 0x00, 0x00, 0x00],
            dlc: 8,
            is_extended: true,
            is_rx: true,
        };
        assert!(motor.accepts_frame(&status));
        // 7-byte ACK
        assert!(motor.accepts_frame(&ack_frame(1)));
        // 5-byte Int8×3 register reply (cmd=reply|Int8|count3, 2+3=5
        // bytes): after the version query was removed, 5-byte frames go
        // through the register-decode branch instead — the old
        // implementation's `5 => decode_version_reply` would reject this
        // frame, so in Background mode the Int8×3 consecutive-read reply
        // was never claimed (regression guard).
        let int8x3 = CanFrame {
            arbitration_id: 0x0100,
            data: [0x23, 0x22, 0x11, 0x22, 0x33, 0, 0, 0],
            dlc: 5,
            is_extended: true,
            is_rx: true,
        };
        assert!(motor.accepts_frame(&int8x3));
        motor.process_feedback_frame(int8x3).expect("process");
        // Foreign Int8 cache entries are read through the generalized
        // conversion (2026-10-22):
        // 0x22 feedforward torque I8(17) → 17/2×k (0.5) = 4.25 N·m;
        // 0x23 Kp proportional I8(0x22=34) → 34/32767 normalized.
        assert!((motor.cached_register(0x22).unwrap() - 4.25).abs() < 1e-4);
        assert!((motor.cached_register(0x23).unwrap() - 34.0 / 32767.0).abs() < 1e-6);
        // The deleted version-reply frame (15 B5 02.., dlc 5): first byte
        // 0x15 is not REPLY_NIBBLE, register decode rejects it, and the
        // passive path still does not claim it.
        assert!(!motor.accepts_frame(&CanFrame {
            arbitration_id: 0x0100,
            data: [0x15, 0xB5, 0x02, 0x12, 0x34, 0, 0, 0],
            dlc: 5,
            is_extended: true,
            is_rx: true,
        }));
        // Frames of other motors are not accepted
        assert!(!motor.accepts_frame(&ack_frame(2)));
        // 3-byte short frames are not accepted
        let short = CanFrame {
            arbitration_id: 0x0100,
            data: [0x01, 0x00, 0x00, 0, 0, 0, 0, 0],
            dlc: 3,
            is_extended: true,
            is_rx: true,
        };
        assert!(!motor.accepts_frame(&short));
    }

    #[test]
    fn ensure_control_mode_accepts_0_through_17() {
        let (_bus, motor) = make_motor(1);
        for mode in 0..=17 {
            assert!(
                motor.ensure_control_mode(mode, Duration::ZERO).is_ok(),
                "mode {mode} should be accepted"
            );
        }
    }

    #[test]
    fn ensure_control_mode_rejects_above_17() {
        let (_bus, motor) = make_motor(1);
        for mode in [18, 19, 100, u32::MAX] {
            assert!(
                motor.ensure_control_mode(mode, Duration::ZERO).is_err(),
                "mode {mode} should be rejected"
            );
        }
    }

    /// 2026-10-09: control commands default to **standard 11-bit
    /// bare-id frames** (the v2.0.0 reference firmware's original
    /// behavior). The 2026-09-29 extended `0x8000|id` + bit15 form was
    /// falsified by the candump real-bus test and reverted — bit15 on a
    /// non-query command only buys one DLC=0 empty frame, no status
    /// data.
    #[test]
    fn control_commands_default_to_standard_bare_id_frames() {
        let (bus, motor) = make_motor(1);
        // Velocity command (0x07 0x07)
        motor.send_cmd_vel(1.0).expect("vel");
        // Coordinated pos-vel (0x07 0x35)
        motor.send_cmd_pos_vel(0.5, 1.0).expect("pos_vel");
        // Trapezoid (0x07 0x2d)
        motor
            .send_cmd_pos_vel_acc(0.5, 1.0, 0.5)
            .expect("pos_vel_acc");
        // Stop (0x01 0x00 0x00)
        motor.send_stop().expect("stop");
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 4);
        for f in sent.iter() {
            assert!(!f.is_extended, "控制帧默认应为标准 11 位");
            assert_eq!(f.arbitration_id, 1, "控制帧 ID 应为裸 motor_id");
            assert!(!f.is_rx);
        }
    }

    /// set_control_reply(true): switches to the PDF §1.4 example form —
    /// extended 29-bit `0x8000|id` + bit15. Real-bus behavior: the motor
    /// replies to non-query commands with one DLC=0 empty frame (§1.2),
    /// no status data; diagnostics/reproduction only.
    #[test]
    fn control_commands_reply_mode_uses_extended_frames() {
        let (bus, motor) = make_motor(1);
        motor.set_control_reply(true);
        motor.send_cmd_pos_vel(0.5, 1.0).expect("pos_vel");
        motor.send_stop().expect("stop");
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 2);
        for f in sent.iter() {
            assert!(f.is_extended, "reply 控制帧应为扩展 29 位");
            assert_eq!(
                f.arbitration_id, 0x8001,
                "reply 控制帧 ID 应为 0x8000|motor_id"
            );
            assert!(!f.is_rx);
        }
    }

    /// G3: clear_error and disable are the same frame (`0x01 0x00 0x00`,
    /// table-2 mode 0 "stop, clear errors"); the protocol has no
    /// separate clear-error command.
    #[test]
    fn clear_error_sends_stop_frame() {
        let (bus, motor) = make_motor(1);
        motor.clear_error().expect("clear_error");
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 1);
        let f = &sent[0];
        assert!(!f.is_extended, "清错帧默认应为标准 11 位(参考固件原行为)");
        assert_eq!(f.arbitration_id, 1, "清错帧 ID 应为裸 motor_id");
        assert_eq!(f.dlc, 3);
        assert_eq!(&f.data[..3], &[0x01, 0x00, 0x00]);
    }

    /// P2-12/13: query/config commands use extended 29-bit frames,
    /// ID=0x8000|motor_id, with the reply bit set.
    #[test]
    fn query_commands_use_extended_frame_with_reply_bit() {
        let (bus, motor) = make_motor(1);
        // Register read (the read command derived from 0x17 0x01)
        motor
            .read_registers(
                &[ReadCmd {
                    addr: 0x01,
                    count: 3,
                }],
                Duration::from_millis(10),
            )
            .ok();
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 1);
        for f in sent.iter() {
            assert!(f.is_extended, "查询帧应为扩展 29 位");
            assert_eq!(f.arbitration_id, 0x8001, "查询帧 ID 应为 0x8000|motor_id");
            assert!(!f.is_rx);
        }
    }

    /// Regression: read_registers takes the "send + poll fresh cache"
    /// path, with replies entering the cache via the passive path (the
    /// background thread entry process_feedback_frame). While waiting,
    /// register replies of other transactions (the `0x27 0x01` of a
    /// concurrent `17 01` state query, addresses 0x01-0x03) enter their
    /// own cache keys by address and do not satisfy the 0x0E request;
    /// the call returns its value only after the 0x0E reply is cached.
    #[test]
    fn read_registers_waits_for_fresh_reply_at_requested_address() {
        let (_bus, motor) = make_motor(1);
        let m = motor.clone();
        // Simulate the background polling thread: 10ms after the request
        // goes out, first deliver someone else's state reply, then the
        // 0x0E temperature reply this transaction asked for
        let handle = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(10));
            m.process_feedback_frame(CanFrame {
                arbitration_id: 0x0100,
                data: [0x27, 0x01, 0x34, 0x12, 0x09, 0x00, 0x00, 0x00],
                dlc: 8,
                is_extended: true,
                is_rx: true,
            })
            .expect("merge state reply");
            m.process_feedback_frame(CanFrame {
                arbitration_id: 0x0100,
                data: [0x25, 0x0E, 0x64, 0x00, 0, 0, 0, 0],
                dlc: 4,
                is_extended: true,
                is_rx: true,
            })
            .expect("merge temp reply");
        });
        let vals = motor
            .read_registers(
                &[ReadCmd {
                    addr: 0x0E,
                    count: 1,
                }],
                Duration::from_millis(500),
            )
            .expect("应等到 0x0E 的新鲜回复");
        handle.join().expect("thread join");
        assert_eq!(vals.len(), 1);
        assert_eq!(vals[0].0, 0x0E, "返回的应是地址 0x0E 的值");
        // I16 raw 100 → temperature 0.1°C/LSB → 10.0°C (converted
        // physical value)
        assert!((vals[0].1 - 10.0).abs() < 1e-4);
    }

    /// An old value cached before the request does not satisfy the
    /// freshness check (ts < request_at) and cannot be claimed by this
    /// read_registers — the timestamp is the transaction id.
    #[test]
    fn read_registers_rejects_stale_cached_value() {
        let (_bus, motor) = make_motor(1);
        // First cache an old 0x0E value, then issue the read: the old
        // value predates request_at
        motor
            .process_feedback_frame(CanFrame {
                arbitration_id: 0x0100,
                data: [0x25, 0x0E, 0x64, 0x00, 0, 0, 0, 0],
                dlc: 4,
                is_extended: true,
                is_rx: true,
            })
            .expect("merge old reply");
        let r = motor.read_registers(
            &[ReadCmd {
                addr: 0x0E,
                count: 1,
            }],
            Duration::from_millis(30),
        );
        assert!(
            matches!(r, Err(MotorError::Timeout(_))),
            "旧缓存值不应被认领, 应超时: {r:?}"
        );
    }

    /// Regression (A3, 2026-10-21; rewritten 2026-10-22 after reads were
    /// fixed to Int16): cache entries of different types at the same
    /// address do not pollute each other. The request claims strictly by
    /// the Int16 key `(addr, Int16)` — a foreign Int8 reply arriving
    /// first (a 0x21 frame sent by another host, cached under the Int8
    /// key) is newer and at the same address, but the key does not match
    /// and it is never claimed by this transaction; only when our own
    /// Int16 reply (0x25 frame) arrives 40ms later does the call return.
    /// The old implementation (address-only key) would claim the Int8
    /// frame on arrival, returning the foreign-path converted value
    /// 1.0°C (0x0A=10, /10) instead of this transaction's 10.0°C.
    #[test]
    fn read_registers_rejects_fresh_reply_of_different_type() {
        let (_bus, motor) = make_motor(1);
        let m = motor.clone();
        let handle = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(10));
            // Arriving first after the request goes out: a foreign-host
            // Int8 read of 0x0E reply (0x21, dlc 3).
            // 40ms gap to the Int16 reply (>> 2ms poll interval)
            // guarantees that when the read loop polls at least once,
            // only the Int8 entry is in the cache — the old
            // implementation (address-only key) would claim the I8 at
            // that moment and return, making the test discriminative;
            // if the two frames were back-to-back, the old
            // "last writer wins" would coincidentally return the I16
            // and the test would lose discriminating power.
            m.process_feedback_frame(CanFrame {
                arbitration_id: 0x0100,
                data: [0x21, 0x0E, 0x0A, 0, 0, 0, 0, 0],
                dlc: 3,
                is_extended: true,
                is_rx: true,
            })
            .expect("merge int8 reply");
            std::thread::sleep(Duration::from_millis(40));
            // Then: this transaction's (fixed Int16) reply for 0x0E
            // (0x25, dlc 4)
            m.process_feedback_frame(CanFrame {
                arbitration_id: 0x0100,
                data: [0x25, 0x0E, 0x64, 0x00, 0, 0, 0, 0],
                dlc: 4,
                is_extended: true,
                is_rx: true,
            })
            .expect("merge int16 reply");
        });
        let vals = motor
            .read_registers(
                &[ReadCmd {
                    addr: 0x0E,
                    count: 1,
                }],
                Duration::from_millis(500),
            )
            .expect("应等到 Int16 回复, 而非认领外来 Int8 缓存条目");
        handle.join().expect("thread join");
        assert_eq!(vals.len(), 1);
        assert_eq!(vals[0].0, 0x0E);
        // I16 raw 100 → 10.0°C (temperature conversion); the foreign I8
        // entry (0x0A=10) would instead convert to 1.0°C (Int8
        // temperature is also 0.1°C/LSB) — sufficient discrimination.
        assert!(
            (vals[0].1 - 10.0).abs() < 1e-4,
            "应返回 Int16 路径的 10.0°C, 实得 {:?}(旧实现会错拿外来 Int8 条目)",
            vals[0].1
        );
    }

    /// Regression (B3, 2026-10-21; rewritten after the 2026-10-22
    /// follow-up): gap/out-of-table addresses are rejected — if any
    /// address covered by the read span is outside the register table,
    /// the **whole span errors with zero frames sent**. The table's
    /// highest address is 0x45, so a legal request never crosses 0xFF;
    /// the address wrap of a consecutive read starting at 0xFF (the
    /// original B3 bug) is unreachable on our own request path, and the
    /// wrap semantics are kept only for the decode side handling
    /// foreign frames.
    #[test]
    fn read_registers_rejects_unknown_addr_span() {
        let (_bus, motor) = make_motor(1);
        // Out-of-table single address (0xFF)
        let r = motor.read_registers(
            &[ReadCmd {
                addr: 0xFF,
                count: 1,
            }],
            Duration::from_millis(30),
        );
        assert!(
            matches!(r, Err(MotorError::InvalidArgument(ref e)) if e.contains("0xFF")),
            "表外地址应整段拒绝: {r:?}"
        );
        // Gap address (0x0A, a hole inside 0x00-0x45)
        let r = motor.read_registers(
            &[ReadCmd {
                addr: 0x0A,
                count: 1,
            }],
            Duration::from_millis(30),
        );
        assert!(
            matches!(r, Err(MotorError::InvalidArgument(ref e)) if e.contains("0x0A")),
            "缺号地址应拒绝: {r:?}"
        );
        // Consecutive read stepping into a gap: 0x2D (Ki, in table)
        // count2 covers 0x2E (gap) → whole span rejected
        let r = motor.read_registers(
            &[ReadCmd {
                addr: 0x2D,
                count: 2,
            }],
            Duration::from_millis(30),
        );
        assert!(
            matches!(r, Err(MotorError::InvalidArgument(ref e)) if e.contains("0x2E")),
            "span 覆盖缺号应整段拒绝: {r:?}"
        );
        // In-table consecutive read passes as usual (it will time out,
        // but the frame has been sent)
        let (bus, motor) = make_motor(1);
        let _ = motor.read_registers(
            &[ReadCmd {
                addr: 0x01,
                count: 3,
            }],
            Duration::from_millis(10),
        );
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(
            sent.len(),
            1,
            "in-table addresses should send frames normally"
        );
        // The write path likewise rejects out-of-table
        let (bus, motor) = make_motor(1);
        let r = motor.write_register(0x50, 0, false);
        assert!(
            matches!(r, Err(MotorError::InvalidArgument(ref e)) if e.contains("0x50")),
            "写表外地址应拒绝: {r:?}"
        );
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 0, "拒绝发生在发帧之前");
    }

    /// 2026-10-22 decision guard: for the same `27 01` reply frame, the
    /// converted f32 values returned by `read_registers` (0x01-0x03,
    /// fixed Int16) must match `latest_state()`'s pos/vel/torq bit for
    /// bit — the register-read path and the state path must always stay
    /// conversion-aligned (this test is the backstop when either path
    /// changes its scales/coefficients).
    #[test]
    fn read_registers_pos_matches_state_path() {
        let (_bus, motor) = make_motor(1);
        let m = motor.clone();
        let handle = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(10));
            m.process_feedback_frame(CanFrame {
                arbitration_id: 0x0100,
                data: [0x27, 0x01, 0x34, 0x12, 0x78, 0x56, 0x00, 0x0A],
                dlc: 8,
                is_extended: false,
                is_rx: true,
            })
            .expect("merge state reply");
        });
        let vals = motor
            .read_registers(
                &[ReadCmd {
                    addr: 0x01,
                    count: 3,
                }],
                Duration::from_millis(500),
            )
            .expect("应等到状态回复");
        handle.join().expect("thread join");
        let st = motor.latest_state().expect("state cache");
        assert_eq!(vals.len(), 3);
        assert!(
            (vals[0].1 - st.pos).abs() < 1e-5,
            "pos 分歧: {:?} vs {st:?}",
            vals[0].1
        );
        assert!(
            (vals[1].1 - st.vel).abs() < 1e-5,
            "vel 分歧: {:?} vs {st:?}",
            vals[1].1
        );
        assert!(
            (vals[2].1 - st.torq).abs() < 1e-5,
            "torq 分歧: {:?} vs {st:?}",
            vals[2].1
        );
    }

    /// 0x0F (fault code) has no conversion → returned 1:1 as the raw
    /// value.
    #[test]
    fn read_registers_identity_for_0x0f() {
        let (_bus, motor) = make_motor(1);
        let m = motor.clone();
        let handle = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(10));
            m.process_feedback_frame(CanFrame {
                arbitration_id: 0x0100,
                data: [0x25, 0x0F, 0x20, 0x00, 0, 0, 0, 0],
                dlc: 4,
                is_extended: true,
                is_rx: true,
            })
            .expect("merge err reply");
        });
        let vals = motor
            .read_registers(
                &[ReadCmd {
                    addr: 0x0F,
                    count: 1,
                }],
                Duration::from_millis(500),
            )
            .expect("应等到 0x0F 回复");
        handle.join().expect("thread join");
        assert_eq!(vals.len(), 1);
        assert_eq!(vals[0].0, 0x0F);
        assert_eq!(vals[0].1, 32.0, "无换算关系的寄存器应 1:1 返回 raw");
    }

    /// Non-blocking path: request_registers sends and returns
    /// immediately (frame format/extended id correct), cached_register
    /// is None before the reply is cached and readable after; and
    /// accepts_frame claims 4-byte register replies (previously that
    /// length was rejected and dropped).
    #[test]
    fn request_registers_then_cached_register_via_passive_path() {
        let (bus, motor) = make_motor(1);
        motor
            .request_registers(&[ReadCmd {
                addr: 0x0E,
                count: 1,
            }])
            .expect("send query");
        let sent = bus.sent.lock().expect("sent frames");
        assert_eq!(sent.len(), 1, "request_registers 只发一帧不等回帧");
        assert_eq!(sent[0].arbitration_id, 0x8000 | 1, "查询帧应为扩展 29 位");
        assert_eq!(sent[0].dlc, 2, "int16 count1 读请求载荷 2 字节");
        assert_eq!(
            &sent[0].data[..2],
            &[0x15, 0x0E],
            "cmd=0x15(int16 count1) addr=0x0E"
        );
        drop(sent);

        let reply = CanFrame {
            arbitration_id: 0x0100,
            data: [0x25, 0x0E, 0x64, 0x00, 0, 0, 0, 0],
            dlc: 4,
            is_extended: true,
            is_rx: true,
        };
        assert!(
            MotorDevice::accepts_frame(&*motor, &reply),
            "4 字节寄存器回复应被被动路径认领"
        );
        assert!(motor.cached_register(0x0E).is_none(), "入缓前应无值");
        motor.process_feedback_frame(reply).expect("merge");
        // I16 raw 100 → 10.0°C (read back as the converted physical
        // value)
        assert!((motor.cached_register(0x0E).unwrap() - 10.0).abs() < 1e-4);
    }

    /// The `0x27 0x01` status reply feeds both the state cache and the
    /// register cache (a state query is itself a register read of
    /// 0x01-0x03); both latest_state and cached_register are readable.
    #[test]
    fn state_reply_feeds_both_state_and_register_cache() {
        let (_bus, motor) = make_motor(1);
        motor
            .process_feedback_frame(CanFrame {
                arbitration_id: 0x0100,
                data: [0x27, 0x01, 0x34, 0x12, 0x09, 0x00, 0x00, 0x00],
                dlc: 8,
                is_extended: true,
                is_rx: true,
            })
            .expect("merge");
        let st = motor.latest_state().expect("state cache");
        assert!((st.pos - (0x1234 as f32 * 0.0001 * std::f32::consts::TAU)).abs() < 1e-6);
        // Read back as the converted physical value, same formula as
        // the state path.
        assert!(
            (motor.cached_register(0x01).unwrap() - 0x1234 as f32 * 0.0001 * std::f32::consts::TAU)
                .abs()
                < 1e-6
        );
        // I16(0) → 0×0.01×k+d = 0.0
        assert!((motor.cached_register(0x03).unwrap() - 0.0).abs() < 1e-6);
    }

    /// 2026-10-23 decision regression: fields not carried by a frame do
    /// not overwrite the state cache — the 8-byte status frame
    /// (byte0=mode, byte1=fault, §3.2 format) first brings fault=0x22,
    /// then the `27 01` read reply only has pos/vel/tqe; fault code/run
    /// mode should keep their old values (the old implementation zeroed
    /// them, so one state query wiped them). Note: on the v2.0.0 real
    /// bus this status frame has no known producer (control commands
    /// reply only with a DLC=0 empty frame, see can_protocol §3.2
    /// correction/§1.2) — this is a constructed frame testing the merge
    /// logic; fields present in the frame (pos) still update as usual.
    #[test]
    fn read_reply_preserves_cached_fault_code() {
        let (_bus, motor) = make_motor(1);
        // Status frame attached to a control command: byte0=mode (0x0A
        // position), byte1=fault (0x22).
        motor
            .process_feedback_frame(CanFrame {
                arbitration_id: 0x0100,
                data: [0x0A, 0x22, 0x34, 0x12, 0x78, 0x56, 0x00, 0x0A],
                dlc: 8,
                is_extended: true,
                is_rx: true,
            })
            .expect("merge status frame");
        let st = motor.latest_state().expect("state cache");
        assert_eq!(st.status_code, 0x0A);
        assert_eq!(st.fault_code, 0x22);

        // Followed by the 27 01 read reply (no mode/fault bytes).
        motor
            .process_feedback_frame(CanFrame {
                arbitration_id: 0x0100,
                data: [0x27, 0x01, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00],
                dlc: 8,
                is_extended: true,
                is_rx: true,
            })
            .expect("merge read reply");
        let st = motor.latest_state().expect("state cache");
        assert_eq!(st.status_code, 0x0A, "27 01 不携带运行模式, 应保持旧值");
        assert_eq!(st.fault_code, 0x22, "27 01 不携带故障码, 应保持旧值");
        // Fields present in the frame still update: pos raw 0x0009 →
        // 9×0.0001 turns×2π.
        assert!((st.pos - (9.0 * 0.0001 * std::f32::consts::TAU)).abs() < 1e-6);

        // Reverse direction: a new control status frame refreshes the
        // fault code as usual (mode 0/fault 0 = the motor's normal reply
        // after stop-and-clear-errors).
        motor
            .process_feedback_frame(CanFrame {
                arbitration_id: 0x0100,
                data: [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
                dlc: 8,
                is_extended: true,
                is_rx: true,
            })
            .expect("merge clearing frame");
        let st = motor.latest_state().expect("state cache");
        assert_eq!(st.status_code, 0x00);
        assert_eq!(st.fault_code, 0x00, "控制状态帧携带故障码, 照常刷新");
    }

    /// P2-14: classic position mode `0x07 0x07` byte layout, torque
    /// None → 0x8000 (unlimited), standard 11-bit frame. Compare the
    /// reference firmware `motor_control_pos` template
    /// `[0x07,0x07,pos@2-3,0x00,0x00,tqe@6-7]`.
    #[test]
    fn send_cmd_pos_classic_byte_layout() {
        let (bus, motor) = make_motor(1);
        // Target position 0.5 turns → raw = 0.5 * 10000 = 5000 = 0x1388
        motor
            .send_cmd_pos_classic(0.5 * std::f32::consts::TAU, None)
            .expect("pos_classic");
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 1);
        let f = &sent[0];
        assert!(!f.is_extended, "普通位置帧默认应为标准 11 位");
        assert_eq!(f.arbitration_id, 1);
        assert_eq!(&f.data[0..2], &[0x07, 0x07]);
        // pos = 5000 LE
        assert_eq!(i16::from_le_bytes([f.data[2], f.data[3]]), 5000);
        // bytes 4-5 fixed 0x0000
        assert_eq!(&f.data[4..6], &[0x00, 0x00]);
        // tqe = 0x8000 (unlimited)
        assert_eq!(i16::from_le_bytes([f.data[6], f.data[7]]), i16::MIN);
    }

    /// P2-14 + P2-15: with a given torque, classic pos's tqe field =
    /// tqe_adjust_to_raw(nm, coeff).
    /// Test motor model="ht-test" → General (k=0.5), so 1.0 Nm →
    /// ((1.0/0.5)*100) = 200.
    #[test]
    fn send_cmd_pos_classic_with_torque_limit() {
        let (bus, motor) = make_motor(1);
        assert_eq!(motor.torque_coeff().k, 0.5, "测试电机应为 General k=0.5");
        // 1.0 Nm → raw = 200 = 0x00C8
        motor
            .send_cmd_pos_classic(0.0, Some(1.0))
            .expect("pos_classic");
        let f = &bus.sent.lock().expect("sent")[0];
        assert_eq!(i16::from_le_bytes([f.data[6], f.data[7]]), 200);
    }

    /// Torque mode (protocol command 5): `05 13 <tqe16>` 4 bytes;
    /// 1.1 Nm × General (k=0.5) → ((1.1/0.5)*100) = 220 = 0x00DC.
    #[test]
    fn send_cmd_torque_byte_layout() {
        let (bus, motor) = make_motor(1);
        motor.send_cmd_torque(1.1).expect("torque");
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 1);
        let f = &sent[0];
        assert_eq!(f.dlc, 4);
        assert_eq!(&f.data[..4], &[0x05, 0x13, 0xDC, 0x00]);
        assert!(!f.is_extended, "力矩帧默认走控制通道(标准裸 id)");
        assert_eq!(f.arbitration_id, 1);
    }

    /// enable is a semantic no-op (option B, 2026-10-08): returns Ok
    /// and sends no frame on the bus. The protocol has no enable frame
    /// and commands execute on receipt, so the postcondition always
    /// holds; clearing errors/stopping is done explicitly via
    /// clear_error()/disable(), keeping enable side-effect free. This
    /// contract also keeps mixed-bus enable_all from being poisoned by
    /// an Err abort.
    #[test]
    fn enable_is_frameless_noop() {
        let (bus, motor) = make_motor(1);
        motor.enable().expect("enable is a no-op success");
        assert!(
            bus.sent.lock().expect("sent").is_empty(),
            "enable must not send any frame"
        );
    }

    /// Run-mode switch (protocol command 6): `01 00 <mode>` 3 bytes,
    /// same shape as send_stop.
    #[test]
    fn send_run_mode_byte_layout() {
        let (bus, motor) = make_motor(1);
        motor.send_run_mode(5).expect("run mode");
        motor.send_run_mode(17).expect("run mode 17 保留值仍合法");
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 2);
        assert_eq!(&sent[0].data[..3], &[0x01, 0x00, 0x05]);
        assert_eq!(&sent[1].data[..3], &[0x01, 0x00, 0x11]);
        assert_eq!(sent[0].arbitration_id, 1);
        assert!(!sent[0].is_extended, "run_mode 走控制通道(标准裸 id)");
    }

    #[test]
    fn send_run_mode_rejects_above_17() {
        let (bus, motor) = make_motor(1);
        for mode in [18, 100, 255] {
            assert!(
                motor.send_run_mode(mode).is_err(),
                "mode {mode} should be rejected"
            );
        }
        // No frames sent on rejection
        assert!(bus.sent.lock().expect("sent").is_empty());
    }

    /// Brake = mode 15 (table 2).
    #[test]
    fn brake_sends_mode_15_frame() {
        let (bus, motor) = make_motor(1);
        motor.brake().expect("brake");
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 1);
        assert_eq!(&sent[0].data[..3], &[0x01, 0x00, 0x0F]);
        assert_eq!(sent[0].dlc, 3);
    }

    /// DQ voltage (protocol command 6): mode write + register write
    /// packed into one 7-byte frame, `01 00 08 05 1B <volt16>`;
    /// 1.0 V → 0.1 V/LSB → raw 10.
    #[test]
    fn send_cmd_dq_voltage_byte_layout() {
        let (bus, motor) = make_motor(1);
        motor.send_cmd_dq_voltage(1.0).expect("dq voltage");
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 1);
        let f = &sent[0];
        assert_eq!(f.dlc, 7);
        assert_eq!(&f.data[..7], &[0x01, 0x00, 0x08, 0x05, 0x1B, 0x0A, 0x00]);
    }

    /// DQ current (protocol command 6): `01 00 09 05 1C <cur16>`;
    /// 0.5 A → raw 5.
    #[test]
    fn send_cmd_dq_current_byte_layout() {
        let (bus, motor) = make_motor(1);
        motor.send_cmd_dq_current(0.5).expect("dq current");
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 1);
        let f = &sent[0];
        assert_eq!(f.dlc, 7);
        assert_eq!(&f.data[..7], &[0x01, 0x00, 0x09, 0x05, 0x1C, 0x05, 0x00]);
    }

    /// Generic register write (derived from protocol command 9):
    /// `05 <addr> <val16>` 4 bytes.
    /// Control-class goes through the control channel (default standard
    /// 11-bit bare id); config-class always goes through the extended
    /// `0x8000|id` query channel — the config class does not follow the
    /// control_reply frame-type switch.
    #[test]
    fn write_register_both_frame_classes() {
        let (bus, motor) = make_motor(1);
        // Control class (e.g. feedforward torque register 0x22):
        // default standard bare id
        motor
            .write_register(0x22, 500, false)
            .expect("control write");
        // Config class: still goes through the send_query extended
        // frame, even with control_reply enabled
        motor.set_control_reply(true);
        motor.write_register(0x30, 100, true).expect("config write");
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 2);
        assert_eq!(&sent[0].data[..4], &[0x05, 0x22, 0xF4, 0x01]);
        assert!(!sent[0].is_extended);
        assert_eq!(sent[0].arbitration_id, 1);
        assert_eq!(&sent[1].data[..4], &[0x05, 0x30, 0x64, 0x00]);
        assert!(sent[1].is_extended, "config 类写必须恒为扩展帧");
        assert_eq!(sent[1].arbitration_id, 0x8000 | 1);
    }

    /// Periodic report (protocol command 10): `05 B4 02 <t_ms16>`
    /// 5-byte extended frame; 0 = stop.
    #[test]
    fn set_periodic_report_byte_layout() {
        let (bus, motor) = make_motor(1);
        motor.set_periodic_report(10).expect("periodic on");
        motor.set_periodic_report(0).expect("periodic off");
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 2);
        for f in sent.iter() {
            assert_eq!(f.dlc, 5);
            assert!(f.is_extended, "周期上报配置帧应为扩展 0x8000|id");
            assert_eq!(f.arbitration_id, 0x8001);
        }
        assert_eq!(&sent[0].data[..5], &[0x05, 0xB4, 0x02, 0x0A, 0x00]);
        assert_eq!(&sent[1].data[..5], &[0x05, 0xB4, 0x02, 0x00, 0x00]);
    }

    // Merged from origin/main: verifies FeedbackCache semantics — only
    // decodable status frames advance the snapshot sequence and cached
    // time. Frame format adapted to v2.0.0 (reply id = motor_id << 8;
    // the bare-id layout in the original main test was the old protocol).
    #[test]
    fn only_decoded_status_advances_the_snapshot() {
        let bus: Arc<dyn CanBus> = Arc::new(MockBus::new());
        let motor = HightorqueMotor::new(1, 1, "hightorque", bus);

        motor
            .process_feedback_frame(CanFrame {
                arbitration_id: 0x0100,
                data: [0x27, 0x01, 0, 0, 0, 0, 0, 0],
                dlc: 8,
                is_extended: true,
                is_rx: true,
            })
            .expect("status feedback");
        let first = motor.latest_state_snapshot().expect("first snapshot");
        assert_eq!(first.sequence, 1);

        // Under v2.0.0 every 8-byte non-0x27 frame is a valid passive
        // status frame (the original main test's [0x01,..] layout no
        // longer holds), so use a short-frame register reply instead
        // (cmd 0x25 = reply + Int16 + count 1, dlc 4): enters the
        // register cache only, does not advance the state snapshot —
        // also verifying the separation of the two caches.
        motor
            .process_feedback_frame(CanFrame {
                arbitration_id: 0x0100,
                data: [0x25, 0x01, 0x34, 0x12, 0, 0, 0, 0],
                dlc: 4,
                is_extended: true,
                is_rx: true,
            })
            .expect("register reply frame");
        let after_non_state = motor.latest_state_snapshot().expect("snapshot remains");
        assert_eq!(after_non_state.sequence, first.sequence);
        assert_eq!(after_non_state.received_at, first.received_at);
        // Short frames should still enter the register cache, read back
        // as the converted physical value (0x01 raw 0x1234 → rad).
        assert!(
            (motor.cached_register(0x01).unwrap() - 0x1234 as f32 * 0.0001 * std::f32::consts::TAU)
                .abs()
                < 1e-6
        );
    }
}
