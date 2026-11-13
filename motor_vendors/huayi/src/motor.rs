//! `HuayiMotor` — vendor semantics layered on the shared CANopen node.
//!
//! Wire facts encoded here and nowhere else (manual references in each
//! item):
//!
//! - disable is `0x0000` (Disable Voltage) — the Huayi command table has no
//!   RobStride `0x0001` (§3.6),
//! - `6060h = 5` is MIT (§3.5 MIT),
//! - PP targets handshake on controlword bit 4 (§3.4),
//! - position preset only in Switch On Disabled (§3.6),
//! - feedback = TPDO2 anchor + TPDO1/TPDO3 patches (same anchor/patch
//!   model as hightorque's two-frame feedback, so `seq`/age semantics stay
//!   comparable across vendors).

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Mutex, OnceLock};
use std::time::{Duration, Instant};

use motor_canopen::cia402::{self, Cia402State};
use motor_canopen::classify::{is_emcy, is_tpdo, tpdo_owner};
use motor_canopen::node::{CanopenNode, NodeOutcome};
use motor_canopen::nmt::{send_nmt, NmtCommand};
use motor_canopen::objects::{
    rpdo_id, ERROR_CODE, MODES_OF_OPERATION_DISPLAY, POSITION_ACTUAL_VALUE, PROFILE_VELOCITY,
    STATUSWORD, TARGET_POSITION, TARGET_TORQUE, TARGET_VELOCITY,
};
use motor_canopen::send_std_frame;
use motor_core::bus::{CanBus, CanFrame};
use motor_core::device::MotorDevice;
use motor_core::error::{MotorError, Result};

use crate::objects::{error_code_description, od, MOTOR_PEAK_TORQUE, RunMode};
use crate::pdo as hpdo;
use crate::units::{self, MitRanges};

/// Cached feedback state, in unified SI units.
///
/// `seq` advances only on **anchor** events (TPDO2 position/velocity frame
/// or a `query_status` round trip); TPDO1/TPDO3/EMCY frames patch fields in
/// place without advancing it — same freshness semantics as the hightorque
/// two-frame feedback.
///
/// `torque_permille` is the wire-true Int16 value (‰ of the 6076h peak);
/// `torque_nm` is its Nm conversion and reads `0.0` until the peak torque
/// has been read at least once (`query_status`, `send_torque`, or
/// `configure_periodic_report` warm the cache).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HuayiStatus {
    /// Output-shaft position, rad.
    pub pos_rad: f32,
    /// Output-shaft velocity, rad/s.
    pub vel_rad_s: f32,
    /// Actual torque, Nm (0.0 until peak torque is cached — see above).
    pub torque_nm: f32,
    /// Actual torque, ‰ of peak (wire-true).
    pub torque_permille: i16,
    /// Driver temperature, °C (2204h_01, 0.1 °C units).
    pub t_driver_c: f32,
    /// Motor temperature, °C (2204h_02, 0.1 °C units).
    pub t_motor_c: f32,
    /// Raw `6041h`.
    pub statusword: u16,
    /// Raw `6061h` — **Huayi numbering** (5 = MIT).
    pub mode_display: i8,
    /// Last `603Fh` / EMCY error code.
    pub error_code: u16,
    pub received_at: Instant,
    pub seq: u64,
}

impl HuayiStatus {
    /// Manual §4.1 description for the current error code, if known.
    pub fn error_description(&self) -> Option<&'static str> {
        error_code_description(self.error_code)
    }
}

/// Profile-position new-target handshake state (controlword bit 4 level).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PpHandshake {
    /// Bit 4 assumed low (fresh power-up / after enable sequence): the next
    /// command only needs the set edge.
    Low,
    /// Bit 4 high: the next command must clear it first (1→0→1 toggle).
    High,
}

pub struct HuayiMotor {
    node: CanopenNode,
    feedback_id: u16,
    model: String,
    state: Mutex<Option<HuayiStatus>>,
    /// Profile-position handshake level (controlword bit 4).
    pp: Mutex<PpHandshake>,
    /// Mode (6060h) of an already-verified enabled drive — avoids a
    /// write+verify round trip on every command. Cleared by
    /// disable/quick-stop/fault-reset.
    mode_ready: Mutex<Option<i8>>,
    /// Last error code seen (`603Fh` via SDO query, TPDO1, or EMCY),
    /// decoupled from the anchored state: an EMCY fault burst can arrive
    /// before any TPDO2 anchor — the code must stay observable instead of
    /// being dropped with the un-anchorable patch.
    last_error_code: Mutex<Option<u16>>,
    /// 2003h_07 MIT gain scale factor (read-only) — fail-fast on first use.
    mit_scale: OnceLock<f32>,
    /// 2004h_04.._0D compressed-MIT field ranges.
    mit_ranges: OnceLock<MitRanges>,
    /// 6076h peak torque, Nm.
    peak_torque_nm: OnceLock<f32>,
    /// RPDO4 remapped to the compressed-MIT channel (2004h_2/_3).
    mit_rpdo_ready: AtomicBool,
    /// Default TPDOs activated (periodic reporting on).
    tpdos_active: AtomicBool,
}

impl HuayiMotor {
    /// Create the motor handle. `motor_id` is the CANopen node id
    /// (1..=127); `feedback_id` is metadata for the unified interface and
    /// defaults to `motor_id` when passed as 0 (Huayi feedback frames are
    /// the node's TPDOs, not a separate id). `model` is accepted as any
    /// non-empty string — the model catalog is pending (plan §9); limits
    /// are read from the device (6076h / 2004h ranges) instead.
    pub fn new(
        motor_id: u16,
        feedback_id: u16,
        model: &str,
        bus: std::sync::Arc<dyn CanBus>,
    ) -> Result<Self> {
        if model.trim().is_empty() {
            return Err(MotorError::InvalidArgument(
                "huayi model cannot be empty".to_string(),
            ));
        }
        let node = CanopenNode::new(motor_id, bus).map_err(|e| {
            MotorError::InvalidArgument(format!(
                "invalid Huayi motor-id {}: {}",
                motor_id, e
            ))
        })?;
        let feedback_id = if feedback_id == 0 { motor_id } else { feedback_id };
        Ok(Self {
            node,
            feedback_id,
            model: model.to_string(),
            state: Mutex::new(None),
            last_error_code: Mutex::new(None),
            pp: Mutex::new(PpHandshake::Low),
            mode_ready: Mutex::new(None),
            mit_scale: OnceLock::new(),
            mit_ranges: OnceLock::new(),
            peak_torque_nm: OnceLock::new(),
            mit_rpdo_ready: AtomicBool::new(false),
            tpdos_active: AtomicBool::new(false),
        })
    }

    /// Shared CANopen node — SDO/NMT procedures for vendor-private
    /// extensions.
    pub fn node(&self) -> &CanopenNode {
        &self.node
    }

    pub fn feedback_id(&self) -> u16 {
        self.feedback_id
    }

    /// TPDO stream active (see [`Self::configure_periodic_report`]).
    pub fn tpdos_active(&self) -> bool {
        self.tpdos_active.load(Ordering::Acquire)
    }

    // --- state machine (manual §3.1/§3.6) ----------------------------------

    /// NMT start + `Shutdown(0x0006) → Switch On(0x0007) → Enable
    /// Operation(0x000F)` (§3.1).
    pub fn enable_drive(&self, timeout: Duration) -> Result<()> {
        self.node.start_node()?;
        std::thread::sleep(Duration::from_millis(20));
        cia402::enable_sequence(&self.node, timeout)
    }

    /// `Disable Voltage (0x0000)` → Switch On Disabled (§3.6). **Not** the
    /// RobStride private `0x0001` — the Huayi command table has no such
    /// value.
    pub fn disable_drive(&self, timeout: Duration) -> Result<()> {
        cia402::disable_drive(&self.node, timeout)?;
        self.clear_mode_ready();
        Ok(())
    }

    /// `Quick Stop (0x000B)` (§3.2).
    pub fn quick_stop(&self, timeout: Duration) -> Result<()> {
        cia402::quick_stop(&self.node, timeout)?;
        self.clear_mode_ready();
        Ok(())
    }

    /// Fault reset `0x0080 → 0x0000` (§4.2) and clear the cached error code.
    pub fn clear_error(&self, timeout: Duration) -> Result<()> {
        cia402::clear_fault(&self.node, timeout)?;
        self.clear_mode_ready();
        self.record_error_code(0);
        self.patch_state(|s| s.error_code = 0);
        Ok(())
    }

    /// NMT master command (`node = 0` broadcasts). Beyond sending the frame,
    /// commands that affect this node invalidate the local caches so the
    /// next control command does not skip a handshake the drive no longer
    /// remembers:
    ///
    /// - `ResetNode` — the drive loses `6060h`, the controlword (PP bit-4
    ///   level) and its PDO configuration: mode-ready and PP-handshake
    ///   caches are dropped and the TPDO stream flag cleared,
    /// - `ResetCommunication` — the PDO/communication profile resets: the
    ///   TPDO stream flag is cleared,
    /// - `EnterPreOperational` / `StopNode` — TPDO transmission stops
    ///   outside Operational: the TPDO stream flag is cleared,
    /// - `StartRemoteNode` — no cache change (mode and configured PDOs
    ///   resume; if the flag was cleared earlier, the next
    ///   `request_feedback` reconfigures the stream, which is idempotent).
    pub fn nmt(&self, command: NmtCommand, node: u8) -> Result<()> {
        send_nmt(self.node.bus(), command, node)?;
        let affects_self = node == 0 || u16::from(node) == self.node.node_id();
        if !affects_self {
            return Ok(());
        }
        match command {
            NmtCommand::ResetNode => {
                self.clear_mode_ready();
                self.reset_pp_handshake();
                self.tpdos_active.store(false, Ordering::Release);
            }
            NmtCommand::ResetCommunication
            | NmtCommand::EnterPreOperational
            | NmtCommand::StopNode => {
                self.tpdos_active.store(false, Ordering::Release);
            }
            NmtCommand::StartRemoteNode => {}
        }
        Ok(())
    }

    /// Write `6060h` and verify: `6061h` echoes the mode and the drive is
    /// in Operation Enabled (running the enable handshake if not).
    /// Results are cached — later commands in the same mode skip the
    /// round trip (§3.3–§3.5).
    pub fn ensure_mode_enabled(&self, mode: RunMode, timeout: Duration) -> Result<()> {
        let raw = mode as i8;
        {
            let cached = self
                .mode_ready
                .lock()
                .map_err(|_| MotorError::Io("mode cache lock poisoned".to_string()))?;
            if *cached == Some(raw) {
                return Ok(());
            }
        }
        cia402::write_mode(&self.node, raw, timeout)?;
        self.verify_mode_enabled(mode, timeout)?;
        let mut cached = self
            .mode_ready
            .lock()
            .map_err(|_| MotorError::Io("mode cache lock poisoned".to_string()))?;
        *cached = Some(raw);
        Ok(())
    }

    fn verify_mode_enabled(&self, mode: RunMode, timeout: Duration) -> Result<()> {
        let display = cia402::read_mode_display(&self.node, timeout)?;
        if display != mode as i8 {
            return Err(MotorError::Protocol(format!(
                "Huayi mode display {display} does not match requested mode {}",
                mode as i8
            )));
        }
        if cia402::read_state(&self.node, timeout)? != Cia402State::OperationEnabled {
            self.node.start_node()?;
            std::thread::sleep(Duration::from_millis(20));
            cia402::enable_sequence(&self.node, timeout)?;
            if cia402::read_state(&self.node, timeout)? != Cia402State::OperationEnabled {
                return Err(MotorError::Protocol(
                    "Huayi drive did not reach Operation Enabled".to_string(),
                ));
            }
        }
        Ok(())
    }

    fn clear_mode_ready(&self) {
        if let Ok(mut cached) = self.mode_ready.lock() {
            *cached = None;
        }
    }

    /// Drop the PP bit-4 handshake cache (after a node reset the controlword
    /// is back to its power-up value, bit 4 low).
    fn reset_pp_handshake(&self) {
        if let Ok(mut pp) = self.pp.lock() {
            *pp = PpHandshake::Low;
        }
    }

    // --- lazy device constants ---------------------------------------------

    /// 2003h_07 MIT gain scale factor (read-only). Fetched once; a failed
    /// first read is an error, not a silent default (plan §3.4).
    fn mit_scale(&self, timeout: Duration) -> Result<f32> {
        if let Some(v) = self.mit_scale.get() {
            return Ok(*v);
        }
        let v = self.node.sdo_read_f32(od::MIT_PARAMS, 0x07, timeout)?;
        let _ = self.mit_scale.set(v);
        Ok(v)
    }

    /// 2004h_04.._0D compressed-MIT field ranges, fetched once.
    pub fn mit_ranges(&self, timeout: Duration) -> Result<MitRanges> {
        if let Some(r) = self.mit_ranges.get() {
            return Ok(*r);
        }
        let mut values = [0f32; 10];
        for (i, sub) in units::MIT_RANGE_SUBS.iter().enumerate() {
            values[i] = self.node.sdo_read_f32(od::MIT_COMPRESSED, *sub, timeout)?;
        }
        let pairs = pairs_from_slice(&values).ok_or_else(|| {
            MotorError::Protocol("invalid 2004h MIT range registers".to_string())
        })?;
        let ranges = MitRanges {
            pos: pairs[0],
            vel: pairs[1],
            kp: pairs[2],
            kd: pairs[3],
            tff: pairs[4],
        };
        let _ = self.mit_ranges.set(ranges);
        Ok(ranges)
    }

    /// 6076h peak torque (Nm), fetched once.
    pub fn peak_torque_nm(&self, timeout: Duration) -> Result<f32> {
        if let Some(v) = self.peak_torque_nm.get() {
            return Ok(*v);
        }
        let v = self.node.sdo_read_f32(MOTOR_PEAK_TORQUE, 0x00, timeout)?;
        if !v.is_finite() || v <= 0.0 {
            return Err(MotorError::Protocol(format!(
                "invalid peak torque {v} Nm at 6076h"
            )));
        }
        let _ = self.peak_torque_nm.set(v);
        Ok(v)
    }

    fn peak_torque_cached(&self) -> Option<f32> {
        self.peak_torque_nm.get().copied()
    }

    // --- commands ----------------------------------------------------------

    /// MIT slow path over SDO (§3.5): `2003h_1/2/3` take Rev / Rev/s / Nm
    /// Float32; Kp/Kd are dimensionless u16 converted with the 2003h_7
    /// scale factor (`k_SI · 2π / scale`).
    pub fn send_mit(
        &self,
        pos_rad: f32,
        vel_rad_s: f32,
        kp_nm_per_rad: f32,
        kd_nm_per_rad: f32,
        tau_nm: f32,
        timeout: Duration,
    ) -> Result<()> {
        self.ensure_mode_enabled(RunMode::Mit, timeout)?;
        let scale = self.mit_scale(timeout)?;
        self.node
            .sdo_write_f32(od::MIT_PARAMS, 0x01, units::rad_to_rev(pos_rad), timeout)?;
        self.node.sdo_write_f32(
            od::MIT_PARAMS,
            0x02,
            units::rad_s_to_rev_s(vel_rad_s),
            timeout,
        )?;
        self.node
            .sdo_write_f32(od::MIT_PARAMS, 0x03, tau_nm, timeout)?;
        self.node.sdo_write_u16(
            od::MIT_PARAMS,
            0x04,
            units::gain_to_dimensionless(kp_nm_per_rad, scale)?,
            timeout,
        )?;
        self.node.sdo_write_u16(
            od::MIT_PARAMS,
            0x05,
            units::gain_to_dimensionless(kd_nm_per_rad, scale)?,
            timeout,
        )?;
        Ok(())
    }

    /// MIT compressed fast path (§5.2): RPDO4 remapped to
    /// `[2004h_02, 2004h_03]`, one 8-byte classic frame to
    /// `0x500 + node_id` per command. The 64-bit control word packs
    /// Pdes(16b) / Vdes(12b) / Kp(12b) / Kd(12b) / Tff(12b) against the
    /// runtime-read 2004h ranges. Kp/Kd unit chain: unified Nm/rad
    /// →(×2π)→ Nm/Rev →(linear map)→ 12-bit field.
    pub fn send_mit_compressed(
        &self,
        pos_rad: f32,
        vel_rad_s: f32,
        kp_nm_per_rad: f32,
        kd_nm_per_rad: f32,
        tau_nm: f32,
        timeout: Duration,
    ) -> Result<()> {
        self.ensure_mode_enabled(RunMode::Mit, timeout)?;
        if !self.mit_rpdo_ready.load(Ordering::Acquire) {
            hpdo::configure_mit_rpdo(&self.node, timeout)?;
            self.mit_rpdo_ready.store(true, Ordering::Release);
        }
        let ranges = self.mit_ranges(timeout)?;
        let word = units::pack_mit_word(
            units::rad_to_rev(pos_rad),
            units::rad_s_to_rev_s(vel_rad_s),
            units::gain_nm_rad_to_nm_rev(kp_nm_per_rad),
            units::gain_nm_rad_to_nm_rev(kd_nm_per_rad),
            tau_nm,
            &ranges,
        )?;
        let (low, high) = units::split_mit_word(word);
        let mut payload = [0u8; 8];
        payload[0..4].copy_from_slice(&low.to_le_bytes());
        payload[4..8].copy_from_slice(&high.to_le_bytes());
        send_std_frame(
            self.node.bus().as_ref(),
            rpdo_id(4, self.node.node_id()),
            &payload,
        )
    }

    /// PP new-target handshake: controlword bit 4 toggled 1→0→1
    /// (bit 5 = take effect immediately, bit 6 = absolute). Shared by
    /// `send_pos_vel` and `controlled_stop` — the drive only latches a
    /// new 607Ah target on the set edge.
    fn pp_new_target(&self, timeout: Duration) -> Result<()> {
        let mut pp = self
            .pp
            .lock()
            .map_err(|_| MotorError::Io("pp handshake lock poisoned".to_string()))?;
        if *pp == PpHandshake::High {
            // clear bit 4 so the set edge below registers as a new target
            cia402::write_controlword(&self.node, 0x002F, timeout)?;
        }
        cia402::write_controlword(&self.node, 0x003F, timeout)?;
        *pp = PpHandshake::High;
        Ok(())
    }

    /// Profile position (§3.4): profile velocity `6081h` (Rev/s Float32),
    /// target position `607Ah` (Rev Float32), then the new-target
    /// handshake — controlword bit 4 toggled 1→0→1 (bit 5 = take effect
    /// immediately, bit 6 = absolute).
    pub fn send_pos_vel(
        &self,
        pos_rad: f32,
        vlim_rad_s: f32,
        timeout: Duration,
    ) -> Result<()> {
        self.ensure_mode_enabled(RunMode::ProfilePosition, timeout)?;
        self.node.sdo_write_f32(
            PROFILE_VELOCITY,
            0x00,
            units::rad_s_to_rev_s(vlim_rad_s),
            timeout,
        )?;
        self.node
            .sdo_write_f32(TARGET_POSITION, 0x00, units::rad_to_rev(pos_rad), timeout)?;
        self.pp_new_target(timeout)
    }

    /// Profile velocity (§3.3): idempotent mode 3 + `60FFh` target
    /// velocity (Rev/s Float32).
    pub fn send_vel(&self, vel_rad_s: f32, timeout: Duration) -> Result<()> {
        self.ensure_mode_enabled(RunMode::ProfileVelocity, timeout)?;
        self.node.sdo_write_f32(
            TARGET_VELOCITY,
            0x00,
            units::rad_s_to_rev_s(vel_rad_s),
            timeout,
        )
    }

    /// Torque mode (§3.5): `6071h` Int16, ‰ of the 6076h peak.
    pub fn send_torque(&self, tau_nm: f32, timeout: Duration) -> Result<()> {
        self.ensure_mode_enabled(RunMode::Torque, timeout)?;
        let peak = self.peak_torque_nm(timeout)?;
        let permille = units::torque_nm_to_permille(tau_nm, peak)?;
        self.node.sdo_write_i16(TARGET_TORQUE, 0x00, permille, timeout)
    }

    /// Mode-aware stop that keeps the drive enabled (aligned with the
    /// RobStride semantics): PV → target velocity 0; PP → re-target the
    /// current position; MIT → all-zero MIT; torque → target 0.
    pub fn controlled_stop(&self, timeout: Duration) -> Result<()> {
        let mode = {
            let cached = self
                .mode_ready
                .lock()
                .map_err(|_| MotorError::Io("mode cache lock poisoned".to_string()))?;
            match *cached {
                Some(raw) => raw,
                None => self.node.sdo_read_i8(MODES_OF_OPERATION_DISPLAY, 0x00, timeout)?,
            }
        };
        match mode {
            1 => {
                // lock in place: target = current actual position, then the
                // same bit4 new-target handshake send_pos_vel uses — without
                // the set edge the drive ignores the re-target and keeps
                // moving toward the old one
                let pos_rev = self.node.sdo_read_f32(POSITION_ACTUAL_VALUE, 0x00, timeout)?;
                self.node.sdo_write_f32(TARGET_POSITION, 0x00, pos_rev, timeout)?;
                self.pp_new_target(timeout)
            }
            3 => self.node.sdo_write_f32(TARGET_VELOCITY, 0x00, 0.0, timeout),
            4 => self.node.sdo_write_i16(TARGET_TORQUE, 0x00, 0, timeout),
            5 => self.send_mit(0.0, 0.0, 0.0, 0.0, 0.0, timeout),
            other => Err(MotorError::Unsupported(format!(
                "controlled stop not implemented for Huayi mode {other}"
            ))),
        }
    }

    // --- position preset (§3.6) --------------------------------------------

    /// Preset the current rotor position to `value_rev` (range ±0.5 Rev).
    /// Requires Switch On Disabled — disable the drive first (the same
    /// rule Damiao applies to its zeroing).
    pub fn position_preset(&self, value_rev: f32, timeout: Duration) -> Result<()> {
        let state = cia402::read_state(&self.node, timeout)?;
        if state != Cia402State::SwitchOnDisabled {
            return Err(MotorError::InvalidArgument(format!(
                "Huayi position preset requires Switch On Disabled, drive is {state:?} — disable it first"
            )));
        }
        let (min, max) = od::POSITION_PRESET_RANGE_REV;
        if !value_rev.is_finite() || value_rev < min || value_rev > max {
            return Err(MotorError::InvalidArgument(format!(
                "position preset value {value_rev} Rev outside the ±0.5 Rev range"
            )));
        }
        self.node
            .sdo_write_f32(od::POSITION_PRESET, 0x01, value_rev, timeout)?;
        self.node
            .sdo_write_u32(od::POSITION_PRESET, 0x02, od::POSITION_PRESET_CODE, timeout)
    }

    /// Preset the current position to zero (the unified-interface special
    /// case of [`Self::position_preset`]).
    pub fn set_zero_position(&self, timeout: Duration) -> Result<()> {
        self.position_preset(0.0, timeout)
    }

    // --- parameters / maintenance ------------------------------------------

    /// Store all parameters (`1010h_1` = "save" signature, §5.1).
    pub fn store_parameters(&self, timeout: Duration) -> Result<()> {
        self.node
            .sdo_write_u32(od::STORE_PARAMS, 0x01, od::STORE_SIGNATURE, timeout)
    }

    /// Restore default parameters (`1011h_1` = "load" signature; takes
    /// effect after reset / power cycle).
    pub fn restore_parameters(&self, timeout: Duration) -> Result<()> {
        self.node
            .sdo_write_u32(od::RESTORE_PARAMS, 0x01, od::RESTORE_SIGNATURE, timeout)
    }

    /// Node id change (`2001h_1`); effective only after
    /// [`Self::store_parameters`] + power cycle (§5.2).
    pub fn set_node_id(&self, new_id: u16, timeout: Duration) -> Result<()> {
        if new_id == 0 || new_id > 127 {
            return Err(MotorError::InvalidArgument(format!(
                "invalid Huayi node id {new_id}, expected 1..127"
            )));
        }
        self.node.sdo_write_u8(od::DEVICE_SETTING, 0x01, new_id as u8, timeout)
    }

    /// 1003h error-history walk (sub 1..=count; §4.1).
    pub fn get_error_history(&self, timeout: Duration) -> Result<Vec<u16>> {
        let count = self
            .node
            .sdo_read_u8(motor_canopen::objects::PRE_DEFINED_ERROR_FIELD, 0x00, timeout)?;
        let mut history = Vec::with_capacity(usize::from(count));
        for sub in 1..=count {
            let code = self
                .node
                .sdo_read_u16(motor_canopen::objects::PRE_DEFINED_ERROR_FIELD, sub, timeout)?;
            history.push(code);
        }
        Ok(history)
    }

    /// Activate (or with `period_ms = 0`, deactivate) the three default
    /// TPDOs with an event timer of `period_ms` (§2.8.5 / §4.5). Plays the
    /// role of RobStride `set_active_report`. Warming the peak-torque
    /// cache here is best-effort so TPDO3 torque converts to Nm; the
    /// wire-true `torque_permille` is always cached regardless.
    pub fn configure_periodic_report(&self, period_ms: u16, timeout: Duration) -> Result<()> {
        if period_ms == 0 {
            hpdo::deactivate_default_tpdos(&self.node, timeout)?;
            self.tpdos_active.store(false, Ordering::Release);
            return Ok(());
        }
        let _ = self.peak_torque_nm(timeout);
        hpdo::activate_default_tpdos(&self.node, period_ms, timeout)?;
        self.tpdos_active.store(true, Ordering::Release);
        Ok(())
    }

    /// Blocking SDO status query — the backup path when TPDO streaming is
    /// off (aligned with the hexfellow `query_status` role, §5.3).
    pub fn query_status(&self, timeout: Duration) -> Result<HuayiStatus> {
        let mode_display = self.node.sdo_read_i8(MODES_OF_OPERATION_DISPLAY, 0x00, timeout)?;
        let statusword = self.node.sdo_read_u16(STATUSWORD, 0x00, timeout)?;
        let error_code = self.node.sdo_read_u16(ERROR_CODE, 0x00, timeout).unwrap_or(0);
        let pos_rev = self.node.sdo_read_f32(POSITION_ACTUAL_VALUE, 0x00, timeout)?;
        let vel_rev_s = self.node.sdo_read_f32(
            motor_canopen::objects::VELOCITY_ACTUAL_VALUE,
            0x00,
            timeout,
        )?;
        let torque_permille = self
            .node
            .sdo_read_i16(motor_canopen::objects::TORQUE_ACTUAL_VALUE, 0x00, timeout)
            .unwrap_or(0);
        let t_driver_raw = self.node.sdo_read_i16(od::TEMPERATURES, 0x01, timeout).unwrap_or(0);
        let t_motor_raw = self.node.sdo_read_i16(od::TEMPERATURES, 0x02, timeout).unwrap_or(0);
        let torque_nm = self
            .peak_torque_nm(timeout)
            .ok()
            .map(|peak| units::permille_to_torque_nm(torque_permille, peak))
            .unwrap_or(0.0);

        let mut guard = self
            .state
            .lock()
            .map_err(|_| MotorError::Io("state lock poisoned".to_string()))?;
        let seq = guard.as_ref().map_or(1, |s| s.seq + 1);
        let status = HuayiStatus {
            pos_rad: units::rev_to_rad(pos_rev),
            vel_rad_s: units::rev_s_to_rad_s(vel_rev_s),
            torque_nm,
            torque_permille,
            t_driver_c: units::raw_temp_to_celsius(t_driver_raw),
            t_motor_c: units::raw_temp_to_celsius(t_motor_raw),
            statusword,
            mode_display,
            error_code,
            received_at: Instant::now(),
            seq,
        };
        *guard = Some(status);
        self.record_error_code(error_code);
        Ok(status)
    }

    /// Last cached feedback (TPDO stream and/or `query_status`).
    pub fn latest_state(&self) -> Option<HuayiStatus> {
        self.state.lock().ok().and_then(|s| *s)
    }

    /// Last-seen error code (`603Fh` via SDO query, TPDO1, or EMCY) —
    /// observable even before a TPDO2 anchor arrives. `None` until some
    /// feedback source has reported a code. No bus traffic.
    pub fn cached_error_code(&self) -> Option<u16> {
        self.last_error_code.lock().ok().and_then(|c| *c)
    }

    fn record_error_code(&self, code: u16) {
        if let Ok(mut c) = self.last_error_code.lock() {
            *c = Some(code);
        }
    }

    // --- SDO passthrough (vendor-private interface channel, §2.7) ---------

    pub fn sdo_read_u8(&self, index: u16, subindex: u8, timeout: Duration) -> Result<u8> {
        self.node.sdo_read_u8(index, subindex, timeout)
    }
    pub fn sdo_read_i8(&self, index: u16, subindex: u8, timeout: Duration) -> Result<i8> {
        self.node.sdo_read_i8(index, subindex, timeout)
    }
    pub fn sdo_read_u16(&self, index: u16, subindex: u8, timeout: Duration) -> Result<u16> {
        self.node.sdo_read_u16(index, subindex, timeout)
    }
    pub fn sdo_read_i16(&self, index: u16, subindex: u8, timeout: Duration) -> Result<i16> {
        self.node.sdo_read_i16(index, subindex, timeout)
    }
    pub fn sdo_read_u32(&self, index: u16, subindex: u8, timeout: Duration) -> Result<u32> {
        self.node.sdo_read_u32(index, subindex, timeout)
    }
    pub fn sdo_read_i32(&self, index: u16, subindex: u8, timeout: Duration) -> Result<i32> {
        self.node.sdo_read_i32(index, subindex, timeout)
    }
    pub fn sdo_read_f32(&self, index: u16, subindex: u8, timeout: Duration) -> Result<f32> {
        self.node.sdo_read_f32(index, subindex, timeout)
    }
    pub fn sdo_write_u8(&self, index: u16, subindex: u8, value: u8, timeout: Duration) -> Result<()> {
        self.node.sdo_write_u8(index, subindex, value, timeout)
    }
    pub fn sdo_write_i8(&self, index: u16, subindex: u8, value: i8, timeout: Duration) -> Result<()> {
        self.node.sdo_write_i8(index, subindex, value, timeout)
    }
    pub fn sdo_write_u16(&self, index: u16, subindex: u8, value: u16, timeout: Duration) -> Result<()> {
        self.node.sdo_write_u16(index, subindex, value, timeout)
    }
    pub fn sdo_write_i16(&self, index: u16, subindex: u8, value: i16, timeout: Duration) -> Result<()> {
        self.node.sdo_write_i16(index, subindex, value, timeout)
    }
    pub fn sdo_write_u32(&self, index: u16, subindex: u8, value: u32, timeout: Duration) -> Result<()> {
        self.node.sdo_write_u32(index, subindex, value, timeout)
    }
    pub fn sdo_write_i32(&self, index: u16, subindex: u8, value: i32, timeout: Duration) -> Result<()> {
        self.node.sdo_write_i32(index, subindex, value, timeout)
    }
    pub fn sdo_write_f32(&self, index: u16, subindex: u8, value: f32, timeout: Duration) -> Result<()> {
        self.node.sdo_write_f32(index, subindex, value, timeout)
    }

    // --- feedback processing ------------------------------------------------

    fn patch_state(&self, patch: impl FnOnce(&mut HuayiStatus)) {
        let Ok(mut guard) = self.state.lock() else { return };
        // Patch frames (TPDO1/TPDO3/EMCY) only carry partial fields —
        // fabricating a blank state here would report a valid-looking
        // pos=0/vel=0 before any TPDO2 anchor or query_status arrived.
        // Drop the patch instead: latest_state stays None (has_value=0).
        let Some(mut s) = guard.take() else { return };
        patch(&mut s);
        *guard = Some(s);
    }

    fn anchor_state(&self, pos_rad: f32, vel_rad_s: f32) {
        let Ok(mut guard) = self.state.lock() else { return };
        let mut s = guard.take().unwrap_or_else(blank_state);
        s.pos_rad = pos_rad;
        s.vel_rad_s = vel_rad_s;
        s.seq += 1;
        s.received_at = Instant::now();
        *guard = Some(s);
    }

    fn apply_tpdo(&self, n: u8, frame: &CanFrame) {
        match n {
            2 => {
                if let Some(d) = hpdo::decode_tpdo2(&frame.data, frame.dlc) {
                    self.anchor_state(
                        units::rev_to_rad(d.position_rev),
                        units::rev_s_to_rad_s(d.velocity_rev_s),
                    );
                }
            }
            1 => {
                if let Some(d) = hpdo::decode_tpdo1(&frame.data, frame.dlc) {
                    self.record_error_code(d.error_code);
                    self.patch_state(|s| {
                        s.error_code = d.error_code;
                        s.statusword = d.statusword;
                        s.mode_display = d.mode_display as i8;
                    });
                }
            }
            3 => {
                if let Some(d) = hpdo::decode_tpdo3(&frame.data, frame.dlc) {
                    let torque_nm = self
                        .peak_torque_cached()
                        .map(|peak| units::permille_to_torque_nm(d.torque_permille, peak))
                        .unwrap_or(0.0);
                    self.patch_state(|s| {
                        s.torque_permille = d.torque_permille;
                        s.torque_nm = torque_nm;
                        s.t_driver_c = units::raw_temp_to_celsius(d.t_driver_raw);
                        s.t_motor_c = units::raw_temp_to_celsius(d.t_motor_raw);
                    });
                }
            }
            _ => {}
        }
    }
}

fn blank_state() -> HuayiStatus {
    HuayiStatus {
        pos_rad: 0.0,
        vel_rad_s: 0.0,
        torque_nm: 0.0,
        torque_permille: 0,
        t_driver_c: 0.0,
        t_motor_c: 0.0,
        statusword: 0,
        mode_display: 0,
        error_code: 0,
        received_at: Instant::now(),
        seq: 0,
    }
}

fn pairs_from_slice(values: &[f32; 10]) -> Option<[(f32, f32); 5]> {
    let mut pairs = [(0.0, 0.0); 5];
    for (i, pair) in pairs.iter_mut().enumerate() {
        let (min, max) = (values[i * 2], values[i * 2 + 1]);
        if !min.is_finite() || !max.is_finite() || max <= min {
            return None;
        }
        *pair = (min, max);
    }
    Some(pairs)
}

impl MotorDevice for HuayiMotor {
    fn vendor(&self) -> &'static str {
        "huayi"
    }

    fn model(&self) -> &str {
        &self.model
    }

    fn motor_id(&self) -> u16 {
        self.node.node_id()
    }

    fn feedback_id(&self) -> u16 {
        self.feedback_id
    }

    fn enable(&self) -> Result<()> {
        self.enable_drive(Duration::from_millis(200))
    }

    fn disable(&self) -> Result<()> {
        self.disable_drive(Duration::from_millis(200))
    }

    fn accepts_frame(&self, frame: &CanFrame) -> bool {
        let node = self.node.node_id();
        self.node.accepts_std(frame)
            || is_emcy(frame, node)
            || (1..=3).any(|n| is_tpdo(frame, n, node))
    }

    fn process_feedback_frame(&self, frame: CanFrame) -> Result<()> {
        match self.node.process(&frame) {
            NodeOutcome::NotClaimed => {}
            NodeOutcome::SdoQueued | NodeOutcome::Heartbeat(_) => return Ok(()),
        }
        let node = self.node.node_id();
        if let Some((n, owner)) = tpdo_owner(&frame) {
            if owner == node {
                self.apply_tpdo(n, &frame);
            }
            return Ok(());
        }
        // EMCY: [error register u8][error code u16 LE][manufacturer bytes]
        if is_emcy(&frame, node) && frame.dlc >= 3 {
            let code = u16::from_le_bytes([frame.data[1], frame.data[2]]);
            self.record_error_code(code);
            self.patch_state(|s| s.error_code = code);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use motor_core::test_support::MockBus;
    use std::sync::Arc;

    const TIMEOUT: Duration = Duration::from_millis(200);

    fn motor(bus: &Arc<MockBus>) -> HuayiMotor {
        HuayiMotor::new(1, 0, "huayi-joint", bus.clone()).expect("motor")
    }

    /// SDO write-ack frame for (index, sub).
    fn ack(node: u16, index: u16, sub: u8) -> CanFrame {
        sdo_rsp(node, 0x60, index, sub, [0, 0, 0, 0])
    }

    /// SDO expedited upload reply carrying a 4-byte payload.
    fn sdo_rsp(node: u16, cmd: u8, index: u16, sub: u8, payload: [u8; 4]) -> CanFrame {
        let mut data = [0u8; 8];
        data[0] = cmd;
        data[1] = (index & 0xFF) as u8;
        data[2] = ((index >> 8) & 0xFF) as u8;
        data[3] = sub;
        data[4..8].copy_from_slice(&payload);
        CanFrame {
            arbitration_id: 0x580 + u32::from(node),
            data,
            dlc: 8,
            is_extended: false,
            is_rx: true,
        }
    }

    fn u16p(v: u16) -> [u8; 4] {
        [v as u8, (v >> 8) as u8, 0, 0]
    }

    fn i16p(v: i16) -> [u8; 4] {
        let b = v.to_le_bytes();
        [b[0], b[1], 0, 0]
    }

    fn f32p(v: f32) -> [u8; 4] {
        v.to_le_bytes()
    }

    /// Queue replies for a full `ensure_mode_enabled` round trip:
    /// 6060h write ack, 6061h echo, 6041h = Operation Enabled (0x0237).
    fn queue_mode_enabled(m: &HuayiMotor, node: u16, mode: i8) {
        m.process_feedback_frame(ack(node, 0x6060, 0x00)).expect("feed");
        m.process_feedback_frame(sdo_rsp(node, 0x43, 0x6061, 0x00, [mode as u8, 0, 0, 0]))
            .expect("feed");
        m.process_feedback_frame(sdo_rsp(node, 0x43, 0x6041, 0x00, u16p(0x0237)))
            .expect("feed");
    }

    fn sent_frames(bus: &Arc<MockBus>) -> Vec<CanFrame> {
        bus.sent.lock().expect("sent").clone()
    }

    #[test]
    fn node_id_range() {
        let bus = Arc::new(MockBus::new());
        assert!(HuayiMotor::new(0, 0, "m", bus.clone()).is_err());
        assert!(HuayiMotor::new(128, 0, "m", bus.clone()).is_err());
        assert!(HuayiMotor::new(1, 0, "", bus.clone()).is_err());
        assert!(HuayiMotor::new(1, 0, "   ", bus).is_err());
        // feedback_id 0 defaults to the node id
        let bus = Arc::new(MockBus::new());
        let m = motor(&bus);
        assert_eq!(m.feedback_id(), 1);
        assert_eq!(m.motor_id(), 1);
        assert_eq!(m.vendor(), "huayi");
    }

    #[test]
    fn enable_sequence_frames() {
        let bus = Arc::new(MockBus::new());
        let m = motor(&bus);
        for _ in 0..3 {
            m.process_feedback_frame(ack(1, 0x6040, 0x00)).expect("feed");
        }
        m.enable_drive(TIMEOUT).expect("enable");

        let sent = sent_frames(&bus);
        // NMT start, then 0x06 / 0x07 / 0x0F controlword writes
        assert_eq!(sent.len(), 4);
        assert_eq!(sent[0].arbitration_id, 0x000);
        assert_eq!(sent[0].data[0], 0x01);
        assert_eq!(sent[0].data[1], 0x01);
        let words: Vec<u16> = sent[1..]
            .iter()
            .map(|f| u16::from_le_bytes([f.data[4], f.data[5]]))
            .collect();
        assert_eq!(words, vec![0x0006, 0x0007, 0x000F]);
        for f in &sent[1..] {
            assert_eq!(f.arbitration_id, 0x601);
            assert_eq!(f.data[0], 0x2B);
            assert_eq!(f.data[1..4], [0x40, 0x60, 0x00]);
        }
    }

    #[test]
    fn disable_uses_disable_voltage() {
        let bus = Arc::new(MockBus::new());
        let m = motor(&bus);
        m.process_feedback_frame(ack(1, 0x6040, 0x00)).expect("feed");
        m.disable_drive(TIMEOUT).expect("disable");

        let sent = sent_frames(&bus);
        assert_eq!(sent.len(), 1);
        // 0x0000 (Disable Voltage) — NOT RobStride's private 0x0001
        assert_eq!(u16::from_le_bytes([sent[0].data[4], sent[0].data[5]]), 0x0000);
    }

    #[test]
    fn ensure_mode_verifies() {
        let bus = Arc::new(MockBus::new());
        let m = motor(&bus);
        // write ack, but 6061h echoes a different mode
        m.process_feedback_frame(ack(1, 0x6060, 0x00)).expect("feed");
        m.process_feedback_frame(sdo_rsp(1, 0x43, 0x6061, 0x00, [3, 0, 0, 0]))
            .expect("feed");
        let err = m
            .ensure_mode_enabled(RunMode::Mit, TIMEOUT)
            .expect_err("must fail");
        assert!(err.to_string().contains("mode display"));

        // mode cache must not latch on failure
        assert!(m.mode_ready.lock().expect("cache").is_none());
    }

    #[test]
    fn ensure_mode_caches_verified_mode() {
        let bus = Arc::new(MockBus::new());
        let m = motor(&bus);
        queue_mode_enabled(&m, 1, 5);
        m.ensure_mode_enabled(RunMode::Mit, TIMEOUT).expect("first");
        // second call: no SDO traffic at all
        let before = sent_frames(&bus).len();
        m.ensure_mode_enabled(RunMode::Mit, TIMEOUT).expect("second");
        assert_eq!(sent_frames(&bus).len(), before);

        // disable clears the cache: next ensure re-handshakes
        m.process_feedback_frame(ack(1, 0x6040, 0x00)).expect("feed");
        m.disable_drive(TIMEOUT).expect("disable");
        queue_mode_enabled(&m, 1, 5);
        m.ensure_mode_enabled(RunMode::Mit, TIMEOUT).expect("re-enable");
        let sent = sent_frames(&bus);
        // a fresh 6060h=5 write landed after the disable (0x0000) frame
        let disable_at = sent
            .iter()
            .position(|f| {
                f.data[1..4] == [0x40, 0x60, 0x00]
                    && u16::from_le_bytes([f.data[4], f.data[5]]) == 0x0000
            })
            .expect("disable frame");
        assert!(sent[disable_at..]
            .iter()
            .any(|f| f.data[1..4] == [0x60, 0x60, 0x00] && f.data[4] == 5));
    }

    #[test]
    fn send_pos_vel_handshake() {
        let bus = Arc::new(MockBus::new());
        let m = motor(&bus);
        queue_mode_enabled(&m, 1, 1);
        // first command: 6081h, 607Ah, then CW set edge only (0x003F)
        m.process_feedback_frame(ack(1, 0x6081, 0x00)).expect("feed");
        m.process_feedback_frame(ack(1, 0x607A, 0x00)).expect("feed");
        m.process_feedback_frame(ack(1, 0x6040, 0x00)).expect("feed");
        m.send_pos_vel(std::f32::consts::PI, 10.0, TIMEOUT)
            .expect("first");

        let sent = sent_frames(&bus);
        // frames: 6060 write, 6061 read, 6041 read, 6081 write, 607A write, 6040 write
        assert_eq!(sent.len(), 6);
        assert_eq!(sent[3].data[1..4], [0x81, 0x60, 0x00]);
        // profile velocity 10 rad/s = 10/2π Rev/s
        let v = f32::from_le_bytes([sent[3].data[4], sent[3].data[5], sent[3].data[6], sent[3].data[7]]);
        assert!((v - 10.0 / std::f32::consts::TAU).abs() < 1e-4);
        // target position π rad = 0.5 Rev
        let p = f32::from_le_bytes([sent[4].data[4], sent[4].data[5], sent[4].data[6], sent[4].data[7]]);
        assert!((p - 0.5).abs() < 1e-5);
        // first handshake: only the set edge (bit4 0→1), immediate + absolute
        assert_eq!(u16::from_le_bytes([sent[5].data[4], sent[5].data[5]]), 0x003F);

        // second command: mode cached; CW toggles 1→0→1
        m.process_feedback_frame(ack(1, 0x6081, 0x00)).expect("feed");
        m.process_feedback_frame(ack(1, 0x607A, 0x00)).expect("feed");
        for _ in 0..2 {
            m.process_feedback_frame(ack(1, 0x6040, 0x00)).expect("feed");
        }
        m.send_pos_vel(0.0, 10.0, TIMEOUT).expect("second");

        let sent = sent_frames(&bus);
        // 6081, 607A, 0x002F, 0x003F
        assert_eq!(sent.len(), 10);
        let words: Vec<u16> = sent[6..]
            .iter()
            .filter(|f| f.data[1..4] == [0x40, 0x60, 0x00])
            .map(|f| u16::from_le_bytes([f.data[4], f.data[5]]))
            .collect();
        assert_eq!(words, vec![0x002F, 0x003F]);
    }

    #[test]
    fn send_vel_writes_target_velocity_float() {
        let bus = Arc::new(MockBus::new());
        let m = motor(&bus);
        queue_mode_enabled(&m, 1, 3);
        m.process_feedback_frame(ack(1, 0x60FF, 0x00)).expect("feed");
        m.send_vel(std::f32::consts::TAU, TIMEOUT).expect("vel");

        let sent = sent_frames(&bus);
        let target = sent
            .iter()
            .find(|f| f.data[1..4] == [0xFF, 0x60, 0x00])
            .expect("60FF write");
        let v = f32::from_le_bytes([target.data[4], target.data[5], target.data[6], target.data[7]]);
        assert!((v - 1.0).abs() < 1e-5); // 2π rad/s = 1 Rev/s
        assert_eq!(target.data[0], 0x23); // Float32 download
    }

    #[test]
    fn send_torque_uses_peak_permille() {
        let bus = Arc::new(MockBus::new());
        let m = motor(&bus);
        queue_mode_enabled(&m, 1, 4);
        // 6076h peak = 10 Nm
        m.process_feedback_frame(sdo_rsp(1, 0x43, MOTOR_PEAK_TORQUE, 0x00, f32p(10.0)))
            .expect("feed");
        m.process_feedback_frame(ack(1, 0x6071, 0x00)).expect("feed");
        m.send_torque(5.0, TIMEOUT).expect("torque");

        let sent = sent_frames(&bus);
        let target = sent
            .iter()
            .find(|f| f.data[1..4] == [0x71, 0x60, 0x00])
            .expect("6071 write");
        assert_eq!(target.data[0], 0x2B); // Int16 download
        assert_eq!(i16::from_le_bytes([target.data[4], target.data[5]]), 500);
    }

    #[test]
    fn mit_slow_path_writes_2003h_and_fails_fast_on_scale() {
        let bus = Arc::new(MockBus::new());
        let m = motor(&bus);
        queue_mode_enabled(&m, 1, 5);
        // 2003h_7 scale = 0.01
        m.process_feedback_frame(sdo_rsp(1, 0x43, od::MIT_PARAMS, 0x07, f32p(0.01)))
            .expect("feed");
        for sub in [0x01u8, 0x02, 0x03, 0x04, 0x05] {
            m.process_feedback_frame(ack(1, od::MIT_PARAMS, sub)).expect("feed");
        }
        m.send_mit(0.0, 0.0, 0.0, 0.0, 1.5, TIMEOUT).expect("mit");

        let sent = sent_frames(&bus);
        let find = |sub: u8| {
            *sent
                .iter()
                .find(|f| f.data[1..4] == [0x03, 0x20, sub])
                .expect("2003h write")
        };
        // Tff 1.5 Nm passes through unchanged
        let tff = f32::from_le_bytes([
            find(0x03).data[4],
            find(0x03).data[5],
            find(0x03).data[6],
            find(0x03).data[7],
        ]);
        assert!((tff - 1.5).abs() < 1e-5);
        // kp = kd = 0 → dimensionless 0
        assert_eq!(find(0x04).data[4], 0);
        assert_eq!(find(0x05).data[4], 0);

        // scale read failure → error, not a silent default
        let bus2 = Arc::new(MockBus::new());
        let m2 = motor(&bus2);
        queue_mode_enabled(&m2, 1, 5);
        let err = m2.send_mit(0.0, 0.0, 1.0, 1.0, 0.0, TIMEOUT).expect_err("no scale");
        assert!(matches!(err, MotorError::Timeout(_)));
    }

    #[test]
    fn mit_compressed_bitpacking() {
        let bus = Arc::new(MockBus::new());
        let m = motor(&bus);
        queue_mode_enabled(&m, 1, 5);
        // RPDO4 remap + 2004h_1 enable acks
        let comm = motor_canopen::pdo::rpdo_comm_index(4);
        let map = motor_canopen::pdo::rpdo_map_index(4);
        for (index, sub) in [
            (comm, 0x01),
            (map, 0x00),
            (map, 0x01),
            (map, 0x02),
            (map, 0x00),
            (comm, 0x02),
            (comm, 0x01),
            (od::MIT_COMPRESSED, 0x01),
        ] {
            m.process_feedback_frame(ack(1, index, sub)).expect("feed");
        }
        // 2004h_4.._D range reads: manual defaults
        let defaults = [
            -0.5f32, 0.5, -10.0, 10.0, 0.0, 100.0, 0.0, 20.0, -10.0, 10.0,
        ];
        for (i, v) in defaults.iter().enumerate() {
            m.process_feedback_frame(sdo_rsp(
                1,
                0x43,
                od::MIT_COMPRESSED,
                units::MIT_RANGE_SUBS[i],
                f32p(*v),
            ))
            .expect("feed");
        }
        // max out every field: pos 0.5 Rev = π rad; vel -10 Rev/s;
        // kp 100 Nm/Rev = 100/2π Nm/rad; kd 20 Nm·s/Rev = 20/2π; Tff -10 Nm
        let t2 = std::f32::consts::TAU;
        m.send_mit_compressed(
            0.5 * t2,
            -10.0 * t2,
            100.0 / t2,
            20.0 / t2,
            -10.0,
            TIMEOUT,
        )
        .expect("compressed");

        let sent = sent_frames(&bus);
        let frame = sent.last().expect("rpdo frame");
        assert_eq!(frame.arbitration_id, 0x501);
        assert_eq!(frame.dlc, 8);
        // golden word: P=0xFFFF, V=0x000, Kp=0xFFF, Kd=0xFFF, T=0x000
        // word  = 0xFFFF_000F_FFFF_F000
        //         (Kp bits 35:24 and Kd bits 23:12 are both 0xFFF)
        // low32 = 0xFFFF_F000, high32 = 0xFFFF_000F
        assert_eq!(
            frame.data,
            [0x00, 0xF0, 0xFF, 0xFF, 0x0F, 0x00, 0xFF, 0xFF]
        );

        // ranges are cached: a second command adds only the RPDO frame
        // (mode cached, rpdo cached, ranges cached)
        let before = sent.len();
        m.send_mit_compressed(0.0, 0.0, 0.0, 0.0, 0.0, TIMEOUT)
            .expect("second");
        assert_eq!(sent_frames(&bus).len(), before + 1);
    }

    #[test]
    fn set_zero_state_guard() {
        let bus = Arc::new(MockBus::new());
        let m = motor(&bus);
        // drive reports Operation Enabled → preset rejected
        m.process_feedback_frame(sdo_rsp(1, 0x43, 0x6041, 0x00, u16p(0x0237)))
            .expect("feed");
        let err = m.set_zero_position(TIMEOUT).expect_err("guard");
        assert!(matches!(err, MotorError::InvalidArgument(_)));
        assert!(err.to_string().contains("Switch On Disabled"));
        // nothing was written to 3001h
        assert!(!sent_frames(&bus)
            .iter()
            .any(|f| f.data[1..4] == [0x01, 0x30, 0x01]));

        // in Switch On Disabled (bit6 set, e.g. 0x0040): value write + code
        m.process_feedback_frame(sdo_rsp(1, 0x43, 0x6041, 0x00, u16p(0x0040)))
            .expect("feed");
        m.process_feedback_frame(ack(1, od::POSITION_PRESET, 0x01)).expect("feed");
        m.process_feedback_frame(ack(1, od::POSITION_PRESET, 0x02)).expect("feed");
        m.set_zero_position(TIMEOUT).expect("zero");

        let sent = sent_frames(&bus);
        let value = sent
            .iter()
            .find(|f| f.data[1..4] == [0x01, 0x30, 0x01])
            .expect("3001h_1");
        assert_eq!(f32::from_le_bytes([value.data[4], value.data[5], value.data[6], value.data[7]]), 0.0);
        let code = sent
            .iter()
            .find(|f| f.data[1..4] == [0x01, 0x30, 0x02])
            .expect("3001h_2");
        assert_eq!(code.data[0], 0x23); // u32 download
        assert_eq!(
            u32::from_le_bytes([code.data[4], code.data[5], code.data[6], code.data[7]]),
            od::POSITION_PRESET_CODE
        );

        // out-of-range preset value rejected (still Switch On Disabled)
        m.process_feedback_frame(sdo_rsp(1, 0x43, 0x6041, 0x00, u16p(0x0040)))
            .expect("feed");
        let err = m.position_preset(1.0, TIMEOUT).expect_err("range");
        assert!(matches!(err, MotorError::InvalidArgument(_)));
    }

    #[test]
    fn tpdo_anchor_patch() {
        let bus = Arc::new(MockBus::new());
        let m = motor(&bus);
        let t2 = std::f32::consts::TAU;

        // TPDO2 anchor: 1.5 Rev, -0.25 Rev/s → seq 1
        let mut buf = [0u8; 8];
        buf[0..4].copy_from_slice(&1.5f32.to_le_bytes());
        buf[4..8].copy_from_slice(&(-0.25f32).to_le_bytes());
        m.process_feedback_frame(tpdo_frame(0x281, &buf)).expect("feed");
        let s = m.latest_state().expect("state");
        assert_eq!(s.seq, 1);
        assert!((s.pos_rad - 1.5 * t2).abs() < 1e-3);
        assert!((s.vel_rad_s + 0.25 * t2).abs() < 1e-3);

        // TPDO1 patch: error code / statusword / mode — seq stays 1
        m.process_feedback_frame(tpdo_frame(0x181, &[0x10, 0x23, 0x37, 0x02, 5]))
            .expect("feed");
        let s = m.latest_state().expect("state");
        assert_eq!(s.seq, 1);
        assert_eq!(s.error_code, 0x2310);
        assert_eq!(s.statusword, 0x0237);
        assert_eq!(s.mode_display, 5);

        // TPDO3 patch: torque permille + temperatures — seq stays 1
        m.process_feedback_frame(tpdo_frame(0x381, &[0x06, 0xFF, 0xFF, 0x00, 0x9C, 0xFF]))
            .expect("feed");
        let s = m.latest_state().expect("state");
        assert_eq!(s.seq, 1);
        assert_eq!(s.torque_permille, -250);
        assert!((s.t_driver_c - 25.5).abs() < 1e-4);
        assert!((s.t_motor_c + 10.0).abs() < 1e-4);
        // peak torque unknown → torque_nm stays 0 (documented)
        assert_eq!(s.torque_nm, 0.0);

        // second anchor advances the sequence
        m.process_feedback_frame(tpdo_frame(0x281, &buf)).expect("feed");
        assert_eq!(m.latest_state().expect("state").seq, 2);

        // after warming the peak, patches convert to Nm
        m.process_feedback_frame(sdo_rsp(1, 0x43, MOTOR_PEAK_TORQUE, 0x00, f32p(10.0)))
            .expect("feed");
        m.peak_torque_nm(TIMEOUT).expect("peak");
        m.process_feedback_frame(tpdo_frame(0x381, &[0xF4, 0x01, 0xFF, 0x00, 0x9C, 0xFF]))
            .expect("feed");
        let s = m.latest_state().expect("state");
        assert_eq!(s.torque_permille, 500);
        assert!((s.torque_nm - 5.0).abs() < 1e-4);
    }

    #[test]
    fn emcy_error_code() {
        let bus = Arc::new(MockBus::new());
        let m = motor(&bus);
        // EMCY patch before any anchor: dropped, not fabricated — a blank
        // state would report a valid-looking pos=0/vel=0. The error code
        // itself stays observable through the dedicated cache.
        m.process_feedback_frame(tpdo_frame(0x081, &[0x10, 0x10, 0x23, 0x00]))
            .expect("feed");
        assert!(m.latest_state().is_none());
        assert_eq!(m.cached_error_code(), Some(0x2310));

        // anchor first (TPDO2: 1 Rev, 0 Rev/s → seq 1), then the EMCY
        // patch lands on the anchored state without advancing the seq
        let mut buf = [0u8; 8];
        buf[0..4].copy_from_slice(&1.0f32.to_le_bytes());
        m.process_feedback_frame(tpdo_frame(0x281, &buf)).expect("feed");
        m.process_feedback_frame(tpdo_frame(0x081, &[0x10, 0x10, 0x23, 0x00]))
            .expect("feed");
        let s = m.latest_state().expect("state");
        assert_eq!(s.error_code, 0x2310);
        assert_eq!(s.error_description(), Some("相线过流 (phase overcurrent)"));
        assert_eq!(s.seq, 1); // patch semantics: seq stays at the anchor
    }

    #[test]
    fn nmt_reset_node_invalidates_caches() {
        let bus = Arc::new(MockBus::new());
        let m = motor(&bus);
        queue_mode_enabled(&m, 1, 1);
        m.ensure_mode_enabled(RunMode::ProfilePosition, TIMEOUT)
            .expect("mode");
        m.process_feedback_frame(ack(1, 0x6081, 0x00)).expect("feed");
        m.process_feedback_frame(ack(1, 0x607A, 0x00)).expect("feed");
        m.process_feedback_frame(ack(1, 0x6040, 0x00)).expect("feed");
        m.send_pos_vel(0.5, 1.0, TIMEOUT).expect("pp");
        m.tpdos_active.store(true, Ordering::Release);
        assert_eq!(*m.pp.lock().expect("pp"), PpHandshake::High);

        m.nmt(NmtCommand::ResetNode, 1).expect("nmt");

        let sent = sent_frames(&bus);
        let nmt = sent.last().expect("nmt frame");
        assert_eq!(nmt.arbitration_id, 0);
        assert_eq!([nmt.data[0], nmt.data[1]], [0x81, 0x01]);
        // all three caches dropped
        assert!(m.mode_ready.lock().expect("mode").is_none());
        assert_eq!(*m.pp.lock().expect("pp"), PpHandshake::Low);
        assert!(!m.tpdos_active.load(Ordering::Acquire));

        // observable effect: the next send_pos_vel re-runs the full mode
        // round trip and uses only the set edge — the drive controlword is
        // back at its power-up value (bit 4 low), so no 1→0→1 toggle
        queue_mode_enabled(&m, 1, 1);
        m.process_feedback_frame(ack(1, 0x6081, 0x00)).expect("feed");
        m.process_feedback_frame(ack(1, 0x607A, 0x00)).expect("feed");
        m.process_feedback_frame(ack(1, 0x6040, 0x00)).expect("feed");
        m.send_pos_vel(0.5, 1.0, TIMEOUT).expect("pp again");

        let sent = sent_frames(&bus);
        let nmt_at = sent
            .iter()
            .rposition(|f| f.arbitration_id == 0 && f.data[0] == 0x81)
            .expect("nmt frame");
        let words: Vec<u16> = sent[nmt_at + 1..]
            .iter()
            .filter(|f| f.data[1..4] == [0x40, 0x60, 0x00])
            .map(|f| u16::from_le_bytes([f.data[4], f.data[5]]))
            .collect();
        assert_eq!(words, vec![0x003F]);
        // the mode cache was dropped too: a fresh 6060h=1 write landed
        assert!(sent[nmt_at + 1..]
            .iter()
            .any(|f| f.data[1..4] == [0x60, 0x60, 0x00] && f.data[4] == 1));
    }

    #[test]
    fn nmt_cache_invalidation_matrix() {
        let bus = Arc::new(MockBus::new());
        let m = motor(&bus);
        queue_mode_enabled(&m, 1, 5);
        m.ensure_mode_enabled(RunMode::Mit, TIMEOUT).expect("mode");
        *m.pp.lock().expect("pp") = PpHandshake::High;
        m.tpdos_active.store(true, Ordering::Release);

        // reset of an unrelated node: frame sent, caches untouched
        m.nmt(NmtCommand::ResetNode, 2).expect("nmt other");
        assert!(m.mode_ready.lock().expect("mode").is_some());
        assert_eq!(*m.pp.lock().expect("pp"), PpHandshake::High);
        assert!(m.tpdos_active.load(Ordering::Acquire));

        // broadcast reset communication: only the TPDO flag drops
        m.nmt(NmtCommand::ResetCommunication, 0).expect("nmt broadcast");
        assert!(m.mode_ready.lock().expect("mode").is_some());
        assert_eq!(*m.pp.lock().expect("pp"), PpHandshake::High);
        assert!(!m.tpdos_active.load(Ordering::Acquire));

        // start remote node: nothing changes
        m.nmt(NmtCommand::StartRemoteNode, 1).expect("start");
        assert!(m.mode_ready.lock().expect("mode").is_some());
        assert!(!m.tpdos_active.load(Ordering::Acquire));

        let sent = sent_frames(&bus);
        let cmds: Vec<(u8, u8)> = sent
            .iter()
            .filter(|f| f.arbitration_id == 0)
            .map(|f| (f.data[0], f.data[1]))
            .collect();
        assert_eq!(cmds, vec![(0x81, 0x02), (0x82, 0x00), (0x01, 0x01)]);
    }

    #[test]
    fn controlled_stop_pp_retargets_with_handshake() {
        let bus = Arc::new(MockBus::new());
        let m = motor(&bus);
        queue_mode_enabled(&m, 1, 1);
        m.ensure_mode_enabled(RunMode::ProfilePosition, TIMEOUT)
            .expect("mode");
        // actual position 0.25 Rev, target write ack, controlword ack
        m.process_feedback_frame(sdo_rsp(1, 0x43, POSITION_ACTUAL_VALUE, 0x00, f32p(0.25)))
            .expect("feed");
        m.process_feedback_frame(ack(1, TARGET_POSITION, 0x00)).expect("feed");
        m.process_feedback_frame(ack(1, 0x6040, 0x00)).expect("feed");
        m.controlled_stop(TIMEOUT).expect("stop");

        let sent = sent_frames(&bus);
        // re-target = current actual position
        let target = sent
            .iter()
            .find(|f| f.data[1..4] == [0x7A, 0x60, 0x00])
            .expect("607A write");
        let p = f32::from_le_bytes([target.data[4], target.data[5], target.data[6], target.data[7]]);
        assert!((p - 0.25).abs() < 1e-5);
        // first stop: single set edge (pp starts Low)
        let words: Vec<u16> = sent
            .iter()
            .filter(|f| f.data[1..4] == [0x40, 0x60, 0x00])
            .map(|f| u16::from_le_bytes([f.data[4], f.data[5]]))
            .collect();
        assert_eq!(words, vec![0x003F]);

        // second stop: same 1→0→1 toggle send_pos_vel uses — without the
        // set edge the drive would keep moving toward the old target
        m.process_feedback_frame(sdo_rsp(1, 0x43, POSITION_ACTUAL_VALUE, 0x00, f32p(0.25)))
            .expect("feed");
        m.process_feedback_frame(ack(1, TARGET_POSITION, 0x00)).expect("feed");
        for _ in 0..2 {
            m.process_feedback_frame(ack(1, 0x6040, 0x00)).expect("feed");
        }
        m.controlled_stop(TIMEOUT).expect("stop again");

        let sent = sent_frames(&bus);
        let words: Vec<u16> = sent
            .iter()
            .filter(|f| f.data[1..4] == [0x40, 0x60, 0x00])
            .map(|f| u16::from_le_bytes([f.data[4], f.data[5]]))
            .collect();
        assert_eq!(words, vec![0x003F, 0x002F, 0x003F]);
    }

    #[test]
    fn store_signature() {
        let bus = Arc::new(MockBus::new());
        let m = motor(&bus);
        m.process_feedback_frame(ack(1, od::STORE_PARAMS, 0x01)).expect("feed");
        m.store_parameters(TIMEOUT).expect("store");

        let sent = sent_frames(&bus);
        assert_eq!(sent.len(), 1);
        assert_eq!(sent[0].data[1..4], [0x10, 0x10, 0x01]);
        assert_eq!(sent[0].data[0], 0x23);
        assert_eq!(
            u32::from_le_bytes([sent[0].data[4], sent[0].data[5], sent[0].data[6], sent[0].data[7]]),
            0x6576_6173 // "save"
        );
    }

    #[test]
    fn query_status_anchors_and_converts() {
        let bus = Arc::new(MockBus::new());
        let m = motor(&bus);
        // one full SDO round trip in read order
        m.process_feedback_frame(sdo_rsp(1, 0x43, 0x6061, 0x00, [5, 0, 0, 0])).expect("feed");
        m.process_feedback_frame(sdo_rsp(1, 0x43, 0x6041, 0x00, u16p(0x0237))).expect("feed");
        m.process_feedback_frame(sdo_rsp(1, 0x43, ERROR_CODE, 0x00, u16p(0))).expect("feed");
        m.process_feedback_frame(sdo_rsp(1, 0x43, POSITION_ACTUAL_VALUE, 0x00, f32p(0.25)))
            .expect("feed");
        m.process_feedback_frame(sdo_rsp(
            1,
            0x43,
            motor_canopen::objects::VELOCITY_ACTUAL_VALUE,
            0x00,
            f32p(-1.0),
        ))
        .expect("feed");
        m.process_feedback_frame(sdo_rsp(
            1,
            0x43,
            motor_canopen::objects::TORQUE_ACTUAL_VALUE,
            0x00,
            i16p(-250),
        ))
        .expect("feed");
        m.process_feedback_frame(sdo_rsp(1, 0x43, MOTOR_PEAK_TORQUE, 0x00, f32p(8.0))).expect("feed");
        m.process_feedback_frame(sdo_rsp(1, 0x43, od::TEMPERATURES, 0x01, i16p(300))).expect("feed");
        m.process_feedback_frame(sdo_rsp(1, 0x43, od::TEMPERATURES, 0x02, i16p(250))).expect("feed");

        let s = m.query_status(TIMEOUT).expect("status");
        assert_eq!(s.mode_display, 5);
        assert_eq!(s.statusword, 0x0237);
        let t2 = std::f32::consts::TAU;
        assert!((s.pos_rad - 0.25 * t2).abs() < 1e-3);
        assert!((s.vel_rad_s + 1.0 * t2).abs() < 1e-3);
        assert_eq!(s.torque_permille, -250);
        assert!((s.torque_nm + 2.0).abs() < 1e-4); // -250‰ of 8 Nm
        assert!((s.t_driver_c - 30.0).abs() < 1e-4);
        assert!((s.t_motor_c - 25.0).abs() < 1e-4);
        assert_eq!(s.seq, 1);
        assert_eq!(m.latest_state().expect("cached").seq, 1);
    }

    fn tpdo_frame(id: u32, payload: &[u8]) -> CanFrame {
        let mut data = [0u8; 8];
        data[..payload.len()].copy_from_slice(payload);
        CanFrame {
            arbitration_id: id,
            data,
            dlc: payload.len() as u8,
            is_extended: false,
            is_rx: true,
        }
    }
}
