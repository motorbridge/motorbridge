use motor_core::bus::CanFrame;
use std::f32::consts::PI;

const TWO_PI: f32 = PI * 2.0;

#[derive(Debug, Clone, Copy)]
pub struct HightorqueFeedbackState {
    pub can_id: u8,
    pub arbitration_id: u32,
    /// Run mode (table 2). Zero in register read replies; byte0 in status feedback frames.
    pub status_code: u8,
    /// Fault code (table 3). Zero in register read replies; byte1 in status feedback frames.
    pub fault_code: u8,
    pub pos: f32,
    pub vel: f32,
    pub torq: f32,
    pub t_mos: f32,
    pub t_rotor: f32,
}

impl HightorqueFeedbackState {
    /// Parses `status_code` (byte0) into a run mode; register read replies
    /// (status_code=0, not a status frame) and unknown codes return `None`.
    pub fn run_mode(&self) -> Option<RunMode> {
        RunMode::from_code(self.status_code)
    }

    /// Human-readable fault code description (table 3).
    pub fn fault_description(&self) -> &'static str {
        decode_fault(self.fault_code)
    }
}

/// Run mode (table 2). byte0 of a status feedback frame is this value.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RunMode {
    Stop = 0,               // stop / clear error
    Error = 1,              // fault
    Ready2 = 2,             // ready
    Ready3 = 3,             // ready
    Ready4 = 4,             // ready
    Pwm = 5,                // PWM output
    Voltage = 6,            // voltage output
    FocVoltage = 7,         // FOC voltage
    DqVoltage = 8,          // DQ voltage
    DqCurrent = 9,          // DQ current
    Position = 10,          // position mode
    Timeout = 11,           // timeout
    ZeroSpeed = 12,         // zero speed
    Range = 13,             // range
    MeasureInductance = 14, // measure inductance
    Brake = 15,             // brake
    Reserved16 = 16,
    Reserved17 = 17,
}

impl RunMode {
    /// Parses a run mode from byte0; codes above 17 return `None`.
    pub fn from_code(code: u8) -> Option<Self> {
        Some(match code {
            0 => Self::Stop,
            1 => Self::Error,
            2 => Self::Ready2,
            3 => Self::Ready3,
            4 => Self::Ready4,
            5 => Self::Pwm,
            6 => Self::Voltage,
            7 => Self::FocVoltage,
            8 => Self::DqVoltage,
            9 => Self::DqCurrent,
            10 => Self::Position,
            11 => Self::Timeout,
            12 => Self::ZeroSpeed,
            13 => Self::Range,
            14 => Self::MeasureInductance,
            15 => Self::Brake,
            16 => Self::Reserved16,
            17 => Self::Reserved17,
            _ => return None,
        })
    }
}

/// Fault code (table 3) → human-readable description. Codes 8-31 and >47 are
/// undefined by the spec and return "unknown fault code".
pub fn decode_fault(code: u8) -> &'static str {
    match code {
        0 => "正常",
        // 1-7: UART/DMA related errors (spec names none individually; grouped together)
        1..=7 => "UART/DMA 错误",
        32 => "校准错误",
        33 => "驱动故障",
        34 => "过压",
        35 => "编码器错误",
        36 => "未校准",
        37 => "PWM 周期错误",
        38 => "过温",
        39 => "起始位置超范围",
        40 => "欠压",
        41 => "配置已变更",
        42 => "无效角度",
        43 => "无效位置",
        44 => "驱动使能故障",
        45 => "停止位置误用",
        46 => "时序错误",
        47 => "反电动势前馈错误",
        _ => "未知故障码",
    }
}

/// Extract the motor id from a reply frame's arbitration ID.
///
/// Per the Hightorque CAN protocol (v2.0.0 firmware `motor.c::motor_process_state_all`,
/// `id = fdcan_rx_header.Identifier >> 8`), the motor always places its id in the
/// *high* byte of the reply ID (`source = motor_id`, `dest = 0`, reply bit = 0),
/// for both standard 11-bit and extended 29-bit frames. The previous extended-frame
/// branch read the low byte (`& 0x7F`), which only matched `motor_id == 0`.
///
/// Returns `None` for frames addressed to a non-zero destination (other hosts) —
/// replies to host 0 always carry `dest = 0`.
pub(crate) fn reply_motor_id(frame: &CanFrame) -> Option<u8> {
    if (frame.arbitration_id & 0x00FF) != 0 {
        return None;
    }
    Some(((frame.arbitration_id >> 8) & 0x7F) as u8)
}

/// Parses the `0x27 0x01` status query reply (reference firmware motor.c
/// `len==8 && p_data[0]==0x27` branch): only pos/vel/tqe as three
/// little-endian int16 fields.
///
/// The frame carries no run mode/fault code/temperature; this function
/// fills those fields with 0/0.0 — **that is not "value is zero"**. The
/// cache side (motor.rs `process_feedback_frame`) merges under the
/// absent-fields-are-not-overwritten rule, preserving previous values
/// (2026-10-23 decision).
pub(crate) fn decode_read_reply(
    frame: CanFrame,
    c: TorqueCoeff,
) -> Option<HightorqueFeedbackState> {
    if frame.dlc < 8 {
        return None;
    }
    if frame.data[0] != 0x27 || frame.data[1] != 0x01 {
        return None;
    }
    let can_id = reply_motor_id(&frame)?;
    let pos_raw = i16::from_le_bytes([frame.data[2], frame.data[3]]);
    let vel_raw = i16::from_le_bytes([frame.data[4], frame.data[5]]);
    let tqe_raw = i16::from_le_bytes([frame.data[6], frame.data[7]]);
    Some(HightorqueFeedbackState {
        can_id,
        arbitration_id: frame.arbitration_id,
        status_code: 0,
        fault_code: 0,
        pos: pos_raw as f32 * 0.0001 * TWO_PI,
        vel: vel_raw as f32 * 0.00025 * TWO_PI,
        torq: tqe_restore_raw(tqe_raw, c),
        t_mos: 0.0,
        t_rotor: 0.0,
    })
}

/// Parses the 8-byte status feedback frame (§3.2 format, reference firmware
/// `motor.c::motor_process_state` `len==8 && p_data[0]!=0x27` branch):
/// byte0=mode (table 2), byte1=fault (table 3), bytes 2-7 =
/// pos/vel/tqe int16 (little-endian), same scaling as register read replies.
///
/// ⚠ On v2.0.0 real hardware this frame has no known producer (2026-10-09
/// candump: control commands with bit15 set still get only a DLC=0 empty
/// frame back, see can_protocol §1.2/§3.2 corrections; periodic reports
/// use the `27 01` format) — this path is kept because the reference
/// firmware / official PDF define the format, so it can still appear under
/// older firmware or unknown triggers. The frame carries no temperature;
/// t_mos/t_rotor are filled with 0.0 — the cache side merges under the
/// absent-fields-are-not-overwritten rule, preserving previous values
/// (2026-10-23 decision).
///
/// Register read replies (0x27...), setting ACKs (0x41, 7 bytes) and
/// firmware version replies (5 bytes) all fail this condition and return
/// `None`; each is handled by its own path.
pub(crate) fn decode_status_frame(
    frame: &CanFrame,
    c: TorqueCoeff,
) -> Option<HightorqueFeedbackState> {
    if frame.dlc < 8 || frame.data[0] == 0x27 {
        return None;
    }
    let can_id = reply_motor_id(frame)?;
    let pos_raw = i16::from_le_bytes([frame.data[2], frame.data[3]]);
    let vel_raw = i16::from_le_bytes([frame.data[4], frame.data[5]]);
    let tqe_raw = i16::from_le_bytes([frame.data[6], frame.data[7]]);
    Some(HightorqueFeedbackState {
        can_id,
        arbitration_id: frame.arbitration_id,
        status_code: frame.data[0],
        fault_code: frame.data[1],
        pos: pos_raw as f32 * 0.0001 * TWO_PI,
        vel: vel_raw as f32 * 0.00025 * TWO_PI,
        torq: tqe_restore_raw(tqe_raw, c),
        t_mos: 0.0,
        t_rotor: 0.0,
    })
}

/// Passive feedback dispatch: try register read reply first, then status feedback frame.
pub(crate) fn decode_feedback(frame: CanFrame, c: TorqueCoeff) -> Option<HightorqueFeedbackState> {
    decode_read_reply(frame, c).or_else(|| decode_status_frame(&frame, c))
}

// ----- Setting ACK (reference firmware motor.c setting-parse branch) -----
//
// After a setting-class command succeeds, the motor replies with a 7-byte
// ACK: `0x41 0x01 0x04` + `"OK\r\n"`. Such frames are transactional
// replies, not passive feedback (decode_feedback returns None); they are
// consumed by the active read path (send_with_ack).

/// Success ACK of a setting-class command (7 bytes): `0x41 0x01 0x04` + `"OK\r\n"`.
pub const SETTING_ACK: [u8; 7] = [0x41, 0x01, 0x04, 0x4F, 0x4B, 0x0D, 0x0A];

/// Tells whether a frame is a setting ACK (dlc==7 and the first 7 bytes
/// exactly match `SETTING_ACK`).
pub(crate) fn is_setting_ack(frame: &CanFrame) -> bool {
    frame.dlc == 7 && frame.data.starts_with(&SETTING_ACK)
}

// ----- P2-16 unit and data type selection -----
//
// Mirrors the reference firmware `convert.c`: `pos_vel_type_t`
// (RADIAN_2PI/ANGLE_360/TURNS, default `MOTOR_DATA_TYPE_FLAG = TURNS`) and
// `data_type_t` (TFLOAT/TINT32/TINT16; TINT8 is unimplemented in the
// firmware switch, but §1.3 cmd bits include int8 and the rint8 scale is
// still passed in). Write path: `conv_to_turns(v,unit) →
// xxx_float2int(turns,type) = turns*rintX`; read path:
// `xxx_int2float(raw,type) = raw/rintX → conv_from_turns(turns,unit)`.

/// Angle unit (reference firmware `pos_vel_type_t`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AngleUnit {
    /// Radians (`RADIAN_2PI`, firmware `MY_2PI` basis). Crate default.
    Rad,
    /// Degrees (`ANGLE_360`, 0–360°).
    Degree,
    /// Turns (`TURNS`, firmware default `MOTOR_DATA_TYPE_FLAG`).
    Turn,
}

/// Data type (reference firmware `data_type_t` + §1.3 cmd half-bits `[3:2]`).
///
/// Note: the firmware `data_float2int`/`data_int2float` switch does not
/// implement `TINT8` (default → `MOTOR_ERR()`), but the §1.3 cmd bit
/// encoding includes int8 and every `*_float2int` still receives the rint8
/// scale, so `Int8` is kept here with that scale.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DataType {
    Int8,
    Int16,
    Int32,
    Float,
}

/// Raw scale of a physical quantity under Int8/Int16/Int32 (Float is the
/// identity 1.0 and skips the table). Mirrors the rint8/rint16/rint32
/// arguments of each `*_float2int` in `convert.c`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct QuantityScale {
    pub int8: f32,
    pub int16: f32,
    pub int32: f32,
}

impl QuantityScale {
    pub(crate) const fn scale(self, ty: DataType) -> f32 {
        match ty {
            DataType::Int8 => self.int8,
            DataType::Int16 => self.int16,
            DataType::Int32 => self.int32,
            DataType::Float => 1.0,
        }
    }
}

/// Position raw scale (mirrors `pos_float2int`: rint8=100, rint16=10000, rint32=100000).
pub const POS_SCALE: QuantityScale = QuantityScale {
    int8: 100.0,
    int16: 10000.0,
    int32: 100000.0,
};
/// Velocity raw scale (mirrors `vel_float2int`: rint8=100, rint16=4000, rint32=100000).
pub const VEL_SCALE: QuantityScale = QuantityScale {
    int8: 100.0,
    int16: 4000.0,
    int32: 100000.0,
};
/// Torque raw scale (mirrors `tqe_float2int`: rint8=2, rint16=100, rint32=1000).
pub const TQE_SCALE: QuantityScale = QuantityScale {
    int8: 2.0,
    int16: 100.0,
    int32: 1000.0,
};
/// Acceleration raw scale (mirrors `acc_float2int`: rint8=20, rint16=1000, rint32=100000).
pub const ACC_SCALE: QuantityScale = QuantityScale {
    int8: 20.0,
    int16: 1000.0,
    int32: 100000.0,
};
/// Current raw scale (mirrors `cur_float2int`: rint8=1, rint16=10, rint32=1000).
pub const CUR_SCALE: QuantityScale = QuantityScale {
    int8: 1.0,
    int16: 10.0,
    int32: 1000.0,
};
/// Voltage raw scale (mirrors `vol_float2int`: rint8=2, rint16=10, rint32=1000).
pub const VOL_SCALE: QuantityScale = QuantityScale {
    int8: 2.0,
    int16: 10.0,
    int32: 1000.0,
};
/// PID raw scale (mirrors `pid_float2int`: rint8=1, rint16=10, rint32=1000).
pub const PID_SCALE: QuantityScale = QuantityScale {
    int8: 1.0,
    int16: 10.0,
    int32: 1000.0,
};
/// Temperature raw scale (vendor register unit table: int16 0.1 °C/LSB;
/// added 2026-10-22 — the user-supplied conversion is one per quantity,
/// so all three columns share the value).
pub const TEMP_SCALE: QuantityScale = QuantityScale {
    int8: 10.0,
    int16: 10.0,
    int32: 10.0,
};
/// Time raw scale (vendor register unit table: int16 0.001 s/LSB; the
/// current register table 0x00-0x45 has no time register — range defined
/// for future use).
pub const TIME_SCALE: QuantityScale = QuantityScale {
    int8: 1000.0,
    int16: 1000.0,
    int32: 1000.0,
};
/// PWM / rKp / rKd (0x23/0x24) raw scale (vendor register unit table:
/// normalized 1/32767, i.e. raw=32767 corresponds to full scale 1.0).
pub const PWM_SCALE: QuantityScale = QuantityScale {
    int8: 32767.0,
    int16: 32767.0,
    int32: 32767.0,
};

/// Unit → turns (mirrors `conv_to_turns`).
pub fn to_turns(value: f32, unit: AngleUnit) -> f32 {
    match unit {
        AngleUnit::Rad => value / TWO_PI,
        AngleUnit::Degree => value / 360.0,
        AngleUnit::Turn => value,
    }
}

/// Turns → unit (mirrors `conv_from_turns`).
pub fn from_turns(turns: f32, unit: AngleUnit) -> f32 {
    match unit {
        AngleUnit::Rad => turns * TWO_PI,
        AngleUnit::Degree => turns * 360.0,
        AngleUnit::Turn => turns,
    }
}

/// Symmetric raw saturation limit per data type (mirrors the max arguments
/// of `convert.c::data_limit`: TINT16=32760, TINT32=2147483640). Note: the
/// firmware `data_limit` has a typo on the negative side (`return -min` →
/// negative saturation returns a positive value); corrected here to
/// symmetric saturation so a negative overflow is not mistaken for
/// positive full scale.
const fn int_clamp_limit(ty: DataType) -> f32 {
    match ty {
        DataType::Int8 => 127.0,
        DataType::Int16 => 32760.0,
        DataType::Int32 => 2_147_483_640.0,
        DataType::Float => f32::INFINITY,
    }
}

/// Write: physical quantity (in the given unit) → float raw (the unit is
/// first converted to turns via `conv_to_turns`, then multiplied by the
/// scale; mirrors `xxx_float2int`'s `in_data * rintX`). Symmetrically
/// saturated to the integer limit of `ty`. `DataType::Float` is returned
/// as-is (no scale, no saturation).
pub fn encode_unit_raw(value: f32, unit: AngleUnit, ty: DataType, scale: QuantityScale) -> f32 {
    if ty == DataType::Float {
        return value;
    }
    let turns = to_turns(value, unit);
    let raw = turns * scale.scale(ty);
    let lim = int_clamp_limit(ty);
    raw.clamp(-lim, lim)
}

/// Read: float raw → physical quantity (in the given unit; mirrors
/// `xxx_int2float`'s `raw / rintX` followed by `conv_from_turns`).
/// `DataType::Float` is returned as-is.
pub fn decode_unit_raw(raw: f32, unit: AngleUnit, ty: DataType, scale: QuantityScale) -> f32 {
    if ty == DataType::Float {
        return raw;
    }
    let turns = raw / scale.scale(ty);
    from_turns(turns, unit)
}

/// Convenience path to i16 raw (Rad+Int16, as the crate's existing command paths use).
fn to_i16_raw(value: f32, unit: AngleUnit, scale: QuantityScale) -> i16 {
    let raw = encode_unit_raw(value, unit, DataType::Int16, scale);
    (raw.round() as i32).clamp(i16::MIN as i32, i16::MAX as i32) as i16
}

pub(crate) fn rad_to_pos_raw(rad: f32) -> i16 {
    to_i16_raw(rad, AngleUnit::Rad, POS_SCALE)
}

pub(crate) fn radps_to_vel_raw(radps: f32) -> i16 {
    to_i16_raw(radps, AngleUnit::Rad, VEL_SCALE)
}

/// Torque compensation coefficients {k, d} (reference firmware `motor_tqe_adj_t`).
///
/// Write torque: `raw = tqe_adjust(tau) * 100 = ((tau - d) / k) * 100`;
/// read torque: `tau = tqe_restore(raw * 0.01) = raw * 0.01 * k + d`.
/// On current firmware `d` is 0 for every model (the d values in comments are disabled).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TorqueCoeff {
    pub k: f32,
    pub d: f32,
}

/// Motor model (reference firmware `motor_type_t`). Determines the torque
/// raw↔Nm compensation coefficients. Variant names mirror the firmware
/// enum codes (e.g. `M60SG_35`); not camel-cased, to stay comparable
/// against the source.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(non_camel_case_types)]
pub enum MotorModel {
    Null,
    M3536_32,
    M4438_30,
    M4438_32,
    M4538_19,
    M5043_20,
    M5046_20,
    M5047_09,
    M5047_36,
    /// HTDW-5036-02-CNE (50mm planetary joint, gear ratio 36). k=0.64 taken
    /// from the M5047_36 row of the vendor register unit table's k table
    /// (2026-10-20 decision); the module manual's torque constant
    /// Kt=0.54 Nm/A is a different concept and is not used as the
    /// compensation coefficient (firmware table lacks this model, pending calibration).
    M5036_36,
    M6056_36,
    M7256_35,
    M60SG_35,
    M60BM_35,
    /// Generic / no specific model (k=0.5, d=0). The firmware uses this when the model is unknown.
    General,
}

impl MotorModel {
    /// Torque compensation coefficients (mirrors the reference firmware `motor_tqe_adj[]` item by item).
    pub const fn coeff(self) -> TorqueCoeff {
        match self {
            Self::Null => TorqueCoeff { k: 0.0, d: 0.0 },
            Self::M3536_32 => TorqueCoeff {
                k: 0.458_105,
                d: 0.0,
            },
            Self::M4438_30 => TorqueCoeff {
                k: 0.525_600,
                d: 0.0,
            },
            Self::M4438_32 => TorqueCoeff {
                k: 0.485_565,
                d: 0.0,
            },
            Self::M4538_19 => TorqueCoeff {
                k: 0.493_835,
                d: 0.0,
            },
            Self::M5043_20 => TorqueCoeff { k: 0.966, d: 0.0 },
            Self::M5046_20 => TorqueCoeff {
                k: 0.533_654,
                d: 0.0,
            },
            Self::M5047_09 => TorqueCoeff {
                k: 0.547_474,
                d: 0.0,
            },
            // 2026-10-20 decision: take 0.64 from the Feishu unit table;
            // firmware motor_tqe_adj is 0.35, identical across both firmware
            // versions, but dropped per the vendor-docs-over-firmware
            // principle (doc wins while the dispute awaits vendor confirmation).
            Self::M5047_36 => TorqueCoeff { k: 0.64, d: 0.0 },
            Self::M5036_36 => TorqueCoeff { k: 0.64, d: 0.0 },
            Self::M6056_36 => TorqueCoeff { k: 0.677, d: 0.0 },
            Self::M7256_35 => TorqueCoeff {
                k: 0.676_524,
                d: 0.0,
            },
            Self::M60SG_35 => TorqueCoeff { k: 0.794_2, d: 0.0 },
            Self::M60BM_35 => TorqueCoeff { k: 0.794_2, d: 0.0 },
            Self::General => TorqueCoeff { k: 0.5, d: 0.0 },
        }
    }

    /// Parses from a model hint string. Accepts case and separator variants
    /// like `5046-20` / `5046_20` / `M5046_20` / `ht-5046-20` / `5036` /
    /// `ht-5036`; generic placeholders (`""`/`hightorque`/`ht`/`auto`/
    /// `default`/`general`) return [`MotorModel::General`]; unrecognized
    /// strings return `None`.
    pub fn from_hint(hint: &str) -> Option<Self> {
        let s = hint.trim().to_ascii_lowercase();
        match s.as_str() {
            "" | "hightorque" | "ht" | "auto" | "default" | "general" => {
                return Some(Self::General)
            }
            "null" | "none" => return Some(Self::Null),
            _ => {}
        }
        let mut t = s.replace('-', "_");
        for prefix in ["hightorque_", "ht_", "m"] {
            if let Some(rest) = t.strip_prefix(prefix) {
                t = rest.to_string();
                break;
            }
        }
        Some(match t.as_str() {
            "3536_32" => Self::M3536_32,
            "4438_30" => Self::M4438_30,
            "4438_32" => Self::M4438_32,
            "4538_19" => Self::M4538_19,
            "5043_20" => Self::M5043_20,
            "5046_20" => Self::M5046_20,
            "5047_09" => Self::M5047_09,
            "5047_36" => Self::M5047_36,
            // HTDW-5036-02-CNE: the suffix follows the SDK convention and is
            // the gear ratio (36), not the commercial code -02. Bare "5036",
            // ratio style "5036_36" and commercial code "5036_02" all map to
            // the single 5036 variant.
            "5036" | "5036_36" | "5036_02" => Self::M5036_36,
            "6056_36" => Self::M6056_36,
            "7256_35" => Self::M7256_35,
            "60sg_35" => Self::M60SG_35,
            "60bm_35" => Self::M60BM_35,
            _ => return None,
        })
    }
}

/// Write torque: real Nm → int16 raw (via `tqe_adjust` + TINT16 scale=100).
/// Returns 0 when `k == 0` (MNULL) to avoid division by zero.
pub(crate) fn tqe_adjust_to_raw(tau_nm: f32, c: TorqueCoeff) -> i16 {
    if c.k == 0.0 {
        return 0;
    }
    let v = (((tau_nm - c.d) / c.k) * 100.0).round() as i32;
    v.clamp(i16::MIN as i32, i16::MAX as i32) as i16
}

/// Read torque: int16 raw → real Nm (via TINT16 scale=0.01 + `tqe_restore`).
pub(crate) fn tqe_restore_raw(raw: i16, c: TorqueCoeff) -> f32 {
    raw as f32 * 0.01 * c.k + c.d
}

/// DQ voltage mode: V → int16 raw (protocol §1 command 6, 0.1 V/LSB). Saturates out of range.
pub(crate) fn volts_to_raw(voltage_v: f32) -> i16 {
    let v = (voltage_v * 10.0).round() as i32;
    v.clamp(i16::MIN as i32, i16::MAX as i32) as i16
}

/// DQ current mode: A → int16 raw (protocol §1 command 6, 0.1 A/LSB). Saturates out of range.
pub(crate) fn amps_to_raw(current_a: f32) -> i16 {
    let v = (current_a * 10.0).round() as i32;
    v.clamp(i16::MIN as i32, i16::MAX as i32) as i16
}

/// Acceleration (rad/s²) → trapezoid mode int16 raw value.
///
/// The reference firmware `motor_set_pos_vel_acc` lands via `conv_to_turns` →
/// `acc_float2int(TINT16)`, where `data_float2int` uses scale=1000 for TINT16
/// (turns/s² → raw), i.e. `acc_raw = acc_turns * 1000 = (acc_rad_s2 / 2π) * 1000`.
///
/// Note: the `libelybot_can.c` comment on `motor_control_pos_vel_acc` says
/// "unit 0.01 turns/s²" (scale 100), inconsistent with the actual `convert.c`
/// code path (scale 1000); this follows the code path. 2026-11-02 resolution:
/// the Feishu unit table and the firmware code agree on scale 1000, so the
/// old comment is a deprecated outlier — final confirmation pending a
/// real-bus trapezoid timing test.
pub(crate) fn radps2_to_acc_raw(radps2: f32) -> i16 {
    to_i16_raw(radps2, AngleUnit::Rad, ACC_SCALE)
}

// ----- MIT mode (motor firmware v4.6.0+) -----
//
// Reference `motor_control_pos_vel_tqe_kp_kd` (`libelybot_can.c`): frame
// ID = `0x18000 | id` (extended 29-bit), 8 bytes bit-packed: pos (16 bit),
// vel (12), tqe (12), kp (12), kd (12). Ranges come from
// `motor_set_pos_vel_tqe_kp_kd` (`motor_control.c`): pos/vel in "turns"
// (via conv_to_turns), tqe in Nm (model adaptation see P2-15, not
// corrected here).

pub const MIT_ID_PREFIX: u32 = 0x18000;

pub const MIT_POS_MIN: f32 = -3.2768;
pub const MIT_POS_MAX: f32 = 3.2767;
pub const MIT_VEL_MIN: f32 = -2.0;
pub const MIT_VEL_MAX: f32 = 2.0;
pub const MIT_TQE_MIN: f32 = -10.0;
pub const MIT_TQE_MAX: f32 = 10.0;
pub const MIT_KP_MIN: f32 = -400.0;
pub const MIT_KP_MAX: f32 = 400.0;
pub const MIT_KD_MIN: f32 = -100.0;
pub const MIT_KD_MAX: f32 = 100.0;

/// Mirrors the reference firmware `mit_float2int`: `raw = (x - x_min) *
/// ((1<<bits) / span)`, saturated to `[0, (1<<bits)-1]` (otherwise a
/// 12-bit field at full scale yields 1<<12=4096, and the overflowing high
/// bit misaligns the bit packing).
pub(crate) fn mit_float2int(x: f32, x_min: f32, x_max: f32, bits: u8) -> u16 {
    let span = x_max - x_min;
    let scale = ((1u32 << bits) as f32) / span;
    let raw = (x - x_min) * scale;
    let max = ((1u32 << bits) - 1) as f32;
    raw.clamp(0.0, max) as u16
}

/// Encodes the MIT frame (reused by upper layers such as CLI/ws_gateway,
/// avoiding a 4th copy of the protocol logic).
/// Inputs: pos (rad), vel (rad/s), tqe (Nm), dimensionless kp/kd.
/// Saturates within range.
pub fn encode_mit_frame(pos_rad: f32, vel_radps: f32, tqe_nm: f32, kp: f32, kd: f32) -> [u8; 8] {
    let pos_turns = pos_rad / TWO_PI;
    let vel_turns = vel_radps / TWO_PI;
    let pos_u = mit_float2int(pos_turns, MIT_POS_MIN, MIT_POS_MAX, 16);
    let vel_u = mit_float2int(vel_turns, MIT_VEL_MIN, MIT_VEL_MAX, 12);
    let tqe_u = mit_float2int(tqe_nm, MIT_TQE_MIN, MIT_TQE_MAX, 12);
    let kp_u = mit_float2int(kp, MIT_KP_MIN, MIT_KP_MAX, 12);
    let kd_u = mit_float2int(kd, MIT_KD_MIN, MIT_KD_MAX, 12);
    let b0 = (pos_u & 0xff) as u8;
    let b1 = ((pos_u >> 8) & 0xff) as u8;
    let b2 = (vel_u & 0xff) as u8;
    let vel_hi = ((vel_u >> 8) & 0x0f) as u8;
    let tqe_lo = (tqe_u & 0x0f) as u8;
    let b3 = vel_hi | (tqe_lo << 4);
    let b4 = ((tqe_u >> 4) & 0xff) as u8;
    let b5 = (kp_u & 0xff) as u8;
    let kp_hi = ((kp_u >> 8) & 0x0f) as u8;
    let kd_lo = (kd_u & 0x0f) as u8;
    let b6 = kp_hi | (kd_lo << 4);
    let b7 = ((kd_u >> 4) & 0xff) as u8;
    [b0, b1, b2, b3, b4, b5, b6, b7]
}

// ----- Generic register read (protocol §1.3) -----
//
// Read command byte (cmd) = `[7:4]=0001 read / 0010 reply`, `[3:2]` type
// (00=int8, 01=int16, 10=int32, 11=float), `[1:0]` count (01=1, 10=2,
// 11=3). Multiple `(cmd, addr)` pairs may be concatenated up to 8 bytes
// total. The reply mirrors it (cmd high half-byte=0010); each pair is
// `(reply_cmd, start_addr, value0, value1, ...)` with addresses ascending.

/// Register data type (the two `[3:2]` cmd bits of the protocol). Hash/Eq
/// support the motor.rs register cache keyed by `(addr, ty)` (A3).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum RegisterType {
    Int8 = 0b00,
    Int16 = 0b01,
    Int32 = 0b10,
    Float = 0b11,
}

impl RegisterType {
    /// Decodes the type back from the two `[3:2]` cmd bits.
    pub(crate) fn from_cmd_bits(bits: u8) -> Option<Self> {
        match bits & 0b11 {
            0 => Some(Self::Int8),
            1 => Some(Self::Int16),
            2 => Some(Self::Int32),
            3 => Some(Self::Float),
            _ => None,
        }
    }

    pub(crate) fn cmd_bits(self) -> u8 {
        self as u8
    }

    pub(crate) fn size(self) -> usize {
        match self {
            Self::Int8 => 1,
            Self::Int16 => 2,
            Self::Int32 | Self::Float => 4,
        }
    }

    /// Bridges to the `DataType` used by control-frame conversion (the
    /// register read conversion reuses `QuantityScale::scale`, which picks
    /// the column by `DataType`).
    pub(crate) fn data_type(self) -> DataType {
        match self {
            Self::Int8 => DataType::Int8,
            Self::Int16 => DataType::Int16,
            Self::Int32 => DataType::Int32,
            Self::Float => DataType::Float,
        }
    }
}

/// Raw register value as read.
#[derive(Debug, Clone, Copy)]
pub enum RegisterValue {
    I8(i8),
    I16(i16),
    I32(i32),
    F32(f32),
}

impl RegisterValue {
    /// Value as f32 (for downstream scale conversion of position/velocity/torque etc.).
    pub fn as_f32(self) -> f32 {
        match self {
            Self::I8(v) => v as f32,
            Self::I16(v) => v as f32,
            Self::I32(v) => v as f32,
            Self::F32(v) => v,
        }
    }

    /// The value's data type (the `ty` source of the motor.rs register
    /// cache's `(addr, ty)` key; the reply frame's cmd half-byte encodes
    /// the same type as the decoded variant).
    pub fn register_type(self) -> RegisterType {
        match self {
            Self::I8(_) => RegisterType::Int8,
            Self::I16(_) => RegisterType::Int16,
            Self::I32(_) => RegisterType::Int32,
            Self::F32(_) => RegisterType::Float,
        }
    }
}

/// One read request: read `count` consecutive values starting at `addr`
/// (count ∈ 1..=3). 2026-10-22 decision: reads are **fixed Int16**
/// (`cmd [3:2]` is always 01); callers no longer specify the data type.
#[derive(Debug, Clone, Copy)]
pub struct ReadCmd {
    pub addr: u8,
    pub count: u8,
}

impl ReadCmd {
    fn cmd_byte(self) -> Option<u8> {
        let count = self.count;
        if !(1..=3).contains(&count) {
            return None;
        }
        // count 1/2/3 is exactly the 2-bit encoding 01/10/11; type bits are always Int16 (01)
        Some(0b0001_0000 | (RegisterType::Int16.cmd_bits() << 2) | (count & 0b11))
    }
}

const REPLY_NIBBLE: u8 = 0b0010_0000;

/// Canonical frame data of a single read request: exactly 2 bytes `(cmd, addr)`.
///
/// 2026-10-24 real-bus tests (can0, candump): the firmware parses only the
/// **first** `(cmd, addr)` pair of a frame and the trailing bytes must be
/// all zero — packing multiple read requests into one frame (non-zero
/// trailing bytes) makes the motor **drop the whole frame silently** (zero
/// replies, not even a DLC=0 empty frame; dlc 2-8 zero-trailing frames are
/// all accepted). So read requests are always one pair per frame;
/// consecutive addresses are covered by the cmd count bits (1..=3), and
/// non-consecutive addresses go in separate frames.
pub(crate) fn encode_read(cmd: ReadCmd) -> Option<[u8; 2]> {
    Some([cmd.cmd_byte()?, cmd.addr])
}

/// Parses a register read reply. Returns a list of `(addr, value)` with
/// addresses ascending consecutively per count.
///
/// Returns `None` for non-reply frames (first byte high half-byte != 0010)
/// or insufficient length.
pub(crate) fn decode_register_reply(data: &[u8]) -> Option<Vec<(u8, RegisterValue)>> {
    if data.is_empty() {
        return None;
    }
    let mut out = Vec::new();
    let mut pos = 0usize;
    while pos < data.len() {
        let cmd = data[pos];
        if cmd & 0xF0 != REPLY_NIBBLE {
            return None;
        }
        if pos + 1 >= data.len() {
            return None;
        }
        let addr = data[pos + 1];
        let ty = RegisterType::from_cmd_bits((cmd >> 2) & 0b11)?;
        let count = (cmd & 0b11) as usize;
        if count == 0 {
            return None;
        }
        pos += 2;
        let size = ty.size();
        for i in 0..count {
            let start = pos + i * size;
            if start + size > data.len() {
                return None;
            }
            let buf = &data[start..start + size];
            let value = match ty {
                RegisterType::Int8 => RegisterValue::I8(i8::from_le_bytes([buf[0]])),
                RegisterType::Int16 => RegisterValue::I16(i16::from_le_bytes([buf[0], buf[1]])),
                RegisterType::Int32 => {
                    RegisterValue::I32(i32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]))
                }
                RegisterType::Float => {
                    RegisterValue::F32(f32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]))
                }
            };
            out.push(((addr.wrapping_add(i as u8)), value));
        }
        pos += count * size;
    }
    Some(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn reply_frame(arbitration_id: u32, is_extended: bool) -> CanFrame {
        CanFrame {
            arbitration_id,
            data: [0x27, 0x01, 0x34, 0x12, 0x78, 0x56, 0x00, 0x0A],
            dlc: 8,
            is_extended,
            is_rx: true,
        }
    }

    fn frame_with(arbitration_id: u32, data: [u8; 8], dlc: u8, is_extended: bool) -> CanFrame {
        CanFrame {
            arbitration_id,
            data,
            dlc,
            is_extended,
            is_rx: true,
        }
    }

    /// Generic test coefficient (k=0.5, d=0).
    const GEN_COEFF: TorqueCoeff = TorqueCoeff { k: 0.5, d: 0.0 };

    // The bug: extended-frame replies put the motor id in the HIGH byte
    // (e.g. id=1 → 0x0100), but the old code read the LOW byte (`& 0x7F` → 0),
    // so only motor_id 0 ever matched.
    #[test]
    fn reply_motor_id_extended_frame_uses_high_byte() {
        assert_eq!(reply_motor_id(&reply_frame(0x0100, true)), Some(1));
    }

    #[test]
    fn reply_motor_id_standard_frame_uses_high_byte() {
        assert_eq!(reply_motor_id(&reply_frame(0x0100, false)), Some(1));
    }

    #[test]
    fn reply_motor_id_larger_id_extended() {
        // id=16 → reply ID 0x1000 (extended, since >0x7FF)
        assert_eq!(reply_motor_id(&reply_frame(0x1000, true)), Some(16));
    }

    #[test]
    fn reply_motor_id_rejects_nonzero_dest() {
        // dest != 0 → addressed to another host, not ours
        assert_eq!(reply_motor_id(&reply_frame(0x0101, true)), None);
    }

    #[test]
    fn decode_read_reply_extended_frame_decodes_correct_id() {
        let frame = reply_frame(0x0100, true);
        let state = decode_read_reply(frame, GEN_COEFF).expect("extended reply should decode");
        assert_eq!(state.can_id, 1);
    }

    // ----- Generic register read -----

    #[test]
    fn encode_read_single_int16_count3_matches_0x17_0x01() {
        assert_eq!(
            encode_read(ReadCmd {
                addr: 0x01,
                count: 3
            }),
            Some([0x17, 0x01])
        );
    }

    /// 2026-10-22 decision: type bits always Int16 → count1 cmd=0x15,
    /// count2=0x16. 2026-10-24 real-bus tests: one pair per frame (the
    /// firmware drops multi-pair frames with non-zero trailing bytes).
    #[test]
    fn encode_read_fixed_int16_cmd_bytes() {
        assert_eq!(
            encode_read(ReadCmd {
                addr: 0x0E,
                count: 1
            }),
            Some([0x15, 0x0E])
        );
        assert_eq!(
            encode_read(ReadCmd {
                addr: 0xFF,
                count: 2
            }),
            Some([0x16, 0xFF])
        );
    }

    #[test]
    fn encode_read_rejects_invalid_count() {
        assert_eq!(
            encode_read(ReadCmd {
                addr: 0x01,
                count: 0
            }),
            None
        );
    }

    #[test]
    fn decode_register_reply_pos_vel_tqe() {
        // 0x27 0x01, pos=0x1234, vel=0x0009, tqe=0x0000 (spec example)
        let data = [0x27, 0x01, 0x34, 0x12, 0x09, 0x00, 0x00, 0x00];
        let vals = decode_register_reply(&data).expect("reply decodes");
        assert_eq!(vals.len(), 3);
        assert_eq!(vals[0].0, 0x01);
        assert!(matches!(vals[0].1, RegisterValue::I16(0x1234)));
        assert_eq!(vals[1].0, 0x02);
        assert!(matches!(vals[1].1, RegisterValue::I16(0x0009)));
        assert_eq!(vals[2].0, 0x03);
        assert!(matches!(vals[2].1, RegisterValue::I16(0x0000)));
    }

    #[test]
    fn decode_register_reply_float_single() {
        // cmd = 0001(read)|11(float)|01(count1) = 0x1D; reply = 0x2D
        let val = 1.5f32.to_le_bytes();
        let mut data = vec![0x2D, 0x0F];
        data.extend_from_slice(&val);
        let vals = decode_register_reply(&data).expect("reply decodes");
        assert_eq!(vals.len(), 1);
        assert_eq!(vals[0].0, 0x0F);
        assert!(matches!(vals[0].1, RegisterValue::F32(v) if (v - 1.5).abs() < 1e-6));
    }

    #[test]
    fn decode_register_reply_rejects_status_frame() {
        // Status feedback frame first byte = mode (high half-byte 0/1), not a reply (0010)
        let data = [0x0A, 0x00, 0x34, 0x12, 0x09, 0x00, 0x00, 0x00];
        assert!(decode_register_reply(&data).is_none());
    }

    #[test]
    fn decode_register_reply_rejects_non_reply_first_byte() {
        let data = [0x17, 0x01, 0, 0, 0, 0, 0, 0]; // 0x17 high half-byte=0001, not a reply
        assert!(decode_register_reply(&data).is_none());
    }

    #[test]
    fn decode_register_reply_multi_pair() {
        // Two pairs: int16 count1 @0x01, int16 count1 @0x0F
        // pair1: 0x25 0x01 (reply int16 count1), data 0x34 0x12
        // pair2: 0x25 0x0F, data 0x09 0x00
        let data = [0x25, 0x01, 0x34, 0x12, 0x25, 0x0F, 0x09, 0x00];
        let vals = decode_register_reply(&data).expect("reply decodes");
        assert_eq!(vals.len(), 2);
        assert_eq!(vals[0].0, 0x01);
        assert_eq!(vals[1].0, 0x0F);
    }

    #[test]
    fn decode_register_reply_rejects_empty_data() {
        // The motor may reply with a DLC=0 empty frame (e.g. when bit15 is set); empty data is not a reply
        assert!(decode_register_reply(&[]).is_none());
    }

    // ----- Status feedback frame (P1-4) -----

    #[test]
    fn decode_status_frame_extracts_mode_and_fault() {
        // mode=0x0A (position mode), fault=0, pos=0x1234, vel=0x0009, tqe=0x0000
        let frame = frame_with(
            0x0100,
            [0x0A, 0x00, 0x34, 0x12, 0x09, 0x00, 0x00, 0x00],
            8,
            true,
        );
        let state = decode_status_frame(&frame, GEN_COEFF).expect("status frame decodes");
        assert_eq!(state.can_id, 1);
        assert_eq!(state.status_code, 0x0A);
        assert_eq!(state.fault_code, 0x00);
        assert_eq!(state.pos, 0x1234i16 as f32 * 0.0001 * TWO_PI);
        assert_eq!(state.vel, 9.0 * 0.00025 * TWO_PI);
        assert_eq!(state.torq, 0.0);
    }

    #[test]
    fn decode_status_frame_carries_fault_code() {
        // mode=0x02 (ready), fault=0x22 (34=overvoltage)
        let frame = frame_with(0x0100, [0x02, 0x22, 0, 0, 0, 0, 0, 0], 8, true);
        let state = decode_status_frame(&frame, GEN_COEFF).expect("status frame decodes");
        assert_eq!(state.status_code, 0x02);
        assert_eq!(state.fault_code, 0x22);
    }

    #[test]
    fn decode_status_frame_rejects_register_reply_and_short() {
        // A 0x27 register reply must not be treated as a status frame
        let reg = frame_with(0x0100, [0x27, 0x01, 0, 0, 0, 0, 0, 0], 8, true);
        assert!(decode_status_frame(&reg, GEN_COEFF).is_none());
        // short frame
        let short = frame_with(0x0100, [0x0A, 0x00, 0, 0, 0, 0, 0, 0], 5, true);
        assert!(decode_status_frame(&short, GEN_COEFF).is_none());
    }

    #[test]
    fn decode_feedback_dispatches_both_paths() {
        // Register read reply goes through the read_reply branch (mode=0, fault=0)
        let reg = frame_with(
            0x0100,
            [0x27, 0x01, 0x34, 0x12, 0x09, 0x00, 0x00, 0x00],
            8,
            true,
        );
        let s = decode_feedback(reg, GEN_COEFF).expect("feedback decodes");
        assert_eq!(s.status_code, 0);
        assert_eq!(s.fault_code, 0);

        // Status feedback frame goes through the status branch
        let st = frame_with(
            0x0100,
            [0x0A, 0x00, 0x34, 0x12, 0x09, 0x00, 0x00, 0x00],
            8,
            true,
        );
        let s = decode_feedback(st, GEN_COEFF).expect("feedback decodes");
        assert_eq!(s.status_code, 0x0A);
    }

    #[test]
    fn decode_feedback_rejects_ack_and_version() {
        // Setting ACKs (7 bytes) and version replies (5 bytes) are both not feedback
        let ack = frame_with(
            0x0100,
            [0x41, 0x01, 0x04, 0x4F, 0x4B, 0x0D, 0x0A, 0],
            7,
            true,
        );
        assert!(decode_feedback(ack, GEN_COEFF).is_none());
        let ver = frame_with(0x0100, [0x00, 0xB5, 0x02, 0x12, 0x34, 0, 0, 0], 5, true);
        assert!(decode_feedback(ver, GEN_COEFF).is_none());
    }

    // ----- MIT mode bit packing (P1-2) -----
    //
    // Verifies the bit layout of `encode_mit_frame` (pos@byte0-1,
    // vel@byte2+byte3 high nibble, tqe@byte3 low nibble+byte4,
    // kp@byte5+byte6 high nibble, kd@byte6 low nibble+byte7): set each
    // field to its maximum and the rest to their minimum — only that
    // field's bits should flip.

    #[test]
    fn mit_float2int_midpoint_is_half_scale() {
        // Symmetric range: the midpoint 0 should map to half scale.
        assert_eq!(mit_float2int(0.0, -10.0, 10.0, 12), 2048);
        assert_eq!(mit_float2int(0.0, -2.0, 2.0, 12), 2048);
        assert_eq!(mit_float2int(0.0, -400.0, 400.0, 12), 2048);
    }

    #[test]
    fn encode_mit_frame_all_min_is_all_zero() {
        let d = encode_mit_frame(
            MIT_POS_MIN * TWO_PI,
            MIT_VEL_MIN * TWO_PI,
            MIT_TQE_MIN,
            MIT_KP_MIN,
            MIT_KD_MIN,
        );
        assert_eq!(d, [0, 0, 0, 0, 0, 0, 0, 0]);
    }

    #[test]
    fn encode_mit_frame_all_max_is_all_ff() {
        let d = encode_mit_frame(
            MIT_POS_MAX * TWO_PI,
            MIT_VEL_MAX * TWO_PI,
            MIT_TQE_MAX,
            MIT_KP_MAX,
            MIT_KD_MAX,
        );
        assert_eq!(d, [0xFF; 8]);
    }

    #[test]
    fn encode_mit_frame_pos_occupies_bytes_0_1() {
        let d = encode_mit_frame(
            MIT_POS_MAX * TWO_PI,
            MIT_VEL_MIN * TWO_PI,
            MIT_TQE_MIN,
            MIT_KP_MIN,
            MIT_KD_MIN,
        );
        assert_eq!(d[0], 0xFF);
        assert_eq!(d[1], 0xFF);
        assert_eq!(d[2], 0x00);
        assert_eq!(d[3], 0x00);
        assert_eq!(d[4], 0x00);
        assert_eq!(d[5], 0x00);
        assert_eq!(d[6], 0x00);
        assert_eq!(d[7], 0x00);
    }

    #[test]
    fn encode_mit_frame_vel_occupies_byte2_and_high_nibble_of_byte3() {
        let d = encode_mit_frame(
            MIT_POS_MIN * TWO_PI,
            MIT_VEL_MAX * TWO_PI,
            MIT_TQE_MIN,
            MIT_KP_MIN,
            MIT_KD_MIN,
        );
        // byte2 = vel low 8 bits (0xFF), byte3 high nibble = vel high 4 bits (0x0F),
        // byte3 low nibble = tqe low 4 bits (0) → byte3 = 0x0F.
        assert_eq!(d[0], 0x00);
        assert_eq!(d[1], 0x00);
        assert_eq!(d[2], 0xFF);
        assert_eq!(d[3], 0x0F);
        assert_eq!(d[4], 0x00);
    }

    #[test]
    fn encode_mit_frame_tqe_occupies_low_nibble_of_byte3_and_byte4() {
        let d = encode_mit_frame(
            MIT_POS_MIN * TWO_PI,
            MIT_VEL_MIN * TWO_PI,
            MIT_TQE_MAX,
            MIT_KP_MIN,
            MIT_KD_MIN,
        );
        // byte3 low nibble = tqe low 4 bits (0x0F) << 4 → 0xF0, byte4 = tqe high 8 bits (0xFF).
        assert_eq!(d[2], 0x00);
        assert_eq!(d[3], 0xF0);
        assert_eq!(d[4], 0xFF);
        assert_eq!(d[5], 0x00);
    }

    #[test]
    fn encode_mit_frame_kp_occupies_byte5_and_high_nibble_of_byte6() {
        let d = encode_mit_frame(
            MIT_POS_MIN * TWO_PI,
            MIT_VEL_MIN * TWO_PI,
            MIT_TQE_MIN,
            MIT_KP_MAX,
            MIT_KD_MIN,
        );
        assert_eq!(d[5], 0xFF);
        assert_eq!(d[6], 0x0F);
        assert_eq!(d[7], 0x00);
    }

    #[test]
    fn encode_mit_frame_kd_occupies_low_nibble_of_byte6_and_byte7() {
        let d = encode_mit_frame(
            MIT_POS_MIN * TWO_PI,
            MIT_VEL_MIN * TWO_PI,
            MIT_TQE_MIN,
            MIT_KP_MIN,
            MIT_KD_MAX,
        );
        assert_eq!(d[5], 0x00);
        assert_eq!(d[6], 0xF0);
        assert_eq!(d[7], 0xFF);
    }

    // ----- Trapezoid mode acceleration scale (P1-3) -----

    #[test]
    fn radps2_to_acc_raw_one_rev_per_s2_is_1000() {
        // 1 turn/s² = 2π rad/s² → acc_turns=1 → raw=1000 (scale=1000).
        let raw = radps2_to_acc_raw(TWO_PI);
        assert_eq!(raw, 1000);
    }

    #[test]
    fn radps2_to_acc_raw_saturates() {
        // Far beyond the i16 range → saturates to the firmware data_limit
        // ±32760 (not i16::MAX — 0x8000 is reserved as the NAN_INT16
        // "no limit" sentinel, per convert.c).
        let raw = radps2_to_acc_raw(1.0e9);
        assert_eq!(raw, 32760);
        let raw = radps2_to_acc_raw(-1.0e9);
        assert_eq!(raw, -32760);
    }

    // ----- P2-16 units and data types -----

    #[test]
    fn quantity_scales_match_convert_c() {
        // pos_float2int(in, type) = data_float2int(in, type, 100, 10000, 100000)
        assert_eq!(
            POS_SCALE,
            QuantityScale {
                int8: 100.0,
                int16: 10000.0,
                int32: 100000.0
            }
        );
        // vel: (100, 4000, 100000)
        assert_eq!(
            VEL_SCALE,
            QuantityScale {
                int8: 100.0,
                int16: 4000.0,
                int32: 100000.0
            }
        );
        // tqe: (2, 100, 1000)
        assert_eq!(
            TQE_SCALE,
            QuantityScale {
                int8: 2.0,
                int16: 100.0,
                int32: 1000.0
            }
        );
        // acc: (20, 1000, 100000)
        assert_eq!(
            ACC_SCALE,
            QuantityScale {
                int8: 20.0,
                int16: 1000.0,
                int32: 100000.0
            }
        );
        // cur: (1, 10, 1000)
        assert_eq!(
            CUR_SCALE,
            QuantityScale {
                int8: 1.0,
                int16: 10.0,
                int32: 1000.0
            }
        );
        // vol: (2, 10, 1000)
        assert_eq!(
            VOL_SCALE,
            QuantityScale {
                int8: 2.0,
                int16: 10.0,
                int32: 1000.0
            }
        );
        // pid: (1, 10, 1000)
        assert_eq!(
            PID_SCALE,
            QuantityScale {
                int8: 1.0,
                int16: 10.0,
                int32: 1000.0
            }
        );
    }

    #[test]
    fn to_turns_from_turns_are_inverse() {
        for unit in [AngleUnit::Rad, AngleUnit::Degree, AngleUnit::Turn] {
            for v in [0.0f32, 1.0, 0.5, -2.0, 3.5] {
                let turns = to_turns(v, unit);
                let back = from_turns(turns, unit);
                assert!((back - v).abs() < 1e-5, "unit {unit:?} v={v} back={back}");
            }
        }
        // One turn in each unit: Rad=2π, Degree=360, Turn=1
        assert!((to_turns(TWO_PI, AngleUnit::Rad) - 1.0).abs() < 1e-5);
        assert!((to_turns(360.0, AngleUnit::Degree) - 1.0).abs() < 1e-5);
        assert_eq!(to_turns(1.0, AngleUnit::Turn), 1.0);
    }

    /// encode→decode round trip for every (AngleUnit, DataType) combination, using pos as the example.
    #[test]
    fn encode_decode_unit_round_trip_all_combos() {
        for unit in [AngleUnit::Rad, AngleUnit::Degree, AngleUnit::Turn] {
            for ty in [
                DataType::Int8,
                DataType::Int16,
                DataType::Int32,
                DataType::Float,
            ] {
                // Pick a value within every type's range: 0.3 turns
                let v_in_unit = match unit {
                    AngleUnit::Rad => 0.3 * TWO_PI,
                    AngleUnit::Degree => 0.3 * 360.0,
                    AngleUnit::Turn => 0.3,
                };
                let raw = encode_unit_raw(v_in_unit, unit, ty, POS_SCALE);
                let back = decode_unit_raw(raw, unit, ty, POS_SCALE);
                // Quantization error: Float lossless, integers ≤ 1/scale
                let tol = match ty {
                    DataType::Float => 1e-5,
                    DataType::Int8 => 1.0 / POS_SCALE.int8,
                    DataType::Int16 => 1.0 / POS_SCALE.int16,
                    DataType::Int32 => 1.0 / POS_SCALE.int32,
                };
                assert!(
                    (back - v_in_unit).abs() < tol,
                    "unit {unit:?} ty {ty:?} raw={raw} back={back} tol {tol}"
                );
            }
        }
    }

    #[test]
    fn encode_unit_raw_float_is_identity() {
        // Float type: no scale, no saturation, returned as-is
        let raw = encode_unit_raw(1.5, AngleUnit::Rad, DataType::Float, POS_SCALE);
        assert_eq!(raw, 1.5);
        let back = decode_unit_raw(1.5, AngleUnit::Rad, DataType::Float, POS_SCALE);
        assert_eq!(back, 1.5);
    }

    #[test]
    fn encode_unit_raw_int16_matches_legacy_rad_to_pos_raw() {
        // The P2-16 generalized path should match the legacy Rad+Int16 fixed-constant path.
        for rad in [0.0f32, 0.5 * TWO_PI, 1.0 * TWO_PI, -0.25 * TWO_PI] {
            let via_unit =
                encode_unit_raw(rad, AngleUnit::Rad, DataType::Int16, POS_SCALE).round() as i32;
            let via_legacy = rad_to_pos_raw(rad) as i32;
            assert_eq!(via_unit, via_legacy, "rad={rad}");
        }
    }

    #[test]
    fn encode_unit_raw_saturation_limits() {
        // int8: ±127
        assert_eq!(
            encode_unit_raw(1.0e9, AngleUnit::Turn, DataType::Int8, POS_SCALE),
            127.0
        );
        assert_eq!(
            encode_unit_raw(-1.0e9, AngleUnit::Turn, DataType::Int8, POS_SCALE),
            -127.0
        );
        // int16: ±32760
        assert_eq!(
            encode_unit_raw(1.0e9, AngleUnit::Turn, DataType::Int16, POS_SCALE),
            32760.0
        );
        assert_eq!(
            encode_unit_raw(-1.0e9, AngleUnit::Turn, DataType::Int16, POS_SCALE),
            -32760.0
        );
        // int32: ±2147483640
        assert_eq!(
            encode_unit_raw(1.0e15, AngleUnit::Turn, DataType::Int32, POS_SCALE),
            2_147_483_640.0
        );
        assert_eq!(
            encode_unit_raw(-1.0e15, AngleUnit::Turn, DataType::Int32, POS_SCALE),
            -2_147_483_640.0
        );
        // float: no saturation
        assert_eq!(
            encode_unit_raw(1.0e15, AngleUnit::Turn, DataType::Float, POS_SCALE),
            1.0e15
        );
    }

    /// Unit conversion correctness: 1 turn encoded via Degree+Int16 should give raw = 1 * 360/360 * 10000 = 10000.
    #[test]
    fn encode_unit_raw_degree_one_rev_is_10000_int16() {
        let raw = encode_unit_raw(360.0, AngleUnit::Degree, DataType::Int16, POS_SCALE);
        assert_eq!(raw, 10000.0);
    }

    /// Turns + Int16: 1.0 turn → 1 * 10000 = 10000 (matches the firmware default TURNS).
    #[test]
    fn encode_unit_raw_turn_one_rev_is_10000_int16() {
        let raw = encode_unit_raw(1.0, AngleUnit::Turn, DataType::Int16, POS_SCALE);
        assert_eq!(raw, 10000.0);
        // decode round trip
        assert!(
            (decode_unit_raw(10000.0, AngleUnit::Turn, DataType::Int16, POS_SCALE) - 1.0).abs()
                < 1e-6
        );
    }

    // ----- Setting ACK (P1-6) -----

    #[test]
    fn is_setting_ack_matches_exact_frame() {
        let f = frame_with(
            0x0100,
            [0x41, 0x01, 0x04, 0x4F, 0x4B, 0x0D, 0x0A, 0],
            7,
            true,
        );
        assert!(is_setting_ack(&f));
    }

    #[test]
    fn is_setting_ack_rejects_wrong_dlc_or_bytes() {
        // dlc != 7
        let long = frame_with(
            0x0100,
            [0x41, 0x01, 0x04, 0x4F, 0x4B, 0x0D, 0x0A, 0],
            8,
            true,
        );
        assert!(!is_setting_ack(&long));
        // scrambled bytes (0x4E instead of 0x4F)
        let bad = frame_with(
            0x0100,
            [0x41, 0x01, 0x04, 0x4E, 0x4B, 0x0D, 0x0A, 0],
            7,
            true,
        );
        assert!(!is_setting_ack(&bad));
        // status frame
        let status = frame_with(0x0100, [0x0A, 0x00, 0, 0, 0, 0, 0, 0], 8, true);
        assert!(!is_setting_ack(&status));
    }

    #[test]
    fn run_mode_from_code_covers_0_through_17() {
        for code in 0u8..=17 {
            assert!(
                RunMode::from_code(code).is_some(),
                "code {code} should map to a RunMode"
            );
        }
        // 18 and above are outside the table 2 range
        assert_eq!(RunMode::from_code(18), None);
        assert_eq!(RunMode::from_code(255), None);
    }

    #[test]
    fn run_mode_from_code_matches_repr() {
        assert_eq!(RunMode::from_code(0), Some(RunMode::Stop));
        assert_eq!(RunMode::from_code(1), Some(RunMode::Error));
        assert_eq!(RunMode::from_code(5), Some(RunMode::Pwm));
        assert_eq!(RunMode::from_code(10), Some(RunMode::Position));
        assert_eq!(RunMode::from_code(15), Some(RunMode::Brake));
        assert_eq!(RunMode::from_code(17), Some(RunMode::Reserved17));
    }

    #[test]
    fn decode_fault_known_codes() {
        assert_eq!(decode_fault(0), "正常");
        // 1-7 grouped together as UART/DMA errors
        for c in 1u8..=7 {
            assert_eq!(decode_fault(c), "UART/DMA 错误", "code {c}");
        }
        assert_eq!(decode_fault(32), "校准错误");
        assert_eq!(decode_fault(33), "驱动故障");
        assert_eq!(decode_fault(34), "过压");
        assert_eq!(decode_fault(35), "编码器错误");
        assert_eq!(decode_fault(36), "未校准");
        assert_eq!(decode_fault(37), "PWM 周期错误");
        assert_eq!(decode_fault(38), "过温");
        assert_eq!(decode_fault(39), "起始位置超范围");
        assert_eq!(decode_fault(40), "欠压");
        assert_eq!(decode_fault(41), "配置已变更");
        assert_eq!(decode_fault(42), "无效角度");
        assert_eq!(decode_fault(43), "无效位置");
        assert_eq!(decode_fault(44), "驱动使能故障");
        assert_eq!(decode_fault(45), "停止位置误用");
        assert_eq!(decode_fault(46), "时序错误");
        assert_eq!(decode_fault(47), "反电动势前馈错误");
    }

    #[test]
    fn decode_fault_unknown_codes() {
        // 8-31 and >47 are undefined by the spec
        assert_eq!(decode_fault(8), "未知故障码");
        assert_eq!(decode_fault(31), "未知故障码");
        assert_eq!(decode_fault(48), "未知故障码");
        assert_eq!(decode_fault(255), "未知故障码");
    }

    #[test]
    fn feedback_state_run_mode_and_fault_description() {
        let mut s = HightorqueFeedbackState {
            can_id: 1,
            arbitration_id: 0x0100,
            status_code: 10,
            fault_code: 34,
            pos: 0.0,
            vel: 0.0,
            torq: 0.0,
            t_mos: 0.0,
            t_rotor: 0.0,
        };
        assert_eq!(s.run_mode(), Some(RunMode::Position));
        assert_eq!(s.fault_description(), "过压");
        // unknown codes
        s.status_code = 99;
        s.fault_code = 200;
        assert_eq!(s.run_mode(), None);
        assert_eq!(s.fault_description(), "未知故障码");
    }

    // ----- P2-15 torque model adaptation -----

    /// Per-model coefficients against the reference firmware `convert.c::motor_tqe_adj[]`. All d=0.
    #[test]
    fn motor_model_coeff_matches_firmware_table() {
        assert_eq!(MotorModel::Null.coeff(), TorqueCoeff { k: 0.0, d: 0.0 });
        assert_eq!(
            MotorModel::M3536_32.coeff(),
            TorqueCoeff {
                k: 0.458_105,
                d: 0.0
            }
        );
        assert_eq!(
            MotorModel::M4438_30.coeff(),
            TorqueCoeff {
                k: 0.525_600,
                d: 0.0
            }
        );
        assert_eq!(
            MotorModel::M4438_32.coeff(),
            TorqueCoeff {
                k: 0.485_565,
                d: 0.0
            }
        );
        assert_eq!(
            MotorModel::M4538_19.coeff(),
            TorqueCoeff {
                k: 0.493_835,
                d: 0.0
            }
        );
        assert_eq!(
            MotorModel::M5043_20.coeff(),
            TorqueCoeff { k: 0.966, d: 0.0 }
        );
        assert_eq!(
            MotorModel::M5046_20.coeff(),
            TorqueCoeff {
                k: 0.533_654,
                d: 0.0
            }
        );
        assert_eq!(
            MotorModel::M5047_09.coeff(),
            TorqueCoeff {
                k: 0.547_474,
                d: 0.0
            }
        );
        assert_eq!(
            MotorModel::M5047_36.coeff(),
            // 2026-10-20 decision: Feishu 0.64 wins (both firmware versions say 0.35, dropped).
            TorqueCoeff { k: 0.64, d: 0.0 }
        );
        // HTDW-5036-02-CNE: k=0.64 (Feishu unit table M5047_36 row, 2026-10-20 decision;
        // Kt=0.54 is the module torque constant, a different concept from k, not used as compensation).
        assert_eq!(
            MotorModel::M5036_36.coeff(),
            TorqueCoeff { k: 0.64, d: 0.0 }
        );
        assert_eq!(
            MotorModel::M6056_36.coeff(),
            TorqueCoeff { k: 0.677, d: 0.0 }
        );
        assert_eq!(
            MotorModel::M7256_35.coeff(),
            TorqueCoeff {
                k: 0.676_524,
                d: 0.0
            }
        );
        assert_eq!(
            MotorModel::M60SG_35.coeff(),
            TorqueCoeff { k: 0.794_2, d: 0.0 }
        );
        assert_eq!(
            MotorModel::M60BM_35.coeff(),
            TorqueCoeff { k: 0.794_2, d: 0.0 }
        );
        assert_eq!(MotorModel::General.coeff(), TorqueCoeff { k: 0.5, d: 0.0 });
    }

    #[test]
    fn from_hint_accepts_placeholders_as_general() {
        for s in ["", "hightorque", "ht", "auto", "default", "general"] {
            assert_eq!(
                MotorModel::from_hint(s),
                Some(MotorModel::General),
                "placeholder {s:?} → General"
            );
        }
    }

    #[test]
    fn from_hint_accepts_specific_model_codes() {
        assert_eq!(MotorModel::from_hint("5046-20"), Some(MotorModel::M5046_20));
        assert_eq!(MotorModel::from_hint("5046_20"), Some(MotorModel::M5046_20));
        assert_eq!(
            MotorModel::from_hint("M5046_20"),
            Some(MotorModel::M5046_20)
        );
        assert_eq!(
            MotorModel::from_hint("ht-5046-20"),
            Some(MotorModel::M5046_20)
        );
        assert_eq!(
            MotorModel::from_hint("hightorque-5046-20"),
            Some(MotorModel::M5046_20)
        );
        assert_eq!(
            MotorModel::from_hint("m5046_20"),
            Some(MotorModel::M5046_20)
        );
        // HTDW-5036-02: both the bare model code and the gear-ratio variant parse as M5036_36
        assert_eq!(MotorModel::from_hint("5036"), Some(MotorModel::M5036_36));
        assert_eq!(MotorModel::from_hint("5036_36"), Some(MotorModel::M5036_36));
        assert_eq!(
            MotorModel::from_hint("M5036_36"),
            Some(MotorModel::M5036_36)
        );
        assert_eq!(MotorModel::from_hint("ht-5036"), Some(MotorModel::M5036_36));
        assert_eq!(
            MotorModel::from_hint("ht-5036-36"),
            Some(MotorModel::M5036_36)
        );
        // The commercial code -02 (HTDW-5036-02) also maps to the same variant (-02 is not a gear ratio)
        assert_eq!(MotorModel::from_hint("5036-02"), Some(MotorModel::M5036_36));
        assert_eq!(MotorModel::from_hint("5036_02"), Some(MotorModel::M5036_36));
        // case-insensitive
        assert_eq!(
            MotorModel::from_hint("M5046_20"),
            Some(MotorModel::M5046_20)
        );
        assert_eq!(MotorModel::from_hint("5046-20"), Some(MotorModel::M5046_20));
    }

    #[test]
    fn from_hint_rejects_unknown_model_codes() {
        assert_eq!(MotorModel::from_hint("9999_99"), None);
        assert_eq!(MotorModel::from_hint("garbage"), None);
    }

    #[test]
    fn from_hint_accepts_null_and_none() {
        assert_eq!(MotorModel::from_hint("null"), Some(MotorModel::Null));
        assert_eq!(MotorModel::from_hint("none"), Some(MotorModel::Null));
    }

    /// `tqe_adjust_to_raw` / `tqe_restore_raw` should be near-inverses under
    /// every model's coefficients. Null is skipped (k=0, always returns 0).
    #[test]
    fn tqe_adjust_restore_round_trip_per_model() {
        let models = [
            MotorModel::M3536_32,
            MotorModel::M4438_30,
            MotorModel::M4438_32,
            MotorModel::M4538_19,
            MotorModel::M5043_20,
            MotorModel::M5046_20,
            MotorModel::M5047_09,
            MotorModel::M5047_36,
            MotorModel::M5036_36,
            MotorModel::M6056_36,
            MotorModel::M7256_35,
            MotorModel::M60SG_35,
            MotorModel::M60BM_35,
            MotorModel::General,
        ];
        for m in models {
            let c = m.coeff();
            for tau in [0.0f32, 0.5, 1.0, 2.0, -1.0, 5.0] {
                let raw = tqe_adjust_to_raw(tau, c);
                let restored = tqe_restore_raw(raw, c);
                // TINT16 scale=100 quantization error ≤ k * 0.01
                let tol = (c.k * 0.01).max(1e-6);
                assert!(
                    (restored - tau).abs() < tol,
                    "model {m:?} k={} tau={tau} raw={raw} restored={restored} (tol {tol})",
                    c.k
                );
            }
        }
    }

    /// Null model (k=0): write torque is always 0, avoiding division by zero.
    #[test]
    fn tqe_adjust_to_raw_null_model_returns_zero() {
        let c = MotorModel::Null.coeff();
        assert_eq!(tqe_adjust_to_raw(5.0, c), 0);
        assert_eq!(tqe_restore_raw(0, c), 0.0);
    }

    /// General (k=0.5): 1.0 Nm → ((1.0/0.5)*100) = 200; raw 200 → 200*0.01*0.5 = 1.0.
    #[test]
    fn tqe_adjust_to_raw_general_k_half() {
        let c = MotorModel::General.coeff();
        assert_eq!(c.k, 0.5);
        assert_eq!(tqe_adjust_to_raw(1.0, c), 200);
        assert_eq!(tqe_restore_raw(200, c), 1.0);
    }

    /// Quantization error: with General k=0.5, 0.005 Nm is under one raw step (0.01*0.5=0.005).
    #[test]
    fn tqe_adjust_to_raw_quantizes_to_steps_of_k_times_001() {
        let c = MotorModel::General.coeff();
        // 0.007 Nm → 0.007/0.5=0.014 → *100=1.4 → round=1 → raw=1 → restore=0.005
        let raw = tqe_adjust_to_raw(0.007, c);
        let restored = tqe_restore_raw(raw, c);
        assert_eq!(raw, 1);
        assert!((restored - 0.005).abs() < 1e-6, "restored={restored}");
    }
}
