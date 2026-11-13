//! Huayi object dictionary — mode numbers, manufacturer objects, error codes.
//!
//! Standard `60xx` profile indices are **not** duplicated here; use
//! `motor_canopen::objects`. Everything below is Huayi-specific.

use motor_core::error::{MotorError, Result};

/// Huayi run modes (`6060h` values). Semantic red line: `5` is **MIT** on
/// this device — the same numeric value is CSP on RobStride CiA402. This
/// enum must never be shared with another vendor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RunMode {
    ProfilePosition = 1,
    ProfileVelocity = 3,
    Torque = 4,
    Mit = 5,
    InterpolatedPosition = 7,
    DirectVs = 8,
}

impl RunMode {
    /// Parse a `6061h` (mode display) or `6060h` value.
    pub fn from_raw(value: i8) -> Result<Self> {
        match value {
            1 => Ok(Self::ProfilePosition),
            3 => Ok(Self::ProfileVelocity),
            4 => Ok(Self::Torque),
            5 => Ok(Self::Mit),
            7 => Ok(Self::InterpolatedPosition),
            8 => Ok(Self::DirectVs),
            _ => Err(MotorError::Protocol(format!(
                "unsupported Huayi run mode value {value}"
            ))),
        }
    }
}

/// Manufacturer object indices and protocol magic codes (manual §5.1/§5.2).
pub mod od {
    // --- manufacturer region 2xxx ---
    /// Device settings: `_1` node id, `_2/3/4` CAN-FD baud/BRS (store +
    /// power cycle to take effect).
    pub const DEVICE_SETTING: u16 = 0x2001;
    /// Controller gains: `_1` velocity / `_2` position / `_3` Idq.
    pub const CONTROLLER_GAINS: u16 = 0x2002;
    /// MIT control parameters (slow path): `_1` Pdes (Rev), `_2` Vdes
    /// (Rev/s), `_3` Tff (Nm), `_4` Kp (u16 dimensionless), `_5` Kd (u16
    /// dimensionless), `_6` P/D-term torque limit (‰ of peak), `_7` Kp/Kd
    /// scale factor (Float32, read-only).
    pub const MIT_PARAMS: u16 = 0x2003;
    /// Compressed MIT control parameters: `_1` enable (0 = params from
    /// 2003h, 1 = params from 2004h), `_2` control word bits 31:0, `_3`
    /// bits 63:32, `_4..=_D` field ranges (Float32), `_E` P/D-term torque
    /// limit (‰ of peak).
    pub const MIT_COMPRESSED: u16 = 0x2004;
    /// Load parameters: `_1` inertia ratio, `_2` viscous friction.
    pub const LOAD_PARAMETER: u16 = 0x2010;
    /// Direct Vs control: `_1` duty (Pct), `_2` slew (Pct/s).
    pub const DIRECT_VS: u16 = 0x2020;
    /// Temperatures: `_1` driver, `_2` motor (both 0.1 °C).
    pub const TEMPERATURES: u16 = 0x2204;

    // --- manufacturer region 3xxx ---
    /// Position preset: `_1` value (Rev, range ±0.5), `_2` option code.
    /// Only writable in Switch On Disabled.
    pub const POSITION_PRESET: u16 = 0x3001;
    /// Manufacturer option codes: reset device / factory calibration
    /// (calibration codes are factory-only and not exposed).
    pub const MANUFACTURER_OPTION: u16 = 0x3002;

    // --- communication region 1xxx ---
    /// Store parameters: `_1` all / `_2` communication / `_3` sub-protocol
    /// / `_4` manufacturer region (§5.1).
    pub const STORE_PARAMS: u16 = 0x1010;
    /// Restore parameters: same sub-index split as 1010h; takes effect
    /// after reset / power cycle.
    pub const RESTORE_PARAMS: u16 = 0x1011;
    /// Heartbeat producer time in ms (0 = off), default 1000.
    pub const HEARTBEAT_PRODUCER: u16 = 0x1017;
    /// EMCY COB-ID: write `0x080 + node_id` to enable, `0x8000_0080 +
    /// node_id` to disable.
    pub const EMCY_COB_ID: u16 = 0x1014;

    // --- protocol magic codes ---
    /// 1010h signature, "save" (§5.1).
    pub const STORE_SIGNATURE: u32 = 0x6576_6173;
    /// 1011h signature, "load".
    pub const RESTORE_SIGNATURE: u32 = 0x6461_6F6C;
    /// 3001h_2 position preset code, "pres".
    pub const POSITION_PRESET_CODE: u32 = 0x7365_7270;
    /// 3002h_0 device reset code, "rese".
    pub const MANUFACTURER_RESET_CODE: u32 = 0x6573_6572;

    /// 3001h_1 value range in Rev (±0.5).
    pub const POSITION_PRESET_RANGE_REV: (f32, f32) = (-0.5, 0.5);
}

/// `6076h` — motor **peak torque** (Float32, Nm, read-only, factory
/// calibrated; manual §5.1 names it "Motor Peak Torque"). The base for the
/// Int16 permille torque encoding of 6071h/6077h. Kept vendor-private like
/// RobStride's 6078h/6079h, because the "peak" reading of this index is a
/// Huayi fact.
pub const MOTOR_PEAK_TORQUE: u16 = 0x6076;

/// `603Fh` error code → description (manual §4.1).
pub const ERROR_CODES: &[(u16, &str)] = &[
    (0x2310, "相线过流 (phase overcurrent)"),
    (0x3210, "母线过压 (DC overvoltage)"),
    (0x3220, "母线欠压 (DC undervoltage)"),
    (0x4210, "驱动器过温 (drive overtemperature)"),
    (0x5300, "CPU 错误 (CPU error)"),
    (0x5530, "EEPROM 存储错误 (EEPROM error)"),
    (0x7121, "电机堵转 (motor stalled)"),
    (0x7124, "电机过温 (motor overtemperature)"),
    (0x7330, "电机霍尔传感器错误 (Hall sensor error)"),
    (0x7340, "电机编码器错误 (encoder error)"),
    (0x8130, "心跳检测错误 (heartbeat error)"),
    (0x8140, "总线恢复错误 (bus recovery error)"),
    (0x8312, "启动困难 (start-up difficulty)"),
    (0x8400, "速度控制错误 (velocity control error)"),
    (0x8500, "位置控制错误 (position control error)"),
];

/// Human-readable description for a `603Fh` / EMCY error code, if known.
pub fn error_code_description(code: u16) -> Option<&'static str> {
    ERROR_CODES
        .iter()
        .find(|(c, _)| *c == code)
        .map(|(_, d)| *d)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mode_5_is_mit() {
        // Semantic red line: on Huayi 6060h=5 means MIT (CSP on RobStride).
        assert_eq!(RunMode::from_raw(5).expect("mode 5"), RunMode::Mit);
        assert_eq!(RunMode::Mit as i8, 5);
        assert_eq!(RunMode::from_raw(1).expect("mode 1"), RunMode::ProfilePosition);
        assert_eq!(RunMode::from_raw(3).expect("mode 3"), RunMode::ProfileVelocity);
        assert_eq!(RunMode::from_raw(4).expect("mode 4"), RunMode::Torque);
        assert!(RunMode::from_raw(0).is_err());
        assert!(RunMode::from_raw(2).is_err());
        assert!(RunMode::from_raw(6).is_err());
    }

    #[test]
    fn error_code_table_lookups() {
        assert_eq!(
            error_code_description(0x2310),
            Some("相线过流 (phase overcurrent)")
        );
        assert_eq!(error_code_description(0x0000), None);
    }
}
