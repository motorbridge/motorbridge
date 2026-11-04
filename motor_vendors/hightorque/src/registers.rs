//! HighTorque motor register table.
//!
//! Data source: HighTorque register function sheets (register functions,
//! motor run modes, error codes, one-drives-many mode, xlsx sheet1).
//! Address range 0x00–0x45, including reserved entries.
//!
//! 2026-10-22 decision (user): register reads are **fixed Int16** — callers
//! no longer choose a data type; read values are converted to physical
//! values per this table's `quantity` classification (conversions provided
//! by the user: position/velocity→rad, torque→N·m via the model k
//! coefficient, current/voltage→A/V, temperature→°C, PWM/rKp/rKd→
//! normalized, PID→0.1/LSB); **registers unclassified in the table are
//! returned 1:1 as raw values**. Entries classified by name (position
//! command/velocity command/limits etc.) are inferred, pending real-bus
//! calibration.
//!
//! Same-day follow-up decision: **gap and out-of-table addresses are
//! rejected** — holes inside 0x00-0x45 that the xlsx lacks
//! (0x07-0x0C/0x13/0x17/0x1E/0x1F/0x2A/0x2E/0x2F/0x35-0x3F) and 0x46-0xFF
//! are all outside this table; reads/writes are rejected atomically (those
//! addresses can be framed and will reply, but their semantics are guesses,
//! so they are no longer admitted).

use crate::protocol::{
    decode_unit_raw, tqe_restore_raw, AngleUnit, RegisterType, RegisterValue, TorqueCoeff,
    ACC_SCALE, CUR_SCALE, PID_SCALE, POS_SCALE, PWM_SCALE, TEMP_SCALE, TIME_SCALE, TQE_SCALE,
    VEL_SCALE, VOL_SCALE,
};

/// Physical quantity a register belongs to (determines the raw→physical
/// conversion). `None` (unclassified in the table, or an out-of-table
/// address) = identity, returns the raw value.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Quantity {
    /// Position (rad).
    Pos,
    /// Velocity (rad/s).
    Vel,
    /// Acceleration (rad/s²; the int16 scale follows firmware convert.c's
    /// 1000 — the dispute with the legacy doc's 0.01 is noted at the
    /// ACC_SCALE comment in protocol.rs, pending real-bus calibration).
    Acc,
    /// Torque (N·m; applies model k/d coefficients, same formula as the
    /// state path).
    Torque,
    /// Current (A).
    Current,
    /// Voltage (V).
    Voltage,
    /// Temperature (°C).
    Temperature,
    /// Time (s; no time registers in the current table, scale reserved).
    Time,
    /// PWM / rKp / rKd (normalized; raw=32767 maps to 1.0).
    Pwm,
    /// PID gain (0.1/LSB).
    Pid,
}

impl Quantity {
    /// Unit label for output (CLI/gateway display; normalized and PID are
    /// dimensionless and return empty).
    pub fn unit_label(self) -> &'static str {
        match self {
            Self::Pos => "rad",
            Self::Vel => "rad/s",
            Self::Acc => "rad/s^2",
            Self::Torque => "N·m",
            Self::Current => "A",
            Self::Voltage => "V",
            Self::Temperature => "°C",
            Self::Time => "s",
            Self::Pwm => "",
            Self::Pid => "",
        }
    }
}

/// Register metadata.
#[derive(Debug, Clone, Copy)]
pub struct RegisterInfo {
    /// Register address (0x00–0x45).
    pub addr: u8,
    /// Register name.
    pub name: &'static str,
    /// Whether the register is writable.
    pub writable: bool,
    /// Quantity classification (determines the conversion); `None` means
    /// no conversion, read value is 1:1.
    pub quantity: Option<Quantity>,
}

/// Full register table (ascending by address). Quantity classification:
/// 0x01-0x03 mirrors the state-frame conversion; current/voltage/PID mirror
/// firmware convert.c; temperature/PWM/rKp/rKd use conversions the user
/// provided on 2026-10-22; the rest are inferred by name (see the inline
/// notes on each entry).
pub const PARAMETER_TABLE: &[RegisterInfo] = &[
    RegisterInfo {
        addr: 0x00,
        name: "模式",
        writable: true,
        quantity: None,
    },
    // Position/velocity/torque: the reference firmware motor.c reads 3 int16
    // at once with 0x17 0x01; the conversion mirrors the 27 01 state frame
    // (POS/VEL/TQE_SCALE + k).
    RegisterInfo {
        addr: 0x01,
        name: "位置",
        writable: false,
        quantity: Some(Quantity::Pos),
    },
    RegisterInfo {
        addr: 0x02,
        name: "速度",
        writable: false,
        quantity: Some(Quantity::Vel),
    },
    RegisterInfo {
        addr: 0x03,
        name: "转矩",
        writable: false,
        quantity: Some(Quantity::Torque),
    },
    RegisterInfo {
        addr: 0x04,
        name: "Q 相电流",
        writable: false,
        quantity: Some(Quantity::Current),
    },
    RegisterInfo {
        addr: 0x05,
        name: "D 相电流",
        writable: false,
        quantity: Some(Quantity::Current),
    },
    RegisterInfo {
        addr: 0x06,
        name: "保留",
        writable: false,
        quantity: None,
    },
    RegisterInfo {
        addr: 0x0D,
        name: "电压",
        writable: false,
        quantity: Some(Quantity::Voltage),
    },
    RegisterInfo {
        addr: 0x0E,
        name: "温度",
        writable: false,
        quantity: Some(Quantity::Temperature),
    },
    RegisterInfo {
        addr: 0x0F,
        name: "错误代码",
        writable: false,
        quantity: None,
    },
    RegisterInfo {
        addr: 0x10,
        name: "PWM 相位 A",
        writable: true,
        quantity: Some(Quantity::Pwm),
    },
    RegisterInfo {
        addr: 0x11,
        name: "PWM 相位 B",
        writable: true,
        quantity: Some(Quantity::Pwm),
    },
    RegisterInfo {
        addr: 0x12,
        name: "PWM 相位 C",
        writable: true,
        quantity: Some(Quantity::Pwm),
    },
    RegisterInfo {
        addr: 0x14,
        name: "电压相位 A",
        writable: true,
        quantity: Some(Quantity::Voltage),
    },
    RegisterInfo {
        addr: 0x15,
        name: "电压相位 B",
        writable: true,
        quantity: Some(Quantity::Voltage),
    },
    RegisterInfo {
        addr: 0x16,
        name: "电压相位 C",
        writable: true,
        quantity: Some(Quantity::Voltage),
    },
    RegisterInfo {
        addr: 0x18,
        name: "电压 FOC 角度",
        // Angle quantity, but the user provided no conversion for this
        // register → 1:1 raw value.
        writable: true,
        quantity: None,
    },
    RegisterInfo {
        addr: 0x19,
        name: "电压 FOC 电压",
        writable: true,
        quantity: Some(Quantity::Voltage),
    },
    RegisterInfo {
        addr: 0x1A,
        name: "D 电压",
        writable: true,
        quantity: Some(Quantity::Voltage),
    },
    RegisterInfo {
        addr: 0x1B,
        name: "Q 电压",
        writable: true,
        quantity: Some(Quantity::Voltage),
    },
    RegisterInfo {
        addr: 0x1C,
        name: "Q 电流",
        writable: true,
        quantity: Some(Quantity::Current),
    },
    RegisterInfo {
        addr: 0x1D,
        name: "D 电流",
        writable: true,
        quantity: Some(Quantity::Current),
    },
    RegisterInfo {
        addr: 0x20,
        name: "位置指令",
        // Classified as position by name (inferred, pending real-bus
        // calibration).
        writable: true,
        quantity: Some(Quantity::Pos),
    },
    RegisterInfo {
        addr: 0x21,
        name: "速度命令",
        writable: true,
        quantity: Some(Quantity::Vel),
    },
    RegisterInfo {
        addr: 0x22,
        name: "前馈扭矩",
        writable: true,
        quantity: Some(Quantity::Torque),
    },
    RegisterInfo {
        addr: 0x23,
        name: "Kp 比例",
        writable: true,
        quantity: Some(Quantity::Pwm),
    },
    RegisterInfo {
        addr: 0x24,
        name: "Kd 比例",
        writable: true,
        quantity: Some(Quantity::Pwm),
    },
    RegisterInfo {
        addr: 0x25,
        name: "最大扭矩",
        writable: true,
        quantity: Some(Quantity::Torque),
    },
    RegisterInfo {
        addr: 0x26,
        name: "停止位置",
        writable: true,
        quantity: Some(Quantity::Pos),
    },
    RegisterInfo {
        addr: 0x27,
        name: "保留",
        writable: false,
        quantity: None,
    },
    RegisterInfo {
        addr: 0x28,
        name: "速度限制",
        writable: true,
        quantity: Some(Quantity::Vel),
    },
    RegisterInfo {
        addr: 0x29,
        name: "加速度限制",
        writable: true,
        quantity: Some(Quantity::Acc),
    },
    RegisterInfo {
        addr: 0x2B,
        name: "Kp",
        writable: true,
        quantity: Some(Quantity::Pid),
    },
    RegisterInfo {
        addr: 0x2C,
        name: "Kd",
        writable: true,
        quantity: Some(Quantity::Pid),
    },
    RegisterInfo {
        addr: 0x2D,
        name: "Ki",
        writable: true,
        quantity: Some(Quantity::Pid),
    },
    RegisterInfo {
        addr: 0x30,
        name: "比例扭矩",
        writable: false,
        quantity: Some(Quantity::Torque),
    },
    RegisterInfo {
        addr: 0x31,
        name: "积分扭矩",
        writable: false,
        quantity: Some(Quantity::Torque),
    },
    RegisterInfo {
        addr: 0x32,
        name: "微分扭矩",
        writable: false,
        quantity: Some(Quantity::Torque),
    },
    RegisterInfo {
        addr: 0x33,
        name: "前馈扭矩",
        writable: false,
        quantity: Some(Quantity::Torque),
    },
    RegisterInfo {
        addr: 0x34,
        name: "总控制扭矩",
        writable: false,
        quantity: Some(Quantity::Torque),
    },
    RegisterInfo {
        addr: 0x40,
        name: "下限",
        // Classified as position limit by name (inferred, pending real-bus
        // calibration).
        writable: true,
        quantity: Some(Quantity::Pos),
    },
    RegisterInfo {
        addr: 0x41,
        name: "上限",
        writable: true,
        quantity: Some(Quantity::Pos),
    },
    RegisterInfo {
        addr: 0x42,
        name: "前馈扭矩(映射 0x22)",
        writable: false,
        quantity: Some(Quantity::Torque),
    },
    RegisterInfo {
        addr: 0x43,
        name: "Kp 比例(映射 0x23)",
        writable: false,
        quantity: Some(Quantity::Pwm),
    },
    RegisterInfo {
        addr: 0x44,
        name: "Kd 比例(映射 0x24)",
        writable: false,
        quantity: Some(Quantity::Pwm),
    },
    RegisterInfo {
        addr: 0x45,
        name: "最大扭矩(映射 0x25)",
        writable: false,
        quantity: Some(Quantity::Torque),
    },
];

/// Look up by address.
pub fn parameter_info(addr: u8) -> Option<&'static RegisterInfo> {
    PARAMETER_TABLE.iter().find(|info| info.addr == addr)
}

/// Whether the address is in the register table (2026-10-22 follow-up
/// decision: gap and out-of-table addresses always return `false`,
/// reads/writes are rejected atomically).
pub fn is_known_register(addr: u8) -> bool {
    parameter_info(addr).is_some()
}

/// Validate that a consecutive read span lies entirely inside the table
/// (atomic over the whole span; any out-of-range address errors at once;
/// 0xB3/0xB4 are command addresses and do not count as registers). The
/// error text carries the specific address for upper layers to pass
/// through.
pub fn ensure_known_span(addr: u8, count: u8) -> Result<(), String> {
    for i in 0..count {
        let a = addr.wrapping_add(i);
        if !is_known_register(a) {
            return Err(format!(
                "register 0x{a:02X} is not in the parameter table \
                 (缺号/表外地址拒绝访问, 2026-10-22)"
            ));
        }
    }
    Ok(())
}

/// Quantity classification by address; unclassified or out-of-table
/// addresses return `None` (1:1 conversion).
pub fn parameter_quantity(addr: u8) -> Option<Quantity> {
    parameter_info(addr).and_then(|info| info.quantity)
}

/// Raw register value → physical quantity (2026-10-22 decision: reads are
/// fixed Int16 and return converted physical values; 1:1 when no
/// conversion applies).
///
/// - `RegisterValue::F32` returns as-is (a float raw is already a physical
///   value);
/// - Position/velocity/acceleration: raw/scale to turns, then ×2π to rad
///   (same formula as the state path's `decode_read_reply`, bit-identical);
/// - Torque: Int16 goes directly through `tqe_restore_raw`
///   (raw×0.01×k+d, the same function as the state path); foreign Int8/I32
///   cache entries generalize to raw/scale×k+d;
/// - Other quantities: raw/scale (via `decode_unit_raw`, reusing the
///   single QuantityScale source);
/// - `None` (unclassified in the table): 1:1 raw value. Gap/out-of-table
///   addresses are rejected at the entry points
///   (`ensure_known_span`/`is_known_register`) and never reach this
///   function.
pub fn register_physical_value(addr: u8, v: RegisterValue, c: TorqueCoeff) -> f32 {
    let raw = v.as_f32();
    let rt = v.register_type();
    if rt == RegisterType::Float {
        return raw;
    }
    match parameter_quantity(addr) {
        None => raw,
        Some(Quantity::Torque) => {
            if let RegisterValue::I16(x) = v {
                tqe_restore_raw(x, c)
            } else {
                // Foreign frames (Int8/I32) use the same formula generalized
                // via scale.
                raw / TQE_SCALE.scale(rt.data_type()) * c.k + c.d
            }
        }
        Some(q) => {
            let (scale, angle) = match q {
                Quantity::Pos => (POS_SCALE, true),
                Quantity::Vel => (VEL_SCALE, true),
                Quantity::Acc => (ACC_SCALE, true),
                Quantity::Current => (CUR_SCALE, false),
                Quantity::Voltage => (VOL_SCALE, false),
                Quantity::Temperature => (TEMP_SCALE, false),
                Quantity::Time => (TIME_SCALE, false),
                Quantity::Pwm => (PWM_SCALE, false),
                Quantity::Pid => (PID_SCALE, false),
                Quantity::Torque => unreachable!("handled above"),
            };
            // Angle quantities convert to rad; non-angle quantities pass
            // through unit conversion with the Turn identity, degenerating
            // to raw/scale.
            let unit = if angle {
                AngleUnit::Rad
            } else {
                AngleUnit::Turn
            };
            decode_unit_raw(raw, unit, rt.data_type(), scale)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::RegisterValue;

    #[test]
    fn parameter_table_has_no_duplicate_addresses() {
        let mut addrs: Vec<u8> = PARAMETER_TABLE.iter().map(|i| i.addr).collect();
        addrs.sort_unstable();
        let before = addrs.len();
        addrs.dedup();
        assert_eq!(addrs.len(), before, "duplicate register addresses present");
    }

    #[test]
    fn known_registers_resolved() {
        assert_eq!(parameter_info(0x01).unwrap().name, "位置");
        assert_eq!(parameter_info(0x0F).unwrap().name, "错误代码");
        assert_eq!(parameter_info(0x45).unwrap().name, "最大扭矩(映射 0x25)");
    }

    #[test]
    fn unknown_address_returns_none() {
        assert!(parameter_info(0x07).is_none());
        assert!(parameter_info(0x50).is_none());
    }

    /// 2026-10-22 follow-up decision: gap and out-of-table addresses are
    /// rejected outright.
    #[test]
    fn known_register_membership() {
        // In table
        assert!(is_known_register(0x00));
        assert!(is_known_register(0x01));
        assert!(is_known_register(0x45));
        // Gap (holes inside 0x00-0x45)
        assert!(!is_known_register(0x07));
        assert!(!is_known_register(0x0A));
        assert!(!is_known_register(0x13));
        assert!(!is_known_register(0x2E));
        assert!(!is_known_register(0x35));
        // Out of table (0x46-0xFF)
        assert!(!is_known_register(0x46));
        assert!(!is_known_register(0x50));
        assert!(!is_known_register(0xB3));
        assert!(!is_known_register(0xFF));
    }

    /// Span validation: any out-of-range address errors at once, with the
    /// specific address in the error text.
    #[test]
    fn ensure_known_span_atomic() {
        assert!(ensure_known_span(0x01, 3).is_ok());
        assert!(ensure_known_span(0x45, 1).is_ok());
        // Single gap/out-of-table address
        let e = ensure_known_span(0x0A, 1).unwrap_err();
        assert!(e.contains("0x0A"), "err: {e}");
        let e = ensure_known_span(0x50, 1).unwrap_err();
        assert!(e.contains("0x50"), "err: {e}");
        // Consecutive read crossing into a gap: 0x2D is in table, 0x2E is a gap
        let e = ensure_known_span(0x2D, 2).unwrap_err();
        assert!(e.contains("0x2E"), "err: {e}");
        // Consecutive read crossing past the table end: 0x45 is in table,
        // 0x46 is out of table
        let e = ensure_known_span(0x45, 2).unwrap_err();
        assert!(e.contains("0x46"), "err: {e}");
        // 0xB3/0xB4 are command addresses, not registers
        assert!(!is_known_register(0xB3));
        assert!(!is_known_register(0xB4));
    }

    /// 2026-10-22 decision: addresses classified by physical quantity;
    /// those without a conversion (0x00/0x06/0x0F/0x18/0x27/out-of-table)
    /// have quantity None → read value 1:1.
    #[test]
    fn quantity_assignment_by_class() {
        assert_eq!(parameter_quantity(0x01), Some(Quantity::Pos));
        assert_eq!(parameter_quantity(0x02), Some(Quantity::Vel));
        assert_eq!(parameter_quantity(0x03), Some(Quantity::Torque));
        assert_eq!(parameter_quantity(0x04), Some(Quantity::Current));
        assert_eq!(parameter_quantity(0x0D), Some(Quantity::Voltage));
        assert_eq!(parameter_quantity(0x0E), Some(Quantity::Temperature));
        assert_eq!(parameter_quantity(0x10), Some(Quantity::Pwm));
        assert_eq!(parameter_quantity(0x2B), Some(Quantity::Pid));
        assert_eq!(parameter_quantity(0x29), Some(Quantity::Acc));
        // None (1:1): mode/reserved/error code/FOC angle (no conversion).
        assert_eq!(parameter_quantity(0x00), None);
        assert_eq!(parameter_quantity(0x06), None);
        assert_eq!(parameter_quantity(0x0F), None);
        assert_eq!(parameter_quantity(0x18), None);
        assert_eq!(parameter_quantity(0x27), None);
        assert_eq!(parameter_quantity(0x50), None);
    }

    /// Unit labels (CLI/gateway display).
    #[test]
    fn unit_labels() {
        assert_eq!(Quantity::Pos.unit_label(), "rad");
        assert_eq!(Quantity::Vel.unit_label(), "rad/s");
        assert_eq!(Quantity::Torque.unit_label(), "N·m");
        assert_eq!(Quantity::Temperature.unit_label(), "°C");
        assert_eq!(Quantity::Pwm.unit_label(), "");
    }

    /// Conversion spot-check: same formula as the state path.
    /// 0x01 raw 0x1234 → 0.1234 turns × 2π.
    #[test]
    fn converts_pos_like_state_path() {
        let c = TorqueCoeff { k: 0.5, d: 0.0 };
        let v = register_physical_value(0x01, RegisterValue::I16(0x1234), c);
        assert!((v - 0x1234 as f32 * 0.0001 * std::f32::consts::TAU).abs() < 1e-6);
    }

    /// Velocity: raw/4000 × 2π (same formula as the state path; note the
    /// use of /4000.0 rather than ×0.00025 — the two differ by ~2e-6 in
    /// f32, which exceeds the 1e-6 tolerance at values around 35).
    #[test]
    fn converts_vel_like_state_path() {
        let c = TorqueCoeff { k: 0.5, d: 0.0 };
        let v = register_physical_value(0x02, RegisterValue::I16(0x5678), c);
        assert!((v - 0x5678 as f32 / 4000.0 * std::f32::consts::TAU).abs() < 1e-6);
    }

    /// Torque: Int16 goes through tqe_restore_raw (raw×0.01×k+d), the same
    /// function as the state path.
    #[test]
    fn converts_torque_with_model_coeff() {
        let c = TorqueCoeff { k: 0.5, d: 0.0 };
        let v = register_physical_value(0x03, RegisterValue::I16(200), c);
        assert!((v - 200.0 * 0.01 * 0.5).abs() < 1e-6);
    }

    /// Temperature 0.1 °C/LSB, PWM 1/32767, PID 0.1/LSB.
    #[test]
    fn converts_temperature_pwm_pid() {
        let c = TorqueCoeff { k: 0.5, d: 0.0 };
        assert!((register_physical_value(0x0E, RegisterValue::I16(100), c) - 10.0).abs() < 1e-6);
        assert!((register_physical_value(0x10, RegisterValue::I16(32767), c) - 1.0).abs() < 1e-6);
        assert!((register_physical_value(0x2B, RegisterValue::I16(25), c) - 2.5).abs() < 1e-6);
    }

    /// No conversion → 1:1; F32 identity.
    #[test]
    fn identity_for_unclassified_and_float() {
        let c = TorqueCoeff { k: 0.5, d: 0.0 };
        assert!((register_physical_value(0x0F, RegisterValue::I16(5), c) - 5.0).abs() < 1e-6);
        assert_eq!(
            register_physical_value(0x50, RegisterValue::I16(-3), c),
            -3.0
        );
        assert_eq!(
            register_physical_value(0x03, RegisterValue::F32(1.5), c),
            1.5
        );
    }

    /// A foreign Int8 cache entry (e.g. an int8 torque frame sent by
    /// another host) converts via the same generalized formula:
    /// Int8 torque scale=2 → raw/2 × k.
    #[test]
    fn converts_foreign_int8_torque_entry() {
        let c = TorqueCoeff { k: 0.5, d: 0.0 };
        let v = register_physical_value(0x22, RegisterValue::I8(17), c);
        assert!((v - 17.0 / 2.0 * 0.5).abs() < 1e-6);
    }
}
