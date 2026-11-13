//! RobStride CiA402 vendor objects.
//!
//! Standard CANopen / CiA402 profile indices and command codes are
//! re-exported from [`motor_canopen::objects`]; everything kept here is
//! RobStride-specific: mode-number meanings (`6060h` values), unit
//! conversions (pulses per rev, 0.1 rpm, 1000 = 5 N·m torque scaling), the
//! `0xFFF` protocol-switch extended frame, and the `6099h` CAN watchdog.

// --- shared CANopen / CiA402 profile objects (no vendor semantics) ---
pub use motor_canopen::objects::{
    heartbeat_id, sdo_req_id, sdo_rsp_id, CONTROLWORD, CW_ENABLE_OPERATION, CW_FAULT_RESET,
    CW_QUICK_STOP, CW_SHUTDOWN, CW_SWITCH_ON, ERROR_CODE, MODES_OF_OPERATION,
    MODES_OF_OPERATION_DISPLAY, NMT_ID, NMT_START_REMOTE_NODE, POSITION_ACTUAL_VALUE,
    POSITION_DEMAND_VALUE, PROFILE_ACCELERATION, PROFILE_VELOCITY, STATUSWORD, TARGET_POSITION,
    TARGET_TORQUE, TARGET_VELOCITY, TORQUE_ACTUAL_VALUE, VELOCITY_ACTUAL_VALUE,
    VELOCITY_DEMAND_VALUE,
};

// --- RobStride-specific profile objects ---
pub const POSITION_WINDOW: u16 = 0x6067;
pub const POSITION_WINDOW_TIME: u16 = 0x6068;
pub const CURRENT_ACTUAL_VALUE: u16 = 0x6078;
pub const DC_LINK_CIRCUIT_VOLTAGE: u16 = 0x6079;
pub const HOMING_METHOD: u16 = 0x6098;
pub const CAN_WATCHDOG: u16 = 0x6099;

// --- RobStride mode numbers (6060h values are vendor facts) ---
pub const MODE_PROFILE_POSITION: i8 = 1;
pub const MODE_PROFILE_VELOCITY: i8 = 3;
pub const MODE_TORQUE: i8 = 4;
pub const MODE_CSP: i8 = 5;
pub const MODE_HOMING: i8 = 6;

// --- RobStride-specific controlword commands ---
/// RobStride's disable command (command table has no `0x0001`-style
/// distinction; `0x0001` reaches Switch On Disabled on this firmware).
pub const CW_DISABLE_TO_SWITCH_ON_DISABLED: u16 = 0x0001;

pub const PROTOCOL_SWITCH_EXT_ID: u32 = 0xFFF;
pub const PROTOCOL_PRIVATE: u8 = 0;
pub const PROTOCOL_CANOPEN: u8 = 1;
pub const PROTOCOL_MIT: u8 = 2;

pub const CAN_WATCHDOG_RAW_PER_SECOND: f32 = 20000.0;
pub const PULSES_PER_REV: f32 = 16384.0;
pub const TWO_PI: f32 = core::f32::consts::PI * 2.0;

pub fn rad_to_pulses(rad: f32) -> i32 {
    (rad / TWO_PI * PULSES_PER_REV).round() as i32
}

pub fn pulses_to_rad(pulses: i32) -> f32 {
    pulses as f32 / PULSES_PER_REV * TWO_PI
}

pub fn rad_s_to_0p1rpm(rad_s: f32) -> i32 {
    (rad_s * 60.0 / TWO_PI * 10.0).round() as i32
}

pub fn unit_0p1rpm_to_rad_s(raw: i32) -> f32 {
    raw as f32 / 10.0 * TWO_PI / 60.0
}

pub fn torque_nm_to_robstride_raw(nm: f32) -> i16 {
    // RobStride CANopen manual: 1000 represents 5 N.m.
    (nm * 200.0).round().clamp(i16::MIN as f32, i16::MAX as f32) as i16
}

pub fn robstride_raw_to_torque_nm(raw: i16) -> f32 {
    raw as f32 / 200.0
}

pub fn watchdog_seconds_to_raw(seconds: f32) -> u32 {
    if !seconds.is_finite() || seconds <= 0.0 {
        return 0;
    }
    (seconds * CAN_WATCHDOG_RAW_PER_SECOND)
        .round()
        .clamp(0.0, u32::MAX as f32) as u32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn converts_position_velocity_and_torque_units() {
        assert_eq!(rad_to_pulses(TWO_PI), 16384);
        assert!((pulses_to_rad(8192) - core::f32::consts::PI).abs() < 1e-5);
        assert_eq!(rad_s_to_0p1rpm(TWO_PI), 600);
        assert!((unit_0p1rpm_to_rad_s(600) - TWO_PI).abs() < 1e-5);
        assert_eq!(torque_nm_to_robstride_raw(5.0), 1000);
        assert!((robstride_raw_to_torque_nm(1000) - 5.0).abs() < 1e-6);
        assert_eq!(watchdog_seconds_to_raw(1.0), 20000);
        assert_eq!(watchdog_seconds_to_raw(0.0), 0);
    }
}
