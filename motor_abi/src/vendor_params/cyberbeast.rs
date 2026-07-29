use crate::MotorHandleInner;
use motor_vendor_cyberbeast::CyberBeastMotor;
use std::sync::Arc;
use std::time::Duration;

const DEFAULT_PARAM_TIMEOUT_MS: u64 = 200;

/// Get a float32 parameter value from a CyberBeast motor by SDO endpoint ID.
pub(crate) fn get_f32(
    motor: &MotorHandleInner,
    param_id: u16,
    timeout_ms: u32,
) -> Result<f32, String> {
    let m: &Arc<CyberBeastMotor> = match motor {
        MotorHandleInner::CyberBeast(m) => m,
        _ => return Err("motor is not a CyberBeast motor".to_string()),
    };
    let timeout = Duration::from_millis(u64::from(timeout_ms).max(DEFAULT_PARAM_TIMEOUT_MS));
    m.send_param_read(param_id).map_err(|e| e.to_string())?;
    m.get_param_f32(param_id, timeout).map_err(|e| e.to_string())
}

/// Write a float32 parameter value to a CyberBeast motor by SDO endpoint ID.
pub(crate) fn write_f32(
    motor: &MotorHandleInner,
    param_id: u16,
    value: f32,
) -> Result<(), String> {
    let m: &Arc<CyberBeastMotor> = match motor {
        MotorHandleInner::CyberBeast(m) => m,
        _ => return Err("motor is not a CyberBeast motor".to_string()),
    };
    m.set_param_f32(param_id, value).map_err(|e| e.to_string())
}
