use crate::model::Target;
use motor_vendor_cyberbeast::CyberBeastController;
use serde_json::{json, Value};
use std::time::Duration;

const SCAN_TIMEOUT_MS: u64 = 200;
const SCAN_MAX_ID: u16 = 32;

/// Scan for CyberBeast motors on the CAN bus.
pub(crate) fn cmd_scan_cyberbeast(v: &Value, base: &Target) -> Result<Value, String> {
    let start_id = v.get("start_id").and_then(|s| s.as_u64()).unwrap_or(1) as u16;
    let end_id = v
        .get("end_id")
        .and_then(|s| s.as_u64())
        .unwrap_or(SCAN_MAX_ID as u64) as u16;
    let model = v
        .get("model")
        .and_then(|s| s.as_str())
        .unwrap_or("odrive-default");

    let ctrl = CyberBeastController::new_socketcan(&base.channel)
        .map_err(|e| format!("open bus failed: {e}"))?;

    let mut found: Vec<Value> = Vec::new();
    for id in start_id..=end_id {
        let motor = match ctrl.add_motor(id, id, model) {
            Ok(m) => m,
            Err(_) => continue,
        };
        let _ = motor.send_start_motor();
        let _ = motor.send_query_status();
        std::thread::sleep(Duration::from_millis(SCAN_TIMEOUT_MS));
        let _ = ctrl.poll_feedback_once();

        if let Some(state) = motor.latest_state() {
            found.push(json!({
                "motor_id": id,
                "pos": state.pos,
                "vel": state.vel,
                "current": state.current,
                "error_code": state.error_code,
                "mode_state": state.mode_state,
                "motor_temp": state.motor_temp,
                "mos_temp": state.mos_temp,
                "heartbeat_life": state.heartbeat_life,
            }));
        }
        let _ = motor.send_stop_motor();
    }
    let _ = ctrl.shutdown();

    Ok(json!({
        "vendor": "cyberbeast",
        "found": found.len(),
        "motors": found,
    }))
}
