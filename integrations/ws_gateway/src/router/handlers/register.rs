use crate::commands::{
    as_bool, as_u16, as_u64, handle_robstride_read_param, handle_robstride_write_param,
    parse_damiao_mode, parse_robstride_mode,
};
use crate::model::{ControllerHandle, MotorHandle};
use crate::session::SessionCtx;
use crate::vendors::damiao_ws::ensure_control_mode_soft;
use crate::vendors::fashionstar_ws::parse_fashionstar_u64;
use motor_vendor_robstride::ParameterValue as RobstrideParameterValue;
use serde_json::{json, Value};
use std::time::Duration;

pub(crate) fn handle(op: &str, v: &Value, ctx: &mut SessionCtx) -> Option<Result<Value, String>> {
    match op {
        "clear_error" => Some(handle_clear_error(v, ctx)),
        "set_zero_position" => Some(handle_set_zero_position(v, ctx)),
        "ensure_mode" => Some(handle_ensure_mode(v, ctx)),
        "request_feedback" => Some(handle_request_feedback(v, ctx)),
        "set_active_report" => Some(handle_set_active_report(v, ctx)),
        "store_parameters" => Some(handle_store_parameters(v, ctx)),
        "set_can_timeout_ms" => Some(handle_set_can_timeout_ms(v, ctx)),
        "write_register_u32" => Some(handle_write_register_u32(v, ctx)),
        "write_register_f32" => Some(handle_write_register_f32(v, ctx)),
        "get_register_u32" => Some(handle_get_register_u32(v, ctx)),
        "get_register_f32" => Some(handle_get_register_f32(v, ctx)),
        "robstride_ping" => Some(handle_robstride_ping(v, ctx)),
        "robstride_read_param" => Some(handle_robstride_read_param_op(v, ctx)),
        "robstride_write_param" => Some(handle_robstride_write_param_op(v, ctx)),
        _ => None,
    }
}

fn handle_clear_error(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.retarget_from_request_if_present(
        v.get("vendor")
            .and_then(Value::as_str)
            .map(crate::model::Vendor::from_str)
            .transpose()?,
        v.get("model").and_then(Value::as_str),
        v.get("motor_id")
            .map(|_| as_u16(v, "motor_id", ctx.target.motor_id)),
        v.get("feedback_id")
            .map(|_| as_u16(v, "feedback_id", ctx.target.feedback_id)),
    )?;
    ctx.ensure_connected()?;
    match ctx.motor.as_ref() {
        Some(MotorHandle::Damiao(m)) => m.clear_error().map_err(|e| e.to_string())?,
        Some(MotorHandle::Robstride(m)) => m.clear_error().map_err(|e| e.to_string())?,
        Some(MotorHandle::Hexfellow(_)) => {
            return Err("clear_error is not supported for hexfellow".to_string())
        }
        Some(MotorHandle::Hightorque(m)) => {
            // Table 2 mode 0 is literally "stop, clear error": the single
            // `0x01 0x00 0x00` frame both stops and clears, same frame as
            // the stop op (G3: protocol-level clear_error ≡ stop; converged
            // through the vendor, frame type follows the global default of
            // extended 29-bit with bit15).
            m.clear_error().map_err(|e| e.to_string())?;
        }
        Some(MotorHandle::Myactuator(_)) => {
            return Err("clear_error is not supported for myactuator".to_string())
        }
        Some(MotorHandle::Fashionstar(_)) => {
            return Err("clear_error is not supported for fashionstar".to_string())
        }
        None => return Err("motor not connected".to_string()),
    }
    Ok(json!({"cleared": true}))
}

fn handle_set_zero_position(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.retarget_from_request_if_present(
        v.get("vendor")
            .and_then(Value::as_str)
            .map(crate::model::Vendor::from_str)
            .transpose()?,
        v.get("model").and_then(Value::as_str),
        v.get("motor_id")
            .map(|_| as_u16(v, "motor_id", ctx.target.motor_id)),
        v.get("feedback_id")
            .map(|_| as_u16(v, "feedback_id", ctx.target.feedback_id)),
    )?;
    ctx.ensure_connected()?;
    match ctx.motor.as_ref() {
        Some(MotorHandle::Damiao(m)) => m.set_zero_position().map_err(|e| e.to_string())?,
        Some(MotorHandle::Robstride(m)) => m.set_zero_position().map_err(|e| e.to_string())?,
        Some(MotorHandle::Myactuator(m)) => m
            .set_current_position_as_zero()
            .map_err(|e| e.to_string())?,
        Some(MotorHandle::Hexfellow(_)) => {
            return Err("set_zero_position is not supported for hexfellow".to_string())
        }
        Some(MotorHandle::Hightorque(m)) => {
            // Converged (2026-09-29): the gateway used to reject this, but
            // the vendor has always supported the 0x40 zero-position frame
            // (auto-persists internally) — aligned with ABI/Python.
            m.set_zero_position().map_err(|e| e.to_string())?;
        }
        Some(MotorHandle::Fashionstar(m)) => {
            ctx.active = None;
            m.set_zero_position().map_err(|e| e.to_string())?;
        }
        None => return Err("motor not connected".to_string()),
    }
    Ok(json!({"zero_set": true}))
}

fn handle_ensure_mode(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    let timeout_ms = as_u64(v, "timeout_ms", 1000);
    match ctx.motor.as_ref() {
        Some(MotorHandle::Damiao(m)) => {
            let mode = parse_damiao_mode(v)?;
            let warning = ensure_control_mode_soft(m, mode, Duration::from_millis(timeout_ms))?;
            if let Some(warning) = warning {
                return Ok(json!({"ensured": true, "warning": warning, "warnings": [warning]}));
            }
        }
        Some(MotorHandle::Robstride(m)) => {
            let mode = parse_robstride_mode(v)?;
            m.ensure_control_mode(mode, Duration::from_millis(timeout_ms))
                .map_err(|e| e.to_string())?;
        }
        Some(MotorHandle::Hexfellow(m)) => {
            let mode = v
                .get("mode")
                .and_then(Value::as_str)
                .unwrap_or("mit")
                .to_lowercase();
            let raw_mode = if mode == "mit" || mode == "1" {
                5
            } else if mode == "pos_vel" || mode == "pos-vel" || mode == "2" {
                1
            } else {
                return Err("hexfellow mode must be mit|pos_vel".to_string());
            };
            m.ensure_mode_enabled(raw_mode, Duration::from_millis(timeout_ms))
                .map_err(|e| e.to_string())?;
        }
        Some(MotorHandle::Myactuator(_)) => {
            return Err("ensure_mode is not supported for myactuator".to_string())
        }
        Some(MotorHandle::Hightorque(_)) => {
            return Err("ensure_mode is not supported for hightorque".to_string())
        }
        Some(MotorHandle::Fashionstar(m)) => {
            let mode = match v.get("mode") {
                Some(Value::String(mode)) => mode.to_lowercase(),
                Some(Value::Number(mode)) => mode.to_string(),
                _ => String::new(),
            };
            if !matches!(mode.as_str(), "pos_vel" | "pos-vel" | "2") {
                return Err("fashionstar mode must be pos_vel".to_string());
            }
            m.ensure_control_mode(2, Duration::from_millis(timeout_ms))
                .map_err(|e| e.to_string())?;
        }
        None => return Err("motor not connected".to_string()),
    }
    Ok(json!({"ensured": true}))
}

fn handle_request_feedback(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.retarget_from_request_if_present(
        v.get("vendor")
            .and_then(Value::as_str)
            .map(crate::model::Vendor::from_str)
            .transpose()?,
        v.get("model").and_then(Value::as_str),
        v.get("motor_id")
            .map(|_| as_u16(v, "motor_id", ctx.target.motor_id)),
        v.get("feedback_id")
            .map(|_| as_u16(v, "feedback_id", ctx.target.feedback_id)),
    )?;
    ctx.ensure_connected()?;
    match (&ctx.controller, &ctx.motor) {
        (Some(ControllerHandle::Damiao(_)), Some(MotorHandle::Damiao(m))) => {
            m.request_motor_feedback().map_err(|e| e.to_string())?;
        }
        (Some(ControllerHandle::Robstride(c)), Some(MotorHandle::Robstride(_))) => {
            c.poll_feedback_once().map_err(|e| e.to_string())?;
        }
        (Some(ControllerHandle::Hexfellow(c)), Some(MotorHandle::Hexfellow(_))) => {
            c.poll_feedback_once().map_err(|e| e.to_string())?;
        }
        (Some(ControllerHandle::Myactuator(c)), Some(MotorHandle::Myactuator(m))) => {
            m.request_status().map_err(|e| e.to_string())?;
            c.poll_feedback_once().map_err(|e| e.to_string())?;
        }
        (Some(ControllerHandle::Hightorque(_)), Some(MotorHandle::Hightorque(m))) => {
            // Converged (2026-10-12): command 8 (read state) is a special
            // case of command 7 (generic register read) — the 17 01 state
            // query = reading 3 int16s starting at 0x01. Non-blocking
            // "send and return" goes through the vendor method of the same
            // name (since 2026-10-24 it sends two frames: 17 01 + 16 0E
            // temperature/error code; real-bus testing showed the firmware
            // parses one pair per frame). Replies are decoded by the vendor
            // background receive thread and written into both the state and
            // register caches; the 26 0E reply patches t_mos/fault_code in
            // place (without bumping the state count); state_once/poll and
            // cached_register can all read it back.
            m.request_motor_feedback().map_err(|e| e.to_string())?;
        }
        (Some(ControllerHandle::Fashionstar(_)), Some(MotorHandle::Fashionstar(m))) => m
            .request_motor_feedback(Duration::from_millis(500))
            .map_err(|e| e.to_string())?,
        _ => return Err("motor not connected".to_string()),
    }
    Ok(json!({"requested": true}))
}

fn handle_set_active_report(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.retarget_from_request_if_present(
        v.get("vendor")
            .and_then(Value::as_str)
            .map(crate::model::Vendor::from_str)
            .transpose()?,
        v.get("model").and_then(Value::as_str),
        v.get("motor_id")
            .map(|_| as_u16(v, "motor_id", ctx.target.motor_id)),
        v.get("feedback_id")
            .map(|_| as_u16(v, "feedback_id", ctx.target.feedback_id)),
    )?;
    ctx.ensure_connected()?;
    let enabled = as_bool(v, "enabled", true);
    match ctx.motor.as_ref() {
        Some(MotorHandle::Robstride(m)) => {
            m.set_active_report(enabled).map_err(|e| e.to_string())?;
        }
        Some(MotorHandle::Damiao(_)) => {
            return Err("set_active_report is not supported for damiao".to_string())
        }
        Some(MotorHandle::Hexfellow(_)) => {
            return Err("set_active_report is not supported for hexfellow".to_string())
        }
        Some(MotorHandle::Hightorque(_)) => {
            return Err("set_active_report is not supported for hightorque".to_string())
        }
        Some(MotorHandle::Myactuator(_)) => {
            return Err("set_active_report is not supported for myactuator".to_string())
        }
        Some(MotorHandle::Fashionstar(_)) => {
            return Err("set_active_report is not supported for fashionstar".to_string())
        }
        None => return Err("motor not connected".to_string()),
    }
    Ok(json!({"active_report": enabled}))
}

fn handle_store_parameters(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.retarget_from_request_if_present(
        v.get("vendor")
            .and_then(Value::as_str)
            .map(crate::model::Vendor::from_str)
            .transpose()?,
        v.get("model").and_then(Value::as_str),
        v.get("motor_id")
            .map(|_| as_u16(v, "motor_id", ctx.target.motor_id)),
        v.get("feedback_id")
            .map(|_| as_u16(v, "feedback_id", ctx.target.feedback_id)),
    )?;
    ctx.ensure_connected()?;
    match ctx.motor.as_ref() {
        Some(MotorHandle::Damiao(m)) => {
            ctx.active = None;
            m.store_parameters().map_err(|e| e.to_string())?
        }
        Some(MotorHandle::Robstride(m)) => m.save_parameters().map_err(|e| e.to_string())?,
        Some(MotorHandle::Hexfellow(_)) => {
            return Err("store_parameters is not supported for hexfellow".to_string())
        }
        Some(MotorHandle::Hightorque(m)) => {
            // Converged (2026-09-29): the gateway used to reject this, but
            // the vendor has always supported the `05 B3` conf_write
            // persist — aligned with ABI/Python.
            m.store_parameters().map_err(|e| e.to_string())?;
        }
        Some(MotorHandle::Myactuator(_)) => {
            return Err("store_parameters is not supported for myactuator".to_string())
        }
        Some(MotorHandle::Fashionstar(_)) => {
            return Err("store_parameters is not supported for fashionstar".to_string())
        }
        None => return Err("motor not connected".to_string()),
    }
    Ok(json!({"stored": true}))
}

fn handle_set_can_timeout_ms(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    let timeout_ms = as_u64(v, "timeout_ms", 1000);
    match ctx.motor.as_ref() {
        Some(MotorHandle::Damiao(m)) => {
            let reg_value = (timeout_ms as u32).saturating_mul(20);
            m.write_register_u32(9, reg_value)
                .map_err(|e| e.to_string())?;
            Ok(json!({"timeout_ms": timeout_ms, "reg9_value": reg_value}))
        }
        Some(MotorHandle::Robstride(m)) => {
            m.write_parameter(0x7028, RobstrideParameterValue::U32(timeout_ms as u32))
                .map_err(|e| e.to_string())?;
            Ok(json!({"timeout_ms": timeout_ms, "param_id":"0x7028"}))
        }
        Some(MotorHandle::Hexfellow(_)) => {
            Err("set_can_timeout_ms is not supported for hexfellow".to_string())
        }
        Some(MotorHandle::Hightorque(_)) => {
            Err("set_can_timeout_ms is not supported for hightorque".to_string())
        }
        Some(MotorHandle::Myactuator(_)) => {
            Err("set_can_timeout_ms is not supported for myactuator".to_string())
        }
        Some(MotorHandle::Fashionstar(_)) => {
            Err("set_can_timeout_ms is not supported for fashionstar".to_string())
        }
        None => Err("motor not connected".to_string()),
    }
}

fn handle_write_register_u32(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    let rid = as_u16(v, "rid", 0) as u8;
    let value = as_u64(v, "value", 0) as u32;
    let verify = as_bool(v, "verify", true);
    let verify_timeout_ms = as_u64(v, "verify_timeout_ms", as_u64(v, "timeout_ms", 1000));
    let verify_attempts = as_u64(v, "verify_attempts", 2).clamp(1, 5);
    match ctx.motor.as_ref() {
        Some(MotorHandle::Damiao(m)) => {
            m.write_register_u32(rid, value)
                .map_err(|e| e.to_string())?;
            if !verify {
                return Ok(json!({"rid": rid, "value": value, "verified": false}));
            }
            match verify_register_u32(m, rid, value, verify_timeout_ms, verify_attempts) {
                Ok(readback) => Ok(json!({
                    "rid": rid,
                    "value": value,
                    "readback": readback,
                    "verified": true
                })),
                Err(VerifyError::Timeout(warning)) => Ok(json!({
                    "rid": rid,
                    "value": value,
                    "verified": false,
                    "warning": warning,
                    "warnings": [warning]
                })),
                Err(VerifyError::Mismatch(err)) => Err(err),
            }
        }
        Some(MotorHandle::Robstride(_)) => {
            Err("write_register_u32 is damiao-only; use robstride_write_param".to_string())
        }
        Some(MotorHandle::Fashionstar(m)) => {
            let rid = parse_fashionstar_u64(v, "rid", 0, 255)? as u8;
            let value = parse_fashionstar_u64(v, "value", 0, u32::MAX as u64)? as u32;
            // Manual motion, origin and output changes replace the periodic target.
            if matches!(rid, 0x28 | 0x50..=0x59 | 0x78..=0x7B) {
                ctx.active = None;
            }
            m.write_register_u32(rid, value)
                .map_err(|e| e.to_string())?;
            Ok(json!({
                "rid": rid,
                "value": value,
                "verified": false,
                "requires_power_cycle": !matches!(rid, 0x28 | 0x50..=0x59 | 0x79..=0x7B | 0xFF)
            }))
        }
        Some(_) => Err("write_register_u32 is damiao-only".to_string()),
        None => Err("motor not connected".to_string()),
    }
}

fn handle_write_register_f32(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    let rid = as_u16(v, "rid", 0) as u8;
    let value = v.get("value").and_then(Value::as_f64).unwrap_or(0.0) as f32;
    let verify = as_bool(v, "verify", true);
    let verify_timeout_ms = as_u64(v, "verify_timeout_ms", as_u64(v, "timeout_ms", 1000));
    let verify_attempts = as_u64(v, "verify_attempts", 2).clamp(1, 5);
    match ctx.motor.as_ref() {
        Some(MotorHandle::Damiao(m)) => {
            m.write_register_f32(rid, value)
                .map_err(|e| e.to_string())?;
            if !verify {
                return Ok(json!({"rid": rid, "value": value, "verified": false}));
            }
            match verify_register_f32(m, rid, value, verify_timeout_ms, verify_attempts) {
                Ok(readback) => Ok(json!({
                    "rid": rid,
                    "value": value,
                    "readback": readback,
                    "verified": true
                })),
                Err(VerifyError::Timeout(warning)) => Ok(json!({
                    "rid": rid,
                    "value": value,
                    "verified": false,
                    "warning": warning,
                    "warnings": [warning]
                })),
                Err(VerifyError::Mismatch(err)) => Err(err),
            }
        }
        Some(MotorHandle::Robstride(_)) => {
            Err("write_register_f32 is damiao-only; use robstride_write_param".to_string())
        }
        Some(_) => Err("write_register_f32 is damiao-only".to_string()),
        None => Err("motor not connected".to_string()),
    }
}

enum VerifyError {
    Timeout(String),
    Mismatch(String),
}

fn verify_register_u32(
    motor: &motor_vendor_damiao::DamiaoMotor,
    rid: u8,
    expected: u32,
    timeout_ms: u64,
    attempts: u64,
) -> Result<u32, VerifyError> {
    let mut last_timeout = None;
    for attempt in 0..attempts {
        match motor.get_register_u32(rid, Duration::from_millis(timeout_ms.max(1))) {
            Ok(readback) if readback == expected => return Ok(readback),
            Ok(readback) => {
                return Err(VerifyError::Mismatch(format!(
                    "register {rid} verify failed: expected {expected}, got {readback}"
                )));
            }
            Err(motor_core::error::MotorError::Timeout(err)) => {
                last_timeout = Some(err);
            }
            Err(err) => return Err(VerifyError::Mismatch(err.to_string())),
        }
        if attempt + 1 < attempts {
            std::thread::sleep(Duration::from_millis(20));
        }
    }
    Err(VerifyError::Timeout(format!(
        "register {rid} write sent but readback was not received after {attempts} attempt(s); value may still have been written{}",
        last_timeout
            .map(|err| format!(": {err}"))
            .unwrap_or_default()
    )))
}

fn verify_register_f32(
    motor: &motor_vendor_damiao::DamiaoMotor,
    rid: u8,
    expected: f32,
    timeout_ms: u64,
    attempts: u64,
) -> Result<f32, VerifyError> {
    let mut last_timeout = None;
    for attempt in 0..attempts {
        match motor.get_register_f32(rid, Duration::from_millis(timeout_ms.max(1))) {
            Ok(readback) if f32_close(readback, expected) => return Ok(readback),
            Ok(readback) => {
                return Err(VerifyError::Mismatch(format!(
                    "register {rid} verify failed: expected {expected}, got {readback}"
                )));
            }
            Err(motor_core::error::MotorError::Timeout(err)) => {
                last_timeout = Some(err);
            }
            Err(err) => return Err(VerifyError::Mismatch(err.to_string())),
        }
        if attempt + 1 < attempts {
            std::thread::sleep(Duration::from_millis(20));
        }
    }
    Err(VerifyError::Timeout(format!(
        "register {rid} write sent but readback was not received after {attempts} attempt(s); value may still have been written{}",
        last_timeout
            .map(|err| format!(": {err}"))
            .unwrap_or_default()
    )))
}

fn f32_close(a: f32, b: f32) -> bool {
    let scale = a.abs().max(b.abs()).max(1.0);
    (a - b).abs() <= scale * 1e-4
}

fn handle_get_register_u32(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    let rid = as_u16(v, "rid", 0) as u8;
    let timeout_ms = as_u64(v, "timeout_ms", 1000);
    match ctx.motor.as_ref() {
        Some(MotorHandle::Damiao(m)) => {
            let val = m
                .get_register_u32(rid, Duration::from_millis(timeout_ms))
                .map_err(|e| e.to_string())?;
            Ok(json!({"rid": rid, "value": val}))
        }
        Some(MotorHandle::Robstride(_)) => {
            Err("get_register_u32 is damiao-only; use robstride_read_param".to_string())
        }
        Some(MotorHandle::Fashionstar(m)) => {
            let rid = parse_fashionstar_u64(v, "rid", 0, 255)? as u8;
            let value = m
                .get_register_u32(
                    rid,
                    Duration::from_millis(as_u64(v, "timeout_ms", 500).min(60_000)),
                )
                .map_err(|e| e.to_string())?;
            Ok(json!({"rid":rid,"value":value}))
        }
        Some(_) => Err("get_register_u32 is damiao-only".to_string()),
        None => Err("motor not connected".to_string()),
    }
}

fn handle_get_register_f32(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    let rid = as_u16(v, "rid", 0) as u8;
    let timeout_ms = as_u64(v, "timeout_ms", 1000);
    match ctx.motor.as_ref() {
        Some(MotorHandle::Damiao(m)) => {
            let val = m
                .get_register_f32(rid, Duration::from_millis(timeout_ms))
                .map_err(|e| e.to_string())?;
            Ok(json!({"rid": rid, "value": val}))
        }
        Some(MotorHandle::Robstride(_)) => {
            Err("get_register_f32 is damiao-only; use robstride_read_param".to_string())
        }
        Some(_) => Err("get_register_f32 is damiao-only".to_string()),
        None => Err("motor not connected".to_string()),
    }
}

fn handle_robstride_ping(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    match ctx.motor.as_ref() {
        Some(MotorHandle::Robstride(m)) => {
            let p = m
                .ping(Duration::from_millis(as_u64(v, "timeout_ms", 200)))
                .map_err(|e| e.to_string())?;
            Ok(json!({"device_id": p.device_id, "responder_id": p.responder_id}))
        }
        Some(MotorHandle::Damiao(_)) => Err("robstride_ping requires vendor=robstride".to_string()),
        Some(_) => Err("robstride_ping requires vendor=robstride".to_string()),
        None => Err("motor not connected".to_string()),
    }
}

fn handle_robstride_read_param_op(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    match ctx.motor.as_ref() {
        Some(MotorHandle::Robstride(m)) => handle_robstride_read_param(m, v),
        Some(MotorHandle::Damiao(_)) => {
            Err("robstride_read_param requires vendor=robstride".to_string())
        }
        Some(_) => Err("robstride_read_param requires vendor=robstride".to_string()),
        None => Err("motor not connected".to_string()),
    }
}

fn handle_robstride_write_param_op(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    match ctx.motor.as_ref() {
        Some(MotorHandle::Robstride(m)) => handle_robstride_write_param(m, v),
        Some(MotorHandle::Damiao(_)) => {
            Err("robstride_write_param requires vendor=robstride".to_string())
        }
        Some(_) => Err("robstride_write_param requires vendor=robstride".to_string()),
        None => Err("motor not connected".to_string()),
    }
}
