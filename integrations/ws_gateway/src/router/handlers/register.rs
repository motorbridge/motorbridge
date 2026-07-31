use crate::commands::{
    as_bool, as_u16, as_u64, handle_robstride_read_param, handle_robstride_write_param,
    parse_damiao_mode, parse_robstride_mode,
};
use crate::model::{ControllerHandle, MotorHandle};
use crate::session::SessionCtx;
use crate::vendors::damiao_ws::ensure_control_mode_soft;
use crate::vendors::hightorque_ws::send_hightorque_ext;
use motor_vendor_robstride::{
    ParameterId as RobstrideParameterId, ParameterValue as RobstrideParameterValue,
};
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
        Some(MotorHandle::Hightorque(_)) => {
            return Err("clear_error is not supported for hightorque".to_string())
        }
        Some(MotorHandle::Myactuator(_)) => {
            return Err("clear_error is not supported for myactuator".to_string())
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
        Some(MotorHandle::Hightorque(_)) => {
            return Err("set_zero_position is not supported for hightorque".to_string())
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
        (Some(ControllerHandle::Hightorque(bus)), Some(MotorHandle::Hightorque(mid))) => {
            send_hightorque_ext(bus.as_ref(), *mid, &[0x17, 0x01, 0, 0, 0, 0, 0, 0])?;
        }
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
        Some(MotorHandle::Hightorque(_)) => {
            return Err("store_parameters is not supported for hightorque".to_string())
        }
        Some(MotorHandle::Myactuator(_)) => {
            return Err("store_parameters is not supported for myactuator".to_string())
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
    let run_mode_changed = as_u16(v, "param_id", 0x700A) == RobstrideParameterId::Mode as u16;
    let result = match ctx.motor.as_ref() {
        Some(MotorHandle::Robstride(m)) => handle_robstride_write_param(m, v),
        Some(MotorHandle::Damiao(_)) => {
            Err("robstride_write_param requires vendor=robstride".to_string())
        }
        Some(_) => Err("robstride_write_param requires vendor=robstride".to_string()),
        None => Err("motor not connected".to_string()),
    };
    if result.is_ok() && run_mode_changed {
        // Raw run_mode writes bypass the typed control handlers, so any MIT
        // gains remembered by this gateway session can no longer be trusted.
        ctx.robstride_mit_gains = None;
    }
    result
}
