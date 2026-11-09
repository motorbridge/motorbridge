use crate::commands::{as_u64, parse_transport_in_msg};
use crate::model::{Target, Vendor};
use crate::vendors::transport_ws::open_fashionstar_controller;
use motor_core::error::MotorError;
use motor_vendor_fashionstar::{FashionstarController, CAN_ID};
use serde_json::{json, Value};
use std::time::Duration;

pub(crate) fn parse_fashionstar_u64(
    v: &Value,
    key: &str,
    default: u64,
    max: u64,
) -> Result<u64, String> {
    let value = match v.get(key) {
        None => default,
        Some(Value::String(text)) => {
            if let Some(hex) = text.strip_prefix("0x") {
                u64::from_str_radix(hex, 16).map_err(|_| format!("invalid {key}"))?
            } else {
                text.parse().map_err(|_| format!("invalid {key}"))?
            }
        }
        Some(value) => value
            .as_u64()
            .ok_or_else(|| format!("{key} must be an unsigned integer"))?,
    };
    if value > max {
        return Err(format!("{key} exceeds {max}"));
    }
    Ok(value)
}

pub(crate) fn parse_fashionstar_target(v: &Value, base: &Target) -> Result<Target, String> {
    let mut target = parse_fashionstar_connection_target(v, base)?;
    target.motor_id = parse_fashionstar_u64(v, "motor_id", u64::from(base.motor_id), 254)? as u16;
    Ok(target)
}

fn parse_fashionstar_connection_target(v: &Value, base: &Target) -> Result<Target, String> {
    let mut target = base.clone();
    target.vendor = Vendor::Fashionstar;
    target.model = v
        .get("model")
        .and_then(Value::as_str)
        .unwrap_or(&base.model)
        .to_string();
    if target.model == "4340" || target.model == "4340P" {
        target.model = "fashionstar".to_string();
    }
    target.transport = parse_transport_in_msg(v, base.transport)?;
    target.channel = v
        .get("channel")
        .and_then(Value::as_str)
        .unwrap_or(&base.channel)
        .to_string();
    target.serial_port = v
        .get("serial_port")
        .and_then(Value::as_str)
        .unwrap_or(&base.serial_port)
        .to_string();
    target.serial_baud = parse_fashionstar_u64(
        v,
        "serial_baud",
        u64::from(base.serial_baud),
        u32::MAX as u64,
    )? as u32;
    target.motor_id = 0;
    target.feedback_id =
        parse_fashionstar_u64(v, "feedback_id", u64::from(CAN_ID), u16::MAX as u64)? as u16;
    if target.feedback_id != CAN_ID {
        return Err("FashionStar feedback_id is fixed at 0x321".to_string());
    }
    Ok(target)
}

fn with_controller<F>(target: &Target, run: F) -> Result<Value, String>
where
    F: FnOnce(&FashionstarController) -> Result<Value, String>,
{
    let ctrl = open_fashionstar_controller(target, target.transport)?;
    let result = run(&ctrl);
    let close = ctrl.close_bus().map_err(|e| e.to_string());
    result.and_then(|value| close.map(|_| value))
}

pub(crate) fn cmd_scan_fashionstar(v: &Value, base: &Target) -> Result<Value, String> {
    let target = parse_fashionstar_connection_target(v, base)?;
    let start_id = parse_fashionstar_u64(v, "start_id", 0, 254)? as u16;
    let end_id = parse_fashionstar_u64(v, "end_id", 16, 254)? as u16;
    if start_id > end_id {
        return Err("end_id must be >= start_id".to_string());
    }
    let timeout = Duration::from_millis(as_u64(v, "timeout_ms", 80).min(60_000));
    with_controller(&target, |ctrl| {
        let mut hits = Vec::new();
        for id in start_id..=end_id {
            let motor = ctrl
                .add_motor(id, CAN_ID, &target.model)
                .map_err(|e| e.to_string())?;
            match motor.get_register_u32(1, timeout) {
                Ok(model_code) => hits.push(json!({
                    "vendor": "fashionstar",
                    "probe": id,
                    "motor_id": id,
                    "feedback_id": CAN_ID,
                    "model": target.model,
                    "model_code": model_code
                })),
                Err(MotorError::Timeout(_)) => {}
                Err(e) => return Err(e.to_string()),
            }
        }
        Ok(json!({"vendor": "fashionstar", "count": hits.len(), "hits": hits}))
    })
}

pub(crate) fn cmd_verify_fashionstar(v: &Value, base: &Target) -> Result<Value, String> {
    let target = parse_fashionstar_target(v, base)?;
    with_controller(&target, |ctrl| {
        let motor = ctrl
            .add_motor(target.motor_id, CAN_ID, &target.model)
            .map_err(|e| e.to_string())?;
        let model_code = motor
            .get_register_u32(
                1,
                Duration::from_millis(as_u64(v, "timeout_ms", 500).min(60_000)),
            )
            .map_err(|e| e.to_string())?;
        Ok(json!({
            "vendor": "fashionstar",
            "verified": true,
            "motor_id": target.motor_id,
            "feedback_id": CAN_ID,
            "model_code": model_code
        }))
    })
}

pub(crate) fn cmd_set_id_fashionstar(v: &Value, base: &Target) -> Result<Value, String> {
    let mut target = parse_fashionstar_connection_target(v, base)?;
    target.motor_id = if v.get("old_motor_id").is_some() {
        parse_fashionstar_u64(v, "old_motor_id", 0, 254)?
    } else {
        parse_fashionstar_u64(v, "motor_id", u64::from(base.motor_id), 254)?
    } as u16;
    let new_motor_id =
        parse_fashionstar_u64(v, "new_motor_id", u64::from(target.motor_id), 254)? as u32;
    for field in ["old_feedback_id", "new_feedback_id"] {
        if parse_fashionstar_u64(v, field, u64::from(CAN_ID), u16::MAX as u64)? != u64::from(CAN_ID)
        {
            return Err("FashionStar CAN arbitration ID is fixed at 0x321".to_string());
        }
    }
    with_controller(&target, |ctrl| {
        let motor = ctrl
            .add_motor(target.motor_id, CAN_ID, &target.model)
            .map_err(|e| e.to_string())?;
        motor
            .write_register_u32(0x81, new_motor_id)
            .map_err(|e| e.to_string())?;
        Ok(json!({
            "vendor": "fashionstar",
            "old_motor_id": target.motor_id,
            "new_motor_id": new_motor_id,
            "feedback_id": CAN_ID,
            "verified": false,
            "requires_power_cycle": true
        }))
    })
}
