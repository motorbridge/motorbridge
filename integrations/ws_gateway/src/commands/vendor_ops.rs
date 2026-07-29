use crate::model::{Target, Vendor};
use serde_json::{json, Value};
use std::time::Duration;

use super::{
    as_bool, as_u16, as_u64, build_scan_model_hints, parse_transport_in_msg, parse_vendor_in_msg,
};
use crate::vendors::hightorque_ws::{send_hightorque_ext, wait_hightorque_status_for_motor};
use crate::vendors::transport_ws::{
    myactuator_feedback_default, open_damiao_controller, open_hexfellow_controller,
    open_hightorque_bus, open_myactuator_controller, open_robstride_controller,
};

fn robstride_device_id(id: u16, name: &str) -> Result<u8, String> {
    if (1..=255).contains(&id) {
        Ok(id as u8)
    } else {
        Err(format!("robstride {name} must be 1..255, got {id}"))
    }
}

fn robstride_host_id(id: u16, name: &str) -> Result<u16, String> {
    if id <= 255 {
        Ok(id)
    } else {
        Err(format!("robstride {name}/host_id must be 0..255, got {id}"))
    }
}

pub(crate) fn cmd_verify(v: &Value, base: &Target) -> Result<Value, String> {
    let vendor = parse_vendor_in_msg(v, base.vendor)?;
    let transport = parse_transport_in_msg(v, base.transport)?;
    let mid = as_u16(v, "motor_id", base.motor_id);
    let fid = as_u16(v, "feedback_id", base.feedback_id);
    let timeout_ms = as_u64(v, "timeout_ms", 1000);

    match vendor {
        Vendor::Damiao => {
            let preferred_model = v
                .get("model")
                .and_then(Value::as_str)
                .unwrap_or(&base.model);
            let model_hints = build_scan_model_hints(preferred_model);
            let mut last_err: Option<String> = None;
            let mut out: Option<Value> = None;
            for model in model_hints {
                let ctrl = match open_damiao_controller(base, transport) {
                    Ok(c) => c,
                    Err(e) => {
                        last_err = Some(e.to_string());
                        continue;
                    }
                };
                let motor = match ctrl.add_motor(mid, fid, &model) {
                    Ok(m) => m,
                    Err(e) => {
                        last_err = Some(e.to_string());
                        let _ = ctrl.close_bus();
                        continue;
                    }
                };
                let esc = match motor.get_register_u32(8, Duration::from_millis(timeout_ms)) {
                    Ok(v) => v,
                    Err(e) => {
                        last_err = Some(e.to_string());
                        let _ = ctrl.close_bus();
                        continue;
                    }
                };
                let mst = match motor.get_register_u32(7, Duration::from_millis(timeout_ms)) {
                    Ok(v) => v,
                    Err(e) => {
                        last_err = Some(e.to_string());
                        let _ = ctrl.close_bus();
                        continue;
                    }
                };
                let _ = ctrl.close_bus();
                out = Some(json!({
                    "vendor": "damiao",
                    "transport": transport.as_str(),
                    "model_used": model,
                    "motor_id": mid,
                    "feedback_id": fid,
                    "esc_id": esc,
                    "mst_id": mst,
                    "ok": esc == mid as u32 && mst == fid as u32,
                }));
                break;
            }
            match out {
                Some(v) => Ok(v),
                None => Err(last_err.unwrap_or_else(|| "damiao verify failed".to_string())),
            }
        }
        Vendor::Robstride => {
            let mid_u8 = robstride_device_id(mid, "motor_id")?;
            robstride_host_id(fid, "feedback_id")?;
            let ctrl = open_robstride_controller(base, transport)?;
            let motor = ctrl
                .add_motor(mid, fid, &base.model)
                .map_err(|e| e.to_string())?;
            let ping = motor
                .ping(Duration::from_millis(timeout_ms))
                .map_err(|e| e.to_string())?;
            let _ = ctrl.close_bus();
            Ok(json!({
                "vendor": "robstride",
                "transport": transport.as_str(),
                "motor_id": mid,
                "feedback_id": fid,
                "device_id": ping.device_id,
                "responder_id": ping.responder_id,
                "ok": ping.device_id == mid_u8,
            }))
        }
        Vendor::Hexfellow => {
            let ctrl = open_hexfellow_controller(base, transport)?;
            let motor = ctrl
                .add_motor(mid, as_u16(v, "feedback_id", base.feedback_id), &base.model)
                .map_err(|e| e.to_string())?;
            let status = motor
                .query_status(Duration::from_millis(timeout_ms))
                .map_err(|e| e.to_string())?;
            let _ = ctrl.close_bus();
            Ok(json!({
                "vendor": "hexfellow",
                "transport": transport.as_str(),
                "motor_id": mid,
                "statusword": status.statusword,
                "mode_display": status.mode_display,
                "ok": true,
            }))
        }
        Vendor::Myactuator => {
            let ctrl = open_myactuator_controller(base, transport)?;
            let fid = as_u16(v, "feedback_id", base.feedback_id);
            let eff_fid = if fid == 0 {
                myactuator_feedback_default(mid)
            } else {
                fid
            };
            let motor = ctrl
                .add_motor(mid, eff_fid, &base.model)
                .map_err(|e| e.to_string())?;
            motor.request_version_date().map_err(|e| e.to_string())?;
            let version = motor
                .await_version_date(Duration::from_millis(timeout_ms))
                .map_err(|e| e.to_string())?;
            let _ = ctrl.close_bus();
            Ok(json!({
                "vendor": "myactuator",
                "transport": transport.as_str(),
                "motor_id": mid,
                "feedback_id": eff_fid,
                "version": version,
                "ok": true,
            }))
        }
        Vendor::Hightorque => {
            let bus = open_hightorque_bus(base, transport)?;
            send_hightorque_ext(bus.as_ref(), mid, &[0x17, 0x01, 0, 0, 0, 0, 0, 0])?;
            let status = wait_hightorque_status_for_motor(
                bus.as_ref(),
                mid,
                Duration::from_millis(timeout_ms),
            )?;
            let _ = bus.shutdown();
            Ok(json!({
                "vendor": "hightorque",
                "transport": transport.as_str(),
                "motor_id": mid,
                "ok": status.is_some(),
                "state": status.map(|s| json!({"pos_raw": s.pos_raw, "vel_raw": s.vel_raw, "tqe_raw": s.tqe_raw})),
            }))
        }
        Vendor::CyberBeast => {
            Err("verify is not supported for cyberbeast".to_string())
        }
    }
}

pub(crate) fn cmd_set_id(v: &Value, base: &Target) -> Result<Value, String> {
    let vendor = parse_vendor_in_msg(v, base.vendor)?;
    let transport = parse_transport_in_msg(v, base.transport)?;
    match vendor {
        Vendor::Damiao => {
            let old_mid = as_u16(v, "old_motor_id", base.motor_id);
            let old_fid = as_u16(v, "old_feedback_id", base.feedback_id);
            let new_mid = as_u16(v, "new_motor_id", old_mid);
            let new_fid = as_u16(v, "new_feedback_id", old_fid);
            let store = as_bool(v, "store", true);
            let verify = as_bool(v, "verify", true);
            let timeout_ms = as_u64(v, "timeout_ms", 1000);

            let ctrl = open_damiao_controller(base, transport)?;
            let preferred_model = v
                .get("model")
                .and_then(Value::as_str)
                .unwrap_or(&base.model);
            let model_hints = build_scan_model_hints(preferred_model);
            let mut model_used: Option<String> = None;
            let mut last_err: Option<String> = None;
            let mut motor_opt = None;
            for model in model_hints {
                match ctrl.add_motor(old_mid, old_fid, &model) {
                    Ok(m) => {
                        model_used = Some(model);
                        motor_opt = Some(m);
                        break;
                    }
                    Err(e) => {
                        last_err = Some(e.to_string());
                    }
                }
            }
            let motor = motor_opt.ok_or_else(|| {
                last_err.unwrap_or_else(|| "failed to add damiao motor for set_id".to_string())
            })?;

            if new_fid != old_fid {
                motor
                    .write_register_u32(7, new_fid as u32)
                    .map_err(|e| e.to_string())?;
            }
            if new_mid != old_mid {
                motor
                    .write_register_u32(8, new_mid as u32)
                    .map_err(|e| e.to_string())?;
            }
            if store {
                motor.store_parameters().map_err(|e| e.to_string())?;
            }
            let _ = ctrl.close_bus();

            let mut out = json!({
                "vendor": "damiao",
                "transport": transport.as_str(),
                "old_motor_id": old_mid,
                "old_feedback_id": old_fid,
                "new_motor_id": new_mid,
                "new_feedback_id": new_fid,
                "model_used": model_used,
                "store": store,
            });
            if verify {
                std::thread::sleep(Duration::from_millis(120));
                let verify_result = cmd_verify(
                    &json!({
                        "vendor":"damiao",
                        "transport": transport.as_str(),
                        "motor_id": new_mid,
                        "feedback_id": new_fid,
                        "timeout_ms": timeout_ms
                    }),
                    base,
                )?;
                out["verify"] = verify_result;
            }
            Ok(out)
        }
        Vendor::Robstride => {
            let old_mid = as_u16(v, "old_motor_id", base.motor_id);
            let fid = as_u16(v, "feedback_id", base.feedback_id);
            let new_mid = as_u16(v, "new_motor_id", old_mid);
            robstride_device_id(old_mid, "old_motor_id")?;
            robstride_host_id(fid, "feedback_id")?;
            let new_mid_u8 = robstride_device_id(new_mid, "new_motor_id")?;
            let verify = as_bool(v, "verify", true);
            let timeout_ms = as_u64(v, "timeout_ms", 1000);
            let ctrl = open_robstride_controller(base, transport)?;
            let motor = ctrl
                .add_motor(old_mid, fid, &base.model)
                .map_err(|e| e.to_string())?;
            motor.set_device_id(new_mid_u8).map_err(|e| e.to_string())?;
            let _ = ctrl.close_bus();

            let mut out = json!({
                "vendor": "robstride",
                "transport": transport.as_str(),
                "old_motor_id": old_mid,
                "new_motor_id": new_mid,
                "feedback_id": fid
            });
            if verify {
                std::thread::sleep(Duration::from_millis(120));
                let verify_result = cmd_verify(
                    &json!({
                        "vendor":"robstride",
                        "transport": transport.as_str(),
                        "motor_id": new_mid,
                        "feedback_id": fid,
                        "timeout_ms": timeout_ms
                    }),
                    base,
                )?;
                out["verify"] = verify_result;
            }
            Ok(out)
        }
        Vendor::Hexfellow
        | Vendor::Myactuator
        | Vendor::Hightorque
        | Vendor::CyberBeast => {
            Err(format!("set_id is not supported for {}", vendor.as_str()))
        }
    }
}
