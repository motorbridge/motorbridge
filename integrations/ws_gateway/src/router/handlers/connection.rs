use crate::commands::{as_bool, as_u16, as_u64, parse_transport_in_msg, parse_vendor_in_msg};
use crate::model::{ControllerHandle, MotorHandle, Vendor};
use crate::router::stream::ParamStream;
use crate::session::SessionCtx;
use motor_core::MotorDevice;
use serde_json::{json, Value};

pub(crate) fn handle(
    op: &str,
    v: &Value,
    ctx: &mut SessionCtx,
    state_stream_enabled: &mut bool,
    param_stream: &mut ParamStream,
    dt_ms: u64,
) -> Option<Result<Value, String>> {
    match op {
        "capabilities" => Some(handle_capabilities(ctx)),
        "ping" => Some(handle_ping(v, ctx)),
        "set_target" => Some(handle_set_target(v, ctx)),
        "enable" => Some(handle_enable(v, ctx)),
        "disable" => Some(handle_disable(v, ctx)),
        "stop" => Some(handle_stop(ctx)),
        "state_once" => Some(handle_state_once(ctx)),
        "damiao_state_many" => Some(handle_damiao_state_many(v, ctx)),
        "state_stream" => Some(handle_state_stream(v, state_stream_enabled)),
        "param_stream" => Some(handle_param_stream(v, ctx, param_stream, dt_ms, None)),
        "robstride_param_stream" => Some(handle_param_stream(
            v,
            ctx,
            param_stream,
            dt_ms,
            Some(Vendor::Robstride),
        )),
        "damiao_param_stream" => Some(handle_param_stream(
            v,
            ctx,
            param_stream,
            dt_ms,
            Some(Vendor::Damiao),
        )),
        "status" => Some(handle_status(ctx)),
        "poll_feedback_once" => Some(handle_poll_feedback_once(ctx)),
        "shutdown" => Some(handle_shutdown(ctx)),
        "close_bus" => Some(handle_close_bus(ctx)),
        _ => None,
    }
}

fn handle_capabilities(ctx: &SessionCtx) -> Result<Value, String> {
    Ok(json!({
        "api_version": "v1",
        "gateway_version": env!("CARGO_PKG_VERSION"),
        "default_vendor": ctx.target.vendor.as_str(),
        "default_target": {
            "vendor": ctx.target.vendor.as_str(),
            "transport": ctx.target.transport.as_str(),
            "channel": ctx.target.channel,
            "dm_device_type": ctx.target.dm_device_type,
            "dm_channel": ctx.target.dm_channel,
            "model": ctx.target.model,
            "motor_id": ctx.target.motor_id,
            "feedback_id": ctx.target.feedback_id,
        },
        "features": [
            "dynamic_target",
            "batch_scan",
            "state_stream",
            "damiao_state_many",
            "param_stream",
            "robstride_exact_host_scan"
        ],
        "vendors": {
            "damiao": {
                "transports": ["auto", "socketcan", "socketcanfd", "dm-serial", "dm-device"],
                "modes": ["mit", "pos_vel", "vel", "force_pos"],
                "ops_unified": ["scan", "set_id", "enable", "disable", "stop", "state_once", "status", "verify"],
                "ops_vendor_native": ["write_register_u32", "write_register_f32", "get_register_u32", "get_register_f32", "damiao_state_many"]
            },
            "robstride": {
                "transports": ["auto", "socketcan", "socketcanfd"],
                "modes": ["mit", "pos_vel", "vel"],
                "ops_unified": ["scan", "set_id", "enable", "disable", "stop", "state_once", "status", "verify"],
                "ops_vendor_native": ["robstride_ping", "robstride_read_param", "robstride_write_param", "set_active_report"]
            },
            "hexfellow": {
                "transports": ["auto", "socketcanfd"],
                "modes": ["mit", "pos_vel"],
                "ops_unified": ["scan", "enable", "disable", "stop", "state_once", "status", "verify"],
                "ops_vendor_native": []
            },
            "myactuator": {
                "transports": ["auto", "socketcan", "socketcanfd"],
                "modes": ["pos_vel", "vel"],
                "ops_unified": ["scan", "enable", "disable", "stop", "state_once", "status", "verify"],
                "ops_vendor_native": ["status", "version", "mode-query"]
            },
            "hightorque": {
                "transports": ["auto", "socketcan"],
                "modes": ["mit", "pos_vel", "vel", "force_pos"],
                "ops_unified": ["scan", "stop", "state_once", "status", "verify"],
                "ops_vendor_native": ["read"]
            }
        },
        "unsupported_behavior": "return {ok:false,error:'unsupported ...'}"
    }))
}

fn handle_ping(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    match ctx.target.vendor {
        Vendor::Robstride => {
            ctx.ensure_connected()?;
            if let Some(MotorHandle::Robstride(m)) = ctx.motor.as_ref() {
                let p = m
                    .ping(std::time::Duration::from_millis(as_u64(
                        v,
                        "timeout_ms",
                        200,
                    )))
                    .map_err(|e| e.to_string())?;
                Ok(
                    json!({"pong":true,"vendor":"robstride","device_id":p.device_id,"responder_id":p.responder_id}),
                )
            } else {
                Err("motor not connected".to_string())
            }
        }
        Vendor::Damiao => Ok(json!({"pong": true, "vendor":"damiao"})),
        Vendor::Hexfellow => Ok(json!({"pong": true, "vendor":"hexfellow"})),
        Vendor::Myactuator => Ok(json!({"pong": true, "vendor":"myactuator"})),
        Vendor::Hightorque => Ok(json!({"pong": true, "vendor":"hightorque"})),
        Vendor::CyberBeast => Ok(json!({"pong": true, "vendor":"cyberbeast"})),
    }
}

fn handle_set_target(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    let mut next = ctx.target.clone();
    next.vendor = parse_vendor_in_msg(v, next.vendor)?;
    next.transport = parse_transport_in_msg(v, next.transport)?;
    next.channel = v
        .get("channel")
        .and_then(Value::as_str)
        .unwrap_or(&next.channel)
        .to_string();
    next.serial_port = v
        .get("serial_port")
        .and_then(Value::as_str)
        .unwrap_or(&next.serial_port)
        .to_string();
    next.serial_baud = as_u64(v, "serial_baud", next.serial_baud as u64) as u32;
    next.dm_device_type = v
        .get("dm_device_type")
        .or_else(|| v.get("dm-device-type"))
        .and_then(Value::as_str)
        .unwrap_or(&next.dm_device_type)
        .to_string();
    next.dm_channel = v
        .get("dm_channel")
        .or_else(|| v.get("dm-channel"))
        .and_then(Value::as_str)
        .unwrap_or(&next.dm_channel)
        .to_string();
    next.model = v
        .get("model")
        .and_then(Value::as_str)
        .unwrap_or(&next.model)
        .to_string();
    next.motor_id = as_u16(v, "motor_id", next.motor_id);
    next.feedback_id = as_u16(v, "feedback_id", next.feedback_id);

    if next.vendor == Vendor::Robstride {
        if next.model == "4340" || next.model == "4340P" {
            next.model = "rs-00".to_string();
        }
        if next.feedback_id == 0x11 {
            next.feedback_id = 0xFD;
        }
    } else if next.vendor == Vendor::Myactuator {
        if next.model == "4340" || next.model == "4340P" {
            next.model = "X8".to_string();
        }
        if next.feedback_id == 0x11 {
            next.feedback_id = 0x241;
        }
    } else if next.vendor == Vendor::Hexfellow {
        if next.model == "4340" || next.model == "4340P" {
            next.model = "hexfellow".to_string();
        }
        if next.feedback_id == 0x11 {
            next.feedback_id = 0;
        }
    } else if next.vendor == Vendor::Hightorque {
        if next.model == "4340" || next.model == "4340P" {
            next.model = "hightorque".to_string();
        }
        if next.feedback_id == 0x11 {
            next.feedback_id = 0x01;
        }
    }

    ctx.disconnect(false);
    ctx.target = next;
    ctx.active = None;
    Ok(json!({
        "vendor": ctx.target.vendor.as_str(),
        "transport": ctx.target.transport.as_str(),
        "channel": ctx.target.channel,
        "serial_port": ctx.target.serial_port,
        "serial_baud": ctx.target.serial_baud,
        "dm_device_type": ctx.target.dm_device_type,
        "dm_channel": ctx.target.dm_channel,
        "model": ctx.target.model,
        "motor_id": ctx.target.motor_id,
        "feedback_id": ctx.target.feedback_id,
    }))
}

fn handle_enable(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
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
        Some(MotorHandle::Damiao(m)) => m.enable().map_err(|e| e.to_string())?,
        Some(MotorHandle::Hexfellow(m)) => m.enable().map_err(|e| e.to_string())?,
        Some(MotorHandle::Hightorque(_)) => {}
        Some(MotorHandle::Myactuator(m)) => m.enable().map_err(|e| e.to_string())?,
        Some(MotorHandle::Robstride(m)) => m.enable().map_err(|e| e.to_string())?,
        Some(MotorHandle::CyberBeast(m)) => m.enable().map_err(|e| e.to_string())?,
        None => return Err("motor not connected".to_string()),
    }
    ctx.active = None;
    Ok(json!({"enabled": true}))
}

fn handle_disable(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
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
        Some(MotorHandle::Damiao(m)) => m.disable().map_err(|e| e.to_string())?,
        Some(MotorHandle::Hexfellow(m)) => m.disable().map_err(|e| e.to_string())?,
        Some(MotorHandle::Hightorque(_)) => {}
        Some(MotorHandle::Myactuator(m)) => m.disable().map_err(|e| e.to_string())?,
        Some(MotorHandle::Robstride(m)) => m.disable().map_err(|e| e.to_string())?,
        Some(MotorHandle::CyberBeast(m)) => m.disable().map_err(|e| e.to_string())?,
        None => return Err("motor not connected".to_string()),
    }
    ctx.active = None;
    Ok(json!({"disabled": true}))
}

fn handle_stop(ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.active = None;
    if let Some(m) = ctx.motor.as_ref() {
        match m {
            MotorHandle::Damiao(mm) => mm.send_cmd_vel(0.0).map_err(|e| e.to_string())?,
            MotorHandle::Hexfellow(mm) => mm
                .command_mit(
                    motor_vendor_hexfellow::MitTarget {
                        position_rev: 0.0,
                        velocity_rev_s: 0.0,
                        torque_nm: 0.0,
                        kp: 0,
                        kd: 0,
                        limit_permille: 1000,
                    },
                    std::time::Duration::from_millis(200),
                )
                .map_err(|e| e.to_string())?,
            MotorHandle::Hightorque(mid) => {
                if let Some(ControllerHandle::Hightorque(bus)) = ctx.controller.as_ref() {
                    crate::vendors::hightorque_ws::send_hightorque_ext(
                        bus.as_ref(),
                        *mid,
                        &[0x01, 0x00, 0x00],
                    )?;
                }
            }
            MotorHandle::Myactuator(mm) => mm.stop_motor().map_err(|e| e.to_string())?,
            MotorHandle::Robstride(mm) => mm.set_velocity_target(0.0).map_err(|e| e.to_string())?,
            MotorHandle::CyberBeast(mm) => mm.send_stop_motor().map_err(|e| e.to_string())?,
        }
    }
    Ok(json!({"stopped": true}))
}

fn handle_state_once(ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    Ok(json!({"state": ctx.build_state_snapshot()?}))
}

fn value_as_u16(v: &Value, key: &str) -> Option<u16> {
    v.get(key).and_then(|x| {
        x.as_u64().and_then(|n| u16::try_from(n).ok()).or_else(|| {
            x.as_str().and_then(|s| {
                let text = s.trim();
                if text.starts_with("0x") || text.starts_with("0X") {
                    u16::from_str_radix(&text[2..], 16).ok()
                } else {
                    text.parse::<u16>().ok()
                }
            })
        })
    })
}

fn damiao_default_feedback_id(motor_id: u16) -> u16 {
    0x10u16.saturating_add(motor_id & 0x0F)
}

fn handle_damiao_state_many(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    let vendor = parse_vendor_in_msg(v, ctx.target.vendor)?;
    if vendor != Vendor::Damiao {
        return Err("damiao_state_many requires vendor=damiao".to_string());
    }
    if ctx.target.vendor != Vendor::Damiao {
        ctx.disconnect(false);
        ctx.target.vendor = Vendor::Damiao;
        ctx.motor = None;
    }

    ctx.target.transport = parse_transport_in_msg(v, ctx.target.transport)?;
    ctx.target.channel = v
        .get("channel")
        .and_then(Value::as_str)
        .unwrap_or(&ctx.target.channel)
        .to_string();
    ctx.target.serial_port = v
        .get("serial_port")
        .and_then(Value::as_str)
        .unwrap_or(&ctx.target.serial_port)
        .to_string();
    ctx.target.serial_baud = as_u64(v, "serial_baud", ctx.target.serial_baud as u64) as u32;
    ctx.target.dm_device_type = v
        .get("dm_device_type")
        .or_else(|| v.get("dm-device-type"))
        .and_then(Value::as_str)
        .unwrap_or(&ctx.target.dm_device_type)
        .to_string();
    ctx.target.dm_channel = v
        .get("dm_channel")
        .or_else(|| v.get("dm-channel"))
        .and_then(Value::as_str)
        .unwrap_or(&ctx.target.dm_channel)
        .to_string();

    let timeout_ms = as_u64(v, "timeout_ms", 120).max(1);
    ctx.ensure_connected()?;
    let ctrl = match ctx.controller.as_ref() {
        Some(ControllerHandle::Damiao(ctrl)) => ctrl,
        _ => return Err("damiao controller not connected".to_string()),
    };

    let default_model = if SessionCtx::model_is_auto(&ctx.target.model) {
        "4310"
    } else {
        &ctx.target.model
    };
    let default_item = json!({
        "motor_id": ctx.target.motor_id,
        "feedback_id": ctx.target.feedback_id,
        "model": default_model,
    });
    let items = v
        .get("items")
        .or_else(|| v.get("motors"))
        .and_then(Value::as_array);
    let fallback_items = [default_item];
    let iter: Box<dyn Iterator<Item = &Value> + '_> = match items {
        Some(items) => Box::new(items.iter()),
        None => Box::new(fallback_items.iter()),
    };

    let mut states = Vec::new();
    for item in iter {
        let motor_id = value_as_u16(item, "motor_id")
            .or_else(|| value_as_u16(item, "esc_id"))
            .unwrap_or(ctx.target.motor_id);
        let feedback_id = value_as_u16(item, "feedback_id")
            .or_else(|| value_as_u16(item, "mst_id"))
            .unwrap_or_else(|| damiao_default_feedback_id(motor_id));
        let model = item
            .get("model")
            .and_then(Value::as_str)
            .filter(|s| !SessionCtx::model_is_auto(s))
            .unwrap_or(default_model);

        let motor = match ctrl.get_motor(motor_id) {
            Ok(motor) => motor,
            Err(_) => ctrl
                .add_motor(motor_id, feedback_id, model)
                .map_err(|e| format!("add motor 0x{motor_id:X} failed: {e}"))?,
        };
        let state = motor
            .request_fresh_state(std::time::Duration::from_millis(timeout_ms))
            .map_err(|e| format!("request state 0x{motor_id:X} failed: {e}"))?;
        if let Some(s) = state {
            states.push(json!({
                "vendor": "damiao",
                "has_value": true,
                "motor_id": motor_id,
                "feedback_id": feedback_id,
                "model": model,
                "can_id": s.can_id,
                "arbitration_id": s.arbitration_id,
                "status_code": s.status_code,
                "status_name": s.status_name,
                "pos": s.pos,
                "vel": s.vel,
                "torq": s.torq,
                "t_mos": s.t_mos,
                "t_rotor": s.t_rotor,
            }));
        } else {
            states.push(json!({
                "vendor": "damiao",
                "has_value": false,
                "motor_id": motor_id,
                "feedback_id": feedback_id,
                "model": model,
            }));
        }
    }

    Ok(json!({
        "vendor": "damiao",
        "states": states,
    }))
}

fn handle_state_stream(v: &Value, state_stream_enabled: &mut bool) -> Result<Value, String> {
    *state_stream_enabled = as_bool(v, "enabled", false);
    Ok(json!({"enabled": *state_stream_enabled}))
}

fn handle_param_stream(
    v: &Value,
    ctx: &mut SessionCtx,
    stream: &mut ParamStream,
    dt_ms: u64,
    required_vendor: Option<Vendor>,
) -> Result<Value, String> {
    if !as_bool(v, "enabled", false) {
        let vendor = required_vendor.unwrap_or(ctx.target.vendor);
        stream.apply_message(v, dt_ms, vendor)?;
        return Ok(json!({
            "enabled": stream.enabled,
            "vendor": vendor.as_str(),
            "interval_ms": stream.tick_div.saturating_mul(dt_ms.max(1)),
            "timeout_ms": stream.timeout_ms,
            "params": stream.params,
        }));
    }

    ctx.ensure_connected()?;
    let vendor = match ctx.motor.as_ref() {
        Some(MotorHandle::Damiao(_)) => Vendor::Damiao,
        Some(MotorHandle::Robstride(_)) => Vendor::Robstride,
        Some(_) => return Err("param_stream is supported for damiao and robstride".to_string()),
        None => return Err("motor not connected".to_string()),
    };
    if let Some(required) = required_vendor {
        if vendor != required {
            return Err(format!(
                "{}_param_stream requires vendor={}",
                required.as_str(),
                required.as_str()
            ));
        }
    }
    stream.apply_message(v, dt_ms, vendor)?;
    Ok(json!({
        "enabled": stream.enabled,
        "vendor": vendor.as_str(),
        "interval_ms": stream.tick_div.saturating_mul(dt_ms.max(1)),
        "timeout_ms": stream.timeout_ms,
        "params": stream.params,
    }))
}

fn handle_status(ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    match (&ctx.controller, &ctx.motor) {
        (Some(ControllerHandle::Myactuator(c)), Some(MotorHandle::Myactuator(m))) => {
            m.request_status().map_err(|e| e.to_string())?;
            m.request_multi_turn_angle().map_err(|e| e.to_string())?;
            c.poll_feedback_once().map_err(|e| e.to_string())?;
        }
        (Some(ControllerHandle::Hexfellow(_)), Some(MotorHandle::Hexfellow(_)))
        | (Some(ControllerHandle::Damiao(_)), Some(MotorHandle::Damiao(_)))
        | (Some(ControllerHandle::Robstride(_)), Some(MotorHandle::Robstride(_)))
        | (Some(ControllerHandle::Hightorque(_)), Some(MotorHandle::Hightorque(_)))
        | (Some(ControllerHandle::CyberBeast(_)), Some(MotorHandle::CyberBeast(_))) => {}
        _ => return Err("motor not connected".to_string()),
    }
    Ok(json!({"state": ctx.build_state_snapshot()?}))
}

fn handle_poll_feedback_once(ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    if let Some(c) = ctx.controller.as_ref() {
        match c {
            ControllerHandle::Damiao(ctrl) => {
                ctrl.poll_feedback_once().map_err(|e| e.to_string())?
            }
            ControllerHandle::Hexfellow(ctrl) => {
                ctrl.poll_feedback_once().map_err(|e| e.to_string())?
            }
            ControllerHandle::Hightorque(_) => {}
            ControllerHandle::Myactuator(ctrl) => {
                ctrl.poll_feedback_once().map_err(|e| e.to_string())?
            }
            ControllerHandle::CyberBeast(ctrl) => {
                ctrl.poll_feedback_once().map_err(|e| e.to_string())?
            }
            ControllerHandle::Robstride(ctrl) => {
                ctrl.poll_feedback_once().map_err(|e| e.to_string())?
            }
        }
    }
    Ok(json!({"polled": true}))
}

fn handle_shutdown(ctx: &mut SessionCtx) -> Result<Value, String> {
    if let Some(c) = ctx.controller.as_ref() {
        match c {
            ControllerHandle::Damiao(ctrl) => ctrl.shutdown().map_err(|e| e.to_string())?,
            ControllerHandle::Hexfellow(ctrl) => ctrl.shutdown().map_err(|e| e.to_string())?,
            ControllerHandle::Hightorque(bus) => bus.shutdown().map_err(|e| e.to_string())?,
            ControllerHandle::Myactuator(ctrl) => ctrl.shutdown().map_err(|e| e.to_string())?,
            ControllerHandle::CyberBeast(ctrl) => ctrl.shutdown().map_err(|e| e.to_string())?,
            ControllerHandle::Robstride(ctrl) => ctrl.shutdown().map_err(|e| e.to_string())?,
        }
    }
    ctx.active = None;
    Ok(json!({"shutdown": true}))
}

fn handle_close_bus(ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.disconnect(false);
    Ok(json!({"closed": true}))
}
