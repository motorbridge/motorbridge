use crate::args::{
    get_f32, get_opt_u16_hex_or_dec, get_str, get_u16_hex_or_dec, get_u16_list_hex_or_dec, get_u64,
    parse_u16_hex_or_dec,
};
use motor_vendor_robstride::{
    model_limits as robstride_model_limits, protocol_name, ControlMode as RobstrideControlMode,
    ParameterDataType, ParameterValue, RobstrideController, PROTOCOL_CANOPEN, PROTOCOL_MIT,
    PROTOCOL_PRIVATE,
};
use std::collections::HashMap;
use std::convert::TryFrom;
use std::time::Duration;

fn parse_robstride_param_value(param_id: u16, raw: &str) -> Result<ParameterValue, String> {
    let info = motor_vendor_robstride::parameter_info(param_id)
        .ok_or_else(|| format!("unknown RobStride parameter 0x{param_id:04X}"))?;
    match info.data_type {
        ParameterDataType::Int8 => raw
            .parse::<i8>()
            .map(ParameterValue::I8)
            .map_err(|e| format!("invalid --param-value: {e}")),
        ParameterDataType::UInt8 => raw
            .parse::<u8>()
            .map(ParameterValue::U8)
            .map_err(|e| format!("invalid --param-value: {e}")),
        ParameterDataType::UInt16 => {
            parse_u16_hex_or_dec(raw, "param-value").map(ParameterValue::U16)
        }
        ParameterDataType::UInt32 => {
            if let Some(hex) = raw.strip_prefix("0x") {
                u32::from_str_radix(hex, 16)
                    .map(ParameterValue::U32)
                    .map_err(|e| format!("invalid --param-value: {e}"))
            } else {
                raw.parse::<u32>()
                    .map(ParameterValue::U32)
                    .map_err(|e| format!("invalid --param-value: {e}"))
            }
        }
        ParameterDataType::Float32 => raw
            .parse::<f32>()
            .map(ParameterValue::F32)
            .map_err(|e| format!("invalid --param-value: {e}")),
    }
}

fn print_robstride_param_value(param_id: u16, value: ParameterValue) {
    let name = motor_vendor_robstride::parameter_info(param_id)
        .map(|info| info.name)
        .unwrap_or("unknown");
    match value {
        ParameterValue::I8(v) => println!("param 0x{param_id:04X} ({name}) = {v}"),
        ParameterValue::U8(v) => println!("param 0x{param_id:04X} ({name}) = {v}"),
        ParameterValue::U16(v) => println!("param 0x{param_id:04X} ({name}) = {v}"),
        ParameterValue::U32(v) => println!("param 0x{param_id:04X} ({name}) = {v}"),
        ParameterValue::F32(v) => println!("param 0x{param_id:04X} ({name}) = {v:.6}"),
    }
}

fn robstride_run_mode_name(mode: i8) -> &'static str {
    match mode {
        0 => "mit/private-motion",
        1 => "position-pp",
        2 => "velocity",
        3 => "current",
        5 => "position-csp",
        _ => "unknown",
    }
}

fn parse_protocol_cmd(raw: &str) -> Result<u8, String> {
    match raw.to_ascii_lowercase().as_str() {
        "0" | "private" | "siyou" => Ok(PROTOCOL_PRIVATE),
        "1" | "canopen" | "cia402" => Ok(PROTOCOL_CANOPEN),
        "2" | "mit" => Ok(PROTOCOL_MIT),
        _ => Err(format!(
            "invalid --protocol {raw}, expected private|canopen|mit or 0|1|2"
        )),
    }
}

fn validate_robstride_device_id(id: u16, name: &str) -> Result<u8, String> {
    if (1..=255).contains(&id) {
        Ok(u8::try_from(id).expect("validated RobStride device id"))
    } else {
        Err(format!("RobStride {name} must be in 1..255, got {id}"))
    }
}

fn validate_robstride_host_id(id: u16, name: &str) -> Result<u16, String> {
    if id <= 255 {
        Ok(id)
    } else {
        Err(format!(
            "RobStride {name}/host_id must be in 0..255, got {id}"
        ))
    }
}

pub fn run_robstride(
    args: &HashMap<String, String>,
    channel: &str,
    model: &str,
    motor_id: u16,
    feedback_id: u16,
    vendor_name: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    let model_input = model;
    let model = if vendor_name == "hightorque" {
        let m = model_input.trim().to_ascii_lowercase();
        if m.is_empty() || m == "hightorque" || m == "ht" || m == "auto" || m == "default" {
            println!(
                "[info] vendor=hightorque model={} -> compat model=rs-00",
                model_input
            );
            "rs-00"
        } else if m.starts_with("rs-") {
            model_input
        } else {
            println!(
                "[warn] vendor=hightorque received unsupported compat model '{}'; fallback to rs-00",
                model_input
            );
            "rs-00"
        }
    } else {
        model_input
    };

    let mode = get_str(args, "mode", "ping");
    let loop_n = get_u64(args, "loop", 1)?;
    let dt_ms = get_u64(args, "dt-ms", 20)?;
    let ensure_mode = get_u64(args, "ensure-mode", 1)? != 0;
    let zero_exp = get_u64(args, "zero-exp", 0)? != 0;
    let set_motor_id = get_opt_u16_hex_or_dec(args, "set-motor-id")?;
    let store_after_set = get_u64(args, "store", 1)? != 0;
    let validated_set_motor_id = if vendor_name == "robstride" {
        validate_robstride_device_id(motor_id, "motor_id")?;
        validate_robstride_host_id(feedback_id, "feedback_id")?;
        set_motor_id
            .map(|id| validate_robstride_device_id(id, "new_motor_id"))
            .transpose()?
    } else {
        set_motor_id.map(|id| id as u8)
    };

    if let Some((pmax, vmax, tmax)) = robstride_model_limits(model) {
        println!(
            "[info] {}(robstride-compatible) model {} limits pmax={:.3} vmax={:.3} tmax={:.3}",
            vendor_name, model, pmax, vmax, tmax
        );
    }
    if feedback_id != 0x00FD {
        println!(
            "[info] robstride feedback_id is host_id, not motor_id; probing candidates including --feedback-id=0x{:X}, 0xFD, 0xFF, 0xFE for control/read robustness",
            feedback_id
        );
    } else {
        println!(
            "[info] robstride feedback_id/host_id candidates: 0xFD (primary), then 0xFF/0xFE fallback; host_id is not motor_id"
        );
    }

    let query_ping = |m: &std::sync::Arc<motor_vendor_robstride::RobstrideMotor>,
                      param_id: u16,
                      timeout: Duration|
     -> Option<u16> {
        if m.get_parameter(param_id, timeout).is_ok() {
            return Some(param_id);
        }
        if param_id != 0x7019 && m.get_parameter(0x7019, timeout).is_ok() {
            return Some(0x7019);
        }
        None
    };

    if mode == "scan" {
        let raw_start_id = get_u16_hex_or_dec(args, "start-id", 1)?;
        let raw_end_id = get_u16_hex_or_dec(args, "end-id", 255)?;
        if raw_start_id == 0
            || raw_end_id == 0
            || raw_start_id > 255
            || raw_end_id > 255
            || raw_start_id > raw_end_id
        {
            return Err("invalid scan range: expected 1..255 and start<=end".into());
        }
        let (start_id, end_id) = if vendor_name == "hightorque" {
            (raw_start_id.clamp(1, 32), raw_end_id.clamp(1, 32))
        } else {
            (raw_start_id, raw_end_id)
        };
        if start_id > end_id {
            return Err("invalid scan range after clamping".into());
        }
        let scan_feedback_ids = if vendor_name == "robstride" {
            let ids = get_u16_list_hex_or_dec(
                args,
                "feedback-ids",
                &[0x00FD, 0x00FF, 0x00FE, 0x0000, 0x00AA],
            )?;
            for fid in &ids {
                validate_robstride_host_id(*fid, "feedback-ids")?;
            }
            ids
        } else {
            vec![feedback_id]
        };
        let timeout_ms = get_u64(
            args,
            "timeout-ms",
            if vendor_name == "hightorque" { 40 } else { 80 },
        )?;
        let param_timeout_ms = get_u64(args, "param-timeout-ms", 120)?;
        let param_id = get_u16_hex_or_dec(args, "param-id", 0x7019)?;

        if vendor_name == "robstride" {
            println!(
                "[scan:robstride] channel={} model={} id_range=[0x{:X},0x{:X}] timeout_ms={} feedback_ids={} param_id=0x{:04X}",
                channel,
                model,
                start_id,
                end_id,
                timeout_ms,
                scan_feedback_ids
                    .iter()
                    .map(|v| format!("0x{v:X}"))
                    .collect::<Vec<_>>()
                    .join(","),
                param_id
            );
            println!(
                "[scan:robstride] note: probe/device_id is the motor ID; feedback_id/host_id is the host-side ID"
            );
        } else {
            println!(
                "[scan] probing {} IDs {}..{} on {}",
                vendor_name, start_id, end_id, channel
            );
        }
        if vendor_name == "hightorque" && (raw_start_id != start_id || raw_end_id != end_id) {
            println!(
                "[scan] vendor=hightorque range clamped to {}..{} (requested {}..{})",
                start_id, end_id, raw_start_id, raw_end_id
            );
        }
        let mut hits = 0usize;
        for id in start_id..=end_id {
            let mut hit = false;
            for fid in &scan_feedback_ids {
                let probe_ctrl = RobstrideController::new_socketcan(channel)?;
                let candidate = probe_ctrl.add_motor(id, *fid, model)?;
                if let Ok(reply) =
                    candidate.ping_with_host_id(*fid, Duration::from_millis(timeout_ms))
                {
                    println!(
                        "[hit] probe=0x{:02X} vendor={} via=ping feedback_id=0x{:X} device_id={} responder_id={} model_hint={} payload={:02x?}",
                        id,
                        vendor_name,
                        fid,
                        reply.device_id,
                        reply.responder_id,
                        model,
                        reply.payload
                    );
                    hit = true;
                } else if vendor_name != "hightorque" {
                    let exact_param = candidate
                        .get_parameter_with_host_id(
                            param_id,
                            *fid,
                            Duration::from_millis(param_timeout_ms),
                        )
                        .map(|_| param_id)
                        .or_else(|_| {
                            if param_id == 0x7019 {
                                Err(())
                            } else {
                                candidate
                                    .get_parameter_with_host_id(
                                        0x7019,
                                        *fid,
                                        Duration::from_millis(param_timeout_ms),
                                    )
                                    .map(|_| 0x7019)
                                    .map_err(|_| ())
                            }
                        })
                        .ok();
                    if let Some(pid) = exact_param {
                        println!(
                            "[hit] probe=0x{:02X} vendor={} via=read-param feedback_id=0x{:X} device_id={} param_id=0x{:04X} model_hint={}",
                            id, vendor_name, fid, id, pid, model
                        );
                        hit = true;
                    }
                }
                probe_ctrl.close_bus()?;
                if hit {
                    break;
                }
            }
            if hit {
                hits += 1;
            } else if vendor_name == "robstride" {
                println!("[.. ] vendor=robstride probe=0x{id:02X} no reply");
            }
            std::thread::sleep(Duration::from_millis(2));
        }
        if hits == 0 {
            let fallback = RobstrideController::new_socketcan(channel)?;
            let manual_vel = get_f32(args, "manual-vel", 0.2)?;
            let manual_ms = get_u64(args, "manual-ms", 200)?;
            let manual_gap_ms = get_u64(args, "manual-gap-ms", 200)?;
            println!(
                "[scan] no ping replies for {}; fallback to blind pulse probing (observe motor motion)",
                vendor_name
            );
            println!(
                "[scan] pulse: vel={:.3} for {}ms, gap={}ms",
                manual_vel, manual_ms, manual_gap_ms
            );
            for id in start_id..=end_id {
                let candidate = fallback.add_motor(id, scan_feedback_ids[0], model)?;
                let _ = candidate.enable();
                let _ = candidate.set_mode(RobstrideControlMode::Velocity);
                let mut state_seen = false;
                let t_end = std::time::Instant::now() + Duration::from_millis(manual_ms);
                while std::time::Instant::now() < t_end {
                    let _ = candidate.set_velocity_target(manual_vel);
                    if candidate.latest_state().is_some() {
                        state_seen = true;
                    }
                    std::thread::sleep(Duration::from_millis(40));
                }
                for _ in 0..3 {
                    let _ = candidate.set_velocity_target(0.0);
                    if candidate.latest_state().is_some() {
                        state_seen = true;
                    }
                    std::thread::sleep(Duration::from_millis(30));
                }
                let _ = candidate.disable();
                if state_seen {
                    hits += 1;
                    if let Some(s) = candidate.latest_state() {
                        println!(
                            "[hit] vendor={} id={} by=status pos={:+.3} vel={:+.3} torq={:+.3}",
                            vendor_name, id, s.position, s.velocity, s.torque
                        );
                    } else {
                        println!("[hit] vendor={} id={} by=status", vendor_name, id);
                    }
                } else {
                    println!(
                        "[probe] vendor={} id={} model_hint={} (if this ID moved, note it)",
                        vendor_name, id, model
                    );
                }
                std::thread::sleep(Duration::from_millis(manual_gap_ms));
            }
            fallback.close_bus()?;
            println!("[scan] done vendor={} hits={hits}", vendor_name);
            return Ok(());
        }
        println!("[scan] done vendor={} hits={hits}", vendor_name);
        return Ok(());
    }
    let controller = RobstrideController::new_socketcan(channel)?;
    let motor = controller.add_motor(motor_id, feedback_id, model)?;

    if let Some(new_motor_id_u8) = validated_set_motor_id {
        let new_motor_id = u16::from(new_motor_id_u8);
        if mode != "ping" {
            println!(
                "[info] robstride id-set requested; --mode {} is ignored during id-set flow",
                mode
            );
        }
        motor.set_device_id(new_motor_id_u8)?;
        println!(
            "[id-set] {} device id update requested: {} -> {}",
            vendor_name, motor_id, new_motor_id
        );
        if store_after_set {
            motor.save_parameters()?;
            println!("[id-set] {} save-parameters sent", vendor_name);
        }
        std::thread::sleep(Duration::from_millis(120));
        let old_ping = motor.ping(Duration::from_millis(140)).ok();
        if old_ping.is_some() {
            println!(
                "[warn] {} old-id ping still responded on id={} (firmware apply may be delayed)",
                vendor_name, motor_id
            );
        }
        controller.close_bus()?;
        let verify_ctrl = RobstrideController::new_socketcan(channel)?;
        let verify_motor = verify_ctrl.add_motor(new_motor_id, feedback_id, model)?;
        match verify_motor.ping(Duration::from_millis(260)) {
            Ok(reply) => {
                println!(
                    "[id-set] verify ping ok: responder_id={} new_id={}",
                    reply.responder_id, reply.device_id
                );
            }
            Err(e) => {
                println!(
                    "[warn] {} id-set verify ping on new id={} failed: {}",
                    vendor_name, new_motor_id, e
                );
            }
        }
        let _ = verify_ctrl.close_bus();
        return Ok(());
    }

    match mode.as_str() {
        "ping" => {
            if let Ok(reply) = motor.ping(Duration::from_millis(500)) {
                println!(
                    "[ok] ping device_id={} responder_id={} payload={:02x?}",
                    reply.device_id, reply.responder_id, reply.payload
                );
            } else if let Some(pid) = query_ping(&motor, 0x7019, Duration::from_millis(120)) {
                println!("[ok] ping(by query) param 0x{pid:04X} responded");
            } else {
                return Err(format!(
                    "{} ping failed: no response to GET_DEVICE_ID or query parameters",
                    vendor_name
                )
                .into());
            }
            controller.close_bus()?;
            return Ok(());
        }
        "read-param" => {
            let param_id = get_u16_hex_or_dec(args, "param-id", 0)?;
            let value = motor.get_parameter(param_id, Duration::from_millis(500))?;
            print_robstride_param_value(param_id, value);
            controller.close_bus()?;
            return Ok(());
        }
        "get-protocol" | "protocol" | "query-protocol" => {
            let value = motor.get_protocol_flag(Duration::from_millis(500))?;
            println!(
                "[protocol] vendor=robstride current={} ({}) note=read protocol_1 param 0x2022",
                value,
                protocol_name(value)
            );
            controller.close_bus()?;
            return Ok(());
        }
        "get-mode" | "query-mode" => {
            let value = motor.get_parameter_i8(0x7005, Duration::from_millis(500))?;
            println!(
                "[mode] vendor=robstride run_mode={} ({}) note=read run_mode param 0x7005",
                value,
                robstride_run_mode_name(value)
            );
            controller.close_bus()?;
            return Ok(());
        }
        "set-protocol" | "protocol-switch" => {
            let protocol = parse_protocol_cmd(&get_str(args, "protocol", "private"))?;
            let wait_ack = get_u64(args, "wait-ack", 1)? != 0;
            let timeout = if wait_ack {
                Duration::from_millis(get_u64(args, "timeout-ms", 500)?)
            } else {
                Duration::ZERO
            };
            motor.set_protocol(protocol, timeout)?;
            println!(
                "[ok] robstride private protocol switch sent target={} ({}) via comm_type=25; power-cycle motor to apply",
                protocol,
                protocol_name(protocol)
            );
            controller.close_bus()?;
            return Ok(());
        }
        "write-param" => {
            let param_id = get_u16_hex_or_dec(args, "param-id", 0)?;
            let raw = args
                .get("param-value")
                .ok_or_else(|| "missing --param-value".to_string())?;
            let value = parse_robstride_param_value(param_id, raw)?;
            let store_after_write = get_u64(args, "store", 0)? != 0;
            motor.write_parameter(param_id, value)?;
            std::thread::sleep(Duration::from_millis(50));
            let verify = motor.get_parameter(param_id, Duration::from_millis(500))?;
            print_robstride_param_value(param_id, verify);
            if store_after_write {
                motor.save_parameters()?;
                println!("[ok] save-parameters requested (store=1)");
            }
            controller.close_bus()?;
            return Ok(());
        }
        "save" => {
            motor.save_parameters()?;
            println!("[ok] save-parameters requested");
            controller.close_bus()?;
            return Ok(());
        }
        "active-report" => {
            let enabled = get_u64(args, "active-report", 1)? != 0;
            motor.set_active_report(enabled)?;
            println!(
                "[ok] active report {}",
                if enabled { "enabled" } else { "disabled" }
            );
            controller.close_bus()?;
            return Ok(());
        }
        "zero-by-offset" => {
            let _ = get_u64(args, "offset-negate", 0)?;
            let _ = get_u64(args, "store", 1)?;
            println!(
                "[warn] robstride zero-by-offset is temporarily disabled due to firmware inconsistency; no calibration CAN frames sent"
            );
            controller.close_bus()?;
            return Ok(());
        }
        _ => {}
    }

    if ensure_mode {
        match mode.as_str() {
            "mit" => {
                motor.ensure_control_mode(RobstrideControlMode::Mit, Duration::from_millis(1000))?
            }
            "pos-vel" => motor
                .ensure_control_mode(RobstrideControlMode::Position, Duration::from_millis(1000))?,
            "pos-vel-pp" | "pos-vel-csp" => {}
            "vel" => motor
                .ensure_control_mode(RobstrideControlMode::Velocity, Duration::from_millis(1000))?,
            _ => {}
        }
        // Align with official sequence: mode write first, then allow a brief settle window
        // before sending target parameters/operation frames.
        std::thread::sleep(Duration::from_millis(30));
    }

    if mode != "disable"
        && mode != "stop"
        && mode != "zero"
        && mode != "set-zero"
        && mode != "save"
        && mode != "clear-error"
        && mode != "active-report"
        && mode != "zero-by-offset"
        && mode != "pos-vel-pp"
        && mode != "pos-vel-csp"
    {
        controller.enable_all()?;
        std::thread::sleep(Duration::from_millis(100));
    }

    for i in 0..loop_n {
        match mode.as_str() {
            "enable" => {
                if let Err(e) = motor.enable() {
                    let msg = e.to_string();
                    if msg.contains("control ack timeout") {
                        println!(
                            "[warn] enable ack timeout; command may still have been applied (no immediate status frame)"
                        );
                    } else {
                        return Err(e.into());
                    }
                }
            }
            "disable" => {
                if let Err(e) = motor.disable() {
                    let msg = e.to_string();
                    if msg.contains("control ack timeout") {
                        println!(
                            "[warn] disable ack timeout; command may still have been applied (no immediate status frame)"
                        );
                    } else {
                        return Err(e.into());
                    }
                }
            }
            "stop" => match motor.controlled_stop(std::time::Duration::from_millis(300)) {
                Ok(mode) => {
                    println!(
                        "[ok] controlled stop: mode={} strategy={}",
                        mode.as_str(),
                        match mode {
                            motor_vendor_robstride::ControlMode::Mit => "hold-current-position-mit",
                            motor_vendor_robstride::ControlMode::Position => "pp-vel-max-zero",
                            motor_vendor_robstride::ControlMode::Velocity => "velocity-target-zero",
                            motor_vendor_robstride::ControlMode::PositionCsp =>
                                "hold-current-position-csp",
                        }
                    );
                }
                Err(e) => {
                    let msg = e.to_string();
                    if msg.contains("control ack timeout") {
                        println!("[warn] stop ack timeout; command may still have been applied");
                    } else {
                        return Err(e.into());
                    }
                }
            },
            "clear-error" => {
                if let Err(e) = motor.clear_error() {
                    let msg = e.to_string();
                    if msg.contains("control ack timeout") {
                        println!(
                            "[warn] clear-error ack timeout; command may still have been applied"
                        );
                    } else {
                        return Err(e.into());
                    }
                } else {
                    println!("[ok] clear-error requested");
                }
            }
            "zero" | "set-zero" => {
                if !zero_exp {
                    println!(
                        "[warn] robstride set-zero requires experimental sequence; no CAN frame sent. Re-run with --zero-exp 1"
                    );
                } else {
                    let pre_mech = motor.get_parameter(0x7019, Duration::from_millis(200)).ok();
                    // Experimental calibration sequence:
                    // disable -> set-zero -> optional save -> readback hints.
                    let _ = controller.disable_all();
                    std::thread::sleep(Duration::from_millis(80));
                    if let Err(e) = motor.set_zero_position() {
                        let msg = e.to_string();
                        if msg.contains("control ack timeout") {
                            println!(
                                "[warn] zero ack timeout; command may still be applied. continue with parameter verification"
                            );
                        } else {
                            return Err(e.into());
                        }
                    }
                    std::thread::sleep(Duration::from_millis(80));
                    if store_after_set {
                        motor.save_parameters()?;
                        std::thread::sleep(Duration::from_millis(80));
                    }
                    let zero_sta = motor.get_parameter(0x7029, Duration::from_millis(200)).ok();
                    if let Some(v) = zero_sta {
                        print_robstride_param_value(0x7029, v);
                    }
                    let post_mech = motor.get_parameter(0x7019, Duration::from_millis(200)).ok();
                    if let Some(v) = post_mech {
                        print_robstride_param_value(0x7019, v);
                    }
                    let pre_mech_f32 = match pre_mech {
                        Some(ParameterValue::F32(v)) => Some(v),
                        _ => None,
                    };
                    let post_mech_f32 = match post_mech {
                        Some(ParameterValue::F32(v)) => Some(v),
                        _ => None,
                    };
                    let zero_sta_u8 = match zero_sta {
                        Some(ParameterValue::U8(v)) => Some(v),
                        _ => None,
                    };
                    let pos_zero_ok = post_mech_f32.map(|v| v.abs() < 0.05).unwrap_or(false);
                    let zero_flag_ok = zero_sta_u8.map(|v| v != 0).unwrap_or(false);
                    if !(pos_zero_ok || zero_flag_ok) {
                        return Err(format!(
                            "robstride set-zero verify failed: pre_mechPos={:?}, post_mechPos={:?}, zero_sta={:?}. firmware likely ignored zero command in current state",
                            pre_mech_f32, post_mech_f32, zero_sta_u8
                        )
                        .into());
                    }
                    println!(
                        "[ok] robstride set-zero experimental sequence finished (store={})",
                        if store_after_set { 1 } else { 0 }
                    );
                }
            }
            "mit" => {
                if i == 0 {
                    println!(
                        "[info] robstride mit mapping: effective args are --pos(rad) --vel(rad/s) --kp --kd --tau(Nm)"
                    );
                }
                let kp = get_f32(args, "kp", 8.0)?;
                let kd = get_f32(args, "kd", 0.2)?;
                let tau = get_f32(args, "tau", 0.0)?;
                if kp == 0.0 && kd == 0.0 && tau != 0.0 {
                    println!(
                        "[warn] mit with kp=0,kd=0,tau!=0 behaves as near open-loop torque drive and can keep accelerating under low load"
                    );
                }
                motor.send_cmd_mit(
                    get_f32(args, "pos", 0.0)?,
                    get_f32(args, "vel", 0.0)?,
                    kp,
                    kd,
                    tau,
                )?;
            }
            "pos-vel" => {
                if args.contains_key("vel") || args.contains_key("kd") || args.contains_key("tau") {
                    println!(
                        "[warn] robstride pos-vel maps to native Position mode; --vel/--kd/--tau are ignored"
                    );
                }
                let vlim = get_f32(args, "vlim", 1.0)?.abs();
                if vlim.is_finite() && vlim > 0.0 {
                    motor.write_parameter(0x7017, ParameterValue::F32(vlim))?;
                }
                // Position mode may appear "not moving" if internal position gain is too low.
                // Allow explicit gain mapping via --loc-kp (or reuse --kp for convenience).
                let loc_kp = if args.contains_key("loc-kp") {
                    if args.contains_key("kp") {
                        println!(
                            "[warn] both --loc-kp and --kp provided; robstride pos-vel uses --loc-kp"
                        );
                    }
                    Some(get_f32(args, "loc-kp", 0.0)?)
                } else if args.contains_key("kp") {
                    Some(get_f32(args, "kp", 0.0)?)
                } else {
                    None
                };
                if let Some(kp) = loc_kp {
                    if kp.is_finite() && kp >= 0.0 {
                        motor.write_parameter(0x701E, ParameterValue::F32(kp))?;
                    }
                }
                motor.write_parameter(0x7016, ParameterValue::F32(get_f32(args, "pos", 0.0)?))?;
            }
            "pos-vel-pp" => {
                if i == 0 {
                    println!(
                        "[info] robstride pos-vel-pp maps to native PP mode: run_mode=1, vel_max(0x7024), acc_set(0x7025), loc_ref(0x7016)"
                    );
                }
                motor.send_cmd_pos_vel_pp(
                    get_f32(args, "pos", 0.0)?,
                    get_f32(args, "vlim", 1.0)?,
                    get_f32(args, "acc", 10.0)?,
                )?;
            }
            "pos-vel-csp" => {
                if i == 0 {
                    println!(
                        "[info] robstride pos-vel-csp maps to native CSP mode: run_mode=5, limit_spd(0x7017), loc_ref(0x7016)"
                    );
                }
                motor.send_cmd_pos_vel_csp(
                    get_f32(args, "pos", 0.0)?,
                    get_f32(args, "vlim", 1.0)?,
                )?;
            }
            "vel" => {
                motor.set_velocity_target(get_f32(args, "vel", 0.0)?)?;
            }
            _ => return Err(format!("unknown {vendor_name} mode: {mode}").into()),
        }

        if let Some(s) = motor.latest_state() {
            println!(
                "#{i} pos={:+.3} vel={:+.3} torq={:+.3} temp={:.1} flags[u={} stall={} enc={} ot={} oc={} uv={}]",
                s.position,
                s.velocity,
                s.torque,
                s.temperature_c,
                s.uncalibrated,
                s.stall,
                s.magnetic_encoder_fault,
                s.overtemperature,
                s.overcurrent,
                s.undervoltage
            );
            if let Some(f) = motor.latest_fault_report() {
                println!(
                    "    fault_raw=0x{:08X} warning_raw=0x{:08X} fault[a_oc={} b_oc={} c_oc={} stall={} pos_init={} hw_id={} enc_uncal={} ov={} uv={} drv={} ot={}] warn[ot={}]",
                    f.fault_raw,
                    f.warning_raw,
                    f.faults.phase_a_overcurrent,
                    f.faults.phase_b_overcurrent,
                    f.faults.phase_c_overcurrent,
                    f.faults.stall_overload,
                    f.faults.position_init_fault,
                    f.faults.hardware_id_fault,
                    f.faults.encoder_uncalibrated,
                    f.faults.overvoltage,
                    f.faults.undervoltage,
                    f.faults.driver_fault,
                    f.faults.overtemperature,
                    f.warnings.overtemperature_warning
                );
            }
        }
        std::thread::sleep(Duration::from_millis(dt_ms));
    }

    if mode == "enable"
        || mode == "disable"
        || mode == "stop"
        || mode == "zero"
        || mode == "set-zero"
        || mode == "save"
        || mode == "clear-error"
        || mode == "active-report"
        || mode == "zero-by-offset"
    {
        controller.close_bus()?;
    } else {
        controller.shutdown()?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_robstride_param_value_uses_parameter_type() {
        let mode = parse_robstride_param_value(0x7005, "2").expect("int8 mode");
        let timeout = parse_robstride_param_value(0x7028, "123").expect("u32 timeout");
        let mech = parse_robstride_param_value(0x7019, "1.5").expect("f32 mech pos");

        match mode {
            ParameterValue::I8(v) => assert_eq!(v, 2),
            _ => panic!("expected I8"),
        }
        match timeout {
            ParameterValue::U32(v) => assert_eq!(v, 123),
            _ => panic!("expected U32"),
        }
        match mech {
            ParameterValue::F32(v) => assert!((v - 1.5).abs() < 1e-6),
            _ => panic!("expected F32"),
        }
    }
}
