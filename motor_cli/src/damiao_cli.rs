use crate::args::{get_f32, get_opt_u16_hex_or_dec, get_str, get_u16_hex_or_dec, get_u64};
use motor_core::dm_device::DmDeviceType;
use motor_vendor_damiao::{
    display_models, match_models_by_limits, model_limits as damiao_model_limits,
    suggest_models_by_limits, ControlMode as DamiaoControlMode, DamiaoController, DamiaoMotor,
};
use std::collections::HashMap;
use std::convert::TryFrom;
use std::time::Duration;

const DAMIAO_SCAN_MODEL_HINTS: &[&str] = &[
    "4340P", "4340", "4310", "4310P", "3507", "6006", "8006", "8009", "10010L", "10010", "H3510",
    "G6215", "H6220", "JH11", "6248P",
];

fn build_scan_model_hints() -> Vec<String> {
    let mut out: Vec<String> = Vec::new();
    for m in DAMIAO_SCAN_MODEL_HINTS {
        if !out.iter().any(|x| x.eq_ignore_ascii_case(m)) {
            out.push((*m).to_string());
        }
    }
    out
}

fn build_scan_feedback_hints(base_feedback_id: u16, motor_id: u16) -> Vec<u16> {
    let mut out = Vec::new();
    let inferred = motor_id.saturating_add(0x10);
    for fid in [inferred, base_feedback_id, 0x0011, 0x0017] {
        if !out.contains(&fid) {
            out.push(fid);
        }
    }
    out
}

fn verify_declared_damiao_model(
    motor: &DamiaoMotor,
    declared_model: &str,
    timeout: Duration,
    tol: f32,
) -> Result<(), String> {
    let expected = damiao_model_limits(declared_model)
        .ok_or_else(|| format!("unknown model in catalog: {declared_model}"))?;

    let pmax = motor
        .get_register_f32(21, timeout)
        .map_err(|e| format!("model handshake failed reading PMAX(rid=21): {e}"))?;
    let vmax = motor
        .get_register_f32(22, timeout)
        .map_err(|e| format!("model handshake failed reading VMAX(rid=22): {e}"))?;
    let tmax = motor
        .get_register_f32(23, timeout)
        .map_err(|e| format!("model handshake failed reading TMAX(rid=23): {e}"))?;

    let matched = match_models_by_limits(pmax, vmax, tmax, tol);
    if matched.contains(&declared_model) {
        println!(
            "[ok] model handshake passed: --model {} matches PMAX/VMAX/TMAX=({:.3}, {:.3}, {:.3})",
            declared_model, pmax, vmax, tmax
        );
        return Ok(());
    }

    let suggested = suggest_models_by_limits(pmax, vmax, tmax, 3);
    let suggest_text = if suggested.is_empty() {
        "none".to_string()
    } else {
        suggested.join(", ")
    };
    Err(format!(
        "model handshake mismatch: --model {} expects ({:.3}, {:.3}, {:.3}), \
device reports ({:.3}, {:.3}, {:.3}), suggested: {}. \
If intentional, run with --verify-model 0",
        declared_model, expected.0, expected.1, expected.2, pmax, vmax, tmax, suggest_text
    ))
}

fn open_damiao_controller(
    transport: &str,
    channel: &str,
    serial_port: &str,
    serial_baud: u32,
    dm_device_type: &str,
    dm_channel: &str,
) -> Result<DamiaoController, Box<dyn std::error::Error>> {
    let p = motor_core::bus::TransportParams {
        channel,
        serial_port,
        serial_baud,
    };
    match transport {
        "auto" | "socketcan" => {
            Ok(DamiaoController::new(motor_core::bus::open_transport(
                motor_core::bus::Transport::SocketCan,
                &p,
            )?))
        }
        "socketcanfd" => {
            Ok(DamiaoController::new(motor_core::bus::open_transport(
                motor_core::bus::Transport::SocketCanFd,
                &p,
            )?))
        }
        "dm-serial" => {
            Ok(DamiaoController::new(motor_core::bus::open_transport(
                motor_core::bus::Transport::DmSerial,
                &p,
            )?))
        }
        "mcu-serial" => {
            Ok(DamiaoController::new(motor_core::bus::open_transport(
                motor_core::bus::Transport::McuSerial,
                &p,
            )?))
        }
        "dm-device" => Ok(DamiaoController::new_dm_device(
            DmDeviceType::parse(dm_device_type)?,
            dm_channel,
        )?),
        _ => Err(format!(
            "unknown Damiao transport: {} (expected auto|socketcan|socketcanfd|dm-serial|mcu-serial|dm-device)",
            transport
        )
        .into()),
    }
}

fn dm_device_scan_channels(
    args: &HashMap<String, String>,
    dm_device_type: &str,
) -> Result<Vec<String>, Box<dyn std::error::Error>> {
    if let Some(explicit) = args.get("dm-channel") {
        return Ok(vec![explicit.clone()]);
    }
    let device_type = DmDeviceType::parse(dm_device_type)?;
    match device_type {
        DmDeviceType::Usb2CanFd => Ok(vec!["0".to_string()]),
        DmDeviceType::Usb2CanFdDual => Ok(vec!["0".to_string(), "1".to_string()]),
        DmDeviceType::LinkX4C => Ok(vec![
            "0".to_string(),
            "1".to_string(),
            "2".to_string(),
            "3".to_string(),
        ]),
    }
}

fn scan_damiao_dm_device_channel(
    dm_device_type: &str,
    dm_channel: &str,
    model: &str,
    start_id: u16,
    end_id: u16,
    close_after_scan: bool,
) -> Result<usize, Box<dyn std::error::Error>> {
    let controller = open_damiao_controller(
        "dm-device",
        "can0",
        "/dev/ttyACM0",
        921600,
        dm_device_type,
        dm_channel,
    )?;
    println!(
        "[scan] probing Damiao IDs {}..{} on dm-device:{} using feedback request",
        start_id, end_id, dm_channel
    );
    let mut motors = Vec::new();
    for id in start_id..=end_id {
        let fid = id.saturating_add(0x10);
        if let Ok(motor) = controller.add_motor(id, fid, model) {
            motors.push((id, fid, motor));
        }
    }

    let mut hits = 0usize;
    for (id, fid, motor) in motors {
        let _ = motor.request_motor_feedback();
        for _ in 0..20 {
            let _ = controller.poll_feedback_once();
            if let Some(s) = motor.latest_state() {
                println!(
                    "[hit] vendor=damiao dm_channel={} id={} feedback_id=0x{:X} detected_by=dm-device-feedback status={} pos={:+.3} vel={:+.3} torq={:+.3}",
                    dm_channel, id, fid, s.status_code, s.pos, s.vel, s.torq
                );
                hits += 1;
                break;
            }
            std::thread::sleep(Duration::from_millis(8));
        }
        std::thread::sleep(Duration::from_millis(2));
    }
    for _ in 0..50 {
        let _ = controller.poll_feedback_once();
        std::thread::sleep(Duration::from_millis(10));
    }
    println!(
        "[scan] done vendor=damiao dm_channel={} hits={hits}",
        dm_channel
    );
    if close_after_scan {
        controller.close_bus()?;
    }
    Ok(hits)
}

#[cfg(unix)]
fn hard_exit(code: i32) -> ! {
    unsafe extern "C" {
        fn _exit(status: i32) -> !;
    }
    unsafe { _exit(code) }
}

#[cfg(not(unix))]
fn hard_exit(code: i32) -> ! {
    std::process::exit(code)
}

fn damiao_param_type(args: &HashMap<String, String>) -> String {
    get_str(args, "type", &get_str(args, "param-type", "f32")).to_ascii_lowercase()
}

pub fn run_damiao(
    args: &HashMap<String, String>,
    channel: &str,
    model: &str,
    motor_id: u16,
    feedback_id: u16,
) -> Result<(), Box<dyn std::error::Error>> {
    let mode = get_str(args, "mode", "mit");
    let loop_n = get_u64(args, "loop", 1)?;
    let dt_ms = get_u64(args, "dt-ms", 20)?;
    let ensure_mode = get_u64(args, "ensure-mode", 1)? != 0;
    let set_motor_id = get_opt_u16_hex_or_dec(args, "set-motor-id")?;
    let set_feedback_id = get_opt_u16_hex_or_dec(args, "set-feedback-id")?;
    let store_after_set = get_u64(args, "store", 1)? != 0;
    let verify_id = get_u64(args, "verify-id", 1)? != 0;
    let verify_model = get_u64(args, "verify-model", 1)? != 0;
    let verify_timeout_ms = get_u64(args, "verify-timeout-ms", 500)?;
    let verify_tol = get_f32(args, "verify-tol", 0.2)?;
    let transport = get_str(args, "transport", "auto");
    let serial_port = get_str(args, "serial-port", "/dev/ttyACM0");
    let serial_baud_u64 = get_u64(args, "serial-baud", 921600)?;
    let serial_baud = u32::try_from(serial_baud_u64)
        .map_err(|_| format!("invalid --serial-baud (too large): {serial_baud_u64}"))?;
    let dm_device_type = get_str(args, "dm-device-type", "usb2canfd-dual");
    let dm_channel = get_str(args, "dm-channel", "0");

    if transport == "dm-serial" {
        println!(
            "[note] transport=dm-serial uses adapter-specific serial bridge; intended for Damiao motors only"
        );
        println!(
            "[note] dm-serial options: --serial-port {} --serial-baud {}",
            serial_port, serial_baud
        );
    }
    if transport == "dm-device" {
        let note_dm_channel = if mode == "scan" && !args.contains_key("dm-channel") {
            "all".to_string()
        } else {
            dm_channel.clone()
        };
        println!(
            "[note] transport=dm-device uses DM_Device_SDK/libdm_device.so; device_type={} dm_channel={}",
            dm_device_type, note_dm_channel
        );
    }

    if mode == "scan" {
        let start_id = get_u16_hex_or_dec(args, "start-id", 1)?;
        let end_id = get_u16_hex_or_dec(args, "end-id", 255)?;
        if start_id == 0 || end_id == 0 || start_id > 255 || end_id > 255 || start_id > end_id {
            return Err("invalid scan range: expected 1..255 and start<=end".into());
        }
        if transport == "dm-device" {
            let scan_channels = dm_device_scan_channels(args, &dm_device_type)?;
            let channel_label = if args.contains_key("dm-channel") {
                dm_channel.clone()
            } else {
                "all".to_string()
            };
            println!(
                "[scan] dm-device channel target: {}",
                if channel_label == "all" {
                    match DmDeviceType::parse(&dm_device_type).ok() {
                        Some(DmDeviceType::LinkX4C) => "all (0,1,2,3)".to_string(),
                        Some(DmDeviceType::Usb2CanFdDual) => "all (0,1)".to_string(),
                        _ => "all (0)".to_string(),
                    }
                } else {
                    channel_label.clone()
                }
            );
            let hard_exit_after_scan = get_u64(args, "dm-device-hard-exit", 1)? != 0;
            let mut total_hits = 0usize;
            for scan_channel in scan_channels {
                total_hits += scan_damiao_dm_device_channel(
                    &dm_device_type,
                    &scan_channel,
                    model,
                    start_id,
                    end_id,
                    !hard_exit_after_scan,
                )?;
            }
            println!(
                "[scan] done vendor=damiao dm_channel={channel_label} total_hits={total_hits}"
            );
            if hard_exit_after_scan {
                hard_exit(0);
            }
            return Ok(());
        }
        let controller = open_damiao_controller(
            &transport,
            channel,
            &serial_port,
            serial_baud,
            &dm_device_type,
            &dm_channel,
        )?;
        let model_hints = build_scan_model_hints();
        println!(
            "[scan] probing Damiao IDs {}..{} on {}",
            start_id, end_id, channel
        );
        let mut hits = 0usize;
        let mut fallback_hits = 0usize;
        for id in start_id..=end_id {
            let feedback_hints = build_scan_feedback_hints(feedback_id, id);
            enum ScanHit {
                Registers {
                    p: f32,
                    v: f32,
                    t: f32,
                    fid: u16,
                },
                Feedback {
                    fid: u16,
                    status: u8,
                    pos: f32,
                    vel: f32,
                    torq: f32,
                },
            }
            let mut found: Option<ScanHit> = None;
            for fid in &feedback_hints {
                for mh in &model_hints {
                    let Ok(candidate) = controller.add_motor(id, *fid, mh) else {
                        continue;
                    };
                    let pmax = candidate.get_register_f32(21, Duration::from_millis(120));
                    let vmax = candidate.get_register_f32(22, Duration::from_millis(120));
                    let tmax = candidate.get_register_f32(23, Duration::from_millis(120));
                    if let (Ok(p), Ok(v), Ok(t)) = (pmax, vmax, tmax) {
                        found = Some(ScanHit::Registers { p, v, t, fid: *fid });
                        break;
                    }
                }
                if found.is_some() {
                    break;
                }
            }
            if found.is_none() {
                for fid in &feedback_hints {
                    for mh in &model_hints {
                        let Ok(candidate) = controller.add_motor(id, *fid, mh) else {
                            continue;
                        };
                        let _ = candidate.request_motor_feedback();
                        for _ in 0..4 {
                            let _ = controller.poll_feedback_once();
                            if let Some(s) = candidate.latest_state() {
                                found = Some(ScanHit::Feedback {
                                    fid: *fid,
                                    status: s.status_code,
                                    pos: s.pos,
                                    vel: s.vel,
                                    torq: s.torq,
                                });
                                break;
                            }
                            std::thread::sleep(Duration::from_millis(8));
                        }
                        if found.is_some() {
                            break;
                        }
                    }
                    if found.is_some() {
                        break;
                    }
                }
            }
            if let Some(hit) = found {
                match hit {
                    ScanHit::Registers { p, v, t, fid } => {
                        let matched = match_models_by_limits(p, v, t, 0.2);
                        let model_guess = if matched.is_empty() {
                            "unknown".to_string()
                        } else {
                            display_models(&matched).join(",")
                        };
                        println!(
                            "[hit] vendor=damiao id={} feedback_id=0x{:X} model_guess={} limits=({:.3},{:.3},{:.3})",
                            id, fid, model_guess, p, v, t
                        );
                        hits += 1;
                    }
                    ScanHit::Feedback {
                        fid,
                        status,
                        pos,
                        vel,
                        torq,
                    } => {
                        println!(
                            "[hit] vendor=damiao id={} feedback_id=0x{:X} detected_by=feedback status={} pos={:+.3} vel={:+.3} torq={:+.3}",
                            id, fid, status, pos, vel, torq
                        );
                        hits += 1;
                        fallback_hits += 1;
                    }
                }
            }
            std::thread::sleep(Duration::from_millis(2));
        }
        if fallback_hits > 0 {
            println!(
                "[scan] fallback feedback-detection hits={fallback_hits} (register read unavailable on some motors)"
            );
        }
        println!("[scan] done vendor=damiao hits={hits}");
        controller.close_bus()?;
        return Ok(());
    }

    let controller = open_damiao_controller(
        &transport,
        channel,
        &serial_port,
        serial_baud,
        &dm_device_type,
        &dm_channel,
    )?;
    let motor = controller.add_motor(motor_id, feedback_id, model)?;

    if set_motor_id.is_some() || set_feedback_id.is_some() {
        let new_motor_id = set_motor_id.unwrap_or(motor_id);
        let new_feedback_id = set_feedback_id.unwrap_or(feedback_id);
        println!(
            "[id-set] old motor_id=0x{:X} feedback_id=0x{:X} -> new motor_id=0x{:X} feedback_id=0x{:X}",
            motor_id, feedback_id, new_motor_id, new_feedback_id
        );

        if let Some(v) = set_feedback_id {
            motor.write_register_u32(7, v as u32)?;
            println!("[id-set] write rid=7 (MST_ID) = 0x{:X}", v);
        }
        if let Some(v) = set_motor_id {
            motor.write_register_u32(8, v as u32)?;
            println!("[id-set] write rid=8 (ESC_ID) = 0x{:X}", v);
        }
        controller.close_bus()?;

        // Reconnect using NEW IDs before store/verify.
        // Otherwise a store sent via an old-ID handle may be lost.
        if store_after_set || verify_id {
            std::thread::sleep(Duration::from_millis(120));
            let verify_ctrl = open_damiao_controller(
                &transport,
                channel,
                &serial_port,
                serial_baud,
                &dm_device_type,
                &dm_channel,
            )?;
            let verify_motor = verify_ctrl.add_motor(new_motor_id, new_feedback_id, model)?;

            if store_after_set {
                verify_motor.store_parameters()?;
                println!("[id-set] store parameters sent (via new id)");
                std::thread::sleep(Duration::from_millis(120));
            }

            if verify_id {
                let esc = verify_motor.get_register_u32(8, Duration::from_millis(1000))?;
                let mst = verify_motor.get_register_u32(7, Duration::from_millis(1000))?;
                println!("[id-set] verify rid=8 (ESC_ID)=0x{:X}", esc);
                println!("[id-set] verify rid=7 (MST_ID)=0x{:X}", mst);
                if esc != new_motor_id as u32 || mst != new_feedback_id as u32 {
                    verify_ctrl.close_bus()?;
                    return Err(format!(
                        "id verify failed: expected ESC_ID=0x{:X}, MST_ID=0x{:X}, got ESC_ID=0x{:X}, MST_ID=0x{:X}",
                        new_motor_id, new_feedback_id, esc, mst
                    )
                    .into());
                }
                println!("[id-set] verify ok");
            }
            verify_ctrl.close_bus()?;
        }
        return Ok(());
    }

    if mode == "read-param" || mode == "write-param" {
        let param_id = get_u16_hex_or_dec(args, "param-id", 0)?;
        let rid = u8::try_from(param_id)
            .map_err(|_| format!("Damiao --param-id must fit in u8, got 0x{param_id:X}"))?;
        let param_type = damiao_param_type(args);
        let timeout = Duration::from_millis(get_u64(args, "timeout-ms", 500)?);
        match mode.as_str() {
            "read-param" => match param_type.as_str() {
                "u32" => {
                    let value = motor.get_register_u32(rid, timeout)?;
                    println!("param 0x{param_id:04X} (u32) = {value}");
                }
                "f32" => {
                    let value = motor.get_register_f32(rid, timeout)?;
                    println!("param 0x{param_id:04X} (f32) = {value:.6}");
                }
                _ => {
                    return Err(
                        format!("Damiao --type must be u32 or f32, got {param_type}").into(),
                    )
                }
            },
            "write-param" => {
                let raw = args
                    .get("param-value")
                    .ok_or_else(|| "missing --param-value".to_string())?;
                let verify = get_u64(args, "verify", 1)? != 0;
                let store_after_write = get_u64(args, "store", 0)? != 0;
                match param_type.as_str() {
                    "u32" => {
                        let value = if let Some(hex) = raw.strip_prefix("0x") {
                            u32::from_str_radix(hex, 16)
                                .map_err(|e| format!("invalid --param-value: {e}"))?
                        } else {
                            raw.parse::<u32>()
                                .map_err(|e| format!("invalid --param-value: {e}"))?
                        };
                        motor.write_register_u32(rid, value)?;
                        if verify {
                            let readback = motor.get_register_u32(rid, timeout)?;
                            println!("param 0x{param_id:04X} (u32) = {readback}");
                        }
                    }
                    "f32" => {
                        let value = raw
                            .parse::<f32>()
                            .map_err(|e| format!("invalid --param-value: {e}"))?;
                        motor.write_register_f32(rid, value)?;
                        if verify {
                            let readback = motor.get_register_f32(rid, timeout)?;
                            println!("param 0x{param_id:04X} (f32) = {readback:.6}");
                        }
                    }
                    _ => {
                        return Err(
                            format!("Damiao --type must be u32 or f32, got {param_type}").into(),
                        )
                    }
                }
                if store_after_write {
                    motor.store_parameters()?;
                    println!("[ok] store-parameters requested (store=1)");
                }
            }
            _ => unreachable!(),
        }
        controller.close_bus()?;
        return Ok(());
    }

    if verify_model {
        verify_declared_damiao_model(
            &motor,
            model,
            Duration::from_millis(verify_timeout_ms),
            verify_tol,
        )
        .map_err(|e| e.to_string())?;
    }
    if mode != "enable" && mode != "disable" {
        controller.enable_all()?;
        std::thread::sleep(Duration::from_millis(200));
    }

    if ensure_mode && mode != "enable" && mode != "disable" {
        let cm = match mode.as_str() {
            "mit" => DamiaoControlMode::Mit,
            "pos-vel" => DamiaoControlMode::PosVel,
            "vel" => DamiaoControlMode::Vel,
            "force-pos" => DamiaoControlMode::ForcePos,
            _ => return Err(format!("unknown Damiao mode: {mode}").into()),
        };
        if let Err(e) = motor.ensure_control_mode(cm, Duration::from_millis(1000)) {
            eprintln!("[warn] ensure_mode failed: {e}");
        }
    }

    let mit_pos = get_f32(args, "pos", 0.0)?;
    let mit_vel = get_f32(args, "vel", 0.0)?;
    let mit_kp = get_f32(args, "kp", 2.0)?;
    let mit_kd = get_f32(args, "kd", 1.0)?;
    let mit_tau = get_f32(args, "tau", 0.0)?;
    let pos_target = get_f32(args, "pos", 0.0)?;
    let vel_limit = get_f32(args, "vlim", 1.0)?;
    let force_ratio = get_f32(args, "ratio", 0.1)?;

    for i in 0..loop_n {
        match mode.as_str() {
            "enable" => {
                motor.enable()?;
                let _ = motor.request_motor_feedback();
            }
            "disable" => {
                motor.disable()?;
                let _ = motor.request_motor_feedback();
            }
            "mit" => {
                motor.send_cmd_mit(mit_pos, mit_vel, mit_kp, mit_kd, mit_tau)?;
            }
            "pos-vel" => {
                motor.send_cmd_pos_vel(pos_target, vel_limit)?;
            }
            "vel" => {
                motor.send_cmd_vel(mit_vel)?;
            }
            "force-pos" => {
                motor.send_cmd_force_pos(pos_target, vel_limit, force_ratio)?;
            }
            _ => return Err(format!("unknown Damiao mode: {mode}").into()),
        }

        if mode == "enable" || mode == "disable" {
            for _ in 0..20 {
                let _ = controller.poll_feedback_once();
                if motor.latest_state().is_some() {
                    break;
                }
                std::thread::sleep(Duration::from_millis(5));
            }
        }

        if let Some(s) = motor.latest_state() {
            let base = format!(
                "#{i} id={} arb=0x{:03X} pos={:+.3} vel={:+.3} torq={:+.3} status={}({}) t_mos={:.1}C t_rotor={:.1}C",
                s.can_id, s.arbitration_id, s.pos, s.vel, s.torq, s.status_code, s.status_name, s.t_mos, s.t_rotor
            );
            match mode.as_str() {
                "mit" => {
                    println!(
                        "{} cmd_pos={:+.3} cmd_vel={:+.3} kp={:.3} kd={:.3} cmd_tau={:+.3} e_pos={:+.3} e_vel={:+.3}",
                        base,
                        mit_pos,
                        mit_vel,
                        mit_kp,
                        mit_kd,
                        mit_tau,
                        mit_pos - s.pos,
                        mit_vel - s.vel
                    );
                }
                "pos-vel" => {
                    println!(
                        "{} cmd_pos={:+.3} vlim={:.3} e_pos={:+.3}",
                        base,
                        pos_target,
                        vel_limit,
                        pos_target - s.pos
                    );
                }
                "vel" => {
                    println!(
                        "{} cmd_vel={:+.3} e_vel={:+.3}",
                        base,
                        mit_vel,
                        mit_vel - s.vel
                    );
                }
                "force-pos" => {
                    println!(
                        "{} cmd_pos={:+.3} vlim={:.3} ratio={:.3} e_pos={:+.3}",
                        base,
                        pos_target,
                        vel_limit,
                        force_ratio,
                        pos_target - s.pos
                    );
                }
                _ => println!("{base}"),
            }
        } else if mode == "enable" || mode == "disable" {
            println!(
                "[ok] Damiao {} command sent to motor_id=0x{:X} feedback_id=0x{:X}; no feedback frame observed within 100ms",
                mode, motor_id, feedback_id
            );
        }
        std::thread::sleep(Duration::from_millis(dt_ms));
    }

    if mode == "enable" || mode == "disable" {
        controller.close_bus()?;
    } else {
        controller.shutdown()?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::{build_scan_feedback_hints, build_scan_model_hints};

    #[test]
    fn scan_model_hints_are_unique() {
        let hints = build_scan_model_hints();
        assert!(!hints.is_empty());
        let count_4310 = hints.iter().filter(|m| m.as_str() == "4310").count();
        assert_eq!(count_4310, 1);
        assert!(hints.iter().any(|m| m == "4340P"));
    }

    #[test]
    fn scan_feedback_hints_include_common_ids() {
        let fids = build_scan_feedback_hints(0x0017, 0x0007);
        assert!(fids.contains(&0x0011));
        assert!(fids.contains(&0x0017));
        assert_eq!(fids[0], 0x0017);
    }
}
