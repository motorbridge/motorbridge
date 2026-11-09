use crate::args::{get_f32, get_opt_u16_hex_or_dec, get_str, get_u16_hex_or_dec, get_u64};
use motor_core::bus::{open_transport, CanBus, Transport, TransportParams};
use motor_core::error::MotorError;
use motor_vendor_fashionstar::{FashionstarController, CAN_ID};
use std::collections::HashMap;
use std::convert::TryFrom;
use std::sync::Arc;
use std::time::Duration;

fn open_fashionstar_controller(
    transport: &str,
    channel: &str,
    serial_port: &str,
    serial_baud: u32,
) -> Result<FashionstarController, Box<dyn std::error::Error>> {
    let p = TransportParams {
        channel,
        serial_port,
        serial_baud,
    };
    let bus: Arc<dyn CanBus> = match transport {
        "auto" | "socketcan" => open_transport(Transport::SocketCan, &p)?,
        "mcu-serial" => open_transport(Transport::McuSerial, &p)?,
        "socketcanfd" => {
            return Err(
                "transport socketcanfd unsupported (fashionstar uses standard CAN only)".into(),
            )
        }
        "dm-serial" | "dm-device" => {
            return Err(format!(
                "transport {transport} is damiao-only (fashionstar supports auto|socketcan|mcu-serial)"
            )
            .into());
        }
        _ => {
            return Err(format!(
                "unknown FashionStar transport: {transport} (expected auto|socketcan|mcu-serial)"
            )
            .into());
        }
    };
    Ok(FashionstarController::new(bus))
}

pub fn run_fashionstar(
    args: &HashMap<String, String>,
    channel: &str,
    model: &str,
    motor_id: u16,
    feedback_id: u16,
) -> Result<(), Box<dyn std::error::Error>> {
    let mode = get_str(args, "mode", "status");
    let loop_n = get_u64(args, "loop", 1)?;
    let dt_ms = get_u64(args, "dt-ms", 20)?;
    let print_state = get_u64(args, "print-state", 0)? != 0;
    let set_motor_id = get_opt_u16_hex_or_dec(args, "set-motor-id")?;
    if set_motor_id.is_none()
        && !matches!(
            mode.as_str(),
            "scan"
                | "enable"
                | "disable"
                | "stop"
                | "zero"
                | "set-zero"
                | "pos"
                | "pos-vel"
                | "status"
                | "read"
                | "ping"
                | "read-param"
                | "write-param"
        )
    {
        return Err(format!("unknown FashionStar mode: {mode}").into());
    }
    if args.contains_key("set-feedback-id") {
        return Err("FashionStar CAN arbitration ID is fixed at 0x321".into());
    }
    if matches!(mode.as_str(), "read-param" | "write-param")
        && !matches!(get_str(args, "param-type", "u32").as_str(), "u16" | "u32")
    {
        return Err("FashionStar --param-type must be u16 or u32".into());
    }

    let transport = get_str(args, "transport", "auto");
    let serial_port = get_str(args, "serial-port", "/dev/ttyACM0");
    let serial_baud_u64 = get_u64(args, "serial-baud", 921600)?;
    let serial_baud = u32::try_from(serial_baud_u64)
        .map_err(|_| format!("invalid --serial-baud (too large): {serial_baud_u64}"))?;
    let timeout_ms = get_u64(args, "timeout-ms", 500)?;
    let timeout = Duration::from_millis(timeout_ms);

    if mode == "scan" {
        let start_id = get_u16_hex_or_dec(args, "start-id", 0)?;
        let mut end_id = get_u16_hex_or_dec(args, "end-id", 254)?;
        if get_str(args, "vendor", "fashionstar") == "all" {
            end_id = end_id.min(254);
            if start_id > end_id {
                return Ok(());
            }
        }
        if start_id > end_id || end_id > 254 {
            return Err("invalid scan range: expected 0..254 and start<=end".into());
        }
        let controller =
            open_fashionstar_controller(&transport, channel, &serial_port, serial_baud)?;
        let mut hits = 0usize;
        for id in start_id..=end_id {
            let motor = controller.add_motor(id, CAN_ID, model)?;
            match motor.get_register_u32(1, timeout) {
                Ok(model_code) => {
                    hits += 1;
                    println!("[hit] vendor=fashionstar probe=0x{id:02X} feedback_id=0x321 model_code={model_code}");
                }
                Err(MotorError::Timeout(_)) => {}
                Err(e) => return Err(e.into()),
            }
        }
        println!("[scan] done vendor=fashionstar hits={hits}");
        controller.close_bus()?;
        return Ok(());
    }

    let controller = open_fashionstar_controller(&transport, channel, &serial_port, serial_baud)?;
    let motor = controller.add_motor(motor_id, feedback_id, model)?;
    if let Some(new_motor_id) = set_motor_id {
        motor.write_register_u32(0x81, u32::from(new_motor_id))?;
        println!("[id-set] FashionStar device id update requested: {motor_id} -> {new_motor_id}; power-cycle the servo before connecting with the new ID");
        controller.close_bus()?;
        return Ok(());
    }

    for i in 0..loop_n {
        match mode.as_str() {
            "enable" => motor.enable()?,
            "disable" => motor.disable()?,
            "stop" => motor.stop()?,
            "zero" | "set-zero" => motor.set_zero_position()?,
            "pos" | "pos-vel" => {
                motor.send_cmd_pos_vel(get_f32(args, "pos", 0.0)?, get_f32(args, "vlim", 1.0)?)?
            }
            "status" | "read" => motor.request_motor_feedback(timeout)?,
            "ping" => println!("model_code={}", motor.get_register_u32(1, timeout)?),
            "read-param" => {
                let param_id = get_u16_hex_or_dec(args, "param-id", 0x14)?;
                let rid = u8::try_from(param_id).map_err(|_| {
                    format!("FashionStar --param-id must fit in u8, got 0x{param_id:X}")
                })?;
                let value = motor.get_register_u32(rid, timeout)?;
                println!("param 0x{param_id:04X} (u32) = {value}");
            }
            "write-param" => {
                let param_id = get_u16_hex_or_dec(args, "param-id", 0)?;
                let rid = u8::try_from(param_id).map_err(|_| {
                    format!("FashionStar --param-id must fit in u8, got 0x{param_id:X}")
                })?;
                let raw = args
                    .get("param-value")
                    .ok_or_else(|| "missing --param-value".to_string())?;
                let value = if let Some(hex) = raw.strip_prefix("0x") {
                    u32::from_str_radix(hex, 16)
                        .map_err(|e| format!("invalid --param-value: {e}"))?
                } else {
                    raw.parse::<u32>()
                        .map_err(|e| format!("invalid --param-value: {e}"))?
                };
                motor.write_register_u32(rid, value)?;
                println!("param 0x{param_id:04X} (u32) = {value}; configuration changes take effect after a power cycle");
            }
            _ => return Err(format!("unknown FashionStar mode: {mode}").into()),
        }
        if print_state && !matches!(mode.as_str(), "status" | "read") {
            motor.request_motor_feedback(timeout)?;
        }
        if let Some(s) = motor.latest_state() {
            println!(
                "#{i} pos={:+.5} vel={:+.3} current={:.3}A temp={:.1}C status=0x{:02X}",
                s.pos, s.vel, s.current_a, s.temperature_c, s.status_code
            );
        }
        if i + 1 < loop_n {
            std::thread::sleep(Duration::from_millis(dt_ms));
        }
    }
    controller.close_bus()?;
    Ok(())
}
