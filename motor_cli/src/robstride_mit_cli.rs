use crate::args::{get_f32, get_str, get_u16_hex_or_dec, get_u64};
use motor_core::bus::{open_transport, CanBus, Transport, TransportParams};
use motor_vendor_robstride_mit::{
    model_limits as robstride_mit_model_limits, RobstrideMitController, MODE_MIT, MODE_POSITION,
    MODE_VELOCITY, PROTOCOL_CANOPEN, PROTOCOL_MIT, PROTOCOL_PRIVATE,
};
use std::collections::HashMap;
use std::sync::Arc;
use std::time::Duration;

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

fn parse_mode_cmd(raw: &str) -> Result<u8, String> {
    match raw.to_ascii_lowercase().as_str() {
        "0" | "mit" => Ok(MODE_MIT),
        "1" | "pos" | "position" | "csp" | "pos-vel" => Ok(MODE_POSITION),
        "2" | "vel" | "velocity" => Ok(MODE_VELOCITY),
        _ => Err(format!(
            "invalid --run-mode {raw}, expected mit|position|velocity or 0|1|2"
        )),
    }
}

fn parse_param_value(args: &HashMap<String, String>) -> Result<[u8; 4], String> {
    let raw = get_str(args, "param-value", "0");
    let ty = get_str(args, "param-type", &get_str(args, "type", "f32"));
    match ty.as_str() {
        "f32" | "float" => Ok(raw
            .parse::<f32>()
            .map_err(|e| format!("invalid --param-value f32: {e}"))?
            .to_le_bytes()),
        "u8" => Ok([
            raw.parse::<u8>()
                .map_err(|e| format!("invalid --param-value u8: {e}"))?,
            0,
            0,
            0,
        ]),
        "i8" => Ok([
            raw.parse::<i8>()
                .map_err(|e| format!("invalid --param-value i8: {e}"))? as u8,
            0,
            0,
            0,
        ]),
        "u16" => {
            let b = raw
                .parse::<u16>()
                .map_err(|e| format!("invalid --param-value u16: {e}"))?
                .to_le_bytes();
            Ok([b[0], b[1], 0, 0])
        }
        "i16" => {
            let b = raw
                .parse::<i16>()
                .map_err(|e| format!("invalid --param-value i16: {e}"))?
                .to_le_bytes();
            Ok([b[0], b[1], 0, 0])
        }
        "u32" => Ok(raw
            .parse::<u32>()
            .map_err(|e| format!("invalid --param-value u32: {e}"))?
            .to_le_bytes()),
        "i32" => Ok(raw
            .parse::<i32>()
            .map_err(|e| format!("invalid --param-value i32: {e}"))?
            .to_le_bytes()),
        other => Err(format!(
            "invalid --param-type {other}, expected f32|u8|i8|u16|i16|u32|i32"
        )),
    }
}

fn print_feedback(prefix: &str, feedback: motor_vendor_robstride_mit::MitFeedback) {
    println!(
        "{prefix} id={} mode_state={} fault={} warn={} pos={:+.6}rad vel={:+.6}rad/s torque={:+.3}Nm temp={:.1}C",
        feedback.motor_id,
        feedback.mode_state,
        feedback.has_fault,
        feedback.has_warning,
        feedback.position_rad,
        feedback.velocity_rad_s,
        feedback.torque_nm,
        feedback.winding_temp_c
    );
}

/// Open a RobstrideMit controller for the requested transport. Universal
/// transports route through `open_transport` (core); damiao-only transports
/// are rejected. Robstride-MIT is classic+FD CAN, so mcu-serial is allowed.
fn open_robstride_mit_controller(
    transport: &str,
    channel: &str,
    serial_port: &str,
    serial_baud: u32,
) -> Result<RobstrideMitController, Box<dyn std::error::Error>> {
    let p = TransportParams {
        channel,
        serial_port,
        serial_baud,
    };
    let bus: Arc<dyn CanBus> = match transport {
        "auto" | "socketcan" => open_transport(Transport::SocketCan, &p)?,
        "socketcanfd" => open_transport(Transport::SocketCanFd, &p)?,
        "mcu-serial" => open_transport(Transport::McuSerial, &p)?,
        "dm-serial" | "dm-device" => {
            return Err(format!(
                "transport {transport} is damiao-only (robstride-mit supports auto|socketcan|socketcanfd|mcu-serial)"
            )
            .into());
        }
        _ => {
            return Err(format!(
                "unknown Robstride-MIT transport: {transport} (expected auto|socketcan|socketcanfd|mcu-serial)"
            )
            .into());
        }
    };
    Ok(RobstrideMitController::new(bus))
}

pub fn run_robstride_mit(
    args: &HashMap<String, String>,
    channel: &str,
    model: &str,
    motor_id: u16,
    feedback_id: u16,
) -> Result<(), Box<dyn std::error::Error>> {
    let mode = get_str(args, "mode", "status");
    let loop_n = get_u64(args, "loop", 1)?;
    let dt_ms = get_u64(args, "dt-ms", 20)?;
    let timeout_ms = get_u64(args, "timeout-ms", 300)?;
    let timeout = Duration::from_millis(timeout_ms);
    let transport = get_str(args, "transport", "auto");
    let serial_port = get_str(args, "serial-port", "/dev/ttyACM0");
    let serial_baud_u64 = get_u64(args, "serial-baud", 921600)?;
    let serial_baud = u32::try_from(serial_baud_u64)
        .map_err(|_| format!("invalid --serial-baud (too large): {serial_baud_u64}"))?;

    if let Some((pmax, vmax, tmax)) = robstride_mit_model_limits(model) {
        println!(
            "[info] robstride_mit model {model} protocol limits pmax={pmax:.3} vmax={vmax:.3} tmax={tmax:.3}"
        );
    }

    if mode == "scan" {
        let start_id = get_u16_hex_or_dec(args, "start-id", 1)?;
        let end_id = get_u16_hex_or_dec(args, "end-id", 127)?;
        let controller =
            open_robstride_mit_controller(&transport, channel, &serial_port, serial_baud)?;
        println!(
            "[scan:robstride_mit] channel={channel} model={model} host_id=0x{feedback_id:X} id_range=[0x{start_id:X},0x{end_id:X}] timeout_ms={timeout_ms}"
        );
        let hits = controller.scan_ids(start_id, end_id, feedback_id, timeout)?;
        for hit in &hits {
            println!(
                "[hit] vendor=robstride_mit node={} host_id=0x{:X} fault_code={:?}",
                hit.node_id, hit.host_id, hit.fault_code
            );
        }
        println!("[scan] done vendor=robstride_mit hits={}", hits.len());
        controller.close_bus()?;
        return Ok(());
    }

    let controller = open_robstride_mit_controller(&transport, channel, &serial_port, serial_baud)?;
    let motor = controller.add_motor(motor_id, feedback_id, model)?;

    match mode.as_str() {
        "status" | "fault" => {
            let status = motor.query_status(timeout)?;
            println!(
                "[status] node={} fault_code={:?}",
                motor_id, status.fault_code
            );
            if let Some(feedback) = status.feedback {
                print_feedback("[latest-feedback]", feedback);
            }
            controller.close_bus()?;
            return Ok(());
        }
        "enable" => {
            print_feedback("[ok] enable", motor.enable_drive(timeout)?);
            controller.close_bus()?;
            return Ok(());
        }
        "disable" | "stop" => {
            print_feedback("[ok] stop", motor.disable_drive(timeout)?);
            controller.close_bus()?;
            return Ok(());
        }
        "zero" | "set-zero" => {
            print_feedback("[ok] zero", motor.set_current_position_zero(timeout)?);
            controller.close_bus()?;
            return Ok(());
        }
        "clear-error" | "clear-fault" => {
            print_feedback("[ok] clear fault", motor.clear_fault(timeout)?);
            controller.close_bus()?;
            return Ok(());
        }
        "set-mode" => {
            let run_mode = parse_mode_cmd(&get_str(args, "run-mode", "mit"))?;
            print_feedback("[ok] set mode", motor.set_mode(run_mode, timeout)?);
            controller.close_bus()?;
            return Ok(());
        }
        "set-can-id" => {
            let new_id = get_u16_hex_or_dec(args, "set-motor-id", motor_id)?;
            let reply = motor.set_can_id(new_id as u8, timeout)?;
            println!(
                "[ok] set CAN id old={} new={} unique_id={:02X?}",
                motor_id, new_id, reply
            );
            controller.close_bus()?;
            return Ok(());
        }
        "set-host-id" => {
            let host_id = get_u16_hex_or_dec(args, "set-host-id", feedback_id)?;
            let reply = motor.set_host_id(host_id as u8, timeout)?;
            println!("[ok] set host id new=0x{host_id:X} unique_id={reply:02X?}");
            controller.close_bus()?;
            return Ok(());
        }
        "set-protocol" | "protocol-switch" => {
            let protocol = parse_protocol_cmd(&get_str(args, "protocol", "mit"))?;
            let reply = motor.set_protocol(protocol, timeout)?;
            println!(
                "[ok] protocol switch cmd={} unique_id={:02X?}; power-cycle motor to apply",
                protocol, reply
            );
            controller.close_bus()?;
            return Ok(());
        }
        "save" => {
            let reply = motor.save(timeout)?;
            println!("[ok] save sent unique_id={reply:02X?}");
            controller.close_bus()?;
            return Ok(());
        }
        "active-report" => {
            let enable = get_u64(args, "active-report", 1)? != 0;
            print_feedback(
                "[ok] active report",
                motor.set_active_report(enable, timeout)?,
            );
            controller.close_bus()?;
            return Ok(());
        }
        "read-param" => {
            let index = get_u16_hex_or_dec(args, "param-id", 0)?;
            let raw = motor.read_parameter(index, timeout)?;
            println!(
                "[param] index=0x{index:04X} raw={:02X?} u32={} i32={} f32={}",
                raw,
                u32::from_le_bytes(raw),
                i32::from_le_bytes(raw),
                f32::from_le_bytes(raw)
            );
            controller.close_bus()?;
            return Ok(());
        }
        "get-protocol" | "protocol" | "query-protocol" => {
            let raw = motor.read_parameter(0x2022, timeout)?;
            let value = raw[0];
            println!(
                "[protocol] vendor=robstride_mit current={} ({}) note=read protocol_1 param 0x2022",
                value,
                match value {
                    PROTOCOL_PRIVATE => "private",
                    PROTOCOL_CANOPEN => "canopen",
                    PROTOCOL_MIT => "mit",
                    _ => "unknown",
                }
            );
            controller.close_bus()?;
            return Ok(());
        }
        "write-param" => {
            let index = get_u16_hex_or_dec(args, "param-id", 0)?;
            let value = parse_param_value(args)?;
            let echoed = motor.write_parameter(index, value, timeout)?;
            println!(
                "[param] wrote index=0x{index:04X} raw={:02X?} echoed={:02X?}",
                value, echoed
            );
            controller.close_bus()?;
            return Ok(());
        }
        _ => {}
    }

    for i in 0..loop_n {
        match mode.as_str() {
            "mit" => {
                if i == 0 {
                    println!(
                        "[info] robstride_mit MIT dynamic command: standard id=motor_id, packed pos/vel/kp/kd/tau"
                    );
                }
                let feedback = motor.command_mit(
                    get_f32(args, "pos", 0.0)?,
                    get_f32(args, "vel", 0.0)?,
                    get_f32(args, "kp", 8.0)?,
                    get_f32(args, "kd", 0.2)?,
                    get_f32(args, "tau", 0.0)?,
                    timeout,
                )?;
                if loop_n == 1 || i + 1 == loop_n {
                    print_feedback("[feedback]", feedback);
                }
            }
            "pos" | "pos-vel" => {
                if i == 0 {
                    println!(
                        "[info] robstride_mit position command: id=(1<<8)|motor_id, payload=float pos + float vel"
                    );
                }
                let feedback = motor.command_position(
                    get_f32(args, "pos", 0.0)?,
                    get_f32(args, "vlim", get_f32(args, "vel", 1.0)?)?,
                    timeout,
                )?;
                if loop_n == 1 || i + 1 == loop_n {
                    print_feedback("[feedback]", feedback);
                }
            }
            "vel" => {
                if i == 0 {
                    println!(
                        "[info] robstride_mit velocity command: id=(2<<8)|motor_id, payload=float vel + float current_limit"
                    );
                }
                let feedback = motor.command_velocity(
                    get_f32(args, "vel", 0.0)?,
                    get_f32(args, "current", get_f32(args, "ilim", 2.0)?)?,
                    timeout,
                )?;
                if loop_n == 1 || i + 1 == loop_n {
                    print_feedback("[feedback]", feedback);
                }
            }
            _ => {
                controller.close_bus()?;
                return Err(format!("unknown robstride_mit mode: {mode}").into());
            }
        }
        if loop_n > 1 {
            std::thread::sleep(Duration::from_millis(dt_ms));
        }
    }

    controller.shutdown()?;
    Ok(())
}
