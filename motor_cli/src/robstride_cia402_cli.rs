use crate::args::{get_f32, get_str, get_u16_hex_or_dec, get_u64};
use motor_core::bus::{open_transport, CanBus, Transport, TransportParams};
use motor_vendor_robstride_cia402::{
    model_limits as robstride_cia402_model_limits, watchdog_seconds_to_raw,
    RobstrideCia402Controller, PROTOCOL_CANOPEN, PROTOCOL_MIT, PROTOCOL_PRIVATE,
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

/// Open a RobstrideCia402 controller for the requested transport. Universal
/// transports route through `open_transport` (core); damiao-only transports
/// are rejected. Robstride-CiA402 is classic+FD CAN, so mcu-serial is allowed.
fn open_robstride_cia402_controller(
    transport: &str,
    channel: &str,
    serial_port: &str,
    serial_baud: u32,
) -> Result<RobstrideCia402Controller, Box<dyn std::error::Error>> {
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
                "transport {transport} is damiao-only (robstride-cia402 supports auto|socketcan|socketcanfd|mcu-serial)"
            )
            .into());
        }
        _ => {
            return Err(format!(
                "unknown Robstride-CiA402 transport: {transport} (expected auto|socketcan|socketcanfd|mcu-serial)"
            )
            .into());
        }
    };
    Ok(RobstrideCia402Controller::new(bus))
}

pub fn run_robstride_cia402(
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

    if feedback_id != 0 {
        println!(
            "[info] robstride_cia402 ignores feedback_id=0x{feedback_id:X}; CANopen uses 0x600/0x580/0x700 + node_id"
        );
    }
    if let Some((pmax, vmax, tmax)) = robstride_cia402_model_limits(model) {
        println!(
            "[info] robstride_cia402 model {model} limits pmax={pmax:.3} vmax={vmax:.3} tmax={tmax:.3}"
        );
    }

    if mode == "scan" {
        let start_id = get_u16_hex_or_dec(args, "start-id", 1)?;
        let end_id = get_u16_hex_or_dec(args, "end-id", 127)?;
        let controller =
            open_robstride_cia402_controller(&transport, channel, &serial_port, serial_baud)?;
        println!(
            "[scan:robstride_cia402] channel={channel} model={model} id_range=[0x{start_id:X},0x{end_id:X}] timeout_ms={timeout_ms}"
        );
        let hits = controller.scan_ids(start_id, end_id, timeout)?;
        for hit in &hits {
            println!(
                "[hit] vendor=robstride_cia402 node={} statusword={:?} mode_display={:?} error_code={:?}",
                hit.node_id, hit.statusword, hit.mode_display, hit.error_code
            );
        }
        println!("[scan] done vendor=robstride_cia402 hits={}", hits.len());
        controller.close_bus()?;
        return Ok(());
    }

    let controller =
        open_robstride_cia402_controller(&transport, channel, &serial_port, serial_baud)?;

    if mode == "set-protocol" || mode == "protocol-switch" {
        let protocol = parse_protocol_cmd(&get_str(args, "protocol", "canopen"))?;
        println!(
            "[protocol-switch:robstride_cia402] send ext_id=0xFFF protocol_cmd={} (0=private,1=canopen,2=mit)",
            protocol
        );
        let reply = controller.switch_protocol(protocol, timeout)?;
        match reply {
            Some((reply_id, unique_id)) => {
                println!(
                    "[ok] protocol switch ack id={} unique_id={:02X?}; power-cycle motor to apply",
                    reply_id, unique_id
                );
            }
            None => {
                println!("[warn] protocol switch frame sent, no ack before timeout; power-cycle motor if the drive accepted it");
            }
        }
        controller.close_bus()?;
        return Ok(());
    }

    let motor = controller.add_motor(motor_id, feedback_id, model)?;

    match mode.as_str() {
        "status" => {
            let status = motor.query_status(timeout)?;
            println!(
                "[status] node={} mode={} statusword=0x{:04X} error=0x{:04X} pos={:+.6}rad vel={:+.6}rad/s torque={:+.3}Nm current={}mA vbus={}mV heartbeat={:?}",
                motor_id,
                status.mode_display,
                status.statusword,
                status.error_code,
                status.position_rad,
                status.velocity_rad_s,
                status.torque_nm,
                status.current_ma,
                status.dc_link_mv,
                status.heartbeat_state
            );
            controller.close_bus()?;
            return Ok(());
        }
        "enable" => {
            motor.enable_drive(timeout)?;
            println!("[ok] robstride_cia402 enable sent");
            controller.close_bus()?;
            return Ok(());
        }
        "disable" => {
            motor.disable_drive(timeout)?;
            println!("[ok] robstride_cia402 disable sent");
            controller.close_bus()?;
            return Ok(());
        }
        "quick-stop" | "emergency-stop" => {
            motor.quick_stop(timeout)?;
            println!("[ok] robstride_cia402 quick-stop sent (controlword=11; use carefully)");
            controller.close_bus()?;
            return Ok(());
        }
        "clear-error" | "clear-fault" => {
            motor.clear_fault(timeout)?;
            println!("[ok] robstride_cia402 fault reset sent");
            controller.close_bus()?;
            return Ok(());
        }
        "zero" | "set-zero" => {
            motor.set_current_position_zero(timeout)?;
            println!("[ok] robstride_cia402 zero sent: 6060=6 while disabled, then controlword=15");
            controller.close_bus()?;
            return Ok(());
        }
        "watchdog" => {
            let watchdog_s = get_f32(args, "watchdog-s", 0.0)?;
            let default_raw = watchdog_seconds_to_raw(watchdog_s) as u64;
            let raw = get_u64(args, "watchdog-raw", default_raw)?;
            if raw > u32::MAX as u64 {
                controller.close_bus()?;
                return Err(
                    format!("invalid --watchdog-raw {raw}, expected <= {}", u32::MAX).into(),
                );
            }
            motor.set_can_watchdog_raw(raw as u32, timeout)?;
            println!(
                "[ok] robstride_cia402 watchdog set raw={} (manual: 20000 means 1s, 0 disables)",
                raw
            );
            controller.close_bus()?;
            return Ok(());
        }
        "get-protocol" | "protocol" | "query-protocol" => {
            let value = motor.sdo_read_i8(0x2022, 0x00, timeout)?;
            println!(
                "[protocol] vendor=robstride_cia402 current={} ({}) note=read vendor object 0x2022 by SDO",
                value,
                match value as u8 {
                    PROTOCOL_PRIVATE => "private",
                    PROTOCOL_CANOPEN => "canopen",
                    PROTOCOL_MIT => "mit",
                    _ => "unknown",
                }
            );
            controller.close_bus()?;
            return Ok(());
        }
        _ => {}
    }

    for i in 0..loop_n {
        match mode.as_str() {
            "pos-vel" => {
                if i == 0 {
                    println!(
                        "[info] robstride_cia402 pos-vel maps to PP: 6060=1, 6071, 6081, 6083, 607A"
                    );
                }
                if args.contains_key("position-window")
                    || args.contains_key("position-window-time-ms")
                {
                    motor.command_profile_position_full(
                        get_f32(args, "pos", 0.0)?,
                        get_f32(args, "vlim", 1.0)?,
                        get_f32(args, "acc", 10.0)?,
                        if args.contains_key("position-window") {
                            Some(get_f32(args, "position-window", 0.0)?)
                        } else {
                            None
                        },
                        if args.contains_key("position-window-time-ms") {
                            Some(get_u16_hex_or_dec(args, "position-window-time-ms", 0)?)
                        } else {
                            None
                        },
                        timeout,
                    )?;
                } else {
                    motor.command_profile_position(
                        get_f32(args, "pos", 0.0)?,
                        get_f32(args, "vlim", 1.0)?,
                        get_f32(args, "acc", 10.0)?,
                        timeout,
                    )?;
                }
            }
            "vel" => {
                if i == 0 {
                    println!("[info] robstride_cia402 vel maps to Profile Velocity: 6060=3, 60FF");
                }
                motor.command_velocity(get_f32(args, "vel", 0.0)?, timeout)?;
            }
            "torque" => {
                if i == 0 {
                    println!("[info] robstride_cia402 torque maps to Torque mode: 6060=4, 6071");
                }
                motor.command_torque(get_f32(args, "tau", 0.0)?, timeout)?;
            }
            "mit" => {
                if i == 0 {
                    println!(
                        "[warn] robstride_cia402 has no true MIT object path; mapping to CSP: 6060=5, 6071, 6081, 607A; kp/kd ignored"
                    );
                }
                let tau = get_f32(args, "tau", 0.0)?.abs();
                let torque_limit = if tau > 0.0 { tau } else { 5.0 };
                if args.contains_key("position-window")
                    || args.contains_key("position-window-time-ms")
                {
                    motor.command_csp_full(
                        get_f32(args, "pos", 0.0)?,
                        get_f32(args, "vel", 1.0)?,
                        torque_limit,
                        if args.contains_key("position-window") {
                            Some(get_f32(args, "position-window", 0.0)?)
                        } else {
                            None
                        },
                        if args.contains_key("position-window-time-ms") {
                            Some(get_u16_hex_or_dec(args, "position-window-time-ms", 0)?)
                        } else {
                            None
                        },
                        timeout,
                    )?;
                } else {
                    motor.command_csp(
                        get_f32(args, "pos", 0.0)?,
                        get_f32(args, "vel", 1.0)?,
                        torque_limit,
                        timeout,
                    )?;
                }
            }
            _ => {
                controller.close_bus()?;
                return Err(format!("unknown robstride_cia402 mode: {mode}").into());
            }
        }
        if loop_n > 1 {
            std::thread::sleep(Duration::from_millis(dt_ms));
        }
    }

    controller.shutdown()?;
    Ok(())
}
