use crate::args::{get_f32, get_str, get_u16_hex_or_dec, get_u64};
use motor_core::bus::{open_transport, CanBus, Transport, TransportParams};
use motor_vendor_huayi::HuayiController;
use std::collections::HashMap;
use std::sync::Arc;
use std::time::Duration;

/// Open a Huayi controller for the requested transport. Huayi is classic-CAN
/// CANopen: socketcan / mcu-serial are valid, FD and damiao transports are not.
fn open_huayi_controller(
    transport: &str,
    channel: &str,
    serial_port: &str,
    serial_baud: u32,
) -> Result<HuayiController, Box<dyn std::error::Error>> {
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
                "vendor=huayi is classic CAN; use --transport auto|socketcan|mcu-serial".into(),
            );
        }
        "dm-serial" | "dm-device" => {
            return Err(format!(
                "transport {transport} is damiao-only (huayi supports auto|socketcan|mcu-serial)"
            )
            .into());
        }
        _ => {
            return Err(format!(
                "unknown huayi transport: {transport} (expected auto|socketcan|mcu-serial)"
            )
            .into());
        }
    };
    Ok(HuayiController::new(bus))
}

pub fn run_huayi(
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

    if motor_id == 0 || motor_id > 127 {
        return Err(format!(
            "huayi motor_id must be a CANopen node id in 1..=127, got 0x{motor_id:X}"
        )
        .into());
    }
    if feedback_id != 0 {
        println!(
            "[info] huayi feedback_id=0x{feedback_id:X} is metadata only — CANopen frames are claimed by node id (0x{motor_id:X}), the filter does not change"
        );
    }

    if mode == "scan" {
        let start_id = get_u16_hex_or_dec(args, "start-id", 1)?;
        let end_id = get_u16_hex_or_dec(args, "end-id", 127)?;
        if start_id == 0 || end_id > 127 || start_id > end_id {
            return Err("scan range must be within 1..=127 with start <= end".into());
        }
        let controller = open_huayi_controller(&transport, channel, &serial_port, serial_baud)?;
        println!(
            "[scan:huayi] channel={channel} model={model} id_range=[0x{start_id:X},0x{end_id:X}] timeout_ms={timeout_ms}"
        );
        let hits = controller.scan_ids(start_id, end_id, timeout)?;
        for hit in &hits {
            println!(
                "[hit] vendor=huayi node={} statusword={:?} mode_display={:?} error_code={:?}",
                hit.node_id, hit.statusword, hit.mode_display, hit.error_code
            );
        }
        println!("[scan] done vendor=huayi hits={}", hits.len());
        controller.close_bus()?;
        return Ok(());
    }

    let controller = open_huayi_controller(&transport, channel, &serial_port, serial_baud)?;
    let motor = controller.add_motor(motor_id, feedback_id, model)?;

    match mode.as_str() {
        "status" => {
            let status = motor.query_status(timeout)?;
            println!(
                "[status] node={} mode_display={} statusword=0x{:04X} error=0x{:04X} pos={:+.6}rad vel={:+.6}rad/s torque={:+.3}Nm({}permille) t_driver={:.1}C t_motor={:.1}C",
                motor_id,
                status.mode_display,
                status.statusword,
                status.error_code,
                status.pos_rad,
                status.vel_rad_s,
                status.torque_nm,
                status.torque_permille,
                status.t_driver_c,
                status.t_motor_c
            );
            controller.close_bus()?;
            return Ok(());
        }
        "enable" => {
            motor.enable_drive(timeout)?;
            println!("[ok] huayi enable sent (NMT start + 0x0006/0x0007/0x000F)");
            controller.close_bus()?;
            return Ok(());
        }
        "disable" => {
            motor.disable_drive(timeout)?;
            println!("[ok] huayi disable sent (0x0000 Disable Voltage)");
            controller.close_bus()?;
            return Ok(());
        }
        "quick-stop" | "emergency-stop" => {
            motor.quick_stop(timeout)?;
            println!("[ok] huayi quick-stop sent (0x000B)");
            controller.close_bus()?;
            return Ok(());
        }
        "stop" => {
            motor.controlled_stop(timeout)?;
            println!("[ok] huayi mode-aware stop sent (drive stays enabled)");
            controller.close_bus()?;
            return Ok(());
        }
        "clear-error" | "clear-fault" => {
            motor.clear_error(timeout)?;
            println!("[ok] huayi fault reset sent (0x0080 -> 0x0000)");
            controller.close_bus()?;
            return Ok(());
        }
        "zero" | "set-zero" => {
            // 3001h preset requires Switch On Disabled — disable first, the
            // same guard Damiao applies.
            motor.disable_drive(timeout)?;
            motor.set_zero_position(timeout)?;
            println!("[ok] huayi zero sent (3001h preset 0 Rev; requires-disabled guard passed)");
            controller.close_bus()?;
            return Ok(());
        }
        "save" => {
            motor.store_parameters(timeout)?;
            println!("[ok] huayi store sent (1010h 'save')");
            controller.close_bus()?;
            return Ok(());
        }
        "restore" => {
            motor.restore_parameters(timeout)?;
            println!("[ok] huayi restore sent (1011h 'load'; takes effect after reset)");
            controller.close_bus()?;
            return Ok(());
        }
        "set-node-id" => {
            let new_id = get_u16_hex_or_dec(args, "set-motor-id", 0)?;
            if new_id == 0 || new_id > 127 {
                controller.close_bus()?;
                return Err(format!("invalid --set-motor-id {new_id}, expected 1..=127").into());
            }
            motor.set_node_id(new_id, timeout)?;
            println!(
                "[ok] huayi node id {} -> {} written (2001h_1); store + power cycle to apply",
                motor_id, new_id
            );
            controller.close_bus()?;
            return Ok(());
        }
        "history" => {
            let codes = motor.get_error_history(timeout)?;
            println!(
                "[history] node={} count={} codes=[{}]",
                motor_id,
                codes.len(),
                codes
                    .iter()
                    .map(|c| format!("0x{c:04X}"))
                    .collect::<Vec<_>>()
                    .join(", ")
            );
            controller.close_bus()?;
            return Ok(());
        }
        "report" => {
            let period_ms = get_u64(args, "period-ms", 0)?;
            if period_ms > u16::MAX as u64 {
                controller.close_bus()?;
                return Err(format!("invalid --period-ms {period_ms}, expected 0..=65535").into());
            }
            motor.configure_periodic_report(period_ms as u16, timeout)?;
            if period_ms == 0 {
                println!("[ok] huayi TPDO report stopped (0)");
            } else {
                println!(
                    "[ok] huayi TPDO report active: TPDO1/2/3 event timer {period_ms} ms (0 to stop)"
                );
            }
            controller.close_bus()?;
            return Ok(());
        }
        "nmt" => {
            let cmd_raw = get_u64(args, "nmt-cmd", 0x01)?;
            if cmd_raw > 0xFF {
                controller.close_bus()?;
                return Err(format!("invalid --nmt-cmd 0x{cmd_raw:X}, expected <= 0xFF").into());
            }
            let node = get_u16_hex_or_dec(args, "nmt-node", 0)?;
            if node > 127 {
                controller.close_bus()?;
                return Err(format!("invalid --nmt-node {node}, expected 0..=127").into());
            }
            let cmd = match cmd_raw {
                0x01 => motor_canopen::nmt::NmtCommand::StartRemoteNode,
                0x02 => motor_canopen::nmt::NmtCommand::StopNode,
                0x80 => motor_canopen::nmt::NmtCommand::EnterPreOperational,
                0x81 => motor_canopen::nmt::NmtCommand::ResetNode,
                0x82 => motor_canopen::nmt::NmtCommand::ResetCommunication,
                _ => {
                    controller.close_bus()?;
                    return Err(format!(
                        "unknown NMT command 0x{cmd_raw:02X}, expected 0x01/0x02/0x80/0x81/0x82"
                    )
                    .into());
                }
            };
            // routed through the vendor so a reset targeting this node (or a
            // broadcast) also invalidates the motor's mode/PP/TPDO caches
            motor.nmt(cmd, node as u8)?;
            println!(
                "[ok] huayi NMT 0x{cmd_raw:02X} node={node} sent (node 0 = broadcast)"
            );
            controller.close_bus()?;
            return Ok(());
        }
        "read-obj" | "write-obj" => {
            let index = get_u16_hex_or_dec(args, "index", 0)?;
            let sub = get_u16_hex_or_dec(args, "sub", 0)?;
            if sub > 0xFF {
                controller.close_bus()?;
                return Err(format!("invalid --sub {sub}, expected 0..=255").into());
            }
            let sub = sub as u8;
            let ty = get_str(args, "ty", "u16");
            match mode.as_str() {
                "read-obj" => match ty.as_str() {
                    // one bus round trip per line; u8 reinterprets the wire
                    // byte as unsigned (sdo_read_i8 would print >= 0x80 wrong)
                    "u8" => {
                        let v = motor.sdo_read_i8(index, sub, timeout)? as u8;
                        println!("[obj] 0x{index:04X}_{sub:02X} = {v} (0x{v:02X})");
                    }
                    "i8" => println!(
                        "[obj] 0x{index:04X}_{sub:02X} = {}",
                        motor.sdo_read_i8(index, sub, timeout)?
                    ),
                    "u16" => {
                        let v = motor.sdo_read_u16(index, sub, timeout)?;
                        println!("[obj] 0x{index:04X}_{sub:02X} = 0x{v:04X} ({v})");
                    }
                    "i16" => println!(
                        "[obj] 0x{index:04X}_{sub:02X} = {}",
                        motor.sdo_read_i16(index, sub, timeout)?
                    ),
                    "u32" => {
                        let v = motor.sdo_read_u32(index, sub, timeout)?;
                        println!("[obj] 0x{index:04X}_{sub:02X} = 0x{v:08X} ({v})");
                    }
                    "i32" => println!(
                        "[obj] 0x{index:04X}_{sub:02X} = {}",
                        motor.sdo_read_i32(index, sub, timeout)?
                    ),
                    "f32" => println!(
                        "[obj] 0x{index:04X}_{sub:02X} = {}",
                        motor.sdo_read_f32(index, sub, timeout)?
                    ),
                    _ => {
                        controller.close_bus()?;
                        return Err(format!(
                            "unknown --ty {ty}, expected u8|i8|u16|i16|u32|i32|f32"
                        )
                        .into());
                    }
                },
                _ => {
                    let value_str = get_str(args, "value", "0");
                    match ty.as_str() {
                        "u8" | "i8" => {
                            let v: i8 = value_str
                                .parse()
                                .map_err(|_| format!("invalid --value {value_str}"))?;
                            motor.sdo_write_i8(index, sub, v, timeout)?;
                        }
                        "u16" => {
                            let v = crate::args::parse_u16_hex_or_dec(&value_str, "value")?;
                            motor.sdo_write_u16(index, sub, v, timeout)?;
                        }
                        "i16" => {
                            let v: i16 = value_str
                                .parse()
                                .map_err(|_| format!("invalid --value {value_str}"))?;
                            motor.sdo_write_i16(index, sub, v, timeout)?;
                        }
                        "u32" => {
                            let v = parse_u32_hex_or_dec(&value_str)?;
                            motor.sdo_write_u32(index, sub, v, timeout)?;
                        }
                        "i32" => {
                            let v: i32 = value_str
                                .parse()
                                .map_err(|_| format!("invalid --value {value_str}"))?;
                            motor.sdo_write_i32(index, sub, v, timeout)?;
                        }
                        "f32" => {
                            let v: f32 = value_str
                                .parse()
                                .map_err(|_| format!("invalid --value {value_str}"))?;
                            motor.sdo_write_f32(index, sub, v, timeout)?;
                        }
                        _ => {
                            controller.close_bus()?;
                            return Err(format!(
                                "unknown --ty {ty}, expected u8|i8|u16|i16|u32|i32|f32"
                            )
                            .into());
                        }
                    }
                    println!("[ok] 0x{index:04X}_{sub:02X} <- {value_str} (ty={ty})");
                }
            }
            controller.close_bus()?;
            return Ok(());
        }
        "preset" => {
            let value_rev = get_f32(args, "preset-rev", 0.0)?;
            motor.disable_drive(timeout)?;
            motor.position_preset(value_rev, timeout)?;
            println!(
                "[ok] huayi position preset {value_rev} Rev (disabled first; range is +/-0.5 Rev)"
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
                        "[info] huayi pos-vel maps to PP: 6060=1, 6081 profile velocity, 607A target, CW bit4 handshake"
                    );
                }
                motor.send_pos_vel(
                    get_f32(args, "pos", 0.0)?,
                    get_f32(args, "vlim", 1.0)?,
                    timeout,
                )?;
            }
            "vel" => {
                if i == 0 {
                    println!("[info] huayi vel maps to Profile Velocity: 6060=3, 60FF");
                }
                motor.send_vel(get_f32(args, "vel", 0.0)?, timeout)?;
            }
            "torque" => {
                if i == 0 {
                    println!("[info] huayi torque maps to Torque mode: 6060=4, 6071 permille of 6076h peak");
                }
                motor.send_torque(get_f32(args, "tau", 0.0)?, timeout)?;
            }
            "mit" => {
                if i == 0 {
                    println!(
                        "[info] huayi mit slow path: 2003h five SDO writes, kp/kd via 2003h_7 scale factor"
                    );
                }
                motor.send_mit(
                    get_f32(args, "pos", 0.0)?,
                    get_f32(args, "vel", 0.0)?,
                    get_f32(args, "kp", 0.0)?,
                    get_f32(args, "kd", 0.0)?,
                    get_f32(args, "tau", 0.0)?,
                    timeout,
                )?;
            }
            "mit-compressed" => {
                if i == 0 {
                    println!(
                        "[info] huayi mit compressed fast path: one 8-byte frame to 0x500+node (RPDO4)"
                    );
                }
                motor.send_mit_compressed(
                    get_f32(args, "pos", 0.0)?,
                    get_f32(args, "vel", 0.0)?,
                    get_f32(args, "kp", 0.0)?,
                    get_f32(args, "kd", 0.0)?,
                    get_f32(args, "tau", 0.0)?,
                    timeout,
                )?;
            }
            _ => {
                controller.close_bus()?;
                return Err(format!("unknown huayi mode: {mode}").into());
            }
        }
        if loop_n > 1 {
            std::thread::sleep(Duration::from_millis(dt_ms));
        }
    }

    controller.shutdown()?;
    Ok(())
}

fn parse_u32_hex_or_dec(s: &str) -> Result<u32, String> {
    if let Some(hex) = s.strip_prefix("0x") {
        u32::from_str_radix(hex, 16).map_err(|e| format!("invalid --value: {e}"))
    } else {
        s.parse::<u32>().map_err(|e| format!("invalid --value: {e}"))
    }
}
