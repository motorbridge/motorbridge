use crate::args::{get_f32, get_str, get_u16_hex_or_dec, get_u64};
use motor_core::bus::{open_transport, CanBus, Transport, TransportParams};
use motor_vendor_hexfellow::{HexfellowController, MitTarget, PosVelTarget};
use std::collections::HashMap;
use std::f32::consts::PI;
use std::sync::Arc;
use std::time::Duration;

fn to_rev(rad: f32) -> f32 {
    rad / (2.0 * PI)
}

/// Open a Hexfellow controller for the requested transport. Hexfellow is
/// CAN-FD only, so the sole supported transport is socketcanfd (and `auto`
/// resolves to it); classic-CAN and serial transports are rejected.
fn open_hexfellow_controller(
    transport: &str,
    channel: &str,
    serial_port: &str,
    serial_baud: u32,
) -> Result<HexfellowController, Box<dyn std::error::Error>> {
    let p = TransportParams {
        channel,
        serial_port,
        serial_baud,
    };
    let bus: Arc<dyn CanBus> = match transport {
        "auto" | "socketcanfd" => open_transport(Transport::SocketCanFd, &p)?,
        "socketcan" => {
            return Err(
                "transport socketcan unsupported (hexfellow is CAN-FD only, use socketcanfd)".into(),
            )
        }
        "mcu-serial" => {
            return Err(
                "transport mcu-serial unsupported (hexfellow is CAN-FD only; mcu-serial is classic CAN)"
                    .into(),
            )
        }
        "dm-serial" | "dm-device" => {
            return Err(format!(
                "transport {transport} is damiao-only (hexfellow supports auto|socketcanfd)"
            )
            .into());
        }
        _ => {
            return Err(format!(
                "unknown Hexfellow transport: {transport} (expected auto|socketcanfd)"
            )
            .into());
        }
    };
    Ok(HexfellowController::new(bus))
}

pub fn run_hexfellow(
    args: &HashMap<String, String>,
    channel: &str,
    model: &str,
    motor_id: u16,
    feedback_id: u16,
) -> Result<(), Box<dyn std::error::Error>> {
    let mode = get_str(args, "mode", "status");
    let transport = get_str(args, "transport", "auto");
    let serial_port = get_str(args, "serial-port", "/dev/ttyACM0");
    let serial_baud_u64 = get_u64(args, "serial-baud", 921600)?;
    let serial_baud = u32::try_from(serial_baud_u64)
        .map_err(|_| format!("invalid --serial-baud (too large): {serial_baud_u64}"))?;

    let timeout_ms = get_u64(args, "timeout-ms", 200)?;
    let timeout = Duration::from_millis(timeout_ms);
    let controller = open_hexfellow_controller(&transport, channel, &serial_port, serial_baud)?;

    if mode == "scan" {
        let start_id = get_u16_hex_or_dec(args, "start-id", 1)?;
        let end_id = get_u16_hex_or_dec(args, "end-id", 32)?;
        let hits = controller.scan_ids(start_id, end_id, timeout)?;
        for h in &hits {
            println!(
                "[hit] vendor=hexfellow node={} sw_ver={:?} peak_torque_raw={:?} kp_kd_factor_raw={:?} dev_type={:?}",
                h.node_id, h.sw_ver, h.peak_torque_raw, h.kp_kd_factor_raw, h.dev_type
            );
        }
        println!("[scan] done vendor=hexfellow hits={}", hits.len());
        controller.close_bus()?;
        return Ok(());
    }

    let motor = controller.add_motor(motor_id, feedback_id, model)?;
    match mode.as_str() {
        "enable" => {
            motor.enable_drive(timeout)?;
            println!("[ok] hexfellow enable sent");
        }
        "disable" => {
            motor.disable_drive(timeout)?;
            println!("[ok] hexfellow disable sent");
        }
        "status" => {
            let s = motor.query_status(timeout)?;
            println!(
                "[status] mode_display={} statusword={} pos_rev={:.6} vel_rev_s={:.6} torque_permille={} hb={:?}",
                s.mode_display,
                s.statusword,
                s.position_rev,
                s.velocity_rev_s,
                s.torque_permille,
                s.heartbeat_state
            );
        }
        "pos-vel" => {
            let pos_rad = get_f32(args, "pos", 0.0)?;
            let vel_rad_s = get_f32(args, "vlim", 2.0)?;
            motor.command_pos_vel(
                PosVelTarget {
                    position_rev: to_rev(pos_rad),
                    velocity_rev_s: to_rev(vel_rad_s),
                },
                timeout,
            )?;
            println!(
                "[ok] hexfellow pos-vel sent pos_rad={:.6} vlim_rad_s={:.6}",
                pos_rad, vel_rad_s
            );
        }
        "mit" => {
            let pos_rad = get_f32(args, "pos", 0.0)?;
            let vel_rad_s = get_f32(args, "vel", 0.0)?;
            let tau = get_f32(args, "tau", 0.0)?;
            let kp = get_f32(args, "kp", 1000.0)? as u16;
            let kd = get_f32(args, "kd", 100.0)? as u16;
            let limit_permille_u64 = get_u64(args, "limit-permille", 1000)?;
            if limit_permille_u64 > 1000 {
                return Err(format!(
                    "invalid --limit-permille {} (expected 0..=1000)",
                    limit_permille_u64
                )
                .into());
            }
            let limit_permille = limit_permille_u64 as u16;
            motor.command_mit(
                MitTarget {
                    position_rev: to_rev(pos_rad),
                    velocity_rev_s: to_rev(vel_rad_s),
                    torque_nm: tau,
                    kp,
                    kd,
                    limit_permille,
                },
                timeout,
            )?;
            println!(
                "[ok] hexfellow mit sent pos_rad={:.6} vel_rad_s={:.6} tau={:.6} kp={} kd={} limit_permille={}",
                pos_rad, vel_rad_s, tau, kp, kd, limit_permille
            );
        }
        _ => {
            return Err(
                "unknown hexfellow mode: expected scan|status|enable|disable|pos-vel|mit".into(),
            );
        }
    }

    controller.close_bus()?;
    Ok(())
}
