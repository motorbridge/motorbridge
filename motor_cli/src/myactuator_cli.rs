use crate::args::{get_f32, get_str, get_u16_hex_or_dec, get_u64};
use motor_core::bus::{open_transport, CanBus, Transport, TransportParams};
use motor_vendor_myactuator::MyActuatorController;
use std::collections::HashMap;
use std::convert::TryFrom;
use std::sync::Arc;
use std::time::Duration;

fn myactuator_feedback_default(motor_id: u16) -> u16 {
    0x240u16.saturating_add(motor_id)
}

/// Open a MyActuator controller for the requested transport. Universal
/// transports route through `open_transport` (core); damiao-only transports
/// are rejected. MyActuator is classic+FD CAN, so mcu-serial is allowed.
fn open_myactuator_controller(
    transport: &str,
    channel: &str,
    serial_port: &str,
    serial_baud: u32,
) -> Result<MyActuatorController, Box<dyn std::error::Error>> {
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
                "transport {transport} is damiao-only (myactuator supports auto|socketcan|socketcanfd|mcu-serial)"
            )
            .into());
        }
        _ => {
            return Err(format!(
                "unknown MyActuator transport: {transport} (expected auto|socketcan|socketcanfd|mcu-serial)"
            )
            .into());
        }
    };
    Ok(MyActuatorController::new(bus))
}

pub fn run_myactuator(
    args: &HashMap<String, String>,
    channel: &str,
    model: &str,
    motor_id: u16,
    feedback_id: u16,
) -> Result<(), Box<dyn std::error::Error>> {
    let mode = get_str(args, "mode", "status");
    let loop_n = get_u64(args, "loop", 1)?;
    let dt_ms = get_u64(args, "dt-ms", 20)?;

    let transport = get_str(args, "transport", "auto");
    let serial_port = get_str(args, "serial-port", "/dev/ttyACM0");
    let serial_baud_u64 = get_u64(args, "serial-baud", 921600)?;
    let serial_baud = u32::try_from(serial_baud_u64)
        .map_err(|_| format!("invalid --serial-baud (too large): {serial_baud_u64}"))?;
    let controller = open_myactuator_controller(&transport, channel, &serial_port, serial_baud)?;

    if mode == "scan" {
        let start_id = get_u16_hex_or_dec(args, "start-id", 1)?;
        let end_id_in = get_u16_hex_or_dec(args, "end-id", 32)?;
        if start_id == 0 || end_id_in == 0 || start_id > 32 || start_id > end_id_in {
            return Err("invalid scan range: expected start in 1..32 and start<=end".into());
        }
        let end_id = end_id_in.min(32);
        if end_id_in > 32 {
            println!(
                "[scan] note: MyActuator scan end-id {} clamped to 32",
                end_id_in
            );
        }
        println!(
            "[scan] probing MyActuator IDs {}..{} on {}",
            start_id, end_id, channel
        );
        let mut hits = 0usize;
        for id in start_id..=end_id {
            let m = controller.add_motor(id, myactuator_feedback_default(id), model)?;
            let _ = m.request_version_date();
            if let Ok(version) = m.await_version_date(Duration::from_millis(100)) {
                println!(
                    "[hit] vendor=myactuator id={} feedback_id=0x{:X} version={}",
                    id,
                    myactuator_feedback_default(id),
                    version
                );
                hits += 1;
            }
            std::thread::sleep(Duration::from_millis(3));
        }
        println!("[scan] done vendor=myactuator hits={hits}");
        controller.close_bus()?;
        return Ok(());
    }

    let effective_feedback_id = if feedback_id == 0 {
        myactuator_feedback_default(motor_id)
    } else {
        feedback_id
    };
    let motor = controller.add_motor(motor_id, effective_feedback_id, model)?;

    if mode != "disable" {
        motor.release_brake()?;
        std::thread::sleep(Duration::from_millis(80));
    }

    if mode == "pos" {
        // Absolute-position setpoint: --pos is sent as absolute motor position.
        // Position setpoint is sent once, then we only poll feedback in loop.
        let target_pos_rad = get_f32(args, "pos", 0.0)?;
        motor.send_position_absolute_setpoint(
            target_pos_rad.to_degrees(),
            get_f32(args, "max-speed", 8.726646)?.to_degrees(),
        )?;
    }

    for i in 0..loop_n {
        match mode.as_str() {
            "enable" => motor.release_brake()?,
            "disable" => motor.shutdown_motor()?,
            "stop" => motor.stop_motor()?,
            "set-zero" => {
                motor.set_current_position_as_zero()?;
                println!("#{i} set-zero command sent (0x64). Power-cycle actuator to apply persistent zero.");
            }
            "status" => {
                motor.request_status()?;
                motor.request_multi_turn_angle()?;
            }
            "current" => motor.send_current_setpoint(get_f32(args, "current", 0.0)?)?,
            "vel" => {
                let vel_rad_s = get_f32(args, "vel", 0.0)?;
                motor.send_velocity_setpoint(vel_rad_s.to_degrees())?
            }
            "pos" => {
                motor.request_status()?;
                motor.request_multi_turn_angle()?;
            }
            "version" => motor.request_version_date()?,
            "mode-query" => motor.request_control_mode()?,
            _ => {
                return Err(format!(
                    "unknown MyActuator mode: {mode} (supported: scan|enable|disable|stop|set-zero|status|current|vel|pos|version|mode-query)"
                )
                .into())
            }
        }

        std::thread::sleep(Duration::from_millis(10));

        if let Some(v) = motor.latest_version_date() {
            println!("#{i} version={v}");
        }
        if let Some(cm) = motor.latest_control_mode() {
            println!("#{i} control_mode={cm}");
        }
        if let Some(s) = motor.latest_state() {
            let speed_rad_s = s.speed_dps.to_radians();
            let angle_rad = s.shaft_angle_deg.to_radians();
            let mt_angle = motor.latest_multi_turn_angle_deg().map(|d| d.to_radians());
            println!(
                "#{i} cmd=0x{:02X} temp={}C current={:+.2}A speed={:+.3}rad/s angle={:+.3}rad mt_angle={}",
                s.command,
                s.temperature_c,
                s.current_a,
                speed_rad_s,
                angle_rad,
                mt_angle
                    .map(|v| format!("{:+.3}rad", v))
                    .unwrap_or_else(|| "n/a".to_string())
            );
        }

        std::thread::sleep(Duration::from_millis(dt_ms));
    }

    if mode == "disable" {
        controller.close_bus()?;
    } else {
        controller.shutdown()?;
    }
    Ok(())
}
