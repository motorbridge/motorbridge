use crate::args::{get_f32, get_str, get_u16_hex_or_dec, get_u64};
use motor_vendor_cyberbeast::CyberBeastController;
use std::collections::HashMap;
use std::time::Duration;

const SCAN_TIMEOUT_MS: u64 = 200;

pub fn run_cyberbeast(
    args: &HashMap<String, String>,
    channel: &str,
    model: &str,
    motor_id: u16,
    _feedback_id: u16,
) -> Result<(), Box<dyn std::error::Error>> {
    let mode = get_str(args, "mode", "status");
    let ctrl = CyberBeastController::new_socketcan(channel)?;

    match mode.as_str() {
        "scan" => {
            let start_id = get_u16_hex_or_dec(args, "start-id", 1)?;
            let end_id = get_u16_hex_or_dec(args, "end-id", 32)?;
            println!("scanning CyberBeast motors on {channel} (IDs {start_id}..{end_id})...");

            for id in start_id..=end_id {
                let motor = match ctrl.add_motor(id, id, model) {
                    Ok(m) => m,
                    Err(_) => continue,
                };
                let _ = motor.send_start_motor();
                let _ = motor.send_query_status();
                std::thread::sleep(Duration::from_millis(SCAN_TIMEOUT_MS));
                let _ = ctrl.poll_feedback_once();

                if let Some(state) = motor.latest_state() {
                    println!(
                        "  [found] id=0x{id:02X} pos={:.3} vel={:.3} current={:.3} err=0x{:X} mode={} motor_temp={:.1}°C mos_temp={:.1}°C",
                        state.pos, state.vel, state.current, state.error_code, state.mode_state, state.motor_temp, state.mos_temp
                    );
                }
                let _ = motor.send_stop_motor();
            }
            ctrl.shutdown()?;
        }

        "status" => {
            let motor = ctrl.add_motor(motor_id, motor_id, model)?;
            let _ = motor.send_start_motor();
            let _ = motor.send_query_status();
            std::thread::sleep(Duration::from_millis(SCAN_TIMEOUT_MS));
            let _ = ctrl.poll_feedback_once();

            if let Some(state) = motor.latest_state() {
                println!("status for motor 0x{motor_id:02X}:");
                println!("  pos={:.4} rad, vel={:.4} rad/s", state.pos, state.vel);
                println!(
                    "  current={:.3} A, error=0x{:X}, mode={}",
                    state.current, state.error_code, state.mode_state
                );
                println!(
                    "  motor_temp={:.1}°C, mos_temp={:.1}°C",
                    state.motor_temp, state.mos_temp
                );
                println!(
                    "  heartbeat_life={} error_flags=0x{:02X}",
                    state.heartbeat_life, state.error_flags
                );
            } else {
                println!("no response from motor 0x{motor_id:02X}");
            }
            let _ = motor.send_stop_motor();
            ctrl.shutdown()?;
        }

        "mit" => {
            let motor = ctrl.add_motor(motor_id, motor_id, model)?;
            println!("starting MIT control for motor 0x{motor_id:02X} (Ctrl+C to stop)...");
            let _ = motor.send_start_motor();
            let kp = get_f32(args, "kp", 100.0)?;
            let kd = get_f32(args, "kd", 10.0)?;
            let loop_ms = get_u64(args, "loop-ms", 5)?;

            loop {
                let pos = get_f32(args, "pos", 0.0)?;
                let vel = get_f32(args, "vel", 0.0)?;
                let torque = get_f32(args, "torque", 0.0)?;
                motor.send_mit_command(pos, vel, kp, kd, torque)?;
                let _ = ctrl.poll_feedback_once();
                if let Some(state) = motor.latest_state() {
                    print!(
                        "\rpos={:.4} vel={:.4} cur={:.3} err=0x{:X} mode={} temp={:.1}°C",
                        state.pos,
                        state.vel,
                        state.current,
                        state.error_code,
                        state.mode_state,
                        state.motor_temp
                    );
                }
                std::thread::sleep(Duration::from_millis(loop_ms));
            }
        }

        "pos" => {
            let motor = ctrl.add_motor(motor_id, motor_id, model)?;
            println!("starting POS control for motor 0x{motor_id:02X} (Ctrl+C to stop)...");
            let _ = motor.send_start_motor();
            let loop_ms = get_u64(args, "loop-ms", 10)?;

            loop {
                let pos = get_f32(args, "pos", 0.0)?;
                let vel_limit = get_f32(args, "vel-limit", 100.0)?;
                motor.send_pos_control(pos, vel_limit, 0.0)?;
                let _ = ctrl.poll_feedback_once();
                if let Some(state) = motor.latest_state() {
                    print!(
                        "\rpos={:.4} vel={:.4} err=0x{:X}",
                        state.pos, state.vel, state.error_code
                    );
                }
                std::thread::sleep(Duration::from_millis(loop_ms));
            }
        }

        "vel" => {
            let motor = ctrl.add_motor(motor_id, motor_id, model)?;
            println!("starting VEL control for motor 0x{motor_id:02X} (Ctrl+C to stop)...");
            let _ = motor.send_start_motor();
            let loop_ms = get_u64(args, "loop-ms", 10)?;

            loop {
                let vel_rpm = get_f32(args, "vel", 0.0)?;
                motor.send_vel_control(vel_rpm, 0.0)?;
                let _ = ctrl.poll_feedback_once();
                if let Some(state) = motor.latest_state() {
                    print!(
                        "\rvel={:.4} cur={:.3} err=0x{:X}",
                        state.vel, state.current, state.error_code
                    );
                }
                std::thread::sleep(Duration::from_millis(loop_ms));
            }
        }

        "torque" => {
            let motor = ctrl.add_motor(motor_id, motor_id, model)?;
            println!("starting TORQUE control for motor 0x{motor_id:02X} (Ctrl+C to stop)...");
            let _ = motor.send_start_motor();
            let loop_ms = get_u64(args, "loop-ms", 5)?;

            loop {
                let torque_nm = get_f32(args, "torque", 0.0)?;
                motor.send_torque_control(torque_nm)?;
                let _ = ctrl.poll_feedback_once();
                if let Some(state) = motor.latest_state() {
                    print!(
                        "\rtorque_target={:.4} cur={:.3} err=0x{:X}",
                        torque_nm, state.current, state.error_code
                    );
                }
                std::thread::sleep(Duration::from_millis(loop_ms));
            }
        }

        "enable" => {
            let motor = ctrl.add_motor(motor_id, motor_id, model)?;
            motor.send_start_motor()?;
            println!("enabled motor 0x{motor_id:02X}");
            ctrl.shutdown()?;
        }

        "disable" => {
            let motor = ctrl.add_motor(motor_id, motor_id, model)?;
            motor.send_stop_motor()?;
            println!("disabled motor 0x{motor_id:02X}");
            ctrl.shutdown()?;
        }

        other => {
            eprintln!(
                "unknown mode: {other}. Supported: scan, status, mit, pos, vel, torque, enable, disable"
            );
            ctrl.shutdown()?;
        }
    }

    Ok(())
}
