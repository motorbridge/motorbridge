use crate::commands::{as_bool, as_f32, as_u64};
use crate::model::{ActiveCommand, ControllerHandle, MotorHandle};
use crate::session::SessionCtx;
use crate::vendors::damiao_ws::ensure_control_mode_soft;
use crate::vendors::hightorque_ws::{
    pos_raw_from_rad, send_hightorque_ext, tqe_raw_from_tau, vel_raw_from_rad_s, TWO_PI,
};
use motor_vendor_damiao::ControlMode as DamiaoControlMode;
use motor_vendor_hexfellow::{
    MitTarget as HexfellowMitTarget, PosVelTarget as HexfellowPosVelTarget,
};
use motor_vendor_robstride::{
    ControlMode as RobstrideControlMode, ParameterValue as RobstrideParameterValue,
};
use serde_json::{json, Value};
use std::time::Duration;

pub(crate) fn handle(op: &str, v: &Value, ctx: &mut SessionCtx) -> Option<Result<Value, String>> {
    match op {
        "mit" => Some(handle_mit(v, ctx)),
        "pos_vel" | "pos-vel" => Some(handle_pos_vel(v, ctx)),
        "vel" => Some(handle_vel(v, ctx)),
        "force_pos" | "force-pos" => Some(handle_force_pos(v, ctx)),
        _ => None,
    }
}

fn handle_mit(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    let mut warnings = Vec::new();
    let cmd = ActiveCommand::Mit {
        pos: as_f32(v, "pos", 0.0),
        vel: as_f32(v, "vel", 0.0),
        kp: as_f32(v, "kp", 30.0),
        kd: as_f32(v, "kd", 1.0),
        tau: as_f32(v, "tau", 0.0),
    };
    match ctx.motor.as_ref() {
        Some(MotorHandle::Damiao(m)) => {
            if let Some(warning) = ensure_control_mode_soft(
                m,
                DamiaoControlMode::Mit,
                Duration::from_millis(as_u64(v, "ensure_timeout_ms", 1000)),
            )? {
                warnings.push(warning);
            }
            if let ActiveCommand::Mit {
                pos,
                vel,
                kp,
                kd,
                tau,
            } = cmd
            {
                m.send_cmd_mit(pos, vel, kp, kd, tau)
                    .map_err(|e| e.to_string())?;
            }
        }
        Some(MotorHandle::Robstride(m)) => {
            ensure_robstride_mode(ctx, m, RobstrideControlMode::Mit, "mit")?;
            if let ActiveCommand::Mit {
                pos,
                vel,
                kp,
                kd,
                tau,
            } = cmd
            {
                m.send_cmd_mit(pos, vel, kp, kd, tau)
                    .map_err(|e| e.to_string())?;
            }
        }
        Some(MotorHandle::Hexfellow(m)) => {
            if let ActiveCommand::Mit {
                pos,
                vel,
                kp,
                kd,
                tau,
            } = cmd
            {
                m.command_mit(
                    HexfellowMitTarget {
                        position_rev: pos / TWO_PI,
                        velocity_rev_s: vel / TWO_PI,
                        torque_nm: tau,
                        kp: kp.clamp(0.0, u16::MAX as f32).round() as u16,
                        kd: kd.clamp(0.0, u16::MAX as f32).round() as u16,
                        limit_permille: 1000,
                    },
                    Duration::from_millis(300),
                )
                .map_err(|e| e.to_string())?;
            }
        }
        Some(MotorHandle::Hightorque(mid)) => {
            if let ActiveCommand::Mit { pos, vel, tau, .. } = cmd {
                let pos_raw = pos_raw_from_rad(pos);
                let vel_raw = vel_raw_from_rad_s(vel);
                let tqe_raw = tqe_raw_from_tau(tau);
                let mut data = [0x07, 0x35, 0, 0, 0, 0, 0, 0];
                data[2..4].copy_from_slice(&vel_raw.to_le_bytes());
                data[4..6].copy_from_slice(&tqe_raw.to_le_bytes());
                data[6..8].copy_from_slice(&pos_raw.to_le_bytes());
                if let Some(ControllerHandle::Hightorque(bus)) = ctx.controller.as_ref() {
                    send_hightorque_ext(bus.as_ref(), *mid, &data)?;
                }
            }
        }
        Some(MotorHandle::Myactuator(_)) => {
            return Err("mit is not supported for myactuator".to_string());
        }
        Some(MotorHandle::CyberBeast(_)) => {
            return Err("cyberbeast not supported for this operation".to_string());
        }
        None => return Err("motor not connected".to_string()),
    }
    ctx.active = if as_bool(v, "continuous", false) {
        Some(cmd)
    } else {
        None
    };
    let mut out = json!({"op":"mit","continuous": as_bool(v, "continuous", false)});
    add_warnings(&mut out, warnings);
    Ok(out)
}

fn handle_pos_vel(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    let cmd = ActiveCommand::PosVel {
        pos: as_f32(v, "pos", 0.0),
        vlim: as_f32(v, "vlim", 1.0),
    };
    match ctx.motor.as_ref() {
        Some(MotorHandle::Damiao(m)) => {
            let warning = ensure_control_mode_soft(
                m,
                DamiaoControlMode::PosVel,
                Duration::from_millis(as_u64(v, "ensure_timeout_ms", 1000)),
            )?;
            if let ActiveCommand::PosVel { pos, vlim } = cmd {
                m.send_cmd_pos_vel(pos, vlim).map_err(|e| e.to_string())?;
            }
            ctx.active = if as_bool(v, "continuous", false) {
                Some(cmd)
            } else {
                None
            };
            let mut out = json!({"op":"pos_vel","continuous": as_bool(v, "continuous", false)});
            if let Some(warning) = warning {
                add_warnings(&mut out, vec![warning]);
            }
            Ok(out)
        }
        Some(MotorHandle::Hexfellow(m)) => {
            if let ActiveCommand::PosVel { pos, vlim } = cmd {
                m.command_pos_vel(
                    HexfellowPosVelTarget {
                        position_rev: pos / TWO_PI,
                        velocity_rev_s: vlim / TWO_PI,
                    },
                    Duration::from_millis(300),
                )
                .map_err(|e| e.to_string())?;
            }
            ctx.active = if as_bool(v, "continuous", false) {
                Some(cmd)
            } else {
                None
            };
            Ok(json!({"op":"pos_vel","continuous": as_bool(v, "continuous", false)}))
        }
        Some(MotorHandle::Robstride(m)) => {
            ensure_robstride_mode(ctx, m, RobstrideControlMode::Position, "pos_vel")?;
            if let ActiveCommand::PosVel { pos, vlim } = cmd {
                let speed = vlim.abs();
                if speed.is_finite() && speed > 0.0 {
                    m.write_parameter(0x7017, RobstrideParameterValue::F32(speed))
                        .map_err(|e| e.to_string())?;
                }
                let loc_kp = v
                    .get("loc_kp")
                    .or_else(|| v.get("kp"))
                    .and_then(|x| x.as_f64())
                    .map(|x| x as f32);
                if let Some(kp) = loc_kp {
                    if kp.is_finite() && kp >= 0.0 {
                        m.write_parameter(0x701E, RobstrideParameterValue::F32(kp))
                            .map_err(|e| e.to_string())?;
                    }
                }
                m.write_parameter(0x7016, RobstrideParameterValue::F32(pos))
                    .map_err(|e| e.to_string())?;
            }
            ctx.active = if as_bool(v, "continuous", false) {
                Some(cmd)
            } else {
                None
            };
            Ok(json!({"op":"pos_vel","continuous": as_bool(v, "continuous", false)}))
        }
        Some(MotorHandle::Hightorque(_)) => {
            Err("pos_vel is not supported for hightorque".to_string())
        }
        Some(MotorHandle::Myactuator(_)) => {
            Err("pos_vel is not supported for myactuator".to_string())
        }
        Some(MotorHandle::CyberBeast(_)) => {
            Err("cyberbeast not supported for this operation".to_string())
        }
        None => Err("motor not connected".to_string()),
    }
}

fn ensure_robstride_mode(
    _ctx: &SessionCtx,
    motor: &std::sync::Arc<motor_vendor_robstride::RobstrideMotor>,
    mode: RobstrideControlMode,
    mode_name: &str,
) -> Result<(), String> {
    motor
        .ensure_control_mode(mode, Duration::from_millis(1000))
        .map_err(|e| format!("robstride {mode_name} mode switch failed: {e}"))?;

    motor.enable().map_err(|e| e.to_string())?;
    std::thread::sleep(Duration::from_millis(100));
    Ok(())
}

fn handle_vel(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    let mut warnings = Vec::new();
    let cmd = ActiveCommand::Vel {
        vel: as_f32(v, "vel", 0.0),
    };
    match ctx.motor.as_ref() {
        Some(MotorHandle::Damiao(m)) => {
            if let Some(warning) = ensure_control_mode_soft(
                m,
                DamiaoControlMode::Vel,
                Duration::from_millis(as_u64(v, "ensure_timeout_ms", 1000)),
            )? {
                warnings.push(warning);
            }
            if let ActiveCommand::Vel { vel } = cmd {
                m.send_cmd_vel(vel).map_err(|e| e.to_string())?;
            }
        }
        Some(MotorHandle::Robstride(m)) => {
            ensure_robstride_mode(ctx, m, RobstrideControlMode::Velocity, "vel")?;
            if let ActiveCommand::Vel { vel } = cmd {
                m.set_velocity_target(vel).map_err(|e| e.to_string())?;
            }
        }
        Some(MotorHandle::Myactuator(m)) => {
            if let ActiveCommand::Vel { vel } = cmd {
                m.send_velocity_setpoint(vel.to_degrees())
                    .map_err(|e| e.to_string())?;
            }
        }
        Some(MotorHandle::Hightorque(mid)) => {
            if let ActiveCommand::Vel { vel } = cmd {
                let vel_raw = vel_raw_from_rad_s(vel);
                let tqe_raw = 0i16;
                let mut data = [0x07, 0x07, 0x00, 0x80, 0x20, 0x00, 0x80, 0x00];
                data[4..6].copy_from_slice(&vel_raw.to_le_bytes());
                data[6..8].copy_from_slice(&tqe_raw.to_le_bytes());
                if let Some(ControllerHandle::Hightorque(bus)) = ctx.controller.as_ref() {
                    send_hightorque_ext(bus.as_ref(), *mid, &data)?;
                }
            }
        }
        Some(MotorHandle::Hexfellow(_)) => {
            return Err("vel is not supported for hexfellow; use pos_vel or mit".to_string())
        }
        Some(MotorHandle::CyberBeast(_)) => {
            return Err("cyberbeast not supported for this operation".to_string());
        }
        None => return Err("motor not connected".to_string()),
    }
    ctx.active = if as_bool(v, "continuous", false) {
        Some(cmd)
    } else {
        None
    };
    let mut out = json!({"op":"vel","continuous": as_bool(v, "continuous", false)});
    add_warnings(&mut out, warnings);
    Ok(out)
}

fn handle_force_pos(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    let cmd = ActiveCommand::ForcePos {
        pos: as_f32(v, "pos", 0.0),
        vlim: as_f32(v, "vlim", 1.0),
        ratio: as_f32(v, "ratio", 0.3),
    };
    match ctx.motor.as_ref() {
        Some(MotorHandle::Damiao(m)) => {
            let warning = ensure_control_mode_soft(
                m,
                DamiaoControlMode::ForcePos,
                Duration::from_millis(as_u64(v, "ensure_timeout_ms", 1000)),
            )?;
            if let ActiveCommand::ForcePos { pos, vlim, ratio } = cmd {
                m.send_cmd_force_pos(pos, vlim, ratio)
                    .map_err(|e| e.to_string())?;
            }
            ctx.active = if as_bool(v, "continuous", false) {
                Some(cmd)
            } else {
                None
            };
            let mut out = json!({"op":"force_pos","continuous": as_bool(v, "continuous", false)});
            if let Some(warning) = warning {
                add_warnings(&mut out, vec![warning]);
            }
            Ok(out)
        }
        Some(MotorHandle::Robstride(_)) => {
            Err("force_pos is not supported for robstride".to_string())
        }
        Some(MotorHandle::Hexfellow(_)) => {
            Err("force_pos is not supported for hexfellow".to_string())
        }
        Some(MotorHandle::Hightorque(_)) => {
            Err("force_pos is not supported for hightorque".to_string())
        }
        Some(MotorHandle::Myactuator(_)) => {
            Err("force_pos is not supported for myactuator".to_string())
        }
        Some(MotorHandle::CyberBeast(_)) => {
            Err("cyberbeast not supported for this operation".to_string())
        }
        None => Err("motor not connected".to_string()),
    }
}

fn add_warnings(out: &mut Value, warnings: Vec<String>) {
    if warnings.is_empty() {
        return;
    }
    if let Some(obj) = out.as_object_mut() {
        obj.insert("warning".to_string(), json!(warnings[0]));
        obj.insert("warnings".to_string(), json!(warnings));
    }
}
