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
    if let Some(mode) = robstride_position_mode_for_op(op) {
        return Some(handle_robstride_position(v, ctx, mode));
    }
    match op {
        "mit" => Some(handle_mit(v, ctx)),
        "pos_vel" | "pos-vel" => Some(handle_pos_vel(v, ctx)),
        "vel" => Some(handle_vel(v, ctx)),
        "force_pos" | "force-pos" => Some(handle_force_pos(v, ctx)),
        _ => None,
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum RobstridePositionMode {
    Pp,
    Csp,
}

fn robstride_position_mode_for_op(op: &str) -> Option<RobstridePositionMode> {
    match op {
        "pos_vel_pp" | "pos-vel-pp" => Some(RobstridePositionMode::Pp),
        "pos_vel_csp" | "pos-vel-csp" => Some(RobstridePositionMode::Csp),
        _ => None,
    }
}

fn as_f32_alias(v: &Value, primary: &str, alias: &str, default: f32) -> f32 {
    v.get(primary)
        .or_else(|| v.get(alias))
        .and_then(Value::as_f64)
        .map(|value| value as f32)
        .unwrap_or(default)
}

fn positive_finite(value: f32, name: &str) -> Result<f32, String> {
    let value = value.abs();
    if value.is_finite() && value > 0.0 {
        Ok(value)
    } else {
        Err(format!("{name} must be a finite value greater than zero"))
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
            if as_bool(v, "continuous", false) {
                return Err(
                    "RobStride legacy pos_vel uses PP mode and does not support continuous=true; use pos_vel_csp for cyclic targets"
                        .to_string(),
                );
            }
            ensure_robstride_mode(ctx, m, RobstrideControlMode::Position, "pos_vel")?;
            if let ActiveCommand::PosVel { pos, vlim } = cmd {
                let speed = positive_finite(vlim, "vlim")?;
                m.write_parameter(0x7024, RobstrideParameterValue::F32(speed))
                    .map_err(|e| e.to_string())?;
                if v.get("acc_set").is_some() || v.get("acc").is_some() {
                    let acceleration =
                        positive_finite(as_f32_alias(v, "acc_set", "acc", 10.0), "acc_set")?;
                    m.write_parameter(0x7025, RobstrideParameterValue::F32(acceleration))
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
            ctx.active = None;
            let mut out = json!({
                "op": "pos_vel",
                "continuous": false,
                "native_mode": "pp",
                "velocity_parameter": "vel_max"
            });
            add_warnings(
                &mut out,
                vec![
                    "RobStride pos_vel is a legacy PP alias; prefer pos_vel_pp with vel_max and acc_set"
                        .to_string(),
                ],
            );
            Ok(out)
        }
        Some(MotorHandle::Hightorque(_)) => {
            Err("pos_vel is not supported for hightorque".to_string())
        }
        Some(MotorHandle::Myactuator(_)) => {
            Err("pos_vel is not supported for myactuator".to_string())
        }
        None => Err("motor not connected".to_string()),
    }
}

fn handle_robstride_position(
    v: &Value,
    ctx: &mut SessionCtx,
    mode: RobstridePositionMode,
) -> Result<Value, String> {
    ctx.ensure_connected()?;
    let pos = as_f32(v, "pos", 0.0);
    match ctx.motor.as_ref() {
        Some(MotorHandle::Robstride(motor)) => match mode {
            RobstridePositionMode::Pp => {
                if as_bool(v, "continuous", false) {
                    return Err(
                        "pos_vel_pp does not support continuous=true; PP speed and acceleration cannot be changed during motion"
                            .to_string(),
                    );
                }
                let vel_max = positive_finite(as_f32_alias(v, "vel_max", "vlim", 1.0), "vel_max")?;
                let acc_set = positive_finite(as_f32_alias(v, "acc_set", "acc", 10.0), "acc_set")?;
                ensure_robstride_mode(ctx, motor, RobstrideControlMode::Position, "pos_vel_pp")?;
                motor
                    .send_cmd_pos_vel_pp(pos, vel_max, acc_set)
                    .map_err(|e| e.to_string())?;
                ctx.active = None;
                Ok(json!({
                    "op": "pos_vel_pp",
                    "continuous": false,
                    "native_mode": "pp",
                    "pos": pos,
                    "vel_max": vel_max,
                    "acc_set": acc_set
                }))
            }
            RobstridePositionMode::Csp => {
                let limit_spd =
                    positive_finite(as_f32_alias(v, "limit_spd", "vlim", 1.0), "limit_spd")?;
                ensure_robstride_mode(
                    ctx,
                    motor,
                    RobstrideControlMode::PositionCsp,
                    "pos_vel_csp",
                )?;
                motor
                    .send_cmd_pos_vel_csp(pos, limit_spd)
                    .map_err(|e| e.to_string())?;
                let continuous = as_bool(v, "continuous", false);
                ctx.active = if continuous {
                    Some(ActiveCommand::PosVel {
                        pos,
                        vlim: limit_spd,
                    })
                } else {
                    None
                };
                Ok(json!({
                    "op": "pos_vel_csp",
                    "continuous": continuous,
                    "native_mode": "csp",
                    "pos": pos,
                    "limit_spd": limit_spd
                }))
            }
        },
        Some(_) => Err(format!(
            "{} is only supported for robstride",
            match mode {
                RobstridePositionMode::Pp => "pos_vel_pp",
                RobstridePositionMode::Csp => "pos_vel_csp",
            }
        )),
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{Target, Transport, Vendor};
    use motor_core::bus::{CanBus, CanFrame};
    use motor_core::device::MotorDevice;
    use motor_core::test_support::MockBus;
    use motor_vendor_robstride::protocol::build_ext_id;
    use motor_vendor_robstride::{
        ext_id_parts, CommunicationType, ParameterId, RobstrideController, RobstrideMotor,
    };
    use std::sync::Arc;

    fn robstride_ctx() -> (SessionCtx, Arc<RobstrideMotor>, Arc<MockBus>) {
        let bus = Arc::new(MockBus::new());
        let can_bus: Arc<dyn CanBus> = bus.clone();
        let controller = RobstrideController::new(can_bus);
        let motor = controller
            .add_motor(2, 0xFD, "rs-00")
            .expect("add RobStride motor");
        let mut ctx = SessionCtx::new(Target {
            vendor: Vendor::Robstride,
            transport: Transport::Auto,
            channel: "test".to_string(),
            serial_port: String::new(),
            serial_baud: 115_200,
            dm_device_type: String::new(),
            dm_channel: String::new(),
            model: "rs-00".to_string(),
            motor_id: 2,
            feedback_id: 0xFD,
        });
        ctx.controller = Some(ControllerHandle::Robstride(controller));
        ctx.motor = Some(MotorHandle::Robstride(Arc::clone(&motor)));
        (ctx, motor, bus)
    }

    fn spawn_mode_and_enable_responder(
        motor: Arc<RobstrideMotor>,
        bus: Arc<MockBus>,
        mode: i8,
        expected_enables: usize,
    ) -> std::thread::JoinHandle<()> {
        std::thread::spawn(move || {
            let deadline = std::time::Instant::now() + Duration::from_secs(2);
            let mut cursor = 0;
            let mut mode_replied = false;
            let mut enable_replies = 0;
            while !mode_replied || enable_replies < expected_enables {
                let frames = {
                    let sent = bus.sent.lock().expect("sent frames");
                    sent.iter().skip(cursor).copied().collect::<Vec<_>>()
                };
                cursor += frames.len();
                for frame in frames {
                    let (comm_type, _, _) = ext_id_parts(frame.arbitration_id);
                    if comm_type == CommunicationType::READ_PARAMETER
                        && u16::from_le_bytes([frame.data[0], frame.data[1]])
                            == ParameterId::Mode as u16
                    {
                        let mut data = [0u8; 8];
                        data[0..2].copy_from_slice(&(ParameterId::Mode as u16).to_le_bytes());
                        data[4] = mode as u8;
                        motor
                            .process_feedback_frame(CanFrame {
                                arbitration_id: build_ext_id(
                                    CommunicationType::READ_PARAMETER,
                                    2,
                                    0xFD,
                                ),
                                data,
                                dlc: 8,
                                is_extended: true,
                                is_rx: true,
                            })
                            .expect("process mode response");
                        mode_replied = true;
                    } else if comm_type == CommunicationType::ENABLE {
                        motor
                            .process_feedback_frame(CanFrame {
                                arbitration_id: build_ext_id(
                                    CommunicationType::OPERATION_STATUS,
                                    2,
                                    0xFD,
                                ),
                                data: [0u8; 8],
                                dlc: 8,
                                is_extended: true,
                                is_rx: true,
                            })
                            .expect("process enable response");
                        enable_replies += 1;
                    }
                }
                assert!(
                    std::time::Instant::now() < deadline,
                    "timed out waiting for routed command frames"
                );
                std::thread::sleep(Duration::from_millis(1));
            }
        })
    }

    fn written_parameter_ids(bus: &MockBus) -> Vec<u16> {
        bus.sent
            .lock()
            .expect("sent frames")
            .iter()
            .filter_map(|frame| {
                let (comm_type, _, _) = ext_id_parts(frame.arbitration_id);
                (comm_type == CommunicationType::WRITE_PARAMETER)
                    .then(|| u16::from_le_bytes([frame.data[0], frame.data[1]]))
            })
            .collect()
    }

    #[test]
    fn routes_robstride_pp_and_csp_operation_aliases() {
        assert_eq!(
            robstride_position_mode_for_op("pos_vel_pp"),
            Some(RobstridePositionMode::Pp)
        );
        assert_eq!(
            robstride_position_mode_for_op("pos-vel-pp"),
            Some(RobstridePositionMode::Pp)
        );
        assert_eq!(
            robstride_position_mode_for_op("pos_vel_csp"),
            Some(RobstridePositionMode::Csp)
        );
        assert_eq!(
            robstride_position_mode_for_op("pos-vel-csp"),
            Some(RobstridePositionMode::Csp)
        );
        assert_eq!(robstride_position_mode_for_op("pos_vel"), None);
    }

    #[test]
    fn parses_native_parameter_names_and_legacy_aliases() {
        let native = json!({"vel_max": 0.02, "acc_set": 0.05, "limit_spd": 0.03});
        assert!((as_f32_alias(&native, "vel_max", "vlim", 1.0) - 0.02).abs() < f32::EPSILON);
        assert!((as_f32_alias(&native, "acc_set", "acc", 10.0) - 0.05).abs() < f32::EPSILON);
        assert!((as_f32_alias(&native, "limit_spd", "vlim", 1.0) - 0.03).abs() < f32::EPSILON);

        let aliases = json!({"vlim": 0.04, "acc": 0.06});
        assert!((as_f32_alias(&aliases, "vel_max", "vlim", 1.0) - 0.04).abs() < f32::EPSILON);
        assert!((as_f32_alias(&aliases, "acc_set", "acc", 10.0) - 0.06).abs() < f32::EPSILON);
    }

    #[test]
    fn routes_pp_operation_to_pp_parameters() {
        let (mut ctx, motor, bus) = robstride_ctx();
        let responder = spawn_mode_and_enable_responder(motor, Arc::clone(&bus), 1, 2);

        let result = handle(
            "pos_vel_pp",
            &json!({"pos": 0.02, "vel_max": 0.005, "acc_set": 0.02}),
            &mut ctx,
        )
        .expect("operation routed")
        .expect("PP operation succeeds");
        responder.join().expect("responder");

        assert_eq!(result["op"], "pos_vel_pp");
        let ids = written_parameter_ids(&bus);
        assert!(ids.contains(&(ParameterId::PpVelocityMax as u16)));
        assert!(ids.contains(&(ParameterId::PpAccelerationTarget as u16)));
        assert!(ids.contains(&(ParameterId::PositionTarget as u16)));
        assert!(!ids.contains(&(ParameterId::VelocityLimit as u16)));
    }

    #[test]
    fn legacy_robstride_pos_vel_maps_vlim_to_pp_vel_max() {
        let (mut ctx, motor, bus) = robstride_ctx();
        let responder = spawn_mode_and_enable_responder(motor, Arc::clone(&bus), 1, 1);

        let result = handle(
            "pos_vel",
            &json!({"pos": 0.02, "vlim": 0.005, "acc": 0.02}),
            &mut ctx,
        )
        .expect("operation routed")
        .expect("legacy PP operation succeeds");
        responder.join().expect("responder");

        assert_eq!(result["native_mode"], "pp");
        assert_eq!(result["velocity_parameter"], "vel_max");
        let ids = written_parameter_ids(&bus);
        assert!(ids.contains(&(ParameterId::PpVelocityMax as u16)));
        assert!(ids.contains(&(ParameterId::PpAccelerationTarget as u16)));
        assert!(ids.contains(&(ParameterId::PositionTarget as u16)));
        assert!(!ids.contains(&(ParameterId::VelocityLimit as u16)));
    }

    #[test]
    fn routes_csp_operation_to_csp_parameters() {
        let (mut ctx, motor, bus) = robstride_ctx();
        let responder = spawn_mode_and_enable_responder(motor, Arc::clone(&bus), 5, 2);

        let result = handle(
            "pos-vel-csp",
            &json!({"pos": -0.02, "limit_spd": 0.01}),
            &mut ctx,
        )
        .expect("operation routed")
        .expect("CSP operation succeeds");
        responder.join().expect("responder");

        assert_eq!(result["op"], "pos_vel_csp");
        let ids = written_parameter_ids(&bus);
        assert!(ids.contains(&(ParameterId::VelocityLimit as u16)));
        assert!(ids.contains(&(ParameterId::PositionTarget as u16)));
        assert!(!ids.contains(&(ParameterId::PpVelocityMax as u16)));
        assert!(!ids.contains(&(ParameterId::PpAccelerationTarget as u16)));
    }
}
