use super::*;

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_ensure_mode(
    motor: *mut MotorHandle,
    mode: u32,
    timeout_ms: u32,
) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::Damiao(m) => {
                let dm_mode = to_damiao_mode(mode).map_err(|e| e.to_string())?;
                m.ensure_control_mode(dm_mode, Duration::from_millis(timeout_ms as u64))
                    .map_err(|e| e.to_string())
            }
            MotorHandleInner::Hexfellow(m) => {
                let timeout = Duration::from_millis(timeout_ms as u64);
                match mode {
                    1 => m.ensure_mode_enabled(5, timeout).map_err(|e| e.to_string()),
                    2 => m.ensure_mode_enabled(1, timeout).map_err(|e| e.to_string()),
                    _ => Err("Hexfellow mode must be 1(MIT) / 2(POS_VEL)".to_string()),
                }
            }
            MotorHandleInner::MyActuator(_m) => {
                let _ = to_myactuator_mode(mode).map_err(|e| e.to_string())?;
                Ok(())
            }
            MotorHandleInner::Robstride(m) => {
                let rs_mode = to_robstride_mode(mode).map_err(|e| e.to_string())?;
                m.ensure_control_mode(rs_mode, Duration::from_millis(timeout_ms as u64))
                    .map_err(|e| e.to_string())
            }
            MotorHandleInner::Hightorque(m) => m
                .ensure_control_mode(mode, Duration::from_millis(timeout_ms as u64))
                .map_err(|e| e.to_string()),
        }
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_send_mit(
    motor: *mut MotorHandle,
    target_position: f32,
    target_velocity: f32,
    stiffness: f32,
    damping: f32,
    feedforward_torque: f32,
) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::Damiao(m) => m
                .send_cmd_mit(
                    target_position,
                    target_velocity,
                    stiffness,
                    damping,
                    feedforward_torque,
                )
                .map_err(|e| e.to_string()),
            MotorHandleInner::Hexfellow(m) => {
                let kp = stiffness.clamp(0.0, u16::MAX as f32).round() as u16;
                let kd = damping.clamp(0.0, u16::MAX as f32).round() as u16;
                m.command_mit(
                    HexfellowMitTarget {
                        position_rev: target_position / (2.0 * PI),
                        velocity_rev_s: target_velocity / (2.0 * PI),
                        torque_nm: feedforward_torque,
                        kp,
                        kd,
                        limit_permille: 1000,
                    },
                    Duration::from_millis(300),
                )
                .map_err(|e| e.to_string())
            }
            MotorHandleInner::MyActuator(_) => {
                Err("send_mit is not supported for MyActuator; use pos-vel or vel".to_string())
            }
            MotorHandleInner::Robstride(m) => m
                .send_cmd_mit(
                    target_position,
                    target_velocity,
                    stiffness,
                    damping,
                    feedforward_torque,
                )
                .map_err(|e| e.to_string()),
            MotorHandleInner::Hightorque(m) => m
                .send_cmd_mit(
                    target_position,
                    target_velocity,
                    stiffness,
                    damping,
                    feedforward_torque,
                )
                .map_err(|e| e.to_string()),
        }
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_send_pos_vel(
    motor: *mut MotorHandle,
    target_position: f32,
    velocity_limit: f32,
) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::Damiao(m) => m
                .send_cmd_pos_vel(target_position, velocity_limit)
                .map_err(|e| e.to_string()),
            MotorHandleInner::Hexfellow(m) => m
                .command_pos_vel(
                    HexfellowPosVelTarget {
                        position_rev: target_position / (2.0 * PI),
                        velocity_rev_s: velocity_limit / (2.0 * PI),
                    },
                    Duration::from_millis(300),
                )
                .map_err(|e| e.to_string()),
            MotorHandleInner::MyActuator(m) => m
                .send_position_absolute_setpoint(
                    target_position * (180.0 / PI),
                    velocity_limit * (180.0 / PI),
                )
                .map_err(|e| e.to_string()),
            MotorHandleInner::Robstride(m) => {
                // Map unified POS_VEL -> RobStride native position mode:
                // run_mode=Position(1), optional limit_spd(0x7017), then loc_ref(0x7016).
                let mut rc = m
                    .set_mode(RobstrideControlMode::Position)
                    .map_err(|e| e.to_string());
                let vlim = velocity_limit.abs();
                if vlim.is_finite() && vlim > 0.0 {
                    rc = rc.and_then(|_| {
                        m.write_parameter(0x7017, ParameterValue::F32(vlim))
                            .map_err(|e| e.to_string())
                    });
                }
                rc.and_then(|_| {
                    m.write_parameter(0x7016, ParameterValue::F32(target_position))
                        .map_err(|e| e.to_string())
                })
            }
            MotorHandleInner::Hightorque(m) => m
                .send_cmd_pos_vel(target_position, velocity_limit)
                .map_err(|e| e.to_string()),
        }
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_robstride_send_pos_vel_csp(
    motor: *mut MotorHandle,
    target_position: f32,
    velocity_limit: f32,
) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::Robstride(m) => m
                .send_cmd_pos_vel_csp(target_position, velocity_limit)
                .map_err(|e| e.to_string()),
            _ => Err("robstride_send_pos_vel_csp requires a RobStride motor".to_string()),
        }
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_robstride_send_pos_vel_pp(
    motor: *mut MotorHandle,
    target_position: f32,
    velocity_max: f32,
    acceleration: f32,
) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::Robstride(m) => m
                .send_cmd_pos_vel_pp(target_position, velocity_max, acceleration)
                .map_err(|e| e.to_string()),
            _ => Err("robstride_send_pos_vel_pp requires a RobStride motor".to_string()),
        }
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_send_vel(motor: *mut MotorHandle, target_velocity: f32) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::Damiao(m) => {
                m.send_cmd_vel(target_velocity).map_err(|e| e.to_string())
            }
            MotorHandleInner::Hexfellow(_) => {
                Err("send_vel is not supported for Hexfellow; use MIT or POS_VEL".to_string())
            }
            MotorHandleInner::MyActuator(m) => m
                .send_velocity_setpoint(target_velocity * (180.0 / PI))
                .map_err(|e| e.to_string()),
            MotorHandleInner::Robstride(m) => m
                .set_velocity_target(target_velocity)
                .map_err(|e| e.to_string()),
            MotorHandleInner::Hightorque(m) => {
                m.send_cmd_vel(target_velocity).map_err(|e| e.to_string())
            }
        }
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_send_force_pos(
    motor: *mut MotorHandle,
    target_position: f32,
    velocity_limit: f32,
    torque_limit_ratio: f32,
) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::Damiao(m) => m
                .send_cmd_force_pos(target_position, velocity_limit, torque_limit_ratio)
                .map_err(|e| e.to_string()),
            MotorHandleInner::Hexfellow(_) => {
                Err("send_force_pos is not supported for Hexfellow".to_string())
            }
            MotorHandleInner::MyActuator(_) => {
                Err("send_force_pos is not supported for MyActuator".to_string())
            }
            MotorHandleInner::Robstride(_) => {
                Err("send_force_pos is not supported for RobStride".to_string())
            }
            MotorHandleInner::Hightorque(m) => m
                .send_cmd_force_pos(target_position, velocity_limit, torque_limit_ratio)
                .map_err(|e| e.to_string()),
        }
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_store_parameters(motor: *mut MotorHandle) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::Damiao(m) => m.store_parameters().map_err(|e| e.to_string()),
            MotorHandleInner::Hexfellow(_) => {
                Err("store_parameters is not supported for Hexfellow".to_string())
            }
            MotorHandleInner::MyActuator(_) => {
                Err("store_parameters is not supported for MyActuator".to_string())
            }
            MotorHandleInner::Robstride(m) => m.save_parameters().map_err(|e| e.to_string()),
            MotorHandleInner::Hightorque(m) => m.store_parameters().map_err(|e| e.to_string()),
        }
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_request_feedback(motor: *mut MotorHandle) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::Damiao(m) => m.request_motor_feedback().map_err(|e| e.to_string()),
            MotorHandleInner::Hexfellow(m) => m
                .query_status(Duration::from_millis(300))
                .map(|_| ())
                .map_err(|e| e.to_string()),
            MotorHandleInner::MyActuator(m) => m.request_status().map_err(|e| e.to_string()),
            // RobStride has no single-shot "request status frame" command in the private
            // protocol. Do not turn this unified feedback request into a blocking ping:
            // ping does not synthesize MotorState and can stall control loops. Use
            // robstride_ping() for connectivity checks, active report for streaming state,
            // or typed parameter reads for fresh position/velocity values.
            MotorHandleInner::Robstride(_) => Ok(()),
            MotorHandleInner::Hightorque(m) => m
                .request_motor_feedback(Duration::from_millis(500))
                .map_err(|e| e.to_string()),
        }
    })
}

/// 受控停止：保留电机使能，按当前运行模式选择零化策略。
///
/// 与 `motor_handle_disable`（失能、取消力矩）语义不同。
///
/// 各厂商行为：
/// - Damiao: 发送零速度 MIT 帧
/// - Hexfellow: 发送零 MIT 命令
/// - MyActuator: 调用 `stop_motor()`
/// - Robstride: 模式感知受控停止（MIT 保持位置、PP 写 vel_max=0、Velocity 写 0、CSP 锁位置）
/// - Hightorque: 发送原始 stop 帧（与 disable 同实现）
#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_stop(motor: *mut MotorHandle) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::Damiao(m) => m.send_cmd_vel(0.0).map_err(|e| e.to_string()),
            MotorHandleInner::Hexfellow(m) => m
                .command_mit(
                    HexfellowMitTarget {
                        position_rev: 0.0,
                        velocity_rev_s: 0.0,
                        torque_nm: 0.0,
                        kp: 0,
                        kd: 0,
                        limit_permille: 1000,
                    },
                    Duration::from_millis(200),
                )
                .map_err(|e| e.to_string()),
            MotorHandleInner::MyActuator(m) => m.stop_motor().map_err(|e| e.to_string()),
            MotorHandleInner::Robstride(m) => m
                .controlled_stop(Duration::from_millis(300))
                .map(|_| ())
                .map_err(|e| e.to_string()),
            MotorHandleInner::Hightorque(m) => m.disable().map_err(|e| e.to_string()),
        }
    })
}
