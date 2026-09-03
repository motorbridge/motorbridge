use super::*;

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_get_state(
    motor: *mut MotorHandle,
    out_state: *mut MotorState,
) -> i32 {
    if motor.is_null() || out_state.is_null() {
        set_last_error("motor or out_state is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let out = unsafe { &mut *out_state };
    match &*motor {
        MotorHandleInner::Damiao(m) => {
            if let Some(state) = m.latest_state() {
                *out = MotorState {
                    has_value: 1,
                    can_id: state.can_id,
                    arbitration_id: state.arbitration_id,
                    status_code: state.status_code,
                    pos: state.pos,
                    vel: state.vel,
                    torq: state.torq,
                    t_mos: state.t_mos,
                    t_rotor: state.t_rotor,
                };
            } else {
                *out = MotorState::default();
            }
        }
        MotorHandleInner::Hexfellow(m) => match m.query_status(Duration::from_millis(20)) {
            Ok(state) => {
                *out = MotorState {
                    has_value: 1,
                    can_id: m.motor_id as u8,
                    arbitration_id: 0,
                    status_code: state.heartbeat_state.unwrap_or(0),
                    pos: state.position_rev * (2.0 * PI),
                    vel: state.velocity_rev_s * (2.0 * PI),
                    torq: state.torque_permille as f32 / 1000.0,
                    t_mos: 0.0,
                    t_rotor: 0.0,
                };
            }
            Err(e) => {
                set_last_error(e.to_string());
                return -1;
            }
        },
        MotorHandleInner::MyActuator(m) => {
            if let Some(state) = m.latest_state() {
                *out = MotorState {
                    has_value: 1,
                    can_id: m.motor_id as u8,
                    arbitration_id: state.arbitration_id,
                    status_code: state.command,
                    pos: state.shaft_angle_deg * (PI / 180.0),
                    vel: state.speed_dps * (PI / 180.0),
                    torq: state.current_a,
                    t_mos: f32::from(state.temperature_c),
                    t_rotor: 0.0,
                };
            } else {
                *out = MotorState::default();
            }
        }
        MotorHandleInner::Robstride(m) => {
            if let Some(state) = m.latest_state() {
                let mut status = 0u8;
                if state.uncalibrated {
                    status |= 1 << 5;
                }
                if state.stall {
                    status |= 1 << 4;
                }
                if state.magnetic_encoder_fault {
                    status |= 1 << 3;
                }
                if state.overtemperature {
                    status |= 1 << 2;
                }
                if state.overcurrent {
                    status |= 1 << 1;
                }
                if state.undervoltage {
                    status |= 1;
                }
                *out = MotorState {
                    has_value: 1,
                    can_id: state.device_id,
                    arbitration_id: state.arbitration_id,
                    status_code: status,
                    pos: state.position,
                    vel: state.velocity,
                    torq: state.torque,
                    t_mos: state.temperature_c,
                    t_rotor: 0.0,
                };
            } else {
                *out = MotorState::default();
            }
        }
        MotorHandleInner::CyberBeast(m) => {
            if let Some(state) = m.latest_state() {
                *out = MotorState {
                    has_value: 1,
                    can_id: state.can_id_parts.source,
                    arbitration_id: state.arbitration_id,
                    status_code: state.error_code,
                    pos: state.pos,
                    vel: state.vel,
                    torq: state.current,
                    t_mos: state.mos_temp,
                    t_rotor: state.motor_temp,
                };
            } else {
                *out = MotorState::default();
            }
        }
        MotorHandleInner::Hightorque(m) => {
            if let Some(state) = m.latest_state() {
                *out = MotorState {
                    has_value: 1,
                    can_id: state.can_id,
                    arbitration_id: state.arbitration_id,
                    status_code: state.status_code,
                    pos: state.pos,
                    vel: state.vel,
                    torq: state.torq,
                    t_mos: state.t_mos,
                    t_rotor: state.t_rotor,
                };
            } else {
                *out = MotorState::default();
            }
        }
    }
    0
}
