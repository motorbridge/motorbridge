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

/// Count received RobStride state frames without changing the existing MotorState ABI.
#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_robstride_feedback_sequence(
    motor: *mut MotorHandle,
    out_sequence: *mut u64,
) -> i32 {
    if motor.is_null() || out_sequence.is_null() {
        set_last_error("motor or out_sequence is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    match &*motor {
        MotorHandleInner::Robstride(m) => {
            unsafe {
                *out_sequence = m.feedback_sequence();
            }
            0
        }
        _ => {
            set_last_error("feedback_sequence requires a RobStride motor");
            -1
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use motor_core::bus::CanFrame;
    use motor_core::device::MotorDevice;
    use motor_core::test_support::MockBus;
    use motor_vendor_robstride::protocol::{build_ext_id, CommunicationType};

    #[test]
    fn sequence_and_state_follow_received_frames_through_abi() {
        let motor =
            Arc::new(RobstrideMotor::new(2, 0xFD, "rs-00", Arc::new(MockBus::new())).unwrap());
        let mut handle = MotorHandle {
            inner: Mutex::new(MotorHandleInner::Robstride(motor.clone())),
        };
        let mut sequence = u64::MAX;
        assert_eq!(
            motor_handle_robstride_feedback_sequence(&mut handle, &mut sequence),
            0
        );
        assert_eq!(sequence, 0);
        let mut state = MotorState::default();
        assert_eq!(motor_handle_get_state(&mut handle, &mut state), 0);
        assert_eq!(state.has_value, 0);

        let mut expected = 0;
        for comm_type in [
            CommunicationType::OPERATION_STATUS,
            CommunicationType::ACTIVE_REPORT,
        ] {
            let frame = CanFrame {
                arbitration_id: build_ext_id(comm_type, 2, 0xFD),
                data: [0x90, 0, 0x80, 0, 0x7F, 0xFF, 0x05, 0x78],
                dlc: 8,
                is_extended: true,
                is_rx: true,
            };
            for _ in 0..2 {
                motor.process_feedback_frame(frame).unwrap();
                expected += 1;
                assert_eq!(
                    motor_handle_robstride_feedback_sequence(&mut handle, &mut sequence),
                    0
                );
                assert_eq!(sequence, expected);
                assert_eq!(motor_handle_get_state(&mut handle, &mut state), 0);
                assert_eq!(state.has_value, 1);
                assert_eq!(state.can_id, 2);
                assert_eq!(state.arbitration_id, frame.arbitration_id);
                assert_eq!(
                    motor_handle_robstride_feedback_sequence(&mut handle, &mut sequence),
                    0
                );
                assert_eq!(sequence, expected);
            }
        }
        for comm_type in [
            CommunicationType::GET_DEVICE_ID,
            CommunicationType::READ_PARAMETER,
            CommunicationType::FAULT_REPORT,
        ] {
            motor
                .process_feedback_frame(CanFrame {
                    arbitration_id: build_ext_id(comm_type, 2, 0xFD),
                    data: [0x19, 0x70, 0, 0, 0, 0, 0, 0],
                    dlc: 8,
                    is_extended: true,
                    is_rx: true,
                })
                .unwrap();
            assert_eq!(
                motor_handle_robstride_feedback_sequence(&mut handle, &mut sequence),
                0
            );
            assert_eq!(sequence, expected);
        }
    }

    #[test]
    fn sequence_rejects_null_and_other_vendor_without_writing_output() {
        let mut out = 17;
        assert_eq!(
            motor_handle_robstride_feedback_sequence(ptr::null_mut(), &mut out),
            -1
        );
        assert_eq!(out, 17);
        let motor = Arc::new(DamiaoMotor::new(2, 3, "4310", Arc::new(MockBus::new())).unwrap());
        let mut handle = MotorHandle {
            inner: Mutex::new(MotorHandleInner::Damiao(motor)),
        };
        assert_eq!(
            motor_handle_robstride_feedback_sequence(&mut handle, &mut out),
            -1
        );
        assert_eq!(out, 17);
        assert_eq!(
            motor_handle_robstride_feedback_sequence(&mut handle, ptr::null_mut()),
            -1
        );
    }
}
