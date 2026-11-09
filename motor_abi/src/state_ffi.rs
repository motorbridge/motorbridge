use super::*;
use motor_core::feedback::FeedbackSnapshot;

fn snapshot_metadata<T>(snapshot: &FeedbackSnapshot<T>) -> MotorStateMetadata {
    MotorStateMetadata {
        feedback_age_ns: snapshot.age().as_nanos().min(u128::from(u64::MAX)) as u64,
        feedback_sequence: snapshot.sequence,
    }
}

/// Robstride 状态帧六故障标志拼位域(bit5..bit0:未标定/堵转/编码器
/// 故障/过温/过流/欠压),作为 fault_code 的值;status_code 承载
/// 协议原生 mode_state。可多故障并存,0=无故障。
fn robstride_fault_code(state: &motor_vendor_robstride::MotorFeedbackState) -> u8 {
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
    status
}

/// Damiao 协议状态枚举的 0x08–0x0E 段为故障码。该枚举状态与故障
/// 混装在同一字节,无法完全切开——status_code 保留原始枚举值,
/// 故障段抽出为 fault_code,0=无故障。
fn damiao_fault_code(state: &motor_vendor_damiao::MotorFeedbackState) -> u8 {
    if (0x08..=0x0E).contains(&state.status_code) {
        state.status_code
    } else {
        0
    }
}

/// 统一契约:status_code = 电机状态(各厂商状态字节),fault_code =
/// 错误码(0=无故障)。各厂商错误码来源:
/// - Hightorque: 表3 故障码(反馈帧 byte1,vendor 解码后 `state.fault_code`)。
/// - Robstride: 状态帧六故障标志拼位域(`robstride_fault_code`)。
/// - Damiao: 协议状态枚举 0x08–0x0E 故障段(`damiao_fault_code`)。
/// - Hexfellow / MyActuator: 协议无故障码,恒 0。
fn latest_snapshot(
    motor: &MotorHandleInner,
) -> Result<Option<(MotorState, MotorStateMetadata)>, String> {
    match motor {
        MotorHandleInner::Damiao(m) => Ok(m.latest_state_snapshot().map(|snapshot| {
            let state = snapshot.state;
            (
                MotorState {
                    has_value: 1,
                    can_id: state.can_id,
                    arbitration_id: state.arbitration_id,
                    status_code: state.status_code,
                    pos: state.pos,
                    vel: state.vel,
                    torq: state.torq,
                    t_mos: state.t_mos,
                    t_rotor: state.t_rotor,
                    fault_code: damiao_fault_code(&state),
                },
                snapshot_metadata(&snapshot),
            )
        })),
        MotorHandleInner::Hexfellow(m) => {
            m.query_status(Duration::from_millis(20))
                .map_err(|e| e.to_string())?;
            let snapshot = m
                .latest_state_snapshot()
                .ok_or_else(|| "Hexfellow status query returned no state".to_string())?;
            let state = snapshot.state;
            Ok(Some((
                MotorState {
                    has_value: 1,
                    can_id: m.motor_id as u8,
                    arbitration_id: 0,
                    status_code: state.heartbeat_state.unwrap_or(0),
                    pos: state.position_rev * (2.0 * PI),
                    vel: state.velocity_rev_s * (2.0 * PI),
                    torq: state.torque_permille as f32 / 1000.0,
                    t_mos: 0.0,
                    t_rotor: 0.0,
                    fault_code: 0,
                },
                snapshot_metadata(&snapshot),
            )))
        }
        MotorHandleInner::MyActuator(m) => Ok(m.latest_state_snapshot().map(|snapshot| {
            let state = snapshot.state;
            (
                MotorState {
                    has_value: 1,
                    can_id: m.motor_id as u8,
                    arbitration_id: state.arbitration_id,
                    status_code: state.command,
                    pos: state.shaft_angle_deg * (PI / 180.0),
                    vel: state.speed_dps * (PI / 180.0),
                    torq: state.current_a,
                    t_mos: f32::from(state.temperature_c),
                    t_rotor: 0.0,
                    fault_code: 0,
                },
                snapshot_metadata(&snapshot),
            )
        })),
        MotorHandleInner::Robstride(m) => Ok(m.latest_state_snapshot().map(|snapshot| {
            let state = snapshot.state;
            (
                MotorState {
                    has_value: 1,
                    can_id: state.device_id,
                    arbitration_id: state.arbitration_id,
                    status_code: state.mode_state,
                    pos: state.position,
                    vel: state.velocity,
                    torq: state.torque,
                    t_mos: state.temperature_c,
                    t_rotor: 0.0,
                    // 状态帧六故障标志拼位域(bit5..bit0:未标定/堵转/
                    // 编码器故障/过温/过流/欠压),可多故障并存。
                    fault_code: robstride_fault_code(&state),
                },
                snapshot_metadata(&snapshot),
            )
        })),
        MotorHandleInner::Hightorque(m) => Ok(m.latest_state_snapshot().map(|snapshot| {
            let state = snapshot.state;
            (
                MotorState {
                    has_value: 1,
                    can_id: state.can_id,
                    arbitration_id: state.arbitration_id,
                    status_code: state.status_code,
                    pos: state.pos,
                    vel: state.vel,
                    torq: state.torq,
                    t_mos: state.t_mos,
                    t_rotor: state.t_rotor,
                    fault_code: state.fault_code,
                },
                snapshot_metadata(&snapshot),
            )
        })),
    }
}

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
    match latest_snapshot(&motor) {
        Ok(Some((state, _))) => *out = state,
        Ok(None) => *out = MotorState::default(),
        Err(error) => {
            set_last_error(error);
            return -1;
        }
    }
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_get_state_snapshot(
    motor: *mut MotorHandle,
    out_state: *mut MotorState,
    out_metadata: *mut MotorStateMetadata,
) -> i32 {
    if motor.is_null() || out_state.is_null() || out_metadata.is_null() {
        set_last_error("motor, out_state, or out_metadata is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let out_state = unsafe { &mut *out_state };
    let out_metadata = unsafe { &mut *out_metadata };
    match latest_snapshot(&motor) {
        Ok(Some((state, metadata))) => {
            *out_state = state;
            *out_metadata = metadata;
        }
        Ok(None) => {
            *out_state = MotorState::default();
            *out_metadata = MotorStateMetadata::default();
        }
        Err(error) => {
            set_last_error(error);
            return -1;
        }
    }
    0
}

#[cfg(test)]
mod tests {
    use super::*;
    use motor_core::bus::CanFrame;
    use motor_core::device::MotorDevice;
    use motor_core::test_support::MockBus;
    use motor_vendor_robstride::protocol::{build_ext_id, CommunicationType};

    #[test]
    fn snapshot_sequence_and_state_follow_received_frames_through_abi() {
        let motor =
            Arc::new(RobstrideMotor::new(2, 0xFD, "rs-00", Arc::new(MockBus::new())).unwrap());
        let mut handle = MotorHandle {
            inner: Mutex::new(MotorHandleInner::Robstride(motor.clone())),
        };
        let mut state = MotorState::default();
        let mut metadata = MotorStateMetadata {
            feedback_age_ns: u64::MAX,
            feedback_sequence: u64::MAX,
        };
        assert_eq!(
            motor_handle_get_state_snapshot(&mut handle, &mut state, &mut metadata),
            0
        );
        assert_eq!(state.has_value, 0);
        assert_eq!(metadata.feedback_sequence, 0);
        assert_eq!(metadata.feedback_age_ns, 0);

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
                metadata = MotorStateMetadata {
                    feedback_age_ns: u64::MAX,
                    feedback_sequence: u64::MAX,
                };
                assert_eq!(
                    motor_handle_get_state_snapshot(&mut handle, &mut state, &mut metadata),
                    0
                );
                assert_eq!(state.has_value, 1);
                assert_eq!(state.can_id, 2);
                assert_eq!(state.arbitration_id, frame.arbitration_id);
                assert_eq!(metadata.feedback_sequence, expected);
                assert!(metadata.feedback_age_ns < u64::MAX);
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
            metadata = MotorStateMetadata::default();
            assert_eq!(
                motor_handle_get_state_snapshot(&mut handle, &mut state, &mut metadata),
                0
            );
            assert_eq!(state.has_value, 1);
            assert_eq!(metadata.feedback_sequence, expected);
        }
    }

    #[test]
    fn snapshot_rejects_null_pointers_without_writing_output() {
        let motor =
            Arc::new(RobstrideMotor::new(2, 0xFD, "rs-00", Arc::new(MockBus::new())).unwrap());
        let mut handle = MotorHandle {
            inner: Mutex::new(MotorHandleInner::Robstride(motor)),
        };
        let mut state = MotorState::default();
        let mut metadata = MotorStateMetadata {
            feedback_age_ns: 17,
            feedback_sequence: 17,
        };
        assert_eq!(
            motor_handle_get_state_snapshot(ptr::null_mut(), &mut state, &mut metadata),
            -1
        );
        assert_eq!(metadata.feedback_sequence, 17);
        assert_eq!(
            motor_handle_get_state_snapshot(&mut handle, ptr::null_mut(), &mut metadata),
            -1
        );
        assert_eq!(metadata.feedback_sequence, 17);
        assert_eq!(
            motor_handle_get_state_snapshot(&mut handle, &mut state, ptr::null_mut()),
            -1
        );
        assert_eq!(state.has_value, 0);
    }
}
