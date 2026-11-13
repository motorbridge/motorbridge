use crate::objects::*;
use motor_canopen::node::CanopenNode;
use motor_canopen::send_std_frame;
use motor_core::bus::{CanBus, CanFrame};
use motor_core::device::MotorDevice;
use motor_core::error::{MotorError, Result};
use motor_core::model::{ModelCatalog, MotorModelSpec, PvTLimits, StaticModelCatalog};
use std::sync::Arc;
use std::time::Duration;

const ROBSTRIDE_CIA402_MODELS: &[MotorModelSpec] = &[
    MotorModelSpec {
        vendor: "robstride_cia402",
        model: "rs-00",
        pmax: 4.0 * std::f32::consts::PI,
        vmax: 50.0,
        tmax: 17.0,
    },
    MotorModelSpec {
        vendor: "robstride_cia402",
        model: "rs-01",
        pmax: 4.0 * std::f32::consts::PI,
        vmax: 44.0,
        tmax: 17.0,
    },
    MotorModelSpec {
        vendor: "robstride_cia402",
        model: "rs-02",
        pmax: 4.0 * std::f32::consts::PI,
        vmax: 44.0,
        tmax: 17.0,
    },
    MotorModelSpec {
        vendor: "robstride_cia402",
        model: "rs-03",
        pmax: 4.0 * std::f32::consts::PI,
        vmax: 50.0,
        tmax: 60.0,
    },
    MotorModelSpec {
        vendor: "robstride_cia402",
        model: "rs-04",
        pmax: 4.0 * std::f32::consts::PI,
        vmax: 15.0,
        tmax: 120.0,
    },
    MotorModelSpec {
        vendor: "robstride_cia402",
        model: "rs-05",
        pmax: 4.0 * std::f32::consts::PI,
        vmax: 33.0,
        tmax: 17.0,
    },
    MotorModelSpec {
        vendor: "robstride_cia402",
        model: "rs-06",
        pmax: 4.0 * std::f32::consts::PI,
        vmax: 20.0,
        tmax: 60.0,
    },
];

const ROBSTRIDE_CIA402_CATALOG: StaticModelCatalog = StaticModelCatalog {
    vendor_name: "robstride_cia402",
    models: ROBSTRIDE_CIA402_MODELS,
};

pub fn model_limits(model: &str) -> Option<(f32, f32, f32)> {
    ROBSTRIDE_CIA402_CATALOG
        .get(model)
        .map(|spec| (spec.pmax, spec.vmax, spec.tmax))
}

#[derive(Debug, Clone, Copy)]
pub struct RobstrideCia402Target {
    pub position_rad: f32,
    pub velocity_rad_s: f32,
    pub torque_nm: f32,
}

#[derive(Debug, Clone, Copy)]
pub struct RobstrideCia402Status {
    pub mode_display: i8,
    pub statusword: u16,
    pub error_code: u16,
    pub position_rad: f32,
    pub velocity_rad_s: f32,
    pub torque_nm: f32,
    pub current_ma: i16,
    pub dc_link_mv: i32,
    pub heartbeat_state: Option<u8>,
}

pub struct RobstrideCia402Motor {
    pub motor_id: u16,
    pub feedback_id: u16,
    pub model: String,
    node: CanopenNode,
    limits: PvTLimits,
}

impl RobstrideCia402Motor {
    pub fn new(motor_id: u16, feedback_id: u16, model: &str, bus: Arc<dyn CanBus>) -> Result<Self> {
        if motor_id == 0 || motor_id > 127 {
            return Err(MotorError::InvalidArgument(format!(
                "invalid RobStride CiA402 node id {motor_id}, expected 1..127"
            )));
        }
        let spec = ROBSTRIDE_CIA402_CATALOG.get(model).ok_or_else(|| {
            MotorError::InvalidArgument(format!("unknown RobStride CiA402 model: {model}"))
        })?;
        Ok(Self {
            motor_id,
            feedback_id,
            model: model.to_string(),
            node: CanopenNode::new(motor_id, bus)?,
            limits: PvTLimits::from_spec(spec),
        })
    }

    // --- SDO / NMT plumbing now lives in motor_canopen::node; the typed
    // wrappers below keep this crate's public surface stable. ---

    pub fn send_nmt(&self, command: u8, node_id: u8) -> Result<()> {
        send_std_frame(self.node.bus().as_ref(), NMT_ID, &[command, node_id])
    }

    pub fn sdo_read_i8(&self, index: u16, subindex: u8, timeout: Duration) -> Result<i8> {
        self.node.sdo_read_i8(index, subindex, timeout)
    }

    pub fn sdo_read_u16(&self, index: u16, subindex: u8, timeout: Duration) -> Result<u16> {
        self.node.sdo_read_u16(index, subindex, timeout)
    }

    pub fn sdo_read_i16(&self, index: u16, subindex: u8, timeout: Duration) -> Result<i16> {
        self.node.sdo_read_i16(index, subindex, timeout)
    }

    pub fn sdo_read_i32(&self, index: u16, subindex: u8, timeout: Duration) -> Result<i32> {
        self.node.sdo_read_i32(index, subindex, timeout)
    }

    pub fn sdo_write_i8(
        &self,
        index: u16,
        subindex: u8,
        value: i8,
        timeout: Duration,
    ) -> Result<()> {
        self.node.sdo_write_i8(index, subindex, value, timeout)
    }

    pub fn sdo_write_u16(
        &self,
        index: u16,
        subindex: u8,
        value: u16,
        timeout: Duration,
    ) -> Result<()> {
        self.node.sdo_write_u16(index, subindex, value, timeout)
    }

    pub fn sdo_write_i16(
        &self,
        index: u16,
        subindex: u8,
        value: i16,
        timeout: Duration,
    ) -> Result<()> {
        self.node.sdo_write_i16(index, subindex, value, timeout)
    }

    pub fn sdo_write_i32(
        &self,
        index: u16,
        subindex: u8,
        value: i32,
        timeout: Duration,
    ) -> Result<()> {
        self.node.sdo_write_i32(index, subindex, value, timeout)
    }

    pub fn start_node(&self) -> Result<()> {
        self.node.start_node()
    }

    pub fn set_mode(&self, mode: i8, timeout: Duration) -> Result<()> {
        self.sdo_write_i8(MODES_OF_OPERATION, 0x00, mode, timeout)
    }

    pub fn enable_drive(&self, timeout: Duration) -> Result<()> {
        self.start_node()?;
        std::thread::sleep(Duration::from_millis(10));
        for cw in [CW_SHUTDOWN, CW_SWITCH_ON, CW_ENABLE_OPERATION] {
            self.sdo_write_u16(CONTROLWORD, 0x00, cw, timeout)?;
            std::thread::sleep(Duration::from_millis(10));
        }
        Ok(())
    }

    pub fn disable_drive(&self, timeout: Duration) -> Result<()> {
        self.sdo_write_u16(CONTROLWORD, 0x00, CW_DISABLE_TO_SWITCH_ON_DISABLED, timeout)
    }

    pub fn quick_stop(&self, timeout: Duration) -> Result<()> {
        self.sdo_write_u16(CONTROLWORD, 0x00, CW_QUICK_STOP, timeout)
    }

    pub fn clear_fault(&self, timeout: Duration) -> Result<()> {
        self.sdo_write_u16(CONTROLWORD, 0x00, CW_FAULT_RESET, timeout)?;
        std::thread::sleep(Duration::from_millis(10));
        self.sdo_write_u16(CONTROLWORD, 0x00, 0x0000, timeout)
    }

    pub fn prepare_mode_disabled(&self, mode: i8, timeout: Duration) -> Result<()> {
        self.start_node()?;
        std::thread::sleep(Duration::from_millis(10));
        self.disable_drive(timeout).ok();
        std::thread::sleep(Duration::from_millis(10));
        self.set_mode(mode, timeout)
    }

    pub fn ensure_mode_enabled(&self, mode: i8, timeout: Duration) -> Result<()> {
        self.prepare_mode_disabled(mode, timeout)?;
        self.enable_drive(timeout)
    }

    pub fn set_can_watchdog_raw(&self, value: u32, timeout: Duration) -> Result<()> {
        self.node.sdo_write_u32(CAN_WATCHDOG, 0x01, value, timeout)
    }

    pub fn set_can_watchdog_seconds(&self, seconds: f32, timeout: Duration) -> Result<()> {
        self.set_can_watchdog_raw(watchdog_seconds_to_raw(seconds), timeout)
    }

    pub fn set_position_window_rad(&self, window_rad: f32, timeout: Duration) -> Result<()> {
        self.sdo_write_i32(
            POSITION_WINDOW,
            0x00,
            rad_to_pulses(window_rad.abs()),
            timeout,
        )
    }

    pub fn set_position_window_time_ms(&self, time_ms: u16, timeout: Duration) -> Result<()> {
        self.sdo_write_u16(POSITION_WINDOW_TIME, 0x00, time_ms, timeout)
    }

    pub fn set_current_position_zero(&self, timeout: Duration) -> Result<()> {
        self.prepare_mode_disabled(MODE_HOMING, timeout)?;
        self.sdo_write_u16(CONTROLWORD, 0x00, CW_ENABLE_OPERATION, timeout)
    }

    pub fn command_profile_position(
        &self,
        target_position_rad: f32,
        velocity_limit_rad_s: f32,
        acceleration_rad_s2: f32,
        timeout: Duration,
    ) -> Result<()> {
        self.command_profile_position_full(
            target_position_rad,
            velocity_limit_rad_s,
            acceleration_rad_s2,
            None,
            None,
            timeout,
        )
    }

    pub fn command_profile_position_full(
        &self,
        target_position_rad: f32,
        velocity_limit_rad_s: f32,
        acceleration_rad_s2: f32,
        position_window_rad: Option<f32>,
        position_window_time_ms: Option<u16>,
        timeout: Duration,
    ) -> Result<()> {
        self.prepare_mode_disabled(MODE_PROFILE_POSITION, timeout)?;
        self.sdo_write_i16(
            TARGET_TORQUE,
            0x00,
            torque_nm_to_robstride_raw(self.limits.t_max),
            timeout,
        )?;
        self.sdo_write_i32(
            PROFILE_VELOCITY,
            0x00,
            rad_s_to_0p1rpm(velocity_limit_rad_s.abs()),
            timeout,
        )?;
        self.sdo_write_i32(
            PROFILE_ACCELERATION,
            0x00,
            rad_s_to_0p1rpm(acceleration_rad_s2.abs()),
            timeout,
        )?;
        if let Some(window_rad) = position_window_rad {
            self.set_position_window_rad(window_rad, timeout)?;
        }
        if let Some(window_time_ms) = position_window_time_ms {
            self.set_position_window_time_ms(window_time_ms, timeout)?;
        }
        self.enable_drive(timeout)?;
        self.sdo_write_i32(
            TARGET_POSITION,
            0x00,
            rad_to_pulses(target_position_rad),
            timeout,
        )
    }

    pub fn command_csp(
        &self,
        target_position_rad: f32,
        velocity_limit_rad_s: f32,
        torque_limit_nm: f32,
        timeout: Duration,
    ) -> Result<()> {
        self.command_csp_full(
            target_position_rad,
            velocity_limit_rad_s,
            torque_limit_nm,
            None,
            None,
            timeout,
        )
    }

    pub fn command_csp_full(
        &self,
        target_position_rad: f32,
        velocity_limit_rad_s: f32,
        torque_limit_nm: f32,
        position_window_rad: Option<f32>,
        position_window_time_ms: Option<u16>,
        timeout: Duration,
    ) -> Result<()> {
        self.prepare_mode_disabled(MODE_CSP, timeout)?;
        self.sdo_write_i16(
            TARGET_TORQUE,
            0x00,
            torque_nm_to_robstride_raw(torque_limit_nm.abs()),
            timeout,
        )?;
        if velocity_limit_rad_s.is_finite() && velocity_limit_rad_s.abs() > 0.0 {
            self.sdo_write_i32(
                PROFILE_VELOCITY,
                0x00,
                rad_s_to_0p1rpm(velocity_limit_rad_s.abs()),
                timeout,
            )?;
        }
        if let Some(window_rad) = position_window_rad {
            self.set_position_window_rad(window_rad, timeout)?;
        }
        if let Some(window_time_ms) = position_window_time_ms {
            self.set_position_window_time_ms(window_time_ms, timeout)?;
        }
        self.enable_drive(timeout)?;
        self.sdo_write_i32(
            TARGET_POSITION,
            0x00,
            rad_to_pulses(target_position_rad),
            timeout,
        )
    }

    pub fn command_velocity(&self, target_velocity_rad_s: f32, timeout: Duration) -> Result<()> {
        self.prepare_mode_disabled(MODE_PROFILE_VELOCITY, timeout)?;
        self.sdo_write_i16(
            TARGET_TORQUE,
            0x00,
            torque_nm_to_robstride_raw(self.limits.t_max),
            timeout,
        )?;
        self.enable_drive(timeout)?;
        self.sdo_write_i32(
            TARGET_VELOCITY,
            0x00,
            rad_s_to_0p1rpm(target_velocity_rad_s),
            timeout,
        )
    }

    pub fn command_torque(&self, target_torque_nm: f32, timeout: Duration) -> Result<()> {
        self.prepare_mode_disabled(MODE_TORQUE, timeout)?;
        self.enable_drive(timeout)?;
        self.sdo_write_i16(
            TARGET_TORQUE,
            0x00,
            torque_nm_to_robstride_raw(target_torque_nm),
            timeout,
        )
    }

    pub fn query_status(&self, timeout: Duration) -> Result<RobstrideCia402Status> {
        let mode_display = self.sdo_read_i8(MODES_OF_OPERATION_DISPLAY, 0x00, timeout)?;
        let statusword = self.sdo_read_u16(STATUSWORD, 0x00, timeout)?;
        let error_code = self.sdo_read_u16(ERROR_CODE, 0x00, timeout).unwrap_or(0);
        let position = self
            .sdo_read_i32(POSITION_ACTUAL_VALUE, 0x00, timeout)
            .map(pulses_to_rad)
            .unwrap_or(0.0);
        let velocity = self
            .sdo_read_i32(VELOCITY_ACTUAL_VALUE, 0x00, timeout)
            .map(unit_0p1rpm_to_rad_s)
            .unwrap_or(0.0);
        let torque = self
            .sdo_read_i16(TORQUE_ACTUAL_VALUE, 0x00, timeout)
            .map(robstride_raw_to_torque_nm)
            .unwrap_or(0.0);
        let current_ma = self
            .sdo_read_i16(CURRENT_ACTUAL_VALUE, 0x00, timeout)
            .unwrap_or(0);
        let dc_link_mv = self
            .sdo_read_i32(DC_LINK_CIRCUIT_VOLTAGE, 0x00, timeout)
            .unwrap_or(0);
        let heartbeat_state = self.node.heartbeat_state();
        Ok(RobstrideCia402Status {
            mode_display,
            statusword,
            error_code,
            position_rad: position,
            velocity_rad_s: velocity,
            torque_nm: torque,
            current_ma,
            dc_link_mv,
            heartbeat_state,
        })
    }
}

impl MotorDevice for RobstrideCia402Motor {
    fn vendor(&self) -> &'static str {
        "robstride_cia402"
    }

    fn model(&self) -> &str {
        &self.model
    }

    fn motor_id(&self) -> u16 {
        self.motor_id
    }

    fn feedback_id(&self) -> u16 {
        self.feedback_id
    }

    fn enable(&self) -> Result<()> {
        self.enable_drive(Duration::from_millis(250))
    }

    fn disable(&self) -> Result<()> {
        self.disable_drive(Duration::from_millis(250))
    }

    fn accepts_frame(&self, frame: &CanFrame) -> bool {
        self.node.accepts_std(frame)
    }

    fn process_feedback_frame(&self, frame: CanFrame) -> Result<()> {
        // SDO replies queue up for blocked round trips; heartbeats update the
        // cached state. Anything else is not this motor's frame.
        let _ = self.node.process(&frame);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use motor_core::test_support::MockBus;

    fn ack(index: u16, sub: u8) -> CanFrame {
        CanFrame {
            arbitration_id: sdo_rsp_id(1),
            data: [
                0x60,
                (index & 0xFF) as u8,
                (index >> 8) as u8,
                sub,
                0,
                0,
                0,
                0,
            ],
            dlc: 8,
            is_extended: false,
            is_rx: true,
        }
    }

    #[test]
    fn command_velocity_uses_cia402_sdo_ids() {
        let bus = Arc::new(MockBus::new());
        let motor = RobstrideCia402Motor::new(1, 0, "rs-06", bus.clone()).expect("motor");
        for item in [
            ack(CONTROLWORD, 0),
            ack(MODES_OF_OPERATION, 0),
            ack(CONTROLWORD, 0),
            ack(CONTROLWORD, 0),
            ack(CONTROLWORD, 0),
            ack(TARGET_TORQUE, 0),
            ack(TARGET_VELOCITY, 0),
        ] {
            motor.process_feedback_frame(item).expect("queue sdo ack");
        }
        motor
            .command_velocity(TWO_PI, Duration::from_millis(10))
            .expect("velocity command");
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent[0].arbitration_id, NMT_ID);
        assert_eq!(sent[1].arbitration_id, sdo_req_id(1));
        assert!(!sent[1].is_extended);
        let last = sent.last().expect("last");
        assert_eq!(last.arbitration_id, sdo_req_id(1));
        assert_eq!(last.data[1], (TARGET_VELOCITY & 0xFF) as u8);
        assert_eq!(last.data[2], (TARGET_VELOCITY >> 8) as u8);
        assert_eq!(
            i32::from_le_bytes([last.data[4], last.data[5], last.data[6], last.data[7]]),
            600
        );
    }
}
