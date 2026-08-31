use crate::protocol::{
    build_ext_id, decode_fault_report, decode_ping_reply, decode_read_parameter_value,
    decode_status_frame, encode_mit_command, encode_parameter_read, encode_parameter_value,
    encode_parameter_write, encode_set_protocol, ext_id_parts, validate_protocol_cmd,
    CommunicationType, FaultReport, PingReply,
};
use crate::registers::{parameter_info, ParameterDataType, ParameterId};
use motor_core::bus::{CanBus, CanFrame};
use motor_core::device::MotorDevice;
use motor_core::error::{MotorError, Result};
use motor_core::model::{ModelCatalog, MotorModelSpec, PvTLimits, StaticModelCatalog};
use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex, OnceLock};
use std::time::{Duration, Instant};

const ROBSTRIDE_MODELS: &[MotorModelSpec] = &[
    MotorModelSpec {
        vendor: "robstride",
        model: "rs-00",
        pmax: 4.0 * std::f32::consts::PI,
        vmax: 33.0,
        tmax: 14.0,
    },
    MotorModelSpec {
        vendor: "robstride",
        model: "rs-01",
        pmax: 4.0 * std::f32::consts::PI,
        vmax: 44.0,
        tmax: 17.0,
    },
    MotorModelSpec {
        vendor: "robstride",
        model: "rs-02",
        pmax: 4.0 * std::f32::consts::PI,
        vmax: 44.0,
        tmax: 17.0,
    },
    MotorModelSpec {
        vendor: "robstride",
        model: "rs-03",
        pmax: 4.0 * std::f32::consts::PI,
        vmax: 20.0,
        tmax: 60.0,
    },
    MotorModelSpec {
        vendor: "robstride",
        model: "rs-04",
        pmax: 4.0 * std::f32::consts::PI,
        vmax: 15.0,
        tmax: 120.0,
    },
    MotorModelSpec {
        vendor: "robstride",
        model: "rs-05",
        pmax: 4.0 * std::f32::consts::PI,
        vmax: 50.0,
        tmax: 5.5,
    },
    MotorModelSpec {
        vendor: "robstride",
        model: "rs-06",
        pmax: 4.0 * std::f32::consts::PI,
        vmax: 50.0,
        tmax: 36.0,
    },
];

const ROBSTRIDE_CATALOG: StaticModelCatalog = StaticModelCatalog {
    vendor_name: "robstride",
    models: ROBSTRIDE_MODELS,
};

pub fn model_limits(model: &str) -> Option<(f32, f32, f32)> {
    ROBSTRIDE_CATALOG
        .get(model)
        .map(|spec| (spec.pmax, spec.vmax, spec.tmax))
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControlMode {
    Mit = 0,
    Position = 1,
    Velocity = 2,
    PositionCsp = 5,
}

impl ControlMode {
    pub fn from_raw(value: i8) -> Result<Self> {
        match value {
            0 => Ok(Self::Mit),
            1 => Ok(Self::Position),
            2 => Ok(Self::Velocity),
            5 => Ok(Self::PositionCsp),
            _ => Err(MotorError::Protocol(format!(
                "unsupported RobStride run_mode value {value}"
            ))),
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::Mit => "mit",
            Self::Position => "pp",
            Self::Velocity => "velocity",
            Self::PositionCsp => "csp",
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum ParameterValue {
    I8(i8),
    U8(u8),
    U16(u16),
    U32(u32),
    F32(f32),
}

#[derive(Debug, Clone, Copy)]
pub struct MotorFeedbackState {
    pub arbitration_id: u32,
    pub device_id: u8,
    pub mode_state: u8,
    pub position: f32,
    pub velocity: f32,
    pub torque: f32,
    pub temperature_c: f32,
    pub uncalibrated: bool,
    pub stall: bool,
    pub magnetic_encoder_fault: bool,
    pub overtemperature: bool,
    pub overcurrent: bool,
    pub undervoltage: bool,
}

pub struct RobstrideMotor {
    pub motor_id: u16,
    pub feedback_id: u16,
    pub model: String,
    bus: Arc<dyn CanBus>,
    limits: PvTLimits,
    kp_max: f32,
    kd_max: f32,
    state: Mutex<Option<MotorFeedbackState>>,
    fault_report: Mutex<Option<FaultReport>>,
    status_seq: AtomicU64,
    response_seq: AtomicU64,
    param_state: Mutex<ParameterState>,
    ping_reply: Mutex<Option<PingReply>>,
    last_mit_gains: Mutex<Option<(f32, f32)>>,
}

#[derive(Default)]
struct ParameterState {
    values: HashMap<u16, ParameterValue>,
    pending: Option<u16>,
}

impl RobstrideMotor {
    pub fn new(motor_id: u16, feedback_id: u16, model: &str, bus: Arc<dyn CanBus>) -> Result<Self> {
        Self::validate_device_id(motor_id, "motor_id")?;
        Self::validate_host_id(feedback_id, "feedback_id")?;
        let spec = ROBSTRIDE_CATALOG.get(model).ok_or_else(|| {
            MotorError::InvalidArgument(format!("unknown RobStride model: {model}"))
        })?;
        let (kp_max, kd_max) = match model {
            "rs-00" | "rs-01" | "rs-02" | "rs-05" => (500.0, 5.0),
            "rs-03" | "rs-04" | "rs-06" => (5000.0, 100.0),
            _ => (500.0, 5.0),
        };
        Ok(Self {
            motor_id,
            feedback_id,
            model: model.to_string(),
            bus,
            limits: PvTLimits::from_spec(spec),
            kp_max,
            kd_max,
            state: Mutex::new(None),
            fault_report: Mutex::new(None),
            status_seq: AtomicU64::new(0),
            response_seq: AtomicU64::new(0),
            param_state: Mutex::new(ParameterState::default()),
            ping_reply: Mutex::new(None),
            last_mit_gains: Mutex::new(None),
        })
    }

    fn device_id_u8(&self) -> Result<u8> {
        Ok(self.motor_id as u8)
    }

    fn host_id_u16(&self) -> u16 {
        self.feedback_id
    }

    fn host_id_u8(&self) -> u8 {
        self.host_id_u16() as u8
    }

    fn host_id_candidates(&self) -> Vec<u16> {
        let mut cands = vec![self.feedback_id, 0x00FD, 0x00FF, 0x00FE];
        cands.dedup();
        cands
    }

    fn validate_device_id(id: u16, name: &str) -> Result<()> {
        if (1..=255).contains(&id) {
            Ok(())
        } else {
            Err(MotorError::InvalidArgument(format!(
                "RobStride {name} must be in 1..255, got {id}"
            )))
        }
    }

    fn validate_host_id(id: u16, name: &str) -> Result<()> {
        if id <= 255 {
            Ok(())
        } else {
            Err(MotorError::InvalidArgument(format!(
                "RobStride {name}/host_id must be in 0..255, got {id}"
            )))
        }
    }

    fn send_ext(&self, comm_type: u32, extra_data: u16, data: [u8; 8], dlc: u8) -> Result<()> {
        self.bus.send(CanFrame {
            arbitration_id: build_ext_id(comm_type, extra_data, self.device_id_u8()?),
            data,
            dlc,
            is_extended: true,
            is_rx: false,
        })
    }

    fn control_mode_value(mode: ControlMode) -> i8 {
        mode as i8
    }

    fn send_with_status_ack(
        &self,
        comm_type: u32,
        data: [u8; 8],
        dlc: u8,
        timeout: Duration,
    ) -> Result<()> {
        let cands = self.host_id_candidates();
        let per_try = Duration::from_millis((timeout.as_millis() as u64).max(120));
        for host in cands {
            let start_seq = self.status_seq.load(Ordering::Acquire);
            self.send_ext(comm_type, host, data, dlc)?;
            let deadline = Instant::now() + per_try;
            while Instant::now() < deadline {
                if self.status_seq.load(Ordering::Acquire) > start_seq {
                    return Ok(());
                }
                std::thread::sleep(Duration::from_millis(4));
            }
        }
        Err(MotorError::Timeout(format!(
            "control ack timeout: comm_type={comm_type}"
        )))
    }

    fn send_with_response_ack(
        &self,
        comm_type: u32,
        data: [u8; 8],
        dlc: u8,
        timeout: Duration,
    ) -> Result<()> {
        let cands = self.host_id_candidates();
        let per_try = Duration::from_millis((timeout.as_millis() as u64).max(120));
        for host in cands {
            let start_seq = self.response_seq.load(Ordering::Acquire);
            self.send_ext(comm_type, host, data, dlc)?;
            let deadline = Instant::now() + per_try;
            while Instant::now() < deadline {
                if self.response_seq.load(Ordering::Acquire) > start_seq {
                    return Ok(());
                }
                std::thread::sleep(Duration::from_millis(4));
            }
        }
        Err(MotorError::Timeout(format!(
            "response ack timeout: comm_type={comm_type}"
        )))
    }

    pub fn ping(&self, timeout: Duration) -> Result<PingReply> {
        let cands = self.host_id_candidates();
        let per_try = Duration::from_millis((timeout.as_millis() as u64).max(120));
        for host in cands {
            if let Ok(reply) = self.ping_with_host_id(host, per_try) {
                return Ok(reply);
            }
        }
        Err(MotorError::Timeout(format!(
            "ping {} timed out",
            self.motor_id
        )))
    }

    pub fn ping_with_host_id(&self, host_id: u16, timeout: Duration) -> Result<PingReply> {
        Self::validate_host_id(host_id, "feedback_id")?;
        self.ping_reply
            .lock()
            .map_err(|_| MotorError::Io("ping reply lock poisoned".to_string()))?
            .take();
        self.send_ext(CommunicationType::GET_DEVICE_ID, host_id, [0u8; 8], 8)?;
        let deadline = Instant::now() + timeout;
        loop {
            if let Some(reply) = *self
                .ping_reply
                .lock()
                .map_err(|_| MotorError::Io("ping reply lock poisoned".to_string()))?
            {
                return Ok(reply);
            }
            if Instant::now() >= deadline {
                break;
            }
            std::thread::sleep(Duration::from_millis(8));
        }
        Err(MotorError::Timeout(format!(
            "ping {} timed out for host_id 0x{host_id:X}",
            self.motor_id
        )))
    }

    pub fn set_mode(&self, mode: ControlMode) -> Result<()> {
        self.write_parameter(ParameterId::Mode as u16, ParameterValue::I8(mode as i8))
    }

    pub fn ensure_control_mode(&self, mode: ControlMode, timeout: Duration) -> Result<()> {
        let desired = Self::control_mode_value(mode);
        let read_timeout = timeout.max(Duration::from_millis(150));

        if let Ok(current) = self.get_parameter_i8(ParameterId::Mode as u16, read_timeout) {
            if current == desired {
                return Ok(());
            }
        }

        // Some RobStride firmware variants ignore run_mode writes while torque is
        // enabled. Disable first, but still rely on the final run_mode readback
        // as the source of truth.
        let mut last_error = self.disable().err().map(|e| e.to_string());
        std::thread::sleep(Duration::from_millis(60));

        let mut actual = None;
        for attempt in 0..3 {
            if let Err(err) = self.set_mode(mode) {
                last_error = Some(err.to_string());
                eprintln!(
                    "[warn] ensure_control_mode: set_mode attempt {} failed: {}",
                    attempt + 1,
                    err
                );
                std::thread::sleep(Duration::from_millis(30));
                continue;
            }

            std::thread::sleep(Duration::from_millis(30));
            match self.get_parameter_i8(ParameterId::Mode as u16, read_timeout) {
                Ok(value) if value == desired => {
                    if attempt > 0 {
                        eprintln!(
                            "[info] ensure_control_mode: mode switch succeeded on attempt {}",
                            attempt + 1
                        );
                    }
                    return Ok(());
                }
                Ok(value) => {
                    actual = Some(value);
                    last_error = None;
                    eprintln!(
                        "[warn] ensure_control_mode: mode readback mismatch, expected {} got {}",
                        desired, value
                    );
                }
                Err(err) => {
                    last_error = Some(err.to_string());
                    eprintln!("[warn] ensure_control_mode: mode readback failed: {}", err);
                }
            }
            std::thread::sleep(Duration::from_millis(30));
        }

        Err(MotorError::Protocol(format!(
            "control mode verify failed: expected {desired}, got {actual:?}, last_error={last_error:?}"
        )))
    }

    pub fn set_zero_position(&self) -> Result<()> {
        let mut payload = [0u8; 8];
        payload[0] = 0x01;
        self.send_with_status_ack(
            CommunicationType::SET_ZERO_POSITION,
            payload,
            8,
            Duration::from_millis(320),
        )?;

        // RobStride zeroing is two-part: type 6 sets the mechanical zero, while
        // zero_sta selects the startup coordinate range. Keep the unified upper
        // set-zero API and make RobStride default to -pi..pi after zeroing.
        self.write_parameter(ParameterId::ZeroState as u16, ParameterValue::U8(1))
    }

    pub fn save_parameters(&self) -> Result<()> {
        self.send_with_response_ack(
            CommunicationType::SAVE_PARAMETERS,
            [1, 2, 3, 4, 5, 6, 7, 8],
            8,
            Duration::from_millis(500),
        )
    }

    pub fn get_protocol_flag(&self, timeout: Duration) -> Result<u8> {
        match self.get_parameter(ParameterId::ProtocolFlag as u16, timeout)? {
            ParameterValue::U8(v) => Ok(v),
            other => Err(MotorError::Protocol(format!(
                "protocol flag parameter returned unexpected value {other:?}"
            ))),
        }
    }

    pub fn set_protocol(&self, protocol_cmd: u8, timeout: Duration) -> Result<()> {
        validate_protocol_cmd(protocol_cmd)?;
        let data = encode_set_protocol(protocol_cmd)?;
        if timeout.is_zero() {
            return self.send_ext(CommunicationType::SET_PROTOCOL, self.host_id_u16(), data, 8);
        }
        self.send_with_response_ack(CommunicationType::SET_PROTOCOL, data, 8, timeout)
    }

    pub fn set_device_id(&self, new_id: u8) -> Result<()> {
        Self::validate_device_id(u16::from(new_id), "new_device_id")?;
        let extra = (u16::from(new_id) << 8) | u16::from(self.host_id_u8());
        let payload = self.ping(Duration::from_millis(140))?.payload;
        self.send_ext(CommunicationType::SET_DEVICE_ID, extra, payload, 8)
    }

    pub fn enable(&self) -> Result<()> {
        self.send_with_status_ack(
            CommunicationType::ENABLE,
            [0u8; 8],
            8,
            Duration::from_millis(240),
        )
    }

    pub fn disable(&self) -> Result<()> {
        self.send_with_status_ack(
            CommunicationType::DISABLE,
            [0u8; 8],
            8,
            Duration::from_millis(240),
        )
    }

    pub fn clear_error(&self) -> Result<()> {
        self.send_with_status_ack(
            CommunicationType::DISABLE,
            [1, 0, 0, 0, 0, 0, 0, 0],
            8,
            Duration::from_millis(240),
        )?;
        self.fault_report
            .lock()
            .map_err(|_| MotorError::Io("fault report lock poisoned".to_string()))?
            .take();
        Ok(())
    }

    pub fn set_active_report(&self, enabled: bool) -> Result<()> {
        let cmd = if enabled { 1 } else { 0 };
        let data = [1, 2, 3, 4, 5, 6, cmd, 0];
        self.send_ext(
            CommunicationType::ACTIVE_REPORT,
            self.host_id_u16(),
            data,
            8,
        )
    }

    pub fn send_cmd_mit(
        &self,
        target_position: f32,
        target_velocity: f32,
        stiffness: f32,
        damping: f32,
        feedforward_torque: f32,
    ) -> Result<()> {
        let (extra_data, data) = encode_mit_command(
            target_position,
            target_velocity,
            stiffness,
            damping,
            feedforward_torque,
            self.limits.p_max,
            self.limits.v_max,
            self.limits.t_max,
            self.kp_max,
            self.kd_max,
        );
        self.send_ext(CommunicationType::OPERATION_CONTROL, extra_data, data, 8)?;
        if let Ok(mut gains) = self.last_mit_gains.lock() {
            *gains = Some((stiffness, damping));
        }
        Ok(())
    }

    pub fn set_velocity_target(&self, velocity: f32) -> Result<()> {
        self.write_parameter(
            ParameterId::VelocityTarget as u16,
            ParameterValue::F32(velocity),
        )
    }

    pub fn get_control_mode(&self, timeout: Duration) -> Result<ControlMode> {
        ControlMode::from_raw(self.get_parameter_i8(ParameterId::Mode as u16, timeout)?)
    }

    pub fn controlled_stop(&self, timeout: Duration) -> Result<ControlMode> {
        let mode = self.get_control_mode(timeout)?;
        match mode {
            ControlMode::Velocity => {
                self.set_velocity_target(0.0)?;
            }
            ControlMode::Position => {
                self.write_parameter(ParameterId::PpVelocityMax as u16, ParameterValue::F32(0.0))?;
            }
            ControlMode::PositionCsp => {
                let position =
                    self.get_parameter_f32(ParameterId::MechanicalPosition as u16, timeout)?;
                self.write_parameter(
                    ParameterId::PositionTarget as u16,
                    ParameterValue::F32(position),
                )?;
            }
            ControlMode::Mit => {
                const DEFAULT_KP_RATIO: f32 = 0.10;
                const DEFAULT_KD_RATIO: f32 = 0.10;
                const MIN_KP_RATIO: f32 = 0.02;
                const MIN_KD_RATIO: f32 = 0.02;

                let (cached_kp, cached_kd) = {
                    let gains = self
                        .last_mit_gains
                        .lock()
                        .map_err(|_| MotorError::Io("last_mit_gains lock poisoned".to_string()))?;
                    gains.unwrap_or((0.0, 0.0))
                };

                let (kp, kd) = {
                    let min_kp = self.kp_max * MIN_KP_RATIO;
                    let min_kd = self.kd_max * MIN_KD_RATIO;
                    let default_kp = self.kp_max * DEFAULT_KP_RATIO;
                    let default_kd = self.kd_max * DEFAULT_KD_RATIO;

                    let effective_kp = if cached_kp < min_kp {
                        eprintln!(
                            "[warn] MIT stop: cached kp={} too small (min={}), using default={}",
                            cached_kp, min_kp, default_kp
                        );
                        default_kp
                    } else {
                        cached_kp
                    };

                    let effective_kd = if cached_kd < min_kd {
                        eprintln!(
                            "[warn] MIT stop: cached kd={} too small (min={}), using default={}",
                            cached_kd, min_kd, default_kd
                        );
                        default_kd
                    } else {
                        cached_kd
                    };

                    (effective_kp, effective_kd)
                };

                let position =
                    self.get_parameter_f32(ParameterId::MechanicalPosition as u16, timeout)?;
                self.send_cmd_mit(position, 0.0, kp, kd, 0.0)?;
            }
        }
        Ok(mode)
    }

    pub fn write_parameter(&self, param_id: u16, value: ParameterValue) -> Result<()> {
        let raw = encode_parameter_value(param_id, value)?;
        let data = encode_parameter_write(param_id, raw);
        let timeout = Self::write_ack_timeout();
        if timeout.is_zero() {
            return self.send_ext(
                CommunicationType::WRITE_PARAMETER,
                self.host_id_u16(),
                data,
                8,
            );
        }
        self.send_with_status_ack(CommunicationType::WRITE_PARAMETER, data, 8, timeout)
    }

    fn write_ack_timeout() -> Duration {
        static TIMEOUT: OnceLock<Duration> = OnceLock::new();
        *TIMEOUT.get_or_init(|| {
            std::env::var("MOTORBRIDGE_ROBSTRIDE_WRITE_ACK_TIMEOUT_MS")
                .ok()
                .and_then(|s| s.parse::<u64>().ok())
                .map(Duration::from_millis)
                .unwrap_or(Duration::ZERO)
        })
    }

    pub fn send_cmd_pos_vel_csp(&self, target_position: f32, velocity_limit: f32) -> Result<()> {
        self.set_mode(ControlMode::PositionCsp)?;
        self.enable()?;
        let vlim = velocity_limit.abs();
        if vlim.is_finite() && vlim > 0.0 {
            self.write_parameter(ParameterId::VelocityLimit as u16, ParameterValue::F32(vlim))?;
        }
        self.write_parameter(
            ParameterId::PositionTarget as u16,
            ParameterValue::F32(target_position),
        )
    }

    pub fn send_cmd_pos_vel_pp(
        &self,
        target_position: f32,
        velocity_max: f32,
        acceleration: f32,
    ) -> Result<()> {
        self.set_mode(ControlMode::Position)?;
        self.enable()?;
        let vel_max = velocity_max.abs();
        if vel_max.is_finite() && vel_max > 0.0 {
            self.write_parameter(
                ParameterId::PpVelocityMax as u16,
                ParameterValue::F32(vel_max),
            )?;
        }
        let acc_set = acceleration.abs();
        if acc_set.is_finite() && acc_set > 0.0 {
            self.write_parameter(
                ParameterId::PpAccelerationTarget as u16,
                ParameterValue::F32(acc_set),
            )?;
        }
        self.write_parameter(
            ParameterId::PositionTarget as u16,
            ParameterValue::F32(target_position),
        )
    }

    pub fn request_parameter(&self, param_id: u16) -> Result<()> {
        let mut ps = self
            .param_state
            .lock()
            .map_err(|_| MotorError::Io("param state lock poisoned".to_string()))?;
        ps.values.remove(&param_id);
        ps.pending.replace(param_id);
        drop(ps);
        let data = encode_parameter_read(param_id);
        self.send_ext(
            CommunicationType::READ_PARAMETER,
            self.host_id_u16(),
            data,
            8,
        )
    }

    pub fn get_parameter(&self, param_id: u16, timeout: Duration) -> Result<ParameterValue> {
        let cands = self.host_id_candidates();
        let per_try = Duration::from_millis((timeout.as_millis() as u64).max(150));

        for host in cands {
            if let Ok(value) = self.get_parameter_with_host_id(param_id, host, per_try) {
                return Ok(value);
            }
        }
        Err(MotorError::Timeout(format!(
            "parameter 0x{param_id:04X} not received within {:?}",
            timeout
        )))
    }

    pub fn get_parameter_with_host_id(
        &self,
        param_id: u16,
        host_id: u16,
        timeout: Duration,
    ) -> Result<ParameterValue> {
        Self::validate_host_id(host_id, "feedback_id")?;
        let mut ps = self
            .param_state
            .lock()
            .map_err(|_| MotorError::Io("param state lock poisoned".to_string()))?;
        ps.values.remove(&param_id);
        ps.pending.replace(param_id);
        drop(ps);
        let data = encode_parameter_read(param_id);
        self.send_ext(CommunicationType::READ_PARAMETER, host_id, data, 8)?;

        let deadline = Instant::now() + timeout;
        loop {
            if let Some(value) = self
                .param_state
                .lock()
                .map_err(|_| MotorError::Io("param state lock poisoned".to_string()))?
                .values
                .get(&param_id)
                .copied()
            {
                return Ok(value);
            }
            if Instant::now() >= deadline {
                break;
            }
            std::thread::sleep(Duration::from_millis(8));
        }
        Err(MotorError::Timeout(format!(
            "parameter 0x{param_id:04X} not received within {:?} for host_id 0x{host_id:X}",
            timeout
        )))
    }

    pub fn get_parameter_f32(&self, param_id: u16, timeout: Duration) -> Result<f32> {
        match self.get_parameter(param_id, timeout)? {
            ParameterValue::F32(v) => Ok(v),
            _ => Err(MotorError::Protocol(format!(
                "parameter 0x{param_id:04X} is not f32"
            ))),
        }
    }

    pub fn get_parameter_i8(&self, param_id: u16, timeout: Duration) -> Result<i8> {
        match self.get_parameter(param_id, timeout)? {
            ParameterValue::I8(v) => Ok(v),
            _ => Err(MotorError::Protocol(format!(
                "parameter 0x{param_id:04X} is not i8"
            ))),
        }
    }

    pub fn latest_state(&self) -> Option<MotorFeedbackState> {
        self.state.lock().ok().and_then(|s| *s)
    }

    pub fn latest_fault_report(&self) -> Option<FaultReport> {
        self.fault_report.lock().ok().and_then(|s| *s)
    }

    fn process_feedback_frame_impl(&self, frame: CanFrame) -> Result<()> {
        self.response_seq.fetch_add(1, Ordering::Release);
        let (comm_type, extra_data, _) = ext_id_parts(frame.arbitration_id);
        match comm_type {
            CommunicationType::GET_DEVICE_ID => {
                let reply = decode_ping_reply(frame.arbitration_id, frame.data)?;
                self.ping_reply
                    .lock()
                    .map_err(|_| MotorError::Io("ping reply lock poisoned".to_string()))?
                    .replace(reply);
                Ok(())
            }
            CommunicationType::READ_PARAMETER => {
                let mut ps = self
                    .param_state
                    .lock()
                    .map_err(|_| MotorError::Io("param state lock poisoned".to_string()))?;
                let param_id = ps
                    .pending
                    .take()
                    .unwrap_or_else(|| u16::from_le_bytes([frame.data[0], frame.data[1]]));
                let raw = decode_read_parameter_value(param_id, frame.data)?;
                let value = if let Some(info) = parameter_info(param_id) {
                    match info.data_type {
                        ParameterDataType::Int8 => ParameterValue::I8(raw[0] as i8),
                        ParameterDataType::UInt8 => ParameterValue::U8(raw[0]),
                        ParameterDataType::UInt16 => {
                            ParameterValue::U16(u16::from_le_bytes([raw[0], raw[1]]))
                        }
                        ParameterDataType::UInt32 => ParameterValue::U32(u32::from_le_bytes(raw)),
                        ParameterDataType::Float32 => ParameterValue::F32(f32::from_le_bytes(raw)),
                    }
                } else {
                    // Tolerate unknown vendor firmware params instead of surfacing hard errors
                    // in polling worker logs. Preserve raw payload as U32 for diagnostics.
                    ParameterValue::U32(u32::from_le_bytes(raw))
                };
                ps.values.insert(param_id, value);
                Ok(())
            }
            CommunicationType::OPERATION_STATUS => {
                let status = decode_status_frame(
                    extra_data,
                    frame.data,
                    self.limits.p_max,
                    self.limits.v_max,
                    self.limits.t_max,
                );
                self.state
                    .lock()
                    .map_err(|_| MotorError::Io("state lock poisoned".to_string()))?
                    .replace(MotorFeedbackState {
                        arbitration_id: frame.arbitration_id,
                        device_id: status.flags.device_id,
                        mode_state: status.flags.mode_state,
                        position: status.position,
                        velocity: status.velocity,
                        torque: status.torque,
                        temperature_c: status.temperature_c,
                        uncalibrated: status.flags.uncalibrated,
                        stall: status.flags.stall,
                        magnetic_encoder_fault: status.flags.magnetic_encoder_fault,
                        overtemperature: status.flags.overtemperature,
                        overcurrent: status.flags.overcurrent,
                        undervoltage: status.flags.undervoltage,
                    });
                self.status_seq.fetch_add(1, Ordering::Release);
                Ok(())
            }
            CommunicationType::FAULT_REPORT => {
                self.fault_report
                    .lock()
                    .map_err(|_| MotorError::Io("fault report lock poisoned".to_string()))?
                    .replace(decode_fault_report(frame.data));
                // Fault reports are real device responses, but their payload is not a
                // position/velocity status payload or a control ACK.
                Ok(())
            }
            _ => Ok(()),
        }
    }
}

impl MotorDevice for RobstrideMotor {
    fn vendor(&self) -> &'static str {
        "robstride"
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
        RobstrideMotor::enable(self)
    }

    fn disable(&self) -> Result<()> {
        RobstrideMotor::disable(self)
    }

    fn accepts_frame(&self, frame: &CanFrame) -> bool {
        if !frame.is_extended {
            return false;
        }
        let (comm_type, extra_data, _responder_id) = ext_id_parts(frame.arbitration_id);
        let device_id = extra_data & 0xFF;
        match comm_type {
            CommunicationType::GET_DEVICE_ID => device_id == self.motor_id,
            CommunicationType::READ_PARAMETER => device_id == self.motor_id,
            CommunicationType::SET_PROTOCOL => device_id == self.motor_id,
            // Status/fault frames must belong to this motor. Accepting only by responder_id
            // can pollute state with frames from other motors on the same bus.
            CommunicationType::OPERATION_STATUS | CommunicationType::FAULT_REPORT => {
                device_id == self.motor_id
            }
            _ => false,
        }
    }

    fn process_feedback_frame(&self, frame: CanFrame) -> Result<()> {
        self.process_feedback_frame_impl(frame)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use motor_core::device::MotorDevice;
    use motor_core::test_support::MockBus;

    #[test]
    fn get_parameter_times_out_when_no_reply_arrives() {
        let bus: Arc<dyn CanBus> = Arc::new(MockBus::new());
        let motor = RobstrideMotor::new(127, 0xFF, "rs-00", bus).expect("create motor");
        let err = motor
            .get_parameter(0x7019, Duration::from_millis(5))
            .expect_err("timeout expected");
        assert!(matches!(err, MotorError::Timeout(_)));
    }

    #[test]
    fn constructor_rejects_out_of_range_ids() {
        let bus: Arc<dyn CanBus> = Arc::new(MockBus::new());
        assert!(RobstrideMotor::new(0, 0xFD, "rs-00", Arc::clone(&bus)).is_err());
        assert!(RobstrideMotor::new(256, 0xFD, "rs-00", Arc::clone(&bus)).is_err());
        assert!(RobstrideMotor::new(1, 0x100, "rs-00", bus).is_err());
    }

    #[test]
    fn ping_with_host_id_uses_exact_host_without_fallback() {
        let bus = Arc::new(MockBus::new());
        let motor = RobstrideMotor::new(2, 0xFD, "rs-00", bus.clone()).expect("create motor");
        let err = motor
            .ping_with_host_id(0xAA, Duration::from_millis(1))
            .expect_err("timeout expected");
        assert!(matches!(err, MotorError::Timeout(_)));

        let sent = bus.sent.lock().expect("sent frames");
        assert_eq!(sent.len(), 1);
        let (comm_type, extra_data, node_id) = ext_id_parts(sent[0].arbitration_id);
        assert_eq!(comm_type, CommunicationType::GET_DEVICE_ID);
        assert_eq!(extra_data, 0x00AA);
        assert_eq!(node_id, 2);
    }

    #[test]
    fn read_parameter_filter_rejects_other_device_with_same_host() {
        let bus: Arc<dyn CanBus> = Arc::new(MockBus::new());
        let motor = RobstrideMotor::new(2, 0xFD, "rs-00", bus).expect("create motor");
        let frame = CanFrame {
            arbitration_id: build_ext_id(CommunicationType::READ_PARAMETER, 0x0003, 0xFD),
            data: encode_parameter_read(0x7019),
            dlc: 8,
            is_extended: true,
            is_rx: true,
        };
        assert!(!motor.accepts_frame(&frame));
    }

    #[test]
    fn send_cmd_mit_sends_operation_control_with_encoded_inputs() {
        let bus = Arc::new(MockBus::new());
        let motor = RobstrideMotor::new(2, 0xFD, "rs-00", bus.clone()).expect("create motor");

        motor
            .send_cmd_mit(0.1, 0.2, 3.0, 0.4, 0.5)
            .expect("send mit");

        let sent = bus.sent.lock().expect("sent frames");
        assert_eq!(sent.len(), 1);
        let (expected_extra, expected_data) = encode_mit_command(
            0.1,
            0.2,
            3.0,
            0.4,
            0.5,
            motor.limits.p_max,
            motor.limits.v_max,
            motor.limits.t_max,
            motor.kp_max,
            motor.kd_max,
        );
        let (comm_type, extra_data, node_id) = ext_id_parts(sent[0].arbitration_id);
        assert_eq!(comm_type, CommunicationType::OPERATION_CONTROL);
        assert_eq!(extra_data, expected_extra);
        assert_eq!(node_id, 2);
        assert_eq!(sent[0].data, expected_data);
        assert_eq!(sent[0].dlc, 8);
        assert!(sent[0].is_extended);
        assert!(!sent[0].is_rx);
    }

    #[test]
    fn fault_report_does_not_overwrite_latest_state() {
        let bus: Arc<dyn CanBus> = Arc::new(MockBus::new());
        let motor = RobstrideMotor::new(2, 0xFD, "rs-00", bus).expect("create motor");

        motor
            .process_feedback_frame(CanFrame {
                arbitration_id: build_ext_id(CommunicationType::OPERATION_STATUS, 0x0002, 0xFD),
                data: [0x90, 0x00, 0x80, 0x00, 0x7F, 0xFF, 0x05, 0x78],
                dlc: 8,
                is_extended: true,
                is_rx: true,
            })
            .expect("status frame");
        let before = motor.latest_state().expect("state from status");
        let seq_before = motor.status_seq.load(Ordering::Acquire);

        motor
            .process_feedback_frame(CanFrame {
                arbitration_id: build_ext_id(CommunicationType::FAULT_REPORT, 0x0002, 0xFD),
                data: [0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00],
                dlc: 8,
                is_extended: true,
                is_rx: true,
            })
            .expect("fault frame");

        let after = motor.latest_state().expect("state should remain");
        let seq_after = motor.status_seq.load(Ordering::Acquire);
        assert_eq!(after.arbitration_id, before.arbitration_id);
        assert_eq!(after.position, before.position);
        assert_eq!(after.velocity, before.velocity);
        assert_eq!(after.torque, before.torque);
        assert_eq!(after.temperature_c, before.temperature_c);
        assert_eq!(seq_after, seq_before);

        let fault = motor.latest_fault_report().expect("fault report");
        assert_eq!(fault.fault_raw, 0);
        assert_eq!(fault.warning_raw, 1);
        assert!(fault.warnings.overtemperature_warning);
    }

    #[test]
    fn save_parameters_accepts_non_status_device_reply() {
        let bus_impl = Arc::new(MockBus::new());
        let bus: Arc<dyn CanBus> = bus_impl.clone();
        let motor = Arc::new(RobstrideMotor::new(1, 0xFD, "rs-00", bus).expect("create motor"));
        let responder = Arc::clone(&motor);
        let bus_for_thread = Arc::clone(&bus_impl);

        let handle = std::thread::spawn(move || {
            let deadline = Instant::now() + Duration::from_millis(100);
            loop {
                let saw_save = {
                    let sent = bus_for_thread.sent.lock().expect("sent lock");
                    sent.iter().any(|f| {
                        let (comm_type, _, _) = ext_id_parts(f.arbitration_id);
                        comm_type == CommunicationType::SAVE_PARAMETERS
                    })
                };
                if saw_save {
                    responder
                        .process_feedback_frame(CanFrame {
                            arbitration_id: build_ext_id(
                                CommunicationType::GET_DEVICE_ID,
                                0x0001,
                                0xFE,
                            ),
                            data: [0x35, 0x10, 0x32, 0x31, 0x30, 0x37, 0x35, 0x0D],
                            dlc: 8,
                            is_extended: true,
                            is_rx: true,
                        })
                        .expect("process save reply");
                    return;
                }
                if Instant::now() >= deadline {
                    return;
                }
                std::thread::sleep(Duration::from_millis(1));
            }
        });

        motor
            .save_parameters()
            .expect("save should accept device reply");
        handle.join().expect("responder thread");
    }

    fn parameter_write(sent: &[CanFrame], param_id: u16) -> Option<[u8; 4]> {
        sent.iter().find_map(|frame| {
            let (comm_type, _, _) = ext_id_parts(frame.arbitration_id);
            if comm_type == CommunicationType::WRITE_PARAMETER
                && u16::from_le_bytes([frame.data[0], frame.data[1]]) == param_id
            {
                Some([frame.data[4], frame.data[5], frame.data[6], frame.data[7]])
            } else {
                None
            }
        })
    }

    fn assert_no_torque_off_frame(sent: &[CanFrame]) {
        assert!(sent.iter().all(|frame| {
            let (comm_type, _, _) = ext_id_parts(frame.arbitration_id);
            comm_type != CommunicationType::DISABLE
        }));
    }

    fn spawn_param_responder(
        motor: Arc<RobstrideMotor>,
        bus: Arc<MockBus>,
        replies: Vec<(u16, ParameterValue)>,
    ) -> std::thread::JoinHandle<()> {
        std::thread::spawn(move || {
            let mut replied = vec![false; replies.len()];
            let deadline = Instant::now() + Duration::from_millis(500);
            while replied.iter().any(|&r| !r) && Instant::now() < deadline {
                let sent_snapshot = {
                    let sent = bus.sent.lock().expect("sent frames");
                    sent.clone()
                };
                for frame in &sent_snapshot {
                    let (comm_type, _, _) = ext_id_parts(frame.arbitration_id);
                    if comm_type == CommunicationType::READ_PARAMETER {
                        let req_param_id = u16::from_le_bytes([frame.data[0], frame.data[1]]);
                        for (i, (param_id, value)) in replies.iter().enumerate() {
                            if req_param_id == *param_id && !replied[i] {
                                let raw = encode_parameter_value(*param_id, *value)
                                    .expect("encode param");
                                motor
                                    .process_feedback_frame(CanFrame {
                                        arbitration_id: build_ext_id(
                                            CommunicationType::READ_PARAMETER,
                                            motor.motor_id,
                                            motor.feedback_id as u8,
                                        ),
                                        data: encode_parameter_write(*param_id, raw),
                                        dlc: 8,
                                        is_extended: true,
                                        is_rx: true,
                                    })
                                    .expect("process param reply");
                                replied[i] = true;
                                break;
                            }
                        }
                    }
                }
                std::thread::sleep(Duration::from_millis(1));
            }
        })
    }

    #[test]
    fn controlled_stop_in_pp_writes_vel_max_zero_without_disabling() {
        let bus = Arc::new(MockBus::new());
        let motor =
            Arc::new(RobstrideMotor::new(2, 0xFD, "rs-00", bus.clone()).expect("create motor"));

        let responder = spawn_param_responder(
            Arc::clone(&motor),
            Arc::clone(&bus),
            vec![(ParameterId::Mode as u16, ParameterValue::I8(1))],
        );

        let mode = motor
            .controlled_stop(Duration::from_millis(100))
            .expect("controlled PP stop");
        responder.join().expect("responder thread");

        assert_eq!(mode, ControlMode::Position);
        let sent = bus.sent.lock().expect("sent frames");
        assert_eq!(
            parameter_write(&sent, ParameterId::PpVelocityMax as u16),
            Some(0.0f32.to_le_bytes())
        );
        assert_no_torque_off_frame(&sent);
    }

    #[test]
    fn controlled_stop_in_velocity_mode_writes_zero_speed_without_disabling() {
        let bus = Arc::new(MockBus::new());
        let motor =
            Arc::new(RobstrideMotor::new(2, 0xFD, "rs-00", bus.clone()).expect("create motor"));

        let responder = spawn_param_responder(
            Arc::clone(&motor),
            Arc::clone(&bus),
            vec![(ParameterId::Mode as u16, ParameterValue::I8(2))],
        );

        let mode = motor
            .controlled_stop(Duration::from_millis(100))
            .expect("controlled velocity stop");
        responder.join().expect("responder thread");

        assert_eq!(mode, ControlMode::Velocity);
        let sent = bus.sent.lock().expect("sent frames");
        assert_eq!(
            parameter_write(&sent, ParameterId::VelocityTarget as u16),
            Some(0.0f32.to_le_bytes())
        );
        assert_no_torque_off_frame(&sent);
    }

    #[test]
    fn controlled_stop_in_csp_holds_measured_position_without_disabling() {
        let bus = Arc::new(MockBus::new());
        let motor =
            Arc::new(RobstrideMotor::new(2, 0xFD, "rs-00", bus.clone()).expect("create motor"));

        let responder = spawn_param_responder(
            Arc::clone(&motor),
            Arc::clone(&bus),
            vec![
                (ParameterId::Mode as u16, ParameterValue::I8(5)),
                (
                    ParameterId::MechanicalPosition as u16,
                    ParameterValue::F32(0.42),
                ),
            ],
        );

        let mode = motor
            .controlled_stop(Duration::from_millis(100))
            .expect("controlled CSP stop");
        responder.join().expect("responder");

        assert_eq!(mode, ControlMode::PositionCsp);
        let sent = bus.sent.lock().expect("sent frames");
        assert_eq!(
            parameter_write(&sent, ParameterId::PositionTarget as u16),
            Some(0.42f32.to_le_bytes())
        );
        assert_no_torque_off_frame(&sent);
    }

    #[test]
    fn controlled_stop_in_mit_holds_position_with_cached_gains() {
        let bus = Arc::new(MockBus::new());
        let motor =
            Arc::new(RobstrideMotor::new(2, 0xFD, "rs-00", bus.clone()).expect("create motor"));

        {
            let mut gains = motor.last_mit_gains.lock().expect("lock gains");
            *gains = Some((12.0, 0.5));
        }

        let responder = spawn_param_responder(
            Arc::clone(&motor),
            Arc::clone(&bus),
            vec![
                (ParameterId::Mode as u16, ParameterValue::I8(0)), // MIT mode
                (
                    ParameterId::MechanicalPosition as u16,
                    ParameterValue::F32(-0.25),
                ),
            ],
        );

        let mode = motor
            .controlled_stop(Duration::from_millis(100))
            .expect("controlled MIT stop");
        responder.join().expect("responder");

        assert_eq!(mode, ControlMode::Mit);
        let sent = bus.sent.lock().expect("sent frames");
        // 断言发出了 OPERATION_CONTROL 帧（MIT 命令帧）
        let operation_frame = sent
            .iter()
            .find(|frame| {
                let (comm_type, _, _) = ext_id_parts(frame.arbitration_id);
                comm_type == CommunicationType::OPERATION_CONTROL
            })
            .expect("MIT hold frame");
        let (expected_extra, expected_data) = encode_mit_command(
            -0.25,
            0.0,
            12.0,
            0.5,
            0.0,
            motor.limits.p_max,
            motor.limits.v_max,
            motor.limits.t_max,
            motor.kp_max,
            motor.kd_max,
        );
        let (_, actual_extra, _) = ext_id_parts(operation_frame.arbitration_id);
        assert_eq!(actual_extra, expected_extra);
        assert_eq!(operation_frame.data, expected_data);
        assert_no_torque_off_frame(&sent);
    }

    #[test]
    fn controlled_stop_in_mit_uses_default_gains_when_no_prior_command() {
        let bus = Arc::new(MockBus::new());
        let motor =
            Arc::new(RobstrideMotor::new(2, 0xFD, "rs-00", bus.clone()).expect("create motor"));

        let responder = spawn_param_responder(
            Arc::clone(&motor),
            Arc::clone(&bus),
            vec![
                (ParameterId::Mode as u16, ParameterValue::I8(0)),
                (
                    ParameterId::MechanicalPosition as u16,
                    ParameterValue::F32(-0.25),
                ),
            ],
        );

        let mode = motor
            .controlled_stop(Duration::from_millis(100))
            .expect("should succeed with default gains");
        responder.join().expect("responder");

        assert_eq!(mode, ControlMode::Mit);
        let sent = bus.sent.lock().expect("sent frames");

        let operation_frame = sent
            .iter()
            .find(|frame| {
                let (comm_type, _, _) = ext_id_parts(frame.arbitration_id);
                comm_type == CommunicationType::OPERATION_CONTROL
            })
            .expect("MIT hold frame");

        let (expected_extra, expected_data) = encode_mit_command(
            -0.25,
            0.0,
            50.0, // default kp = 500 * 0.10
            0.5,  // default kd = 5 * 0.10
            0.0,
            motor.limits.p_max,
            motor.limits.v_max,
            motor.limits.t_max,
            motor.kp_max,
            motor.kd_max,
        );
        let (_, actual_extra, _) = ext_id_parts(operation_frame.arbitration_id);
        assert_eq!(actual_extra, expected_extra);
        assert_eq!(operation_frame.data, expected_data);
        assert_no_torque_off_frame(&sent);
    }

    #[test]
    fn controlled_stop_in_mit_uses_default_gains_when_cached_gains_too_small() {
        let bus = Arc::new(MockBus::new());
        let motor =
            Arc::new(RobstrideMotor::new(2, 0xFD, "rs-00", bus.clone()).expect("create motor"));

        // 发送一个纯力矩命令（kp=0, kd=0）
        motor
            .send_cmd_mit(0.0, 0.0, 0.0, 0.0, 5.0)
            .expect("send MIT");

        let responder = spawn_param_responder(
            Arc::clone(&motor),
            Arc::clone(&bus),
            vec![
                (ParameterId::Mode as u16, ParameterValue::I8(0)),
                (
                    ParameterId::MechanicalPosition as u16,
                    ParameterValue::F32(0.42),
                ),
            ],
        );

        let mode = motor
            .controlled_stop(Duration::from_millis(100))
            .expect("should succeed with default gains");
        responder.join().expect("responder");

        assert_eq!(mode, ControlMode::Mit);
        let sent = bus.sent.lock().expect("sent frames");

        let operation_frame = sent
            .iter()
            .rev()
            .find(|frame| {
                let (comm_type, _, _) = ext_id_parts(frame.arbitration_id);
                comm_type == CommunicationType::OPERATION_CONTROL
            })
            .expect("MIT hold frame");

        let (expected_extra, expected_data) = encode_mit_command(
            0.42,
            0.0,
            50.0, // default kp (not 0.0)
            0.5,  // default kd (not 0.0)
            0.0,
            motor.limits.p_max,
            motor.limits.v_max,
            motor.limits.t_max,
            motor.kp_max,
            motor.kd_max,
        );
        let (_, actual_extra, _) = ext_id_parts(operation_frame.arbitration_id);
        assert_eq!(actual_extra, expected_extra);
        assert_eq!(operation_frame.data, expected_data);
        assert_no_torque_off_frame(&sent);
    }
}
