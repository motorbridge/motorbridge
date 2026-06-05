use crate::protocol::{
    decode_register_value, decode_sensor_feedback, encode_clear_error_cmd, encode_disable_cmd,
    encode_enable_cmd, encode_feedback_request_cmd, encode_force_pos_cmd, encode_mit_cmd,
    encode_pos_vel_cmd, encode_register_read_cmd, encode_register_write_cmd, encode_set_zero_cmd,
    encode_store_params_cmd, encode_vel_cmd, is_register_reply, status_name, Limits,
};
use crate::registers::{register_info, RegisterAccess, RegisterDataType};
use motor_core::bus::{CanBus, CanFrame};
use motor_core::device::MotorDevice;
use motor_core::error::{MotorError, Result};
use motor_core::model::{ModelCatalog, MotorModelSpec, PvTLimits, StaticModelCatalog};
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

const REGISTER_POLL_INTERVAL_MS: u64 = 2;
const SET_ZERO_SETTLE_MS: u64 = 20;
const ENSURE_MODE_VERIFY_ATTEMPTS: usize = 3;
const ENSURE_MODE_VERIFY_RETRY_GAP_MS: u64 = 10;
const WRITE_MODE_REGISTER_WAIT_MS: u64 = 20;

const DAMIAO_MODELS: &[MotorModelSpec] = &[
    MotorModelSpec {
        vendor: "damiao",
        model: "3507",
        pmax: 12.566,
        vmax: 50.0,
        tmax: 5.0,
    },
    MotorModelSpec {
        vendor: "damiao",
        model: "4310",
        pmax: 12.5,
        vmax: 30.0,
        tmax: 10.0,
    },
    MotorModelSpec {
        vendor: "damiao",
        model: "4310P",
        pmax: 12.5,
        vmax: 50.0,
        tmax: 10.0,
    },
    MotorModelSpec {
        vendor: "damiao",
        model: "4340",
        pmax: 12.5,
        vmax: 10.0,
        tmax: 28.0,
    },
    MotorModelSpec {
        vendor: "damiao",
        model: "4340P",
        pmax: 12.5,
        vmax: 10.0,
        tmax: 28.0,
    },
    MotorModelSpec {
        vendor: "damiao",
        model: "6006",
        pmax: 12.5,
        vmax: 45.0,
        tmax: 20.0,
    },
    MotorModelSpec {
        vendor: "damiao",
        model: "8006",
        pmax: 12.5,
        vmax: 45.0,
        tmax: 40.0,
    },
    MotorModelSpec {
        vendor: "damiao",
        model: "8009",
        pmax: 12.5,
        vmax: 45.0,
        tmax: 54.0,
    },
    MotorModelSpec {
        vendor: "damiao",
        model: "10010L",
        pmax: 12.5,
        vmax: 25.0,
        tmax: 200.0,
    },
    MotorModelSpec {
        vendor: "damiao",
        model: "10010",
        pmax: 12.5,
        vmax: 20.0,
        tmax: 200.0,
    },
    MotorModelSpec {
        vendor: "damiao",
        model: "H3510",
        pmax: 12.5,
        vmax: 280.0,
        tmax: 1.0,
    },
    MotorModelSpec {
        vendor: "damiao",
        model: "G6215",
        pmax: 12.5,
        vmax: 45.0,
        tmax: 10.0,
    },
    MotorModelSpec {
        vendor: "damiao",
        model: "H6220",
        pmax: 12.5,
        vmax: 45.0,
        tmax: 10.0,
    },
    MotorModelSpec {
        vendor: "damiao",
        model: "JH11",
        pmax: 12.5,
        vmax: 10.0,
        tmax: 12.0,
    },
    MotorModelSpec {
        vendor: "damiao",
        model: "6248P",
        pmax: 12.566,
        vmax: 20.0,
        tmax: 120.0,
    },
];

const DAMIAO_CATALOG: StaticModelCatalog = StaticModelCatalog {
    vendor_name: "damiao",
    models: DAMIAO_MODELS,
};

pub fn model_limits(model: &str) -> Option<(f32, f32, f32)> {
    DAMIAO_CATALOG
        .get(model)
        .map(|spec| (spec.pmax, spec.vmax, spec.tmax))
}

pub fn match_models_by_limits(pmax: f32, vmax: f32, tmax: f32, tol: f32) -> Vec<&'static str> {
    DAMIAO_MODELS
        .iter()
        .filter(|spec| {
            (spec.pmax - pmax).abs() <= tol
                && (spec.vmax - vmax).abs() <= tol
                && (spec.tmax - tmax).abs() <= tol
        })
        .map(|spec| spec.model)
        .collect()
}

pub fn suggest_models_by_limits(
    pmax: f32,
    vmax: f32,
    tmax: f32,
    top_n: usize,
) -> Vec<&'static str> {
    let mut scored: Vec<(&'static str, f32)> = DAMIAO_MODELS
        .iter()
        .map(|spec| {
            let d = (spec.pmax - pmax).powi(2)
                + (spec.vmax - vmax).powi(2)
                + (spec.tmax - tmax).powi(2);
            (spec.model, d.sqrt())
        })
        .collect();
    scored.sort_by(|a, b| a.1.total_cmp(&b.1));
    scored
        .into_iter()
        .take(top_n)
        .map(|(name, _)| name)
        .collect()
}

#[derive(Debug, Clone, Copy)]
pub enum ControlMode {
    Mit = 1,
    PosVel = 2,
    Vel = 3,
    ForcePos = 4,
}

#[derive(Debug, Clone, Copy)]
pub enum RegisterValue {
    Float(f32),
    UInt32(u32),
}

#[derive(Debug, Clone, Copy)]
pub struct MotorFeedbackState {
    pub can_id: u8,
    pub arbitration_id: u32,
    pub status_code: u8,
    pub status_name: &'static str,
    pub pos: f32,
    pub vel: f32,
    pub torq: f32,
    pub t_mos: f32,
    pub t_rotor: f32,
}

pub struct DamiaoMotor {
    pub motor_id: u16,
    pub feedback_id: u16,
    pub model: String,
    bus: Arc<dyn CanBus>,
    limits: PvTLimits,
    state: Mutex<Option<(MotorFeedbackState, Instant)>>,
    // Software-side guard for set-zero sequencing:
    // set_zero_position() is allowed only after disable() was issued.
    disabled_hint: AtomicBool,
    register_cache: Mutex<RegisterCache>,
}

#[derive(Default)]
struct RegisterCache {
    values: HashMap<u8, RegisterValue>,
    reply_time: HashMap<u8, Instant>,
}

impl DamiaoMotor {
    pub fn new(motor_id: u16, feedback_id: u16, model: &str, bus: Arc<dyn CanBus>) -> Result<Self> {
        let spec = DAMIAO_CATALOG
            .get(model)
            .ok_or_else(|| MotorError::InvalidArgument(format!("unknown Damiao model: {model}")))?;

        Ok(Self {
            motor_id,
            feedback_id,
            model: model.to_string(),
            bus,
            limits: PvTLimits::from_spec(spec),
            state: Mutex::new(None),
            disabled_hint: AtomicBool::new(true),
            register_cache: Mutex::new(RegisterCache::default()),
        })
    }

    fn send_raw(&self, arbitration_id: u32, data: [u8; 8]) -> Result<()> {
        self.bus.send(CanFrame {
            arbitration_id,
            data,
            dlc: 8,
            is_extended: false,
            is_rx: false,
        })
    }

    pub fn enable(&self) -> Result<()> {
        self.send_raw(self.motor_id.into(), encode_enable_cmd())?;
        self.disabled_hint.store(false, Ordering::Release);
        Ok(())
    }

    pub fn disable(&self) -> Result<()> {
        self.send_raw(self.motor_id.into(), encode_disable_cmd())?;
        self.disabled_hint.store(true, Ordering::Release);
        Ok(())
    }

    pub fn clear_error(&self) -> Result<()> {
        self.send_raw(self.motor_id.into(), encode_clear_error_cmd())
    }

    pub fn set_zero_position(&self) -> Result<()> {
        if !self.disabled_hint.load(Ordering::Acquire) {
            return Err(MotorError::InvalidArgument(format!(
                "motor 0x{:X} is not disabled; set_zero_position skipped. call disable() first",
                self.motor_id
            )));
        }
        self.send_raw(self.motor_id.into(), encode_set_zero_cmd())?;
        std::thread::sleep(Duration::from_millis(SET_ZERO_SETTLE_MS));
        Ok(())
    }

    pub fn send_cmd_mit(
        &self,
        target_position: f32,
        target_velocity: f32,
        stiffness: f32,
        damping: f32,
        feedforward_torque: f32,
    ) -> Result<()> {
        let data = encode_mit_cmd(
            target_position,
            target_velocity,
            feedforward_torque,
            stiffness,
            damping,
            Limits {
                p_min: self.limits.p_min,
                p_max: self.limits.p_max,
                v_min: self.limits.v_min,
                v_max: self.limits.v_max,
                t_min: self.limits.t_min,
                t_max: self.limits.t_max,
            },
        );
        self.send_raw(self.motor_id.into(), data)
    }

    pub fn send_cmd_pos_vel(&self, target_position: f32, velocity_limit: f32) -> Result<()> {
        self.send_raw(
            u32::from(0x100u16 + self.motor_id),
            encode_pos_vel_cmd(target_position, velocity_limit),
        )
    }

    pub fn send_cmd_vel(&self, target_velocity: f32) -> Result<()> {
        self.send_raw(
            u32::from(0x200u16 + self.motor_id),
            encode_vel_cmd(target_velocity),
        )
    }

    pub fn send_cmd_force_pos(
        &self,
        target_position: f32,
        velocity_limit: f32,
        torque_limit_ratio: f32,
    ) -> Result<()> {
        self.send_raw(
            u32::from(0x300u16 + self.motor_id),
            encode_force_pos_cmd(target_position, velocity_limit, torque_limit_ratio),
        )
    }

    pub fn ensure_control_mode(&self, mode: ControlMode, timeout: Duration) -> Result<()> {
        let desired = mode as u32;
        match self.get_register_u32(10, timeout) {
            Ok(current) if current == desired => return Ok(()),
            Ok(_) => {}
            Err(MotorError::Timeout(_)) => {}
            Err(e) => return Err(e),
        }

        self.write_register_u32(10, desired)?;

        std::thread::sleep(std::time::Duration::from_millis(
            WRITE_MODE_REGISTER_WAIT_MS,
        ));

        let mut last_error = None;
        for attempt in 0..ENSURE_MODE_VERIFY_ATTEMPTS {
            match self.get_register_u32(10, timeout) {
                Ok(verify) if verify == desired => return Ok(()),
                Ok(verify) => {
                    last_error = Some(MotorError::Protocol(format!(
                        "control mode verify failed: expected {desired}, got {verify}"
                    )));
                }
                Err(MotorError::Timeout(e)) => {
                    last_error = Some(MotorError::Timeout(e));
                }
                Err(e) => return Err(e),
            }
            if attempt + 1 < ENSURE_MODE_VERIFY_ATTEMPTS {
                std::thread::sleep(Duration::from_millis(ENSURE_MODE_VERIFY_RETRY_GAP_MS));
            }
        }

        Err(last_error.unwrap_or_else(|| {
            MotorError::Protocol(format!("control mode verify failed: expected {desired}"))
        }))
    }

    pub fn request_register_reading(&self, rid: u8) -> Result<()> {
        if register_info(rid).is_none() {
            return Err(MotorError::InvalidArgument(format!(
                "unknown register rid {rid}"
            )));
        }
        self.send_raw(0x7FF, encode_register_read_cmd(self.motor_id, rid))
    }

    pub fn write_register_f32(&self, rid: u8, value: f32) -> Result<()> {
        let info = register_info(rid)
            .ok_or_else(|| MotorError::InvalidArgument(format!("unknown register rid {rid}")))?;
        if info.access != RegisterAccess::ReadWrite {
            return Err(MotorError::InvalidArgument(format!(
                "register {rid} is read-only"
            )));
        }
        if info.data_type != RegisterDataType::Float {
            return Err(MotorError::InvalidArgument(format!(
                "register {rid} expects float"
            )));
        }
        self.send_raw(
            0x7FF,
            encode_register_write_cmd(self.motor_id, rid, value.to_le_bytes()),
        )
    }

    pub fn write_register_u32(&self, rid: u8, value: u32) -> Result<()> {
        let info = register_info(rid)
            .ok_or_else(|| MotorError::InvalidArgument(format!("unknown register rid {rid}")))?;
        if info.access != RegisterAccess::ReadWrite {
            return Err(MotorError::InvalidArgument(format!(
                "register {rid} is read-only"
            )));
        }
        if info.data_type != RegisterDataType::UInt32 {
            return Err(MotorError::InvalidArgument(format!(
                "register {rid} expects uint32"
            )));
        }
        self.send_raw(
            0x7FF,
            encode_register_write_cmd(self.motor_id, rid, value.to_le_bytes()),
        )
    }

    pub fn store_parameters(&self) -> Result<()> {
        self.send_raw(0x7FF, encode_store_params_cmd(self.motor_id))
    }

    pub fn request_motor_feedback(&self) -> Result<()> {
        self.send_raw(0x7FF, encode_feedback_request_cmd(self.motor_id))
    }

    pub fn get_register_u32(&self, rid: u8, timeout: Duration) -> Result<u32> {
        let request_at = Instant::now();
        self.request_register_reading(rid)?;
        let deadline = Instant::now() + timeout;
        loop {
            let has_fresh_reply = self
                .register_cache
                .lock()
                .map_err(|_| MotorError::Io("register cache lock poisoned".to_string()))?
                .reply_time
                .get(&rid)
                .copied()
                .map(|ts| ts >= request_at)
                .unwrap_or(false);
            if has_fresh_reply {
                if let Some(value) = self
                    .register_cache
                    .lock()
                    .map_err(|_| MotorError::Io("register cache lock poisoned".to_string()))?
                    .values
                    .get(&rid)
                    .copied()
                {
                    return match value {
                        RegisterValue::UInt32(v) => Ok(v),
                        RegisterValue::Float(_) => Err(MotorError::Protocol(format!(
                            "register {rid} holds float, not u32"
                        ))),
                    };
                }
            }
            if Instant::now() >= deadline {
                return Err(MotorError::Timeout(format!(
                    "register {rid} not received within {:?}",
                    timeout
                )));
            }
            std::thread::sleep(Duration::from_millis(REGISTER_POLL_INTERVAL_MS));
        }
    }

    pub fn get_register_f32(&self, rid: u8, timeout: Duration) -> Result<f32> {
        let request_at = Instant::now();
        self.request_register_reading(rid)?;
        let deadline = Instant::now() + timeout;
        loop {
            let has_fresh_reply = self
                .register_cache
                .lock()
                .map_err(|_| MotorError::Io("register cache lock poisoned".to_string()))?
                .reply_time
                .get(&rid)
                .copied()
                .map(|ts| ts >= request_at)
                .unwrap_or(false);
            if has_fresh_reply {
                if let Some(value) = self
                    .register_cache
                    .lock()
                    .map_err(|_| MotorError::Io("register cache lock poisoned".to_string()))?
                    .values
                    .get(&rid)
                    .copied()
                {
                    return match value {
                        RegisterValue::Float(v) => Ok(v),
                        RegisterValue::UInt32(_) => Err(MotorError::Protocol(format!(
                            "register {rid} holds u32, not float"
                        ))),
                    };
                }
            }
            if Instant::now() >= deadline {
                return Err(MotorError::Timeout(format!(
                    "register {rid} not received within {:?}",
                    timeout
                )));
            }
            std::thread::sleep(Duration::from_millis(REGISTER_POLL_INTERVAL_MS));
        }
    }

    pub fn latest_state(&self) -> Option<MotorFeedbackState> {
        self.state
            .lock()
            .ok()
            .and_then(|s| s.map(|(state, _)| state))
    }

    pub fn request_fresh_state(&self, timeout: Duration) -> Result<Option<MotorFeedbackState>> {
        let request_at = Instant::now();
        self.request_motor_feedback()?;
        let deadline = request_at + timeout;
        loop {
            if let Some((state, ts)) = self
                .state
                .lock()
                .map_err(|_| MotorError::Io("state lock poisoned".to_string()))?
                .as_ref()
                .copied()
            {
                if ts >= request_at {
                    return Ok(Some(state));
                }
            }
            if Instant::now() >= deadline {
                return Ok(None);
            }
            std::thread::sleep(Duration::from_millis(REGISTER_POLL_INTERVAL_MS));
        }
    }

    fn process_feedback_frame_impl(&self, frame: CanFrame) -> Result<()> {
        if is_register_reply(&frame.data) {
            let (rid, raw) = decode_register_value(frame.data)?;
            let info = register_info(rid)
                .ok_or_else(|| MotorError::Protocol(format!("unknown register in reply: {rid}")))?;
            let value = match info.data_type {
                RegisterDataType::Float => RegisterValue::Float(f32::from_le_bytes(raw)),
                RegisterDataType::UInt32 => RegisterValue::UInt32(u32::from_le_bytes(raw)),
            };
            let mut cache = self
                .register_cache
                .lock()
                .map_err(|_| MotorError::Io("register cache lock poisoned".to_string()))?;
            cache.values.insert(rid, value);
            cache.reply_time.insert(rid, Instant::now());
            return Ok(());
        }

        let decoded = decode_sensor_feedback(
            frame.data,
            Limits {
                p_min: self.limits.p_min,
                p_max: self.limits.p_max,
                v_min: self.limits.v_min,
                v_max: self.limits.v_max,
                t_min: self.limits.t_min,
                t_max: self.limits.t_max,
            },
        );
        let state = MotorFeedbackState {
            can_id: decoded.can_id,
            arbitration_id: frame.arbitration_id,
            status_code: decoded.status_code,
            status_name: status_name(decoded.status_code),
            pos: decoded.pos,
            vel: decoded.vel,
            torq: decoded.torq,
            t_mos: decoded.t_mos,
            t_rotor: decoded.t_rotor,
        };
        self.state
            .lock()
            .map_err(|_| MotorError::Io("state lock poisoned".to_string()))?
            .replace((state, Instant::now()));
        Ok(())
    }
}

impl MotorDevice for DamiaoMotor {
    fn vendor(&self) -> &'static str {
        "damiao"
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
        DamiaoMotor::enable(self)
    }

    fn disable(&self) -> Result<()> {
        DamiaoMotor::disable(self)
    }

    fn accepts_frame(&self, frame: &CanFrame) -> bool {
        if frame.is_extended {
            return false;
        }
        frame.arbitration_id == u32::from(self.feedback_id)
            || (frame.dlc > 0 && (frame.data[0] & 0x0F) == (self.motor_id as u8 & 0x0F))
    }

    fn process_feedback_frame(&self, frame: CanFrame) -> Result<()> {
        self.process_feedback_frame_impl(frame)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use motor_core::test_support::MockBus;
    use std::time::{Duration, Instant};

    #[test]
    fn model_limits_and_matching_work() {
        let (pmax, vmax, tmax) = model_limits("4340P").expect("known model");
        assert_eq!(pmax, 12.5);
        assert_eq!(vmax, 10.0);
        assert_eq!(tmax, 28.0);

        let matched = match_models_by_limits(12.5, 10.0, 28.0, 0.01);
        assert!(matched.contains(&"4340"));
        assert!(matched.contains(&"4340P"));
    }

    #[test]
    fn suggest_models_returns_closest_first() {
        let suggested = suggest_models_by_limits(12.5, 9.9, 28.1, 3);
        assert!(!suggested.is_empty());
        assert!(suggested[0] == "4340" || suggested[0] == "4340P");
    }

    #[test]
    fn get_register_u32_times_out_when_no_feedback_arrives() {
        let bus: Arc<dyn CanBus> = Arc::new(MockBus::new());
        let motor = DamiaoMotor::new(0x01, 0x11, "4340P", bus).expect("create motor");
        let err = motor
            .get_register_u32(10, Duration::from_millis(1))
            .expect_err("timeout expected");
        assert!(matches!(err, MotorError::Timeout(_)));
    }

    #[test]
    fn get_register_u32_ignores_stale_cached_reply() {
        let bus: Arc<dyn CanBus> = Arc::new(MockBus::new());
        let motor = DamiaoMotor::new(0x01, 0x11, "4340P", bus).expect("create motor");

        let mut cache = motor.register_cache.lock().expect("register cache lock");
        cache.values.insert(10, RegisterValue::UInt32(2));
        cache.reply_time.insert(10, Instant::now());
        drop(cache);

        let err = motor
            .get_register_u32(10, Duration::from_millis(1))
            .expect_err("stale cache must not satisfy new request");
        assert!(matches!(err, MotorError::Timeout(_)));
    }

    #[test]
    fn ensure_control_mode_writes_when_initial_read_times_out() {
        let bus_impl = Arc::new(MockBus::new());
        let bus: Arc<dyn CanBus> = bus_impl.clone();
        let motor = Arc::new(DamiaoMotor::new(0x01, 0x11, "4340P", bus).expect("create motor"));
        let responder = Arc::clone(&motor);
        let bus_for_thread = Arc::clone(&bus_impl);

        let handle = std::thread::spawn(move || {
            let deadline = Instant::now() + Duration::from_millis(100);
            loop {
                let should_reply = {
                    let sent = bus_for_thread.sent.lock().expect("sent lock");
                    let saw_mode_write = sent
                        .iter()
                        .any(|f| f.data == encode_register_write_cmd(0x01, 10, 2u32.to_le_bytes()));
                    let mode_read_count = sent
                        .iter()
                        .filter(|f| f.data == encode_register_read_cmd(0x01, 10))
                        .count();
                    saw_mode_write && mode_read_count >= 2
                };
                if should_reply {
                    responder
                        .process_feedback_frame_impl(CanFrame {
                            arbitration_id: 0x11,
                            data: [0x01, 0x01, 0x33, 10, 2, 0, 0, 0],
                            dlc: 8,
                            is_extended: false,
                            is_rx: true,
                        })
                        .expect("process register reply");
                    return;
                }
                if Instant::now() >= deadline {
                    return;
                }
                std::thread::sleep(Duration::from_millis(1));
            }
        });

        motor
            .ensure_control_mode(ControlMode::PosVel, Duration::from_millis(5))
            .expect("ensure should recover after initial read timeout");
        handle.join().expect("responder thread");

        let sent = bus_impl.sent.lock().expect("sent lock");
        assert!(sent
            .iter()
            .any(|f| f.data == encode_register_write_cmd(0x01, 10, 2u32.to_le_bytes())));
    }

    #[test]
    fn set_zero_requires_disable_first() {
        let bus: Arc<dyn CanBus> = Arc::new(MockBus::new());
        let motor = DamiaoMotor::new(0x01, 0x11, "4340P", bus).expect("create motor");

        motor.enable().expect("enable");
        let err = motor
            .set_zero_position()
            .expect_err("set_zero must fail when motor is not disabled");
        assert!(matches!(err, MotorError::InvalidArgument(_)));
    }

    #[test]
    fn set_zero_sends_command_after_disable() {
        let bus_impl = Arc::new(MockBus::new());
        let bus: Arc<dyn CanBus> = bus_impl.clone();
        let motor = DamiaoMotor::new(0x04, 0x14, "4310", bus).expect("create motor");

        motor.disable().expect("disable");
        motor.set_zero_position().expect("set_zero");

        let sent = bus_impl.sent.lock().expect("sent lock");
        let has_set_zero = sent.iter().any(|f| {
            f.arbitration_id == 0x04 && f.data == [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE]
        });
        assert!(has_set_zero, "set_zero command frame should be sent");
    }

    #[test]
    fn register_type_errors_name_expected_type() {
        let bus: Arc<dyn CanBus> = Arc::new(MockBus::new());
        let motor = DamiaoMotor::new(0x01, 0x11, "4340P", bus).expect("create motor");

        let f32_err = motor
            .write_register_f32(10, 1.0)
            .expect_err("u32 register should reject f32 write");
        assert!(
            f32_err.to_string().contains("expects float"),
            "unexpected error: {f32_err}"
        );

        let u32_err = motor
            .write_register_u32(22, 1)
            .expect_err("float register should reject u32 write");
        assert!(
            u32_err.to_string().contains("expects uint32"),
            "unexpected error: {u32_err}"
        );
    }
}
