use crate::protocol::{self, Reply, CAN_ID};
use crate::registers;
use motor_core::bus::{CanBus, CanFrame};
use motor_core::device::MotorDevice;
use motor_core::error::{MotorError, Result};
use motor_core::feedback::{FeedbackCache, FeedbackSnapshot};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex, MutexGuard};
use std::time::{Duration, Instant};

#[derive(Debug, Clone, Copy)]
pub struct MotorFeedbackState {
    pub can_id: u8,
    pub arbitration_id: u32,
    pub status_code: u8,
    pub pos: f32,
    pub vel: f32,
    pub current_a: f32,
    pub temperature_c: f32,
}

struct Pending {
    address: u8,
    pair: bool,
    reply: Option<Reply>,
}

pub struct FashionstarMotor {
    pub motor_id: u16,
    bus: Arc<dyn CanBus>,
    transactions: Mutex<Option<(i32, Instant)>>,
    pending: Mutex<Option<Pending>>,
    ready: Condvar,
    state: FeedbackCache<MotorFeedbackState>,
    state_valid: AtomicBool,
}

impl FashionstarMotor {
    pub fn new(
        motor_id: u16,
        feedback_id: u16,
        _model: &str,
        bus: Arc<dyn CanBus>,
    ) -> Result<Self> {
        if motor_id > 254 {
            return Err(MotorError::InvalidArgument(
                "FashionStar ServoID must be 0..254".to_string(),
            ));
        }
        if feedback_id != CAN_ID {
            return Err(MotorError::InvalidArgument(
                "FashionStar feedback_id is fixed at 0x321".to_string(),
            ));
        }
        Ok(Self {
            motor_id,
            bus,
            transactions: Mutex::new(None),
            pending: Mutex::new(None),
            ready: Condvar::new(),
            state: FeedbackCache::new(),
            state_valid: AtomicBool::new(false),
        })
    }

    fn lock(&self) -> Result<MutexGuard<'_, Option<(i32, Instant)>>> {
        self.transactions
            .lock()
            .map_err(|_| MotorError::Io("FashionStar transaction lock poisoned".to_string()))
    }

    fn write(&self, address: u8, value: u32) -> Result<()> {
        self.bus.send(protocol::write(
            self.motor_id,
            address,
            value,
            registers::pair(address),
        )?)
    }

    fn read(&self, address: u8, timeout: Duration) -> Result<u32> {
        if timeout.is_zero() {
            return Err(MotorError::Timeout("FashionStar read timeout".to_string()));
        }
        let pair = registers::pair(address);
        *self
            .pending
            .lock()
            .map_err(|_| MotorError::Io("FashionStar reply lock poisoned".to_string()))? =
            Some(Pending {
                address,
                pair,
                reply: None,
            });
        if let Err(error) = self.bus.send(protocol::read(self.motor_id, address, pair)?) {
            self.pending
                .lock()
                .map_err(|_| MotorError::Io("FashionStar reply lock poisoned".to_string()))?
                .take();
            return Err(error);
        }
        let deadline = Instant::now() + timeout;
        let mut pending = self
            .pending
            .lock()
            .map_err(|_| MotorError::Io("FashionStar reply lock poisoned".to_string()))?;
        loop {
            if let Some(reply) = pending.as_ref().and_then(|p| p.reply) {
                pending.take();
                return Ok(u32::from(reply.value)
                    | reply
                        .second
                        .map(|(_, value)| u32::from(value) << 16)
                        .unwrap_or(0));
            }
            let Some(remaining) = deadline.checked_duration_since(Instant::now()) else {
                pending.take();
                return Err(MotorError::Timeout(format!(
                    "FashionStar read timeout: ServoID={}, address=0x{address:02X}",
                    self.motor_id
                )));
            };
            pending = self
                .ready
                .wait_timeout(pending, remaining)
                .map_err(|_| MotorError::Io("FashionStar reply lock poisoned".to_string()))?
                .0;
        }
    }

    fn unlock(&self) -> Result<()> {
        self.write(0xFF, 0xE6)?;
        // Match the official CAN web tool's protected-read timing, including
        // its 50 ms raw-write pause and the extra 100/200 ms settling delays.
        std::thread::sleep(Duration::from_millis(150));
        self.write(0xFF, 9)?;
        std::thread::sleep(Duration::from_millis(250));
        Ok(())
    }

    fn with_register_access<T>(
        &self,
        address: u8,
        operation: impl FnOnce() -> Result<T>,
    ) -> Result<T> {
        let _guard = self.lock()?;
        if !registers::protected(address) {
            return operation();
        }
        let result = self.unlock().and_then(|_| operation());
        let relock = self.write(0xFF, 0);
        result.and_then(|value| relock.map(|_| value))
    }

    pub fn get_register_u32(&self, rid: u8, timeout: Duration) -> Result<u32> {
        if !registers::readable(rid) {
            return Err(MotorError::InvalidArgument(format!(
                "FashionStar address 0x{rid:02X} is not readable"
            )));
        }
        if timeout.is_zero() {
            return Err(MotorError::Timeout("FashionStar read timeout".to_string()));
        }
        self.with_register_access(rid, || self.read(rid, timeout))
    }

    /// Writes configuration without assuming that it is already active.
    /// Device configuration changes take effect after a power cycle.
    pub fn write_register_u32(&self, rid: u8, value: u32) -> Result<()> {
        registers::validate_write(rid, value)?;
        self.with_register_access(rid, || {
            self.write(rid, value)?;
            if matches!(rid, 0x78..=0x7A) {
                self.state_valid.store(false, Ordering::Release);
            }
            Ok(())
        })
    }

    /// POS_VEL maps to speed-based position mode (native mode 2).
    /// Preserve the device's configured power and acceleration settings.
    pub fn send_cmd_pos_vel(&self, target_position: f32, velocity_limit: f32) -> Result<()> {
        let target = protocol::angle_raw(target_position)?;
        let speed = protocol::speed_raw(velocity_limit)?;
        let _guard = self.lock()?;
        self.write(0x52, 2)?;
        self.write(0x53, target)?;
        self.write(0x58, u32::from(speed))?;
        self.write(0x59, 1)
    }

    pub fn ensure_control_mode(&self, mode: u32, _timeout: Duration) -> Result<()> {
        if mode != 2 {
            return Err(MotorError::Unsupported(
                "FashionStar mode must be 2(POS_VEL)".to_string(),
            ));
        }
        let _guard = self.lock()?;
        self.write(0x52, 2)
    }

    /// Enable output by holding the current position (native stop-lock).
    pub fn enable(&self) -> Result<()> {
        self.stop()
    }

    pub fn stop(&self) -> Result<()> {
        let _guard = self.lock()?;
        self.write(0x7B, 0x11)
    }

    pub fn disable(&self) -> Result<()> {
        let _guard = self.lock()?;
        self.write(0x7B, 0x10)
    }

    pub fn set_zero_position(&self) -> Result<()> {
        let _guard = self.lock()?;
        self.write(0x7B, 0x10)?;
        // The official tool releases output 50 ms before setting the origin.
        std::thread::sleep(Duration::from_millis(50));
        self.write(0x7A, 0)?;
        self.state_valid.store(false, Ordering::Release);
        Ok(())
    }

    pub fn request_motor_feedback(&self, timeout: Duration) -> Result<()> {
        let mut previous_position = self.lock()?;
        // A failed query breaks the sequence used to estimate velocity.
        let previous = previous_position.take();
        let start = Instant::now();
        let position = self.read(0x15, timeout)? as i32;
        let position_at = Instant::now();
        let status = self.read(0x14, timeout.saturating_sub(start.elapsed()))?;
        let temperature = self.read(0x13, timeout.saturating_sub(start.elapsed()))?;
        let current = self.read(0x11, timeout.saturating_sub(start.elapsed()))?;
        let mut velocity = f32::NAN;
        if let Some((last_position, last_at)) =
            previous.filter(|_| self.state_valid.load(Ordering::Acquire))
        {
            let elapsed = position_at.duration_since(last_at).as_secs_f64();
            if elapsed > 0.0 {
                velocity = ((f64::from(position) - f64::from(last_position))
                    * (std::f64::consts::PI / 1800.0)
                    / elapsed) as f32;
            }
        }
        self.state.update(MotorFeedbackState {
            can_id: self.motor_id as u8,
            arbitration_id: u32::from(CAN_ID),
            status_code: status as u8,
            pos: position as f32 * (std::f32::consts::PI / 1800.0),
            vel: velocity,
            current_a: current as f32 / 1000.0,
            temperature_c: protocol::temperature_c(temperature as u16),
        })?;
        *previous_position = Some((position, position_at));
        self.state_valid.store(true, Ordering::Release);
        Ok(())
    }

    pub fn latest_state_snapshot(&self) -> Option<FeedbackSnapshot<MotorFeedbackState>> {
        if !self.state_valid.load(Ordering::Acquire) {
            return None;
        }
        self.state.latest().ok().flatten()
    }

    pub fn latest_state(&self) -> Option<MotorFeedbackState> {
        self.latest_state_snapshot().map(|s| s.state)
    }
}

impl MotorDevice for FashionstarMotor {
    fn vendor(&self) -> &'static str {
        "fashionstar"
    }

    fn model(&self) -> &str {
        "fashionstar"
    }

    fn motor_id(&self) -> u16 {
        self.motor_id
    }

    fn feedback_id(&self) -> u16 {
        CAN_ID
    }

    fn enable(&self) -> Result<()> {
        FashionstarMotor::enable(self)
    }

    fn disable(&self) -> Result<()> {
        FashionstarMotor::disable(self)
    }

    fn accepts_frame(&self, frame: &CanFrame) -> bool {
        protocol::accepts_frame(frame, self.motor_id)
    }

    fn process_feedback_frame(&self, frame: CanFrame) -> Result<()> {
        let reply = protocol::decode(&frame, self.motor_id)?;
        let mut pending = self
            .pending
            .lock()
            .map_err(|_| MotorError::Io("FashionStar reply lock poisoned".to_string()))?;
        if let Some(pending) = pending.as_mut() {
            let expected_second = pending.pair.then_some(pending.address + 1);
            if pending.address == reply.address
                && expected_second == reply.second.map(|(address, _)| address)
            {
                pending.reply = Some(reply);
                self.ready.notify_one();
            }
        }
        Ok(())
    }
}
