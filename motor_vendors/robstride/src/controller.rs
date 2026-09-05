use crate::motor::RobstrideMotor;
use motor_core::bus::{open_can_bus, open_socketcanfd, CanBus};
use motor_core::error::{MotorError, Result};
use motor_core::vendor_controller::VendorController;
use std::sync::Arc;
use std::time::Duration;

pub struct RobstrideController {
    controller: VendorController<RobstrideMotor>,
}

impl RobstrideController {
    pub fn new(bus: Arc<dyn CanBus>) -> Self {
        Self {
            controller: VendorController::new(bus),
        }
    }

    pub fn new_socketcan(channel: &str) -> Result<Self> {
        Ok(Self::new(open_can_bus(channel)?))
    }

    pub fn new_socketcanfd(channel: &str) -> Result<Self> {
        Ok(Self::new(open_socketcanfd(channel)?))
    }

    pub fn add_motor(
        &self,
        motor_id: u16,
        feedback_id: u16,
        model: &str,
    ) -> Result<Arc<RobstrideMotor>> {
        self.controller.add_motor_with(motor_id, |bus| {
            RobstrideMotor::new(motor_id, feedback_id, model, bus)
        })
    }

    pub fn get_motor(&self, motor_id: u16) -> Result<Arc<RobstrideMotor>> {
        self.controller.get_motor(motor_id)
    }

    pub fn poll_feedback_once(&self) -> Result<()> {
        self.controller.poll_feedback_once()
    }

    pub fn enable_all(&self) -> Result<()> {
        // Zero-point gate: motor 7 must be within ±5° before energizing.
        // Reuses get_parameter_f32 on mechPos (0x7019); skips if no motor 7.
        const ZERO_CHECK_MOTOR_ID: u16 = 7;
        const ZERO_CHECK_TOLERANCE_DEG: f32 = 5.0;
        match self.get_motor(ZERO_CHECK_MOTOR_ID) {
            Ok(motor) => {
                let pos = motor.get_parameter_f32(0x7019, Duration::from_millis(240))?;
                let deg = pos.to_degrees();
                if !deg.is_finite() || deg.abs() > ZERO_CHECK_TOLERANCE_DEG {
                    return Err(MotorError::InvalidArgument(format!(
                        "motor id {} not at zero position: {deg:.2}° exceeds ±{:.0}° tolerance, enable refused",
                        ZERO_CHECK_MOTOR_ID,
                        ZERO_CHECK_TOLERANCE_DEG,
                    )));
                }
            }
            Err(MotorError::InvalidArgument(_)) => {}
            Err(e) => return Err(e),
        }
        self.controller.enable_all()
    }

    pub fn disable_all(&self) -> Result<()> {
        self.controller.disable_all()
    }

    pub fn shutdown(&self) -> Result<()> {
        self.controller.shutdown()
    }

    pub fn close_bus(&self) -> Result<()> {
        self.controller.close_bus()
    }
}
