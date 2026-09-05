use crate::motor::DamiaoMotor;
use motor_core::bus::{open_can_bus, open_socketcanfd, CanBus};
use motor_core::dm_device::{DmDeviceBus, DmDeviceType};
use motor_core::dm_serial::DmSerialBus;
use motor_core::error::{MotorError, Result};
use motor_core::vendor_controller::VendorController;
use std::sync::Arc;
use std::time::Duration;

pub struct DamiaoController {
    controller: VendorController<DamiaoMotor>,
}

impl DamiaoController {
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

    pub fn new_dm_serial(port: &str, baud: u32) -> Result<Self> {
        let bus: Arc<dyn CanBus> = Arc::new(DmSerialBus::open(port, baud)?);
        Ok(Self::new(bus))
    }

    pub fn new_dm_device(device_type: DmDeviceType, dm_channel: &str) -> Result<Self> {
        let bus: Arc<dyn CanBus> = Arc::new(DmDeviceBus::open(device_type, dm_channel)?);
        Ok(Self::new(bus))
    }

    pub fn add_motor(
        &self,
        motor_id: u16,
        feedback_id: u16,
        model: &str,
    ) -> Result<Arc<DamiaoMotor>> {
        self.controller.add_motor_with(motor_id, |bus| {
            DamiaoMotor::new(motor_id, feedback_id, model, bus)
        })
    }

    pub fn get_motor(&self, motor_id: u16) -> Result<Arc<DamiaoMotor>> {
        self.controller.get_motor(motor_id)
    }

    pub fn poll_feedback_once(&self) -> Result<()> {
        self.controller.poll_feedback_once()
    }

    pub fn enable_all(&self) -> Result<()> {
        // Zero-point gate: motor 7 must be within ±3° before energizing.
        // Reuses get_register_f32 on p_m (0x50); skips if no motor 7.
        const ZERO_CHECK_MOTOR_ID: u16 = 7;
        const ZERO_CHECK_TOLERANCE_DEG: f32 = 3.0;
        match self.get_motor(ZERO_CHECK_MOTOR_ID) {
            Ok(motor) => {
                let pos = motor.get_register_f32(80, Duration::from_millis(240))?;
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
