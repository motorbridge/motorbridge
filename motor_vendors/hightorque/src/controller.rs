use crate::motor::HightorqueMotor;
use crate::protocol::MotorModel;
use motor_core::bus::{open_can_bus, CanBus};
use motor_core::controller::CoreController;
use motor_core::error::{MotorError, Result};
use motor_core::vendor_controller::VendorController;
use std::sync::Arc;

pub struct HightorqueController {
    controller: VendorController<HightorqueMotor>,
}

impl HightorqueController {
    pub fn new(bus: Arc<dyn CanBus>) -> Self {
        Self {
            controller: VendorController::new(bus),
        }
    }

    /// Wrap an existing shared `CoreController` (mixed-vendor bus): this
    /// controller's motors join the shared core's device table, sharing one
    /// bus fd and one background receive thread with other vendors.
    pub fn new_shared(core: Arc<CoreController>) -> Self {
        Self {
            controller: VendorController::new_shared(core),
        }
    }

    pub fn new_socketcan(channel: &str) -> Result<Self> {
        Ok(Self::new(open_can_bus(channel)?))
    }

    pub fn add_motor(
        &self,
        motor_id: u16,
        feedback_id: u16,
        model: &str,
    ) -> Result<Arc<HightorqueMotor>> {
        // 型号提示经 MotorModel::from_hint 校验:接受通用占位(ht/hightorque/auto/
        // default)与具体型号码(5046-20 等),其余拒绝。具体型号决定力矩补偿系数(P2-15)。
        let _model_enum = MotorModel::from_hint(model).ok_or_else(|| {
            MotorError::InvalidArgument(format!("unsupported HighTorque model hint: {model}"))
        })?;
        self.controller.add_motor_with(motor_id, |bus| {
            Ok(HightorqueMotor::new(motor_id, feedback_id, model, bus))
        })
    }

    pub fn poll_feedback_once(&self) -> Result<()> {
        self.controller.poll_feedback_once()
    }

    pub fn enable_all(&self) -> Result<()> {
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
