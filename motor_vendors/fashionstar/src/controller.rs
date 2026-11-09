use crate::motor::FashionstarMotor;
use motor_core::bus::{open_can_bus, CanBus};
use motor_core::controller::CoreController;
use motor_core::error::Result;
use motor_core::vendor_controller::VendorController;
use std::sync::Arc;

pub struct FashionstarController {
    controller: VendorController<FashionstarMotor>,
}

impl FashionstarController {
    pub fn new(bus: Arc<dyn CanBus>) -> Self {
        Self {
            controller: VendorController::new(bus),
        }
    }

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
    ) -> Result<Arc<FashionstarMotor>> {
        self.controller.add_motor_with(motor_id, |bus| {
            FashionstarMotor::new(motor_id, feedback_id, model, bus)
        })
    }

    pub fn get_motor(&self, motor_id: u16) -> Result<Arc<FashionstarMotor>> {
        self.controller.get_motor(motor_id)
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
