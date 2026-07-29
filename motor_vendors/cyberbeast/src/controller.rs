use crate::motor::CyberBeastMotor;
use motor_core::bus::{open_can_bus, CanBus};
use motor_core::error::Result;
use motor_core::vendor_controller::VendorController;
use std::sync::Arc;

pub struct CyberBeastController {
    controller: VendorController<CyberBeastMotor>,
}

impl CyberBeastController {
    pub fn new(bus: Arc<dyn CanBus>) -> Self {
        Self {
            controller: VendorController::new(bus),
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
    ) -> Result<Arc<CyberBeastMotor>> {
        self.controller.add_motor_with(motor_id, |bus| {
            CyberBeastMotor::new(motor_id, feedback_id, model, bus)
        })
    }

    pub fn add_motor_with_master(
        &self,
        motor_id: u16,
        feedback_id: u16,
        model: &str,
        master_id: u8,
    ) -> Result<Arc<CyberBeastMotor>> {
        self.controller.add_motor_with(motor_id, |bus| {
            Ok(CyberBeastMotor::new(motor_id, feedback_id, model, bus)?.with_master_id(master_id))
        })
    }

    pub fn get_motor(&self, motor_id: u16) -> Result<Arc<CyberBeastMotor>> {
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
