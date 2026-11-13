//! `HuayiController` — a thin wrapper over the shared `VendorController`,
//! mirroring the slimmed-down robstride_cia402 controller.

use crate::motor::HuayiMotor;
use motor_canopen::scan::scan_ids as shared_scan_ids;
use motor_core::bus::{open_can_bus, CanBus};
use motor_core::error::{MotorError, Result};
use motor_core::vendor_controller::VendorController;
use std::sync::Arc;
use std::time::Duration;

#[derive(Debug, Clone, Copy)]
pub struct HuayiScanHit {
    pub node_id: u16,
    pub statusword: Option<u16>,
    pub mode_display: Option<i8>,
    pub error_code: Option<u16>,
}

pub struct HuayiController {
    controller: VendorController<HuayiMotor>,
}

impl HuayiController {
    pub fn new(bus: Arc<dyn CanBus>) -> Self {
        Self {
            controller: VendorController::new(bus),
        }
    }

    /// Wrap an existing shared `CoreController` — the mixed-vendor entry
    /// point (Huayi and RobStride CiA402 nodes on one bus share one
    /// receive thread; the core's first-accept-wins dispatch routes each
    /// frame by COB-ID).
    pub fn new_shared(core: Arc<motor_core::controller::CoreController>) -> Self {
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
    ) -> Result<Arc<HuayiMotor>> {
        self.controller.add_motor_with(motor_id, |bus| {
            HuayiMotor::new(motor_id, feedback_id, model, bus)
        })
    }

    pub fn get_motor(&self, motor_id: u16) -> Result<Arc<HuayiMotor>> {
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

    fn bus(&self) -> Arc<dyn CanBus> {
        self.controller.bus()
    }

    /// Number of motors registered with the shared core.
    pub fn motor_count(&self) -> Result<usize> {
        self.controller.motor_count()
    }

    /// Scan `start_id..=end_id` (1..=127) for CANopen nodes, using the
    /// shared probe procedure (NMT start broadcast + 6041h/6061h/603Fh SDO
    /// reads).
    ///
    /// The probes read replies straight off the bus, which only works while
    /// **no** motors are registered — once a motor is added, the core's
    /// receive path claims the SDO replies first and the probes time out.
    /// Scanning a controller with registered motors returns an error; scan
    /// before `add_motor` (the CLI/gateway open a fresh controller for
    /// scanning) or use a separate controller on the same bus.
    pub fn scan_ids(
        &self,
        start_id: u16,
        end_id: u16,
        timeout: Duration,
    ) -> Result<Vec<HuayiScanHit>> {
        if self.motor_count()? > 0 {
            return Err(MotorError::InvalidArgument(
                "scan_ids probes read directly off the bus and race the core receive thread \
                 once motors are registered; scan before add_motor or use a separate controller"
                    .to_string(),
            ));
        }
        let hits = shared_scan_ids(&self.bus(), start_id, end_id, timeout)?;
        Ok(hits
            .into_iter()
            .map(|h| HuayiScanHit {
                node_id: h.node_id,
                statusword: h.statusword,
                mode_display: h.mode_display,
                error_code: h.error_code,
            })
            .collect())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use motor_core::device::MotorDevice;
    use motor_core::test_support::MockBus;

    #[test]
    fn add_get_motor_and_duplicate_detection() {
        let bus = Arc::new(MockBus::new());
        let ctl = HuayiController::new(bus);
        let m = ctl.add_motor(1, 0, "huayi-joint").expect("add");
        assert_eq!(m.motor_id(), 1);
        assert_eq!(m.feedback_id(), 1);
        assert!(ctl.get_motor(1).is_ok());
        assert!(ctl.get_motor(2).is_err());
        // cross-vendor node-id dedup lives in the shared core
        assert!(ctl.add_motor(1, 0, "huayi-joint").is_err());
    }

    #[test]
    fn scan_ids_rejected_when_motors_registered() {
        let bus = Arc::new(MockBus::new());
        let ctl = HuayiController::new(bus.clone());
        assert_eq!(ctl.motor_count().expect("count"), 0);

        // once a motor is registered the direct SDO probes would race the
        // core receive thread — the guard rejects before any probing traffic
        ctl.add_motor(1, 0, "huayi-joint").expect("add");
        let err = ctl
            .scan_ids(1, 5, Duration::from_millis(10))
            .expect_err("must reject");
        assert!(matches!(err, MotorError::InvalidArgument(_)));
        assert!(err.to_string().contains("race"));
        assert!(bus.sent.lock().expect("sent").is_empty());
    }
}
