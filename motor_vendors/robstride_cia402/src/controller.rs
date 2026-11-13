use crate::motor::RobstrideCia402Motor;
use crate::objects::{PROTOCOL_CANOPEN, PROTOCOL_MIT, PROTOCOL_PRIVATE, PROTOCOL_SWITCH_EXT_ID};
use motor_canopen::scan::scan_ids as shared_scan_ids;
use motor_core::bus::{open_can_bus, CanBus, CanFrame};
use motor_core::error::{MotorError, Result};
use motor_core::vendor_controller::VendorController;
use std::sync::Arc;
use std::time::{Duration, Instant};

#[derive(Debug, Clone, Copy)]
pub struct RobstrideCia402ScanHit {
    pub node_id: u16,
    pub statusword: Option<u16>,
    pub mode_display: Option<i8>,
    pub error_code: Option<u16>,
}

pub struct RobstrideCia402Controller {
    controller: VendorController<RobstrideCia402Motor>,
}

impl RobstrideCia402Controller {
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
    ) -> Result<Arc<RobstrideCia402Motor>> {
        self.controller.add_motor_with(motor_id, |bus| {
            RobstrideCia402Motor::new(motor_id, feedback_id, model, bus)
        })
    }

    pub fn get_motor(&self, motor_id: u16) -> Result<Arc<RobstrideCia402Motor>> {
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

    fn send_ext_frame(&self, arbitration_id: u32, payload: &[u8]) -> Result<()> {
        if payload.len() > 8 {
            return Err(MotorError::InvalidArgument(format!(
                "payload too long: {}, expected <=8",
                payload.len()
            )));
        }
        let mut data = [0u8; 8];
        data[..payload.len()].copy_from_slice(payload);
        self.bus().send(CanFrame {
            arbitration_id,
            data,
            dlc: payload.len() as u8,
            is_extended: true,
            is_rx: false,
        })
    }

    pub fn scan_ids(
        &self,
        start_id: u16,
        end_id: u16,
        timeout: Duration,
    ) -> Result<Vec<RobstrideCia402ScanHit>> {
        if start_id == 0 || end_id == 0 || start_id > 127 || end_id > 127 || start_id > end_id {
            return Err(MotorError::InvalidArgument(
                "invalid scan range, expected 1..127 and start<=end".to_string(),
            ));
        }
        let hits = shared_scan_ids(&self.bus(), start_id, end_id, timeout)?;
        Ok(hits
            .into_iter()
            .map(|h| RobstrideCia402ScanHit {
                node_id: h.node_id,
                statusword: h.statusword,
                mode_display: h.mode_display,
                error_code: h.error_code,
            })
            .collect())
    }

    pub fn switch_protocol(
        &self,
        protocol_cmd: u8,
        timeout: Duration,
    ) -> Result<Option<(u16, [u8; 8])>> {
        if !matches!(
            protocol_cmd,
            PROTOCOL_PRIVATE | PROTOCOL_CANOPEN | PROTOCOL_MIT
        ) {
            return Err(MotorError::InvalidArgument(format!(
                "invalid RobStride protocol command {protocol_cmd}, expected 0(private), 1(canopen), or 2(mit)"
            )));
        }
        self.send_ext_frame(
            PROTOCOL_SWITCH_EXT_ID,
            &[0x01, 0x02, 0x03, 0x04, 0x05, 0x06, protocol_cmd, 0x00],
        )?;
        let deadline = Instant::now() + timeout;
        while Instant::now() < deadline {
            if let Some(frame) = self.bus().recv(Duration::from_millis(2))? {
                if !frame.is_extended && frame.dlc >= 8 {
                    return Ok(Some((frame.arbitration_id as u16, frame.data)));
                }
            }
        }
        Ok(None)
    }
}
