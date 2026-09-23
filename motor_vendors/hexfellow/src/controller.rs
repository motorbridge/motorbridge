use crate::motor::HexfellowMotor;
use motor_core::bus::{open_socketcanfd, CanBus, CanFrame};
use motor_core::controller::CoreController;
use motor_core::error::{MotorError, Result};
use motor_core::vendor_controller::VendorController;
use std::sync::Arc;
use std::time::{Duration, Instant};

#[derive(Debug, Clone, Copy)]
pub struct HexfellowScanHit {
    pub node_id: u16,
    pub sw_ver: Option<u32>,
    pub peak_torque_raw: Option<u32>,
    pub kp_kd_factor_raw: Option<u32>,
    pub dev_type: Option<u32>,
}

pub struct HexfellowController {
    controller: VendorController<HexfellowMotor>,
}

impl HexfellowController {
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

    pub fn new_socketcanfd(channel: &str) -> Result<Self> {
        Ok(Self::new(open_socketcanfd(channel)?))
    }

    pub fn add_motor(
        &self,
        motor_id: u16,
        feedback_id: u16,
        model: &str,
    ) -> Result<Arc<HexfellowMotor>> {
        self.controller.add_motor_with(motor_id, |bus| {
            HexfellowMotor::new(motor_id, feedback_id, model, bus)
        })
    }

    pub fn get_motor(&self, motor_id: u16) -> Result<Arc<HexfellowMotor>> {
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

    fn send_std_frame(&self, arbitration_id: u32, payload: &[u8]) -> Result<()> {
        if payload.len() > 8 {
            return Err(MotorError::InvalidArgument(format!(
                "payload too long: {}, expected <=8",
                payload.len()
            )));
        }
        let mut data = [0u8; 8];
        data[..payload.len()].copy_from_slice(payload);
        self.controller.bus().send(CanFrame {
            arbitration_id,
            data,
            dlc: payload.len() as u8,
            is_extended: false,
            is_rx: false,
        })
    }

    fn send_nmt(&self, command: u8, node_id: u8) -> Result<()> {
        self.send_std_frame(0x000, &[command, node_id])
    }

    fn sdo_read_direct(
        &self,
        node_id: u16,
        index: u16,
        subindex: u8,
        timeout: Duration,
    ) -> Result<u32> {
        let req_id = 0x600 + u32::from(node_id);
        let rsp_id = 0x580 + u32::from(node_id);
        let req = [
            0x40,
            (index & 0xFF) as u8,
            ((index >> 8) & 0xFF) as u8,
            subindex,
            0,
            0,
            0,
            0,
        ];
        self.send_std_frame(req_id, &req)?;
        let deadline = Instant::now() + timeout;
        while Instant::now() < deadline {
            if let Some(frame) = self.controller.bus().recv(Duration::from_millis(2))? {
                if frame.arbitration_id != rsp_id || frame.dlc < 8 {
                    continue;
                }
                if frame.data[1] != (index & 0xFF) as u8
                    || frame.data[2] != ((index >> 8) & 0xFF) as u8
                    || frame.data[3] != subindex
                {
                    continue;
                }
                if frame.data[0] == 0x80 {
                    let abort = u32::from_le_bytes([
                        frame.data[4],
                        frame.data[5],
                        frame.data[6],
                        frame.data[7],
                    ]);
                    return Err(MotorError::Protocol(format!(
                        "sdo abort node={node_id} idx=0x{index:04X} sub=0x{subindex:02X} code=0x{abort:08X}"
                    )));
                }
                if matches!(frame.data[0], 0x43 | 0x4B | 0x4F | 0x47) {
                    return Ok(u32::from_le_bytes([
                        frame.data[4],
                        frame.data[5],
                        frame.data[6],
                        frame.data[7],
                    ]));
                }
            }
        }
        Err(MotorError::Timeout(format!(
            "scan timeout node={node_id} idx=0x{index:04X} sub=0x{subindex:02X}"
        )))
    }

    pub fn scan_ids(
        &self,
        start_id: u16,
        end_id: u16,
        timeout: Duration,
    ) -> Result<Vec<HexfellowScanHit>> {
        if start_id == 0 || end_id == 0 || start_id > 127 || end_id > 127 || start_id > end_id {
            return Err(MotorError::InvalidArgument(
                "invalid scan range, expected 1..127 and start<=end".to_string(),
            ));
        }
        let has_devices = self.controller.motor_count()? > 0;
        if has_devices {
            return Err(MotorError::InvalidArgument(
                "scan_ids should run before add_motor (to avoid polling interception)".to_string(),
            ));
        }
        self.send_nmt(0x01, 0)?;
        let mut hits = Vec::new();
        for node in start_id..=end_id {
            let r1018 = self.sdo_read_direct(node, 0x1018, 0x03, timeout).ok();
            let r6076 = self.sdo_read_direct(node, 0x6076, 0x00, timeout).ok();
            let r2003 = self.sdo_read_direct(node, 0x2003, 0x07, timeout).ok();
            let r1000 = self.sdo_read_direct(node, 0x1000, 0x00, timeout).ok();
            if r1018.is_some() || r6076.is_some() || r2003.is_some() || r1000.is_some() {
                hits.push(HexfellowScanHit {
                    node_id: node,
                    sw_ver: r1018,
                    peak_torque_raw: r6076,
                    kp_kd_factor_raw: r2003,
                    dev_type: r1000,
                });
            }
        }
        Ok(hits)
    }
}
