//! Direct SDO probes and CiA402 node scanning.
//!
//! These bypass the device/poll dispatch entirely: they send an SDO request
//! and read the reply straight off the bus. That only works when **no**
//! devices are registered with the shared core yet (otherwise the core's
//! receive path claims the reply first) — exactly the scan-before-configure
//! use case, mirroring the original RobStride/Hexfellow controllers.

use std::sync::Arc;
use std::time::{Duration, Instant};

use motor_core::bus::CanBus;
use motor_core::error::{MotorError, Result};

use crate::objects::{
    sdo_req_id, sdo_rsp_id, ERROR_CODE, MODES_OF_OPERATION_DISPLAY, STATUSWORD,
};
use crate::sdo::{build_sdo_upload, classify_sdo_reply, SdoReplyKind};

/// Send an expedited SDO upload and read the reply directly from the bus
/// (pre-registration probe). Returns the raw 4 data bytes as `u32`
/// (little-endian); callers reinterpret per the object's type.
pub fn sdo_read_direct(
    bus: &Arc<dyn CanBus>,
    node_id: u16,
    index: u16,
    subindex: u8,
    timeout: Duration,
) -> Result<u32> {
    let req_id = sdo_req_id(node_id);
    let rsp_id = sdo_rsp_id(node_id);
    let req = build_sdo_upload(index, subindex);
    crate::send_std_frame(bus.as_ref(), req_id, &req)?;
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        if let Some(frame) = bus.recv(Duration::from_millis(2))? {
            if frame.is_extended || frame.arbitration_id != rsp_id || frame.dlc < 8 {
                continue;
            }
            if frame.data[1] != (index & 0xFF) as u8
                || frame.data[2] != ((index >> 8) & 0xFF) as u8
                || frame.data[3] != subindex
            {
                continue;
            }
            match classify_sdo_reply(&frame.data) {
                SdoReplyKind::Abort(abort) => {
                    return Err(MotorError::Protocol(format!(
                        "sdo abort node={node_id} idx=0x{index:04X} sub=0x{subindex:02X} code=0x{abort:08X}"
                    )));
                }
                SdoReplyKind::UploadData(_) => {
                    return Ok(u32::from_le_bytes([
                        frame.data[4],
                        frame.data[5],
                        frame.data[6],
                        frame.data[7],
                    ]));
                }
                _ => continue,
            }
        }
    }
    Err(MotorError::Timeout(format!(
        "scan timeout node={node_id} idx=0x{index:04X} sub=0x{subindex:02X}"
    )))
}

/// A CiA402 node discovered by [`scan_ids`].
#[derive(Debug, Clone, Copy)]
pub struct ScanHit {
    pub node_id: u16,
    pub statusword: Option<u16>,
    pub mode_display: Option<i8>,
    pub error_code: Option<u16>,
}

/// Scan `start_id..=end_id` for CANopen nodes by probing the CiA402 profile
/// objects `6041h` (statusword), `6061h` (mode display) and `603Fh` (error
/// code). A node is a hit if any probe answered. Sends an NMT start
/// broadcast first so nodes in pre-operational state still answer via SDO.
///
/// Vendor controllers needing vendor-specific probes (e.g. `1018h` identity
/// or `2xxx` manufacturer objects) call [`sdo_read_direct`] directly with
/// their own indices.
pub fn scan_ids(
    bus: &Arc<dyn CanBus>,
    start_id: u16,
    end_id: u16,
    timeout: Duration,
) -> Result<Vec<ScanHit>> {
    if start_id == 0 || end_id == 0 || start_id > 127 || end_id > 127 || start_id > end_id {
        return Err(MotorError::InvalidArgument(
            "invalid scan range, expected 1..127 and start<=end".to_string(),
        ));
    }
    crate::nmt::send_nmt_start_all(bus)?;
    let mut hits = Vec::new();
    for node in start_id..=end_id {
        let statusword = sdo_read_direct(bus, node, STATUSWORD, 0x00, timeout)
            .ok()
            .map(|v| v as u16);
        let mode_display = sdo_read_direct(bus, node, MODES_OF_OPERATION_DISPLAY, 0x00, timeout)
            .ok()
            .map(|v| v as u8 as i8);
        let error_code = sdo_read_direct(bus, node, ERROR_CODE, 0x00, timeout)
            .ok()
            .map(|v| v as u16);
        if statusword.is_some() || mode_display.is_some() || error_code.is_some() {
            hits.push(ScanHit {
                node_id: node,
                statusword,
                mode_display,
                error_code,
            });
        }
    }
    Ok(hits)
}

#[cfg(test)]
mod tests {
    use super::*;
    use motor_core::bus::CanFrame;
    use motor_core::test_support::MockBus;

    fn sdo_rsp(node: u16, data: [u8; 8]) -> CanFrame {
        CanFrame {
            arbitration_id: sdo_rsp_id(node),
            data,
            dlc: 8,
            is_extended: false,
            is_rx: true,
        }
    }

    #[test]
    fn rejects_invalid_range() {
        let bus: Arc<dyn CanBus> = Arc::new(MockBus::new());
        let err = scan_ids(&bus, 0, 5, Duration::from_millis(1)).expect_err("range");
        assert!(err.to_string().contains("invalid scan range"));
        let err = scan_ids(&bus, 5, 1, Duration::from_millis(1)).expect_err("range");
        assert!(err.to_string().contains("invalid scan range"));
        let err = scan_ids(&bus, 1, 200, Duration::from_millis(1)).expect_err("range");
        assert!(err.to_string().contains("invalid scan range"));
    }

    #[test]
    fn sdo_read_direct_matches_reply_and_decodes() {
        let mock = MockBus::new();
        // reply to 6041h read for node 1: statusword 0x0237
        mock.push_rx(sdo_rsp(1, [0x43, 0x41, 0x60, 0x00, 0x37, 0x02, 0, 0]));
        let bus: Arc<MockBus> = Arc::new(mock);
        let v = sdo_read_direct(&(bus.clone() as Arc<dyn CanBus>), 1, 0x6041, 0x00, Duration::from_millis(50))
            .expect("read");
        assert_eq!(v, 0x0237);
        // request frame was sent to 0x601
        let sent = bus.sent.lock().expect("sent").clone();
        assert_eq!(sent.len(), 1);
        assert_eq!(sent[0].arbitration_id, 0x601);
        assert_eq!(sent[0].data[0], 0x40);
    }

    #[test]
    fn sdo_read_direct_reports_abort() {
        let mock = MockBus::new();
        mock.push_rx(sdo_rsp(1, [0x80, 0x00, 0x10, 0x00, 0x06, 0x02, 0x00, 0x00]));
        let bus: Arc<dyn CanBus> = Arc::new(mock);
        let err =
            sdo_read_direct(&bus, 1, 0x1000, 0x00, Duration::from_millis(50)).expect_err("abort");
        assert!(err.to_string().contains("sdo abort"));
    }

    #[test]
    fn sdo_read_direct_skips_unrelated_frames_and_times_out() {
        let mock = MockBus::new();
        // unrelated node / index replies must be skipped
        mock.push_rx(sdo_rsp(2, [0x43, 0x41, 0x60, 0x00, 0xFF, 0xFF, 0, 0]));
        mock.push_rx(sdo_rsp(1, [0x43, 0x00, 0x20, 0x00, 1, 2, 3, 4]));
        let bus: Arc<dyn CanBus> = Arc::new(mock);
        let err =
            sdo_read_direct(&bus, 1, 0x6041, 0x00, Duration::from_millis(20)).expect_err("timeout");
        assert!(err.to_string().contains("timeout"));
    }

    #[test]
    fn scan_ids_collects_hits_and_ignores_silent_nodes() {
        let mock = MockBus::new();
        // node 1 answers the statusword probe only
        mock.push_rx(sdo_rsp(1, [0x43, 0x41, 0x60, 0x00, 0x31, 0x02, 0, 0]));
        let bus: Arc<MockBus> = Arc::new(mock);
        let hits = scan_ids(&(bus.clone() as Arc<dyn CanBus>), 1, 2, Duration::from_millis(5))
            .expect("scan");
        assert_eq!(hits.len(), 1);
        assert_eq!(hits[0].node_id, 1);
        assert_eq!(hits[0].statusword, Some(0x0231));
        assert_eq!(hits[0].mode_display, None);
        assert_eq!(hits[0].error_code, None);
        // first frame sent is the NMT start broadcast
        let sent = bus.sent.lock().expect("sent").clone();
        assert_eq!(sent[0].arbitration_id, 0x000);
        assert_eq!(sent[0].data[0], 0x01);
    }
}
