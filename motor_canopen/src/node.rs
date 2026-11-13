//! Per-node CANopen protocol handle composed by vendor motor structs.
//!
//! A `CanopenNode` owns:
//!
//! - the node id and the shared bus handle,
//! - the SDO reply queue fed by the vendor's `process_feedback_frame`
//!   (so SDO request/ack round-trips require the core's poll loop to run —
//!   the ACK-blocking caveat that applies to every CANopen-family vendor),
//! - the last heartbeat payload for error-control liveness checks.
//!
//! It deliberately does **not** implement `MotorDevice`: frame claiming
//! policy (which TPDOs belong to this motor, what feedback state is cached)
//! stays in the vendor crate, which calls [`CanopenNode::accepts_std`] /
//! [`CanopenNode::process`] inside its own trait methods.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use motor_core::bus::{CanBus, CanFrame};
use motor_core::error::{MotorError, Result};

use crate::classify::{is_heartbeat, is_sdo_rsp};
use crate::nmt::nmt_payload;
use crate::nmt::NmtCommand;
use crate::objects::{sdo_req_id, NMT_ID, NMT_START_REMOTE_NODE};
use crate::send_std_frame;
use crate::sdo::{build_sdo_download, build_sdo_upload, expect_write_ack, upload_payload, SdoQueue, SdoValue};

/// Result of handing a received frame to [`CanopenNode::process`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NodeOutcome {
    /// Frame does not belong to this node — the vendor should keep looking
    /// (e.g. at its TPDO decoders) or ignore it.
    NotClaimed,
    /// SDO reply queued; a blocked `sdo_read_*`/`sdo_write_*` will consume it.
    SdoQueued,
    /// Heartbeat consumed; carries the new state byte.
    Heartbeat(u8),
}

pub struct CanopenNode {
    node_id: u16,
    bus: Arc<dyn CanBus>,
    sdo: SdoQueue,
    heartbeat_state: Mutex<Option<u8>>,
}

impl CanopenNode {
    /// Create the node handle. `node_id` must be `1..=127` (the predefined
    /// connection set limit).
    pub fn new(node_id: u16, bus: Arc<dyn CanBus>) -> Result<Self> {
        if node_id == 0 || node_id > 127 {
            return Err(MotorError::InvalidArgument(format!(
                "invalid CANopen node id {node_id}, expected 1..127"
            )));
        }
        Ok(Self {
            node_id,
            bus,
            sdo: SdoQueue::new(),
            heartbeat_state: Mutex::new(None),
        })
    }

    pub fn node_id(&self) -> u16 {
        self.node_id
    }

    pub fn bus(&self) -> &Arc<dyn CanBus> {
        &self.bus
    }

    // --- raw SDO round trips ------------------------------------------------

    fn request_id(&self) -> u32 {
        sdo_req_id(self.node_id)
    }

    /// Expedited SDO upload (read); returns the 4 raw payload bytes.
    pub fn sdo_read_raw(&self, index: u16, subindex: u8, timeout: Duration) -> Result<[u8; 4]> {
        let req = build_sdo_upload(index, subindex);
        send_std_frame(self.bus.as_ref(), self.request_id(), &req)?;
        let reply = self.sdo.pop_matching(index, subindex, timeout)?;
        upload_payload(&reply)
    }

    /// Expedited SDO download (write) of a typed value; blocks for the ack.
    pub fn sdo_write(
        &self,
        index: u16,
        subindex: u8,
        value: SdoValue,
        timeout: Duration,
    ) -> Result<()> {
        let req = build_sdo_download(index, subindex, value);
        send_std_frame(self.bus.as_ref(), self.request_id(), &req)?;
        let reply = self.sdo.pop_matching(index, subindex, timeout)?;
        expect_write_ack(&reply)
    }

    // --- typed SDO reads ----------------------------------------------------

    pub fn sdo_read_u8(&self, index: u16, subindex: u8, timeout: Duration) -> Result<u8> {
        Ok(self.sdo_read_raw(index, subindex, timeout)?[0])
    }

    pub fn sdo_read_i8(&self, index: u16, subindex: u8, timeout: Duration) -> Result<i8> {
        Ok(self.sdo_read_raw(index, subindex, timeout)?[0] as i8)
    }

    pub fn sdo_read_u16(&self, index: u16, subindex: u8, timeout: Duration) -> Result<u16> {
        let raw = self.sdo_read_raw(index, subindex, timeout)?;
        Ok(u16::from_le_bytes([raw[0], raw[1]]))
    }

    pub fn sdo_read_i16(&self, index: u16, subindex: u8, timeout: Duration) -> Result<i16> {
        let raw = self.sdo_read_raw(index, subindex, timeout)?;
        Ok(i16::from_le_bytes([raw[0], raw[1]]))
    }

    pub fn sdo_read_u32(&self, index: u16, subindex: u8, timeout: Duration) -> Result<u32> {
        Ok(u32::from_le_bytes(
            self.sdo_read_raw(index, subindex, timeout)?,
        ))
    }

    pub fn sdo_read_i32(&self, index: u16, subindex: u8, timeout: Duration) -> Result<i32> {
        Ok(i32::from_le_bytes(
            self.sdo_read_raw(index, subindex, timeout)?,
        ))
    }

    /// Read a `Float32` object (manufacturer encodings in both current
    /// CANopen-family manuals use IEEE-754 little-endian).
    pub fn sdo_read_f32(&self, index: u16, subindex: u8, timeout: Duration) -> Result<f32> {
        Ok(f32::from_le_bytes(
            self.sdo_read_raw(index, subindex, timeout)?,
        ))
    }

    // --- typed SDO writes ---------------------------------------------------

    pub fn sdo_write_u8(&self, index: u16, subindex: u8, value: u8, timeout: Duration) -> Result<()> {
        self.sdo_write(index, subindex, SdoValue::U8(value), timeout)
    }

    pub fn sdo_write_i8(&self, index: u16, subindex: u8, value: i8, timeout: Duration) -> Result<()> {
        self.sdo_write(index, subindex, SdoValue::I8(value), timeout)
    }

    pub fn sdo_write_u16(&self, index: u16, subindex: u8, value: u16, timeout: Duration) -> Result<()> {
        self.sdo_write(index, subindex, SdoValue::U16(value), timeout)
    }

    pub fn sdo_write_i16(&self, index: u16, subindex: u8, value: i16, timeout: Duration) -> Result<()> {
        self.sdo_write(index, subindex, SdoValue::I16(value), timeout)
    }

    pub fn sdo_write_u32(&self, index: u16, subindex: u8, value: u32, timeout: Duration) -> Result<()> {
        self.sdo_write(index, subindex, SdoValue::U32(value), timeout)
    }

    pub fn sdo_write_i32(&self, index: u16, subindex: u8, value: i32, timeout: Duration) -> Result<()> {
        self.sdo_write(index, subindex, SdoValue::I32(value), timeout)
    }

    pub fn sdo_write_f32(&self, index: u16, subindex: u8, value: f32, timeout: Duration) -> Result<()> {
        self.sdo_write(index, subindex, SdoValue::F32(value), timeout)
    }

    // --- NMT ----------------------------------------------------------------

    /// Send an NMT command for `node` (`0` = broadcast).
    pub fn send_nmt(&self, command: NmtCommand, node: u8) -> Result<()> {
        send_std_frame(self.bus.as_ref(), NMT_ID, &nmt_payload(command, node))
    }

    /// `StartRemoteNode` for this node (leaves pre-operational; SDO works in
    /// both states, PDO transmission only after start).
    pub fn start_node(&self) -> Result<()> {
        let payload = [NMT_START_REMOTE_NODE, self.node_id as u8];
        send_std_frame(self.bus.as_ref(), NMT_ID, &payload)
    }

    /// Last heartbeat payload byte received on `0x700 + node_id`, if any.
    pub fn heartbeat_state(&self) -> Option<u8> {
        self.heartbeat_state.lock().ok().and_then(|h| *h)
    }

    // --- frame claim / processing -------------------------------------------

    /// Standard-frame claim used by vendor `accepts_frame`: SDO responses and
    /// heartbeats for this node. TPDO claiming is vendor policy.
    pub fn accepts_std(&self, frame: &CanFrame) -> bool {
        is_sdo_rsp(frame, self.node_id) || is_heartbeat(frame, self.node_id)
    }

    /// Feed a received frame; see [`NodeOutcome`].
    pub fn process(&self, frame: &CanFrame) -> NodeOutcome {
        if is_sdo_rsp(frame, self.node_id) {
            self.sdo.push(frame.data);
            NodeOutcome::SdoQueued
        } else if is_heartbeat(frame, self.node_id) {
            let state = frame.data[0];
            if let Ok(mut hb) = self.heartbeat_state.lock() {
                *hb = Some(state);
            }
            NodeOutcome::Heartbeat(state)
        } else {
            NodeOutcome::NotClaimed
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cia402::enable_sequence;
    use crate::objects::{CONTROLWORD, CW_ENABLE_OPERATION, CW_SHUTDOWN, CW_SWITCH_ON, STATUSWORD};
    use motor_core::test_support::MockBus;

    fn rx(id: u32, data: [u8; 8]) -> CanFrame {
        CanFrame {
            arbitration_id: id,
            data,
            dlc: 8,
            is_extended: false,
            is_rx: true,
        }
    }

    fn sdo_ack(node: u16, index: u16, sub: u8) -> CanFrame {
        let mut data = [0u8; 8];
        data[0] = 0x60;
        data[1] = (index & 0xFF) as u8;
        data[2] = ((index >> 8) & 0xFF) as u8;
        data[3] = sub;
        rx(0x580 + u32::from(node), data)
    }

    fn sdo_data(node: u16, index: u16, sub: u8, value: [u8; 4]) -> CanFrame {
        let mut data = [0u8; 8];
        data[0] = 0x43;
        data[1] = (index & 0xFF) as u8;
        data[2] = ((index >> 8) & 0xFF) as u8;
        data[3] = sub;
        data[4..8].copy_from_slice(&value);
        rx(0x580 + u32::from(node), data)
    }

    #[test]
    fn rejects_out_of_range_node_ids() {
        let bus = Arc::new(MockBus::new());
        assert!(CanopenNode::new(0, bus.clone()).is_err());
        assert!(CanopenNode::new(128, bus.clone()).is_err());
        assert!(CanopenNode::new(1, bus.clone()).is_ok());
        assert!(CanopenNode::new(127, bus).is_ok());
    }

    #[test]
    fn sdo_read_u16_round_trips_request_and_reply() {
        let bus = Arc::new(MockBus::new());
        let node = CanopenNode::new(1, bus.clone()).expect("node");
        // reply pre-queued (normally fed by the core poll loop)
        let mut sw = [0u8; 4];
        sw[..2].copy_from_slice(&0x0237u16.to_le_bytes());
        node.process(&sdo_data(1, STATUSWORD, 0x00, sw));
        let sw = node
            .sdo_read_u16(STATUSWORD, 0x00, Duration::from_millis(50))
            .expect("read");
        assert_eq!(sw, 0x0237);
        // request frame on the wire
        let sent = bus.sent.lock().expect("sent").clone();
        assert_eq!(sent.len(), 1);
        assert_eq!(sent[0].arbitration_id, 0x601);
        assert_eq!(
            sent[0].data,
            [0x40, 0x41, 0x60, 0x00, 0, 0, 0, 0]
        );
    }

    #[test]
    fn sdo_read_f32_decodes_little_endian_float() {
        let bus = Arc::new(MockBus::new());
        let node = CanopenNode::new(3, bus).expect("node");
        node.process(&sdo_data(3, 0x6076, 0x00, 12.5f32.to_le_bytes()));
        let v = node
            .sdo_read_f32(0x6076, 0x00, Duration::from_millis(50))
            .expect("read");
        assert_eq!(v, 12.5);
    }

    #[test]
    fn sdo_write_u16_waits_for_matching_ack() {
        let bus = Arc::new(MockBus::new());
        let node = CanopenNode::new(1, bus.clone()).expect("node");
        node.process(&sdo_ack(1, CONTROLWORD, 0x00));
        node.sdo_write_u16(CONTROLWORD, 0x00, 0x0006, Duration::from_millis(50))
            .expect("write");
        let sent = bus.sent.lock().expect("sent").clone();
        assert_eq!(sent.len(), 1);
        assert_eq!(sent[0].arbitration_id, 0x601);
        assert_eq!(
            sent[0].data,
            [0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00]
        );
    }

    #[test]
    fn sdo_write_maps_abort_to_protocol_error() {
        let bus = Arc::new(MockBus::new());
        let node = CanopenNode::new(1, bus).expect("node");
        let mut data = [0u8; 8];
        data[0] = 0x80;
        data[1] = 0x40;
        data[2] = 0x60;
        data[4] = 0x06;
        data[5] = 0x02;
        node.process(&rx(0x581, data));
        let err = node
            .sdo_write_u16(CONTROLWORD, 0x00, 0x000F, Duration::from_millis(50))
            .expect_err("abort");
        assert!(err.to_string().contains("sdo abort"));
    }

    #[test]
    fn start_node_sends_targeted_nmt_start() {
        let bus = Arc::new(MockBus::new());
        let node = CanopenNode::new(7, bus.clone()).expect("node");
        node.start_node().expect("start");
        let sent = bus.sent.lock().expect("sent").clone();
        assert_eq!(sent.len(), 1);
        assert_eq!(sent[0].arbitration_id, 0x000);
        assert_eq!(&sent[0].data[..2], &[0x01, 0x07]);
    }

    #[test]
    fn process_routes_sdo_heartbeat_and_unknown() {
        let bus = Arc::new(MockBus::new());
        let node = CanopenNode::new(1, bus).expect("node");
        // SDO reply for another node is not claimed
        assert_eq!(
            node.process(&sdo_ack(2, CONTROLWORD, 0x00)),
            NodeOutcome::NotClaimed
        );
        // heartbeat for this node
        let hb = rx(0x701, [0x05, 0, 0, 0, 0, 0, 0, 0]);
        assert_eq!(node.process(&hb), NodeOutcome::Heartbeat(0x05));
        assert_eq!(node.heartbeat_state(), Some(0x05));
        // TPDO frame is not claimed by the node itself
        assert_eq!(node.process(&rx(0x181, [0; 8])), NodeOutcome::NotClaimed);
        // SDO reply for this node is queued
        assert_eq!(
            node.process(&sdo_ack(1, CONTROLWORD, 0x00)),
            NodeOutcome::SdoQueued
        );
    }

    #[test]
    fn enable_sequence_writes_shutdown_switchon_enable() {
        let bus = Arc::new(MockBus::new());
        let node = CanopenNode::new(1, bus.clone()).expect("node");
        for _ in 0..3 {
            node.process(&sdo_ack(1, CONTROLWORD, 0x00));
        }
        enable_sequence(&node, Duration::from_millis(50)).expect("enable");
        let sent = bus.sent.lock().expect("sent").clone();
        assert_eq!(sent.len(), 3);
        assert_eq!(sent[0].data[0], 0x2B);
        assert_eq!(u16::from_le_bytes([sent[0].data[4], sent[0].data[5]]), CW_SHUTDOWN);
        assert_eq!(
            u16::from_le_bytes([sent[1].data[4], sent[1].data[5]]),
            CW_SWITCH_ON
        );
        assert_eq!(
            u16::from_le_bytes([sent[2].data[4], sent[2].data[5]]),
            CW_ENABLE_OPERATION
        );
    }
}
