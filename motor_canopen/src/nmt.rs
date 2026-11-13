//! NMT (network management) master procedures.
//!
//! NMT frames use COB-ID `0x000` with a 2-byte payload
//! `[command, node_id]`; `node_id = 0` broadcasts to all nodes.

use std::sync::Arc;

use motor_core::bus::CanBus;

use crate::objects::{NMT_ID, NMT_START_REMOTE_NODE};
use crate::send_std_frame;

/// NMT command specifier (protocol value only — semantics of the resulting
/// node state are defined by the CANopen state model, not by a vendor).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NmtCommand {
    StartRemoteNode,
    StopNode,
    EnterPreOperational,
    ResetNode,
    ResetCommunication,
}

impl NmtCommand {
    pub fn code(self) -> u8 {
        match self {
            Self::StartRemoteNode => NMT_START_REMOTE_NODE,
            Self::StopNode => crate::objects::NMT_STOP_NODE,
            Self::EnterPreOperational => crate::objects::NMT_ENTER_PRE_OPERATIONAL,
            Self::ResetNode => crate::objects::NMT_RESET_NODE,
            Self::ResetCommunication => crate::objects::NMT_RESET_COMMUNICATION,
        }
    }
}

/// NMT frame payload: `[command, node_id]` (`0` = all nodes).
pub fn nmt_payload(command: NmtCommand, node_id: u8) -> [u8; 2] {
    [command.code(), node_id]
}

/// Send an NMT command on COB-ID `0x000`.
pub fn send_nmt(bus: &Arc<dyn CanBus>, command: NmtCommand, node_id: u8) -> motor_core::error::Result<()> {
    send_std_frame(bus.as_ref(), NMT_ID, &nmt_payload(command, node_id))
}

/// Convenience wrapper: `StartRemoteNode` broadcast (used before scanning or
/// enabling so all nodes leave the pre-operational state, where SDO still
/// works but PDO transmission does not).
pub fn send_nmt_start_all(bus: &Arc<dyn CanBus>) -> motor_core::error::Result<()> {
    send_nmt(bus, NmtCommand::StartRemoteNode, 0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use motor_core::bus::CanFrame;
    use motor_core::test_support::MockBus;

    fn sent_frames(bus: &MockBus) -> Vec<CanFrame> {
        bus.sent.lock().expect("sent").clone()
    }

    #[test]
    fn payload_encodes_command_and_node() {
        assert_eq!(
            nmt_payload(NmtCommand::StartRemoteNode, 0x01),
            [0x01, 0x01]
        );
        assert_eq!(
            nmt_payload(NmtCommand::ResetCommunication, 0x7F),
            [0x82, 0x7F]
        );
        assert_eq!(nmt_payload(NmtCommand::StopNode, 0x00), [0x02, 0x00]);
    }

    #[test]
    fn send_nmt_broadcast_uses_cob_id_zero() {
        let bus: Arc<MockBus> = Arc::new(MockBus::new());
        send_nmt_start_all(&(bus.clone() as Arc<dyn CanBus>)).expect("send");
        let frames = sent_frames(&bus);
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].arbitration_id, 0x000);
        assert!(!frames[0].is_extended);
        assert_eq!(frames[0].dlc, 2);
        assert_eq!(&frames[0].data[..2], &[0x01, 0x00]);
    }
}
