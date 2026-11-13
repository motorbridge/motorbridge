//! COB-ID based frame classification for the predefined connection set.
//!
//! Used by vendor `accepts_frame` / `process_feedback_frame` implementations
//! and by scan procedures to decide what a received standard frame is:
//!
//! | COB-ID range | meaning |
//! |---|---|
//! | `0x081..0x0FF` | EMCY (node 1..127) |
//! | `0x181..0x1FF` etc. (base `+0x100` per PDO) | TPDO 1..4 |
//! | `0x581..0x5FF` | SDO server response (expedited, dlc 8) |
//! | `0x701..0x77F` | heartbeat / NMT error control |

use motor_core::bus::CanFrame;

const NODE_MASK: u32 = 0x07F;
const TPDO_BASES: [u32; 4] = [0x180, 0x280, 0x380, 0x480];

/// SDO server response for `node_id` (`0x581..0x5FF`, 8 data bytes).
pub fn is_sdo_rsp(frame: &CanFrame, node_id: u16) -> bool {
    !frame.is_extended
        && frame.dlc >= 8
        && frame.arbitration_id == 0x580 + u32::from(node_id)
}

/// Heartbeat for `node_id` (`0x701..0x77F`, at least 1 data byte).
pub fn is_heartbeat(frame: &CanFrame, node_id: u16) -> bool {
    !frame.is_extended
        && frame.dlc >= 1
        && frame.arbitration_id == 0x700 + u32::from(node_id)
}

/// EMCY for `node_id` (`0x081..0x0FF`).
pub fn is_emcy(frame: &CanFrame, node_id: u16) -> bool {
    !frame.is_extended && frame.arbitration_id == 0x080 + u32::from(node_id)
}

/// TPDO number `n` (1..=4) for `node_id` (base `0x180/0x280/0x380/0x480`).
pub fn is_tpdo(frame: &CanFrame, n: u8, node_id: u16) -> bool {
    !frame.is_extended
        && (1..=4).contains(&n)
        && frame.arbitration_id == TPDO_BASES[usize::from(n - 1)] + u32::from(node_id)
}

/// Node id from an SDO response frame, if the frame is one.
pub fn sdo_rsp_node(frame: &CanFrame) -> Option<u16> {
    if frame.is_extended || frame.dlc < 8 {
        return None;
    }
    let id = frame.arbitration_id;
    match id {
        0x581..=0x5FF => Some((id & NODE_MASK) as u16),
        _ => None,
    }
}

/// Node id from a heartbeat frame, if the frame is one.
pub fn heartbeat_node(frame: &CanFrame) -> Option<u16> {
    if frame.is_extended || frame.dlc < 1 {
        return None;
    }
    let id = frame.arbitration_id;
    match id {
        0x701..=0x77F => Some((id & NODE_MASK) as u16),
        _ => None,
    }
}

/// `(pdo_number, node_id)` from a TPDO frame, if the frame is one.
pub fn tpdo_owner(frame: &CanFrame) -> Option<(u8, u16)> {
    if frame.is_extended || frame.dlc < 1 {
        return None;
    }
    let id = frame.arbitration_id;
    let node = (id & NODE_MASK) as u16;
    if node == 0 {
        return None;
    }
    for (i, base) in TPDO_BASES.iter().enumerate() {
        if id == base + u32::from(node) {
            return Some((i as u8 + 1, node));
        }
    }
    None
}

/// Node id from an EMCY frame, if the frame is one.
pub fn emcy_node(frame: &CanFrame) -> Option<u16> {
    if frame.is_extended || frame.dlc < 1 {
        return None;
    }
    let id = frame.arbitration_id;
    match id {
        0x081..=0x0FF => Some((id & NODE_MASK) as u16),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rx(id: u32, dlc: u8) -> CanFrame {
        CanFrame {
            arbitration_id: id,
            data: [0u8; 8],
            dlc,
            is_extended: false,
            is_rx: true,
        }
    }

    #[test]
    fn classifies_sdo_heartbeat_tpdo_emcy() {
        assert_eq!(sdo_rsp_node(&rx(0x581, 8)), Some(1));
        assert_eq!(sdo_rsp_node(&rx(0x5FF, 8)), Some(127));
        assert_eq!(sdo_rsp_node(&rx(0x581, 7)), None);
        assert_eq!(sdo_rsp_node(&rx(0x600, 8)), None);

        assert_eq!(heartbeat_node(&rx(0x701, 1)), Some(1));
        assert_eq!(heartbeat_node(&rx(0x700, 1)), None);

        assert_eq!(tpdo_owner(&rx(0x181, 8)), Some((1, 1)));
        assert_eq!(tpdo_owner(&rx(0x285, 8)), Some((2, 5)));
        assert_eq!(tpdo_owner(&rx(0x382, 8)), Some((3, 2)));
        assert_eq!(tpdo_owner(&rx(0x483, 8)), Some((4, 3)));
        // RPDO ranges are not TPDOs
        assert_eq!(tpdo_owner(&rx(0x201, 8)), None);
        assert_eq!(tpdo_owner(&rx(0x301, 8)), None);

        assert_eq!(emcy_node(&rx(0x081, 8)), Some(1));
        assert_eq!(emcy_node(&rx(0x080, 8)), None);

        let mut ext = rx(0x581, 8);
        ext.is_extended = true;
        assert_eq!(sdo_rsp_node(&ext), None);
    }

    #[test]
    fn per_node_predicates_match_owner_extraction() {
        assert!(is_sdo_rsp(&rx(0x583, 8), 3));
        assert!(!is_sdo_rsp(&rx(0x584, 8), 3));
        assert!(is_heartbeat(&rx(0x703, 1), 3));
        assert!(is_tpdo(&rx(0x183, 8), 1, 3));
        assert!(is_tpdo(&rx(0x283, 8), 2, 3));
        assert!(!is_tpdo(&rx(0x283, 8), 1, 3));
        assert!(is_emcy(&rx(0x083, 8), 3));
    }
}
