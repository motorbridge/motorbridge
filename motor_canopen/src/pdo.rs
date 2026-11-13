//! PDO mapping / communication configuration procedures.
//!
//! Object layout (predefined connection set — both current CANopen-family
//! manuals match):
//!
//! | PDO | COB-ID base | comm params | mapping params |
//! |---|---|---|---|
//! | RPDO 1..4 | `0x200/0x300/0x400/0x500` | `1400h..1403h` | `1600h..1603h` |
//! | TPDO 1..4 | `0x180/0x280/0x380/0x480` | `1800h..1803h` | `1A00h..1A03h` |
//!
//! COB-ID word (comm param sub 1): bit 31 marks the PDO invalid, bit 30
//! marks "no remote-frame triggering" (required set on devices that reject
//! RTR). Transmission type (sub 2): `0`/`1..240` sync, `254`/`255`
//! event-driven. Inhibit time (sub 3, 100 µs units) and event timer
//! (sub 5, 1 ms units) are only writable while the PDO is invalid — as is
//! the mapping table, hence the disable → remap → activate order below.
//!
//! The *content* of a mapping (which objects, how many bits) is vendor
//! semantics and lives in vendor crates; this module only knows the
//! reconfiguration procedure.

use std::time::Duration;

use motor_core::error::{MotorError, Result};

use crate::node::CanopenNode;
use crate::objects::{
    RPDO_COMMUNICATION_BASE, RPDO_MAPPING_BASE, TPDO_COMMUNICATION_BASE, TPDO_MAPPING_BASE,
};

/// COB-ID word bit 31: PDO invalid (must be set while reconfiguring).
pub const COB_ID_INVALID: u32 = 1 << 31;
/// COB-ID word bit 30: remote-frame triggering not supported (set on devices
/// that require the no-RTR marking).
pub const COB_ID_NO_RTR: u32 = 1 << 30;

/// One mapping entry: an object-dictionary location plus its bit length.
/// Wire encoding (u32, mapping sub 1..=n):
/// `index << 16 | subindex << 8 | bits`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PdoMappingEntry {
    pub index: u16,
    pub subindex: u8,
    pub bits: u8,
}

impl PdoMappingEntry {
    pub fn new(index: u16, subindex: u8, bits: u8) -> Self {
        Self {
            index,
            subindex,
            bits,
        }
    }

    pub fn encode(self) -> u32 {
        (u32::from(self.index) << 16) | (u32::from(self.subindex) << 8) | u32::from(self.bits)
    }
}

/// Communication-parameter object index for TPDO `n` (`1800h..1803h`).
pub fn tpdo_comm_index(n: u8) -> u16 {
    TPDO_COMMUNICATION_BASE + u16::from(n.saturating_sub(1))
}

/// Mapping-parameter object index for TPDO `n` (`1A00h..1A03h`).
pub fn tpdo_map_index(n: u8) -> u16 {
    TPDO_MAPPING_BASE + u16::from(n.saturating_sub(1))
}

/// Communication-parameter object index for RPDO `n` (`1400h..1403h`).
pub fn rpdo_comm_index(n: u8) -> u16 {
    RPDO_COMMUNICATION_BASE + u16::from(n.saturating_sub(1))
}

/// Mapping-parameter object index for RPDO `n` (`1600h..1603h`).
pub fn rpdo_map_index(n: u8) -> u16 {
    RPDO_MAPPING_BASE + u16::from(n.saturating_sub(1))
}

/// Full TPDO reconfiguration request. `cob_id` is the 11-bit identifier
/// (e.g. [`crate::objects::tpdo_id`] for the default layout); `cob_id_flags`
/// carries device-required COB-ID word bits such as
/// [`COB_ID_NO_RTR`] (bit 31 is managed by the procedure itself).
#[derive(Debug, Clone)]
pub struct TpdoConfig {
    /// TPDO number, 1..=4.
    pub number: u8,
    /// 11-bit COB-ID for this PDO.
    pub cob_id: u32,
    /// Extra COB-ID word bits to set in the activated value (bit 30 etc.).
    pub cob_id_flags: u32,
    /// Transmission type: `0`/`1..240` sync, `254`/`255` event-driven.
    pub transmission_type: u8,
    /// Inhibit time in 100 µs units (0 = disabled).
    pub inhibit_time_100us: u16,
    /// Event timer in 1 ms units (0 = disabled).
    pub event_timer_ms: u16,
    /// Mapping entries (at least one, total bits <= 64).
    pub entries: Vec<PdoMappingEntry>,
}

impl TpdoConfig {
    /// Apply the standard reconfiguration procedure on `node`:
    ///
    /// 1. invalidate the PDO (COB-ID bit 31) so mapping/comm params become
    ///    writable,
    /// 2. clear the mapping count, write each entry, restore the count,
    /// 3. write transmission type, inhibit time, event timer,
    /// 4. reactivate with the final COB-ID word (bit 31 clear).
    pub fn apply(&self, node: &CanopenNode, timeout: Duration) -> Result<()> {
        if !(1..=4).contains(&self.number) {
            return Err(MotorError::InvalidArgument(format!(
                "invalid TPDO number {}, expected 1..=4",
                self.number
            )));
        }
        if self.entries.is_empty() {
            return Err(MotorError::InvalidArgument(
                "TPDO mapping must contain at least one entry".to_string(),
            ));
        }
        let total_bits: u32 = self.entries.iter().map(|e| u32::from(e.bits)).sum();
        if total_bits > 64 {
            return Err(MotorError::InvalidArgument(format!(
                "TPDO mapping total {total_bits} bits exceeds the 64-bit PDO payload"
            )));
        }

        let comm = tpdo_comm_index(self.number);
        let map = tpdo_map_index(self.number);

        // 1. invalidate
        node.sdo_write_u32(
            comm,
            0x01,
            self.cob_id | self.cob_id_flags | COB_ID_INVALID,
            timeout,
        )?;
        // 2. remap: clear count, entries, restore count
        node.sdo_write_u8(map, 0x00, 0, timeout)?;
        for (i, entry) in self.entries.iter().enumerate() {
            let sub = u8::try_from(i + 1).map_err(|_| {
                MotorError::InvalidArgument("too many PDO mapping entries".to_string())
            })?;
            node.sdo_write_u32(map, sub, entry.encode(), timeout)?;
        }
        node.sdo_write_u8(map, 0x00, self.entries.len() as u8, timeout)?;
        // 3. communication parameters
        node.sdo_write_u8(comm, 0x02, self.transmission_type, timeout)?;
        node.sdo_write_u16(comm, 0x03, self.inhibit_time_100us, timeout)?;
        node.sdo_write_u16(comm, 0x05, self.event_timer_ms, timeout)?;
        // 4. activate
        node.sdo_write_u32(comm, 0x01, self.cob_id | self.cob_id_flags, timeout)
    }
}

/// Full RPDO reconfiguration request (mirror of [`TpdoConfig`] for the
/// receive direction; RPDOs have no event timer). Some devices ship with
/// all RPDO COB-IDs invalid (bit 31 set), so activation is explicit.
#[derive(Debug, Clone)]
pub struct RpdoConfig {
    /// RPDO number, 1..=4.
    pub number: u8,
    /// 11-bit COB-ID for this PDO (`0x200 + 0x100*(n-1) + node` by default).
    pub cob_id: u32,
    /// Extra COB-ID word bits to set in the activated value (bit 31 is
    /// managed by the procedure itself).
    pub cob_id_flags: u32,
    /// Transmission type: `0..240` updates on SYNC, `254`/`255` immediately.
    pub transmission_type: u8,
    /// Mapping entries (at least one, total bits <= 64).
    pub entries: Vec<PdoMappingEntry>,
}

impl RpdoConfig {
    /// Same disable → remap → activate procedure as TPDOs, minus the
    /// event-timer/inhibit steps.
    pub fn apply(&self, node: &CanopenNode, timeout: Duration) -> Result<()> {
        if !(1..=4).contains(&self.number) {
            return Err(MotorError::InvalidArgument(format!(
                "invalid RPDO number {}, expected 1..=4",
                self.number
            )));
        }
        if self.entries.is_empty() {
            return Err(MotorError::InvalidArgument(
                "RPDO mapping must contain at least one entry".to_string(),
            ));
        }
        let total_bits: u32 = self.entries.iter().map(|e| u32::from(e.bits)).sum();
        if total_bits > 64 {
            return Err(MotorError::InvalidArgument(format!(
                "RPDO mapping total {total_bits} bits exceeds the 64-bit PDO payload"
            )));
        }

        let comm = rpdo_comm_index(self.number);
        let map = rpdo_map_index(self.number);

        node.sdo_write_u32(
            comm,
            0x01,
            self.cob_id | self.cob_id_flags | COB_ID_INVALID,
            timeout,
        )?;
        node.sdo_write_u8(map, 0x00, 0, timeout)?;
        for (i, entry) in self.entries.iter().enumerate() {
            let sub = u8::try_from(i + 1).map_err(|_| {
                MotorError::InvalidArgument("too many PDO mapping entries".to_string())
            })?;
            node.sdo_write_u32(map, sub, entry.encode(), timeout)?;
        }
        node.sdo_write_u8(map, 0x00, self.entries.len() as u8, timeout)?;
        node.sdo_write_u8(comm, 0x02, self.transmission_type, timeout)?;
        node.sdo_write_u32(comm, 0x01, self.cob_id | self.cob_id_flags, timeout)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::objects::tpdo_id;
    use motor_core::bus::CanFrame;
    use motor_core::test_support::MockBus;
    use std::sync::Arc;

    fn sdo_rx(node: u16, cmd: u8, index: u16, sub: u8) -> CanFrame {
        let mut data = [0u8; 8];
        data[0] = cmd;
        data[1] = (index & 0xFF) as u8;
        data[2] = ((index >> 8) & 0xFF) as u8;
        data[3] = sub;
        CanFrame {
            arbitration_id: 0x580 + u32::from(node),
            data,
            dlc: 8,
            is_extended: false,
            is_rx: true,
        }
    }

    /// Queue acks for every write the procedure performs, in order.
    fn queue_acks(node: &CanopenNode, cfg: &TpdoConfig) {
        let comm = tpdo_comm_index(cfg.number);
        let map = tpdo_map_index(cfg.number);
        node.process(&sdo_rx(1, 0x60, comm, 0x01)); // invalidate
        node.process(&sdo_rx(1, 0x60, map, 0x00)); // clear count
        for i in 0..cfg.entries.len() as u8 {
            node.process(&sdo_rx(1, 0x60, map, i + 1));
        }
        node.process(&sdo_rx(1, 0x60, map, 0x00)); // restore count
        node.process(&sdo_rx(1, 0x60, comm, 0x02));
        node.process(&sdo_rx(1, 0x60, comm, 0x03));
        node.process(&sdo_rx(1, 0x60, comm, 0x05));
        node.process(&sdo_rx(1, 0x60, comm, 0x01)); // activate
    }

    fn config() -> TpdoConfig {
        TpdoConfig {
            number: 2,
            cob_id: tpdo_id(2, 1),
            cob_id_flags: COB_ID_NO_RTR,
            transmission_type: 254,
            inhibit_time_100us: 100,
            event_timer_ms: 10,
            entries: vec![
                PdoMappingEntry::new(0x6064, 0x00, 32),
                PdoMappingEntry::new(0x606C, 0x00, 32),
            ],
        }
    }

    #[test]
    fn encodes_mapping_entry_word() {
        let e = PdoMappingEntry::new(0x6040, 0x00, 0x10);
        assert_eq!(e.encode(), 0x60400010);
        let e = PdoMappingEntry::new(0x2204, 0x01, 0x10);
        assert_eq!(e.encode(), 0x22040110);
    }

    #[test]
    fn object_indices_step_by_one() {
        assert_eq!(tpdo_comm_index(1), 0x1800);
        assert_eq!(tpdo_comm_index(4), 0x1803);
        assert_eq!(tpdo_map_index(1), 0x1A00);
        assert_eq!(tpdo_map_index(4), 0x1A03);
        assert_eq!(rpdo_comm_index(1), 0x1400);
        assert_eq!(rpdo_comm_index(4), 0x1403);
        assert_eq!(rpdo_map_index(1), 0x1600);
        assert_eq!(rpdo_map_index(4), 0x1603);
    }

    #[test]
    fn rpdo_apply_writes_remap_sequence() {
        let bus = Arc::new(MockBus::new());
        let node = CanopenNode::new(1, bus.clone()).expect("node");
        let cfg = RpdoConfig {
            number: 4,
            cob_id: crate::objects::rpdo_id(4, 1),
            cob_id_flags: 0,
            transmission_type: 255,
            entries: vec![
                PdoMappingEntry::new(0x2004, 0x02, 32),
                PdoMappingEntry::new(0x2004, 0x03, 32),
            ],
        };
        // acks in write order: invalidate, clear count, entry1, entry2,
        // restore count, type, activate
        let comm = rpdo_comm_index(4);
        let map = rpdo_map_index(4);
        node.process(&sdo_rx(1, 0x60, comm, 0x01));
        node.process(&sdo_rx(1, 0x60, map, 0x00));
        node.process(&sdo_rx(1, 0x60, map, 0x01));
        node.process(&sdo_rx(1, 0x60, map, 0x02));
        node.process(&sdo_rx(1, 0x60, map, 0x00));
        node.process(&sdo_rx(1, 0x60, comm, 0x02));
        node.process(&sdo_rx(1, 0x60, comm, 0x01));
        cfg.apply(&node, Duration::from_millis(50)).expect("apply");

        let sent = bus.sent.lock().expect("sent").clone();
        assert_eq!(sent.len(), 7);
        // mapping entries map 2004h_02 / 2004h_03
        let e1 = u32::from_le_bytes([sent[2].data[4], sent[2].data[5], sent[2].data[6], sent[2].data[7]]);
        assert_eq!(e1, 0x20040220);
        let e2 = u32::from_le_bytes([sent[3].data[4], sent[3].data[5], sent[3].data[6], sent[3].data[7]]);
        assert_eq!(e2, 0x20040320);
        // invalidate has bit31, activate does not
        let w0 = u32::from_le_bytes([sent[0].data[4], sent[0].data[5], sent[0].data[6], sent[0].data[7]]);
        assert_eq!(w0, 0x0000_0501 | COB_ID_INVALID);
        let w6 = u32::from_le_bytes([sent[6].data[4], sent[6].data[5], sent[6].data[6], sent[6].data[7]]);
        assert_eq!(w6, 0x0000_0501);
        // transmission type 255
        assert_eq!(sent[5].data[4], 255);
    }

    #[test]
    fn apply_writes_disable_remap_activate_sequence() {
        let bus = Arc::new(MockBus::new());
        let node = CanopenNode::new(1, bus.clone()).expect("node");
        let cfg = config();
        queue_acks(&node, &cfg);
        cfg.apply(&node, Duration::from_millis(50)).expect("apply");

        let sent = bus.sent.lock().expect("sent").clone();
        assert_eq!(sent.len(), 9);
        // 1. invalidate: COB-ID word (sub 1) has bit31|bit30 set
        assert_eq!(sent[0].data[0], 0x23);
        assert_eq!(sent[0].data[1..4], [0x01, 0x18, 0x01]);
        let word = u32::from_le_bytes([sent[0].data[4], sent[0].data[5], sent[0].data[6], sent[0].data[7]]);
        assert_eq!(word, 0x4000_0281 | COB_ID_INVALID);
        // 2. mapping clear + entries + count
        assert_eq!(sent[1].data[0], 0x2F);
        assert_eq!(sent[1].data[3], 0x00);
        assert_eq!(sent[1].data[4], 0x00);
        let entry = u32::from_le_bytes([sent[2].data[4], sent[2].data[5], sent[2].data[6], sent[2].data[7]]);
        assert_eq!(entry, 0x60640020);
        let entry = u32::from_le_bytes([sent[3].data[4], sent[3].data[5], sent[3].data[6], sent[3].data[7]]);
        assert_eq!(entry, 0x606C0020);
        assert_eq!(sent[4].data[3], 0x00);
        assert_eq!(sent[4].data[4], 0x02);
        // 3. comm params
        assert_eq!(sent[5].data[3], 0x02);
        assert_eq!(sent[5].data[4], 254);
        assert_eq!(sent[6].data[3], 0x03);
        assert_eq!(u16::from_le_bytes([sent[6].data[4], sent[6].data[5]]), 100);
        assert_eq!(sent[7].data[3], 0x05);
        assert_eq!(u16::from_le_bytes([sent[7].data[4], sent[7].data[5]]), 10);
        // 4. activate: bit31 clear, bit30 kept
        let word = u32::from_le_bytes([sent[8].data[4], sent[8].data[5], sent[8].data[6], sent[8].data[7]]);
        assert_eq!(word, 0x4000_0281);
    }

    #[test]
    fn apply_rejects_bad_number_empty_and_oversized_mapping() {
        let bus = Arc::new(MockBus::new());
        let node = CanopenNode::new(1, bus).expect("node");
        let mut cfg = config();
        cfg.number = 5;
        let err = cfg.apply(&node, Duration::from_millis(5)).expect_err("number");
        assert!(err.to_string().contains("TPDO number"));

        let mut cfg = config();
        cfg.entries.clear();
        let err = cfg.apply(&node, Duration::from_millis(5)).expect_err("empty");
        assert!(err.to_string().contains("at least one"));

        let mut cfg = config();
        cfg.entries = vec![PdoMappingEntry::new(0x2004, 0x01, 0xFF)];
        let err = cfg
            .apply(&node, Duration::from_millis(5))
            .expect_err("size");
        assert!(err.to_string().contains("64-bit"));
    }
}
