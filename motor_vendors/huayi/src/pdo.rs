//! Huayi PDO semantics — factory-default mappings (manual §4.5/§5.1), the
//! activation procedures layered on [`motor_canopen::pdo`], and TPDO payload
//! decoders for the vendor's fixed frame layouts.
//!
//! Factory defaults:
//!
//! | PDO | COB-ID word (node 1) | mapping |
//! |---|---|---|
//! | TPDO1 | `0xC000_0181` | `603F`(16b) + `6041`(16b) + `6061`(8b) |
//! | TPDO2 | `0xC000_0281` | `6064`(32b) + `606C`(32b) |
//! | TPDO3 | `0xC000_0381` | `6077`(16b) + `2204h_01`(16b) + `2204h_02`(16b) |
//! | RPDO4 | `0x8000_0501` | 2003h slow-MIT path (7 entries) |
//!
//! All COB-ID words ship with bit 31 set (PDO invalid), so nothing streams
//! until activated. TPDO words also carry bit 30 (no remote-frame
//! triggering) — the manual states this device does not support RTR and the
//! bit must be written as `1`.

use std::time::Duration;

use motor_canopen::node::CanopenNode;
use motor_canopen::objects::{
    rpdo_id, tpdo_id, ERROR_CODE, MODES_OF_OPERATION_DISPLAY, POSITION_ACTUAL_VALUE, STATUSWORD,
    TORQUE_ACTUAL_VALUE, VELOCITY_ACTUAL_VALUE,
};
use motor_canopen::pdo::{
    tpdo_comm_index, COB_ID_INVALID, COB_ID_NO_RTR, PdoMappingEntry, RpdoConfig, TpdoConfig,
};
use motor_core::error::Result;

use crate::objects::od;

// ---------------------------------------------------------------------------
// factory-default mappings (manual §4.5, "默认映射内容")
// ---------------------------------------------------------------------------

/// TPDO1 (`0x180 + node`): error code, statusword, mode display — 5 bytes.
/// **Patch frame** for the cached state (does not advance the sequence).
pub const TPDO1_MAP: &[PdoMappingEntry] = &[
    PdoMappingEntry {
        index: ERROR_CODE,
        subindex: 0x00,
        bits: 16,
    },
    PdoMappingEntry {
        index: STATUSWORD,
        subindex: 0x00,
        bits: 16,
    },
    PdoMappingEntry {
        index: MODES_OF_OPERATION_DISPLAY,
        subindex: 0x00,
        bits: 8,
    },
];

/// TPDO2 (`0x280 + node`): position + velocity, both `Float32` (Rev,
/// Rev/s) — 8 bytes. **Anchor frame** for the cached state (advances the
/// sequence, refreshes the timestamp).
pub const TPDO2_MAP: &[PdoMappingEntry] = &[
    PdoMappingEntry {
        index: POSITION_ACTUAL_VALUE,
        subindex: 0x00,
        bits: 32,
    },
    PdoMappingEntry {
        index: VELOCITY_ACTUAL_VALUE,
        subindex: 0x00,
        bits: 32,
    },
];

/// TPDO3 (`0x380 + node`): torque (Int16, ‰ peak) + driver/motor
/// temperatures (Int16, 0.1 °C) — 6 bytes. **Patch frame**.
pub const TPDO3_MAP: &[PdoMappingEntry] = &[
    PdoMappingEntry {
        index: TORQUE_ACTUAL_VALUE,
        subindex: 0x00,
        bits: 16,
    },
    PdoMappingEntry {
        index: od::TEMPERATURES,
        subindex: 0x01,
        bits: 16,
    },
    PdoMappingEntry {
        index: od::TEMPERATURES,
        subindex: 0x02,
        bits: 16,
    },
];

/// RPDO4 (`0x500 + node`): compressed MIT control word —
/// `2004h_02` (bits 31:0) + `2004h_03` (bits 63:32), 8 bytes. The factory
/// mapping carries the slow 2003h path, so the compressed channel needs
/// this remap before use.
pub const RPDO4_MIT_MAP: &[PdoMappingEntry] = &[
    PdoMappingEntry {
        index: od::MIT_COMPRESSED,
        subindex: 0x02,
        bits: 32,
    },
    PdoMappingEntry {
        index: od::MIT_COMPRESSED,
        subindex: 0x03,
        bits: 32,
    },
];

/// Mapping words as documented in the manual (used by tests as golden
/// values).
pub const TPDO_MAP_WORDS: [&[u32]; 3] = [
    &[0x603F_0010, 0x6041_0010, 0x6061_0008],
    &[0x6064_0020, 0x606C_0020],
    &[0x6077_0010, 0x2204_0110, 0x2204_0210],
];

/// Default asynchronous transmission type (manual: `254`/`255` are
/// event-driven; `255` = device-profile event).
pub const DEFAULT_TRANSMISSION_TYPE: u8 = 255;

// ---------------------------------------------------------------------------
// activation procedures
// ---------------------------------------------------------------------------

/// Build the reconfiguration request for one of the three default TPDOs.
/// The mapping content is the factory one (see [`TPDO1_MAP`] etc.); only
/// the COB-ID word is activated and the event timer set.
pub fn default_tpdo_config(n: u8, node_id: u16, event_timer_ms: u16) -> TpdoConfig {
    let entries = match n {
        1 => TPDO1_MAP,
        2 => TPDO2_MAP,
        3 => TPDO3_MAP,
        _ => TPDO1_MAP,
    };
    TpdoConfig {
        number: n,
        cob_id: tpdo_id(n, node_id),
        // manual: this device does not support RTR — bit 30 must be written 1
        cob_id_flags: COB_ID_NO_RTR,
        transmission_type: DEFAULT_TRANSMISSION_TYPE,
        inhibit_time_100us: 0,
        event_timer_ms,
        entries: entries.to_vec(),
    }
}

/// Activate the three default TPDOs (factory COB-ID words are invalid, so
/// nothing streams until this runs). `event_timer_ms = 0` means the event
/// timer stays off (transmission then only on state-change events).
///
/// Mirrors the role of RobStride `set_active_report`: call once after the
/// drive is configured, then feed [`crate::motor::HuayiMotor`] frames via
/// `process_feedback_frame` (the shared `CoreController` receive loop does
/// this automatically).
pub fn activate_default_tpdos(
    node: &CanopenNode,
    event_timer_ms: u16,
    timeout: Duration,
) -> Result<()> {
    for n in 1..=3u8 {
        default_tpdo_config(n, node.node_id(), event_timer_ms).apply(node, timeout)?;
    }
    Ok(())
}

/// Invalidate the three default TPDO COB-ID words (bit 31) — periodic
/// reporting off. Values stay in the dictionary; reactivation is a plain
/// [`activate_default_tpdos`] away.
pub fn deactivate_default_tpdos(node: &CanopenNode, timeout: Duration) -> Result<()> {
    for n in 1..=3u8 {
        node.sdo_write_u32(
            tpdo_comm_index(n),
            0x01,
            tpdo_id(n, node.node_id()) | COB_ID_NO_RTR | COB_ID_INVALID,
            timeout,
        )?;
    }
    Ok(())
}

/// Remap RPDO4 to the compressed MIT control word and enable the 2004h
/// parameter source. After this, an 8-byte classic frame to
/// `0x500 + node_id` (low 32 bits first) is a complete MIT command — the
/// real-time channel (manual §5.2).
pub fn configure_mit_rpdo(node: &CanopenNode, timeout: Duration) -> Result<()> {
    RpdoConfig {
        number: 4,
        cob_id: rpdo_id(4, node.node_id()),
        cob_id_flags: 0,
        transmission_type: DEFAULT_TRANSMISSION_TYPE,
        entries: RPDO4_MIT_MAP.to_vec(),
    }
    .apply(node, timeout)?;
    // 2004h_1: 1 = parameters come from the compressed 2004h channel
    node.sdo_write_u8(od::MIT_COMPRESSED, 0x01, 1, timeout)
}

// ---------------------------------------------------------------------------
// TPDO payload decoders (vendor-fixed layouts)
// ---------------------------------------------------------------------------

/// Decoded TPDO1 (raw wire values).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Tpdo1Data {
    pub error_code: u16,
    pub statusword: u16,
    pub mode_display: u8,
}

/// Decode TPDO1 (`603F` + `6041` + `6061`, 5 bytes).
pub fn decode_tpdo1(data: &[u8], dlc: u8) -> Option<Tpdo1Data> {
    if (dlc as usize) < 5 || data.len() < 5 {
        return None;
    }
    Some(Tpdo1Data {
        error_code: u16::from_le_bytes([data[0], data[1]]),
        statusword: u16::from_le_bytes([data[2], data[3]]),
        mode_display: data[4],
    })
}

/// Decoded TPDO2 (Huayi wire units: position Rev, velocity Rev/s).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Tpdo2Data {
    pub position_rev: f32,
    pub velocity_rev_s: f32,
}

/// Decode TPDO2 (`6064` + `606C`, two `Float32`, 8 bytes).
pub fn decode_tpdo2(data: &[u8], dlc: u8) -> Option<Tpdo2Data> {
    if (dlc as usize) < 8 || data.len() < 8 {
        return None;
    }
    Some(Tpdo2Data {
        position_rev: f32::from_le_bytes([data[0], data[1], data[2], data[3]]),
        velocity_rev_s: f32::from_le_bytes([data[4], data[5], data[6], data[7]]),
    })
}

/// Decoded TPDO3 (raw wire values: ‰ peak, 0.1 °C units).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Tpdo3Data {
    pub torque_permille: i16,
    pub t_driver_raw: i16,
    pub t_motor_raw: i16,
}

/// Decode TPDO3 (`6077` + `2204h_01` + `2204h_02`, 6 bytes).
pub fn decode_tpdo3(data: &[u8], dlc: u8) -> Option<Tpdo3Data> {
    if (dlc as usize) < 6 || data.len() < 6 {
        return None;
    }
    Some(Tpdo3Data {
        torque_permille: i16::from_le_bytes([data[0], data[1]]),
        t_driver_raw: i16::from_le_bytes([data[2], data[3]]),
        t_motor_raw: i16::from_le_bytes([data[4], data[5]]),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use motor_canopen::pdo::rpdo_comm_index;
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

    /// Queue write-acks for a full TpdoConfig apply, in write order.
    fn queue_tpdo_acks(node: &CanopenNode, cfg: &TpdoConfig) {
        let comm = tpdo_comm_index(cfg.number);
        let map = motor_canopen::pdo::tpdo_map_index(cfg.number);
        node.process(&sdo_rx(1, 0x60, comm, 0x01));
        node.process(&sdo_rx(1, 0x60, map, 0x00));
        for i in 1..=cfg.entries.len() as u8 {
            node.process(&sdo_rx(1, 0x60, map, i));
        }
        node.process(&sdo_rx(1, 0x60, map, 0x00));
        node.process(&sdo_rx(1, 0x60, comm, 0x02));
        node.process(&sdo_rx(1, 0x60, comm, 0x03));
        node.process(&sdo_rx(1, 0x60, comm, 0x05));
        node.process(&sdo_rx(1, 0x60, comm, 0x01));
    }

    #[test]
    fn default_maps_match_manual_words() {
        for (n, words) in TPDO_MAP_WORDS.iter().enumerate() {
            let map = match n + 1 {
                1 => TPDO1_MAP,
                2 => TPDO2_MAP,
                3 => TPDO3_MAP,
                _ => unreachable!(),
            };
            assert_eq!(map.len(), words.len());
            for (entry, word) in map.iter().zip(words.iter()) {
                assert_eq!(entry.encode(), *word);
            }
        }
        // RPDO4 compressed-MIT map
        assert_eq!(RPDO4_MIT_MAP[0].encode(), 0x2004_0220);
        assert_eq!(RPDO4_MIT_MAP[1].encode(), 0x2004_0320);
    }

    #[test]
    fn default_tpdo_config_carries_no_rtr_and_cob_id() {
        let cfg = default_tpdo_config(2, 5, 10);
        assert_eq!(cfg.cob_id, 0x285);
        assert_eq!(cfg.cob_id_flags, COB_ID_NO_RTR);
        assert_eq!(cfg.transmission_type, 255);
        assert_eq!(cfg.event_timer_ms, 10);
        assert_eq!(cfg.entries.len(), 2);
    }

    #[test]
    fn activate_default_tpdos_writes_all_three() {
        let bus = Arc::new(MockBus::new());
        let node = CanopenNode::new(1, bus.clone()).expect("node");
        for n in 1..=3u8 {
            queue_tpdo_acks(&node, &default_tpdo_config(n, 1, 10));
        }
        activate_default_tpdos(&node, 10, Duration::from_millis(50)).expect("activate");

        let sent = bus.sent.lock().expect("sent").clone();
        // 3 TPDOs × (invalidate + clear + entries + count + type + inhibit
        // + event + activate): TPDO1/3 have 3 entries (10 writes), TPDO2
        // has 2 (9 writes) → 10 + 9 + 10 = 29
        assert_eq!(sent.len(), 10 + 9 + 10);
        // first frame of TPDO1: invalidate 1800h_1 with bit31|bit30
        assert_eq!(sent[0].data[1..4], [0x00, 0x18, 0x01]);
        let word = u32::from_le_bytes([
            sent[0].data[4],
            sent[0].data[5],
            sent[0].data[6],
            sent[0].data[7],
        ]);
        assert_eq!(word, 0x4000_0181 | COB_ID_INVALID);
        // last frame of TPDO1: activate without bit31
        let last1 = &sent[9];
        let word = u32::from_le_bytes([
            last1.data[4],
            last1.data[5],
            last1.data[6],
            last1.data[7],
        ]);
        assert_eq!(word, 0x4000_0181);
        // TPDO2 activate word (block 2: frames 10..19)
        let last2 = &sent[10 + 9 - 1];
        let word = u32::from_le_bytes([
            last2.data[4],
            last2.data[5],
            last2.data[6],
            last2.data[7],
        ]);
        assert_eq!(word, 0x4000_0281);
        // TPDO1 event timer: sub 0x05, value 10 (frame 8 of block 1)
        let ev = &sent[8];
        assert_eq!(ev.data[3], 0x05);
        assert_eq!(u16::from_le_bytes([ev.data[4], ev.data[5]]), 10);
    }

    #[test]
    fn deactivate_default_tpdos_sets_invalid_bit() {
        let bus = Arc::new(MockBus::new());
        let node = CanopenNode::new(1, bus.clone()).expect("node");
        for n in 1..=3u8 {
            node.process(&sdo_rx(1, 0x60, tpdo_comm_index(n), 0x01));
        }
        deactivate_default_tpdos(&node, Duration::from_millis(50)).expect("deactivate");

        let sent = bus.sent.lock().expect("sent").clone();
        assert_eq!(sent.len(), 3);
        let words: Vec<u32> = sent
            .iter()
            .map(|f| {
                u32::from_le_bytes([f.data[4], f.data[5], f.data[6], f.data[7]])
            })
            .collect();
        assert_eq!(words[0], 0x4000_0181 | COB_ID_INVALID);
        assert_eq!(words[1], 0x4000_0281 | COB_ID_INVALID);
        assert_eq!(words[2], 0x4000_0381 | COB_ID_INVALID);
    }

    #[test]
    fn configure_mit_rpdo_remaps_and_enables() {
        let bus = Arc::new(MockBus::new());
        let node = CanopenNode::new(1, bus.clone()).expect("node");
        // RPDO4 apply: 7 writes + 2004h_1 enable ack
        let comm = rpdo_comm_index(4);
        let map = motor_canopen::pdo::rpdo_map_index(4);
        node.process(&sdo_rx(1, 0x60, comm, 0x01));
        node.process(&sdo_rx(1, 0x60, map, 0x00));
        node.process(&sdo_rx(1, 0x60, map, 0x01));
        node.process(&sdo_rx(1, 0x60, map, 0x02));
        node.process(&sdo_rx(1, 0x60, map, 0x00));
        node.process(&sdo_rx(1, 0x60, comm, 0x02));
        node.process(&sdo_rx(1, 0x60, comm, 0x01));
        node.process(&sdo_rx(1, 0x60, od::MIT_COMPRESSED, 0x01));
        configure_mit_rpdo(&node, Duration::from_millis(50)).expect("configure");

        let sent = bus.sent.lock().expect("sent").clone();
        assert_eq!(sent.len(), 8);
        // RPDO4 activate word: 0x501, no bit30 (RPDO does not require it)
        let word = u32::from_le_bytes([
            sent[6].data[4],
            sent[6].data[5],
            sent[6].data[6],
            sent[6].data[7],
        ]);
        assert_eq!(word, 0x0000_0501);
        // last write: 2004h_1 = 1 (compressed parameter source)
        assert_eq!(sent[7].data[1..4], [0x04, 0x20, 0x01]);
        assert_eq!(sent[7].data[0], 0x2F);
        assert_eq!(sent[7].data[4], 1);
    }

    #[test]
    fn decoders_roundtrip_known_bytes() {
        // TPDO1
        let d = decode_tpdo1(&[0x10, 0x23, 0x37, 0x02, 5], 5).expect("tpdo1");
        assert_eq!(d.error_code, 0x2310);
        assert_eq!(d.statusword, 0x0237);
        assert_eq!(d.mode_display, 5);
        assert!(decode_tpdo1(&[0, 0, 0, 0], 4).is_none());

        // TPDO2: 1.5 Rev, -0.25 Rev/s
        let mut buf = [0u8; 8];
        buf[0..4].copy_from_slice(&1.5f32.to_le_bytes());
        buf[4..8].copy_from_slice(&(-0.25f32).to_le_bytes());
        let d = decode_tpdo2(&buf, 8).expect("tpdo2");
        assert!((d.position_rev - 1.5).abs() < 1e-6);
        assert!((d.velocity_rev_s + 0.25).abs() < 1e-6);
        assert!(decode_tpdo2(&buf, 7).is_none());

        // TPDO3: -250‰, 25.5 °C (255), -10.0 °C (-100)
        let d = decode_tpdo3(
            &[0x06, 0xFF, 0xFF, 0x00, 0x9C, 0xFF],
            6,
        )
        .expect("tpdo3");
        assert_eq!(d.torque_permille, -250);
        assert_eq!(d.t_driver_raw, 255);
        assert_eq!(d.t_motor_raw, -100);
        assert!(decode_tpdo3(&[0; 5], 5).is_none());
    }
}
