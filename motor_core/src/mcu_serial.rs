//! UART-to-CAN MCU bridge transport.
//!
//! Host-side leaf of the `CanBus` tree: serializes `CanFrame`s to bytes for
//! `send` and parses bytes back for `recv`. The MCU owns the real CAN
//! controller. Multi-vendor frame assembly happens above the trait unchanged.
//!
//! Wire format: `[0xA5][LEN][can_ID LE4][DATA(LEN)][CRC8][0x5A]`.
//! LEN = DLC (0..=8); can_ID bit31 = extended flag; CRC8 (poly 0x07, init 0)
//! covers `[LEN][can_ID][DATA]`. Parse is length-anchored; a stray delimiter
//! in DATA is rejected by CRC and resyncs one byte (mirrors `dm_serial`).

use crate::bus::{CanBus, CanFrame};
use crate::error::{MotorError, Result};
use serialport::{DataBits, FlowControl, Parity, SerialPort, StopBits};
use std::collections::VecDeque;
use std::io::{Read, Write};
use std::sync::Mutex;
use std::time::{Duration, Instant};

const HEADER: u8 = 0xA5;
const TAIL: u8 = 0x5A;
const MAX_DLC: usize = 8;
const MAX_FRAME: usize = 1 + 1 + 4 + MAX_DLC + 1 + 1;

/// Reserved 29-bit can_id the MCU uses to push a structured link-status
/// snapshot back to the host — the mcu-serial analogue of a CAN error frame.
/// Sent as an extended frame (bit31 set on the wire), DLC=8. Sits next to the
/// pure-USB ping id (`0x1FFFFFEE`) at the top of the extended-id space so it
/// never collides with a real motor arbitration id. The host never TXes this
/// id; only the MCU emits it on anomaly (non-ERROR_ACTIVE state or any
/// error/drop counter > 0), so a bus-off / TX-ACK-failure / RX-drop /
/// stream-corruption event surfaces as a `MotorError::BusStatus` instead of
/// every fault collapsing to a recv timeout.
const STATUS_ID: u32 = 0x1FFF_FFEF;

/// Flag bits in a STATUS frame's `flags` byte (bit position). Mirrors the
/// firmware `build_status_payload` flag assignments exactly.
mod status_flag {
    pub const BUS_OFF: u8 = 0x01;
    pub const TX_FAILED: u8 = 0x02;
    pub const RX_DROPPED: u8 = 0x04;
    pub const RX_CRC_BAD: u8 = 0x08;
    pub const RX_OVERSIZE: u8 = 0x10;
    pub const APP_TX_DROPPED: u8 = 0x20;
    pub const BUS_ERROR: u8 = 0x40;
}

/// Decoded MCU link-status snapshot carried in a STATUS frame. Fields map 1:1
/// to the firmware's `can_console_can_status_t` + app drop counters.
#[derive(Clone, Debug)]
pub struct McuSerialStatus {
    /// TWAI state: 0=STOPPED,1=ERROR_ACTIVE,2=ERROR_WARNING,3=ERROR_PASSIVE,
    /// 4=BUS_OFF (see `can_console_can_state_t` on the firmware side).
    pub state: u8,
    /// Bitmask of `status_flag::*` — which counters are non-zero.
    pub flags: u8,
    /// TX error counter (TEC), saturated to u8.
    pub tx_error_count: u8,
    /// RX error counter (REC), saturated to u8.
    pub rx_error_count: u8,
    /// CAN TX attempts that failed (no ACK / tx_failed), saturated to u16.
    pub tx_failed: u16,
    /// Frames the MCU received but dropped before reaching the host (driver
    /// RX-queue overflow + USB write timeout + app-level drops), saturated to
    /// u16.
    pub rx_dropped: u16,
}

impl McuSerialStatus {
    pub fn state_name(&self) -> &'static str {
        match self.state {
            0 => "STOPPED",
            1 => "ERROR_ACTIVE",
            2 => "ERROR_WARNING",
            3 => "ERROR_PASSIVE",
            4 => "BUS_OFF",
            _ => "UNKNOWN",
        }
    }
    pub fn bus_off(&self) -> bool {
        self.flags & status_flag::BUS_OFF != 0
    }
    pub fn tx_failed(&self) -> bool {
        self.flags & status_flag::TX_FAILED != 0
    }
    pub fn rx_dropped(&self) -> bool {
        self.flags & status_flag::RX_DROPPED != 0
    }
    pub fn crc_bad(&self) -> bool {
        self.flags & status_flag::RX_CRC_BAD != 0
    }
    pub fn oversize(&self) -> bool {
        self.flags & status_flag::RX_OVERSIZE != 0
    }
    pub fn app_tx_dropped(&self) -> bool {
        self.flags & status_flag::APP_TX_DROPPED != 0
    }
    pub fn bus_error(&self) -> bool {
        self.flags & status_flag::BUS_ERROR != 0
    }

    /// Whether this status is a hard CAN-bus fault that makes further
    /// communication impossible or unreliable, so `recv` should surface it as
    /// a `BusStatus` error instead of draining and continuing.
    ///
    /// Only `ERROR_PASSIVE` / `BUS_OFF` (or the `BUS_OFF` flag set before
    /// `state` catches up) are hard. Everything else — `ERROR_ACTIVE` /
    /// `ERROR_WARNING` with soft counters like `rx_dropped` / `tx_failed` /
    /// `app_tx_dropped`, or a transient `RX_CRC_BAD` / `RX_OVERSIZE` /
    /// `BUS_ERROR` while the controller is still `ERROR_ACTIVE` — is a benign
    /// anomaly the host drains through. The MCU emits STATUS continuously
    /// once a sticky counter (e.g. `rx_dropped` from a single FIFO overflow)
    /// is non-zero; aborting on it would block every recv-based op until a
    /// power cycle resets the MCU. Draining mirrors the Python demo's poll
    /// loop, which catches `BusStatus` and keeps looping.
    pub fn is_hard_fault(&self) -> bool {
        self.bus_off() || self.state == 3 || self.state == 4
    }

    /// Comma-separated list of the fault flags that are set, for the Display
    /// impl and for log lines. Empty when no flag is set (which shouldn't
    /// happen in practice — the MCU only sends STATUS on anomaly).
    fn flag_names(&self) -> String {
        let mut names: Vec<&str> = Vec::new();
        if self.bus_off() {
            names.push("bus_off");
        }
        if self.tx_failed() {
            names.push("tx_failed");
        }
        if self.rx_dropped() {
            names.push("rx_dropped");
        }
        if self.crc_bad() {
            names.push("rx_crc_bad");
        }
        if self.oversize() {
            names.push("rx_oversize");
        }
        if self.app_tx_dropped() {
            names.push("app_tx_dropped");
        }
        if self.bus_error() {
            names.push("bus_error");
        }
        names.join(",")
    }
}

impl std::fmt::Display for McuSerialStatus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "state={} flags={} TEC={} REC={} tx_failed={} rx_dropped={}",
            self.state_name(),
            self.flag_names(),
            self.tx_error_count,
            self.rx_error_count,
            self.tx_failed,
            self.rx_dropped,
        )
    }
}

/// Decode the 8-byte STATUS payload. Caller ensures `data.len() == 8`.
fn decode_status(data: &[u8]) -> McuSerialStatus {
    McuSerialStatus {
        state: data[0],
        flags: data[1],
        tx_error_count: data[2],
        rx_error_count: data[3],
        tx_failed: u16::from_le_bytes([data[4], data[5]]),
        rx_dropped: u16::from_le_bytes([data[6], data[7]]),
    }
}

fn crc8(data: &[u8]) -> u8 {
    let mut crc: u8 = 0x00;
    for &b in data {
        crc ^= b;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 {
                (crc << 1) ^ 0x07
            } else {
                crc << 1
            };
        }
    }
    crc
}

struct Inner {
    port: Box<dyn SerialPort>,
    rx_buf: VecDeque<u8>,
}

pub struct McuSerialBus {
    inner: Mutex<Inner>,
}

impl McuSerialBus {
    pub fn open(port: &str, baud: u32) -> Result<Self> {
        let port_obj = serialport::new(port, baud)
            .timeout(Duration::from_millis(10))
            .data_bits(DataBits::Eight)
            .stop_bits(StopBits::One)
            .parity(Parity::None)
            .flow_control(FlowControl::None)
            .open()
            .map_err(|e| MotorError::Io(format!("open mcu-serial port {port} failed: {e}")))?;
        Ok(Self {
            inner: Mutex::new(Inner {
                port: port_obj,
                rx_buf: VecDeque::with_capacity(1024),
            }),
        })
    }

    fn encode_tx(frame: CanFrame) -> Result<Vec<u8>> {
        let dlc = frame.dlc as usize;
        if dlc > MAX_DLC {
            return Err(MotorError::InvalidArgument(format!(
                "invalid DLC {}, mcu-serial is classic CAN (<=8 bytes)",
                frame.dlc
            )));
        }
        let can_id = frame.arbitration_id & 0x1FFF_FFFF;
        let wire_id = if frame.is_extended {
            can_id | 0x8000_0000
        } else {
            can_id
        };
        let mut out = Vec::with_capacity(8 + dlc);
        out.push(HEADER);
        out.push(dlc as u8);
        out.extend_from_slice(&wire_id.to_le_bytes());
        out.extend_from_slice(&frame.data[..dlc]);
        let crc = crc8(&out[1..]);
        out.push(crc);
        out.push(TAIL);
        Ok(out)
    }

    fn try_parse_rx(buf: &mut VecDeque<u8>) -> Option<CanFrame> {
        loop {
            while let Some(&first) = buf.front() {
                if first == HEADER {
                    break;
                }
                let _ = buf.pop_front();
            }
            if buf.len() < 2 {
                return None;
            }
            let len = match buf.get(1) {
                Some(&b) => b as usize,
                None => return None,
            };
            if len > MAX_DLC {
                let _ = buf.pop_front();
                continue;
            }
            let total = 8 + len;
            if buf.len() < total {
                return None;
            }
            let mut raw = [0u8; MAX_FRAME];
            for (i, b) in buf.iter().take(total).enumerate() {
                raw[i] = *b;
            }
            if raw[total - 1] != TAIL || crc8(&raw[1..6 + len]) != raw[6 + len] {
                let _ = buf.pop_front();
                continue;
            }
            for _ in 0..total {
                let _ = buf.pop_front();
            }
            let wire_id = u32::from_le_bytes([raw[2], raw[3], raw[4], raw[5]]);
            let mut data = [0u8; 8];
            data[..len].copy_from_slice(&raw[6..6 + len]);
            return Some(CanFrame {
                arbitration_id: wire_id & 0x1FFF_FFFF,
                is_extended: wire_id & 0x8000_0000 != 0,
                data,
                dlc: len as u8,
                is_rx: true,
            });
        }
    }

    fn read_available(inner: &mut Inner, wait_for_data: bool) -> Result<bool> {
        if !wait_for_data {
            match inner.port.bytes_to_read() {
                Ok(0) => return Ok(false),
                Ok(_) => {}
                Err(_) => {}
            }
        }
        let mut tmp = [0u8; 256];
        match inner.port.read(&mut tmp) {
            Ok(n) if n > 0 => {
                inner.rx_buf.extend(tmp[..n].iter().copied());
                Ok(true)
            }
            Ok(_) => Ok(false),
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => Ok(false),
            Err(e) => Err(MotorError::Io(format!("mcu-serial read failed: {e}"))),
        }
    }
}

impl CanBus for McuSerialBus {
    fn send(&self, frame: CanFrame) -> Result<()> {
        let raw = Self::encode_tx(frame)?;
        let mut inner = self
            .inner
            .lock()
            .map_err(|_| MotorError::Io("mcu-serial lock poisoned".to_string()))?;
        inner
            .port
            .write_all(&raw)
            .map_err(|e| MotorError::Io(format!("mcu-serial write failed: {e}")))?;
        Ok(())
    }

    fn recv(&self, timeout: Duration) -> Result<Option<CanFrame>> {
        let wait_for_data = !timeout.is_zero();
        let deadline = Instant::now()
            .checked_add(timeout)
            .unwrap_or_else(|| Instant::now() + Duration::from_secs(3600));
        let mut inner = self
            .inner
            .lock()
            .map_err(|_| MotorError::Io("mcu-serial lock poisoned".to_string()))?;

        loop {
            if let Some(frame) = Self::try_parse_rx(&mut inner.rx_buf) {
                // MCU-pushed link-status frame. Only intercept the full
                // 8-byte payload; a short/malformed STATUS frame falls through
                // as an ordinary data frame (no vendor matches this id, so it
                // is harmlessly ignored downstream). A hard fault (BUS_OFF /
                // ERROR_PASSIVE) surfaces as a structured BusStatus error so a
                // CAN-layer fault is distinguishable from a plain recv
                // timeout; a benign anomaly (rx_dropped while ERROR_ACTIVE,
                // etc.) is drained and recv keeps looking for a data frame.
                if frame.is_extended && frame.arbitration_id == STATUS_ID && frame.dlc == 8 {
                    let status = decode_status(&frame.data);
                    if status.is_hard_fault() {
                        return Err(MotorError::BusStatus(format!(
                            "mcu-serial link status: {status}"
                        )));
                    }
                    // Benign anomaly (e.g. rx_dropped while ERROR_ACTIVE):
                    // drain this STATUS frame and keep looking for a real
                    // data frame. See `McuSerialStatus::is_hard_fault`.
                    continue;
                }
                return Ok(Some(frame));
            }
            let read_any = Self::read_available(&mut inner, wait_for_data)?;
            if !read_any && !wait_for_data {
                return Ok(None);
            }
            if Instant::now() >= deadline {
                return Ok(None);
            }
        }
    }

    fn shutdown(&self) -> Result<()> {
        let mut inner = self
            .inner
            .lock()
            .map_err(|_| MotorError::Io("mcu-serial lock poisoned".to_string()))?;
        inner
            .port
            .flush()
            .map_err(|e| MotorError::Io(format!("mcu-serial flush failed: {e}")))?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(arb: u32, data: &[u8], ext: bool) -> CanFrame {
        let mut d = [0u8; 8];
        let n = data.len().min(8);
        d[..n].copy_from_slice(&data[..n]);
        CanFrame {
            arbitration_id: arb,
            data: d,
            dlc: n as u8,
            is_extended: ext,
            is_rx: false,
        }
    }

    #[test]
    fn roundtrip_classic_std_8_bytes() {
        let f = frame(
            0x123,
            &[0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08],
            false,
        );
        let raw = McuSerialBus::encode_tx(f).unwrap();
        assert_eq!(raw[0], HEADER);
        assert_eq!(raw[1], 8);
        assert_eq!(*raw.last().unwrap(), TAIL);
        let mut buf = VecDeque::new();
        buf.extend(raw);
        let out = McuSerialBus::try_parse_rx(&mut buf).unwrap();
        assert_eq!(out.arbitration_id, 0x123);
        assert!(!out.is_extended);
        assert_eq!(out.dlc, 8);
        assert_eq!(out.data, [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08]);
        assert!(buf.is_empty());
    }

    #[test]
    fn roundtrip_extended_29bit_variable_dlc() {
        for dlc in 0..=8 {
            let data: Vec<u8> = (0..dlc)
                .map(|i| (i as u8).wrapping_mul(0x1F) ^ 0xA5)
                .collect();
            let raw = McuSerialBus::encode_tx(frame(0x1ABCDE0F, &data, true)).unwrap();
            assert_eq!(raw.len(), 8 + dlc);
            let mut buf = VecDeque::new();
            buf.extend(raw);
            let out = McuSerialBus::try_parse_rx(&mut buf).unwrap();
            assert_eq!(out.arbitration_id, 0x1ABCDE0F);
            assert!(out.is_extended);
            assert_eq!(out.dlc as usize, dlc);
            assert_eq!(&out.data[..dlc], &data[..]);
            assert!(buf.is_empty());
        }
    }

    #[test]
    fn crc_covers_len_id_data() {
        let raw = McuSerialBus::encode_tx(frame(0x7F, &[0xDE, 0xAD, 0xBE, 0xEF], false)).unwrap();
        let dlc = 4;
        assert_eq!(raw[6 + dlc], crc8(&raw[1..6 + dlc]));
    }

    #[test]
    fn resync_on_garbage_prefix() {
        let raw = McuSerialBus::encode_tx(frame(0x55, &[0x01, 0x02], false)).unwrap();
        let mut buf = VecDeque::new();
        buf.extend([0x00, 0xFF, 0x7F, 0x33]);
        buf.extend(raw);
        let out = McuSerialBus::try_parse_rx(&mut buf).unwrap();
        assert_eq!(out.arbitration_id, 0x55);
        assert_eq!(out.dlc, 2);
        assert!(buf.is_empty());
    }

    #[test]
    fn corrupted_crc_resyncs_to_next_frame() {
        let mut bad = McuSerialBus::encode_tx(frame(0x10, &[0x01, 0x02, 0x03], false)).unwrap();
        bad[6] ^= 0x01;
        let good = McuSerialBus::encode_tx(frame(0x10, &[0x09, 0x08, 0x07], false)).unwrap();
        let mut buf = VecDeque::new();
        buf.extend(bad);
        buf.extend(good);
        let out = McuSerialBus::try_parse_rx(&mut buf).unwrap();
        assert_eq!(&out.data[..3], &[0x09, 0x08, 0x07]);
    }

    #[test]
    fn corrupted_tail_rejected() {
        let mut raw = McuSerialBus::encode_tx(frame(0x20, &[0xAA], false)).unwrap();
        *raw.last_mut().unwrap() = 0x00;
        let mut buf = VecDeque::new();
        buf.extend(&raw);
        assert!(McuSerialBus::try_parse_rx(&mut buf).is_none());
    }

    #[test]
    fn stray_delimiter_in_data_does_not_desync() {
        let raw = McuSerialBus::encode_tx(frame(0x99, &[0xA5, 0x5A, 0xA5], false)).unwrap();
        let mut buf = VecDeque::new();
        buf.extend(raw);
        let out = McuSerialBus::try_parse_rx(&mut buf).unwrap();
        assert_eq!(&out.data[..3], &[0xA5, 0x5A, 0xA5]);
    }

    #[test]
    fn oversize_dlc_rejected() {
        let f = CanFrame {
            arbitration_id: 0x01,
            data: [0u8; 8],
            dlc: 9,
            is_extended: false,
            is_rx: false,
        };
        assert!(McuSerialBus::encode_tx(f).is_err());
    }

    /// The MCU-pushed STATUS frame must round-trip through the data-frame
    /// codec so `recv` can see `arbitration_id == STATUS_ID` and intercept it.
    /// This locks the wire contract between the firmware `push_status_frame`
    /// and the host decode path.
    #[test]
    fn status_frame_roundtrips_as_extended_data() {
        // state=BUS_OFF(4), flags=0x07 (bus_off|tx_failed|rx_dropped),
        // TEC=208, REC=192, tx_failed=300 (>255, exercises u16 path),
        // rx_dropped=7.
        let payload = [0x04u8, 0x07, 0xD0, 0xC0, 0x2C, 0x01, 0x07, 0x00];
        let f = CanFrame {
            arbitration_id: STATUS_ID,
            data: payload,
            dlc: 8,
            is_extended: true,
            is_rx: true,
        };
        let raw = McuSerialBus::encode_tx(f).unwrap();
        assert_eq!(raw[0], HEADER);
        assert_eq!(raw[1], 8);
        assert_eq!(*raw.last().unwrap(), TAIL);
        let mut buf = VecDeque::new();
        buf.extend(raw);
        let out = McuSerialBus::try_parse_rx(&mut buf).unwrap();
        assert!(out.is_extended);
        assert_eq!(out.arbitration_id, STATUS_ID);
        assert_eq!(out.dlc, 8);
        assert_eq!(out.data, payload);
        assert!(buf.is_empty());
    }

    #[test]
    fn decode_status_maps_payload_fields() {
        // flags=0x7F sets every fault bit (bus_off|tx_failed|rx_dropped|
        // crc_bad|oversize|app_tx_dropped|bus_error); bit7 reserved stays 0.
        let payload = [0x04u8, 0x7F, 0xD0, 0xC0, 0x2C, 0x01, 0x07, 0x00];
        let s = decode_status(&payload);
        assert_eq!(s.state, 4);
        assert_eq!(s.state_name(), "BUS_OFF");
        assert_eq!(s.flags, 0x7F);
        assert!(s.bus_off());
        assert!(s.tx_failed());
        assert!(s.rx_dropped());
        assert!(s.crc_bad());
        assert!(s.oversize());
        assert!(s.app_tx_dropped());
        assert!(s.bus_error());
        assert_eq!(s.tx_error_count, 0xD0);
        assert_eq!(s.rx_error_count, 0xC0);
        assert_eq!(s.tx_failed, 0x012C); // 300
        assert_eq!(s.rx_dropped, 0x0007);
        let msg = format!("{s}");
        assert!(msg.contains("state=BUS_OFF"));
        assert!(msg.contains("TEC=208"));
        assert!(msg.contains("tx_failed=300"));
    }

    #[test]
    fn decode_status_healthy_state_name() {
        let s = decode_status(&[1u8, 0x00, 0, 0, 0, 0, 0, 0]);
        assert_eq!(s.state_name(), "ERROR_ACTIVE");
        assert!(!s.bus_off());
        assert_eq!(s.flag_names(), "");
    }

    /// `is_hard_fault` is the recv drain/abort hinge: only ERROR_PASSIVE /
    /// BUS_OFF abort; everything else (including the user's `rx_dropped=1`
    /// while ERROR_ACTIVE) drains through so scan/read/ping survive a sticky
    /// counter without a power cycle.
    #[test]
    fn is_hard_fault_classifies_by_controller_state() {
        // ERROR_ACTIVE + rx_dropped=1 — the exact case that used to abort scan:
        // benign, must drain.
        let benign = decode_status(&[0x01u8, status_flag::RX_DROPPED, 0, 0, 0, 0, 0x01, 0x00]);
        assert_eq!(benign.state_name(), "ERROR_ACTIVE");
        assert!(benign.rx_dropped());
        assert!(!benign.is_hard_fault());

        // ERROR_WARNING — still communicating, benign.
        let warn = decode_status(&[0x02u8, 0x00, 0, 0, 0, 0, 0, 0]);
        assert!(!warn.is_hard_fault());

        // ERROR_ACTIVE with every soft fault flag set (incl. BUS_ERROR, which
        // is transient while still ERROR_ACTIVE) — benign: drain & continue.
        let soft_all = decode_status(&[0x01u8, 0x7E, 0, 0, 0, 0, 0, 0]);
        assert!(soft_all.bus_error());
        assert!(soft_all.crc_bad());
        assert!(soft_all.oversize());
        assert!(soft_all.rx_dropped());
        assert!(!soft_all.is_hard_fault());

        // ERROR_PASSIVE — hard (controller can't TX reliably).
        let passive = decode_status(&[0x03u8, 0x00, 0, 0, 0, 0, 0, 0]);
        assert!(passive.is_hard_fault());

        // BUS_OFF — hard, both via state and the flag.
        let busoff = decode_status(&[0x04u8, status_flag::BUS_OFF, 0, 0, 0, 0, 0, 0]);
        assert!(busoff.bus_off());
        assert!(busoff.is_hard_fault());
    }
}
