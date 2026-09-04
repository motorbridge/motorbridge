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
}
