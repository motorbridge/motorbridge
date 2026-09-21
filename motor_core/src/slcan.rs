use crate::bus::{CanBus, CanFrame};
use crate::error::{MotorError, Result};
use serialport::{DataBits, FlowControl, Parity, SerialPort, StopBits};
use std::collections::VecDeque;
use std::io::{Read, Write};
use std::sync::Mutex;
use std::time::{Duration, Instant};

/// Default host baud. Most slcan adapters enumerate as USB CDC-ACM, where the
/// line rate is ignored, but a value is still required to open the port.
pub const DEFAULT_SERIAL_BAUD: u32 = 115_200;
pub const DEFAULT_BITRATE: u32 = 1_000_000;

const MAX_LINE_LEN: usize = 32;

struct Inner {
    port: Box<dyn SerialPort>,
    rx_buf: VecDeque<u8>,
}

/// CAN bus over an slcan (LAWICEL ASCII) serial adapter, such as CANable,
/// CANtact or USBtin.
///
/// Frames are `T<id:8><dlc:1><data>\r` for extended ids and `t<id:3><dlc:1><data>\r`
/// for standard ids, with data as uppercase hex.
pub struct SlcanBus {
    inner: Mutex<Inner>,
}

impl SlcanBus {
    pub fn open(port: &str, baud: u32, bitrate: u32) -> Result<Self> {
        let code = Self::bitrate_code(bitrate)?;
        let mut port_obj = serialport::new(port, baud)
            .timeout(Duration::from_millis(10))
            .data_bits(DataBits::Eight)
            .stop_bits(StopBits::One)
            .parity(Parity::None)
            .flow_control(FlowControl::None)
            .open()
            .map_err(|e| MotorError::Io(format!("open serial port {port} failed: {e}")))?;

        // Close first: the bitrate is only accepted while the channel is closed,
        // and the adapter may still be open from a previous session.
        for cmd in [b"C\r".to_vec(), vec![b'S', code, b'\r'], b"O\r".to_vec()] {
            port_obj
                .write_all(&cmd)
                .map_err(|e| MotorError::Io(format!("slcan init write failed: {e}")))?;
            port_obj
                .flush()
                .map_err(|e| MotorError::Io(format!("slcan init flush failed: {e}")))?;
            std::thread::sleep(Duration::from_millis(20));
        }
        // Drop any banner or error bytes the bring-up produced.
        let mut sink = [0u8; 256];
        let _ = port_obj.read(&mut sink);

        Ok(Self {
            inner: Mutex::new(Inner {
                port: port_obj,
                rx_buf: VecDeque::with_capacity(1024),
            }),
        })
    }

    /// Map a bitrate in bit/s to the slcan `S` digit.
    fn bitrate_code(bitrate: u32) -> Result<u8> {
        let digit = match bitrate {
            10_000 => b'0',
            20_000 => b'1',
            50_000 => b'2',
            100_000 => b'3',
            125_000 => b'4',
            250_000 => b'5',
            500_000 => b'6',
            800_000 => b'7',
            1_000_000 => b'8',
            other => {
                return Err(MotorError::InvalidArgument(format!(
                    "unsupported slcan bitrate {other}, expected one of \
                     10000, 20000, 50000, 100000, 125000, 250000, 500000, 800000, 1000000"
                )))
            }
        };
        Ok(digit)
    }

    fn encode_tx(frame: CanFrame) -> Result<Vec<u8>> {
        if frame.dlc > 8 {
            return Err(MotorError::InvalidArgument(format!(
                "invalid DLC {}, expected <= 8",
                frame.dlc
            )));
        }
        if !frame.is_extended && frame.arbitration_id > 0x7FF {
            return Err(MotorError::InvalidArgument(format!(
                "invalid arbitration_id {:X}, expected 11-bit std id",
                frame.arbitration_id
            )));
        }
        if frame.is_extended && frame.arbitration_id > 0x1FFF_FFFF {
            return Err(MotorError::InvalidArgument(format!(
                "invalid arbitration_id {:X}, expected 29-bit ext id",
                frame.arbitration_id
            )));
        }

        let dlc = frame.dlc as usize;
        let mut out = Vec::with_capacity(MAX_LINE_LEN);
        if frame.is_extended {
            out.push(b'T');
            out.extend_from_slice(format!("{:08X}", frame.arbitration_id).as_bytes());
        } else {
            out.push(b't');
            out.extend_from_slice(format!("{:03X}", frame.arbitration_id).as_bytes());
        }
        out.push(b'0' + frame.dlc);
        for byte in &frame.data[..dlc] {
            out.extend_from_slice(format!("{byte:02X}").as_bytes());
        }
        out.push(b'\r');
        Ok(out)
    }

    fn hex_val(byte: u8) -> Option<u32> {
        match byte {
            b'0'..=b'9' => Some(u32::from(byte - b'0')),
            b'a'..=b'f' => Some(u32::from(byte - b'a' + 10)),
            b'A'..=b'F' => Some(u32::from(byte - b'A' + 10)),
            _ => None,
        }
    }

    fn hex_num(bytes: &[u8]) -> Option<u32> {
        let mut acc = 0u32;
        for &b in bytes {
            acc = acc.checked_mul(16)?.checked_add(Self::hex_val(b)?)?;
        }
        Some(acc)
    }

    fn parse_line(line: &[u8]) -> Option<CanFrame> {
        // Only data frames carry a payload; RTR ('r'/'R') and status replies
        // ('z', 'Z', 'v', BEL, ...) are not surfaced as frames.
        let (is_extended, id_len) = match line.first()? {
            b'T' => (true, 8),
            b't' => (false, 3),
            _ => return None,
        };
        if line.len() < 1 + id_len + 1 {
            return None;
        }
        let arbitration_id = Self::hex_num(&line[1..1 + id_len])?;
        let dlc = Self::hex_val(line[1 + id_len])? as u8;
        if dlc > 8 {
            return None;
        }
        let payload = &line[2 + id_len..];
        if payload.len() < usize::from(dlc) * 2 {
            return None;
        }
        let mut data = [0u8; 8];
        for (i, slot) in data.iter_mut().take(usize::from(dlc)).enumerate() {
            *slot = Self::hex_num(&payload[i * 2..i * 2 + 2])? as u8;
        }
        Some(CanFrame {
            arbitration_id,
            data,
            dlc,
            is_extended,
            is_rx: true,
        })
    }

    fn try_parse_rx(buf: &mut VecDeque<u8>) -> Option<CanFrame> {
        while let Some(pos) = buf.iter().position(|&b| b == b'\r') {
            let line: Vec<u8> = buf.drain(..=pos).take(pos).collect();
            if let Some(frame) = Self::parse_line(&line) {
                return Some(frame);
            }
            // Not a data frame: keep scanning the remaining buffered lines.
        }
        // Guard against an adapter that never emits CR.
        if buf.len() > MAX_LINE_LEN * 16 {
            buf.clear();
        }
        None
    }

    fn read_available(inner: &mut Inner, wait_for_data: bool) -> Result<bool> {
        if !wait_for_data {
            match inner.port.bytes_to_read() {
                Ok(0) => return Ok(false),
                Ok(_) => {}
                Err(_) => {
                    // Some serial backends may not support pending-byte queries.
                    // Fall back to the normal timed read path for compatibility.
                }
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
            Err(e) => Err(MotorError::Io(format!("slcan read failed: {e}"))),
        }
    }
}

impl CanBus for SlcanBus {
    fn send(&self, frame: CanFrame) -> Result<()> {
        let raw = Self::encode_tx(frame)?;
        let mut inner = self
            .inner
            .lock()
            .map_err(|_| MotorError::Io("slcan lock poisoned".to_string()))?;
        inner
            .port
            .write_all(&raw)
            .map_err(|e| MotorError::Io(format!("slcan write failed: {e}")))?;
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
            .map_err(|_| MotorError::Io("slcan lock poisoned".to_string()))?;

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
            .map_err(|_| MotorError::Io("slcan lock poisoned".to_string()))?;
        // Best effort: take the adapter off the bus so the next open succeeds.
        let _ = inner.port.write_all(b"C\r");
        inner
            .port
            .flush()
            .map_err(|e| MotorError::Io(format!("slcan flush failed: {e}")))?;
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
    fn encodes_extended_frame_as_uppercase_ascii() {
        let raw = SlcanBus::encode_tx(frame(0x1234_5678, &[0xDE, 0xAD], true)).unwrap();
        assert_eq!(raw, b"T123456782DEAD\r".to_vec());
    }

    #[test]
    fn encodes_standard_frame_with_three_hex_digits() {
        let raw = SlcanBus::encode_tx(frame(0x123, &[0x01], false)).unwrap();
        assert_eq!(raw, b"t123101\r".to_vec());
    }

    #[test]
    fn roundtrip_extended_variable_dlc() {
        for dlc in 0..=8usize {
            let data: Vec<u8> = (0..dlc)
                .map(|i| (i as u8).wrapping_mul(0x1F) ^ 0xA5)
                .collect();
            let raw = SlcanBus::encode_tx(frame(0x1ABC_DE0F, &data, true)).unwrap();
            let mut buf = VecDeque::new();
            buf.extend(raw);
            let out = SlcanBus::try_parse_rx(&mut buf).unwrap();
            assert_eq!(out.arbitration_id, 0x1ABC_DE0F);
            assert!(out.is_extended);
            assert_eq!(out.dlc, dlc as u8);
            assert_eq!(&out.data[..dlc], &data[..]);
            assert!(out.is_rx);
            assert!(buf.is_empty());
        }
    }

    #[test]
    fn roundtrip_standard_id() {
        let raw = SlcanBus::encode_tx(frame(0x7FF, &[0xAA, 0xBB, 0xCC], false)).unwrap();
        let mut buf = VecDeque::new();
        buf.extend(raw);
        let out = SlcanBus::try_parse_rx(&mut buf).unwrap();
        assert_eq!(out.arbitration_id, 0x7FF);
        assert!(!out.is_extended);
        assert_eq!(out.dlc, 3);
        assert_eq!(&out.data[..3], &[0xAA, 0xBB, 0xCC]);
    }

    #[test]
    fn skips_status_replies_and_returns_following_frame() {
        let mut buf = VecDeque::new();
        // Adapter ack, a bell (error), then a real frame.
        buf.extend(b"z\r".to_vec());
        buf.extend(vec![0x07]);
        buf.extend(b"\r".to_vec());
        buf.extend(b"V0120\r".to_vec());
        buf.extend(SlcanBus::encode_tx(frame(0x18, &[0x11], true)).unwrap());
        let out = SlcanBus::try_parse_rx(&mut buf).unwrap();
        assert_eq!(out.arbitration_id, 0x18);
        assert_eq!(out.dlc, 1);
    }

    #[test]
    fn ignores_rtr_frames() {
        let mut buf = VecDeque::new();
        buf.extend(b"r1234\r".to_vec());
        buf.extend(b"R123456780\r".to_vec());
        assert!(SlcanBus::try_parse_rx(&mut buf).is_none());
    }

    #[test]
    fn rejects_oversized_dlc_and_out_of_range_ids() {
        let mut f = frame(0x1, &[], true);
        f.dlc = 9;
        assert!(SlcanBus::encode_tx(f).is_err());
        assert!(SlcanBus::encode_tx(frame(0x800, &[], false)).is_err());
        assert!(SlcanBus::encode_tx(frame(0x2000_0000, &[], true)).is_err());
    }

    #[test]
    fn maps_supported_bitrates_and_rejects_others() {
        assert_eq!(SlcanBus::bitrate_code(1_000_000).unwrap(), b'8');
        assert_eq!(SlcanBus::bitrate_code(500_000).unwrap(), b'6');
        assert_eq!(SlcanBus::bitrate_code(125_000).unwrap(), b'4');
        assert!(SlcanBus::bitrate_code(250_001).is_err());
    }

    #[test]
    fn partial_line_is_buffered_until_terminator_arrives() {
        let mut buf = VecDeque::new();
        buf.extend(b"T1ABCDE0F2DE".to_vec());
        assert!(SlcanBus::try_parse_rx(&mut buf).is_none());
        buf.extend(b"AD\r".to_vec());
        let out = SlcanBus::try_parse_rx(&mut buf).unwrap();
        assert_eq!(out.arbitration_id, 0x1ABC_DE0F);
        assert_eq!(&out.data[..2], &[0xDE, 0xAD]);
    }
}
