//! SDO expedited client encoding and reply matching.
//!
//! Wire format (little-endian, fixed 8-byte data field, zero-padded):
//!
//! | request | reply |
//! |---|---|
//! | `0x40 idx lo idx hi sub 00 00 00 00` | `0x43/0x4B/0x4F idx lo idx hi sub <data>` |
//! | `0x2F/0x2B/0x23 idx lo idx hi sub <data>` | `0x60 idx lo idx hi 00 00 00 00` |
//! | abort (either direction) | `0x80 idx lo idx hi sub <abort code>` |

use std::collections::VecDeque;
use std::sync::Mutex;
use std::time::{Duration, Instant};

use motor_core::error::{MotorError, Result};

/// A typed value for expedited SDO read/write.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SdoValue {
    U8(u8),
    I8(i8),
    U16(u16),
    I16(i16),
    U32(u32),
    I32(i32),
    F32(f32),
}

impl SdoValue {
    /// Payload byte count used to pick the download command specifier.
    pub fn nbytes(self) -> usize {
        match self {
            Self::U8(_) | Self::I8(_) => 1,
            Self::U16(_) | Self::I16(_) => 2,
            Self::U32(_) | Self::I32(_) | Self::F32(_) => 4,
        }
    }

    fn payload4(self) -> [u8; 4] {
        match self {
            Self::U8(v) => [v, 0, 0, 0],
            Self::I8(v) => [v as u8, 0, 0, 0],
            Self::U16(v) => {
                let b = v.to_le_bytes();
                [b[0], b[1], 0, 0]
            }
            Self::I16(v) => {
                let b = v.to_le_bytes();
                [b[0], b[1], 0, 0]
            }
            Self::U32(v) => v.to_le_bytes(),
            Self::I32(v) => v.to_le_bytes(),
            Self::F32(v) => v.to_bits().to_le_bytes(),
        }
    }
}

/// Build an expedited SDO upload (read) request frame payload.
pub fn build_sdo_upload(index: u16, subindex: u8) -> [u8; 8] {
    [
        0x40,
        (index & 0xFF) as u8,
        ((index >> 8) & 0xFF) as u8,
        subindex,
        0,
        0,
        0,
        0,
    ]
}

/// Build an expedited SDO download (write) request frame payload.
pub fn build_sdo_download(index: u16, subindex: u8, value: SdoValue) -> [u8; 8] {
    let cmd = match value.nbytes() {
        1 => 0x2F,
        2 => 0x2B,
        _ => 0x23,
    };
    let p = value.payload4();
    [
        cmd,
        (index & 0xFF) as u8,
        ((index >> 8) & 0xFF) as u8,
        subindex,
        p[0],
        p[1],
        p[2],
        p[3],
    ]
}

/// Extract `(index, subindex)` from an SDO reply frame.
pub fn reply_target(data: &[u8; 8]) -> (u16, u8) {
    (
        u16::from(data[1]) | (u16::from(data[2]) << 8),
        data[3],
    )
}

/// Classification of an SDO reply frame matched by `(index, subindex)`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SdoReplyKind {
    /// Upload response carrying `n` data bytes (`0x43/0x4B/0x4F`).
    UploadData(u8),
    /// Download confirmation (`0x60`).
    WriteAck,
    /// Abort with the carried abort code (`0x80`).
    Abort(u32),
    /// Anything else — protocol violation.
    Unknown(u8),
}

pub fn classify_sdo_reply(data: &[u8; 8]) -> SdoReplyKind {
    match data[0] {
        0x60 => SdoReplyKind::WriteAck,
        0x80 => SdoReplyKind::Abort(u32::from_le_bytes([
            data[4], data[5], data[6], data[7],
        ])),
        cmd @ (0x43 | 0x4B | 0x4F | 0x47) => SdoReplyKind::UploadData(match cmd {
            0x4F => 1,
            0x4B => 2,
            _ => 4,
        }),
        cmd => SdoReplyKind::Unknown(cmd),
    }
}

/// Thread-safe queue of received SDO reply frames (fed by the vendor's
/// `MotorDevice::process_feedback_frame` via [`crate::node::CanopenNode`]).
pub struct SdoQueue {
    replies: Mutex<VecDeque<[u8; 8]>>,
}

impl SdoQueue {
    pub fn new() -> Self {
        Self {
            replies: Mutex::new(VecDeque::new()),
        }
    }

    /// Queue an SDO reply frame (only frames with `dlc >= 8`).
    pub fn push(&self, data: [u8; 8]) {
        if let Ok(mut q) = self.replies.lock() {
            q.push_back(data);
        }
    }

    /// Pop the first queued reply matching `(index, subindex)`, re-queuing
    /// non-matching frames in order. Blocks up to `timeout`.
    pub fn pop_matching(&self, index: u16, subindex: u8, timeout: Duration) -> Result<[u8; 8]> {
        let deadline = Instant::now() + timeout;
        loop {
            let mut pending = self
                .replies
                .lock()
                .map_err(|_| MotorError::Io("sdo queue lock poisoned".to_string()))?;
            let mut hold = VecDeque::new();
            let mut found = None;
            while let Some(msg) = pending.pop_front() {
                let (idx, sub) = reply_target(&msg);
                if idx == index && sub == subindex {
                    found = Some(msg);
                    break;
                }
                hold.push_back(msg);
            }
            while let Some(m) = hold.pop_front() {
                pending.push_back(m);
            }
            drop(pending);
            if let Some(msg) = found {
                return Ok(msg);
            }
            if Instant::now() >= deadline {
                return Err(MotorError::Timeout(format!(
                    "sdo response timeout idx=0x{index:04X} sub=0x{subindex:02X}"
                )));
            }
            std::thread::sleep(Duration::from_millis(1));
        }
    }
}

impl Default for SdoQueue {
    fn default() -> Self {
        Self::new()
    }
}

/// Interpret a matched reply as raw data bytes (upload responses only).
pub fn upload_payload(reply: &[u8; 8]) -> Result<[u8; 4]> {
    match classify_sdo_reply(reply) {
        SdoReplyKind::UploadData(_) => Ok([reply[4], reply[5], reply[6], reply[7]]),
        SdoReplyKind::WriteAck => Err(MotorError::Protocol(
            "unexpected sdo write ack for read request".to_string(),
        )),
        SdoReplyKind::Abort(code) => Err(MotorError::Protocol(format!(
            "sdo abort idx=0x{:04X} sub=0x{:02X} code=0x{code:08X}",
            reply_target(reply).0,
            reply_target(reply).1
        ))),
        SdoReplyKind::Unknown(cmd) => Err(MotorError::Protocol(format!(
            "unexpected sdo read cmd=0x{cmd:02X} idx=0x{:04X} sub=0x{:02X}",
            reply_target(reply).0,
            reply_target(reply).1
        ))),
    }
}

/// Verify a matched reply is a clean write acknowledgement.
pub fn expect_write_ack(reply: &[u8; 8]) -> Result<()> {
    let (index, subindex) = reply_target(reply);
    match classify_sdo_reply(reply) {
        SdoReplyKind::WriteAck => Ok(()),
        SdoReplyKind::Abort(abort) => Err(MotorError::Protocol(format!(
            "sdo abort idx=0x{index:04X} sub=0x{subindex:02X} code=0x{abort:08X}"
        ))),
        SdoReplyKind::UploadData(_) => Err(MotorError::Protocol(format!(
            "unexpected sdo read reply for write request idx=0x{index:04X} sub=0x{subindex:02X}"
        ))),
        SdoReplyKind::Unknown(cmd) => Err(MotorError::Protocol(format!(
            "unexpected sdo write ack cmd=0x{cmd:02X} idx=0x{index:04X} sub=0x{subindex:02X}"
        ))),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn build_upload_request_bytes() {
        assert_eq!(
            build_sdo_upload(0x60FF, 0x00),
            [0x40, 0xFF, 0x60, 0x00, 0, 0, 0, 0]
        );
    }

    #[test]
    fn build_download_picks_command_by_size() {
        assert_eq!(build_sdo_download(0x6040, 0, SdoValue::I8(7))[0], 0x2F);
        assert_eq!(build_sdo_download(0x6040, 0, SdoValue::U16(7))[0], 0x2B);
        assert_eq!(build_sdo_download(0x60FF, 0, SdoValue::I32(600))[0], 0x23);
        assert_eq!(build_sdo_download(0x6064, 0, SdoValue::F32(0.5))[0], 0x23);
    }

    #[test]
    fn f32_round_trips_little_endian() {
        let frame = build_sdo_download(0x607A, 0x00, SdoValue::F32(0.25));
        assert_eq!(frame[0], 0x23);
        let bits = u32::from_le_bytes([frame[4], frame[5], frame[6], frame[7]]);
        assert_eq!(f32::from_bits(bits), 0.25);
    }

    #[test]
    fn classifies_upload_ack_and_abort() {
        let mut reply = [0u8; 8];
        reply[0] = 0x60;
        reply[1] = 0x40;
        reply[2] = 0x60;
        assert_eq!(classify_sdo_reply(&reply), SdoReplyKind::WriteAck);

        reply[0] = 0x43;
        assert_eq!(classify_sdo_reply(&reply), SdoReplyKind::UploadData(4));

        reply[0] = 0x4B;
        assert_eq!(classify_sdo_reply(&reply), SdoReplyKind::UploadData(2));

        reply[0] = 0x4F;
        assert_eq!(classify_sdo_reply(&reply), SdoReplyKind::UploadData(1));

        reply[0] = 0x80;
        reply[4] = 0x11;
        reply[5] = 0x06;
        reply[6] = 0x00;
        reply[7] = 0x00;
        assert_eq!(classify_sdo_reply(&reply), SdoReplyKind::Abort(0x0611));
    }

    #[test]
    fn pop_matching_skips_unrelated_and_times_out() {
        let queue = SdoQueue::new();
        let mut other = [0u8; 8];
        other[1] = 0x41;
        other[2] = 0x60;
        queue.push(other);
        let mut wanted = [0u8; 8];
        wanted[1] = 0x40;
        wanted[2] = 0x60;
        wanted[4] = 0x0F;
        queue.push(wanted);
        let got = queue
            .pop_matching(0x6040, 0x00, Duration::from_millis(10))
            .expect("match");
        assert_eq!(got[4], 0x0F);
        // non-matching frame stays queued (guard dropped before the next call)
        {
            let left = queue.replies.lock().expect("queue");
            assert_eq!(left.len(), 1);
            assert_eq!(left[0][1], 0x41);
        }

        let err = queue
            .pop_matching(0x6060, 0x00, Duration::from_millis(5))
            .expect_err("timeout");
        assert!(err.to_string().contains("timeout"));
    }
}
