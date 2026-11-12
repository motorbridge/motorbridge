use motor_core::error::{MotorError, Result};

pub(crate) fn readable(address: u8) -> bool {
    matches!(address, 0x01..=0x04 | 0x10..=0x18 | 0x28 | 0x50..=0x59 | 0x80..=0x8C | 0x8E..=0x93 | 0xC8..=0xD5)
}

pub(crate) fn writable(address: u8) -> bool {
    matches!(address, 0x28 | 0x50..=0x59 | 0x78..=0x7B | 0x80..=0x8C | 0x8E..=0x93 | 0xC8..=0xD5 | 0xFF)
}

pub(crate) fn pair(address: u8) -> bool {
    matches!(address, 0x03 | 0x15 | 0x50 | 0x53)
}

pub(crate) fn protected(address: u8) -> bool {
    matches!(address, 0xC8..=0xD5)
}

pub(crate) fn validate_write(address: u8, value: u32) -> Result<()> {
    let valid = writable(address)
        && (pair(address) || value <= u16::MAX as u32)
        && match address {
            0x28 => value <= 1000,
            0x52 => value <= 2,
            0x53 => (value as i32).unsigned_abs() <= 3_686_400,
            0x59 | 0x83 | 0x8C | 0x8E | 0x8F => value <= 1,
            0x78 | 0x79 => value == 1,
            0x7B => matches!(value, 0x10..=0x12),
            0x81 => value <= 254,
            0x82 => value <= 9,
            0x8B => value == 0 || (50..=99).contains(&value),
            0xFF => matches!(value, 0 | 9 | 0xE6),
            _ => true,
        };
    if valid {
        Ok(())
    } else {
        Err(MotorError::InvalidArgument(format!(
            "invalid FashionStar write: address=0x{address:02X}, value={value}"
        )))
    }
}
