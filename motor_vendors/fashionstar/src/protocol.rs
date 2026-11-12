use motor_core::bus::CanFrame;
use motor_core::error::{MotorError, Result};

pub const CAN_ID: u16 = 0x321;

pub(crate) fn frame(id: u16, command: u8, fields: &[u8]) -> Result<CanFrame> {
    if id > 254 || fields.len() > 6 {
        return Err(MotorError::InvalidArgument(
            "FashionStar requires ServoID 0..254 and a classic CAN payload".to_string(),
        ));
    }
    let mut data = [0; 8];
    data[0] = command;
    data[1] = id as u8;
    data[2..2 + fields.len()].copy_from_slice(fields);
    Ok(CanFrame {
        arbitration_id: u32::from(CAN_ID),
        data,
        dlc: (fields.len() + 2) as u8,
        is_extended: false,
        is_rx: false,
    })
}

pub(crate) fn read(id: u16, address: u8, pair: bool) -> Result<CanFrame> {
    if pair {
        frame(id, b'R', &[address, address + 1])
    } else {
        frame(id, b'r', &[address])
    }
}

pub(crate) fn write(id: u16, address: u8, value: u32, pair: bool) -> Result<CanFrame> {
    let bytes = value.to_le_bytes();
    if pair {
        frame(
            id,
            b'W',
            &[address, bytes[0], bytes[1], address + 1, bytes[2], bytes[3]],
        )
    } else {
        frame(id, b'w', &[address, bytes[0], bytes[1]])
    }
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct Reply {
    pub address: u8,
    pub value: u16,
    pub second: Option<(u8, u16)>,
}

pub(crate) fn accepts_frame(frame: &CanFrame, id: u16) -> bool {
    frame.is_rx
        && !frame.is_extended
        && frame.arbitration_id == u32::from(CAN_ID)
        && frame.data[1] == id as u8
        && matches!((frame.data[0], frame.dlc), (b'v', 5) | (b'V', 8))
}

pub(crate) fn decode(frame: &CanFrame, id: u16) -> Result<Reply> {
    if !accepts_frame(frame, id) {
        return Err(MotorError::Protocol(
            "invalid FashionStar response".to_string(),
        ));
    }
    Ok(Reply {
        address: frame.data[2],
        value: u16::from_le_bytes([frame.data[3], frame.data[4]]),
        second: (frame.data[0] == b'V').then(|| {
            (
                frame.data[5],
                u16::from_le_bytes([frame.data[6], frame.data[7]]),
            )
        }),
    })
}

pub(crate) fn angle_raw(radians: f32) -> Result<u32> {
    let raw = (f64::from(radians) * 1800.0 / std::f64::consts::PI).round();
    if !radians.is_finite() || raw.abs() > 3_686_400.0 {
        return Err(MotorError::InvalidArgument(
            "FashionStar position must be finite and within +/-1024 turns".to_string(),
        ));
    }
    Ok(raw as i32 as u32)
}

pub(crate) fn speed_raw(radians_per_second: f32) -> Result<u16> {
    let raw = (f64::from(radians_per_second) * 1800.0 / std::f64::consts::PI).round();
    if !radians_per_second.is_finite() || !(1.0..=65535.0).contains(&raw) {
        return Err(MotorError::InvalidArgument(
            "FashionStar velocity limit must be finite and encode to 1..65535 (0.1 deg/s)"
                .to_string(),
        ));
    }
    Ok(raw as u16)
}

pub(crate) fn temperature_c(raw: u16) -> f32 {
    if !(1..4096).contains(&raw) {
        return f32::NAN;
    }
    // Default NTC conversion used by the official CAN SDK and web tool.
    let ratio = f64::from(raw) / (4096.0 - f64::from(raw));
    (1.0 / (1.0 / 298.15 + ratio.ln() / 3435.0) - 273.15) as f32
}
