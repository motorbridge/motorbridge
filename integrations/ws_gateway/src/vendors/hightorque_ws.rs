use crate::model::{Target, Transport};
use motor_core::bus::{CanBus, CanFrame};
use std::sync::Arc;
use std::time::{Duration, Instant};

pub(crate) const TWO_PI: f32 = std::f32::consts::PI * 2.0;

#[derive(Debug, Clone, Copy)]
pub(crate) struct HighTorqueStatus {
    pub(crate) motor_id: u16,
    pub(crate) pos_raw: i16,
    pub(crate) vel_raw: i16,
    pub(crate) tqe_raw: i16,
}

impl HighTorqueStatus {
    pub(crate) fn pos_rad(self) -> f32 {
        self.pos_raw as f32 * 0.0001 * TWO_PI
    }

    pub(crate) fn vel_rad_s(self) -> f32 {
        self.vel_raw as f32 * 0.00025 * TWO_PI
    }
}

fn can_ext_id_for_motor(motor_id: u16) -> u32 {
    u32::from(0x8000u16 | motor_id)
}

pub(crate) fn send_hightorque_ext(
    bus: &dyn CanBus,
    motor_id: u16,
    payload: &[u8],
) -> Result<(), String> {
    if payload.len() > 8 {
        return Err("payload too long (max 8 bytes)".to_string());
    }
    let mut data = [0u8; 8];
    data[..payload.len()].copy_from_slice(payload);
    bus.send(CanFrame {
        arbitration_id: can_ext_id_for_motor(motor_id),
        data,
        dlc: payload.len() as u8,
        is_extended: true,
        is_rx: false,
    })
    .map_err(|e| e.to_string())
}

fn decode_hightorque_read_reply(frame: CanFrame) -> Option<HighTorqueStatus> {
    if frame.dlc < 8 {
        return None;
    }
    if frame.data[0] != 0x27 || frame.data[1] != 0x01 {
        return None;
    }
    let motor_id = if !frame.is_extended && (frame.arbitration_id & 0x00FF) == 0 {
        ((frame.arbitration_id >> 8) & 0x7F) as u16
    } else {
        (frame.arbitration_id & 0x7FF) as u16
    };
    Some(HighTorqueStatus {
        motor_id,
        pos_raw: i16::from_le_bytes([frame.data[2], frame.data[3]]),
        vel_raw: i16::from_le_bytes([frame.data[4], frame.data[5]]),
        tqe_raw: i16::from_le_bytes([frame.data[6], frame.data[7]]),
    })
}

pub(crate) fn wait_hightorque_status_for_motor(
    bus: &dyn CanBus,
    motor_id: u16,
    timeout: Duration,
) -> Result<Option<HighTorqueStatus>, String> {
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        let left = deadline.saturating_duration_since(Instant::now());
        if let Some(frame) = bus
            .recv(left.min(Duration::from_millis(20)))
            .map_err(|e| e.to_string())?
        {
            if let Some(status) = decode_hightorque_read_reply(frame) {
                if status.motor_id == motor_id {
                    return Ok(Some(status));
                }
            }
        }
    }
    Ok(None)
}

pub(crate) fn pos_raw_from_rad(rad: f32) -> i16 {
    round_to_i16_saturated(rad / TWO_PI * 10_000.0)
}

pub(crate) fn vel_raw_from_rad_s(rad_s: f32) -> i16 {
    round_to_i16_saturated(rad_s / TWO_PI / 0.00025)
}

pub(crate) fn tqe_raw_from_tau(tau: f32) -> i16 {
    round_to_i16_saturated(tau * 100.0)
}

fn round_to_i16_saturated(v: f32) -> i16 {
    (v.round() as i32).clamp(i16::MIN as i32, i16::MAX as i32) as i16
}

pub(crate) fn open_hightorque_bus(target: &Target) -> Result<Arc<dyn CanBus>, String> {
    let p = motor_core::bus::TransportParams {
        channel: &target.channel,
        serial_port: &target.serial_port,
        serial_baud: target.serial_baud,
    };
    match target.transport {
        Transport::Auto | Transport::SocketCan => {
            motor_core::bus::open_transport(motor_core::bus::Transport::SocketCan, &p)
                .map_err(|e| format!("open bus failed: {e}"))
        }
        Transport::SocketCanFd => {
            Err("hightorque currently uses standard CAN transport only".to_string())
        }
        Transport::DmSerial => Err("dm-serial transport is damiao-only".to_string()),
        Transport::DmDevice => Err("dm-device transport is damiao-only".to_string()),
        Transport::McuSerial => {
            motor_core::bus::open_transport(motor_core::bus::Transport::McuSerial, &p)
                .map_err(|e| format!("open bus failed: {e}"))
        }
    }
}
