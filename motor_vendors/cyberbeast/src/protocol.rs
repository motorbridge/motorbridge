//! CyberBeast CAN protocol — Big-Endian, extended 29-bit CAN ID with J1939-like layout.
//!
//! CAN ID layout (29-bit extended):
//!   Priority[28:26] | MsgType[25:18] | Dest[17:10] | Source[9:2] | Seq[1:0]
//!
//! Design principles:
//! - Big-Endian (Motorola) byte order for all multi-byte payloads.
//! - Implicit addressing: MsgType < 0x80 = point-to-point, MsgType >= 0x80 = broadcast.
//! - 3-bit priority (8 levels, J1939 style).
//! - 2-bit sequence number for loss detection.
//!
//! Reference: ODrive Firmware/communication/can/can_cyberbeast.hpp

// ============================================================================
// CAN ID bit layout
// ============================================================================

pub const PRIORITY_SHIFT: u32 = 26;
pub const MSGTYPE_SHIFT: u32 = 18;
pub const DEST_SHIFT: u32 = 10;
pub const SOURCE_SHIFT: u32 = 2;
pub const SEQ_SHIFT: u32 = 0;

pub const PRIORITY_MASK: u32 = 0x7;
pub const MSGTYPE_MASK: u32 = 0xFF;
pub const DEST_MASK: u32 = 0xFF;
pub const SOURCE_MASK: u32 = 0xFF;
pub const SEQ_MASK: u32 = 0x3;

/// MsgType threshold: values >= this are broadcast (J1939 PDU2 style).
pub const MSGTYPE_BROADCAST_THRESHOLD: u8 = 0x80;

/// Broadcast address value in the Dest field for non-multicast broadcast frames.
pub const ADDR_BROADCAST: u8 = 0xFF;

/// Maximum number of devices addressable in a single broadcast slot frame.
pub const MAX_BROADCAST_DEVICES: u8 = 8;

/// Default master (host) node address.
pub const DEFAULT_MASTER_ID: u8 = 0x01;

// ============================================================================
// Priority (3-bit, 8 levels, J1939 style)
// ============================================================================

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Priority {
    Critical = 0,
    Emergency = 1,
    HighCtrl = 2,
    Ctrl = 3,
    Config = 4,
    Query = 5,
    Status = 6,
    Low = 7,
}

impl Priority {
    pub fn from_u8(v: u8) -> Self {
        match v & PRIORITY_MASK as u8 {
            0 => Self::Critical,
            1 => Self::Emergency,
            2 => Self::HighCtrl,
            3 => Self::Ctrl,
            4 => Self::Config,
            5 => Self::Query,
            6 => Self::Status,
            _ => Self::Low,
        }
    }
}

// ============================================================================
// MsgType (8-bit, 256 message types)
// ============================================================================

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum MsgType {
    // --- Point-to-point real-time control (0x00–0x1F) ---
    MitControl = 0x00,
    PosControl = 0x01,
    VelControl = 0x02,
    TorqueControl = 0x03,
    CurrentControl = 0x04,

    // --- Point-to-point config management (0x20–0x3F) ---
    ParamRead = 0x20,
    ParamWrite = 0x21,
    ConfigSave = 0x22,
    ConfigReset = 0x23,
    JsonDescRead = 0x24,
    JsonDescData = 0x25,

    // --- Point-to-point status queries (0x40–0x5F) ---
    QueryStatus = 0x40,
    QueryPosVel = 0x41,
    QueryCurrent = 0x42,
    QueryTemperature = 0x43,
    QueryBus = 0x44,
    QueryError = 0x45,
    QueryDeviceInfo = 0x46,
    QueryPower = 0x47,
    Heartbeat = 0x48,
    StatusFeedback = 0x49,

    // --- Point-to-point system management (0x60–0x7F) ---
    SetNodeId = 0x60,
    SetZero = 0x61,
    StartMotor = 0x62,
    StopMotor = 0x63,
    ResetDevice = 0x64,
    ClearErrors = 0x65,

    // --- Broadcast real-time control (0x80–0x9F) ---
    MitControlBcast = 0x80,
    PosControlBcast = 0x81,
    VelControlBcast = 0x82,
    TorqueControlBcast = 0x83,

    // --- Broadcast emergency/system (0xC0–0xDF) ---
    Estop = 0xC0,
    FaultAlert = 0xC1,
}

impl MsgType {
    pub fn from_u8(v: u8) -> Self {
        match v {
            0x00 => Self::MitControl,
            0x01 => Self::PosControl,
            0x02 => Self::VelControl,
            0x03 => Self::TorqueControl,
            0x04 => Self::CurrentControl,
            0x20 => Self::ParamRead,
            0x21 => Self::ParamWrite,
            0x22 => Self::ConfigSave,
            0x23 => Self::ConfigReset,
            0x24 => Self::JsonDescRead,
            0x25 => Self::JsonDescData,
            0x40 => Self::QueryStatus,
            0x41 => Self::QueryPosVel,
            0x42 => Self::QueryCurrent,
            0x43 => Self::QueryTemperature,
            0x44 => Self::QueryBus,
            0x45 => Self::QueryError,
            0x46 => Self::QueryDeviceInfo,
            0x47 => Self::QueryPower,
            0x48 => Self::Heartbeat,
            0x49 => Self::StatusFeedback,
            0x60 => Self::SetNodeId,
            0x61 => Self::SetZero,
            0x62 => Self::StartMotor,
            0x63 => Self::StopMotor,
            0x64 => Self::ResetDevice,
            0x65 => Self::ClearErrors,
            0x80 => Self::MitControlBcast,
            0x81 => Self::PosControlBcast,
            0x82 => Self::VelControlBcast,
            0x83 => Self::TorqueControlBcast,
            0xC0 => Self::Estop,
            0xC1 => Self::FaultAlert,
            _ => {
                // Unknown type: preserve raw value semantics
                // is_broadcast is determined by >= MSGTYPE_BROADCAST_THRESHOLD
                Self::MitControl // safe fallback for routing purposes
            }
        }
    }

    pub fn is_broadcast(&self) -> bool {
        (*self as u8) >= MSGTYPE_BROADCAST_THRESHOLD
    }
}

// ============================================================================
// Error codes in MIT response
// ============================================================================

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ErrorCode {
    None = 0x0,
    Motor = 0x1,
    Encoder = 0x2,
    Controller = 0x3,
    UnderVoltage = 0x4,
    OverTemp = 0x5,
    OverCurrent = 0x6,
    Stall = 0x7,
    CanTimeout = 0x8,
    Multiple = 0xF,
}

impl ErrorCode {
    pub fn from_u8(v: u8) -> Self {
        match v {
            0x0 => Self::None,
            0x1 => Self::Motor,
            0x2 => Self::Encoder,
            0x3 => Self::Controller,
            0x4 => Self::UnderVoltage,
            0x5 => Self::OverTemp,
            0x6 => Self::OverCurrent,
            0x7 => Self::Stall,
            0x8 => Self::CanTimeout,
            _ => Self::Multiple,
        }
    }
}

// ============================================================================
// Mode state in MIT response
// ============================================================================

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ModeState {
    Reset = 0x0,
    Calibrating = 0x1,
    Idle = 0x2,
    ClosedLoop = 0x3,
    Mit = 0x4,
    Position = 0x5,
    Velocity = 0x6,
    Torque = 0x7,
}

impl ModeState {
    pub fn from_u8(v: u8) -> u8 {
        // Return raw value; caller can interpret
        v & 0x0F
    }

    pub fn name(v: u8) -> &'static str {
        match v & 0x0F {
            0x0 => "reset",
            0x1 => "calibrating",
            0x2 => "idle",
            0x3 => "closed_loop",
            0x4 => "mit",
            0x5 => "position",
            0x6 => "velocity",
            0x7 => "torque",
            _ => "unknown",
        }
    }
}

// ============================================================================
// Parsed CAN ID
// ============================================================================

#[derive(Debug, Clone, Copy)]
pub struct CyberBeastCanId {
    pub priority: u8,
    pub msg_type: u8,
    pub dest: u8,
    pub source: u8,
    pub seq: u8,
    pub is_broadcast: bool,
}

// ============================================================================
// CAN ID encode / decode
// ============================================================================

/// Build a 29-bit extended CAN ID from CyberBeast protocol fields.
pub const fn make_can_id(priority: u8, msg_type: u8, dest: u8, source: u8, seq: u8) -> u32 {
    ((priority & PRIORITY_MASK as u8) as u32) << PRIORITY_SHIFT
        | (msg_type as u32) << MSGTYPE_SHIFT
        | ((dest & DEST_MASK as u8) as u32) << DEST_SHIFT
        | ((source & SOURCE_MASK as u8) as u32) << SOURCE_SHIFT
        | ((seq & SEQ_MASK as u8) as u32) << SEQ_SHIFT
}

/// Parse a 29-bit CAN ID into its CyberBeast protocol fields.
pub fn can_id_parts(can_id: u32) -> CyberBeastCanId {
    let msg_type = ((can_id >> MSGTYPE_SHIFT) & MSGTYPE_MASK) as u8;
    CyberBeastCanId {
        priority: ((can_id >> PRIORITY_SHIFT) & PRIORITY_MASK) as u8,
        msg_type,
        dest: ((can_id >> DEST_SHIFT) & DEST_MASK) as u8,
        source: ((can_id >> SOURCE_SHIFT) & SOURCE_MASK) as u8,
        seq: ((can_id >> SEQ_SHIFT) & SEQ_MASK) as u8,
        is_broadcast: msg_type >= MSGTYPE_BROADCAST_THRESHOLD,
    }
}

/// Increment sequence number (mod 4).
pub const fn seq_next(seq: u8) -> u8 {
    (seq + 1) & SEQ_MASK as u8
}

// ============================================================================
// Big-Endian float32 encoding
// ============================================================================

pub fn big_endian_bytes_to_f32(buf: &[u8], offset: usize) -> f32 {
    if offset + 4 > buf.len() {
        return 0.0;
    }
    let raw = ((buf[offset] as u32) << 24)
        | ((buf[offset + 1] as u32) << 16)
        | ((buf[offset + 2] as u32) << 8)
        | (buf[offset + 3] as u32);
    f32::from_bits(raw)
}

pub fn f32_to_big_endian_bytes(val: f32, buf: &mut [u8], offset: usize) {
    if offset + 4 > buf.len() {
        return;
    }
    let raw = val.to_bits();
    buf[offset] = (raw >> 24) as u8;
    buf[offset + 1] = (raw >> 16) as u8;
    buf[offset + 2] = (raw >> 8) as u8;
    buf[offset + 3] = raw as u8;
}

// ============================================================================
// Float/int range mapping (replicates ODrive float_to_uint / uint_to_float)
// ============================================================================

fn float_to_uint(x: f32, x_min: f32, x_max: f32, bits: u32) -> u32 {
    let span = x_max - x_min;
    if span <= 0.0 {
        return 0;
    }
    let max_val = (1u32 << bits) - 1;
    let clamped = x.clamp(x_min, x_max);
    ((clamped - x_min) / span * max_val as f32).round() as u32
}

fn uint_to_float(x_int: u32, x_min: f32, x_max: f32, bits: u32) -> f32 {
    let max_val = (1u32 << bits) - 1;
    let span = x_max - x_min;
    x_min + (x_int as f32 / max_val as f32) * span
}

// ============================================================================
// MIT command pack (8 bytes per device, Big-Endian bit-packed)
//
// Layout per 8-byte slot:
//   Byte 0-1:  pos       [15:0]   16-bit
//   Byte 2-3:  vel[11:0] | kp[11:8]  12-bit vel + 4-bit kp high
//   Byte 4:    kp[7:0]             kp low
//   Byte 5-6:  kd[11:0] | torque[11:8]  12-bit kd + 4-bit torque high
//   Byte 7:    torque[7:0]          torque low
// ============================================================================

pub struct MitCommandParams {
    pub pos: f32,
    pub vel: f32,
    pub kp: f32,
    pub kd: f32,
    pub torque: f32,
}

impl Default for MitCommandParams {
    fn default() -> Self {
        Self {
            pos: 0.0,
            vel: 0.0,
            kp: 0.0,
            kd: 0.0,
            torque: 0.0,
        }
    }
}

pub fn pack_mit_command(
    params: &MitCommandParams,
    pos_limit: f32,
    vel_limit: f32,
    kp_limit: f32,
    kd_limit: f32,
    torque_limit: f32,
) -> [u8; 8] {
    let p_int = float_to_uint(params.pos, -pos_limit, pos_limit, 16);
    let v_int = float_to_uint(params.vel, -vel_limit, vel_limit, 12);
    let kp_int = float_to_uint(params.kp, 0.0, kp_limit, 12);
    let kd_int = float_to_uint(params.kd, 0.0, kd_limit, 12);
    let t_int = float_to_uint(params.torque, -torque_limit, torque_limit, 12);

    let mut buf = [0u8; 8];
    buf[0] = (p_int >> 8) as u8;
    buf[1] = p_int as u8;
    buf[2] = (v_int >> 4) as u8;
    buf[3] = ((v_int & 0x0F) << 4) as u8 | ((kp_int >> 8) & 0x0F) as u8;
    buf[4] = kp_int as u8;
    buf[5] = (kd_int >> 4) as u8;
    buf[6] = ((kd_int & 0x0F) << 4) as u8 | ((t_int >> 8) & 0x0F) as u8;
    buf[7] = t_int as u8;
    buf
}

pub fn unpack_mit_command(
    data: &[u8],
    pos_limit: f32,
    vel_limit: f32,
    kp_limit: f32,
    kd_limit: f32,
    torque_limit: f32,
) -> MitCommandParams {
    if data.len() < 8 {
        return MitCommandParams::default();
    }
    let p_int = ((data[0] as u16) << 8) | (data[1] as u16);
    let v_int = ((data[2] as u16) << 4) | ((data[3] >> 4) as u16);
    let kp_int = (((data[3] & 0x0F) as u16) << 8) | (data[4] as u16);
    let kd_int = ((data[5] as u16) << 4) | ((data[6] >> 4) as u16);
    let t_int = (((data[6] & 0x0F) as u16) << 8) | (data[7] as u16);

    MitCommandParams {
        pos: uint_to_float(p_int as u32, -pos_limit, pos_limit, 16),
        vel: uint_to_float(v_int as u32, -vel_limit, vel_limit, 12),
        kp: uint_to_float(kp_int as u32, 0.0, kp_limit, 12),
        kd: uint_to_float(kd_int as u32, 0.0, kd_limit, 12),
        torque: uint_to_float(t_int as u32, -torque_limit, torque_limit, 12),
    }
}

// ============================================================================
// MIT response unpack
//
// Layout (8 bytes):
//   Byte 0-1:  pos         [15:0]   16-bit
//   Byte 2-3:  vel[11:0] | error[3:0]  12-bit vel + 4-bit error code
//   Byte 4-5:  current[11:0] | mode[3:0] 12-bit current + 4-bit mode
//   Byte 6:    motor_temp    u8 (offset -50: actual = raw - 50)
//   Byte 7:    mos_temp      u8 (offset -50)
// ============================================================================

#[derive(Debug, Clone, Copy)]
pub struct MitResponse {
    pub pos: f32,
    pub vel: f32,
    pub current: f32,
    pub error_code: u8,
    pub mode_state: u8,
    pub motor_temp: f32,
    pub mos_temp: f32,
}

pub fn unpack_mit_response(
    data: &[u8],
    pos_limit: f32,
    vel_limit: f32,
    current_limit: f32,
) -> MitResponse {
    if data.len() < 8 {
        return MitResponse {
            pos: 0.0,
            vel: 0.0,
            current: 0.0,
            error_code: 0,
            mode_state: 0,
            motor_temp: 0.0,
            mos_temp: 0.0,
        };
    }

    let p_int = ((data[0] as u16) << 8) | (data[1] as u16);
    let v_int = ((data[2] as u16) << 4) | ((data[3] >> 4) as u16);
    let error = data[3] & 0x0F;
    let c_int = ((data[4] as u16) << 4) | ((data[5] >> 4) as u16);
    let mode = data[5] & 0x0F;
    let motor_temp_raw = data[6];
    let mos_temp_raw = data[7];

    MitResponse {
        pos: uint_to_float(p_int as u32, -pos_limit, pos_limit, 16),
        vel: uint_to_float(v_int as u32, -vel_limit, vel_limit, 12),
        current: uint_to_float(c_int as u32, -current_limit, current_limit, 12),
        error_code: error,
        mode_state: mode,
        motor_temp: motor_temp_raw as f32 - 50.0,
        mos_temp: mos_temp_raw as f32 - 50.0,
    }
}

// ============================================================================
// POS/VOL/TORQUE/CURRENT control encode
// ============================================================================

/// Encode POS_CONTROL frame (8-byte CAN 2.0 compatible).
///
/// Encodes target position and velocity limit as two float32 BE values.
/// Current limit is omitted — set it separately via PARAM_WRITE (endpoint 0x001C:
/// `motor.config.current_lim`) or use the existing device configuration.
///
/// For full 12-byte encoding (including cur_limit), use CAN FD frames.
pub fn encode_pos_control(target_pos_deg: f32, vel_limit_rpm: f32, _cur_limit_a: f32) -> [u8; 8] {
    let mut buf = [0u8; 8];
    f32_to_big_endian_bytes(target_pos_deg, &mut buf, 0);
    f32_to_big_endian_bytes(vel_limit_rpm, &mut buf, 4);
    buf
}

/// Encode VEL_CONTROL frame: float32 target_vel(RPM) + float32 cur_limit(A)
pub fn encode_vel_control(target_vel_rpm: f32, cur_limit_a: f32) -> [u8; 8] {
    let mut buf = [0u8; 8];
    f32_to_big_endian_bytes(target_vel_rpm, &mut buf, 0);
    f32_to_big_endian_bytes(cur_limit_a, &mut buf, 4);
    buf
}

/// Encode TORQUE_CONTROL frame: float32 target_torque(N·m)
pub fn encode_torque_control(target_torque_nm: f32) -> [u8; 8] {
    let mut buf = [0u8; 8];
    f32_to_big_endian_bytes(target_torque_nm, &mut buf, 0);
    buf
}

/// Encode CURRENT_CONTROL frame: float32 target_current(A)
pub fn encode_current_control(target_current_a: f32) -> [u8; 8] {
    let mut buf = [0u8; 8];
    f32_to_big_endian_bytes(target_current_a, &mut buf, 0);
    buf
}

// ============================================================================
// System management command encodes
// ============================================================================

pub fn encode_set_zero() -> [u8; 8] {
    [0u8; 8]
}

pub fn encode_clear_errors() -> [u8; 8] {
    [0u8; 8]
}

// ============================================================================
// Heartbeat decode
//
// CAN 2.0 heartbeat (8 bytes):
//   Byte 0:    life_counter
//   Byte 1:    device_id
//   Byte 2-3:  axis_error   (uint16, BE)
//   Byte 4-5:  motor_error  (uint16, BE)
//   Byte 6-7:  encoder_error (uint16, BE)
// ============================================================================

#[derive(Debug, Clone, Copy)]
pub struct HeartbeatFrame {
    pub life_counter: u8,
    pub device_id: u8,
    pub axis_error: u16,
    pub motor_error: u16,
    pub encoder_error: u16,
}

pub fn decode_heartbeat(data: &[u8]) -> Option<HeartbeatFrame> {
    if data.len() < 8 {
        return None;
    }
    Some(HeartbeatFrame {
        life_counter: data[0],
        device_id: data[1],
        axis_error: ((data[2] as u16) << 8) | (data[3] as u16),
        motor_error: ((data[4] as u16) << 8) | (data[5] as u16),
        encoder_error: ((data[6] as u16) << 8) | (data[7] as u16),
    })
}

// ============================================================================
// POS/VOL response decode
// ============================================================================

/// Decode a QUERY_POS_VEL response: 2× float32 BE (pos turns, vel turns/s)
pub fn decode_pos_vel_response(data: &[u8]) -> (f32, f32) {
    if data.len() < 8 {
        return (0.0, 0.0);
    }
    let pos = big_endian_bytes_to_f32(data, 0);
    let vel = big_endian_bytes_to_f32(data, 4);
    (pos, vel)
}

/// Decode a QUERY_CURRENT response: 2× float32 BE (iq, id)
pub fn decode_current_response(data: &[u8]) -> (f32, f32) {
    if data.len() < 8 {
        return (0.0, 0.0);
    }
    let iq = big_endian_bytes_to_f32(data, 0);
    let id = big_endian_bytes_to_f32(data, 4);
    (iq, id)
}

/// Decode a QUERY_TEMPERATURE response: 2× float32 BE (motor_temp °C, fet_temp °C)
pub fn decode_temperature_response(data: &[u8]) -> (f32, f32) {
    if data.len() < 8 {
        return (0.0, 0.0);
    }
    let motor_temp = big_endian_bytes_to_f32(data, 0);
    let fet_temp = big_endian_bytes_to_f32(data, 4);
    (motor_temp, fet_temp)
}

/// Decode a QUERY_BUS response: 2× float32 BE (vbus V, ibus A)
pub fn decode_bus_response(data: &[u8]) -> (f32, f32) {
    if data.len() < 8 {
        return (0.0, 0.0);
    }
    let vbus = big_endian_bytes_to_f32(data, 0);
    let ibus = big_endian_bytes_to_f32(data, 4);
    (vbus, ibus)
}
