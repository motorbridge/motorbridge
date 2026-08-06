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
// CAN 2.0 heartbeat (8 bytes) — one frame covers all core state:
//   Byte 0:    [Life:3] [ErrorFlags:5]  — high 3 bits = life counter (0-7),
//                                         low 5 bits = subsystem error flags
//   Byte 1:    [State:4] [Mode:4]       — high 4 bits = motor state, low 4 = control mode
//   Byte 2:    Motor Temp (uint8, offset -50°C → actual = raw - 50)
//   Byte 3-4:  Motor Position (int16, BE, motor turns × 100)
//   Byte 5-6:  Motor Velocity (int16, BE, motor turns/s × 100)
//   Byte 7:    Iq Current (int8, 0.5 A/bit)
//
// ErrorFlags bitmask:
//   bit 0 (0x01): AXIS       — axis.error_
//   bit 1 (0x02): MOTOR      — axis.motor_.error_
//   bit 2 (0x04): ENCODER    — axis.encoder_.error_
//   bit 3 (0x08): CONTROLLER — axis.controller_.error_
//   bit 4 (0x10): BOARD      — odrv.error_ (system level)
// ============================================================================

/// Heartbeat error flags (1 byte bitmask, lower 5 bits used).
pub mod heartbeat_error {
    pub const AXIS: u8 = 0x01;
    pub const MOTOR: u8 = 0x02;
    pub const ENCODER: u8 = 0x04;
    pub const CONTROLLER: u8 = 0x08;
    pub const BOARD: u8 = 0x10;
}

#[derive(Debug, Clone, Copy)]
pub struct HeartbeatFrame {
    /// Life counter (0-7, extracted from high 3 bits of byte 0).
    pub life_counter: u8,
    /// Subsystem error flags (lower 5 bits of byte 0).
    pub error_flags: u8,
    /// Motor state (high 4 bits of byte 1).
    pub motor_state: u8,
    /// Control mode (low 4 bits of byte 1).
    pub control_mode: u8,
    /// Motor temperature in °C (offset -50 decoded).
    pub motor_temp: f32,
    /// Motor position in turns (decoded from int16 BE × 100).
    pub position_turns: f32,
    /// Motor velocity in turns/s (decoded from int16 BE × 100).
    pub velocity_turns_per_s: f32,
    /// Iq current in Amps (decoded from int8, 0.5 A/bit).
    pub iq_current: f32,
}

pub fn decode_heartbeat(data: &[u8]) -> Option<HeartbeatFrame> {
    if data.len() < 8 {
        return None;
    }
    let life_counter = (data[0] >> 5) & 0x07;
    let error_flags = data[0] & 0x1F;
    let motor_state = (data[1] >> 4) & 0x0F;
    let control_mode = data[1] & 0x0F;
    let motor_temp = data[2] as f32 - 50.0;

    let pos_raw = ((data[3] as i16) << 8) | (data[4] as i16);
    let position_turns = pos_raw as f32 / 100.0;

    let vel_raw = ((data[5] as i16) << 8) | (data[6] as i16);
    let velocity_turns_per_s = vel_raw as f32 / 100.0;

    let iq_raw = data[7] as i8;
    let iq_current = iq_raw as f32 * 0.5;

    Some(HeartbeatFrame {
        life_counter,
        error_flags,
        motor_state,
        control_mode,
        motor_temp,
        position_turns,
        velocity_turns_per_s,
        iq_current,
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

// ============================================================================
// Parameter (SDO endpoint) read/write encode/decode
//
// PARAM_READ request (8 bytes):
//   Byte 0:    flags (0x00 for read)
//   Byte 1-2:  endpoint_id (uint16, BE)
//   Byte 3:    data_len (0 for read request)
//   Byte 4-7:  reserved (0)
//
// PARAM_READ response (8 bytes):
//   Byte 0:    flags (echo)
//   Byte 1-2:  endpoint_id (uint16, BE)
//   Byte 3:    data_len = 4
//   Byte 4-7:  value (float32, BE)
//
// PARAM_WRITE request (8 bytes):
//   Byte 0:    flags (0x00 for write)
//   Byte 1-2:  endpoint_id (uint16, BE)
//   Byte 3:    data_len = 4
//   Byte 4-7:  value (float32, BE)
//
// PARAM_WRITE response (8 bytes):
//   Byte 0:    flags (echo)
//   Byte 1-2:  endpoint_id (uint16, BE)
//   Byte 3:    data_len = 0 (write confirmation)
//   Byte 4-7:  reserved (0)
// ============================================================================

/// Encode a PARAM_READ request frame for an ODrive SDO endpoint.
pub fn encode_param_read(endpoint_id: u16) -> [u8; 8] {
    let mut buf = [0u8; 8];
    buf[0] = 0x00; // flags
    buf[1] = (endpoint_id >> 8) as u8;
    buf[2] = endpoint_id as u8;
    buf[3] = 0x00; // data_len = 0 (read request)
    // bytes 4-7 remain zero
    buf
}

/// Encode a PARAM_WRITE request frame for an ODrive SDO endpoint.
pub fn encode_param_write(endpoint_id: u16, value: f32) -> [u8; 8] {
    let mut buf = [0u8; 8];
    buf[0] = 0x00; // flags
    buf[1] = (endpoint_id >> 8) as u8;
    buf[2] = endpoint_id as u8;
    buf[3] = 0x04; // data_len = 4 (float32)
    f32_to_big_endian_bytes(value, &mut buf, 4);
    buf
}

/// Decode a PARAM_READ response value (float32 BE).
/// Returns `None` if the frame is not a valid read response.
pub fn decode_param_read_response(data: &[u8], expected_endpoint_id: u16) -> Option<f32> {
    if data.len() < 8 {
        return None;
    }
    let endpoint_id = ((data[1] as u16) << 8) | (data[2] as u16);
    if endpoint_id != expected_endpoint_id {
        return None;
    }
    if data[3] != 4 {
        return None;
    }
    Some(big_endian_bytes_to_f32(data, 4))
}

/// Check if a frame is a PARAM_WRITE acknowledgment (silent confirmation).
pub fn is_param_write_ack(data: &[u8], expected_endpoint_id: u16) -> bool {
    if data.len() < 8 {
        return false;
    }
    let endpoint_id = ((data[1] as u16) << 8) | (data[2] as u16);
    endpoint_id == expected_endpoint_id && data[3] == 0
}

/// Encode a CONFIG_SAVE request (empty frame).
pub fn encode_config_save() -> [u8; 8] {
    [0u8; 8]
}

/// Encode a CONFIG_RESET request (empty frame).
pub fn encode_config_reset() -> [u8; 8] {
    [0u8; 8]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_big_endian_float_roundtrip() {
        let mut buf = [0u8; 8];
        f32_to_big_endian_bytes(1.5, &mut buf, 0);
        let val = big_endian_bytes_to_f32(&buf, 0);
        assert!((val - 1.5).abs() < 0.001);
    }

    #[test]
    fn test_big_endian_float_zero() {
        let mut buf = [0u8; 8];
        f32_to_big_endian_bytes(0.0, &mut buf, 0);
        assert_eq!(buf[0], 0);
        assert_eq!(buf[1], 0);
        assert_eq!(buf[2], 0);
        assert_eq!(buf[3], 0);
    }

    #[test]
    fn test_encode_param_read() {
        let data = encode_param_read(0x001C);
        assert_eq!(data[0], 0x00); // flags
        assert_eq!(data[1], 0x00); // endpoint_id high
        assert_eq!(data[2], 0x1C); // endpoint_id low
        assert_eq!(data[3], 0x00); // data_len = 0
    }

    #[test]
    fn test_encode_param_write() {
        let data = encode_param_write(0x0035, 50.0);
        assert_eq!(data[0], 0x00); // flags
        assert_eq!(data[1], 0x00); // endpoint_id high
        assert_eq!(data[2], 0x35); // endpoint_id low
        assert_eq!(data[3], 0x04); // data_len = 4
    }

    #[test]
    fn test_decode_param_read_response() {
        // Create a valid read response: endpoint 0x001C, value 10.0
        let mut data = [0u8; 8];
        data[0] = 0x00;
        data[1] = 0x00;
        data[2] = 0x1C;
        data[3] = 0x04;
        f32_to_big_endian_bytes(10.0, &mut data, 4);

        let val = decode_param_read_response(&data, 0x001C);
        assert!(val.is_some());
        assert!((val.unwrap() - 10.0).abs() < 0.001);
    }

    #[test]
    fn test_decode_param_read_response_wrong_endpoint() {
        let mut data = [0u8; 8];
        data[3] = 0x04;
        let val = decode_param_read_response(&data, 0x001C);
        assert!(val.is_none());
    }

    #[test]
    fn test_is_param_write_ack() {
        let mut data = [0u8; 8];
        data[1] = 0x00;
        data[2] = 0x35;
        data[3] = 0x00; // data_len = 0 → ack
        assert!(is_param_write_ack(&data, 0x0035));
        assert!(!is_param_write_ack(&data, 0x0036));
    }

    #[test]
    fn test_pack_unpack_mit_zero_values() {
        let params = MitCommandParams::default();
        let packed = pack_mit_command(&params, 12.5, 50.0, 500.0, 100.0, 10.0);
        // Zero pos → mid-range uint16 (0x8000), zero vel/kp/kd/torque → mid-range
        assert_eq!(packed[0], 0x80);
        assert_eq!(packed[1], 0x00);
    }

    #[test]
    fn test_can_id_boundary_values() {
        // Minimum values
        let id = make_can_id(0, 0, 0, 0, 0);
        assert_eq!(id, 0);

        // Maximum values
        let id = make_can_id(7, 0xFF, 0xFF, 0xFF, 3);
        let parts = can_id_parts(id);
        assert_eq!(parts.priority, 7);
        assert_eq!(parts.msg_type, 0xFF);
        assert_eq!(parts.dest, 0xFF);
        assert_eq!(parts.source, 0xFF);
        assert_eq!(parts.seq, 3);
        assert!(parts.is_broadcast); // 0xFF >= 0x80
    }

    #[test]
    fn test_decode_heartbeat() {
        // Build a heartbeat frame per the new v8.1 format
        let mut data = [0u8; 8];
        data[0] = (2 << 5) | 0x03; // Life=2, ErrorFlags=AXIS|MOTOR
        data[1] = (0x3 << 4) | 0x1; // State=3(CLOSED_LOOP), Mode=1(POSITION)
        data[2] = 80; // Motor Temp raw → 30°C
        let pos_raw: i16 = 5000;
        data[3] = (pos_raw >> 8) as u8;
        data[4] = pos_raw as u8;
        let vel_raw: i16 = -200;
        data[5] = (vel_raw >> 8) as u8;
        data[6] = vel_raw as u8;
        data[7] = 20u8; // Iq = 10.0A

        let hb = decode_heartbeat(&data).expect("valid heartbeat");
        assert_eq!(hb.life_counter, 2);
        assert_eq!(hb.error_flags, 0x03);
        assert_eq!(hb.motor_state, 3);
        assert_eq!(hb.control_mode, 1);
        assert!((hb.motor_temp - 30.0).abs() < 1.0);
        assert!((hb.position_turns - 50.0).abs() < 0.1);
        assert!((hb.velocity_turns_per_s - (-2.0)).abs() < 0.1);
        assert!((hb.iq_current - 10.0).abs() < 0.5);
    }
}
