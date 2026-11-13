//! Standard CANopen object-dictionary indices and command constants.
//!
//! Only indices from the predefined connection set and the CiA402 profile
//! region (`60xx`) live here. Manufacturer-specific objects (`2xxx`/`3xxx`)
//! and unit conversions belong to the vendor crates.

// --- NMT ---
pub const NMT_ID: u32 = 0x000;

pub const NMT_START_REMOTE_NODE: u8 = 0x01;
pub const NMT_STOP_NODE: u8 = 0x02;
pub const NMT_ENTER_PRE_OPERATIONAL: u8 = 0x80;
pub const NMT_RESET_NODE: u8 = 0x81;
pub const NMT_RESET_COMMUNICATION: u8 = 0x82;

// --- CiA402 profile objects ---
pub const CONTROLWORD: u16 = 0x6040;
pub const STATUSWORD: u16 = 0x6041;
pub const ERROR_CODE: u16 = 0x603F;
pub const MODES_OF_OPERATION: u16 = 0x6060;
pub const MODES_OF_OPERATION_DISPLAY: u16 = 0x6061;
pub const POSITION_DEMAND_VALUE: u16 = 0x6062;
pub const POSITION_ACTUAL_VALUE: u16 = 0x6064;
pub const VELOCITY_DEMAND_VALUE: u16 = 0x606B;
pub const VELOCITY_ACTUAL_VALUE: u16 = 0x606C;
pub const TARGET_TORQUE: u16 = 0x6071;
pub const TORQUE_ACTUAL_VALUE: u16 = 0x6077;
pub const TARGET_POSITION: u16 = 0x607A;
pub const PROFILE_VELOCITY: u16 = 0x6081;
pub const PROFILE_ACCELERATION: u16 = 0x6083;
pub const PROFILE_DECELERATION: u16 = 0x6084;
pub const TARGET_VELOCITY: u16 = 0x60FF;

// --- Communication objects used by the shared layer ---
pub const ERROR_REGISTER: u16 = 0x1001;
pub const PRE_DEFINED_ERROR_FIELD: u16 = 0x1003;
pub const TPDO_COMMUNICATION_BASE: u16 = 0x1800;
pub const TPDO_MAPPING_BASE: u16 = 0x1A00;
pub const RPDO_COMMUNICATION_BASE: u16 = 0x1400;
pub const RPDO_MAPPING_BASE: u16 = 0x1600;

// --- Controlword command values (CiA402, bits 7 and 3..0) ---
pub const CW_DISABLE_VOLTAGE: u16 = 0x0000;
pub const CW_SHUTDOWN: u16 = 0x0006;
pub const CW_SWITCH_ON: u16 = 0x0007;
pub const CW_DISABLE_OPERATION: u16 = 0x0007;
pub const CW_QUICK_STOP: u16 = 0x000B;
pub const CW_ENABLE_OPERATION: u16 = 0x000F;
pub const CW_FAULT_RESET: u16 = 0x0080;

/// CiA402 `Shutdown → Switch On → Enable Operation` command sequence.
pub const CW_ENABLE_SEQUENCE: &[u16] = &[CW_SHUTDOWN, CW_SWITCH_ON, CW_ENABLE_OPERATION];

// --- COB-ID helpers (predefined connection set) ---
pub fn sdo_req_id(node_id: u16) -> u32 {
    0x600 + u32::from(node_id)
}

pub fn sdo_rsp_id(node_id: u16) -> u32 {
    0x580 + u32::from(node_id)
}

pub fn heartbeat_id(node_id: u16) -> u32 {
    0x700 + u32::from(node_id)
}

pub fn emcy_id(node_id: u16) -> u32 {
    0x080 + u32::from(node_id)
}

/// TPDO `n` (1..=4) COB-ID for `node_id`: `0x180 + 0x100*(n-1) + node`.
pub fn tpdo_id(n: u8, node_id: u16) -> u32 {
    0x180 + 0x100 * u32::from(n.saturating_sub(1)) + u32::from(node_id)
}

/// RPDO `n` (1..=4) COB-ID for `node_id`: `0x200 + 0x100*(n-1) + node`.
pub fn rpdo_id(n: u8, node_id: u16) -> u32 {
    0x200 + 0x100 * u32::from(n.saturating_sub(1)) + u32::from(node_id)
}
