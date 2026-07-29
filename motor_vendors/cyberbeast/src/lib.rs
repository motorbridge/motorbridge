pub mod controller;
pub mod motor;
pub mod protocol;
pub mod registers;

pub use controller::CyberBeastController;
pub use motor::{model_limits, ControlMode, CyberBeastMotor, CyberBeastMotorState};
pub use protocol::{
    big_endian_bytes_to_f32, can_id_parts, f32_to_big_endian_bytes, make_can_id,
    unpack_mit_response, CyberBeastCanId, Priority, MsgType, ErrorCode, ModeState,
    ADDR_BROADCAST, MSGTYPE_BROADCAST_THRESHOLD, PRIORITY_MASK, SEQ_MASK,
};
pub use registers::{RegisterInfo, REGISTER_TABLE};
