pub mod controller;
pub mod motor;
pub mod protocol;
pub mod registers;

pub use controller::CyberBeastController;
pub use motor::{model_limits, ControlMode, CyberBeastMotor, CyberBeastMotorState};
pub use protocol::{
    big_endian_bytes_to_f32, can_id_parts, decode_device_info_response, decode_heartbeat,
    decode_param_read_response, encode_config_reset, encode_config_save, encode_param_read,
    encode_param_write, encode_query_error, error_type, f32_to_big_endian_bytes, heartbeat_error,
    is_param_write_ack, make_can_id, unpack_mit_response, CyberBeastCanId, DeviceInfo, ErrorCode,
    HeartbeatFrame, MitCommandParams, MitResponse, ModeState, MsgType, Priority, ADDR_BROADCAST,
    MSGTYPE_BROADCAST_THRESHOLD, PRIORITY_MASK, SEQ_MASK,
};
pub use registers::{RegisterInfo, REGISTER_TABLE};
