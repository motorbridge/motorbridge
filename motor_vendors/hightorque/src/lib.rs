pub mod controller;
pub mod motor;
pub mod protocol;
pub mod registers;

pub use controller::HightorqueController;
pub use motor::HightorqueMotor;
pub use protocol::{
    decode_fault, decode_unit_raw, encode_unit_raw, from_turns, to_turns, FirmwareVersion,
    HightorqueFeedbackState, MotorModel, RunMode, TorqueCoeff, SETTING_ACK,
};
pub use protocol::{
    AngleUnit, DataType, QuantityScale, ACC_SCALE, CUR_SCALE, PID_SCALE, POS_SCALE, TQE_SCALE,
    VEL_SCALE, VOL_SCALE,
};
pub use protocol::{ReadCmd, RegisterType, RegisterValue};
pub use registers::{parameter_info, RegisterInfo, PARAMETER_TABLE};
