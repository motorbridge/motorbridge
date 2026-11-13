//! Huayi (华翼) joint driver — CANopen / CiA402 vendor crate.
//!
//! Protocol procedures (SDO framing, NMT, the CiA402 state machine, PDO
//! configuration) come from the shared [`motor_canopen`] layer; this crate
//! holds only Huayi semantics:
//!
//! - mode-number meanings — **`6060h = 5` is MIT here** (CSP on RobStride;
//!   never share these values across vendors),
//! - units — position/velocity/acceleration are `Float32` in Rev (output
//!   shaft) / Rev/s / Rev/s², torque is ‰ of the 6076h peak torque,
//! - the manufacturer `2xxx`/`3xxx` object region (MIT parameters, position
//!   preset, device settings).
//!
//! Manual: 华翼关节驱动器通信手册 (CANopen), references in per-item docs.

pub mod controller;
pub mod motor;
pub mod objects;
pub mod pdo;
pub mod units;

pub use controller::{HuayiController, HuayiScanHit};
pub use motor::{HuayiMotor, HuayiStatus};
pub use objects::{error_code_description, RunMode, ERROR_CODES};
pub use units::{MitRanges, DEFAULT_MIT_RANGES};
