//! Shared CANopen / CiA402 protocol framework for CANopen-family vendor
//! drivers (e.g. `motor_vendor_robstride_cia402`, `motor_vendor_huayi`).
//!
//! # Layering contract
//!
//! This crate contains **protocol procedures only** — how a frame is encoded,
//! which COB-ID carries it, and which command sequence drives the CiA402
//! state machine. It must **never** contain vendor semantics:
//!
//! - no mode-number meanings (`6060h = 5` is CSP for one vendor, MIT for
//!   another — such constants belong in each vendor's own `RunMode` enum),
//! - no unit conversions (`pulses`-per-rev vs `Float32`-rev encodings are
//!   vendor facts),
//! - no object-dictionary entries from the manufacturer-specific `2xxx`
//!   region.
//!
//! The shared layer is *composed* by vendor motor structs (via
//! [`node::CanopenNode`]); it does not implement [`motor_core::MotorDevice`]
//! behavior — frame claiming policy and state caching stay in the vendor.
//!
//! Byte-level wire format reference (identical across the CANopen-family
//! manuals consulted so far): SDO §2.7, NMT §2.4, PDO §2.8, CiA402 state
//! machine §3.1.

pub mod cia402;
pub mod classify;
pub mod nmt;
pub mod node;
pub mod objects;
pub mod pdo;
pub mod scan;
pub mod sdo;

use motor_core::bus::{CanBus, CanFrame};
use motor_core::error::{MotorError, Result};

/// Send a classic (11-bit) CAN data frame with `payload` left-justified into
/// an 8-byte data field, as required by CANopen (fixed 8-byte SDO frames,
/// zero-padded unused bytes).
pub fn send_std_frame(
    bus: &dyn CanBus,
    arbitration_id: u32,
    payload: &[u8],
) -> Result<()> {
    if payload.len() > 8 {
        return Err(MotorError::InvalidArgument(format!(
            "payload too long: {}, expected <=8",
            payload.len()
        )));
    }
    let mut data = [0u8; 8];
    data[..payload.len()].copy_from_slice(payload);
    bus.send(CanFrame {
        arbitration_id,
        data,
        dlc: payload.len() as u8,
        is_extended: false,
        is_rx: false,
    })
}
