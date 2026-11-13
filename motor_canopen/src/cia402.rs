//! CiA402 drive state machine decoding and command procedures.
//!
//! Statusword (`6041h`) bits used by the state model:
//!
//! | bit | meaning |
//! |---|---|
//! | 0 | ready to switch on |
//! | 1 | switched on |
//! | 2 | operation enabled |
//! | 3 | fault |
//! | 5 | quick stop (0 = active) |
//! | 6 | switch on disabled |
//!
//! The table below lists the statusword values both CANopen-family manuals
//! use for the handshake examples — the decoder is bit-based, so
//! manufacturer-set don't-care bits (4, 7..15) do not affect the result:
//!
//! | state | example values |
//! |---|---|
//! | Switch On Disabled | `0x0250`, `0x0270` |
//! | Ready To Switch On | `0x0231` |
//! | Switched On | `0x0233` |
//! | Operation Enabled | `0x0237` |
//! | Quick Stop Active | `0x0007`, `0x0217` |
//! | Fault | `0x0218`, `0x0238` |

use motor_core::error::Result;

use crate::objects::{
    CONTROLWORD, CW_DISABLE_OPERATION, CW_DISABLE_VOLTAGE, CW_ENABLE_OPERATION, CW_FAULT_RESET,
    CW_QUICK_STOP, CW_SHUTDOWN, CW_SWITCH_ON, MODES_OF_OPERATION, MODES_OF_OPERATION_DISPLAY,
    STATUSWORD,
};
use crate::node::CanopenNode;

/// CiA402 drive state derived from `6041h` (statusword).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Cia402State {
    NotReadyToSwitchOn,
    SwitchOnDisabled,
    ReadyToSwitchOn,
    SwitchedOn,
    OperationEnabled,
    QuickStopActive,
    FaultReactionActive,
    Fault,
}

impl Cia402State {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::NotReadyToSwitchOn => "not_ready_to_switch_on",
            Self::SwitchOnDisabled => "switch_on_disabled",
            Self::ReadyToSwitchOn => "ready_to_switch_on",
            Self::SwitchedOn => "switched_on",
            Self::OperationEnabled => "operation_enabled",
            Self::QuickStopActive => "quick_stop_active",
            Self::FaultReactionActive => "fault_reaction_active",
            Self::Fault => "fault",
        }
    }
}

/// Decode a `6041h` statusword into a drive state, following the DS402
/// status table:
///
/// | state | bit6 | bit5 | bit3 | bit2 | bit1 | bit0 |
/// |---|---|---|---|---|---|---|
/// | Not Ready To Switch On | 0 | 0 | 0 | 0 | 0 | 0 |
/// | Switch On Disabled | 1 | x | 0 | 0 | 0 | 0 |
/// | Ready To Switch On | 0 | 1 | 0 | 0 | 0 | 1 |
/// | Switched On | 0 | 1 | 0 | 0 | 1 | 1 |
/// | Operation Enabled | 0 | 1 | 0 | 1 | 1 | 1 |
/// | Quick Stop Active | 0 | 0 | 0 | 1 | 1 | 1 |
/// | Fault (and Fault Reaction) | 0 | x | 1 | x | x | x |
///
/// Bit 3 (fault) dominates; the `Fault Reaction Active` transition is
/// indistinguishable from `Fault` on the statusword alone, so both map to
/// [`Cia402State::Fault`].
pub fn decode_statusword(statusword: u16) -> Cia402State {
    let fault = (statusword & 0x0008) != 0;
    let switch_on_disabled = (statusword & 0x0040) != 0;
    let operation_enabled = (statusword & 0x0004) != 0;
    let switched_on = (statusword & 0x0002) != 0;
    let ready_to_switch_on = (statusword & 0x0001) != 0;
    let quick_stop_inactive = (statusword & 0x0020) != 0;

    if fault {
        Cia402State::Fault
    } else if switch_on_disabled {
        Cia402State::SwitchOnDisabled
    } else {
        match (operation_enabled, switched_on, ready_to_switch_on) {
            (true, true, true) if quick_stop_inactive => Cia402State::OperationEnabled,
            (true, true, true) => Cia402State::QuickStopActive,
            (false, true, true) => Cia402State::SwitchedOn,
            (false, false, true) => Cia402State::ReadyToSwitchOn,
            _ => Cia402State::NotReadyToSwitchOn,
        }
    }
}

/// Write `6040h` (controlword).
pub fn write_controlword(node: &CanopenNode, command: u16, timeout: std::time::Duration) -> Result<()> {
    node.sdo_write_u16(CONTROLWORD, 0x00, command, timeout)
}

/// Read `6041h` (statusword).
pub fn read_statusword(node: &CanopenNode, timeout: std::time::Duration) -> Result<u16> {
    node.sdo_read_u16(STATUSWORD, 0x00, timeout)
}

/// Read the current drive state.
pub fn read_state(node: &CanopenNode, timeout: std::time::Duration) -> Result<Cia402State> {
    Ok(decode_statusword(read_statusword(node, timeout)?))
}

/// Write `6060h` (modes of operation). The mode *number's meaning* is a
/// vendor fact (`5` = CSP for one vendor, MIT for another) — the caller
/// passes its own `RunMode` value.
pub fn write_mode(node: &CanopenNode, mode: i8, timeout: std::time::Duration) -> Result<()> {
    node.sdo_write_i8(MODES_OF_OPERATION, 0x00, mode, timeout)
}

/// Read `6061h` (modes of operation display).
pub fn read_mode_display(node: &CanopenNode, timeout: std::time::Duration) -> Result<i8> {
    node.sdo_read_i8(MODES_OF_OPERATION_DISPLAY, 0x00, timeout)
}

/// `Shutdown → Switch On → Enable Operation` controlword sequence with a
/// 10 ms gap between commands (both CANopen-family manuals show this
/// handshake). Does **not** send NMT start — compose with
/// [`crate::node::CanopenNode::start_node`] in the vendor.
pub fn enable_sequence(node: &CanopenNode, timeout: std::time::Duration) -> Result<()> {
    for cw in [CW_SHUTDOWN, CW_SWITCH_ON, CW_ENABLE_OPERATION] {
        write_controlword(node, cw, timeout)?;
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
    Ok(())
}

/// `Disable Voltage` (`0x0000`) — falls back to Switch On Disabled, the
/// state where mode changes and homing/zero-position commands are accepted.
pub fn disable_drive(node: &CanopenNode, timeout: std::time::Duration) -> Result<()> {
    write_controlword(node, CW_DISABLE_VOLTAGE, timeout)
}

/// `Disable Operation` (`0x0007`) — keeps power stage ready, drops to
/// Switched On. Available for vendors whose stop semantics need it.
pub fn disable_operation(node: &CanopenNode, timeout: std::time::Duration) -> Result<()> {
    write_controlword(node, CW_DISABLE_OPERATION, timeout)
}

/// `Quick Stop` (`0x000B`).
pub fn quick_stop(node: &CanopenNode, timeout: std::time::Duration) -> Result<()> {
    write_controlword(node, CW_QUICK_STOP, timeout)
}

/// `Fault Reset` (`0x0080`) followed by `Disable Voltage` to clear the reset
/// command itself.
pub fn clear_fault(node: &CanopenNode, timeout: std::time::Duration) -> Result<()> {
    write_controlword(node, CW_FAULT_RESET, timeout)?;
    std::thread::sleep(std::time::Duration::from_millis(10));
    write_controlword(node, CW_DISABLE_VOLTAGE, timeout)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_manual_handshake_values() {
        assert_eq!(decode_statusword(0x0000), Cia402State::NotReadyToSwitchOn);
        assert_eq!(decode_statusword(0x0250), Cia402State::SwitchOnDisabled);
        assert_eq!(decode_statusword(0x0270), Cia402State::SwitchOnDisabled);
        assert_eq!(decode_statusword(0x0231), Cia402State::ReadyToSwitchOn);
        assert_eq!(decode_statusword(0x0233), Cia402State::SwitchedOn);
        assert_eq!(decode_statusword(0x0237), Cia402State::OperationEnabled);
        assert_eq!(decode_statusword(0x0007), Cia402State::QuickStopActive);
        assert_eq!(decode_statusword(0x0217), Cia402State::QuickStopActive);
        assert_eq!(decode_statusword(0x0218), Cia402State::Fault);
        assert_eq!(decode_statusword(0x0238), Cia402State::Fault);
    }

    #[test]
    fn fault_bit_dominates_other_bits() {
        // fault + operation-enabled pattern must still report Fault
        assert_eq!(decode_statusword(0x023F), Cia402State::Fault);
    }

    #[test]
    fn quick_stop_bit_overrides_enabled_pattern() {
        // operation enabled pattern but bit5 clear -> quick stop active
        assert_eq!(decode_statusword(0x0017), Cia402State::QuickStopActive);
    }
}
