//! CyberBeast register table.
//!
//! The CyberBeast protocol uses ODrive's SDO endpoint system for parameter
//! access (via `MSG_PARAM_READ` / `MSG_PARAM_WRITE` with 16-bit endpoint IDs).
//! This is different from traditional fixed-register protocols like Damiao or
//! RobStride — the full parameter map is discovered at runtime via
//! `MSG_JSON_DESC_READ` (endpoint descriptor JSON).
//!
//! This module provides a minimal static table of commonly-used endpoint IDs
//! for documentation and tooling. For complete parameter access, use the
//! dynamic JSON descriptor mechanism.

#[derive(Debug, Clone, Copy)]
pub struct RegisterInfo {
    /// SDO endpoint ID (16-bit).
    pub endpoint_id: u16,
    /// Human-readable variable name.
    pub variable: &'static str,
    /// Description.
    pub description: &'static str,
}

/// Commonly-used ODrive endpoint IDs for CyberBeast protocol.
///
/// These are the most frequently accessed parameters. The full endpoint map
/// (hundreds of entries) is obtained at runtime via `MSG_JSON_DESC_READ`.
pub static REGISTER_TABLE: &[RegisterInfo] = &[
    // ── Axis state ──
    RegisterInfo {
        endpoint_id: 0x0001,
        variable: "axis.requested_state",
        description: "Requested axis state (AXIS_STATE_*)",
    },
    RegisterInfo {
        endpoint_id: 0x0012,
        variable: "axis.error",
        description: "Axis error flags",
    },
    RegisterInfo {
        endpoint_id: 0x0000,
        variable: "axis.current_state",
        description: "Current axis state",
    },

    // ── Motor ──
    RegisterInfo {
        endpoint_id: 0x0019,
        variable: "motor.config.torque_constant",
        description: "Motor torque constant (Nm/A)",
    },
    RegisterInfo {
        endpoint_id: 0x001D,
        variable: "motor.error",
        description: "Motor error flags",
    },
    RegisterInfo {
        endpoint_id: 0x001C,
        variable: "motor.config.current_lim",
        description: "Motor current limit (A)",
    },

    // ── Encoder ──
    RegisterInfo {
        endpoint_id: 0x002C,
        variable: "encoder.error",
        description: "Encoder error flags",
    },

    // ── Controller ──
    RegisterInfo {
        endpoint_id: 0x0030,
        variable: "controller.config.control_mode",
        description: "Control mode (POSITION/VELOCITY/TORQUE)",
    },
    RegisterInfo {
        endpoint_id: 0x0031,
        variable: "controller.config.input_mode",
        description: "Input mode (PASSTHROUGH/POS_FILTER/VEL_RAMP/TORQUE_RAMP/MIT)",
    },
    RegisterInfo {
        endpoint_id: 0x0035,
        variable: "controller.config.pos_gain",
        description: "Position gain (turns/s per turn)",
    },
    RegisterInfo {
        endpoint_id: 0x0036,
        variable: "controller.config.vel_gain",
        description: "Velocity gain (Nm per turn/s)",
    },
    RegisterInfo {
        endpoint_id: 0x0037,
        variable: "controller.config.vel_integrator_gain",
        description: "Velocity integrator gain (Nm per turn)",
    },
    RegisterInfo {
        endpoint_id: 0x0039,
        variable: "controller.config.vel_limit",
        description: "Velocity limit (turns/s)",
    },
    RegisterInfo {
        endpoint_id: 0x003D,
        variable: "controller.error",
        description: "Controller error flags",
    },

    // ── MIT limits ──
    RegisterInfo {
        endpoint_id: 0x0300,
        variable: "controller.config.mit_max_pos",
        description: "MIT max position (rad)",
    },
    RegisterInfo {
        endpoint_id: 0x0301,
        variable: "controller.config.mit_max_vel",
        description: "MIT max velocity (rad/s)",
    },
    RegisterInfo {
        endpoint_id: 0x0302,
        variable: "controller.config.mit_max_kp",
        description: "MIT max Kp",
    },
    RegisterInfo {
        endpoint_id: 0x0303,
        variable: "controller.config.mit_max_kd",
        description: "MIT max Kd",
    },
    RegisterInfo {
        endpoint_id: 0x0304,
        variable: "controller.config.mit_max_torque",
        description: "MIT max torque (Nm)",
    },

    // ── CAN config ──
    RegisterInfo {
        endpoint_id: 0x0100,
        variable: "can.config.node_id",
        description: "CAN node ID",
    },
    RegisterInfo {
        endpoint_id: 0x0101,
        variable: "can.config.break_timeout",
        description: "CAN break timeout (ms, 0 = default 100ms)",
    },
];
