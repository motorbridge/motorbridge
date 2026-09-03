use crate::protocol::{
    self, can_id_parts, encode_clear_errors, encode_config_save, encode_current_control,
    encode_param_read, encode_param_write, encode_pos_control, encode_set_zero,
    encode_torque_control, encode_vel_control, make_can_id, pack_mit_command, seq_next,
    unpack_mit_response, CyberBeastCanId, MitCommandParams, MsgType, Priority, ADDR_BROADCAST,
    DEFAULT_MASTER_ID, MAX_BROADCAST_DEVICES,
};
use motor_core::bus::{CanBus, CanFrame};
use motor_core::device::MotorDevice;
use motor_core::error::{MotorError, Result};
use motor_core::model::{ModelCatalog, MotorModelSpec, PvTLimits, StaticModelCatalog};
use std::collections::HashMap;
use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

// ============================================================================
// Model catalog
// ============================================================================

/// Common ODrive-based motor configurations for CyberBeast protocol.
///
/// These are typical setups. Users may need to adjust P/V/T limits for
/// their specific mechanical configuration (gear ratio, etc.).
const CYBERBEAST_MODELS: &[MotorModelSpec] = &[
    MotorModelSpec {
        vendor: "cyberbeast",
        model: "odrive-default",
        pmax: 4.0 * std::f32::consts::PI, // ±4π rad
        vmax: 100.0,                      // ±100 rad/s (output)
        tmax: 10.0,                       // ±10 Nm
    },
    MotorModelSpec {
        vendor: "cyberbeast",
        model: "odrive-pro",
        pmax: 4.0 * std::f32::consts::PI,
        vmax: 150.0,
        tmax: 20.0,
    },
    MotorModelSpec {
        vendor: "cyberbeast",
        model: "odrive-high-torque",
        pmax: 4.0 * std::f32::consts::PI,
        vmax: 60.0,
        tmax: 50.0,
    },
    MotorModelSpec {
        vendor: "cyberbeast",
        model: "odrive-high-speed",
        pmax: 4.0 * std::f32::consts::PI,
        vmax: 300.0,
        tmax: 5.0,
    },
];

const CYBERBEAST_CATALOG: StaticModelCatalog = StaticModelCatalog {
    vendor_name: "cyberbeast",
    models: CYBERBEAST_MODELS,
};

pub fn model_limits(model: &str) -> Option<(f32, f32, f32)> {
    CYBERBEAST_CATALOG
        .get(model)
        .map(|spec| (spec.pmax, spec.vmax, spec.tmax))
}

// ============================================================================
// Control mode
// ============================================================================

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControlMode {
    Mit = 0,
    Position = 1,
    Velocity = 2,
    Torque = 3,
    Current = 4,
}

// ============================================================================
// Parameter cache for SDO endpoint read/write
// ============================================================================

const DEFAULT_PARAM_TIMEOUT_MS: u64 = 200;
const PARAM_POLL_INTERVAL_MS: u64 = 2;

#[derive(Debug, Clone)]
struct ParamCache {
    /// Cached float values keyed by endpoint_id.
    values: HashMap<u16, f32>,
    /// Timestamp of last response for each endpoint_id.
    reply_time: HashMap<u16, Instant>,
    /// Write acknowledgment: endpoint_id → time of last ack.
    write_ack_time: HashMap<u16, Instant>,
    /// Pending read endpoint (if any).
    pending_read: Option<u16>,
    /// Pending write endpoint (if any).
    pending_write: Option<u16>,
}

impl ParamCache {
    fn new() -> Self {
        Self {
            values: HashMap::new(),
            reply_time: HashMap::new(),
            write_ack_time: HashMap::new(),
            pending_read: None,
            pending_write: None,
        }
    }

    fn record_read_response(&mut self, endpoint_id: u16, value: f32) {
        self.values.insert(endpoint_id, value);
        self.reply_time.insert(endpoint_id, Instant::now());
        self.pending_read = None;
    }

    fn record_write_ack(&mut self, endpoint_id: u16) {
        self.write_ack_time.insert(endpoint_id, Instant::now());
        self.pending_write = None;
    }
}

// ============================================================================
// Default MIT limits (ODrive CyberBeast defaults, per protocol v2.4)
// ============================================================================

const DEFAULT_MIT_POS_LIMIT: f32 = 4.0 * std::f32::consts::PI; // ±4π rad ≈ 12.566
const DEFAULT_MIT_VEL_LIMIT: f32 = 30.0; // ±30 rad/s
const DEFAULT_MIT_KP_LIMIT: f32 = 500.0; // max Kp (N·m/rad)
const DEFAULT_MIT_KD_LIMIT: f32 = 100.0; // max Kd (N·m·s/rad)
const DEFAULT_MIT_TORQUE_LIMIT: f32 = 18.0; // ±18 N·m
const DEFAULT_MIT_CURRENT_LIMIT: f32 = 40.0; // ± A (for response decoding)

// ============================================================================
// Motor state
// ============================================================================

#[derive(Debug, Clone, Copy)]
pub struct CyberBeastMotorState {
    /// The CAN arbitration ID this state was decoded from.
    pub arbitration_id: u32,
    /// Parsed CAN ID fields.
    pub can_id_parts: CyberBeastCanId,
    /// Position in radians (output side).
    pub pos: f32,
    /// Velocity in rad/s (output side).
    pub vel: f32,
    /// Motor current in Amps.
    pub current: f32,
    /// Error code from MIT response.
    pub error_code: u8,
    /// Mode/state from MIT response.
    pub mode_state: u8,
    /// Motor temperature in °C.
    pub motor_temp: f32,
    /// MOSFET temperature in °C.
    pub mos_temp: f32,
    /// Heartbeat error flags bitmask (0 if no heartbeat received).
    pub error_flags: u8,
    /// Life counter from last heartbeat frame.
    pub heartbeat_life: u8,
    /// Hardware version (raw uint32) from QUERY_DEVICE_INFO, if queried.
    pub hw_version: Option<u32>,
    /// Firmware version (raw uint32) from QUERY_DEVICE_INFO, if queried.
    pub fw_version: Option<u32>,
    /// Subsystem error value (raw uint32) from QUERY_ERROR, if queried.
    pub error_value: Option<u32>,
}

impl Default for CyberBeastMotorState {
    fn default() -> Self {
        Self {
            arbitration_id: 0,
            can_id_parts: CyberBeastCanId {
                priority: 0,
                msg_type: 0,
                dest: 0,
                source: 0,
                seq: 0,
                is_broadcast: false,
            },
            pos: 0.0,
            vel: 0.0,
            current: 0.0,
            error_code: 0,
            mode_state: 0,
            motor_temp: 0.0,
            mos_temp: 0.0,
            error_flags: 0,
            heartbeat_life: 0,
            hw_version: None,
            fw_version: None,
            error_value: None,
        }
    }
}

// ============================================================================
// CyberBeastMotor
// ============================================================================

pub struct CyberBeastMotor {
    /// CAN node ID of this motor device (destination for commands, source in responses).
    pub motor_id: u16,
    /// Master (host) CAN node ID. Used as source in outgoing frames.
    pub master_id: u8,
    /// Motor model string (must match a catalog entry).
    pub model: String,
    /// Shared CAN bus handle.
    bus: Arc<dyn CanBus>,
    /// Cached latest state from feedback/heartbeat frames.
    state: Mutex<Option<CyberBeastMotorState>>,
    /// Sequence number for outgoing commands (mod 4).
    tx_seq: AtomicU8,
    /// P/V/T limits derived from model catalog.
    #[allow(dead_code)]
    limits: PvTLimits,
    /// MIT position limit for encoding (rad).
    mit_pos_limit: f32,
    /// MIT velocity limit for encoding (rad/s).
    mit_vel_limit: f32,
    /// MIT Kp limit for encoding.
    mit_kp_limit: f32,
    /// MIT Kd limit for encoding.
    mit_kd_limit: f32,
    /// MIT torque limit for encoding (N·m).
    pub mit_torque_limit: f32,
    /// Current limit for response decoding (A).
    mit_current_limit: f32,
    /// Parameter cache for SDO endpoint read/write operations.
    param_cache: Mutex<ParamCache>,
}

impl CyberBeastMotor {
    pub fn new(
        motor_id: u16,
        _feedback_id: u16,
        model: &str,
        bus: Arc<dyn CanBus>,
    ) -> Result<Self> {
        let spec = CYBERBEAST_CATALOG.get(model).ok_or_else(|| {
            MotorError::InvalidArgument(format!("unknown cyberbeast model: {model}"))
        })?;

        Ok(Self {
            motor_id,
            master_id: DEFAULT_MASTER_ID,
            model: model.to_string(),
            bus,
            state: Mutex::new(None),
            tx_seq: AtomicU8::new(0),
            limits: PvTLimits::from_spec(spec),
            mit_pos_limit: DEFAULT_MIT_POS_LIMIT,
            mit_vel_limit: DEFAULT_MIT_VEL_LIMIT,
            mit_kp_limit: DEFAULT_MIT_KP_LIMIT,
            mit_kd_limit: DEFAULT_MIT_KD_LIMIT,
            mit_torque_limit: DEFAULT_MIT_TORQUE_LIMIT,
            mit_current_limit: DEFAULT_MIT_CURRENT_LIMIT,
            param_cache: Mutex::new(ParamCache::new()),
        })
    }

    pub fn with_master_id(mut self, master_id: u8) -> Self {
        self.master_id = master_id;
        self
    }

    pub fn set_master_id(&mut self, master_id: u8) {
        self.master_id = master_id;
    }

    pub fn latest_state(&self) -> Option<CyberBeastMotorState> {
        self.state.lock().ok().and_then(|s| *s)
    }

    /// Get or compute the next transmit sequence number.
    fn next_seq(&self) -> u8 {
        let seq = self.tx_seq.load(Ordering::Relaxed);
        self.tx_seq.store(seq_next(seq), Ordering::Relaxed);
        seq
    }

    /// Build a CAN ID for a command to this motor.
    fn cmd_can_id(&self, priority: Priority, msg_type: MsgType) -> u32 {
        make_can_id(
            priority as u8,
            msg_type as u8,
            self.motor_id as u8,
            self.master_id,
            self.next_seq(),
        )
    }

    /// Build a broadcast CAN ID (reserved for future CAN FD broadcast support).
    #[allow(dead_code)]
    fn bcast_can_id(&self, priority: Priority, msg_type: MsgType) -> u32 {
        // For broadcast, dest encodes a bitmask of target devices.
        // For full broadcast, use 0xFF.
        make_can_id(
            priority as u8,
            msg_type as u8,
            0xFF,
            self.master_id,
            self.next_seq(),
        )
    }

    /// Send a raw CAN frame with extended ID.
    fn send_ext(&self, arbitration_id: u32, data: [u8; 8]) -> Result<()> {
        self.bus.send(CanFrame {
            arbitration_id,
            data,
            dlc: 8,
            is_extended: true,
            is_rx: false,
        })
    }

    // ========================================================================
    // Command methods
    // ========================================================================

    /// Send MIT (force-position-velocity) control command.
    ///
    /// Parameters are in output-side units:
    /// - `pos`: target position (rad)
    /// - `vel`: target velocity (rad/s)
    /// - `kp`: position gain
    /// - `kd`: velocity damping gain
    /// - `torque`: feed-forward torque (N·m)
    pub fn send_mit_command(
        &self,
        pos: f32,
        vel: f32,
        kp: f32,
        kd: f32,
        torque: f32,
    ) -> Result<()> {
        let params = MitCommandParams {
            pos,
            vel,
            kp,
            kd,
            torque,
        };
        let data = pack_mit_command(
            &params,
            self.mit_pos_limit,
            self.mit_vel_limit,
            self.mit_kp_limit,
            self.mit_kd_limit,
            self.mit_torque_limit,
        );
        let can_id = self.cmd_can_id(Priority::HighCtrl, MsgType::MitControl);
        self.send_ext(can_id, data)
    }

    /// Send position control command (output-side units).
    ///
    /// - `target_pos_deg`: target position in degrees
    /// - `vel_limit_rpm`: velocity limit in RPM
    /// - `cur_limit_a`: current limit in Amps
    pub fn send_pos_control(
        &self,
        target_pos_deg: f32,
        vel_limit_rpm: f32,
        cur_limit_a: f32,
    ) -> Result<()> {
        let data = encode_pos_control(target_pos_deg, vel_limit_rpm, cur_limit_a);
        let can_id = self.cmd_can_id(Priority::Ctrl, MsgType::PosControl);
        self.send_ext(can_id, data)
    }

    /// Send velocity control command (output-side units).
    ///
    /// - `target_vel_rpm`: target velocity in RPM
    /// - `cur_limit_a`: current limit in Amps
    pub fn send_vel_control(&self, target_vel_rpm: f32, cur_limit_a: f32) -> Result<()> {
        let data = encode_vel_control(target_vel_rpm, cur_limit_a);
        let can_id = self.cmd_can_id(Priority::Ctrl, MsgType::VelControl);
        self.send_ext(can_id, data)
    }

    /// Send torque control command.
    ///
    /// - `target_torque_nm`: target torque in N·m
    pub fn send_torque_control(&self, target_torque_nm: f32) -> Result<()> {
        let data = encode_torque_control(target_torque_nm);
        let can_id = self.cmd_can_id(Priority::Ctrl, MsgType::TorqueControl);
        self.send_ext(can_id, data)
    }

    /// Send direct current control command.
    ///
    /// - `target_current_a`: target current in Amps
    pub fn send_current_control(&self, target_current_a: f32) -> Result<()> {
        let data = encode_current_control(target_current_a);
        let can_id = self.cmd_can_id(Priority::Ctrl, MsgType::CurrentControl);
        self.send_ext(can_id, data)
    }

    /// Send start motor command (enter closed-loop control).
    pub fn send_start_motor(&self) -> Result<()> {
        let can_id = self.cmd_can_id(Priority::Ctrl, MsgType::StartMotor);
        self.send_ext(can_id, [0u8; 8])
    }

    /// Send stop motor command (enter IDLE).
    pub fn send_stop_motor(&self) -> Result<()> {
        let can_id = self.cmd_can_id(Priority::Ctrl, MsgType::StopMotor);
        self.send_ext(can_id, [0u8; 8])
    }

    /// Send set-zero command (set current position as zero).
    pub fn send_set_zero(&self) -> Result<()> {
        let data = encode_set_zero();
        let can_id = self.cmd_can_id(Priority::Config, MsgType::SetZero);
        self.send_ext(can_id, data)
    }

    /// Send clear-errors command.
    pub fn send_clear_errors(&self) -> Result<()> {
        let data = encode_clear_errors();
        let can_id = self.cmd_can_id(Priority::Config, MsgType::ClearErrors);
        self.send_ext(can_id, data)
    }

    /// Send emergency stop.
    pub fn send_estop(&self) -> Result<()> {
        let can_id = self.cmd_can_id(Priority::Emergency, MsgType::Estop);
        self.send_ext(can_id, [0u8; 8])
    }

    /// Send status query (requests MIT response).
    pub fn send_query_status(&self) -> Result<()> {
        let can_id = self.cmd_can_id(Priority::Query, MsgType::QueryStatus);
        self.send_ext(can_id, [0u8; 8])
    }

    /// Send position+velocity query.
    pub fn send_query_pos_vel(&self) -> Result<()> {
        let can_id = self.cmd_can_id(Priority::Query, MsgType::QueryPosVel);
        self.send_ext(can_id, [0u8; 8])
    }

    /// Send current query (Iq + Id).
    pub fn send_query_current(&self) -> Result<()> {
        let can_id = self.cmd_can_id(Priority::Query, MsgType::QueryCurrent);
        self.send_ext(can_id, [0u8; 8])
    }

    /// Send temperature query (motor + FET temp).
    pub fn send_query_temperature(&self) -> Result<()> {
        let can_id = self.cmd_can_id(Priority::Query, MsgType::QueryTemperature);
        self.send_ext(can_id, [0u8; 8])
    }

    /// Send bus voltage/current query.
    pub fn send_query_bus(&self) -> Result<()> {
        let can_id = self.cmd_can_id(Priority::Query, MsgType::QueryBus);
        self.send_ext(can_id, [0u8; 8])
    }

    /// Send device info query (hardware + firmware version).
    pub fn send_query_device_info(&self) -> Result<()> {
        let can_id = self.cmd_can_id(Priority::Query, MsgType::QueryDeviceInfo);
        self.send_ext(can_id, [0u8; 8])
    }

    /// Send detailed error query for a subsystem.
    pub fn send_query_error(&self, err_type: u8) -> Result<()> {
        let data = protocol::encode_query_error(err_type);
        let can_id = self.cmd_can_id(Priority::Query, MsgType::QueryError);
        self.send_ext(can_id, data)
    }

    /// Request an immediate status feedback (device responds with MIT frame).
    pub fn send_status_feedback(&self) -> Result<()> {
        let can_id = self.cmd_can_id(Priority::Query, MsgType::StatusFeedback);
        self.send_ext(can_id, [0u8; 8])
    }

    /// Send config reset command (erase config and reboot).
    pub fn send_config_reset(&self) -> Result<()> {
        let data = protocol::encode_config_reset();
        let can_id = self.cmd_can_id(Priority::Config, MsgType::ConfigReset);
        self.send_ext(can_id, data)
    }

    // ========================================================================
    // Parameter (SDO endpoint) access
    // ========================================================================

    /// Send a parameter read request for an ODrive SDO endpoint.
    pub fn send_param_read(&self, endpoint_id: u16) -> Result<()> {
        {
            let mut cache = self
                .param_cache
                .lock()
                .map_err(|_| MotorError::Io("param cache lock poisoned".into()))?;
            cache.pending_read = Some(endpoint_id);
        }
        let data = encode_param_read(endpoint_id);
        let can_id = self.cmd_can_id(Priority::Config, MsgType::ParamRead);
        self.send_ext(can_id, data)
    }

    /// Send a parameter write request for an ODrive SDO endpoint.
    pub fn send_param_write(&self, endpoint_id: u16, value: f32) -> Result<()> {
        {
            let mut cache = self
                .param_cache
                .lock()
                .map_err(|_| MotorError::Io("param cache lock poisoned".into()))?;
            cache.pending_write = Some(endpoint_id);
        }
        let data = encode_param_write(endpoint_id, value);
        let can_id = self.cmd_can_id(Priority::Config, MsgType::ParamWrite);
        self.send_ext(can_id, data)
    }

    /// Read a parameter value, blocking until response or timeout.
    pub fn get_param_f32(&self, endpoint_id: u16, timeout: Duration) -> Result<f32> {
        self.wait_for_param(endpoint_id, timeout)
    }

    /// Write a parameter value and wait for acknowledgment.
    pub fn set_param_f32(&self, endpoint_id: u16, value: f32) -> Result<()> {
        self.send_param_write(endpoint_id, value)?;
        self.wait_for_write_ack(endpoint_id, Duration::from_millis(DEFAULT_PARAM_TIMEOUT_MS))
    }

    /// Store parameters to flash (CONFIG_SAVE).
    pub fn store_parameters(&self) -> Result<()> {
        let data = encode_config_save();
        let can_id = self.cmd_can_id(Priority::Config, MsgType::ConfigSave);
        self.send_ext(can_id, data)
    }

    /// Block until a param read response arrives or timeout.
    fn wait_for_param(&self, endpoint_id: u16, timeout: Duration) -> Result<f32> {
        let deadline = Instant::now() + timeout;
        loop {
            {
                let cache = self
                    .param_cache
                    .lock()
                    .map_err(|_| MotorError::Io("param cache lock poisoned".into()))?;
                if let Some(ts) = cache.reply_time.get(&endpoint_id) {
                    if *ts > Instant::now() - timeout {
                        // Fresh value
                        if let Some(val) = cache.values.get(&endpoint_id) {
                            return Ok(*val);
                        }
                    }
                }
            }
            if Instant::now() > deadline {
                return Err(MotorError::Timeout(format!(
                    "timeout waiting for param read 0x{endpoint_id:04X}",
                )));
            }
            std::thread::sleep(Duration::from_millis(PARAM_POLL_INTERVAL_MS));
        }
    }

    /// Block until a param write ack arrives or timeout.
    fn wait_for_write_ack(&self, endpoint_id: u16, timeout: Duration) -> Result<()> {
        let deadline = Instant::now() + timeout;
        loop {
            {
                let cache = self
                    .param_cache
                    .lock()
                    .map_err(|_| MotorError::Io("param cache lock poisoned".into()))?;
                if let Some(ts) = cache.write_ack_time.get(&endpoint_id) {
                    if *ts > Instant::now() - timeout {
                        return Ok(());
                    }
                }
            }
            if Instant::now() > deadline {
                return Err(MotorError::Timeout(format!(
                    "timeout waiting for param write ack 0x{endpoint_id:04X}",
                )));
            }
            std::thread::sleep(Duration::from_millis(PARAM_POLL_INTERVAL_MS));
        }
    }

    // ========================================================================
    // Feedback processing
    // ========================================================================

    /// Process an incoming frame: decode MIT response, heartbeat, or other feedback.
    fn process_feedback_frame_impl(&self, frame: CanFrame) -> Result<()> {
        let parts = can_id_parts(frame.arbitration_id);

        match parts.msg_type {
            // MIT response: reused MIT control msg type from motor → host
            t if t == MsgType::MitControl as u8 => {
                let resp = unpack_mit_response(
                    &frame.data,
                    self.mit_pos_limit,
                    self.mit_vel_limit,
                    self.mit_current_limit,
                );

                let mut state = self
                    .state
                    .lock()
                    .map_err(|_| MotorError::Io("state lock poisoned".into()))?;
                let existing = state.unwrap_or_default();

                state.replace(CyberBeastMotorState {
                    arbitration_id: frame.arbitration_id,
                    can_id_parts: parts,
                    pos: resp.pos,
                    vel: resp.vel,
                    current: resp.current,
                    error_code: resp.error_code,
                    mode_state: resp.mode_state,
                    motor_temp: resp.motor_temp,
                    mos_temp: resp.mos_temp,
                    error_flags: existing.error_flags,
                    heartbeat_life: existing.heartbeat_life,
                    hw_version: existing.hw_version,
                    fw_version: existing.fw_version,
                    error_value: existing.error_value,
                });
            }

            // Heartbeat
            t if t == MsgType::Heartbeat as u8 => {
                if let Some(hb) = protocol::decode_heartbeat(&frame.data) {
                    let mut state = self
                        .state
                        .lock()
                        .map_err(|_| MotorError::Io("state lock poisoned".into()))?;
                    let existing = state.unwrap_or_default();

                    state.replace(CyberBeastMotorState {
                        arbitration_id: frame.arbitration_id,
                        can_id_parts: parts,
                        // Heartbeat now provides position, velocity, current,
                        // temperature, error flags, and life counter all in one frame.
                        // Position/velocity are in motor turns — convert to output rad.
                        pos: hb.position_turns * (2.0 * std::f32::consts::PI),
                        vel: hb.velocity_turns_per_s * (2.0 * std::f32::consts::PI),
                        current: hb.iq_current,
                        error_flags: hb.error_flags,
                        motor_temp: hb.motor_temp,
                        mos_temp: existing.mos_temp,
                        heartbeat_life: hb.life_counter,
                        ..existing
                    });
                }
            }

            // QUERY_POS_VEL response
            t if t == MsgType::QueryPosVel as u8 => {
                let (pos, vel) = protocol::decode_pos_vel_response(&frame.data);
                let mut state = self
                    .state
                    .lock()
                    .map_err(|_| MotorError::Io("state lock poisoned".into()))?;
                let existing = state.unwrap_or_default();
                state.replace(CyberBeastMotorState {
                    arbitration_id: frame.arbitration_id,
                    can_id_parts: parts,
                    pos,
                    vel,
                    ..existing
                });
            }

            // QUERY_CURRENT response
            t if t == MsgType::QueryCurrent as u8 => {
                let (iq, _id) = protocol::decode_current_response(&frame.data);
                let mut state = self
                    .state
                    .lock()
                    .map_err(|_| MotorError::Io("state lock poisoned".into()))?;
                let existing = state.unwrap_or_default();
                state.replace(CyberBeastMotorState {
                    arbitration_id: frame.arbitration_id,
                    can_id_parts: parts,
                    current: iq,
                    ..existing
                });
            }

            // QUERY_TEMPERATURE response
            t if t == MsgType::QueryTemperature as u8 => {
                let (motor_temp, mos_temp) = protocol::decode_temperature_response(&frame.data);
                let mut state = self
                    .state
                    .lock()
                    .map_err(|_| MotorError::Io("state lock poisoned".into()))?;
                let existing = state.unwrap_or_default();
                state.replace(CyberBeastMotorState {
                    arbitration_id: frame.arbitration_id,
                    can_id_parts: parts,
                    motor_temp,
                    mos_temp,
                    ..existing
                });
            }

            // QUERY_BUS response
            t if t == MsgType::QueryBus as u8 => {
                // Bus voltage/current — stored in state for now (could add dedicated fields)
                let _ = protocol::decode_bus_response(&frame.data);
            }

            // QUERY_DEVICE_INFO response (Classic CAN: HW + FW version)
            t if t == MsgType::QueryDeviceInfo as u8 => {
                if let Some(info) = protocol::decode_device_info_response(&frame.data) {
                    let mut state = self
                        .state
                        .lock()
                        .map_err(|_| MotorError::Io("state lock poisoned".into()))?;
                    let existing = state.unwrap_or_default();
                    state.replace(CyberBeastMotorState {
                        arbitration_id: frame.arbitration_id,
                        can_id_parts: parts,
                        hw_version: Some(info.hw_version),
                        fw_version: Some(info.fw_version),
                        ..existing
                    });
                }
            }

            // QUERY_ERROR response: Byte 0=type echo, Byte 4-7=error value (uint32 BE)
            t if t == MsgType::QueryError as u8 => {
                if frame.data.len() >= 8 {
                    let value = ((frame.data[4] as u32) << 24)
                        | ((frame.data[5] as u32) << 16)
                        | ((frame.data[6] as u32) << 8)
                        | (frame.data[7] as u32);
                    let mut state = self
                        .state
                        .lock()
                        .map_err(|_| MotorError::Io("state lock poisoned".into()))?;
                    let existing = state.unwrap_or_default();
                    state.replace(CyberBeastMotorState {
                        arbitration_id: frame.arbitration_id,
                        can_id_parts: parts,
                        error_value: Some(value),
                        ..existing
                    });
                }
            }

            // PARAM_READ response
            t if t == MsgType::ParamRead as u8 => {
                // Parse endpoint_id from response
                if frame.data.len() >= 3 {
                    let endpoint_id = ((frame.data[1] as u16) << 8) | (frame.data[2] as u16);
                    if let Some(val) =
                        protocol::decode_param_read_response(&frame.data, endpoint_id)
                    {
                        let mut cache = self
                            .param_cache
                            .lock()
                            .map_err(|_| MotorError::Io("param cache lock poisoned".into()))?;
                        cache.record_read_response(endpoint_id, val);
                    }
                }
            }

            // PARAM_WRITE acknowledgment
            t if t == MsgType::ParamWrite as u8 => {
                if frame.data.len() >= 3 {
                    let endpoint_id = ((frame.data[1] as u16) << 8) | (frame.data[2] as u16);
                    if protocol::is_param_write_ack(&frame.data, endpoint_id) {
                        let mut cache = self
                            .param_cache
                            .lock()
                            .map_err(|_| MotorError::Io("param cache lock poisoned".into()))?;
                        cache.record_write_ack(endpoint_id);
                    }
                }
            }

            _ => {
                // Unknown response — ignore
            }
        }

        Ok(())
    }

    /// Check if a broadcast CAN frame targets this motor.
    ///
    /// Protocol v2.4 broadcast semantics:
    /// - `Dest=0xFF`: global broadcast → reaches ALL devices (no bitmap limit)
    /// - `Dest=bitmap`: multicast to Device#0~7 via 8-bit bitmap (bit0=Dev#0)
    /// - In Classic CAN, MIT broadcast applies the same 8-byte command to every
    ///   bitmap-matched device (no per-device slots, which require CAN FD).
    fn is_broadcast_slot_for_me(&self, parts: &CyberBeastCanId) -> bool {
        let device_id = self.motor_id as u8;
        if device_id == 0 {
            return false;
        }
        // Dest=0xFF is global broadcast — every device responds/applies.
        if parts.dest == ADDR_BROADCAST {
            return true;
        }
        // Bitmap multicast only addresses Device#0~7.
        if device_id >= MAX_BROADCAST_DEVICES {
            return false;
        }
        (parts.dest & (1 << device_id)) != 0
    }
}

// ============================================================================
// MotorDevice trait impl
// ============================================================================

impl MotorDevice for CyberBeastMotor {
    fn vendor(&self) -> &'static str {
        "cyberbeast"
    }

    fn model(&self) -> &str {
        &self.model
    }

    fn motor_id(&self) -> u16 {
        self.motor_id
    }

    fn feedback_id(&self) -> u16 {
        // In CyberBeast protocol, feedback frames use the motor's node_id
        // as the source address. Same as motor_id.
        self.motor_id
    }

    fn enable(&self) -> Result<()> {
        self.send_start_motor()
    }

    fn disable(&self) -> Result<()> {
        self.send_stop_motor()
    }

    fn accepts_frame(&self, frame: &CanFrame) -> bool {
        if !frame.is_rx {
            return false;
        }
        let parts = can_id_parts(frame.arbitration_id);

        // Point-to-point: source matches our motor_id
        if !parts.is_broadcast {
            return parts.source == self.motor_id as u8;
        }

        // Broadcast: check bitmask in dest field
        self.is_broadcast_slot_for_me(&parts)
    }

    fn process_feedback_frame(&self, frame: CanFrame) -> Result<()> {
        self.process_feedback_frame_impl(frame)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use motor_core::bus::{CanBus, CanFrame};
    use motor_core::test_support::MockBus;
    use std::sync::Arc;

    fn make_motor() -> CyberBeastMotor {
        let bus: Arc<dyn CanBus> = Arc::new(MockBus::new());
        CyberBeastMotor::new(0x01, 0x01, "odrive-default", bus).unwrap()
    }

    #[test]
    fn test_model_catalog() {
        let spec = CYBERBEAST_CATALOG.get("odrive-default").unwrap();
        assert_eq!(spec.vendor, "cyberbeast");
        assert_eq!(spec.model, "odrive-default");
    }

    #[test]
    fn test_unknown_model_rejected() {
        let bus: Arc<dyn CanBus> = Arc::new(MockBus::new());
        let result = CyberBeastMotor::new(0x01, 0x01, "nonexistent", bus);
        assert!(result.is_err());
    }

    #[test]
    fn test_can_id_roundtrip() {
        let id = make_can_id(2, 0x00, 0x01, 0x0A, 1);
        let parts = can_id_parts(id);
        assert_eq!(parts.priority, 2);
        assert_eq!(parts.msg_type, 0x00); // MIT_CONTROL
        assert_eq!(parts.dest, 0x01);
        assert_eq!(parts.source, 0x0A);
        assert_eq!(parts.seq, 1);
        assert!(!parts.is_broadcast);
    }

    #[test]
    fn test_broadcast_detection() {
        let id = make_can_id(2, 0x80, 0xFF, 0x0A, 0);
        let parts = can_id_parts(id);
        assert!(parts.is_broadcast);
    }

    #[test]
    fn test_mit_pack_unpack_roundtrip() {
        let params = MitCommandParams {
            pos: 1.5,
            vel: 10.0,
            kp: 100.0,
            kd: 5.0,
            torque: 0.5,
        };
        let packed = pack_mit_command(&params, 12.5, 50.0, 500.0, 100.0, 10.0);
        let unpacked = protocol::unpack_mit_command(&packed, 12.5, 50.0, 500.0, 100.0, 10.0);

        // Due to quantization, compare with tolerance
        assert!((unpacked.pos - 1.5).abs() < 0.01);
        assert!((unpacked.vel - 10.0).abs() < 0.1);
        assert!((unpacked.kp - 100.0).abs() < 1.0);
        assert!((unpacked.kd - 5.0).abs() < 0.1);
        assert!((unpacked.torque - 0.5).abs() < 0.01);
    }

    #[test]
    fn test_mit_response_decode() {
        // Verify decode of zero-filled MIT response doesn't panic and returns defaults
        let resp = unpack_mit_response(&[0u8; 8], 12.5, 50.0, 40.0);
        assert_eq!(resp.error_code, 0);
        assert_eq!(resp.mode_state, 0);
    }

    #[test]
    fn test_accepts_frame() {
        let motor = make_motor();
        // Frame from our motor (source == motor_id)
        let id = make_can_id(2, 0x00, 0x0A, 0x01, 0); // source=0x01 matches motor_id=1
        let frame = CanFrame {
            arbitration_id: id,
            data: [0u8; 8],
            dlc: 8,
            is_extended: true,
            is_rx: true,
        };
        assert!(motor.accepts_frame(&frame));

        // Frame from a different motor
        let id2 = make_can_id(2, 0x00, 0x0A, 0x02, 0); // source=0x02
        let frame2 = CanFrame {
            arbitration_id: id2,
            data: [0u8; 8],
            dlc: 8,
            is_extended: true,
            is_rx: true,
        };
        assert!(!motor.accepts_frame(&frame2));

        // TX frame should not be accepted
        let frame3 = CanFrame {
            arbitration_id: id,
            data: [0u8; 8],
            dlc: 8,
            is_extended: true,
            is_rx: false,
        };
        assert!(!motor.accepts_frame(&frame3));
    }

    #[test]
    fn test_send_mit_command_encodes_correct_frame() {
        let mock_bus: Arc<MockBus> = Arc::new(MockBus::new());
        let bus: Arc<dyn CanBus> = Arc::clone(&mock_bus) as Arc<dyn CanBus>;
        let motor = CyberBeastMotor::new(0x01, 0x01, "odrive-default", bus).unwrap();

        motor.send_mit_command(1.0, 0.5, 100.0, 10.0, 0.2).unwrap();

        let sent: Vec<CanFrame> = mock_bus.sent.lock().unwrap().drain(..).collect();
        assert_eq!(sent.len(), 1);
        let frame = &sent[0];
        assert!(frame.is_extended);
        assert!(!frame.is_rx);

        // Verify CAN ID: Priority=2(HighCtrl), MsgType=0x00(MIT), dest=0x01, source=0x01
        let parts = can_id_parts(frame.arbitration_id);
        assert_eq!(parts.priority, Priority::HighCtrl as u8);
        assert_eq!(parts.msg_type, MsgType::MitControl as u8);
        assert_eq!(parts.dest, 0x01);
        assert_eq!(parts.source, DEFAULT_MASTER_ID);
    }

    #[test]
    fn test_mit_response_process_updates_state() {
        let bus: Arc<dyn CanBus> = Arc::new(MockBus::new());
        let motor = CyberBeastMotor::new(0x02, 0x02, "odrive-default", Arc::clone(&bus)).unwrap();

        // Build a mock MIT response frame
        // pos=1.0 rad, vel=2.0 rad/s, current=3.0 A, error=0, mode=4(MIT)
        let params = MitCommandParams {
            pos: 1.0,
            vel: 2.0,
            kp: 0.0,
            kd: 0.0,
            torque: 3.0, // Using torque value to represent current in packed form
        };
        let packed = pack_mit_command(&params, 12.5, 50.0, 500.0, 100.0, 40.0);

        let can_id = make_can_id(2, MsgType::MitControl as u8, 0x0A, 0x02, 0);
        let frame = CanFrame {
            arbitration_id: can_id,
            data: packed,
            dlc: 8,
            is_extended: true,
            is_rx: true,
        };

        motor.process_feedback_frame(frame).unwrap();

        let state = motor.latest_state().unwrap();
        // With packed MIT command layout mapped to response decode, values won't match
        // exactly, but state should be populated
        assert!(state.pos != 0.0 || state.vel != 0.0 || state.current != 0.0);
        assert_eq!(state.can_id_parts.source, 0x02);
    }

    #[test]
    fn test_heartbeat_decode_updates_state() {
        let bus: Arc<dyn CanBus> = Arc::new(MockBus::new());
        let motor = CyberBeastMotor::new(0x03, 0x03, "odrive-default", Arc::clone(&bus)).unwrap();

        // Build a mock heartbeat frame (new v8.1 format)
        // Life=2 (0b010xxxxx), ErrorFlags=0x03 (AXIS|MOTOR)
        // State=0x3 (CLOSED_LOOP), Mode=0x1 (POSITION)
        // Motor Temp=80 (raw) → actual 30°C
        // Position=5000 (raw int16) → 50.0 turns
        // Velocity=200 (raw int16) → 2.0 turns/s
        // Iq=20 (raw int8) → 10.0 A
        let mut data = [0u8; 8];
        data[0] = (2 << 5) | 0x03; // Life=2, ErrorFlags=AXIS|MOTOR
        data[1] = (0x3 << 4) | 0x1; // State=3, ControlMode=1
        data[2] = 80; // Motor Temp = 30°C
        let pos_raw: i16 = 5000;
        data[3] = (pos_raw >> 8) as u8;
        data[4] = pos_raw as u8;
        let vel_raw: i16 = 200;
        data[5] = (vel_raw >> 8) as u8;
        data[6] = vel_raw as u8;
        data[7] = 20; // Iq = 10.0 A

        let can_id = make_can_id(6, MsgType::Heartbeat as u8, 0x01, 0x03, 0);
        let frame = CanFrame {
            arbitration_id: can_id,
            data,
            dlc: 8,
            is_extended: true,
            is_rx: true,
        };

        motor.process_feedback_frame(frame).unwrap();

        let state = motor.latest_state().unwrap();
        assert_eq!(state.heartbeat_life, 2);
        assert_eq!(state.error_flags, 0x03); // AXIS|MOTOR
        assert!((state.motor_temp - 30.0).abs() < 1.0);
        // Position: 50.0 turns → output rad = 50.0 * 2π
        assert!((state.pos - 50.0 * 2.0 * std::f32::consts::PI).abs() < 1.0);
        // Velocity: 2.0 turns/s → output rad/s = 2.0 * 2π
        assert!((state.vel - 2.0 * 2.0 * std::f32::consts::PI).abs() < 1.0);
        assert!((state.current - 10.0).abs() < 0.5);
    }

    #[test]
    fn test_process_mit_response_updates_core_fields() {
        let bus: Arc<dyn CanBus> = Arc::new(MockBus::new());
        let motor = CyberBeastMotor::new(0x04, 0x04, "odrive-default", Arc::clone(&bus)).unwrap();

        // Build a proper MIT response (different layout from MIT command)
        // Response: pos(16-bit), vel(12-bit)+error(4-bit), current(12-bit)+mode(4-bit), motor_temp, mos_temp
        // Use mid-range values: pos=0x8000, vel=0x0800, error=0, current=0x0800, mode=3
        let buf: [u8; 8] = [
            0x80, 0x00, // pos mid-range
            0x80, 0x00, // vel mid-range + error=0
            0x80, 0x03, // current mid-range + mode=3 (CLOSED_LOOP)
            0x6E, // motor_temp=110 → actual = 60°C
            0x6E, // mos_temp=110 → actual = 60°C
        ];

        let can_id = make_can_id(2, MsgType::MitControl as u8, 0x01, 0x04, 0);
        let frame = CanFrame {
            arbitration_id: can_id,
            data: buf,
            dlc: 8,
            is_extended: true,
            is_rx: true,
        };

        motor.process_feedback_frame(frame).unwrap();

        let state = motor.latest_state().unwrap();
        assert_eq!(state.mode_state, 3); // CLOSED_LOOP
        assert_eq!(state.error_code, 0);
        assert!((state.motor_temp - 60.0).abs() < 1.0);
        assert!((state.mos_temp - 60.0).abs() < 1.0);
    }
}
