//! Huayi vendor-private FFI: SDO typed access, the compressed-MIT fast
//! path, position preset, periodic (TPDO) report control, diagnostics,
//! node-id and NMT management, and node scanning.
//!
//! Design boundary (plan §4.4): the generic `motor_handle_*_register`
//! u8-rid interface cannot address 16-bit CANopen indices, so this module
//! is the canonical register interface for Huayi — the register functions
//! reject Huayi motors with a pointer here.

use crate::{ensure_huayi_controller, ffi_rc, set_last_error, MotorController, MotorHandle, MotorHandleInner};
use motor_canopen::nmt::NmtCommand;
use std::time::Duration;

const DIAG_TIMEOUT: Duration = Duration::from_millis(500);

fn require_huayi(motor: &MotorHandleInner) -> Result<&motor_vendor_huayi::HuayiMotor, String> {
    match motor {
        MotorHandleInner::Huayi(m) => Ok(m),
        _ => Err("this operation requires a Huayi motor".to_string()),
    }
}

// ---------------------------------------------------------------------------
// SDO typed access (16-bit index + 8-bit sub)
// ---------------------------------------------------------------------------

macro_rules! huayi_sdo_read {
    ($fn_name:ident, $ty:ty, $method:ident) => {
        #[unsafe(no_mangle)]
        pub extern "C" fn $fn_name(
            motor: *mut MotorHandle,
            index: u16,
            sub: u8,
            timeout_ms: u32,
            out_value: *mut $ty,
        ) -> i32 {
            if motor.is_null() || out_value.is_null() {
                set_last_error("motor or out_value is null");
                return -1;
            }
            let motor = lock_motor_inner!(motor, "motor is null");
            let out = unsafe { &mut *out_value };
            let rc = require_huayi(&motor)
                .and_then(|m| {
                    m.$method(index, sub, Duration::from_millis(timeout_ms as u64))
                        .map_err(|e| e.to_string())
                })
                .map(|v| *out = v);
            ffi_rc(rc)
        }
    };
}

huayi_sdo_read!(motor_handle_huayi_sdo_read_u16, u16, sdo_read_u16);
huayi_sdo_read!(motor_handle_huayi_sdo_read_i16, i16, sdo_read_i16);
huayi_sdo_read!(motor_handle_huayi_sdo_read_i32, i32, sdo_read_i32);
huayi_sdo_read!(motor_handle_huayi_sdo_read_f32, f32, sdo_read_f32);

macro_rules! huayi_sdo_write {
    ($fn_name:ident, $ty:ty, $method:ident) => {
        #[unsafe(no_mangle)]
        pub extern "C" fn $fn_name(
            motor: *mut MotorHandle,
            index: u16,
            sub: u8,
            value: $ty,
        ) -> i32 {
            if motor.is_null() {
                set_last_error("motor is null");
                return -1;
            }
            let motor = lock_motor_inner!(motor, "motor is null");
            let rc = require_huayi(&motor).and_then(|m| {
                m.$method(index, sub, value, Duration::from_millis(300))
                    .map_err(|e| e.to_string())
            });
            ffi_rc(rc)
        }
    };
}

huayi_sdo_write!(motor_handle_huayi_sdo_write_u16, u16, sdo_write_u16);
huayi_sdo_write!(motor_handle_huayi_sdo_write_i16, i16, sdo_write_i16);
huayi_sdo_write!(motor_handle_huayi_sdo_write_i32, i32, sdo_write_i32);
huayi_sdo_write!(motor_handle_huayi_sdo_write_f32, f32, sdo_write_f32);

// ---------------------------------------------------------------------------
// control: torque, compressed MIT, position preset
// ---------------------------------------------------------------------------

/// Torque mode (6060h=4): `6071h` Int16 in ‰ of the 6076h peak. `torque_nm`
/// is the unified SI value; conversion (and peak caching) is in the vendor
/// layer. Huayi motors only.
#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_huayi_send_torque(motor: *mut MotorHandle, torque_nm: f32) -> i32 {
    if motor.is_null() {
        set_last_error("motor is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let rc = require_huayi(&motor).and_then(|m| {
        m.send_torque(torque_nm, Duration::from_millis(300))
            .map_err(|e| e.to_string())
    });
    ffi_rc(rc)
}

/// Compressed MIT fast path (manual §5.2): RPDO4 remapped to
/// `[2004h_02, 2004h_03]`, one 8-byte classic frame to `0x500 + node_id`
/// per command. Gains go through the unit chain Nm/rad → Nm/Rev → linear
/// map onto the 12-bit fields using the runtime-read 2004h ranges. All
/// arguments are unified SI (rad, rad/s, Nm/rad, Nm·s/rad, Nm). Huayi
/// motors only; the slow 2003h SDO path is `motor_handle_send_mit`.
#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_huayi_send_mit_compressed(
    motor: *mut MotorHandle,
    pos: f32,
    vel: f32,
    kp: f32,
    kd: f32,
    tau: f32,
) -> i32 {
    if motor.is_null() {
        set_last_error("motor is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let rc = require_huayi(&motor).and_then(|m| {
        m.send_mit_compressed(pos, vel, kp, kd, tau, Duration::from_millis(300))
            .map_err(|e| e.to_string())
    });
    ffi_rc(rc)
}

/// Position preset (3001h, code "pres"): writes the output-shaft position
/// the drive treats as the new zero-reference point. Requires Switch On
/// Disabled (disable the motor first) and |`position_rev`| ≤ 0.5 Rev —
/// the vendor layer rejects with a readable error otherwise. Huayi motors
/// only.
#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_huayi_position_preset(
    motor: *mut MotorHandle,
    position_rev: f32,
) -> i32 {
    if motor.is_null() {
        set_last_error("motor is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let rc = require_huayi(&motor).and_then(|m| {
        m.position_preset(position_rev, Duration::from_millis(200))
            .map_err(|e| e.to_string())
    });
    ffi_rc(rc)
}

// ---------------------------------------------------------------------------
// periodic report (TPDO stream)
// ---------------------------------------------------------------------------

/// Periodic feedback stream control — the Huayi counterpart of RobStride
/// `set_active_report`. `period_ms > 0` activates the three factory TPDOs
/// (TPDO2 pos/vel anchors the cached state, TPDO1/3 patch in place) with
/// that event-timer period; `period_ms = 0` invalidates the COB-ID words
/// (reporting off; reactivation keeps the mappings). Valid range is
/// 1..=65535 ms (the 0x1x00h event timer is a u16). Huayi motors only.
#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_huayi_configure_periodic_report(
    motor: *mut MotorHandle,
    period_ms: u32,
) -> i32 {
    if motor.is_null() {
        set_last_error("motor is null");
        return -1;
    }
    if period_ms > u16::MAX as u32 {
        set_last_error("period_ms exceeds the 16-bit event timer range (1..=65535)");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let rc = require_huayi(&motor).and_then(|m| {
        m.configure_periodic_report(period_ms as u16, Duration::from_millis(300))
            .map_err(|e| e.to_string())
    });
    ffi_rc(rc)
}

// ---------------------------------------------------------------------------
// diagnostics
// ---------------------------------------------------------------------------

/// Last cached `603Fh`/EMCY error code (u16; 0 = no error). Read from the
/// feedback-fed error cache — no bus traffic. EMCY fault bursts are
/// recorded even before a TPDO2 anchor arrives (they only anchor the full
/// state). Errors if no feedback source has ever reported a code (enable
/// the periodic report or call `motor_handle_request_feedback` first).
/// Full code descriptions:
/// `motor_vendor_huayi::error_code_description`. Huayi motors only.
#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_huayi_get_error_code(
    motor: *mut MotorHandle,
    out_code: *mut u16,
) -> i32 {
    if motor.is_null() || out_code.is_null() {
        set_last_error("motor or out_code is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let out = unsafe { &mut *out_code };
    let rc = require_huayi(&motor).and_then(|m| match m.cached_error_code() {
        Some(code) => {
            *out = code;
            Ok(())
        }
        None => Err(
            "no Huayi feedback cached yet; enable the periodic report or call request_feedback first"
                .to_string(),
        ),
    });
    ffi_rc(rc)
}

/// Walk the 1003h error-history object (sub 1.. until sub 0 / error 0) and
/// write up to 32 codes into `out_codes` (oldest first, vendor order).
/// `out_count` receives the number of entries written. Huayi motors only.
#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_huayi_get_error_history(
    motor: *mut MotorHandle,
    out_codes: *mut u16,
    out_count: *mut u32,
) -> i32 {
    if motor.is_null() || out_codes.is_null() || out_count.is_null() {
        set_last_error("motor, out_codes, or out_count is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let rc = require_huayi(&motor)
        .and_then(|m| m.get_error_history(DIAG_TIMEOUT).map_err(|e| e.to_string()))
        .map(|history| unsafe {
            let n = history.len().min(32);
            for (i, code) in history.iter().take(n).enumerate() {
                *out_codes.add(i) = *code;
            }
            *out_count = n as u32;
        });
    ffi_rc(rc)
}

// ---------------------------------------------------------------------------
// node management
// ---------------------------------------------------------------------------

/// Change the node id via 2001h_1 (takes effect after a node reset).
/// Huayi motors only.
#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_huayi_set_node_id(motor: *mut MotorHandle, new_id: u8) -> i32 {
    if motor.is_null() {
        set_last_error("motor is null");
        return -1;
    }
    if !(1..=127).contains(&new_id) {
        set_last_error("new_id must be a CANopen node id in 1..=127");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let rc = require_huayi(&motor).and_then(|m| {
        m.set_node_id(u16::from(new_id), Duration::from_millis(200))
            .map_err(|e| e.to_string())
    });
    ffi_rc(rc)
}

/// Raw NMT master command on this motor's bus (COB-ID 0x000, payload
/// `[command, node]`). `command` is the protocol command specifier:
/// 0x01 start, 0x02 stop, 0x80 enter pre-operational, 0x81 reset node,
/// 0x82 reset communication. `node = 0` broadcasts to all nodes on the
/// bus (including other vendors' CANopen nodes). Fire-and-forget: no
/// reply is waited for. Huayi motors only (any motor on the bus would
/// do; the handle just supplies the bus).
#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_huayi_nmt(motor: *mut MotorHandle, command: u8, node: u8) -> i32 {
    if motor.is_null() {
        set_last_error("motor is null");
        return -1;
    }
    let command = match command {
        0x01 => NmtCommand::StartRemoteNode,
        0x02 => NmtCommand::StopNode,
        0x80 => NmtCommand::EnterPreOperational,
        0x81 => NmtCommand::ResetNode,
        0x82 => NmtCommand::ResetCommunication,
        other => {
            set_last_error(format!(
                "unknown NMT command 0x{other:02X} (expected 01/02/80/81/82)"
            ));
            return -1;
        }
    };
    let motor = lock_motor_inner!(motor, "motor is null");
    let rc = require_huayi(&motor)
        // routed through the vendor so reset commands targeting this node
        // (or broadcast) also invalidate the mode-ready / PP-handshake /
        // TPDO caches — a stale cache would make the next control command
        // skip a handshake the drive no longer remembers
        .and_then(|m| m.nmt(command, node).map_err(|e| e.to_string()));
    ffi_rc(rc)
}

/// Scan `start_id..=end_id` (1..=127) for CANopen nodes: NMT start
/// broadcast, then 6041h/6061h/603Fh SDO probes per id. A node is a hit if
/// any probe answered. `out_nodes` must have room for
/// `end_id - start_id + 1` u16 entries; `out_count` receives the number of
/// hits written. The probes read directly off the bus, so this returns an
/// error if any motor is already registered on the controller (the core
/// receive thread would claim the probe replies) — call it before
/// `add_huayi_motor`, or from a separate controller on the same bus. The
/// scan holds the controller for its whole duration (up to 3 probes ×
/// `timeout_ms` per dead node), so keep `timeout_ms` modest. Huayi
/// controllers only — the shared procedure is vendor-agnostic, but the
/// entry point rides the Huayi controller.
#[unsafe(no_mangle)]
pub extern "C" fn motor_controller_huayi_scan_ids(
    controller: *mut MotorController,
    start_id: u16,
    end_id: u16,
    timeout_ms: u32,
    out_nodes: *mut u16,
    out_count: *mut u32,
) -> i32 {
    if controller.is_null() || out_nodes.is_null() || out_count.is_null() {
        set_last_error("controller, out_nodes, or out_count is null");
        return -1;
    }
    if !(1..=127).contains(&start_id) || !(1..=127).contains(&end_id) || start_id > end_id {
        set_last_error("scan range must be within 1..=127 with start <= end");
        return -1;
    }
    let controller = unsafe { &*controller };
    let mut inner = match controller.inner.lock() {
        Ok(inner) => inner,
        Err(_) => {
            set_last_error("controller lock poisoned");
            return -1;
        }
    };
    let span = (end_id - start_id + 1) as usize;
    let rc = ensure_huayi_controller(&mut inner)
        .and_then(|ctrl| {
            ctrl.scan_ids(
                start_id,
                end_id,
                Duration::from_millis(timeout_ms as u64),
            )
            .map_err(|e| e.to_string())
        })
        .map(|hits| unsafe {
            let n = hits.len().min(span);
            for (i, hit) in hits.iter().take(n).enumerate() {
                *out_nodes.add(i) = hit.node_id;
            }
            *out_count = n as u32;
        });
    ffi_rc(rc)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{BoundController, ControllerInner, MotorState};
    use motor_core::bus::{CanBus, CanFrame, Transport};
    use motor_core::device::MotorDevice;
    use motor_core::test_support::MockBus;
    use motor_vendor_huayi::HuayiController;
    use motor_vendor_robstride::RobstrideMotor;
    use std::sync::{Arc, Mutex};

    const TIMEOUT_MS: u32 = 200;

    fn huayi_handle() -> (Arc<MockBus>, MotorHandle) {
        let bus = Arc::new(MockBus::new());
        let ctrl = HuayiController::new(Arc::clone(&bus) as Arc<dyn CanBus>);
        let motor = ctrl.add_motor(1, 0, "huayi-joint").expect("add motor");
        (
            bus,
            MotorHandle {
                inner: Mutex::new(MotorHandleInner::Huayi(motor)),
            },
        )
    }

    fn sdo_rsp(cmd: u8, index: u16, sub: u8, payload: [u8; 4]) -> CanFrame {
        let mut data = [0u8; 8];
        data[0] = cmd;
        data[1] = (index & 0xFF) as u8;
        data[2] = ((index >> 8) & 0xFF) as u8;
        data[3] = sub;
        data[4..8].copy_from_slice(&payload);
        CanFrame {
            arbitration_id: 0x580 + 1,
            data,
            dlc: 8,
            is_extended: false,
            is_rx: true,
        }
    }

    fn rx_frame(id: u32, data: [u8; 8], dlc: u8) -> CanFrame {
        CanFrame {
            arbitration_id: id,
            data,
            dlc,
            is_extended: false,
            is_rx: true,
        }
    }

    fn motor_of(handle: &MotorHandle) -> Arc<motor_vendor_huayi::HuayiMotor> {
        match &*handle.inner.lock().expect("inner") {
            MotorHandleInner::Huayi(m) => Arc::clone(m),
            _ => panic!("expected huayi motor"),
        }
    }

    #[test]
    fn sdo_read_u16_roundtrip() {
        let (_bus, mut handle) = huayi_handle();
        // Queue a 6041h upload reply (expedited 2-byte, statusword 0x0237).
        motor_of(&handle)
            .process_feedback_frame(sdo_rsp(0x4B, 0x6041, 0x00, [0x37, 0x02, 0, 0]))
            .expect("queue reply");
        let mut value = 0u16;
        let rc = motor_handle_huayi_sdo_read_u16(
            (&mut handle) as *mut MotorHandle,
            0x6041,
            0x00,
            TIMEOUT_MS,
            &mut value,
        );
        assert_eq!(rc, 0);
        assert_eq!(value, 0x0237);
    }

    #[test]
    fn sdo_write_u16_sends_expedited_frame() {
        let (bus, mut handle) = huayi_handle();
        motor_of(&handle)
            .process_feedback_frame(sdo_rsp(0x60, 0x6060, 0x00, [0; 4]))
            .expect("queue ack");
        let rc = motor_handle_huayi_sdo_write_u16(
            (&mut handle) as *mut MotorHandle,
            0x6060,
            0x00,
            5,
        );
        assert_eq!(rc, 0);
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 1);
        // expedited 16-bit write: cmd 0x2B, 6060h sub 0, value 5
        assert_eq!(sent[0].arbitration_id, 0x601);
        assert_eq!(sent[0].data[0], 0x2B);
        assert_eq!(&sent[0].data[1..4], &[0x60, 0x60, 0x00]);
        assert_eq!(u16::from_le_bytes([sent[0].data[4], sent[0].data[5]]), 5);
    }

    #[test]
    fn sdo_read_rejects_non_huayi_motor() {
        let motor = Arc::new(
            RobstrideMotor::new(2, 0xFD, "rs-00", Arc::new(MockBus::new())).expect("motor"),
        );
        let mut handle = MotorHandle {
            inner: Mutex::new(MotorHandleInner::Robstride(motor)),
        };
        let mut value = 0u16;
        let rc = motor_handle_huayi_sdo_read_u16(
            (&mut handle) as *mut MotorHandle,
            0x6041,
            0x00,
            TIMEOUT_MS,
            &mut value,
        );
        assert_eq!(rc, -1);
        assert_eq!(value, 0);
    }

    #[test]
    fn configure_periodic_report_zero_deactivates_tpdos() {
        let (bus, mut handle) = huayi_handle();
        // deactivate_default_tpdos: 3 COB-ID invalidation writes.
        for n in 1..=3u16 {
            motor_of(&handle)
                .process_feedback_frame(sdo_rsp(0x60, 0x1800 + n - 1, 0x01, [0; 4]))
                .expect("queue ack");
        }
        let rc = motor_handle_huayi_configure_periodic_report(
            (&mut handle) as *mut MotorHandle,
            0,
        );
        assert_eq!(rc, 0);
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 3);
        let words: Vec<u32> = sent
            .iter()
            .map(|f| u32::from_le_bytes([f.data[4], f.data[5], f.data[6], f.data[7]]))
            .collect();
        for word in words {
            assert_eq!(word & 0x8000_0000, 0x8000_0000, "COB-ID invalid bit set");
        }
    }

    #[test]
    fn configure_periodic_report_rejects_oversized_period() {
        let (_bus, mut handle) = huayi_handle();
        let rc = motor_handle_huayi_configure_periodic_report(
            (&mut handle) as *mut MotorHandle,
            u32::from(u16::MAX) + 1,
        );
        assert_eq!(rc, -1);
    }

    #[test]
    fn get_error_code_reads_emcy_cached_code() {
        let (_bus, mut handle) = huayi_handle();
        // Before any feedback: rc -1 with a readable message.
        let mut code = 0u16;
        let rc =
            motor_handle_huayi_get_error_code((&mut handle) as *mut MotorHandle, &mut code);
        assert_eq!(rc, -1);
        // EMCY frame: COB-ID 0x80 + node, data[1..3] = error code LE.
        motor_of(&handle)
            .process_feedback_frame(rx_frame(0x81, [0, 0x10, 0x23, 0, 0, 0, 0, 0], 4))
            .expect("emcy");
        let rc =
            motor_handle_huayi_get_error_code((&mut handle) as *mut MotorHandle, &mut code);
        assert_eq!(rc, 0);
        assert_eq!(code, 0x2310);
    }

    #[test]
    fn nmt_sends_protocol_frame() {
        let (bus, mut handle) = huayi_handle();
        let rc = motor_handle_huayi_nmt((&mut handle) as *mut MotorHandle, 0x01, 0x05);
        assert_eq!(rc, 0);
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 1);
        assert_eq!(sent[0].arbitration_id, 0x000);
        assert_eq!(&sent[0].data[..2], &[0x01, 0x05]);
        assert_eq!(sent[0].dlc, 2);
    }

    #[test]
    fn nmt_rejects_unknown_command() {
        let (bus, mut handle) = huayi_handle();
        let rc = motor_handle_huayi_nmt((&mut handle) as *mut MotorHandle, 0x42, 0x01);
        assert_eq!(rc, -1);
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 0);
    }

    #[test]
    fn scan_ids_finds_answering_node() {
        let bus = Arc::new(MockBus::new());
        let mut controller = MotorController {
            inner: Mutex::new(ControllerInner::Bound(BoundController::new(
                Arc::clone(&bus) as Arc<dyn CanBus>,
                Transport::SocketCan,
            ))),
        };
        // Node 1 answers all three probes (6041h, 6061h, 603Fh); node 2
        // stays silent. (0x4B = expedited 2-byte upload reply.)
        bus.push_rx(sdo_rsp(0x4B, 0x6041, 0x00, [0x37, 0x02, 0, 0]));
        bus.push_rx(sdo_rsp(0x4F, 0x6061, 0x00, [5, 0, 0, 0]));
        bus.push_rx(sdo_rsp(0x4B, 0x603F, 0x00, [0, 0, 0, 0]));

        let mut nodes = [0u16; 2];
        let mut count = 0u32;
        let rc = motor_controller_huayi_scan_ids(
            (&mut controller) as *mut MotorController,
            1,
            2,
            100,
            nodes.as_mut_ptr(),
            &mut count,
        );
        assert_eq!(rc, 0);
        assert_eq!(count, 1);
        assert_eq!(nodes[0], 1);
        // NMT start broadcast went out first.
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent[0].arbitration_id, 0x000);
        assert_eq!(&sent[0].data[..2], &[0x01, 0x00]);
    }

    #[test]
    fn scan_ids_rejects_invalid_range() {
        let bus = Arc::new(MockBus::new());
        let mut controller = MotorController {
            inner: Mutex::new(ControllerInner::Bound(BoundController::new(
                Arc::clone(&bus) as Arc<dyn CanBus>,
                Transport::SocketCan,
            ))),
        };
        let mut nodes = [0u16; 4];
        let mut count = 0u32;
        let rc = motor_controller_huayi_scan_ids(
            (&mut controller) as *mut MotorController,
            5,
            2,
            100,
            nodes.as_mut_ptr(),
            &mut count,
        );
        assert_eq!(rc, -1);
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 0);
    }

    #[test]
    fn get_state_maps_huayi_status() {
        let (_bus, mut handle) = huayi_handle();
        let motor = motor_of(&handle);
        // TPDO2 anchor: pos 1.5 Rev, vel -0.25 Rev/s at 0x280 + node.
        let mut data = [0u8; 8];
        data[0..4].copy_from_slice(&1.5f32.to_le_bytes());
        data[4..8].copy_from_slice(&(-0.25f32).to_le_bytes());
        motor
            .process_feedback_frame(rx_frame(0x281, data, 8))
            .expect("tpdo2");
        // TPDO1 patch: error 0x2310, statusword 0x0237, mode 5.
        motor
            .process_feedback_frame(rx_frame(
                0x181,
                [0x10, 0x23, 0x37, 0x02, 0x05, 0, 0, 0],
                5,
            ))
            .expect("tpdo1");

        let mut state = MotorState::default();
        let rc = crate::state_ffi::motor_handle_get_state(
            (&mut handle) as *mut MotorHandle,
            &mut state,
        );
        assert_eq!(rc, 0);
        assert_eq!(state.has_value, 1);
        assert_eq!(state.can_id, 1);
        // 1.5 Rev → 3π rad
        assert!((state.pos - 1.5 * std::f32::consts::TAU).abs() < 1e-4);
        assert!((state.vel + 0.25 * std::f32::consts::TAU).abs() < 1e-4);
        // 6061h mode display lands in status_code (Huayi numbering: 5 = MIT).
        assert_eq!(state.status_code, 5);
        // 603Fh error code low byte lands in fault_code; the full u16 is
        // motor_handle_huayi_get_error_code's job.
        assert_eq!(state.fault_code, 0x10);
    }
}
