use crate::args::{get_f32, get_i16, get_str, get_u16_hex_or_dec, get_u64};
use motor_core::bus::{open_transport, CanBus, Transport, TransportParams};
use motor_core::CanFrame;
use motor_vendor_hightorque::{
    parameter_info, HightorqueController, HightorqueFeedbackState, ReadCmd,
};
use std::collections::HashMap;
use std::convert::TryFrom;
use std::sync::Arc;
use std::time::{Duration, Instant};

const TWO_PI: f32 = std::f32::consts::PI * 2.0;

/// HighTorque (HT) broadcast discovery: the vendor crate has no scan API,
/// so broadcast queries must hand-write the 17 01 frame on the raw bus and
/// collect the 27 01 reply (since the 2026-09-29 convergence this is the
/// only remaining raw-bus HT code in the CLI, discovery only; control/
/// status/parameters all go through the vendor).
#[derive(Debug, Clone, Copy)]
struct HtScanHit {
    motor_id: u16,
    pos_raw: i16,
    vel_raw: i16,
    tqe_raw: i16,
}

fn send_ht_scan_query(bus: &dyn CanBus, motor_id: u16) -> Result<(), Box<dyn std::error::Error>> {
    let payload = [0x17u8, 0x01, 0, 0, 0, 0, 0, 0];
    bus.send(CanFrame {
        arbitration_id: u32::from(0x8000u16 | motor_id),
        data: payload,
        dlc: payload.len() as u8,
        is_extended: true,
        is_rx: false,
    })?;
    Ok(())
}

fn decode_ht_scan_reply(frame: CanFrame) -> Option<HtScanHit> {
    if frame.dlc < 8 || frame.data[0] != 0x27 || frame.data[1] != 0x01 {
        return None;
    }
    // The motor puts its own id in the high byte of the reply ID (both
    // standard and extended frames; v2.0.0 firmware motor.c::
    // motor_process_state_all: `id = Identifier >> 8`), reply dest=0.
    if (frame.arbitration_id & 0x00FF) != 0 {
        return None;
    }
    let motor_id = ((frame.arbitration_id >> 8) & 0x7F) as u16;
    Some(HtScanHit {
        motor_id,
        pos_raw: i16::from_le_bytes([frame.data[2], frame.data[3]]),
        vel_raw: i16::from_le_bytes([frame.data[4], frame.data[5]]),
        tqe_raw: i16::from_le_bytes([frame.data[6], frame.data[7]]),
    })
}

fn wait_ht_scan_reply(
    bus: &dyn CanBus,
    motor_id: u16,
    timeout: Duration,
) -> Result<Option<HtScanHit>, Box<dyn std::error::Error>> {
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        let left = deadline.saturating_duration_since(Instant::now());
        if let Some(frame) = bus.recv(left.min(Duration::from_millis(20)))? {
            if let Some(hit) = decode_ht_scan_reply(frame) {
                if hit.motor_id == motor_id {
                    return Ok(Some(hit));
                }
            }
        }
    }
    Ok(None)
}

fn print_scan_hit(prefix: &str, s: HtScanHit) {
    println!(
        "{} id={} pos_raw={} vel_raw={} tqe_raw={} pos_turn={:+.4} vel_rps={:+.4}",
        prefix,
        s.motor_id,
        s.pos_raw,
        s.vel_raw,
        s.tqe_raw,
        s.pos_raw as f32 * 0.0001,
        s.vel_raw as f32 * 0.00025
    );
}

/// Convergence (2026-09-29): read/ping print vendor-decoded physical values
/// (with torque coeff compensation and status/fault/temperature), replacing
/// the old hand-written raw + self-computed conversion.
fn print_state(prefix: &str, s: &HightorqueFeedbackState) {
    println!(
        "{} id={} arb_id=0x{:X} status={} fault={} pos={:+.4}rad vel={:+.4}rad/s torq={:+.3}Nm t_mos={:.1}C t_rotor={:.1}C",
        prefix,
        s.can_id,
        s.arbitration_id,
        s.status_code,
        s.fault_code,
        s.pos,
        s.vel,
        s.torq,
        s.t_mos,
        s.t_rotor
    );
}

/// raw int16 scale → physical value (pos: 1 raw = 0.0001 turn; vel: 1 raw = 0.00025 turn/s).
fn pos_rad_from_args(args: &HashMap<String, String>) -> Result<f32, String> {
    if args.contains_key("raw-pos") {
        let raw = get_i16(args, "raw-pos", 0)?;
        return Ok(raw as f32 * 0.0001 * TWO_PI);
    }
    if args.contains_key("pos-deg") {
        let deg = get_f32(args, "pos-deg", 0.0)?;
        return Ok(deg.to_radians());
    }
    get_f32(args, "pos", 0.0)
}

fn vel_rad_s_from_args(args: &HashMap<String, String>) -> Result<f32, String> {
    if args.contains_key("raw-vel") {
        let raw = get_i16(args, "raw-vel", 0)?;
        return Ok(raw as f32 * 0.00025 * TWO_PI);
    }
    if args.contains_key("vel-deg-s") {
        let deg_s = get_f32(args, "vel-deg-s", 0.0)?;
        return Ok(deg_s.to_radians());
    }
    get_f32(args, "vel", 0.0)
}

fn tau_nm_from_args(args: &HashMap<String, String>) -> Result<Option<f32>, String> {
    if args.contains_key("raw-tqe") {
        return Err(
            "raw-tqe is no longer accepted here: torque is physical Nm (use tau); encoding is vendor-side with model torque coeff".to_string(),
        );
    }
    if args.contains_key("tau") {
        return Ok(Some(get_f32(args, "tau", 0.0)?));
    }
    Ok(None)
}

/// Register address argument parsing (B2 fix, 2026-10-21): parse to u8 and
/// **reject out-of-range** (> 0xFF). The old `get_u16_hex_or_dec(...)? as u8`
/// silently truncated 0x145 to 0x45 — reading/writing the wrong register and
/// reporting the result as the requested one. Valid domain 0x00-0xFF (single
/// address; the address-advance wrap of consecutive reads is vendor-side
/// address arithmetic, outside argument validation). `mode_label` locates
/// the error (read-reg/write-reg).
fn reg_addr_from_args(args: &HashMap<String, String>, mode_label: &str) -> Result<u8, String> {
    match args.get("addr") {
        Some(_) => {
            let raw = get_u16_hex_or_dec(args, "addr", 0).map_err(|e| e.to_string())?;
            u8::try_from(raw)
                .map_err(|_| format!("{mode_label} --addr out of range 0x00-0xFF, got 0x{raw:X}"))
        }
        None => Err(format!(
            "{mode_label} requires --addr (hex or decimal, 0x00-0xFF)"
        )),
    }
}

/// Open the CAN bus for the requested transport. Routes universal transports
/// (socketcan / mcu-serial) through `open_transport` so the platform driver
/// construction lives in one place (core). HighTorque uses standard CAN only,
/// so socketcanfd is rejected (it is not a CAN-FD device here); damiao-only
/// transports are rejected.
fn open_hightorque_bus(
    transport: &str,
    channel: &str,
    serial_port: &str,
    serial_baud: u32,
) -> Result<Arc<dyn CanBus>, Box<dyn std::error::Error>> {
    let p = TransportParams {
        channel,
        serial_port,
        serial_baud,
    };
    let bus: Arc<dyn CanBus> = match transport {
        "auto" | "socketcan" => open_transport(Transport::SocketCan, &p)?,
        "mcu-serial" => open_transport(Transport::McuSerial, &p)?,
        "socketcanfd" => {
            return Err(
                "transport socketcanfd unsupported (hightorque uses standard CAN only)".into(),
            )
        }
        "dm-serial" | "dm-device" => {
            return Err(format!(
                "transport {transport} is damiao-only (hightorque supports auto|socketcan|mcu-serial)"
            )
            .into());
        }
        _ => {
            return Err(format!(
                "unknown HighTorque transport: {transport} (expected auto|socketcan|mcu-serial)"
            )
            .into());
        }
    };
    Ok(bus)
}

/// Raw diagnostic frames with no vendor counterpart are all gone
/// (2026-09-30): tqe/volt/cur/brake/periodic-report-config all go through
/// vendor primitives; the only remaining raw-bus HT code in the CLI is scan
/// (the vendor has no scan API; broadcast discovery must hand-write the
/// 17 01 frame and collect the 27 01 reply).
pub fn run_hightorque(
    args: &HashMap<String, String>,
    channel: &str,
    model: &str,
    motor_id: u16,
    feedback_id: u16,
) -> Result<(), Box<dyn std::error::Error>> {
    let mode = get_str(args, "mode", "ping");
    let loop_n = get_u64(args, "loop", 1)?;
    let dt_ms = get_u64(args, "dt-ms", 20)?;
    let transport = get_str(args, "transport", "auto");
    let serial_port = get_str(args, "serial-port", "/dev/ttyACM0");
    let serial_baud_u64 = get_u64(args, "serial-baud", 921600)?;
    let serial_baud = u32::try_from(serial_baud_u64)
        .map_err(|_| format!("invalid --serial-baud (too large): {serial_baud_u64}"))?;
    let bus = open_hightorque_bus(&transport, channel, &serial_port, serial_baud)?;

    if mode == "scan" {
        let start_id = get_u16_hex_or_dec(args, "start-id", 1)?.clamp(1, 127);
        let end_id = get_u16_hex_or_dec(args, "end-id", 32)?.clamp(1, 127);
        if start_id > end_id {
            return Err("invalid scan range after clamp (start-id > end-id)".into());
        }
        println!(
            "[scan] probing hightorque IDs {}..{} on {} by 0x17/0x01",
            start_id, end_id, channel
        );
        let mut hits = 0usize;
        for id in start_id..=end_id {
            send_ht_scan_query(bus.as_ref(), id)?;
            if let Some(s) = wait_ht_scan_reply(bus.as_ref(), id, Duration::from_millis(80))? {
                print_scan_hit("[hit]", s);
                hits += 1;
            }
            std::thread::sleep(Duration::from_millis(2));
        }
        println!("[scan] done vendor=hightorque hits={hits}");
        bus.shutdown()?;
        return Ok(());
    }

    // Convergence (2026-09-29): control/status/parameter commands all go
    // through the vendor crate (HightorqueController wraps CoreController +
    // a background receive thread); motors are registered via add_motor,
    // and --model now takes effect (the model decides the torque
    // compensation coeff; unknown model codes are rejected at add_motor).
    // Since 2026-09-30 the raw diagnostic frame path (diag_bus) is deleted;
    // the old diag modes (tqe/volt/cur/brake/timed-read) all use vendor
    // physical-value primitives.
    let ctrl = HightorqueController::new(bus);
    let motor = ctrl
        .add_motor(motor_id, feedback_id, model)
        .map_err(|e| format!("add motor failed: {e}"))?;

    let mut send_count = loop_n.max(1);
    if matches!(mode.as_str(), "ping" | "read" | "read-reg" | "write-reg") {
        send_count = 1;
    }

    for i in 0..send_count {
        match mode.as_str() {
            "ping" | "read" => {
                // Block for a fresh state (≤500ms): vendor request_fresh_state;
                // timeout errors out — ping semantics = confirm the motor is
                // on the bus and replying.
                let state = motor
                    .request_fresh_state(Duration::from_millis(500))
                    .map_err(|e| format!("hightorque {} on id={} failed: {e}", mode, motor_id))?;
                print_state("[ok]", &state);
            }
            "pos" => {
                // Classic position mode 07 07: vendor send_cmd_pos_classic;
                // torque is physical Nm (model-adaptive compensation),
                // default unlimited (0x8000).
                let pos = pos_rad_from_args(args)?;
                let tqe = tau_nm_from_args(args)?;
                println!(
                    "[tx] mode=pos id={} pos={:+.4}rad tqe_limit={:?}",
                    motor_id, pos, tqe
                );
                motor
                    .send_cmd_pos_classic(pos, tqe)
                    .map_err(|e| e.to_string())?;
            }
            "vel" => {
                let vel = vel_rad_s_from_args(args)?;
                println!("[tx] mode=vel id={} vel={:+.4}rad/s", motor_id, vel);
                motor.send_cmd_vel(vel).map_err(|e| e.to_string())?;
            }
            "tqe" => {
                // Torque mode 05 13: vendor send_cmd_torque, physical Nm
                // (0.01 N·m/LSB base + model torque coeff compensation).
                if args.contains_key("raw-tqe") {
                    return Err(
                        "raw-tqe is no longer accepted: torque is physical Nm (use --tau); encoding is vendor-side with model torque coeff".into(),
                    );
                }
                let tau = get_f32(args, "tau", 0.0)?;
                println!(
                    "[tx] mode=tqe id={} tau={:+.3}Nm (model coeff k={:.3})",
                    motor_id,
                    tau,
                    motor.torque_coeff().k
                );
                motor.send_cmd_torque(tau).map_err(|e| e.to_string())?;
            }
            "mit" => {
                // v2.0.0 MIT (motor firmware v4.6.0+): vendor send_cmd_mit,
                // bit-packed pos(16)/vel(12)/tqe(12)/kp(12)/kd(12),
                // saturating within range.
                let pos = pos_rad_from_args(args)?;
                let vel = vel_rad_s_from_args(args)?;
                let tau = get_f32(args, "tau", 0.0)?;
                let kp = get_f32(args, "kp", 0.0)?;
                let kd = get_f32(args, "kd", 0.0)?;
                println!(
                    "[tx] mode=mit id={} pos={:+.4}rad vel={:+.4}rad/s tau={:+.3}Nm kp={:.3} kd={:.3}",
                    motor_id, pos, vel, tau, kp, kd
                );
                motor
                    .send_cmd_mit(pos, vel, kp, kd, tau)
                    .map_err(|e| e.to_string())?;
            }
            "volt" => {
                // DQ voltage mode 01 00 08 05 1B: vendor send_cmd_dq_voltage,
                // physical V (0.1 V/LSB); mode write and register write packed
                // into one frame.
                if args.contains_key("raw-vol") {
                    return Err(
                        "raw-vol is no longer accepted: voltage is physical V (use --volt); scale is 0.1 V/LSB vendor-side".into(),
                    );
                }
                let volt = get_f32(args, "volt", 0.0)?;
                println!("[tx] mode=volt id={} volt={:+.3}V", motor_id, volt);
                motor.send_cmd_dq_voltage(volt).map_err(|e| e.to_string())?;
            }
            "cur" => {
                // DQ current mode 01 00 09 05 1C: vendor send_cmd_dq_current,
                // physical A (0.1 A/LSB), packed into one frame.
                if args.contains_key("raw-cur") {
                    return Err(
                        "raw-cur is no longer accepted: current is physical A (use --cur); scale is 0.1 A/LSB vendor-side".into(),
                    );
                }
                let cur = get_f32(args, "cur", 0.0)?;
                println!("[tx] mode=cur id={} cur={:+.3}A", motor_id, cur);
                motor.send_cmd_dq_current(cur).map_err(|e| e.to_string())?;
            }
            "set-mode" => {
                // Run mode switch 01 00 <mode>: writes mode register 0x000
                // (table 2, 0-17). Complementary to stop/clear-error
                // (mode 0); can explicitly enter DQ voltage (8) / DQ current
                // (9) / position (10) / brake (15) and other modes.
                let run_mode = get_u64(args, "run-mode", 0)?;
                if run_mode > 17 {
                    return Err(format!("run-mode must be 0-17 (表2), got {run_mode}").into());
                }
                println!(
                    "[tx] mode=set-mode id={} run-mode={} (register 0x000)",
                    motor_id, run_mode
                );
                motor
                    .send_run_mode(run_mode as u8)
                    .map_err(|e| e.to_string())?;
            }
            "pos-vel-tqe" => {
                // Coordinated mode 07 35: vendor send_cmd_pos_vel (velocity
                // limit converted); torque is fixed unlimited (0x8000) by
                // the vendor, raw-tqe no longer accepted.
                let pos = pos_rad_from_args(args)?;
                let vel = vel_rad_s_from_args(args)?;
                println!(
                    "[tx] mode=pos-vel-tqe id={} pos={:+.4}rad vel={:+.4}rad/s (torque unlimited)",
                    motor_id, pos, vel
                );
                motor
                    .send_cmd_pos_vel(pos, vel)
                    .map_err(|e| e.to_string())?;
            }
            "pos-vel-acc" => {
                // Trapezoid mode 07 2d (firmware v4.6.0+): vendor
                // send_cmd_pos_vel_acc, position + velocity limit +
                // acceleration limit, all physical values.
                let pos = pos_rad_from_args(args)?;
                let vlim = get_f32(args, "vlim", 1.0)?;
                let acc = get_f32(args, "acc", 1.0)?;
                println!(
                    "[tx] mode=pos-vel-acc id={} pos={:+.4}rad vlim={:+.4}rad/s acc={:+.4}rad/s^2",
                    motor_id, pos, vlim, acc
                );
                motor
                    .send_cmd_pos_vel_acc(pos, vlim, acc)
                    .map_err(|e| e.to_string())?;
            }
            "read-reg" => {
                // Generic register read (17 family): vendor read_registers
                // (fixed Int16 since 2026-10-22, --ty removed). Returns
                // **converted physical values** (classified by the vendor
                // register table's quantity: position/velocity → rad family,
                // torque with model k, current/voltage/temperature/PWM/PID
                // per their own scale; registers with no conversion 1:1).
                // Name/unit resolved per row address via parameter_info
                // (C2: the old implementation printed the base address's
                // register name on every row). Out-of-range addresses are
                // rejected in reg_addr_from_args (B2).
                if args.contains_key("ty") {
                    return Err(
                        "--ty was removed: register reads are fixed Int16 (2026-10-22)".into(),
                    );
                }
                let addr = reg_addr_from_args(args, "read-reg")?;
                let count = get_u64(args, "count", 1)?.clamp(1, 3) as u8;
                let timeout_ms = get_u64(args, "timeout-ms", 500)?;
                let vals = motor
                    .read_registers(
                        &[ReadCmd { addr, count }],
                        Duration::from_millis(timeout_ms),
                    )
                    .map_err(|e| e.to_string())?;
                for (a, v) in &vals {
                    // C2: resolve name/unit by each row's own address, not the base address.
                    let (name, unit) = match parameter_info(*a) {
                        Some(i) => (i.name, i.quantity.map(|q| q.unit_label()).unwrap_or("")),
                        None => ("unknown", ""),
                    };
                    println!(
                        "[ok] mode=read-reg id={} addr=0x{:02X} name={} unit={} value={:+.4}",
                        motor_id, a, name, unit, v
                    );
                }
            }
            "stop" => {
                motor.disable().map_err(|e| e.to_string())?;
            }
            "brake" => {
                // Brake (table 2 mode 15): vendor brake, three-phase shorted
                // damping brake.
                println!("[tx] mode=brake id={} (run mode 15)", motor_id);
                motor.brake().map_err(|e| e.to_string())?;
            }
            "conf-write" => {
                // Persist to flash (05 B3), vendor store_parameters.
                motor.store_parameters().map_err(|e| e.to_string())?;
            }
            "rezero" => {
                // 0x40 zeroing + auto persist, vendor set_zero_position.
                motor.set_zero_position().map_err(|e| e.to_string())?;
            }
            "periodic-report" | "timed-read" => {
                // Periodic report config 05 B4 02 <t_ms>: vendor
                // set_periodic_report (extended frame). period-ms >= 1 makes
                // the motor actively send 27 01 status frames, 0 stops.
                // timed-read is the old alias, kept for existing scripts.
                let t_ms = get_u64(args, "period-ms", 100)?;
                if t_ms > u16::MAX as u64 {
                    return Err(format!("period-ms must fit uint16, got {t_ms}").into());
                }
                println!(
                    "[tx] mode=periodic-report id={} period_ms={} ({})",
                    motor_id,
                    t_ms,
                    if t_ms == 0 { "stop" } else { "start" }
                );
                motor
                    .set_periodic_report(t_ms as u16)
                    .map_err(|e| e.to_string())?;
            }
            "write-reg" => {
                // Generic register write 05 <addr> <val16>: vendor write_register.
                // Read/write asymmetry (2026-10-22 decision): reads return
                // converted physical values, writes still take raw i16 (the
                // register domain is i16; conversion is the caller's job per
                // the table). --config 1 means a config-class register
                // (extended 0x8000|id frame); the default 0 uses the control
                // channel (control-target class). Out-of-range addresses are
                // rejected in reg_addr_from_args (B2: no more as u8 truncation).
                let addr = reg_addr_from_args(args, "write-reg")?;
                let value = get_i16(args, "value", 0)?;
                let config = get_u64(args, "config", 0)? != 0;
                let name = parameter_info(addr).map(|i| i.name).unwrap_or("unknown");
                println!(
                    "[tx] mode=write-reg id={} addr=0x{addr:02X} name={} value={} frame={}",
                    motor_id,
                    name,
                    value,
                    if config {
                        "config(extended)"
                    } else {
                        "control"
                    }
                );
                motor
                    .write_register(addr, value, config)
                    .map_err(|e| e.to_string())?;
                // --store 1: after writing, send a conf-write (05 B3) frame to
                // persist, matching the Damiao/RS write-param --store
                // convenience. Default 0 writes RAM only; persistence is a
                // whole-device save (not per-register), and the firmware
                // recommends power-cycling for it to take effect. write-reg
                // forces a single send (send_count=1), avoiding repeated
                // flash writes in loop mode.
                let store_after_write = get_u64(args, "store", 0)? != 0;
                if store_after_write {
                    motor.store_parameters().map_err(|e| e.to_string())?;
                    println!("[ok] conf-write sent (store=1; 整机落盘,建议重新上电使配置生效)");
                }
            }
            _ => {
                return Err(format!(
                "unknown hightorque mode: {}. expected ping|scan|read|mit|pos|vel|tqe|volt|cur|set-mode|pos-vel-tqe|pos-vel-acc|read-reg|write-reg|stop|brake|rezero|conf-write|periodic-report",
                mode
            )
            .into());
            }
        }
        if send_count > 1 {
            println!("[loop] #{i} sent mode={mode}");
        }
        if i + 1 < send_count {
            std::thread::sleep(Duration::from_millis(dt_ms));
        }
    }

    ctrl.close_bus()?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::reg_addr_from_args;

    use std::collections::HashMap;

    fn args(pairs: &[(&str, &str)]) -> HashMap<String, String> {
        pairs
            .iter()
            .map(|(k, v)| (k.to_string(), v.to_string()))
            .collect()
    }

    /// B2 regression (2026-10-21): out-of-range addresses (> 0xFF) rejected,
    /// no more as u8 truncation (the old implementation would silently
    /// operate on 0x45 for --addr 0x145).
    #[test]
    fn reg_addr_rejects_out_of_range() {
        let a = args(&[("addr", "0x100")]);
        assert!(reg_addr_from_args(&a, "read-reg").is_err());

        let a = args(&[("addr", "0x145")]);
        let e = reg_addr_from_args(&a, "write-reg").unwrap_err();
        assert!(e.contains("out of range"), "got: {e}");

        // Valid-domain boundaries pass
        let a = args(&[("addr", "0xFF")]);
        assert_eq!(reg_addr_from_args(&a, "read-reg"), Ok(0xFF));
        let a = args(&[("addr", "0")]);
        assert_eq!(reg_addr_from_args(&a, "read-reg"), Ok(0));

        // Missing argument errors
        let a: HashMap<String, String> = HashMap::new();
        assert!(reg_addr_from_args(&a, "read-reg")
            .unwrap_err()
            .contains("requires --addr"));
    }
}
