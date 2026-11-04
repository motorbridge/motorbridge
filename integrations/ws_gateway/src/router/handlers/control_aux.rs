use crate::commands::{as_f32, as_u64};
use crate::model::{ControllerHandle, MotorHandle};
use crate::session::SessionCtx;
use motor_vendor_hightorque::{parameter_info, ReadCmd};
use serde_json::{json, Value};
use std::time::Duration;

pub(crate) fn handle(op: &str, v: &Value, ctx: &mut SessionCtx) -> Option<Result<Value, String>> {
    match op {
        "current" => Some(handle_current(v, ctx)),
        "pos" => Some(handle_pos(v, ctx)),
        "pos_classic" | "pos-classic" => Some(handle_pos_classic(v, ctx)),
        "version" => Some(handle_version(v, ctx)),
        "mode_query" | "mode-query" => Some(handle_mode_query(ctx)),
        "read" => Some(handle_read(v, ctx)),
        "read_register" | "read-register" => Some(handle_read_register(v, ctx)),
        "dq_voltage" | "dq-voltage" => Some(handle_dq_voltage(v, ctx)),
        "dq_current" | "dq-current" => Some(handle_dq_current(v, ctx)),
        "brake" => Some(handle_brake(ctx)),
        "run_mode" | "run-mode" => Some(handle_run_mode(v, ctx)),
        "write_register" | "write-register" => Some(handle_write_register(v, ctx)),
        "periodic_report" | "periodic-report" => Some(handle_periodic_report(v, ctx)),
        _ => None,
    }
}

fn handle_current(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    let current = as_f32(v, "current", 0.0);
    match ctx.motor.as_ref() {
        Some(MotorHandle::Myactuator(m)) => {
            m.send_current_setpoint(current)
                .map_err(|e| e.to_string())?;
            Ok(json!({"op":"current", "current": current}))
        }
        Some(_) => Err("current is supported for myactuator only".to_string()),
        None => Err("motor not connected".to_string()),
    }
}

fn handle_pos(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    let pos = as_f32(v, "pos", 0.0);
    let max_speed = as_f32(v, "max_speed", 8.726646);
    match ctx.motor.as_ref() {
        Some(MotorHandle::Myactuator(m)) => {
            m.send_position_absolute_setpoint(pos.to_degrees(), max_speed.to_degrees())
                .map_err(|e| e.to_string())?;
            Ok(json!({"op":"pos", "pos": pos, "max_speed": max_speed}))
        }
        Some(_) => Err("pos is supported for myactuator only".to_string()),
        None => Err("motor not connected".to_string()),
    }
}

/// HighTorque classic position mode (`07 07`): target position + optional
/// torque limit. `tqe` absent means unlimited torque (0x8000 sentinel);
/// torque gets model-adaptive compensation. A present-but-non-numeric field
/// errors out (review A2: the limit is a safety field, a type error must
/// not silently degrade to "unlimited"). One-shot command, does not enter
/// the continuous active command (same shape as myactuator "pos").
fn handle_pos_classic(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    let pos = as_f32(v, "pos", 0.0);
    // Distinguish "field absent = intentionally unlimited torque" from
    // "field present but wrong type = client typo": the latter errors out,
    // so "tqe":"1.5" is never silently encoded as the 0x8000 unlimited
    // torque sentinel.
    let tqe = match v.get("tqe") {
        None => None,
        Some(value) => Some(
            value
                .as_f64()
                .map(|x| x as f32)
                .ok_or_else(|| "pos_classic 'tqe' must be a number (Nm)".to_string())?,
        ),
    };
    match ctx.motor.as_ref() {
        Some(MotorHandle::Hightorque(m)) => {
            m.send_cmd_pos_classic(pos, tqe)
                .map_err(|e| e.to_string())?;
            Ok(json!({"op":"pos_classic","pos": pos,"tqe": tqe}))
        }
        Some(_) => Err("pos_classic is supported for hightorque only".to_string()),
        None => Err("motor not connected".to_string()),
    }
}

fn handle_version(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    let timeout_ms = as_u64(v, "timeout_ms", 500);
    match ctx.motor.as_ref() {
        Some(MotorHandle::Myactuator(m)) => {
            m.request_version_date().map_err(|e| e.to_string())?;
            let version = m
                .await_version_date(Duration::from_millis(timeout_ms))
                .map_err(|e| e.to_string())?;
            Ok(json!({"version": version}))
        }
        Some(_) => Err("version is supported for myactuator only".to_string()),
        None => Err("motor not connected".to_string()),
    }
}

fn handle_mode_query(ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    match (&ctx.controller, &ctx.motor) {
        (Some(ControllerHandle::Myactuator(c)), Some(MotorHandle::Myactuator(m))) => {
            m.request_control_mode().map_err(|e| e.to_string())?;
            c.poll_feedback_once().map_err(|e| e.to_string())?;
            Ok(json!({"mode": m.latest_control_mode()}))
        }
        (Some(_), Some(_)) => Err("mode_query is supported for myactuator only".to_string()),
        _ => Err("motor not connected".to_string()),
    }
}

fn handle_read(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    match (&ctx.controller, &ctx.motor) {
        (Some(ControllerHandle::Hightorque(_)), Some(MotorHandle::Hightorque(m))) => {
            // Converged (2026-09-29): the 17 01 query + wait + decode all go
            // through the vendor, physical values with torque coeff
            // compensation (the old hand-written path offset torque by ÷100).
            // Since 2026-10-21 the vendor blocking state query is
            // request_fresh_state (request_motor_feedback was returned to
            // DM non-blocking semantics).
            let timeout = Duration::from_millis(as_u64(v, "timeout_ms", 500));
            let s = m.request_fresh_state(timeout).map_err(|e| e.to_string())?;
            Ok(json!({
                "motor_id": s.can_id,
                "arbitration_id": s.arbitration_id,
                "pos": s.pos,
                "vel": s.vel,
                "torq": s.torq,
                "status_code": s.status_code,
                "fault_code": s.fault_code,
            }))
        }
        (Some(_), Some(_)) => Err("read op is reserved for hightorque".to_string()),
        _ => Err("motor not connected".to_string()),
    }
}

/// HighTorque generic register read (17 family): read `count` (1-3)
/// consecutive registers starting at `addr`. Since 2026-10-22: reads are
/// **fixed Int16**, the message `ty` field is removed — explicitly carrying
/// `ty` errors out (not silently ignored, so old clients find out
/// immediately when migrating); the return value is the **converted
/// physical value** (classified by the vendor register table quantity:
/// position/velocity → rad family, torque with model k, current/voltage/
/// temperature/PWM/PID per their own scale; registers with no conversion
/// 1:1). Each row's name/unit is resolved by that row's own address (not
/// the base address).
fn handle_read_register(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    match ctx.motor.as_ref() {
        Some(MotorHandle::Hightorque(m)) => {
            // B2 fix (2026-10-21): out-of-range addresses (> 0xFF) are
            // rejected outright, no more silent `as u8` truncation
            // (addr=300 → 0x2C reading the wrong register).
            let addr_raw = v
                .get("addr")
                .and_then(Value::as_u64)
                .ok_or_else(|| "read_register requires integer 'addr' (0x00-0xFF)".to_string())?;
            let addr = u8::try_from(addr_raw).map_err(|_| {
                format!("read_register 'addr' out of range 0x00-0xFF, got {addr_raw}")
            })?;
            if v.get("ty").is_some() {
                return Err(
                    "read_register 'ty' was removed: reads are fixed Int16 (2026-10-22)"
                        .to_string(),
                );
            }
            let count = as_u64(v, "count", 1).clamp(1, 3) as u8;
            let timeout = Duration::from_millis(as_u64(v, "timeout_ms", 500));
            let vals = m
                .read_registers(&[ReadCmd { addr, count }], timeout)
                .map_err(|e| e.to_string())?;
            // C2: resolve each row's name/unit by its own address, not the base address.
            let registers: Vec<Value> = vals
                .iter()
                .map(|(a, val)| {
                    let (name, unit) = match parameter_info(*a) {
                        Some(i) => (
                            json!(i.name),
                            json!(i.quantity.map(|q| q.unit_label()).unwrap_or("")),
                        ),
                        None => (json!("unknown"), json!("")),
                    };
                    json!({
                        "addr": a,
                        "name": name,
                        "unit": unit,
                        "value": val,
                    })
                })
                .collect();
            Ok(json!({
                "op": "read_register",
                "vendor": "hightorque",
                "addr": addr,
                "count": count,
                "registers": registers,
            }))
        }
        Some(_) => Err("read_register is supported for hightorque only".to_string()),
        None => Err("motor not connected".to_string()),
    }
}

/// HighTorque DQ voltage mode (`01 00 08 05 1B`, mode write + register
/// write packed into one frame): Q-axis voltage (V, 0.1 V/LSB). The protocol
/// only defines the Q component. One-shot command, does not enter
/// continuous (loop this op for sustained DQ voltage control).
fn handle_dq_voltage(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    let volt = as_f32(v, "volt", 0.0);
    match ctx.motor.as_ref() {
        Some(MotorHandle::Hightorque(m)) => {
            m.send_cmd_dq_voltage(volt).map_err(|e| e.to_string())?;
            Ok(json!({"op":"dq_voltage","volt": volt}))
        }
        Some(_) => Err("dq_voltage is supported for hightorque only".to_string()),
        None => Err("motor not connected".to_string()),
    }
}

/// HighTorque DQ current mode (`01 00 09 05 1C`, packed into one frame):
/// Q-axis current (A, 0.1 A/LSB).
fn handle_dq_current(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    let cur = as_f32(v, "cur", 0.0);
    match ctx.motor.as_ref() {
        Some(MotorHandle::Hightorque(m)) => {
            m.send_cmd_dq_current(cur).map_err(|e| e.to_string())?;
            Ok(json!({"op":"dq_current","cur": cur}))
        }
        Some(_) => Err("dq_current is supported for hightorque only".to_string()),
        None => Err("motor not connected".to_string()),
    }
}

/// HighTorque brake (table 2 mode 15): three-phase shorted damping brake.
/// One-shot command.
fn handle_brake(ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    match ctx.motor.as_ref() {
        Some(MotorHandle::Hightorque(m)) => {
            m.brake().map_err(|e| e.to_string())?;
            Ok(json!({"op":"brake","mode": 15}))
        }
        Some(_) => Err("brake is supported for hightorque only".to_string()),
        None => Err("motor not connected".to_string()),
    }
}

/// HighTorque run mode switch (`01 00 <mode>` writing mode register 0x000):
/// mode 0-17 (table 2; 0=stop and clear error, 8=DQ voltage, 9=DQ current,
/// 10=position, 15=brake). Actually sends a frame, unlike `ensure_mode`
/// (validation only).
fn handle_run_mode(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    let mode = v
        .get("mode")
        .and_then(Value::as_u64)
        .ok_or_else(|| "run_mode requires integer 'mode' (0-17)".to_string())?;
    if mode > 17 {
        return Err("run_mode 'mode' must be 0-17 (表2)".to_string());
    }
    match ctx.motor.as_ref() {
        Some(MotorHandle::Hightorque(m)) => {
            m.send_run_mode(mode as u8).map_err(|e| e.to_string())?;
            Ok(json!({"op":"run_mode","mode": mode}))
        }
        Some(_) => Err("run_mode is supported for hightorque only".to_string()),
        None => Err("motor not connected".to_string()),
    }
}

/// HighTorque generic register write (`05 <addr> <val16>`): int16 value.
/// Non-zero `config_class` means a config-class register (extended
/// 0x8000|id frame). Mode register 0x000 does not apply here — use
/// run_mode.
fn handle_write_register(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    // B2 fix (2026-10-21): same as the read side — out-of-range addresses
    // are rejected, no more as u8 truncation.
    let addr_raw = v
        .get("addr")
        .and_then(Value::as_u64)
        .ok_or_else(|| "write_register requires integer 'addr' (0x00-0xFF)".to_string())?;
    let addr = u8::try_from(addr_raw)
        .map_err(|_| format!("write_register 'addr' out of range 0x00-0xFF, got {addr_raw}"))?;
    let value = v
        .get("value")
        .and_then(Value::as_i64)
        .ok_or_else(|| "write_register requires integer 'value' (int16)".to_string())?;
    if !(-32768..=32767).contains(&value) {
        return Err(format!(
            "write_register 'value' must fit int16, got {value}"
        ));
    }
    let config_class = v.get("config_class").and_then(Value::as_u64).unwrap_or(0) != 0;
    let info = parameter_info(addr);
    match ctx.motor.as_ref() {
        Some(MotorHandle::Hightorque(m)) => {
            m.write_register(addr, value as i16, config_class)
                .map_err(|e| e.to_string())?;
            Ok(json!({
                "op": "write_register",
                "vendor": "hightorque",
                "addr": addr,
                "value": value,
                "config_class": config_class,
                "name": info.map(|i| i.name),
            }))
        }
        Some(_) => Err("write_register is supported for hightorque only".to_string()),
        None => Err("motor not connected".to_string()),
    }
}

/// HighTorque periodic report config (`05 B4 02 <t_ms16>`, extended frame):
/// `period_ms` >= 1 makes the motor periodically send 27 01 status frames
/// on its own (consumed into the state cache by the passive feedback path);
/// `period_ms`=0 stops reporting.
fn handle_periodic_report(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    let period_ms = v.get("period_ms").and_then(Value::as_u64).ok_or_else(|| {
        "periodic_report requires integer 'period_ms' (>=1 enable, 0 stop)".to_string()
    })?;
    if period_ms > u16::MAX as u64 {
        return Err(format!(
            "periodic_report 'period_ms' must fit uint16, got {period_ms}"
        ));
    }
    match ctx.motor.as_ref() {
        Some(MotorHandle::Hightorque(m)) => {
            m.set_periodic_report(period_ms as u16)
                .map_err(|e| e.to_string())?;
            Ok(json!({
                "op": "periodic_report",
                "period_ms": period_ms,
                "enabled": period_ms > 0,
            }))
        }
        Some(_) => Err("periodic_report is supported for hightorque only".to_string()),
        None => Err("motor not connected".to_string()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{Target, Transport, Vendor};
    use motor_core::bus::CanBus;
    use motor_core::test_support::MockBus;
    use motor_vendor_hightorque::HightorqueController;
    use std::sync::Arc;

    fn ht_ctx() -> (Arc<MockBus>, SessionCtx) {
        let bus: Arc<MockBus> = Arc::new(MockBus::new());
        let ctrl = HightorqueController::new(bus.clone() as Arc<dyn CanBus>);
        let motor = ctrl.add_motor(1, 1, "hightorque").expect("add motor");
        let target = Target {
            vendor: Vendor::Hightorque,
            transport: Transport::SocketCan,
            channel: "mock".to_string(),
            serial_port: String::new(),
            serial_baud: 0,
            dm_device_type: String::new(),
            dm_channel: String::new(),
            model: "hightorque".to_string(),
            motor_id: 1,
            feedback_id: 1,
        };
        (
            bus,
            SessionCtx {
                target,
                controller: Some(ControllerHandle::Hightorque(ctrl)),
                motor: Some(MotorHandle::Hightorque(motor)),
                active: None,
            },
        )
    }

    /// Review A2 regression: a non-numeric JSON type (string/bool/null) for
    /// pos_classic's tqe errors out, and the error happens before any
    /// control frame is sent — the limit is a safety field, a type error
    /// must not silently degrade to the 0x8000 unlimited torque sentinel.
    #[test]
    fn pos_classic_rejects_non_numeric_tqe_without_sending() {
        for bad in [json!("1.5"), json!(true), json!(null), json!([1.5])] {
            let (bus, mut ctx) = ht_ctx();
            let r = handle_pos_classic(&json!({ "pos": 1.0, "tqe": bad }), &mut ctx);
            let err = r.expect_err("non-numeric tqe must be rejected");
            assert!(err.contains("tqe"), "error should name the field: {err}");
            let sent = bus.sent.lock().expect("sent");
            assert!(
                sent.is_empty(),
                "no control frame may be sent on type error, got {sent:?}"
            );
        }
    }

    /// B2 regression (2026-10-21): out-of-range register addresses (> 0xFF)
    /// are rejected outright, no more silent `as u8` truncation. Both the
    /// read and write ops are verified (the old implementation truncated
    /// addr=300 to 0x2C, reading/writing the wrong register), and the
    /// rejection happens before any frame is sent (zero bus frames).
    #[test]
    fn read_write_register_reject_out_of_range_addr() {
        let (bus, mut ctx) = ht_ctx();
        let r = handle_read_register(&json!({ "addr": 300 }), &mut ctx);
        assert!(r.is_err());
        assert!(r.unwrap_err().contains("out of range"));

        let r = handle_write_register(&json!({ "addr": 0x145, "value": 0 }), &mut ctx);
        assert!(r.is_err());
        assert!(r.unwrap_err().contains("out of range"));

        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 0, "越界地址应在发帧前拒绝");

        // Valid-domain boundary: the argument domain ceiling 0xFF is now
        // out-of-table (rejected by the vendor table validation, see
        // read_write_register_rejects_unknown_table_addr); use the highest
        // in-table address 0x45 as the "argument passes and a frame goes
        // out" boundary case (it will time out, but the frame was sent)
        let (_bus, mut ctx) = ht_ctx();
        let _ = handle_read_register(&json!({ "addr": 0x45, "timeout_ms": 10 }), &mut ctx);
        let sent = _bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 1, "0x45 边界地址应正常发帧");
        assert_eq!(sent[0].data[1], 0x45);
    }

    /// 2026-10-22 follow-up decision regression: gap/out-of-table addresses
    /// are rejected — holes inside the 0x00-0xFF argument domain (gap 0x0A)
    /// and 0x46-0xFF (out-of-table 0x50/0xFF) all error out with zero bus
    /// frames (the vendor `ensure_known_span` rejects before sending); the
    /// write path rejects the same way.
    #[test]
    fn read_write_register_rejects_unknown_table_addr() {
        let (bus, mut ctx) = ht_ctx();
        for addr in [0x0Au32, 0x50, 0xFF] {
            let r = handle_read_register(&json!({ "addr": addr }), &mut ctx);
            assert!(r.is_err(), "读 0x{addr:02X} 应被表校验拒绝");
            assert!(
                r.unwrap_err().contains("not in the parameter table"),
                "报错应指明不在寄存器表"
            );
        }
        let r = handle_write_register(&json!({ "addr": 0x0A, "value": 0 }), &mut ctx);
        assert!(r.is_err(), "写缺号地址应被拒绝");
        assert!(r.unwrap_err().contains("not in the parameter table"));
        let r = handle_write_register(&json!({ "addr": 0x50, "value": 0 }), &mut ctx);
        assert!(r.is_err(), "写表外地址应被拒绝");
        assert!(r.unwrap_err().contains("not in the parameter table"));
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 0, "拒绝应发生在发帧之前");
    }

    /// 2026-10-22 decision regression: reads are fixed Int16 — the 0x0E
    /// (temperature) read frame has cmd=0x15 (read|Int16|count1), extended
    /// 0x8000|id, dlc 2. The mock bus never replies, so the blocking read
    /// timing out is expected.
    #[test]
    fn read_register_defaults_unverified_addr_to_int16() {
        let (bus, mut ctx) = ht_ctx();
        let r = handle_read_register(&json!({ "addr": 14, "timeout_ms": 10 }), &mut ctx);
        assert!(r.is_err(), "no reply on mock bus: timeout expected");
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 1, "exactly one register read frame");
        assert_eq!(sent[0].arbitration_id, 0x8000 | 1);
        assert!(sent[0].is_extended, "register reads go on 0x8000|id");
        assert_eq!(sent[0].dlc, 2);
        assert_eq!(
            &sent[0].data[..2],
            &[0x15, 0x0E],
            "cmd 0x15 = read | Int16 | count 1"
        );
    }

    /// 2026-10-22: a message explicitly carrying `ty` errors out (the field
    /// is removed, reads are fixed Int16), not silently ignored; the
    /// rejection happens before any frame is sent (zero bus frames).
    #[test]
    fn read_register_rejects_removed_ty_field() {
        let (bus, mut ctx) = ht_ctx();
        let r = handle_read_register(&json!({ "addr": 14, "ty": 1, "timeout_ms": 10 }), &mut ctx);
        let err = r.expect_err("ty must be rejected");
        assert!(err.contains("ty"), "error should name the field: {err}");
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 0, "ty 拒绝应在发帧前");
    }

    /// Field absent = intentionally unlimited torque (None); a number
    /// parses normally. Both legal shapes send the 07 07 frame as usual.
    #[test]
    fn pos_classic_accepts_absent_and_numeric_tqe() {
        let (bus, mut ctx) = ht_ctx();
        let r = handle_pos_classic(&json!({ "pos": 1.0 }), &mut ctx).expect("absent tqe ok");
        assert_eq!(r["tqe"], serde_json::Value::Null);
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 1, "absent tqe still sends the 07 07 frame");

        let (_bus, mut ctx) = ht_ctx();
        let r = handle_pos_classic(&json!({ "pos": 1.0, "tqe": 1.5 }), &mut ctx)
            .expect("numeric tqe ok");
        assert_eq!(r["tqe"], json!(1.5));
    }
}
