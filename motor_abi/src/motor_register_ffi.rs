use super::*;

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_set_can_timeout_ms(motor: *mut MotorHandle, timeout_ms: u32) -> i32 {
    if motor.is_null() {
        set_last_error("motor is null");
        return -1;
    }
    let reg_value = timeout_ms.saturating_mul(20);
    let motor = lock_motor_inner!(motor, "motor is null");
    let rc = match &*motor {
        MotorHandleInner::Damiao(m) => m
            .write_register_u32(9, reg_value)
            .map_err(|e| e.to_string()),
        MotorHandleInner::Hexfellow(_) => {
            Err("set_can_timeout_ms is not supported for Hexfellow".to_string())
        }
        MotorHandleInner::MyActuator(_) => {
            Err("set_can_timeout_ms is not supported for MyActuator".to_string())
        }
        MotorHandleInner::Robstride(m) => m
            .write_parameter(0x7028, ParameterValue::U32(timeout_ms))
            .map_err(|e| e.to_string()),
        MotorHandleInner::Hightorque(_) => {
            Err("set_can_timeout_ms is not supported for HighTorque".to_string())
        }
        // Manual has no 6099h-equivalent watchdog; heartbeat consumption is
        // a future item (0x1016h on the master side).
        MotorHandleInner::Huayi(_) => {
            Err("set_can_timeout_ms is not supported for Huayi".to_string())
        }
    };
    ffi_rc(rc)
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_write_register_f32(
    motor: *mut MotorHandle,
    rid: u8,
    value: f32,
) -> i32 {
    if motor.is_null() {
        set_last_error("motor is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let rc = match &*motor {
        MotorHandleInner::Damiao(m) => m.write_register_f32(rid, value).map_err(|e| e.to_string()),
        MotorHandleInner::Huayi(_) => Err(
            "u8 rid cannot address 16-bit CANopen index; use motor_handle_huayi_sdo_write_f32"
                .to_string(),
        ),
        _ => Err("Damiao register write is only available for Damiao motors".to_string()),
    };
    ffi_rc(rc)
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_write_register_u32(
    motor: *mut MotorHandle,
    rid: u8,
    value: u32,
) -> i32 {
    if motor.is_null() {
        set_last_error("motor is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let rc = match &*motor {
        MotorHandleInner::Damiao(m) => m.write_register_u32(rid, value).map_err(|e| e.to_string()),
        MotorHandleInner::Huayi(_) => Err(
            "u8 rid cannot address 16-bit CANopen index; use motor_handle_huayi_sdo_write_i32 for 32-bit values (a u32 SDO variant is not exported)"
                .to_string(),
        ),
        _ => Err("Damiao register write is only available for Damiao motors".to_string()),
    };
    ffi_rc(rc)
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_get_register_f32(
    motor: *mut MotorHandle,
    rid: u8,
    timeout_ms: u32,
    out_value: *mut f32,
) -> i32 {
    if motor.is_null() || out_value.is_null() {
        set_last_error("motor or out_value is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let out = unsafe { &mut *out_value };
    let rc = match &*motor {
        MotorHandleInner::Damiao(m) => m
            .get_register_f32(rid, Duration::from_millis(timeout_ms as u64))
            .map_err(|e| e.to_string())
            .map(|v| *out = v),
        MotorHandleInner::Huayi(_) => Err(
            "u8 rid cannot address 16-bit CANopen index; use motor_handle_huayi_sdo_read_f32"
                .to_string(),
        ),
        _ => Err("Damiao register read is only available for Damiao motors".to_string()),
    };
    ffi_rc(rc)
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_get_register_u32(
    motor: *mut MotorHandle,
    rid: u8,
    timeout_ms: u32,
    out_value: *mut u32,
) -> i32 {
    if motor.is_null() || out_value.is_null() {
        set_last_error("motor or out_value is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let out = unsafe { &mut *out_value };
    let rc = match &*motor {
        MotorHandleInner::Damiao(m) => m
            .get_register_u32(rid, Duration::from_millis(timeout_ms as u64))
            .map_err(|e| e.to_string())
            .map(|v| *out = v),
        MotorHandleInner::Huayi(_) => Err(
            "u8 rid cannot address 16-bit CANopen index; use motor_handle_huayi_sdo_read_i32 for 32-bit values (a u32 SDO variant is not exported)"
                .to_string(),
        ),
        _ => Err("Damiao register read is only available for Damiao motors".to_string()),
    };
    ffi_rc(rc)
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_robstride_ping(
    motor: *mut MotorHandle,
    out_device_id: *mut u8,
    out_responder_id: *mut u8,
) -> i32 {
    if motor.is_null() || out_device_id.is_null() || out_responder_id.is_null() {
        set_last_error("motor or output pointer is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let rc = match &*motor {
        MotorHandleInner::Robstride(m) => m
            .ping(Duration::from_millis(500))
            .map_err(|e| e.to_string()),
        MotorHandleInner::Damiao(_)
        | MotorHandleInner::Hexfellow(_)
        | MotorHandleInner::MyActuator(_)
        | MotorHandleInner::Hightorque(_)
        | MotorHandleInner::Huayi(_) => {
            Err("robstride_ping requires a RobStride motor".to_string())
        }
    };
    match rc {
        Ok(reply) => {
            unsafe {
                *out_device_id = reply.device_id;
                *out_responder_id = reply.responder_id;
            }
            0
        }
        Err(e) => {
            set_last_error(e);
            -1
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_robstride_ping_host_id(
    motor: *mut MotorHandle,
    host_id: u16,
    timeout_ms: u32,
    out_device_id: *mut u8,
    out_responder_id: *mut u8,
) -> i32 {
    if motor.is_null() || out_device_id.is_null() || out_responder_id.is_null() {
        set_last_error("motor or output pointer is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let rc = match &*motor {
        MotorHandleInner::Robstride(m) => m
            .ping_with_host_id(host_id, Duration::from_millis(timeout_ms as u64))
            .map_err(|e| e.to_string()),
        MotorHandleInner::Damiao(_)
        | MotorHandleInner::Hexfellow(_)
        | MotorHandleInner::MyActuator(_)
        | MotorHandleInner::Hightorque(_)
        | MotorHandleInner::Huayi(_) => {
            Err("robstride_ping_host_id requires a RobStride motor".to_string())
        }
    };
    match rc {
        Ok(reply) => {
            unsafe {
                *out_device_id = reply.device_id;
                *out_responder_id = reply.responder_id;
            }
            0
        }
        Err(e) => {
            set_last_error(e);
            -1
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_robstride_get_param_f32_host_id(
    motor: *mut MotorHandle,
    param_id: u16,
    host_id: u16,
    timeout_ms: u32,
    out_value: *mut f32,
) -> i32 {
    if motor.is_null() || out_value.is_null() {
        set_last_error("motor or output pointer is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let out = unsafe { &mut *out_value };
    let rc = match &*motor {
        MotorHandleInner::Robstride(m) => m
            .get_parameter_with_host_id(param_id, host_id, Duration::from_millis(timeout_ms as u64))
            .and_then(|value| match value {
                ParameterValue::F32(v) => Ok(v),
                _ => Err(motor_core::error::MotorError::Protocol(format!(
                    "parameter 0x{param_id:04X} is not f32"
                ))),
            })
            .map_err(|e| e.to_string())
            .map(|v| *out = v),
        MotorHandleInner::Damiao(_)
        | MotorHandleInner::Hexfellow(_)
        | MotorHandleInner::MyActuator(_)
        | MotorHandleInner::Hightorque(_)
        | MotorHandleInner::Huayi(_) => {
            Err("robstride_get_param_f32_host_id requires a RobStride motor".to_string())
        }
    };
    ffi_rc(rc)
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_robstride_get_fault_report(
    motor: *mut MotorHandle,
    out_fault_raw: *mut u32,
    out_warning_raw: *mut u32,
) -> i32 {
    if motor.is_null() || out_fault_raw.is_null() || out_warning_raw.is_null() {
        set_last_error("motor or output pointer is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    match &*motor {
        MotorHandleInner::Robstride(m) => {
            let report = m.latest_fault_report();
            unsafe {
                *out_fault_raw = report.map(|r| r.fault_raw).unwrap_or(0);
                *out_warning_raw = report.map(|r| r.warning_raw).unwrap_or(0);
            }
            0
        }
        MotorHandleInner::Damiao(_)
        | MotorHandleInner::Hexfellow(_)
        | MotorHandleInner::MyActuator(_)
        | MotorHandleInner::Hightorque(_)
        | MotorHandleInner::Huayi(_) => {
            set_last_error("robstride_get_fault_report requires a RobStride motor");
            -1
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_robstride_set_device_id(
    motor: *mut MotorHandle,
    new_device_id: u8,
) -> i32 {
    if motor.is_null() {
        set_last_error("motor is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let rc = match &*motor {
        MotorHandleInner::Robstride(m) => m.set_device_id(new_device_id).map_err(|e| e.to_string()),
        MotorHandleInner::Damiao(_)
        | MotorHandleInner::Hexfellow(_)
        | MotorHandleInner::MyActuator(_)
        | MotorHandleInner::Hightorque(_)
        | MotorHandleInner::Huayi(_) => {
            Err("robstride_set_device_id requires a RobStride motor".to_string())
        }
    };
    ffi_rc(rc)
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_robstride_set_active_report(
    motor: *mut MotorHandle,
    enabled: u8,
) -> i32 {
    if motor.is_null() {
        set_last_error("motor is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let rc = match &*motor {
        MotorHandleInner::Robstride(m) => {
            m.set_active_report(enabled != 0).map_err(|e| e.to_string())
        }
        MotorHandleInner::Damiao(_)
        | MotorHandleInner::Hexfellow(_)
        | MotorHandleInner::MyActuator(_)
        | MotorHandleInner::Hightorque(_)
        | MotorHandleInner::Huayi(_) => {
            Err("robstride_set_active_report requires a RobStride motor".to_string())
        }
    };
    ffi_rc(rc)
}

/// HighTorque generic register read (protocol §1.3 `17 <cmd> <addr>` family).
///
/// Since 2026-10-22: reads are **fixed Int16** (`cmd[3:2]` always 01); the `ty`
/// parameter has been removed from the signature. The return value is the
/// **converted physical quantity** (position/velocity/acceleration → rad
/// family, torque via model k/d, current/voltage/temperature/PWM/PID per
/// their own scale; registers with no conversion and out-of-table addresses
/// return raw 1:1).
///
/// - `addr`: register address (table 1, 0x00–0x45)
/// - `count`: number of consecutive registers to read (1–3); addresses
///   increment from `addr` (wrapping past 0xFF)
/// - `out_values`: receives `count` f32 values (converted physical quantities)
///
/// Simplified wrapper for single-address reads; multi-segment arbitrary
/// address-list reads remain in the vendor `read_registers`.
/// HighTorque motors only.
#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_hightorque_get_register(
    motor: *mut MotorHandle,
    addr: u8,
    count: u8,
    timeout_ms: u32,
    out_values: *mut f32,
) -> i32 {
    if motor.is_null() || out_values.is_null() {
        set_last_error("motor or out_values is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let rc = match &*motor {
        MotorHandleInner::Hightorque(m) => m
            .read_registers(
                &[ReadCmd { addr, count }],
                Duration::from_millis(timeout_ms as u64),
            )
            .map(|vals| {
                // The returned list increments by count consecutively; when
                // fewer than count values arrive, write out what exists.
                unsafe {
                    for (i, (_, v)) in vals.iter().take(count as usize).enumerate() {
                        *out_values.add(i) = *v;
                    }
                };
            })
            .map_err(|e| e.to_string()),
        _ => Err("hightorque_get_register requires a HighTorque motor".to_string()),
    };
    ffi_rc(rc)
}

/// HighTorque non-blocking register query (DM-style "request and send"):
/// only sends the `17` family read request frame (**fixed Int16** since
/// 2026-10-22) and returns immediately without waiting for a reply. The
/// reply is decoded by the CoreController background receive thread and
/// written into the motor register cache by address (with the receive
/// timestamp); afterwards `hightorque_get_cached_register` reads the most
/// recent value (non-blocking, already converted to a physical quantity),
/// or `hightorque_get_register`/vendor `read_registers` blocks for a fresh
/// value. Intended for fixed-rate monitoring loops (fire and forget, read
/// the cache on the next tick).
///
/// - `addr`: register address (table 1, 0x00–0x45)
/// - `count`: number of consecutive registers to read (1–3)
///
/// HighTorque motors only.
#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_hightorque_request_register(
    motor: *mut MotorHandle,
    addr: u8,
    count: u8,
) -> i32 {
    if motor.is_null() {
        set_last_error("motor is null");
        return -1;
    }
    if !(1..=3).contains(&count) {
        set_last_error("count must be in 1..=3");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let rc = match &*motor {
        MotorHandleInner::Hightorque(m) => m
            .request_registers(&[ReadCmd { addr, count }])
            .map_err(|e| e.to_string()),
        _ => Err("hightorque_request_register requires a HighTorque motor".to_string()),
    };
    ffi_rc(rc)
}

/// HighTorque read of the most recent cached register value (sends no
/// frame, does not wait). Writes 1 to `out_has_value` and the **converted
/// physical quantity** (`f32`) to `out_value`; writes 0 to `out_has_value`
/// if no reply for this address has ever been received. Note: freshness is
/// not distinguished (the last value persists after the motor goes
/// offline); for a fresh value use `hightorque_get_register`, or first
/// send a query via `hightorque_request_register` and read this cache one
/// tick later.
/// HighTorque motors only.
#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_hightorque_get_cached_register(
    motor: *mut MotorHandle,
    addr: u8,
    out_has_value: *mut u8,
    out_value: *mut f32,
) -> i32 {
    if motor.is_null() || out_has_value.is_null() || out_value.is_null() {
        set_last_error("motor or out pointers are null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let rc = match &*motor {
        MotorHandleInner::Hightorque(m) => {
            match m.cached_register(addr) {
                Some(v) => unsafe {
                    *out_has_value = 1;
                    *out_value = v;
                },
                None => unsafe { *out_has_value = 0 },
            }
            Ok(())
        }
        _ => Err("hightorque_get_cached_register requires a HighTorque motor".to_string()),
    };
    ffi_rc(rc)
}

#[cfg(test)]
mod tests {
    use super::*;
    use motor_core::bus::{CanBus, CanFrame};
    use motor_core::device::MotorDevice;
    use motor_core::test_support::MockBus;
    use motor_vendor_hightorque::HightorqueController;

    fn ht_handle() -> (Arc<MockBus>, MotorHandle) {
        let bus = Arc::new(MockBus::new());
        let ctrl = HightorqueController::new(Arc::clone(&bus) as Arc<dyn CanBus>);
        let motor = ctrl.add_motor(1, 0x101, "ht").expect("add motor");
        (
            bus,
            MotorHandle {
                inner: Mutex::new(MotorHandleInner::Hightorque(motor)),
            },
        )
    }

    fn rx_frame(id: u32, data: [u8; 8], dlc: u8, ext: bool) -> CanFrame {
        CanFrame {
            arbitration_id: id,
            data,
            dlc,
            is_extended: ext,
            is_rx: true,
        }
    }

    /// F4: generic register read, fixed Int16 single address (count=1).
    /// Reply is 0x25 (cmd = reply nibble|Int16<<2|count) + addr + i16
    /// value. 0x0F (error code, unclassified in the table) → no
    /// conversion, returns raw 1:1. After the 2026-10-22 follow-up,
    /// gap/out-of-table addresses are rejected at the vendor layer for
    /// the whole span (see hightorque_get_register_rejects_unknown_addr).
    #[test]
    fn hightorque_get_register_int16_single() {
        let (bus, mut handle) = ht_handle();
        // The reply must enter the cache after the request (timestamp
        // freshness): simulate the background polling thread delivering
        // the register reply 10ms after the request is sent.
        let deliver = {
            let m = match &*handle.inner.lock().expect("inner") {
                MotorHandleInner::Hightorque(m) => Arc::clone(m),
                _ => panic!("expected hightorque motor"),
            };
            std::thread::spawn(move || {
                std::thread::sleep(Duration::from_millis(10));
                m.process_feedback_frame(rx_frame(
                    0x0100,
                    [0x25, 0x0F, 0x34, 0x12, 0, 0, 0, 0],
                    4,
                    true,
                ))
                .expect("merge reply");
            })
        };
        let mut out = [0f32; 3];
        let rc = motor_handle_hightorque_get_register(
            (&mut handle) as *mut MotorHandle,
            0x0F,
            1,
            500,
            out.as_mut_ptr(),
        );
        deliver.join().expect("join deliver thread");
        assert_eq!(rc, 0);
        assert_eq!(out[0], 0x1234 as f32);
        // Query frame: extended 0x8000|id, payload = read command 0x15
        // (0x1X family|Int16<<2|count1) + addr
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 1);
        assert_eq!(sent[0].arbitration_id, 0x8001);
        assert_eq!(&sent[0].data[..2], &[0x15, 0x0F]);
    }

    /// F4: gap/out-of-table addresses are rejected (2026-10-22 follow-up
    /// decision) — the read returns rc=-1 with zero frames on the bus
    /// (rejection happens before sending).
    #[test]
    fn hightorque_get_register_rejects_unknown_addr() {
        let (bus, mut handle) = ht_handle();
        let mut out = [0f32; 3];
        // Gap address (0x0A)
        let rc = motor_handle_hightorque_get_register(
            (&mut handle) as *mut MotorHandle,
            0x0A,
            1,
            50,
            out.as_mut_ptr(),
        );
        assert_eq!(rc, -1);
        // Out-of-table (0x50)
        let rc = motor_handle_hightorque_get_register(
            (&mut handle) as *mut MotorHandle,
            0x50,
            1,
            50,
            out.as_mut_ptr(),
        );
        assert_eq!(rc, -1);
        // A consecutive read crossing into a gap (0x2D in-table + 0x2E
        // gap) is rejected for the whole span
        let rc = motor_handle_hightorque_get_register(
            (&mut handle) as *mut MotorHandle,
            0x2D,
            2,
            50,
            out.as_mut_ptr(),
        );
        assert_eq!(rc, -1);
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 0, "拒绝应发生在发帧之前");
    }

    /// F4: non-blocking register query + cached-read round trip:
    /// request_register returns right after sending; get_cached_register
    /// returns has=0 before the reply enters the cache, and the converted
    /// physical quantity afterwards (0x0E temperature: raw 100 → 10.0°C).
    #[test]
    fn hightorque_request_register_and_cached_register_roundtrip() {
        let (bus, mut handle) = ht_handle();
        let rc =
            motor_handle_hightorque_request_register((&mut handle) as *mut MotorHandle, 0x0E, 1);
        assert_eq!(rc, 0);
        // Query frame: extended 0x8000|id, payload 0x15 0x0E (fixed
        // int16 count1 read of 0x0E)
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 1);
        assert_eq!(sent[0].arbitration_id, 0x8001);
        assert_eq!(&sent[0].data[..2], &[0x15, 0x0E]);
        drop(sent);

        let (mut has, mut val) = (0u8, 0f32);
        let rc = motor_handle_hightorque_get_cached_register(
            (&mut handle) as *mut MotorHandle,
            0x0E,
            &mut has,
            &mut val,
        );
        assert_eq!(rc, 0);
        assert_eq!(has, 0, "入缓前应无值");

        // Simulate the passive path (background polling thread) entering
        // the cache
        let m = match &*handle.inner.lock().expect("inner") {
            MotorHandleInner::Hightorque(m) => Arc::clone(m),
            _ => panic!("expected hightorque motor"),
        };
        m.process_feedback_frame(rx_frame(
            0x0100,
            [0x25, 0x0E, 0x64, 0x00, 0, 0, 0, 0],
            4,
            true,
        ))
        .expect("merge");
        let rc = motor_handle_hightorque_get_cached_register(
            (&mut handle) as *mut MotorHandle,
            0x0E,
            &mut has,
            &mut val,
        );
        assert_eq!(rc, 0);
        assert_eq!(has, 1);
        assert!((val - 10.0).abs() < 1e-4, "raw 100 → 10.0°C, 实得 {val}");
    }

    /// 2026-10-22: reads return converted physical quantities. 0x03
    /// torque, "ht"→General k=0.5: raw 2560 → 2560×0.01×0.5 = 12.8 N·m.
    #[test]
    fn hightorque_get_register_converts_torque_with_model_coeff() {
        let (bus, mut handle) = ht_handle();
        let deliver = {
            let m = match &*handle.inner.lock().expect("inner") {
                MotorHandleInner::Hightorque(m) => Arc::clone(m),
                _ => panic!("expected hightorque motor"),
            };
            std::thread::spawn(move || {
                std::thread::sleep(Duration::from_millis(10));
                m.process_feedback_frame(rx_frame(
                    0x0100,
                    [0x25, 0x03, 0x00, 0x0A, 0, 0, 0, 0],
                    4,
                    true,
                ))
                .expect("merge reply");
            })
        };
        let mut out = [0f32; 1];
        let rc = motor_handle_hightorque_get_register(
            (&mut handle) as *mut MotorHandle,
            0x03,
            1,
            500,
            out.as_mut_ptr(),
        );
        deliver.join().expect("join deliver thread");
        assert_eq!(rc, 0);
        assert!(
            (out[0] - 12.8).abs() < 1e-4,
            "raw 2560 × 0.01 × k(0.5) = 12.8, 实得 {}",
            out[0]
        );
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(&sent[0].data[..2], &[0x15, 0x03]);
    }

    /// F5: get_state outputs fault_code (table 3, status-frame byte1).
    #[test]
    fn hightorque_get_state_includes_fault_code() {
        let (_bus, mut handle) = ht_handle();
        let motor = match &*handle.inner.lock().expect("lock") {
            MotorHandleInner::Hightorque(m) => Arc::clone(m),
            _ => panic!("expected hightorque motor"),
        };
        // Status frame attached to a control command: byte0 = run mode
        // (table 2), byte1 = fault code (table 3).
        // (27 01 register read replies carry no fault code; status/fault
        // are both 0.)
        motor
            .process_feedback_frame(rx_frame(
                0x0100,
                [0x05, 0x07, 0x34, 0x12, 0x78, 0x56, 0x00, 0x0A],
                8,
                false,
            ))
            .expect("process");
        let mut state = MotorState::default();
        let rc =
            crate::state_ffi::motor_handle_get_state((&mut handle) as *mut MotorHandle, &mut state);
        assert_eq!(rc, 0);
        assert_eq!(state.has_value, 1);
        assert_eq!(state.fault_code, 0x07);
        assert_eq!(state.status_code, 0x05);
    }
}
