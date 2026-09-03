use super::*;
use motor_core::device::MotorDevice;

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_free(motor: *mut MotorHandle) {
    if motor.is_null() {
        return;
    }
    let _ = unsafe { Box::from_raw(motor) };
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_enable(motor: *mut MotorHandle) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::Damiao(m) => m.enable().map_err(|e| e.to_string()),
            MotorHandleInner::Hexfellow(m) => m
                .enable_drive(Duration::from_millis(200))
                .map_err(|e| e.to_string()),
            MotorHandleInner::MyActuator(m) => m.release_brake().map_err(|e| e.to_string()),
            MotorHandleInner::Robstride(m) => m.enable().map_err(|e| e.to_string()),
            MotorHandleInner::CyberBeast(m) => m.enable().map_err(|e| e.to_string()),
            MotorHandleInner::Hightorque(m) => m.enable().map_err(|e| e.to_string()),
        }
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_disable(motor: *mut MotorHandle) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::CyberBeast(m) => m.disable().map_err(|e| e.to_string()),
            MotorHandleInner::Damiao(m) => m.disable().map_err(|e| e.to_string()),
            MotorHandleInner::Hexfellow(m) => m
                .disable_drive(Duration::from_millis(200))
                .map_err(|e| e.to_string()),
            MotorHandleInner::MyActuator(m) => m.shutdown_motor().map_err(|e| e.to_string()),
            MotorHandleInner::Robstride(m) => m.disable().map_err(|e| e.to_string()),
            MotorHandleInner::Hightorque(m) => m.disable().map_err(|e| e.to_string()),
        }
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_clear_error(motor: *mut MotorHandle) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::CyberBeast(m) => m.send_clear_errors().map_err(|e| e.to_string()),
            MotorHandleInner::Damiao(m) => m.clear_error().map_err(|e| e.to_string()),
            MotorHandleInner::Hexfellow(_) => {
                Err("clear_error is not supported for Hexfellow".to_string())
            }
            MotorHandleInner::MyActuator(m) => m.stop_motor().map_err(|e| e.to_string()),
            MotorHandleInner::Robstride(m) => m.clear_error().map_err(|e| e.to_string()),
            MotorHandleInner::Hightorque(m) => m.clear_error().map_err(|e| e.to_string()),
        }
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_set_zero_position(motor: *mut MotorHandle) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::Damiao(m) => m.set_zero_position().map_err(|e| e.to_string()),
            MotorHandleInner::Hexfellow(_) => {
                Err("set_zero_position is not supported for Hexfellow".to_string())
            }
            MotorHandleInner::MyActuator(_) => {
                Err("set_zero_position is not supported for MyActuator".to_string())
            }
            MotorHandleInner::CyberBeast(m) => m.send_set_zero().map_err(|e| e.to_string()),
            MotorHandleInner::Robstride(m) => m.set_zero_position().map_err(|e| e.to_string()),
            MotorHandleInner::Hightorque(m) => m.set_zero_position().map_err(|e| e.to_string()),
        }
    })
}
