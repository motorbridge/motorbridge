use super::*;

macro_rules! dispatch_controller {
    ($inner:expr, $method:ident) => {
        match $inner {
            ControllerInner::Damiao(ctrl) => ctrl.$method().map_err(|e| e.to_string()),
            ControllerInner::Hexfellow(ctrl) => ctrl.$method().map_err(|e| e.to_string()),
            ControllerInner::MyActuator(ctrl) => ctrl.$method().map_err(|e| e.to_string()),
            ControllerInner::Robstride(ctrl) => ctrl.$method().map_err(|e| e.to_string()),
            ControllerInner::Hightorque(ctrl) => ctrl.$method().map_err(|e| e.to_string()),
            ControllerInner::Unbound(_) => Err(
                "controller has no motor; add a motor before calling this operation".to_string(),
            ),
            ControllerInner::UnboundMcuSerial { .. } => Err(
                "controller has no motor; add a motor before calling this operation".to_string(),
            ),
        }
    };
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_last_error_message() -> *const c_char {
    ok_ptr()
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_abi_version() -> *const c_char {
    concat!(env!("CARGO_PKG_VERSION"), "\0").as_ptr() as *const c_char
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_abi_capabilities_json() -> *const c_char {
    abi_capabilities_json().as_ptr()
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_controller_new_socketcan(channel: *const c_char) -> *mut MotorController {
    let channel = match parse_cstr(channel, "channel") {
        Ok(v) => v,
        Err(e) => {
            set_last_error(e);
            return ptr::null_mut();
        }
    };
    // Lazy: store the channel name; the SocketCAN bus is opened on first
    // `add_*_motor` (legacy timing). No hardware contact here.
    Box::into_raw(Box::new(MotorController {
        inner: Mutex::new(ControllerInner::Unbound(channel)),
    }))
}
#[unsafe(no_mangle)]
pub extern "C" fn motor_controller_new_socketcanfd(channel: *const c_char) -> *mut MotorController {
    let channel = match parse_cstr(channel, "channel") {
        Ok(v) => v,
        Err(e) => {
            set_last_error(e);
            return ptr::null_mut();
        }
    };
    Box::into_raw(Box::new(MotorController {
        inner: Mutex::new(ControllerInner::Unbound(channel)),
    }))
}
#[unsafe(no_mangle)]
pub extern "C" fn motor_controller_new_dm_serial(
    serial_port: *const c_char,
    baud: u32,
) -> *mut MotorController {
    let serial_port = match parse_cstr(serial_port, "serial_port") {
        Ok(v) => v,
        Err(e) => {
            set_last_error(e);
            return ptr::null_mut();
        }
    };
    let controller = match DamiaoController::new_dm_serial(&serial_port, baud) {
        Ok(c) => c,
        Err(e) => {
            set_last_error(e.to_string());
            return ptr::null_mut();
        }
    };
    Box::into_raw(Box::new(MotorController {
        inner: Mutex::new(ControllerInner::Damiao(controller)),
    }))
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_controller_new_dm_device(
    dm_device_type: *const c_char,
    dm_channel: *const c_char,
) -> *mut MotorController {
    let dm_device_type = match parse_cstr(dm_device_type, "dm_device_type") {
        Ok(v) => v,
        Err(e) => {
            set_last_error(e);
            return ptr::null_mut();
        }
    };
    let dm_channel = match parse_cstr(dm_channel, "dm_channel") {
        Ok(v) => v,
        Err(e) => {
            set_last_error(e);
            return ptr::null_mut();
        }
    };
    let device_type = match DmDeviceType::parse(&dm_device_type) {
        Ok(v) => v,
        Err(e) => {
            set_last_error(e.to_string());
            return ptr::null_mut();
        }
    };
    let controller = match DamiaoController::new_dm_device(device_type, &dm_channel) {
        Ok(c) => c,
        Err(e) => {
            set_last_error(e.to_string());
            return ptr::null_mut();
        }
    };
    Box::into_raw(Box::new(MotorController {
        inner: Mutex::new(ControllerInner::Damiao(controller)),
    }))
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_controller_new_mcu_serial(
    serial_port: *const c_char,
    baud: u32,
) -> *mut MotorController {
    let serial_port = match parse_cstr(serial_port, "serial_port") {
        Ok(v) => v,
        Err(e) => {
            set_last_error(e);
            return ptr::null_mut();
        }
    };
    // mcu-serial is a sibling of socketcan: store the port spec and defer
    // opening the bus until a motor is added (legacy lazy timing). Left
    // UnboundMcuSerial so any classic-CAN vendor (damiao/robstride/myactuator/
    // hightorque) can bind later; hexfellow (CAN-FD) is not supported on this link.
    Box::into_raw(Box::new(MotorController {
        inner: Mutex::new(ControllerInner::UnboundMcuSerial {
            port: serial_port,
            baud,
        }),
    }))
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_controller_free(controller: *mut MotorController) {
    if controller.is_null() {
        return;
    }
    let _ = unsafe { Box::from_raw(controller) };
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_controller_poll_feedback_once(controller: *mut MotorController) -> i32 {
    if controller.is_null() {
        set_last_error("controller is null");
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
    let rc = dispatch_controller!(&mut *inner, poll_feedback_once);
    ffi_rc(rc)
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_controller_enable_all(controller: *mut MotorController) -> i32 {
    if controller.is_null() {
        set_last_error("controller is null");
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
    let rc = dispatch_controller!(&mut *inner, enable_all);
    ffi_rc(rc)
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_controller_disable_all(controller: *mut MotorController) -> i32 {
    if controller.is_null() {
        set_last_error("controller is null");
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
    let rc = dispatch_controller!(&mut *inner, disable_all);
    ffi_rc(rc)
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_controller_shutdown(controller: *mut MotorController) -> i32 {
    if controller.is_null() {
        set_last_error("controller is null");
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
    let rc = dispatch_controller!(&mut *inner, shutdown);
    ffi_rc(rc)
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_controller_close_bus(controller: *mut MotorController) -> i32 {
    if controller.is_null() {
        set_last_error("controller is null");
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
    let rc = dispatch_controller!(&mut *inner, close_bus);
    ffi_rc(rc)
}
