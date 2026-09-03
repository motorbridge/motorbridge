use super::*;

macro_rules! add_motor_ffi {
    ($fn_name:ident, $ensure_fn:ident, $inner_variant:ident) => {
        #[unsafe(no_mangle)]
        pub extern "C" fn $fn_name(
            controller: *mut MotorController,
            motor_id: u16,
            feedback_id: u16,
            model: *const c_char,
        ) -> *mut MotorHandle {
            if controller.is_null() {
                set_last_error("controller is null");
                return ptr::null_mut();
            }
            let model = match parse_cstr(model, "model") {
                Ok(v) => v,
                Err(e) => {
                    set_last_error(e);
                    return ptr::null_mut();
                }
            };
            let controller = unsafe { &*controller };
            let mut inner = match controller.inner.lock() {
                Ok(inner) => inner,
                Err(_) => {
                    set_last_error("controller lock poisoned");
                    return ptr::null_mut();
                }
            };
            match $ensure_fn(&mut *inner).and_then(|ctrl| {
                ctrl.add_motor(motor_id, feedback_id, &model)
                    .map_err(|e| e.to_string())
            }) {
                Ok(motor) => Box::into_raw(Box::new(MotorHandle {
                    inner: Mutex::new(MotorHandleInner::$inner_variant(motor)),
                })),
                Err(e) => {
                    set_last_error(e);
                    ptr::null_mut()
                }
            }
        }
    };
}

add_motor_ffi!(
    motor_controller_add_damiao_motor,
    ensure_damiao_controller,
    Damiao
);
add_motor_ffi!(
    motor_controller_add_hexfellow_motor,
    ensure_hexfellow_controller,
    Hexfellow
);
add_motor_ffi!(
    motor_controller_add_robstride_motor,
    ensure_robstride_controller,
    Robstride
);
add_motor_ffi!(
    motor_controller_add_myactuator_motor,
    ensure_myactuator_controller,
    MyActuator
);
add_motor_ffi!(
    motor_controller_add_hightorque_motor,
    ensure_hightorque_controller,
    Hightorque
);
add_motor_ffi!(
    motor_controller_add_cyberbeast_motor,
    ensure_cyberbeast_controller,
    CyberBeast
);
