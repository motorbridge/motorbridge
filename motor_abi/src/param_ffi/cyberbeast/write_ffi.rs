use super::super::common::ffi_run;
use crate::vendor_params::cyberbeast;
use crate::MotorHandle;

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_cyberbeast_write_param_f32(
    motor: *mut MotorHandle,
    param_id: u16,
    value: f32,
) -> i32 {
    ffi_run(motor, |m| cyberbeast::write_f32(m, param_id, value))
}
