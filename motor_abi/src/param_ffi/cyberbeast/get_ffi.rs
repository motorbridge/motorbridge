use super::super::common::ffi_get;
use crate::vendor_params::cyberbeast;
use crate::MotorHandle;

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_cyberbeast_get_param_f32(
    motor: *mut MotorHandle,
    param_id: u16,
    timeout_ms: u32,
    out_value: *mut f32,
) -> i32 {
    ffi_get(motor, out_value, |m| {
        cyberbeast::get_f32(m, param_id, timeout_ms)
    })
}
