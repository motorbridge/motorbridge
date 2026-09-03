macro_rules! define_param_get_ffis_5 {
    (
        $vendor_mod:ident,
        $fn_i8:ident,
        $fn_u8:ident,
        $fn_u16:ident,
        $fn_u32:ident,
        $fn_f32:ident
    ) => {
        #[unsafe(no_mangle)]
        pub extern "C" fn $fn_i8(
            motor: *mut crate::MotorHandle,
            param_id: u16,
            timeout_ms: u32,
            out_value: *mut i8,
        ) -> i32 {
            crate::param_ffi::common::ffi_get(motor, out_value, |m| {
                $vendor_mod::get_i8(m, param_id, timeout_ms)
            })
        }

        #[unsafe(no_mangle)]
        pub extern "C" fn $fn_u8(
            motor: *mut crate::MotorHandle,
            param_id: u16,
            timeout_ms: u32,
            out_value: *mut u8,
        ) -> i32 {
            crate::param_ffi::common::ffi_get(motor, out_value, |m| {
                $vendor_mod::get_u8(m, param_id, timeout_ms)
            })
        }

        #[unsafe(no_mangle)]
        pub extern "C" fn $fn_u16(
            motor: *mut crate::MotorHandle,
            param_id: u16,
            timeout_ms: u32,
            out_value: *mut u16,
        ) -> i32 {
            crate::param_ffi::common::ffi_get(motor, out_value, |m| {
                $vendor_mod::get_u16(m, param_id, timeout_ms)
            })
        }

        #[unsafe(no_mangle)]
        pub extern "C" fn $fn_u32(
            motor: *mut crate::MotorHandle,
            param_id: u16,
            timeout_ms: u32,
            out_value: *mut u32,
        ) -> i32 {
            crate::param_ffi::common::ffi_get(motor, out_value, |m| {
                $vendor_mod::get_u32(m, param_id, timeout_ms)
            })
        }

        #[unsafe(no_mangle)]
        pub extern "C" fn $fn_f32(
            motor: *mut crate::MotorHandle,
            param_id: u16,
            timeout_ms: u32,
            out_value: *mut f32,
        ) -> i32 {
            crate::param_ffi::common::ffi_get(motor, out_value, |m| {
                $vendor_mod::get_f32(m, param_id, timeout_ms)
            })
        }
    };
}

macro_rules! define_param_write_ffis_5 {
    (
        $vendor_mod:ident,
        $fn_i8:ident,
        $fn_u8:ident,
        $fn_u16:ident,
        $fn_u32:ident,
        $fn_f32:ident
    ) => {
        #[unsafe(no_mangle)]
        pub extern "C" fn $fn_i8(motor: *mut crate::MotorHandle, param_id: u16, value: i8) -> i32 {
            crate::param_ffi::common::ffi_run(motor, |m| $vendor_mod::write_i8(m, param_id, value))
        }

        #[unsafe(no_mangle)]
        pub extern "C" fn $fn_u8(motor: *mut crate::MotorHandle, param_id: u16, value: u8) -> i32 {
            crate::param_ffi::common::ffi_run(motor, |m| $vendor_mod::write_u8(m, param_id, value))
        }

        #[unsafe(no_mangle)]
        pub extern "C" fn $fn_u16(
            motor: *mut crate::MotorHandle,
            param_id: u16,
            value: u16,
        ) -> i32 {
            crate::param_ffi::common::ffi_run(motor, |m| $vendor_mod::write_u16(m, param_id, value))
        }

        #[unsafe(no_mangle)]
        pub extern "C" fn $fn_u32(
            motor: *mut crate::MotorHandle,
            param_id: u16,
            value: u32,
        ) -> i32 {
            crate::param_ffi::common::ffi_run(motor, |m| $vendor_mod::write_u32(m, param_id, value))
        }

        #[unsafe(no_mangle)]
        pub extern "C" fn $fn_f32(
            motor: *mut crate::MotorHandle,
            param_id: u16,
            value: f32,
        ) -> i32 {
            crate::param_ffi::common::ffi_run(motor, |m| $vendor_mod::write_f32(m, param_id, value))
        }
    };
}

mod common;
mod cyberbeast;
mod damiao;
mod hexfellow;
mod hightorque;
mod myactuator;
mod robstride;
