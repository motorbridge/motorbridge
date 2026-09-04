import ctypes
import ctypes.util
import json
import os
import sys
from ctypes import POINTER, Structure, c_char_p, c_float, c_int8, c_int32, c_uint8, c_uint16, c_uint32, c_void_p
from pathlib import Path

from .errors import AbiLoadError


class CState(Structure):
    _fields_ = [
        ("has_value", c_int32),
        ("can_id", c_uint8),
        ("arbitration_id", c_uint32),
        ("status_code", c_uint8),
        ("pos", c_float),
        ("vel", c_float),
        ("torq", c_float),
        ("t_mos", c_float),
        ("t_rotor", c_float),
    ]


def _candidate_lib_paths() -> list[Path]:
    candidates: list[Path] = []
    env = os.getenv("MOTORBRIDGE_LIB")
    if env:
        candidates.append(Path(env).expanduser())

    here = Path(__file__).resolve()
    pkg_lib = here.parent / "lib"
    if pkg_lib.exists():
        candidates.extend(
            [
                pkg_lib / "libmotor_abi.so",
                pkg_lib / "libmotor_abi.dylib",
                pkg_lib / "motor_abi.dll",
            ]
        )

    repo_root = here.parents[4]
    candidates.extend(
        [
            repo_root / "target" / "release" / "libmotor_abi.so",
            repo_root / "target" / "release" / "libmotor_abi.dylib",
            repo_root / "target" / "release" / "motor_abi.dll",
        ]
    )

    cwd = Path.cwd()
    candidates.extend(
        [
            cwd / "target" / "release" / "libmotor_abi.so",
            cwd / "target" / "release" / "libmotor_abi.dylib",
            cwd / "target" / "release" / "motor_abi.dll",
        ]
    )
    return candidates


def _platform_dm_device_lib_name() -> str:
    if sys.platform.startswith("win"):
        return "dm_device.dll"
    if sys.platform == "darwin":
        return "libdm_device.dylib"
    return "libdm_device.so"


def _configure_packaged_dm_device_runtime() -> None:
    if os.getenv("MOTOR_DM_DEVICE_LIB"):
        return

    pkg_dm_lib = Path(__file__).resolve().parent / "lib" / "dm_device" / _platform_dm_device_lib_name()
    if pkg_dm_lib.exists():
        os.environ["MOTOR_DM_DEVICE_LIB"] = str(pkg_dm_lib)


def _load_library() -> ctypes.CDLL:
    _configure_packaged_dm_device_runtime()
    tried: list[str] = []
    for p in _candidate_lib_paths():
        tried.append(str(p))
        if p.exists():
            return ctypes.CDLL(str(p))

    found = ctypes.util.find_library("motor_abi")
    if found:
        return ctypes.CDLL(found)

    raise AbiLoadError(
        "Failed to load motor_abi shared library. Tried:\n"
        + "\n".join(f"- {x}" for x in tried)
        + "\nHint: build ABI first: cargo build -p motor_abi --release"
    )


class Abi:
    def __init__(self) -> None:
        self.lib = _load_library()
        self._bind()

    def _bind(self) -> None:
        lib = self.lib

        lib.motor_last_error_message.restype = c_char_p
        lib.motor_abi_version.restype = c_char_p
        lib.motor_abi_capabilities_json.restype = c_char_p

        lib.motor_controller_new_socketcan.argtypes = [c_char_p]
        lib.motor_controller_new_socketcan.restype = c_void_p
        lib.motor_controller_new_socketcanfd.argtypes = [c_char_p]
        lib.motor_controller_new_socketcanfd.restype = c_void_p
        lib.motor_controller_new_dm_serial.argtypes = [c_char_p, c_uint32]
        lib.motor_controller_new_dm_serial.restype = c_void_p
        lib.motor_controller_new_dm_device.argtypes = [c_char_p, c_char_p]
        lib.motor_controller_new_dm_device.restype = c_void_p
        lib.motor_controller_new_mcu_serial.argtypes = [c_char_p, c_uint32]
        lib.motor_controller_new_mcu_serial.restype = c_void_p
        lib.motor_controller_free.argtypes = [c_void_p]
        lib.motor_controller_poll_feedback_once.argtypes = [c_void_p]
        lib.motor_controller_poll_feedback_once.restype = c_int32
        lib.motor_controller_enable_all.argtypes = [c_void_p]
        lib.motor_controller_enable_all.restype = c_int32
        lib.motor_controller_disable_all.argtypes = [c_void_p]
        lib.motor_controller_disable_all.restype = c_int32
        lib.motor_controller_shutdown.argtypes = [c_void_p]
        lib.motor_controller_shutdown.restype = c_int32
        lib.motor_controller_close_bus.argtypes = [c_void_p]
        lib.motor_controller_close_bus.restype = c_int32

        lib.motor_controller_add_damiao_motor.argtypes = [c_void_p, c_uint16, c_uint16, c_char_p]
        lib.motor_controller_add_damiao_motor.restype = c_void_p
        lib.motor_controller_add_hexfellow_motor.argtypes = [c_void_p, c_uint16, c_uint16, c_char_p]
        lib.motor_controller_add_hexfellow_motor.restype = c_void_p
        lib.motor_controller_add_myactuator_motor.argtypes = [c_void_p, c_uint16, c_uint16, c_char_p]
        lib.motor_controller_add_myactuator_motor.restype = c_void_p
        lib.motor_controller_add_robstride_motor.argtypes = [c_void_p, c_uint16, c_uint16, c_char_p]
        lib.motor_controller_add_robstride_motor.restype = c_void_p
        lib.motor_controller_add_hightorque_motor.argtypes = [c_void_p, c_uint16, c_uint16, c_char_p]
        lib.motor_controller_add_hightorque_motor.restype = c_void_p

        lib.motor_handle_free.argtypes = [c_void_p]
        lib.motor_handle_enable.argtypes = [c_void_p]
        lib.motor_handle_enable.restype = c_int32
        lib.motor_handle_disable.argtypes = [c_void_p]
        lib.motor_handle_disable.restype = c_int32
        lib.motor_handle_clear_error.argtypes = [c_void_p]
        lib.motor_handle_clear_error.restype = c_int32
        lib.motor_handle_set_zero_position.argtypes = [c_void_p]
        lib.motor_handle_set_zero_position.restype = c_int32
        lib.motor_handle_ensure_mode.argtypes = [c_void_p, c_uint32, c_uint32]
        lib.motor_handle_ensure_mode.restype = c_int32

        lib.motor_handle_send_mit.argtypes = [c_void_p, c_float, c_float, c_float, c_float, c_float]
        lib.motor_handle_send_mit.restype = c_int32
        lib.motor_handle_send_pos_vel.argtypes = [c_void_p, c_float, c_float]
        lib.motor_handle_send_pos_vel.restype = c_int32
        lib.motor_handle_robstride_send_pos_vel_pp.argtypes = [c_void_p, c_float, c_float, c_float]
        lib.motor_handle_robstride_send_pos_vel_pp.restype = c_int32
        lib.motor_handle_robstride_send_pos_vel_csp.argtypes = [c_void_p, c_float, c_float]
        lib.motor_handle_robstride_send_pos_vel_csp.restype = c_int32
        lib.motor_handle_send_vel.argtypes = [c_void_p, c_float]
        lib.motor_handle_send_vel.restype = c_int32
        lib.motor_handle_send_force_pos.argtypes = [c_void_p, c_float, c_float, c_float]
        lib.motor_handle_send_force_pos.restype = c_int32

        lib.motor_handle_store_parameters.argtypes = [c_void_p]
        lib.motor_handle_store_parameters.restype = c_int32
        lib.motor_handle_request_feedback.argtypes = [c_void_p]
        lib.motor_handle_request_feedback.restype = c_int32
        lib.motor_handle_set_can_timeout_ms.argtypes = [c_void_p, c_uint32]
        lib.motor_handle_set_can_timeout_ms.restype = c_int32

        lib.motor_handle_write_register_f32.argtypes = [c_void_p, c_uint8, c_float]
        lib.motor_handle_write_register_f32.restype = c_int32
        lib.motor_handle_write_register_u32.argtypes = [c_void_p, c_uint8, c_uint32]
        lib.motor_handle_write_register_u32.restype = c_int32
        lib.motor_handle_get_register_f32.argtypes = [c_void_p, c_uint8, c_uint32, POINTER(c_float)]
        lib.motor_handle_get_register_f32.restype = c_int32
        lib.motor_handle_get_register_u32.argtypes = [c_void_p, c_uint8, c_uint32, POINTER(c_uint32)]
        lib.motor_handle_get_register_u32.restype = c_int32

        lib.motor_handle_robstride_ping.argtypes = [c_void_p, POINTER(c_uint8), POINTER(c_uint8)]
        lib.motor_handle_robstride_ping.restype = c_int32
        lib.motor_handle_robstride_ping_host_id.argtypes = [
            c_void_p, c_uint16, c_uint32, POINTER(c_uint8), POINTER(c_uint8)
        ]
        lib.motor_handle_robstride_ping_host_id.restype = c_int32
        lib.motor_handle_robstride_get_param_f32_host_id.argtypes = [
            c_void_p, c_uint16, c_uint16, c_uint32, POINTER(c_float)
        ]
        lib.motor_handle_robstride_get_param_f32_host_id.restype = c_int32
        lib.motor_handle_robstride_get_fault_report.argtypes = [c_void_p, POINTER(c_uint32), POINTER(c_uint32)]
        lib.motor_handle_robstride_get_fault_report.restype = c_int32
        lib.motor_handle_robstride_set_device_id.argtypes = [c_void_p, c_uint8]
        lib.motor_handle_robstride_set_device_id.restype = c_int32
        lib.motor_handle_robstride_set_active_report.argtypes = [c_void_p, c_uint8]
        lib.motor_handle_robstride_set_active_report.restype = c_int32
        lib.motor_handle_robstride_write_param_i8.argtypes = [c_void_p, c_uint16, c_int8]
        lib.motor_handle_robstride_write_param_i8.restype = c_int32
        lib.motor_handle_robstride_write_param_u8.argtypes = [c_void_p, c_uint16, c_uint8]
        lib.motor_handle_robstride_write_param_u8.restype = c_int32
        lib.motor_handle_robstride_write_param_u16.argtypes = [c_void_p, c_uint16, c_uint16]
        lib.motor_handle_robstride_write_param_u16.restype = c_int32
        lib.motor_handle_robstride_write_param_u32.argtypes = [c_void_p, c_uint16, c_uint32]
        lib.motor_handle_robstride_write_param_u32.restype = c_int32
        lib.motor_handle_robstride_write_param_f32.argtypes = [c_void_p, c_uint16, c_float]
        lib.motor_handle_robstride_write_param_f32.restype = c_int32
        lib.motor_handle_robstride_get_param_i8.argtypes = [c_void_p, c_uint16, c_uint32, POINTER(c_int8)]
        lib.motor_handle_robstride_get_param_i8.restype = c_int32
        lib.motor_handle_robstride_get_param_u8.argtypes = [c_void_p, c_uint16, c_uint32, POINTER(c_uint8)]
        lib.motor_handle_robstride_get_param_u8.restype = c_int32
        lib.motor_handle_robstride_get_param_u16.argtypes = [c_void_p, c_uint16, c_uint32, POINTER(c_uint16)]
        lib.motor_handle_robstride_get_param_u16.restype = c_int32
        lib.motor_handle_robstride_get_param_u32.argtypes = [c_void_p, c_uint16, c_uint32, POINTER(c_uint32)]
        lib.motor_handle_robstride_get_param_u32.restype = c_int32
        lib.motor_handle_robstride_get_param_f32.argtypes = [c_void_p, c_uint16, c_uint32, POINTER(c_float)]
        lib.motor_handle_robstride_get_param_f32.restype = c_int32

        lib.motor_handle_get_state.argtypes = [c_void_p, POINTER(CState)]
        lib.motor_handle_get_state.restype = c_int32

        lib.motor_handle_damiao_get_param_f32.argtypes = [c_void_p, c_uint16, c_uint32, POINTER(c_float)]
        lib.motor_handle_damiao_get_param_f32.restype = c_int32
        lib.motor_handle_damiao_get_param_u32.argtypes = [c_void_p, c_uint16, c_uint32, POINTER(c_uint32)]
        lib.motor_handle_damiao_get_param_u32.restype = c_int32
        lib.motor_handle_damiao_write_param_f32.argtypes = [c_void_p, c_uint16, c_float]
        lib.motor_handle_damiao_write_param_f32.restype = c_int32
        lib.motor_handle_damiao_write_param_u32.argtypes = [c_void_p, c_uint16, c_uint32]
        lib.motor_handle_damiao_write_param_u32.restype = c_int32


_abi_singleton: Abi | None = None


def get_abi() -> Abi:
    global _abi_singleton
    if _abi_singleton is None:
        _abi_singleton = Abi()
    return _abi_singleton


def abi_version() -> str:
    msg = get_abi().lib.motor_abi_version()
    return msg.decode() if msg else ""


def abi_capabilities() -> dict:
    msg = get_abi().lib.motor_abi_capabilities_json()
    return json.loads(msg.decode() if msg else "{}")
