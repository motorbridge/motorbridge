from __future__ import annotations

import ctypes
from ctypes import c_float, c_int8, c_uint8, c_uint16, c_uint32

from .abi import CState, get_abi
from .dm_device_runtime import ensure_dm_device_runtime
from .errors import CallError
from .models import Mode, MotorState


def _err_text() -> str:
    msg = get_abi().lib.motor_last_error_message()
    return msg.decode() if msg else "unknown error"


def _ok(rc: int, what: str) -> None:
    if rc != 0:
        raise CallError(f"{what} failed: {_err_text()}")


class Controller:
    def __init__(self, channel: str = "can0") -> None:
        self._abi = get_abi()
        self._ptr = self._abi.lib.motor_controller_new_socketcan(channel.encode())
        if not self._ptr:
            raise CallError(f"new_socketcan failed: {_err_text()}")

    @classmethod
    def from_socketcanfd(cls, channel: str = "can0") -> "Controller":
        self = cls.__new__(cls)
        self._abi = get_abi()
        self._ptr = self._abi.lib.motor_controller_new_socketcanfd(channel.encode())
        if not self._ptr:
            raise CallError(f"new_socketcanfd failed: {_err_text()}")
        return self

    @classmethod
    def from_dm_serial(cls, serial_port: str = "/dev/ttyACM0", baud: int = 921600) -> "Controller":
        self = cls.__new__(cls)
        self._abi = get_abi()
        self._ptr = self._abi.lib.motor_controller_new_dm_serial(serial_port.encode(), int(baud))
        if not self._ptr:
            raise CallError(f"new_dm_serial failed: {_err_text()}")
        return self

    @classmethod
    def from_mcu_serial(cls, serial_port: str = "/dev/ttyUSB0", baud: int = 921600) -> "Controller":
        self = cls.__new__(cls)
        self._abi = get_abi()
        self._ptr = self._abi.lib.motor_controller_new_mcu_serial(serial_port.encode(), int(baud))
        if not self._ptr:
            raise CallError(f"new_mcu_serial failed: {_err_text()}")
        return self

    @classmethod
    def from_dm_device(
        cls,
        dm_device_type: str = "usb2canfd-dual",
        dm_channel: str = "0",
    ) -> "Controller":
        self = cls.__new__(cls)
        ensure_dm_device_runtime(quiet=True)
        self._abi = get_abi()
        self._ptr = self._abi.lib.motor_controller_new_dm_device(
            dm_device_type.encode(),
            dm_channel.encode(),
        )
        if not self._ptr:
            raise CallError(f"new_dm_device failed: {_err_text()}")
        return self

    def close(self) -> None:
        if self._ptr:
            self._abi.lib.motor_controller_free(self._ptr)
            self._ptr = None

    def _require_open(self) -> int:
        if not self._ptr:
            raise CallError("controller is closed")
        return self._ptr

    def shutdown(self) -> None:
        _ok(self._abi.lib.motor_controller_shutdown(self._require_open()), "controller_shutdown")

    def close_bus(self) -> None:
        _ok(self._abi.lib.motor_controller_close_bus(self._require_open()), "controller_close_bus")

    def enable_all(self) -> None:
        _ok(self._abi.lib.motor_controller_enable_all(self._require_open()), "enable_all")

    def disable_all(self) -> None:
        _ok(self._abi.lib.motor_controller_disable_all(self._require_open()), "disable_all")

    def poll_feedback_once(self) -> None:
        _ok(
            self._abi.lib.motor_controller_poll_feedback_once(self._require_open()),
            "poll_feedback_once",
        )

    def add_damiao_motor(self, motor_id: int, feedback_id: int, model: str) -> "Motor":
        m = self._abi.lib.motor_controller_add_damiao_motor(
            self._require_open(), motor_id, feedback_id, model.encode()
        )
        if not m:
            raise CallError(f"add_damiao_motor failed: {_err_text()}")
        return Motor(m)

    def add_hexfellow_motor(self, motor_id: int, feedback_id: int, model: str) -> "Motor":
        m = self._abi.lib.motor_controller_add_hexfellow_motor(
            self._require_open(), motor_id, feedback_id, model.encode()
        )
        if not m:
            raise CallError(f"add_hexfellow_motor failed: {_err_text()}")
        return Motor(m)

    def add_myactuator_motor(self, motor_id: int, feedback_id: int, model: str) -> "Motor":
        m = self._abi.lib.motor_controller_add_myactuator_motor(
            self._require_open(), motor_id, feedback_id, model.encode()
        )
        if not m:
            raise CallError(f"add_myactuator_motor failed: {_err_text()}")
        return Motor(m)

    def add_robstride_motor(self, motor_id: int, feedback_id: int, model: str) -> "Motor":
        if not 1 <= int(motor_id) <= 255:
            raise ValueError(f"RobStride motor_id must be in 1..255, got {motor_id}")
        if not 0 <= int(feedback_id) <= 255:
            raise ValueError(f"RobStride feedback_id/host_id must be in 0..255, got {feedback_id}")
        m = self._abi.lib.motor_controller_add_robstride_motor(
            self._require_open(), motor_id, feedback_id, model.encode()
        )
        if not m:
            raise CallError(f"add_robstride_motor failed: {_err_text()}")
        return Motor(m)

    def add_hightorque_motor(self, motor_id: int, feedback_id: int, model: str) -> "Motor":
        m = self._abi.lib.motor_controller_add_hightorque_motor(
            self._require_open(), motor_id, feedback_id, model.encode()
        )
        if not m:
            raise CallError(f"add_hightorque_motor failed: {_err_text()}")
        return Motor(m)

    def __enter__(self) -> "Controller":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        try:
            self.shutdown()
        finally:
            self.close()


class Motor:
    def __init__(self, ptr: int) -> None:
        self._abi = get_abi()
        self._ptr = ptr

    def close(self) -> None:
        if self._ptr:
            self._abi.lib.motor_handle_free(self._require_open())
            self._ptr = None

    def _require_open(self) -> int:
        if not self._ptr:
            raise CallError("motor handle is closed")
        return self._ptr

    def enable(self) -> None:
        _ok(self._abi.lib.motor_handle_enable(self._require_open()), "enable")

    def disable(self) -> None:
        _ok(self._abi.lib.motor_handle_disable(self._require_open()), "disable")

    def clear_error(self) -> None:
        _ok(self._abi.lib.motor_handle_clear_error(self._require_open()), "clear_error")

    def set_zero_position(self) -> None:
        _ok(self._abi.lib.motor_handle_set_zero_position(self._require_open()), "set_zero_position")

    def ensure_mode(self, mode: Mode | int, timeout_ms: int = 1000) -> None:
        _ok(self._abi.lib.motor_handle_ensure_mode(self._require_open(), int(mode), timeout_ms), "ensure_mode")

    def send_mit(self, pos: float, vel: float, kp: float, kd: float, tau: float) -> None:
        _ok(self._abi.lib.motor_handle_send_mit(self._require_open(), pos, vel, kp, kd, tau), "send_mit")

    def send_pos_vel(self, pos: float, vlim: float) -> None:
        _ok(self._abi.lib.motor_handle_send_pos_vel(self._require_open(), pos, vlim), "send_pos_vel")

    def robstride_send_pos_vel_pp(self, pos: float, vel_max: float, acc_set: float) -> None:
        _ok(
            self._abi.lib.motor_handle_robstride_send_pos_vel_pp(self._require_open(), pos, vel_max, acc_set),
            "robstride_send_pos_vel_pp",
        )

    def robstride_send_pos_vel_csp(self, pos: float, vlim: float) -> None:
        _ok(
            self._abi.lib.motor_handle_robstride_send_pos_vel_csp(self._require_open(), pos, vlim),
            "robstride_send_pos_vel_csp",
        )

    def send_vel(self, vel: float) -> None:
        _ok(self._abi.lib.motor_handle_send_vel(self._require_open(), vel), "send_vel")

    def send_force_pos(self, pos: float, vlim: float, ratio: float) -> None:
        _ok(
            self._abi.lib.motor_handle_send_force_pos(self._require_open(), pos, vlim, ratio),
            "send_force_pos",
        )

    def request_feedback(self) -> None:
        _ok(self._abi.lib.motor_handle_request_feedback(self._require_open()), "request_feedback")

    def set_can_timeout_ms(self, timeout_ms: int) -> None:
        _ok(self._abi.lib.motor_handle_set_can_timeout_ms(self._require_open(), timeout_ms), "set_can_timeout_ms")

    def store_parameters(self) -> None:
        _ok(self._abi.lib.motor_handle_store_parameters(self._require_open()), "store_parameters")

    def write_register_f32(self, rid: int, value: float) -> None:
        _ok(self._abi.lib.motor_handle_write_register_f32(self._require_open(), rid, value), "write_register_f32")

    def write_register_u32(self, rid: int, value: int) -> None:
        _ok(self._abi.lib.motor_handle_write_register_u32(self._require_open(), rid, value), "write_register_u32")

    def get_register_f32(self, rid: int, timeout_ms: int = 1000) -> float:
        out = c_float(0.0)
        _ok(
            self._abi.lib.motor_handle_get_register_f32(
                self._require_open(), rid, timeout_ms, ctypes.byref(out)
            ),
            "get_register_f32",
        )
        return float(out.value)

    def get_register_u32(self, rid: int, timeout_ms: int = 1000) -> int:
        out = c_uint32(0)
        _ok(
            self._abi.lib.motor_handle_get_register_u32(
                self._require_open(), rid, timeout_ms, ctypes.byref(out)
            ),
            "get_register_u32",
        )
        return int(out.value)

    def robstride_ping(self) -> tuple[int, int]:
        device_id = c_uint8(0)
        responder_id = c_uint8(0)
        _ok(
            self._abi.lib.motor_handle_robstride_ping(
                self._require_open(), ctypes.byref(device_id), ctypes.byref(responder_id)
            ),
            "robstride_ping",
        )
        return int(device_id.value), int(responder_id.value)

    def robstride_ping_host_id(self, host_id: int, timeout_ms: int = 500) -> tuple[int, int]:
        if not 0 <= int(host_id) <= 255:
            raise ValueError(f"RobStride host_id must be in 0..255, got {host_id}")
        device_id = c_uint8(0)
        responder_id = c_uint8(0)
        _ok(
            self._abi.lib.motor_handle_robstride_ping_host_id(
                self._require_open(), host_id, timeout_ms, ctypes.byref(device_id), ctypes.byref(responder_id)
            ),
            "robstride_ping_host_id",
        )
        return int(device_id.value), int(responder_id.value)

    def robstride_get_param_f32_host_id(self, param_id: int, host_id: int, timeout_ms: int = 1000) -> float:
        if not 0 <= int(host_id) <= 255:
            raise ValueError(f"RobStride host_id must be in 0..255, got {host_id}")
        out = c_float(0.0)
        _ok(
            self._abi.lib.motor_handle_robstride_get_param_f32_host_id(
                self._require_open(), param_id, host_id, timeout_ms, ctypes.byref(out)
            ),
            "robstride_get_param_f32_host_id",
        )
        return float(out.value)

    def robstride_get_fault_report(self) -> tuple[int, int]:
        fault_raw = c_uint32(0)
        warning_raw = c_uint32(0)
        _ok(
            self._abi.lib.motor_handle_robstride_get_fault_report(
                self._require_open(), ctypes.byref(fault_raw), ctypes.byref(warning_raw)
            ),
            "robstride_get_fault_report",
        )
        return int(fault_raw.value), int(warning_raw.value)

    def robstride_set_device_id(self, new_device_id: int) -> None:
        if not 1 <= int(new_device_id) <= 255:
            raise ValueError(f"RobStride new_device_id must be in 1..255, got {new_device_id}")
        _ok(self._abi.lib.motor_handle_robstride_set_device_id(self._require_open(), new_device_id), "robstride_set_device_id")

    def robstride_set_active_report(self, enabled: bool) -> None:
        _ok(
            self._abi.lib.motor_handle_robstride_set_active_report(
                self._require_open(), 1 if enabled else 0
            ),
            "robstride_set_active_report",
        )

    def robstride_write_param_i8(self, param_id: int, value: int) -> None:
        _ok(self._abi.lib.motor_handle_robstride_write_param_i8(self._require_open(), param_id, value), "robstride_write_param_i8")

    def robstride_write_param_u8(self, param_id: int, value: int) -> None:
        _ok(self._abi.lib.motor_handle_robstride_write_param_u8(self._require_open(), param_id, value), "robstride_write_param_u8")

    def robstride_write_param_u16(self, param_id: int, value: int) -> None:
        _ok(self._abi.lib.motor_handle_robstride_write_param_u16(self._require_open(), param_id, value), "robstride_write_param_u16")

    def robstride_write_param_u32(self, param_id: int, value: int) -> None:
        _ok(self._abi.lib.motor_handle_robstride_write_param_u32(self._require_open(), param_id, value), "robstride_write_param_u32")

    def robstride_write_param_f32(self, param_id: int, value: float) -> None:
        _ok(self._abi.lib.motor_handle_robstride_write_param_f32(self._require_open(), param_id, value), "robstride_write_param_f32")

    def robstride_get_param_i8(self, param_id: int, timeout_ms: int = 1000) -> int:
        out = c_int8(0)
        _ok(
            self._abi.lib.motor_handle_robstride_get_param_i8(
                self._require_open(), param_id, timeout_ms, ctypes.byref(out)
            ),
            "robstride_get_param_i8",
        )
        return int(out.value)

    def robstride_get_param_u8(self, param_id: int, timeout_ms: int = 1000) -> int:
        out = c_uint8(0)
        _ok(
            self._abi.lib.motor_handle_robstride_get_param_u8(
                self._require_open(), param_id, timeout_ms, ctypes.byref(out)
            ),
            "robstride_get_param_u8",
        )
        return int(out.value)

    def robstride_get_param_u16(self, param_id: int, timeout_ms: int = 1000) -> int:
        out = c_uint16(0)
        _ok(
            self._abi.lib.motor_handle_robstride_get_param_u16(
                self._require_open(), param_id, timeout_ms, ctypes.byref(out)
            ),
            "robstride_get_param_u16",
        )
        return int(out.value)

    def robstride_get_param_u32(self, param_id: int, timeout_ms: int = 1000) -> int:
        out = c_uint32(0)
        _ok(
            self._abi.lib.motor_handle_robstride_get_param_u32(
                self._require_open(), param_id, timeout_ms, ctypes.byref(out)
            ),
            "robstride_get_param_u32",
        )
        return int(out.value)

    def robstride_get_param_f32(self, param_id: int, timeout_ms: int = 1000) -> float:
        out = c_float(0.0)
        _ok(
            self._abi.lib.motor_handle_robstride_get_param_f32(
                self._require_open(), param_id, timeout_ms, ctypes.byref(out)
            ),
            "robstride_get_param_f32",
        )
        return float(out.value)

    def damiao_get_param_f32(self, param_id: int, timeout_ms: int = 1000) -> float:
        out = c_float(0.0)
        _ok(
            self._abi.lib.motor_handle_damiao_get_param_f32(
                self._require_open(), param_id, timeout_ms, ctypes.byref(out)
            ),
            "damiao_get_param_f32",
        )
        return float(out.value)

    def damiao_get_param_u32(self, param_id: int, timeout_ms: int = 1000) -> int:
        out = c_uint32(0)
        _ok(
            self._abi.lib.motor_handle_damiao_get_param_u32(
                self._require_open(), param_id, timeout_ms, ctypes.byref(out)
            ),
            "damiao_get_param_u32",
        )
        return int(out.value)

    def damiao_write_param_f32(self, param_id: int, value: float) -> None:
        _ok(self._abi.lib.motor_handle_damiao_write_param_f32(self._require_open(), param_id, value), "damiao_write_param_f32")

    def damiao_write_param_u32(self, param_id: int, value: int) -> None:
        _ok(self._abi.lib.motor_handle_damiao_write_param_u32(self._require_open(), param_id, value), "damiao_write_param_u32")

    def get_state(self) -> MotorState | None:
        st = CState()
        _ok(self._abi.lib.motor_handle_get_state(self._require_open(), ctypes.byref(st)), "get_state")
        if not st.has_value:
            return None
        return MotorState(
            can_id=int(st.can_id),
            arbitration_id=int(st.arbitration_id),
            status_code=int(st.status_code),
            pos=float(st.pos),
            vel=float(st.vel),
            torq=float(st.torq),
            t_mos=float(st.t_mos),
            t_rotor=float(st.t_rotor),
        )
