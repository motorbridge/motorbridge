from __future__ import annotations

import argparse
import time
from typing import Any

from ..core import Controller
from ..models import Mode
from .common import _open_controller, _parse_id, _vendor_defaults

def _infer_robstride_param_type(param_id: int, explicit_type: str = "") -> str:
    if explicit_type:
        return explicit_type
    # Match the Rust registers.rs RobStride parameter typing (section-4
    # non-float params; everything else in the table is Float32).
    non_f32_types = {
        0x7005: "i8",   # run_mode
        0x7026: "u16",  # EPScan_time
        0x7028: "u32",  # canTimeout
        0x7029: "u8",   # zero_sta
        0x702A: "u8",   # damper
        0x702C: "u8",   # alveolous_open
        0x702D: "u8",   # iq_test
    }
    return non_f32_types.get(param_id, "f32")

def _read_robstride_param(motor: Any, param_id: int, param_type: str, timeout_ms: int) -> int | float:
    if param_type == "i8":
        return motor.robstride_get_param_i8(param_id, timeout_ms)
    if param_type == "u8":
        return motor.robstride_get_param_u8(param_id, timeout_ms)
    if param_type == "u16":
        return motor.robstride_get_param_u16(param_id, timeout_ms)
    if param_type == "u32":
        return motor.robstride_get_param_u32(param_id, timeout_ms)
    return motor.robstride_get_param_f32(param_id, timeout_ms)

def _write_robstride_param(motor: Any, param_id: int, param_type: str, raw_value: str) -> None:
    if param_type == "i8":
        motor.robstride_write_param_i8(param_id, int(raw_value, 0))
    elif param_type == "u8":
        motor.robstride_write_param_u8(param_id, int(raw_value, 0))
    elif param_type == "u16":
        motor.robstride_write_param_u16(param_id, int(raw_value, 0))
    elif param_type == "u32":
        motor.robstride_write_param_u32(param_id, int(raw_value, 0))
    else:
        motor.robstride_write_param_f32(param_id, float(raw_value))

def _robstride_mode_expect(mode: str) -> tuple[Mode, int, str]:
    if mode == "mit":
        return Mode.MIT, 0, "mit"
    if mode == "pos-vel":
        return Mode.POS_VEL, 1, "pos-vel"
    if mode == "vel":
        return Mode.VEL, 2, "vel"
    raise ValueError(f"RobStride mode guard does not support {mode}")

def _ensure_robstride_mode_for_control(
    ctrl: Controller,
    motor: Any,
    mode: str,
    timeout_ms: int,
    strict: bool,
) -> None:
    target_mode, expect, mode_name = _robstride_mode_expect(mode)
    try:
        actual = motor.robstride_get_param_i8(0x7005, 120)
        if actual == expect:
            return
    except Exception:
        actual = None

    # Match Rust CLI / WS gateway sequencing. Some RobStride firmware variants
    # ignore run_mode writes while torque is enabled.
    try:
        ctrl.disable_all()
    except Exception as e:
        if strict:
            raise
        print(f"[warn] robstride pre-mode disable_all failed: {e}; continue")
    time.sleep(0.06)

    last_error: Exception | None = None
    for _ in range(3):
        try:
            motor.ensure_mode(target_mode, timeout_ms)
            time.sleep(0.03)
            actual = motor.robstride_get_param_i8(0x7005, 120)
            if actual == expect:
                return
        except Exception as e:
            last_error = e
        time.sleep(0.03)

    msg = (
        f"robstride {mode_name} mode switch failed: expect={expect} "
        f"actual={actual!r} error={last_error}"
    )
    if strict:
        raise RuntimeError(msg)
    print(f"[warn] {msg}; continue anyway")

def _robstride_device_id(value: int, name: str) -> int:
    if not 1 <= value <= 255:
        raise ValueError(f"RobStride {name} must be in 1..255, got {value}")
    return value

def _robstride_host_id(value: int, name: str) -> int:
    if not 0 <= value <= 255:
        raise ValueError(f"RobStride {name}/host_id must be in 0..255, got {value}")
    return value

def _robstride_read_param_command(args: argparse.Namespace) -> None:
    if args.vendor != "robstride":
        raise ValueError("robstride-read-param is only valid for --vendor robstride")
    args.model, args.feedback_id = _vendor_defaults(args.vendor, args.model, args.feedback_id)
    motor_id = _parse_id(args.motor_id)
    feedback_id = _parse_id(args.feedback_id)
    param_id = _parse_id(args.param_id)
    with _open_controller(args, args.vendor) as ctrl:
        motor = ctrl.add_robstride_motor(motor_id, feedback_id, args.model)
        try:
            value = _read_robstride_param(motor, param_id, args.type, args.timeout_ms)
            print(
                f"command=robstride-read-param channel={args.channel} model={args.model} "
                f"motor_id=0x{motor_id:X} param_id=0x{param_id:X} type={args.type} value={value}"
            )
        finally:
            motor.close()

def _robstride_write_param_command(args: argparse.Namespace) -> None:
    if args.vendor != "robstride":
        raise ValueError("robstride-write-param is only valid for --vendor robstride")
    args.model, args.feedback_id = _vendor_defaults(args.vendor, args.model, args.feedback_id)
    motor_id = _parse_id(args.motor_id)
    feedback_id = _parse_id(args.feedback_id)
    param_id = _parse_id(args.param_id)
    with _open_controller(args, args.vendor) as ctrl:
        motor = ctrl.add_robstride_motor(motor_id, feedback_id, args.model)
        try:
            _write_robstride_param(motor, param_id, args.type, args.value)
            verify = _read_robstride_param(motor, param_id, args.type, args.timeout_ms) if args.verify else None
            if args.store:
                motor.store_parameters()
            print(
                f"command=robstride-write-param channel={args.channel} model={args.model} "
                f"motor_id=0x{motor_id:X} param_id=0x{param_id:X} type={args.type} "
                f"value={args.value} verify={verify} store={int(bool(args.store))}"
            )
        finally:
            motor.close()
