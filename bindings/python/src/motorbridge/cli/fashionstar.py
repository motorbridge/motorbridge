from __future__ import annotations

import argparse
import time
from contextlib import contextmanager

from .common import _open_controller, _parse_id


def _parse_unsigned(text: str, name: str, maximum: int = 0xFFFFFFFF) -> int:
    value = _parse_id(text)
    if not 0 <= value <= maximum:
        raise ValueError(f"FashionStar {name} must be in 0..{maximum}")
    return value


@contextmanager
def _fashionstar_controller(args: argparse.Namespace):
    ctrl = _open_controller(args, "fashionstar")
    try:
        yield ctrl
    finally:
        try:
            ctrl.close_bus()
        finally:
            ctrl.close()


def _run_fashionstar(args: argparse.Namespace) -> None:
    if args.mode not in ("enable", "disable", "pos-vel", "zero", "set-zero", "ping", "read-param", "write-param"):
        raise ValueError(f"FashionStar mode {args.mode} is not currently supported")
    motor_id = _parse_unsigned(args.motor_id, "motor_id", 254)
    feedback_id = _parse_unsigned(args.feedback_id, "feedback_id", 0xFFFF)
    if feedback_id != 0x321:
        raise ValueError("FashionStar CAN arbitration ID is fixed at 0x321")
    if args.mode in ("ping", "read-param"):
        timeout_ms = _parse_unsigned(str(args.timeout_ms), "timeout_ms")
    if args.mode in ("read-param", "write-param"):
        if args.param_type not in ("", "u16", "u32"):
            raise ValueError("FashionStar parameters use integer raw values (u16/u32)")
        rid = _parse_unsigned(args.param_id, "register address", 255)
        if args.mode == "write-param":
            if args.param_value == "":
                raise ValueError("run --mode write-param requires --param-value")
            value = _parse_unsigned(args.param_value, "register value")
    with _fashionstar_controller(args) as ctrl:
        motor = ctrl.add_fashionstar_motor(motor_id, feedback_id, args.model)
        try:
            if args.mode == "read-param":
                value = motor.get_register_u32(rid, timeout_ms)
                print(
                    f"command=run mode=read-param vendor=fashionstar param_id=0x{rid:X} "
                    f"type=u32 value={value}"
                )
                return
            if args.mode == "write-param":
                motor.write_register_u32(rid, value)
                print(
                    f"command=run mode=write-param vendor=fashionstar param_id=0x{rid:X} "
                    f"type=u32 value={value}; configuration changes take effect after a power cycle"
                )
                return
            for i in range(args.loop):
                if args.mode == "enable":
                    motor.enable()
                elif args.mode == "disable":
                    motor.disable()
                elif args.mode == "pos-vel":
                    motor.send_pos_vel(args.pos, args.vlim)
                elif args.mode in ("zero", "set-zero"):
                    motor.set_zero_position()
                elif args.mode == "ping":
                    print(f"model_code={motor.get_register_u32(1, timeout_ms)}")
                if args.print_state:
                    motor.request_feedback()
                    st = motor.get_state()
                    if st is None:
                        print(f"#{i} no feedback yet")
                    else:
                        print(f"#{i} pos={st.pos:+.3f} vel={st.vel:+.3f} current={st.torq:.3f}A temp={st.t_mos:.1f}C status={st.status_code}")
                if args.mode != "pos-vel":
                    break
                time.sleep(max(args.dt_ms, 0) / 1000.0)
        finally:
            motor.close()
