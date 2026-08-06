#!/usr/bin/env python3
from __future__ import annotations

import argparse
import time

from motorbridge import Controller, Mode


def _parse_id(text: str) -> int:
    return int(text, 0)


def main() -> None:
    parser = argparse.ArgumentParser(description="RobStride wrapper demo for Python SDK")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--model", default="rs-00")
    parser.add_argument("--motor-id", default="127")
    parser.add_argument("--feedback-id", default="0xFD")
    parser.add_argument(
        "--mode",
        choices=[
            "ping",
            "clear-error",
            "read-param",
            "write-param",
            "active-report",
            "pos-vel",
            "mit",
            "vel",
        ],
        default="ping",
    )
    parser.add_argument("--pos", type=float, default=0.0)
    parser.add_argument("--vel", type=float, default=0.0)
    parser.add_argument("--kp", type=float, default=8.0)
    parser.add_argument("--kd", type=float, default=0.2)
    parser.add_argument("--tau", type=float, default=0.0)
    parser.add_argument("--loop", type=int, default=20)
    parser.add_argument("--dt-ms", type=int, default=50)
    parser.add_argument("--param-id", default="0x7019")
    parser.add_argument("--param-type", choices=["u8", "u16", "u32", "f32"], default="f32")
    parser.add_argument("--param-value", default="")
    parser.add_argument("--param-timeout-ms", type=int, default=1000)
    parser.add_argument("--active-report", type=int, default=1)
    parser.add_argument("--vlim", type=float, default=1.0)
    parser.add_argument("--loc-kp", type=float, default=5.0)
    args = parser.parse_args()

    motor_id = _parse_id(args.motor_id)
    feedback_id = _parse_id(args.feedback_id)
    param_id = _parse_id(args.param_id)

    with Controller(args.channel) as ctrl:
        motor = ctrl.add_robstride_motor(motor_id, feedback_id, args.model)
        try:
            if args.mode == "ping":
                device_id, responder_id = motor.robstride_ping()
                print(f"ping ok device_id={device_id} responder_id={responder_id}")
                print(motor.get_state())
                return

            if args.mode == "clear-error":
                motor.clear_error()
                print("clear-error requested")
                return

            if args.mode == "read-param":
                if args.param_type == "u8":
                    value = motor.robstride_get_param_u8(param_id, args.param_timeout_ms)
                elif args.param_type == "u16":
                    value = motor.robstride_get_param_u16(param_id, args.param_timeout_ms)
                elif args.param_type == "u32":
                    value = motor.robstride_get_param_u32(param_id, args.param_timeout_ms)
                else:
                    value = motor.robstride_get_param_f32(param_id, args.param_timeout_ms)
                print(f"param 0x{param_id:04X} = {value}")
                print(motor.get_state())
                return

            if args.mode == "write-param":
                if args.param_value == "":
                    raise ValueError("--mode write-param requires --param-value")
                if args.param_type == "u8":
                    motor.robstride_write_param_u8(param_id, int(args.param_value, 0))
                    value = motor.robstride_get_param_u8(param_id, args.param_timeout_ms)
                elif args.param_type == "u16":
                    motor.robstride_write_param_u16(param_id, int(args.param_value, 0))
                    value = motor.robstride_get_param_u16(param_id, args.param_timeout_ms)
                elif args.param_type == "u32":
                    motor.robstride_write_param_u32(param_id, int(args.param_value, 0))
                    value = motor.robstride_get_param_u32(param_id, args.param_timeout_ms)
                else:
                    motor.robstride_write_param_f32(param_id, float(args.param_value))
                    value = motor.robstride_get_param_f32(param_id, args.param_timeout_ms)
                print(f"param 0x{param_id:04X} wrote {args.param_value}; verify={value}")
                return

            if args.mode == "active-report":
                motor.robstride_set_active_report(bool(args.active_report))
                print(f"active-report {'enabled' if args.active_report else 'disabled'}")
                return

            ctrl.enable_all()
            target_mode = Mode.MIT
            if args.mode == "vel":
                target_mode = Mode.VEL
            elif args.mode == "pos-vel":
                target_mode = Mode.POS_VEL
            motor.ensure_mode(target_mode, 1000)

            for i in range(args.loop):
                if args.mode == "mit":
                    motor.send_mit(args.pos, args.vel, args.kp, args.kd, args.tau)
                elif args.mode == "pos-vel":
                    if args.vlim > 0:
                        # PP reads velocity cap from 0x7024 vel_max, not 0x7017
                        # limit_spd (the latter belongs to CSP run_mode=5 and is
                        # ignored in PP).
                        motor.robstride_write_param_f32(0x7024, abs(args.vlim))
                    if args.loc_kp >= 0:
                        motor.robstride_write_param_f32(0x701E, args.loc_kp)
                    motor.robstride_write_param_f32(0x7016, args.pos)
                else:
                    motor.send_vel(args.vel)
                print(f"#{i} {motor.get_state()}")
                if args.dt_ms > 0:
                    time.sleep(args.dt_ms / 1000.0)
        finally:
            motor.close()


if __name__ == "__main__":
    main()
