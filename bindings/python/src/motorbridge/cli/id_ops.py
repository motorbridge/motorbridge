from __future__ import annotations

import argparse
import time

from .common import _open_controller, _parse_id, _vendor_defaults
from .robstride import _robstride_device_id, _robstride_host_id

def _id_set_command(args: argparse.Namespace) -> None:
    if args.vendor not in ("damiao", "robstride"):
        raise ValueError("id-set currently supports Damiao and RobStride only")
    args.model, args.feedback_id = _vendor_defaults(args.vendor, args.model, args.feedback_id)
    motor_id = _parse_id(args.motor_id)
    feedback_id = _parse_id(args.feedback_id)
    new_motor_id = _parse_id(args.new_motor_id) if args.new_motor_id else motor_id
    new_feedback_id = _parse_id(args.new_feedback_id) if args.new_feedback_id else feedback_id

    if args.vendor == "robstride":
        _robstride_device_id(motor_id, "motor_id")
        _robstride_device_id(new_motor_id, "new_motor_id")
        _robstride_host_id(feedback_id, "feedback_id")
        if args.new_feedback_id and new_feedback_id != feedback_id:
            raise ValueError(
                "RobStride id-set changes device_id only; feedback_id/host_id is not motor_id"
            )
        print(
            f"command=id-set vendor=robstride transport={args.transport} channel={args.channel} model={args.model} "
            f"old_motor_id=0x{motor_id:X} feedback_id/host_id=0x{feedback_id:X} new_motor_id=0x{new_motor_id:X}"
        )
        print("[info] RobStride feedback_id/host_id is the host-side ID, not the motor/device ID")
        ctrl = _open_controller(args, args.vendor)
        motor = ctrl.add_robstride_motor(motor_id, feedback_id, args.model)
        try:
            motor.robstride_set_device_id(new_motor_id)
            print(f"robstride_set_device_id requested: 0x{motor_id:X} -> 0x{new_motor_id:X}")
            if args.store:
                motor.store_parameters()
                print("save_parameters sent")
        finally:
            motor.close()
            ctrl.close_bus()
            ctrl.close()

        if not args.verify:
            return

        time.sleep(0.12)
        verify_ctrl = _open_controller(args, args.vendor)
        verify_motor = verify_ctrl.add_robstride_motor(new_motor_id, feedback_id, args.model)
        try:
            device_id, responder_id = verify_motor.robstride_ping()
            print(f"verify ping ok: device_id=0x{device_id:X} responder_id=0x{responder_id:X}")
            if device_id != new_motor_id:
                raise RuntimeError(
                    f"verify failed: expected device_id=0x{new_motor_id:X}, got 0x{device_id:X}"
                )
            print("verify ok")
        finally:
            verify_motor.close()
            verify_ctrl.close_bus()
            verify_ctrl.close()
        return

    print(
        f"command=id-set vendor=damiao transport={args.transport} channel={args.channel} model={args.model} "
        f"old_motor_id=0x{motor_id:X} old_feedback_id=0x{feedback_id:X} "
        f"new_motor_id=0x{new_motor_id:X} new_feedback_id=0x{new_feedback_id:X}"
    )

    ctrl = _open_controller(args, args.vendor)
    motor = ctrl.add_damiao_motor(motor_id, feedback_id, args.model)
    try:
        if new_feedback_id != feedback_id:
            motor.write_register_u32(7, new_feedback_id)
            print(f"write rid=7 (MST_ID) <= 0x{new_feedback_id:X}")
        if new_motor_id != motor_id:
            motor.write_register_u32(8, new_motor_id)
            print(f"write rid=8 (ESC_ID) <= 0x{new_motor_id:X}")
    finally:
        motor.close()
        ctrl.close_bus()
        ctrl.close()

    if not args.store and not args.verify:
        return

    verify_ctrl = _open_controller(args, args.vendor)
    verify_motor = verify_ctrl.add_damiao_motor(new_motor_id, new_feedback_id, args.model)
    try:
        if args.store:
            verify_motor.store_parameters()
            print("store_parameters sent (via new id)")
            time.sleep(0.12)
        if args.verify:
            esc = verify_motor.get_register_u32(8, args.timeout_ms)
            mst = verify_motor.get_register_u32(7, args.timeout_ms)
            print(f"verify rid=8 (ESC_ID): 0x{esc:X}")
            print(f"verify rid=7 (MST_ID): 0x{mst:X}")
            if esc != new_motor_id or mst != new_feedback_id:
                raise RuntimeError(
                    f"verify failed: expected ESC_ID=0x{new_motor_id:X}, MST_ID=0x{new_feedback_id:X}, "
                    f"got ESC_ID=0x{esc:X}, MST_ID=0x{mst:X}"
                )
            print("verify ok")
    finally:
        verify_motor.close()
        verify_ctrl.close_bus()
        verify_ctrl.close()
