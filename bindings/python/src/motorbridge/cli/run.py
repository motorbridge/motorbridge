from __future__ import annotations

import argparse
import time

from .common import _add_motor, _mode_to_enum, _open_controller, _parse_id, _vendor_defaults
from .damiao import _verify_damiao_model
from .id_ops import _id_set_command
from .robstride import (
    _ensure_robstride_mode_for_control,
    _infer_robstride_param_type,
    _read_robstride_param,
    _write_robstride_param,
)

def _run_command(args: argparse.Namespace) -> None:
    args.model, args.feedback_id = _vendor_defaults(args.vendor, args.model, args.feedback_id)
    motor_id = _parse_id(args.motor_id)
    feedback_id = _parse_id(args.feedback_id)
    print(
        f"command=run vendor={args.vendor} transport={args.transport} channel={args.channel} "
        f"model={args.model} motor_id=0x{motor_id:X} feedback_id=0x{feedback_id:X} mode={args.mode}"
    )

    if args.set_motor_id or args.set_feedback_id:
        if args.vendor not in ("damiao", "robstride"):
            raise ValueError("run --set-motor-id/--set-feedback-id supports Damiao and RobStride only")
        if args.vendor == "robstride" and args.set_feedback_id:
            raise ValueError("RobStride --set-feedback-id is not supported; feedback_id/host_id is not motor_id")
        id_args = argparse.Namespace(**vars(args))
        id_args.command = "id-set"
        id_args.new_motor_id = args.set_motor_id
        id_args.new_feedback_id = args.set_feedback_id
        id_args.verify = args.verify_id
        id_args.timeout_ms = args.verify_timeout_ms
        _id_set_command(id_args)
        return

    with _open_controller(args, args.vendor) as ctrl:
        motor = _add_motor(ctrl, args.vendor, motor_id, feedback_id, args.model)
        try:
            if args.mode in ("read-param", "write-param"):
                if args.vendor != "robstride":
                    raise ValueError("run --mode read-param/write-param is currently supported for --vendor robstride only")
                param_id = _parse_id(args.param_id)
                param_type = _infer_robstride_param_type(param_id, args.param_type)
                if args.mode == "read-param":
                    value = _read_robstride_param(motor, param_id, param_type, args.timeout_ms)
                    print(
                        f"command=run mode=read-param vendor=robstride param_id=0x{param_id:X} "
                        f"type={param_type} value={value}"
                    )
                else:
                    if args.param_value == "":
                        raise ValueError("run --mode write-param requires --param-value")
                    _write_robstride_param(motor, param_id, param_type, args.param_value)
                    time.sleep(0.05)
                    value = _read_robstride_param(motor, param_id, param_type, args.timeout_ms)
                    if args.store:
                        motor.store_parameters()
                    print(
                        f"command=run mode=write-param vendor=robstride param_id=0x{param_id:X} "
                        f"type={param_type} value={args.param_value} verify={value} "
                        f"store={int(bool(args.store))}"
                    )
                return
            if args.mode == "save":
                if args.vendor != "robstride":
                    raise ValueError("run --mode save is currently supported for --vendor robstride only")
                motor.store_parameters()
                print("[ok] save-parameters requested")
                return
            if args.mode == "active-report":
                if args.vendor != "robstride":
                    raise ValueError("run --mode active-report is currently supported for --vendor robstride only")
                enabled = bool(args.active_report)
                motor.robstride_set_active_report(enabled)
                print(f"[ok] active report {'enabled' if enabled else 'disabled'}")
                return
            if args.mode == "zero-by-offset":
                if args.vendor != "robstride":
                    raise ValueError("run --mode zero-by-offset is currently supported for --vendor robstride only")
                print(
                    "[warn] robstride zero-by-offset is temporarily disabled due to firmware inconsistency; "
                    "no calibration CAN frames sent"
                )
                return

            if args.vendor == "damiao" and args.verify_model and args.mode not in ("enable", "disable"):
                _verify_damiao_model(motor, args.model, args.verify_timeout_ms, args.verify_tol)

            control_modes = ("mit", "pos-vel", "vel", "force-pos")
            if args.vendor == "robstride" and args.mode in ("mit", "pos-vel", "vel") and args.ensure_mode:
                _ensure_robstride_mode_for_control(
                    ctrl,
                    motor,
                    args.mode,
                    args.ensure_timeout_ms,
                    bool(args.ensure_strict),
                )
                ctrl.enable_all()
                time.sleep(0.1)
            elif args.mode not in (
                "enable",
                "disable",
                "clear-error",
                "active-report",
                "ping",
                "zero",
                "set-zero",
                "pos-vel-pp",
                "pos-vel-csp",
            ):
                ctrl.enable_all()
                time.sleep(0.3)

            if (
                args.ensure_mode
                and args.mode in control_modes
                and not (args.vendor == "robstride" and args.mode in ("mit", "pos-vel", "vel"))
            ):
                try:
                    if args.vendor == "robstride" and args.mode == "force-pos":
                        raise ValueError("robstride does not support force-pos")
                    motor.ensure_mode(_mode_to_enum(args.mode), args.ensure_timeout_ms)
                except Exception as e:
                    if args.ensure_strict:
                        raise
                    print(f"[warn] ensure_mode failed: {e}; continue anyway")

            for i in range(args.loop):
                if args.mode == "enable":
                    motor.enable()
                    if args.vendor == "damiao":
                        motor.request_feedback()
                elif args.mode == "disable":
                    motor.disable()
                    if args.vendor == "damiao":
                        motor.request_feedback()
                elif args.mode == "clear-error":
                    motor.clear_error()
                    print("[ok] clear-error requested")
                    break
                elif args.mode == "ping":
                    if args.vendor != "robstride":
                        raise ValueError("ping mode is only valid for RobStride")
                    device_id, responder_id = motor.robstride_ping()
                    print(f"#{i} ping device_id={device_id} responder_id={responder_id}")
                    break
                elif args.mode == "mit":
                    if args.vendor == "myactuator":
                        raise ValueError("myactuator does not support mit command")
                    motor.send_mit(args.pos, args.vel, args.kp, args.kd, args.tau)
                elif args.mode == "pos-vel":
                    if args.vendor == "robstride":
                        provided = getattr(args, "_provided_options", set())
                        ignored = sorted({"vel", "kd", "tau"} & set(provided))
                        if ignored:
                            print(
                                "[warn] robstride pos-vel maps to native Position mode; "
                                f"ignored args: {', '.join('--' + item for item in ignored)}"
                            )
                        speed = abs(float(args.vlim))
                        if speed > 0.0:
                            motor.robstride_write_param_f32(0x7024, speed)
                        loc_kp = args.loc_kp if args.loc_kp is not None else args.kp
                        if loc_kp is not None and loc_kp >= 0.0:
                            motor.robstride_write_param_f32(0x701E, float(loc_kp))
                        motor.robstride_write_param_f32(0x7016, float(args.pos))
                    else:
                        motor.send_pos_vel(args.pos, args.vlim)
                elif args.mode == "pos-vel-pp":
                    if args.vendor != "robstride":
                        raise ValueError("pos-vel-pp mode is only valid for RobStride")
                    motor.robstride_send_pos_vel_pp(args.pos, args.vlim, args.acc)
                elif args.mode == "pos-vel-csp":
                    if args.vendor != "robstride":
                        raise ValueError("pos-vel-csp mode is only valid for RobStride")
                    motor.robstride_send_pos_vel_csp(args.pos, args.vlim)
                elif args.mode == "vel":
                    if args.vendor == "hexfellow":
                        raise ValueError("hexfellow does not support vel command")
                    motor.send_vel(args.vel)
                elif args.mode == "force-pos":
                    if args.vendor in ("robstride", "myactuator", "hexfellow"):
                        raise ValueError(f"{args.vendor} does not support force-pos command")
                    motor.send_force_pos(args.pos, args.vlim, args.ratio)
                elif args.mode in ("zero", "set-zero"):
                    if args.vendor != "robstride":
                        raise ValueError("zero/set-zero mode is currently supported for --vendor robstride only")
                    if not args.zero_exp:
                        print(
                            "[warn] robstride zero requires experimental sequence; "
                            "no CAN frame sent. Re-run with --zero-exp 1"
                        )
                        break
                    # Experimental sequence aligned with core CLI: disable -> set-zero -> optional store.
                    try:
                        motor.disable()
                    except Exception as e:
                        print(f"[warn] pre-zero disable failed: {e}; continue")
                    time.sleep(0.05)
                    motor.set_zero_position()
                    if args.store:
                        motor.store_parameters()
                    print(f"[ok] robstride zero sequence finished (store={int(bool(args.store))})")
                    break

                # Keep feedback state fresh during active control loops.
                if args.vendor == "damiao" and args.mode in ("mit", "pos-vel", "vel", "force-pos"):
                    motor.request_feedback()
                    try:
                        ctrl.poll_feedback_once()
                    except Exception:
                        # Best-effort polling; command loop should keep running.
                        pass

                if args.print_state:
                    st = motor.get_state()
                    if st is None:
                        print(f"#{i} no feedback yet")
                    else:
                        print(
                            f"#{i} pos={st.pos:+.3f} vel={st.vel:+.3f} "
                            f"torq={st.torq:+.3f} status={st.status_code}"
                        )
                        if args.vendor == "robstride":
                            fault_raw, warning_raw = motor.robstride_get_fault_report()
                            if fault_raw or warning_raw:
                                print(f"    fault_raw=0x{fault_raw:08X} warning_raw=0x{warning_raw:08X}")
                time.sleep(max(args.dt_ms, 0) / 1000.0)
        finally:
            motor.close()
