#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import statistics
import time
from collections.abc import Callable

from motorbridge import Controller


"""
RobStride POS_VEL timing and protocol-path demo.

This script compares three RobStride position-control paths. PP/CSP can be
measured in two profiles:

- full: every timed call runs the complete manual sequence, including enable.
- prepared: setup runs once, then the timed loop only writes the cyclic
  command register(s).

1. legacy
   - Keeps the historical motorbridge mapping.
   - Assumes Position/run_mode=1 has been selected by the wrapper path.
   - Writes:
     - 0x7017 limit_spd = --vlim
     - 0x701E loc_kp = --loc-kp, if >= 0
     - 0x7016 loc_ref = --pos

2. pp
   - Follows the RobStride manual PP position sequence.
   - One call performs:
     - 0x7005 run_mode = 1
     - enable frame, communication type 3
     - 0x7024 vel_max = --vlim
     - 0x7025 acc_set = --acc
     - 0x7016 loc_ref = --pos

3. csp
   - Follows the RobStride manual CSP position sequence.
   - One call performs:
     - 0x7005 run_mode = 5
     - enable frame, communication type 3
     - 0x7017 limit_spd = --vlim
     - 0x7016 loc_ref = --pos

RobStride parameter writes are no-ack by default in this build. To restore
conservative synchronous waiting, set:

    MOTORBRIDGE_ROBSTRIDE_WRITE_ACK_TIMEOUT_MS=260

This script measures Python call duration and loop cadence. It is intended for
safe low-amplitude bench tests; start with --pos 0.0 and a low --vlim.
"""


def _parse_id(text: str) -> int:
    return int(text, 0)


def _percentile(values: list[float], pct: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    idx = min(len(ordered) - 1, max(0, round((pct / 100.0) * (len(ordered) - 1))))
    return ordered[idx]


def _summarize(label: str, call_ms: list[float], period_ms: list[float]) -> None:
    if not call_ms:
        print(f"[summary] {label}: no samples")
        return
    mean_call = statistics.fmean(call_ms)
    p95_call = _percentile(call_ms, 95)
    max_call = max(call_ms)
    if period_ms:
        mean_period = statistics.fmean(period_ms)
        hz = 1000.0 / mean_period if mean_period > 0 else 0.0
        print(
            f"[summary] {label}: call_ms mean={mean_call:.3f} p95={p95_call:.3f} max={max_call:.3f}; "
            f"period_ms mean={mean_period:.3f}; approx_hz={hz:.2f}"
        )
    else:
        print(
            f"[summary] {label}: call_ms mean={mean_call:.3f} p95={p95_call:.3f} max={max_call:.3f}; "
            "period_ms n/a"
        )


def _sleep_remaining(loop_start: float, dt_s: float) -> None:
    if dt_s <= 0:
        return
    elapsed = time.perf_counter() - loop_start
    remaining = dt_s - elapsed
    if remaining > 0:
        time.sleep(remaining)


def _run_timed_loop(
    label: str,
    loop_n: int,
    dt_s: float,
    send: Callable[[], None],
    print_every: int,
) -> None:
    call_ms: list[float] = []
    period_ms: list[float] = []
    last_start: float | None = None

    for i in range(loop_n):
        loop_start = time.perf_counter()
        if last_start is not None:
            period_ms.append((loop_start - last_start) * 1000.0)
        last_start = loop_start

        call_start = time.perf_counter()
        send()
        call_elapsed_ms = (time.perf_counter() - call_start) * 1000.0
        call_ms.append(call_elapsed_ms)

        if print_every > 0 and (i == 0 or (i + 1) % print_every == 0):
            print(f"[{label}] #{i + 1}/{loop_n} call_ms={call_elapsed_ms:.3f}")

        _sleep_remaining(loop_start, dt_s)

    _summarize(label, call_ms, period_ms)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="RobStride POS_VEL legacy / PP / CSP timing demo via Python binding"
    )
    p.add_argument("--channel", default="can0")
    p.add_argument("--model", default="rs-00")
    p.add_argument("--motor-id", default="127")
    p.add_argument("--feedback-id", default="0xFD")
    p.add_argument("--mode", choices=["legacy", "pp", "csp", "all"], default="all")
    p.add_argument(
        "--profile",
        choices=["full", "prepared", "both"],
        default="both",
        help="PP/CSP timing profile: full sequence, prepared cyclic writes, or both",
    )
    p.add_argument("--pos", type=float, default=0.0, help="target loc_ref in radians")
    p.add_argument("--vlim", type=float, default=1.0, help="legacy/CSP limit_spd or PP vel_max")
    p.add_argument("--acc", type=float, default=10.0, help="PP acc_set in rad/s^2")
    p.add_argument("--loc-kp", type=float, default=5.0, help="legacy loc_kp; set <0 to skip")
    p.add_argument("--loop", type=int, default=100)
    p.add_argument("--dt-ms", type=float, default=10.0)
    p.add_argument("--gap-ms", type=float, default=300.0, help="pause between modes when --mode all")
    p.add_argument("--print-every", type=int, default=10)
    p.add_argument(
        "--ack-timeout-ms",
        type=int,
        default=None,
        help="set MOTORBRIDGE_ROBSTRIDE_WRITE_ACK_TIMEOUT_MS before opening the controller",
    )
    return p.parse_args()


def main() -> None:
    args = parse_args()
    if args.ack_timeout_ms is not None:
        os.environ["MOTORBRIDGE_ROBSTRIDE_WRITE_ACK_TIMEOUT_MS"] = str(args.ack_timeout_ms)

    ack_env = os.environ.get("MOTORBRIDGE_ROBSTRIDE_WRITE_ACK_TIMEOUT_MS", "0")
    motor_id = _parse_id(args.motor_id)
    feedback_id = _parse_id(args.feedback_id)
    dt_s = max(args.dt_ms, 0.0) / 1000.0

    selected = ["legacy", "pp", "csp"] if args.mode == "all" else [args.mode]
    print(
        f"channel={args.channel} model={args.model} motor_id=0x{motor_id:X} "
        f"feedback_id=0x{feedback_id:X} ack_timeout_ms={ack_env}"
    )
    print(
        "modes: legacy=(0x7017,0x701E,0x7016), "
        "pp-full=(run_mode=1,enable,0x7024,0x7025,0x7016), "
        "pp-prepared=(setup once, loop 0x7016), "
        "csp-full=(run_mode=5,enable,0x7017,0x7016), "
        "csp-prepared=(setup once, loop 0x7016)"
    )

    with Controller(args.channel) as ctrl:
        motor = ctrl.add_robstride_motor(motor_id, feedback_id, args.model)
        try:
            for idx, mode in enumerate(selected):
                if idx > 0 and args.gap_ms > 0:
                    time.sleep(args.gap_ms / 1000.0)

                if mode == "legacy":
                    print("[legacy] historical mapping: limit_spd + optional loc_kp + loc_ref")

                    def send_legacy() -> None:
                        speed = abs(float(args.vlim))
                        if speed > 0:
                            motor.robstride_write_param_f32(0x7017, speed)
                        if args.loc_kp >= 0:
                            motor.robstride_write_param_f32(0x701E, float(args.loc_kp))
                        motor.robstride_write_param_f32(0x7016, float(args.pos))

                    _run_timed_loop("legacy", args.loop, dt_s, send_legacy, args.print_every)
                elif mode == "pp":
                    profiles = ["full", "prepared"] if args.profile == "both" else [args.profile]
                    for profile in profiles:
                        if profile == "full":
                            print("[pp-full] manual PP sequence every call: run_mode=1 -> enable -> vel_max -> acc_set -> loc_ref")

                            def send_pp_full() -> None:
                                motor.robstride_send_pos_vel_pp(float(args.pos), float(args.vlim), float(args.acc))

                            _run_timed_loop("pp-full", args.loop, dt_s, send_pp_full, args.print_every)
                        else:
                            print("[pp-prepared] setup once: run_mode=1 -> enable -> vel_max -> acc_set; loop: loc_ref")
                            motor.robstride_write_param_i8(0x7005, 1)
                            motor.enable()
                            motor.robstride_write_param_f32(0x7024, abs(float(args.vlim)))
                            motor.robstride_write_param_f32(0x7025, abs(float(args.acc)))

                            def send_pp_prepared() -> None:
                                motor.robstride_write_param_f32(0x7016, float(args.pos))

                            _run_timed_loop("pp-prepared", args.loop, dt_s, send_pp_prepared, args.print_every)
                elif mode == "csp":
                    profiles = ["full", "prepared"] if args.profile == "both" else [args.profile]
                    for profile in profiles:
                        if profile == "full":
                            print("[csp-full] manual CSP sequence every call: run_mode=5 -> enable -> limit_spd -> loc_ref")

                            def send_csp_full() -> None:
                                motor.robstride_send_pos_vel_csp(float(args.pos), float(args.vlim))

                            _run_timed_loop("csp-full", args.loop, dt_s, send_csp_full, args.print_every)
                        else:
                            print("[csp-prepared] setup once: run_mode=5 -> enable -> limit_spd; loop: loc_ref")
                            motor.robstride_write_param_i8(0x7005, 5)
                            motor.enable()
                            motor.robstride_write_param_f32(0x7017, abs(float(args.vlim)))

                            def send_csp_prepared() -> None:
                                motor.robstride_write_param_f32(0x7016, float(args.pos))

                            _run_timed_loop("csp-prepared", args.loop, dt_s, send_csp_prepared, args.print_every)
        finally:
            motor.close()


if __name__ == "__main__":
    main()
