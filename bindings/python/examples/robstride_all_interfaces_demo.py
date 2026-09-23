#!/usr/bin/env python3
"""RobStride 单电机主要接口全过一遍(Python SDK 真机示例)。

跑(默认参数,SocketCAN):
    python robstride_all_interfaces_demo.py [channel] [motor_id] [model]
    # 例: python robstride_all_interfaces_demo.py can0 1 rs-01

走 mcuserial 链路(UART→CAN MCU 桥,经典 CAN,支持 RobStride 11 位标准帧):
    python robstride_all_interfaces_demo.py can0 0x7F rs-06 \
        --transport mcu-serial --serial-port /dev/ttyUSB0 --serial-baud 921600
    # 注:位置参数 channel 在 mcu-serial 下被忽略,给占位 'can0' 即可;
    #     motor_id / model 仍按位置传。

覆盖接口(Python SDK):
- ``robstride_ping`` → 设备识别
- ``clear_error``
- ``ensure_mode(Mode.MIT / VEL / POS_VEL / ROBSTRIDE_POS_VEL_CSP)`` ← 统一模式
  接口,内部已做 disable → 写 run_mode → 回读校验,用来替代"enable +
  get_control_mode"的分离步骤
- ``enable`` / ``disable``
- ``robstride_get_param_f32``(MechanicalPosition / Vbus / MeasuredTorque)
- ``send_mit``(MIT 模式,正弦摆动)
- ``send_vel``(Velocity 模式,正弦速度)
- ``robstride_send_pos_vel_csp``(CSP 模式)
- ``robstride_send_pos_vel_pp``(PP 模式,含加速度)
- ``get_state``(状态回读)
- ``set_zero_position`` + ``store_parameters``(改写电机配置,**默认开**,见下)
  归零/保存放在所有运动之前,且在 ``ensure_mode`` 之前,使落盘只固化零点、
  不改变开机模式(见关键点 3)

关键点:
1. 切换模式前一律先 ``disable()``。RobStride 固件在力矩使能时可能忽略
   ``run_mode`` 写入;``ensure_mode`` 内部已会失能,但 CSP/PP 的
   ``robstride_send_pos_vel_csp/pp`` 只 ``set_mode``+``enable`` 不失能,
   跨模式直调会落模式。本示例每段前显式 ``disable`` + 短停顿,再
   ``ensure_mode`` + ``enable``。
2. 被动状态刷新:RobStride 的 ``send_ext`` 只发不收,``enable`` /
   ``disable`` / ``ping`` / ``get_param`` 等走 ACK,依赖
   ``process_feedback_frame`` 递增序号才能返回。故必须起后台线程循环
   ``ctrl.poll_feedback_once()`` 排空总线;否则 ``enable()`` 都会超时、
   ``get_state()`` 永空。(Rust 侧 ``send_with_status_ack`` 在
   ``status_seq`` 上自旋,该序号只在 ``process_feedback_frame`` 里递增。)

改写电机配置(归零/保存)**默认开启**:每次运行都会 ``set_zero_position`` +
``store_parameters`` 把当前机械零点固化到闪存。设环境变量
``RS_EXAMPLE_WRITE_CONFIG=0`` 可跳过(例如反复测试不想反复写 flash)。
"""
from __future__ import annotations

import argparse
import math
import os
import threading
import time
from typing import Callable

from motorbridge import Controller, Mode

# RobStride 参数 ID(motor_vendors/robstride/src/registers.rs::ParameterId)
PARAM_MECHANICAL_POSITION = 0x7019
PARAM_VBUS = 0x701C
PARAM_MEASURED_TORQUE = 0x302C

HOLD_SECS = 3.0


def main() -> None:
    parser = argparse.ArgumentParser(description="RobStride all-interfaces demo (Python SDK)")
    parser.add_argument("channel", nargs="?", default="can0",
                        help="SocketCAN channel (仅 --transport socketcan 使用;mcu-serial 下忽略)")
    parser.add_argument("motor_id", nargs="?", type=lambda s: int(s, 0), default=1)
    parser.add_argument("model", nargs="?", default="rs-01")
    parser.add_argument("--feedback-id", type=lambda s: int(s, 0), default=0xFD)
    parser.add_argument("--transport", default="socketcan",
                        choices=["socketcan", "mcu-serial"],
                        help="传输后端;mcu-serial=UART→CAN MCU 桥(经典 CAN,支持 11 位标准帧)")
    parser.add_argument("--serial-port", default="/dev/ttyUSB0",
                        help="mcu-serial 串口设备路径")
    parser.add_argument("--serial-baud", type=int, default=921600,
                        help="mcu-serial 串口波特率")
    args = parser.parse_args()

    if not 1 <= args.motor_id <= 255:
        parser.error("motor_id must be in 1..255")
    if not 0 <= args.feedback_id <= 255:
        parser.error("feedback_id must be in 0..255")

    write_config = os.environ.get("RS_EXAMPLE_WRITE_CONFIG", "1") == "1"
    print(
        "== RobStride 主要接口全过一遍(Python SDK)== "
        f"transport={args.transport} "
        f"{('channel=' + args.channel) if args.transport == 'socketcan' else ('port=' + args.serial_port + ' baud=' + str(args.serial_baud))} "
        f"id={args.motor_id} model={args.model} "
        f"fb=0x{args.feedback_id:X} (WRITE_CONFIG={write_config})"
    )

    if args.transport == "mcu-serial":
        ctrl_ctx = Controller.from_mcu_serial(args.serial_port, args.serial_baud)
    else:
        ctrl_ctx = Controller(args.channel)
    with ctrl_ctx as ctrl:
        motor = ctrl.add_robstride_motor(args.motor_id, args.feedback_id, args.model)

        # 0. 被动反馈刷新线程:必须先起,否则 ACK 全超时、状态永空。
        stop = threading.Event()
        poll_thread = threading.Thread(
            target=_poll_loop, name="rs-poll", args=(ctrl, stop), daemon=True
        )
        poll_thread.start()
        time.sleep(0.05)  # 给 poll 线程起步 + 电机主动上报一点时间

        try:
            _run_all(motor, write_config)
        finally:
            stop.set()
            try:
                ctrl.disable_all()
            except Exception as exc:  # noqa: BLE001
                print(f"[cleanup] disable_all: {exc}")
    print("== 完成 ==")


def _poll_loop(ctrl: Controller, stop: threading.Event) -> None:
    while not stop.is_set():
        try:
            ctrl.poll_feedback_once()
        except Exception:  # noqa: BLE001
            # 单次排空失败不应终止刷新线程(总线瞬断、控制器关闭中均可能抛)。
            pass
        time.sleep(0.002)


def _run_all(motor, write_config: bool) -> None:
    # 1. ping:探测设备与 host_id。
    try:
        device_id, responder_id = motor.robstride_ping()
        print(f"[1.ping] device_id={device_id} responder_id={responder_id}")
    except Exception as exc:  # noqa: BLE001
        print(f"[1.ping] 失败: {exc}(继续)")

    # 2. clear_error:清故障(同时清 fault_report 缓存)。
    motor.clear_error()
    print("[2.clear_error] ok")

    # 3. 归零 + 落盘(默认开):在电机静止、未使能力矩的状态下设定机械零点并
    #    固化到闪存。**刻意放在 ensure_mode 之前**:此刻 RAM 里尚未写入
    #    run_mode,store_parameters 落盘时只会改变零点、不会把 MIT 顺带
    #    持久化为开机模式。set_zero_position / store_parameters 均为配置
    #    ACK 指令,不依赖模式,在 disable 状态下即可执行。设
    #    RS_EXAMPLE_WRITE_CONFIG=0 可跳过。
    if write_config:
        motor.disable()
        time.sleep(0.1)
        print("[3.set_zero] 归零中(机械动作,需等待)...")
        motor.set_zero_position()
        print("[3.store_parameters] 保存到闪存(只固化零点)...")
        motor.store_parameters()
        print("[3] ok")
    else:
        print("[3] 跳过 set_zero/store(设 RS_EXAMPLE_WRITE_CONFIG=0 跳过;默认开)")

    # 4. 模式选择 + 使能:用 ensure_mode 统一接口(内部 disable→写 run_mode→
    #    回读校验),替代"enable + get_control_mode"分离步骤。ensure_mode 只
    #    选定模式、不闭合力矩,故随后 enable()。此处写入的 run_mode 仅在 RAM,
    #    不落盘(落盘已在第 3 步完成)。
    motor.ensure_mode(Mode.MIT, 1000)
    motor.enable()
    print(f"[4.ensure_mode+enable] -> MIT  state={_fmt_state(motor.get_state())}")

    # 5. get_parameter:机械位置(相对已归零零点)/ 母线电压 / 实测力矩。
    pos = motor.robstride_get_param_f32(PARAM_MECHANICAL_POSITION, 300)
    vbus = motor.robstride_get_param_f32(PARAM_VBUS, 300)
    trq = motor.robstride_get_param_f32(PARAM_MEASURED_TORQUE, 300)
    print(f"[5.get_param] pos={pos:+.3f} vbus={vbus:.2f}V torque={trq:+.3f}")

    # ---- 模式段:每段前 disable + 短停顿,再 ensure_mode + enable ----

    # 6. MIT 模式:正弦位置摆动。
    _switch_mode(motor, Mode.MIT)
    _run_hold(motor, "6.MIT", HOLD_SECS, lambda t: motor.send_mit(math.sin(2.0 * t), 0.0, 5.0, 0.5, 0.0))
    _stop_and_disable(motor)

    # 7. Velocity 模式:正弦速度。
    _switch_mode(motor, Mode.VEL)
    _run_hold(motor, "7.VEL", HOLD_SECS, lambda t: motor.send_vel(math.sin(1.0 * t) * 2.0))
    _stop_and_disable(motor)

    # 8. Position-CSP 模式:目标位置缓动、速度限 1.0。
    _switch_mode(motor, Mode.ROBSTRIDE_POS_VEL_CSP)
    _run_hold(
        motor,
        "8.CSP",
        HOLD_SECS,
        lambda t: motor.robstride_send_pos_vel_csp(math.sin(0.5 * t), 1.0),
    )
    _stop_and_disable(motor)

    # 9. Position-PP 模式:含加速度。
    _switch_mode(motor, Mode.POS_VEL)
    _run_hold(
        motor,
        "9.PP",
        HOLD_SECS,
        lambda t: motor.robstride_send_pos_vel_pp(math.sin(0.5 * t), 1.0, 2.0),
    )
    _stop_and_disable(motor)


def _switch_mode(motor, mode: Mode) -> None:
    """切换模式前的统一前置:失能 → 短停顿 → ensure_mode(内部再失能+写
    run_mode+回读校验)→ 使能。显式 disable 是因为 RobStride 固件在 torque
    使能时可能忽略 run_mode 写入。"""
    motor.disable()
    time.sleep(0.06)
    motor.ensure_mode(mode, 1000)
    motor.enable()
    print(f"[switch] -> {mode.name}")


def _stop_and_disable(motor) -> None:
    """Python SDK 未暴露 controlled_stop,直接 disable 作为段间停机。"""
    motor.disable()
    time.sleep(0.08)


def _run_hold(motor, label: str, secs: float, cmd: Callable[[float], None]) -> None:
    """运行 ``secs`` 秒,每周期发一次命令,~100ms 打印一次 ``get_state()``。"""
    start = time.monotonic()
    last_print = start
    while time.monotonic() - start < secs:
        t = time.monotonic() - start
        try:
            cmd(t)
        except Exception as exc:  # noqa: BLE001
            print(f"[{label}] 命令失败: {exc}")
            break
        if time.monotonic() - last_print >= 0.1:
            last_print = time.monotonic()
            print(f"[{label}] t={t:.2f} {_fmt_state(motor.get_state())}")
        time.sleep(0.01)


def _fmt_state(state) -> str:
    if state is None:
        return "(无状态反馈)"
    return (
        f"pos={state.pos:+.3f} vel={state.vel:+.3f} torq={state.torq:+.3f} "
        f"t_mos={state.t_mos:.1f}°C"
    )


if __name__ == "__main__":
    main()
