#!/usr/bin/env python3
"""HighTorque 单电机主要接口全过一遍(Python SDK 真机示例)。

结构与 ``robstride_all_interfaces_demo.py`` 对齐:位置参数、``with ctrl``
上下文、后台 ``poll_feedback_once`` 线程、``_run_all`` 编号步骤、
``_switch_mode``/``_run_hold``/``_stop_and_disable``/``_fmt_state`` 助手、
print-and-continue。差异点见各步注释。

跑(默认参数,SocketCAN):
    python hightorque_all_interfaces_demo.py [channel] [motor_id] [model]
    # 例: python hightorque_all_interfaces_demo.py can0 1 ht

走 mcuserial 链路(UART→CAN MCU 桥,经典 CAN):
    python hightorque_all_interfaces_demo.py can0 1 ht \
        --transport mcu-serial --serial-port /dev/ttyUSB0 --serial-baud 921600
    # 注:位置参数 channel 在 mcu-serial 下被忽略,给占位 'can0' 即可。

覆盖接口(Python SDK,运动仅测位置模式):
- ``request_feedback`` + ``get_state`` → 设备探测(HT 无 ping,以状态回读代)
- ``clear_error`` ← HT 显式 Unsupported,打印并继续
- ``ensure_mode(Mode.POS_VEL)`` ← 统一模式接口(HT 仅校验 mode<=17,
  不写寄存器;运动命令帧自带模式)
- ``enable`` / ``disable`` ← HT 的 enable=Unsupported,disable=send_stop
- ``set_zero_position`` + ``store_parameters``(改写电机配置,默认开,见下)
- ``send_pos_vel``(位置模式,协同 pos-vel 0x07 0x35,按当前位小幅正弦振荡)

与 RS 的关键差异:
1. HT 协议无 separate 使能帧 / 模式切换帧:控制帧的 cmd 字节即决定模式
   (0x07 0x35=位置、0x07 0x07=速度、0x18000|id=MIT),电机收到即执行,
   无需先 enable 再切 run_mode。故 ``_switch_mode`` 不调 enable,``ensure_mode``
   仅做范围校验(协议确无 run_mode 寄存器可写,参考固件 `motor_control.c`
   全表无 `motor_enable`/`motor_set_mode`)。
2. 本次按用户要求只测位置模式(``send_pos_vel``);MIT / 速度 / force_pos 不测。

改写电机配置(归零/保存)默认**开启**:每次运行会在当前位置设定机械零点
并写闪存。设环境变量 ``HT_EXAMPLE_WRITE_CONFIG=0`` 可跳过(仅测运动、不动配置)。
"""
from __future__ import annotations

import argparse
import math
import os
import threading
import time
from typing import Callable

from motorbridge import Controller, Mode
from motorbridge.errors import CallError

HOLD_SECS = 3.0


def main() -> None:
    parser = argparse.ArgumentParser(description="HighTorque all-interfaces demo (Python SDK)")
    parser.add_argument("channel", nargs="?", default="can0",
                        help="SocketCAN channel(仅 --transport socketcan 使用;mcu-serial 下忽略)")
    parser.add_argument("motor_id", nargs="?", type=lambda s: int(s, 0), default=1)
    parser.add_argument("model", nargs="?", default="ht",
                        help="型号提示(默认 ht=General k=0.5;未知型号走 General)")
    parser.add_argument("--feedback-id", type=lambda s: int(s, 0), default=0x01,
                        help="反馈/回复源 ID(默认 0x01)")
    parser.add_argument("--transport", default="socketcan",
                        choices=["socketcan", "mcu-serial"],
                        help="传输后端;mcu-serial=UART→CAN MCU 桥(经典 CAN)")
    parser.add_argument("--serial-port", default="/dev/ttyUSB0",
                        help="mcu-serial 串口设备路径")
    parser.add_argument("--serial-baud", type=int, default=921600,
                        help="mcu-serial 串口波特率")
    args = parser.parse_args()

    if not 1 <= args.motor_id <= 127:
        parser.error("motor_id must be in 1..127 (hightorque dest field is 7-bit)")
    if not 0 <= args.feedback_id <= 127:
        parser.error("feedback_id must be in 0..127")

    write_config = os.environ.get("HT_EXAMPLE_WRITE_CONFIG", "1") not in {"0", "false", "off", "no"}
    print(
        "== HighTorque 主要接口全过一遍(Python SDK)== "
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
        motor = ctrl.add_hightorque_motor(args.motor_id, args.feedback_id, args.model)

        # 0. 被动反馈刷新线程:必须先起,否则状态永空。
        #    send_cmd_* 只发不收,状态靠被动上报;0x27 回读也由它排空填 state。
        #    (set_zero/store 已改 fire-and-forget,不再依赖 ACK 轮询。)
        stop = threading.Event()
        poll_thread = threading.Thread(
            target=_poll_loop, name="ht-poll", args=(ctrl, stop), daemon=True
        )
        poll_thread.start()
        time.sleep(0.05)

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


def _current_pos(motor) -> float | None:
    """读当前位置作运动起点。request_feedback 与 poll 线程抢帧,超时也容忍
    (poll 抢到回复时 state 仍会被填)。无反馈返回 None——调用方应跳过运动,
    避免绝对位置命令致大幅运动。"""
    try:
        motor.request_feedback()
    except CallError:
        pass
    time.sleep(0.1)
    s = motor.get_state()
    return s.pos if s is not None else None


def _run_all(motor, write_config: bool) -> None:
    # 1. 设备探测:HT 无 ping,以 request_feedback + get_state 代替(RS 用 robstride_ping)。
    #    request_feedback 发 0x17 0x01 查询后,内部 wait_status 自调 bus.recv 等
    #    0x27 回复——这与后台 poll 线程抢帧:poll 抢到时 request_feedback 会假性
    #    Timeout,但 poll 的 process_feedback_frame 经 decode_feedback 也认 0x27
    #    回复,仍会填 state cache。故不把它的 Timeout 当失败,以 get_state 判定。
    try:
        motor.request_feedback()
    except CallError as exc:
        print(f"[1.probe] request_feedback 报错(可能 poll 线程抢帧): {exc}")
    time.sleep(0.1)  # 给被动反馈一点时间落地
    state = motor.get_state()
    print(f"[1.probe] state={_fmt_state(state)}")

    # 2. clear_error:HT 显式 Unsupported(协议无此命令),打印并继续。
    try:
        motor.clear_error()
        print("[2.clear_error] ok")
    except CallError as exc:
        print(f"[2.clear_error] 跳过(Unsupported): {exc}")
    except Exception as exc:  # noqa: BLE001
        print(f"[2.clear_error] 失败: {exc}(继续)")

    # 3. 模式选择:ensure_mode 统一接口(HT 仅校验 mode<=17,不写寄存器)。
    #    HT 协议无 separate 使能帧,故不调 enable()(RS 此处会 enable 闭合力矩)。
    #    本次只测位置模式,故选 Mode.POS_VEL。
    motor.ensure_mode(Mode.POS_VEL, 1000)
    print(f"[3.ensure_mode] -> POS_VEL  state={_fmt_state(motor.get_state())}")

    # 4. 设置当前位置为零点 + 落盘。默认开。
    #    set_zero_position 发 0x40(仅在 RAM 改位置偏置,无机械动作——固件
    #    注释"此指令只是在 RAM 中修改"),fire-and-forget 不等 ACK;随即
    #    store_parameters 发 0x05 0xB3 落盘 flash。归零/落盘放运动之前:
    #    静止状态下设零点,避免运动后基准漂移。
    if write_config:
        motor.disable()
        time.sleep(0.1)
        print("[4.set_zero] 设置当前位置为零点(fire-and-forget)...")
        motor.set_zero_position()
        print("[4.store_parameters] 保存到闪存...")
        motor.store_parameters()
        print("[4] ok")
    else:
        print("[4] 跳过 set_zero/store(设 HT_EXAMPLE_WRITE_CONFIG=0 禁用)")

    # ---- 位置模式段:仅测此模式 ----
    # HT 协议无 separate 使能帧 / 模式切换帧:发 0x07 0x35(协同 pos-vel)即
    # 进入位置模式,电机收到即执行。ensure_mode 仅范围校验;段前
    # disable(send_stop = 0x01 0x00 0x00)为安全停机,不依赖模式。
    # send_pos_vel 单位:pos=rad,vlim=rad/s(走 SDK 统一单位)。
    #
    # 位置命令是绝对位置(相对机械零点)。为避免未归零时大幅运动,先读当前
    # 位作起点,以小幅正弦在当前位附近振荡(start_pos + amp·sin),不依赖
    # 零点基准。无反馈则跳过(无法确定安全起点)。amp=0.2 rad≈11.5°,
    # 角频 2.0 rad/s(峰值速度 amp·2=0.4 rad/s < vlim 1.0)。
    start_pos = _current_pos(motor)
    if start_pos is None:
        print("[5.POS_VEL] 跳过:无状态反馈,无法确定安全起点(绝对位置命令可能致大幅运动)")
    else:
        _switch_mode(motor, Mode.POS_VEL)
        amp, freq, vlim = 0.2, 2.0, 1.0
        print(f"[5.POS_VEL] start_pos={start_pos:+.3f} amp={amp} vlim={vlim}")
        _run_hold(motor, "5.POS_VEL", HOLD_SECS,
                  lambda t: motor.send_pos_vel(start_pos + amp * math.sin(freq * t), vlim))
        _stop_and_disable(motor)


def _switch_mode(motor, mode: Mode) -> None:
    """切换模式前的统一前置:失能(send_stop)→ 短停顿 → ensure_mode(校验)。

    与 RS 的差异:HT 无 enable,故不调 enable();ensure_mode 仅校验 mode<=17,
    不写 run_mode 寄存器(HT 运动模式由命令帧 cmd 字节决定)。
    """
    motor.disable()
    time.sleep(0.06)
    motor.ensure_mode(mode, 1000)
    print(f"[switch] -> {mode.name}")


def _stop_and_disable(motor) -> None:
    """段间停机:disable 即 send_stop(0x01 0x00 0x00)。"""
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
