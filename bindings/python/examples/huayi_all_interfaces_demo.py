#!/usr/bin/env python3
"""华翼(Huayi)单电机主要接口全过一遍(Python SDK 真机示例)。

结构与 ``hightorque_all_interfaces_demo.py`` / ``robstride_all_interfaces_demo.py``
对齐: 位置参数、``with ctrl`` 上下文、后台 ``poll_feedback_once`` 线程、
``_run_all`` 编号步骤、``_switch_mode``/``_run_hold``/``_stop_and_disable``/
``_fmt_state`` 助手、print-and-continue。同一调用序列可对
damiao/robstride/hightorque/huayi 复跑——这就是跨厂商语义对齐的验收脚本。

用法:
    # SocketCAN:
    python huayi_all_interfaces_demo.py [channel] [motor_id] [model]
    # 例: python huayi_all_interfaces_demo.py can0 1 huayi-joint

    # mcu-serial(UART→CAN MCU 桥, 经典 CAN):
    python huayi_all_interfaces_demo.py can0 1 huayi-joint \
        --transport mcu-serial --serial-port /dev/ttyACM0 --serial-baud 921600

    # 跳过 set_zero/store(不动电机闪存):
    HUAYI_EXAMPLE_WRITE_CONFIG=0 python huayi_all_interfaces_demo.py ...(参数同上)

覆盖接口(统一 + huayi 私有):
- ``huayi_scan_ids`` ← 节点扫描(NMT start 广播 + 6041h/6061h/603Fh 探测)
- ``request_feedback`` + ``get_state`` → 设备探测(未激活时自动激活 TPDO 流)
- ``clear_error`` ← CiA402 故障复位(0x0006→0x0007)
- ``huayi_sdo_read_u16``(6041h 状态字只读诊断) + ``huayi_get_error_code``
- ``ensure_mode``(统一 1/2/3 → 6060h 编号;含 6061h 回读校验)
- ``set_zero_position`` + ``store_parameters``(3001h "pres" + 1010h;须先 disable)
- ``send_pos_vel``(PP 模式: 6081h+607Ah+CW bit4 握手,小幅正弦)
- ``send_vel``(PV 模式: 60FFh Float32 Rev/s)
- ``huayi_send_torque``(力矩模式: 6071h ‰ 峰值)
- ``send_mit``(慢速 2003h SDO 路径, Kp/Kd 经尺度因子)
- ``huayi_send_mit_compressed``(§5.2 实时通道: RPDO4 一帧 8 字节, 单位链
  Nm/rad→Nm/Rev→12 位线性映射)
- ``huayi_configure_periodic_report``(TPDO 流: 10ms 开/0 停)
- ``huayi_get_error_history``(1003h 遍历)

与 HT/RS 的关键差异:
1. CiA402 状态机是真实的: enable=NMT start+Shutdown→SwitchOn→Enable,
   disable=0x0000(Disable Voltage); PP 位置命令有 CW bit4 新目标握手。
2. 模式编号是 6060h 编号(5=MIT——RobStride 私有协议里 5 是 CSP, 勿混淆);
   统一接口 1/2/3 由 ABI 层映射, FORCE_POS 报错不支持。
3. 状态来自 TPDO 流(锚定/补丁模型, 同 hightorque 两帧反馈语义):
   TPDO2(pos/vel)锚定 seq+1, TPDO1(错误码/状态字/模式)与
   TPDO3(力矩/双温度)就地补丁; 10ms 事件计时器下持续入缓存。

改写电机配置(归零/保存)默认**开启**: 每次运行将当前位置设为零点并落盘。
设 ``HUAYI_EXAMPLE_WRITE_CONFIG=0`` 可跳过。
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time
from pathlib import Path
from typing import Callable

# 默认加载同仓库正在修改的 motorbridge(见 hightorque_all_interfaces_demo 注释)。
_SRC = Path(__file__).resolve().parents[1] / "src"
if _SRC.is_dir():
    sys.path.insert(0, str(_SRC))

import motorbridge
from motorbridge import Controller, Mode
from motorbridge.errors import CallError

HOLD_SECS = 3.0
# TPDO 流周期: 运动段依赖 [1.probe] 的 request_feedback 自动激活(10ms);
# 步骤 10 演示显式开/停。


def main() -> None:
    parser = argparse.ArgumentParser(description="Huayi all-interfaces demo (Python SDK)")
    parser.add_argument("channel", nargs="?", default="can0",
                        help="SocketCAN channel(仅 --transport socketcan 使用)")
    parser.add_argument("motor_id", nargs="?", type=lambda s: int(s, 0), default=1,
                        help="CANopen 节点号(1..=127)")
    parser.add_argument("model", nargs="?", default="huayi-joint",
                        help="型号标签(仅用于显示/记录)")
    parser.add_argument("--feedback-id", type=lambda s: int(s, 0), default=0,
                        help="反馈节点号(0=沿用 motor_id)")
    parser.add_argument("--transport", default="socketcan",
                        choices=["socketcan", "mcu-serial"],
                        help="传输后端; 华翼为经典 CAN, 两种均可用")
    parser.add_argument("--serial-port", default="/dev/ttyACM0")
    parser.add_argument("--serial-baud", type=int, default=921600)
    parser.add_argument("--scan", action="store_true",
                        help="先跑 huayi_scan_ids 扫描 1..=32 节点再注册")
    args = parser.parse_args()

    if not 1 <= args.motor_id <= 127:
        parser.error("motor_id must be a CANopen node id in 1..127")
    if not 0 <= args.feedback_id <= 127:
        parser.error("feedback_id must be in 0..127")

    write_config = os.environ.get("HUAYI_EXAMPLE_WRITE_CONFIG", "1") not in {"0", "false", "off", "no"}
    print(f"motorbridge: {motorbridge.__file__}")
    print(
        "== 华翼主要接口全过一遍(Python SDK)== "
        f"transport={args.transport} "
        f"{('channel=' + args.channel) if args.transport == 'socketcan' else ('port=' + args.serial_port + ' baud=' + str(args.serial_baud))} "
        f"node={args.motor_id} model={args.model} (WRITE_CONFIG={write_config})"
    )

    if args.transport == "mcu-serial":
        ctrl_ctx = Controller.from_mcu_serial(args.serial_port, args.serial_baud)
    else:
        ctrl_ctx = Controller(args.channel)
    with ctrl_ctx as ctrl:
        # 0. 可选扫描(配置前发现节点)。
        if args.scan:
            try:
                nodes = ctrl.huayi_scan_ids(1, 32, 50)
                print(f"[0.scan] 在线节点: {nodes}")
            except CallError as exc:
                print(f"[0.scan] 失败: {exc}(继续)")

        motor = ctrl.add_huayi_motor(args.motor_id, args.feedback_id, args.model)

        # 0b. 被动反馈刷新线程: TPDO 流帧经 CoreController 接收线程分发,
        #     poll 线程兜底(与 HT 示例同骨架)。
        stop = threading.Event()
        poll_thread = threading.Thread(
            target=_poll_loop, name="huayi-poll", args=(ctrl, stop), daemon=True
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
            pass
        time.sleep(0.002)


def _current_pos(motor) -> float | None:
    """读当前位置作运动起点(query_status 语义: request_feedback 在 TPDO
    未激活时会自动激活, 之后由流持续入缓存)。无反馈返回 None——
    调用方应跳过运动, 避免绝对位置命令致大幅运动。"""
    try:
        motor.request_feedback()
    except CallError:
        pass
    time.sleep(0.1)
    s = motor.get_state()
    return s.pos if s is not None else None


def _run_all(motor, write_config: bool) -> None:
    # 1. 设备探测: request_feedback(TPDO 未激活时自动激活 10ms 流)+
    #    get_state。掉线表现为 get_state 返回 None 或冻结旧值; 严格新鲜度
    #    用 get_state_snapshot 的 feedback_age/sequence。
    try:
        motor.request_feedback()
    except CallError as exc:
        print(f"[1.probe] request_feedback 报错: {exc}")
    time.sleep(0.2)
    print(f"[1.probe] state={_fmt_state(motor.get_state())}")

    # 2. clear_error: CiA402 故障复位; 也清掉缓存里的 603Fh 错误码。
    try:
        motor.clear_error()
        print("[2.clear_error] ok")
    except CallError as exc:
        print(f"[2.clear_error] 失败: {exc}(继续)")

    # 2b. 只读诊断(无运动): 6041h 状态字 + 缓存错误码。
    try:
        sw = motor.huayi_sdo_read_u16(0x6041, 0x00)
        print(f"[2b.diag] 6041h statusword=0x{sw:04X}")
    except CallError as exc:
        print(f"[2b.diag] SDO 读失败: {exc}(继续)")
    try:
        print(f"[2b.diag] error_code=0x{motor.huayi_get_error_code():04X}")
    except CallError as exc:
        print(f"[2b.diag] get_error_code: {exc}(可能尚无反馈, 继续)")

    # 3. 模式选择: 统一 ensure_mode(1/2/3 → 6060h; 6061h 回读校验, 通过后
    #    缓存, 后续命令免握手轮次)。enable 是完整 CiA402 序列。
    motor.ensure_mode(Mode.POS_VEL, 1000)
    print(f"[3.ensure_mode] -> POS_VEL  state={_fmt_state(motor.get_state())}")

    # 4. 设零 + 落盘: 3001h 位置预设要求 Switch On Disabled——先 disable
    #    (与 Damiao set_zero 的守卫同构), 再 set_zero_position(= preset 0)
    #    + store_parameters(1010h "save")。归零/落盘放运动之前。
    if write_config:
        motor.disable()
        time.sleep(0.1)
        print("[4.set_zero] 当前位置设为零点(3001h 'pres')...")
        motor.set_zero_position()
        print("[4.store_parameters] 保存(1010h 'save')...")
        motor.store_parameters()
        print("[4] ok")
    else:
        print("[4] 跳过 set_zero/store(设 HUAYI_EXAMPLE_WRITE_CONFIG=0 禁用)")

    # ---- 运动段 ----
    start_pos = _current_pos(motor)
    if start_pos is None:
        print("[5+] 跳过运动: 无状态反馈, 无法确定安全起点")
    else:
        # 5. PP 位置模式(统一 send_pos_vel): 6081h 速度上限 + 607Ah 目标 +
        #    CW bit4 新目标握手(首条 0x003F, 后续 0x002F→0x003F 翻转)。
        _switch_mode(motor, Mode.POS_VEL)
        amp, freq, vlim = 0.2, 2.0, 0.5
        print(f"[5.POS_VEL] start_pos={start_pos:+.3f} amp={amp} vlim={vlim}")
        _run_hold(motor, "5.POS_VEL", HOLD_SECS,
                  lambda t: motor.send_pos_vel(start_pos + amp * math.sin(freq * t), vlim))
        _stop_and_disable(motor)

        # 6. PV 速度模式(send_vel: 60FFh Float32 Rev/s): 小幅正弦速度。
        _switch_mode(motor, Mode.VEL)
        print("[6.VEL] 小幅正弦速度(±0.4 rad/s)")
        _run_hold(motor, "6.VEL", HOLD_SECS,
                  lambda t: motor.send_vel(0.4 * math.sin(freq * t)))
        _stop_and_disable(motor)

        # 7. 力矩模式(huayi_send_torque: 6071h ‰ 6076h 峰值): 小幅正弦
        #    力矩(峰值 0.3 Nm)零起振。
        _switch_mode(motor, Mode.MIT)  # 任意模式先使能; 力矩命令自带切模式
        print("[7.TORQUE] 小幅正弦力矩 0→±0.3 Nm")
        _run_hold(motor, "7.TORQUE", HOLD_SECS,
                  lambda t: motor.huayi_send_torque(0.3 * math.sin(freq * t)))
        _stop_and_disable(motor)

        # 8. MIT 慢速路径(统一 send_mit: 2003h 五次 SDO 写; Kp/Kd 经
        #    尺度因子换算为无量纲 u16)。小增益保守。
        _switch_mode(motor, Mode.MIT)
        print("[8.MIT-slow] 2003h SDO 路径(kp=5 kd=1 保守)")
        _run_hold(motor, "8.MIT-slow", HOLD_SECS,
                  lambda t: motor.send_mit(
                      start_pos + 0.1 * math.sin(1.0 * t), 0.0, 5.0, 1.0, 0.0))
        _stop_and_disable(motor)

        # 9. MIT 压缩实时通道(huayi_send_mit_compressed: RPDO4 一帧 8 字节
        #    到 0x500+node; 首次调用自动重映射+读范围, 之后每命令一帧)。
        #    Kp/Kd 单位链: Nm/rad→Nm/Rev→12 位线性映射。
        _switch_mode(motor, Mode.MIT)
        print("[9.MIT-compressed] RPDO4 实时通道(每命令一帧)")
        _run_hold(motor, "9.MIT-compressed", HOLD_SECS,
                  lambda t: motor.huayi_send_mit_compressed(
                      start_pos + 0.1 * math.sin(1.0 * t), 0.0, 5.0, 1.0, 0.0))
        _stop_and_disable(motor)

    # ---- 无运动诊断段 ----

    # 10. 周期上报开/停: 10ms 静观 2s(流持续入缓存), 0 停止静观 1s
    #     (读数冻结=正确); 越界(>65535)在 Python 层拒绝。
    motor.huayi_configure_periodic_report(10)
    print("[10.REPORT] 10ms TPDO 流开启, 静观 2s(无主动查询)...")
    t0 = time.monotonic()
    last_print = t0
    while time.monotonic() - t0 < 2.0:
        s = motor.get_state()
        if s is not None and time.monotonic() - last_print >= 0.5:
            last_print = time.monotonic()
            print(f"[10.REPORT] passive {_fmt_state(s)}")
        time.sleep(0.01)
    motor.huayi_configure_periodic_report(0)
    print("[10.REPORT] t=0 停止, 静观 1s 确认读数冻结...")
    frozen = None
    t0 = time.monotonic()
    while time.monotonic() - t0 < 1.0:
        s = motor.get_state()
        if s is not None:
            frozen = s
        time.sleep(0.01)
    print(f"[10.REPORT] 停止后 state={_fmt_state(frozen)}(数值不再变=正确)")
    try:
        motor.huayi_configure_periodic_report(70000)
    except ValueError as exc:
        print(f"[10.REPORT] 越界正确拒绝: {exc}")

    # 11. 错误历史(1003h 遍历, 只读)。
    try:
        history = motor.huayi_get_error_history()
        print(f"[11.HISTORY] error_history={[f'0x{c:04X}' for c in history]}")
    except CallError as exc:
        print(f"[11.HISTORY] 失败: {exc}(继续)")

    # 12. 收尾: disable(段 10 已停止 TPDO 流)。
    motor.disable()


def _switch_mode(motor, mode: Mode) -> None:
    """切换模式前的统一前置: disable(0x0000)→短停顿→enable(CiA402 完整
    序列: NMT start + Shutdown→SwitchOn→Enable)→ensure_mode(6060h 写 +
    6061h 回读校验)。"""
    motor.disable()
    time.sleep(0.06)
    motor.enable()
    motor.ensure_mode(mode, 1000)
    print(f"[switch] -> {mode.name}")


def _stop_and_disable(motor) -> None:
    """段间停机: disable 即 0x0000 Disable Voltage(取消力矩)。"""
    motor.disable()
    time.sleep(0.08)


def _run_hold(motor, label: str, secs: float, cmd: Callable[[float], None]) -> None:
    """运行 ``secs`` 秒, 每周期发一次命令, ~100ms 打印一次 ``get_state()``。
    状态由 TPDO 流(PERIODIC_REPORT_MS)持续入缓存, 无需循环内主动查询。"""
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
        f"t_mos={state.t_mos:.1f}°C t_rotor={state.t_rotor:.1f}°C"
    )


if __name__ == "__main__":
    main()
