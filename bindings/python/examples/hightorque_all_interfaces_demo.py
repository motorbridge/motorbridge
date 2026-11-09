#!/usr/bin/env python3
"""HighTorque 单电机主要接口全过一遍(Python SDK 真机示例)。

结构与 ``robstride_all_interfaces_demo.py`` 对齐:位置参数、``with ctrl``
上下文、后台 ``poll_feedback_once`` 线程、``_run_all`` 编号步骤、
``_switch_mode``/``_run_hold``/``_stop_and_disable``/``_fmt_state`` 助手、
print-and-continue。差异点见各步注释。

用法:
    # SocketCAN:
    python hightorque_all_interfaces_demo.py [channel] [motor_id] [model]
    # 例: python hightorque_all_interfaces_demo.py can0 1 ht

    # mcu-serial(UART→CAN MCU 桥):
    python hightorque_all_interfaces_demo.py can0 1 6056_36 \
        --transport mcu-serial --serial-port /dev/ttyUSB0 --serial-baud 921600
    # 注:channel 在 mcu-serial 下被忽略;型号须写全(如 6056_36),裸 6056 不识别。

    # 跳过 set_zero/store(不动电机闪存):
    HT_EXAMPLE_WRITE_CONFIG=0 python hightorque_all_interfaces_demo.py ...(参数同上)

覆盖接口(Python SDK,运动=位置三帧+力矩,另含底层原语):
- ``request_feedback`` + ``get_state`` → 设备探测(HT 无 ping,以状态回读代)
- ``clear_error`` ← 映射到停止帧(表2 模式 0 "停止，清除错误",与 disable 同帧)
- ``ensure_mode(Mode.POS_VEL)`` ← 统一模式接口(HT 仅校验 mode<=17,
  不写寄存器;运动命令帧自带模式)
- ``enable`` / ``disable`` ← HT 的 enable=语义空操作(2026-10-08 起不发帧
  直接成功),disable=send_stop
- ``set_zero_position`` + ``store_parameters``(改写电机配置,默认开,见下)
- ``hightorque_get_register``(17 族通用寄存器读 0x01-0x03,只读诊断)
- ``send_pos_vel``(位置模式,协同 pos-vel 0x07 0x35,按当前位小幅正弦振荡)
- ``hightorque_send_pos_vel_acc``(梯形 07 2d,同幅正弦;需固件 v4.6.0+)
- ``hightorque_send_torque``(力矩模式 05 13,小幅正弦力矩 0→±0.3 Nm)
- ``hightorque_send_pos_classic``(普通位置 07 07,固件内部 PID,幅值减半保守)
- ``hightorque_brake``(01 00 0F 阻尼刹车,运动中刹车观察 vel 衰减)
- ``send_mit``(MIT 位打包 0x18000|id,固件 v4.6.0+,小增益正弦,段 9b)
- ``hightorque_set_run_mode``(01 00 <mode> 显式模式切换;0=停止清错,18 拒绝)
- ``hightorque_write_register``(05 寄存器写:0x22 前馈扭矩写 0,读回校验)
- ``hightorque_set_periodic_report``(05 B4 02 周期上报:2c 按文件头开关开
  10ms;2026-10-24 起运动段循环内主动 request_feedback 拉状态,本开关仅
  演示被动上报与段 12 的停止后冻结对比)

与 RS 的关键差异:
1. HT 协议无 separate 使能帧 / 模式切换帧:控制帧的 cmd 字节即决定模式
   (0x07 0x35=位置、0x07 0x07=速度、0x18000|id=MIT),电机收到即执行,
   无需先 enable 再切 run_mode。``enable`` 为语义空操作(2026-10-08 起
   方案 B:不发任何帧,直接成功),``_switch_mode`` 保留调用仅为对齐统一
   enable→command 流程;``ensure_mode`` 仅做范围校验(协议确无 run_mode
   寄存器可写,参考固件 `motor_control.c` 全表无 `motor_enable`/`motor_set_mode`)。
2. 运动测试覆盖位置三帧(协同 07 35 / 梯形 07 2d / 普通 07 07)+ 力矩
   (05 13)+ MIT(0x18000|id 位打包,段 9b)+ 底层原语(brake/run_mode/
   write_reg/periodic_report);速度 / force_pos 不测。

改写电机配置(归零/保存)默认**开启**:每次运行会在当前位置设定机械零点
并写闪存。设环境变量 ``HT_EXAMPLE_WRITE_CONFIG=0`` 可跳过(仅测运动、不动配置)。

文件开头另有运行开关(直接改源码常量,段代码不动):``PERIODIC_REPORT_MS``
控制 [2c] 主动(周期)上报,默认 ``None``=跳过(维护者有意关闭)。2026-10-24
起运动段循环内主动 request_feedback,读数实时与该开关无关;开关仅演示
被动上报形式,段 12 自含不受影响。
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

# 默认加载同仓库正在修改的 motorbridge,而非 pip install 的旧版本:
# 本脚本在 bindings/python/examples/,把 bindings/python/src 插到
# sys.path 首位后 import 优先命中本地源码;ABI .so 由 abi.py 按
# 仓库根 target/release 自动发现(或 MOTORBRIDGE_LIB 显式指定)。
_SRC = Path(__file__).resolve().parents[1] / "src"
if _SRC.is_dir():
    sys.path.insert(0, str(_SRC))

import motorbridge
from motorbridge import Controller, Mode
from motorbridge.errors import CallError

HOLD_SECS = 3.0

# ---- 文件内配置开关(编辑此处定制运行,无需改段代码) ----
# [2c] 周期上报(主动上报):设整数 = 上报周期 ms(推荐 10);设 None =
#     跳过 [2c]。控制帧只发不收,但 2026-10-24 起运动段(_run_hold)
#     循环内主动 request_feedback 拉状态,读数不再冻结;本开关现仅影响
#     [2c] 演示段与段 12 的被动观察对比。段 12 自含,不受本开关影响。
PERIODIC_REPORT_MS: int | None = None


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
    print(f"motorbridge: {motorbridge.__file__}")
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
    """读当前位置作运动起点。request_feedback 2026-10-23 起为 DM 语义
    非阻塞(2026-10-24 起连发两帧: 17 01 位置/速度/力矩 + 16 0E 温度/
    错误码,发完即返回,不抛 Timeout);下方 catch CallError 仅兜底
    总线级错误。发完 sleep 一拍等回帧入缓,再 get_state 取值。无反馈
    返回 None——调用方应跳过运动,避免绝对位置命令致大幅运动。"""
    try:
        motor.request_feedback()
    except CallError:
        pass
    time.sleep(0.1)
    s = motor.get_state()
    return s.pos if s is not None else None


def _run_all(motor, write_config: bool) -> None:
    # 1. 设备探测:HT 无 ping,以 request_feedback + get_state 代替(RS 用 robstride_ping)。
    #    request_feedback(ABI 层)2026-10-23 起为 DM 语义非阻塞,2026-10-24 起连发
    #    两帧 2 字节查询(17 01 位置/速度/力矩 + 16 0E 温度/错误码;真机实测
    #    固件一帧一对,总耗时 ~1ms)即返回,回帧由后台 poll 线程入缓——
    #    27 01 回帧锚定一次状态计数,26 0E 就地补丁 t_mos/fault_code(不推进
    #    计数)。掉线不再表现为 Timeout 报错,而是 get_state 返回 None(从未
    #    收到)或冻结旧值;需要严格新鲜度判定用 get_state_snapshot 的
    #    received_at 时间戳。catch 仅兜底总线级错误。
    try:
        motor.request_feedback()
    except CallError as exc:
        print(f"[1.probe] request_feedback 报错: {exc}")
    time.sleep(0.1)  # 给被动反馈一点时间落地
    state = motor.get_state()
    print(f"[1.probe] state={_fmt_state(state)}")

    # 2. clear_error:表2 模式 0 名称即"停止，清除错误",与 disable 同帧
    #    (0x01 0x00 0x00)。注意:清错会让电机停止。
    try:
        motor.clear_error()
        print("[2.clear_error] ok")
    except CallError as exc:
        print(f"[2.clear_error] 失败: {exc}(继续)")

    # 2b. 只读诊断(无运动):通用寄存器读(17 族,固定 Int16,0x01-0x03 =
    #     位置/速度/转矩,返回换算后物理量:rad / rad/s / N·m)。
    try:
        vals = motor.hightorque_get_register(0x01, count=3)
        print(f"[2b.read_reg] 0x01-0x03 物理 pos={vals[0]:+.4f}rad vel={vals[1]:+.4f}rad/s torq={vals[2]:+.4f}N·m")
    except CallError as exc:
        print(f"[2b.read_reg] 失败: {exc}(继续)")

    # 2c. 周期上报开启(05 B4 02 <t>,扩展帧):可选——由文件开头
    #     PERIODIC_REPORT_MS 开关控制。控制帧只发不收;2026-10-24 起
    #     运动段(_run_hold)循环内主动 request_feedback 拉状态,读数
    #     不再依赖本开关。开启后电机按周期主动发 27 01 状态帧,由后台
    #     poll 线程吃进——演示"电机主动推"的形式,与运动段的"主机主动
    #     拉"形成对照。段 12 自含,不受本开关影响。
    if PERIODIC_REPORT_MS is not None and PERIODIC_REPORT_MS > 0:
        motor.hightorque_set_periodic_report(PERIODIC_REPORT_MS)
        print(f"[2c.periodic_report] {PERIODIC_REPORT_MS}ms 被动上报开启(电机主动推)")
    else:
        print("[2c.periodic_report] 跳过(PERIODIC_REPORT_MS=None):运动段由循环内 request_feedback 主动拉")

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
        amp, freq, vlim = 0.2, 2.0, 0.5
        print(f"[5.POS_VEL] start_pos={start_pos:+.3f} amp={amp} vlim={vlim}")
        _run_hold(motor, "5.POS_VEL", HOLD_SECS,
                  lambda t: motor.send_pos_vel(start_pos + amp * math.sin(freq * t), vlim))
        _stop_and_disable(motor)

        # 6. 梯形模式(07 2d,固件 v4.6.0+):同一幅正弦改走
        #    send_pos_vel_acc(加速度上限 2.0 rad/s²)。旧固件不识别 07 2d,
        #    电机不响应,无副作用。
        _switch_mode(motor, Mode.POS_VEL)
        print("[6.POS_VEL_ACC] 同幅正弦改走梯形帧(需固件 v4.6.0+)")
        _run_hold(motor, "6.POS_VEL_ACC", HOLD_SECS,
                  lambda t: motor.hightorque_send_pos_vel_acc(
                      start_pos + amp * math.sin(freq * t), vlim, 2.0))
        _stop_and_disable(motor)

        # 7. 力矩模式(05 13):send_torque 物理量 Nm(型号系数补偿),
        #    无需预先切模式。小幅正弦力矩(峰值 0.3 Nm)零起振,
        #    电机在外力下小幅摆动或保持,结束时 disable 停机。
        print("[7.TORQUE] 小幅正弦力矩 0→±0.3 Nm(05 13 帧直发力矩目标)")
        _run_hold(motor, "7.TORQUE", HOLD_SECS,
                  lambda t: motor.hightorque_send_torque(0.1 * math.sin(freq * t)))
        _stop_and_disable(motor)

        # 8. 普通位置模式(07 07):目标位置+力矩上限走固件内部 PID
        #    (Kp/Ki/Kd = 寄存器 0x2B-0x2D,出厂默认值,宿主不参与)。
        #    首测幅值减半(0.1 rad)、频率减半(1.0 rad/s)——固件增益未知,
        #    保守起见;若观察明显过冲/抖动属增益配置问题,链路本身已验证。
        _switch_mode(motor, Mode.POS_VEL)
        print("[8.POS_CLASSIC] 普通位置帧 07 07(固件内部 PID,amp 减半保守)")
        _run_hold(motor, "8.POS_CLASSIC", HOLD_SECS,
                  lambda t: motor.hightorque_send_pos_classic(
                      start_pos + 0.1 * math.sin(1.0 * t)))
        _stop_and_disable(motor)

        # 9. 刹车(01 00 0F = 模式 15 相短接阻尼):先 1s 小幅位置正弦获得
        #    低速,中途 brake 后主动查询观察 vel 衰减——刹车后不再发控制
        #    帧故无被动回帧,需 request_feedback 拉状态(读查询不退出刹车态)。
        #    brake 内部即 set_run_mode(15),等效间接验证模式切换帧。
        print("[9.BRAKE] 小幅运动中刹车(01 00 0F),观察 vel 衰减")
        _run_hold(motor, "9.BRAKE-motion", 1.0,
                  lambda t: motor.send_pos_vel(start_pos + 0.1 * math.sin(4.0 * t), 0.5))
        motor.hightorque_brake()
        for i in range(3):
            try:
                motor.request_feedback()
            except CallError:
                pass
            time.sleep(0.2)
            print(f"[9.BRAKE] t+{(i + 1) * 0.2:.1f}s {_fmt_state(motor.get_state())}")
        motor.disable()  # 刹车态回停止/清错(01 00 00)

        # 9b. MIT 模式(0x18000|id 位打包帧,固件 v4.6.0+):统一 ``send_mit``
        #     接口,pos/vel/kp/kd/tau 一帧五位打包。旧固件不识别该帧,电机
        #     不响应,无副作用。幅值同段 8 减半保守(0.1 rad,1.0 rad/s),
        #     小增益 kp=20/kd=1:输出力矩 = (目标位-当前位)·kp +
        #     (目标速度-当前速度)·kd + 前馈 0(参考固件按型号做 pid_adjust
        #     增益补偿,本链路未补偿,增益过大易过冲)。SDK 单位:pos=rad,
        #     vel=rad/s,tau=前馈 Nm;量程 pos ±3.2768 圈 / vel ±2.0 圈/s /
        #     tau ±10 Nm / kp ±400 / kd ±100,超限饱和打包。
        _switch_mode(motor, Mode.MIT)
        print("[9b.MIT] 位打包 MIT 帧 0x18000|id(需 v4.6.0+;kp=20 kd=1 保守)")
        _run_hold(motor, "9b.MIT", HOLD_SECS,
                  lambda t: motor.send_mit(
                      start_pos + 0.1 * math.sin(1.0 * t), 0.0, 5.0, 1.0, 0.0))
        _stop_and_disable(motor)

    # ---- 底层原语段(无运动,不依赖 start_pos/反馈) ----

    # 10. 显式模式切换(01 00 <mode>):0=停止清错(与 disable 同帧)、
    #     5=预选位置模式(只切模式不发目标,电机保持静止)。这是命令帧
    #     之外唯一的手动模式路径;18 越界在 Python 层 ValueError,不占总线。
    motor.hightorque_set_run_mode(0)
    print("[10.SET_RUN_MODE] mode=0 停止/清错 ok")
    time.sleep(0.1)
    motor.hightorque_set_run_mode(5)
    print("[10.SET_RUN_MODE] mode=5 预选位置模式 ok(不发目标,静止)")
    time.sleep(0.2)
    print(f"[10.SET_RUN_MODE] state={_fmt_state(motor.get_state())}")
    try:
        motor.hightorque_set_run_mode(18)
    except ValueError as exc:
        print(f"[10.SET_RUN_MODE] 18 越界正确拒绝: {exc}")
    motor.disable()

    # 11. 寄存器写(05 <addr> <val16>):0x22 前馈扭矩写 0(可逆、零副作用),
    #     随即 17 族读回校验往返。控制类寄存器走标准控制通道
    #     (config_class=False);int16 越界在 Python 层直接拒绝。
    #     注意读写不对称: 写 raw i16, 读回的是换算后物理量(N·m, 套
    #     型号 k 系数)——写 0 读回 0.0 即往返一致。
    motor.hightorque_write_register(0x22, 0)
    time.sleep(0.05)
    try:
        vals = motor.hightorque_get_register(0x22)
        print(f"[11.WRITE_REG] 0x22 前馈扭矩 写0 读回物理量={vals[0]:+.4f}N·m")
    except CallError as exc:
        print(f"[11.WRITE_REG] 读回失败: {exc}(继续)")
    try:
        motor.hightorque_write_register(0x22, 40000)
    except ValueError as exc:
        print(f"[11.WRITE_REG] int16 越界正确拒绝: {exc}")

    # 12. 周期上报(05 B4 02 <t>):本段自含——先自行(重)设 10ms,不依赖
    #     2c 的配置开关;随后静观 2s 确认状态持续刷新(全程无主动查询,
    #     全靠被动帧);再 t=0 停止并静观 1s,读数不再变化即上报已停。
    #     越界(>65535)在 Python 层拒绝。
    motor.hightorque_set_periodic_report(10)
    print("[12.PERIODIC_REPORT] 已自行(重)设 10ms,静观 2s(无主动查询)...")
    t0 = time.monotonic()
    last_print = t0
    while time.monotonic() - t0 < 2.0:
        s = motor.get_state()
        if s is not None and time.monotonic() - last_print >= 0.5:
            last_print = time.monotonic()
            print(f"[12.PERIODIC_REPORT] passive {_fmt_state(s)}")
        time.sleep(0.01)
    motor.hightorque_set_periodic_report(0)
    print("[12.PERIODIC_REPORT] t=0 停止上报,静观 1s 确认读数冻结...")
    frozen = None
    t0 = time.monotonic()
    while time.monotonic() - t0 < 1.0:
        s = motor.get_state()
        if s is not None:
            frozen = s
        time.sleep(0.01)
    print(f"[12.PERIODIC_REPORT] 停止后 state={_fmt_state(frozen)}(数值不再变=正确)")
    try:
        motor.hightorque_set_periodic_report(70000)
    except ValueError as exc:
        print(f"[12.PERIODIC_REPORT] 越界正确拒绝: {exc}")


def _switch_mode(motor, mode: Mode) -> None:
    """切换模式前的统一前置:失能(send_stop)→ 短停顿 → enable(no-op)→ ensure_mode(校验)。

    与 RS 的差异:HT 无 separate 使能帧——enable 为语义空操作(2026-10-08
    起方案 B:不发任何帧,直接成功),调用仅为对齐统一 enable→command 流程;
    ensure_mode 仅校验 mode<=17,不写 run_mode 寄存器(HT 运行模式由命令帧
    cmd 字节决定)。
    """
    motor.disable()
    time.sleep(0.06)
    motor.enable()  # HT:语义空操作,不发任何帧(清错/停机走 disable)
    motor.ensure_mode(mode, 1000)
    print(f"[switch] -> {mode.name}")


def _stop_and_disable(motor) -> None:
    """段间停机:disable 即 send_stop(0x01 0x00 0x00)。"""
    motor.disable()
    time.sleep(0.08)


def _run_hold(motor, label: str, secs: float, cmd: Callable[[float], None]) -> None:
    """运行 ``secs`` 秒,每周期发一次命令,~100ms 打印一次 ``get_state()``。

    2026-10-24:控制帧不产生状态回帧(真机复证),循环内每周期
    ``request_feedback()`` 主动拉状态(两帧 17 01 + 16 0E,~10ms 周期
    ≈ 100Hz 查询,回帧由后台 poll 线程入缓)——打印读到的是 ≤10ms
    前的缓存快照。掉线时 get_state 冻结旧值,查询失败静默(CallError
    仅总线级错误)。"""
    start = time.monotonic()
    last_print = start
    while time.monotonic() - start < secs:
        t = time.monotonic() - start
        try:
            cmd(t)
        except Exception as exc:  # noqa: BLE001
            print(f"[{label}] 命令失败: {exc}")
            break
        try:
            motor.request_feedback()
        except CallError:
            pass
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
