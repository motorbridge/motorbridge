#!/usr/bin/env python3
"""Damiao 单电机主要接口全过一遍(Python SDK 真机示例)。

本示例是 robstride_all_interfaces_demo.py 的 DM(达妙)对照版:一个脚本内
按编号阶段把 Python SDK 暴露的 Damiao 主要接口依次过一遍。

跑(默认参数,mcu-serial 串口桥):
    python damiao_all_interfaces_demo.py can0
    # (位置参数 channel 在 mcu-serial 下被忽略,给占位 'can0' 即可;
    #   motor_id / model / feedback_id 用文件开头的运行配置或命令行覆盖)
    # 例: python damiao_all_interfaces_demo.py can0 0x02 4310 \
    #         --feedback-id 0x12 --transport mcu-serial

走 dm-serial(Damiao 专用串口桥):
    python damiao_all_interfaces_demo.py can0 0x01 4310 --transport dm-serial

走 SocketCAN(经典 CAN):
    python damiao_all_interfaces_demo.py can0 0x01 4310 --transport socketcan

走 DM_Device SDK 适配器(适配器须为 USB 模式;DM_DEVICE_TYPE 仅此
后端使用):
    python damiao_all_interfaces_demo.py can0 0x01 4310 \
        --transport dm-device --dm-device-type usb2canfd-dual --dm-channel 0
    # (channel 同样被忽略;仅 Damiao 电机)

导入说明:本示例默认导入**本仓库源码树**里的 motorbridge(examples/ 上级
的 ``src/motorbridge``,需先 ``cargo build -p motor_abi --release`` 生成 ABI
动态库),而非 site-packages 里已安装的版本;设环境变量
``DM_EXAMPLE_USE_INSTALLED=1`` 可改用已安装的发行包。

覆盖接口(Python SDK):
- ``request_feedback`` + ``get_state`` → 设备识别(DM 无 ping 命令;辅以只读
  寄存器 Tpcb / Tmtr 温度与 TMAX 力矩映射范围回读)
- ``clear_error``
- ``set_can_timeout_ms`` + ``get_register_u32``(TIMEOUT,rid=9)回读校验
- ``set_zero_position`` + ``store_parameters``(改写电机配置,**默认关**,见下)
- ``ensure_mode(Mode.MIT / POS_VEL / VEL / FORCE_POS)`` ← 统一模式接口,
  在**使能态**调用(见关键点 5):内部读当前位置作 hold、把四个模式命令
  通道全部清零、写 CTRL_MODE 并回读校验,切换后在新模式下发位置保持
- ``enable`` / ``disable``
- ``get_register_f32``(PMAX,rid=21)/ ``get_register_u32``(CTRL_MODE,rid=10)
- 可选 ``write_register_f32`` / ``write_register_u32``(``--register-write``:
  原值回写 + 回读,不改配置,仅验证写链路)
- ``send_mit``(MIT 模式,正弦摆动)
- ``send_vel``(VEL 模式,正弦速度)
- ``send_pos_vel``(POS_VEL 模式,正弦位置)
- ``send_force_pos``(FORCE_POS 模式,正弦位置 + 力矩混合)
- ``get_state``(状态回读)
- ``get_state_snapshot``(状态快照:状态 + ``feedback_sequence`` +
  ``feedback_age_s``,每个编号阶段读一次;见关键点 4)

关键点:
1. 识别:DM(达妙)协议没有 RobStride 那样的 ping 命令,设备识别走
   ``request_feedback`` 触发的状态帧(can_id / status_code),并用只读寄存器
   61(Tpcb 驱动板温度)/ 62(Tmtr 电机温度)/ 23(TMAX 力矩映射范围)佐证,
   顺带与状态帧的 t_mos / t_rotor / torq 字段交叉对照。
2. 被动状态刷新:后台线程循环 ``ctrl.poll_feedback_once()`` 排空总线;控制环
   内每周期再主动 ``request_feedback()`` 一次。candump 实测(4310 本机固件)
   的回帧规则:**四种模式的命令帧(MIT 0x004 / POS_VEL 0x104 / VEL 0x204 /
   FORCE_POS 0x304)每条都回一条状态帧**(延迟 ≈0.2~0.4ms,与模式无关,
   1201 命令帧 : 1201 状态帧严格 1:1);enable/disable 也回;**0xCC 反馈请求
   帧一条都不回**(1167 条 0 响应)——即状态流由命令帧驱动,
   ``request_feedback`` 在此固件上不触发回帧。回帧 CAN ID 是电机的 MST_ID
   (实测 0x011),Rust 侧 ``accepts_frame`` 以 ``data[0]&0x0F==motor_id``
   兜底匹配,故 feedback_id 配成 0x014 也能收到。寄存器读(带 timeout_ms)在
   Rust 侧内部等待应答,同样依赖反馈被排空处理,否则 ``get_register_*`` 会超时。
3. ``set_zero_position`` 必须在 ``disable`` 状态下调用:enable 态的 set_zero
   会被 core guard 拒绝(见 dm_serial_08 负向测试);set_zero 后需等待再
   enable(见 dm_serial_04/05/07 的时序结论)。故归零放在所有运动之前,
   且 disable → set_zero → 等待 → enable。
4. ``get_state_snapshot()`` 的判读:``feedback_sequence`` 只在收到完整状态帧
   时递增,寄存器/错误回复不推进;``feedback_age_s`` 是距最近一次状态帧的
   主机单调时钟时长。**seq 不变 + age 增长 = 缓存未刷新**(总线可能已停),
   seq 递增才代表读到新反馈。DM 每条控制帧/状态请求帧都带反馈,运动段
   (6~9)期间 seq 快速增长;段间停顿期不发命令,快照 age 变大。
5. 模式切换在**使能态**进行(与 RobStride 相反):Damiao 的
   ``ensure_control_mode``(motor_vendors/damiao/src/motor.rs)内部不做
   disable,而是先读当前位置作 hold 位,把 MIT/POS_VEL/VEL/FORCE_POS 四个
   命令通道全部清零(达妙手册要求切换前位置/速度/力矩前馈/kp/kd 归零),
   再写 CTRL_MODE 并回读校验,最后在新模式下以当前位置发位置保持命令。
   仓库内全部 DM 例程均为 enable(一次)→ ensure_mode(可反复切换),使能态
   连续切换 200 次的稳定性验证见 dm_serial_mode_switch_200_demo.py。
   "使能时模式写入被固件忽略"是 RobStride 固件约束,对 DM 不成立。故本
   示例开头 enable 一次,段间保持使能只做短停顿,结尾统一 disable。
6. ``ensure_mode`` 的回读校验依赖固件支持 CTRL_MODE(rid=10)读回:实测部分
   固件对写返回值匹配的 0x55 ack,但 rid=10 读回恒为 0,于是报
   "control mode verify failed: expected 1, got 0"——写已被 ack,模式多半
   已切换。本示例默认 ``--ensure-strict 0``:校验失败仅警告并继续,靠运动段
   的实际响应判断模式是否生效;``--ensure-strict 1`` 恢复严格中止。
   每次``ensure_mode`` 之后本脚本还会**显式读一次 CTRL_MODE**(rid=10)
   核对模式数值(见 ``_check_ctrl_mode``),打印"一致/不一致"判读,便于
   定位是"固件不回读"还是"模式确实未写入"。

改写电机配置(归零/保存)**默认关闭**:与 RobStride 不同,Damiao 的
``store_parameters`` 会把当前 RAM 配置**整体**固化到闪存(不只零点),反复
运行反复写 flash 风险更高。设环境变量 ``DM_EXAMPLE_WRITE_CONFIG=1`` 或加
``--write-config 1`` 开启:在静止、disable 状态下 set_zero + store,且放在
所有运动与 ensure_mode 之前(store 时 RAM 里只有零点改动,不会顺带固化
运行模式)。
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

# 默认使用本示例所属仓库的 motorbridge 源码(bindings/python/src),
# 而非已安装的发行包。设 DM_EXAMPLE_USE_INSTALLED=1 跳过,改用安装版。
if os.environ.get("DM_EXAMPLE_USE_INSTALLED", "0") != "1":
    _LOCAL_SRC = Path(__file__).resolve().parents[1] / "src"
    if (_LOCAL_SRC / "motorbridge").is_dir() and str(_LOCAL_SRC) not in sys.path:
        sys.path.insert(0, str(_LOCAL_SRC))

from motorbridge import Controller, Mode  # noqa: E402

# ========================= 运行配置(按需修改) =========================
# 传输后端: "mcu-serial" = UART→CAN MCU 桥(经典 CAN); "dm-serial" = Damiao
# 专用串口桥; "dm-device" = DM_Device SDK 适配器; "socketcan" = SocketCAN
TRANSPORT = "mcu-serial"
# SocketCAN 通道(仅 transport=socketcan 使用,其余传输下忽略)
CHANNEL = "can0"
# mcu-serial / dm-serial 串口设备路径与波特率
SERIAL_PORT = "/dev/ttyACM0"
SERIAL_BAUD = 921600
# dm-device 适配器型号(usb2canfd / usb2canfd-dual / linkx4c)与通道(0..3);
# 仅 --transport dm-device 使用,其余传输下不起作用
DM_DEVICE_TYPE = "usb2canfd-dual"
DM_CHANNEL = "0"
# 电机 ID、型号与反馈 ID(MST_ID)
MOTOR_ID = 0x04
MODEL = "4340"
FEEDBACK_ID = 0x14
# ======================================================================

# Damiao 寄存器 ID(motor_vendors/damiao/src/registers.rs)。
REG_ACC = 4         # ACC, f32 RW(可选写链路验证用,原值回写)
REG_TIMEOUT = 9     # TIMEOUT, u32 RW(超时报警时间,ms)
REG_CTRL_MODE = 10  # CTRL_MODE, u32 RW(控制模式)
REG_TMAX = 23       # TMAX, f32 RW(力矩映射范围;实际力矩走状态帧 torq 字段)
REG_PMAX = 21       # PMAX, f32 RW(位置映射范围)
REG_TPCB = 61       # Tpcb, f32 RO(驱动板温度)
REG_TMTR = 62       # Tmtr, f32 RO(电机温度)

HOLD_SECS = 3.0
# 段间模式切换后的停顿(电机在使能态被 ensure_mode 收位到 hold 位置,
# 给它一点时间稳定;参照 dm_serial_mode_switch_200_demo 的 mode-gap 300ms)。
MODE_GAP_S = 0.3
# set_zero 后的等待:立刻 enable/控制有丢使能风险(见 dm_serial_04/05 的
# A/B 结论),这里保守等 0.5s。
SETZERO_SETTLE_S = 0.5


def main() -> None:
    parser = argparse.ArgumentParser(description="Damiao all-interfaces demo (Python SDK)")
    parser.add_argument("channel", nargs="?", default=CHANNEL,
                        help="SocketCAN channel (仅 --transport socketcan 使用;mcu-serial/"
                             "dm-serial/dm-device 下忽略,给占位即可;默认取运行配置 CHANNEL)")
    parser.add_argument("motor_id", nargs="?", type=lambda s: int(s, 0), default=MOTOR_ID)
    parser.add_argument("model", nargs="?", default=MODEL)
    parser.add_argument("--feedback-id", type=lambda s: int(s, 0), default=FEEDBACK_ID)
    parser.add_argument("--transport", default=TRANSPORT,
                        choices=["socketcan", "mcu-serial", "dm-serial", "dm-device"],
                        help="传输后端;mcu-serial=UART→CAN MCU 桥,dm-serial=Damiao "
                             "专用串口桥,dm-device=DM_Device SDK 适配器")
    parser.add_argument("--serial-port", default=SERIAL_PORT,
                        help="mcu-serial / dm-serial 串口设备路径")
    parser.add_argument("--serial-baud", type=int, default=SERIAL_BAUD,
                        help="mcu-serial / dm-serial 串口波特率")
    parser.add_argument("--dm-device-type", default=DM_DEVICE_TYPE,
                        choices=["usb2canfd", "usb2canfd-dual", "linkx4c"],
                        help="dm-device 适配器型号")
    parser.add_argument("--dm-channel", default=DM_CHANNEL,
                        help="dm-device 通道(0..3)")
    parser.add_argument("--hold-secs", type=float, default=HOLD_SECS,
                        help="每个运动段时长(秒)")
    parser.add_argument("--ensure-timeout-ms", type=int, default=1000,
                        help="ensure_mode 回读校验超时(ms)")
    parser.add_argument("--can-timeout-ms", type=int, default=1000,
                        help="写入 DM TIMEOUT 寄存器的总线超时(ms)")
    parser.add_argument("--write-config", type=int, default=None,
                        help="1=set_zero+store_parameters(写闪存);0=跳过;默认取环境变量 "
                             "DM_EXAMPLE_WRITE_CONFIG(缺省 0,即不写)")
    parser.add_argument("--register-write", action="store_true",
                        help="额外验证 write_register_f32/u32 写链路(原值回写,不改配置)")
    parser.add_argument("--ensure-strict", type=int, default=0,
                        help="1=ensure_mode 回读校验失败即中止;默认 0=仅警告并继续"
                             "(模式写已获 0x55 ack,部分固件不回读 CTRL_MODE,见关键点 6)")
    args = parser.parse_args()

    if not 1 <= args.motor_id <= 0x7FF:
        parser.error("motor_id must be in 1..0x7FF")
    if not 0 <= args.feedback_id <= 0x7FF:
        parser.error("feedback_id must be in 0..0x7FF")

    if args.write_config is None:
        write_config = os.environ.get("DM_EXAMPLE_WRITE_CONFIG", "0") == "1"
    else:
        write_config = bool(args.write_config)
    print(
        "== Damiao 主要接口全过一遍(Python SDK)== "
        f"transport={args.transport} "
        f"{_endpoint_desc(args)} "
        f"id=0x{args.motor_id:X} model={args.model} "
        f"fb=0x{args.feedback_id:X} (WRITE_CONFIG={write_config} "
        f"REGISTER_WRITE={args.register_write})"
    )

    if args.transport == "mcu-serial":
        ctrl_ctx = Controller.from_mcu_serial(args.serial_port, args.serial_baud)
    elif args.transport == "dm-serial":
        ctrl_ctx = Controller.from_dm_serial(args.serial_port, args.serial_baud)
    elif args.transport == "dm-device":
        ctrl_ctx = Controller.from_dm_device(args.dm_device_type, args.dm_channel)
    else:
        ctrl_ctx = Controller(args.channel)
    with ctrl_ctx as ctrl:
        motor = ctrl.add_damiao_motor(args.motor_id, args.feedback_id, args.model)

        # 0. 被动反馈刷新线程:排空总线,寄存器读/状态缓存都依赖它(关键点 2)。
        stop = threading.Event()
        poll_thread = threading.Thread(
            target=_poll_loop, name="dm-poll", args=(ctrl, stop), daemon=True
        )
        poll_thread.start()
        time.sleep(0.05)  # 给 poll 线程起步 + 电机反馈一点时间

        try:
            _run_all(motor, write_config, args)
        finally:
            stop.set()
            try:
                motor.disable()
            except Exception:  # noqa: BLE001
                pass
            try:
                ctrl.disable_all()
            except Exception as exc:  # noqa: BLE001
                print(f"[cleanup] disable_all: {exc}")
            motor.close()
    print("== 完成 ==")


def _endpoint_desc(args) -> str:
    if args.transport in ("mcu-serial", "dm-serial"):
        return f"port={args.serial_port} baud={args.serial_baud}"
    if args.transport == "dm-device":
        return f"type={args.dm_device_type} ch={args.dm_channel}"
    return f"channel={args.channel}"


def _poll_loop(ctrl: Controller, stop: threading.Event) -> None:
    while not stop.is_set():
        try:
            ctrl.poll_feedback_once()
        except Exception:  # noqa: BLE001
            # 单次排空失败不应终止刷新线程(总线瞬断、控制器关闭中均可能抛)。
            pass
        time.sleep(0.002)


def _run_all(motor, write_config: bool, args) -> None:
    # 1. 识别:DM 无 ping 命令,用状态帧 + 只读寄存器代替(关键点 1)。
    try:
        motor.request_feedback()
        time.sleep(0.1)
    except Exception as exc:  # noqa: BLE001
        print(f"[1.ident] request_feedback 失败: {exc}(继续)")
    st = motor.get_state()
    if st is None:
        print("[1.ident] (尚无状态反馈;电机未上电/ID 不符?)")
    else:
        print(f"[1.ident] can_id=0x{st.can_id:X} status={st.status_code} {_fmt_state(st)}")
    # 温度/力矩寄存器读:61 Tpcb(驱动板温度)/ 62 Tmtr(电机温度)均为 f32
    # 只读测量寄存器;23 TMAX 为力矩映射范围(f32,MIT 位打包量程)——DM 协议
    # 无"实际力矩"测量寄存器,实时力矩/温度走状态帧(torq / t_mos / t_rotor,
    # 上方 get_state 已打印),此处寄存器读可与状态帧交叉对照,同时佐证设备
    # 在线(任意一次成功的寄存器应答即证明电机响应)。
    try:
        tpcb = motor.get_register_f32(REG_TPCB, 500)
        tmtr = motor.get_register_f32(REG_TMTR, 500)
        tmax = motor.get_register_f32(REG_TMAX, 500)
        print(f"[1.ident] Tpcb={tpcb:.1f}°C Tmtr={tmtr:.1f}°C TMAX={tmax:.3f}(力矩映射范围)")
    except Exception as exc:  # noqa: BLE001
        print(f"[1.ident] 温度/力矩寄存器读取失败: {exc}(继续)")
    _print_snapshot(motor, "1")

    # 2. clear_error:清故障。
    motor.clear_error()
    print("[2.clear_error] ok")
    _print_snapshot(motor, "2")

    # 3. set_can_timeout_ms + 回读校验:DM 的总线超时走 TIMEOUT 寄存器(rid=9),
    #    单位为 50µs——ABI 侧按 ×20 换算(motor_abi/src/motor_register_ffi.rs:
    #    ``timeout_ms.saturating_mul(20)``),故 1000ms 的寄存器值是 20000。
    motor.set_can_timeout_ms(args.can_timeout_ms)
    expected_reg = args.can_timeout_ms * 20
    try:
        back = motor.get_register_u32(REG_TIMEOUT, 500)
        print(f"[3.set_can_timeout_ms] 写 {args.can_timeout_ms}ms(寄存器值 {expected_reg})"
              f" 回读={back}{' ok' if back == expected_reg else ' 不一致!'}")
    except Exception as exc:  # noqa: BLE001
        print(f"[3.set_can_timeout_ms] 回读失败: {exc}(继续)")
    _print_snapshot(motor, "3")

    # 4. 归零 + 落盘(默认关):**刻意放在 enable 之前**,store 时 RAM 里
    #    只有零点改动,不会顺带固化运行模式。必须在 disable 状态执行(核心
    #    guard 会拒绝 enable 态的 set_zero,见 dm_serial_08),set_zero 后等待
    #    SETZERO_SETTLE_S 再继续(见 dm_serial_04/05)。设
    #    DM_EXAMPLE_WRITE_CONFIG=1 或 --write-config 1 开启。
    if write_config:
        try:
            motor.disable()
        except Exception:  # noqa: BLE001
            pass  # 此刻本就未使能,disable 失败不阻塞。
        time.sleep(0.1)
        print("[4.set_zero] 归零中(机械动作,需等待)...")
        motor.set_zero_position()
        time.sleep(SETZERO_SETTLE_S)
        print("[4.store_parameters] 保存到闪存(固化当前 RAM 配置)...")
        motor.store_parameters()
        print("[4] ok")
    else:
        print("[4] 跳过 set_zero/store(DM_EXAMPLE_WRITE_CONFIG 默认 0;设 1 或 "
              "--write-config 1 开启)")
    _print_snapshot(motor, "4")

    # 5. 使能 + 模式选择:DM 的模式切换在使能态进行(关键点 5),与 RobStride
    #    的"disable→写模式→enable"相反。ensure_mode 内部清零命令通道、写
    #    CTRL_MODE、回读校验,并在新模式下以当前位置发位置保持;写入的模式
    #    仅在 RAM,不落盘。
    motor.enable()
    _ensure_mode(motor, Mode.MIT, args.ensure_timeout_ms, bool(args.ensure_strict))
    print(f"[5.enable+ensure_mode] -> MIT  state={_fmt_state(motor.get_state())}")

    # 5b. 参数读:PMAX(f32)。CTRL_MODE 不在此读——每次 ensure_mode 后都会
    #     显式读回核对(见 _check_ctrl_mode),覆盖各模式段。
    try:
        pmax = motor.get_register_f32(REG_PMAX, 500)
        print(f"[5b.get_register] PMAX={pmax:.3f}")
    except Exception as exc:  # noqa: BLE001
        print(f"[5b.get_register] 失败: {exc}(继续)")

    # 5c. 可选写链路验证:原值回写 ACC(f32)与 TIMEOUT(u32),读回比对。
    #     写入的就是刚读出的值,不改变任何配置;开关 --register-write。
    if args.register_write:
        try:
            acc = motor.get_register_f32(REG_ACC, 500)
            motor.write_register_f32(REG_ACC, acc)
            acc_back = motor.get_register_f32(REG_ACC, 500)
            to = motor.get_register_u32(REG_TIMEOUT, 500)
            motor.write_register_u32(REG_TIMEOUT, to)
            to_back = motor.get_register_u32(REG_TIMEOUT, 500)
            print(f"[5c.write_register] ACC 原值回写 verify={acc_back}"
                  f"{' ok' if math.isclose(acc_back, acc, rel_tol=1e-6) else ' 不一致!'}; "
                  f"TIMEOUT 原值回写 verify={to_back}{' ok' if to_back == to else ' 不一致!'}")
        except Exception as exc:  # noqa: BLE001
            print(f"[5c.write_register] 失败: {exc}(继续)")
    _print_snapshot(motor, "5")

    # ---- 模式段:保持使能,段间 ensure_mode(内部清零+收位)+ 短停顿 ----

    # 6. MIT 模式:正弦位置摆动。
    _switch_mode(motor, Mode.MIT, args.ensure_timeout_ms, bool(args.ensure_strict))
    _run_hold(motor, "6.MIT", args.hold_secs,
              lambda t: motor.send_mit(math.sin(2.0 * t), 0.0, 10.0, 0.1, 0.0))
    _print_snapshot(motor, "6")

    # 7. Velocity 模式:正弦速度。
    _switch_mode(motor, Mode.VEL, args.ensure_timeout_ms, bool(args.ensure_strict))
    _run_hold(motor, "7.VEL", args.hold_secs,
              lambda t: motor.send_vel(math.sin(1.0 * t) * 1.5))
    _print_snapshot(motor, "7")

    # 8. Position-VEL 模式:目标位置正弦缓动、速度限 1.0。
    _switch_mode(motor, Mode.POS_VEL, args.ensure_timeout_ms, bool(args.ensure_strict))
    _run_hold(motor, "8.POS_VEL", args.hold_secs,
              lambda t: motor.send_pos_vel(math.sin(0.5 * t), 1.0))
    _print_snapshot(motor, "8")

    # 9. FORCE_POS 模式:位置 + 力矩混合(ratio=力矩前馈占比)。
    _switch_mode(motor, Mode.FORCE_POS, args.ensure_timeout_ms, bool(args.ensure_strict))
    _run_hold(motor, "9.FORCE_POS", args.hold_secs,
              lambda t: motor.send_force_pos(math.sin(0.5 * t), 1.0, 0.3))
    _print_snapshot(motor, "9")
    # 结尾统一 disable(由 main 的 finally 兜底)。


def _ensure_mode(motor, mode: Mode, ensure_timeout_ms: int, strict: bool) -> None:
    """``ensure_mode`` 包装:非严格模式下回读校验失败仅警告(关键点 6)。
    写寄存器已获值匹配的 0x55 ack;校验失败多为固件不回读 CTRL_MODE,
    由后续运动段的实际响应再行判断。"""
    try:
        motor.ensure_mode(mode, ensure_timeout_ms)
    except Exception as exc:  # noqa: BLE001
        if strict:
            raise
        print(f"[ensure_mode] -> {mode.name} 失败: {exc}"
              "(模式写已获 0x55 ack,继续;--ensure-strict 1 可改为中止)")
    _check_ctrl_mode(motor, mode)


def _check_ctrl_mode(motor, mode: Mode) -> None:
    """ensure_mode 之后显式读 CTRL_MODE(rid=10)核对模式。

    DM 的 ControlMode 枚举(MIT=1/POS_VEL=2/VEL=3/FORCE_POS=4)与 Python
    ``Mode`` 数值一致。读回不一致 ≠ 模式未生效:实测部分固件对 rid=10 写有
    0x55 ack 但读回恒 0(见关键点 6),最终以运动段的实际响应为准。"""
    expected = int(mode)
    try:
        back = motor.get_register_u32(REG_CTRL_MODE, 500)
    except Exception as exc:  # noqa: BLE001
        print(f"[ctrl_mode] -> {mode.name} 读回失败: {exc}")
        return
    verdict = "一致" if back == expected else "不一致(固件不回读 rid=10 或未生效)"
    print(f"[ctrl_mode] -> {mode.name} 读回={back} 期望={expected} {verdict}")


def _switch_mode(motor, mode: Mode, ensure_timeout_ms: int, strict: bool) -> None:
    """段间模式切换:保持使能态直接 ``ensure_mode``(DM 的正确姿势,关键点 5)。
    内部会清零全部命令通道、写 CTRL_MODE、回读校验,并以当前位置在新模式下
    发位置保持;随后短停顿让电机稳定在 hold 位再进入运动段。"""
    _ensure_mode(motor, mode, ensure_timeout_ms, strict)
    print(f"[switch] -> {mode.name}")
    time.sleep(MODE_GAP_S)


def _run_hold(motor, label: str, secs: float, cmd: Callable[[float], None]) -> None:
    """运行 ``secs`` 秒,每周期发一次命令 + ``request_feedback()``,
    ~100ms 打印一次 ``get_state()``。"""
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
        except Exception:  # noqa: BLE001
            pass  # 个别请求失败不终止运动段(后台 poll 线程仍在排空)。
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


def _print_snapshot(motor, stage: str) -> None:
    """每个编号阶段读一次 ``get_state_snapshot()``。

    seq 只在完整状态帧时递增(关键点 4):seq 不变 + age 增长 ⇒ 缓存未刷新;
    seq 递增才代表读到新反馈。
    """
    snapshot = motor.get_state_snapshot()
    if snapshot is None:
        print(f"[{stage}.snapshot] (无快照:尚未收到任何状态帧)")
        return
    print(
        f"[{stage}.snapshot] seq={snapshot.feedback_sequence} "
        f"age={snapshot.feedback_age_s:.3f}s "
        + _fmt_state(snapshot.state)
    )


if __name__ == "__main__":
    main()
