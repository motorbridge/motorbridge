from __future__ import annotations

import argparse

from ..core import Controller
from ..models import Mode

def _mode_to_enum(mode: str) -> Mode:
    return {
        "mit": Mode.MIT,
        "pos-vel": Mode.POS_VEL,
        "pos-vel-pp": Mode.POS_VEL,
        "pos-vel-csp": Mode.ROBSTRIDE_POS_VEL_CSP,
        "vel": Mode.VEL,
        "force-pos": Mode.FORCE_POS,
    }[mode]

def _parse_id(text: str) -> int:
    return int(text, 0)

def _parse_rids(text: str) -> list[int]:
    return [int(x.strip(), 0) for x in text.split(",") if x.strip()]

def _add_common_args(p: argparse.ArgumentParser) -> None:
    p.add_argument(
        "--vendor",
        default="damiao",
        choices=["damiao", "myactuator", "robstride", "hightorque", "hexfellow"],
        help="motor vendor/protocol family, default damiao",
    )
    p.add_argument("--channel", default="can0", help="SocketCAN/CAN-FD channel, default can0")
    p.add_argument(
        "--transport",
        default="auto",
        choices=["auto", "socketcan", "socketcanfd", "dm-serial", "dm-device", "mcu-serial"],
        help="transport backend; dm-serial/dm-device are Damiao-only; mcu-serial is a vendor-agnostic UART-to-CAN MCU bridge (classic CAN only)",
    )
    p.add_argument("--serial-port", default="/dev/ttyACM0", help="serial port for dm-serial")
    p.add_argument("--serial-baud", type=int, default=921600, help="baud rate for dm-serial")
    p.add_argument(
        "--dm-device-type",
        default="usb2canfd-dual",
        help="DM_Device SDK adapter type for dm-device, e.g. usb2canfd-dual",
    )
    p.add_argument(
        "--dm-channel",
        default="0",
        help="DM_Device SDK channel number: usb2canfd=0, usb2canfd-dual=0|1, linkx4c=0|1|2|3",
    )
    p.add_argument("--model", default="4340", help="model name/hint, e.g. 4340P or rs-00")
    p.add_argument("--motor-id", default="0x01", help="command/device ID, hex or decimal")
    p.add_argument(
        "--feedback-id",
        default="0x11",
        help="feedback/host ID, hex or decimal; RobStride commonly uses 0xFD",
    )

def _add_run_args(p: argparse.ArgumentParser) -> None:
    p.add_argument(
        "--mode",
        default="mit",
        choices=[
            "enable",
            "disable",
            "clear-error",
            "active-report",
            "mit",
            "pos-vel",
            "pos-vel-pp",
            "pos-vel-csp",
            "vel",
            "force-pos",
            "ping",
            "zero",
            "set-zero",
            "save",
            "zero-by-offset",
            "read-param",
            "write-param",
        ],
        help="operation mode; use scan subcommand for wide bus scans",
    )
    p.add_argument("--loop", type=int, default=100, help="send cycles for run mode")
    p.add_argument("--dt-ms", type=int, default=20, help="period between control frames in ms")
    p.add_argument("--ensure-mode", type=int, default=1, help="mode guard before control, 1/0")
    p.add_argument("--ensure-strict", type=int, default=0, help="fail if mode guard cannot confirm, 1/0")
    p.add_argument("--ensure-timeout-ms", type=int, default=1000, help="mode guard timeout")
    p.add_argument("--print-state", type=int, default=1, help="print returned motor state, 1/0")
    p.add_argument("--pos", type=float, default=0.0, help="target position in radians")
    p.add_argument("--vel", type=float, default=0.0, help="target velocity in rad/s")
    p.add_argument("--kp", type=float, default=30.0, help="MIT kp; RobStride pos-vel fallback for loc_kp")
    p.add_argument("--loc-kp", type=float, default=None, help="RobStride pos-vel native position-loop gain")
    p.add_argument("--kd", type=float, default=1.0, help="MIT kd")
    p.add_argument("--tau", type=float, default=0.0, help="target torque/feed-forward torque")
    p.add_argument("--vlim", type=float, default=1.0, help="velocity limit for pos-vel/force-pos")
    p.add_argument("--acc", type=float, default=10.0, help="RobStride pos-vel-pp acc_set in rad/s^2")
    p.add_argument("--ratio", type=float, default=0.3, help="force-pos interpolation ratio")
    p.add_argument("--zero-exp", type=int, default=0, help="RobStride experimental zero sequence, 1/0")
    p.add_argument(
        "--offset-negate",
        type=int,
        default=0,
        help="accepted for RobStride zero-by-offset; currently no calibration frames are sent",
    )
    p.add_argument("--store", type=int, default=1, help="save/store after supported write/zero operations, 1/0")
    p.add_argument("--param-id", default="0x0", help="parameter/register ID for read-param/write-param")
    p.add_argument("--param-value", default="", help="value for write-param")
    p.add_argument(
        "--param-type",
        choices=["i8", "u8", "u16", "u32", "f32"],
        default="",
        help="parameter type; inferred for common RobStride params when omitted",
    )
    p.add_argument("--timeout-ms", type=int, default=500, help="operation timeout in ms")
    p.add_argument("--active-report", type=int, default=1, help="RobStride active report on/off for active-report mode, 1/0")
    p.add_argument("--set-motor-id", default="", help="change motor/device ID from run command")
    p.add_argument("--set-feedback-id", default="", help="Damiao feedback ID change from run command")
    p.add_argument("--verify-id", type=int, default=1, help="verify ID change, 1/0")
    p.add_argument("--verify-model", type=int, default=1, help="Damiao model verification before control, 1/0")
    p.add_argument("--verify-timeout-ms", type=int, default=500, help="model/ID verification timeout")
    p.add_argument("--verify-tol", type=float, default=0.2, help="Damiao model limit verification tolerance")

def _vendor_defaults(vendor: str, model: str, feedback_id: str) -> tuple[str, str]:
    resolved_model = model
    resolved_feedback = feedback_id
    if vendor == "robstride":
        if resolved_model == "4340":
            resolved_model = "rs-00"
        if resolved_feedback == "0x11":
            resolved_feedback = "0xFD"
    elif vendor == "myactuator":
        if resolved_model == "4340":
            resolved_model = "X8"
        if resolved_feedback == "0x11":
            resolved_feedback = "0x241"
    elif vendor == "hightorque":
        if resolved_model == "4340":
            resolved_model = "hightorque"
        if resolved_feedback == "0x11":
            resolved_feedback = "0x01"
    elif vendor == "hexfellow":
        if resolved_model == "4340":
            resolved_model = "hexfellow"
        if resolved_feedback == "0x11":
            resolved_feedback = "0x00"
    return resolved_model, resolved_feedback

def _add_motor(ctrl: Controller, vendor: str, motor_id: int, feedback_id: int, model: str):
    if vendor == "myactuator":
        return ctrl.add_myactuator_motor(motor_id, feedback_id, model)
    if vendor == "robstride":
        return ctrl.add_robstride_motor(motor_id, feedback_id, model)
    if vendor == "hightorque":
        return ctrl.add_hightorque_motor(motor_id, feedback_id, model)
    if vendor == "hexfellow":
        return ctrl.add_hexfellow_motor(motor_id, feedback_id, model)
    return ctrl.add_damiao_motor(motor_id, feedback_id, model)

def _open_controller(args: argparse.Namespace, vendor: str) -> Controller:
    transport = getattr(args, "transport", "auto")
    if transport == "dm-serial":
        if vendor != "damiao":
            raise ValueError("transport=dm-serial is supported only for --vendor damiao")
        return Controller.from_dm_serial(args.serial_port, int(args.serial_baud))
    if transport == "dm-device":
        if vendor != "damiao":
            raise ValueError("transport=dm-device is supported only for --vendor damiao")
        return Controller.from_dm_device(args.dm_device_type, args.dm_channel)
    if transport == "mcu-serial":
        if vendor == "hexfellow":
            raise ValueError("transport=mcu-serial is classic-CAN only; hexfellow needs CAN-FD")
        return Controller.from_mcu_serial(args.serial_port, int(args.serial_baud))
    if vendor == "hexfellow":
        if transport == "socketcan":
            raise ValueError("vendor=hexfellow requires --transport socketcanfd (or auto)")
        return Controller.from_socketcanfd(args.channel)
    if transport == "socketcanfd":
        return Controller.from_socketcanfd(args.channel)
    return Controller(args.channel)
