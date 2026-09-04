from __future__ import annotations

import argparse
import sys

from .. import get_version
from ..platform_hints import preflight_can_runtime
from .common import _add_common_args, _add_run_args
from .damiao import _damiao_read_param_command, _damiao_write_param_command, _id_dump_command
from .id_ops import _id_set_command
from .robstride import _robstride_read_param_command, _robstride_write_param_command
from .run import _run_command
from .scan import _scan_command

def _provided_options(argv: list[str]) -> set[str]:
    out: set[str] = set()
    for item in argv:
        if not item.startswith("--"):
            continue
        key = item[2:].split("=", 1)[0]
        if key:
            out.add(key)
    return out

def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="motorbridge Python SDK CLI",
        formatter_class=argparse.RawTextHelpFormatter,
        allow_abbrev=False,
        epilog=(
            "Run-mode argument map:\n"
            "  damiao mit:       --pos --vel --kp --kd --tau\n"
            "  damiao pos-vel:   --pos --vlim\n"
            "  damiao vel:       --vel\n"
            "  damiao force-pos: --pos --vlim --ratio\n"
            "  robstride mit:    --pos --vel --kp --kd --tau\n"
            "  robstride pos-vel: --pos --vlim --loc-kp (or --kp fallback); --vel/--kd/--tau ignored\n"
            "  robstride vel:    --vel\n"
            "  hightorque mit:   --pos --vel --tau; --kp/--kd ignored by ht_can v1.5.5\n"
            "  hexfellow mit:    --pos --vel --kp --kd --tau\n"
            "  hexfellow pos-vel: --pos --vlim\n"
            "\n"
            "Examples:\n"
            "  motorbridge-cli scan --vendor robstride --channel can0 --start-id 1 --end-id 127\n"
            "  motorbridge-cli run --vendor robstride --channel can0 --model rs-00 --motor-id 1 --feedback-id 0xFD --mode enable --loop 1\n"
            "  motorbridge-cli run --vendor robstride --channel can0 --model rs-00 --motor-id 127 --feedback-id 0xFD --mode read-param --param-id 0x7019\n"
            "  motorbridge-cli run --vendor robstride --channel can0 --model rs-00 --motor-id 127 --feedback-id 0xFD --mode pos-vel --pos 0 --vlim 1 --loc-kp 1\n"
            "  motorbridge-cli run --vendor robstride --channel can0 --model rs-00 --motor-id 2 --feedback-id 0xFD --mode clear-error --loop 1\n"
            "  motorbridge-cli run --vendor robstride --channel can0 --model rs-00 --motor-id 2 --feedback-id 0xFD --mode active-report --active-report 1 --loop 1\n"
            "  motorbridge-cli damiao-read-param --channel can0 --model 4340P --motor-id 1 --feedback-id 0x11 --param-id 10 --type f32\n"
        ),
    )
    p.add_argument("-v", "--version", action="version", version=f"motorbridge {get_version()}")
    sub = p.add_subparsers(
        dest="command",
        parser_class=lambda *a, **kw: argparse.ArgumentParser(
            *a, allow_abbrev=False, **kw
        ),
    )

    run = sub.add_parser("run", help="send control commands (default command)")
    _add_common_args(run)
    _add_run_args(run)

    dump = sub.add_parser("id-dump", help="read key ID/mode/timeout registers")
    _add_common_args(dump)
    dump.add_argument("--timeout-ms", type=int, default=500, help="register read timeout in ms")
    dump.add_argument("--rids", default="7,8,9,10,21,22,23", help="comma-separated Damiao register IDs")

    set_id = sub.add_parser(
        "id-set",
        help="change motor ID; Damiao supports ESC_ID/MST_ID, RobStride supports device_id",
    )
    _add_common_args(set_id)
    set_id.add_argument("--new-motor-id", default="", help="new command/device ID, hex or decimal")
    set_id.add_argument(
        "--new-feedback-id",
        default="",
        help="Damiao MST_ID only; RobStride host_id is not changed",
    )
    set_id.add_argument("--store", type=int, default=1, help="store ID change when supported, 1/0")
    set_id.add_argument("--verify", type=int, default=1, help="verify ID change after write, 1/0")
    set_id.add_argument("--timeout-ms", type=int, default=800, help="verification timeout in ms")

    scan = sub.add_parser("scan", help="scan active motor IDs")
    scan.add_argument(
        "--vendor",
        default="damiao",
        choices=["damiao", "myactuator", "robstride", "hightorque", "hexfellow", "all"],
        help="vendor/protocol to scan, or all for combined scan",
    )
    scan.add_argument("--channel", default="can0", help="SocketCAN/CAN-FD channel")
    scan.add_argument(
        "--transport",
        default="auto",
        choices=["auto", "socketcan", "socketcanfd", "dm-serial", "dm-device", "mcu-serial"],
        help="transport backend; dm-serial/dm-device are Damiao-only; mcu-serial is a vendor-agnostic UART-to-CAN MCU bridge",
    )
    scan.add_argument("--serial-port", default="/dev/ttyACM0", help="serial port for dm-serial")
    scan.add_argument("--serial-baud", type=int, default=921600, help="baud rate for dm-serial")
    scan.add_argument(
        "--dm-device-type",
        default="usb2canfd-dual",
        help="DM_Device SDK adapter type for dm-device, e.g. usb2canfd-dual",
    )
    scan.add_argument(
        "--dm-channel",
        default=None,
        help="DM_Device SDK channel number; omitted scans all channels for the selected adapter",
    )
    scan.add_argument("--model", default="4340", help="model hint used by vendor scanner")
    scan.add_argument("--start-id", default="0x01", help="first motor/device ID to probe")
    scan.add_argument("--end-id", default="0x10", help="last motor/device ID to probe")
    scan.add_argument(
        "--feedback-ids",
        default="0xFD,0xFF,0xFE,0x00,0xAA",
        help="RobStride host_id candidates; these are not motor/device IDs",
    )
    scan.add_argument("--feedback-base", default="0x10", help="Damiao feedback ID base")
    scan.add_argument("--timeout-ms", type=int, default=80, help="scan ping/status timeout in ms")
    scan.add_argument(
        "--param-id",
        default="0x7019",
        help="RobStride parameter used as scan fallback",
    )
    scan.add_argument(
        "--param-timeout-ms",
        type=int,
        default=120,
        help="RobStride parameter fallback timeout in ms",
    )

    rs_read = sub.add_parser("robstride-read-param", help="read a RobStride parameter")
    _add_common_args(rs_read)
    rs_read.set_defaults(vendor="robstride")
    rs_read.set_defaults(model="rs-00", feedback_id="0xFD")
    rs_read.add_argument("--param-id", required=True, help="RobStride parameter ID, hex or decimal")
    rs_read.add_argument("--type", required=True, choices=["i8", "u8", "u16", "u32", "f32"], help="parameter value type")
    rs_read.add_argument("--timeout-ms", type=int, default=500, help="read timeout in ms")

    rs_write = sub.add_parser("robstride-write-param", help="write a RobStride parameter")
    _add_common_args(rs_write)
    rs_write.set_defaults(vendor="robstride")
    rs_write.set_defaults(model="rs-00", feedback_id="0xFD")
    rs_write.add_argument("--param-id", required=True, help="RobStride parameter ID, hex or decimal")
    rs_write.add_argument("--type", required=True, choices=["i8", "u8", "u16", "u32", "f32"], help="parameter value type")
    rs_write.add_argument("--value", required=True, help="value to write")
    rs_write.add_argument("--verify", type=int, default=1, help="read back after write, 1/0")
    rs_write.add_argument("--store", type=int, default=0, help="save/store after verified write, 1/0")
    rs_write.add_argument("--timeout-ms", type=int, default=500, help="write/readback timeout in ms")

    dm_read = sub.add_parser("damiao-read-param", help="read a Damiao parameter/register")
    _add_common_args(dm_read)
    dm_read.set_defaults(vendor="damiao")
    dm_read.add_argument("--param-id", required=True, help="Damiao register/parameter ID, hex or decimal")
    dm_read.add_argument("--type", required=True, choices=["u32", "f32"], help="parameter value type")
    dm_read.add_argument("--timeout-ms", type=int, default=500, help="read timeout in ms")

    dm_write = sub.add_parser("damiao-write-param", help="write a Damiao parameter/register")
    _add_common_args(dm_write)
    dm_write.set_defaults(vendor="damiao")
    dm_write.add_argument("--param-id", required=True, help="Damiao register/parameter ID, hex or decimal")
    dm_write.add_argument("--type", required=True, choices=["u32", "f32"], help="parameter value type")
    dm_write.add_argument("--value", required=True, help="value to write")
    dm_write.add_argument("--verify", type=int, default=1, help="read back after write, 1/0")
    dm_write.add_argument("--store", type=int, default=0, help="save/store after verified write, 1/0")
    dm_write.add_argument("--timeout-ms", type=int, default=500, help="write/readback timeout in ms")

    return p

def _parse_with_legacy_support() -> argparse.Namespace:
    parser = _build_parser()
    provided = _provided_options(sys.argv[1:])
    if len(sys.argv) == 1:
        parser.print_help()
        raise SystemExit(0)
    if len(sys.argv) > 1 and sys.argv[1].startswith("--") and sys.argv[1] not in ("-h", "--help"):
        legacy = argparse.ArgumentParser(description="motorbridge Python SDK CLI (legacy run mode)", allow_abbrev=False)
        legacy.add_argument("-v", "--version", action="version", version=f"motorbridge {get_version()}")
        _add_common_args(legacy)
        _add_run_args(legacy)
        legacy_args = legacy.parse_args()
        legacy_args.command = "run"
        legacy_args._provided_options = provided
        return legacy_args

    args, extras = parser.parse_known_args()
    if args.command is not None:
        if extras:
            parser.error(f"unrecognized arguments: {' '.join(extras)}")
        args._provided_options = provided
        return args

    legacy = argparse.ArgumentParser(description="motorbridge Python SDK CLI (legacy run mode)", allow_abbrev=False)
    legacy.add_argument("-v", "--version", action="version", version=f"motorbridge {get_version()}")
    _add_common_args(legacy)
    _add_run_args(legacy)
    legacy_args = legacy.parse_args()
    legacy_args.command = "run"
    legacy_args._provided_options = provided
    return legacy_args

def main() -> None:
    args = _parse_with_legacy_support()
    try:
        transport = str(getattr(args, "transport", "auto") or "auto")
        vendor = str(getattr(args, "vendor", "damiao") or "damiao")
        provided = getattr(args, "_provided_options", set())
        if transport == "auto" and vendor == "damiao" and "serial-port" in provided:
            args.transport = "dm-serial"
            transport = "dm-serial"
        channel = str(getattr(args, "channel", "can0") or "can0")
        hint = preflight_can_runtime("motorbridge-cli", transport, channel)
        if hint:
            raise RuntimeError(hint)

        if args.command == "run":
            _run_command(args)
        elif args.command == "id-dump":
            _id_dump_command(args)
        elif args.command == "id-set":
            _id_set_command(args)
        elif args.command == "scan":
            _scan_command(args)
        elif args.command == "robstride-read-param":
            _robstride_read_param_command(args)
        elif args.command == "robstride-write-param":
            _robstride_write_param_command(args)
        elif args.command == "damiao-read-param":
            _damiao_read_param_command(args)
        elif args.command == "damiao-write-param":
            _damiao_write_param_command(args)
        else:
            raise RuntimeError(f"unknown command: {args.command}")
    except Exception as e:
        print(f"[motorbridge-cli] {e}", file=sys.stderr)
        raise SystemExit(2)


if __name__ == "__main__":
    main()
