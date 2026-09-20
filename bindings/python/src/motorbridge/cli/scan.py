from __future__ import annotations

import argparse
import time
from copy import copy

from ..core import Controller
from .common import _open_controller, _parse_id, _parse_rids, _vendor_defaults
from .robstride import _robstride_host_id


def _bus_hint(args: argparse.Namespace) -> str:
    """Bus locator for scan headers: serial_port for UART links, channel for CAN."""
    t = getattr(args, "transport", "auto")
    if t in ("mcu-serial", "dm-serial"):
        return f"serial_port={args.serial_port} serial_baud={args.serial_baud}"
    return f"channel={args.channel}"


def _dm_device_scan_channels(args: argparse.Namespace) -> list[str]:
    explicit = getattr(args, "dm_channel", None)
    if explicit:
        return [explicit]
    device_type = getattr(args, "dm_device_type", "usb2canfd-dual").lower().replace("_", "-")
    if device_type == "usb2canfd":
        return ["0"]
    if device_type == "linkx4c":
        return ["0", "1", "2", "3"]
    return ["0", "1"]


def _scan_damiao_dm_device_channel(
    args: argparse.Namespace,
    start_id: int,
    end_id: int,
    dm_channel: str,
) -> list[tuple[int, str]]:
    ch_args = copy(args)
    ch_args.dm_channel = dm_channel
    found: list[tuple[int, str]] = []
    print(
        f"[scan:damiao] transport=dm-device dm_channel={dm_channel} model={args.model} "
        f"id_range=[0x{start_id:X},0x{end_id:X}] timeout_ms={args.timeout_ms}"
    )
    ctrl = _open_controller(ch_args, "damiao")
    try:
        for mid in range(start_id, end_id + 1):
            fid = mid + 0x10
            motor = ctrl.add_damiao_motor(mid, fid, args.model)
            try:
                try:
                    motor.request_feedback()
                    hit = None
                    for _ in range(20):
                        ctrl.poll_feedback_once()
                        st = motor.get_state()
                        if st is not None:
                            hit = st
                            break
                        time.sleep(0.008)
                    if hit is None:
                        raise RuntimeError("no feedback")
                    meta = (
                        f"vendor=damiao dm_channel={dm_channel} feedback_id=0x{fid:X} "
                        f"detected_by=dm-device-feedback status={hit.status_code} "
                        f"pos={hit.pos:+.3f}rad vel={hit.vel:+.3f}rad/s "
                        f"torq={hit.torq:+.3f}Nm"
                    )
                    found.append((mid, meta))
                    print(f"[hit] vendor=damiao probe=0x{mid:02X} {meta}")
                except Exception:
                    print(f"[.. ] vendor=damiao dm_channel={dm_channel} probe=0x{mid:02X} no reply")
            finally:
                motor.close()
                time.sleep(0.002)
        for _ in range(10):
            ctrl.poll_feedback_once()
            time.sleep(0.01)
    finally:
        ctrl.close_bus()
        ctrl.close()
    return found


def _scan_damiao(args: argparse.Namespace, start_id: int, end_id: int) -> list[tuple[int, str]]:
    feedback_base = _parse_id(args.feedback_base)
    found: list[tuple[int, str]] = []
    print(
        f"[scan:damiao] {_bus_hint(args)} model={args.model} "
        f"id_range=[0x{start_id:X},0x{end_id:X}] timeout_ms={args.timeout_ms}"
    )
    if getattr(args, "transport", "auto") == "dm-device":
        channels = _dm_device_scan_channels(args)
        print(f"[scan:damiao] dm-device channel target={getattr(args, 'dm_channel', None) or 'all'}")
        for dm_channel in channels:
            found.extend(_scan_damiao_dm_device_channel(args, start_id, end_id, dm_channel))
        return found

    ctrl = _open_controller(args, "damiao")
    try:
        for mid in range(start_id, end_id + 1):
            fid = feedback_base + (mid & 0x0F)
            motor = ctrl.add_damiao_motor(mid, fid, args.model)
            try:
                esc_id = motor.get_register_u32(8, args.timeout_ms)
                mst_id = motor.get_register_u32(7, args.timeout_ms)
                found.append((mid, f"vendor=damiao esc_id=0x{esc_id:X} mst_id=0x{mst_id:X}"))
                print(f"[hit] vendor=damiao probe=0x{mid:02X} esc_id=0x{esc_id:X} mst_id=0x{mst_id:X}")
            except Exception:
                print(f"[.. ] vendor=damiao probe=0x{mid:02X} no reply")
            finally:
                motor.close()
    finally:
        ctrl.close_bus()
        ctrl.close()
    return found

def _scan_robstride(args: argparse.Namespace, start_id: int, end_id: int) -> list[tuple[int, str]]:
    feedback_ids = _parse_rids(args.feedback_ids)
    if not 1 <= start_id <= 255 or not 1 <= end_id <= 255:
        raise ValueError("RobStride scan range must be within 1..255")
    for fid in feedback_ids:
        _robstride_host_id(fid, "feedback-ids")
    param_id = _parse_id(args.param_id)
    found_by_mid: dict[int, str] = {}
    print(
        f"[scan:robstride] {_bus_hint(args)} model={args.model} "
        f"id_range=[0x{start_id:X},0x{end_id:X}] timeout_ms={args.timeout_ms} "
        f"feedback_ids={','.join(f'0x{x:X}' for x in feedback_ids)} param_id=0x{param_id:X}"
    )
    print(
        "[scan:robstride] note: probe/device_id is the motor ID; "
        "feedback_id/host_id is the host-side ID"
    )
    for fid in feedback_ids:
        ctrl = _open_controller(args, "robstride")
        bound = False
        try:
            for mid in range(start_id, end_id + 1):
                if mid in found_by_mid:
                    continue
                motor = ctrl.add_robstride_motor(mid, fid, args.model)
                bound = True
                try:
                    hit_meta = None
                    try:
                        device_id, responder_id = motor.robstride_ping_host_id(fid, args.timeout_ms)
                        hit_meta = (
                            f"vendor=robstride via=ping feedback_id=0x{fid:X} "
                            f"device_id={device_id} responder_id={responder_id}"
                        )
                    except Exception:
                        try:
                            value = motor.robstride_get_param_f32_host_id(
                                param_id, fid, args.param_timeout_ms
                            )
                            hit_meta = (
                                f"vendor=robstride via=read-param feedback_id=0x{fid:X} "
                                f"param_id=0x{param_id:X} value={value}"
                            )
                        except Exception:
                            # Keep probing next feedback candidate / next ID on timeout/no-response.
                            pass
                    if hit_meta is not None:
                        found_by_mid[mid] = hit_meta
                        print(f"[hit] probe=0x{mid:02X} {hit_meta}")
                    elif len(feedback_ids) == 1:
                        print(f"[.. ] vendor=robstride probe=0x{mid:02X} no reply")
                finally:
                    motor.close()
        finally:
            if bound:
                ctrl.close_bus()
            ctrl.close()
    found = [
        (mid, found_by_mid[mid])
        for mid in range(start_id, end_id + 1)
        if mid in found_by_mid
    ]
    if len(feedback_ids) != 1:
        for mid in range(start_id, end_id + 1):
            if mid not in found_by_mid:
                print(f"[.. ] vendor=robstride probe=0x{mid:02X} no reply")
    return found

def _scan_myactuator(args: argparse.Namespace, start_id: int, end_id: int) -> list[tuple[int, str]]:
    found: list[tuple[int, str]] = []
    lo = max(1, start_id)
    hi = min(32, end_id)
    print(
        f"[scan:myactuator] {_bus_hint(args)} model={args.model} "
        f"id_range=[0x{lo:X},0x{hi:X}] timeout_ms={args.timeout_ms}"
    )
    ctrl = _open_controller(args, "myactuator")
    try:
        for mid in range(lo, hi + 1):
            fid = 0x240 + mid
            motor = ctrl.add_myactuator_motor(mid, fid, args.model)
            try:
                try:
                    motor.request_feedback()
                    time.sleep(min(max(args.timeout_ms, 10), 300) / 1000.0)
                    ctrl.poll_feedback_once()
                    st = motor.get_state()
                    if st is None:
                        raise RuntimeError("no feedback")
                    meta = (
                        f"vendor=myactuator feedback_id=0x{fid:X} "
                        f"temp={st.t_mos:.1f}C vel={st.vel:+.3f}rad/s angle={st.pos:+.3f}rad"
                    )
                    found.append((mid, meta))
                    print(f"[hit] probe=0x{mid:02X} {meta}")
                except Exception:
                    print(f"[.. ] vendor=myactuator probe=0x{mid:02X} no reply")
            finally:
                motor.close()
    finally:
        ctrl.close_bus()
        ctrl.close()
    return found

def _scan_hightorque(args: argparse.Namespace, start_id: int, end_id: int) -> list[tuple[int, str]]:
    found: list[tuple[int, str]] = []
    lo = max(1, start_id)
    hi = min(127, end_id)
    print(
        f"[scan:hightorque] {_bus_hint(args)} model={args.model} "
        f"id_range=[0x{lo:X},0x{hi:X}] timeout_ms={args.timeout_ms}"
    )
    ctrl = _open_controller(args, "hightorque")
    try:
        for mid in range(lo, hi + 1):
            motor = ctrl.add_hightorque_motor(mid, 0x01, args.model)
            try:
                try:
                    motor.request_feedback()
                    st = motor.get_state()
                    if st is None:
                        raise RuntimeError("no feedback")
                    meta = (
                        f"vendor=hightorque pos={st.pos:+.4f}rad "
                        f"vel={st.vel:+.4f}rad/s torq={st.torq:+.3f}Nm"
                    )
                    found.append((mid, meta))
                    print(f"[hit] probe=0x{mid:02X} {meta}")
                except Exception:
                    print(f"[.. ] vendor=hightorque probe=0x{mid:02X} no reply")
            finally:
                motor.close()
    finally:
        ctrl.close_bus()
        ctrl.close()
    return found

def _scan_command(args: argparse.Namespace) -> None:
    start_id = _parse_id(args.start_id)
    end_id = _parse_id(args.end_id)
    if end_id < start_id:
        raise ValueError("end-id must be >= start-id")
    if args.transport in ("dm-serial", "dm-device") and args.vendor != "damiao":
        raise ValueError(f"scan with transport={args.transport} currently supports --vendor damiao only")
    resolved_model, _ = _vendor_defaults(args.vendor if args.vendor != "all" else "damiao", args.model, "0x11")
    args.model = resolved_model
    print(
        f"command=scan vendor={args.vendor} transport={args.transport} {_bus_hint(args)} model={args.model} "
        f"id_range=[0x{start_id:X},0x{end_id:X}] timeout_ms={args.timeout_ms}"
    )

    found: list[tuple[int, str]] = []
    damiao_model = args.model
    myactuator_model = "X8" if args.model == "4340" else args.model
    robstride_model = "rs-00" if args.model == "4340" else args.model
    hightorque_model = "hightorque" if args.model == "4340" else args.model
    if args.vendor in ("damiao", "all"):
        args.model = damiao_model
        found.extend(_scan_damiao(args, start_id, end_id))
    if args.vendor in ("myactuator", "all"):
        args.model = myactuator_model
        found.extend(_scan_myactuator(args, start_id, end_id))
    if args.vendor in ("robstride", "all"):
        args.model = robstride_model
        found.extend(_scan_robstride(args, start_id, end_id))
    if args.vendor in ("hightorque", "all"):
        args.model = hightorque_model
        found.extend(_scan_hightorque(args, start_id, end_id))

    print(f"scan done: {len(found)} motor(s) found")
    for probe, meta in found:
        print(f"  probe=0x{probe:02X} {meta}")
