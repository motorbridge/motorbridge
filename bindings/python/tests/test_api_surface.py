from __future__ import annotations

import json
from pathlib import Path


def test_api_surface_includes_binding_parity_metadata() -> None:
    root = Path(__file__).resolve().parents[3]
    surface = json.loads((root / "bindings" / "api_surface.json").read_text(encoding="utf-8"))

    assert surface["schema"] == 1
    assert "motor_abi_version" in surface["abi"]["metadata"]
    assert "motor_abi_capabilities_json" in surface["abi"]["metadata"]
    assert "motorbridge.abi_version()" in surface["bindings"]["python"]["module_metadata"]
    assert "motorbridge::abi_version()" in surface["bindings"]["cpp"]["namespace_metadata"]
    assert "Motor.robstride_ping_host_id(host_id, timeout_ms)" in surface["bindings"]["motor_methods"]
    assert "Motor.robstride_get_fault_report()" in surface["bindings"]["motor_methods"]
    assert "motor_handle_robstride_feedback_sequence" in surface["abi"]["robstride"]
    assert "Motor.robstride_feedback_sequence()" in surface["bindings"]["motor_methods"]
