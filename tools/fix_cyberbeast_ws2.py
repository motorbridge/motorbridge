#!/usr/bin/env python3
"""Fix all match exhaustion errors in ws_gateway by adding CyberBeast arms."""
import re, sys

fixes = {
    # connection.rs
    "integrations/ws_gateway/src/router/handlers/connection.rs": [
        # enable: after Robstride arm (line 234), before None
        (r"(Some\(MotorHandle::Robstride\(m\)\) => m\.enable\(\)\.map_err\(\|e\| e\.to_string\(\)\)\?\s*,\s*\n\s*None)", 
         r"Some(MotorHandle::CyberBeast(m)) => m.enable().map_err(|e| e.to_string())?,\n        \1"),
        # disable: after Robstride arm
        (r"(Some\(MotorHandle::Robstride\(m\)\) => m\.disable\(\)\.map_err\(\|e\| e\.to_string\(\)\)\?\s*,\s*\n\s*None)",
         r"Some(MotorHandle::CyberBeast(m)) => m.disable().map_err(|e| e.to_string())?,\n        \1"),
        # stop_active: after Robstride arm 
        (r"(MotorHandle::Robstride\(mm\) => mm\.set_velocity_target\(0\.0\)\.map_err\(\|e\| e\.to_string\(\)\)\?\s*,\s*\n\s*None)",
         r"MotorHandle::CyberBeast(mm) => mm.send_stop_motor().map_err(|e| e.to_string())?,\n            \1"),
        # vendor detection: after Robstride arm
        (r"(Some\(MotorHandle::Robstride\(_\)\) => Vendor::Robstride\s*,)",
         r"Some(MotorHandle::CyberBeast(_)) => Vendor::CyberBeast,\n        \1"),
        # vendor matching pair: after Robstride pair
        (r"(\| \(Some\(ControllerHandle::Robstride\(_\)\), Some\(MotorHandle::Robstride\(_\)\)\)\s*=> \{\})",
         r"\1\n        | (Some(ControllerHandle::CyberBeast(_)), Some(MotorHandle::CyberBeast(_))) => {}"),
        # enable_all: after Robstride
        (r"(ControllerHandle::Robstride\(ctrl\) => \{\s*\n\s*ctrl\.enable_all)",
         r"ControllerHandle::CyberBeast(ctrl) => {\n                ctrl.enable_all"),
        # shutdown: after Robstride
        (r"(ControllerHandle::Robstride\(ctrl\) => ctrl\.shutdown\(\)\.map_err\(\|e\| e\.to_string\(\)\)\?\s*,)",
         r"ControllerHandle::CyberBeast(ctrl) => ctrl.shutdown().map_err(|e| e.to_string())?,\n            \1"),
    ],
    # control.rs  
    "integrations/ws_gateway/src/router/handlers/control.rs": [],
    # register.rs
    "integrations/ws_gateway/src/router/handlers/register.rs": [],
    # runtime.rs
    "integrations/ws_gateway/src/session/runtime.rs": [],
}

# For simpler approach - just add catch-all CyberBeast arms
# to all match blocks that lack them
files_with_matches = {
    "integrations/ws_gateway/src/router/handlers/connection.rs": [
        # Each entry: (line_before, arm_to_add)
    ],
}

print("Skipping complex pattern-based approach, will use manual edits.")
print("Key files to fix:")
for f in fixes:
    print(f"  {f}")
