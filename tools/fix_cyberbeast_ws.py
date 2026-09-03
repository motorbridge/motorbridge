#!/usr/bin/env python3
"""Add CyberBeast catch-all arms to all match exhaustion points in ws_gateway."""
import re, subprocess, sys

ws_root = "integrations/ws_gateway/src"

# Run cargo check to find all error locations
r = subprocess.run(["cargo", "check", "-p", "ws_gateway"], capture_output=True, text=True, cwd=".")
files_with_errors = set()
for line in r.stderr.splitlines():
    m = re.search(r'integrations/ws_gateway/src/(\S+\.rs):(\d+)', line)
    if m and "non-exhaustive" in r.stderr:
        files_with_errors.add(ws_root + "/" + m.group(1))

print(f"Files with match exhaustion errors: {len(files_with_errors)}")
for f in sorted(files_with_errors):
    print(f"  {f}")
