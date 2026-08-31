#!/usr/bin/env python3
"""bump_version.py — keep motorbridge's version strings in sync.

motorbridge stores its package version in several hand-maintained source
files (Rust workspace, Python _version, C++ CMake, api_surface.json, the
two binding READMEs) plus changelog/release-note scaffolds. This script is
the single lever: given a new version it rewrites every mechanical file,
creates the release-test-note stub, and updates the docs "current version"
pointer. `pyproject.toml` is dynamic and needs no edit; the CHANGELOG
section body is left for a human to fill (only the header is scaffolded).

Usage:
  python3 scripts/bump_version.py <new_version>     # perform the bump
  python3 scripts/bump_version.py <new_version> -n  # dry-run (no writes)
  python3 scripts/bump_version.py --check           # report drift, exit 1 if any
  python3 scripts/bump_version.py --check --json    # machine-readable drift report

The new version must match N.N.N (semantic version, e.g. 0.5.3). The
current version is read from Cargo.toml's [workspace.package] block.
"""

from __future__ import annotations

import argparse
import datetime
import json
import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]

# Files where the version appears as a literal token exactly once and can be
# swapped safely. Each entry is (relative_path, short_label).
LITERAL_FILES: list[tuple[str, str]] = [
    ("Cargo.toml", "Rust workspace"),
    ("bindings/python/src/motorbridge/_version.py", "Python _version"),
    ("bindings/cpp/CMakeLists.txt", "C++ CMake project"),
    ("bindings/api_surface.json", "api_surface.json"),
    ("bindings/python/README.md", "Python README"),
    ("bindings/python/README.zh-CN.md", "Python README.zh-CN"),
]

# Docs "current version" pointer lines (zh / en).
DOCS_TESTING = [
    ("docs/zh/testing.md", "当前版本"),
    ("docs/en/testing.md", "Current version"),
]

SEMVER_RE = re.compile(r"^\d+\.\d+\.\d+$")
WORKSPACE_VERSION_RE = re.compile(
    r'(?<=\[workspace\.package\]\n)(version\s*=\s*")([0-9][^"]*)(")'
)


def read_current_version() -> str:
    cargo = (REPO_ROOT / "Cargo.toml").read_text(encoding="utf-8")
    m = WORKSPACE_VERSION_RE.search(cargo)
    if not m:
        raise SystemExit("error: could not find [workspace.package] version in Cargo.toml")
    return m.group(2)


def fail(msg: str) -> None:
    print(f"error: {msg}", file=sys.stderr)
    raise SystemExit(1)


def replace_literal(path: Path, old: str, new: str, dry: bool, label: str) -> str:
    """Replace the single old-version literal in `path`. Fail if the count
    is not exactly 1, so silent drift (a file that already moved, or one that
    accidentally carries the token twice) is caught loudly."""
    text = path.read_text(encoding="utf-8")
    count = text.count(old)
    if count == 0:
        return f"skip  {label}: no occurrence of {old} (already bumped?)"
    if count > 1:
        fail(f"{label}: found {count} occurrences of {old} in {path}; refusing to bulk-replace")
    new_text = text.replace(old, new)
    if not dry:
        path.write_text(new_text, encoding="utf-8")
    return f"ok    {label}: {old} -> {new}"


def update_docs_pointer(rel: str, new: str, dry: bool, label: str) -> str:
    """Rewrite the `release_test_notes/<ver>.md` link in a testing.md line to
    `<new>`. Matches any version in that pointer so a stale pointer is fixed
    regardless of how far behind it was."""
    path = REPO_ROOT / rel
    text = path.read_text(encoding="utf-8")
    pat = re.compile(
        rf"({re.escape(label)}[:：]?\s*\[`release_test_notes/)[0-9]+\.[0-9]+\.[0-9]+(\.md`\]\(../../release_test_notes/)[0-9]+\.[0-9]+\.[0-9]+(\.md\))"
    )
    if not pat.search(text):
        return f"skip  {label}: pointer line not found (already at {new}?)"
    new_text = pat.sub(rf"\g<1>{new}\g<2>{new}\g<3>", text)
    if not dry:
        path.write_text(new_text, encoding="utf-8")
    return f"ok    {label}: pointer -> release_test_notes/{new}.md"


def scaffold_changelog(new: str, dry: bool) -> str:
    """Insert a `## [<new>] - <date>` stub under `## [Unreleased]` if absent.
    Body is left empty for a human to fill — this script never edits existing
    historical entries."""
    path = REPO_ROOT / "CHANGELOG.md"
    text = path.read_text(encoding="utf-8")
    header = f"## [{new}]"
    if re.search(rf"^## \[{re.escape(new)}\]", text, re.M):
        return f"skip  CHANGELOG: section {header} already present"
    date = datetime.date.today().isoformat()
    block = f"## [{new}] - {date}\n\n### Changed\n\n- _TODO: describe changes._\n\n"
    # Insert right after the `## [Unreleased]` header line.
    new_text = re.sub(r"(## \[Unreleased\]\n)", r"\1\n" + block, text, count=1)
    if new_text == text:
        return "skip  CHANGELOG: `## [Unreleased]` anchor not found; left untouched"
    if not dry:
        path.write_text(new_text, encoding="utf-8")
    return f"ok    CHANGELOG: inserted {header} stub"


def scaffold_release_note(new: str, dry: bool) -> str:
    """Create release_test_notes/<new>.md from a minimal template if absent."""
    path = REPO_ROOT / "release_test_notes" / f"{new}.md"
    if path.exists():
        return f"skip  release_test_notes/{new}.md: already exists"
    tmpl = (
        f"# Release Test Notes - {new}\n\n"
        f"Date: {datetime.date.today().isoformat()}\n\n"
        "## Scope\n\n_TODO: describe the scope of this release._\n\n"
        "## Key Changes\n\n- _TODO: list key changes._\n\n"
        "## Required Local Checks\n\n```bash\ncargo fmt --all -- --check\ncargo build -p motor_abi\n```\n\n"
        "## Expected Version Smoke\n\n"
        f"```text\nmotor_cli {new}\nws_gateway {new}\nmotorbridge {new}\nmotor_abi {new}\n```\n"
    )
    if not dry:
        path.write_text(tmpl, encoding="utf-8")
    return f"ok    release_test_notes/{new}.md: created (stub)"


def gather_drift(expected: str) -> list[dict]:
    """Return a list of drift records: files whose literal version != expected."""
    drift = []
    for rel, label in LITERAL_FILES:
        path = REPO_ROOT / rel
        text = path.read_text(encoding="utf-8")
        found = _find_version_token(text, rel)
        if found != expected:
            drift.append({"file": rel, "label": label, "found": found, "expected": expected})
    for rel, label in DOCS_TESTING:
        path = REPO_ROOT / rel
        text = path.read_text(encoding="utf-8")
        m = re.search(r"release_test_notes/([0-9]+\.[0-9]+\.[0-9]+)\.md", text)
        found = m.group(1) if m else None
        if found != expected:
            drift.append({"file": rel, "label": label + " pointer", "found": found, "expected": expected})
    return drift


def _find_version_token(text: str, rel: str) -> str | None:
    """Return just the version string declared in `text`, or None."""
    if rel == "Cargo.toml":
        m = WORKSPACE_VERSION_RE.search(text)
        return m.group(2) if m else None
    if rel == "bindings/python/src/motorbridge/_version.py":
        m = re.search(r'(?m)^VERSION\s*=\s*"([0-9][^"]*)"', text)
        return m.group(1) if m else None
    if rel == "bindings/cpp/CMakeLists.txt":
        m = re.search(r"project\(motorbridge_cpp\s+VERSION\s+([0-9][0-9.]*)", text)
        return m.group(1) if m else None
    if rel == "bindings/api_surface.json":
        m = re.search(r'"version"\s*:\s*"([0-9][^"]*)"', text)
        return m.group(1) if m else None
    if rel.endswith("README.md"):
        m = re.search(r"package target version: `([0-9][0-9.]*)`", text)
        return m.group(1) if m else None
    if rel.endswith("README.zh-CN.md"):
        m = re.search(r"目标包版本：`([0-9][0-9.]*)`", text)
        return m.group(1) if m else None
    return None


def cmd_bump(new: str, dry: bool) -> int:
    old = read_current_version()
    if not SEMVER_RE.match(new):
        fail(f"version must match N.N.N (got '{new}')")
    if old == new:
        print(f"current version is already {new}; nothing to do")
        return 0
    print(f"bumping motorbridge {old} -> {new}" + ("  (dry-run)" if dry else ""))
    for rel, label in LITERAL_FILES:
        print("  " + replace_literal(REPO_ROOT / rel, old, new, dry, label))
    for rel, label in DOCS_TESTING:
        print("  " + update_docs_pointer(rel, new, dry, label))
    print("  " + scaffold_changelog(new, dry))
    print("  " + scaffold_release_note(new, dry))
    print("\nDone. Reminder: fill in the CHANGELOG body and release-test-note scope.")
    print("pyproject.toml is dynamic (attr = motorbridge._version.VERSION) — no edit needed.")
    return 0


def cmd_check(expected: str | None, as_json: bool) -> int:
    exp = expected or read_current_version()
    drift = gather_drift(exp)
    if as_json:
        print(json.dumps({"expected": exp, "drift": drift}, indent=2))
    else:
        print(f"expected version: {exp}")
        if not drift:
            print("all version-bearing files agree.")
        else:
            print("DRIFT:")
            for d in drift:
                print(f"  {d['label']:<22} {d['file']}: found {d['found']} (expected {d['expected']})")
    return 1 if drift else 0


def main(argv: list[str]) -> int:
    p = argparse.ArgumentParser(description="Bump or check motorbridge version sync.")
    p.add_argument("version", nargs="?", help="target version, e.g. 0.5.3")
    p.add_argument("-n", "--dry-run", action="store_true", help="print changes without writing")
    p.add_argument("--check", action="store_true", help="report version drift only")
    p.add_argument("--json", action="store_true", help="machine-readable (with --check)")
    args = p.parse_args(argv)

    if args.check:
        return cmd_check(args.version, args.json)
    if not args.version:
        p.error("version is required (or use --check)")
    return cmd_bump(args.version, args.dry_run)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
