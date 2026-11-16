#!/usr/bin/env python3
"""Link-check the focused Catch test executables that ordinary ALL excludes.

Inventory: scripts/focused_targets.txt, one target per line, optionally
``name optional: <reason>`` for targets that exist only in some configurations.
Subcommands: ``check`` (inventory equals CMake-declared set) and ``build``.
"""

from pathlib import Path
import argparse
import os
import re
import shutil
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
INVENTORY = ROOT / "scripts" / "focused_targets.txt"
SUITES = ("fff_print", "libslic3r", "sla_print")
# Pre-existing explicit-only helper, not a focused Catch test executable.
NOT_FOCUSED = frozenset({"slicer_settings_inventory"})

_DECL = re.compile(
    r"\b(?P<kw>endif|if|add_executable)\s*\(\s*(?P<name>[A-Za-z0-9_]+)?\s*(?P<flag>EXCLUDE_FROM_ALL\b)?",
    re.IGNORECASE)


def cmake_targets(root=ROOT):
    """Return {target: is_conditional} for focused EXCLUDE_FROM_ALL executables.

    Matches against whole-file text (comments removed), so declarations split
    across lines are found; if()/endif() depth marks conditional targets.
    """
    found = {}
    for suite in SUITES:
        text = (Path(root) / "tests" / suite / "CMakeLists.txt").read_text(encoding="utf-8")
        text = re.sub(r"#[^\n]*", "", text)
        depth = 0
        for match in _DECL.finditer(text):
            keyword = match.group("kw").lower()
            if keyword == "if":
                depth += 1
            elif keyword == "endif":
                depth -= 1
            elif match.group("flag") and match.group("name") not in NOT_FOCUSED:
                found[match.group("name")] = depth > 0
    return found


def load_inventory(path=INVENTORY):
    """Return {target: optional reason or None}."""
    entries = {}
    for raw in Path(path).read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        name, marker, reason = line.partition(" optional:")
        name = name.strip()
        if name in entries or not re.fullmatch(r"[A-Za-z0-9_]+", name):
            raise ValueError("bad or duplicate inventory entry: " + raw)
        entries[name] = (reason.strip() or "unspecified") if marker else None
    return entries


def check(root=ROOT, inventory=INVENTORY):
    declared, listed = cmake_targets(root), load_inventory(inventory)
    problems = ["in CMake, not inventory: " + n for n in sorted(set(declared) - set(listed))]
    problems += ["in inventory, not CMake: " + n for n in sorted(set(listed) - set(declared))]
    problems += ["conditional in CMake but not marked optional: " + n
                 for n in sorted(declared) if declared[n] and n in listed and not listed[n]]
    return problems


def _run(cmd, timeout):
    # Argument lists only, never a shell; names come from the checked-in inventory.
    # The child leads its own process group so a timeout kills ninja and the
    # compilers too, not just cmake.
    kwargs = {"start_new_session": True} if hasattr(os, "killpg") else {}
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, errors="replace", **kwargs)
    try:
        out, _ = proc.communicate(timeout=timeout)
        return proc.returncode, out
    except subprocess.TimeoutExpired:
        if kwargs:
            os.killpg(proc.pid, signal.SIGKILL)
        else:
            proc.kill()
        out, _ = proc.communicate()
        return 124, (out or "") + "\n[timed out after %ss]" % timeout


def build(build_dir, config, jobs, target_timeout, smoke_timeout, deadline=3300):
    build_dir = Path(build_dir).resolve()
    inventory = load_inventory()
    end = time.monotonic() + deadline
    code, out = _run(["cmake", "--build", str(build_dir), "--config", config,
                      "--target", "help"], 300)
    if code:
        print("cannot list configured targets:\n" + out[-2000:])
        return 2
    configured = set(re.findall(r"^\.\.\. ([A-Za-z0-9_]+)", out, re.M)) | \
        set(re.findall(r"^([A-Za-z0-9_]+):", out, re.M))
    built, skipped, failed, smoked = [], [], [], 0
    for name, reason in inventory.items():
        if name not in configured:
            if reason:
                skipped.append(name)
                print("SKIPPED %s (not configured; optional: %s)" % (name, reason))
            else:
                failed.append(name)
                print("FAILED %s (required target not configured)" % name)
            continue
        start = time.monotonic()
        remaining = end - start
        if remaining <= 0:
            failed.append(name)
            print("FAILED %s (not attempted: deadline)" % name)
            continue
        code, out = _run(["cmake", "--build", str(build_dir), "--config", config,
                          "--target", name, "-j", str(jobs)], min(target_timeout, remaining))
        if code:
            failed.append(name)
            print("FAILED %s (exit %d, %.0fs); last output:\n%s" % (
                name, code, time.monotonic() - start, out[-3000:]))
            continue
        exes = [p for p in (build_dir / "tests").rglob(name) if p.is_file()]
        if not exes:
            failed.append(name)
            print("FAILED %s (built but executable not found under build/tests)" % name)
            continue
        code, _ = _run([str(exes[0]), "--list-tests"], smoke_timeout)
        if code:
            failed.append(name)
            print("FAILED %s (--list-tests exit %d)" % (name, code))
        else:
            built.append(name)
            smoked += 1
            print("OK %s (built, linked, --list-tests exit 0, %.0fs)" % (
                name, time.monotonic() - start))
        # Remove only this target's executable and object directory.
        for exe in exes:
            exe.unlink()
        for objs in (build_dir / "tests").rglob("CMakeFiles/%s.dir" % name):
            shutil.rmtree(objs, ignore_errors=True)
    print("FOCUSED_TARGETS_SUMMARY inventory=%d built=%d skipped_not_configured=%d "
          "failed=%d smoked=%d failed_targets=%s" % (
              len(inventory), len(built), len(skipped), len(failed), smoked,
              ",".join(failed) or "none"))
    return 1 if failed else 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="cmd", required=True)
    sub.add_parser("check")
    b = sub.add_parser("build")
    b.add_argument("--build-dir", default="build")
    b.add_argument("--config", default="Release")
    b.add_argument("--jobs", type=int, default=2)
    b.add_argument("--target-timeout", type=int, default=1500)
    b.add_argument("--smoke-timeout", type=int, default=60)
    b.add_argument("--deadline", type=int, default=3300)
    args = parser.parse_args(argv)
    if args.cmd == "check":
        problems = check()
        print("\n".join(problems) or "focused target inventory matches CMake")
        return 1 if problems else 0
    return build(args.build_dir, args.config, args.jobs, args.target_timeout, args.smoke_timeout,
                 args.deadline)


if __name__ == "__main__":
    sys.exit(main())
