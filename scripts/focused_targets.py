#!/usr/bin/env python3
"""Link-check the focused Catch test executables that ordinary ALL excludes.

Inventory: scripts/focused_targets.txt, one target per line, optionally
``name optional: <CMAKE_VAR>`` for targets declared only while that variable is on.
Subcommands: ``check`` (inventory equals CMake-declared set) and ``build``.
"""

from pathlib import Path
import argparse
import re
import shutil
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))
import slicer_process  # noqa: E402

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
        reason = reason.strip()
        if marker and not re.fullmatch(r"[A-Z0-9_]+", reason):
            raise ValueError("optional reason must name a CMake variable: " + raw)
        entries[name] = reason if marker else None
    return entries


def check(root=ROOT, inventory=INVENTORY):
    declared, listed = cmake_targets(root), load_inventory(inventory)
    problems = ["in CMake, not inventory: " + n for n in sorted(set(declared) - set(listed))]
    problems += ["in inventory, not CMake: " + n for n in sorted(set(listed) - set(declared))]
    problems += ["conditional in CMake but not marked optional: " + n
                 for n in sorted(declared) if declared[n] and n in listed and not listed[n]]
    problems += ["marked optional but unconditional in CMake: " + n
                 for n in sorted(listed) if listed[n] and n in declared and not declared[n]]
    return problems


_FALSY = frozenset({"", "OFF", "0", "FALSE", "NO", "N", "IGNORE"})
TAIL_BYTES = 3000


def _run(cmd, timeout, tail=TAIL_BYTES):
    """Run cmd in its own process group; return (exit code, output tail).

    Timeouts map to 124 after the whole group is killed. ``tail=None`` returns
    the full output. Names come from the checked-in inventory and are never
    passed through a shell.
    """
    with tempfile.TemporaryDirectory(prefix="focused-targets-") as scratch:
        log = Path(scratch) / "output.log"
        with open(log, "wb") as handle:
            try:
                code = slicer_process.run_captured(
                    cmd, timeout=timeout, output_file=handle).returncode
            except subprocess.TimeoutExpired:
                code = 124
        with open(log, "rb") as handle:
            if tail is not None:
                handle.seek(max(0, log.stat().st_size - tail))
            text = handle.read().decode("utf-8", errors="replace")
    return code, text + ("\n[timed out after %ss]" % timeout if code == 124 else "")


def cache_value(build_dir, variable):
    """Return the CMakeCache.txt value of ``variable`` or None when unknown."""
    try:
        text = (Path(build_dir) / "CMakeCache.txt").read_text(encoding="utf-8", errors="replace")
    except OSError:
        return None
    match = re.search(r"^%s:\w+=(.*)$" % re.escape(variable), text, re.M)
    return match.group(1).strip() if match else None


def _is_off(value):
    return value.upper() in _FALSY or value.upper().endswith("-NOTFOUND")


def _inside(path, parent):
    try:
        path.resolve().relative_to(parent)
        return True
    except (OSError, ValueError):
        return False


def build(build_dir, config, jobs, target_timeout, smoke_timeout, deadline=3300):
    build_dir = Path(build_dir).resolve()
    inventory = load_inventory()
    end = time.monotonic() + deadline
    try:
        tests = (build_dir / "tests").resolve(strict=True)
        tests.relative_to(build_dir)
    except (OSError, ValueError):
        print("build/tests is missing or outside the build directory")
        return 2
    code, out = _run(["cmake", "--build", str(build_dir), "--config", config,
                      "--target", "help"], 300, tail=None)
    if code:
        print("cannot list configured targets:\n" + out[-2000:])
        return 2
    configured = set(re.findall(r"^\.\.\. ([A-Za-z0-9_]+)", out, re.M)) | \
        set(re.findall(r"^([A-Za-z0-9_]+):", out, re.M))
    built, skipped, failed, smoked = [], [], [], 0

    def fail(name, why):
        failed.append(name)
        print("FAILED %s (%s)" % (name, why))

    for name, reason in inventory.items():
        if name not in configured:
            value = cache_value(build_dir, reason) if reason else None
            if reason and value is not None and _is_off(value):
                skipped.append(name)
                print("SKIPPED %s (not configured; %s=%s)" % (name, reason, value))
            elif reason:
                fail(name, "optional condition ON but target absent (%s=%s)" % (reason, value))
            else:
                fail(name, "required target not configured")
            continue
        objects = tests.rglob("CMakeFiles/%s.dir" % name)
        if any(True for _ in tests.rglob(name)) or any(True for _ in objects):
            fail(name, "pre-existing output; nothing deleted")
            continue
        start = time.monotonic()
        remaining = end - start
        if remaining <= 0:
            fail(name, "not attempted: deadline")
            continue
        code, out = _run(["cmake", "--build", str(build_dir), "--config", config,
                          "--target", name, "-j", str(jobs)], min(target_timeout, remaining))
        if code:
            failed.append(name)
            print("FAILED %s (exit %d, %.0fs); last output:\n%s" % (
                name, code, time.monotonic() - start, out))
            continue
        exes = list(tests.rglob(name))
        if len(exes) != 1 or exes[0].is_symlink() or not exes[0].is_file() \
                or not _inside(exes[0], tests):
            fail(name, "expected exactly one regular executable under build/tests, found %d;"
                       " nothing deleted" % len(exes))
            continue
        exe = exes[0]
        remaining = end - time.monotonic()
        if remaining <= 0:
            fail(name, "deadline before --list-tests")
        else:
            code, _ = _run([str(exe), "--list-tests"], min(smoke_timeout, remaining))
            if code:
                fail(name, "--list-tests exit %d" % code)
            else:
                built.append(name)
                smoked += 1
                print("OK %s (built, linked, --list-tests exit 0, %.0fs)" % (
                    name, time.monotonic() - start))
        # Remove only the outputs of this focused target, verified absent before the build.
        exe.unlink()
        for objs in tests.rglob("CMakeFiles/%s.dir" % name):
            if not objs.is_symlink() and _inside(objs, tests):
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
