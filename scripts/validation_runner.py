"""Run validation in a leased, budgeted artifact directory.

The command receives VALIDATION_ARTIFACT_DIR for disposable output. Only
explicit --promote paths survive completion, with their own owner and expiry.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

if __package__:
    from . import validation_artifacts as artifacts
    from .slicer_process import run_captured
else:
    import validation_artifacts as artifacts
    from slicer_process import run_captured


def _inside(path: Path, parent: Path) -> bool:
    return path == parent or parent in path.parents


def _option(argv: list[str], name: str) -> str | None:
    found = []
    for index, value in enumerate(argv):
        if value == name:
            if index + 1 == len(argv):
                raise artifacts.ArtifactError(f"missing value for {name}")
            found.append(argv[index + 1])
        if value.startswith(name + "="):
            found.append(value.split("=", 1)[1])
    if len(found) > 1:
        raise artifacts.ArtifactError(f"duplicate protected option {name}")
    return found[0] if found else None


def managed_main(main, *, output_option: str, default_name: str,
                 argv: list[str] | None = None, readonly_actions: tuple[str, ...] = ()) -> int:
    """Guard a producer's executable entrypoint; unit-level APIs stay reusable."""
    arguments = list(sys.argv[1:] if argv is None else argv)
    if any(value in {"--help", "-h"} for value in arguments) or \
            (arguments and arguments[0] in readonly_actions):
        return main(arguments)
    try:
        required = ("VALIDATION_ARTIFACT_ROOT", "VALIDATION_ARTIFACT_RUN_ID",
                    "VALIDATION_ARTIFACT_TOKEN", "VALIDATION_ARTIFACT_DIR",
                    "VALIDATION_SOURCE_SHA")
        if any(not os.environ.get(key) for key in required):
            raise artifacts.ArtifactError("run this writer through scripts/validation_runner.py")
        receipt = artifacts.check(os.environ[required[0]], os.environ[required[1]],
                                  os.environ[required[2]])
        payload = Path(os.environ["VALIDATION_ARTIFACT_DIR"]).resolve(strict=True)
        if payload != Path(receipt["payload_dir"]).resolve(strict=True):
            raise artifacts.ArtifactError("artifact directory does not match its lease")
        root = Path(_option(arguments, "--root") or ".").resolve()
        head = subprocess.check_output(["git", "-C", str(root), "rev-parse", "HEAD"],
                                       text=True).strip()
        if head != os.environ["VALIDATION_SOURCE_SHA"] or head != receipt["source_sha"]:
            raise artifacts.ArtifactError("source checkout differs from the artifact source revision")
        requested = _option(arguments, output_option)
        output = Path(requested) if requested else payload / default_name
        if not output.is_absolute():
            output = payload / output
        output = output.resolve()
        if not _inside(output, payload) or output == payload:
            raise artifacts.ArtifactError("output must be a child of the leased payload directory")
        # Retaining/reusing a build outside this run requires a separately owned
        # cache adapter. These modes must not recreate permanent evidence trees.
        for flag in ("--keep-worktree", "--persistent-worktree", "--reuse-detached-checkout",
                     "--reuse-worktree-from-report", "--seed-worktree"):
            if flag in arguments or any(value.startswith(flag + "=") for value in arguments):
                raise artifacts.ArtifactError(f"{flag} needs a separately managed cache adapter")
        work_root = _option(arguments, "--work-root")
        if work_root and (not Path(work_root).is_absolute() or
                          not _inside(Path(work_root).resolve(), payload)):
            raise artifacts.ArtifactError("work-root must be an absolute path inside the leased payload")
        build = _option(arguments, "--build")
        if build and (not Path(build).is_absolute() or not _inside(Path(build).resolve(), payload)):
            raise artifacts.ArtifactError("build must be an absolute path inside the leased payload")
        build_dir = _option(arguments, "--build-dir")
        if build_dir and (Path(build_dir).is_absolute() or ".." in Path(build_dir).parts):
            raise artifacts.ArtifactError("build-dir must stay relative to the disposable worktree")
        if not requested:
            arguments.extend([output_option, str(output)])
        else:
            # Resolve relative output against the owned payload, not the source.
            index = next(i for i, value in enumerate(arguments)
                         if value == output_option or value.startswith(output_option + "="))
            if arguments[index] == output_option:
                arguments[index + 1] = str(output)
            else:
                arguments[index] = output_option + "=" + str(output)
        return main(arguments)
    except (artifacts.ArtifactError, OSError, subprocess.CalledProcessError) as error:
        print(f"validation artifacts: {error}", file=sys.stderr)
        return 2


def run(args: argparse.Namespace) -> int:
    if not args.command:
        raise artifacts.ArtifactError("a command is required after --")
    command = args.command[1:] if args.command[0] == "--" else args.command
    if not command:
        raise artifacts.ArtifactError("command cannot be empty")
    if args.promote and (args.retain_ttl_seconds is None or args.retain_budget_bytes is None):
        raise artifacts.ArtifactError("promotion requires explicit retention TTL and budget")
    record = artifacts.create(args.root, args.work_item, args.source_sha, args.owner,
                              args.ttl_seconds, args.budget_bytes)
    run_id, token = record["run_id"], record["token"]
    payload = Path(record["payload_dir"])
    outcome, code, promoted = "failed", 2, None
    checked, renewed = 0.0, time.monotonic()

    def guard():
        nonlocal checked, renewed
        now = time.monotonic()
        if now - checked >= 0.5:
            artifacts.check(args.root, run_id, token)
            checked = now
        if now - renewed >= 20:
            artifacts.renew(args.root, run_id, token)
            renewed = now

    def terminate(signum, frame):
        raise KeyboardInterrupt(f"validation interrupted by signal {signum}")

    previous = None
    try:
        environment = dict(os.environ, VALIDATION_ARTIFACT_ROOT=str(Path(args.root).resolve()),
                           VALIDATION_ARTIFACT_RUN_ID=run_id, VALIDATION_ARTIFACT_TOKEN=token,
                           VALIDATION_ARTIFACT_DIR=str(payload), VALIDATION_SOURCE_SHA=args.source_sha)
        # Native tool scratch goes into this run rather than an unowned system temp.
        scratch = payload / "tmp"
        scratch.mkdir()
        environment.update(TMPDIR=str(scratch), TMP=str(scratch), TEMP=str(scratch))
        if threading.current_thread() is threading.main_thread():
            previous = signal.signal(signal.SIGTERM, terminate)
        with (payload / "command.log").open("xb") as log:
            result = run_captured(command, env=environment, timeout=args.ttl_seconds,
                                  output_file=log, check=guard)
        code = result.returncode
        outcome = "completed" if code == 0 else "failed"
        artifacts.check(args.root, run_id, token)
        if args.promote:
            promoted = artifacts.promote(args.root, run_id, token, args.owner,
                                         args.retain_ttl_seconds, args.retain_budget_bytes,
                                         args.promote)
    except KeyboardInterrupt:
        outcome, code = "interrupted", 130
    except (artifacts.ArtifactError, OSError, subprocess.TimeoutExpired) as error:
        print(f"validation artifacts: {error}", file=sys.stderr)
        outcome, code = "failed", 2
    finally:
        if previous is not None:
            signal.signal(signal.SIGTERM, previous)
        # Authentication remains necessary, but an exceeded budget or expired
        # lease must not prevent discarding the run's disposable data.
        artifacts.finish(args.root, run_id, token, outcome)
    print(json.dumps({"work_item": args.work_item, "source_sha": args.source_sha,
                      "run_id": run_id, "outcome": outcome, "returncode": code,
                      "retained": promoted}, sort_keys=True))
    return code


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", required=True, help="initialized managed artifact store")
    parser.add_argument("--work-item", required=True)
    parser.add_argument("--source-sha", required=True)
    parser.add_argument("--owner", required=True)
    parser.add_argument("--ttl-seconds", required=True, type=int)
    parser.add_argument("--budget-bytes", required=True, type=int)
    parser.add_argument("--promote", action="append", default=[], metavar="RELATIVE_PATH")
    parser.add_argument("--retain-ttl-seconds", type=int)
    parser.add_argument("--retain-budget-bytes", type=int)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    try:
        return run(parser.parse_args(argv))
    except (artifacts.ArtifactError, OSError) as error:
        print(f"validation artifacts: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
