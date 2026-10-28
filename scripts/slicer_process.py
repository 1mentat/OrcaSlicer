"""Captured native commands whose timeouts and interruptions stop their child processes."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
from typing import IO, Sequence


def run_captured(argv: Sequence[str], *, cwd: Path | str | None = None,
                 env: dict[str, str] | None = None, timeout: float | None = None,
                 input_data: str | bytes | None = None,
                 output_file: IO | None = None) -> subprocess.CompletedProcess:
    """Run a build or test in its own process group, preserving partial output.

    Killing only CMake, Ninja, or CTest leaves compiler/test children running
    against a worktree the caller may be about to restore or remove.
    With ``output_file``, both streams go directly to that file without being
    buffered in Python; the returned stdout/stderr are then None.
    """
    text_mode = not isinstance(input_data, bytes)
    group_options = ({"start_new_session": True} if os.name == "posix" else
                     {"creationflags": subprocess.CREATE_NEW_PROCESS_GROUP})
    with subprocess.Popen(
        list(argv), cwd=cwd, env=env, shell=False,
        stdin=subprocess.PIPE if input_data is not None else None,
        stdout=output_file if output_file is not None else subprocess.PIPE,
        stderr=subprocess.STDOUT if output_file is not None else subprocess.PIPE,
        text=text_mode, encoding="utf-8" if text_mode else None,
        errors="replace" if text_mode else None, **group_options,
    ) as process:
        try:
            stdout, stderr = process.communicate(input_data, timeout=timeout)
        except subprocess.TimeoutExpired:
            if os.name == "posix":
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            else:
                # taskkill is supplied by Windows and terminates the tree (/T).
                try:
                    subprocess.run(["taskkill", "/PID", str(process.pid), "/T", "/F"],
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                   timeout=10, check=False)
                finally:
                    if process.poll() is None:
                        process.kill()
            stdout, stderr = process.communicate()
            error = subprocess.TimeoutExpired(list(argv), timeout, output=stdout, stderr=stderr)
            # Keep the actual post-cleanup status available to recorded callers.
            # TimeoutExpired remains the raised type and keeps its usual output.
            error.returncode = process.returncode
            raise error from None
        except BaseException:
            # A caller can be interrupted while waiting for a compiler or test
            # process (KeyboardInterrupt, cancellation, or a test-framework
            # exception).  The worktree may be reset/removed immediately after
            # this function returns, so clean the same process group used for
            # timeout handling before re-raising the original exception.
            if os.name == "posix":
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            else:
                try:
                    subprocess.run(["taskkill", "/PID", str(process.pid), "/T", "/F"],
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                   timeout=10, check=False)
                except (OSError, subprocess.TimeoutExpired):
                    pass
                if process.poll() is None:
                    process.kill()
            try:
                process.communicate()
            except BaseException:
                # Preserve the caller's interruption rather than replacing it
                # with a cleanup error.
                pass
            raise
        return subprocess.CompletedProcess(list(argv), process.returncode, stdout, stderr)


def _write_json_receipt(path: Path, receipt: dict[str, object]) -> None:
    """Atomically publish one command receipt without replacing prior evidence."""
    if path.exists() or path.is_symlink():
        raise FileExistsError(f"refusing to replace command receipt: {path}")
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent,
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(receipt, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        # Linking publishes the complete file atomically and fails if another
        # caller already owns this receipt path.
        os.link(temporary, path)
        temporary.unlink()
    except BaseException:
        try:
            os.close(descriptor)
        except OSError:
            pass
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass
        raise


def _record_without_masking(path: Path, receipt: dict[str, object], original: BaseException) -> None:
    try:
        _write_json_receipt(path, receipt)
    except BaseException as receipt_error:
        add_note = getattr(original, "add_note", None)
        if callable(add_note):
            add_note(f"Could not persist command receipt: {receipt_error!r}")


def run_recorded(argv: Sequence[str], *, cwd: Path | str, env: dict[str, str],
                 timeout: float, log_path: Path | str,
                 receipt_path: Path | str) -> subprocess.CompletedProcess:
    """Run one explicit command and atomically save its process outcome first.

    The combined log streams directly to ``log_path``. The receipt is written
    before returning, including for nonzero exits; it records process outcome
    only and does not decide whether a test or later assessment passed. If the
    process launch fails, times out, or is interrupted, a receipt is attempted
    and the original exception is re-raised.
    """
    command = list(argv)
    if not command or not all(isinstance(argument, str) for argument in command):
        raise ValueError("argv must be a nonempty sequence of strings")
    if not isinstance(env, dict) or not all(
            isinstance(key, str) and isinstance(value, str) for key, value in env.items()):
        raise TypeError("env must be an explicit string-to-string mapping")
    if isinstance(timeout, bool) or not isinstance(timeout, (int, float)) or \
            not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("timeout must be a finite positive number")

    working_directory = Path(cwd)
    log_file_path = Path(log_path).absolute()
    receipt_file_path = Path(receipt_path).absolute()
    if log_file_path == receipt_file_path:
        raise ValueError("log_path and receipt_path must be different files")
    if not log_file_path.parent.is_dir() or not receipt_file_path.parent.is_dir():
        raise FileNotFoundError("log and receipt parent directories must already exist")
    if log_file_path.exists() or log_file_path.is_symlink():
        raise FileExistsError(f"refusing to replace command log: {log_file_path}")
    if receipt_file_path.exists() or receipt_file_path.is_symlink():
        raise FileExistsError(f"refusing to replace command receipt: {receipt_file_path}")

    started = time.monotonic()

    def make_receipt(*, outcome: str, returncode: int | None, timed_out: bool,
                     error: BaseException | None = None, stage: str | None = None) -> dict[str, object]:
        result: dict[str, object] = {
            "schema": "slicer-recorded-command-v1",
            "argv": command,
            "cwd": str(working_directory),
            "timeout_seconds": timeout,
            "elapsed_seconds": round(time.monotonic() - started, 3),
            "log_path": str(log_file_path),
            "receipt_path": str(receipt_file_path),
            "process_outcome": outcome,
            "returncode": returncode,
            "timed_out": timed_out,
        }
        if stage is not None:
            result["stage"] = stage
        if error is not None:
            result["exception"] = {
                "type": type(error).__name__,
                "message": str(error),
            }
        return result

    completed: subprocess.CompletedProcess | None = None
    stage = "open_log"
    try:
        with log_file_path.open("xb") as output:
            stage = "run_captured"
            completed = run_captured(command, cwd=working_directory, env=dict(env),
                                     timeout=timeout, output_file=output)
    except BaseException as error:
        timed_out = isinstance(error, subprocess.TimeoutExpired)
        interrupted = not isinstance(error, Exception)
        outcome = "not_started" if stage == "open_log" else \
            "timed_out" if timed_out else "interrupted" if interrupted else "execution_error"
        returncode = getattr(error, "returncode", None)
        if completed is not None:
            returncode = completed.returncode
        receipt = make_receipt(outcome=outcome, returncode=returncode,
                               timed_out=timed_out, error=error, stage=stage)
        _record_without_masking(receipt_file_path, receipt, error)
        raise

    assert completed is not None
    _write_json_receipt(receipt_file_path, make_receipt(
        outcome="exited", returncode=completed.returncode, timed_out=False,
    ))
    return completed
