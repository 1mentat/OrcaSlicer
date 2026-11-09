"""Captured native commands whose timeouts and interruptions stop their child processes."""

from __future__ import annotations

import os
from pathlib import Path
import signal
import subprocess
from typing import IO, Sequence


def _kill_process_group(process: subprocess.Popen) -> None:
    if os.name == "posix":
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        return
    # taskkill is supplied by Windows and terminates the tree (/T).
    try:
        subprocess.run(["taskkill", "/PID", str(process.pid), "/T", "/F"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                       timeout=10, check=False)
    except (OSError, subprocess.TimeoutExpired):
        pass
    if process.poll() is None:
        process.kill()


def run_captured(argv: Sequence[str], *, cwd: Path | str | None = None,
                 env: dict[str, str] | None = None, timeout: float | None = None,
                 input_data: str | bytes | None = None,
                 output_file: IO | None = None) -> subprocess.CompletedProcess:
    """Run a build or test in its own process group, preserving partial output.

    Killing only CMake, Ninja, or CTest would leave compiler or test children
    running, and still writing profiles, after the caller has moved on.
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
            _kill_process_group(process)
            stdout, stderr = process.communicate()
            error = subprocess.TimeoutExpired(list(argv), timeout, output=stdout, stderr=stderr)
            # Keep the post-cleanup status available to callers that record it.
            error.returncode = process.returncode
            raise error from None
        except BaseException:
            # KeyboardInterrupt, cancellation or a test-framework exception:
            # stop the same process group before re-raising the original error.
            _kill_process_group(process)
            try:
                process.communicate()
            except BaseException:
                pass  # Preserve the caller's interruption over a cleanup error.
            raise
        return subprocess.CompletedProcess(list(argv), process.returncode, stdout, stderr)
