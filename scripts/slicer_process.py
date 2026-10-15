"""Captured native commands whose timeouts and interruptions stop their child processes."""

from __future__ import annotations

import os
from pathlib import Path
import signal
import subprocess
from typing import Sequence


def run_captured(argv: Sequence[str], *, cwd: Path | str | None = None,
                 env: dict[str, str] | None = None, timeout: float | None = None,
                 input_data: str | bytes | None = None) -> subprocess.CompletedProcess:
    """Run a build or test in its own process group, preserving partial output.

    Killing only CMake, Ninja, or CTest leaves compiler/test children running
    against a worktree the caller may be about to restore or remove.
    """
    text_mode = not isinstance(input_data, bytes)
    group_options = ({"start_new_session": True} if os.name == "posix" else
                     {"creationflags": subprocess.CREATE_NEW_PROCESS_GROUP})
    with subprocess.Popen(
        list(argv), cwd=cwd, env=env, shell=False,
        stdin=subprocess.PIPE if input_data is not None else None,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
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
            raise subprocess.TimeoutExpired(list(argv), timeout, output=stdout, stderr=stderr) from None
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
