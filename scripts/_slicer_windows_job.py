"""Windows command ownership through a gated, trusted Python job member.

The requesting parent owns the sole KillOnClose job handle. The broker cannot
start the requested command until that parent has assigned it to the job.
Control handles carry argv/status only; requested stdin and output are untouched.
"""
from __future__ import annotations

import ctypes
from ctypes import wintypes
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

_SETUP_MS = 10000
_DRAIN_SECONDS = 5
_MAX_CONTROL = 1024 * 1024


class _BasicLimits(ctypes.Structure):
    _fields_ = [("PerProcessUserTimeLimit", ctypes.c_longlong),
                ("PerJobUserTimeLimit", ctypes.c_longlong),
                ("LimitFlags", wintypes.DWORD),
                ("MinimumWorkingSetSize", ctypes.c_size_t),
                ("MaximumWorkingSetSize", ctypes.c_size_t),
                ("ActiveProcessLimit", wintypes.DWORD),
                ("Affinity", ctypes.c_size_t),
                ("PriorityClass", wintypes.DWORD),
                ("SchedulingClass", wintypes.DWORD)]


class _IoCounters(ctypes.Structure):
    _fields_ = [(name, ctypes.c_ulonglong) for name in (
        "ReadOperationCount", "WriteOperationCount", "OtherOperationCount",
        "ReadTransferCount", "WriteTransferCount", "OtherTransferCount")]


class _ExtendedLimits(ctypes.Structure):
    _fields_ = [("BasicLimitInformation", _BasicLimits),
                ("IoInfo", _IoCounters), ("ProcessMemoryLimit", ctypes.c_size_t),
                ("JobMemoryLimit", ctypes.c_size_t),
                ("PeakProcessMemoryUsed", ctypes.c_size_t),
                ("PeakJobMemoryUsed", ctypes.c_size_t)]


class _Accounting(ctypes.Structure):
    _fields_ = [("TotalUserTime", ctypes.c_longlong), ("TotalKernelTime", ctypes.c_longlong),
                ("ThisPeriodTotalUserTime", ctypes.c_longlong),
                ("ThisPeriodTotalKernelTime", ctypes.c_longlong),
                ("TotalPageFaultCount", wintypes.DWORD), ("TotalProcesses", wintypes.DWORD),
                ("ActiveProcesses", wintypes.DWORD), ("TotalTerminatedProcesses", wintypes.DWORD)]


class _Api:
    def __init__(self):
        self.kernel = ctypes.WinDLL("kernel32", use_last_error=True)
        signatures = {
            "CreateJobObjectW": ([ctypes.c_void_p, wintypes.LPCWSTR], wintypes.HANDLE),
            "SetInformationJobObject": ([wintypes.HANDLE, ctypes.c_int,
                                          ctypes.c_void_p, wintypes.DWORD], wintypes.BOOL),
            "CreateEventW": ([ctypes.c_void_p, wintypes.BOOL, wintypes.BOOL,
                               wintypes.LPCWSTR], wintypes.HANDLE),
            "SetEvent": ([wintypes.HANDLE], wintypes.BOOL),
            "WaitForSingleObject": ([wintypes.HANDLE, wintypes.DWORD], wintypes.DWORD),
            "OpenProcess": ([wintypes.DWORD, wintypes.BOOL, wintypes.DWORD], wintypes.HANDLE),
            "AssignProcessToJobObject": ([wintypes.HANDLE, wintypes.HANDLE], wintypes.BOOL),
            "TerminateJobObject": ([wintypes.HANDLE, wintypes.UINT], wintypes.BOOL),
            "QueryInformationJobObject": ([wintypes.HANDLE, ctypes.c_int, ctypes.c_void_p,
                                            wintypes.DWORD, ctypes.c_void_p], wintypes.BOOL),
            "CloseHandle": ([wintypes.HANDLE], wintypes.BOOL),
        }
        for name, (arguments, result) in signatures.items():
            function = getattr(self.kernel, name)
            function.argtypes, function.restype = arguments, result

    def checked(self, name, *arguments):
        value = getattr(self.kernel, name)(*arguments)
        if not value:
            raise ctypes.WinError(ctypes.get_last_error())
        return value

    def assign(self, job, broker):
        # Public handle acquisition; never access CPython's private Popen handle.
        if broker.poll() is not None:
            raise RuntimeError("Windows command broker exited before assignment")
        process = self.checked("OpenProcess", 0x0001 | 0x0100 | 0x100000, False, broker.pid)
        try:
            # The public Popen poll uses its original stable process identity.
            # Check again after OpenProcess so a recycled PID cannot be assigned.
            if broker.poll() is not None:
                raise RuntimeError("Windows command broker exited before assignment")
            self.checked("AssignProcessToJobObject", job, process)
        finally:
            self.checked("CloseHandle", process)


class _Job:
    def __init__(self, api):
        self.api, self.handle = api, None
        handle = api.checked("CreateJobObjectW", None, None)
        try:
            limits = _ExtendedLimits()
            limits.BasicLimitInformation.LimitFlags = 0x2000  # KILL_ON_JOB_CLOSE; no breakaway.
            api.checked("SetInformationJobObject", handle, 9,
                        ctypes.byref(limits), ctypes.sizeof(limits))
            assert not os.get_handle_inheritable(handle)
        except BaseException:
            api.checked("CloseHandle", handle)
            raise
        self.handle = handle

    def close(self):
        if self.handle is not None:
            self.api.checked("CloseHandle", self.handle)
            self.handle = None

    def stop(self):
        if self.handle is None:
            return
        try:
            self.api.checked("TerminateJobObject", self.handle, 1)
            deadline = time.monotonic() + _DRAIN_SECONDS
            while True:
                accounting = _Accounting()
                self.api.checked("QueryInformationJobObject", self.handle, 1,
                                 ctypes.byref(accounting), ctypes.sizeof(accounting), None)
                if accounting.ActiveProcesses == 0:
                    break
                if time.monotonic() >= deadline:
                    raise RuntimeError("Windows job still contains live processes after termination")
                time.sleep(.01)
        finally:
            self.close()


def _read_status(stream):
    stream.seek(0)
    data = stream.read(_MAX_CONTROL + 1)
    if not data or len(data) > _MAX_CONTROL:
        raise RuntimeError("Windows command broker did not return a bounded status record")
    record = json.loads(data.decode("utf-8"))
    if not isinstance(record, dict):
        raise RuntimeError("Invalid Windows command broker status")
    if record.get("outcome") == "launch_error":
        raise OSError(record["errno"], record["strerror"], record["filename"],
                      record.get("winerror"))
    if record.get("outcome") != "exited" or type(record.get("returncode")) is not int:
        raise RuntimeError("Windows command broker failed before command completion")
    return record["returncode"]


def _cleanup(process, job, original):
    """Never replace the requested timeout/interruption or enter an unbounded wait."""
    failures = []
    for operation in [job.stop]:
        try:
            operation()
        except Exception as error:
            failures.append(repr(error))
    try:
        if process.poll() is None:
            process.kill()  # Also covers a trusted broker whose assignment failed.
        result = process.communicate(timeout=_DRAIN_SECONDS)
    except BaseException as error:
        failures.append(repr(error))
        result = None
    if failures:
        original.add_note("Windows command cleanup incomplete: " + "; ".join(failures))
        original.cleanup_incomplete = True
    return result


def run_captured(argv, *, cwd=None, env=None, timeout=None, input_data=None, output_file=None):
    import msvcrt

    command = list(argv)
    # An owned temporary handle avoids consuming stdin or inflating the Windows
    # command line with encoded argv. No environment values enter either record.
    control = json.dumps([os.fspath(arg) for arg in command], ensure_ascii=True).encode("utf-8")
    if len(control) > _MAX_CONTROL:
        raise ValueError("Windows command argv control record exceeds its bounded size")
    api, process = _Api(), None
    job = _Job(api)
    gate = None
    cleaned = False
    try:
        gate = api.checked("CreateEventW", None, True, False, None)
        with tempfile.TemporaryFile("w+b") as arguments, tempfile.TemporaryFile("w+b") as status:
            arguments.write(control)
            arguments.flush()
            arguments.seek(0)
            handles = [gate, msvcrt.get_osfhandle(arguments.fileno()),
                       msvcrt.get_osfhandle(status.fileno())]
            startup = subprocess.STARTUPINFO()
            startup.lpAttributeList = {"handle_list": handles}
            try:
                for handle in handles:
                    os.set_handle_inheritable(handle, True)
                text = not isinstance(input_data, bytes)
                process = subprocess.Popen(
                    [str(Path(sys.executable).resolve()), "-I", "-S", str(Path(__file__).resolve()),
                     "--broker", *(str(handle) for handle in handles)],
                    cwd=cwd, env=env, shell=False, close_fds=True,
                    startupinfo=startup, creationflags=subprocess.CREATE_NEW_PROCESS_GROUP,
                    stdin=subprocess.PIPE if input_data is not None else None,
                    stdout=output_file if output_file is not None else subprocess.PIPE,
                    stderr=subprocess.STDOUT if output_file is not None else subprocess.PIPE,
                    text=text, encoding="utf-8" if text else None, errors="replace" if text else None,
                )
            finally:
                for handle in handles:
                    os.set_handle_inheritable(handle, False)
            try:
                api.assign(job.handle, process)
                api.checked("SetEvent", gate)  # Never signal a broker outside the job.
                deadline = None if timeout is None else time.monotonic() + timeout
                pending_input = input_data
                partial = None
                while True:
                    remaining = None if deadline is None else deadline - time.monotonic()
                    if remaining is not None and remaining <= 0:
                        raise subprocess.TimeoutExpired(command, timeout,
                                                        output=partial.output if partial else None,
                                                        stderr=partial.stderr if partial else None)
                    try:
                        stdout, stderr = process.communicate(
                            pending_input, timeout=min(0.1, remaining) if remaining is not None else 0.1)
                        break
                    except subprocess.TimeoutExpired as progress:
                        partial = progress
                        pending_input = None
                        if process.poll() is not None:
                            # Broker may exit before grandchildren release inherited PIPEs.
                            job.stop()
                            stdout, stderr = process.communicate(timeout=_DRAIN_SECONDS)
                            break
                job.stop()  # Normal return also waits for surviving descendants to exit.
                return subprocess.CompletedProcess(command, _read_status(status), stdout, stderr)
            except subprocess.TimeoutExpired as error:
                drained = _cleanup(process, job, error)
                cleaned = True
                if drained is not None:
                    error.output, error.stderr = drained
                error.cmd, error.timeout, error.returncode = command, timeout, process.returncode
                raise
            except BaseException as error:
                _cleanup(process, job, error)
                cleaned = True
                raise
    except BaseException as error:
        if process is not None and not cleaned:
            _cleanup(process, job, error)
        raise
    finally:
        original = sys.exception()
        for release in [job.close, lambda: api.checked("CloseHandle", gate) if gate is not None else None]:
            try:
                release()
            except OSError as error:
                if original is None:
                    raise
                original.add_note("Windows command handle cleanup incomplete: " + repr(error))
                original.cleanup_incomplete = True


def _broker(gate, arguments_handle, status_handle):
    import msvcrt

    api = _Api()
    if api.kernel.WaitForSingleObject(gate, _SETUP_MS) != 0:  # WAIT_OBJECT_0 only.
        api.checked("CloseHandle", gate)
        api.checked("CloseHandle", arguments_handle)
        api.checked("CloseHandle", status_handle)
        return 125
    api.checked("CloseHandle", gate)
    with os.fdopen(msvcrt.open_osfhandle(arguments_handle, os.O_RDONLY | os.O_BINARY), "rb") as stream:
        data = stream.read(_MAX_CONTROL + 1)
    if len(data) > _MAX_CONTROL:
        api.checked("CloseHandle", status_handle)
        return 125
    command = json.loads(data.decode("utf-8"))
    if not isinstance(command, list) or not command or not all(isinstance(arg, str) for arg in command):
        api.checked("CloseHandle", status_handle)
        return 125
    os.set_handle_inheritable(status_handle, False)
    with os.fdopen(msvcrt.open_osfhandle(status_handle, os.O_WRONLY | os.O_BINARY), "wb") as status:
        try:
            target = subprocess.Popen(command, shell=False, close_fds=True)
        except OSError as error:
            record = {"outcome": "launch_error", "errno": error.errno,
                      "strerror": error.strerror, "filename": error.filename,
                      "winerror": getattr(error, "winerror", None)}
            result = 127
        else:
            result = target.wait()
            record = {"outcome": "exited", "returncode": result}
        status.write(json.dumps(record).encode("utf-8"))
        status.flush()
    return result


if __name__ == "__main__":
    if len(sys.argv) != 5 or sys.argv[1] != "--broker":
        raise SystemExit(125)
    raise SystemExit(_broker(*(int(value) for value in sys.argv[2:])))
