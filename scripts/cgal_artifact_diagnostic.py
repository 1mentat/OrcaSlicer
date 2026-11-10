#!/usr/bin/env python3
"""One owned debugger launch of a source-bound, unchanged Mac ARM test artifact."""
import base64
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import shlex
import signal
import subprocess
import sys
import tarfile
import time
import urllib.parse
import urllib.request
import xml.etree.ElementTree as ET
import zipfile

CONFIG = Path('.github/diagnostics/cgal-artifact.json')
OUT = Path('cgal-diagnostic')
CASE = 'CGAL segmentation preserves the oriented surface of a dumbbell'
SEED = '1410530457'
DEADLINE_SECONDS = 180


def write_json(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2) + '\n')
    temporary.replace(path)


def digest(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            result.update(chunk)
    return result.hexdigest()


def runtime_env():
    return {k: v for k, v in os.environ.items()
            if not any(s in k.upper() for s in ('TOKEN', 'SECRET', 'PASSWORD'))}


def api(path):
    request = urllib.request.Request('https://api.github.com/' + path, headers={
        'Authorization': 'Bearer ' + os.environ['GITHUB_TOKEN'],
        'Accept': 'application/vnd.github+json', 'X-GitHub-Api-Version': '2022-11-28'})
    with urllib.request.urlopen(request, timeout=30) as response:
        return json.load(response)


class ArtifactRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, request, fp, code, message, headers, newurl):
        url = urllib.parse.urlsplit(newurl)
        if url.scheme != 'https' or not (url.hostname or '').endswith(
                ('.blob.core.windows.net', '.actions.githubusercontent.com')):
            raise ValueError('Unexpected artifact redirect host')
        # Signed storage requests must not receive the API bearer token.
        return urllib.request.Request(newurl)


def validate_zip(path, expected_digest, expected_bytes):
    assert path.stat().st_size == expected_bytes
    assert 'sha256:' + digest(path) == expected_digest
    with zipfile.ZipFile(path) as archive:
        assert len(archive.infolist()) == 1
        member = archive.infolist()[0]
        assert member.filename == 'build_tests.tar' and not member.is_dir()
        assert 0 < member.file_size <= 2 * 1024**3
        with archive.open(member) as source, Path('diagnostic-input/build_tests.tar').open('wb') as dest:
            while chunk := source.read(1024 * 1024):
                dest.write(chunk)


def verify():
    binding = json.loads(CONFIG.read_text())
    assert binding['repository'] == '1mentat/OrcaSlicer'
    assert binding['rng_seed'] == SEED
    prefix = 'repos/' + binding['repository']
    run = api(prefix + '/actions/runs/' + str(binding['run_id']))
    artifact = api(prefix + '/actions/artifacts/' + str(binding['artifact_id']))
    commit = api(prefix + '/git/commits/' + binding['checkout_sha'])
    assert run['head_sha'] == binding['head_sha']
    assert run['head_repository']['full_name'] == binding['repository']
    assert run['path'] == '.github/workflows/build_all.yml'
    assert commit['tree']['sha'] == binding['source_tree']
    assert [p['sha'] for p in commit['parents']] == binding['checkout_parents']
    head = api(prefix + '/git/commits/' + binding['head_sha'])
    assert head['tree']['sha'] == binding['source_tree']
    assert artifact['workflow_run']['id'] == binding['run_id']
    assert artifact['workflow_run']['head_sha'] == binding['head_sha']
    assert artifact['name'] == binding['artifact_name'] and artifact['expired'] is False
    assert artifact['digest'] == binding['artifact_digest']
    assert artifact['size_in_bytes'] == binding['artifact_bytes']
    consumer = api(prefix + '/actions/jobs/' + str(binding['failed_job_id']))
    assert consumer['run_id'] == binding['run_id'] and consumer['conclusion'] == 'failure'
    assert consumer['name'] == 'macOS arm64 / Unit Tests'
    for path, expected in binding['source_sha256'].items():
        content = api(prefix + '/contents/' + path + '?ref=' + binding['checkout_sha'])
        assert content['type'] == 'file' and content['encoding'] == 'base64'
        assert hashlib.sha256(base64.b64decode(content['content'])).hexdigest() == expected
    write_json(OUT / 'binding.json', {'binding': binding, 'artifact_created_at': artifact['created_at'],
               'artifact_expires_at': artifact['expires_at'], 'workflow_status': run['status']})
    Path('diagnostic-input').mkdir(exist_ok=True)
    request = urllib.request.Request('https://api.github.com/' + prefix + '/actions/artifacts/' +
        str(binding['artifact_id']) + '/zip', headers={'Authorization': 'Bearer ' + os.environ['GITHUB_TOKEN']})
    started = time.monotonic()
    zip_path = Path('diagnostic-input/artifact.zip')
    with urllib.request.build_opener(ArtifactRedirect()).open(request, timeout=30) as response, zip_path.open('wb') as dest:
        while chunk := response.read(1024 * 1024):
            if dest.tell() + len(chunk) > binding['artifact_bytes'] or time.monotonic() - started > 300:
                raise ValueError('Artifact download exceeded its bound')
            dest.write(chunk)
    validate_zip(zip_path, binding['artifact_digest'], binding['artifact_bytes'])
    write_json(OUT / 'download.json', {'zip_sha256': digest(zip_path),
               'zip_bytes': zip_path.stat().st_size, 'tar_sha256': digest(Path('diagnostic-input/build_tests.tar'))})


def classify(process, lldb, entry_stop_id=None):
    state = process.GetState()
    if state == lldb.eStateExited:
        return {'kind': 'native_exit', 'exit_status': process.GetExitStatus()}
    reasons = []
    if state in (lldb.eStateStopped, lldb.eStateCrashed):
        if entry_stop_id is not None and process.GetStopID() == entry_stop_id:
            return None  # Async Continue may still expose the previous entry stop.
        signals = process.GetUnixSignals()
        for thread in process:
            reason = thread.GetStopReason()
            data = [thread.GetStopReasonDataAtIndex(i) for i in range(thread.GetStopReasonDataCount())]
            reasons.append({'thread_id': thread.GetThreadID(), 'reason': reason, 'data': data,
                            'description': thread.GetStopDescription(1024) or ''})
        sigill = any(x['reason'] == lldb.eStopReasonSignal and x['data'] and
                     x['data'][0] == signals.GetSignalNumberFromName('SIGILL') for x in reasons)
        mach_illegal = any(x['reason'] == lldb.eStopReasonException and
                           re.search(r'\bEXC_BAD_INSTRUCTION\b', x['description']) for x in reasons)
        kind = 'SIGILL_stop' if sigill else ('Mach_BAD_INSTRUCTION_stop' if mach_illegal else 'other_stop')
        return {'kind': kind, 'state': state, 'threads': reasons}
    return None


def process_info(pid):
    result = subprocess.run(['/bin/ps', '-p', str(pid), '-o', 'pid=,ppid=,pgid=,lstart=,stat='],
                            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                            env=runtime_env(), timeout=3)
    if result.returncode == 1 and not result.stdout.strip():
        return None
    assert result.returncode == 0
    parts = result.stdout.decode().split()
    assert len(parts) == 9 and int(parts[0]) == pid
    return {'pid': pid, 'ppid': int(parts[1]), 'pgid': int(parts[2]),
            'birth': ' '.join(parts[3:8]), 'state': parts[8]}


def cleanup_inferior(info, debugger_pid):
    """Signal only a matching generation still descended from our owned group."""
    if info is None:
        return {'drained': False, 'reason': 'no_inferior_record_cleanup_unproved'}
    assert info['debugger_pid'] == debugger_pid
    pid = info['pid']
    assert type(pid) is int and pid > 1 and pid != debugger_pid
    current = process_info(pid)
    if current is None or current['state'].startswith('Z'):
        return {'drained': True, 'reason': 'absent_or_zombie'}
    if 'identity' not in info:
        return {'drained': False, 'reason': 'incomplete_launch_identity'}
    if current['birth'] != info['identity']['birth']:
        return {'drained': True, 'reason': 'original_generation_gone_no_signal'}
    ancestor = current
    for _ in range(8):
        if ancestor['pgid'] == debugger_pid:
            break
        if ancestor['ppid'] <= 1:
            return {'drained': False, 'reason': 'owned_ancestry_unproved_no_signal'}
        ancestor = process_info(ancestor['ppid'])
        if ancestor is None:
            return {'drained': False, 'reason': 'owned_ancestry_vanished_no_signal'}
    else:
        return {'drained': False, 'reason': 'owned_ancestry_bound_no_signal'}
    if current['pgid'] != pid:
        return {'drained': False, 'reason': 'unexpected_inferior_group_no_signal'}
    try:
        os.killpg(pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        state = process_info(pid)
        if state is None or state['state'].startswith('Z') or state['birth'] != current['birth']:
            return {'drained': True, 'reason': 'owned_group_killed_and_drained'}
        time.sleep(0.1)
    return {'drained': False, 'reason': 'owned_inferior_not_drained'}


def entry_stop(process, lldb):
    found = False
    signals = process.GetUnixSignals()
    allowed_signals = {signals.GetSignalNumberFromName(n) for n in ('SIGSTOP', 'SIGTRAP')}
    for thread in process:
        reason = thread.GetStopReason()
        if reason in (lldb.eStopReasonInvalid, lldb.eStopReasonNone):
            continue
        found = True
        if reason == lldb.eStopReasonSignal:
            if thread.GetStopReasonDataCount() != 1 or thread.GetStopReasonDataAtIndex(0) not in allowed_signals:
                return False
        elif reason not in (lldb.eStopReasonTrace, lldb.eStopReasonPlanComplete, lldb.eStopReasonBreakpoint):
            return False
    return found


def debug():
    """Called only inside hosted LLDB; no assumed prelaunch process-handle command."""
    import lldb
    request = json.loads((OUT / 'launch.json').read_text())
    debugger = lldb.debugger
    process = None
    record = {'kind': 'debugger_startup_failure', 'native_exit_status': None}
    start = time.monotonic()
    phase = 'debugger_startup'
    continue_requested = False
    try:
        debugger.SetAsync(False)
        target = debugger.CreateTarget(request['binary'])
        assert target.IsValid()
        debugger.SetSelectedTarget(target)
        launch = lldb.SBLaunchInfo(request['argv'])
        launch.SetWorkingDirectory(request['cwd'])
        launch.SetEnvironmentEntries([k + '=' + v for k, v in runtime_env().items()], False)
        launch.SetLaunchFlags(lldb.eLaunchFlagDebug | lldb.eLaunchFlagStopAtEntry |
                              lldb.eLaunchFlagLaunchInSeparateProcessGroup)
        assert launch.AddOpenFileAction(0, '/dev/null', True, False)
        assert launch.AddOpenFileAction(1, str((OUT / 'native.stdout').resolve()), False, True)
        assert launch.AddOpenFileAction(2, str((OUT / 'native.stderr').resolve()), False, True)
        error = lldb.SBError()
        process = target.Launch(launch, error)
        assert error.Success() and process.IsValid(), 'Owned debugger launch failed'
        info = {'pid': process.GetProcessID(), 'debugger_pid': os.getpid()}
        write_json(OUT / 'inferior.json', info)  # Prompt ownership record, even if the next query fails.
        info['identity'] = process_info(process.GetProcessID())
        assert info['identity'] is not None
        write_json(OUT / 'inferior.json', info)
        # Explicit separate group permits the outer timeout guard to clear only
        # this recorded inferior group if LLDB itself becomes unresponsive.
        assert os.getpgid(process.GetProcessID()) == process.GetProcessID(), 'Unexpected inferior process group'
        assert process.GetState() == lldb.eStateStopped, 'No initial entry stop'
        if not entry_stop(process, lldb):
            record = classify(process, lldb)
            assert record is not None
            record['stopped_during_launch'] = True
        else:
            phase = 'signal_policy'
            signals = process.GetUnixSignals()
            sigill = signals.GetSignalNumberFromName('SIGILL')
            assert sigill > 0
            assert signals.SetShouldStop(sigill, True)
            assert signals.SetShouldNotify(sigill, True)
            assert signals.SetShouldSuppress(sigill, True)
            phase = 'native_execution'
            entry_stop_id = process.GetStopID()
            # Synchronous Continue returns at the next stop/exit. The external
            # 180-second watchdog handles a hung debugger or running inferior.
            continue_requested = True
            write_json(OUT / 'debugger-phase.json', {'phase': phase, 'continue_requested': True,
                       'entry_stop_id': entry_stop_id})
            assert process.Continue().Success()
            record = classify(process, lldb, entry_stop_id)
            assert record is not None, 'Debugger returned a stale entry/running state'
        if record['kind'] in ('SIGILL_stop', 'Mach_BAD_INSTRUCTION_stop', 'other_stop'):
            record['context_commands'] = []
            with (OUT / 'stop-context.txt').open('w') as log:
                for command in ('process status', 'thread backtrace all', 'register read',
                                'disassemble --pc --count 24', 'image list -o -f'):
                    result = lldb.SBCommandReturnObject()
                    debugger.GetCommandInterpreter().HandleCommand(command, result)
                    log.write(command + '\n' + (result.GetOutput() or '') + (result.GetError() or '') + '\n')
                    record['context_commands'].append({'command': command, 'succeeded': result.Succeeded()})
            record['context_complete'] = all(x['succeeded'] for x in record['context_commands'])
        if record['kind'] == 'native_exit':
            record['native_exit_status'] = record['exit_status']
    except Exception as error:
        record = {'kind': 'debugger_startup_or_control_failure', 'phase': phase, 'exception_type': type(error).__name__}
    finally:
        if process is not None and process.IsValid() and process.GetState() not in (lldb.eStateExited, lldb.eStateDetached):
            killed = process.Destroy()
            record['owned_inferior_kill_success'] = killed.Success()
        record['continue_requested'] = continue_requested
        record['owned_inferior_pid'] = process.GetProcessID() if process is not None and process.IsValid() else None
        record['elapsed_seconds'] = time.monotonic() - start
        write_json(OUT / 'debugger-result.json', record)


def run():
    assert platform.system() == 'Darwin' and platform.machine() == 'arm64'
    assert (OUT / 'download.json').is_file()
    root = Path.cwd().resolve()
    archive = Path('diagnostic-input/build_tests.tar')
    with tarfile.open(archive) as tar:
        for member in tar.getmembers():
            name = Path(member.name)
            assert not name.is_absolute() and name.parts[:3] == ('build', 'arm64', 'tests')
            assert (root / name).resolve().is_relative_to(root / 'build/arm64/tests')
            assert member.isfile() or member.isdir() or member.issym() or member.islnk()
            if member.issym() or member.islnk():
                target = (root / name.parent / member.linkname if member.issym() else root / member.linkname).resolve()
                assert target.is_relative_to(root / 'build/arm64/tests')
    subprocess.run(['tar', '-xf', str(archive)], check=True, timeout=60)
    suite = root / 'build/arm64/tests/libslic3r'
    candidates = list(suite.glob('**/libslic3r_tests'))
    assert len(candidates) == 1
    binary = candidates[0].resolve()
    assert binary.is_file() and os.access(binary, os.X_OK)
    binary_hash = digest(binary)
    env = runtime_env()
    with (OUT / 'native-artifact.txt').open('w') as log:
        for command in (['/usr/bin/uname', '-a'], ['/usr/bin/sw_vers'],
                        ['/usr/bin/file', str(binary)], ['/usr/bin/otool', '-L', str(binary)],
                        ['/usr/bin/xcrun', 'dwarfdump', '--uuid', str(binary)], ['/usr/bin/xcrun', 'lldb', '--version']):
            result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env, timeout=15, check=True)
            text = result.stdout.decode(errors='replace')
            log.write(text)
            if command[0] == '/usr/bin/file':
                assert 'Mach-O' in text and 'arm64' in text
    with (OUT / 'selected-case.xml').open('wb') as registry:
        subprocess.run([str(binary), CASE, '--list-tests', '--reporter', 'xml'], cwd=suite,
                       stdin=subprocess.DEVNULL, stdout=registry, stderr=subprocess.PIPE, env=env, check=True, timeout=30)
    cases = ET.parse(OUT / 'selected-case.xml').findall('.//TestCase')
    assert len(cases) == 1 and cases[0].findtext('Name') == CASE
    write_json(OUT / 'launch.json', {'binary': str(binary), 'argv': [CASE, '--rng-seed', SEED, '--reporter', 'console'],
               'cwd': str(suite), 'cwd_scope': 'reconstructed suite binary directory, not claimed original printed argv',
               'binary_sha256': binary_hash, 'case': CASE, 'seed': SEED})
    script = shlex.quote(str(Path(__file__).resolve()))
    command = ['/usr/bin/xcrun', 'lldb', '--batch', '--no-lldbinit', '-o', 'command script import ' + script,
               '-o', 'script cgal_artifact_diagnostic.debug()']
    timed_out = False
    with (OUT / 'lldb.log').open('wb') as log:
        child = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                                 env=env, start_new_session=True)
        try:
            child.wait(timeout=DEADLINE_SECONDS)
        except subprocess.TimeoutExpired:
            timed_out = True
        finally:
            info_path = OUT / 'inferior.json'
            try:
                info = json.loads(info_path.read_text()) if info_path.exists() else None
                cleanup = cleanup_inferior(info, child.pid)
            except Exception as error:
                cleanup = {'drained': False, 'reason': 'cleanup_exception',
                           'exception_type': type(error).__name__}
            # Clear only the supervisor's own session/group after checking inferior.
            try:
                os.killpg(child.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            child.wait(timeout=10)
    result_path = OUT / 'debugger-result.json'
    result = json.loads(result_path.read_text()) if result_path.exists() else {'kind': 'debugger_failed_without_result'}
    write_json(OUT / 'result.json', {'debugger': result, 'supervisor_timeout': timed_out,
               'debugger_rc': child.returncode, 'owned_cleanup': cleanup, 'binary_sha256_before': binary_hash,
               'binary_sha256_after': digest(binary), 'archive_sha256': digest(archive),
               'case': CASE, 'seed': SEED, 'requested_selected_case_launches': 1, 'listing_invocations': 1,
               'fixture_or_binary_modified': False, 'diagnostic_only_not_product_acceptance': True})
    assert digest(binary) == binary_hash
    assert cleanup['drained']
    assert not timed_out and child.returncode == 0
    assert result['kind'] in ('native_exit', 'SIGILL_stop', 'Mach_BAD_INSTRUCTION_stop', 'other_stop')
    assert result.get('owned_inferior_kill_success', True)
    if result['kind'] in ('SIGILL_stop', 'Mach_BAD_INSTRUCTION_stop', 'other_stop'):
        assert result['context_complete']


if __name__ == '__main__':
    OUT.mkdir(exist_ok=True)
    try:
        if sys.argv[1:] == ['verify']:
            verify()
        elif sys.argv[1:] == ['run']:
            run()
        else:
            raise ValueError('Expected verify or run')
    except Exception as error:
        write_json(OUT / 'failure.json', {'exception_type': type(error).__name__, 'stage': sys.argv[1:],
                   'evidence_only': True})
        sys.exit(1)
