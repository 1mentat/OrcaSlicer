#!/usr/bin/env python3
"""One exact-artifact direct case; static registration gate precedes native launch."""
import datetime
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import signal
import subprocess
import sys
import tarfile
import time

# Reuse the reviewed parent verifier; importing it launches nothing.
from cgal_artifact_diagnostic import CASE, SEED, OUT, digest, write_json


def commands(text):
    # Only the literal format emitted by the bound CatchAddTests add_command.
    # Never evaluate CMake, includes, generator expressions, or substitutions.
    if len(text.encode()) > 4 * 1024 * 1024:
        raise ValueError('Registration size bound')
    pos = 0
    while pos < len(text):
        space = re.match(r'\s*', text[pos:]); pos += space.end()
        if pos == len(text):
            return
        name = re.match(r'(add_test|set_tests_properties|set)\(', text[pos:])
        if not name:
            raise ValueError('Unknown generated registration command')
        pos += name.end(); args = []
        while True:
            space = re.match(r'\s*', text[pos:]); pos += space.end()
            if pos >= len(text):
                raise ValueError('Unterminated registration')
            if text[pos] == ')':
                pos += 1; break
            if text.startswith('[==[', pos):
                end = text.find(']==]', pos + 4)
                if end < 0:
                    raise ValueError('Unterminated bracket argument')
                value = text[pos + 4:end]; pos = end + 4
            else:
                word = re.match(r'[-./:a-zA-Z0-9_]+', text[pos:])
                if not word:
                    raise ValueError('Unsupported generated argument')
                value = word.group(); pos += word.end()
            if '\x00' in value or '${' in value or '$<' in value:
                raise ValueError('Nonliteral registration')
            args.append(value)
        yield name.group(1), args


def mapped_path(value, tests):
    # Artifact paths retain the original checkout prefix. Map only the exact
    # build/arm64/tests suffix, never arbitrary absolute paths or outside files.
    marker = '/build/arm64/tests'
    if value.count(marker) != 1:
        raise ValueError('Unmappable artifact path')
    prefix, suffix = value.split(marker)
    if not Path(prefix).is_absolute() or (suffix and not suffix.startswith('/')):
        raise ValueError('Invalid original checkout path')
    candidate = (tests / suffix.lstrip('/')).resolve()
    if not candidate.is_relative_to(tests.resolve()):
        raise ValueError('Escaping artifact path')
    return candidate


def registration(tests, binary):
    matches = []; scanned = []
    files = sorted((tests / 'libslic3r').rglob('*_tests*.cmake'))
    if not files or len(files) > 64:
        raise ValueError('Registration inventory bound')
    for path in files:
        if path.is_symlink() or not path.is_file() or not path.resolve().is_relative_to(tests):
            raise ValueError('Unsafe registration file')
        # Include wrappers can execute discovery; inspect only literal generated
        # files containing add_test, never interpret/include the wrappers.
        text = path.read_text(encoding='utf-8')
        if 'add_test(' not in text:
            continue
        rows = list(commands(text)); selected = [a for n, a in rows if n == 'add_test' and a and a[0] == CASE]
        scanned.append({'path': str(path), 'sha256': digest(path), 'bytes': path.stat().st_size})
        for args in selected:
            if len(args) < 3 or args[2] != CASE or mapped_path(args[1], tests) != binary:
                raise ValueError('Selected command is not exact binary/case')
            extras = args[3:]
            if extras not in ([], ['--reporter', 'console'], ['--reporter console']):
                raise ValueError('Unsupported original selected-case arguments')
            props = {}
            for name, values in rows:
                if name != 'set_tests_properties' or not values or values[0] != CASE:
                    continue
                if len(values) < 2 or values[1] != 'PROPERTIES' or len(values[2:]) % 2:
                    raise ValueError('Unsupported selected properties')
                for key, value in zip(values[2::2], values[3::2]):
                    if key in props:
                        raise ValueError('Repeated selected property')
                    props[key] = value
            allowed = {'WORKING_DIRECTORY', 'LABELS', 'PROCESSORS', 'TIMEOUT', 'SKIP_RETURN_CODE'}
            if not set(props).issubset(allowed) or 'WORKING_DIRECTORY' not in props:
                raise ValueError('Unmapped environment/executor/property')
            if 'PROCESSORS' in props and not props['PROCESSORS'].isdigit():
                raise ValueError('Invalid processor metadata')
            cwd = mapped_path(props['WORKING_DIRECTORY'], tests)
            if not cwd.is_dir():
                raise ValueError('Missing original working directory')
            matches.append({'registration': str(path), 'original_argv': args[1:],
                            'original_properties': props, 'mapped_cwd': str(cwd),
                            'argv': [str(binary), *args[2:], '--rng-seed', SEED]})
    if len(matches) != 1:
        raise ValueError('Selected case must have one generated registration')
    return {'selected': matches[0], 'scanned': scanned,
            'difference': 'Explicit recorded Catch seed appended; environment inherited from current diagnostic job, not claimed recovered original job environment.'}


def crash_reports(before, child_pid, start, end, uuid, binary):
    directory = Path.home() / 'Library/Logs/DiagnosticReports'
    records = []
    # Only this exact executable basename, and only newly created files. No
    # reading unrelated reports or pre-existing reports is permitted.
    paths = sorted(directory.glob(binary.name + '-*.ips')) if directory.is_dir() else []
    if len(paths) > 32:
        raise ValueError('Owned report candidate bound')
    for path in paths:
        if str(path) in before or path.is_symlink():
            continue
        stat = path.stat()
        if not path.is_file() or not start <= stat.st_mtime <= end + 30 or stat.st_size > 2 * 1024 * 1024:
            continue
        raw = path.read_bytes(); text = raw.decode('utf-8'); decoder = json.JSONDecoder()
        header, offset = decoder.raw_decode(text); payload, finish = decoder.raw_decode(text, len(text[:offset]) + len(text[offset:]) - len(text[offset:].lstrip()))
        if text[finish:].strip() or not isinstance(header, dict) or not isinstance(payload, dict):
            continue
        if payload.get('pid') != child_pid or payload.get('procName') != binary.name:
            continue
        launched = payload.get('procLaunch', '')
        try:
            born = datetime.datetime.fromisoformat(launched.replace('Z', '+00:00')).timestamp()
        except (ValueError, TypeError):
            continue
        if not start - 2 <= born <= end:
            continue
        images = payload.get('usedImages', [])
        if not any(str(x.get('uuid', '')).replace('-', '').lower() == uuid.replace('-', '').lower()
                   and Path(x.get('path', '')).name == binary.name for x in images if isinstance(x, dict)):
            continue
        target = OUT / ('owned-crash-' + str(len(records)) + '.ips')
        target.write_bytes(raw)
        records.append({'path': str(target), 'sha256': hashlib.sha256(raw).hexdigest(),
                        'bytes': len(raw), 'pid': child_pid, 'procLaunch': launched,
                        'binary_uuid': uuid, 'ownership': 'new basename candidate plus exactPID/launchtime/imageUUID'})
    return records


def run():
    assert platform.system() == 'Darwin' and platform.machine() == 'arm64'
    assert (OUT / 'download.json').is_file()
    root = Path.cwd().resolve(); tests = root / 'build/arm64/tests'
    archive = Path('diagnostic-input/build_tests.tar')
    # Exact same trusted SHA-bound artifact domain and extraction checks as parent.
    with tarfile.open(archive) as tar:
        for member in tar.getmembers():
            name = Path(member.name)
            assert not name.is_absolute() and name.parts[:3] == ('build', 'arm64', 'tests')
            assert (root / name).resolve().is_relative_to(tests)
            assert member.isfile() or member.isdir() or member.issym() or member.islnk()
            if member.issym() or member.islnk():
                target = (root / name.parent / member.linkname if member.issym() else root / member.linkname).resolve()
                assert target.is_relative_to(tests)
    subprocess.run(['tar', '-xf', str(archive)], check=True, timeout=60)
    candidates = [p.resolve() for p in (tests / 'libslic3r').rglob('libslic3r_tests') if p.is_file()]
    assert len(candidates) == 1
    binary = candidates[0]; assert binary.is_relative_to(tests) and os.access(binary, os.X_OK)
    binding = registration(tests, binary)  # Static gate before ANY binary invocation.
    write_json(OUT / 'direct-registration.json', binding)
    uuid_text = subprocess.run(['/usr/bin/xcrun', 'dwarfdump', '--uuid', str(binary)],
                               capture_output=True, timeout=15, check=True).stdout.decode()
    found = re.findall(r'UUID: ([A-Fa-f0-9-]{36}) \(arm64\)', uuid_text); assert len(found) == 1
    uuid = found[0]; before_hash = digest(binary)
    directory = Path.home() / 'Library/Logs/DiagnosticReports'
    before = {str(p) for p in directory.glob(binary.name + '-*.ips')} if directory.is_dir() else set()
    selected = binding['selected']; started = time.time(); mono = time.monotonic(); timed_out = False
    # Inherit the current CI environment, removing only credential variables.
    # This is explicitly not a reconstruction of the lost original CTest env.
    env = {k: v for k, v in os.environ.items() if not any(s in k.upper() for s in ('TOKEN', 'SECRET', 'PASSWORD'))}
    with (OUT / 'direct.stdout').open('wb') as stdout, (OUT / 'direct.stderr').open('wb') as stderr:
        child = subprocess.Popen(selected['argv'], cwd=selected['mapped_cwd'], env=env,
                                 stdin=subprocess.DEVNULL, stdout=stdout, stderr=stderr, start_new_session=True)
        try:
            write_json(OUT / 'direct-owned-child.json', {'pid': child.pid, 'started': started, 'uuid': uuid,
                                                       'argv': selected['argv'], 'cwd': selected['mapped_cwd']})
            try:
                child.wait(timeout=180)
            except subprocess.TimeoutExpired:
                timed_out = True
        finally:
            # Only the direct Popen child; no LLDB/inferior/group discovery.
            try:
                if child.returncode is None:
                    child.kill()
            finally:
                child.wait(timeout=10)
    ended = time.time(); elapsed = time.monotonic() - mono
    reports = []; report_error = None
    if child.returncode is not None and child.returncode < 0:
        until = time.monotonic() + 20
        try:
            while True:
                reports = crash_reports(before, child.pid, started, ended, uuid, binary)
                if reports or time.monotonic() >= until:
                    break
                time.sleep(0.5)
        except Exception as error:
            report_error = type(error).__name__
    write_json(OUT / 'direct-result.json', {'rc': child.returncode, 'timeout': timed_out,
               'native_elapsed_seconds': elapsed, 'pid': child.pid, 'case': CASE, 'seed': SEED,
               'binary_sha256_before': before_hash, 'binary_sha256_after': digest(binary), 'uuid': uuid,
               'new_owned_crash_reports': reports, 'crash_report_error': report_error,
               'outcome': 'signal' if child.returncode < 0 else 'native_exit',
               'diagnostic_only_no_product_acceptance': True, 'selected_case_launches': 1})
    assert digest(binary) == before_hash
    assert not timed_out
    # A native failure is a measurement; no retry or product-PASS conversion.


if __name__ == '__main__':
    OUT.mkdir(exist_ok=True)
    try:
        assert sys.argv[1:] == ['run']
        run()
    except Exception as error:
        write_json(OUT / 'direct-failure.json', {'exception_type': type(error).__name__, 'no_retry': True})
        raise
