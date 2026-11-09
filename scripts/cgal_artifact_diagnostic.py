#!/usr/bin/env python3
"""Profile an unchanged CGAL case from a verified, existing macOS test artifact."""
import base64
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
import urllib.request
import xml.etree.ElementTree as ET

CONFIG = Path('.github/diagnostics/cgal-artifact.json')
OUT = Path('cgal-diagnostic')
CASE = 'CGAL segmentation preserves the oriented surface of a dumbbell'
REPEATS, SLOW_SECONDS, DEADLINE_SECONDS = 5, 30, 180


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def digest(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            result.update(chunk)
    return result.hexdigest()


def signal_child_group(child, sig):
    try:
        os.killpg(child.pid, sig)
    except ProcessLookupError:
        pass  # The owned child may finish between poll and signal.


def api(path):
    request = urllib.request.Request('https://api.github.com/' + path, headers={
        'Authorization': 'Bearer ' + os.environ['GITHUB_TOKEN'],
        'Accept': 'application/vnd.github+json',
        'X-GitHub-Api-Version': '2022-11-28',
    })
    with urllib.request.urlopen(request, timeout=30) as response:
        return json.load(response)


def verify():
    binding = json.loads(CONFIG.read_text())
    if binding['repository'] != '1mentat/OrcaSlicer':
        raise ValueError('Unexpected artifact repository')
    if any(type(binding.get(key)) is not int or binding[key] <= 0
           for key in ('run_id', 'artifact_id')):
        raise ValueError('Bind a real available run and artifact before publication')
    if not re.fullmatch(r'[0-9a-f]{40}', binding.get('head_sha') or ''):
        raise ValueError('Missing immutable artifact source head')
    if not re.fullmatch(r'sha256:[0-9a-f]{64}', binding.get('artifact_digest') or ''):
        raise ValueError('Missing actual artifact digest')
    prefix = 'repos/' + binding['repository']
    run = api(prefix + '/actions/runs/' + str(binding['run_id']))
    artifact = api(prefix + '/actions/artifacts/' + str(binding['artifact_id']))
    assert run['head_sha'] == binding['head_sha']
    assert run['head_repository']['full_name'] == binding['repository']
    assert run['path'] == '.github/workflows/build_all.yml'
    assert artifact['workflow_run']['id'] == binding['run_id']
    assert artifact['workflow_run']['head_sha'] == binding['head_sha']
    assert artifact['name'].endswith('-tests-macos-arm64')
    assert artifact['expired'] is False
    assert artifact['digest'] == binding['artifact_digest']
    for path, expected in binding['source_sha256'].items():
        assert re.fullmatch(r'[0-9a-f]{64}', expected)
        content = api(prefix + '/contents/' + path + '?ref=' + binding['head_sha'])
        assert content['type'] == 'file' and content['encoding'] == 'base64'
        assert hashlib.sha256(base64.b64decode(content['content'])).hexdigest() == expected, path
    write_json(OUT / 'binding.json', {
        'binding': binding, 'artifact_name': artifact['name'],
        'size_in_bytes': artifact['size_in_bytes'], 'created_at': artifact['created_at'],
        'expires_at': artifact['expires_at'], 'workflow_status': run['status'],
    })
    with open(os.environ['GITHUB_OUTPUT'], 'a') as output:
        output.write('artifact_id=' + str(binding['artifact_id']) + '\n')
        output.write('run_id=' + str(binding['run_id']) + '\n')


def run():
    assert platform.system() == 'Darwin' and platform.machine() == 'arm64'
    assert Path('/usr/bin/sample').is_file()
    assert (OUT / 'binding.json').is_file()
    archive = Path('diagnostic-input/build_tests.tar')
    assert archive.is_file() and not archive.is_symlink()
    root = Path.cwd().resolve()
    # Reuse the unit job's tar layout, refusing traversal or escaping links.
    with tarfile.open(archive) as tar:
        for member in tar.getmembers():
            name = Path(member.name)
            assert not name.is_absolute() and name.parts[:3] == ('build', 'arm64', 'tests')
            assert (root / name).resolve().is_relative_to(root / 'build/arm64/tests')
            assert member.isfile() or member.isdir() or member.issym() or member.islnk()
            if member.issym() or member.islnk():
                target = (root / name.parent / member.linkname if member.issym()
                          else root / member.linkname).resolve()
                assert target.is_relative_to(root / 'build/arm64/tests')
    subprocess.run(['tar', '-xf', str(archive)], check=True, timeout=60)
    candidates = list(Path('build/arm64/tests/libslic3r').glob('**/libslic3r_tests'))
    assert len(candidates) == 1
    binary = candidates[0].resolve()
    assert binary.is_file() and os.access(binary, os.X_OK)
    binary_hash = digest(binary)
    env = {key: value for key, value in os.environ.items()
           if not any(secret in key.upper() for secret in ('TOKEN', 'SECRET', 'PASSWORD'))}
    registry_path = OUT / 'selected-case.xml'
    with registry_path.open('wb') as registry:
        subprocess.run([str(binary), CASE, '--list-tests', '--reporter', 'xml'],
                       stdin=subprocess.DEVNULL, stdout=registry, stderr=subprocess.PIPE,
                       env=env, check=True, timeout=30)
    cases = ET.parse(registry_path).findall('.//TestCase')
    assert len(cases) == 1 and cases[0].findtext('Name') == CASE
    with (OUT / 'native-artifact.txt').open('w') as log:
        for command in (['/usr/bin/uname', '-a'], ['/usr/bin/sw_vers'],
                        ['/usr/sbin/sysctl', 'hw.ncpu', 'hw.memsize'],
                        ['/usr/bin/file', str(binary)], ['/usr/bin/otool', '-L', str(binary)]):
            subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=15)
    records = []
    for repetition in range(REPEATS):
        log_path = OUT / ('repeat-' + str(repetition + 1) + '.log')
        command = [str(binary), CASE, '--rng-seed', '7', '--reporter', 'console']
        timed_out = sampled = False
        start = time.monotonic()
        with log_path.open('wb') as log:
            child = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=log,
                                     stderr=subprocess.STDOUT, env=env, start_new_session=True)
            try:
                while child.poll() is None:
                    elapsed = time.monotonic() - start
                    if elapsed >= SLOW_SECONDS and not sampled:
                        sampled = True
                        with (OUT / ('sample-' + str(repetition + 1) + '-status.log')).open('wb') as status:
                            try:
                                result = subprocess.run(['/usr/bin/sample', str(child.pid), '5', '10', '-file',
                                    str((OUT / ('sample-' + str(repetition + 1) + '.txt')).resolve())],
                                    stdin=subprocess.DEVNULL, stdout=status, stderr=subprocess.STDOUT, timeout=15)
                                status.write(('\nreturncode=' + str(result.returncode) + '\n').encode())
                            except subprocess.TimeoutExpired:
                                status.write(b'\nsample tool exceeded its 15-second bound\n')
                    if time.monotonic() - start >= DEADLINE_SECONDS and child.poll() is None:
                        timed_out = True
                        signal_child_group(child, signal.SIGTERM)
                        try:
                            child.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            signal_child_group(child, signal.SIGKILL)
                        break
                    time.sleep(0.2)
            finally:
                if child.poll() is None:
                    signal_child_group(child, signal.SIGKILL)
                child.wait(timeout=10)
        records.append({'repetition': repetition + 1, 'argv': command,
                        'returncode': child.returncode, 'timeout': timed_out, 'sample_attempted': sampled,
                        'elapsed_seconds': time.monotonic() - start, 'log_sha256': digest(log_path)})
        write_json(OUT / 'results.json', {'binary': str(binary.relative_to(root)),
                   'binary_sha256': binary_hash, 'archive_sha256': digest(archive),
                   'case': CASE, 'records': records, 'fixture_or_binary_modified': False})
        assert digest(binary) == binary_hash
        if timed_out or child.returncode != 0:
            raise RuntimeError('Diagnostic child failed or reached its bound; evidence preserved')


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
        # API exceptions are not printed: redirected/signed URLs must not enter logs.
        write_json(OUT / 'failure.json', {'exception_type': type(error).__name__,
                   'stage': sys.argv[1:], 'evidence_only': True,
                   'reason': str(error) if type(error) in (ValueError, AssertionError, RuntimeError) else None})
        sys.exit(1)
