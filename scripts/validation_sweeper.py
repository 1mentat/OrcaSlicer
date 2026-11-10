#!/usr/bin/env python3
"""Sweep managed validation roots; install a per-user macOS schedule."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import plistlib
import re
import stat
import subprocess
import sys
import time

import validation_artifacts as manager

LABEL = 'org.orcaslicer.validation-artifacts'
LIMIT = 16384
FILES = ('validation_artifacts.py', 'validation_sweeper.py')


def _roots(roots):
    if not 1 <= len(roots) <= 32:
        raise manager.ArtifactError('Require 1..32 managed roots')
    result = []
    for root in roots:
        if not Path(root).is_absolute() or len(str(root)) > 4096:
            raise manager.ArtifactError('Root must be absolute and bounded')
        value = str(manager._root(root))
        if value in result:
            raise manager.ArtifactError('Duplicate root')
        result.append(value)
    return result


def sweep(roots, dry_run=False, config=None):
    roots = _roots(roots)
    result = {'roots': len(roots), 'removed_records': 0, 'reclaimed_bytes': 0,
              'errors': 0, 'dry_run': dry_run}
    identities = {}
    if config is not None:
        value = manager._read(Path(config), LIMIT)
        if (value.get('schema') != 'validation-sweeper-v1' or value.get('roots') != roots
                or not re.fullmatch('[0-9a-fA-F]{40}', value.get('source_sha', ''))):
            raise manager.ArtifactError('Invalid sweeper configuration')
        manager._text(value.get('owner'), 'owner')
        for name, local in zip(FILES, (Path(manager.__file__), Path(__file__))):
            if hashlib.sha256(local.read_bytes()).hexdigest() != value['files'][name]:
                raise manager.ArtifactError('Installed source differs from configuration')
        identities = value['root_ids']
        if set(identities) != set(roots):
            raise manager.ArtifactError('Managed root identity inventory differs')
        for identity in identities.values():
            manager._id(identity)
        result.update({k: value[k] for k in ('owner', 'source_sha', 'installed_at')})
        result['status_expires_at'] = manager._stamp(time.time() + 2 * value['interval_seconds'])
    for root in roots:
        try:
            value = manager.sweep(root, dry_run=dry_run, expected_root_id=identities.get(root))
            result['removed_records'] += len(value['removed_run_ids'])
            result['reclaimed_bytes'] += value['reclaimed_bytes']
        except (manager.ArtifactError, OSError, ValueError, KeyError):
            result['errors'] += 1
    return result


def _source(repository, source_sha):
    if not re.fullmatch('[0-9a-fA-F]{40}', source_sha):
        raise manager.ArtifactError('Require full source commit SHA')
    repository = Path(repository)
    if not repository.is_absolute():
        raise manager.ArtifactError('Repository must be absolute')
    env = {k: v for k, v in os.environ.items() if not k.upper().startswith('GIT_')}
    env.update(GIT_CONFIG_NOSYSTEM='1', GIT_CONFIG_GLOBAL=os.devnull, GIT_NO_REPLACE_OBJECTS='1')
    def git(*args):
        return subprocess.run(['git', '-C', str(repository), '-c', 'core.fsmonitor=false',
                               'cat-file', *args], env=env, check=True, capture_output=True, timeout=15).stdout
    if git('-t', source_sha).strip() != b'commit':
        raise manager.ArtifactError('Source must identify a commit')
    result = {}
    for name, path in zip(FILES, (Path(manager.__file__), Path(__file__))):
        spec = source_sha + ':scripts/' + name
        size = int(git('-s', spec))
        if not 0 < size <= 262144:
            raise manager.ArtifactError('Source file size out of bounds')
        blob = git('blob', spec)
        info = manager._plain(path)
        if not stat.S_ISREG(info.st_mode) or info.st_size != size or path.read_bytes() != blob:
            raise manager.ArtifactError('Executing source differs from selected Git blob')
        result[name] = blob
    return result


def _directory(path, private=False):
    for parent in reversed((path, *path.parents)):
        if parent.exists() or parent.is_symlink():
            manager._plain(parent, directory=True)
        else:
            parent.mkdir(mode=0o700)
    if private:
        manager._root(path)


def _write(path, data):
    if path.exists() or path.is_symlink():
        info = manager._plain(path)
        if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
            raise manager.ArtifactError('Unsafe installation file')
    fd, temporary = manager.tempfile.mkstemp(prefix='.metadata-', dir=path.parent)
    try:
        with os.fdopen(fd, 'wb') as stream:
            stream.write(data); stream.flush(); os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def _launch_domain():
    return f'gui/{os.getuid()}'


def install_macos(roots, repository, source_sha, owner, interval_seconds=300, *, home=None, bootstrap=None):
    if sys.platform != 'darwin':
        raise manager.ArtifactError('Installation requires macOS')
    manager._text(owner, 'owner')
    if type(interval_seconds) is not int or not 60 <= interval_seconds <= 3600:
        raise manager.ArtifactError('Interval must be 60..3600 seconds')
    roots = _roots(roots); blobs = _source(repository, source_sha)
    if sweep(roots, dry_run=True)['errors']:
        raise manager.ArtifactError('Managed root preflight failed')
    home = Path.home() if home is None else Path(home)
    if not home.is_absolute():
        raise manager.ArtifactError('Home must be absolute')
    tools = home / 'Library/Application Support/OrcaSlicer/validation-tools'
    agents = home / 'Library/LaunchAgents'; config_path = tools / 'config.json'
    _directory(tools, private=True); _directory(agents, private=True)
    existing = any(tools.iterdir())
    if existing:
        old = manager._read(config_path, LIMIT)
        if old.get('schema') != 'validation-sweeper-v1' or old.get('owner') != owner:
            raise manager.ArtifactError('Unmanaged installation or different owner')
        if any(p.name not in (*FILES, 'config.json') for p in tools.iterdir()):
            raise manager.ArtifactError('Unmanaged installation files')
    now = time.time()
    config = {'schema': 'validation-sweeper-v1', 'owner': owner, 'source_sha': source_sha,
              'installed_at': manager._stamp(now),
              'root_ids': {root: manager._read(Path(root) / 'root.json', manager.ROOT_METADATA_LIMIT)['root_id'] for root in roots},
              'roots': roots, 'interval_seconds': interval_seconds,
              'files': {name: hashlib.sha256(blob).hexdigest() for name, blob in blobs.items()}}
    encoded = (json.dumps(config, sort_keys=True) + '\n').encode()
    if len(encoded) > LIMIT:
        raise manager.ArtifactError('Installation metadata exceeds limit')
    argv = [sys.executable, '-B', str(tools / FILES[1]), 'sweep', '--config', str(config_path)]
    for root in roots:
        argv.extend(['--root', root])
    plist = {'Label': LABEL, 'ProgramArguments': argv, 'StartInterval': interval_seconds,
             'RunAtLoad': True, 'StandardOutPath': '/dev/null', 'StandardErrorPath': '/dev/null'}
    plist_path = agents / (LABEL + '.plist')
    if not existing and (plist_path.exists() or plist_path.is_symlink()):
        raise manager.ArtifactError('Existing unmanaged LaunchAgent')
    # Validate every destination before replacing any installed source.
    for path in [*(tools / name for name in FILES), config_path, plist_path]:
        if path.exists() or path.is_symlink():
            info = manager._plain(path)
            if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
                raise manager.ArtifactError('Unsafe installation destination')
    for name, blob in blobs.items():
        _write(tools / name, blob)
    _write(config_path, encoded); _write(plist_path, plistlib.dumps(plist))
    execute = bootstrap or (lambda args: subprocess.run(args, check=False, capture_output=True, timeout=15))
    domain = _launch_domain()
    execute(['/bin/launchctl', 'bootout', domain + '/' + LABEL])
    response = execute(['/bin/launchctl', 'bootstrap', domain, str(plist_path)])
    if response.returncode != 0:
        raise manager.ArtifactError('LaunchAgent bootstrap failed; files prepared but not installed')
    return {'installed': True, 'label': LABEL, 'owner': owner, 'source_sha': source_sha,
            'installed_at': config['installed_at'], 'roots': len(roots), 'interval_seconds': interval_seconds}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    sub = parser.add_subparsers(dest='command', required=True)
    command = sub.add_parser('sweep', allow_abbrev=False)
    command.add_argument('--root', action='append', required=True); command.add_argument('--dry-run', action='store_true')
    command.add_argument('--config')
    command = sub.add_parser('install-macos', allow_abbrev=False)
    command.add_argument('--root', action='append', required=True); command.add_argument('--repository', required=True)
    command.add_argument('--source-sha', required=True); command.add_argument('--owner', required=True)
    command.add_argument('--interval-seconds', type=int, default=300)
    args = vars(parser.parse_args(argv)); command = args.pop('command'); args['roots'] = args.pop('root')
    try:
        result = sweep(**args) if command == 'sweep' else install_macos(**args)
        print(json.dumps(result, sort_keys=True)); return int(bool(result.get('errors')))
    except (manager.ArtifactError, OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError):
        print(json.dumps({'error': 'Validation sweeper operation failed', 'installed': False})); return 1


if __name__ == '__main__':
    raise SystemExit(main())
