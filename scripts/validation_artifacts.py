#!/usr/bin/env python3
"""Bounded, leased validation output. Only create() returns a secret capability.

Producers write inside payload_dir, call renew/check at least every 20 seconds,
stop their owned work on failure, and call finish in finally on every outcome.
This module never starts or kills processes. Metadata is not a published receipt.
Root budgets include logical file bytes: payload reservations plus bounded metadata
(including atomic-write overlap). Filesystem allocation/inode overhead is not a quota.
"""
import argparse
import contextlib
import datetime as dt
import hashlib
import hmac
import json
import os
from pathlib import Path, PurePosixPath
import re
import secrets
import shutil
import stat
import sys
import tempfile
import time
import uuid

MAX_TTL = 30 * 24 * 3600
MAX_LEASE = 60
RENEW_INTERVAL = 20
LOCK_TIMEOUT = 5
METADATA_LIMIT = 4096
ROOT_METADATA_LIMIT = 511
ROOT_METADATA_RESERVE = 1024  # Root JSON plus atomic replacement and one-byte lock.
RUN_METADATA_RESERVE = 8192  # One bounded JSON record plus its atomic replacement.


class ArtifactError(RuntimeError):
    pass


def _positive(value, name, maximum=None):
    if type(value) is not int or value <= 0 or (maximum and value > maximum):
        raise ArtifactError('Invalid ' + name)
    return value


def _text(value, name):
    if not isinstance(value, str) or not value.strip() or len(value) > 256 or '\0' in value:
        raise ArtifactError('Invalid ' + name)
    return value


def _id(value):
    try:
        if str(uuid.UUID(value)) != value:
            raise ValueError()
    except (ValueError, TypeError, AttributeError):
        raise ArtifactError('Invalid run ID') from None
    return value


def _stamp(value):
    return dt.datetime.fromtimestamp(value, dt.timezone.utc).isoformat()


def _seconds(value):
    try:
        parsed = dt.datetime.fromisoformat(value)
        if parsed.utcoffset() != dt.timedelta(0):
            raise ValueError()
        return parsed.timestamp()
    except (ValueError, TypeError, AttributeError):
        raise ArtifactError('Invalid UTC metadata timestamp') from None


def _plain(path, directory=False):
    info = path.lstat()
    reparse = getattr(info, 'st_file_attributes', 0) & 0x400
    if stat.S_ISLNK(info.st_mode) or reparse or (directory and not stat.S_ISDIR(info.st_mode)):
        raise ArtifactError('Unsafe managed path')
    return info


def _root(root):
    path = Path(os.path.abspath(root))
    for parent in reversed((path, *path.parents)):
        if parent.exists() or parent.is_symlink():
            _plain(parent, directory=True)
    if path.exists():
        info = _plain(path, directory=True)
        if hasattr(os, 'getuid') and (info.st_uid != os.getuid() or info.st_mode & 0o077):
            raise ArtifactError('Root must be private and owned by this user')
    return path


def _read(path, limit=METADATA_LIMIT):
    info = _plain(path)
    if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1 or info.st_size > limit:
        raise ArtifactError('Unsafe metadata file')
    fd = os.open(path, os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0))
    with os.fdopen(fd, 'r', encoding='utf-8') as stream:
        opened = os.fstat(stream.fileno())
        if (opened.st_dev, opened.st_ino) != (info.st_dev, info.st_ino):
            raise ArtifactError('Metadata changed while opening')
        try:
            value = json.load(stream)
        except (ValueError, UnicodeError):
            raise ArtifactError('Invalid metadata JSON') from None
    if not isinstance(value, dict):
        raise ArtifactError('Invalid metadata object')
    return value


def _write(path, value, limit=METADATA_LIMIT):
    encoded = (json.dumps(value, sort_keys=True) + '\n').encode('utf-8')
    if len(encoded) > limit:
        raise ArtifactError('Metadata size limit exceeded')
    fd, temporary = tempfile.mkstemp(prefix='.metadata-', dir=path.parent)
    try:
        with os.fdopen(fd, 'wb') as stream:
            stream.write(encoded); stream.flush(); os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


@contextlib.contextmanager
def _locked(root, operation='internal'):
    if operation not in ('init', 'create', 'check', 'renew', 'finish', 'promote', 'sweep'):
        operation = 'internal'
    path = _root(root)
    info = _plain(path / '.lock')
    if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1 or info.st_size != 1:
        raise ArtifactError('Unsafe root lock')
    fd = os.open(path / '.lock', os.O_RDWR | getattr(os, 'O_NOFOLLOW', 0))
    acquired = False
    acquired_at = None
    completed = False
    started = time.monotonic()
    try:
        opened = os.fstat(fd)
        if (opened.st_dev, opened.st_ino) != (info.st_dev, info.st_ino):
            raise ArtifactError('Root lock changed')
        until = time.monotonic() + LOCK_TIMEOUT
        while not acquired:
            try:
                if os.name == 'nt':
                    import msvcrt
                    os.lseek(fd, 0, os.SEEK_SET); msvcrt.locking(fd, msvcrt.LK_NBLCK, 1)
                else:
                    import fcntl
                    fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                acquired = True
                acquired_at = time.monotonic()
            except (BlockingIOError, OSError):
                if time.monotonic() >= until:
                    raise ArtifactError('Root lock timed out') from None
                time.sleep(0.02)
        meta = _read(path / 'root.json', ROOT_METADATA_LIMIT)
        if meta.get('schema') != 'validation-root-v1':
            raise ArtifactError('Unmanaged root')
        _id(meta.get('root_id')); _positive(meta.get('total_budget_bytes'), 'root budget')
        allowed = {'.lock', 'root.json', 'runs'}
        if any(p.name not in allowed and not re.fullmatch(r'\.metadata-[\w-]+', p.name) for p in path.iterdir()):
            raise ArtifactError('Unmanaged root content')
        _plain(path / 'runs', directory=True)
        yield path, meta
        completed = True
    finally:
        try:
            if acquired:
                if os.name == 'nt':
                    import msvcrt
                    os.lseek(fd, 0, os.SEEK_SET); msvcrt.locking(fd, msvcrt.LK_UNLCK, 1)
                else:
                    import fcntl
                    fcntl.flock(fd, fcntl.LOCK_UN)
        finally:
            os.close(fd)
            ended = time.monotonic()
            waited = (acquired_at if acquired_at is not None else ended) - started
            held = ended - acquired_at if acquired_at is not None else 0
            if waited >= 1 or held >= 1:
                # One bounded slow-operation record, after releasing the lock.
                # No paths, capabilities, work-item contents or exception text.
                diagnostic = {'schema': 'validation-lock-timing-v1', 'operation': operation,
                              'wait_ms': round(waited * 1000), 'hold_ms': round(held * 1000),
                              'outcome': 'ok' if completed else 'error'}
                try:
                    sys.stderr.write(json.dumps(diagnostic, sort_keys=True) + '\n')
                except (AttributeError, OSError, ValueError):
                    pass  # Diagnostic delivery must not change transaction results.


def init(root, budget_bytes):
    """Initialize a new/empty private root, or verify its identical budget."""
    _positive(budget_bytes, 'root budget')
    if budget_bytes < ROOT_METADATA_RESERVE:
        raise ArtifactError('Root budget cannot cover metadata')
    path = _root(root)
    if path.exists() and any(path.iterdir()):
        with _locked(path, 'init') as (_, meta):
            if meta['total_budget_bytes'] != budget_bytes:
                raise ArtifactError('Existing root budget differs')
            return dict(meta)
    path.mkdir(mode=0o700, parents=False, exist_ok=True)
    fd = os.open(path / '.lock', os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
    with os.fdopen(fd, 'wb') as stream:
        stream.write(b'0')
    (path / 'runs').mkdir(mode=0o700)
    meta = {'schema': 'validation-root-v1', 'root_id': str(uuid.uuid4()),
            'created_at': _stamp(time.time()), 'total_budget_bytes': budget_bytes}
    _write(path / 'root.json', meta, ROOT_METADATA_LIMIT)
    return dict(meta)


def _record(directory, rootmeta):
    _plain(directory, directory=True); meta = _read(directory / 'run.json')
    if meta.get('schema') != 'validation-artifact-v1' or meta.get('root_id') != rootmeta['root_id']:
        raise ArtifactError('Unmanaged run')
    _id(meta.get('run_id'))
    if directory.name != meta['run_id']:
        raise ArtifactError('Mismatched run directory')
    _text(meta.get('owner'), 'owner'); _text(meta.get('work_item'), 'work item')
    if not isinstance(meta.get('source_sha'), str) or not re.fullmatch(r'[0-9a-fA-F]{40}', meta['source_sha']):
        raise ArtifactError('Invalid source SHA')
    _positive(meta.get('size_budget_bytes'), 'run budget')
    if meta.get('state') not in ('active', 'completed', 'failed', 'interrupted', 'retained'):
        raise ArtifactError('Invalid run state')
    if not isinstance(meta.get('token_hash'), str) or not re.fullmatch(r'[0-9a-f]{64}', meta['token_hash']):
        raise ArtifactError('Invalid capability metadata')
    created, expires = _seconds(meta['created_at']), _seconds(meta['expires_at'])
    lease = _seconds(meta['lease_expires_at'])
    if expires <= created or expires - created > MAX_TTL or lease > expires:
        raise ArtifactError('Invalid run expiry')
    allowed = {'run.json', 'payload'}
    if any(p.name not in allowed and not re.fullmatch(r'\.metadata-[\w-]+', p.name) for p in directory.iterdir()):
        raise ArtifactError('Unmanaged run content')
    return meta


def _inventory(path, rootmeta):
    records = []; stages = []
    for directory in (path / 'runs').iterdir():
        if directory.name.startswith('.new-'):
            _id(directory.name[5:]); _plain(directory, directory=True); stages.append(directory)
        else:
            _id(directory.name); records.append((directory, _record(directory, rootmeta)))
    return records, stages


def _size(path, *, payloads=(), _transient=False, _observed=None):
    # Producers may remove build intermediates during accounting. Only descendants
    # of validated payload roots may disappear; manager metadata remains strict.
    try:
        info = path.lstat()
        if stat.S_ISLNK(info.st_mode) or getattr(info, 'st_file_attributes', 0) & 0x400:
            return info.st_size
        _plain(path, directory=True)
        entries = os.scandir(path)
    except FileNotFoundError:
        if _transient:
            return 0
        raise
    mutable_children = _transient or path in payloads
    total = 0
    with entries:
        for entry in entries:
            try:
                value = entry.stat(follow_symlinks=False)
            except FileNotFoundError:
                if mutable_children:
                    continue
                raise
            if stat.S_ISLNK(value.st_mode) or getattr(value, 'st_file_attributes', 0) & 0x400:
                total += value.st_size  # Never traverse payload links.
            elif stat.S_ISDIR(value.st_mode):
                total += _size(Path(entry.path), payloads=payloads, _transient=mutable_children, _observed=_observed)
            elif stat.S_ISREG(value.st_mode):
                total += value.st_size
            else:
                raise ArtifactError('Unsupported payload file')
    if _observed is not None and path in payloads:
        _observed[path] = total
    return total


def _remove(path):
    if path.is_symlink():
        path.unlink()
    elif path.exists():
        info = path.lstat()
        if getattr(info, 'st_file_attributes', 0) & 0x400:
            path.rmdir() if stat.S_ISDIR(info.st_mode) else path.unlink()
        elif path.is_dir():
            shutil.rmtree(path)  # stdlib does not follow internal symlinks/junctions.
        else:
            path.unlink()


def _auth(meta, token):
    if not isinstance(token, str) or not hmac.compare_digest(meta['token_hash'], hashlib.sha256(token.encode()).hexdigest()):
        raise ArtifactError('Invalid capability')


def _receipt(directory, meta):
    return {k: v for k, v in meta.items() if k != 'token_hash'} | {'payload_dir': str(directory / 'payload')}


def _get(path, rootmeta, run_id, token):
    directory = path / 'runs' / _id(run_id); meta = _record(directory, rootmeta); _auth(meta, token)
    return directory, meta


def _live(directory, meta, *, size=None):
    now = time.time()
    if meta['state'] != 'active' or now >= min(_seconds(meta['expires_at']), _seconds(meta['lease_expires_at'])):
        raise ArtifactError('Run lease or absolute TTL expired')
    _plain(directory / 'payload', directory=True)
    if size is None:
        size = _size(directory / 'payload', payloads=(directory / 'payload',))
    if size < 0:
        raise ArtifactError('Payload absent from root accounting')
    if size > meta['size_budget_bytes']:
        raise ArtifactError('Run size budget exceeded')
    return size


def _metadata_temps(path, records):
    result = []
    for directory in (path, *(d for d, _ in records)):
        for item in directory.glob('.metadata-*'):
            info = _plain(item)
            if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1 or info.st_size > METADATA_LIMIT:
                raise ArtifactError('Unsafe interrupted metadata file')
            result.append(item)
    return result


def _admit(path, rootmeta, budget, new_record=True, *, payload_sizes=None):
    records, stages = _inventory(path, rootmeta)
    if stages or _metadata_temps(path, records):
        raise ArtifactError('Interrupted manager transaction; sweep first')
    reserved = ROOT_METADATA_RESERVE + len(records) * RUN_METADATA_RESERVE
    reserved += sum(m['size_budget_bytes'] for _, m in records if m['state'] in ('active', 'retained'))
    if new_record:
        reserved += RUN_METADATA_RESERVE
    if reserved + budget > rootmeta['total_budget_bytes']:
        raise ArtifactError('Root reservation budget exceeded')
    _root_size(path, rootmeta, payload_sizes=payload_sizes)


def _root_size(path, rootmeta, *, payload_sizes=None):
    records, stages = _inventory(path, rootmeta)
    payloads = tuple(d / 'payload' for d in (*stages, *(d for d, _ in records)))
    size = _size(path, payloads=payloads, _observed=payload_sizes)
    if size > rootmeta['total_budget_bytes']:
        raise ArtifactError('Root actual size budget exceeded')
    return size


def _new(path, rootmeta, work_item, source_sha, owner, ttl, budget, lease, token, state):
    now = time.time(); run_id = str(uuid.uuid4())
    meta = {'schema': 'validation-artifact-v1', 'root_id': rootmeta['root_id'], 'run_id': run_id,
            'work_item': work_item, 'source_sha': source_sha, 'owner': owner,
            'created_at': _stamp(now), 'expires_at': _stamp(now + ttl), 'size_budget_bytes': budget,
            'state': state, 'lease_expires_at': _stamp(now + min(ttl, lease)),
            'token_hash': hashlib.sha256(token.encode()).hexdigest()}
    directory = path / 'runs' / ('.new-' + run_id); directory.mkdir(mode=0o700)
    try:
        _write(directory / 'run.json', meta); (directory / 'payload').mkdir(mode=0o700)
    except BaseException:
        _remove(directory); raise
    return directory, meta


def create(root, work_item, source_sha, owner, ttl_seconds, budget_bytes, lease_seconds=MAX_LEASE):
    _text(work_item, 'work item'); _text(owner, 'owner')
    if not isinstance(source_sha, str) or not re.fullmatch(r'[0-9a-fA-F]{40}', source_sha):
        raise ArtifactError('Source SHA must be full 40 hexadecimal characters')
    _positive(ttl_seconds, 'TTL', MAX_TTL); _positive(budget_bytes, 'run budget'); _positive(lease_seconds, 'lease', MAX_LEASE)
    with _locked(root, 'create') as (path, meta):
        _admit(path, meta, budget_bytes); token = secrets.token_urlsafe(32)
        stage, run = _new(path, meta, work_item, source_sha, owner, ttl_seconds, budget_bytes, lease_seconds, token, 'active')
        directory = stage.with_name(run['run_id']); stage.rename(directory)
        return _receipt(directory, run) | {'token': token, 'renew_interval_seconds': RENEW_INTERVAL}


def check(root, run_id, token):
    with _locked(root, 'check') as (path, meta):
        directory, run = _get(path, meta, run_id, token)
        sizes = {}
        _root_size(path, meta, payload_sizes=sizes)
        size = _live(directory, run, size=sizes.get(directory / 'payload', -1))
        return _receipt(directory, run) | {'actual_bytes': size}


def renew(root, run_id, token, lease_seconds=MAX_LEASE):
    _positive(lease_seconds, 'lease', MAX_LEASE)
    with _locked(root, 'renew') as (path, meta):
        directory, run = _get(path, meta, run_id, token)
        sizes = {}
        _admit(path, meta, 0, new_record=False, payload_sizes=sizes)
        size = _live(directory, run, size=sizes.get(directory / 'payload', -1))
        run['lease_expires_at'] = _stamp(min(time.time() + lease_seconds, _seconds(run['expires_at'])))
        _write(directory / 'run.json', run)
        return _receipt(directory, run) | {'actual_bytes': size}


def finish(root, run_id, token, outcome):
    if outcome not in ('completed', 'failed', 'interrupted'):
        raise ArtifactError('Invalid terminal outcome')
    with _locked(root, 'finish') as (path, meta):
        directory, run = _get(path, meta, run_id, token)
        if run['state'] == 'retained':
            raise ArtifactError('Cannot finish a retained promotion')
        run.update(state=outcome, lease_expires_at=_stamp(min(time.time(), _seconds(run['expires_at']))))
        receipt = _receipt(directory, run) | {'actual_bytes': 0}
        # Authenticate even after expiry/overbudget, then discard all unpromoted
        # bytes including the record. The caller retains the returned receipt.
        _remove(directory)
        return receipt


def _selected(payload, paths):
    if not paths:
        raise ArtifactError('Promotion requires explicit relative paths')
    selected = []
    for value in paths:
        if not isinstance(value, str) or '\\' in value or ':' in value or '\0' in value:
            raise ArtifactError('Invalid promotion path')
        relative = PurePosixPath(value)
        if relative.is_absolute() or any(p in ('', '.', '..') for p in value.split('/')):
            raise ArtifactError('Promotion path must be relative and confined')
        target = payload
        for part in relative.parts:
            target /= part; _plain(target)
        if not target.is_file() or target.stat().st_nlink != 1:
            raise ArtifactError('Select individual regular files, without links')
        if value in selected:
            raise ArtifactError('Duplicate promotion path')
        selected.append(value)
    return selected


def _payload_identity(payload):
    info = _plain(payload, directory=True)
    return info.st_dev, info.st_ino


def _verify_payload(payload, identity):
    if _payload_identity(payload) != identity:
        raise ArtifactError('Payload root replaced during promotion')


def _open_selected(payload, relative, identity, canonical):
    _verify_payload(payload, identity)
    parts = PurePosixPath(relative).parts
    if os.open in os.supports_dir_fd and hasattr(os, 'O_NOFOLLOW'):
        directory = os.open(payload, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
        try:
            opened = os.fstat(directory)
            if (opened.st_dev, opened.st_ino) != identity:
                raise ArtifactError('Opened payload root differs')
            for part in parts[:-1]:
                child = os.open(part, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=directory)
                os.close(directory); directory = child
            return os.open(parts[-1], os.O_RDONLY | os.O_NOFOLLOW, dir_fd=directory)
        finally:
            os.close(directory)
    if os.name != 'nt':
        raise ArtifactError('Safe relative file opening unavailable')
    # Verify the actual opened Windows handle before reading any bytes. This
    # rejects an intermediate junction redirected outside the owned payload.
    import ctypes
    from ctypes import wintypes
    import msvcrt
    fd = os.open(payload / relative, os.O_RDONLY | os.O_BINARY)
    try:
        kernel = ctypes.WinDLL('kernel32', use_last_error=True)
        final_name = kernel.GetFinalPathNameByHandleW
        final_name.argtypes = [wintypes.HANDLE, wintypes.LPWSTR, wintypes.DWORD, wintypes.DWORD]
        final_name.restype = wintypes.DWORD
        buffer = ctypes.create_unicode_buffer(32768)
        length = final_name(msvcrt.get_osfhandle(fd), buffer, len(buffer), 0)
        if not length or length >= len(buffer):
            raise ArtifactError('Cannot verify opened promotion file')
        actual = os.path.normcase(buffer.value.removeprefix('\\\\?\\'))
        expected = os.path.normcase(str(canonical))
        if os.path.commonpath([actual, expected]) != expected:
            raise ArtifactError('Opened promotion file escapes payload')
        _verify_payload(payload, identity)
        _selected(payload, [relative])
        return fd
    except BaseException:
        os.close(fd); raise


def promote(root, run_id, token, owner, ttl_seconds, budget_bytes, paths):
    _text(owner, 'retention owner'); _positive(ttl_seconds, 'retention TTL', MAX_TTL); _positive(budget_bytes, 'retention budget')
    with _locked(root, 'promote') as (path, meta):
        directory, run = _get(path, meta, run_id, token); _live(directory, run)
        payload = directory / 'payload'
        identity = _payload_identity(payload); canonical = payload.resolve(strict=True)
        _verify_payload(payload, identity)
        selected = _selected(payload, paths); _verify_payload(payload, identity)
        _admit(path, meta, budget_bytes)
        stage, retained = _new(path, meta, run['work_item'], run['source_sha'], owner, ttl_seconds, budget_bytes, min(MAX_LEASE, ttl_seconds), token, 'retained')
        try:
            used = 0; private_token = token.encode()
            for value in selected:
                destination = stage / 'payload' / value
                destination.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
                fd = _open_selected(payload, value, identity, canonical)
                with os.fdopen(fd, 'rb') as reader, destination.open('xb') as writer:
                    info = os.fstat(reader.fileno())
                    if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
                        raise ArtifactError('Promotion source changed')
                    overlap = b''
                    while chunk := reader.read(65536):
                        if private_token in overlap + chunk:
                            raise ArtifactError('Promotion contains private capability')
                        overlap = (overlap + chunk)[-(len(private_token) - 1):]
                        used += len(chunk)
                        if used > budget_bytes:
                            raise ArtifactError('Promotion size budget exceeded')
                        writer.write(chunk)
                _verify_payload(payload, identity)
            final = stage.with_name(retained['run_id']); stage.rename(final)
        except BaseException:
            _remove(stage); raise
        return _receipt(final, retained) | {'actual_bytes': used, 'selected_paths': selected}


def sweep(root, dry_run=False, *, expected_root_id=None):
    with _locked(root, 'sweep') as (path, meta):
        if expected_root_id is not None and meta['root_id'] != expected_root_id:
            raise ArtifactError('Managed root identity changed')
        records, stages = _inventory(path, meta); now = time.time(); deleted = []; reclaimed = 0
        for directory, run in records:
            expired = now >= _seconds(run['expires_at'])
            abandoned = run['state'] == 'active' and now >= _seconds(run['lease_expires_at'])
            if expired or abandoned or run['state'] in ('completed', 'failed', 'interrupted'):
                reclaimed += _size(directory, payloads=(directory / 'payload',)); deleted.append(run['run_id'])
                if not dry_run:
                    _remove(directory)
        live_records = [(d, r) for d, r in records if r['run_id'] not in deleted]
        for temporary in _metadata_temps(path, live_records):
            if temporary.exists():
                reclaimed += temporary.stat().st_size
                if not dry_run:
                    temporary.unlink()
        # Staging directories are never returned to producers; a held OS root lock
        # proves no other manager transaction is still creating/copying them.
        for stage in stages:
            reclaimed += _size(stage, payloads=(stage / 'payload',)); deleted.append(stage.name)
            if not dry_run:
                _remove(stage)
        return {'schema': 'validation-sweep-v1', 'dry_run': dry_run,
                'removed_run_ids': deleted, 'reclaimed_bytes': reclaimed}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('init', 'create', 'renew', 'check', 'finish', 'promote', 'sweep'))
    parser.add_argument('--root', required=True); parser.add_argument('--run-id'); parser.add_argument('--token')
    parser.add_argument('--budget-bytes', type=int); parser.add_argument('--ttl-seconds', type=int)
    parser.add_argument('--lease-seconds', type=int, default=MAX_LEASE)
    parser.add_argument('--work-item'); parser.add_argument('--source-sha'); parser.add_argument('--owner')
    parser.add_argument('--outcome'); parser.add_argument('--path', action='append', dest='paths')
    parser.add_argument('--dry-run', action='store_true'); args = vars(parser.parse_args(argv)); command = args.pop('command')
    names = {'init': ('root', 'budget_bytes'), 'create': ('root', 'work_item', 'source_sha', 'owner', 'ttl_seconds', 'budget_bytes', 'lease_seconds'),
             'renew': ('root', 'run_id', 'token', 'lease_seconds'), 'check': ('root', 'run_id', 'token'),
             'finish': ('root', 'run_id', 'token', 'outcome'), 'promote': ('root', 'run_id', 'token', 'owner', 'ttl_seconds', 'budget_bytes', 'paths'), 'sweep': ('root', 'dry_run')}
    try:
        receipt = globals()[command](**{k: args[k] for k in names[command]})
        print(json.dumps(receipt, sort_keys=True)); return 0
    except (ArtifactError, OSError, ValueError, KeyError, TypeError) as error:
        print(json.dumps({'error': type(error).__name__, 'message': str(error)})); return 1


if __name__ == '__main__':
    raise SystemExit(main())
