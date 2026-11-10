import contextlib
import ctypes
import importlib.util
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import types
import unittest
from unittest.mock import patch

SCRIPT = Path(__file__).resolve().parents[1] / 'validation_artifacts.py'
SPEC = importlib.util.spec_from_file_location('validation_artifacts', SCRIPT)
a = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(a)


class Artifacts(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='artifact-contract-')
        self.base = Path(self.tmp.name).resolve()
        self.root = self.base / 'managed'
        a.init(self.root, 1000)

    def tearDown(self):
        self.tmp.cleanup()

    def create(self, budget=400, ttl=120, lease=60):
        return a.create(self.root, 'M001', 'a' * 40, 'test-owner', ttl, budget, lease)

    def call(self, function, run, *args, **kwargs):
        return function(self.root, run['run_id'], run['token'], *args, **kwargs)

    def test_create_check_renew_token_private_metadata(self):
        run = self.create()
        Path(run['payload_dir'], 'data').write_bytes(b'abc')
        self.assertEqual(self.call(a.check, run)['actual_bytes'], 3)
        receipt = self.call(a.renew, run)
        self.assertNotIn('token', receipt)
        self.assertNotIn('token_hash', receipt)
        meta = Path(run['payload_dir']).parent / 'run.json'
        self.assertNotIn(run['token'], meta.read_text())
        self.assertLessEqual(a._seconds(receipt['lease_expires_at']), a._seconds(run['expires_at']))

    def test_all_finish_outcomes_delete_payload(self):
        for outcome in ('completed', 'failed', 'interrupted'):
            with self.subTest(outcome=outcome):
                run = self.create()
                Path(run['payload_dir'], 'unpromoted').write_text('discard')
                self.call(a.finish, run, outcome)
                self.assertFalse(Path(run['payload_dir']).exists())
        self.assertEqual(len(a.sweep(self.root)['removed_run_ids']), 3)

    def test_finish_after_expiry_and_overbudget_still_authenticates(self):
        run = self.create(budget=1, ttl=1)
        Path(run['payload_dir'], 'large').write_bytes(b'too large')
        with patch.object(a.time, 'time', return_value=time.time() + 100):
            self.assertRaises(a.ArtifactError, self.call, a.check, run)
            self.assertRaises(a.ArtifactError, a.finish, self.root, run['run_id'], 'wrong', 'failed')
            self.assertTrue(Path(run['payload_dir']).exists())
            self.call(a.finish, run, 'interrupted')
            self.assertFalse(Path(run['payload_dir']).exists())
            self.assertEqual(a.sweep(self.root)['removed_run_ids'], [run['run_id']])

    def test_expired_lease_not_resurrected_or_swept_while_valid(self):
        run = self.create(lease=1)
        self.assertEqual(a.sweep(self.root)['removed_run_ids'], [])
        with patch.object(a.time, 'time', return_value=time.time() + 2):
            self.assertRaises(a.ArtifactError, self.call, a.renew, run)
            self.assertEqual(a.sweep(self.root)['removed_run_ids'], [run['run_id']])

    def test_absolute_ttl_and_lease_caps(self):
        for kwargs in ({'ttl': a.MAX_TTL + 1}, {'lease': 61}, {'budget': 0}):
            self.assertRaises(a.ArtifactError, self.create, **kwargs)
        run = self.create(ttl=10)
        self.assertEqual(run['expires_at'], run['lease_expires_at'])
        with patch.object(a.time, 'time', return_value=time.time() + 9):
            self.assertEqual(self.call(a.renew, run)['lease_expires_at'], run['expires_at'])
        self.assertRaises(a.ArtifactError, a.create, self.root, '', 'bad', '', 10, 1)

    def test_root_budget_and_size_check(self):
        run = self.create(budget=800)
        self.assertRaises(a.ArtifactError, self.create, budget=201)
        Path(run['payload_dir'], 'large').write_bytes(b'x' * 801)
        self.assertRaises(a.ArtifactError, self.call, a.check, run)
        self.assertRaises(a.ArtifactError, self.call, a.renew, run)
        self.call(a.finish, run, 'failed')
        self.assertTrue(self.create(budget=1000))

    def test_promote_selected_files_retains_owner_expiry_and_budget(self):
        run = self.create()
        payload = Path(run['payload_dir'])
        (payload / 'summary').write_text('summary')
        (payload / 'discard').write_text('large private payload')
        retained = self.call(a.promote, run, 'review-owner', 10, 100, ['summary'])
        self.assertEqual(retained['state'], 'retained')
        self.assertEqual(retained['owner'], 'review-owner')
        self.assertNotIn('token', retained)
        self.call(a.finish, run, 'completed')
        self.assertFalse(payload.exists())
        self.assertEqual(Path(retained['payload_dir'], 'summary').read_text(), 'summary')
        self.assertFalse(Path(retained['payload_dir'], 'discard').exists())
        self.assertRaises(a.ArtifactError, self.create, budget=901)
        with patch.object(a.time, 'time', return_value=time.time() + 11):
            a.sweep(self.root)
        self.assertFalse(Path(retained['payload_dir']).exists())

    def test_promotion_budget_and_selection_rejection(self):
        run = self.create()
        Path(run['payload_dir'], 'file').write_text('four')
        for paths in ([], ['../outside'], ['/absolute'], ['file', 'file'], ['a\\b'], ['C:drive'], ['file:stream'], ['.']):
            self.assertRaises(a.ArtifactError, self.call, a.promote, run, 'owner', 10, 20, paths)
        self.assertRaises(a.ArtifactError, self.call, a.promote, run, 'owner', 10, 3, ['file'])
        self.assertEqual(len(list((self.root / 'runs').iterdir())), 1)
        self.assertRaises(a.ArtifactError, self.call, a.promote, run, 'owner', 10, 601, ['file'])

    def test_internal_symlink_not_followed_or_promoted(self):
        outside = self.base / 'protected'
        outside.mkdir(); (outside / 'keep').write_text('keep')
        run = self.create()
        Path(run['payload_dir'], 'link').symlink_to(outside, target_is_directory=True)
        self.assertRaises(a.ArtifactError, self.call, a.promote, run, 'owner', 10, 50, ['link/keep'])
        self.call(a.finish, run, 'failed')
        self.assertEqual((outside / 'keep').read_text(), 'keep')

    def test_promotion_rejects_parent_swap_after_selection(self):
        run = self.create(); payload = Path(run['payload_dir'])
        (payload / 'sub').mkdir(); (payload / 'sub/file').write_text('owned')
        outside = self.base / 'external'; outside.mkdir(); (outside / 'file').write_text('protected')
        original = a._selected
        def swap(*args):
            selected = original(*args)
            (payload / 'sub').rename(payload / 'saved')
            (payload / 'sub').symlink_to(outside, target_is_directory=True)
            return selected
        with patch.object(a, '_selected', side_effect=swap):
            self.assertRaises((a.ArtifactError, OSError), self.call, a.promote, run, 'owner', 10, 100, ['sub/file'])
        self.assertEqual((outside / 'file').read_text(), 'protected')
        self.assertEqual(len(list((self.root / 'runs').iterdir())), 1)

    def test_promotion_rejects_entire_payload_root_swap(self):
        run = self.create(); payload = Path(run['payload_dir'])
        (payload / 'file').write_text('owned')
        outside = self.base / 'external'; outside.mkdir(); (outside / 'file').write_text('protected')
        original = a._selected
        def swap(*args):
            selected = original(*args)
            payload.rename(self.base / 'original-payload')
            payload.symlink_to(outside, target_is_directory=True)
            return selected
        with patch.object(a, '_selected', side_effect=swap):
            self.assertRaises((a.ArtifactError, OSError), self.call, a.promote, run, 'owner', 10, 100, ['file'])
        self.assertEqual((outside / 'file').read_text(), 'protected')
        self.assertEqual(len(list((self.root / 'runs').iterdir())), 1)

    def test_promotion_rejects_private_capability_without_rewriting(self):
        run = self.create(); summary = Path(run['payload_dir'], 'summary')
        summary.write_text('token=' + run['token'])
        original = summary.read_bytes()
        with self.assertRaisesRegex(a.ArtifactError, 'private capability'):
            self.call(a.promote, run, 'owner', 10, 100, ['summary'])
        self.assertEqual(summary.read_bytes(), original)
        self.assertEqual(len(list((self.root / 'runs').iterdir())), 1)

    def test_promotion_detects_private_capability_across_copy_chunks(self):
        self.root = self.base / 'large-managed'; a.init(self.root, 200000)
        run = self.create(budget=70000); summary = Path(run['payload_dir'], 'summary')
        summary.write_bytes(b'x' * (65536 - 20) + run['token'].encode() + b'end')
        with self.assertRaisesRegex(a.ArtifactError, 'private capability'):
            self.call(a.promote, run, 'owner', 10, 70000, ['summary'])
        self.assertEqual(len(list((self.root / 'runs').iterdir())), 1)

    def test_windows_handle_rejects_root_swap_after_initial_identity_check(self):
        run = self.create(); payload = Path(run['payload_dir'])
        (payload / 'file').write_text('owned')
        outside = self.base / 'external'; outside.mkdir(); (outside / 'file').write_text('protected')
        identity = a._payload_identity(payload); canonical = payload.resolve()
        original_open = os.open
        def swapped_open(*args, **kwargs):
            payload.rename(self.base / 'original-payload')
            payload.symlink_to(outside, target_is_directory=True)
            return original_open(*args, **kwargs)
        def final_name(handle, buffer, length, flags):
            buffer.value = str(outside / 'file')
            return len(buffer.value)
        from unittest.mock import Mock
        kernel = types.SimpleNamespace(GetFinalPathNameByHandleW=Mock(side_effect=final_name))
        with patch.object(a.os, 'supports_dir_fd', set()), patch.object(a.os, 'name', 'nt'), \
                patch.object(a.os, 'O_BINARY', 0, create=True), \
                patch.object(a.os, 'open', side_effect=swapped_open), \
                patch.object(ctypes, 'WinDLL', return_value=kernel, create=True), \
                patch.dict(sys.modules, {'msvcrt': types.SimpleNamespace(get_osfhandle=lambda fd: fd)}):
            with self.assertRaisesRegex(a.ArtifactError, 'escapes payload'):
                a._open_selected(payload, 'file', identity, canonical)
        self.assertEqual((outside / 'file').read_text(), 'protected')

    def test_root_run_and_ancestor_symlinks_rejected(self):
        alias = self.base / 'alias'; alias.symlink_to(self.root, target_is_directory=True)
        self.assertRaises(a.ArtifactError, a.sweep, alias)
        self.assertRaises(a.ArtifactError, a.init, alias / 'child', 10)
        run = self.create(); directory = Path(run['payload_dir']).parent
        saved = self.base / 'saved'; directory.rename(saved); directory.symlink_to(saved, target_is_directory=True)
        self.assertRaises(a.ArtifactError, self.call, a.check, run)
        self.assertRaises(a.ArtifactError, a.sweep, self.root)
        self.assertTrue((saved / 'run.json').exists())

    def test_unmanaged_existing_root_protected(self):
        unmanaged = self.base / 'unmanaged'; unmanaged.mkdir(mode=0o700)
        keep = unmanaged / 'keep'; keep.write_text('keep')
        self.assertRaises((a.ArtifactError, OSError), a.init, unmanaged, 100)
        self.assertEqual(keep.read_text(), 'keep')
        (self.root / 'unknown').write_text('also protected')
        self.assertRaises(a.ArtifactError, a.sweep, self.root)

    def test_dry_run_no_file_or_metadata_changes(self):
        run = self.create(ttl=1); Path(run['payload_dir'], 'data').write_text('data')
        before = {str(p): (p.read_bytes(), p.stat().st_mtime_ns) for p in self.root.rglob('*') if p.is_file()}
        with patch.object(a.time, 'time', return_value=time.time() + 2):
            result = a.sweep(self.root, dry_run=True)
        after = {str(p): (p.read_bytes(), p.stat().st_mtime_ns) for p in self.root.rglob('*') if p.is_file()}
        self.assertEqual(before, after)
        self.assertEqual(result['removed_run_ids'], [run['run_id']])
        self.assertEqual(result['reclaimed_bytes'], 4)

    def test_concurrent_reservation_only_one_admitted(self):
        command = [sys.executable, '-B', str(SCRIPT), 'create', '--root', str(self.root), '--work-item', 'parallel',
                   '--source-sha', 'b' * 40, '--owner', 'test', '--ttl-seconds', '30', '--budget-bytes', '600']
        children = [subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE) for _ in range(4)]
        results = [(p, p.communicate(timeout=15)) for p in children]
        self.assertEqual(sum(p.returncode == 0 for p, _ in results), 1)
        self.assertEqual(len(list((self.root / 'runs').iterdir())), 1)

    def test_os_lock_released_after_controller_interrupt(self):
        code = "import sys;sys.path.insert(0,sys.argv[1]);import validation_artifacts as a;\nwith a._locked(sys.argv[2]):\n print('locked',flush=True);__import__('time').sleep(60)"
        child = subprocess.Popen([sys.executable, '-B', '-c', code, str(SCRIPT.parent), str(self.root)], stdout=subprocess.PIPE)
        try:
            self.assertEqual(child.stdout.readline().strip(), b'locked')
            child.kill(); child.wait(timeout=5)
            self.assertTrue(self.create())
        finally:
            if child.poll() is None:
                child.kill(); child.wait(timeout=5)
            child.stdout.close()

    def test_interrupted_manager_stage_collected_under_lock(self):
        stage = self.root / 'runs' / ('.new-' + str(a.uuid.uuid4()))
        stage.mkdir(); (stage / 'partial').write_text('partial')
        self.assertRaises(a.ArtifactError, self.create)
        self.assertEqual(a.sweep(self.root)['reclaimed_bytes'], 7)
        self.assertFalse(stage.exists())
        self.assertTrue(self.create())

    def test_atomic_metadata_replace_failure_preserves_old_record(self):
        run = self.create(); metadata = Path(run['payload_dir']).parent / 'run.json'
        original = metadata.read_bytes()
        with patch.object(a.os, 'replace', side_effect=OSError('owned injected failure')):
            self.assertRaises(OSError, self.call, a.renew, run)
        self.assertEqual(metadata.read_bytes(), original)
        self.assertFalse(list(metadata.parent.glob('.metadata-*')))
        self.assertTrue(self.call(a.check, run))

    def test_payload_root_symlink_gc_unlinks_without_touching_target(self):
        run = self.create(ttl=1); payload = Path(run['payload_dir']); payload.rmdir()
        outside = self.base / 'kept'; outside.mkdir(); (outside / 'file').write_text('keep')
        payload.symlink_to(outside, target_is_directory=True)
        self.assertRaises(a.ArtifactError, self.call, a.check, run)
        with patch.object(a.time, 'time', return_value=time.time() + 2):
            a.sweep(self.root)
        self.assertEqual((outside / 'file').read_text(), 'keep')

    def test_cli_json_error_and_capability_not_echoed(self):
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            rc = a.main(['check', '--root', str(self.root), '--run-id', '../bad', '--token', 'SECRET'])
        self.assertEqual(rc, 1)
        self.assertNotIn('SECRET', output.getvalue())
        self.assertIn('error', json.loads(output.getvalue()))


if __name__ == '__main__':
    unittest.main()
