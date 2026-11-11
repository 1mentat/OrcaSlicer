import contextlib
import io
import json
import os
from pathlib import Path
import plistlib
import shutil
import subprocess
import sys
import tempfile
import time
import types
import unittest
from unittest.mock import patch

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))
import validation_artifacts as manager
import validation_sweeper as sweeper


class Sweeper(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='sweeper-test-')
        self.base = Path(self.temp.name).resolve()
        self.root = self.base / 'root'; manager.init(self.root, 50000)
        self.home = self.base / 'home'; self.home.mkdir(mode=0o700)
        self.repo = self.base / 'repository'; (self.repo / 'scripts').mkdir(parents=True)
        for name in sweeper.FILES:
            shutil.copyfile(SCRIPTS / name, self.repo / 'scripts' / name)
        self.env = {k: v for k, v in os.environ.items() if not k.upper().startswith('GIT_')}
        self.git('init', '-q')
        self.git('config', 'core.autocrlf', 'false')
        self.git('add', 'scripts')
        self.git('-c', 'user.name=Fixture', '-c', 'user.email=fixture@example.invalid', 'commit', '-qm', 'source')
        self.sha = self.git('rev-parse', 'HEAD').strip().decode()
        for name in sweeper.FILES:
            self.assertEqual(self.git('cat-file', 'blob', self.sha + ':scripts/' + name),
                             (SCRIPTS / name).read_bytes())
        self.calls = []

    def tearDown(self):
        self.temp.cleanup()

    def git(self, *args):
        return subprocess.run(['git', '-C', str(self.repo), *args], check=True, capture_output=True,
                              env=self.env, timeout=10).stdout

    def launch(self, argv):
        self.calls.append(argv)
        return types.SimpleNamespace(returncode=0)

    def install(self, **kwargs):
        with patch.object(sweeper.sys, 'platform', 'darwin'), patch.object(sweeper, '_launch_domain', return_value='gui/123'):
            return sweeper.install_macos([str(self.root)], str(self.repo), self.sha, 'fixture-owner',
                                         home=self.home, bootstrap=kwargs.pop('bootstrap', self.launch), **kwargs)

    def config(self):
        return self.home / 'Library/Application Support/OrcaSlicer/validation-tools/config.json'

    def test_sweep_portable_bounded_counts_and_dryrun(self):
        run = manager.create(self.root, 'fixture', 'a' * 40, 'owner', 1, 10)
        Path(run['payload_dir'], 'data').write_bytes(b'data')
        with patch.object(sweeper.time, 'time', return_value=time.time() + 2):
            dry = sweeper.sweep([str(self.root)], dry_run=True)
            self.assertEqual(dry['removed_records'], 1)
            self.assertTrue(Path(run['payload_dir']).exists())
            result = sweeper.sweep([str(self.root)])
        self.assertEqual(result['removed_records'], 1)
        self.assertGreater(result['reclaimed_bytes'], 4)
        self.assertNotIn(run['run_id'], json.dumps(result))
        self.assertFalse(Path(run['payload_dir']).exists())

    def test_install_exact_blobs_private_stable_files_and_launchctl_arrays(self):
        result = self.install()
        self.assertTrue(result['installed'])
        tools = self.config().parent
        self.assertEqual({p.name for p in tools.iterdir()}, {*sweeper.FILES, 'config.json'})
        for name in sweeper.FILES:
            self.assertEqual((tools / name).read_bytes(), (SCRIPTS / name).read_bytes())
            if os.name != 'nt':
                self.assertEqual((tools / name).stat().st_mode & 0o777, 0o600)
        self.assertLessEqual(self.config().stat().st_size, sweeper.LIMIT)
        value = json.loads(self.config().read_text())
        self.assertEqual(value['root_ids'][str(self.root)], manager._read(self.root / 'root.json')['root_id'])
        plist = self.home / 'Library/LaunchAgents' / (sweeper.LABEL + '.plist')
        agent = plistlib.loads(plist.read_bytes())
        self.assertEqual(agent['StartInterval'], 300); self.assertTrue(agent['RunAtLoad'])
        self.assertEqual(agent['StandardOutPath'], '/dev/null'); self.assertEqual(agent['StandardErrorPath'], '/dev/null')
        self.assertEqual(agent['ProgramArguments'], [sys.executable, '-B', str(tools / sweeper.FILES[1]), 'sweep',
                         '--config', str(self.config()), '--root', str(self.root)])
        self.assertEqual(self.calls, [['/bin/launchctl', 'bootout', 'gui/123/' + sweeper.LABEL],
                                     ['/bin/launchctl', 'bootstrap', 'gui/123', str(plist)]])
        self.install(interval_seconds=60)
        self.assertEqual(len(list(tools.iterdir())), 3)

    def test_git_environment_scrub_and_mismatch_before_any_install(self):
        with patch.dict(os.environ, {'GIT_DIR': '/does/not/exist', 'GIT_WORK_TREE': '/wrong'}):
            self.install()
        self.calls.clear()
        (self.repo / 'scripts/validation_artifacts.py').write_text('wrong')
        self.git('add', 'scripts'); self.git('-c', 'user.name=Fixture', '-c', 'user.email=fixture@example.invalid', 'commit', '-qm', 'different')
        self.sha = self.git('rev-parse', 'HEAD').strip().decode()
        self.assertRaises(manager.ArtifactError, self.install)
        self.assertEqual(self.calls, [])

    def test_unmanaged_roots_installations_and_symlink_paths_rejected(self):
        other = self.base / 'unmanaged'; other.mkdir(mode=0o700); (other / 'keep').write_text('keep')
        with patch.object(sweeper.sys, 'platform', 'darwin'):
            self.assertRaises(manager.ArtifactError, sweeper.install_macos, [str(other)], str(self.repo), self.sha,
                              'fixture-owner', home=self.home, bootstrap=self.launch)
        self.assertEqual(self.calls, []); self.assertEqual((other / 'keep').read_text(), 'keep')
        library = self.home / 'Library'; library.symlink_to(self.base / 'repository', target_is_directory=True)
        self.assertRaises(manager.ArtifactError, self.install)
        self.assertEqual(self.calls, [])

    def test_existing_unmanaged_agent_and_different_owner_preserved(self):
        self.install(); before = self.config().read_bytes()
        with patch.object(sweeper.sys, 'platform', 'darwin'):
            self.assertRaises(manager.ArtifactError, sweeper.install_macos, [str(self.root)], str(self.repo),
                              self.sha, 'different-owner', home=self.home, bootstrap=self.launch)
        self.assertEqual(self.config().read_bytes(), before)
        shutil.rmtree(self.config().parent)
        self.assertRaises(manager.ArtifactError, self.install)

    def test_bootstrap_failure_never_claims_installed(self):
        def fail(argv):
            self.calls.append(argv)
            return types.SimpleNamespace(returncode=5)
        self.assertRaises(manager.ArtifactError, self.install, bootstrap=fail)
        self.assertEqual(len(self.calls), 2)
        self.assertTrue(self.config().exists())

    def test_root_uuid_replacement_rejected_but_old_schedule_keeps_collecting(self):
        self.install()
        with patch.object(sweeper.time, 'time', return_value=time.time() + 90 * 86400):
            self.assertEqual(sweeper.sweep([str(self.root)], config=self.config())['errors'], 0)
        metadata = manager._read(self.root / 'root.json'); metadata['root_id'] = str(manager.uuid.uuid4())
        manager._write(self.root / 'root.json', metadata, manager.ROOT_METADATA_LIMIT)
        self.assertEqual(sweeper.sweep([str(self.root)], config=self.config())['errors'], 1)

    def test_installed_destination_links_rejected_without_touching_target(self):
        self.install(); source = self.config().parent / sweeper.FILES[0]
        outside = self.base / 'protected'; outside.write_text('keep')
        source.unlink(); source.symlink_to(outside)
        self.calls.clear()
        self.assertRaises(manager.ArtifactError, self.install)
        self.assertEqual(outside.read_text(), 'keep'); self.assertEqual(self.calls, [])

    def test_bad_interval_source_identity_and_platform_precede_install(self):
        for interval in (59, 3601, True):
            self.assertRaises(manager.ArtifactError, self.install, interval_seconds=interval)
        with patch.object(sweeper.sys, 'platform', 'linux'):
            self.assertRaises(manager.ArtifactError, sweeper.install_macos, [str(self.root)],
                              str(self.repo), self.sha, 'owner', home=self.home, bootstrap=self.launch)
        with patch.object(sweeper.sys, 'platform', 'darwin'):
            self.assertRaises(manager.ArtifactError, sweeper.install_macos, [str(self.root)],
                              str(self.repo), 'HEAD', 'owner', home=self.home, bootstrap=self.launch)
        self.assertEqual(self.calls, [])
        self.assertFalse((self.home / 'Library').exists())

    def test_missing_configured_identity_cannot_degrade_to_unpinned_sweep(self):
        self.install()
        value = json.loads(self.config().read_text()); value['root_ids'][str(self.root)] = None
        manager._write(self.config(), value, sweeper.LIMIT)
        with patch.object(sweeper.manager, 'sweep') as call:
            self.assertRaises(manager.ArtifactError, sweeper.sweep, [str(self.root)], config=self.config())
            call.assert_not_called()

    @unittest.skipUnless(hasattr(os, 'getuid'), 'macOS/POSIX ownership permissions')
    def test_existing_owned_0755_launchagents_installs_without_chmod(self):
        agents = self.home / 'Library/LaunchAgents'; agents.mkdir(parents=True)
        agents.chmod(0o755)
        before = agents.stat().st_mode
        self.assertTrue(self.install()['installed'])
        self.assertEqual(agents.stat().st_mode, before)
        self.assertEqual(self.config().parent.stat().st_mode & 0o777, 0o700)

    @unittest.skipUnless(hasattr(os, 'getuid'), 'macOS/POSIX ownership permissions')
    def test_group_or_world_writable_launchagents_rejected_without_chmod(self):
        agents = self.home / 'Library/LaunchAgents'; agents.mkdir(parents=True)
        for mode in (0o775, 0o757, 0o777):
            with self.subTest(mode=oct(mode)):
                agents.chmod(mode)
                self.assertRaises(manager.ArtifactError, self.install)
                self.assertEqual(agents.stat().st_mode & 0o777, mode)
                self.assertEqual(self.calls, [])
                self.assertFalse(self.config().exists())

    @unittest.skipUnless(hasattr(os, 'getuid'), 'macOS/POSIX ownership permissions')
    def test_different_launchagents_owner_rejected(self):
        agents = self.home / 'Library/LaunchAgents'; agents.mkdir(parents=True)
        agents.chmod(0o755)
        plain = manager._plain
        def wrong_owner(path, directory=False):
            info = plain(path, directory)
            if path == agents:
                return types.SimpleNamespace(st_uid=os.getuid() + 1, st_mode=info.st_mode)
            return info
        with patch.object(manager, '_plain', side_effect=wrong_owner):
            self.assertRaises(manager.ArtifactError, self.install)
        self.assertEqual(self.calls, [])
        self.assertFalse(self.config().exists())

    def test_cli_rejects_home_override_and_reports_bounded_failure(self):
        with contextlib.redirect_stderr(io.StringIO()):
            self.assertRaises(SystemExit, sweeper.main, ['install-macos', '--root', str(self.root),
                              '--repository', str(self.repo), '--source-sha', self.sha, '--owner', 'x', '--home', str(self.home)])
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            rc = sweeper.main(['sweep', '--root', str(self.base / 'missing')])
        self.assertEqual(rc, 1); self.assertEqual(json.loads(output.getvalue())['errors'], 1)
        self.assertLess(len(output.getvalue()), 500)


if __name__ == '__main__':
    unittest.main()
