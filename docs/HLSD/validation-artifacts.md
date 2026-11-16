# Validation artifacts

Validation output is disposable data associated with a work item and an exact
source revision. Source, tests and reproduction scripts belong in Git. A result
summary belongs on its PR or CI run; compiler objects, executables, profiles and
bulk command logs do not belong in a permanent versioned evidence directory.

## Ownership and storage

`scripts/validation_artifacts.py` manages a private artifact store. Each run has
a UUID, work-item ID, full source SHA, owner, absolute expiry, byte reservation
and renewable lease. Store admission reserves the run's entire budget. Concurrent
reservations cannot exceed the store budget. A lease expires after at most sixty
seconds and cannot extend the run's absolute expiry. Retention is bounded to
thirty days.

Budgets include logical metadata bytes: admission reserves 1 KiB for the store
and 8 KiB per live or retained record, in addition to its payload budget. Finish
returns its terminal receipt and removes the entire record, so tiny completed
runs cannot accumulate uncharged history. Filesystem allocation overhead and
bytes written between guard polls are distinct from the logical reservation.

Metadata updates and reservation/deletion decisions hold an operating-system
file lock. Process termination releases the lock. Metadata is published
atomically. Capability tokens authenticate run operations; metadata stores their
hashes. Tokens must not be published with validation receipts.

Producers write only inside the returned `payload_dir`. They check size and
renew their lease during execution, stop owned processes if either check fails,
and finish in `finally`. Finishing discards the payload on success, failure and
interruption. A killed controller leaves an expiring lease; the host's periodic
`sweep` reclaims abandoned runs and expired retained data. Valid leases protect
active work, including queued handoffs between pipeline jobs.

The manager only removes marked directories beneath its own private store. It
rejects unmanaged roots, traversal and symlink escapes. Source checkouts and
shared dependency caches remain outside disposable payloads. A reusable cache
requires its own bounded owner and retention policy; calling a directory a cache
does not exempt it from storage accounting.

## Scheduled collection

`scripts/validation_sweeper.py` collects configured managed stores and returns
bounded counts, bytes and errors. On macOS its installer creates a per-user
LaunchAgent with a finite collection interval. The installer verifies its two
source modules against an explicit Git revision, then copies them into a private
stable tools directory. The installed configuration binds each store UUID;
collection checks that identity while holding the manager lock before deleting
anything. Replacing a path with a different store cannot authorize collection.

The service retains only bounded configuration and latest status, without
versioned logs. Collection continues independently of producer processes and
run expiry. A service configuration does not grant authority over legacy data,
source checkouts or dependency caches outside its configured managed stores.

## Command-line producers

`scripts/validation_runner.py` supervises commands through the existing scoped
process helper. Its guard checks payload size and lease validity during execution
and terminates the owned process tree on failure. It also checks final size,
including commands that finish between polling intervals. This is cooperative
producer enforcement with a bounded polling interval, not an operating-system
filesystem quota or a sandbox for arbitrary commands.

Initialize a store and run a command with explicit ownership and limits:

```sh
python scripts/validation_artifacts.py init --root /absolute/private/store --budget-bytes 8589934592
python scripts/validation_runner.py --root /absolute/private/store \
  --work-item PR-610 --source-sha FULL_40_CHARACTER_COMMIT_SHA --owner maintainer \
  --ttl-seconds 3600 --budget-bytes 1073741824 \
  --promote settings-inventory.json --retain-ttl-seconds 604800 --retain-budget-bytes 2097152 \
  -- python scripts/settings_inventory.py --root /absolute/source/checkout
```

Registered public writers require the supervisor's authenticated environment.
Their output paths must resolve beneath its payload and their source checkout
must match its source SHA. The environment also redirects temporary files into
the payload. Persistent-worktree modes need a separately managed cache adapter;
they cannot silently preserve a per-run build tree. Test fixtures use temporary,
owned stores without weakening production guards.

## Test-validation producers

The coverage, mutation, inventory, module-feedback and PR-validation scripts
(`slicer_coverage.py`, `slicer_coverage_pair_collector.py`, `slicer_catch_batch.py`,
`slicer_coverage_reconcile.py`, `slicer_mutation.py`, `slicer_module_feedback.py`,
`settings_inventory.py` and `slicer_pr_validation.py`) end in `managed_main`, which
refuses to run unless `scripts/validation_runner.py` supplied the artifact
environment. Only `--help`/`-h` and `slicer_mutation.py list-defaults` run without it.
The runner's `--source-sha` must equal `HEAD` of the checkout named by the producer's
`--root`. The producer's output option (`--out`, or `--output-dir` for
`slicer_pr_validation.py`) must resolve inside the leased payload and defaults to a
path there. Options that would keep a build or worktree beyond the run
(`--keep-worktree`, `--persistent-worktree`, `--reuse-detached-checkout`,
`--reuse-worktree-from-report`, `--seed-worktree`) are rejected. `--work-root`,
`--gcov-data-dir` and `--build` must be absolute paths inside the payload, and
`--build-dir` must be relative without `..`.

Settings inventory, which only reads sources:

```sh
python scripts/validation_runner.py --root /absolute/private/store \
  --work-item WORK_ITEM --source-sha FULL_40_CHARACTER_COMMIT_SHA --owner maintainer \
  --ttl-seconds 3600 --budget-bytes 1073741824 -- \
  python scripts/settings_inventory.py --root /absolute/source/checkout
```

Bounded mutation validation uses `slicer_mutation.py run` in explicit mode, with a
catalog from `tests/mutations/`. The checkout must be clean. The script configures and
builds inside its own disposable worktree, expanding `{root}`, `{build}` and `{filter}`
in the commands; with no `{filter}` in a CTest command it appends `-L <test_filter>`.
Registering focused tests with CTest needs `SLICER_ENABLE_FOCUSED_PR_CTEST_DISCOVERY`:

```sh
python scripts/validation_runner.py ...runner options as above... -- \
  python scripts/slicer_mutation.py run --root /absolute/source/checkout \
  --configure-command "cmake -S {root} -B {build} -DSLICER_ENABLE_FOCUSED_PR_CTEST_DISCOVERY=ON ..." \
  --build-command "cmake --build {build} --config Release --target fff_print_wipe_tests" \
  --test-command "ctest --test-dir {build} -C Release -L {filter} --no-tests=error" \
  --mutants tests/mutations/<catalog>.json
```

`scripts/focused_control_check.py` is the hosted Linux leg's proof that a focused target's
CTest registration matches its control inventory. It reconfigures the existing build with
focused discovery on, links only that target, and requires each name in
`tests/controls/wipe_tower.json` to be registered once, against the focused binary, and
to pass when run by exact name. It never lowers the control count; extra registered names
are reported but tolerated.

The `--target` mode, which reads an exact control inventory from `tests/controls/`,
needs an already configured build that persists between runs and cannot configure it
itself, so it is not available under the runner. Explicit mode rejects
`--expected-control-inventory`, `--control-regex` and the other target-mode options.

A coverage run needs a build configured with `SLICER_ENABLE_LLVM_SOURCE_COVERAGE`
(or `SLICER_ENABLE_GCC_GCOV_COVERAGE` for `--backend gcov`); the module docstring of
`scripts/slicer_coverage.py` shows its `run` invocation. The profiles and the report
it writes are payload files and disappear with the lease unless a report is promoted
as described below. A run's result is a summary on the PR or CI job, not a retained
tree.

### GCC/gcov coverage with Catch batches

`slicer_coverage.py run --backend gcov` runs the CTest selection test by test. With
`--gcov-catch-batch`, `slicer_catch_batch.py` instead discovers the live CTest selection
and runs it as Catch2 batches, so one executable start covers many cases. CTest remains
the source of the selection. This is an opt-in speed path for the `gcov` backend only;
it supports only CTest registrations that invoke one Catch executable with exactly one
test name; any other registration fails while the selection is discovered, before any
test runs. The option is rejected without `--backend gcov`.
`--gcov-catch-batch-timeout SECONDS` bounds each Catch process (one per executable and
working directory, or per FFF shard; default: `--timeout`).
`--gcov-catch-batch-fff-shards 2` splits the single-CPU `fff_print_tests` cohort into two
concurrent shards and needs a matching `gcov-tool`, found beside `--gcov` or on `PATH`,
or given with `--gcov-tool` (which the parser accepts only with two shards). The profile
directory must not already exist; the default, `OUT/gcov-data`, is inside the payload
because the runner injects `--out`. The gcov backend, with or without batching, needs
exactly the five test executables (`libslic3r_tests`, `fff_print_tests`,
`sla_print_tests`, `libnest2d_tests`, `filament_group_tests`) as `--object`, and seven
`--manifest` files, one per target: `libslic3r`, `libslic3r_cgal` and those five. The
example abbreviates both lists:

```sh
python scripts/validation_runner.py ...runner options as above... -- \
  python scripts/slicer_coverage.py run --backend gcov --gcov-catch-batch \
  --root /absolute/source/checkout --object build/tests/libslic3r/libslic3r_tests ... --manifest ... \
  --test-command 'ctest --test-dir build/tests -C Release -LE "NotWorking|RequiresApp" --no-tests=error' ...
```

### PR-validation evidence gates

`slicer_pr_validation.py` applies, by default, a test-only harness overlay to each PR head and to its base. All of these options keep their meaning under the runner; its `--output-dir`,
`--build-dir` and worktree-reuse rules above still apply.

- **Tracked-diff binding.** The tracked state of a worktree is hashed from
  `git diff --binary --full-index`, and overlay patches are generated the same way, so
  the recorded digest binds full 40-character blob IDs. An abbreviated ID changes as
  the object database grows, which would change the digest without any file changing.
  `--reuse-worktree-from-report` (standalone use only; the runner rejects it) accepts
  an older report's abbreviated-ID digest only when every other tree and status binding
  matches.
- **`--overlay-resolution-manifest FILE`.** The overlay is applied with
  `git apply --3way --index`; a conflict is never auto-resolved. The manifest (JSON,
  `schema_version` 1) holds a non-empty `resolutions` list; each row is keyed by an exact
  base commit, head commit and overlay patch SHA-256. Each conflicting path pins `stage_blobs` for index stages `"1"`
  (merge base), `"2"` (ours) and `"3"` (theirs) as full blob IDs, plus a
  `resolved_path` payload with its `resolved_sha256` and `resolved_blob_sha1`. For an
  add/add conflict there is no merge base, so stage `"1"` is `null`; stages 2 and 3
  must still match. The conflict paths and stages seen in the index must equal the
  manifest exactly, the resolved bytes must hash to the pinned blob, and no unmerged
  path may remain; otherwise the run reports `overlay_resolution_invalid`. A base, head
  and patch with no row reports `overlay_resolution_missing`.
- **`--minimum-tests N`** (default 1, must be positive). A run is comparable evidence
  only when its JUnit testcase count is at least `N`; otherwise a passing or failing
  result becomes `test_evidence_incomplete`. Infrastructure failures are not masked.
- **`--baseline-expectation-manifest FILE`.** Requires reference-baseline execution
  (the default). A source-pinned manifest lets comparison proceed after a harness
  baseline that fails ordinary assertions. It pins `harness_ref_sha`,
  `harness_ref_tree_sha`, `tracked_source_index_sha256` and `testcase_statuses`
  (`passed` or `failed`, at least one `failed`). The baseline qualifies only when its
  JUnit identities and statuses equal the manifest exactly. Raw baseline failures stay
  visible and the overall status stays nonzero.

## Promotion and hosted CI

Promotion selects individual relative regular files, with an explicit retention
owner, expiry and budget. It copies only those files into a separately accounted
retained run. It rejects capability-token content rather than changing evidence
bytes. Promoted data still expires; promotion does not create a permanent archive.

The validation upload action checks actual selected bytes before upload and
requires finite retention. It records work-item, checkout SHA, owner, expiry and
size in the CI summary without changing artifact names or download layout.
GitHub-hosted jobs use provider disposal for local scratch. Uploads through the
action from a self-hosted runner require a managed payload; they cannot assume a
runner will discard its disk. The action never skips its preflight, so a
self-hosted job without an artifact manager context cannot call it: such jobs use `actions/upload-artifact`
directly, guarded by `runner.environment`, and apply the same retention. On
hosted runners where git cannot read the checkout (containers, a checkout
below the workspace root), the receipt records the `source-sha` input instead.

`.github/validation-producers.json` lists tracked entrypoints and their lifecycle
adapters. Repository lint checks registration and retention declarations; runtime
size checks and disposal are verified separately. The public coverage producers
(`coverage-validation`, `coverage-pair-collector`, `catch-batch` and
`coverage-reconcile`) are registered in the same registry. Private coverage producers
keep a corresponding registry in their own repository.

## Legacy data

Migration classifies individual files by role. A directory may contain both
disposable build products and irreplaceable source or reproduction scripts.
Absence of `.git` does not make a directory disposable. Active runs, source and
required evidence remain protected until their owners identify retained files.
Every retained exception needs a named owner and a finite review/expiry date;
unknown provenance is recorded rather than attributed to an invented revision.
