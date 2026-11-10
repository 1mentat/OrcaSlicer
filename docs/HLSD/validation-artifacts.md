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

## Promotion and hosted CI

Promotion selects individual relative regular files, with an explicit retention
owner, expiry and budget. It copies only those files into a separately accounted
retained run. It rejects capability-token content rather than changing evidence
bytes. Promoted data still expires; promotion does not create a permanent archive.

The validation upload action checks actual selected bytes before upload and
requires finite retention. It records work-item, checkout SHA, owner, expiry and
size in the CI summary without changing artifact names or download layout.
GitHub-hosted jobs use provider disposal for local scratch. Self-hosted uploads
require a managed payload; they cannot assume a runner will discard its disk.

`.github/validation-producers.json` lists tracked entrypoints and their lifecycle
adapters. Repository lint checks registration and retention declarations; runtime
size checks and disposal are verified separately. Private coverage producers
keep a corresponding registry in their own repository.

## Legacy data

Migration classifies individual files by role. A directory may contain both
disposable build products and irreplaceable source or reproduction scripts.
Absence of `.git` does not make a directory disposable. Active runs, source and
required evidence remain protected until their owners identify retained files.
Every retained exception needs a named owner and a finite review/expiry date;
unknown provenance is recorded rather than attributed to an invented revision.
