# Diagnostic coverage replay

`python scripts/slicer_coverage_reconcile.py --manifest <run.json> --out <new-directory>`
replays the reviewed LLVM 18 mapping-warning reconciliation for one archived
full-suite run. The command produces structural export, warning dump, ELF
inspector output, strict and all-kind matcher results, input hash checks, and
per-stage wall timings. It does not run tests, merge profiles, compute or
promote a threshold, waive an integrity warning, or make a coverage claim.

The manifest is the single run binding. Paths in it may be absolute or relative
to the manifest file. Every input and helper has a SHA-256. The expected commit,
full test count, exact profile-mapping warning, and warning occurrence count
are explicit fields; the runtime and coverage reports must agree with them.
The warning's numeric function count must equal `warning_occurrences` before
LLVM is invoked. Preflight also binds the runtime-report hash through
`coverage_report.test_report.sha256`, requires identical profile inventories,
stable before/after source hashes, and clean commit/tree/checkout provenance
where those snapshots are recorded. The manifest itself is included in the
immutable inputs rechecked after replay.
The archived ELF set must contain exactly the five named test binaries with
distinct hashes. The coverage report must already preserve its raw
`coverage_integrity_error`, `integrity_status: error`, and
`coverage_claim: false` state.

Manifest outline (values shown with angle brackets are required real values):

```json
{
  "schema": 1,
  "contract": {
    "expected_commit": "<40 lowercase hex characters>",
    "expected_tests": 3734,
    "profile_mapping_warning": "warning: 3371 functions have mismatched data",
    "warning_occurrences": 3371
  },
  "paths": {
    "report_binary_root": "/absolute/path/namespace/from/runtime/report",
    "working_directory": "/absolute/existing/working/directory"
  },
  "tools": {
    "llvm_cov": {"path": "/path/to/llvm-cov-18", "sha256": "<64 lowercase hex>"}
  },
  "helpers": {
    "all_kind_matcher": {"path": "/repo/scripts/coverage_replay/all_kinds_matcher.py", "sha256": "<64 lowercase hex>"},
    "strict_matcher": {"path": "/repo/scripts/coverage_replay/strict_warning_matcher.py", "sha256": "<64 lowercase hex>"},
    "strict_decoder": {"path": "/repo/scripts/coverage_replay/strict_llvm18_mapping.py", "sha256": "<64 lowercase hex>"},
    "coverage_warning_matcher": {"path": "/repo/scripts/coverage_replay/coverage_warning_matcher.py", "sha256": "<64 lowercase hex>"},
    "elf_inspector": {"path": "/repo/scripts/coverage_replay/elf_covfun_identity.py", "sha256": "<64 lowercase hex>"},
    "replay_controller": {"path": "/repo/scripts/coverage_replay/replay_controller.py", "sha256": "<64 lowercase hex>"},
    "process_runner": {"path": "/repo/scripts/slicer_process.py", "sha256": "<64 lowercase hex>"}
  },
  "artifacts": {
    "runtime_test_report": {"path": "runtime-test-report-adapter.json", "sha256": "<64 lowercase hex>"},
    "coverage_report": {"path": "coverage-report.json", "sha256": "<64 lowercase hex>"},
    "coverage_export_summary": {"path": "coverage-export.json", "sha256": "<64 lowercase hex>"},
    "export_stderr": {"path": "llvm-cov-export.stderr.txt", "sha256": "<64 lowercase hex>"},
    "merged_profile": {"path": "merged.profdata", "sha256": "<64 lowercase hex>"},
    "compile_commands": {"path": "compile_commands.json", "sha256": "<64 lowercase hex>"},
    "source_manifest_libslic3r": {"path": "libslic3r.sources.txt", "sha256": "<64 lowercase hex>"},
    "source_manifest_cgal": {"path": "libslic3r_cgal.sources.txt", "sha256": "<64 lowercase hex>"},
    "llvm_cov": {"path": "/path/to/llvm-cov-18", "sha256": "<64 lowercase hex>"},
    "source_filter": {"path": "/absolute/source/tree/src/libslic3r/AABBMesh.cpp", "sha256": "<64 lowercase hex>"}
  },
  "binary_objects": [
    {"name": "libslic3r", "path": "/archive/libslic3r_tests", "sha256": "<64 lowercase hex>"},
    {"name": "fff_print", "path": "/archive/fff_print_tests", "sha256": "<64 lowercase hex>"},
    {"name": "sla_print", "path": "/archive/sla_print_tests", "sha256": "<64 lowercase hex>"},
    {"name": "libnest2d", "path": "/archive/libnest2d_tests", "sha256": "<64 lowercase hex>"},
    {"name": "filament_group", "path": "/archive/filament_group_tests", "sha256": "<64 lowercase hex>"}
  ]
}
```

Preflight verifies all helper deployment and input hashes, runtime commit/test
identity and provenance, raw coverage status, source/profile bindings, and all
five ELF hashes before invoking LLVM. Any failure stops before the first subprocess. LLVM,
inspector, and matcher commands use explicit argv arrays with bounded timeouts
through the existing `slicer_process.run_captured` utility. Stage times and
stdout/stderr hashes are written to `timings.json`; output must not already
exist.

The copied matcher/decoder/inspector sources retain their reviewed checks. The
small relocation edits only point imports at this packaged closure, while the
controller reads run identity and helper hashes from the manifest. A replay
can reconcile the expected warning geometry, but its result remains diagnostic
and cannot become a percentage threshold pass.

Run source-level checks with:

```sh
python3 -m unittest discover -s scripts/tests -p 'test_coverage_replay.py' -v
```

Normal tests use embedded mapping and tiny-ELF fixtures to exercise region
geometry and ordering, branch identity and fallback rejection, unsupported
MCDC rejection, strict decoder bounds, and the known used-mapping diagnostic
boundary. Those checks do not depend on ignored `build/evidence` files. The
archived parity check is an optional extra: it compares a511 and accepted 6a06
replay JSON when both local evidence bundles are available.
