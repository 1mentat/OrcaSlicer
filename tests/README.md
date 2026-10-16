# OrcaSlicer tests

Building, running and writing tests is documented on the wiki, under [How to Test](https://www.orcaslicer.com/wiki/developer_reference/how_to_test.html).

Two files here rather than there, because coding agents only read what is in the repository:

- [AGENTS.md](AGENTS.md) is the same guidance in short form, and is what an agent working under `tests/` picks up.
- [CATCH2.md](CATCH2.md) is the Catch2 reference, including the mistakes that break a test at runtime.

Catch2 uses file-descriptor output capture so parallel slicer diagnostics can
be collected safely by the JUnit reporter. Worker threads must finish before
test assertions run, and tests must not depend on diagnostic line ordering.

For CPU-heavy Linux runs, configure with `-DSLICER_TEST_PROCESSORS=1` to give
each test a CPU affinity mask and one CTest scheduling slot. Tests tagged
`[Parallel]` retain two CPUs because their assertions depend on concurrency.
Use `ctest --parallel 8` to allow eight slots in total; build parallelism is a
separate limit. A larger positive setting gives every test that many CPUs;
the default `0` leaves scheduling and affinity unrestricted. CTest applies
affinity on supported operating systems. Hidden Catch tests remain hidden in
every mode.

The CTest build must support affinity. Some Linux distribution packages link
an unpatched system libuv and silently ignore `PROCESSOR_AFFINITY`; the
[official CMake binaries](https://cmake.org/download/) include the required
support. Check a running test's CPU mask when enabling this option on a new
runner; seeing the property in CTest's JSON inventory alone is insufficient.

Tag tests that explicitly verify concurrent execution or thread scheduling
with `[Parallel]`. When adding the first such test to another suite, add its
target to `SLICER_TEST_TARGETS_WITH_PARALLEL` in `tests/CMakeLists.txt` so the
one-CPU mode registers its concurrent tests separately.

`CATCH_EXTRA_ARGS` passes a semicolon-separated argument list to every Catch2
test registered with CTest. For reproducible generator inputs, configure with
`-DCATCH_EXTRA_ARGS="--rng-seed;21907"` and rebuild to refresh discovery. Record
the seed with the results and vary it in additional runs to explore other
inputs. The default empty value keeps Catch2's automatic seed selection.

## Confidence runs

The checked-in harnesses keep coverage and mutation evidence tied to the real
`libslic3r` production objects. They are intentionally separate from the
ordinary test command, and require a configured Release build with the
coverage option enabled. On macOS, use the compiler's matching LLVM tools:

```sh
LLVM_COV="$(xcrun --find llvm-cov)"
LLVM_PROFDATA="$(xcrun --find llvm-profdata)"
DEPS_PREFIX=/path/to/OrcaSlicer-deps/usr/local
UV="$(command -v uv)"
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_COMPILER=clang++ -DSLICER_ENABLE_LLVM_SOURCE_COVERAGE=ON \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DCATCH_EXTRA_ARGS="--rng-seed;21907" \
  -DSLIC3R_GUI=OFF -DBUILD_TESTS=ON -DCMAKE_PREFIX_PATH="$DEPS_PREFIX" \
  -DORCA_BUNDLED_UV_EXECUTABLE="$UV"
cmake --build build --config Release --target \
  libslic3r_tests fff_print_tests sla_print_tests libnest2d_tests filament_group_tests
python3 scripts/slicer_coverage.py run \
  --build-dir build --source-root src/libslic3r \
  --object build/tests/libslic3r/libslic3r_tests \
  --object build/tests/fff_print/fff_print_tests \
  --object build/tests/sla_print/sla_print_tests \
  --object build/tests/libnest2d/libnest2d_tests \
  --object build/tests/filament_group/filament_group_tests \
  --manifest build/slicer-coverage/libslic3r.sources.txt \
  --manifest build/slicer-coverage/libslic3r_cgal.sources.txt \
  --compile-commands build/compile_commands.json \
  --test-command 'ctest --test-dir {build}/tests -C Release -LE "NotWorking|RequiresApp" --no-tests=error' \
  --llvm-cov "$LLVM_COV" --llvm-profdata "$LLVM_PROFDATA" \
  --threshold 90 --out build/slicer-coverage
```

Use the executable paths and source manifests emitted by the selected
generator when they differ from the Ninja example. The CMake coverage helper
force-loads the static production libraries, so a source object with zero
runtime hits remains in the denominator. Every executable mapping within the
declared source root, including mappings contributed by headers, must be
present in `llvm-cov export`; a missing mapping fails the integrity gate. An
object with no executable mapping (for example an include-only translation
unit) is retained in the report as `mapping_free_translation_unit`. Headers are
included when LLVM provides executable mappings for them; uncompiled headers
and conditional platform sources are outside the target manifests. The report
records the source root and manifests used.

The runner uses `compile_commands.json` as compiler evidence for every
manifest source. It invokes `llvm-cov export --summary-only -instr-profile <merged profile>` on
each instrumented object (including with LLVM 18): an object with no coverage map is reported as a
`mapping_free_translation_unit` (for example an include-only PCH source),
whereas an object whose mappings come only from an inline or generated header
is reported as `object_mappings_without_source` and remains subject to the
link integrity check. An object with mappings that is absent from the linked
executable fails the integrity gate as a missing compiled-source mapping. This
keeps mapping-free translation units visible without excusing accidentally
unlinked executable code. Object classification uses mapping counts, never runtime
hit counts, so completely unexecuted functions remain in the denominator. The report's line/function/branch percentages are
**source coverage** from LLVM's mappings. They are not percentages of registered options. The
`[SettingsVariance]` tests exercise registered enum/schema values, then slice a real
`Print` and inspect geometry or G-code; that is **actual settings execution**.
An option serialization loop alone is schema evidence and does not establish
pipeline behavior.

`[MotionFlowVariance]` compares a fresh slice with a processed print after
applying the same changed setting. It checks separate geometry and parsed
G-code evidence, deposited material, feature feedrates, and acceleration
states. Its speed/width/flow partitions have explicit companion settings so
cooling or volumetric limits cannot silently mask the tested effect.
`[ValidationContracts]` exercises rejected configurations and warning gates;
that evidence is declared separately from sliced-output coverage.

`[SettingsSpeedAccelVariance]` checks first-layer and role-specific feedrates,
acceleration, and jerk against every eligible parsed move. Layer markers keep
first-layer travel separate from later motion. Its direct firmware-writer
checks are identified as writer evidence in the settings map; they do not
establish a complete print-pipeline sweep or full numeric-domain coverage.

`[ModelImport]` uses actual STL, OBJ, AMF, and 3MF inputs and checks geometry,
source metadata, colors, decoded texture pixels, cancellation, and error types.
`[EmbossLayout]`, `[TriangleCursor]`, and `[PolylineOperations]` use analytic
geometry and transformation invariants, including integer-grid partitioning
and translation far from the coordinate origin.

The export is summary-only because the report and integrity checks consume
file-level LLVM summaries; rerun `llvm-cov export` without `--summary-only` when
line or segment details are needed. Profile-merge and export diagnostics are retained
in the report even when the LLVM commands succeed. Investigate mismatch warnings
with `llvm-cov export --dump` against the same binaries and profile; an integrity
pass checks mapping presence and does not certify that LLVM emitted no warnings. A zero-summary header that the successful
object audit proves has no executable mapping is recorded as
`mapping_free_headers`. A header with an executable object mapping but no final
mapping remains an integrity failure. Reports expose `integrity_status`
separately from test status, so a failed test run cannot hide a coverage
integrity error.

The mutation run uses a clean checkout and a disposable detached worktree.
The configure command is run once in that worktree, each mutant is an exact
single-match replacement, and every source is restored before the worktree is
removed. Baselines are run per manifest filter so a broad unrelated suite
failure cannot masquerade as a killed mutant:

```sh
DEPS_PREFIX=/path/to/OrcaSlicer-deps/usr/local
UV="$(command -v uv)"
python3 scripts/slicer_mutation.py run \
  --build-dir build \
  --configure-command "cmake -S {root} -B {build} -G Ninja -DCMAKE_BUILD_TYPE=Release -DSLIC3R_GUI=OFF -DBUILD_TESTS=ON -DCMAKE_PREFIX_PATH=$DEPS_PREFIX -DORCA_BUNDLED_UV_EXECUTABLE=$UV" \
  --build-command 'cmake --build {build} --config Release --target libslic3r_tests fff_print_tests sla_print_tests libnest2d_tests filament_group_tests' \
  --test-command 'ctest --test-dir {build}/tests -C Release --no-tests=error' \
  --mutants tests/mutations/slicer.json --max-mutants 7 \
  --out build/slicer-mutation
```

`mutation-report.json` records the commit, worktree commands, exact source
match and replacement, build/test commands, and separate statuses for build
errors, timeouts, killed mutants, and survivors. A mutant is counted only
after its selected unmutated baseline passes with executed test evidence.

The native harnesses run commands in separate process groups. A timeout or
interruption stops the command and its descendants before restoring or removing
the disposable worktree. Partial output is retained as failure evidence. CTest
child timeouts are reported as incomplete mutation evidence, even when CTest
itself exits before the outer command deadline.

Repeat `--mutants` to combine subsystem manifests in one configured build;
`--max-mutants` bounds the total across all selected manifests. For example,
`--mutants tests/mutations/cut_surface.json --mutants tests/mutations/emboss.json
--max-mutants 4` checks both geometry contracts without a second clean build.
The manifest filters are CTest labels; a test name is not a label unless it
has explicitly been registered as one.

The settings inventory keeps schema accounting separate from source coverage.
It derives the effective `PrintConfigDef` option set from
`src/libslic3r/PrintConfig.cpp`, including literal generated overrides and
inherited enum domains, then joins only explicit claims from
`tests/settings_coverage.json`:

```sh
cmake --build build --target slicer_settings_inventory
mkdir -p build/evidence
build/tests/libslic3r/slicer_settings_inventory > build/evidence/settings-registry.json
python3 scripts/settings_inventory.py \
  --root . \
  --source src/libslic3r/PrintConfig.cpp \
  --evidence tests/settings_coverage.json \
  --runtime-registry build/evidence/settings-registry.json \
  --out build/evidence/settings-inventory.json
```

Each option is reported as `declared_covered`, `declared_partial`, or
`uncovered` with required dimensions and a reason. These are declared claims;
the report does not assert that the native tests passed. `declared_covered`
requires a setting-type-specific value
dimension (all enum values, boolean values, numeric boundaries, or
representative values) plus a pipeline oracle. A source mention,
`full_print_config()` construction, or serialization round-trip does not
count. `--fail-on-uncovered` is available when a future evidence map is ready
to enforce a complete settings gate.

The native registry audit compares every compiled `PrintConfigDef` key with
the static inventory and fails on omitted or spurious settings. Omitting
`--runtime-registry` produces a static report with an unverified registry.

## Exact-head PR validation

`scripts/slicer_pr_validation.py` validates the exact head OIDs in the
externally provided `build/evidence/open-prs.json` snapshot. It first verifies
that the complete harness at `--harness-ref` passes. It then applies the
selected diff from `--harness-base..--harness-ref` to the unmodified production
reference at `--harness-base` and to each PR head in a disposable worktree.
The default `--patch-scope test-only` includes tests, scripts, `cmake/`, the
root `CMakeLists.txt` and `.gitignore`. It excludes production bug fixes from
the testing branch, and verifies the `src/` index hash is unchanged by the
overlay. The report records every applied and excluded path.

Use `--patch-scope integration` to also include the two source CMake files,
or `--patch-scope full` for the full branch diff. Repeated `--patch-include`
values add literal paths or directories explicitly. Results from these broader
scopes describe the combined changes, rather than an unchanged PR source tree.
A patch conflict is recorded as a failure for that head and is never resolved
automatically. The runner does not write the original checkout; if its observed
state changes during a run, the report records that fact without attributing
who changed it and exits nonzero.

Run all snapshot heads, or select reproducible PR numbers with repeated or
comma-separated `--numbers` values:

```sh
DEPS_PREFIX=/path/to/OrcaSlicer-deps/usr/local
UV="$(command -v uv)"
python3 scripts/slicer_pr_validation.py \
  --snapshot build/evidence/open-prs.json \
  --harness-base main \
  --harness-ref codex/slicer-confidence-tests \
  --deps-prefix "$DEPS_PREFIX" \
  --uv-executable "$UV" \
  --output-dir build/evidence/pr-validation \
  --jobs 12
# Example focused rerun:
python3 scripts/slicer_pr_validation.py \
  --snapshot build/evidence/open-prs.json \
  --harness-base main \
  --harness-ref codex/slicer-confidence-tests \
  --deps-prefix "$DEPS_PREFIX" \
  --uv-executable "$UV" \
  --numbers 10,12 48 \
  --output-dir build/evidence/pr-validation-selected \
  --jobs 12
```

The native defaults configure Release/Ninja with `SLIC3R_GUI=OFF`,
`BUILD_TESTS=ON`, `SLIC3R_CAD=ON`, build the five slicer test targets, and run
CTest while excluding `NotWorking|RequiresApp`. Use repeated `--cmake-arg`
or `--target` options for a deliberate override. The JSON report records the
harness guard, production reference and every exact head, tested tree SHA,
source-index hashes, configure/build/test commands, elapsed time, logs,
JUnit output, and cleanup result. The reference baseline runs by default;
its failures do not prevent PR tests from running after a green harness guard.
JUnit comparison records failures seen on both reference and PR, newly failing
cases, resolved cases, and cases that cannot be compared. Shared failing test
identities do not establish a shared root cause; raw failures remain visible. A head is not marked
passing if the baseline fails, configuration/build/test fails, no tests run,
tests are skipped, or the harness patch conflicts; the process exits nonzero
when any selected head or the baseline is not a verified pass.
