# OrcaSlicer tests

Building, running and writing tests is documented on the wiki, under [How to Test](https://www.orcaslicer.com/wiki/developer_reference/how_to_test.html).

Two files here rather than there, because coding agents only read what is in the repository:

- [AGENTS.md](AGENTS.md) is the same guidance in short form, and is what an agent working under `tests/` picks up.
- [CATCH2.md](CATCH2.md) is the Catch2 reference, including the mistakes that break a test at runtime.

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
  --test-command 'ctest --test-dir {build}/tests -C Release -LE "NotWorking|RequiresApp" --no-tests=error' \
  --llvm-cov "$LLVM_COV" --llvm-profdata "$LLVM_PROFDATA" \
  --threshold 90 --out build/slicer-coverage
```

Use the executable paths and source manifests emitted by the selected
generator when they differ from the Ninja example. The CMake coverage helper
force-loads the static production libraries, so a source object with zero
runtime hits remains in the denominator. A source file listed by that target
manifest but absent from `llvm-cov export` is reported as a missing compiled
source mapping and fails the integrity gate. Headers are included when LLVM
provides executable mappings for them; uncompiled headers and conditional
platform sources are outside the target manifests. The report records the
source root and manifests used.

The report's line/function/branch percentages are **source coverage** from
LLVM's mappings. They are not percentages of registered options. The
`[SettingsVariance]` tests exercise registered enum/schema values, then slice a real
`Print` and inspect geometry or G-code; that is **actual settings execution**.
An option serialization loop alone is schema evidence and does not establish
pipeline behavior.

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
externally provided `build/evidence/open-prs.json` snapshot. It first runs the
same test harness at `--harness-ref` as a baseline, then applies the binary
testing diff from `--harness-base..--harness-ref` with `git apply --3way
--index` to each selected PR head in one disposable worktree. The original
checkout is never changed, and a patch conflict is recorded as a failure for
that head rather than resolved or hidden.

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
baseline and every exact head, tested tree SHA, configure/build/test commands,
elapsed time, logs, JUnit output, and cleanup result. A head is not marked
passing if the baseline fails, configuration/build/test fails, no tests run,
tests are skipped, or the harness patch conflicts; the process exits nonzero
when any selected head or the baseline is not a verified pass.
