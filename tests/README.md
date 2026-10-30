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

CTest discovery uses a Python 3 interpreter to parse each Catch2 JSON listing
once before CMake generates its test commands and properties. This avoids
reparsing the entire listing for every test after a suite is linked. Run
`python3 -m unittest discover -s scripts/tests -p test_catch_preprocessor.py`
to check malformed listings and literal names, tags, and output paths without
compiling C++.

Run expensive development, coverage, and mutation controllers inside an OS
resource boundary. Compiler job counts and test timeouts do not impose a memory
limit. The Linux example below assumes a 64 GiB worker with a systemd user
manager and cgroup v2. Its shared slice limits concurrent validation jobs to
16 GiB with no swap and eight CPUs in total; the individual build gets 12 GiB.
Choose smaller budgets when the worker has less available memory.

```sh
systemctl --user set-property --runtime orca-validation.slice \
  MemoryMax=16G MemorySwapMax=0 CPUQuota=800% TasksMax=256
mkdir -p build/tmp
systemd-run --user --wait --pipe --slice=orca-validation.slice \
  --working-directory="$PWD" --setenv="TMPDIR=$PWD/build/tmp" \
  -p MemoryMax=12G -p MemorySwapMax=0 -p CPUQuota=800% -p TasksMax=128 \
  -p RuntimeMaxSec=1800 -p LimitFSIZE=4G -p LimitCORE=0 -p TimeoutStopSec=2 \
  -p KillMode=control-group -p OOMPolicy=kill \
  cmake --build build --config Release --parallel 8 --target libslic3r_tests
```

Place the entire Python controller or CTest command inside the service so its
descendants share the limits and are stopped when it exits. Use smaller limits
for focused tests, such as 4 GiB, two CPUs, and a five-minute service deadline.
For concurrent Linux workers, prefix the service command with `taskset -c`
and its assigned CPU list. Verify `Cpus_allowed_list` in `/proc/<pid>/status`
for the service process and a compiler child. `AllowedCPUs` requires the cgroup
cpuset controller, which some containers do not expose; CPU quota still limits
total CPU use but does not select particular cores.

When a worker will assess output after a command exits, use
`scripts/slicer_process.py:run_recorded` with explicit `cwd`, `env`, timeout,
combined-log path, and a fresh receipt path. It atomically records the raw
process outcome before returning; callers assess test acceptance separately.

Check a suspected build conflict using the service's `MainPID`, `SubState`, and
`ControlGroup`, then inspect the live compiler's working directory and cgroup.
A service with `SubState=exited` and `MainPID=0` has completed. An old
`systemd-run --wait` wrapper can remain afterward with the build command in its
arguments, so command-line searches alone can mistake it for ongoing work.

`LimitFSIZE` limits each output file, including libraries and coverage exports;
leave enough room for the expected artifacts and check free storage before a
run. Disable core dumps so an assertion failure cannot fill the worker's disk.
Keep build trees, temporary files, profiles, and compiler caches on disk-backed
storage; tmpfs retains these artifacts in RAM even after their processes exit.
Use persistent paths outside `/dev/shm` for source checkouts and build trees.
On Linux, `systemd-logind` can remove a user's shared-memory files after the
last session closes; a disk-backed bind mount below `/dev/shm` does not protect
its contents from this cleanup. Check free space on both the build volume and
the root filesystem, which may still hold Git metadata and service logs.
Verify parent-death, memory-limit, and timeout cleanup with small synthetic
processes when adopting a new worker. Preserve failed logs and binaries before
rebuilding; an interrupted run is incomplete evidence.

Serialize heavy controllers on a shared worker. On tmpfs, retained build and
mutation artifacts also consume memory charged to the cgroup; check both free
storage and the shared slice's memory use between runs. Archive successful
artifacts with verified hashes before pruning them, and retain failures for
diagnosis. An OOM termination is an infrastructure failure, never a mutation kill.

On Linux, run permission tests as a non-root user and provide a `de_DE.UTF-8`
locale for `[LocalesUtils]`. If the locale was generated in a private directory,
pass `--setenv="LOCPATH=/absolute/path/to/locales"` to `systemd-run`; an export
in the launching shell does not set the service's environment. First run
`[LocalesUtils]` under the same user and service environment and require zero
skips, keeping its coverage profiles separate from the full run. A skipped
test leaves a confidence run incomplete even when CTest exits successfully.

LLVM's [`%m` profile pooling](https://clang.llvm.org/docs/SourceBasedCodeCoverage.html#running-the-instrumented-program)
updates shared profile files across test processes. Validate this on the chosen
filesystem before a full run: execute two disjoint test filters and check that
both contribute coverage after the second filter. On the confidence worker,
the shared mount failed this check while ext4 retained both filters' hits.
Generate active profiles on the validated local volume; copy them to shared
storage with verified hashes after all test processes exit. Copying an already
incomplete profile to another filesystem cannot recover its lost counts.

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

FilamentGroup's elapsed-time ceilings are enforced by ordinary Release tests.
Coverage builds retain its score, constraint, and property checks, but report
instrumented timings as diagnostics; the run's CTest timeout remains the hang
guard. Validate both configurations before accepting this suite's coverage
evidence. In a separate Release build configured with
`SLICER_ENABLE_LLVM_SOURCE_COVERAGE=OFF`, run all three cases with their original
timing ceilings:

```sh
ctest --test-dir build-release/tests/filament_group -C Release \
  --output-on-failure --no-tests=error
```

For an archived LLVM 18 run with profile-mapping warnings, use
`python3 scripts/slicer_coverage_reconcile.py --manifest <run.json> --out <new-directory>`
to replay the structural export and strict mapping checks. The
[diagnostic replay contract](../scripts/coverage_replay/README.md) describes the
required input hashes and provenance. Replay records stage timings and retains
the raw integrity error; it cannot waive a warning or pass a coverage threshold.

For an offline physical-line union across multiple executables, use the
[pair-aware collector](../scripts/slicer_coverage_pair_collector.md). It exports
each executable with its own hash-bound raw profiles before combining LCOV
tracefiles. This avoids cross-executable profile collisions and preserves
zero-hit source mappings. It does not run tests or attest a passing threshold;
mapping warnings still fail its integrity check.

For feedback on one subsystem in an existing coverage build, run:

```sh
python3 scripts/slicer_module_feedback.py \
  --root "$PWD" --build build --target libslic3r_tests \
  --binary build/tests/libslic3r/libslic3r_tests \
  --tag PlaceholderParser --expected-cases 50 \
  --test-source tests/libslic3r/test_placeholder_parser.cpp \
  --production-source src/libslic3r/PlaceholderParser.cpp \
  --cmake "$(command -v cmake)" \
  --llvm-profdata "$LLVM_PROFDATA" --llvm-cov "$LLVM_COV" \
  --out build/evidence/parser-feedback
```

Choose a fresh output path inside the build tree for each run. Missing evidence
parent directories are created after preflight; existing run directories are
never reused. If a commit changes CMake inputs, regenerate the existing CMake
configuration before this command: the runner requires the compile database to
remain unchanged throughout its measured cycle.
Use the configured CMake executable and the original source/build path spellings
recorded in the retained Ninja configuration. Resolving a symlink alias or changing
CMake versions can invalidate otherwise unchanged objects. Inspect a Ninja dry run
before compiling: adding a focused test target should compile its test/support
sources and link, without recompiling production objects.
For GNU C++, `-DSLICER_TEST_GNU_LTO_JOBS=4` bounds LTO work to four jobs in
Release test-executable links. The default `0` preserves existing linker flags.
This option affects test links only; production targets, compile options, and
other build configurations keep their existing settings.
Before building, the runner checks that the compile database assigns the test
source to the requested CMake target. Canonical and focused targets may both
own the same source; selecting an unrelated target fails before compilation.
For G-code and cooling iteration, build `fff_print_gcode_tests` or
`fff_print_cooling_tests` to compile only that test file and its shared setup.
Both targets are excluded from the default build and CTest discovery. Enabling
`SLICER_ENABLE_FOCUSED_PR_CTEST_DISCOVERY` registers their cases with
`FocusedGCode::` and `FocusedCooling::` prefixes for bounded PR validation;
the option is off by default.
The same option registers `libslic3r_3mf_tests` and
`fff_print_wipe_tower_tests` with `Focused3MF::` and `FocusedWipeTower::`
prefixes, so format and tower contracts can join a focused PR run.
It also registers `fff_print_tree_support_tests` and
`fff_print_multifilament_tests` with `FocusedTreeSupport::` and
`FocusedMultiFilament::` prefixes. These excluded targets compile only their
respective test source and shared setup for support and filament iteration.

`fff_print_composite_focused_tests` combines existing G-code, motion-settings,
Wipe, interlocking, fill, Arachne, pressure-equalizer, thumbnail, utility, and
G-code-processor tests with Precise Seam and TreeSupport. It keeps one FFF main and one copy of
the shared test helpers. The target is excluded from the default build; enabling
`SLICER_ENABLE_FOCUSED_PR_CTEST_DISCOVERY` registers its source cases with the
`FocusedComposite::` prefix. It is added to the parallel-test target list so
`[Parallel]` cases retain the normal two-slot partition when
`SLICER_TEST_PROCESSORS=1`.

Configure an existing test build with focused discovery enabled, build the
composite target, and run its full source inventory with:

```sh
cmake -S . -B build -DBUILD_TESTS=ON \
  -DSLICER_ENABLE_FOCUSED_PR_CTEST_DISCOVERY=ON
cmake --build build --config Release --target fff_print_composite_focused_tests --parallel 2
ctest --test-dir build/tests -C Release --output-on-failure \
  -R '^FocusedComposite::' -LE 'NotWorking|RequiresApp' \
  --parallel 2 --timeout 180 --no-tests=error \
  --output-junit build/composite-junit.xml
```

Keep the configured source and build paths stable between edits. After a test-only
edit, repeat the build and CTest commands to compile the changed test sources and
relink the target. CTest runs each case in a separate process, matching the
canonical suite's isolation of shared library state.

`[RoleFilamentEndpoints]` checks default, available IDs 1–3, and the first
unavailable ID in a three-filament configuration against the emitted tools,
including each role's inheritance peers and the invalid-ID fallback. The
`role_filament_routing.json` mutation redirects top-surface collections to the
bottom-surface filament. Its filter also selects the two equal-ID feature
controls, which preserve their tool assignments under that substitution.
The `[SupportFilamentRouting]` case checks support-base and interface selectors
independently in a two-tool fixture, including zero/default, IDs 1 and 2, the
invalid negative boundary, and ID 3 as the first unavailable selector. The
`support_filament_routing.json` catalog records two validated faults: reading
the interface selector for base deposition, and clamping an unavailable ID to
the last tool instead of the default. Its exact filter includes the new case
and both existing support-routing controls. Numeric inputs beyond this bounded
count+1 case are not claimed.
Use `--filter 'Case A,Case B'` instead of `--tag` to select an explicit Catch
expression. Keep the entire expression in one argument. The same source,
expected-count, and JUnit checks apply.
The command builds the Release target with four jobs, verifies Catch's test
identities and source file, runs the selected tests in random order with seed
21907, and reports coverage for the specified production file. `run.json`
records total and stage timings, source and executable hashes, and JUnit
outcomes. Build and inventory profiles are kept separate from test profiles.
An ordinary test failure still produces diagnostic coverage and returns a
failure; missing tests, skips, timeouts, and changed inputs also fail the run.
This file-scoped report cannot establish the overall coverage threshold.

For Parser iteration, the opt-in target `libslic3r_placeholder_parser_tests`
compiles the same test file and links the production archive members it needs.
Use that target name and `build/tests/libslic3r/libslic3r_placeholder_parser_tests`
as `--target` and `--binary` in the command above. It is excluded from the
default build and CTest discovery. The canonical `libslic3r_tests` image still
retains every production object for the overall coverage denominator.

For Model iteration, use `libslic3r_model_tests` and
`build/tests/libslic3r/libslic3r_model_tests` with `--tag Model` and
`--production-source src/libslic3r/Model.cpp`. This opt-in target compiles
`test_model.cpp` and also stays outside default builds and CTest discovery.

For texture-bake stage diagnostics, use `libslic3r_texture_bake_debug_tests` and
`build/tests/libslic3r/libslic3r_texture_bake_debug_tests` with `--tag TextureBakeDebug`,
`--test-source tests/libslic3r/test_texture_bake_debug.cpp`, and
`--production-source src/libslic3r/TextureBake/TextureBakeDebug.cpp`.
This target also stays outside default builds and CTest discovery.

For motion-setting iteration, use `fff_print_motion_settings_tests` and
`build/tests/fff_print/fff_print_motion_settings_tests` with
`--test-source tests/fff_print/test_gcode_motion_settings.cpp`. This target
compiles the 17 motion-setting cases separately from the other GCode tests.
All cases remain in the canonical `fff_print_tests` suite; the focused target
stays outside default builds and CTest discovery. The focused discovery option
registers its cases with the distinct `FocusedMotionSettings::` prefix.
Splitting the files adds
work to a clean full build and reduces the scope of focused rebuilds.

For faster Release iteration, enable `SLICER_FAST_FOCUSED_TEST_COMPILE` in a
configured test build and build one focused target:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=ON \
  -DSLICER_FAST_FOCUSED_TEST_COMPILE=ON
cmake --build build --config Release --target fff_print_wipe_tower_tests --parallel 4
```

The option defaults to `OFF`. When enabled, it appends `-O1` only for C++ sources
in executable targets named `*_tests` whose `EXCLUDE_FROM_ALL` property is true,
and only in the `libslic3r`, `fff_print`, and `sla_print` test directories. The
canonical default-build suites and production targets keep their normal Release
flags. GNU and native Clang frontends are supported; MSVC and clang-cl fail at
configure time. Debug and non-Release configurations are unchanged.

A Linux GCC probe on the WipeTower test translation unit measured a direct,
non-coverage compile at 99.3 seconds with `-O3` and 73.8 seconds with `-O1`.
In a separate coverage-enabled paired run, both binaries passed the same 137
`[WipeTower]` cases; execution took 15.15 seconds at `-O3` and 16.37 seconds at
`-O1`. These measurements cover this file on Linux GCC; other suites and
toolchains need their own checks.

A coverage-enabled Linux GCC comparison on the G-code test translation unit
passed the same 19 speed, acceleration, and export-control cases at both
optimization levels. Compiling the edited file, linking against unchanged
production libraries and reused test helpers, and running those cases took
140.5 seconds at `-O1` and 168.8 seconds at `-O3`. Compilation accounted for
101.6 and 124.5 seconds respectively. This is one paired worker measurement;
the per-arm times exclude shared setup and integrity checks.

Coverage builds include Orca's source headers as ordinary project headers.
Clang suppresses system-header mappings even when it emits profile counters;
mixing those counters with unused inline mappings can silently remove executed
functions from an export. Dependency headers keep their existing classification.
Coverage targets retain Clang's unused-private-field diagnostics as warnings:
project-header classification exposes existing fields that normal system-header
classification hides. Other warning categories retain their existing policy.

The runner uses `compile_commands.json` as compiler evidence for every
manifest source. Coverage targets require `-fprofile-update=atomic` because
slicing uses concurrent workers. Non-atomic updates can corrupt LLVM's derived
counts and credit branches that never executed. The runner rejects missing or
overridden atomic flags and unresolved compiler response files before running
tests. A non-empty compiled-source manifest and compile database are required.
Rebuild all instrumented objects and collect fresh
profiles when changing this flag. Do not merge profiles from non-atomic builds
into an atomic coverage measurement. See Clang's
[profile update modes](https://releases.llvm.org/18.1.6/tools/clang/docs/UsersManual.html#cmdoption-fprofile-update).

For opt-in instrumentation-cost experiments, configure with both
`-DSLICER_ENABLE_LLVM_SOURCE_COVERAGE=ON` and
`-DSLICER_COVERAGE_PROJECT_FUNCTIONS_ONLY=ON`. The latter defaults to `OFF`
and has no effect unless LLVM source coverage is enabled. It writes
`build/slicer-coverage/project-functions-only.profiler-list` with an allow rule
for the exact configured `${CMAKE_SOURCE_DIR}/src/*` source path and a forbid
default, then adds `-fprofile-list=<file>` only to targets that already opt into
coverage. CMake passes that path as one compiler argument, including when the
build directory contains spaces; the profile rule preserves the configured
`CMAKE_SOURCE_DIR` spelling instead of resolving symlink aliases.
The helper rewrites the list only when its bytes change, since Clang tracks it as
a compile input and touching it would make Ninja rebuild the instrumented TUs.

This experiment leaves `-fprofile-update=atomic`, coverage mappings, target
source manifests, and the normal coverage flags in place. Source files under
`src/` remain eligible for counters; test definitions and external definitions
outside that source path, including dependency-header inline functions, do not
receive counters. That changes instrumentation overhead and can remove
non-production mappings. It is a diagnostic mode, not
a replacement for the normal all-functions coverage run or a basis for claiming
a smaller production denominator. Before comparing results, require identical
production mapping identities and runtime-hit identities between the normal and
filtered runs, and report any difference as an evidence-integrity failure.
Reconfigure, rebuild all instrumented objects, and collect fresh profiles when
switching this option.
It invokes `llvm-cov export --summary-only -instr-profile <merged profile>` on
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
The numeric outer-wall, inner-wall, sparse-infill, and overhang flow-ratio
cases check both registered endpoints (0 and 2) and a representative ratio
of 0.75 against a ratio-1 control. Only the selected role's material changes;
sliced geometry, role-tagged XY motion, and other roles' material stay stable.
These cases explicitly enable `set_other_flow_ratios` and pin the first-layer,
print, and filament multipliers to 1. Wrong-role multiplier probes are in
`role_flow_ratio_sources.json`.
The first-layer, top, bottom, internal-solid, and gap-fill ratios also exercise
registered endpoints 0 and 2. Zero removes deposited E while retaining role XY
paths; two doubles only the selected material, with independent layer and role
controls. The endpoint case uses an explicit single-nozzle vector. A separate
nullable-flow case checks a top-solid object override: parent ratio 1.2 is retained by an
object-level nil slot, explicit 0.7 scales deposited top material by 0.7/1.2,
and returning to nil restores parent behavior through incremental `Print::apply`
with fresh-export equivalence. This remains single-nozzle evidence; multi-nozzle
variant interactions and exhaustive combinations are unclaimed.
`role_flow_variance.json` includes a gap-fill clamp below the maximum to challenge
the boundary oracle and a dropped object override to challenge nullable flow.
The shared role fixture pins the seam to `back`, avoiding visibility sampling
that these oracles do not exercise. In a matched single-run comparison, all 23
cases passed in 17.49 seconds versus 239.69 seconds with the aligned seam,
using the same seed, compiler flags, assertions, and four-CPU limit. Dedicated
seam settings tests retain their own variance.
`[ObjectSpeedNullable]` checks four object speed overrides through a
nil, explicit, then nil transition on the same print. Fresh parent and explicit
controls verify geometry, material, and feeds for standard and high-flow nozzle
variants. Its mutations are in `object_speed_nullable.json`.
`[OuterWallSpeed]`, `[InnerWallSpeed]`, `[SparseInfillSpeed]`,
`[TopSurfaceSpeed]`, `[InternalSolidInfillSpeed]`, and `[GapInfillSpeed]`
check the registered minimum, a representative feed, and nil/explicit/nil
object overrides. Parsed maximum role feedrate must equal the configured speed while geometry and
deposited material remain unchanged. Outer-wall controls disable the separate
small-perimeter branch. Routing probes for outer wall, top surface, solid infill,
and gap fill are in `role_speed_sources.json`.
`[FilamentFlowVariance]` and `[FilamentMaxVolumetricSpeed]` check deposited
material and volumetric feedrate limits, including fresh/incremental agreement.
`[LayerGeometrySettings]` checks initial height, independent shell thicknesses,
and both registered surface-density bounds through emitted extrusion roles.
`[GCodeTravelPointReachable]` checks the six toolchange route placeholders
against bounded obstacle geometry and configured endpoints.
`[FilamentLifecycleVariance]` checks empty and distinct custom filament start,
end, and change scripts around one real BBL 0-to-1 tool transition. Its
mutations are in `filament_lifecycle_variance.json`.
`[ValidationContracts]` exercises rejected configurations and warning gates;
that evidence is declared separately from sliced-output coverage.
`[PrintValidationPublicBoundary]` checks printable-height and spiral-copy
boundaries, prime-tower layer agreement, unused support enforcers, and belt-brim
warnings with valid controls. Build `fff_print_workflow_tests` for focused
feedback; its mutation probes are in `print_validation_public_boundary.json`.

`[RelativeBridgeAngle]` compares automatic, absolute, and relative internal
bridge directions. `[PaintedCutWidth]` checks bounded painted-region areas,
alternating interlocking depth, and beam bypass. `[IroningAngleFixed]` uses an
empty rotation template and the longest emitted ironing segment on each layer
to distinguish absolute direction from inherited direction and alternation.

`[PowerLossRecovery]` checks all three Marlin recovery modes, including the
second-layer transition and disabling recovery after the last extrusion.
`[BedTemperatureFormula]` uses two printed filaments with distinct bed
temperatures to distinguish first-filament and highest-temperature selection.
`[ConfigBlock]` checks explicit inclusion and suppression of resolved settings.
`[PlaceholderParserExpressions]` and `[PlaceholderParserConfig]` check nullable
expressions, literal/regex alternatives, and selective configuration updates.
`[PlaceholderParserVectors]` checks typed indexed writes, vector copying, and
invalid writes to public output vectors without changing their contents.
Its mutations are in `placeholder_vectors.json`.

`[PlaceholderParserConfig]`, `[PlaceholderParserLookup]`, `[PlaceholderParserOutput]`,
`[PlaceholderParserVectors]`, `[PlaceholderParserLegacyIndex]`, and
`[PlaceholderParserUnicode]` exercise configuration merge/precedence, typed output
assignment and scope conflicts, read-only values, legacy filament selection,
escape decoding, and invalid UTF-8 through the public parser.
`[PlaceholderParserAssignmentGaps]` checks assignment conversions, invalid counts,
nullable and incompatible vector copies, and preservation after rejected writes.
`[PlaceholderParserPercentParents]` checks percentage resolution against indexed
parent values and single-element vector fallback.
The expression cases check type errors and skipped evaluation; interpolation
uses an off-midpoint input to distinguish averaging from a linear blend for
nearly coincident coordinates. Their probes are in
`placeholder_public_paths.json`. These parser checks do not establish print-pipeline
settings variance.
The lazy-expression fixture omits `floor` and `ceil`: their handlers reject
suppressed operands. Active valid and invalid arguments remain covered.

`[SettingsSpeedAccelVariance]` checks first-layer and role-specific feedrates,
acceleration, and jerk against every eligible parsed move. Layer markers keep
first-layer travel separate from later motion. Its direct firmware-writer
checks are identified as writer evidence in the settings map; they do not
establish a complete print-pipeline sweep or full numeric-domain coverage.
`[SmallPerimeterSpeed]` checks absolute and percentage speeds against every
later-layer outer-wall move on a six-layer fixture, with a disabled-threshold
control and conserved material. Its mutations are in `small_perimeter_speed.json`.

`[PresetCloudRemoval]` checks configured printer and filament fallbacks after
cloud deletion, missing filament-slot repair, and retention of listed, syncing,
unsaved and dirty presets. Its mutations are in `preset_cloud_removal.json`.

`[PresetCloudSelection]` checks typed cloud-profile loading, inherited filament
identity, and printer snapshot restoration, including missing and hidden
filament slots, optional color arrays, and normalized flush volumes. Its
mutations are in `preset_cloud_selection.json`.

`[PressureEqualizer]` checks smoothing of tagged external-wall flow transitions
against an internal-wall control in absolute and relative extrusion modes.
The output must preserve extrusion, path length, and endpoint while reducing
feedrates on the higher-flow moves. Its mutations are in `pressure_equalizer.json`.
`[VolumetricRateSlope]` compares configured flow-rate slopes in both extrusion
modes; `[VolumetricRateSlopeSegmentLength]` checks the registered 0.5 and 5 mm
segment bounds. Their paired Print exports require segment-length independence
at disabled slope zero and a changed ramp response when enabled, with conserved
path and material. Independent flow and cooling limits are explicitly controlled.
Two configuration-ignoring mutations challenge the direct and exported oracles.
`[PressureEqualizerStateTransitions]` checks G92 logical-position resets and
tool-specific filament cross sections through emitted moves and conserved
extrusion (`pressure_equalizer_state_transitions.json`).
The E-only reset contract compares equivalent physical blocks at two logical
extrusion origins. `[PressureEqualizerCoordinateResetDefect]` separately retains
the combined XY/E reset regression described under known baseline defects.

`[ThumbnailFormatSetting]` checks all five registered legacy default formats
through config normalization, parsing, and encoded G-code export. PNG, JPEG,
and QOI payloads are decoded to dimensions and pixels; BTT and ColPic output
is checked as RGB565 pixel streams, including reserved-black and transparent
pixel conversion. Its three mutations are in `thumbnails.json`. These checks
do not exercise the renderer or explicit per-thumbnail format overrides.

`[GCodeReaderFileStreaming]` checks long-line buffering, mixed line endings,
numbered commands, final unterminated moves, cancellation and reuse, and
empty/missing files. Its mutations are in `gcode_reader_file_streaming.json`.

`[FillRotationTemplates]` checks fourteen deterministic interpolation curves
and equivalent physical duration units over two complete rotation cycles.
The path-direction oracle accounts for whole-degree rounding and axial wrap;
its mutations are in `fill_rotation_templates.json`.

`[CurledLineInfluence]` checks projected curled-edge coverage, height, and distance
against disabled controls through the public extrusion-quality estimator. It
asserts numeric speed and overlap changes while preserving the path. The fixture
supplies layer geometry directly and does not claim complete print settings
wiring. `[PointEstimatorMaxLength]` checks enabled and disabled segment limits.
Their mutations are in `extrusion_quality_curled.json`.

`[ModelSplit]` checks painted disconnected components and multiple painted
volumes through public model splitting. Geometry, painting categories, per-volume
configuration, and source metadata must remain paired after the split. Its
mutations are in `model_split_painted.json`.

`[ModelGapContracts]` checks OBJ face and vertex painting through reconstructed
facet geometry, affine mesh and convex-hull transforms, direct meter conversion,
fuzzy-paint aggregation, and cut-connector detection. Its mutation probes are in
`model_gap_contracts.json`.

`[TimelapseAllLayers]` checks smooth timelapse through public BBL G-code export
with objects of different heights. Snapshot positions must stay fixed across
layers, inside the bed, and clear of both object projections with the configured
clearance. Its mutations are in `timelapse_all_layers.json`.

`[SolidSkinInfill]` checks locked-zag skin density, depth, and locking overlap
through material and path length with unchanged perimeters. `[MinimumSparseArea]`
checks selective solid promotion of a small component. `[TopBottomOverlap]`
checks bottom, top, and interior terrace fill against fixed perimeters. Their
mutations are in `infill_settings_active.json`.

`[InfillOverhangAngle]` checks lateral-honeycomb tracks at the registered 15° and
75° boundaries. `[InfillOverhangForwarding]` checks those values through region
configuration and sliced infill on two layers. `[InfillShiftStep]` checks the
0 mm and 10 mm Crosszag shift boundaries against emitted line phases, including
the unchanged first layer pair and opposite shifts on the next pair. Their
consumer and forwarding mutations are in `infill_geometry_settings.json`.

`[ArachneWallTransitionMapping]` checks nozzle-relative and scaled parameter
conversion. `[ArachneWallTransition]` checks physical width redistribution through
narrow and wide regions. `[ArachneWallResolution]` checks circular wall sampling
with separate resolution and deviation controls, bounded circumference change,
and retention of a shallow boundary feature at strict deviation.
`[ArachneWallTransitionAngle]` exercises the registered angle endpoints on a
wedge; `[ArachneWallTransitionLength]` checks zero-length conversion and geometry
plus a positive-width control at a longer transition. These exercise public wall
generation directly. Their probes are in `arachne_wall_transition.json` and
`arachne_settings_boundaries.json`; parameter mapping alone does not establish
full settings coverage. The separate `[ArachneWallTransitionKnownDefect]` case
marks a baseline negative-width defect and provides no positive coverage credit.

`[PrintExtents]` checks planned extrusion bounds across layer cutoffs, two
instances, unequal path widths, and nested infill collections without slicing.
Its mutations are in `print_extents.json`.

`[CadDocumentHistory]`, `[CadDocumentBooleanKeepTool]`,
`[CadDocumentThickenSurface]`, and `[CadDocumentRevolveEntities]` check history
replay and branching, retained boolean tools, opposite sheet offsets, and a
self-contained revolved profile using volume and bounds. Their production
mutations are in `cad_active_history.json`.

`[SLAElephantFootFade]` checks first-layer compensation, its fade across three
and four layers, and preservation of features below the configured minimum
width through public SLA slicing. Its mutations are in
`sla_elephant_foot_fade.json`.

`[SLARasterPipeline]` attaches the production SL1 archive before SLA slicing and
checks decoded layer images for a stepped model: ordered masks, physical area,
centroid, and dimensions. Its mutations are in `sla_raster_pipeline.json`.

`[SL1Settings]` generates slow and fast material profiles through SLAPrint and
checks the emitted `config.ini` profile flag and identity values. Its focused
probes are in `sla_sl1_settings_variance.json`.

`[SLAExposureBounds]` checks inclusive zero and nonzero exposure limits through
public SLA validation. `[SLAExposureStatistics]`, `[SLATiltStatistics]`, and
`[SLAAreaFillStatistics]` check six-layer exposure fade weights, fast/slow tilt
durations, and layer classification against physical time totals and layer-time
multisets. Setting-only pairs preserve slice areas and object material; zero
exposure and tilt minima retain positive geometry and material. Timing-source
and area-fill classification probes are in `sla_exposure_statistics.json`.

`[PresetPersistence]` checks inherited JSON imports, unresolved-parent rejection,
subscription refreshes with newer preset timestamps, complete root-filament
exports, and preservation of system presets during user-file loading. Its
mutations are in `preset_persistence.json`.

`[BbsImportContracts]` checks invalid native mesh data, semantic model-settings
errors in native and foreign archives, exact sidecar extraction paths and bytes,
and metadata-only `SkipModel` packages. Its mutations are in
`bbs_import_contracts.json`; mesh-guard bypasses are excluded because they can
crash before an ordinary assertion.

`[AppConfigPreferences]` checks directory-history precedence and gallery
exclusion, independent vendor copies, ordered custom-color updates, and
per-printer nozzle preferences across save/load. These persistence contracts do
not claim slicing-settings variance coverage.

`[PresetBundleSubscribedMultiType]` checks typed subscribed profiles, canonical
inheritance, missing and unsupported types, and removal of obsolete printer files.
Its mutations are in `preset_subscribed_multitype.json`.

`[LayerProfileEditing]` checks decrease and increase saturation, reduction toward
nominal thickness, local smoothing, and fixed first-layer/object endpoints.
`[TreeSupportLayerHeight]` compares bounded independent support intervals with
synchronized object layers. Their mutations are in `layer_profile_editing.json`
and `tree_support_layer_height.json`.

`[FuzzySkinModePipeline]` checks positive transverse displacement with unchanged
widths; the Perlin field is deterministic but sampling positions are stochastic.
`[SeamSlopeSettings]` compares full-loop scarf distance with a bounded scarf.
Their mutations are in `fuzzy_displacement_scarf_loop.json`.
`[SlicingModeHoleTopology]` checks a real cavity's hole count and filled area
across regular, even-odd, and close-holes slicing (`slicing_hole_topology.json`).

`[FanSpeedupOverhangs]` locates genuine rising PWM targets in physical motion,
without assigning roles from comments that can remain behind moved commands.
Deposition remains conserved. Its mutations are in `fan_speedup_overhangs.json`.

`[BBSStreamThumbnailFallback]` checks explicit and fallback image bytes remain
on their own plates, including a missing fallback (`bbs_stream_thumbnail.json`).
`[FilamentTPUGrouping]` checks preference costs and physical restrictions through
the public grouping API (`filament_tpu_grouping.json`); it provides no evidence
for the currently inactive automatic TPU dispatch.

`[PresetBundleImport]` imports an actual ZIP bundle with process, filament, and
printer profiles, then checks metadata, canonical identities, typed settings,
and persisted per-type files. Its mutations are in `preset_bundle_import.json`.

`[Bbs3mfMetadata][LayerProfile]` imports a four-object archive containing valid,
malformed, and duplicate layer-height profiles. Invalid rows must not prevent
later valid rows from loading, the first duplicate wins, and one-based profile
IDs must reach the correct objects. Its probes are in
`bbs_layer_profile_validation.json`. Build `libslic3r_3mf_tests` for focused
feedback without duplicate CTest registration.

`[CutSurfaceTopology]` checks projected depth selection, disconnected components
within one mesh, and multiple holes using contour validity and geometric area.
`[TpmsAdaptiveNormalY]` checks density invariance along Y and compares shell and
core path lengths per unit area against a uniform-density control. Their
mutations are in `cut_surface_topology.json`, `cut_surface_boundary.json` and
`tpms_normal_y.json`. The surface-boundary regression includes an outer contour
coincident with the model boundary, with two holes whose area must remain removed.

`[ZContouringSettings]` checks all four combinations of Z contouring and its
infill-direction flag through emitted sparse paths. It also checks two minimum-Z
values against the resulting slice planes. `[ZContouringPerimeter]` compares
perimeter Z correction across disabled, zero, moderate and maximum slope
thresholds, using length-weighted geometry to tolerate path resampling. Their
mutations are in `z_contouring_settings.json` and `z_contouring_perimeter.json`.

`[TransformGCode]` checks rotated, translated, partial-axis and repeated linear
moves, including forced coordinate emission. `[WipeTowerPrimePublic]` checks
tool sequencing, purge volume, circular-bed placement, cooling delays and
filament monitoring through the public priming API. `[TriangleMeshSlicerContracts]`
checks vertical face geometry, coplanar slab ownership, mirrored transforms and
independent top/bottom projections. Their mutation manifests are
`transform_gcode.json`, `wipetower_prime_public.json` and
`trianglemesh_slicer_contracts.json`.

`[SketchEdit]` checks circle and arc cutters against analytic intersections,
complementary arc midpoints, synchronized endpoints and the nearest forward
extension. Circular-cutter mutation probes are in `sketch_circular_cutters.json`.

`[Int128Predicates]` checks the signs returned by public point cross-product and
orientation predicates for collinear vectors, near-collinear determinants of
one, well-separated vectors, and translated coordinates. Its mutation in
`point_contracts.json` reverses the approximate determinant filter decision.

`[LineContracts]` checks affine endpoint transforms, finite and infinite
intersections, perpendicular distances, bounding-box clipping and 3D plane
interpolation against analytic geometry. Its probes are in `line_geometry.json`.

`[LineSplit]` checks clipping through holes, closed endpoints, boundary overlap
and source-index provenance, including empty paths. `[RegionExpansion]` checks
bounded wave growth, source and boundary IDs at shared vertices, and selection
of the source-bearing merged component against analytic bounds and areas.
Their probes are in `algorithm_region_expansion.json`.
`[TriangleMeshCutMesh]` requires watertight halves with analytic volumes when a
cut follows existing edges or passes through vertices; its probes are in
`trianglemesh_slicer_contracts.json`. These cases also run in the focused
`libslic3r_geometry_tests` executable.

`[WipeTowerSparsePipeline]` compares ordinary and skipped sparse tower layers
through public slicing and G-code export. Tower deposition must decrease while
the object's deposition segments remain unchanged. Its mutations are in
`wipe_tower_sparse_pipeline.json`.

`[PrintValidationTravelAcceleration]` checks firmware-specific travel acceleration
limits, equality, and disabled-limit controls. Its mutations are in
`print_travel_validation.json`.

`[BedTemperaturePipeline]` checks all six bed families through initial and
second-layer heater commands, including declared maximum targets, suppression
of redundant initial zero targets, and explicit later heater-off commands.
`[ScheduledLayerGCode]` checks pause/template expansion, literal custom commands,
layer placement, current-plate selection, and sequential-print suppression while
preserving deposition. Their mutations are in `bed_temperature_pipeline.json`
and `scheduled_layer_gcode.json`.

`[Bbs3mfExportGaps]` checks parsed layer-filament sequences and ranges, four decoded
thumbnail channels, and invalid thumbnail-list cardinalities. Its mutations are
in `bbs_export_metadata.json`.

`[ColorGroups]` checks object-specific extruder assignments from root and related
3MF model color groups, including reuse of the same color across model files.
Its mutations in `bbs_import_colors.json` challenge slot allocation and reuse.
`[SpiralSequential]` checks rising perimeter motion, effective smoothing, and
starting/finishing flow in sequential vase exports against controls and the
equivalent layer-by-layer output. Its mutations are in `spiral_sequential.json`.

`[VendorJson]` checks printer model metadata loaded from JSON, including FFF/SLA
technology, deduplicated nozzle and material lists, bed assets, empty lists, and
error accounting for a malformed material list. Its mutations are in
`vendor_json_metadata.json`; these metadata tests do not imply settings pipeline coverage.

`[BeltTreeLanding]` checks support geometry and exported support deposition on
an explicit 0.8 mm layer grid, with lower/raised belt floor controls and one/two
wall configurations. Finite zero-material paths left by sub-resolution
simplification are counted separately from physical support geometry. Exported
motion is measured in XYZ to include belt remapping. `belt_tree_landing.json`
challenges the floor-offset control and two-neighbour merge termination; branch
reachability and mutation kills require native validation.

`[Bbs3mfUUID]` restores marked and marker-free split component graphs, checking
named object/volume identities, analytic mesh volumes, and world placements.
`bbs_restore_graph.json` challenges the wrapper, child, and marker remapping.

`[MixedBBLLabels]` checks ordered BBL/Marlin object boundaries across mixed
sublayers, the encoded single-instance BBL label, and conserved deposition.
Marlin object definitions are validated separately from body boundaries.
`mixed_bbl_labels.json` challenges each BBL boundary and its dedicated end flush.

`[AdaptivePressureAdvanceRamps]` checks emitted pressure-advance values against a
linear calibration model for overhang events with an immediate feed update,
including retained values when duplicate commands are omitted. A disabled
control checks that pressure advance remains off. Its mutation in
`adaptive_pa_pipeline.json` replaces interpolation with a configured constant.

`[AdaptivePressureAdvanceOverhangGate]` varies only the overhang adjustment,
checks actual pressure-advance state during matching deposited segments, and
preserves their geometry and filament. Its setting-binding mutation is in
`adaptive_pa_overhang_gate.json`.

The paired `[H2C][Dynamic]` selector test toggles `enable_filament_dynamic_map`
on the same sequential two-object plate. It checks the static result and empty
plan cache when disabled, then stitched layers and cached plans when enabled,
and exports both modes. Its mutation is in `h2c_dynamic_map.json`; this fixture
does not claim physical nozzle migration.

`[PrintValidationInteractions]` checks belt/raft/draft-shield constraints,
sequential-print feature gates, and the full ooze-prevention/single-extruder
multi-material truth table, including valid controls and blocking-error keys.
Its mutations are in `print_validation_interactions.json`.

`[PresetBundleFullFffMaps]` and `[PresetBundleFullFffProvenance]` check explicit
filament-volume map overrides and selected system-profile changes, with
mutations in `preset_bundle_full_fff.json`.

`[SupportCache]` checks support-layer cache round trips through `Print`, including
island coordinates, extrusion geometry, interface metadata, layer links, and
malformed payload rejection. `[WipeTowerContact]` checks parsed feedrates,
interface temperature commands, pre-extrusion lengths, and bed clipping through
the tower generator; these settings claims remain separate from full Print wiring.

`[WipeTowerContactPipeline]` checks tool-targeted interface temperatures, the
range-high sentinel, cooldown ordering, and zero pre-extrusion settings in
exported Print G-code while conserving model deposition.
`[WipeTowerGeometryEndpoints]` checks width and brim boundaries, physical purge
flow, and rib outlines. Their mutations are in
`tower_interface_geometry_endpoints.json`.

`[SupportBasePattern]` checks all six registered normal-support base patterns,
including structured fallback warnings, honeycomb geometry, and alternating
rectilinear-grid layer orientation on a bounded overhang fixture.

`[FilamentPlanContracts]` checks physical nozzle identity with repeated loaded
filament values, filtered extruder groups, multiple assignment epochs, and
independently enumerated flow costs. Empty-nozzle inputs have an explicit error
contract, while the legacy solver continues to return filament values.

`[ModelImport]` uses actual STL, OBJ, AMF, and 3MF inputs and checks geometry,
source metadata, colors, decoded texture pixels, cancellation, and error types.
`[EmbossLayout]`, `[TriangleCursor]`, and `[PolylineOperations]` use analytic
geometry and transformation invariants, including integer-grid partitioning
and translation far from the coordinate origin.

`[TriangleSelectorRecursive]` checks bucket filling through split paint facets,
local and propagated selection, distinct paint-state boundaries, and nondegenerate
contours while conserving the square fixture's area. Its mutations are in
`triangle_selector_recursive.json`.

`[Bbs3mfThumbnailDownsample]` compares every decoded preview pixel against an
independent block average for square and wide source images. `[SubscribedBundleReload]`
checks metadata and typed preset membership after disk reload, while
`[VendorVariantValidation]` checks valid and mismatched nozzle variants with
validation enabled and disabled. `[ModelMirror]` checks all three reflection
axes, cached bounds, and restoration after a second reflection. Their bounded
mutants are in `bbs3mf_thumbnail_downsample.json`, `preset_reload_variants.json`,
and `model_mirror.json` under `tests/mutations/`.
`[MixedGradient]` checks configured gradient ratios and independent five- and
seven-layer volume runs, with per-part gradients enabled and disabled; its
mutants are in `tests/mutations/mixed_gradient.json`.
`[MixedGradientGCode]` checks emitted tool-specific flow shares and fractional
layer heights for linear and explicitly curved gradients, plus a disabled
control. Its bounded mutants are in `tests/mutations/mixed_gradient_gcode.json`.

`[PrintCacheProcess]` loads calibrated path flow into a fresh print and checks
that cache processing retains it while ordinary processing recomputes the
original flow. It also checks geometry and completed object steps, with mutants
in `tests/mutations/print_cache_process.json`. `[GCodeProcessorUnknownAxis]`
and `[GCodeProcessorArcR]` check unknown-axis origin resets and radius-form arc
geometry through the public parser; their mutants are in
`tests/mutations/gcode_processor_g92_arc.json`.

`[WrappingDetectionGCode]` checks clumping-hook gates, zero-based layer metadata,
and deposition height recovery after a hook moves Z. `[CalibrationInputShapingGCode]`
checks per-layer frequency and damping ramps through Marlin 2, RepRapFirmware,
and Klipper exports, including independent X/Y ranges and a disabled control.
Their mutants are in `wrapping_detection_gcode.json` and
`calibration_input_shaping_gcode.json` under `tests/mutations/`.
`[Bbs3mfSourceMetadata]` checks source paths, source-volume IDs, offsets, and
unit-conversion flags after archive reload; its mutants are in
`tests/mutations/bbs_source_metadata.json`. `[PhysicalPrinterDeletion]` checks
sole and shared preset membership, saved settings, file removal, and selection
clearing, with mutants in `tests/mutations/physical_printer_deletion.json`.
`[FilamentRetractionApplyTransition]` reuses a print through explicit, inherited,
and replacement filament values and checks its public applied retraction config.
It complements exported-motion checks under `[FilamentRetractionOverrides]`;
`filament_retraction_routing.json` tests the nullable override lookup itself.

`[FilamentCutRetractionOverride]` also checks exported cut and extruder-change
placeholders on two tools with reversed machine-slot mapping. Four selected
scenarios cover all three named cut gates, machine and filament cut-distance
endpoints, nil cut overrides, explicit Boolean overrides, and explicit
extruder-change endpoints. Every scenario retains positive extrusion on both
tools and conserves deposited material. The oracle observes values supplied to
user G-code; it does not measure a physical cut or retract, and extruder-change
nil behavior remains unclaimed. The two bounded mutations are in
`filament_cut_ec_placeholders.json`. Its CTest label also selects the existing
mapped cut-distance override case.

`[SkirtDistance]` measures emitted skirt clearances at 0, 3, and 60 mm against
an explicitly placed square, accounting for half the public skirt-flow spacing.
`[SkirtAngle]` checks -180, -90, 0, 90, and 180 degree start bearings while
preserving loop extents, path length, and deposited material. `[SkirtSpeed]`
checks the zero-speed fallback at an explicit 23 mm/s and a 37 mm/s override.
The three bounded mutations are in `skirt_settings_geometry.json`. These are
selected geometry and speed scenarios, not a full interaction matrix.

`[ExtrusionRoleGCode]` checks machine, filament, and process hook ordering,
role transitions, and layer placeholders against emitted motion roles.
`[CalibrationTowerOutput]` checks temperature, outer-wall speed, junction
deviation, and jerk bands on six-layer fixtures, including modal values when
firmware commands are deduplicated. Their mutants are in
`extrusion_role_hooks.json` and `calibration_tower_output.json`.
`[PrintStatisticsFilename]` checks deferred placeholders, final filename units
and aliases, and unchanged filenames after placeholder errors, with mutants in
`print_statistics_filename.json`. `[AmsComboFallback]` checks generic, previous,
and system material selection priority and tray metadata, with mutants in
`ams_combo_fallback.json`. These two groups do not slice or export a print.
`[Bbs3mfForeignMultiInstance]` checks independent objects, geometry, and world
placement when the BBS importer loads a foreign multi-instance archive. Its
mutants are in `bbs_foreign_multi_instance.json`.

`[CloudTimestamp]` checks stale, equal, and unknown cloud timestamps against
local configuration and persisted preset metadata, without slicing. Its mutants
are in `preset_cloud_timestamp.json`. `[ToolOrderingSupportInterface]` checks
automatic support filament selection on support-only layers containing both
base and interface paths. It varies interface-body exclusion and verifies flush
cost selection, with mutants in `tool_ordering_support_interface.json`.

`[PrintSharedPrintableArea]` checks per-nozzle printable regions, their excluded
complements, the shared intersection, and single-nozzle or degenerate-area
fallbacks. `[GCodeProcessorBuildVolume]` checks plate and nozzle bounds, mapped
filament restrictions, diagnostic reset, and exclusion of custom extrusion from
object bounds. These groups avoid slicing and export; their mutants are in
`print_shared_printable_area.json` and `gcp_build_volume.json`.

`[TextureColorRegions]` checks connected region area, the strict merge threshold,
summed shared boundary lengths, and unpainted borders on a four-triangle mesh.
Its bounded mutations are in `texture_region_merge.json`.
`[TextureColorDespeckle]` checks isolated color removal, the zero-pass control,
strict ties, uncolored neighbors, and triangle-order independence through a
public color bake. Its mutations are in `texture_despeckle.json`.
`[TextureProjectionColor]` checks cylindrical, spherical, and projective color
and height sampling against literal texels, including normalized axes, perspective
division, rejected depth, and untiled bounds. Its mutations are in
`texture_projection_color.json`.

`[BbsAuxiliaryExport]` checks root and nested model auxiliary payloads, archive
path mapping, root containment, and the `SkipAuxiliary` disabled control.
Its mutations are in `bbs_auxiliary_export.json`.
`[BbsPlateRelationships]` checks selected-plate thumbnail relationships and
decodes the referenced full-size and small PNG members. Its mutations are in
`bbs_plate_thumbnail_relationships.json`.

`[SplitModel]` and `[AMFArchive]` cover compressed model parts, physical
placement, mesh ownership, metadata, and malformed inputs. `[SplitModelRestore]`
checks non-sequential backup IDs, geometry preservation, and missing-part
rejection. `[Bbs3mfAssemblyErrors]` checks missing object references in assembly
metadata. `[TextureDisplacementView]` checks projective placement, excluded UV
windows, and mirrored tiling against literal height-map samples. `[GcpPost]` checks
rewritten G-code against independently calculated time, filament volume,
mass, and cost. `[PyramidWinding]` checks oriented topology and analytic
volume under translation; `[SLADrainTransforms]` checks normals against
transformed surface tangents under rotation, unequal scaling, and printer
correction. `[ColorUtilsExtended]` checks clustering, palette preservation,
UV correspondence, mesh closure, and remeshing invariants.

`[GCodeProcessorReachable]` checks reserved-tag selection and preheating through
the public post-processing path. A completed extruder's final idle window must
emit cooling without scheduling a later heating command or heating block.
The preheating mutation manifest challenges the usage-block gates, temperature
branches, extruder selectors, and generated command markers.

`[LegacyWipeTowerContactPipeline]` exports a type-1 tower with alternating
material categories and two physical heaters. Distinct explicit temperatures
and sentinel range highs check each incoming heater's template and contact-grid
waits, base-temperature restores, and conserved model deposition.
Its sentinel fallback probe is in
`tests/mutations/tower_interface_geometry_endpoints.json`, alongside the Type2
fallback probe.

`[RoleFanBoundaries]` checks fan state on real internal-bridge, support-interface
and ironing extrusion across selected endpoint values, cached settings changes,
fresh prints and restored settings. The fixture keeps both role-generating mesh
groups in one object and pins `seam_position=back`, so fan checks do not repeat
visibility-based seam ray tracing. Path counts, XY length and deposited filament
must remain stable. `[FanPwmBoundaryValues]` checks the writer's off behavior,
rounding and minimum-PWM clamp with explicit endpoint and adjacent values.
Their probes are in `cooling_role_fans.json`.

The partial-free cooling case varies the cooling rate against a configured
190 C target and requires preheating and nozzle-map commands before optional
motion. Its probes are in `gcode_processor_partial_free_cooling.json`; the
separate heat-only branch probe remains a documented survivor because this
fixture does not reach it.

`[SettingsFamilyVariance]` connects support speeds, bridge flow, overhang
speed bands, ironing overrides, and infill connection limits to emitted
motion or geometry. Its mutation manifest retains two `known_survivors`
for requested infill-anchor length and clamping. They are documented gaps,
excluded from the executable `mutants` list and from coverage claims for
those dimensions; they must not be reported as assertion kills.

`[SupportThresholdAngle]`, `[EnforceSupportLayers]`, and
`[SupportObjectXYDistance]` check the 0/30/90 degree threshold classes, the 0/5000
support-layer endpoints, and the 0/10 mm XY-gap endpoints on a 120-layer ramp.
Their CTest labels select only these three new cases; the probes are in
`tests/mutations/support_boundaries.json`. They exercise selected endpoints and
classes on this fixture, not an exhaustive sweep of each numeric domain.

`[UtilsIOContracts]` checks layer-pattern boundaries, file content across buffer
boundaries, recursive copy filtering, UTF-8 byte spans, and formatting contracts.
`[UtilsFrameworkFailure]` checks that nested framework-copy errors propagate to
the caller. Unicode NFC composition remains unclaimed: the current build leaves
decomposed accents unchanged, and its Boost.Locale dependency disables ICU.

`[GCodeBroad]` checks coordinate transforms, volumetric limits, object labels,
firmware commands, belt and prime-tower exports, and snapshot positions parsed
from executable commands. Its delayed-slowdown mutation remains a documented
survivor; that process-layer gate is unclaimed. `[PresetBundleContracts]` checks
complete per-nozzle purge matrices, selected profiles, inheritance, and the
ownership of embedded preset files.

The firmware dialect case retains the model's XY geometry in a six-layer
fixture. It compares deposited paths across five firmware flavors and checks
their configured acceleration and pressure-advance commands. Its probes are in
`firmware_flavor_dialect.json`. That catalog uses the full Catch case name;
pass `{filter}` without tag brackets in the test command:

```sh
--test-command '{build}/tests/fff_print/fff_print_gcode_tests {filter} --order rand --rng-seed 21907 --reporter compact'
```

`[GCodeTimingCentripetal]` checks virtual-move timing at shallow turns, with
straight, opposite-turn, wide-turn, and low-speed controls. Its mutations are
in `gcode_timing_centripetal.json`.

`[GCodeFanPipeline]` exports small prints in both sequence modes. It checks the
configured cooling range and the kickstart
pulse followed by its target speed while conserving extrusion and path length.
Its mutations are in `gcode_fan_pipeline.json`.

`[AirFiltration]` checks exhaust-fan commands before and after deposition, with
independent capability, master-switch, and phase controls. Five four-layer
exports preserve object material; its mutation is in `air_filtration.json`.
`[DraftShieldSingleLoop]` checks three first-layer skirt loops and either three
or one later-layer loop while preserving object deposition. Its two four-layer
exports use actual extrusion runs; its mutation is in
`draft_shield_single_loop.json`. `[SupportInterfaceLoopPattern]` checks added
contact paths with positive support-interface deposition in both setting
states; its mutation is in `support_interface_loop_pattern.json`.

`[TreeSupportInteractions]` compares automatic and explicit brim areas and
zero/one/two support-wall settings through public slicing. It checks closed
support loops with an explicit rectilinear base pattern; its mutations are in
`tree_support_interactions.json`.

`[TimelapsePosPickerGap]` checks X0 clearance with two configured radii,
the inclusive rod-height collision boundary, and parking inside the selected
extruder's printable region. It uses public picker calls and validated Print
fixtures. Build `fff_print_timelapse_pos_picker_tests` for focused feedback;
the same cases belong to the canonical FFF suite. Its mutation probes are in
`timelapse_pos_picker.json`.

The same tag checks smooth snapshots below, at, and above a dual-nozzle height
gap, stable all-layer positions, and reinitialization with a different printable
region. These boundary probes are in `timelapse_smooth_height.json`.

`[PaintedSegmentationGap]` checks that sinking clips bottom paint below the
build plane and shell projections retain both model volumes. Expected regions
come from independently sliced geometry. Build
`fff_print_painted_segmentation_tests` for focused
feedback; the cases also belong to the canonical FFF suite. The shell probes
are in `painted_shell_projection_gaps.json`.

`[PaintedHoleProjection]` checks top and bottom shell propagation beside an
acute external through-hole. It compares the affected painted area with an
unholed control and a fixed geometric counterexample, so pruning geometry
outside the paint bounding box must preserve the hole's miter influence.

`[PrintObjectSharing]` checks independent layer geometry when mesh, transform,
extruder, layer profile, layer-range settings, or material settings differ.
The range and material cases use equal meshes with separate allocations and
check both objects' actual perimeter paths. Build `fff_print_workflow_tests`
for focused feedback without duplicate CTest registration.

`[ToolOrderingDontCareSeed]` checks discovery of the first physical tool,
inheritance across placeholder layers, and the no-tool case through public
ToolOrdering methods. Its probes are in `toolordering_dontcare_seed.json`;
build `libslic3r_toolordering_tests` for focused feedback.

`[MultiNozzle]` checks ordered nozzle representatives, directed per-nozzle flush
history, safe fallback queries, and legacy group metadata conversion through
public utility results. Its probes are in `multinozzle_utils.json`; build
`libslic3r_toolordering_tests` for focused feedback.

`[Producer]` checks public G-code file import with unregistered producer headers,
compatible role tags, ignored legacy Simplify3D metadata, and active Orca config
and feature metadata. Build `fff_print_gcodeprocessor_tests` for focused feedback;
it includes both processor test files. Its probes are in
`gcodeprocessor_producer_import.json`.

`[WipeTowerEstimate]` checks pre-slice footprints for both planners. The Type1
spacing case compares the registered 100% minimum with 150% and checks the
scaled depth while Type2 follows its own spacing option. The setting-selection
probe is in `wipe_tower_estimate_spacing.json`. Scope is footprint estimation.

`[MultimaterialWipe]` checks direct WipeTower2 purge matrices, wall geometry,
ramming, cooling, stamping, loading, and delay commands. Independent cooling
variants change one setting at a time, with feedrates read from executable
G-code. These direct-consumer checks do not establish complete Print pipeline
coverage; joint variations and numeric-domain gaps remain explicit in the
settings map. `[WipeTowerBroad]` covers tower estimators, sparse-layer folding,
role-specific filament selection, and object/support wiping eligibility.
`[WipeTowerPurgeSpeedVariance]` parses modal feedrates in positive purge and
sparse-grid extrusion at caps of 10, 40, and 200 mm/s with a 160 mm/s region
speed. It checks both limits while retaining geometry and material invariants;
the production probes are in `wipe_tower_purge_speed.json`. This direct
WipeTower2 fixture does not claim legacy WipeTower or full Print coverage.
The short-stamping case in `[WipeTowerVariance]` checks positive-extrusion return
strokes and derives total filament loading and X travel from the configured
speeds and distances. Build `fff_print_wipe_tower_tests` for focused feedback;
the same cases run in the canonical FFF suite without duplicate registration.
Its probes are in `wipe_tower2_short_stamping.json`.

`[CompactedTower]` checks pre-slice collision diagnostics for zero, one, and two
objects in the rod's clearance band, plus the disabled-setting control. It
checks the reported height and selected object without slicing; its mutations
are in `compacted_tower_clearance.json`.

`[StaticEnumDefaults]` checks that static and dynamic default configurations retain
named enum lists before and after public deserialization. Its mutation is in
`static_enum_defaults.json`; default-name assertions precede parsing so missing
metadata produces an assertion failure instead of a crash.

`[PresetAssetResolution]` checks installed printer assets taking precedence over
bundled copies, bundled and generic hotend fallbacks, and unresolved printer
models through the public `PresetUtils` getters. Temporary data and resource
roots are restored after each case. Its probes are in `preset_asset_resolution.json`.
Build `libslic3r_preset_bundle_tests` for focused feedback without duplicate CTest
registration or compilation of unrelated core test files.

The three `PrinterPresetCollection` cases cover system and custom model/variant lookup,
empty-model rejection, and visible-user classification. Their lookup mutations are in
`printer_preset_lookup.json`. The opt-in `libslic3r_preset_collection_tests` target
compiles those cases with the existing preset-diff contracts and stays outside default
builds and CTest discovery:

```sh
cmake --build build --config Release --target libslic3r_preset_collection_tests
build/tests/libslic3r/libslic3r_preset_collection_tests "[PrinterPresetCollection]" --rng-seed 21907 --order rand
```

`public_gap_contracts.json` challenges public behavior across geometry, model
ownership and import cancellation, nullable setting migration, filament
temperature validation, tool ordering, wipe-tower finish paths, and G-code
preheating. Geometry cases use analytic distances and preserved fitting metadata;
pipeline cases verify the effective settings before checking output. Mutants
target those assertions without treating crashes or build failures as kills.

The same manifest covers export recovery, resonance speed limits, motion warnings,
project migration, and format round trips. `[GCodeExportFailures]` retries the same
Print after temporary-file, thumbnail, and destination errors while checking
lifecycle outcomes and preserved files. `[ResonanceAvoidanceGCode]` checks emitted
outer-wall speeds at the midpoint and bounds, with cooling and flow limits set
explicitly. `[ConfigJsonMigration]`, `[PresetNullableSave]`, and
`[PlaceholderParserMapping]` check persisted migration markers, inherited variant
slots, and externally mapped vector values. `[CadDocumentStepExport]` reimports
two solids and checks analytic volumes and centers after per-body transforms.

`[ConfigContracts]` checks typed values, grouped serialization, ownership,
nullable options, equality and difference reports, and environment export with
restoration. `[ConfigGroupOrder]` requires strict lexicographic ordering for
identical, empty, and unequal-length integer groups. Header mutations require
recompiling consumers; an isolated header check must avoid archived weak inline
definitions masking the changed implementation.

The ownership cases check copy isolation and overwriting existing values through
both lvalue and rvalue merges. Resolver, typed string-key import, and mutable
indexed access cases check public API behavior; indexed writes are read back
through the separate const accessor. Their overwrite and index probes are in
`config_public_contracts.json`.

`[ConfigVariantRestore]` checks mapped vector slots, insertion, clamped removal,
short source vectors, and invalid inputs. `[ConfigSerialization]` checks escaped
strings and trailing empty vector entries. `[ConfigSubstitution]` checks typed
fallback values and recorded substitutions; `[ConfigMigration]` also checks a
minimal ConfigDef-backed alias through public deserialization. Build
`libslic3r_config_tests` for focused feedback. Its probes are in
`config_public_contracts.json`.

The registered print and CLI archive round trips use the existing
`libslic3r_config_variant_expansion_tests` target. They check default metadata
for every option and round-trip each distinct type/nullable dispatch. The
declared point/grouped archive, substitution-order, and vector-collection cases use
`libslic3r_config_tests`; their four value-corrupting `Config.hpp` probes are
listed in `config_option_archive_roundtrip.json`.

The Config JSON loading case checks flat and nested polygon groups and declared
vector aliases using the print setting schema with generic ConfigBase loading.
The fixture preserves each group and vertex and checks both aliased values.
Its probes are in `config_json_groups.json`.

`[ConfigGetter]` checks scalar and indexed values, inherited line width, and
percentage values resolved against their declared base. `[ConfigVectorOverride]`
checks regular and nullable slots, empty destinations, and mapped defaults.
The getter probes are in `config_getters.json`.

The PrintConfig additions have three independently named contracts:
`ASA-Aero legacy filament spelling is normalized on deserialization`
(`[Config][PrintConfigCompatibility]`), `Incomplete extruder ID maps resolve
variant slots with the configured stride` (`[Config][PrintConfigVariants]`),
and `Filament metadata getters return the first stored value`
(`[Config][Filament]`). Build `libslic3r_config_variant_expansion_tests` for
focused feedback; the two PrintConfig implementation probes select the first
two exact case names in `tests/mutations/printconfig_public_contracts.json`.

`Extra overhang perimeters add material in the unsupported step band`
(`[Perimeters][OverhangExtraPerimeters]`) compares the option-off control with
an analytic support/unsupported-band geometry and checks clipped path length,
material, and valid flow. Build `fff_print_perimeters_tests` for focused
feedback; its two implementation probes are in
`tests/mutations/extra_overhang_perimeters.json`.

`[Transformation][Skew]` checks perpendicular axes after scaling, rotation, and
reflection, and nonorthogonal axes under shear with either determinant sign.
Build `libslic3r_geometry_tests` for focused feedback; its probes are in
`geometry_transform_skew.json`.

`[TransformContract]` checks affine composition, normalized coordinate frames,
rotation about a fixed point, signed component access, axis setters, and resets.
Its probes are in `geometry_transform_components.json`.

`[TreeSupportPaintedEnforcer]` checks support extrusion near an independently
located painted vertical facet, with an unpainted control.

`[TreeSupport][TreeModelVolumes]` checks empty, below-first, exact, between, and
above-last collision-cache radius lookups. Returned radii and polygon references
must identify the closest cached area no larger than the request. The probe in
[`tree_model_volumes_lower_bound.json`](mutations/tree_model_volumes_lower_bound.json)
replaces the requested maximum radius with zero; the exact-radius assertion
detects the error.

`[TreeSupportBottomInterface]` checks emitted base/interface roles and alternating
directions above a lower model surface. Build `fff_print_tree_support_tests` for
focused feedback; the same cases run in the canonical FFF suite without duplicate
CTest registration. Its probes are in `tree_support_contacts.json`.

`[TreeSupportRoofContact]` checks zero-gap organic roof contact against an
analytic cap underside, with a 0.2 mm positive-gap control. A subminimum-area
overhang checks that zero-gap tree tips preserve contact-interface extrusion,
while the positive-gap control emits regular support one layer below. The
broad-cap mutation probe is in `tests/mutations/tree_support_roof_contact.json`.
`tests/mutations/tree_support_thin_roof_tip.json` removes only the zero-gap
trigger for recovering a subminimum roof through tree tips. With the
`TreeSupportRoofContact` CTest label, the thin-roof case fails its physical
interface assertion while the broad-cap control stays green.

`[TreeSupportRaftContact]` checks raft expansion on both a plain cube and a cap
overhang. The overhang case measures raft-contact interface extrusion in a known
ring outside the stem and retains a separate support witness under the cap.
`tests/mutations/tree_support_brim_width.json` challenges the explicit brim-width
footprint comparison by forcing the configured width to zero. This validates the
oracle without claiming that it uniquely detects the mutation.

`[ToolOrderUtils]` checks constrained flow capacities, group requirements,
minimum flush costs, and filament ordering. Small path instances use exhaustive
permutation oracles, including transitions into the next layer; large instances
check membership and the independently summed path cost. Mutation cases cover
edge restrictions, capacity defaults, group bonuses, and ordering decisions.

`[MeasureGeometryExtended]` and `[MeasureSkew]` check distances and closest
points against analytic circle geometry and independent angular minimization.
The cases include axial offsets, intersecting projected circles, nearly
concentric circles, reversed inputs, and common rigid transforms. Every closest
point must lie on the requested feature and agree with the reported distance.

`[MeshBooleanContracts]` checks analytic solid volumes, valid mesh indices,
remeshing, UV conformality, and conversion ownership. `[EmbossExtra]` checks
Euclidean radius boundaries, healed areas, font loading and cancellation,
shape transforms, and projection round trips. The radius regression distinguishes
a public distance from the squared distance required by the AABB query.

`[TextureBakeRelocate]` checks movement toward known linear contours, tangent
motion, displacement limits, gradient gates, locked vertices, welded copies,
and positive face orientation. Constant and stationary height fields exercise
no-op and finite-output behavior; auxiliary weights must survive unchanged.

`[Visvalingam]` checks polyline simplification against an independent linear
search for each least-area point, including endpoints, exact area thresholds,
translation, scaling, and salient turns. A default-comparator `pop_heap` mutation
remains an unclaimed survivor; later heap rebuilding masks it in these cases.
The production fix sizes the area array and uses the same area comparator for
heap construction and removal.

`[AABBMesh]` checks analytic ray intersections, closest points, unit normals,
seam deduplication, disconnected surfaces, and mesh ownership. `[LayerTextureContracts]`
checks packed RGBA and mip dimensions, interpolation limits, numeric guards,
palette values, and duplicated edge pixels. Its oversized canary lets a bounds
regression fail an assertion without corrupting the test process; sanitizer
validation also uses the exact GUI allocation.

`[AdaptivePAValidation]` checks complete decimal fields, physical line numbers,
blank lines, and strict PA/flow/acceleration ordering. `[TextureAtlas]` checks
encoded PNG and raw RGB/RGBA conversion, material mapping, patterned tiled UVs,
color swatches, cancellation, and truncated raw buffers. PNG decoding is exercised
through the encoded-image route used by model importers.

`[AdaptivePABridgeOverride]` checks zero and maximum bridge pressure advance on
one selected tool, with bridge and ordinary extrusion roles. Numeric pressure
advance commands must follow the setting while the ordered motion commands stay
unchanged. Its role, zero-value, and assignment mutations are in
`adaptive_pa_bridge_override.json`.

`[AdaptivePAModelVariance]` checks distinct nonlinear calibration models for two
filament config indices. `[AdaptivePAMalformedFallback]` and
`[AdaptivePANoCurveFallback]` require configured static pressure advance when a
model cannot supply a prediction. All cases preserve the ordered motion commands;
their probes are in `adaptive_pa_model_variance.json`.

`[TextureToColorExtended]` checks three-way vertex-color boundaries across cyclic
face orderings, preserves oriented area, and verifies the adaptive palette limit
on a closed mesh. Its mutations are in `texture_to_color_extended.json`.

`[TextureToColorOptions]` checks repair decisions, declined repair callbacks,
automatic subdivision thresholds, and adaptive vertex-color palette limits.
Geometry oracles preserve volume, consistent orientation, and signed area. Its
mutations are in `texture_to_color_options.json`.
The same tag checks fixed-palette color retention and adaptive palette values
after region smoothing. Those probes are in `texture_cluster_retention.json`.

`[TextureToColorPublic]` checks textured tetrahedron conversion and cancellation
output clearing. `[TextureToColorRepair]` checks independent boundary limits,
face orientation, degenerate-face removal, topology, volume, and cancellation
at each published stage. Its inline-header mutants require rebuilding every
affected consumer before testing. The repair pipeline cases check that
oversampling precedes the callback, the callback's closed topology and both
texture colors survive, and cancellation after repair prevents color clustering.
Late cancellation may retain partial output. These pipeline mutations are in
`texture_repair_pipeline.json`.

`[ColorUtilsStep3]` uses a four-face strip with an unresolved two-face component.
Swapping the component's face areas must reverse its selected neighboring color
while preserving both seeds and mesh geometry. Independent weighted RGB values
and large color-distance margins support the expected assignments. Its mutations
are in `color_component_weighting.json`.

`[CalibrationContracts]` checks PA command sequences, feedrates, bed extents,
annotation geometry, layer heights, and coordinate offsets. `[PresetAmsSync]`
checks physical tray mappings, placeholders, deduplication, mixed-filament
metadata, and vendor-specific nozzle hardness lookup. The hardness test populates
its fixtures before the first lookup; it does not claim cache invalidation.

`[AmsValidation]` checks AMS tray temperature reconciliation, per-nozzle
filament variants, and duplicate system material IDs through the public bundle
validator. Negative controls cover ignored tags, user/system scope, library
exclusions, distinct IDs, and printer compatibility.

`[SettingsReslice]` compares fresh, incremental, and restored slicing across
solid patterns, fill ordering, gap fill, bridge layers, and activated geometry
settings. The settings inventory distinguishes observable variants from negative
controls; combined expansion direction and close-holes behavior remain partial.

`[PrintObjectPainted]` checks painted facet projections against independent
Z-slab clipping and analytic areas, including transformed and combined volumes.
Bridge-contact checks retain supported anchors and split overlong spans. Six
mutation survivors and one process failure remain explicit, unclaimed experiments.

`[GCodeBeltApron]` checks extrusion in every prologue band, ordered layer markers,
first-layer speed and fan, filament selection, instances, and exact hook counts
before the first object layer. Disabling and inner-only controls must suppress
the leading apron; widening and lengthening must increase the emitted path.

`[NSVGUtils]` checks SVG import against analytic fill areas, stroke bounds,
dash periods, and a cubic Bézier area integral, along with lazy loading, source
IDs, holes, and coordinate transforms. Odd dash arrays must repeat after twice
the sum of their lengths; finer tessellation must approach the integral.

`[FuzzyRipple]` checks phase groups, full- and half-wave offsets, sampling,
amplitude, degenerate paths, and Arachne width metadata. A truth table covers
all fuzzy-skin selection modes across layer, loop, and contour choices. Open
ripple paths and mode-specific ripple width behavior remain unclaimed; the
current ripple dispatch ignores those parameters.

The export is summary-only because the report and integrity checks consume
file-level LLVM summaries; rerun `llvm-cov export` without `--summary-only` when
line or segment details are needed. Profile-merge and export diagnostics are retained
in the report even when the LLVM commands succeed. A nonzero function-profile
mismatch count fails the integrity gate, because LLVM can omit those functions
from the denominator while returning success. Investigate these warnings with
`llvm-cov export --dump` against the same binaries and profile. A zero-summary header that the successful
object audit proves has no executable mapping is recorded as
`mapping_free_headers`. A header with an executable object mapping but no final
mapping remains an integrity failure. Reports expose `integrity_status`
separately from test status, so a failed test run cannot hide a coverage
integrity error.

Compiler-object inventory uses a separate empty indexed profile generated by
`llvm-profdata`, including on LLVM 18. Its artifact hashes and merge diagnostics
are recorded as `compiler_inventory_profile`; it supplies mapping presence
only. Runtime coverage still comes from the test-generated profiles, and
failure to generate the inventory profile fails the report.

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

Automated launchers can use `--build-argv-file` and `--test-argv-file` instead
of command strings. Each file contains a JSON array of literal arguments; paths
are relative to `--root` unless absolute. The same `{root}`, `{build}`, and
`{filter}` placeholders apply. A test command can also pass `{junit}` to CTest's
`--output-junit` option. The runner replaces it with a unique retained XML path
beside that test command's captured log and records the artifact path, SHA-256,
size, testcase counts, and validation result. A missing, malformed, incomplete,
or count-mismatched requested JUnit file makes that test evidence incomplete and
cannot classify a mutation as killed. Omitting `{junit}` preserves the existing
command behavior. Empty arguments after the executable are retained, and the
preflight report records each argument file's path and SHA-256.
For a CTest command launched through a wrapper such as `env`, include
`--output-on-failure` and `-L {filter}` explicitly in its argument array.
Automatic CTest option insertion inspects the first executable.

Pin a reviewed catalog with `--mutants-sha256 <64-character SHA-256>`.
Repeat this option once per `--mutants`, in the same order. The runner checks
the raw catalog bytes before repository checks or compilation, selects mutants
from those verified bytes, and records the manifest paths and digests in the
preflight report. A mismatched digest fails the run without building.

`mutation-report.json` records the commit, worktree commands, exact source
match and replacement, build/test commands, and separate statuses for build
errors, timeouts, killed mutants, and survivors. A mutant is counted only
after its selected unmutated baseline passes, the same number of tests executes,
and Catch2 reports a positive assertion-failure count. CTest receives
`--output-on-failure` so that this assertion evidence is retained. Child crashes,
fatal signals, unexpected exceptions, and nonzero exits without an assertion
summary are infrastructure or unverified failures, not mutation kills. Custom
test commands must retain Catch2's console summary for kill classification.

Use a new or empty output directory. Configure, build, and test output is written
to separate files, with their hashes and sizes recorded in the report. Test
classification scans the complete log with bounded memory; an overlong line
makes the evidence incomplete. Atomic checkpoints retain completed baselines
and mutants while marking an unfinished run `in_progress`. Cleanup failures
and execution errors take precedence over survivors in the final status.

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

For a dedicated worker checkout with an existing build, add
`--reuse-detached-checkout` to keep the same source and build paths. This mode
requires a clean detached HEAD and takes an exclusive lock in that checkout's
Git directory. It cannot be combined with `--persistent-worktree` or
`--work-root`. An existing lock stops the run for inspection. The default
continues to use a disposable worktree.

In explicit build/test argv mode, `--persistent-worktree` and
`--reuse-detached-checkout` restore source bytes while retaining build outputs,
which may still contain the last mutant. Rebuild the baseline before using
those outputs for other tests or coverage; tracked source cleanliness alone
does not prove binary restoration.

For an already configured worker build, target mode derives its executable
from CMake File API and its test commands from post-build CTest discovery.
Enable `SLICER_ENABLE_FOCUSED_PR_CTEST_DISCOVERY` when configuring the focused
targets. From a clean detached checkout, a Wipe Tower run is:

```sh
python3 scripts/slicer_mutation.py run --root . --build-dir build \
  --reuse-detached-checkout --target fff_print_wipe_tower_tests \
  --config Release --jobs 2 --control-regex '^FocusedWipeTower::' \
  --exclude-label-regex 'NotWorking|RequiresApp' \
  --expected-control-inventory tests/controls/wipe_tower.json \
  --mutants tests/mutations/wipe_tower_filament_speed_boundaries.json \
  --max-mutants 2 --out build/mutations/wipe-filament-speeds
```

Use `fff_print_gcodeprocessor_tests`, `^FocusedGCodeProcessor::`,
`tests/controls/gcode_processor.json`, and
`tests/mutations/processor_min_rate_modes.json` for the processor cohort.
Pass `--cmake` and `--ctest` when the configured worker needs specific tools.
Keep the worker's existing CPU, memory, swap, and locale limits.

Target mode runs the exact focused control before any mutant, shares one
baseline per catalog filter, and finishes by rebuilding and checking the normal
executable and full control. A failed restore fails the run. It retains command
receipts, JUnit, and baseline/mutant/restored executable identities in the report.
Do not combine target mode with explicit build/test commands. Update the
checked-in control inventory only for intentional case additions or removals;
fresh discovery is checked against that specification, not used to redefine it.

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
representative values) plus a pipeline oracle. Nullable settings also require
`nullable_behavior`: setting-specific evidence for nil and explicit values
through the resolved pipeline. Nil does not necessarily mean inheritance;
the oracle must establish that setting's behavior. A source mention,
`full_print_config()` construction, or serialization round-trip does not
count. `--fail-on-uncovered` is available when a future evidence map is ready
to enforce a complete settings gate.

The native registry audit compares every compiled `PrintConfigDef` key with
the static inventory and fails on omitted or spurious settings. Complete
metadata reconciliation also validates type IDs and exposes effective nullable
and enum metadata without replacing the source-parser fields. Required evidence
and counts use the effective nullable metadata. A key-only registry does not
verify metadata. Omitting `--runtime-registry` produces a static report with an
unverified registry.

The `[InputShaping]` export cases exercise all registered shaper choices as
serialization inputs. Exact RepRapFirmware type tokens are checked only for
the documented modern subset; this does not establish firmware acceptance of
every registry choice. Marlin 2 cases cover independent X/Y values, inherited
zero values, configured endpoints, and the disable command.

One known compatibility gap remains: the current writer emits
`M593 F0.00 S0.000` for RepRapFirmware `Disable`. In pinned firmware source
[`a8bc59d`](https://github.com/Duet3D/RepRapFirmware/blob/a8bc59db51bb56fe8e3e9d45fea3d573d1f8d073/src/Movement/AxisShaper.cpp#L79),
disabling requires selecting `P"none"`, and the frequency minimum is 4 Hz.
The [firmware documentation](https://docs.duet3d.com/User_manual/Reference/Gcodes/M593)
describes zero-frequency disabling only for older versions. The tests do not
require the defective modern encoding or claim to validate firmware execution;
version compatibility still needs a production fix and an executed regression.

## Exact-head PR validation

`[KnownBaselineDefect]` identifies explicit production regressions:
`DynamicPrintConfig::set_num_filaments()` does not resize filament vectors because
`PrintConfigDef` never initializes its filament-option registry. The case retains
the desired resize assertion and uses Catch2's `[!shouldfail]`; an unexpected pass
also fails, requiring the marker to be removed when the production defect is fixed.
Report this expected failure separately from passing behavior. It provides no
validated resizing or settings-variance credit. The active
`PresetBundle::set_num_filaments()` path has separate passing tests.

`[PressureEqualizerCoordinateResetDefect]` retains the physical-equivalence
assertion for combined `G92 X0 Y0 E0` and `G92 X0 Y0 E5` resets. The pressure
smoother currently classifies extrusion from logical coordinate deltas without
checking the command type, so the second reset changes subsequent smoothing
despite representing the same physical block. This diagnostic also uses
`[!shouldfail]` and provides no validated reset-behavior or settings credit.
The E-only reset contract remains a separate ordinary test and mutation baseline.

`scripts/slicer_pr_validation.py` validates the exact head OIDs in the
externally provided `build/evidence/open-prs.json` snapshot. It first verifies
that the complete harness at `--harness-ref` passes. It then applies the
selected diff from `--harness-base..--harness-ref` to each PR's exact
`baseRefOid` and `headRefOid` in a disposable worktree. Both snapshot fields
must contain full commit OIDs. Each head is compared with its own base;
`--harness-base` determines the test overlay, not the comparison reference.
The default `--patch-scope test-only` includes tests, scripts, `cmake/`, the
root `CMakeLists.txt` and `.gitignore`. It excludes production bug fixes from
the testing branch, and verifies the `src/` index hash is unchanged by the
overlay. The report records every applied and excluded path.

If `CCACHE_CONFIGPATH` is set, use an absolute path to a readable regular config
file; symlink targets are accepted. The runner checks it before resolving tools
or starting native work, because child commands use worktree and build directories
where relative paths would change meaning.

Use `--patch-scope integration` to also include the two source CMake files,
or `--patch-scope full` for the full branch diff. Repeated `--patch-include`
values add literal paths or directories explicitly. Results from these broader
scopes describe the combined changes, rather than an unchanged PR source tree.
A patch conflict is recorded as a failure for that head and is never resolved
automatically. The runner does not write the original checkout; if its observed
state changes during a run, the report records that fact without attributing
who changed it and exits nonzero.

Use `--keep-worktree` to preserve the validation checkout and its native build
after the run. The retained location is recorded in `report.json` under
`cleanup.path`.

Use `--reuse-worktree-from-report prior/report.json` with a new output directory
to continue with that build. The runner verifies the retained checkout against
the terminal report, locks it, and rejects unexpected edits or tracked paths
that would overwrite the build. It preserves build outputs through every tree
change, including changes to ignore rules. Baseline, reference, and head tests
run again; previous JUnit results do not satisfy the new run.

After a successful configure and build, locked reuse also preserves timestamps
for tracked overlay files whose bytes, size, and permissions still match after
reapplication. Changed files retain their new timestamps. The report records
eligibility, skipped paths, and restored file/byte counts under
`build_input_mtime_preservation`. Configure, build, and tests still execute.
Use a consistent `umask 022` before the initial checkout and every subsequent
reset and overlay application (`UMask=0022` for a systemd service). A more
permissive mask can recreate an otherwise identical file as mode `0664` after
it was built as `0644`; the permission guard then correctly skips timestamp
reuse and the build repeats. Do not change modes in a live validation checkout.

Use a new or empty output directory for every run. Before building, the runner
records the snapshot SHA-256 and verifies every selected head and base resolves
to its exact commit. Configure, build, and test output goes directly to log files;
the JSON report keeps only a bounded tail. Run the controller under the resource
limits described above, including a file-size limit for those logs.

Both validation and mutation reports use `status: in_progress` during execution.
Active baseline, reference, head, and mutant rows are recorded before their child
commands start. An unfinished report is a checkpoint, not proof that its process
is still alive; inspect the controller process or service before deciding whether
to wait or recover. Completed outcomes retain their existing classifications.

A passing head requires complete JUnit evidence whose testcase count agrees
with CTest. Missing or contradictory reports and duplicate testcase identities
are incomplete evidence. Crashes, timeouts, and JUnit errors are execution
failures; they do not enter the new/inherited/resolved test-failure comparison.
Both the reference and the head must have comparable execution evidence.
`--minimum-tests` adds an independent positive testcase floor (default `1`);
set it to the expected suite size for a validation campaign. Every passing
testcase identity from the harness baseline must also appear in the reference
and each head. Equal totals cannot hide a missing test replaced by another name.
A requested reference with incomplete execution evidence makes the overall
matrix incomplete, even if individual heads pass.

The report's `references` map binds reference evidence to exact base OIDs.
Shared bases run once. Within the same invocation, an exact commit already
tested as a head or reference may supply the other role under the identical
overlay and execution configuration. This reuse is disabled when an identity
manifest is active, because head-specific exceptions may differ.

For PRs based on older test suites, `--identity-manifest <path>` accepts an
audited JSON manifest with `schema_version: 2`, exact `harness_base_sha` and
`harness_ref_sha`, `required_harness_cases`, named `exception_groups`, and one
`heads` entry per snapshot PR. Each head binds its `number`, `head_oid`,
`exception_group`, and `required_head_cases`. Each case binds a JUnit
`runtime_identity` to its source `path`, `macro`, `name`, and `line`.
Only original base cases absent by name from both the exact head and its applied
overlay may be exceptions. The head's baseline identity set and minimum count
are reduced by those verified exceptions; the reference stays strict.
New harness cases and every declared PR-added case remain mandatory. Moved
tests, stale commits or source lines, missing PR additions, skips, and duplicate
JUnit identities cannot be waived. The source check supports literal
`TEST_CASE` and `SCENARIO` declarations; escaped names are rejected explicitly,
and SCENARIO runtime names include Catch's `Scenario:` prefix. Without a
manifest, the original strict identity and count checks remain in effect.

Run all snapshot heads, or select reproducible PR numbers with repeated or
comma-separated `--numbers` values:

```sh
umask 022
DEPS_PREFIX=/path/to/OrcaSlicer-deps/usr/local
UV="$(command -v uv)"
python3 scripts/slicer_pr_validation.py \
  --snapshot build/evidence/open-prs.json \
  --harness-base main \
  --harness-ref codex/slicer-confidence-tests \
  --deps-prefix "$DEPS_PREFIX" \
  --uv-executable "$UV" \
  --output-dir build/evidence/pr-validation \
  --jobs 4 --test-jobs 8 \
  --cmake-arg=-DSLICER_TEST_PROCESSORS=1
# Example focused rerun:
python3 scripts/slicer_pr_validation.py \
  --snapshot build/evidence/open-prs.json \
  --harness-base main \
  --harness-ref codex/slicer-confidence-tests \
  --deps-prefix "$DEPS_PREFIX" \
  --uv-executable "$UV" \
  --numbers 10,12 48 \
  --output-dir build/evidence/pr-validation-selected \
  --jobs 4 --test-jobs 8 \
  --cmake-arg=-DSLICER_TEST_PROCESSORS=1
```

`--jobs` limits compiler parallelism; `--test-jobs` independently sets CTest
scheduling slots and defaults to `--jobs` when omitted. The examples keep four
compiler jobs while allowing eight test slots within the resource boundary
described above. Test affinity reserves two slots for `[Parallel]` cases.

`--test-timeout` sets CTest's default per-test deadline (180 seconds); test
properties may override that default. `--timeout` separately bounds each
configure, build, and complete test phase (3600 seconds). Both values are
recorded in the report. Individual timeouts are listed in
`timed_out_test_names`, separately from assertion failures; an outer phase
timeout is identified by `test_timeout_source`.

The native defaults configure Release/Ninja with `SLIC3R_GUI=OFF`,
`BUILD_TESTS=ON`, `SLIC3R_CAD=ON`, build the five slicer test targets, and run
CTest while excluding `NotWorking|RequiresApp`. Use repeated `--cmake-arg`
or `--target` options for a deliberate override. The JSON report records the
harness guard, production references and every exact head, tested tree SHA,
source-index hashes, configure/build/test commands, elapsed time, logs,
JUnit output, and cleanup result. The reference baseline runs by default;
its failures do not prevent PR tests from running after a green harness guard.
JUnit comparison records failures seen on both reference and PR, newly failing
cases, resolved cases, and cases that cannot be compared. Shared failing test
identities do not establish a shared root cause; raw failures remain visible. A head is not marked
passing if the baseline fails, configuration/build/test fails, no tests run,
tests are skipped, or the harness patch conflicts; the process exits nonzero
when any selected head or the baseline is not a verified pass.

`[ShortEdgeCollapse]` checks the no-op face-budget boundary and verifies valid,
nondegenerate output for closed and open indexed meshes. Its two CPP probes
challenge the boundary guard and final face-index remapping.

`[ArchivePathMetadata]` checks ZIP Unicode-path version and CRC trust, UTF-8
filename precedence, and fallback for truncated extra fields.
`[ArchiveDiagnostics]` checks recognized archive error codes through the public
error-string API without freezing English wording.

`[SurfaceCollection]` checks metadata and hole preservation during
simplification, grouping by merge properties, source-order filtering and
removal, contour accumulation, and SVG colors and optional labels. Each
subsystem has a separate two-probe CPP catalog so the mutation build stays on
its focused executable: `tests/mutations/short_edge_collapse.json`,
`tests/mutations/miniz_paths.json`, and `tests/mutations/surface_collection.json`.
Each catalog is independently SHA-256 pinned and selects exact case names.


## Focused feedback: Surface, adaptive bed mesh, and texture placement

These explicit targets support focused iteration without duplicate CTest
registrations. Run them from the repository root:

```sh
cmake --build build --config Release --target fff_print_adaptive_bed_mesh_tests
build/tests/fff_print/fff_print_adaptive_bed_mesh_tests "[AdaptiveBedMesh]" --rng-seed 21907 --order rand

cmake --build build --config Release --target libslic3r_surface_collection_tests
build/tests/libslic3r/libslic3r_surface_collection_tests "Surface conversions preserve contour and hole order across copies and moves" --rng-seed 21907 --order rand
build/tests/libslic3r/libslic3r_surface_collection_tests "Surface collections preserve geometry and metadata across set append and retag" --rng-seed 21907 --order rand
build/tests/libslic3r/libslic3r_surface_collection_tests "Surface extents include negative and disjoint contours and define empty results" --rng-seed 21907 --order rand
build/tests/libslic3r/libslic3r_surface_collection_tests "Surface SVG export preserves holes colors and requested opacity" --rng-seed 21907 --order rand

cmake --build build --config Release --target libslic3r_texture_displacement_tests
build/tests/libslic3r/libslic3r_texture_displacement_tests "Joining charts aligns a reversed shared edge to the placed parent" --rng-seed 21907 --order rand
build/tests/libslic3r/libslic3r_texture_displacement_tests "Averaging island scales preserves placement and handles small inputs" --rng-seed 21907 --order rand
build/tests/libslic3r/libslic3r_texture_displacement_tests "RGB texture smoothing preserves flat channels and reduces image contrast" --rng-seed 21907 --order rand
```

The Surface and Texture probes exercise public geometry/placement contracts; the
adaptive bed-mesh cases check emitted placeholders against an analytic first-layer
island envelope. These focused runs do not replace the canonical suite or establish
full settings variance.

The Arachne `[SkeletalPublicAPI]` contracts check nonlinear width thresholds through
factory wrappers, an analytic taper width floor, and balanced variable-width
paths through a narrow neck. Build `libslic3r_arachne_tests` for focused runs;
`tests/mutations/arachne_width_markers.json` challenges marker forwarding and the
extra-rib generation that consumes those markers.

The `[FlushVolPredictor]` contracts check sRGB/D65 color conversion, published
CIEDE2000 reference pairs, and directional predictions from all three bundled
purge datasets. Build `libslic3r_flush_volume_tests` for focused runs. Its two CPP
probes are in `tests/mutations/flush_volume_predictor.json`. The four existing
`[FlushVolCalculator]` contracts remain in the same file and focused executable.
These direct algorithm contracts do not establish complete settings variance.

## PNG, orientation, and layer SVG contracts

The PNG writer and scaling contracts check grayscale and RGB pixel values across the public overloads, row orientation, nearest-neighbor scaling, invalid signatures, and a safe file-open failure. Build `libslic3r_png_tests` for focused feedback; its probes are in `tests/mutations/png_write_scale.json`.

```sh
cmake --build build --config Release --target libslic3r_png_tests
build/tests/libslic3r/libslic3r_png_tests "Grayscale PNG writer overloads preserve pixels and nearest-neighbor scaling" --rng-seed 21907 --order rand
build/tests/libslic3r/libslic3r_png_tests "RGB PNG writer overloads preserve channels and decoded row order" --rng-seed 21907 --order rand
```

The orientation fan-direction tests use asymmetric and symmetric solids to check the public cooling-axis behavior. Build `libslic3r_orient_tests` for focused feedback; the probes are in `tests/mutations/orient_fan_direction.json`.

```sh
cmake --build build --config Release --target libslic3r_orient_tests
build/tests/libslic3r/libslic3r_orient_tests "Parallel fan orientation aims an exposed slope toward the configured cooling axis" --rng-seed 21907 --order rand
build/tests/libslic3r/libslic3r_orient_tests "Parallel fan orientation keeps the configured axis for a symmetric solid" --rng-seed 21907 --order rand
```

Layer and region SVG contracts verify that each public exporter uses its matching slice or fill collection, retains a disjoint fill island with a hole, and emits the documented bounds, outlines, and opacity. These cases are part of the FFF workflow tests and their probes are in `tests/mutations/layer_svg_contracts.json`.

```sh
cmake --build build --config Release --target fff_print_workflow_tests
build/tests/fff_print/fff_print_workflow_tests "Layer and region SVG exports reflect their public surface collections" --rng-seed 21907 --order rand
```


## Focused feedback: TriangleMeshSlicer contour repair

`[TriangleMeshSlicer][SinglePlaneSlice]` checks public single-plane slicing of
a closed rectangular box whose triangle vertices are deliberately split by
index. It requires one contour with analytic 20 by 14 mm bounds and 280 mm²
area, including a mixed-winding facet variant. Its probes are in
`tests/mutations/trianglemesh_slicer_open_contour.json`. Build and run the two
exact cases with:

```sh
cmake --build build --config Release --target libslic3r_trianglemesh_slicer_tests
build/tests/libslic3r/libslic3r_trianglemesh_slicer_tests "Single-plane slicing closes a box with coincident split triangle vertices" --rng-seed 21907 --order rand
build/tests/libslic3r/libslic3r_trianglemesh_slicer_tests "Single-plane slicing joins reversed split facets into one contour" --rng-seed 21907 --order rand
```

## Focused feedback: extrusion entities

`[ExtrusionEntity]` checks multipath footprint bounds against explicit path
widths, preservation of contoured vertex-height pairs through clone and
reversal, loop/scarf clipping, seam-gap trimming, and loop smoothness. Its two
probes are in `tests/mutations/extrusion_entity_contracts.json`. Build and run
the focused target with:

```sh
cmake --build build --config Release --target fff_print_extrusion_entity_tests
build/tests/fff_print/fff_print_extrusion_entity_tests "[ExtrusionEntity]" --rng-seed 21907 --order rand
```

## Elephant-foot settings pipeline

`[ElephantFoot]` checks contour compensation fade over configured layers and
the solid-fill density ramp using a small FFF print and measured slice/fill
geometry. Its two source probes are in
`tests/mutations/elephant_foot.json`. This case runs in the existing
`fff_print_tests` target.

The TriangleMeshSlicer and ExtrusionEntity focused targets add no duplicate
CTest registrations.

## Focused SVG export contracts

The `libslic3r_svg_tests` target is excluded from CTest to avoid duplicate registrations. Its `[SVGExportContracts]` cases check nested-path/open failures, surface collection forwarding, tapered primitive geometry, color fallbacks, and the numeric three-column legend layout. Build and run the focused cases directly:

```sh
cmake --build build --config Release --target libslic3r_svg_tests
build/tests/libslic3r/libslic3r_svg_tests "[SVGExportContracts]" --rng-seed 21907 --order rand
```

## GCC gcov coverage backend

The opt-in `SLICER_ENABLE_GCC_GCOV_COVERAGE` backend is mutually exclusive
with LLVM source coverage. It instruments the same two production archives
and five canonical test targets, uses atomic GCC counters, and filters compiler
instrumentation to the configured checkout's `src/` tree. It records GCC's
physical source-line mapping as a separate backend-specific measurement. GCC
and LLVM can map different physical lines, so a gcov denominator must never be
substituted for an LLVM denominator or presented as directly comparable to an
accepted LLVM percentage. Keep `--source-root src` when the report should also
retain project sources outside `src/libslic3r`.

When a target's compiler launcher is `ccache`, the module adds `--ccache-skip`
immediately before GCC's source-filter option. This lets ccache pass the option
through while retaining it in the cache key; GCC receives the same coverage,
atomic-counter, and source-filter flags. Other compiler launchers are unchanged.
This does not change ccache's requirements for precompiled headers. Check the
generated launcher arguments on Linux with
`python3 -m unittest discover -s scripts/tests -p test_slicer_coverage_cmake.py`.

Configure with GCC, `-DSLICER_ENABLE_GCC_GCOV_COVERAGE=ON`, and the usual
Release/test options. Build `libslic3r_tests`, `fff_print_tests`,
`sla_print_tests`, `libnest2d_tests`, and `filament_group_tests`, then run the
full selected suite into a new, empty output directory. Pass the two production
manifests and all five test manifests so each compiled translation unit has an
explicit notes-file inventory:

```sh
python3 scripts/slicer_coverage.py run --backend gcov \
  --root . --build-dir build --source-root src \
  --object build/tests/libslic3r/libslic3r_tests \
  --object build/tests/fff_print/fff_print_tests \
  --object build/tests/sla_print/sla_print_tests \
  --object build/tests/libnest2d/libnest2d_tests \
  --object build/tests/filament_group/filament_group_tests \
  --manifest build/slicer-coverage/libslic3r.sources.txt \
  --manifest build/slicer-coverage/libslic3r_cgal.sources.txt \
  --manifest build/slicer-coverage/libslic3r_tests.sources.txt \
  --manifest build/slicer-coverage/fff_print_tests.sources.txt \
  --manifest build/slicer-coverage/sla_print_tests.sources.txt \
  --manifest build/slicer-coverage/libnest2d_tests.sources.txt \
  --manifest build/slicer-coverage/filament_group_tests.sources.txt \
  --compile-commands build/compile_commands.json \
  --test-command 'ctest --test-dir build/tests -C Release --no-tests=error -LE "NotWorking|RequiresApp"' \
  --metric lines --out build/gcov-coverage-run-1
```

The runner binds the CTest discovery list to JUnit identities, preserves raw
stdout/stderr and test profiles, inventories every selected `.gcno` and any
unselected build notes, and retains mapping-free translation units explicitly.
It deduplicates gcov line records by resolved physical path and line number,
including duplicates within a record and header mappings across translation
units. The report records source hashes and the matching GCC/gcov versions.
Threshold status applies only to the gcov physical-line metric; a valid result
below threshold is still a measurement, not an LLVM result.

## Nullable motion variants and boundaries

The `[NullableMotionBatch]` contract loads real parent/child process presets, maps
inherited and explicit values to two tools, and checks travel, bridge, infill and
surface commands without changing deposited geometry or material. The
`[MotionBoundaries]` contracts exercise the registered travel-speed minimum and zero
acceleration/jerk values against distinct positive controls. They distinguish an
inherited printer state from an explicit role fallback, including top-surface jerk
falling through to infill jerk before default jerk.

`tests/mutations/motion_nullable_boundaries.json` supplies four reproducible
selector/fallback probes. Its `test_filter` values are CTest label expressions (`-L`);
direct Catch runs use tags or test-case selectors.

`[MotionRoleFallback]` checks zero outer-wall and internal-solid acceleration,
plus zero outer-wall, inner-wall and infill jerk, against distinct positive
controls and explicit fallback commands. `[NullableMotionObject]` exercises
absent, nil and explicit object overrides on those roles and on actual internal
bridges, preserving positive deposited material and geometry.

`[NullableFirstLayerSpeed]` resolves nil and explicit first-layer perimeter and
infill speeds through real parent/child process presets. Reversed tool mapping
requires nonempty role feeds on both tools to match their resolved slots.
`tests/mutations/motion_object_boundaries.json` probes the zero-acceleration
fallback source and first-layer tool-slot selection; its filters are CTest label
expressions.

`[InitialLayerJerk]` and `[NullableInitialLayerJerk]` check first-layer jerk
fallbacks and object inheritance. `[SmallPerimeterSpeed]` includes minimum and
zero-auto speeds; `[NullableSmallPerimeterSpeed]` resolves inherited and explicit
values across two tools. `[RetractionSpeedRounding]` checks machine and filament
speeds around their integer rounding transition, including modal G-code feeds
and zero deretraction fallback. `[InterlockingVariance]` includes minimum depth
and beam width on bounded geometry, plus physical equivalence at 0/360 degrees.
`tests/mutations/motion_interlocking_boundaries.json` supplies six assertion-based
probes for these contracts, using CTest label expressions.

The Wipe fixtures check zero retraction length as a disable value and an exact
5 mm minimum-travel decision before G-code coordinate rounding. The Z-hop cases
compare inclusive layer-plane bounds with adjacent blocked controls, retaining
unaffected lifts and deposited material. Each boundary exercises machine values
and nullable filament overrides. `tests/mutations/retraction_boundaries.json`
challenges the three inclusive comparisons using the `Wipe` CTest label.
