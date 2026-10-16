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
  -p RuntimeMaxSec=1800 -p LimitFSIZE=4G -p TimeoutStopSec=2 \
  -p KillMode=control-group -p OOMPolicy=kill \
  cmake --build build --config Release --parallel 8 --target libslic3r_tests
```

Place the entire Python controller or CTest command inside the service so its
descendants share the limits and are stopped when it exits. Use smaller limits
for focused tests, such as 4 GiB, two CPUs, and a five-minute service deadline.
`LimitFSIZE` limits each output file, including libraries and coverage exports;
leave enough room for the expected artifacts and check free storage before a
run. Verify parent-death, memory-limit, and timeout cleanup with small synthetic
processes when adopting a new worker. Preserve failed logs and binaries before
rebuilding; an interrupted run is incomplete evidence.

Serialize heavy controllers on a shared worker. On tmpfs, retained build and
mutation artifacts also consume memory charged to the cgroup; check both free
storage and the shared slice's memory use between runs. Archive successful
artifacts with verified hashes before pruning them, and retain failures for
diagnosis. An OOM termination is an infrastructure failure, never a mutation kill.

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

`[SupportCache]` checks support-layer cache round trips through `Print`, including
island coordinates, extrusion geometry, interface metadata, layer links, and
malformed payload rejection. `[WipeTowerContact]` checks parsed feedrates,
interface temperature commands, pre-extrusion lengths, and bed clipping through
the tower generator; these settings claims remain separate from full Print wiring.

`[FilamentPlanContracts]` checks physical nozzle identity with repeated loaded
filament values, filtered extruder groups, multiple assignment epochs, and
independently enumerated flow costs. Empty-nozzle inputs have an explicit error
contract, while the legacy solver continues to return filament values.

`[ModelImport]` uses actual STL, OBJ, AMF, and 3MF inputs and checks geometry,
source metadata, colors, decoded texture pixels, cancellation, and error types.
`[EmbossLayout]`, `[TriangleCursor]`, and `[PolylineOperations]` use analytic
geometry and transformation invariants, including integer-grid partitioning
and translation far from the coordinate origin.

`[SplitModel]` and `[AMFArchive]` cover compressed model parts, physical
placement, mesh ownership, metadata, and malformed inputs. `[GcpPost]` checks
rewritten G-code against independently calculated time, filament volume,
mass, and cost. `[PyramidWinding]` checks oriented topology and analytic
volume under translation; `[SLADrainTransforms]` checks normals against
transformed surface tangents under rotation, unequal scaling, and printer
correction. `[ColorUtilsExtended]` checks clustering, palette preservation,
UV correspondence, mesh closure, and remeshing invariants.

`[SettingsFamilyVariance]` connects support speeds, bridge flow, overhang
speed bands, ironing overrides, and infill connection limits to emitted
motion or geometry. Its mutation manifest retains two `known_survivors`
for requested infill-anchor length and clamping. They are documented gaps,
excluded from the executable `mutants` list and from coverage claims for
those dimensions; they must not be reported as assertion kills.

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

`[MultimaterialWipe]` checks direct WipeTower2 purge matrices, wall geometry,
ramming, cooling, stamping, loading, and delay commands. Independent cooling
variants change one setting at a time, with feedrates read from executable
G-code. These direct-consumer checks do not establish complete Print pipeline
coverage; joint variations and numeric-domain gaps remain explicit in the
settings map. `[WipeTowerBroad]` covers tower estimators, sparse-layer folding,
role-specific filament selection, and object/support wiping eligibility.

`[ConfigContracts]` checks typed values, grouped serialization, ownership,
nullable options, equality and difference reports, and environment export with
restoration. `[ConfigGroupOrder]` requires strict lexicographic ordering for
identical, empty, and unequal-length integer groups. Header mutations require
recompiling consumers; an isolated header check must avoid archived weak inline
definitions masking the changed implementation.

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
