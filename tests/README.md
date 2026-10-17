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
  -p RuntimeMaxSec=1800 -p LimitFSIZE=4G -p LimitCORE=0 -p TimeoutStopSec=2 \
  -p KillMode=control-group -p OOMPolicy=kill \
  cmake --build build --config Release --parallel 8 --target libslic3r_tests
```

Place the entire Python controller or CTest command inside the service so its
descendants share the limits and are stopped when it exits. Use smaller limits
for focused tests, such as 4 GiB, two CPUs, and a five-minute service deadline.
`LimitFSIZE` limits each output file, including libraries and coverage exports;
leave enough room for the expected artifacts and check free storage before a
run. Disable core dumps so an assertion failure cannot fill the worker's disk.
Keep build trees, temporary files, profiles, and compiler caches on disk-backed
storage; tmpfs retains these artifacts in RAM even after their processes exit.
Verify parent-death, memory-limit, and timeout cleanup with small synthetic
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

Coverage builds include Orca's source headers as ordinary project headers.
Clang suppresses system-header mappings even when it emits profile counters;
mixing those counters with unused inline mappings can silently remove executed
functions from an export. Dependency headers keep their existing classification.
Coverage targets retain Clang's unused-private-field diagnostics as warnings:
project-header classification exposes existing fields that normal system-header
classification hides. Other warning categories retain their existing policy.

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
`[ObjectSpeedNullable]` checks four object speed overrides through a
nil, explicit, then nil transition on the same print. Fresh parent and explicit
controls verify geometry, material, and feeds for standard and high-flow nozzle
variants. Its mutations are in `object_speed_nullable.json`.
`[FilamentFlowVariance]` and `[FilamentMaxVolumetricSpeed]` check deposited
material and volumetric feedrate limits, including fresh/incremental agreement.
`[LayerGeometrySettings]` checks initial height, independent shell thicknesses,
and both registered surface-density bounds through emitted extrusion roles.
`[GCodeTravelPointReachable]` checks the six toolchange route placeholders
against bounded obstacle geometry and configured endpoints.
`[ValidationContracts]` exercises rejected configurations and warning gates;
that evidence is declared separately from sliced-output coverage.

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

`[SettingsSpeedAccelVariance]` checks first-layer and role-specific feedrates,
acceleration, and jerk against every eligible parsed move. Layer markers keep
first-layer travel separate from later motion. Its direct firmware-writer
checks are identified as writer evidence in the settings map; they do not
establish a complete print-pipeline sweep or full numeric-domain coverage.
`[SmallPerimeterSpeed]` checks absolute and percentage speeds against every
later-layer outer-wall move on a six-layer fixture, with a disabled-threshold
control and conserved material. Its mutations are in `small_perimeter_speed.json`.

`[PresetBundleImport]` imports an actual ZIP bundle with process, filament, and
printer profiles, then checks metadata, canonical identities, typed settings,
and persisted per-type files. Its mutations are in `preset_bundle_import.json`.

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

`[PresetBundleFullFffMaps]` and `[PresetBundleFullFffProvenance]` check explicit
filament-volume map overrides and selected system-profile changes, with
mutations in `preset_bundle_full_fff.json`.

`[SupportCache]` checks support-layer cache round trips through `Print`, including
island coordinates, extrusion geometry, interface metadata, layer links, and
malformed payload rejection. `[WipeTowerContact]` checks parsed feedrates,
interface temperature commands, pre-extrusion lengths, and bed clipping through
the tower generator; these settings claims remain separate from full Print wiring.

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

`[MultimaterialWipe]` checks direct WipeTower2 purge matrices, wall geometry,
ramming, cooling, stamping, loading, and delay commands. Independent cooling
variants change one setting at a time, with feedrates read from executable
G-code. These direct-consumer checks do not establish complete Print pipeline
coverage; joint variations and numeric-domain gaps remain explicit in the
settings map. `[WipeTowerBroad]` covers tower estimators, sparse-layer folding,
role-specific filament selection, and object/support wiping eligibility.
`[CompactedTower]` checks pre-slice collision diagnostics for zero, one, and two
objects in the rod's clearance band, plus the disabled-setting control. It
checks the reported height and selected object without slicing; its mutations
are in `compacted_tower_clearance.json`.

`[StaticEnumDefaults]` checks that static and dynamic default configurations retain
named enum lists before and after public deserialization. Its mutation is in
`static_enum_defaults.json`; default-name assertions precede parsing so missing
metadata produces an assertion failure instead of a crash.

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

`[TextureToColorPublic]` checks textured tetrahedron conversion and cancellation
output clearing. `[TextureToColorRepair]` checks independent boundary limits,
face orientation, degenerate-face removal, topology, volume, and cancellation
at each published stage. Its inline-header mutants require rebuilding every
affected consumer before testing.

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

Use a new or empty output directory for every run. Before building, the runner
records the snapshot SHA-256 and verifies every selected head resolves to its
exact commit. Configure, build, and test output goes directly to log files;
the JSON report keeps only a bounded tail. Run the controller under the resource
limits described above, including a file-size limit for those logs.

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
