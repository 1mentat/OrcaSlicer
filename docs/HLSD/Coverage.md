# Core source coverage

Coverage measures the executable production source exercised by the canonical
headless C++ test suites. It provides a comparable history and file-level gaps for
contributors. A percentage describes execution, not correctness; mutation results
are a separate kind of evidence.

## Measurement boundary

Ordinary builds do not enable instrumentation. The opt-in
`SLICER_ENABLE_LLVM_SOURCE_COVERAGE` CMake option applies LLVM source coverage to
selected production libraries and their test executables. Dependencies retain
their normal build settings. Atomic profile counters preserve increments from
concurrent slicing work.

The Linux tracking configuration uses Release, Clang/LLVM 18, CAD enabled and GUI
disabled. It builds and runs `libslic3r_tests`, `fff_print_tests`, `sla_print_tests`,
`libnest2d_tests` and `filament_group_tests`. CTest excludes `NotWorking` and
`RequiresApp` tests. GUI code, other operating systems, external dependencies and
uncompiled conditional sources are outside this measurement.

The denominator comes from the configured targets' source manifests and compiler
mappings under `src/libslic3r`. Static production objects remain in the link even
when no test calls them, so untouched executable code remains uncovered. Inline
project code contributes when LLVM emits an executable mapping. A file with no
executable mapping is accounted for separately rather than treated as fully
covered. The collector merges by physical source path across executables.

Test discovery uses isolated profiles. Only the actual test run contributes to
the measured profile pool. Missing profiles, failed tests, incompatible tools,
missing executable mappings and invalid compiler evidence prevent a valid
baseline. A valid low percentage succeeds in `--measure-only` mode; the tracking
jobs set no coverage threshold.

## Portable collection and presentation

`scripts/build_coverage.sh` is the shared Linux entry point. It builds the core
dependency prefix and instrumented test targets, then invokes
`scripts/slicer_coverage.py`. The entire process tree has two-CPU affinity so that
dependency recipes that count CPUs themselves respect the same bound. A cached
dependency prefix must be keyed by dependency inputs and the selected toolchain.

The collector preserves source manifests, compiler and binary evidence, tool
versions, test output, raw coverage export and profiles. These artifacts identify
the inputs and observed execution; recording a Git SHA alone does not prove that
an arbitrary pre-existing binary was built from that source. CI therefore builds
the binaries in the identified checkout before collection.

`scripts/coverage_report.py` turns the measurement into a standalone HTML report,
JSON, Markdown summary and SVG badge. It verifies matching commit provenance and
unchanged tracked sources before accepting a baseline. Links identify the exact
source commit. Annotated source uses the same binaries and merged profile as the
summary. The HTML works offline and loads no external assets.

The report leads with line coverage over time, then a searchable and sortable file
table. Coverage changes use percentage points against an older compatible
baseline. Configuration, measurement semantics and tool versions identify a
series; incompatible measurements start a new series. Zero executable counts
mean not measured. Incomplete runs appear as incomplete and do not enter history.

## Forge adapters

The GitHub workflow is independent of PR and release workflows. Manual dispatch
is available; `COVERAGE_TRACKING=true` enables its daily schedule. It skips an
already recorded source commit unless a manual run requests a refresh. Collection
uses read access. A separate job records validated JSON metadata on the
`coverage-data` branch using a non-force update. It never executes files from a
measurement artifact or restores a compiler cache. The current series keeps up to
365 observations; older metadata remains in Git history. Larger HTML and diagnostic
artifacts have short retention periods.

`COVERAGE_PAGES=true` enables the optional Pages adapter after the repository's
Pages source is configured for GitHub Actions. Publishing permissions belong to
separate jobs. A failed or incomplete measurement cannot replace the published
baseline. Coverage is never registered as a required merge check.

A Forgejo installation uses the same collection and rendering commands from a
separate CI repository. Its source repository can remain a read-only pull mirror
with Actions disabled. The controller resolves and checks out an exact mirrored
source commit, uses repository-scoped credentials only for checkout, and supplies
its own artifact and static-site publishing adapters. Native and instrumented
builds share a bounded single-job queue; a shared build lock also protects against
concurrent operator builds. Runner isolation, storage limits, cache retention and
private ingress are deployment configuration and stay outside the portable source
implementation.
