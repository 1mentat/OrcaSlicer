# OrcaSlicer tests

Building, running and writing tests is documented on the wiki, under [How to Test](https://www.orcaslicer.com/wiki/developer_reference/how_to_test.html).

Two files here rather than there, because coding agents only read what is in the repository:

- [AGENTS.md](AGENTS.md) is the same guidance in short form, and is what an agent working under `tests/` picks up.
- [CATCH2.md](CATCH2.md) is the Catch2 reference, including the mistakes that break a test at runtime.

This file describes how the test tree is organised, how its CMake and CTest wiring behaves, and how the tag, control and mutation data under `tests/` fit together.

## Layout

| Path | Contents |
| --- | --- |
| `libslic3r/`, `fff_print/`, `sla_print/`, `libnest2d/`, `slic3rutils/`, `filament_group/`, `cli/` | The test suites; see [AGENTS.md](AGENTS.md#the-suites) for what each covers. `slic3rutils` is built only in GUI builds, and `cli` only on Linux. |
| `catch2/` | Vendored Catch2, updated with `update_catch2.sh`. |
| `data/` | Test inputs, reached through the `TEST_DATA_DIR` compile definition. |
| `test_utils.hpp` | Helpers shared by every suite. |
| `controls/` | Exact lists of CTest case names (`*.json`, some with a `*.regex` selector) that a focused mutation run must pass on the unmodified tree. |
| `mutations/` | Mutation catalogs (`*.json`) consumed by `scripts/slicer_mutation.py`. |
| `settings_coverage.json` | Explicit evidence claims for settings; see [Settings coverage](#settings-coverage). |

Shipped vendor profiles are available to tests through the `PROFILES_DIR` define (`resources/profiles`), so tests can exercise real machine and filament G-code templates.

The Python test tooling has its own unit tests in `scripts/tests/`:

```sh
python3 -m unittest discover -s scripts/tests
```

## Building and running

Tests are off by default; the platform build scripts that enable them are listed in [AGENTS.md](AGENTS.md#building-and-running). The suites build as `libnest2d_tests`, `libslic3r_tests`, `fff_print_tests`, `sla_print_tests`, `filament_group_tests` and `slic3rutils_tests`:

```sh
cmake --build build --config Release --target fff_print_tests
ctest --test-dir build/tests -C Release --output-on-failure                  # everything
ctest --test-dir build/tests/fff_print -C Release --output-on-failure        # one suite
ctest --test-dir build/tests -C Release -L Cooling                           # by tag label
ctest --test-dir build/tests -C Release -LE "NotWorking|RequiresApp"         # what CI runs
```

Always pass `-C <config>` (see AGENTS.md for why). `scripts/run_unit_tests.sh [TEST_DIR] [BUILD_CONFIG]` is the entry point of the CI unit-test job; it needs only `build/tests`, `scripts/` and `tests/`.

The `cli` tests run the built `orca-slicer` binary. They carry the `RequiresApp` label, which the CI job excludes because it receives only `build/tests`. Run them with `ctest --test-dir build/tests -C Release -L RequiresApp`.

### CTest discovery

Each suite registers its Catch2 cases through `orcaslicer_discover_tests()` in `tests/CMakeLists.txt`, which calls `catch_discover_tests(... ADD_TAGS_AS_LABELS)`. A Python 3 helper parses each executable's JSON listing once when the suite links, so the interpreter must be on `PATH`. `python3 -m unittest discover -s scripts/tests -p test_catch_preprocessor.py` checks that helper against malformed listings without compiling C++.

Catch2 captures output through file descriptors (`CATCH_CONFIG_EXPERIMENTAL_REDIRECT`) so diagnostics from parallel slicer code stay synchronized under the JUnit reporter. Worker threads must finish before assertions run, and tests must not depend on diagnostic line ordering.

Cache variables that change registration or compilation:

- `SLICER_TEST_PROCESSORS` (default `0`). A positive value gives every test that many CPU slots and an affinity mask. With `1`, tests tagged `[Parallel]` still receive two CPUs because their assertions depend on concurrency; `ctest --parallel N` sets the total slot count. `0` leaves scheduling and affinity unrestricted. The CTest binary must support affinity: some distribution packages link an unpatched libuv and silently ignore `PROCESSOR_AFFINITY`, so check a running test's CPU mask on a new runner rather than trusting the property in `ctest --show-only=json-v1`. A suite that gains its first `[Parallel]` test must be added to `SLICER_TEST_TARGETS_WITH_PARALLEL`.
- `CATCH_EXTRA_ARGS`: semicolon-separated arguments added to every registered test, for example `-DCATCH_EXTRA_ARGS="--rng-seed;21907"` for reproducible `GENERATE` inputs. Re-run CMake to refresh discovery. The empty default keeps Catch2's automatic seed.
- `SLICER_TEST_GNU_LTO_JOBS` (default `0`): GNU LTO job count for Release test links; GNU compilers only.
- `SLICER_FAST_FOCUSED_TEST_COMPILE` (default `OFF`): compile the excluded focused executables with `-O1`; not available with the MSVC frontend.
- `SLICER_ENABLE_FOCUSED_PR_CTEST_DISCOVERY` (default `OFF`): see below.

### Focused test targets

Large suites also have focused executables declared `EXCLUDE_FROM_ALL`, for example `fff_print_cooling_tests`, `fff_print_wipe_tests` or `libslic3r_arachne_tests`. They are not part of a default build, and they are not registered with CTest unless `SLICER_ENABLE_FOCUSED_PR_CTEST_DISCOVERY` is on. In that case their cases are registered under a per-target prefix such as `FocusedSL1Raster::` or `Focused3MF::`, so names stay unique next to the full suite.

`scripts/focused_targets.txt` lists every such target; a line `name optional: <CMAKE_VAR>` marks one that exists only while that variable is set. Build and run one directly:

```sh
cmake --build build --config Release --target fff_print_cooling_tests
build/tests/fff_print/<config dir, if any>/fff_print_cooling_tests "[Cooling]"
```

`python3 scripts/focused_targets.py check` verifies that the inventory equals the set CMake declares. `python3 scripts/focused_targets.py build --build-dir build --config Release` links every listed target and smoke-runs it.

## Tags, labels and mutation catalogs

Every test carries a PascalCase subsystem tag, and often a narrower one. `ADD_TAGS_AS_LABELS` turns each Catch2 tag into a CTest label, so one name serves three selectors:

| Selector | Example |
| --- | --- |
| Catch2 filter on a built executable | `fff_print_tests "[Cooling]"` |
| CTest label regex | `ctest -L Cooling`, or `-L "^Cooling$"` to avoid matching longer labels |
| `test_filter` in a mutation catalog | `"^PolyholeSettings$"` |

Hidden tests (`[.]`) stay hidden in every mode, and `[NotWorking]` marks tests that CI runs exclude.

### Mutation catalogs

A catalog in `tests/mutations/` has `schema_version`, a `description` and a list of mutants. Each mutant gives the source `path`, the exact `old` text to replace with `new`, a `description`, and a `test_filter`:

```json
{
  "id": "polyhole-edge-limit-is-ignored",
  "path": "src/libslic3r/PrintObject.cpp",
  "old": "...exact original line...",
  "new": "int max_edges = 50;",
  "description": "Ignore configured edge limit; minimum and six-edge output checks must fail.",
  "test_filter": "^PolyholeSettings$"
}
```

`scripts/slicer_mutation.py run` applies each mutant to a disposable checkout, rebuilds, and runs the selected tests, which must fail on an ordinary assertion. How `test_filter` is used depends on the test command (`_test_command` in the script):

- If the test argv contains a `{filter}` placeholder, the filter is substituted there verbatim, so it is whatever that command expects (a Catch2 spec for a direct executable, a regex for `ctest -R`).
- Otherwise, when the command is CTest and has no label option, the script appends `-L <test_filter>`, so the filter is a CTest label regex.
- With `--target` (target mode), the script derives the build and CTest commands itself and uses CTest semantics. Target mode needs `--cmake`, `--ctest`, `--config`, `--jobs` and `--expected-control-inventory`, and an existing configured build. Explicit mode (`--configure-command`, `--build-command`, `--test-command`) rejects those options.

When the production line a mutant names changes, update its `old` text.

### Controls

In target mode, a file in `tests/controls/` (`--expected-control-inventory`) is the exact set of CTest case names that must pass on the unmutated tree before any mutant runs. The names are the Catch2 case names, carrying the focused discovery prefix when the target registers one; a `.regex` file beside it is the equivalent CTest name regex. Update an inventory only for an intentional case change, and keep every existing control.

A direct Catch2 binary exits with 42 (`TestFailureExitCode` in `catch2/src/catch2/catch_session.hpp`) when assertions fail, which differs from CTest's own status. Classify a mutant as killed only from an ordinary assertion failure; a crash, timeout or out-of-memory termination is an infrastructure failure.

## Settings coverage

`tests/settings_coverage.json` is the evidence map for `scripts/settings_inventory.py`. The script derives every registered option from `PrintConfigDef` in `src/libslic3r/PrintConfig.cpp`; an option without an entry is reported `uncovered`. An entry names a `test` file, a `tag` that must appear in that file, and the `dimensions` it exercises: `enum_values`, `boolean_values`, `boundary_values`, `representative_values`, `nullable_behavior`, `pipeline_oracle`, `interaction_matrix`, `fixture_only` (and `schema_roundtrip` for registry-wide serialization evidence).

The option's type determines which dimensions are required (for example `boundary_values` and `pipeline_oracle` for bounded numeric options; numeric scalars with no registered min/max need `representative_values` instead, while numeric vectors keep `boundary_values`). An entry that lacks one is reported `declared_partial` with the missing dimensions. The `reason` states what the named test asserts and what it does not claim. An entry may add `auxiliary_tests` (`test` and `tag` pairs) when several cases together provide the evidence. The report is static: it shows that a named test exists, not that it passed.

Rules for editing it:

- Name the narrowest tag that selects the exercising test, not a suite-wide tag such as `[SettingsVariance]` or `[Cooling]`.
- Claim a dimension only when an assertion in the tagged test fails if the option is ignored or mishandled at that value. If a claim depends on behavior the tree does not have, leave the dimension out so the entry reports partial.
- Add or change only the intended entries and preserve existing text and Unicode escaping. Check the semantic key delta as well as the textual diff; a whole-file rewrite touches unrelated entries.

## Validation runs

Coverage, mutation, settings-inventory, module-feedback and PR-validation runs are produced by scripts under `scripts/`: `slicer_coverage.py`, `slicer_coverage_pair_collector.py`, `slicer_catch_batch.py`, `slicer_coverage_reconcile.py`, `slicer_mutation.py`, `slicer_module_feedback.py`, `settings_inventory.py` and `slicer_pr_validation.py`.

Apart from `--help`/`-h` and `slicer_mutation.py list-defaults`, each one refuses to run directly. It must be started by `scripts/validation_runner.py` inside a leased artifact payload and writes only inside that payload; its output is disposable unless a file is explicitly promoted. The runner invocation, the checks it enforces (target-mode mutation runs cannot pass them) and worked examples for the inventory and explicit-mode mutation producers are in [docs/HLSD/validation-artifacts.md](../docs/HLSD/validation-artifacts.md#test-validation-producers). Run any producer with `--help` for its options. The Catch-batch coverage option and the PR-validation evidence gates are covered there too.

Requirements that apply to these runs:

- Coverage needs a build configured with `-DSLICER_ENABLE_LLVM_SOURCE_COVERAGE=ON`, or `-DSLICER_ENABLE_GCC_GCOV_COVERAGE=ON` for the gcov backend. `-DSLICER_COVERAGE_PROJECT_FUNCTIONS_ONLY=ON` restricts instrumentation to functions defined under `src/`.
- `[LocalesUtils]` needs a `de_DE.UTF-8` locale. If it is generated in a private directory, set `LOCPATH` in the environment the tests run in, and require zero skips: a skipped test leaves a run incomplete even when CTest exits successfully.
- Some tests use Boost memory-mapped files, so `TMPDIR` must be on a filesystem that supports read-only `mmap`; a filesystem can accept writes and still fail to map the file.
- Run permission tests as a non-root user.
- Keep build trees, temporary files and compiler caches on disk-backed storage with enough free space. Compiler job counts and test timeouts do not limit memory, so run expensive builds and test controllers under an operating-system resource limit suited to the machine.
