# Test suite rules

Rules for writing tests under `tests/`. [CATCH2.md](CATCH2.md) is the Catch2 reference. Building and running the suites is covered on the wiki, at <https://www.orcaslicer.com/wiki/developer_reference/how_to_test.html>.

For LLVM coverage, settings inventory, and bounded mutation validation, follow the [Confidence runs](README.md#confidence-runs) commands and reports; keep their production source manifests, evidence map, and exact test filters aligned with the suite tags.

## The suites

- `libslic3r`: the core library. Geometry, meshes, file formats, config and presets, Clipper, algorithms, data structures.
- `fff_print`: the FFF slicing pipeline, from a `Model` plus config through `Print` and `PrintObject` to emitted G-code.
- `sla_print`: SLA support-tree and pad geometry, support-point generation, raycast.
- `libnest2d`: 2D nesting and packing.
- `slic3rutils`: the Python plugin system and its slicing-pipeline bindings.
- `filament_group`: filament-to-extruder grouping, checked against golden files.
- `cli`: end-to-end runs of the built `orca-slicer` binary, Linux only. These tests carry the `RequiresApp` label, which the CI unit-test job excludes because it receives only `build/tests`; run them with `ctest --test-dir build/tests -C Release -L RequiresApp`.

## Building and running

Tests are off by default, so the build has to be told to include them.

- Windows: `build_win.bat -ds --run-tests`, which builds the dependencies and the tests and runs them (`-l -x` for the clang-cl and Ninja build CI uses)
- macOS: `./build_release_macos.sh -s -a arm64 -T`, which builds and runs them
- Linux: `./build_linux.sh -t`, then `ctest --test-dir build/tests -C Release`

Rebuild a single suite with `cmake --build build --config Release --target <suite>_tests`. Visual Studio, Xcode and the Ninja Multi-Config generator that `build_linux.sh` uses are all multi-configuration, so `ctest` needs `-C` on every platform; without it, tests registered with plain `add_test()` lose their labels and report "Not Run".

Before a full Linux confidence run, follow the worker prerequisites in [README.md](README.md). Run `[LocalesUtils]` in the actual service environment and require zero skips. When using `SLICER_TEST_PROCESSORS`, verify a running test's CPU mask with the selected CTest executable; the JSON properties alone do not prove affinity works. Reuse the verified locale environment and affinity-capable CTest runtime.

Worker launchers should reuse `scripts/slicer_process.py`'s `run_captured()` with
an explicit environment copied from the parent, then apply private temp/profile
paths. Preserve inherited executable, loader, and locale settings; do not clear
the launcher's global environment. Use `run_recorded()` when interpreting a
command's output afterward, so its raw outcome survives an assessment error.
Validate imported helper exports and reused receipt schemas before starting
expensive commands. Preserve each successful command's output identities with its
receipt so a later assessment can reuse those outputs.
Require the native control's exit status and exact JUnit inventory to pass before
compiling or running a mutant. Preserve raw assertion text when classifying a
mutation; reporter punctuation alone must not turn an ordinary assertion failure
into a survivor.
Bind discovery to the executable's identity after rebuilding; an inventory copied
from an earlier build can have correct names but identify the wrong executable.
Direct bundled Catch2 binaries return `TestFailureExitCode` (42) for failed
assertions; see `catch2/src/catch2/catch_session.hpp`. Do not assume exit 1 or
confuse the direct binary status with CTest's status when classifying a kill.
Keep successful `SystemExit` outside failure-logging handlers. When compiling a
source alias, retain the original quoted-header search directory and verify the
staged headers before starting the compiler.

## Where a test goes

- Pick the suite by the production code the test exercises, not by how the test is written.
- A property of a class that holds with no `Print` involved belongs in `libslic3r`. Behavior that depends on print settings, or produces or consumes G-code or slicing state, belongs in `fff_print`.
- One file per subsystem, named `test_<subsystem>.cpp`. It owns every test for that subsystem, whether the test reads in-memory state or generated output.
- When you add a file, list it in that suite's `CMakeLists.txt` in the same change.

## Use the existing helpers

Check these before writing your own setup or output-parsing code.

- `tests/test_utils.hpp` is shared by every suite. `load_model()` loads a mesh from `tests/data/`, and `ScopedTemporaryFile` gives a temp path that removes itself.
- `fff_print/test_helpers.hpp` builds and slices a `Print` and parses the emitted G-code. Read it before writing an fff_print test rather than assembling a `Print` by hand.
- The other suites have their own: `sla_print/sla_test_utils.hpp`, `libnest2d/libnest2d_test_utils.hpp`, `slic3rutils/plugin_test_utils.hpp`, `filament_group/fg_test_utils.hpp`. `libslic3r` has none and uses the shared header.
- Test data lives in `tests/data/` and is reached through the `TEST_DATA_DIR` define. Wrap it in `std::string(...)` before joining a path onto it.

## Writing the test

- Name the test case as a plain behavioral sentence in the present tense. No `Subsystem:` prefix.
- Tag it with the subsystem it covers, matching the file, in PascalCase. That tag is what people filter on, so every test needs one.
- Add further tags where they help: a narrower one to slice a large file (`[Rotcalip]`, `[Placer]`), a shared one for something spanning files (`[Python]`, `[H2C]`, `[Regression]`), or `[NotWorking]` / `[.]` to disable or hide a test. Say why in a comment if you disable or hide.
- Prefer a flat `TEST_CASE` per behavior, with `GENERATE` for parameterized cases. Reserve `SCENARIO` / `GIVEN` / `WHEN` / `THEN` for genuine shared setup that branches into a few close variations.
- Set the config keys your test depends on, and derive the expected values from what you set. A 20mm cube sliced at `layer_height` 2 is 10 layers, and the test should state both parts. If a number in your assertion comes from a key you never set, the test is also testing that default.
- For numeric or Boolean `DynamicPrintConfig` values, use the initializer-list overload, such as `config.set_deserialize_strict({{"layer_height", 0.2}})`. The two-argument overload takes serialized text.
- Use at least two real motion blocks per flush interval in GCodeProcessor fixtures that compare motion estimates with configured feed floors or filament delays. Single-block zero-delay and positive-delay histories can take different buffering paths; test that short-stream behavior separately when it is the contract.
- Use registered canonical setting keys in behavior fixtures. `set_deserialize_strict()` may discard obsolete or unknown names; use legacy names only when migration is the behavior being tested.
- Confirm which static config class owns a setting before testing overrides. A key registered as nullable may belong to `GCodeConfig` or `PrintConfig` and have no object-level override. Trace nil resolution through the actual preset or variant merge, then pass that resolved config to the pipeline; nil does not always mean inheritance or restoration.
- Assert the defining property, not an incidental value. "Skirt present" or "at least 2 brim loops" survives a refactor; exact coordinates and byte counts do not.
- Compare total path length when checking ironing spacing. A connected serpentine can remain one extrusion path at different spacings.
- Name a regression test for the behavior it protects, never for an issue or PR number.
- When asserting on G-code, match the meaningful token such as `; skirt` rather than whole lines, whitespace or comment wording. Depend on ordering only when ordering is the contract.

## Catch2 rules that cause real breakage

- Never reuse a `SECTION` name inside a loop. Use `DYNAMIC_SECTION` so each iteration is unique.
- Never assert from a spawned thread. Catch2 assertions are not thread-safe. Collect results in the thread and assert on the main thread.
- Never combine conditions with `&&` or `||` inside one assertion. Split them so Catch2 can print both operands on failure.
- Compare floats with `WithinAbs` or `WithinRel`, never `==`. Prefer these over `Approx` in new tests.
- Keep tests self-contained: no shared state, green under `--order rand`.
- Stage editable private candidates and inventory fixtures with independent file copies. Never hardlink them to tracked source files; a write through either path changes the checkout.
- Count Catch2 JUnit `<testcase>` nodes and check their exact names. The bundled JUnit reporter's `tests` attribute counts assertions.
- Derive runtime inventories from the built executable and CTest, not a source regex: declarations inside `#if 0` are not registered. Focused targets that link `test_helpers.cpp` also include `Scenario: init_print functionality`.
- BDD scenarios and `DYNAMIC_SECTION` can emit additional JUnit entries for section paths. Declare the expected root and section names explicitly when validating a focused run.
- Catch normalizes tag order. Compare tag tokens rather than their source ordering when checking inventories.
- Commas in case names are Catch filter separators. Do not concatenate unescaped names into a filter; use tags or an unfiltered run with an exact inventory check.
