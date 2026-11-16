# Pair-aware coverage collector

This offline collector binds each test executable to its own `.profraw` files using SHA-256 values in `native-pairs.json`. It runs LLVM merge/export per pair, filters each LCOV tracefile to the selected source root, and lets `lcov --add-tracefile` produce the line union. Per-object LLVM JSON and LCOV exports remain available under the output directory. It requires matching LLVM tools and LCOV 2.0.

Run from the OrcaSlicer repository root with a new output directory:

```sh
python3 scripts/slicer_coverage_pair_collector.py \
  --root "$PWD" --source-root src --build-dir build \
  --compile-commands /path/to/compile_commands.json \
  --manifest /path/to/libslic3r.sources.txt \
  --manifest /path/to/libslic3r_cgal.sources.txt \
  --pairs /path/to/pairs.json --out /path/to/new-output
```

The pair file uses `{"schema": 1, "pairs": [{"object": "...", "object_sha256": "...", "profiles": [{"path": "...profraw", "sha256": "..."}]}]}`. One object may appear once, each profile path must appear once, and every input hash is checked again after collection.

The collector does not run tests. Its report sets `test_status` to `not_checked`, `coverage_claim` and `threshold_claim` to false, and fails on any LLVM profile-mapping warning or pair with no executed LLVM counters. The reported line percentage is recomputed from LCOV's covered and total physical-line counts. LLVM's per-object JSON summaries are not added together because nested function lines can be counted more than once; no combined branch or function percentage is claimed.

The union audit checks that LCOV retained exactly the source and line identities
from the input tracefiles, including zero-hit lines. Missing or unexpected
identities fail integrity. Compiler-object audits also check the production
source manifests and executable header mappings; metadata-only headers remain
visible in the report without contributing executable lines.

The source-line bounds audit hashes each mapped source file and rejects DA line
numbers outside its physical lines, including zero-hit rows. Missing sources
and paths resolving outside the source root also fail integrity. Raw exports
and the union remain available when this check fails. This catches stale
mappings that extend past the current file's end;
matching bounds alone do not establish that an executable was built from the
current source bytes.
