# Settings inventory

`settings_inventory.py` builds a static inventory of every OrcaSlicer config key. For each key it records:

- the definition: type, default, enum values, min and max
- the config classes the key belongs to
- the code that reads the key, and the pipeline step each read runs in
- the steps a change of the key resets
- the tests that set the key, and the values they set

It then flags inconsistencies between these facts. It reads the source text only; nothing is compiled or run. It uses the Python 3 standard library and nothing else.

Outputs, written next to the script:

- `settings_inventory.md`: counts, then one table per flag, most material first. This is checked in, so a change that adds a setting or moves an invalidation shows up in its diff.
- `settings_inventory.json`: one record per key. It is about 3 MB, so it is generated on demand and not checked in.

## Running

From the repository root:

```bash
python3 tests/settings_inventory/settings_inventory.py --root . --out-dir tests/settings_inventory/
```

A run takes about 10 seconds. Re-run it after changing `PrintConfig.cpp`/`.hpp`, either `invalidate_state_by_config_options`, the code that reads a setting, the tests or the profiles, and commit the regenerated `settings_inventory.md`.

The script stops with an error if `Print::invalidate_step` or `PrintObject::invalidate_step` no longer has the shape it models. When that happens, update `parse_cascade`.

## How it decides

The script docstring states every rule exactly. In outline:

- **Invalidation.** It simulates both if / else-if chains with first-match semantics. `Print::apply` sends PrintConfig, GCodeConfig and MachineEnvelopeConfig keys to the Print function, and object and region keys to the PrintObject function. Keys outside these classes reach only the full-config diff, which resets `psGCodeExport`. Each reset is closed over the `invalidate_step` cascade, which is parsed from the code. A reset inside a nested `if` is recorded as *conditional*.
- **Consumers.** A consumer is a read of the key in `src/`. Each read gets a confidence:
  - **high**: a config-named receiver (`config.key`, `m_config.key`, `config().key`, `XConfig::key`), a `*_CONFIG(key)` macro, or an accessor such as `opt_*("key")`, `option("key")` or `get_abs_value("key")`.
  - **medium**: any other `.key` / `->key` whose name is unambiguous.
  - **low**: the name is also a variable somewhere, the read is `this->key`, the read only looks at size (`.size()`, `.empty()`), or the key appears as a bare string literal.

  Writes are not counted.
- **Step of a read.** The script maps the enclosing function to a step in this order:
  1. the `FUNCTION_STEP` table
  2. the `set_started(step)` span in `Print::process` / `PrintObject::*`
  3. same-file call-graph propagation, used only when the callee is unique and every call site is attributed
  4. the `FILE_STEP` glob table

  A step of `shared`, `gui` or `cli` is not a pipeline step.

## Flags

| flag | meaning |
|---|---|
| STALE_RISK | The key is read (high or medium confidence) in a cached step that its invalidation branch does not reset, after the cascade. If you change the key in the GUI, that step can keep a result built from the old value. `missing` means the step is never reset; `conditional` means it is reset only under a nested condition. Rows are ranked by materiality (object step before print step, then read confidence, then how precisely the step was mapped). |
| UNKNOWN_IN_INVALIDATION | A name in an invalidation list is not a defined key (a typo or a removed key). That listing does nothing. |
| WRONG_LIST | An object or region key is listed in the Print function, or a print key is listed in the PrintObject function. Because `Print::apply` never routes the key there, the listing is dead. |
| SHADOWED | A key is listed in more than one branch. Only the first listing takes effect; the table shows the steps the later listings would have reset and that are lost. |
| DEFAULT_INVALIDATION | The key is handled only by the catch-all `else`. This flag is informational. |
| UNTESTED | No file under `tests/` sets the key. This covers C++ setters and pairs, `.ini` files and CLI arguments. |
| UNTESTED_VALUE | No test selects this value of an enum or bool key. The default is reported separately as `implicit`, since every test that does not set the key uses it. |
| UNREAD | A defined key has no consumer anywhere in `src/`. |
| GUI_ONLY | An FFF key is read only under `src/slic3r` (the GUI). |
| PROFILE_UNKNOWN | A key used in `resources/profiles/**/*.json` is not defined, is not a preset metadata key, and is not renamed or ignored by `handle_legacy`. The table gives the count of files and the vendors. The best-known case is `inital_layer_height`, in 103 profiles (Prusa 64, Ratrig 39). |
| MEMBER_WITHOUT_DEF | A static config class member has no `PrintConfigDef` definition. |

## Hand verification

The `HAND_VERIFIED` table in the script holds verdicts that were reached by reading the code at the revision named in the md header. Each verdict appears in the STALE_RISK "verdict" column. Some rows were checked and found to be false positives. Those that led to a rule change are in `HAND_REJECTED`, which the md lists below the STALE_RISK table.

When the code changes, re-check the affected verdicts and edit or remove their entries. A verdict whose key is no longer flagged is dropped from the md without warning, so check the list by hand after each refresh.

## Known limits

- Tests that build values at run time, and 3MF test data, are not scanned.
- A read through a copied struct or the generic config visitor is not attributed to its key.
- Helper files mapped to `shared` (for example `Flow.cpp`, `PrintRegion.cpp` and the helpers in `Print.cpp`/`PrintObject.cpp`) never raise STALE_RISK.
- `SlicingParams` is a pseudo-step for `m_slicing_params`, which is cleared by `posSlice` and `posSupportMaterial`. It approximates when slicing parameters are recomputed.
