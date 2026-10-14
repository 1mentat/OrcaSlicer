#!/usr/bin/env python3
"""Generate the OrcaSlicer settings inventory: one record per config key, with mechanical flags.

Run from anywhere:
    python3 tests/settings_inventory/settings_inventory.py --root . --out-dir tests/settings_inventory/

Outputs settings_inventory.json (machine readable, one record per key) and settings_inventory.md
(human summary). Python 3 standard library only; nothing is compiled or run.

Sources read
------------
* ``src/libslic3r/PrintConfig.cpp``: every ``this->add("key", coType)`` in the PrintConfigDef
  ``init_*`` functions, with default, enum values (from the ``s_keys_map_*`` maps), min and max.
  Keys defined in loops (literal-list loops, ``add_belt_remap`` calls, the machine-limit axes,
  ``filament_extruder_override_keys``) are expanded; any other non-literal ``add`` is reported
  as a parse warning.
* ``src/libslic3r/PrintConfig.hpp``: members of the static config classes.
* ``src/libslic3r/Print.hpp``: the ``PrintStep`` / ``PrintObjectStep`` enums.
* ``PrintObject::invalidate_step`` and ``Print::invalidate_step``: the step cascade.
* ``Print::invalidate_state_by_config_options`` and
  ``PrintObject::invalidate_state_by_config_options``: the if / else-if chains, simulated with
  first-match semantics (a key is handled by the first branch that lists it; later listings are
  shadowed). Actions inside a nested ``if`` in a branch are recorded as *conditional* resets.
* ``src/**``: reads of each key (consumers), see "Consumers" below.
* ``tests/**``: places where a test sets a key, and the values it sets.
* ``resources/profiles/**/*.json``: keys used by shipped machine/process/filament profiles.

Which invalidation function sees a key
--------------------------------------
``Print::apply`` diffs PrintConfig keys (which include GCodeConfig and MachineEnvelopeConfig)
into ``Print::invalidate_state_by_config_options`` and PrintObjectConfig / PrintRegionConfig keys
into ``PrintObject::invalidate_state_by_config_options``. Keys in no FFF class only reach the
full-config diff, which invalidates ``psGCodeExport``.

Reset closure
-------------
The steps a change resets are computed from the branch actions with the cascade parsed from
the source: ``PrintObject::invalidate_step(posX)`` adds the steps listed for ``posX`` plus the
unconditional ``psWipeTower`` and ``psGCodeExport``; ``Print::invalidate_step(psX)`` adds
``psGCodeExport``; ``PrintObject::invalidate_all_steps`` resets all object and print steps;
``Print::invalidate_all_steps`` (the Print-level ``else``) resets print steps only. The cached
slicing parameters (pseudo-step ``SlicingParams``) are reset by the cascade branches that clear
``m_slicing_params.valid``.

Consumers
---------
A consumer is a read of the key in ``src/`` (excluding the PrintConfigDef functions, the static
class declarations, the two invalidation functions and ``Preset.cpp``'s option-name lists).
Confidence of each read:
* high: member access whose receiver names a config (``config.key``, ``m_config.key``,
  ``region.config().key``, ``print_cfg->key``), ``SomeConfig::key`` (pointer to member),
  ``EXTRUDER_CONFIG(key)``-style macros, or an accessor with a literal key: ``opt_*("key")``,
  ``opt<T>("key")``, ``option("key")``, ``option<T>("key")``, ``get_abs_value("key")``,
  ``has("key")``;
* medium: ``.key`` / ``->key`` on any other receiver, when no variable or field named ``key`` is
  declared anywhere in ``src/`` (so the name is unlikely to be anything but the config field);
* low: the same with a name that is also declared as a variable or field, ``this->key``, a read
  followed by ``.size()`` / ``.values.size()`` / ``.empty()`` (it counts filaments or extruders,
  a change Print::apply handles on its own path), or a bare string literal ``"key"`` (outside
  PrintConfig.cpp). Low reads are listed but never flag.
Writes (``set``, ``set_key_value``, ``set_deserialize*``, ``erase``) are not consumers.

Pipeline step of a consumer
---------------------------
Mapped per function, then per file:
1. ``FUNCTION_STEP`` (explicit table, qualified function name -> step);
2. a function that calls ``set_started(S)`` for one step S belongs to S; in a function that
   starts several steps (``Print::process``) the lines between ``set_started(S)`` and
   ``set_done(S)`` belong to S;
3. functions of the same file called from 1 or 2 inherit the caller's step (call graph,
   same file only);
4. ``FILE_STEP`` (explicit table, glob -> step) for the rest.
Steps other than ``pos*``, ``psWipeTower``, ``psSkirtBrim`` and ``SlicingParams`` are not cached
across ``Print::apply`` (``psGCodeExport`` is reset by every branch; ``validate``, ``apply``,
``gui`` ... run every time), so they cannot go stale. A step whose entry function never calls
``set_done`` is re-run on every process and is also treated as uncached.

Flags (rules)
-------------
STALE_RISK            A high/medium consumer runs in cached step S, and the reset closure of
                      the key's effective invalidation branch (or of the matching ``else``)
                      does not contain S. ``conditional`` when S is reset only by a nested
                      ``if`` in the branch. One row per (key, step); ranked by: object step
                      missed > print step; consumer confidence; how the step was mapped.
UNKNOWN_IN_INVALIDATION A name listed in an invalidation chain (or in ``steps_gcode``) that is
                      not a defined key.
WRONG_LIST            A defined key listed in the Print function that is not a PrintConfig key
                      (object/region keys never reach it), or listed in the PrintObject function
                      that is not an object/region key.
SHADOWED              A key listed in a branch after an earlier branch that also lists it (the
                      later listing never runs). Informational.
DEFAULT_INVALIDATION  FFF key handled by no branch, only by the catch-all ``else``.
                      Informational.
UNTESTED              FFF key never set by any test. ``UNTESTED_VALUE``: an enum or bool value
                      no test selects; the default value is reported as ``implicit``.
UNREAD                Defined key with no consumer of any confidence anywhere in ``src/``.
GUI_ONLY              Defined key whose consumers are all in ``src/slic3r`` (the GUI).
PROFILE_UNKNOWN       Key used by a shipped profile (machine/process/filament JSON) that is not
                      defined, not handled by ``PrintConfigDef::handle_legacy`` (renamed or
                      ignored), and not a preset metadata key (``*_JSON_KEY_*`` in Preset.hpp).
MEMBER_WITHOUT_DEF    Member of a static FFF config class with no definition in PrintConfigDef.

Hand verification
-----------------
``HAND_VERIFIED`` records verdicts from reading the code for selected STALE_RISK rows. They are
keyed by (key, step) and shown in the md; re-check them when the cited code changes.
"""

import argparse
import bisect
import collections
import fnmatch
import glob
import json
import os
import re
import subprocess

# --------------------------------------------------------------------------------------------
# Tables
# --------------------------------------------------------------------------------------------

FFF_CLASSES = ('PrintObjectConfig', 'PrintRegionConfig', 'PrintConfig', 'GCodeConfig', 'MachineEnvelopeConfig')
PRINT_LEVEL_CLASSES = ('PrintConfig', 'GCodeConfig', 'MachineEnvelopeConfig')
OBJECT_LEVEL_CLASSES = ('PrintObjectConfig', 'PrintRegionConfig')

# Steps that keep their result across Print::apply until invalidated.
PSEUDO_CACHED_STEPS = ('SlicingParams',)
CACHED_PRINT_STEPS = ('psWipeTower', 'psSkirtBrim')

# Explicit function -> step table. Qualified names as written in the definition. The steps here
# are pipeline entry points (or pseudo-steps) that cannot be derived from set_started().
FUNCTION_STEP = {
    # Print-level
    'Print::validate': 'validate',
    'Print::apply': 'apply',
    'Print::export_gcode': 'psGCodeExport',
    'Print::_make_wipe_tower': 'psWipeTower',
    'Print::_make_skirt': 'psSkirtBrim',
    'Print::process': 'process',   # lines outside the set_started spans: orchestration
    # Object steps whose work is done outside PrintObject.cpp
    'Layer::make_perimeters': 'posPerimeters',
    'Layer::is_perimeter_compatible': 'posPerimeters',
    'LayerRegion::make_perimeters': 'posPerimeters',
    'LayerRegion::process_external_surfaces': 'posPrepareInfill',
    'LayerRegion::prepare_fill_surfaces': 'posPrepareInfill',
    'LayerRegion::slices_to_fill_surfaces_clipped': 'posPrepareInfill',
    'LayerRegion::elephant_foot_compensation_step': 'posSlice',
    'Layer::make_fills': 'posInfill',
    'Layer::make_ironing': 'posIroning',
    'Layer::get_sparse_infill_max_void_area': 'posInfill',
    # Cached slicing parameters (reset by posSlice / posSupportMaterial invalidation)
    'SlicingParameters::create_from_config': 'SlicingParams',
    'PrintObject::slicing_parameters': 'SlicingParams',
    'PrintObject::update_slicing_parameters': 'SlicingParams',
    'min_layer_height_from_nozzle': 'SlicingParams',
    'max_layer_height_from_nozzle': 'SlicingParams',
    # Object steps entered from Print::process without set_started in the callee
    'PrintObject::generate_belt_brim': 'posSupportMaterial',
    'PrintObject::prepare_adaptive_infill_data': 'posInfill',
    'PrintObject::prepare_lightning_infill_data': 'posInfill',
    'PrintObject::prepare_tpms_radial_fields': 'posInfill',
}

# Explicit file -> step table (first match wins), for files whose functions are not mapped above.
# 'shared' means helper code used by several steps (never flagged).
FILE_STEP = [
    ('src/libslic3r/PrintObjectSlice.cpp', 'posSlice'),
    ('src/libslic3r/TriangleMeshSlicer.cpp', 'posSlice'),
    ('src/libslic3r/MultiMaterialSegmentation.cpp', 'posSlice'),
    ('src/libslic3r/ElephantFootCompensation.cpp', 'posSlice'),
    ('src/libslic3r/Slicing.cpp', 'posSlice'),
    ('src/libslic3r/SlicingAdaptive.cpp', 'posSlice'),
    ('src/libslic3r/BeltSliceStrategy.cpp', 'posSlice'),
    ('src/libslic3r/Feature/Interlocking/*', 'posSlice'),
    ('src/libslic3r/TextureDisplacement.cpp', 'posSlice'),
    ('src/libslic3r/PerimeterGenerator.cpp', 'posPerimeters'),
    ('src/libslic3r/Arachne/*', 'posPerimeters'),
    ('src/libslic3r/Feature/FuzzySkin/*', 'posPerimeters'),
    ('src/libslic3r/Fill/*', 'posInfill'),
    ('src/libslic3r/ContourZ.cpp', 'posContouring'),
    ('src/libslic3r/Support/*', 'posSupportMaterial'),
    ('src/libslic3r/BeltBrim.cpp', 'posSupportMaterial'),
    ('src/libslic3r/GCode/WipeTower*', 'psWipeTower'),
    ('src/libslic3r/GCode/ToolOrdering.cpp', 'psWipeTower'),
    ('src/libslic3r/GCode/ToolOrderUtils.cpp', 'psWipeTower'),
    ('src/libslic3r/FilamentGroup*.cpp', 'psWipeTower'),
    ('src/libslic3r/BeltPurge.cpp', 'psWipeTower'),
    ('src/libslic3r/Brim.cpp', 'psSkirtBrim'),
    ('src/libslic3r/GCode/ConflictChecker.cpp', 'conflict_check'),
    ('src/libslic3r/GCode/*', 'psGCodeExport'),
    ('src/libslic3r/GCode.cpp', 'psGCodeExport'),
    ('src/libslic3r/GCodeWriter.cpp', 'psGCodeExport'),
    ('src/libslic3r/BeltGCode.cpp', 'psGCodeExport'),
    ('src/libslic3r/PlaceholderParser.cpp', 'psGCodeExport'),
    ('src/libslic3r/PrintApply.cpp', 'apply'),
    ('src/libslic3r/Format/*', 'io'),
    ('src/libslic3r/Preset*.cpp', 'presets'),
    ('src/libslic3r/AppConfig.cpp', 'presets'),
    ('src/libslic3r/calib.cpp', 'calibration'),
    ('src/libslic3r/Model*.cpp', 'model'),
    ('src/libslic3r/Arrange*.cpp', 'model'),
    ('src/libslic3r/SLA*', 'sla'),
    ('src/libslic3r/SLA/*', 'sla'),
    ('src/libslic3r/*', 'shared'),
    ('src/slic3r/*', 'gui'),
    ('src/*', 'cli'),
]

# Files and functions whose literals are lists of option names, not reads.
REGISTRY_FILES = ('src/libslic3r/PrintConfig.cpp', 'src/libslic3r/PrintConfig.hpp', 'src/libslic3r/Preset.cpp')
EXCLUDED_FUNCTIONS = ('Print::invalidate_state_by_config_options', 'PrintObject::invalidate_state_by_config_options')

SOURCE_EXTENSIONS = ('.cpp', '.hpp', '.h', '.cc', '.mm')

# Hand-verification verdicts for STALE_RISK rows, keyed by (key, step), from reading the code at
# revision cd021162. Re-check a verdict when the cited code changes.
HAND_VERIFIED = {
    ('outer_wall_speed', 'posPerimeters'): (
        'confirmed (multi-region)', 'Layer.cpp:168 merges regions with equal wall speeds into one perimeter run '
        '(Layer::make_perimeters, :244); the brim branch PrintObject.cpp:1464 resets posSupportMaterial only. '
        'A modifier whose speed changes keeps the merged walls. Same for inner_wall_speed, small_perimeter_speed, '
        'small_support_perimeter_speed.'),
    ('seam_slope_type', 'posPerimeters'): (
        'confirmed (multi-region)', 'Layer.cpp:189-199 compares the scarf/seam-slope keys for the region merge; '
        'their branch PrintObject.cpp:1832 resets psGCodeExport only. Same for the 10 other scarf keys.'),
    ('sparse_infill_speed', 'posInfill'): (
        'confirmed (multi-region)', 'Fill.cpp:1070 puts the speed into SurfaceFillParams::role_speed, which '
        'orders and groups fills across regions (Fill.cpp:352); the branch resets posSupportMaterial only. '
        'Same for top_surface_speed, internal_solid_infill_speed, and bridge_speed / internal_bridge_speed '
        '(psGCodeExport only).'),
    ('bottom_shell_thickness', 'posSlice'): (
        'confirmed (spiral vase)', 'PrintObjectSlice.cpp:240-241 slices layers below the bottom shell thickness '
        'in regular mode; PrintObject.cpp:1668 resets posPrepareInfill only, while bottom_shell_layers resets '
        'posSlice. LayerRegion.cpp:121 (posPerimeters) has the same gap.'),
    ('gap_infill_speed', 'posSlice'): (
        'confirmed (MM painting)', 'The posSlice condition at PrintObject.cpp:1514 requires opt_key to equal two '
        'names and is never true; MultiMaterialSegmentation.cpp:1419 derives the small-region threshold from '
        'gap_infill_speed > 0.'),
    ('outer_wall_line_width', 'posSlice'): (
        'confirmed (MM painting)', 'MultiMaterialSegmentation.cpp:1232 uses the outer wall width for painted '
        'top/bottom segmentation; PrintObject.cpp:1762 resets posPerimeters and posSupportMaterial only. Same '
        'for outer_wall_filament_id.'),
    ('detect_overhang_wall', 'posSupportMaterial'): (
        'confirmed', 'SupportMaterial.cpp:3405 adds bridging perimeters to the support trimming; the first branch '
        '(PrintObject.cpp:1492) resets posPerimeters, whose cascade excludes posSupportMaterial; the listing that '
        'adds posSupportMaterial (:1753) is shadowed.'),
    ('skirt_height', 'posSupportMaterial'): (
        'confirmed (tree support + skirt)', 'TreeSupport.cpp:2157 builds support lslices only for the skirt '
        'layers; Print.cpp:369 resets psSkirtBrim only, so a taller skirt ignores the tree support above the '
        'old height.'),
    ('retraction_length', 'psWipeTower'): (
        'confirmed (prime tower)', 'WipeTower.cpp:2189 stores the retraction that the tower toolchange writes '
        '(:4340); steps_gcode (Print.cpp:344) resets psGCodeExport only. Same for retraction_speed and '
        'wipe_distance.'),
    ('first_layer_flow_ratio', 'psWipeTower'): (
        'confirmed (prime tower)', 'Print.cpp:4870 sets the tower first-layer flow in _make_wipe_tower; the '
        'region branch PrintObject.cpp:1832 resets psGCodeExport only.'),
    ('brim_width', 'posPerimeters'): (
        'confirmed (narrow)', 'PerimeterGenerator.cpp:1846 prints the outer wall first on layer 0 for an '
        'outer-only brim with brim_width > 0; only a brim_type change adds posPerimeters (PrintObject.cpp:1474), '
        'so brim_width crossing 0 keeps the old wall order.'),
    ('bridge_angle', 'posPerimeters'): (
        'confirmed (narrow)', 'PerimeterGenerator.cpp:2156 uses the bridge angle for counterbore-hole bridging '
        'in the perimeter step; PrintObject.cpp:1668 resets posPrepareInfill only. Needs '
        'counterbore_hole_bridging enabled and a fixed bridge angle.'),
}

# Hand-check results that changed a rule: rows the first version reported and the current rules drop.
HAND_REJECTED = [
    ('filament_colour, filament_diameter', 'posSlice',
     'false positive: MultiMaterialSegmentation.cpp:2265 and PrintObjectSlice.cpp:219 read only .size() (the '
     'filament count), whose change Print::apply handles separately (PrintApply.cpp:1713). Size-only reads '
     'are now low confidence.'),
    ('min_layer_height, max_layer_height', 'posSlice',
     'mis-mapped: Slicing.cpp:40-66 feed SlicingParameters, reset by the posSupportMaterial their branch '
     'invalidates. These helpers now map to SlicingParams.'),
]


# --------------------------------------------------------------------------------------------
# C++ lexing helpers
# --------------------------------------------------------------------------------------------

LEX_RE = re.compile(r'//[^\n]*|/\*.*?\*/|R"([^(\s"]{0,16})\((.*?)\)\1"|"(?:[^"\\\n]|\\.)*"|\'(?:[^\'\\\n]|\\.)*\'', re.S)


def _blank(s):
    """Replace every character except newlines with a space (keeps offsets and line numbers)."""
    return re.sub(r'[^\n]', ' ', s)


class Source:
    """A lexed C++ file: comments removed, string literals indexed, offsets preserved.

    ``code`` has comments, string contents, preprocessor lines and ``#if 0`` blocks blanked,
    for structure and member-access matching. ``strings`` lists (offset, value) for every
    string literal outside comments and disabled blocks.
    """

    def __init__(self, path, rel):
        self.path, self.rel = path, rel
        with open(path, encoding='utf-8', errors='replace') as f:
            self.text = f.read()
        parts, strings, last = [], [], 0
        for m in LEX_RE.finditer(self.text):
            tok = m.group(0)
            parts.append(self.text[last:m.start()])
            if tok.startswith('/'):
                parts.append(_blank(tok))
            elif tok.startswith("'"):
                parts.append("'" + _blank(tok[1:-1]) + "'")
            else:
                value = m.group(2) if tok.startswith('R') else tok[1:-1]
                strings.append((m.start(), value))
                parts.append(tok[0] + _blank(tok[1:-1]) + tok[-1] if not tok.startswith('R') else _blank(tok))
            last = m.end()
        parts.append(self.text[last:])
        code = ''.join(parts)
        code, disabled = self._strip_preprocessor(code)
        self.code = code
        self.strings = [(o, v) for (o, v) in strings if not any(a <= o < b for a, b in disabled)]
        self.line_starts = [0] + [m.end() for m in re.finditer(r'\n', self.text)]
        self.functions = self.function_starts = None

    @staticmethod
    def _strip_preprocessor(code):
        """Blank preprocessor lines (with continuations) and ``#if 0`` / ``#if (0)`` blocks."""
        lines = code.split('\n')
        out, disabled, depth, offset, start = [], [], 0, 0, None
        cont = False
        for line in lines:
            s = line.strip()
            is_pp = cont or s.startswith('#')
            if depth:
                if s.startswith('#if'):
                    depth += 1
                elif s.startswith('#endif') or (depth == 1 and (s.startswith('#else') or s.startswith('#elif'))):
                    depth -= 1
                    if depth == 0:
                        disabled.append((start, offset))
                out.append(_blank(line))
            elif re.match(r'#\s*if\s*\(?\s*0\s*\)?\s*$', s):
                depth, start = 1, offset
                out.append(_blank(line))
            else:
                out.append(_blank(line) if is_pp else line)
            cont = is_pp and line.rstrip().endswith('\\')
            offset += len(line) + 1
        return '\n'.join(out), disabled

    def line_of(self, offset):
        return bisect.bisect_right(self.line_starts, offset)

    def strings_in(self, start, end):
        """String literal values with offsets in [start, end)."""
        i = bisect.bisect_left(self.strings, (start, ''))
        out = []
        while i < len(self.strings) and self.strings[i][0] < end:
            out.append(self.strings[i][1])
            i += 1
        return out

    def get_functions(self):
        if self.functions is None:
            self.functions = index_functions(self.code)
            self.function_starts = [f['start'] for f in self.functions]
        return self.functions

    def function_at(self, offset):
        """The function definition (signature and body) containing offset, or None."""
        funcs = self.get_functions()
        i = bisect.bisect_right(self.function_starts, offset) - 1
        if i >= 0 and funcs[i]['start'] <= offset < funcs[i]['end']:
            return funcs[i]
        return None

    def find_function(self, qualified):
        for f in self.get_functions():
            if f['name'] == qualified:
                return f
        raise KeyError(f'{qualified} not found in {self.rel}')


def match_close(code, pos, open_ch, close_ch):
    """Offset just after the bracket matching code[pos] (which must be open_ch)."""
    depth = 0
    for i in range(pos, len(code)):
        c = code[i]
        if c == open_ch:
            depth += 1
        elif c == close_ch:
            depth -= 1
            if depth == 0:
                return i + 1
    raise ValueError('unbalanced brackets')


FUNC_NAME_RE = re.compile(r'(operator\s*(?:\(\)|[^\s(]+)|[~\w:]+)\s*\($')


def index_functions(code):
    """Return function definitions at namespace or class level: name, start/end offsets.

    ``start`` is the start of the signature (so constructor initializer lists are inside), ``body``
    the opening brace. The name is qualified with the enclosing class for inline methods. Braces
    that open namespaces, classes and initializers are tracked so only function bodies are
    returned; brace-initializers inside a signature or constructor initializer list are skipped.
    """
    funcs, stack, boundary = [], [], 0
    for m in re.finditer(r'[{};]', code):
        pos, ch = m.start(), m.group(0)
        if ch == ';':
            if not stack or stack[-1][0] != 'inline':
                boundary = pos + 1
            continue
        if ch == '}':
            if stack:
                kind, name, start, hstart = stack.pop()
                if kind == 'func':
                    funcs.append(dict(name=name, start=hstart, body=start, end=pos + 1))
                if kind == 'inline':
                    continue
            boundary = pos + 1
            continue
        header = code[boundary:pos].strip()
        hstart = boundary + (len(code[boundary:pos]) - len(code[boundary:pos].lstrip()))
        if any(k == 'func' for k, _, _, _ in stack):
            stack.append(('block', None, pos, pos))
            boundary = pos + 1
            continue
        if stack and stack[-1][0] == 'inline' or header.count('(') > header.count(')') or \
                (re.search(r'\)\s*(?:const\s*)?(?:noexcept\s*)?:(?!:)', header) and re.search(r'\w\s*$', header)):
            # Brace-initializer inside a parameter list or a constructor initializer list.
            stack.append(('inline', None, pos, pos))
            continue
        boundary = pos + 1
        if re.search(r'\bnamespace\b', header) or header.startswith('extern'):
            stack.append(('ns', None, pos, pos))
            continue
        cm = re.search(r'\b(class|struct|union|enum)\s+(?:\w+\s+)*?(\w+)?\s*(?:final\s*)?(?::[^(){}]*)?$', header)
        if cm and '(' not in header.split(cm.group(1), 1)[1]:
            stack.append(('class', cm.group(2) or '<anon>', pos, pos))
            continue
        lam = re.search(r'(\w+)\s*=\s*\[[^\]]*\]\s*\(', header)
        paren = header.find('(')
        if lam or (paren > 0 and '=' not in re.sub(r'operator[^(]*', '', header[:paren])):
            if lam:
                name = lam.group(1)
            else:
                nm = FUNC_NAME_RE.search(header[:paren + 1])
                name = re.sub(r'\s+', '', nm.group(1)) if nm else '<anon>'
            classes = [n for k, n, _, _ in stack if k == 'class']
            if classes and '::' not in name:
                name = classes[-1] + '::' + name
            stack.append(('func', name, pos, hstart))
            continue
        stack.append(('init', None, pos, pos))
    funcs.sort(key=lambda f: f['start'])
    return funcs


# --------------------------------------------------------------------------------------------
# Config definitions
# --------------------------------------------------------------------------------------------

def parse_enum_maps(src):
    """s_keys_map_<Type> -> list of serialized names, and C++ enumerator -> (Type, name)."""
    maps, enumerators = {}, {}
    for m in re.finditer(r't_config_enum_values\s+s_keys_map_(\w+)\s*=?\s*\{', src.code):
        body_end = match_close(src.code, m.end() - 1, '{', '}')
        body = src.text[m.end():body_end]
        names = []
        for e in re.finditer(r'\{\s*"([^"]+)"\s*,\s*(?:int\s*\(\s*)?([\w:]+)', body):
            names.append(e.group(1))
            enumerators.setdefault(e.group(2).split('::')[-1], (m.group(1), e.group(1)))
        maps[m.group(1)] = names
    return maps, enumerators


def normalize_default(expr, enumerators):
    """Best-effort serialized form of a set_default_value() expression."""
    m = re.match(r'new\s+ConfigOption(\w+?)(?:<[^>]*>)?\s*[({](.*)[)}]\s*$', expr.strip(), re.S)
    if not m:
        return None
    kind, inner = m.group(1), m.group(2).strip().strip('{}').strip()
    if inner == '':
        return '' if 'String' in kind else None
    vals = []
    for part in re.split(r',(?![^(]*\))', inner):
        part = part.strip()
        if not part:
            continue
        if part in ('true', 'false'):
            vals.append('1' if part == 'true' else '0')
        elif re.fullmatch(r'"(?:[^"\\]|\\.)*"', part):
            vals.append(part[1:-1])
        elif part.split('::')[-1] in enumerators:
            vals.append(enumerators[part.split('::')[-1]][1])
        else:
            num = re.fullmatch(r'-?\d+(?:\.\d*)?(?:e-?\d+)?f?', part)
            vals.append(part.rstrip('f') if num else part)
    return ','.join(vals)


def parse_defs(root):
    """Parse PrintConfigDef: key -> definition record. Returns (defs, enumerators, warnings)."""
    src = Source(os.path.join(root, 'src/libslic3r/PrintConfig.cpp'), 'src/libslic3r/PrintConfig.cpp')
    enum_maps, enumerators = parse_enum_maps(src)
    warnings, defs = [], {}
    adds = []
    for f in src.get_functions():
        if not f['name'].startswith('PrintConfigDef::init_'):
            continue
        body_code = src.code[f['start']:f['end']]
        for m in re.finditer(r'\bthis->add(_nullable)?\s*\(', body_code):
            adds.append((f['start'] + m.start(), f['start'] + m.end(), bool(m.group(1)), f))
    for idx, (start, args_start, nullable, func) in enumerate(adds):
        end_args = match_close(src.code, args_start - 1, '(', ')')
        args_text = src.text[args_start:end_args - 1]
        args_code = src.code[args_start:end_args - 1]
        block_end = adds[idx + 1][0] if idx + 1 < len(adds) else func['end']
        block_end = min(block_end, func['end'])
        typ = re.search(r'\b(co\w+)\b', args_code)
        typ = typ.group(1) if typ else None
        keys = expand_add_keys(src, start, args_text, args_code, func, warnings)
        props = parse_def_props(src, end_args, block_end, enum_maps, enumerators)
        for key in keys:
            rec = dict(type=typ, nullable=nullable, def_line=src.line_of(start), **props)
            if key in defs:
                # ConfigDef::add() returns the existing def: later fields override, earlier ones stay.
                warnings.append(f'{key} defined twice (PrintConfig.cpp:{defs[key]["def_line"]} and :{rec["def_line"]})')
                rec = {f: (v if v is not None else defs[key][f]) for f, v in rec.items()}
            defs[key] = rec
    # Filament overrides copy the type and enum of the extruder option they override.
    for key, rec in defs.items():
        base = defs.get(key[len('filament_'):]) if key.startswith('filament_') else None
        if rec['type'] is None and base:
            for field in ('type', 'enum_type', 'enum_values', 'min', 'max'):
                rec[field] = rec[field] or base[field]
    return defs, enumerators, warnings


def expand_add_keys(src, start, args_text, args_code, func, warnings):
    """Keys created by one this->add(...) call (literal, or one of the known loop forms)."""
    lit = re.match(r'\s*"([^"]+)"\s*,', args_text)
    if lit:
        return [lit.group(1)]
    base = func['start']
    before = src.code[base:start]

    def literals_of_braces(open_offset):
        return src.strings_in(open_offset, match_close(src.code, open_offset, '{', '}'))

    concat = re.match(r'\s*"([^"]+)"\s*\+\s*(\w+)\.(\w+)\s*,', args_text)
    ident = re.match(r'\s*(\w+)\s*,', args_code)
    if concat:
        # "machine_max_speed_" + axis.name, with axis looping over a braced initializer list.
        loop = list(re.finditer(r'for\s*\(\s*const\s+\w+\s*&\s*' + concat.group(2) + r'\s*:\s*(\w+)\s*\)', before))
        init = loop and list(re.finditer(r'\b' + loop[-1].group(1) + r'\s*\{', before))
        if init:
            ob = base + init[-1].end() - 1
            names = re.findall(r'\{\s*"([^"]+)"', src.text[ob:match_close(src.code, ob, '{', '}')])
            return [concat.group(1) + n for n in names]
    elif ident:
        name = ident.group(1)
        loop = list(re.finditer(r'for\s*\([^:;()]*\b' + name + r'\s*:\s*\{', before))
        if loop:
            return literals_of_braces(base + loop[-1].end() - 1)
        lam = list(re.finditer(r'auto\s+(\w+)\s*=\s*\[[^\]]*\]\s*\(\s*const\s+char\s*\*\s*' + name + r'\b', before))
        if lam:
            rest = src.text[start:func['end']]
            return re.findall(r'\b' + lam[-1].group(1) + r'\s*\(\s*"([^"]+)"', rest)
        over = list(re.finditer(r'for\s*\(\s*auto\s*&\s*' + name + r'\s*:\s*(\w+)\s*\)', before))
        lst = over and re.search(r'\b' + over[-1].group(1) + r'\s*=\s*\{', src.code)
        if lst:
            return literals_of_braces(lst.end() - 1)
    warnings.append(f'unparsed this->add({args_text.strip()[:60]}) at PrintConfig.cpp:{src.line_of(start)}')
    return []


def parse_def_props(src, start, end, enum_maps, enumerators):
    """default, enum type and values, min, max from the statements of one definition."""
    code = src.code[start:end]
    text = src.text[start:end]
    props = dict(default=None, default_value=None, enum_type=None, enum_values=None, min=None, max=None)
    m = re.search(r'set_default_value\s*\(', code)
    if m:
        close = match_close(code, m.end() - 1, '(', ')')
        props['default'] = re.sub(r'\s+', ' ', text[m.end():close - 1]).strip()
        props['default_value'] = normalize_default(props['default'], enumerators)
        et = re.search(r'ConfigOptionEnum(?:sGeneric)?(?:Nullable)?<(\w+)>', props['default'])
        if et:
            props['enum_type'] = et.group(1)
    for which in ('min', 'max'):
        mm = re.search(r'def->' + which + r'\s*=\s*([^;]+);', code)
        if mm:
            props[which] = mm.group(1).strip()
    km = re.search(r'enum_keys_map\s*=\s*&(?:ConfigOptionEnum<(\w+)>::get_enum_values\(\)|s_keys_map_(\w+))', code)
    if km:
        props['enum_type'] = km.group(1) or km.group(2)
    gui_values = []
    for vm in re.finditer(r'def->enum_values\s*(?:\.push_back\s*\(|=\s*\{)', code):
        close = match_close(code, vm.end() - 1, '(' if code[vm.end() - 1] == '(' else '{', ')' if code[vm.end() - 1] == '(' else '}')
        gui_values += src.strings_in(start + vm.end() - 1, start + close)
    if props['enum_type'] in enum_maps:
        props['enum_values'] = enum_maps[props['enum_type']]
    elif gui_values and props['enum_type']:
        props['enum_values'] = gui_values
    return props


def parse_classes(root):
    """Static config class -> list of its own members (PrintConfig's parents are separate classes)."""
    with open(os.path.join(root, 'src/libslic3r/PrintConfig.hpp'), encoding='utf-8', errors='replace') as f:
        lines = f.read().split('\n')
    classes, cur = {}, None
    for i, line in enumerate(lines):
        if re.match(r'PRINT_CONFIG_CLASS_\w*DEFINE\w*\(', line):
            j = i + 1
            while not lines[j].strip():
                j += 1
            cur = lines[j].strip().rstrip(',')
            classes[cur] = []
            continue
        if cur and re.match(r'\s*\)\s*$', line):
            cur = None
        if cur:
            m = re.match(r'\s*\(\(\s*[\w<>:]+\s*,\s*(\w+)\s*\)\)', line)
            if m:
                classes[cur].append(m.group(1))
    return classes


def parse_legacy(root):
    """Keys handle_legacy renames or erases (on the left side of opt_key == "x"), and the ignore set."""
    src = Source(os.path.join(root, 'src/libslic3r/PrintConfig.cpp'), 'src/libslic3r/PrintConfig.cpp')
    f = src.find_function('PrintConfigDef::handle_legacy')
    body = src.text[f['start']:f['end']]
    code = src.code[f['start']:f['end']]
    renamed = {}
    for m in re.finditer(r'(?:\}\s*else\s+)?if\s*\(', code):
        cend = match_close(code, m.end() - 1, '(', ')')
        conds = re.findall(r'opt_key\s*==\s*"([^"]+)"', body[m.end():cend])
        bstart = code.find('{', cend)
        if bstart < 0:
            continue
        bend = match_close(code, bstart, '{', '}')
        targets = re.findall(r'opt_key\s*=\s*"([^"]*)"', body[bstart:bend])
        for k in conds:
            if targets:
                renamed.setdefault(k, targets[0] or '(erased)')
    ig = re.search(r'static\s+std::set<std::string>\s+ignore\s*=\s*\{', code)
    ignored = set(src.strings_in(f['start'] + ig.end(), f['start'] + match_close(code, ig.end() - 1, '{', '}')))
    return renamed, ignored


def parse_step_enums(root):
    """Ordered step names of PrintStep and PrintObjectStep (aliases dropped)."""
    src = Source(os.path.join(root, 'src/libslic3r/Print.hpp'), 'src/libslic3r/Print.hpp')
    out = {}
    for name in ('PrintStep', 'PrintObjectStep'):
        m = re.search(r'enum\s+' + name + r'\s*\{([^}]*)\}', src.code)
        steps = []
        for item in m.group(1).split(','):
            item = item.strip()
            if item and '=' not in item and item not in ('psCount', 'posCount'):
                steps.append(item)
        out[name] = steps
    return out['PrintStep'], out['PrintObjectStep']


# --------------------------------------------------------------------------------------------
# Invalidation
# --------------------------------------------------------------------------------------------

def skip_ws(code, p):
    while p < len(code) and code[p].isspace():
        p += 1
    return p


def parse_statement(code, p):
    """Extent [p, end) of the statement starting at p (a braced block or up to ';')."""
    p = skip_ws(code, p)
    if code[p] == '{':
        return p, match_close(code, p, '{', '}')
    if re.match(r'if\b', code[p:p + 3]):
        return p, parse_if_chain(code, p)[2]
    return p, code.index(';', p) + 1


IF_RE = re.compile(r'if\s*\(')


def parse_if_chain(code, p):
    """Parse ``if (...) S else if (...) S ... else S`` at p.

    Returns ([(if_offset, cond_span, body_span)], else_body_span or None, end_offset).
    """
    branches, else_span = [], None
    while True:
        p = skip_ws(code, p)
        m = IF_RE.match(code, p)
        if not m:
            raise ValueError('expected if')
        cstart = m.end() - 1
        cend = match_close(code, cstart, '(', ')')
        bstart, bend = parse_statement(code, cend)
        branches.append((p, (cstart, cend), (bstart, bend)))
        q = skip_ws(code, bend)
        if code.startswith('else', q) and not re.match(r'\w', code[q + 4:q + 5]):
            q = skip_ws(code, q + 4)
            if re.match(r'if\b', code[q:q + 3]):
                p = q
                continue
            else_span = parse_statement(code, q)
            return branches, else_span, else_span[1]
        return branches, else_span, bend


ACTION_RE = re.compile(r'(?:(\w+)(?:\s*->\s*|\s*\.\s*))?(emplace_back|invalidate_step|invalidate_steps|invalidate_all_steps)\s*\(([^;]*?)\)\s*;')


def branch_actions(src, start, end):
    """Steps a branch body resets: list of (step or ALL_SELF/ALL_PRINT, guards, line).

    ``guards`` lists (condition text, required truth) for every nested ``if`` around the action
    (an action in an ``else`` requires all earlier conditions to be false).
    """
    code = src.code[start:end]
    chains = []
    for m in re.finditer(r'\bif\s*\(', code):
        if re.search(r'\belse\s*$', code[max(0, m.start() - 16):m.start()]):
            continue
        try:
            branches, else_span, chain_end = parse_if_chain(code, m.start())
        except ValueError:
            continue
        conds = [src.text[start + cs + 1:start + ce - 1] for _, (cs, ce), _ in branches]
        chains.append((branches, else_span, conds))
    acts = []
    for m in ACTION_RE.finditer(code):
        recv, call, args = m.group(1), m.group(2), m.group(3)
        guards = []
        for branches, else_span, conds in chains:
            for j, (_, _, (bs, be)) in enumerate(branches):
                if bs <= m.start() < be:
                    guards += [(c, False) for c in conds[:j]] + [(conds[j], True)]
            if else_span and else_span[0] <= m.start() < else_span[1]:
                guards += [(c, False) for c in conds]
        line = src.line_of(start + m.start())
        if call == 'invalidate_all_steps':
            acts.append(('ALL_PRINT' if recv == 'm_print' else 'ALL_SELF', guards, line))
            continue
        for step in re.findall(r'\b(p[os]+[A-Z]\w+)\b', args):
            acts.append((step, guards, line))
    return acts


def eval_condition(cond, key):
    """Three-valued value of a C++ condition for opt_key == key: True, False or None (unknown).

    ``opt_key == "x"`` / ``opt_key != "x"`` are decided; every other operand is unknown.
    """
    toks = re.findall(r'&&|\|\||!(?!=)|\(|\)|"(?:[^"\\]|\\.)*"|[^&|!()"]+|!=', cond)
    toks = [t for t in toks if t.strip()]
    pos = 0

    def peek():
        return toks[pos] if pos < len(toks) else None

    def expr():
        nonlocal pos
        vals = [term()]
        while peek() == '||':
            pos += 1
            vals.append(term())
        return True if True in vals else (False if all(v is False for v in vals) else None)

    def term():
        nonlocal pos
        vals = [factor()]
        while peek() == '&&':
            pos += 1
            vals.append(factor())
        return False if False in vals else (True if all(v is True for v in vals) else None)

    def factor():
        nonlocal pos
        t = peek()
        if t == '!':
            pos += 1
            v = factor()
            return None if v is None else not v
        if t == '(':
            pos += 1
            v = expr()
            if peek() == ')':
                pos += 1
            return v
        # An operand: everything up to the next top-level && / || / unmatched ')'.
        depth, text = 0, ''
        while pos < len(toks):
            t = toks[pos]
            if depth == 0 and t in ('&&', '||', ')'):
                break
            depth += (t == '(') - (t == ')')
            text += t
            pos += 1
        m = re.fullmatch(r'\s*opt_key\s*(==|!=)\s*"([^"]*)"\s*', text)
        if m:
            return (m.group(2) == key) == (m.group(1) == '==')
        return None

    try:
        return expr()
    except IndexError:
        return None


def resolve_actions(actions, key):
    """Split branch actions into (unconditional, conditional) step lists for one key."""
    uncond, cond = [], []
    for step, guards, _ in actions:
        vals = [eval_condition(c, key) == want if eval_condition(c, key) is not None else None for c, want in guards]
        if False in vals:
            continue
        (uncond if all(v is True for v in vals) else cond).append(step)
    return uncond, cond


def parse_cascade(root, object_steps, print_steps):
    """PrintObject::invalidate_step cascade: posX -> set of steps (object, print, SlicingParams)."""
    src = Source(os.path.join(root, 'src/libslic3r/PrintObject.cpp'), 'src/libslic3r/PrintObject.cpp')
    f = src.find_function('PrintObject::invalidate_step')
    code = src.code
    chain_at = re.compile(r'\bif\s*\(\s*step\s*==').search(code, f['start'], f['end'])
    branches, _, chain_end = parse_if_chain(code, chain_at.start())
    cascade = {s: {s} for s in object_steps}
    for _, (cs, ce), (bs, be) in branches:
        steps = re.findall(r'step\s*==\s*(\w+)', code[cs:ce])
        body = code[bs:be]
        add = set(re.findall(r'\b(p[os]+[A-Z]\w+)\b', body))
        if re.search(r'm_slicing_params\.valid\s*=\s*false', body):
            add.add('SlicingParams')
        for s in steps:
            cascade[s] |= add
    tail = code[f['start']:chain_at.start()] + code[chain_end:f['end']]
    always = set(re.findall(r'm_print->invalidate_step\(\s*(ps\w+)\s*\)', tail))
    for s in cascade:
        cascade[s] |= always
    # Print::invalidate_step: any step other than psGCodeExport also resets psGCodeExport.
    psrc = Source(os.path.join(root, 'src/libslic3r/Print.cpp'), 'src/libslic3r/Print.cpp')
    pf = psrc.find_function('Print::invalidate_step')
    pbody = psrc.code[pf['start']:pf['end']]
    if not (re.search(r'step\s*!=\s*psGCodeExport', pbody) and re.search(r'invalidate_step\(\s*psGCodeExport\s*\)', pbody)):
        raise SystemExit('Print::invalidate_step changed shape; update parse_cascade()')
    # PrintObject::invalidate_all_steps also resets the slicing parameters.
    af = src.find_function('PrintObject::invalidate_all_steps')
    all_object = set(object_steps) | set(print_steps)
    if re.search(r'm_slicing_params\.valid\s*=\s*false', code[af['start']:af['end']]):
        all_object.add('SlicingParams')
    return cascade, all_object


def closure(actions, cascade, all_object_and_print, print_steps, context):
    """Steps reset by a list of branch actions (see module docstring)."""
    out = set()
    for step in actions:
        if step == 'ALL_SELF':
            out |= all_object_and_print if context == 'PrintObject' else set(print_steps)
        elif step == 'ALL_PRINT':
            out |= set(print_steps)
        elif step.startswith('pos'):
            out |= cascade.get(step, {step})
        else:
            out |= {step, 'psGCodeExport'}
    return out


def parse_invalidation(root, func_name, path):
    """Branches of an invalidate_state_by_config_options chain (first match is effective)."""
    src = Source(os.path.join(root, path), path)
    f = src.find_function(func_name)
    code = src.code
    loop = re.compile(r'for\s*\(\s*const\s+t_config_option_key\s*&\s*opt_key\s*:\s*opt_keys\s*\)').search(code, f['start'], f['end'])
    lb = code.index('{', loop.end())
    first_if = re.compile(r'\bif\s*\(').search(code, lb)
    branches, else_span, _ = parse_if_chain(code, first_if.start())
    named_sets = {}
    for m in re.finditer(r'static\s+std::unordered_set<std::string>\s+(\w+)\s*(=\s*\{)?', code[f['start']:f['end']]):
        if m.group(2):
            ob = f['start'] + m.end() - 1
            ce = match_close(code, ob, '{', '}')
            named_sets[m.group(1)] = [(v, src.line_of(o)) for o, v in src.strings if ob <= o < ce]
        else:
            named_sets[m.group(1)] = []
    out = []
    for i, (ifpos, (cs, ce), (bs, be)) in enumerate(branches):
        listed = [(v, src.line_of(o)) for o, v in src.strings if cs <= o < ce
                  and re.search(r'opt_key\s*==\s*"$', src.text[max(cs, o - 40):o + 1])]
        for name in named_sets:
            if re.search(r'\b' + name + r'\.find\s*\(\s*opt_key\s*\)', code[cs:ce]):
                listed += named_sets[name]
        out.append(dict(index=i, line=src.line_of(ifpos), keys=listed, actions=branch_actions(src, bs, be)))
    else_branch = dict(index='else', line=src.line_of(else_span[0]), keys=[], actions=branch_actions(src, *else_span))
    return out, else_branch, path


# --------------------------------------------------------------------------------------------
# Consumers
# --------------------------------------------------------------------------------------------

DECL_RE = re.compile(r'\b(?:double|float|int|bool|unsigned|size_t|coordf_t|coord_t|auto|long|char|'
                     r'std::string|std::vector<[^;(){}]*>|Vec\w+|Point\w*|Polygons?|ExPolygons?|Polylines?|'
                     r'BoundingBox\w*|Flow|[A-Z]\w*Type|[A-Z]\w*Pattern|[A-Z]\w*Mode)\s*[&*]?\s*([A-Za-z_]\w*)\s*(?=[;=,)\[{])')
ACCESSOR_RE = re.compile(r'\b(opt_\w+|opt|option|option_throw|optptr|get_abs_value(?:_at)?|has|set|set_key_value|set_deserialize\w*|erase)'
                         r'\s*(?:<[^<>()]*(?:<[^<>]*>)?[^<>()]*>)?\s*\(\s*$')
WRITE_ACCESSORS = ('set', 'set_key_value', 'set_deserialize', 'set_deserialize_strict', 'set_deserialize_nothrow', 'erase')
CONFIG_RECEIVER_RE = re.compile(r'(?:config|cfg|conf)', re.I)
# A read followed by .size() / .values.size() / .empty() only counts the vector's elements: the
# number of filaments or extruders, whose change Print::apply handles separately.
SIZE_ONLY_RE = re.compile(r'\s*(?:(?:\.|->)\s*values\s*)?(?:\.|->)\s*(?:size|empty)\s*\(')


def file_step(rel):
    for pattern, step in FILE_STEP:
        if fnmatch.fnmatch(rel, pattern):
            return step
    return 'unmapped'


def function_table_step(name):
    """FUNCTION_STEP entry for a qualified function name; unqualified keys match by short name."""
    return FUNCTION_STEP.get(name) or FUNCTION_STEP.get(name.split('::')[-1])


def is_pipeline_step(step):
    return step.startswith('pos') or step.startswith('ps') or step in PSEUDO_CACHED_STEPS


def function_steps(src, called_elsewhere):
    """Map each function of a libslic3r file to (steps, how) and set_started spans.

    ``called_elsewhere(name)`` tells whether another libslic3r file calls ``name``; such a
    function is treated as shared and does not inherit a caller's step.
    Returns (by_function_start: {start: (set(steps), how)}, spans: [(start, end, step)]).
    """
    funcs = src.get_functions()
    by_name = collections.defaultdict(list)
    for f in funcs:
        by_name[f['name'].split('::')[-1]].append(f)
    assigned, spans = {}, []
    for f in funcs:
        body = src.code[f['start']:f['end']]
        table_step = function_table_step(f['name'])
        if table_step:
            assigned[f['start']] = ({table_step}, 'function table')
        started = [(m.start(), m.group(1)) for m in re.finditer(r'(?<![\w>.])(?:this->)?set_started\s*\(\s*(p\w+)\s*\)', body)]
        steps = sorted({s for _, s in started})
        if len(steps) == 1 and f['start'] not in assigned:
            assigned[f['start']] = ({steps[0]}, 'set_started')
        elif len(steps) > 1:
            for off, step in started:
                done = re.compile(r'set_done\s*\(\s*' + step + r'\s*\)').search(body, off)
                spans.append((f['start'] + off, f['start'] + (done.end() if done else len(body)), step))

    def region_steps(off):
        for a, b, step in spans:
            if a <= off < b:
                return {step}
        f = src.function_at(off)
        steps = assigned.get(f['start'], (set(), ''))[0] if f else set()
        return {s for s in steps if is_pipeline_step(s)}

    # Call sites of same-file functions; a callee inherits its callers' steps only when every
    # call site is in a step-attributed region and nothing outside the file calls it.
    call_sites = collections.defaultdict(list)
    for m in re.finditer(r'\b(\w+)\s*\(', src.code):
        if m.group(1) in by_name:
            call_sites[m.group(1)].append(m.start())
    changed = True
    while changed:
        changed = False
        for name, fs in by_name.items():
            if len(fs) != 1 or called_elsewhere(name):
                continue
            f = fs[0]
            if f['start'] in assigned and assigned[f['start']][1] != 'call graph':
                continue
            # Call sites inside function bodies (definitions and declarations are outside).
            sites = [o for o in call_sites[name] if not (f['start'] <= o < f['end']) and src.function_at(o) is not None]
            if not sites:
                continue
            per_site = [region_steps(o) for o in sites]
            if any(not s for s in per_site):
                continue
            steps = set().union(*per_site)
            if assigned.get(f['start'], (set(), ''))[0] != steps:
                assigned[f['start']] = (steps, 'call graph')
                changed = True
    return assigned, spans


def called_names(src):
    """Names called inside function bodies of a file (declarations do not count)."""
    names = set()
    for f in src.get_functions():
        names |= set(re.findall(r'\b(\w+)\s*\(', src.code[f['body'] + 1:f['end']]))
    return names


class ConsumerScanner:
    """Finds reads of config keys in the source tree and attributes each to a pipeline step."""

    def __init__(self, root, keys):
        self.root, self.keys = root, set(keys)
        self.files = []
        for base in ('src/libslic3r', 'src/slic3r'):
            for dirpath, _, names in os.walk(os.path.join(root, base)):
                for n in names:
                    if n.endswith(SOURCE_EXTENSIONS):
                        self.files.append(os.path.join(dirpath, n))
        for n in sorted(os.listdir(os.path.join(root, 'src'))):
            if n.endswith(SOURCE_EXTENSIONS):
                self.files.append(os.path.join(root, 'src', n))
        self.files.sort()
        self.sources = [Source(p, os.path.relpath(p, root)) for p in self.files]
        self.ambiguous = self._ambiguous_names()

    def _ambiguous_names(self):
        """Keys that are also declared as a variable, parameter or field somewhere in src/."""
        names = set()
        for src in self.sources:
            if src.rel in REGISTRY_FILES:
                continue
            for m in DECL_RE.finditer(src.code):
                if m.group(1) in self.keys:
                    names.add(m.group(1))
        return names

    def scan(self):
        """key -> list of consumer dicts."""
        out = collections.defaultdict(list)
        lib_sources = [s for s in self.sources if s.rel.startswith('src/libslic3r/')]
        calls = {s.rel: called_names(s) for s in lib_sources}
        call_count = collections.Counter(n for names in calls.values() for n in names)
        for src in self.sources:
            lib = src.rel.startswith('src/libslic3r/')
            own = calls.get(src.rel, set())
            assigned, spans = function_steps(src, lambda n: call_count[n] > (n in own)) if lib else ({}, [])
            fstep = file_step(src.rel)
            best = {}
            for off, key, kind, conf in self._reads(src):
                line = src.line_of(off)
                if (key, line) not in best or CONF_RANK[conf] > CONF_RANK[best[(key, line)][2]]:
                    best[(key, line)] = (off, kind, conf)
            for (key, line), (off, kind, conf) in sorted(best.items(), key=lambda kv: kv[0][1]):
                func = src.function_at(off)
                if func and func['name'] in EXCLUDED_FUNCTIONS:
                    continue
                if src.rel == 'src/libslic3r/PrintConfig.cpp' and (func is None or func['name'].startswith('PrintConfigDef::')):
                    continue
                steps, how = self._steps_for(off, func, assigned, spans, fstep)
                out[key].append(dict(file=src.rel, line=line, function=func['name'] if func else None,
                                     steps=sorted(steps), step_source=how, kind=kind, confidence=conf))
        return out

    @staticmethod
    def _steps_for(off, func, assigned, spans, fstep):
        for a, b, step in spans:
            if a <= off < b:
                return {step}, 'set_started span'
        if func and func['start'] in assigned:
            return assigned[func['start']]
        return {fstep}, 'file table'

    def _reads(self, src):
        """Yield (offset, key, kind, confidence) for every read in a file."""
        code, keys = src.code, self.keys
        registry = src.rel in REGISTRY_FILES
        for m in re.finditer(r'(\.|->)\s*([A-Za-z_]\w*)\b(?!\s*\()', code):
            key = m.group(2)
            if key not in keys:
                continue
            before = code[max(0, m.start() - 60):m.start()]
            rm = re.search(r'([A-Za-z_]\w*)\s*(\(\s*\))?\s*(?:\[[^\[\]]*\])?\s*$', before)
            recv = rm.group(1) if rm else ''
            if SIZE_ONLY_RE.match(code, m.end()):
                yield m.start(2), key, 'size only', 'low'
                continue
            if recv == 'this':
                conf = 'low'
            elif CONFIG_RECEIVER_RE.search(recv):
                conf = 'high'
            else:
                conf = 'low' if key in self.ambiguous else 'medium'
            yield m.start(2), key, 'member', conf
        for m in re.finditer(r'\b(\w*Config)\s*::\s*([A-Za-z_]\w*)\b', code):
            if m.group(2) in keys and not registry:
                yield m.start(2), m.group(2), 'member pointer', 'high'
        for m in re.finditer(r'\b[A-Z_]*CONFIG\s*\(\s*([A-Za-z_]\w*)\s*\)', code):
            if m.group(1) in keys:
                yield m.start(1), m.group(1), 'macro', 'high'
        for off, value in src.strings:
            if value not in keys:
                continue
            window_start = max(0, off - 120)
            am = ACCESSOR_RE.search(code[window_start:off])
            if am:
                if am.group(1) in WRITE_ACCESSORS:
                    continue
                try:
                    after = match_close(code, code.rindex('(', window_start, off), '(', ')')
                except ValueError:
                    after = off
                if SIZE_ONLY_RE.match(code, after):
                    yield off, value, 'size only', 'low'
                else:
                    yield off, value, 'accessor ' + am.group(1), 'high'
            elif not registry:
                yield off, value, 'string literal', 'low'


# --------------------------------------------------------------------------------------------
# Tests
# --------------------------------------------------------------------------------------------

PAIR_RE = re.compile(r'\{\s*"([A-Za-z_]\w*)"\s*,\s*((?:"(?:[^"\\]|\\.)*"|[^{}"])+?)\s*(?:,\s*(?:true|false)\s*)?\}')
CALL_SET_RE = re.compile(r'\b(set_deserialize_strict|set_deserialize_nothrow|set_deserialize|set_key_value|set)\s*\(\s*"([A-Za-z_]\w*)"\s*,\s*(.+)')
OPT_ASSIGN_RE = re.compile(r'\b(?:opt_\w+|opt|option)\s*(?:<[^<>()]*(?:<[^<>]*>)?[^<>()]*>)?\s*\(\s*"([A-Za-z_]\w*)"[^)]*\)\s*(?:->\s*values?|\.\s*values?|\[[^\]]*\])?\s*=(?!=)\s*([^;]+)')
MEMBER_ASSIGN_RE = re.compile(r'(\w*)\s*(?:\.|->)\s*([A-Za-z_]\w*)\s*(\.\s*values?)?\s*=(?!=)\s*([^;]+)')
INI_RE = re.compile(r'^\s*([A-Za-z_]\w*)\s*=\s*(.*)$')
CLI_RE = re.compile(r'--([a-z][a-z0-9_-]*)(?:[= ]+([^\s"\'-][^\s"\']*|"[^"]*"))?')


def test_values(expr, enumerators):
    """Candidate serialized values in a test's value expression (all branches of a ternary)."""
    vals = []
    for s in re.findall(r'"((?:[^"\\]|\\.)*)"', expr):
        vals.append(s)
    for tok in re.findall(r'\b(true|false)\b', expr):
        vals.append('1' if tok == 'true' else '0')
    for tok in re.findall(r'[\w:]+', expr):
        e = enumerators.get(tok.split('::')[-1])
        if e:
            vals.append(e[1])
    if not vals:
        nums = re.findall(r'(?<![\w.])-?\d+(?:\.\d*)?(?![\w.])', expr)
        if nums:
            vals.append(','.join(nums))
    return vals


def scan_tests(root, keys, enumerators):
    """key -> list of {file, line, how, expr, values} for each place a test sets the key."""
    out = collections.defaultdict(list)
    paths = []
    for dirpath, dirnames, names in os.walk(os.path.join(root, 'tests')):
        dirnames[:] = [d for d in dirnames if d != 'catch2']
        for n in names:
            if n.endswith(SOURCE_EXTENSIONS + ('.ini', '.sh')):
                paths.append(os.path.join(dirpath, n))
    for p in sorted(paths):
        rel = os.path.relpath(p, root)
        with open(p, encoding='utf-8', errors='replace') as f:
            lines = f.read().split('\n')
        for i, line in enumerate(lines, 1):
            s = line.strip()
            if s.startswith('//') or s.startswith('*') or (s.startswith('#') and not p.endswith(SOURCE_EXTENSIONS)):
                continue
            line = re.sub(r'//.*$', '', line) if p.endswith(SOURCE_EXTENSIONS) else line
            found = []
            if p.endswith('.ini'):
                m = INI_RE.match(line)
                if m:
                    found.append((m.group(1), m.group(2), 'ini'))
            elif p.endswith('.sh'):
                for m in CLI_RE.finditer(line):
                    found.append((m.group(1).replace('-', '_'), m.group(2) or '', 'cli'))
            else:
                for m in PAIR_RE.finditer(line):
                    found.append((m.group(1), m.group(2), 'pair'))
                for m in CALL_SET_RE.finditer(line):
                    found.append((m.group(2), m.group(3), m.group(1)))
                for m in OPT_ASSIGN_RE.finditer(line):
                    found.append((m.group(1), m.group(2), 'opt assignment'))
                for m in MEMBER_ASSIGN_RE.finditer(line):
                    # cfg.key.value = v, or config.key = v (other receivers may be plain structs).
                    if m.group(3) or CONFIG_RECEIVER_RE.search(m.group(1)):
                        found.append((m.group(2), m.group(4), 'member assignment'))
            seen = set()
            for key, expr, how in found:
                if key not in keys or (key, how) in seen:
                    continue
                seen.add((key, how))
                expr = expr.strip()
                out[key].append(dict(file=rel, line=i, how=how, expr=expr[:120], values=test_values(expr, enumerators)))
    return out


# --------------------------------------------------------------------------------------------
# Profiles
# --------------------------------------------------------------------------------------------

def preset_metadata_keys(root):
    keys = set()
    for name in ('Preset.hpp', 'PresetBundle.hpp'):
        p = os.path.join(root, 'src/libslic3r', name)
        if os.path.exists(p):
            with open(p, encoding='utf-8', errors='replace') as f:
                keys |= set(re.findall(r'#define\s+\w+_JSON_KEY_\w+\s+"([^"]+)"', f.read()))
    return keys


def scan_profiles(root, defs, renamed, ignored):
    """Undefined profile keys: key -> {files, example, legacy} for machine/process/filament JSON."""
    meta = preset_metadata_keys(root)
    found = collections.defaultdict(list)
    for p in sorted(glob.glob(os.path.join(root, 'resources/profiles/**/*.json'), recursive=True)):
        try:
            with open(p, encoding='utf-8') as f:
                j = json.load(f)
        except (ValueError, UnicodeDecodeError):
            continue
        if not isinstance(j, dict) or j.get('type') not in ('machine', 'process', 'filament'):
            continue
        for k in j:
            if k not in defs and k not in meta:
                found[k].append(os.path.relpath(p, root))
    out = {}
    for k, files in found.items():
        legacy = renamed.get(k) or ('(ignored)' if k in ignored else None)
        vendors = collections.Counter(f.split('/')[2] for f in files)
        out[k] = dict(files=len(files), vendors=dict(vendors.most_common()), example=files[0], legacy=legacy)
    return out


# --------------------------------------------------------------------------------------------
# Flags and report
# --------------------------------------------------------------------------------------------

def git_revision(root):
    try:
        return subprocess.run(['git', '-C', root, 'rev-parse', '--short', 'HEAD'], capture_output=True, text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return 'unknown'


def build_inventory(root):
    defs, enumerators, warnings = parse_defs(root)
    classes = parse_classes(root)
    renamed, ignored = parse_legacy(root)
    print_steps, object_steps = parse_step_enums(root)
    cascade, all_steps = parse_cascade(root, object_steps, print_steps)

    member_of = collections.defaultdict(list)
    for cls, members in classes.items():
        for k in members:
            member_of[k].append(cls)
    fff = {k for c in FFF_CLASSES for k in classes.get(c, ()) if k in defs}
    print_level = {k for c in PRINT_LEVEL_CLASSES for k in classes.get(c, ())}
    object_level = {k for c in OBJECT_LEVEL_CLASSES for k in classes.get(c, ())}

    chains = {
        'Print': parse_invalidation(root, 'Print::invalidate_state_by_config_options', 'src/libslic3r/Print.cpp'),
        'PrintObject': parse_invalidation(root, 'PrintObject::invalidate_state_by_config_options', 'src/libslic3r/PrintObject.cpp'),
    }
    cached = set(object_steps) | set(CACHED_PRINT_STEPS) | set(PSEUDO_CACHED_STEPS)
    never_done = steps_never_done(root, object_steps)
    cached -= never_done

    scanner = ConsumerScanner(root, defs.keys())
    consumers = scanner.scan()
    tests = scan_tests(root, set(defs), enumerators)
    profiles = scan_profiles(root, defs, renamed, ignored)

    flags = collections.defaultdict(list)
    listed_anywhere = {}
    for ctx, (branches, else_branch, path) in chains.items():
        first = {}
        for b in branches:
            for k, line in b['keys']:
                listed_anywhere.setdefault((ctx, k), []).append(line)
                if k not in defs:
                    flags['UNKNOWN_IN_INVALIDATION'].append(dict(key=k, where=f'{path}:{line}', function=ctx, legacy=renamed.get(k)))
                    continue
                wrong = (ctx == 'Print' and k not in print_level) or (ctx == 'PrintObject' and k not in object_level)
                if wrong and (k, ctx) not in {(f['key'], f['function']) for f in flags['WRONG_LIST']}:
                    flags['WRONG_LIST'].append(dict(key=k, where=f'{path}:{line}', function=ctx,
                                                    classes=member_of.get(k, []) or ['(none)'],
                                                    effective=effective_branch_label(k, print_level, object_level)))
                if k in first:
                    fb = first[k]
                    flags['SHADOWED'].append(dict(key=k, where=f'{path}:{line}', function=ctx,
                                                  effective_line=f'{path}:{fb["line"]}',
                                                  lost=sorted(closure(sum(resolve_actions(b['actions'], k), []), cascade, all_steps, print_steps, ctx)
                                                             - closure(sum(resolve_actions(fb['actions'], k), []), cascade, all_steps, print_steps, ctx))))
                else:
                    first[k] = b
        chains[ctx] = (branches, else_branch, path, first)

    inventory = {}
    for key in sorted(defs):
        d = defs[key]
        rec = dict(type=d['type'], default=d['default_value'] if d['default_value'] is not None else d['default'],
                   enum_values=d['enum_values'], min=d['min'], max=d['max'], nullable=d['nullable'],
                   defined_at=f'src/libslic3r/PrintConfig.cpp:{d["def_line"]}',
                   classes=sorted(member_of.get(key, [])), fff=key in fff)
        rec['invalidation'] = invalidation_for(key, chains, print_level, object_level, cascade, all_steps, print_steps)
        rec['consumers'] = consumers.get(key, [])
        rec['tests'] = tests.get(key, [])
        rec['flags'] = []
        inventory[key] = rec

    for key, rec in inventory.items():
        derive_key_flags(key, rec, defs[key], cached, flags)

    for cls in FFF_CLASSES:
        for k in classes.get(cls, ()):
            if k not in defs:
                flags['MEMBER_WITHOUT_DEF'].append(dict(key=k, cls=cls))
    for k, info in sorted(profiles.items(), key=lambda kv: -kv[1]['files']):
        if not info['legacy']:
            flags['PROFILE_UNKNOWN'].append(dict(key=k, files=info['files'], vendors=info['vendors'], example=info['example']))
    profile_legacy = {k: v for k, v in profiles.items() if v['legacy']}

    meta = dict(revision=git_revision(root), keys=len(defs), fff_keys=len(fff),
                classes={c: len(classes.get(c, ())) for c in FFF_CLASSES},
                print_steps=print_steps, object_steps=object_steps,
                cascade={k: sorted(v) for k, v in cascade.items()},
                never_done_steps=sorted(never_done),
                parse_warnings=warnings,
                ambiguous_names=sorted(scanner.ambiguous),
                profile_legacy_keys={k: dict(files=v['files'], handled_as=v['legacy']) for k, v in sorted(profile_legacy.items())})
    return inventory, flags, meta


def steps_never_done(root, object_steps):
    """Object steps whose entry function calls set_started but never set_done (re-run every time)."""
    out = set()
    for path in ('src/libslic3r/PrintObject.cpp', 'src/libslic3r/PrintObjectSlice.cpp'):
        src = Source(os.path.join(root, path), path)
        for f in src.get_functions():
            body = src.code[f['start']:f['end']]
            started = set(re.findall(r'set_started\s*\(\s*(pos\w+)\s*\)', body))
            for s in started:
                if s in object_steps and not re.search(r'set_done\s*\(\s*' + s + r'\s*\)', body):
                    out.add(s)
    return out


def effective_branch_label(key, print_level, object_level):
    """Name the invalidation function that sees a change of `key`."""
    if key in object_level:
        return 'PrintObject function'
    if key in print_level:
        return 'Print function'
    return 'full-config diff only (psGCodeExport)'


def invalidation_for(key, chains, print_level, object_level, cascade, all_steps, print_steps):
    """Which branch handles the key and the steps it resets (unconditional and conditional)."""
    if key in object_level:
        ctx = 'PrintObject'
    elif key in print_level:
        ctx = 'Print'
    else:
        return dict(function='full-config diff', branch=None, where=None, resets=['psGCodeExport'], conditional_resets=[])
    branches, else_branch, path, first = chains[ctx]
    b = first.get(key, else_branch)
    uncond, cond = resolve_actions(b['actions'], key)
    resets = closure(uncond, cascade, all_steps, print_steps, ctx)
    cresets = closure(cond, cascade, all_steps, print_steps, ctx) - resets
    return dict(function=f'{ctx}::invalidate_state_by_config_options', branch=b['index'],
                where=f'{path}:{b["line"]}', resets=sorted(resets), conditional_resets=sorted(cresets))


CONF_RANK = {'high': 2, 'medium': 1, 'low': 0}
MAP_RANK = {'function table': 2, 'set_started': 2, 'set_started span': 2, 'file table': 1, 'call graph': 0}


def derive_key_flags(key, rec, d, cached, flags):
    inv = rec['invalidation']
    reset, creset = set(inv['resets']), set(inv['conditional_resets'])
    cons = rec['consumers']
    strong = [c for c in cons if c['confidence'] != 'low']
    by_step = collections.defaultdict(list)
    for c in strong:
        for s in c['steps']:
            if s in cached and s not in reset:
                by_step[s].append(c)
    for step, cs in by_step.items():
        best = max(cs, key=lambda c: (CONF_RANK[c['confidence']], MAP_RANK[c['step_source']]))
        score = (3 if step.startswith('pos') else 2) + CONF_RANK[best['confidence']] \
            + MAP_RANK[best['step_source']] + (0 if step in creset else 1)
        row = dict(key=key, step=step, kind='conditional' if step in creset else 'missing',
                   branch=inv['where'] or inv['function'], resets=inv['resets'], score=score,
                   consumers=list(dict.fromkeys(f"{c['file']}:{c['line']}" for c in cs)),
                   confidence=best['confidence'], step_source=best['step_source'],
                   function=best['function'], verified=HAND_VERIFIED.get((key, step)))
        flags['STALE_RISK'].append(row)
        rec['flags'].append(f'STALE_RISK:{step}')
    if rec['fff'] and inv['branch'] == 'else':
        flags['DEFAULT_INVALIDATION'].append(dict(key=key, function=inv['function'], resets=inv['resets']))
        rec['flags'].append('DEFAULT_INVALIDATION')
    lib = [c for c in cons if c['file'].startswith('src/libslic3r/')]
    gui = [c for c in cons if c['file'].startswith('src/slic3r/')]
    if not cons:
        flags['UNREAD'].append(dict(key=key, fff=rec['fff'], classes=rec['classes']))
        rec['flags'].append('UNREAD')
    elif not lib and gui and len(gui) == len(cons):
        flags['GUI_ONLY'].append(dict(key=key, fff=rec['fff'], classes=rec['classes'], gui_reads=len(gui)))
        rec['flags'].append('GUI_ONLY')
    if rec['fff']:
        selected = set()
        for t in rec['tests']:
            for v in t['values']:
                selected |= {v} | ({x.strip() for x in v.split(',')} if rec['type'].endswith('s') else set())
        if not rec['tests']:
            flags['UNTESTED'].append(dict(key=key, classes=rec['classes'], type=rec['type']))
            rec['flags'].append('UNTESTED')
        domain = None
        if rec['enum_values'] and rec['type'] in ('coEnum', 'coEnums'):
            domain = list(dict.fromkeys(rec['enum_values']))
        elif rec['type'] in ('coBool', 'coBools'):
            domain = ['0', '1']
        if domain:
            default = set((rec['default'] or '').split(',')) if isinstance(rec['default'], str) else set()
            missing = [v for v in domain if v not in selected and v not in default]
            implicit = [v for v in domain if v not in selected and v in default]
            rec['value_coverage'] = dict(selected=sorted(selected & set(domain)), implicit=implicit, never=missing)
            if missing:
                flags['UNTESTED_VALUE'].append(dict(key=key, never=missing, implicit=implicit,
                                                    selected=sorted(selected & set(domain))))
                rec['flags'].append('UNTESTED_VALUE')


def to_json(value, depth=0):
    """JSON text with one line per scalar field and one compact line per list item of dicts."""
    pad, inner = ' ' * depth, ' ' * (depth + 1)
    if isinstance(value, dict) and value:
        items = [f'{inner}{json.dumps(k)}: {to_json(v, depth + 1)}' for k, v in value.items()]
        return '{\n' + ',\n'.join(items) + '\n' + pad + '}'
    if isinstance(value, list) and value and isinstance(value[0], dict):
        items = [inner + json.dumps(v, ensure_ascii=False) for v in value]
        return '[\n' + ',\n'.join(items) + '\n' + pad + ']'
    return json.dumps(value, ensure_ascii=False)


def write_json(path, inventory, flags, meta):
    data = dict(meta=meta, flag_counts={k: len(v) for k, v in sorted(flags.items())}, settings=inventory)
    with open(path, 'w', encoding='utf-8') as f:
        f.write(to_json(data) + '\n')


def md_table(headers, rows):
    out = ['| ' + ' | '.join(headers) + ' |', '|' + '---|' * len(headers)]
    for r in rows:
        out.append('| ' + ' | '.join(str(c).replace('|', '\\|').replace('\n', ' ') for c in r) + ' |')
    return out


def write_md(path, inventory, flags, meta):
    fff_keys = [k for k, r in inventory.items() if r['fff']]
    L = ['# Settings inventory', '',
         f'Generated by `tests/settings_inventory/settings_inventory.py` at revision `{meta["revision"]}`. '
         'Static analysis of the source; nothing is compiled or run. The rules behind each flag are in the '
         'script docstring and in `README.md`. Full per-key data: `settings_inventory.json`.', '',
         '## Counts', '']
    tested = sum(1 for k in fff_keys if inventory[k]['tests'])
    rows = [('defined keys (PrintConfigDef)', meta['keys']), ('FFF keys (union of the classes below)', meta['fff_keys'])]
    rows += [(f'  {c}', n) for c, n in meta['classes'].items()]
    rows += [('FFF keys set by at least one test', tested),
             ('FFF keys with a high/medium consumer in src/libslic3r',
              sum(1 for k in fff_keys if any(c['confidence'] != 'low' and c['file'].startswith('src/libslic3r/') for c in inventory[k]['consumers'])))]
    rows += [(f'flag {k}', len(v)) for k, v in sorted(flags.items())]
    L += md_table(['what', 'count'], rows) + ['']
    if meta['parse_warnings']:
        L += ['Parse warnings: ' + '; '.join(meta['parse_warnings']), '']

    stale = sorted(flags['STALE_RISK'], key=lambda r: (-r['score'], r['kind'] != 'missing', r['key'], r['step']))
    verified = [r for r in stale if r['verified']]
    L += ['## STALE_RISK', '',
          'A read of the key runs in a cached step that the key\'s invalidation branch does not reset, so after '
          'changing the key in the GUI the step can keep a result computed with the old value. Ranked by '
          'materiality (object step > print step, confidence of the read, precision of the step mapping). '
          '`conditional`: the step is reset only under a nested condition in the branch.', '',
          f'Rows: {len(stale)} ({sum(1 for r in stale if r["kind"] == "missing")} missing, '
          f'{sum(1 for r in stale if r["kind"] == "conditional")} conditional). Hand-verified: {len(verified)} '
          '(column "verdict"; verdicts were made by reading the code at the revision above).', '']
    rows = []
    for r in stale:
        v = r['verified']
        rows.append((r['key'], r['step'], r['kind'], r['branch'], ', '.join(r['consumers'][:3]) + (f' (+{len(r["consumers"]) - 3})' if len(r['consumers']) > 3 else ''),
                     r['function'] or '', r['confidence'], r['step_source'], f'**{v[0]}**: {v[1]}' if v else ''))
    L += md_table(['key', 'step not reset', 'kind', 'effective branch', 'reads', 'function', 'confidence', 'step mapped by', 'verdict'], rows) + ['']
    L += ['Hand-check results that changed the rules (these rows are no longer produced):', '']
    L += [f'- `{keys}` / {step}: {note}' for keys, step, note in HAND_REJECTED] + ['']

    L += ['## WRONG_LIST', '', 'Keys listed in an invalidation function that never receives them; the listing is dead '
          'and the key is handled where its class sends it.', '']
    L += md_table(['key', 'listed in', 'function', 'classes', 'actually handled by'],
                  [(r['key'], r['where'], r['function'], ', '.join(r['classes']), r['effective']) for r in flags['WRONG_LIST']]) + ['']
    L += ['## UNKNOWN_IN_INVALIDATION', '', 'Names in an invalidation list that are not defined keys.', '']
    L += md_table(['name', 'listed in', 'handle_legacy maps it to'],
                  [(r['key'], r['where'], r['legacy'] or '') for r in flags['UNKNOWN_IN_INVALIDATION']]) + ['']
    L += ['## PROFILE_UNKNOWN', '', 'Keys in shipped machine/process/filament profiles that are not defined, not '
          'renamed or ignored by `handle_legacy`, and not preset metadata. They are dropped silently on load.', '']
    L += md_table(['key', 'profiles', 'vendors', 'example'],
                  [(r['key'], r['files'], ', '.join(f'{v} {n}' for v, n in list(r['vendors'].items())[:4]) + (' ...' if len(r['vendors']) > 4 else ''),
                    r['example']) for r in flags['PROFILE_UNKNOWN']]) + ['']
    leg = meta['profile_legacy_keys']
    if leg:
        L += ['Profile keys handled by `handle_legacy` (not flagged): ' +
              ', '.join(f'`{k}` ({v["files"]}, {v["handled_as"]})' for k, v in leg.items()), '']

    unread_fff = [r for r in flags['UNREAD'] if r['fff']]
    L += ['## UNREAD', '', f'Defined keys with no read anywhere in `src/`: {len(flags["UNREAD"])}, '
          f'of which {len(unread_fff)} are in an FFF class.', '']
    L += md_table(['key', 'classes'], [(r['key'], ', '.join(r['classes']) or '(none)') for r in sorted(flags['UNREAD'], key=lambda r: (not r['fff'], r['key']))]) + ['']
    gui_fff = [r for r in flags['GUI_ONLY'] if r['fff']]
    L += ['## GUI_ONLY', '', f'Keys read only in `src/slic3r` (GUI): {len(flags["GUI_ONLY"])}; FFF-class keys among them '
          f'({len(gui_fff)}) are listed. Non-FFF keys (presets, hosts, UI state) are expected here.', '']
    L += md_table(['key', 'classes', 'GUI reads'], [(r['key'], ', '.join(r['classes']), r['gui_reads']) for r in gui_fff]) + ['']
    L += ['## MEMBER_WITHOUT_DEF', '', 'Static class members with no PrintConfigDef definition (never get a default, '
          'never appear in `keys()`).', '']
    L += md_table(['member', 'class'], [(r['key'], r['cls']) for r in flags['MEMBER_WITHOUT_DEF']]) + ['']
    L += ['## SHADOWED', '', 'A later listing of a key that an earlier branch already handles (informational; '
          '"lost" shows the actions of the later branch that never run).', '']
    L += md_table(['key', 'shadowed listing', 'effective branch', 'lost actions'],
                  [(r['key'], r['where'], r['effective_line'], ', '.join(r['lost'])) for r in flags['SHADOWED']]) + ['']

    untested = flags['UNTESTED']
    L += ['## UNTESTED', '', f'FFF keys that no test sets: {len(untested)} of {len(fff_keys)}. Detection covers '
          'literal keys in `{"key", value}` pairs, `set*("key", ...)`, `opt*/option("key") = ...`, `cfg.key = ...`, '
          '.ini files and CLI flags under `tests/`; values built at run time are missed.', '']
    by_class = collections.defaultdict(list)
    for r in untested:
        by_class[', '.join(r['classes'])].append(r['key'])
    L += md_table(['class', 'count', 'keys'], [(c, len(ks), ' '.join(f'`{k}`' for k in ks)) for c, ks in sorted(by_class.items())]) + ['']
    uv = flags['UNTESTED_VALUE']
    L += ['### UNTESTED_VALUE', '', f'Enum and bool FFF keys with values no test selects: {len(uv)} keys, '
          f'{sum(len(r["never"]) for r in uv)} values. `implicit` is the default, exercised whenever the key is not set.', '']
    L += md_table(['key', 'never selected', 'implicit (default)', 'selected by a test'],
                  [(r['key'], ', '.join(r['never']), ', '.join(r['implicit']), ', '.join(r['selected'])) for r in uv]) + ['']

    di = flags['DEFAULT_INVALIDATION']
    L += ['## DEFAULT_INVALIDATION', '', 'FFF keys handled only by the catch-all `else` (informational). In the '
          'PrintObject function it resets every step (safe, slow); in the Print function it resets print steps only.', '']
    by_fn = collections.defaultdict(list)
    for r in di:
        by_fn[r['function']].append(r['key'])
    L += md_table(['function', 'count', 'keys'], [(fn, len(ks), ' '.join(f'`{k}`' for k in ks)) for fn, ks in sorted(by_fn.items())]) + ['']

    L += ['## Step model', '', 'Parsed from `Print.hpp` and `PrintObject::invalidate_step`:', '']
    L += md_table(['object step', 'invalidating it also resets'],
                  [(s, ', '.join(x for x in v if x != s)) for s, v in meta['cascade'].items()]) + ['']
    if meta['never_done_steps']:
        L += ['Steps whose entry function never calls `set_done` (re-run on every process, so treated as uncached): '
              + ', '.join(f'`{s}`' for s in meta['never_done_steps']), '']
    L += ['Names also declared as a variable or field in `src/` (their unqualified `.name` reads are low confidence): '
          + ', '.join(f'`{k}`' for k in meta['ambiguous_names']), '']
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(L) + '\n')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--root', default='.', help='OrcaSlicer source root')
    ap.add_argument('--out-dir', default='tests/settings_inventory', help='where to write the json and md')
    args = ap.parse_args()
    root = os.path.abspath(args.root)
    inventory, flags, meta = build_inventory(root)
    os.makedirs(args.out_dir, exist_ok=True)
    write_json(os.path.join(args.out_dir, 'settings_inventory.json'), inventory, flags, meta)
    write_md(os.path.join(args.out_dir, 'settings_inventory.md'), inventory, flags, meta)
    print(f'{meta["keys"]} keys ({meta["fff_keys"]} FFF); ' + ', '.join(f'{k}={len(v)}' for k, v in sorted(flags.items())))


if __name__ == '__main__':
    main()
