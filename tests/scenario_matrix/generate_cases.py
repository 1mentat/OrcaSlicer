#!/usr/bin/env python3
"""Generate scenario matrix cases for tests/fff_print/test_scenario_matrix.cpp.

Each case combines model features with config overrides. Instead of every combination, the cases
form a covering array: every pair of values of any two axes appears in at least one case, and for
each family of axes that interact, every triple of values does too.

    python3 generate_cases.py --out cases.jsonl            # pairwise over all axes + 3-way families
    python3 generate_cases.py --strength 2 --no-families   # pairwise only
    python3 generate_cases.py --list-axes

The output is deterministic for a given --seed.
"""

import argparse
import itertools
import json
import random
import sys

# Model features, interpreted by build_model() in test_scenario_matrix.cpp.
MODEL_AXES = {
    "mesh": ["cube_with_hole", "overhang", "sloping_hole", "ipadstand", "two_hollow_squares", "bridge", "pyramid", "V"],
    "objects": ["one", "two_separate", "two_copied", "two_different"],
    "instances": [1, 3],
    "modifier": ["none", "dense"],
    "negative_volume": ["none", "through"],
    "support_volume": ["none", "enforcer", "blocker"],
    "height_range": ["none", "dense", "fine_layers"],
    "material": ["none", "dense"],
    "object_settings": ["none", "dense"],
    "paint_mm": ["none", "top", "bands"],
    "paint_seam": ["none", "enforce_side", "block_side"],
    "paint_support": ["none", "enforce_down"],
    "filaments": [1, 2, 4],
    "object_extruders": ["same", "alternate"],
}

# Config overrides: key -> values, as the strings set_deserialize_strict() accepts.
CONFIG_AXES = {
    "wall_generator": ["classic", "arachne"],
    "wall_loops": ["1", "2", "4"],
    "wall_sequence": ["inner wall/outer wall", "outer wall/inner wall", "inner-outer-inner wall"],
    "only_one_wall_top": ["0", "1"],
    "precise_outer_wall": ["0", "1"],
    "seam_position": ["nearest", "aligned", "aligned_back", "back", "random"],
    "staggered_inner_seams": ["0", "1"],
    "seam_slope_type": ["none", "external", "all"],
    "top_shell_layers": ["0", "1", "4"],
    "bottom_shell_layers": ["0", "1", "3"],
    "top_shell_thickness": ["0", "1"],
    "ensure_vertical_shell_thickness": ["none", "ensure_critical_only", "ensure_moderate", "ensure_all"],
    "interface_shells": ["0", "1"],
    "sparse_infill_density": ["0%", "15%", "100%"],
    "sparse_infill_pattern": ["grid", "gyroid", "concentric", "zigzag", "lightning", "adaptivecubic"],
    "top_surface_pattern": ["monotonicline", "concentric", "rectilinear"],
    "gap_fill_target": ["everywhere", "topbottom", "nowhere"],
    "ironing_type": ["no ironing", "top", "solid"],
    "fuzzy_skin": ["none", "external", "all"],
    "dont_filter_internal_bridges": ["disabled", "limited", "nofilter"],
    "extra_perimeters_on_overhangs": ["0", "1"],
    "overhang_reverse": ["0", "1"],
    "enable_support": ["0", "1"],
    "support_type": ["normal(auto)", "tree(auto)"],
    "support_style": ["default", "snug", "organic", "tree_hybrid"],
    "support_on_build_plate_only": ["0", "1"],
    "raft_layers": ["0", "2"],
    "brim_type": ["no_brim", "outer_only", "outer_and_inner", "auto_brim", "brim_ears"],
    "brim_width": ["0", "5"],
    "skirt_loops": ["0", "2"],
    "draft_shield": ["disabled", "enabled"],
    "print_sequence": ["by layer", "by object"],
    "print_order": ["default", "as_obj_list"],
    "z_hop": ["0", "0.4"],
    "z_hop_types": ["Auto Lift", "Normal Lift", "Slope Lift", "Spiral Lift"],
    "retraction_minimum_travel": ["0", "2"],
    "reduce_infill_retraction": ["0", "1"],
    "wipe": ["0", "1"],
    "enable_prime_tower": ["0", "1"],
    "wipe_tower_rotation_angle": ["0", "30"],
    "ooze_prevention": ["0", "1"],
    "enable_arc_fitting": ["0", "1"],
    "spiral_mode": ["0", "1"],
    "layer_height": ["0.1", "0.2", "0.3"],
    "xy_hole_compensation": ["0", "0.1"],
    "elefant_foot_compensation": ["0", "0.15"],
    "slice_closing_radius": ["0", "0.049"],
    "gcode_flavor": ["marlin2", "klipper"],
}

# Axes that interact; each gets a strength-3 covering array on top of the pairwise one.
FAMILIES = {
    "walls_seams_painting": ["wall_generator", "seam_position", "paint_seam", "wall_sequence", "staggered_inner_seams", "objects"],
    "regions_shells_infill": ["modifier", "height_range", "material", "object_settings", "objects", "top_shell_layers",
                              "ensure_vertical_shell_thickness", "sparse_infill_density"],
    "supports": ["enable_support", "support_type", "support_style", "support_volume", "paint_support", "raft_layers", "mesh"],
    "multi_material": ["filaments", "paint_mm", "object_extruders", "enable_prime_tower", "ooze_prevention", "top_shell_layers"],
    "travel": ["z_hop_types", "z_hop", "retraction_minimum_travel", "reduce_infill_retraction", "wipe", "print_sequence"],
    "sharing": ["objects", "height_range", "material", "modifier", "paint_mm", "instances"],
}


def axes():
    out = dict(MODEL_AXES)
    out.update({"config." + k: v for k, v in CONFIG_AXES.items()})
    return out


def qualified(name):
    return name if name in MODEL_AXES else "config." + name


def valid(case):
    """Rules a case must satisfy. Combinations the slicer itself rejects are reported by the engine."""
    get = case.get
    if get("filaments") == 1 and (get("paint_mm") not in (None, "none") or get("object_extruders") == "alternate"):
        return False
    if get("config.enable_support") == "0" and get("config.support_style") not in (None, "default"):
        return False
    if get("config.support_type") == "normal(auto)" and get("config.support_style") in ("organic", "tree_hybrid"):
        return False
    if get("config.spiral_mode") == "1" and (get("objects") not in (None, "one") or get("instances") not in (None, 1)):
        return False
    if get("config.z_hop") == "0" and get("config.z_hop_types") not in (None, "Normal Lift"):
        return False
    return True


def required_tuples(all_axes, strength, families):
    names = sorted(all_axes)
    groups = [(combo, strength) for combo in itertools.combinations(names, strength)]
    for family in families.values():
        members = sorted(qualified(n) for n in family)
        groups += [(combo, 3) for combo in itertools.combinations(members, 3)]
    tuples = set()
    for combo, _ in groups:
        for values in itertools.product(*(all_axes[a] for a in combo)):
            partial = dict(zip(combo, values))
            if valid(partial):
                tuples.add(tuple(sorted((a, json.dumps(v)) for a, v in partial.items())))
    return tuples


def covered_by(case, tuples_by_axes):
    hits = set()
    for combo, tuples in tuples_by_axes.items():
        key = tuple(sorted((a, json.dumps(case[a])) for a in combo))
        if key in tuples:
            hits.add(key)
    return hits


def generate(all_axes, strength, families, seed, candidates=10):
    rng = random.Random(seed)
    uncovered = required_tuples(all_axes, strength, families)
    tuples_by_axes = {}
    for t in uncovered:
        tuples_by_axes.setdefault(tuple(a for a, _ in t), set()).add(t)
    combos_with = {a: [c for c in tuples_by_axes if a in c] for a in all_axes}
    names = sorted(all_axes)
    cases = []
    while uncovered:
        best, best_gain = None, -1
        target = rng.choice(sorted(uncovered))
        for _ in range(candidates):
            case = {a: json.loads(v) for a, v in target}
            for a in rng.sample(names, len(names)):
                if a in case:
                    continue
                # Pick the value that covers the most uncovered tuples together with what is set so far.
                options = all_axes[a][:]
                rng.shuffle(options)
                scored = []
                for v in options:
                    trial = dict(case, **{a: v})
                    if not valid(trial):
                        continue
                    gain = sum(1 for combo in combos_with[a]
                               if all(x in trial for x in combo)
                               and tuple(sorted((x, json.dumps(trial[x])) for x in combo)) in uncovered)
                    scored.append((gain, v))
                if not scored:
                    break
                case[a] = max(scored, key=lambda s: s[0])[1]
            if len(case) != len(names) or not valid(case):
                continue
            gain = len(covered_by(case, tuples_by_axes) & uncovered)
            if gain > best_gain:
                best, best_gain = case, gain
        if best is None:
            # The target can't be completed into a valid case; drop it.
            uncovered.discard(target)
            continue
        uncovered -= covered_by(best, tuples_by_axes)
        cases.append(best)
    return cases


def to_engine(case, index, prefix):
    out = {"id": f"{prefix}{index:04d}", "config": {}}
    for a, v in sorted(case.items()):
        if a.startswith("config."):
            out["config"][a[len("config."):]] = v
        else:
            out[a] = v
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", default="-")
    parser.add_argument("--strength", type=int, default=2)
    parser.add_argument("--no-families", action="store_true")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--prefix", default="m")
    parser.add_argument("--list-axes", action="store_true")
    args = parser.parse_args()
    all_axes = axes()
    if args.list_axes:
        for name, values in sorted(all_axes.items()):
            print(f"{name}: {values}")
        return
    cases = generate(all_axes, args.strength, {} if args.no_families else FAMILIES, args.seed)
    stream = sys.stdout if args.out == "-" else open(args.out, "w")
    for i, case in enumerate(cases):
        stream.write(json.dumps(to_engine(case, i, args.prefix)) + "\n")
    print(f"{len(cases)} cases", file=sys.stderr)


if __name__ == "__main__":
    main()
