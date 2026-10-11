# Pattern-matched speedup suggestions (OrcaSlicer b5ef24e)

These are code-reading findings only. Nothing was built or profiled, so impact is estimated from loop counts and per-call cost. Line numbers are against upstream b5ef24e and may shift on the perf branch. They are ranked for large and multi-material models.

Output column: **identical** means a pure cache, hoist, early-out, or deterministic parallel change, which should stay byte-identical. **equivalent** means Clipper result order or rounding may change, so it needs the gcode_equiv checker.

Spot-checked against source: A1, A2, A4, A5, B1, B3, B4, B5, B14 (the code is as described).

## Tier A: process phase (Print::process)

**A1. TreeSupport `draw_circles` rescans every object layer below, for each component on each support layer.** `Support/TreeSupport.cpp:2419-2433`, inside the per-layer parallel_for at :2188.
- Each step calls `collect_region_slices_by_type(*below_layer, stTop)` and `(..., stBottom)` (:124), which copies polygons, then runs `intersection_ex` against the whole layer.
- That is O(support layers × components × object layers) Clipper calls, quadratic in height. It is active for the slim/strong/hybrid tree styles when there is a bottom gap or bottom interface and "on build plate only" is off. `union_ex(band.surfaces)` at :2482 adds more.
- **Fix:** precompute top∪bottom polygons and a bbox once per object layer, keep a list of only the non-empty layers, and skip any layer whose bbox misses `get_extents(comp)`.
- **Output:** identical (a disjoint bbox means an empty intersection). Also clipping the operands to the bbox is equivalent.
- Confidence high, impact high (tall models with tree supports).

**A2. Boolean ops do a full safety offset of the clip set even when the subject is empty.** `ClipperUtils.cpp:641-647` (`_clipper`), :724-730 (`_clipper_ex`), :489-496 (`_pl`). Main caller: `detect_surfaces_type` at `PrintObject.cpp:2027-2070`.
- `c2_clip_paths(clip, Yes)` offsets all of the neighbour `lslices` before anyone checks whether the subject is empty. Painted multi-material gives many regions (parent regions × colors), and most are empty on most layers. Each still pays about 2 full-layer offsets plus Clipper runs per layer.
- **Fix:**
  - (a) In `_clipper*`, return empty for intersection/difference when the subject is empty, and for intersection when the clip is empty. This helps every caller.
  - (b) In `detect_surfaces_type`, `continue` early when `layerm->slices` is empty, still clearing the output.
  - (c) For non-empty subjects, clip `lslices` to the subject bbox before the safety offset, as `diff_clipped` already does.
- **Output:** identical for (a) and (b); (c) is equivalent.
- Confidence high, impact high for multi-region objects.
- Related: `c2_offset_as_group` (`ClipperUtils.cpp:346-361`) runs an O(contours × holes) bbox containment test on every offset, including every safety offset. A sorted or indexed sweep would remove a quadratic term from almost every Boolean op on island-heavy layers. Output: identical.

**A3. `discover_vertical_shells` builds the whole shell before checking whether the region has any internal surface.** `PrintObject.cpp:2715-2848` (region loop :2621, per-layer parallel_for :2663).
- Repeated `union_`/`intersection` runs over all shell layers (:2716-2785). Only then is `polygonsInternal` computed (:2845), and when it is empty the code `continue`s.
- `object_volume = intersection(lslices[i-1], lslices[i+1])` (:2882-2886) is region-independent but is recomputed per region.
- **Fix:** compute `polygonsInternal` first and skip if it is empty. Cache `object_volume` once per layer, lazily.
- **Output:** identical. Confidence high, impact high for multi-material (this is the default evstAll path).

**A4. MM segmentation rebuilds a full `TriangleSelector` (face neighbours, normals, deserialize) per extruder × volume, including unpainted volumes and modifiers.**
- Where: `MultiMaterialSegmentation.cpp:2099-2107`. The `get_facets` call at :2104 runs before the `!is_model_part()` / empty check. The same rebuild happens per state at :1270-1274 (`get_facets_strict`, serial). The rebuild itself is in `Model.cpp:3678-3684` and :3722-3728.
- **Fix:** skip non-model-part and unpainted volumes before the call. Build one selector per volume and extract all states from it: an overload for that already exists at `Model.cpp:3687`; add a strict variant. Keep state 0 for the top/bottom path.
- **Output:** identical. Confidence high, impact high for painted objects with many filaments or volumes (roughly 2 × N filaments × V volumes O(F log F) neighbour builds per object today).

**A5. Normal supports: `top_contact_layers` runs `detect_contacts` and `fill_contact_layer` serially over all layers.** `Support/SupportMaterial.cpp:2404-2440`.
- The log message still says "in parallel", and upstream Prusa runs this loop under parallel_for.
- Inputs are read-only. `layer_storage.allocate` takes a spin mutex (`SupportLayer.hpp:130`). Outputs go to fixed slots `contact_out[layer_id*2(+1)]`.
- **Fix:** wrap the loop in `tbb::parallel_for`, keeping `remove_nulls` and the merge serial.
- **Output:** identical. Confidence high, impact high for tall models with normal supports.

**A6. `Print::process` runs the object phases (slice, perimeters, infill, ironing...) one object at a time.** `Print.cpp:3115-3197`.
- Each step is parallel over layers internally, but has serial sections and barriers: per-region parallel_for launches, the serial `restore_untyped_slices` (`PrintObject.cpp:512`, :616), and the serial `prepare_fill_surfaces` (:633). Supports already run across objects in parallel (:3196-3210).
- **Fix:** run each phase across objects with parallel_for or a task_group, then run the plugin hooks serially afterwards.
- **To verify:** `set_status`, `active_step_add_warning`, and the shared Print caches.
- **Output:** identical. Confidence medium, impact high on many-object plates.

**A7. Perimeters: lower-layer processing is recomputed for every compatible region group.**
- `LayerRegion.cpp:163-167` calls `printable_slices(lower lslices)` (an `opening_ex` of the whole layer). Then `PerimeterGenerator.cpp:1476-1481`/`2535-2540` compute `m_lower_slices_polygons`, and :1509-1515 call `generate_lower_polygons_series` up to 3 times with 2 offsets each.
- Groups split by `outer_wall_filament_id` (`Layer.cpp:159`), so every color is its own group. That is about 9 full-lower-layer offsets per color per layer.
- **Fix:** compute these once per layer (keyed by width/nozzle) in `Layer::make_perimeters` and pass them in.
- Also: `is_perimeter_compatible` calls `opt_serialize("inner/outer_wall_line_width")` per region pair per layer (`Layer.cpp:178-179`). Replace it with a per-object R×R compatibility matrix.
- **Output:** identical. Confidence high, impact medium-high for multi-color.

**A8. Volume slices are deep-copied for all layers, and volume grouping is O(V²·L).**
- `PrintObjectSlice.cpp:1374` copies every volume's ExPolygons for all layers into `firstLayerObjSliceByVolume`, despite the name. `findPartVolumes` copies them again (:668-676), and `groupingVolumes` takes the vector by value (:587), a third copy.
- Grouping then runs `douglas_peucker` and `offset_ex` on every layer (:593-612) and `doesVolumeIntersect` over all V² pairs (:557-631).
- `Print.cpp:2870` deep-copies the set again per shared object.
- **Fix:** pass by reference or move. Free the set after `groupingVolumesForBrim`, keeping only `slices.front()` for negative volumes. Pre-filter pairs by 3D bbox. Use union-find.
- **Output:** identical. Confidence high, impact medium (multi-part, many instances; also saves a lot of memory).

**A9. Fuzzy skin takes the slow path on every layer above the first and redoes region geometry for each loop.** `Feature/FuzzySkin/FuzzySkin.cpp:600-662` (Arachne: :734-792).
- `fuzzy_supported_area` is always set for layer > 0 (`LayerRegion.cpp:166`, `FuzzySkin.cpp:479-488`), so the fast path almost never fires.
- Each loop runs `union_ex`, a pairwise `diff_ex`, and a copy of `*supported`, then `split_line` against the whole-layer polygons.
- **Fix:** cache the merged/restricted regions per should_fuzzify key (only a few exist), use a pointer instead of copying `*supported`, and bbox-clip before `split_line`.
- **Output:** identical (the bbox clip is equivalent). Confidence high, impact medium (fuzzy skin users, every layer).

**A10. Brim diffs each object's areas against the whole plate's no-brim set.** `Brim.cpp:659-679`.
- `extruder_no_brim_area = no_brim_area` is copied per object, then `diff_ex` runs per instance key. This is separate from the already-fixed detached-piece filter.
- :593-595 appends areas into `brim_area`, which :682 then clears, so that append is dead work.
- **Fix:** bbox-clip `no_brim_area` to the object's areas instead of copying it, and delete the dead append.
- **Output:** equivalent (removing the dead append is identical). Confidence high, impact medium on many-object plates.

**A11. Organic tree: `TreeModelVolumes::calculateCollision` recomputes the same per-layer `union_ex(machine_border + outlines[j])` for every radius.** `Support/TreeModelVolumes.cpp:511-523`, :529-574.
- **Fix:** cache one union per layer.
- **Output:** identical. Confidence high, impact medium.

**A12. `bridge_over_infill` processes the whole lower layer before checking for any stInternalSolid surface.** `PrintObject.cpp:3015-3048`.
- **Fix:** check for internal-solid surfaces first and `continue` if there are none.
- **Output:** identical. Confidence high, impact medium on tall models.

**A13. MM segmentation, smaller items.**
- (a) The `EdgeGrid::create` loop at `MultiMaterialSegmentation.cpp:2082-2094` is serial but layer-independent. Make it parallel_for. Output: identical.
- (b) `merge_segmented_layers` at :1906-1915 runs up to N² sequential `diff_ex` calls per layer. Do one diff against the concatenated sets. Output: equivalent.
- (c) `apply_mm_segmentation` at `PrintObjectSlice.cpp:1178-1185` runs one `diff` per extruder. Do one diff. Output: equivalent.
- (d) The cumulative `layer_slices_trimmed` chain at :1441-1452 is recomputed per color. Cache it per layer. Output: identical.
- Impact: each is low-medium, but they add up for 8-16 color prints.

**A14. Fill grouping is quadratic in the number of fills per layer.**
- `Fill/Fill.cpp:1158-1167` diffs against an ever-growing `all_polygons` with a safety offset each time.
- :1476 intersects every expolygon against the whole no-overlap set.
- **Fix:** bbox-clip the clip set (`clip_clipper_polygons_with_subject_bbox`).
- **Output:** equivalent. Impact low-medium.

**A15. Minor (all identical).**
- `detect_surfaces_type` runs the BottomBridge conversion over all regions inside the per-region loop (`PrintObject.cpp:2396`), so it does R² parallel_for launches.
- `restore_untyped_slices` deep-copies serially (:512, :616); make it parallel_for.
- `find_and_insert_cluster` in TreeSupport (:860-872, :839-841) scans all clusters ever created and re-offsets each candidate. Keep a list of active clusters in index order.

## Tier B: G-code export (serial layer generator and post-filters)

**B1. Avoid-crossing-walls boundaries are rebuilt on every instance visit, and the external boundary is O(instances) per rebuild.**
- `GCode.cpp:7050-7051` (and :7284-7285) call `init_layer`, which clears `m_internal`/`m_external` and rebuilds `offset_ex(lslices)` plus an EdgeGrid (`AvoidCrossingPerimeters.cpp:1340-1360`). This happens per extruder × wipe pass × instance.
- `use_external_mp_once()` (:7088, :7174) then rebuilds `get_boundary_external()` (:1157-1213), which loops over all objects × instances, so the per-layer cost is O(instances²) × extruders.
- **Fix:** cache `m_lslices_offset`/grid and `m_internal` keyed by `const Layer*` (they are in object-local coordinates, so copies of an object share them), and `m_external` keyed by print_z.
- **Output:** caching `lslices_offset` is identical; reusing the boundaries is very likely identical, but the unstable sort at :1388 is a tie risk, so run the checker.
- Existing quirk: a boundary built while `m_layer` was a support layer can be reused later. Reproduce this or fix it on purpose.
- Confidence high, impact high on many-instance plates with "Avoid crossing walls" on.

**B2. Wipe tower moves are rewritten with 3 iostreams per G1 line.** `GCode.cpp:1888-1965`, `post_process_wipe_tower_moves`. It runs for every toolchange and tower layer.
- Each G1 line goes through an istringstream plus two ostringstreams. `line.find("G1 ")==0` scans the whole line, and `gcode_out += line + "\n"` builds a temporary.
- **Fix:** use string_view, fast_float/from_chars to parse, `snprintf("%.3f", (double)v)` to format, prefix compares, and reserve.
- **Output:** identical if done carefully (`fixed` with setprecision(3) on a float prints the same as %.3f of the promoted double). Confidence high, impact medium-high for MM (thousands of toolchanges).

**B3. The full region config (about 206 options) is re-applied 3-5 times per island, before the skip check.**
- `GCode.cpp:8281`: `extrude_perimeters` applies before `should_print` (:8284) and is called 3× per island (:7189/:7200/:7202, :7490-7500).
- :8337: `extrude_infill` is called 2× per island. :7046-7047 apply per instance visit.
- **Fix:** test `is_infill_first` from the region config before applying. Track the last-applied PrintRegion* and skip a repeat apply, resetting it at the other apply sites (:7046, :5499, :5681, :6060, :8639).
- **Output:** identical. Confidence medium (needs a profile), impact medium-high on layers with many islands or instances.

**B4. `extrude_infill` deep-copies every infill collection per instance per layer.** `GCode.cpp:8347`, `ExtrusionEntityCollection(*eec)`. This happens even when `no_sort`.
- **Fix:** iterate directly when `no_sort`. Otherwise chain child pointers through the existing const overload `chain_and_reorder_extrusion_entities(..., reversed_clones)` (`ShortestPath.cpp:1098`).
- **Output:** identical. Confidence high, impact medium.

**B5. `SeamPlacer::place_seam` captures the whole `ExtrusionLoop` by value in a lambda.** `GCode/SeamPlacer.cpp:1713`, `[loop]`.
- **Fix:** capture `[&loop]`, a one-character change.
- **Output:** identical. Confidence high, impact medium (one full perimeter copy per loop per instance per layer).

**B6. The G-code processor stage copies each layer string, and the stage could be split.** `GCode.cpp:8455-8463`.
- `write` does `strlen`, then `process_buffer(std::string(gcode))`. The `write(const std::string&)` overload (`GCode.hpp:409`) goes through `c_str()`.
- **Fix:** add a string overload using `size()` with no copy (trivial).
- **Bigger option:** tokenizing in `GCodeReader::parse_line_internal` is stateless, so a parallel pipeline filter could pre-parse each layer, leaving only `process_gcode_line` and the time estimator serial. A separate fwrite filter would overlap disk I/O.
- **Output:** identical. Confidence medium, impact medium (this targets the remaining serial bottleneck).

**B7. Island assignment is O(collections × islands) and is repeated per extruder.** `GCode.cpp:6628-6640`.
- **Fix:** hoist the lookup out of the extruder loop, and index `lslices_bboxes` with a grid, picking the lowest `slices_test_order` rank among hits.
- **Output:** identical. Impact medium-low in general, high on island-heavy layers.

**B8. Island centroids are recomputed per filament × instance.** `GCode.cpp:6757` (and :7150).
- **Fix:** compute them once per object layer and add the instance shift.
- **Output:** identical. Impact low-medium.

**B9. `compute_farthest_point` (farthest-point timelapse) scans all points once per instance.** `GCode.cpp:5745-5895`.
- **Fix:** check only the per-layer hull or corner points.
- **Output:** identical with the same tie order. Only matters when that option is on.

**B10. `CoolingBuffer::parse_layer_gcode` allocates per line.** `GCode/CoolingBuffer.cpp`.
- :395 allocates a `std::string` per line. :413 allocates a `vector<float> new_pos` per G1. :442-460 run 4 `boost::contains` full-line scans, two of them identical.
- **Fix:** use string_view, a `std::array`, and a single search for the comment.
- **Output:** identical. Impact low-medium; it matters once the generator is no longer the bottleneck.

**B11. ConflictChecker (runs in Print::process, `Print.cpp:3640`) copies paths repeatedly and builds lines serially.**
- `ConflictChecker.hpp:74` copies `_piles(paths)` instead of moving. `ConflictChecker.cpp:274-275` copies all paths per instance. :289-294 run `getCurLines()` serially, and the grid is a `std::map`.
- **Fix:** move instead of copy, store paths per object and apply the instance shift on the fly, build lines inside the parallel_for, and use a hash map.
- **Output:** same conflict verdict. Impact medium on large plates.

**B12. Smaller export items (all identical).**
- The pre-heating pass (`GCodeProcessor.cpp:1680/1687/1699`) builds `std::string(";")+Tag` per G1. :1596/:1613 compile a `std::regex` per nozzle change. Make them static.
- `TimelapsePosPicker.cpp:500-534` re-offsets and unions every instance rectangle each layer. Cache them.
- `ToolOrdering.cpp:3440`: the sort comparator captures `object_list` by value and never uses it. Use `[]`. `total_volume()` is computed twice (:3497-3511).
- `calc_max_volumetric_speed` (`GCode.cpp:8523`) re-parses a coefficient string with stringstream per path. Parse once per filament.
- `AdaptivePAProcessor.cpp:82-124` copies each layer into an istringstream and runs a regex per PA line.

## Suggested order for the optimizer

1. Cheap identical wins to batch first: B5, A2(a/b), A12, A3, A4, B4, B12 items, A15.
2. Larger identical wins that need a benchmark: B1 (many-instance plates, avoid-crossing on), B3, A5 (normal supports), A1 (tree supports), A7, A6, B2 (MM toolchanges).
3. Checker-rated (equivalent) work: A2(c), A13(b,c), A10, A14.

Benchmark coverage: 12-Benchy plate for B1/B3/B7/A6; Super+sonic and Pad+Stand for A4/A7/A13/B2; 3dragon for A2/A3/A4; Pad+Stand (tree supports) for A1. A normal-support model is needed for A5.
