# Additional benchmark models for OrcaSlicer slicing performance

Written 2026-10-11. Target printer: Snapmaker U1 (4 toolheads, about 270 mm build cube).

The current set (see final-report.md) is 3dragon (painted, 890 layers), the 12-Benchy plate, Pad+Stand (2 filaments, tree supports), and Super+sonic (4 filaments, 1,107 tool changes). Together they cover painted MMU segmentation, TSP travel ordering, a mid-size multi-object plate and frequent tool changes. They do not cover:

- meshes with millions of triangles
- tree supports on a tall, organic model
- a full-bed plate of 50+ different parts
- large flat top/bottom areas (solid infill, ironing, long perimeters)
- dense, fine-grained painting across 4 filaments
- adaptive layer height, fuzzy skin, arc fitting and spiral vase
- broken meshes that go through the repair paths

## How these were checked

License terms and file listings were read from each source page on 2026-10-11. This cloud container's network blocks graphics.stanford.edu, GitHub archives and Hugging Face, so **none of the download URLs below were test-downloaded**. The first fetch on the perf box confirms them.

Orca does not import PLY. Convert Stanford files to STL first, for example `python3 -m pip install trimesh` and then `trimesh.load(p).export(out.stl)`. Scale every model to fit the U1 bed.

## Recommended models

### 1. Stanford Armadillo: tree supports on an organic model
- URL: http://graphics.stanford.edu/pub/3Dscanrep/armadillo/Armadillo.ply.gz (4 MB, 345,944 triangles)
- License: the Stanford page says "You can do anything you want to the Stanford bunny or the armadillo."
- Why: raised arms, ears and claws give many overhang islands. Scale to about 200 mm tall and enable tree supports (organic and hybrid). This covers TreeSupport `draw_circles` (pattern-suggestions A1), which grows with height, much more than the 9 s Bunny case does.
- Variants: the same model with fuzzy skin on outer walls, and with adaptive layer height. Adaptive layer height is not a CLI option, so apply it once in the GUI and save the 3MF; the profile is stored in the 3MF.

### 2. Armadillo with synthetic 4-filament painting: heavy MMU segmentation
- Source: model 1, painted by a script.
- Why: 3dragon is the only painted case. A model with thousands of small painted patches across 4 filaments drives MMU segmentation, per-region surface detection (pattern-suggestions A2), wipe tower and tool-change export much harder.
- How: write the 3MF with a per-triangle `paint_color` attribute, assigning filaments by low-frequency 3D noise over triangle centroids so patches are a few mm across. Before trusting the generator, paint one triangle per filament in the Orca GUI, save the file and copy its encoding. Then confirm that the sliced G-code uses all 4 tools.
- License: same as model 1.

### 3. XYZ RGB Asian Dragon: very large mesh
- URL: https://graphics.stanford.edu/data/3Dscanrep/xyzrgb/xyzrgb_dragon.ply.gz (69 MB compressed, 7.2M triangles)
- License: free for research, redistribution allowed, **no commercial use**, credit Stanford. The page asks that it not be animated, morphed or put through Boolean operations. Slicing it for a benchmark doesn't do any of those, but don't publish derived meshes.
- Why: 20× more triangles than any current case. This stresses 3MF/STL load, mesh repair, `TriangleMeshSlicer` and per-layer polygon size (very dense contours fed to Clipper, Arachne and simplification). It is a good fit for adaptive layer height too.

### 4. Stanford Lucy: extreme mesh size, tall with thin features
- URL: https://graphics.stanford.edu/data/3Dscanrep/lucy.tar.gz (307 MB compressed, 28M triangles)
- License: same terms as model 3.
- Why: a ceiling test for memory and mesh-to-slice time. Thin wings and drapery need supports at 200+ mm height. Optional if model 3 already shows the mesh-size costs.

### 5. Voron 2.4 printed parts: full plate of many different parts
- URL: https://github.com/VoronDesign/Voron-2/archive/refs/heads/Voron2.4.zip (155 STLs under `STLs/`)
- License: GPL-3.0.
- Why: one bed filled with 50 to 80 distinct functional parts, with holes, bosses, text and bridges. This differs from 12 identical Benchys, because per-object work can't share results. It exercises per-object `Print::process`, the seam placer and brim per object, and travel ordering across many islands.
- Variant: assign parts round-robin to the 4 filaments, so tool changes happen between objects instead of within painted regions.

### 6. Gridfinity 6×6 baseplate: large flat plate
- Source: https://github.com/kennetek/gridfinity-rebuilt-openscad (MIT). Generate it with `openscad -D gridx=6 -D gridy=6 -o baseplate.stl gridfinity-rebuilt-baseplate.scad`. Check the parameter names against the docs at https://kennetek.github.io/gridfinity-rebuilt-openscad/.
- Why: about 252 × 252 mm and mostly top, bottom and solid infill, with long perimeters around 36 cells. It covers solid-infill generation, monotonic ordering, ironing (enable it) and gap fill on large areas. None of the current models is flat or wide.
- Needs OpenSCAD on the perf box. If OpenSCAD isn't available, any MIT or CC0 full-bed flat part will do.

### 7. Thingi10K subset: varied and broken meshes
- URL: https://huggingface.co/datasets/Thingi10K/Thingi10K/resolve/main/Thingi10K.tar.gz, or `python3 -m pip install thingi10k`, which can filter by license, component count and closed or manifold status.
- License: per model. The dataset uses 10 open licenses, so filter to CC0, CC BY and public domain.
- Why: about 45% of the meshes self-intersect and 22% are non-manifold. That exercises repair and slicing edge cases the clean test models never reach. Use it two ways: (a) a correctness sweep of about 50 models through gcode_equiv, which catches regressions the 36 current cases miss, and (b) a many-small-parts plate built from 30 or more closed multi-component models.

### 8. Smithsonian Woolly Mammoth: tall organic model with tree supports (optional)
- Page: https://3d.si.edu/object/3d/mammuthus-primigenius-blumbach:341c96cd-f967-4540-8ed1-d3fc56d31f12
- License: CC0.
- Why: long curved tusks and a trunk give tall, unsupported overhangs. It is a second tree-support case with different geometry from the Armadillo.
- Catch: the download is behind the page's JavaScript Download panel, so there is no stable direct URL. Fetch it once by hand. Use it only if the Armadillo doesn't show tree-support hot spots well.

## Setting variants on existing models

These code paths are switched on by settings, not by geometry, so test them on models already in the set:
- **Arc fitting** (`enable_arc_fitting`) on 3dragon and model 5: this is extra export work.
- **Spiral vase** on a tall single-wall model, such as a scaled vase from Thingi10K.
- **Fuzzy skin** and **ironing** on the 12-Benchy plate, which has a known baseline.

## Suggested priority

Start with 1, 2, 5 and 6. They fill the biggest gaps (tree supports, heavy painting, many-part plates, flat plates) at moderate slice times. Then add 3 and 7. Treat 4 and 8 as optional.
