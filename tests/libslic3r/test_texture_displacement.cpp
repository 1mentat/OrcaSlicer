#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <admesh/stl.h>
#include <memory>
#include <vector>
#include <cstdint>
#include <cstddef>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <catch2/catch_test_macros.hpp>
#include <ios>
#include <iterator>
#include <utility>
#include "libslic3r/Point.hpp"
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <map>
#include <math.h>
#include "libslic3r/TextureBake/TextureBakeDisplace.hpp"
#include "libslic3r/TextureBake/TextureBakeIndex.hpp"
#include <tuple>
#include <Eigen/Geometry>
#include "libslic3r/libslic3r.h"
#include <catch2/catch_message.hpp>
#include <catch2/catch_all.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <set>
#include <boost/filesystem.hpp>

#include "libslic3r/AABBMesh.hpp"
#include "libslic3r/TextureDisplacement.hpp"
#include "libslic3r/TextureBake/TextureBakeDecimate.hpp"
#include "libslic3r/TextureBake/TextureBakeFlip.hpp"
#include "libslic3r/TextureBake/TextureBakeMesh.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/TriangleSelector.hpp"
#include "libslic3r/PNGReadWrite.hpp"
#include "test_utils.hpp"

using namespace Slic3r;
using Catch::Matchers::WithinAbs;

// Encode literal texels through the PNG format accepted by the displacement layer.
static std::shared_ptr<std::vector<unsigned char>> make_gray_png(size_t w, size_t h,
                                                               const std::vector<uint8_t> &pixels)
{
    REQUIRE(pixels.size() == w * h);
    ScopedTemporaryFile file(".png");
    REQUIRE(Slic3r::png::write_gray_to_file(file.string(), w, h, pixels));

    std::vector<unsigned char> bytes;
    {
        std::ifstream ifs(file.string(), std::ios::binary);
        REQUIRE(ifs.good());
        bytes.assign(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>());
    }
    REQUIRE_FALSE(bytes.empty());
    return std::make_shared<std::vector<unsigned char>>(std::move(bytes));
}

// A flat map gives every painted vertex the same height sample.
static std::shared_ptr<std::vector<unsigned char>> make_flat_gray_png(uint8_t value, size_t w = 4, size_t h = 4)
{
    return make_gray_png(w, h, std::vector<uint8_t>(w * h, value));
}

// A hard-edged black/white checkerboard, the worst case for a height map: every texel boundary is a
// step, which is precisely the relief the post-process smoothing exists to round off.
static std::shared_ptr<std::vector<unsigned char>> make_checkerboard_png(size_t w = 16, size_t h = 16)
{
    std::vector<uint8_t> pixels(w * h);
    for (size_t y = 0; y < h; ++y)
        for (size_t x = 0; x < w; ++x)
            pixels[y * w + x] = ((x / 2 + y / 2) % 2) ? 255 : 0;
    return make_gray_png(w, h, pixels);
}

// The bake never drives relief below the model's own resting plane (see build_texture_displacement()),
// so on a fully painted closed solid the vertices already sitting on that plane - a cube's four bottom
// corners, whose normals point downwards - are clamped in Z and do not move by the full depth. The
// tests below are about the displacement maths, so they check the vertices the clamp cannot touch;
// the clamp itself has its own test.

// The tests below check the classic, topology-preserving bake vertex by vertex, so they select it
// explicitly: the default pipeline rebuilds the topology and has no vertex correspondence to check.
static TextureDisplacementOptions classic_options()
{
    TextureDisplacementOptions o;
    o.pipeline_v2 = false;
    return o;
}

static bool above_resting_plane(const indexed_triangle_set &mesh, size_t vi)
{
    float bottom = std::numeric_limits<float>::max();
    for (const Vec3f &v : mesh.vertices)
        bottom = std::min(bottom, v.z());
    return mesh.vertices[vi].z() > bottom + 1e-4f;
}

TEST_CASE("TextureDisplacement: decode_height_texture round-trips an 8-bit grayscale PNG", "[TextureDisplacement]")
{
    TextureDisplacementLayer layer;
    layer.image_data = make_flat_gray_png(128, 4, 4);

    DecodedHeightTexture tex = decode_height_texture(layer);
    REQUIRE_FALSE(tex.empty());
    CHECK(tex.width == 4);
    CHECK(tex.height == 4);
    REQUIRE_THAT(tex.sample(Vec2f(0.5f, 0.5f)), WithinAbs(128.0 / 255.0, 1.0 / 255.0));
}

TEST_CASE("TextureDisplacement: an empty layer list leaves the mesh unchanged", "[TextureDisplacement]")
{
    const indexed_triangle_set cube = its_make_cube(10., 10., 10.);
    const std::vector<TextureDisplacementLayer> layers; // none
    TextureDisplacementFacetsData facets{};              // all empty

    const indexed_triangle_set result = build_texture_displacement(cube, layers, facets, classic_options());

    REQUIRE(result.vertices.size() == cube.vertices.size());
    REQUIRE(result.indices.size() == cube.indices.size());
    for (size_t i = 0; i < cube.vertices.size(); ++i)
        for (int c = 0; c < 3; ++c)
            CHECK(result.vertices[i](c) == cube.vertices[i](c));
}

TEST_CASE("TextureDisplacement: fully painting a mesh displaces every vertex along its own normal", "[TextureDisplacement]")
{
    const indexed_triangle_set cube = its_make_cube(10., 10., 10.);
    const TriangleMesh cube_mesh(cube);

    TriangleSelector selector(cube_mesh);
    for (int f = 0; f < int(cube.indices.size()); ++f)
        selector.set_facet(f, EnforcerBlockerType::ENFORCER);

    TextureDisplacementFacetsData facets{};
    facets[0] = selector.serialize();

    TextureDisplacementLayer layer;
    layer.slot        = 0;
    layer.depth_mm    = 2.0f;
    layer.tiling_scale = 5.0f;
    layer.image_data  = make_flat_gray_png(255); // sample() == 1.0 everywhere -> full depth_mm displacement

    const indexed_triangle_set result = build_texture_displacement(cube, {layer}, facets, classic_options());

    REQUIRE(result.vertices.size() == cube.vertices.size());
    for (size_t i = 0; i < cube.vertices.size(); ++i) {
        if (!above_resting_plane(cube, i))
            continue;
        const float moved = (result.vertices[i] - cube.vertices[i]).norm();
        CHECK_THAT(moved, WithinAbs(layer.depth_mm, 1e-3f));
    }
}

// Paints every facet of `mesh` into a serialized mask, the way "Select whole model" does.
static TriangleSelector::TriangleSplittingData paint_whole_mesh(const indexed_triangle_set &mesh)
{
    const TriangleMesh tm(mesh);
    TriangleSelector   selector(tm);
    for (int f = 0; f < int(mesh.indices.size()); ++f)
        selector.set_facet(f, EnforcerBlockerType::ENFORCER);
    return selector.serialize();
}

static indexed_triangle_set make_texture_sheet(float x0, float x1, float y0, float y1)
{
    indexed_triangle_set sheet;
    sheet.vertices = { { x0, y0, 0.f }, { x1, y0, 0.f }, { x1, y1, 0.f }, { x0, y1, 0.f } };
    sheet.indices  = { { 0, 1, 2 }, { 0, 2, 3 } };
    return sheet;
}

// Paints a disc of the top face of `mesh` (the plane z = `top`) the way the gizmo's brush does: the
// selector splits the triangles under it, so the stroke need not follow the mesh.
static TriangleSelector::TriangleSplittingData paint_top_disc(const indexed_triangle_set &mesh, float top,
                                                              const Vec2f &centre, float radius)
{
    const TriangleMesh tm(mesh);
    const Vec3f        at(centre.x(), centre.y(), top);
    int                start = -1;
    Vec3d              foot;
    AABBMesh(tm).squared_distance(at.cast<double>(), start, foot); // the brush starts on the face under it
    TriangleSelector selector(tm);
    selector.select_patch(start,
                          TriangleSelector::SinglePointCursor::cursor_factory(
                              at, at + Vec3f(0.f, 0.f, 80.f), radius, TriangleSelector::CursorType::SPHERE,
                              Transform3d::Identity(), TriangleSelector::ClippingPlane()),
                          EnforcerBlockerType::ENFORCER, Transform3d::Identity(), /* triangle_splitting */ true);
    return selector.serialize();
}

TEST_CASE("A projective view texture leaves vertices behind its projector flat", "[TextureDisplacement][TextureDisplacementView]")
{
    const indexed_triangle_set sheet = make_texture_sheet(-1.f, 1.f, -1.f, 1.f);
    TextureDisplacementFacetsData facets{};
    facets[0] = paint_whole_mesh(sheet);

    TextureDisplacementLayer layer;
    layer.slot                    = 0;
    layer.image_data              = make_flat_gray_png(255);
    layer.depth_mm                = 0.5f;
    layer.tile_enabled            = false;
    layer.projection_method       = TextureProjectionMethod::ViewProjected;
    layer.view_project_projective = true;
    // u/v cover [0.25, 0.75] on the sheet; w is the signed distance from the projector plane.
    layer.view_project_matrix = { 0.25f, 0.f, 0.f, 0.5f,
                                  0.f, 0.25f, 0.f, 0.5f,
                                  1.f, 0.f, 0.f, 0.f };

    const indexed_triangle_set out = build_texture_displacement(sheet, { layer }, facets, classic_options());
    REQUIRE(out.vertices.size() == sheet.vertices.size());
    REQUIRE(out.indices == sheet.indices);

    for (size_t i = 0; i < sheet.vertices.size(); ++i) {
        const float expected_z = sheet.vertices[i].x() > 0.f ? 0.5f : 0.f;
        CHECK_THAT(out.vertices[i].z(), WithinAbs(expected_z, 1e-4f));
        CHECK_THAT(out.vertices[i].x(), WithinAbs(sheet.vertices[i].x(), 1e-6f));
        CHECK_THAT(out.vertices[i].y(), WithinAbs(sheet.vertices[i].y(), 1e-6f));
    }

    // The rejected branch must leave a caller's UV untouched, rather than returning a mirrored
    // coordinate for a point behind the projector.
    Vec2f rejected_uv(7.f, -3.f);
    CHECK_FALSE(project_uv_projective(layer.view_project_matrix, Vec3f(-1.f, 0.f, 0.f), rejected_uv));
    CHECK_THAT(rejected_uv.x(), WithinAbs(7.f, 1e-6f));
    CHECK_THAT(rejected_uv.y(), WithinAbs(-3.f, 1e-6f));
}

TEST_CASE("A non-tiled projective view uses its UV window for displacement", "[TextureDisplacement][TextureDisplacementView]")
{
    const indexed_triangle_set sheet = make_texture_sheet(-1.f, 1.f, -1.f, 1.f);
    TextureDisplacementFacetsData facets{};
    facets[0] = paint_whole_mesh(sheet);

    TextureDisplacementLayer layer;
    layer.slot                    = 0;
    layer.image_data              = make_gray_png(2, 2, { 0, 255, 255, 0 });
    layer.depth_mm                = 0.4f;
    layer.tile_enabled            = false;
    layer.projection_method       = TextureProjectionMethod::ViewProjected;
    layer.view_project_projective = true;
    // The four sheet corners land at (u,v) = (.125/.375, .125/.375). Projective placement already is
    // normalized UV, so these non-default transform values must not affect the sample.
    layer.tiling_scale = 0.125f;
    layer.rotation_deg = 37.f;
    layer.offset       = Vec2f(0.31f, -0.27f);
    layer.view_project_matrix = { 0.125f, 0.f, 0.f, 0.25f,
                                  0.f, 0.125f, 0.f, 0.25f,
                                  0.f, 0.f, 0.f, 1.f };

    const indexed_triangle_set inside = build_texture_displacement(sheet, { layer }, facets, classic_options());
    REQUIRE(inside.vertices.size() == sheet.vertices.size());
    const std::array<float, 4> expected_z = { 0.15f, 0.25f, 0.15f, 0.25f };
    for (size_t i = 0; i < expected_z.size(); ++i)
        CHECK_THAT(inside.vertices[i].z(), WithinAbs(expected_z[i], 2e-3f));

    // Shifting the projector window completely outside [0,1) is the negative control: with tiling
    // disabled, no border texel may be smeared across the rest of the mesh.
    layer.view_project_matrix[3] = 1.25f;
    const indexed_triangle_set outside = build_texture_displacement(sheet, { layer }, facets, classic_options());
    REQUIRE(outside.vertices.size() == sheet.vertices.size());
    for (const Vec3f &v : outside.vertices)
        CHECK_THAT(v.z(), WithinAbs(0.f, 1e-6f));
}

TEST_CASE("Mirrored texture tiling reflects the second tile before displacement", "[TextureDisplacement][TextureDisplacementView]")
{
    const indexed_triangle_set sheet = make_texture_sheet(7.5f, 12.5f, 0.f, 2.f);
    TextureDisplacementFacetsData facets{};
    facets[0] = paint_whole_mesh(sheet);

    TextureDisplacementLayer layer;
    layer.slot         = 0;
    layer.image_data   = make_gray_png(3, 1, { 0, 255, 64 });
    layer.depth_mm     = 1.f;
    layer.tiling_scale = 10.f;
    layer.tile_enabled = true;
    layer.projection_method = TextureProjectionMethod::Triplanar;

    layer.tile_method = TextureTileMethod::MirroredRepeat;
    const indexed_triangle_set mirrored = build_texture_displacement(sheet, { layer }, facets, classic_options());
    REQUIRE(mirrored.vertices.size() == sheet.vertices.size());
    const float mirrored_height = 48.f / 255.f; // u=.75 and mirrored u=1.25 -> .75, from texels {0,255,64}
    CHECK_THAT(mirrored.vertices[0].z(), WithinAbs(mirrored_height, 1e-4f));
    CHECK_THAT(mirrored.vertices[1].z(), WithinAbs(mirrored_height, 1e-4f));

    // Repeat is the control: the second point wraps to u=.25 and therefore samples the rising edge
    // of the asymmetric map instead of the reflected falling edge.
    layer.tile_method = TextureTileMethod::Repeat;
    const indexed_triangle_set repeated = build_texture_displacement(sheet, { layer }, facets, classic_options());
    REQUIRE(repeated.vertices.size() == sheet.vertices.size());
    CHECK_THAT(repeated.vertices[0].z(), WithinAbs(48.f / 255.f, 1e-4f));
    CHECK_THAT(repeated.vertices[1].z(), WithinAbs(191.25f / 255.f, 1e-4f));
    CHECK(std::abs(repeated.vertices[1].z() - repeated.vertices[0].z()) > 0.5f);
}

// Regression test for the bug this feature shipped with: with two layers painted over the same
// area, the second one was silently dropped (its paint mask was remapped onto the mesh the first
// layer had already displaced, which routinely produced an empty bitstream). Every layer is now
// evaluated against the original mesh instead, so both must show up in the total.
TEST_CASE("TextureDisplacement: a second layer over the same area is applied too", "[TextureDisplacement]")
{
    const indexed_triangle_set cube = its_make_cube(10., 10., 10.);

    TextureDisplacementFacetsData facets{};
    facets[0] = paint_whole_mesh(cube);
    facets[1] = facets[0]; // both layers cover the whole cube

    TextureDisplacementLayer base;
    base.slot         = 0;
    base.depth_mm     = 1.0f;
    base.tiling_scale = 5.0f;
    base.image_data   = make_flat_gray_png(255); // height 1.0 everywhere

    TextureDisplacementLayer second = base;
    second.slot       = 1;
    second.depth_mm   = 0.5f;
    second.blend_mode = TextureBlendMode::Add;

    const indexed_triangle_set result = build_texture_displacement(cube, {base, second}, facets, classic_options());

    // Topology is preserved exactly, so vertices can be compared 1:1 with the input.
    REQUIRE(result.vertices.size() == cube.vertices.size());
    REQUIRE(result.indices.size() == cube.indices.size());
    for (size_t i = 0; i < cube.vertices.size(); ++i)
        if (above_resting_plane(cube, i))
            CHECK_THAT((result.vertices[i] - cube.vertices[i]).norm(), WithinAbs(1.5f, 1e-3f)); // 1.0 + 0.5, not just 1.0
}

TEST_CASE("TextureDisplacement: blend modes combine a layer with the ones below it", "[TextureDisplacement]")
{
    const indexed_triangle_set cube = its_make_cube(10., 10., 10.);

    TextureDisplacementFacetsData facets{};
    facets[0] = paint_whole_mesh(cube);
    facets[1] = facets[0];

    TextureDisplacementLayer base;
    base.slot         = 0;
    base.depth_mm     = 2.0f;
    base.tiling_scale = 5.0f;
    base.image_data   = make_flat_gray_png(255); // -> contributes exactly +2.0 mm

    TextureDisplacementLayer second = base;
    second.slot     = 1;
    second.depth_mm = 0.5f; // -> its own value is 0.5 mm

    // Expected total displacement for each mode, given base = 2.0 mm and second = 0.5 mm. Multiply
    // and Divide treat the layer's value as a factor relative to 1 mm (see TextureBlendMode).
    const auto expected = GENERATE(table<TextureBlendMode, float>({
        { TextureBlendMode::Add,      2.5f },  // 2.0 + 0.5
        { TextureBlendMode::Subtract, 1.5f },  // 2.0 - 0.5
        { TextureBlendMode::Multiply, 1.0f },  // 2.0 * 0.5
        { TextureBlendMode::Divide,   4.0f },  // 2.0 / 0.5
    }));
    second.blend_mode = std::get<0>(expected);

    const indexed_triangle_set result = build_texture_displacement(cube, {base, second}, facets, classic_options());

    REQUIRE(result.vertices.size() == cube.vertices.size());
    for (size_t i = 0; i < cube.vertices.size(); ++i)
        if (above_resting_plane(cube, i))
            CHECK_THAT((result.vertices[i] - cube.vertices[i]).norm(), WithinAbs(std::get<1>(expected), 1e-3f));
}

TEST_CASE("TextureDisplacement: the lowest layer ignores its blend mode", "[TextureDisplacement]")
{
    // Multiply against the implicit zero base would annihilate the only layer present; the first
    // layer to reach a vertex always starts the total off additively instead.
    const indexed_triangle_set cube = its_make_cube(10., 10., 10.);

    TextureDisplacementFacetsData facets{};
    facets[0] = paint_whole_mesh(cube);

    TextureDisplacementLayer layer;
    layer.slot         = 0;
    layer.depth_mm     = 2.0f;
    layer.tiling_scale = 5.0f;
    layer.blend_mode   = TextureBlendMode::Multiply;
    layer.image_data   = make_flat_gray_png(255);

    const indexed_triangle_set result = build_texture_displacement(cube, {layer}, facets, classic_options());

    for (size_t i = 0; i < cube.vertices.size(); ++i)
        if (above_resting_plane(cube, i))
            CHECK_THAT((result.vertices[i] - cube.vertices[i]).norm(), WithinAbs(2.0f, 1e-3f));
}

TEST_CASE("TextureDisplacement: relief is never driven below the model's resting plane", "[TextureDisplacement]")
{
    // A fully painted cube displaces outward everywhere, which on the bottom face means straight
    // down - through the build plate. That geometry cannot be printed, so it is clamped back up.
    const indexed_triangle_set cube = its_make_cube(10., 10., 10.);

    TextureDisplacementFacetsData facets{};
    facets[0] = paint_whole_mesh(cube);

    TextureDisplacementLayer layer;
    layer.slot         = 0;
    layer.depth_mm     = 2.0f;
    layer.tiling_scale = 5.0f;
    layer.image_data   = make_flat_gray_png(255); // full depth everywhere

    float bottom = std::numeric_limits<float>::max();
    for (const Vec3f &v : cube.vertices)
        bottom = std::min(bottom, v.z());

    const indexed_triangle_set result = build_texture_displacement(cube, {layer}, facets, classic_options());

    REQUIRE(result.vertices.size() == cube.vertices.size());
    for (const Vec3f &v : result.vertices)
        CHECK(v.z() >= bottom - 1e-4f);

    // ...and the clamp is confined to Z: a bottom corner still moves outwards in X and Y by the same
    // amount it would have, rather than being pinned wholesale.
    bool any_bottom_moved_sideways = false;
    for (size_t i = 0; i < cube.vertices.size(); ++i)
        if (!above_resting_plane(cube, i) &&
            (result.vertices[i].head<2>() - cube.vertices[i].head<2>()).norm() > 1e-3f)
            any_bottom_moved_sideways = true;
    CHECK(any_bottom_moved_sideways);
}

TEST_CASE("TextureDisplacement: depth is measured in world millimetres, not the volume's own", "[TextureDisplacement]")
{
    // The same painted patch, baked once untransformed and once through a 3x scale. "Depth (mm)" is
    // a millimetre on the printed part, so the *world* relief must come out the same height either
    // way - which means the vertices of the scaled volume move by a third as much in its own
    // coordinates. Baking both in volume space instead gave a 3x deeper relief on the scaled one.
    indexed_triangle_set fan;
    fan.vertices = { {0.f, 0.f, 1.f}, {1.f, 0.f, 1.f}, {0.f, 1.f, 1.f}, {-1.f, 0.f, 1.f}, {0.f, -1.f, 1.f} };
    fan.indices  = { {0, 1, 2}, {0, 2, 3}, {0, 3, 4}, {0, 4, 1} };

    TextureDisplacementFacetsData facets{};
    facets[0] = paint_whole_mesh(fan);

    TextureDisplacementLayer layer;
    layer.slot         = 0;
    layer.depth_mm     = 1.0f;
    layer.tiling_scale = 5.0f;
    layer.image_data   = make_flat_gray_png(255);

    const indexed_triangle_set plain  = build_texture_displacement(fan, {layer}, facets, classic_options());
    Transform3d scale3 = Transform3d::Identity();
    scale3.scale(Vec3d(3.0, 3.0, 3.0));
    const indexed_triangle_set scaled = build_texture_displacement(fan, {layer}, facets, classic_options(), {}, nullptr, scale3);

    REQUIRE(plain.vertices.size() == fan.vertices.size());
    REQUIRE(scaled.vertices.size() == fan.vertices.size());
    for (size_t i = 0; i < fan.vertices.size(); ++i) {
        CHECK_THAT(plain.vertices[i].z() - fan.vertices[i].z(), WithinAbs(1.0f, 1e-3f));
        // A third of the movement locally is the same movement once the 3x scale is applied.
        CHECK_THAT(scaled.vertices[i].z() - fan.vertices[i].z(), WithinAbs(1.0f / 3.0f, 1e-3f));
    }
}

TEST_CASE("TextureDisplacement: a mirrored placement still raises the relief outwards", "[TextureDisplacement]")
{
    // Mirroring reverses the winding, and every normal in the bake is derived from the winding - so
    // without correcting for it the whole relief is carved into the surface instead of raised off it.
    indexed_triangle_set fan;
    fan.vertices = { {0.f, 0.f, 1.f}, {1.f, 0.f, 1.f}, {0.f, 1.f, 1.f}, {-1.f, 0.f, 1.f}, {0.f, -1.f, 1.f} };
    fan.indices  = { {0, 1, 2}, {0, 2, 3}, {0, 3, 4}, {0, 4, 1} };

    TextureDisplacementFacetsData facets{};
    facets[0] = paint_whole_mesh(fan);

    TextureDisplacementLayer layer;
    layer.slot         = 0;
    layer.depth_mm     = 1.0f;
    layer.tiling_scale = 5.0f;
    layer.image_data   = make_flat_gray_png(255);

    // Mirrored in X: the patch's outward direction in world space is still +Z, so in the volume's own
    // coordinates the vertices must still move +Z.
    Transform3d mirror_x = Transform3d::Identity();
    mirror_x.scale(Vec3d(-1.0, 1.0, 1.0));
    const indexed_triangle_set result = build_texture_displacement(fan, {layer}, facets, classic_options(), {}, nullptr, mirror_x);

    REQUIRE(result.vertices.size() == fan.vertices.size());
    for (size_t i = 0; i < fan.vertices.size(); ++i)
        CHECK_THAT(result.vertices[i].z() - fan.vertices[i].z(), WithinAbs(1.0f, 1e-3f));
}

TEST_CASE("TextureDisplacement: the patch border is displaced by default and pinned on request", "[TextureDisplacement]")
{
    // A small triangle fan around a central vertex O, with 4 outer points A/B/C/D forming 4
    // triangles T0..T3 in the XY plane. Only T0, T1, T2 are painted, T3 is left unpainted, so:
    //   O: touches all 4 triangles (incl. unpainted T3)      -> patch border
    //   A: touches T0 (painted) and T3 (unpainted)           -> patch border
    //   D: touches T2 (painted) and T3 (unpainted)           -> patch border
    //   B: touches only T0 and T1 (both painted)              -> interior
    //   C: touches only T1 and T2 (both painted)              -> interior
    indexed_triangle_set fan;
    fan.vertices = { {0.f, 0.f, 0.f}, {1.f, 0.f, 0.f}, {0.f, 1.f, 0.f}, {-1.f, 0.f, 0.f}, {0.f, -1.f, 0.f} };
    fan.indices  = { {0, 1, 2}, {0, 2, 3}, {0, 3, 4}, {0, 4, 1} };

    const TriangleMesh fan_mesh(fan);
    TriangleSelector    selector(fan_mesh);
    selector.set_facet(0, EnforcerBlockerType::ENFORCER);
    selector.set_facet(1, EnforcerBlockerType::ENFORCER);
    selector.set_facet(2, EnforcerBlockerType::ENFORCER);
    // facet 3 (T3) is left at its default EnforcerBlockerType::NONE.

    TextureDisplacementFacetsData facets{};
    facets[0] = selector.serialize();

    TextureDisplacementLayer layer;
    layer.slot        = 0;
    layer.depth_mm    = 1.0f;
    layer.tiling_scale = 5.0f;
    layer.image_data  = make_flat_gray_png(255);

    // The bake is topology-preserving, so it only ever moves fan's own vertices, in order.
    auto moved = [&](const indexed_triangle_set &result, size_t i) {
        return (result.vertices[i] - fan.vertices[i]).norm() > 1e-6f;
    };

    SECTION("by default the whole painted patch moves, border included")
    {
        const indexed_triangle_set result = build_texture_displacement(fan, {layer}, facets, classic_options());
        REQUIRE(result.vertices.size() == fan.vertices.size());
        for (size_t i = 0; i < fan.vertices.size(); ++i)
            CHECK(moved(result, i));
        // ... straight along the painted surface's own normal (+Z here), by the full depth. Nothing
        // has torn: the unpainted triangle T3 simply shares the moved vertices.
        for (size_t i = 0; i < fan.vertices.size(); ++i)
            CHECK_THAT(result.vertices[i].z() - fan.vertices[i].z(), WithinAbs(1.0f, 1e-3f));
        CHECK(result.indices == fan.indices);
    }

    SECTION("pinning the border holds exactly the vertices an unpainted triangle also uses")
    {
        TextureDisplacementOptions options = classic_options();
        options.displace_border = false;
        const indexed_triangle_set result = build_texture_displacement(fan, {layer}, facets, options);
        CHECK_FALSE(moved(result, 0)); // O: border
        CHECK_FALSE(moved(result, 1)); // A: border
        CHECK_FALSE(moved(result, 4)); // D: border
        CHECK(moved(result, 2));       // B: interior
        CHECK(moved(result, 3));       // C: interior
    }

}

TEST_CASE("TextureDisplacement: post-process smoothing relaxes only what moved", "[TextureDisplacement]")
{
    // A checkerboard height map on a fine grid gives a relief full of hard steps - exactly what the
    // smoothing pass is for. Only the central square is painted, so the patch has a real border.
    indexed_triangle_set plane;
    plane.vertices = { { 0.f, 0.f, 0.f }, { 20.f, 0.f, 0.f }, { 20.f, 20.f, 0.f }, { 0.f, 20.f, 0.f } };
    plane.indices  = { { 0, 1, 2 }, { 0, 2, 3 } };
    const indexed_triangle_set grid = subdivide_mesh_uniform(plane, 1.f, 6);
    REQUIRE(grid.indices.size() > 256);

    std::vector<uint8_t> painted(grid.indices.size(), 0);
    const TriangleMesh   grid_mesh(grid);
    TriangleSelector     selector(grid_mesh);
    for (int i = 0; i < int(grid.indices.size()); ++i) {
        const auto &t = grid.indices[i];
        float       cx = 0.f, cy = 0.f;
        for (int k = 0; k < 3; ++k) {
            cx += grid.vertices[t[k]].x() / 3.f;
            cy += grid.vertices[t[k]].y() / 3.f;
        }
        if (cx > 5.f && cx < 15.f && cy > 5.f && cy < 15.f) {
            selector.set_facet(i, EnforcerBlockerType::ENFORCER);
            painted[size_t(i)] = 1;
        }
    }
    REQUIRE(std::count(painted.begin(), painted.end(), uint8_t(1)) > 32);
    TextureDisplacementFacetsData facets{};
    facets[0] = selector.serialize();

    TextureDisplacementLayer layer;
    layer.slot         = 0;
    layer.depth_mm     = 2.0f;
    layer.tiling_scale = 6.0f;

    // The patch rim: vertices shared by a painted and an unpainted triangle - exactly the set
    // TextureDisplacementOptions::smooth_skip_border holds out of the relaxation.
    std::vector<uint8_t> rim(grid.vertices.size(), 0), inside(grid.vertices.size(), 0);
    for (size_t i = 0; i < grid.indices.size(); ++i)
        for (int k = 0; k < 3; ++k)
            (painted[i] ? inside : rim)[size_t(grid.indices[i][k])] = 1;
    size_t rim_count = 0;
    for (size_t v = 0; v < rim.size(); ++v) {
        rim[v] = (rim[v] && inside[v]) ? 1 : 0;
        rim_count += rim[v];
    }
    REQUIRE(rim_count > 8);

    TextureDisplacementOptions options = classic_options();
    options.smooth_enabled    = true;
    options.smooth_strength   = 0.5f;
    options.smooth_iterations = 4;

    SECTION("it rounds off the steps without touching the topology")
    {
        layer.image_data = make_checkerboard_png();

        // Dirichlet energy over the mesh's edges. Laplacian relaxation is gradient descent on exactly
        // this, so it is the quantity guaranteed to fall - unlike the min/max spread of z, which is
        // pinned by whichever vertices are held (the unpainted surface at zero, and by default the
        // patch rim as well) and so need not move at all.
        auto roughness = [](const indexed_triangle_set &its) {
            double e = 0.0;
            for (const auto &t : its.indices)
                for (int k = 0; k < 3; ++k) {
                    const double d = double(its.vertices[t[k]].z()) - double(its.vertices[t[(k + 1) % 3]].z());
                    e += d * d;
                }
            return e;
        };

        const indexed_triangle_set raw = build_texture_displacement(grid, { layer }, facets, classic_options());
        REQUIRE(roughness(raw) > 0.0); // the checkerboard really did produce relief to smooth

        const indexed_triangle_set smoothed = build_texture_displacement(grid, { layer }, facets, options);
        CHECK(roughness(smoothed) < roughness(raw));
        CHECK(smoothed.indices == raw.indices);               // ... without touching the topology
        CHECK(smoothed.vertices.size() == raw.vertices.size());
        for (size_t v = 0; v < rim.size(); ++v)               // ... and the held rim is bit-identical
            if (rim[v])
                CHECK_THAT(smoothed.vertices[v].z(), WithinAbs(raw.vertices[v].z(), 1e-6f));
    }

    SECTION("\"ignore outer ring\" decides whether the patch rim relaxes")
    {
        // A flat white texture makes this exact rather than statistical: every painted vertex is
        // displaced to precisely depth_mm, so a movable interior vertex sees nothing but neighbours at
        // its own height and cannot move, while every rim vertex has at least one neighbour outside the
        // paint pinned at zero and so must come down the moment it is allowed to.
        layer.image_data = make_flat_gray_png(255);
        const indexed_triangle_set raw = build_texture_displacement(grid, { layer }, facets, classic_options());

        options.smooth_skip_border      = true;
        const indexed_triangle_set kept = build_texture_displacement(grid, { layer }, facets, options);

        options.smooth_skip_border         = false;
        const indexed_triangle_set relaxed = build_texture_displacement(grid, { layer }, facets, options);

        // Asserted on z alone, not on the whole position: relaxation averages all three coordinates,
        // and while the tangential drift cancels by symmetry on a regular grid it does so only up to
        // floating-point summation order, which is not something to pin down across three platforms.
        // Height is what the option is about and it is exact - every neighbour of a movable vertex sits
        // at the same height, so its own height cannot move.
        for (size_t v = 0; v < rim.size(); ++v)
            if (rim[v]) {
                CHECK_THAT(kept.vertices[v].z(), WithinAbs(raw.vertices[v].z(), 1e-6f)); // held
                CHECK(relaxed.vertices[v].z() < raw.vertices[v].z());                    // melted down
            }
    }
}

TEST_CASE("TextureDisplacement: smooth_mesh_vertices holds everything outside its mask", "[TextureDisplacement]")
{
    // A single spike on a flat sheet: relaxing it must pull the spike down and leave every vertex
    // that is not flagged movable at exactly the coordinates it started at.
    indexed_triangle_set plane;
    plane.vertices = { { 0.f, 0.f, 0.f }, { 8.f, 0.f, 0.f }, { 8.f, 8.f, 0.f }, { 0.f, 8.f, 0.f } };
    plane.indices  = { { 0, 1, 2 }, { 0, 2, 3 } };
    indexed_triangle_set grid = subdivide_mesh_uniform(plane, 1.f, 4);

    // Raise one interior vertex, and let only it and its immediate neighbours move.
    size_t spike = 0;
    float  best  = std::numeric_limits<float>::max();
    for (size_t i = 0; i < grid.vertices.size(); ++i)
        if (const float d = (grid.vertices[i] - Vec3f(4.f, 4.f, 0.f)).norm(); d < best) {
            best  = d;
            spike = i;
        }
    grid.vertices[spike].z() = 5.f;

    std::vector<uint8_t> movable(grid.vertices.size(), 0);
    movable[spike] = 1;
    for (const auto &t : grid.indices)
        for (int e = 0; e < 3; ++e)
            if (size_t(t[e]) == spike)
                for (int k = 0; k < 3; ++k)
                    movable[size_t(t[k])] = 1;

    const indexed_triangle_set before = grid;
    smooth_mesh_vertices(grid, movable, 0.5f, 3);

    CHECK(grid.vertices[spike].z() < before.vertices[spike].z()); // the spike came down
    CHECK(grid.vertices[spike].z() > 0.f);                        // but was not flattened outright
    CHECK(grid.indices == before.indices);                        // topology untouched
    for (size_t i = 0; i < grid.vertices.size(); ++i)
        if (!movable[i])
            CHECK_THAT((grid.vertices[i] - before.vertices[i]).norm(), WithinAbs(0.f, 1e-9f));

    // Guard rails: each of these must leave the mesh byte-identical.
    for (const auto &noop : { std::make_pair(0.f, 3), std::make_pair(0.5f, 0) }) {
        indexed_triangle_set copy = before;
        smooth_mesh_vertices(copy, movable, noop.first, noop.second);
        CHECK(copy.vertices == before.vertices);
    }
    indexed_triangle_set copy = before;
    smooth_mesh_vertices(copy, std::vector<uint8_t>(3, 1), 0.5f, 3); // mis-sized mask
    CHECK(copy.vertices == before.vertices);
}

// Every undirected edge of a closed manifold mesh is shared by exactly two triangles. A T-junction
// (a hanging node where a refined region meets a coarse one) breaks that: the coarse side spans an
// edge that the fine side has replaced with two half-edges, so those three edges each show up an
// odd number of times. Counting edge uses is therefore an exact crack detector for a closed mesh.
static bool every_edge_used_twice(const indexed_triangle_set &its)
{
    std::map<std::pair<int, int>, int> uses;
    for (const auto &t : its.indices)
        for (int e = 0; e < 3; ++e) {
            int a = t[e], b = t[(e + 1) % 3];
            if (a > b)
                std::swap(a, b);
            ++uses[{ a, b }];
        }
    for (const auto &[edge, n] : uses)
        if (n != 2)
            return false;
    return true;
}

TEST_CASE("TextureDisplacement: adaptive subdivision is conformal and region-restricted", "[TextureDisplacement]")
{
    const indexed_triangle_set cube = its_make_cube(10., 10., 10.);
    REQUIRE(every_edge_used_twice(cube)); // sanity: the input really is a closed manifold

    auto longest_edge = [](const indexed_triangle_set &its, const stl_triangle_vertex_indices &t) {
        float m = 0.f;
        for (int e = 0; e < 3; ++e)
            m = std::max(m, (its.vertices[t[e]] - its.vertices[t[(e + 1) % 3]]).norm());
        return m;
    };

    SECTION("whole-mesh region refines everywhere and stays conformal")
    {
        std::vector<uint8_t> region(cube.indices.size(), 1);
        std::vector<int>     source;
        const indexed_triangle_set out = subdivide_mesh_adaptive(cube, region, 3.f, 100000, &source);

        CHECK(out.indices.size() > cube.indices.size()); // it actually refined
        CHECK(every_edge_used_twice(out));               // ... without opening a single crack

        // Refinement runs to completion, not for a fixed number of passes: with the whole mesh in the
        // region and budget to spare, *every* edge really does end up at or below the target. This is
        // the regression that matters - an earlier version quietly stopped a long way short, having
        // spent its pass budget grading the coarse surroundings.
        float worst = 0.f;
        for (const auto &t : out.indices)
            worst = std::max(worst, longest_edge(out, t));
        CHECK(worst <= 3.f);

        REQUIRE(source.size() == out.indices.size());
        for (int s : source)
            CHECK((s >= 0 && s < int(cube.indices.size()))); // every child names a real parent
    }

    SECTION("a partial region refines only there, and the boundary is still crack-free")
    {
        // Refine only the triangles whose centroid is in the upper (z > 5) half of the cube.
        std::vector<uint8_t> region(cube.indices.size(), 0);
        size_t               region_count = 0;
        for (size_t i = 0; i < cube.indices.size(); ++i) {
            const auto &t = cube.indices[i];
            const float cz = (cube.vertices[t[0]].z() + cube.vertices[t[1]].z() + cube.vertices[t[2]].z()) / 3.f;
            if (cz > 5.f) {
                region[i] = 1;
                ++region_count;
            }
        }
        REQUIRE(region_count > 0);

        std::vector<int>           source;
        const indexed_triangle_set out = subdivide_mesh_adaptive(cube, region, 2.f, 100000, &source);

        CHECK(out.indices.size() > cube.indices.size());
        CHECK(every_edge_used_twice(out)); // the refined/coarse seam has no T-junction

        // Inside the region the target is actually met - refinement is not cut short by a pass budget.
        // Outside it, only the graded transition band conformality requires is touched, so plenty of
        // the unpainted mesh is still coarser than the target: the region was not a suggestion.
        float max_in = 0.f, max_out = 0.f;
        for (size_t i = 0; i < out.indices.size(); ++i) {
            float &acc = region[source[i]] ? max_in : max_out;
            acc        = std::max(acc, longest_edge(out, out.indices[i]));
        }
        CHECK(max_in <= 2.f);
        CHECK(max_out > 2.f);

        std::vector<uint8_t>       all(cube.indices.size(), 1);
        const indexed_triangle_set whole = subdivide_mesh_adaptive(cube, all, 2.f, 100000);
        CHECK(out.indices.size() < whole.indices.size()); // ... and it cost less than doing the lot
    }

    SECTION("the triangle budget caps the result and still leaves a conformal mesh")
    {
        std::vector<uint8_t>       region(cube.indices.size(), 1);
        const indexed_triangle_set out = subdivide_mesh_adaptive(cube, region, 0.05f, /*max_triangles*/ 500);
        CHECK(out.indices.size() <= 500);
        CHECK(out.indices.size() > cube.indices.size()); // it spent the budget rather than giving up
        CHECK(every_edge_used_twice(out));               // stopping on the budget is not a crack
    }

    SECTION("an empty region is a no-op")
    {
        std::vector<uint8_t>       region(cube.indices.size(), 0);
        const indexed_triangle_set out = subdivide_mesh_adaptive(cube, region, 1.f, 100000, nullptr);
        CHECK(out.indices.size() == cube.indices.size());
        CHECK(out.vertices.size() == cube.vertices.size());
    }
}

TEST_CASE("TextureDisplacement: feature-adaptive subdivision follows curvature, not slope", "[TextureDisplacement]")
{
    // A flat sheet, tessellated into a regular grid to give the bisector something to work with.
    indexed_triangle_set plane;
    plane.vertices = { { 0.f, 0.f, 0.f }, { 1.f, 0.f, 0.f }, { 1.f, 1.f, 0.f }, { 0.f, 1.f, 0.f } };
    plane.indices  = { { 0, 1, 2 }, { 0, 2, 3 } };
    const indexed_triangle_set grid = subdivide_mesh_uniform(plane, 0.15f, 5); // ~uniform grid of small triangles
    REQUIRE(grid.indices.size() > 32);

    const std::vector<uint8_t> region(grid.indices.size(), 1);

    auto longest_edge = [](const indexed_triangle_set &its, const stl_triangle_vertex_indices &t) {
        float m = 0.f;
        for (int e = 0; e < 3; ++e)
            m = std::max(m, (its.vertices[t[e]] - its.vertices[t[(e + 1) % 3]]).norm());
        return m;
    };
    auto centroid_xy = [](const indexed_triangle_set &its, const stl_triangle_vertex_indices &t) {
        return Vec2f((its.vertices[t[0]].x() + its.vertices[t[1]].x() + its.vertices[t[2]].x()) / 3.f,
                     (its.vertices[t[0]].y() + its.vertices[t[1]].y() + its.vertices[t[2]].y()) / 3.f);
    };

    SECTION("a sharp hill refines densely at its center and leaves flat corners coarse")
    {
        // A tight Gaussian hill at the sheet's center: strong curvature near (0.5, 0.5), flat far away.
        HeightFieldSampler hill = [](const Vec3f &p, const Vec3f &) {
            const float r2 = (p.x() - 0.5f) * (p.x() - 0.5f) + (p.y() - 0.5f) * (p.y() - 0.5f);
            return 1.0f * std::exp(-r2 / 0.02f);
        };

        // Baseline max edge 0.3 is coarser than the grid's own edges, so the baseline adds nothing
        // here - this isolates the *curvature* contribution (the grid already meets the baseline).
        std::vector<int>           source;
        const indexed_triangle_set out =
            subdivide_mesh_adaptive(grid, region, /*max edge*/ 0.3f, 200000, &source, hill, /*tol*/ 0.02f,
                                    /*min_edge*/ 0.01f);

        CHECK(out.indices.size() > grid.indices.size()); // the hill forced real refinement

        // The largest triangle near the hill's center must be much smaller than the largest in a flat
        // corner - i.e. triangles went where the curvature is, not spread evenly.
        float near_max = 0.f, far_max = 0.f;
        for (const auto &t : out.indices) {
            const Vec2f c   = centroid_xy(out, t);
            const float r   = (c - Vec2f(0.5f, 0.5f)).norm();
            const float len = longest_edge(out, t);
            if (r < 0.1f)
                near_max = std::max(near_max, len);
            else if (r > 0.45f)
                far_max = std::max(far_max, len);
        }
        REQUIRE(near_max > 0.f);
        REQUIRE(far_max > 0.f);
        CHECK(near_max < far_max); // finer at the hill than on the flats
    }

    SECTION("a linear ramp has zero curvature and is left untouched")
    {
        // Height varies, but linearly - a flat triangle represents it exactly, so the chord error is
        // zero everywhere and nothing should be split. This is the case a gradient-based criterion
        // would wrongly over-refine.
        HeightFieldSampler ramp = [](const Vec3f &p, const Vec3f &) { return 2.0f * p.x(); };

        // Same coarse baseline (0.3) that the grid already meets, so any split would be curvature-
        // driven - and a ramp has none.
        const indexed_triangle_set out =
            subdivide_mesh_adaptive(grid, region, /*max edge*/ 0.3f, 200000, nullptr, ramp, /*tol*/ 0.02f,
                                    /*min_edge*/ 0.01f);

        CHECK(out.indices.size() == grid.indices.size()); // not one extra triangle
    }

    SECTION("the max-edge baseline still applies in feature mode")
    {
        // A height field that is flat everywhere the four sample points of a coarse triangle happen to
        // land, but not in between - the aliasing case where a chord test alone reports no error and
        // refinement stalls before it ever starts. The baseline is what stops that: it guarantees a
        // sampling density fine enough for the curvature test to see the texture at all.
        HeightFieldSampler flat = [](const Vec3f &, const Vec3f &) { return 0.f; };

        const indexed_triangle_set out =
            subdivide_mesh_adaptive(grid, region, /*max edge*/ 0.03f, 200000, nullptr, flat, /*tol*/ 0.02f,
                                    /*min_edge*/ 0.001f);

        CHECK(out.indices.size() > grid.indices.size());
        float worst = 0.f;
        for (const auto &t : out.indices)
            worst = std::max(worst, longest_edge(out, t));
        CHECK(worst <= 0.03f);
    }
}

// A 2x2 truecolour PNG: red, green / blue, white. Written out as bytes rather than encoded here
// because libslic3r only *writes* grayscale PNGs (png::write_gray_to_file) - which is also exactly
// why the colour path exists: the GUI importer stores colour images through wxImage instead.
static std::shared_ptr<std::vector<unsigned char>> make_rgb_png_2x2()
{
    static const unsigned char bytes[] = {
        0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a, 0x00, 0x00, 0x00, 0x0d,
        0x49, 0x48, 0x44, 0x52, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02,
        0x08, 0x02, 0x00, 0x00, 0x00, 0xfd, 0xd4, 0x9a, 0x73, 0x00, 0x00, 0x00,
        0x14, 0x49, 0x44, 0x41, 0x54, 0x78, 0xda, 0x63, 0xf8, 0xcf, 0xc0, 0xc0,
        0x00, 0xc2, 0x0c, 0xff, 0xff, 0xff, 0xff, 0x0f, 0x00, 0x1f, 0xee, 0x05,
        0xfb, 0x60, 0x6c, 0x70, 0xf2, 0x00, 0x00, 0x00, 0x00, 0x49, 0x45, 0x4e,
        0x44, 0xae, 0x42, 0x60, 0x82,
    };
    return std::make_shared<std::vector<unsigned char>>(std::begin(bytes), std::end(bytes));
}

// The same image with an opaque alpha channel, so four bytes per pixel instead of three.
static std::shared_ptr<std::vector<unsigned char>> make_rgba_png_2x2()
{
    static const unsigned char bytes[] = {
        0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a, 0x00, 0x00, 0x00, 0x0d,
        0x49, 0x48, 0x44, 0x52, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02,
        0x08, 0x06, 0x00, 0x00, 0x00, 0x72, 0xb6, 0x0d, 0x24, 0x00, 0x00, 0x00,
        0x12, 0x49, 0x44, 0x41, 0x54, 0x78, 0xda, 0x63, 0xf8, 0xcf, 0xc0, 0xf0,
        0x1f, 0x0c, 0x81, 0x34, 0x18, 0x00, 0x00, 0x49, 0xc8, 0x09, 0xf7, 0x03,
        0xd9, 0x64, 0xf1, 0x00, 0x00, 0x00, 0x00, 0x49, 0x45, 0x4e, 0x44, 0xae,
        0x42, 0x60, 0x82,
    };
    return std::make_shared<std::vector<unsigned char>>(std::begin(bytes), std::end(bytes));
}

TEST_CASE("TextureDisplacement: a colour texture decodes to both colour and height", "[TextureDisplacement]")
{
    TextureDisplacementLayer layer;
    layer.slot       = 0;
    // Row stride differs between the two, and both must come out the same way up.
    layer.image_data = GENERATE(make_rgb_png_2x2(), make_rgba_png_2x2());

    const DecodedHeightTexture tex = decode_height_texture(layer);
    REQUIRE_FALSE(tex.empty());
    REQUIRE(tex.has_color());
    REQUIRE(tex.width == 2);
    REQUIRE(tex.height == 2);
    REQUIRE(tex.rgb.size() == 2 * 2 * 3);

    // Row-major, top-to-bottom: red, green / blue, white.
    CHECK(tex.rgb[0] == 255);  CHECK(tex.rgb[1] == 0);    CHECK(tex.rgb[2] == 0);
    CHECK(tex.rgb[3] == 0);    CHECK(tex.rgb[4] == 255);  CHECK(tex.rgb[5] == 0);
    CHECK(tex.rgb[6] == 0);    CHECK(tex.rgb[7] == 0);    CHECK(tex.rgb[8] == 255);
    CHECK(tex.rgb[9] == 255);  CHECK(tex.rgb[10] == 255); CHECK(tex.rgb[11] == 255);

    // Height is the luminance, with wxImage::ConvertToGreyscale()'s coefficients - which is what
    // makes a texture displace identically whether it was imported before or after colour was kept.
    CHECK(int(tex.pixels[0]) == int(std::lround(0.299 * 255))); // red
    CHECK(int(tex.pixels[1]) == int(std::lround(0.587 * 255))); // green
    CHECK(int(tex.pixels[2]) == int(std::lround(0.114 * 255))); // blue
    CHECK(int(tex.pixels[3]) == 255);                           // white
}

TEST_CASE("A 16-bit colour texture decodes to nothing rather than noise", "[TextureDisplacement]")
{
    // The 2x2 image above at 16 bits per channel. The texture library converts such a file to 8-bit
    // on load, so it can only arrive here stored as-is, from a project file.
    static const unsigned char bytes[] = {
        0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a, 0x00, 0x00, 0x00, 0x0d,
        0x49, 0x48, 0x44, 0x52, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02,
        0x10, 0x02, 0x00, 0x00, 0x00, 0xad, 0x44, 0x46, 0x30, 0x00, 0x00, 0x00,
        0x12, 0x49, 0x44, 0x41, 0x54, 0x78, 0xda, 0x63, 0xf8, 0xff, 0x9f, 0x01,
        0x0c, 0x60, 0x34, 0x90, 0x01, 0x01, 0x00, 0x75, 0xa4, 0x0b, 0xf5, 0x97,
        0xf4, 0x36, 0xa1, 0x00, 0x00, 0x00, 0x00, 0x49, 0x45, 0x4e, 0x44, 0xae,
        0x42, 0x60, 0x82,
    };
    TextureDisplacementLayer layer;
    layer.slot       = 0;
    layer.image_data = std::make_shared<std::vector<unsigned char>>(std::begin(bytes), std::end(bytes));

    CHECK(decode_height_texture(layer).empty());
}

TEST_CASE("A truncated texture decodes to nothing instead of aborting", "[TextureDisplacement]")
{
    // A half-copied file in the texture folder, or a damaged project file. libpng reports this by
    // longjmp, and aborts the process if the decoder has not set a jump buffer to land on.
    const auto whole = GENERATE(make_flat_gray_png(128, 16, 16), make_rgb_png_2x2());
    TextureDisplacementLayer layer;
    layer.slot       = 0;
    layer.image_data = std::make_shared<std::vector<unsigned char>>(whole->begin(), whole->begin() + whole->size() / 2);

    CHECK(decode_height_texture(layer).empty());
}

TEST_CASE("TextureDisplacement: a grayscale texture reports no colour", "[TextureDisplacement]")
{
    // The shipped library is all grayscale, and has_color() is what the whole colour feature keys
    // off - a height map must never look like it has colours to apply.
    TextureDisplacementLayer layer;
    layer.slot       = 0;
    layer.image_data = make_flat_gray_png(128);

    const DecodedHeightTexture tex = decode_height_texture(layer);
    REQUIRE_FALSE(tex.empty());
    CHECK_FALSE(tex.has_color());
    CHECK(tex.rgb.empty());

    Vec3f out(9.f, 9.f, 9.f);
    CHECK_FALSE(sample_layer_color(tex, layer, Vec3f::Zero(), Vec3f::UnitZ(), out));
    CHECK(out.x() == 9.f); // left untouched on a false return
}

// Two triangles making a 10x10 quad in the z=0 plane.
static indexed_triangle_set color_test_quad()
{
    indexed_triangle_set quad;
    quad.vertices = { Vec3f(0, 0, 0), Vec3f(10, 0, 0), Vec3f(10, 10, 0), Vec3f(0, 10, 0) };
    quad.indices  = { { 0, 1, 2 }, { 0, 2, 3 } };
    return quad;
}


// This palette fixture keeps one triangle in a blue island within an 8x8 image and its three
// edge-neighbours in red. The mesh is intentionally small but connected: no private
// despeckle helper is called, so the assertions observe only the public bake output.
static std::shared_ptr<std::vector<unsigned char>> make_despeckle_noise_png()
{
    static const unsigned char bytes[] = {
        0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a, 0x00, 0x00, 0x00, 0x0d,
        0x49, 0x48, 0x44, 0x52, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x08,
        0x08, 0x02, 0x00, 0x00, 0x00, 0x4b, 0x6d, 0x29, 0xdc, 0x00, 0x00, 0x00,
        0x18, 0x49, 0x44, 0x41, 0x54, 0x78, 0x9c, 0x63, 0xf8, 0xcf, 0xc0, 0x80,
        0x15, 0x61, 0x17, 0x25, 0x4e, 0x02, 0x42, 0x93, 0x22, 0x41, 0xb1, 0xe5,
        0x00, 0x21, 0x07, 0x3f, 0xc1, 0xb4, 0x44, 0xac, 0x47, 0x00, 0x00, 0x00,
        0x00, 0x49, 0x45, 0x4e, 0x44, 0xae, 0x42, 0x60, 0x82,
    };
    return std::make_shared<std::vector<unsigned char>>(std::begin(bytes), std::end(bytes));
}

static std::shared_ptr<std::vector<unsigned char>> make_despeckle_tie_png()
{
    static const unsigned char bytes[] = {
        0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a, 0x00, 0x00, 0x00, 0x0d,
        0x49, 0x48, 0x44, 0x52, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x08,
        0x08, 0x02, 0x00, 0x00, 0x00, 0x4b, 0x6d, 0x29, 0xdc, 0x00, 0x00, 0x00,
        0x17, 0x49, 0x44, 0x41, 0x54, 0x78, 0x9c, 0x63, 0xf8, 0xcf, 0xc0, 0x00,
        0x41, 0xe8, 0x80, 0x74, 0x09, 0x0c, 0x35, 0xa4, 0x48, 0x50, 0x08, 0x00,
        0x88, 0xa2, 0x0c, 0xf4, 0xbb, 0x9e, 0xa1, 0x18, 0x00, 0x00, 0x00, 0x00,
        0x49, 0x45, 0x4e, 0x44, 0xae, 0x42, 0x60, 0x82,
    };
    return std::make_shared<std::vector<unsigned char>>(std::begin(bytes), std::end(bytes));
}

static indexed_triangle_set make_despeckle_star()
{
    indexed_triangle_set mesh;
    mesh.vertices = {
        { 3.2f, 3.2f, 0.f }, // 0: central triangle
        { 4.0f, 3.2f, 0.f },
        { 3.2f, 4.0f, 0.f },
        { 0.f, 0.f, 0.f },    // red neighbour
        { 7.f, 7.f, 0.f },    // uncoloured neighbour in the tie image
        { 0.f, 7.f, 0.f },    // uncoloured neighbour in the tie image
    };
    mesh.indices = {
        { 0, 1, 2 }, // blue centre; its three edge neighbours are below
        { 1, 0, 3 },
        { 2, 1, 4 },
        { 0, 2, 5 },
    };
    return mesh;
}

static std::vector<uint8_t> bake_despeckle_fixture(const indexed_triangle_set &mesh,
                                                    const std::shared_ptr<std::vector<unsigned char>> &image,
                                                    int passes)
{
    TextureDisplacementLayer layer;
    layer.slot          = 0;
    layer.image_data    = image;
    layer.color_enabled = true;
    layer.depth_mm      = 0.f; // isolate the per-face colour result from height displacement
    layer.tiling_scale  = 8.f;
    layer.tile_enabled  = false;
    layer.projection_method = TextureProjectionMethod::Triplanar;

    TriangleMesh     triangle_mesh(mesh);
    TriangleSelector selector(triangle_mesh);
    for (int face = 0; face < int(mesh.indices.size()); ++face)
        selector.set_facet(face, EnforcerBlockerType::ENFORCER);
    TextureDisplacementFacetsData facets;
    facets[0] = selector.serialize();

    TextureColorRequest request;
    std::vector<uint8_t> colors;
    request.out_triangle         = &colors;
    request.despeckle_passes     = passes;
    request.min_color_region_mm2 = 0.f; // keep this test about the majority filter, not area merging
    request.quantize             = [](const Vec3f &rgb) {
        if (rgb.z() > 0.15f && rgb.z() > rgb.x() + 0.02f)
            return 1; // blue
        if (rgb.x() > 0.15f && rgb.x() > rgb.z() + 0.02f)
            return 0; // red
        return -1; // the tie fixture's black texels are an uncoloured boundary
    };

    const indexed_triangle_set out =
        build_texture_displacement(mesh, { layer }, facets, classic_options(), {}, &request);
    REQUIRE(out.indices == mesh.indices);
    REQUIRE(colors.size() == mesh.indices.size());
    return colors;
}

TEST_CASE("Texture displacement removes an isolated colour speck from a connected bake",
          "[TextureDisplacement][TextureColorDespeckle]")
{
    const indexed_triangle_set mesh = make_despeckle_star();
    const auto                 image = make_despeckle_noise_png();

    const std::vector<uint8_t> without_filter = bake_despeckle_fixture(mesh, image, 0);
    const std::vector<uint8_t> with_filter    = bake_despeckle_fixture(mesh, image, 1);

    // Color indices are +1 in the public request output: red=1, blue=2, uncoloured=0.
    const std::vector<uint8_t> expected_without{ 2, 1, 1, 1 };
    const std::vector<uint8_t> expected_with{ 1, 1, 1, 1 };
    REQUIRE(without_filter == expected_without);
    CHECK(with_filter == expected_with);
}

TEST_CASE("Texture displacement keeps tied colours and uncoloured boundaries stable",
          "[TextureDisplacement][TextureColorDespeckle]")
{
    const std::vector<uint8_t> colors =
        bake_despeckle_fixture(make_despeckle_star(), make_despeckle_tie_png(), 1);

    // The centre sees one blue vote and one red vote; black/uncoloured neighbours do not vote.
    const std::vector<uint8_t> expected{ 2, 1, 0, 0 };
    REQUIRE(colors == expected);
}

TEST_CASE("Texture displacement colour despeckling is independent of triangle order",
          "[TextureDisplacement][TextureColorDespeckle]")
{
    const indexed_triangle_set mesh = make_despeckle_star();
    indexed_triangle_set       reordered = mesh;
    reordered.indices = { mesh.indices[2], mesh.indices[1], mesh.indices[0], mesh.indices[3] };

    const std::vector<uint8_t> original =
        bake_despeckle_fixture(mesh, make_despeckle_tie_png(), 1);
    const std::vector<uint8_t> shuffled =
        bake_despeckle_fixture(reordered, make_despeckle_tie_png(), 1);

    REQUIRE(original.size() == shuffled.size());
    for (size_t i = 0; i < shuffled.size(); ++i) {
        const auto &face = reordered.indices[i];
        size_t      original_index = mesh.indices.size();
        for (size_t j = 0; j < mesh.indices.size(); ++j) {
            const auto &candidate = mesh.indices[j];
            if (candidate[0] == face[0] && candidate[1] == face[1] && candidate[2] == face[2]) {
                original_index = j;
                break;
            }
        }
        REQUIRE(original_index < mesh.indices.size());
        CHECK(shuffled[i] == original[original_index]);
    }
}

TEST_CASE("TextureDisplacement: colour is reported per triangle and only where painted", "[TextureDisplacement]")
{
    const indexed_triangle_set quad = color_test_quad();

    TextureDisplacementLayer layer;
    layer.slot              = 0;
    layer.image_data        = make_rgb_png_2x2();
    layer.color_enabled     = true;
    layer.depth_mm          = 0.f; // colour only, so this isolates the colour path from the geometry
    layer.tiling_scale      = 100.f;
    layer.projection_method = TextureProjectionMethod::Triplanar;

    TriangleMesh     mesh(quad);
    TriangleSelector selector(mesh);
    selector.set_facet(0, EnforcerBlockerType::ENFORCER); // only the first triangle

    TextureDisplacementFacetsData facets;
    facets[0] = selector.serialize();

    // A three-entry palette matched in plain RGB: all this test needs is *an* index. The perceptual
    // matching is the GUI's (make_palette_quantizer), and is deliberately not under test here.
    const std::array<Vec3f, 3> palette = { Vec3f(1, 0, 0), Vec3f(0, 1, 0), Vec3f(0, 0, 1) };
    TextureColorRequest        request;
    std::vector<uint8_t>       triangle_color;
    request.out_triangle = &triangle_color;
    request.quantize     = [&palette](const Vec3f &rgb) {
        int   best = 0;
        float bd   = std::numeric_limits<float>::max();
        for (int i = 0; i < 3; ++i)
            if (const float d = (palette[size_t(i)] - rgb).squaredNorm(); d < bd) {
                bd   = d;
                best = i;
            }
        return best;
    };

    const indexed_triangle_set out = build_texture_displacement(quad, { layer }, facets, classic_options(), {}, &request);

    REQUIRE_FALSE(out.indices.empty());
    REQUIRE(triangle_color.size() == quad.indices.size());
    // The painted triangle takes a filament; the unpainted one is left at 0, which is
    // EnforcerBlockerType::NONE - "use the volume's own filament". That is what confines the effect
    // to the painted area without having to invent a colour for everything outside it.
    CHECK(triangle_color[0] != 0);
    CHECK(triangle_color[1] == 0);
}

TEST_CASE("TextureDisplacement: a layer that is not colouring reports no colours", "[TextureDisplacement]")
{
    const indexed_triangle_set quad = color_test_quad();

    TextureDisplacementLayer layer;
    layer.slot          = 0;
    layer.image_data    = make_rgb_png_2x2();
    layer.color_enabled = false; // the checkbox is off: colour stays off even on a colour texture
    layer.depth_mm      = 1.f;

    TriangleMesh     mesh(quad);
    TriangleSelector selector(mesh);
    selector.set_facet(0, EnforcerBlockerType::ENFORCER);
    selector.set_facet(1, EnforcerBlockerType::ENFORCER);

    TextureDisplacementFacetsData facets;
    facets[0] = selector.serialize();

    TextureColorRequest  request;
    std::vector<uint8_t> triangle_color;
    request.out_triangle = &triangle_color;
    request.quantize     = [](const Vec3f &) { return 0; };

    build_texture_displacement(quad, { layer }, facets, classic_options(), {}, &request);

    REQUIRE(triangle_color.size() == quad.indices.size());
    CHECK(triangle_color[0] == 0);
    CHECK(triangle_color[1] == 0);
}

TEST_CASE("TextureDisplacement: subdivision refines a colour boundary a flat height field hides",
          "[TextureDisplacement]")
{
    // A cube with a flat height field, so *nothing* in the height criteria has any reason to refine
    // it - which is exactly the case the colour criterion exists for. Closed, so every_edge_used_twice()
    // is an exact crack detector: the colour criterion goes through the same conformal bisection as
    // everything else and must not be able to open one.
    const indexed_triangle_set cube = its_make_cube(10., 10., 10.);
    const std::vector<uint8_t> region(cube.indices.size(), REFINE_PAINTED);

    auto longest_edge = [](const indexed_triangle_set &its, const stl_triangle_vertex_indices &t) {
        float m = 0.f;
        for (int e = 0; e < 3; ++e)
            m = std::max(m, (its.vertices[t[e]] - its.vertices[t[(e + 1) % 3]]).norm());
        return m;
    };

    // One filament on each side of x = 5: a step, with no gradient anywhere for a chord test to see.
    ColorFieldSampler split_at_five = [](const Vec3f &p, const Vec3f &) { return p.x() < 5.f ? 0 : 1; };
    ColorFieldSampler all_one       = [](const Vec3f &, const Vec3f &) { return 0; };

    SECTION("a colour boundary gets triangles")
    {
        const indexed_triangle_set out =
            subdivide_mesh_adaptive(cube, region, /*max edge*/ 0.f, 200000, nullptr, nullptr, 0.f,
                                    /*min_edge*/ 0.05f, /*border*/ 0.f, nullptr, split_at_five,
                                    /*colour edge*/ 0.5f);

        CHECK(out.indices.size() > cube.indices.size());
        CHECK(every_edge_used_twice(out)); // still watertight

        // Every triangle still straddling the boundary must be down at the target.
        for (const auto &t : out.indices) {
            bool straddles = false;
            for (int i = 1; i < 3; ++i)
                if ((out.vertices[t[i]].x() < 5.f) != (out.vertices[t[0]].x() < 5.f))
                    straddles = true;
            if (straddles)
                CHECK(longest_edge(out, t) <= 0.5f + 1e-4f);
        }
    }

    SECTION("a uniform colour adds nothing")
    {
        const indexed_triangle_set out =
            subdivide_mesh_adaptive(cube, region, /*max edge*/ 0.f, 200000, nullptr, nullptr, 0.f,
                                    /*min_edge*/ 0.05f, /*border*/ 0.f, nullptr, all_one,
                                    /*colour edge*/ 0.5f);

        CHECK(out.indices.size() == cube.indices.size());
    }

    SECTION("no colour sampler leaves the mesh alone")
    {
        // The regression this guards: the colour criterion must be inert when nothing is colouring,
        // or every bake would start refining geometry for no reason.
        const indexed_triangle_set out =
            subdivide_mesh_adaptive(cube, region, /*max edge*/ 0.f, 200000, nullptr, nullptr, 0.f,
                                    /*min_edge*/ 0.05f, /*border*/ 0.f, nullptr, nullptr,
                                    /*colour edge*/ 0.5f);

        CHECK(out.indices.size() == cube.indices.size());
    }
}

// ---------------------------------------------------------------------------------------------
// Step cutter
// ---------------------------------------------------------------------------------------------

// A flat square sheet in the (x, y) plane split into right triangles of the given edge, with the
// diagonal alternated so the mesh has no preferred direction. Normals +z.
static indexed_triangle_set make_flat_sheet(float size, float edge)
{
    indexed_triangle_set its;
    const int            n = std::max(1, int(std::lround(size / edge)));
    for (int j = 0; j <= n; ++j)
        for (int i = 0; i <= n; ++i)
            its.vertices.emplace_back(size * float(i) / float(n), size * float(j) / float(n), 0.f);
    const auto id = [n](int i, int j) { return j * (n + 1) + i; };
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i) {
            if ((i + j) % 2 == 0) {
                its.indices.emplace_back(id(i, j), id(i + 1, j), id(i + 1, j + 1));
                its.indices.emplace_back(id(i, j), id(i + 1, j + 1), id(i, j + 1));
            } else {
                its.indices.emplace_back(id(i, j), id(i + 1, j), id(i, j + 1));
                its.indices.emplace_back(id(i + 1, j), id(i + 1, j + 1), id(i, j + 1));
            }
        }
    return its;
}

// Every interior edge of a sheet is shared by exactly two triangles; only the sheet's own border may
// be used once.
static bool sheet_is_manifold(const indexed_triangle_set &its, float size)
{
    std::map<std::pair<int, int>, int> uses;
    for (const auto &t : its.indices)
        for (int e = 0; e < 3; ++e) {
            int a = t[e], b = t[(e + 1) % 3];
            if (a > b)
                std::swap(a, b);
            ++uses[{ a, b }];
        }
    for (const auto &[edge, n] : uses) {
        const Vec3f &pa = its.vertices[size_t(edge.first)], &pb = its.vertices[size_t(edge.second)];
        const bool   border = (pa.x() == pb.x() && (pa.x() == 0.f || pa.x() == size)) ||
                            (pa.y() == pb.y() && (pa.y() == 0.f || pa.y() == size));
        if (n > 2 || (n == 1 && !border))
            return false;
    }
    return true;
}

TEST_CASE("TextureDisplacement: the step cutter turns a stepped field into walls", "[TextureDisplacement]")
{
    // Square posts 1.2 mm wide on a 2 mm pitch, as a binary field: a step everywhere along the post
    // edges, flat everywhere else. 1 mm triangles, so every post edge crosses several of them.
    constexpr float RELIEF = 0.4f, SIZE = 6.f, STEP_W = 0.05f, GAP = 0.075f;
    const auto      posts = [](float x, float y) {
        const float fx = std::fmod(std::fmod(x, 2.f) + 2.f, 2.f), fy = std::fmod(std::fmod(y, 2.f) + 2.f, 2.f);
        return (fx > 0.4f && fx < 1.6f && fy > 0.4f && fy < 1.6f) ? RELIEF : 0.f;
    };
    const HeightFieldSampler   sampler = [&](const Vec3f &p, const Vec3f &) { return posts(p.x(), p.y()); };
    const indexed_triangle_set sheet   = make_flat_sheet(SIZE, 1.f);
    const std::vector<uint8_t> region(sheet.indices.size(), 1);

    std::vector<int>           source;
    size_t                     cuts = 0;
    const indexed_triangle_set cut  = cut_mesh_at_steps(sheet, region, sampler, STEP_W, GAP, 0.f, &source, &cuts);

    CHECK(cuts > 0);
    CHECK(cut.indices.size() > sheet.indices.size());
    REQUIRE(source.size() == cut.indices.size());
    CHECK(sheet_is_manifold(cut, SIZE));

    // Displace along the cut mesh's own area-weighted vertex normals, as the bake does.
    std::vector<Vec3f> normal(cut.vertices.size(), Vec3f::Zero());
    for (const auto &t : cut.indices) {
        const Vec3f &a = cut.vertices[size_t(t[0])], &b = cut.vertices[size_t(t[1])], &c = cut.vertices[size_t(t[2])];
        const Vec3f  n = (b - a).cross(c - a);
        CHECK(n.z() > 0.f); // nothing inverted, nothing degenerate
        for (int k = 0; k < 3; ++k)
            normal[size_t(t[k])] += n;
    }
    indexed_triangle_set displaced = cut;
    for (size_t v = 0; v < displaced.vertices.size(); ++v) {
        const Vec3f n = normal[v].normalized();
        displaced.vertices[v] += n * sampler(cut.vertices[v], n);
    }

    // The defining property of the cut: a triangle that spans both heights is a wall, and a wall stands
    // within the seam gap of a step. Anywhere else a mixed triangle would be a ramp - exactly what
    // refinement leaves and the cutter is there to remove.
    const auto near_step = [&](const Vec3f &p) {
        const float h = posts(p.x(), p.y());
        for (int k = 0; k < 8; ++k) {
            const float ang = float(k) * float(M_PI) / 4.f;
            if (posts(p.x() + GAP * std::cos(ang), p.y() + GAP * std::sin(ang)) != h)
                return true;
        }
        return false;
    };
    size_t walls = 0, ramps = 0;
    for (const auto &t : displaced.indices) {
        const float z0 = displaced.vertices[size_t(t[0])].z(), z1 = displaced.vertices[size_t(t[1])].z(),
                    z2 = displaced.vertices[size_t(t[2])].z();
        if (std::abs(z0 - z1) < 1e-4f && std::abs(z1 - z2) < 1e-4f)
            continue; // flat: on one level
        bool wall = true;
        for (int k = 0; k < 3; ++k)
            wall = wall && near_step(cut.vertices[size_t(t[k])]);
        (wall ? walls : ramps)++;
    }
    CHECK(walls > 0);
    CHECK(ramps == 0);
}

TEST_CASE("TextureDisplacement: the step cutter passes a smooth field through untouched", "[TextureDisplacement]")
{
    // A wide hill: its mid-level contour runs through the sheet, but nowhere is it a step, so there is
    // nothing to cut - refinement is the right tool for it.
    const HeightFieldSampler   hill = [](const Vec3f &p, const Vec3f &) {
        const float r2 = (p.x() - 3.f) * (p.x() - 3.f) + (p.y() - 3.f) * (p.y() - 3.f);
        return 0.4f * std::exp(-r2 / 3.f);
    };
    const indexed_triangle_set sheet = make_flat_sheet(6.f, 1.f);
    const std::vector<uint8_t> region(sheet.indices.size(), 1);

    std::vector<int>           source;
    size_t                     cuts = 0;
    const indexed_triangle_set out  = cut_mesh_at_steps(sheet, region, hill, 0.05f, 0.075f, 0.f, &source, &cuts);

    CHECK(cuts == 0);
    CHECK(out.indices.size() == sheet.indices.size());
    CHECK(out.vertices.size() == sheet.vertices.size());
    REQUIRE(source.size() == sheet.indices.size());
    for (size_t i = 0; i < source.size(); ++i)
        CHECK(source[i] == int(i));
}

TEST_CASE("TextureDisplacement: the step cutter leaves features at the step's own scale to refinement",
          "[TextureDisplacement]")
{
    // Square posts filling the middle half of each cell, binary and sharp at every crossing - the only
    // difference between the two is the pitch (offset so no post edge lies along a mesh edge). Posts a
    // step width or so across have no pure interior for a seam copy to land in, and cutting them would
    // double the triangles for walls no bigger than the blur.
    constexpr float STEP_W = 0.05f;
    const float     pitch  = GENERATE(0.12f, 0.6f);
    const auto      posts  = [pitch](const Vec3f &p, const Vec3f &) {
        const float fx = std::fmod(p.x() + 0.17f, pitch) / pitch, fy = std::fmod(p.y() + 0.31f, pitch) / pitch;
        return (fx > 0.25f && fx < 0.75f && fy > 0.25f && fy < 0.75f) ? 0.4f : 0.f;
    };
    const indexed_triangle_set sheet = make_flat_sheet(6.f, 1.f);
    const std::vector<uint8_t> region(sheet.indices.size(), 1);

    size_t                     cuts = 0;
    const indexed_triangle_set out  = cut_mesh_at_steps(sheet, region, posts, STEP_W, STEP_W, 0.f, nullptr, &cuts);

    if (0.5f * pitch < 2.25f * STEP_W) {
        CHECK(cuts == 0);
        CHECK(out.indices.size() == sheet.indices.size());
    } else {
        CHECK(cuts > 0);
        CHECK(sheet_is_manifold(out, 6.f));
    }
}

TEST_CASE("TextureDisplacement: the step cutter only cuts inside the region", "[TextureDisplacement]")
{
    // A single step at x = 3 across the whole sheet, but only the left half is painted.
    const HeightFieldSampler   stripe = [](const Vec3f &p, const Vec3f &) { return p.x() > 3.f ? 0.4f : 0.f; };
    const indexed_triangle_set sheet  = make_flat_sheet(6.f, 1.f);

    SECTION("an empty region is a no-op")
    {
        const std::vector<uint8_t> none(sheet.indices.size(), 0);
        size_t                     cuts = 0;
        const indexed_triangle_set out  = cut_mesh_at_steps(sheet, none, stripe, 0.05f, 0.075f, 0.f, nullptr, &cuts);
        CHECK(cuts == 0);
        CHECK(out.indices.size() == sheet.indices.size());
    }

    SECTION("unpainted triangles away from the paint are untouched")
    {
        std::vector<uint8_t> region(sheet.indices.size(), 0);
        for (size_t t = 0; t < sheet.indices.size(); ++t) {
            const auto &f = sheet.indices[t];
            const float cy = (sheet.vertices[size_t(f[0])].y() + sheet.vertices[size_t(f[1])].y() +
                              sheet.vertices[size_t(f[2])].y()) / 3.f;
            region[t] = cy < 3.f ? 1 : 0; // paint the lower half
        }
        std::vector<int>           source;
        size_t                     cuts = 0;
        const indexed_triangle_set out  = cut_mesh_at_steps(sheet, region, stripe, 0.05f, 0.075f, 0.f, &source, &cuts);
        CHECK(cuts > 0);
        CHECK(sheet_is_manifold(out, 6.f));
        // An unpainted triangle that shares no edge with a painted one comes out exactly as it went in.
        std::vector<int> descendants(sheet.indices.size(), 0);
        for (int s : source)
            ++descendants[size_t(s)];
        for (size_t t = 0; t < sheet.indices.size(); ++t) {
            const auto &f = sheet.indices[t];
            float       ymin = 6.f;
            for (int k = 0; k < 3; ++k)
                ymin = std::min(ymin, sheet.vertices[size_t(f[k])].y());
            if (ymin > 3.f) // strictly above the painted half, so no shared edge with it
                CHECK(descendants[t] == 1);
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Texture smoothing
// ---------------------------------------------------------------------------------------------

TEST_CASE("TextureDisplacement: texture smoothing is a wrapped box blur whatever the radius", "[TextureDisplacement]")
{
    // A small pseudo-random grey image, encoded through the PNG writer so decode_height_texture() takes
    // its normal path. The expected result is the plain definition of the blur - two passes of a
    // (2r+1) box, horizontal then vertical, wrapping at the edges - which the sliding-window
    // implementation must reproduce byte for byte.
    const size_t w = 37, h = 23;
    std::vector<uint8_t> src(w * h);
    uint32_t seed = 12345;
    for (uint8_t &p : src) { seed = seed * 1664525u + 1013904223u; p = uint8_t(seed >> 24); }
    const boost::filesystem::path tmp_path = boost::filesystem::temp_directory_path()
        / boost::filesystem::unique_path("texdisp_test_%%%%%%%%.png");
    REQUIRE(Slic3r::png::write_gray_to_file(tmp_path.string(), w, h, src));
    std::vector<unsigned char> bytes;
    {
        std::ifstream ifs(tmp_path.string(), std::ios::binary);
        bytes.assign(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>());
    }
    boost::system::error_code ec;
    boost::filesystem::remove(tmp_path, ec);
    REQUIRE_FALSE(bytes.empty());

    TextureDisplacementLayer layer;
    layer.image_data = std::make_shared<std::vector<unsigned char>>(std::move(bytes));
    // radius = smoothing * 0.05 * min(w, h) = 0.05 * 23 * smoothing; 1.0 gives 1.15 -> whole radius 2,
    // cross-faded 57.5 % toward the blurred image (see smooth_height_pixels()).
    layer.smoothing = 1.f;
    const DecodedHeightTexture tex = decode_height_texture(layer);
    REQUIRE(tex.width == int(w));
    REQUIRE(tex.height == int(h));

    const int   radius = 2;
    const float mixf   = std::clamp((0.05f * 23.f) / 2.f, 0.f, 1.f);
    auto box = [&](std::vector<uint8_t> px) {
        const int   window = 2 * radius + 1;
        const float inv    = 1.f / float(window);
        std::vector<uint8_t> t2(px.size());
        for (int pass = 0; pass < 2; ++pass) {
            for (int y = 0; y < int(h); ++y)
                for (int x = 0; x < int(w); ++x) {
                    float s = 0.f;
                    for (int k = -radius; k <= radius; ++k) s += float(px[size_t(y) * w + size_t(((x + k) % int(w) + int(w)) % int(w))]);
                    t2[size_t(y) * w + size_t(x)] = uint8_t(std::lround(s * inv));
                }
            for (int x = 0; x < int(w); ++x)
                for (int y = 0; y < int(h); ++y) {
                    float s = 0.f;
                    for (int k = -radius; k <= radius; ++k) s += float(t2[size_t(((y + k) % int(h) + int(h)) % int(h)) * w + size_t(x)]);
                    px[size_t(y) * w + size_t(x)] = uint8_t(std::lround(s * inv));
                }
        }
        return px;
    };
    const std::vector<uint8_t> blurred = box(src);
    size_t mismatches = 0;
    for (size_t i = 0; i < w * h; ++i) {
        const uint8_t expected = uint8_t(std::lround(float(src[i]) + (float(blurred[i]) - float(src[i])) * mixf));
        mismatches += tex.pixels[i] != expected;
    }
    CHECK(mismatches == 0);

    // Asking again with the same smoothing is served from the cache and must be identical.
    const DecodedHeightTexture again = decode_height_texture(layer);
    CHECK(again.pixels == tex.pixels);
}

// ---------------------------------------------------------------------------------------------
// Edge flips along the height field (v2 pipeline)
// ---------------------------------------------------------------------------------------------

TEST_CASE("TextureDisplacement: edge flips lay a stepped field's wall along the grid's diagonals", "[TextureDisplacement]")
{
    // A regular grid crossed by a step at 30 degrees. Before flipping, the step's wall zigzags: many
    // triangles have corners on both sides of it. Flipping each quad's diagonal to follow the step must
    // cut that count down, without changing the triangle count, the winding, or the manifoldness.
    const indexed_triangle_set sheet = make_flat_sheet(6.f, 0.2f);
    const float                c = std::cos(0.5236f), s = std::sin(0.5236f);
    const auto                 side_of = [&](const Vec3f &p) { return c * p.x() + s * p.y() > 3.5f; };
    const TextureBake::HeightSampleFn field = [&](const Vec3f &p, const Vec3f &, const Vec3f &) {
        return side_of(p) ? 0.4f : 0.f;
    };
    const auto mixed = [&](const TextureBake::TriSoup &g) {
        size_t n = 0;
        for (size_t t = 0; t < g.triangle_count(); ++t) {
            const bool a = side_of(g.pos[t * 3]), b = side_of(g.pos[t * 3 + 1]), d = side_of(g.pos[t * 3 + 2]);
            n += (a != b || b != d);
        }
        return n;
    };

    const TextureBake::TriSoup    before = TextureBake::to_soup(sheet);
    const TextureBake::FlipResult after  = TextureBake::flip_edges_to_height(before, {}, field, TextureBake::FlipSettings{}, {});

    CHECK(after.flipped > 0);
    REQUIRE(after.geometry.triangle_count() == before.triangle_count());
    const size_t mixed_before = mixed(before), mixed_after = mixed(after.geometry);
    CHECK(mixed_after < mixed_before);
    // Every triangle still faces up, and none collapsed.
    for (size_t t = 0; t < after.geometry.triangle_count(); ++t) {
        const Vec3f n = (after.geometry.pos[t * 3 + 1] - after.geometry.pos[t * 3]).cross(after.geometry.pos[t * 3 + 2] - after.geometry.pos[t * 3]);
        CHECK(n.z() > 1e-6f);
    }
    // Manifold: rebuild an indexed mesh from the soup and count edge uses.
    indexed_triangle_set rebuilt;
    std::map<std::tuple<int, int, int>, int> ids;
    for (const Vec3f &p : after.geometry.pos) {
        const auto key = std::make_tuple(int(std::lround(p.x() * 1e4)), int(std::lround(p.y() * 1e4)), int(std::lround(p.z() * 1e4)));
        auto it = ids.find(key);
        if (it == ids.end()) { it = ids.emplace(key, int(rebuilt.vertices.size())).first; rebuilt.vertices.push_back(p); }
        (void) it;
    }
    for (size_t t = 0; t < after.geometry.triangle_count(); ++t) {
        int idx[3];
        for (int k = 0; k < 3; ++k) {
            const Vec3f &p = after.geometry.pos[t * 3 + size_t(k)];
            idx[k] = ids.at(std::make_tuple(int(std::lround(p.x() * 1e4)), int(std::lround(p.y() * 1e4)), int(std::lround(p.z() * 1e4))));
        }
        rebuilt.indices.emplace_back(idx[0], idx[1], idx[2]);
    }
    CHECK(sheet_is_manifold(rebuilt, 6.f));
}

// ---------------------------------------------------------------------------------------------
// Automatic resolution (v2 pipeline)
// ---------------------------------------------------------------------------------------------

TEST_CASE("TextureDisplacement: automatic resolution follows the model's size", "[TextureDisplacement]")
{
    // A 20 mm cube: diagonal 34.64 mm, so diagonal / 250 = 0.1386 mm, rounded up to 0.14.
    const indexed_triangle_set cube = its_make_cube(20.f, 20.f, 20.f);
    TextureDisplacementLayer   layer;
    layer.image_data   = make_checkerboard_png(16, 16);
    layer.tiling_scale = 8.f;

    SECTION("edge from the diagonal, budget the standard 750 k")
    {
        const V2Resolution rec = recommend_v2_resolution(cube, { layer });
        CHECK_THAT(rec.edge_mm, WithinAbs(0.14f, 1e-4f));
        CHECK(rec.budget_k == 750);
        CHECK_THAT(rec.texel_mm, WithinAbs(0.5f, 1e-4f)); // reported for the panel
    }

    SECTION("the world transform scales the diagonal")
    {
        const V2Resolution rec = recommend_v2_resolution(cube, { layer }, Transform3d(Eigen::Scaling(3.0)));
        CHECK_THAT(rec.edge_mm, WithinAbs(0.42f, 1e-4f)); // 103.9 / 250 = 0.4157 -> 0.42
    }

    SECTION("a tiny model stops at the 0.05 mm floor")
    {
        const indexed_triangle_set small = its_make_cube(2.f, 2.f, 2.f);
        const V2Resolution         rec   = recommend_v2_resolution(small, { layer });
        CHECK_THAT(rec.edge_mm, WithinAbs(0.05f, 1e-4f));
    }

    SECTION("the texture's sharpness class is still measured")
    {
        const TextureDetail sharp = analyze_texture_detail(layer);
        CHECK_THAT(sharp.pixels_per_edge, WithinAbs(1.f, 1e-6f));
        TextureDisplacementLayer flat;
        flat.image_data = make_flat_gray_png(128, 16, 16);
        CHECK_THAT(analyze_texture_detail(flat).pixels_per_edge, WithinAbs(4.f, 1e-6f));
    }
}

TEST_CASE("TextureDisplacement: the default pipeline colours its rebuilt triangles where painted", "[TextureDisplacement]")
{
    // The same quad and 2x2 colour texture as the classic-path colour test, but baked through the
    // one-run pipeline, which rebuilds the topology: every output triangle has to be coloured from the
    // texture at its own position, and only those over the painted half of the quad.
    const indexed_triangle_set quad = color_test_quad();
    TextureDisplacementLayer   layer;
    layer.slot              = 0;
    layer.image_data        = make_rgb_png_2x2();
    layer.color_enabled     = true;
    layer.depth_mm          = 0.f;
    layer.tiling_scale      = 10.f; // one tile over the whole quad, so all four texels show
    layer.projection_method = TextureProjectionMethod::Triplanar;

    TriangleMesh     mesh(quad);
    TriangleSelector selector(mesh);
    selector.set_facet(0, EnforcerBlockerType::ENFORCER); // the lower-right half only
    TextureDisplacementFacetsData facets;
    facets[0] = selector.serialize();

    const std::array<Vec3f, 3> palette = { Vec3f(1, 0, 0), Vec3f(0, 1, 0), Vec3f(0, 0, 1) };
    TextureColorRequest        request;
    std::vector<uint8_t>       triangle_color;
    request.out_triangle = &triangle_color;
    request.quantize     = [&palette](const Vec3f &rgb) {
        int   best = 0;
        float bd   = std::numeric_limits<float>::max();
        for (int i = 0; i < 3; ++i)
            if (const float d = (palette[size_t(i)] - rgb).squaredNorm(); d < bd) { bd = d; best = i; }
        return best;
    };

    TextureDisplacementOptions options;
    options.pipeline_v2        = true;
    options.v2_refine_mm       = 1.f;
    options.v2_max_triangles_k = 0; // no simplification: a plain refined quad
    const indexed_triangle_set out = build_texture_displacement(quad, { layer }, facets, options, {}, &request);

    REQUIRE(out.indices.size() > 2);
    REQUIRE(triangle_color.size() == out.indices.size());
    size_t coloured = 0, plain = 0;
    std::set<uint8_t> distinct;
    for (size_t i = 0; i < out.indices.size(); ++i) {
        const stl_triangle_vertex_indices &t = out.indices[i];
        const Vec3f c = (out.vertices[size_t(t[0])] + out.vertices[size_t(t[1])] + out.vertices[size_t(t[2])]) / 3.f;
        const bool  painted_side = c.y() < c.x(); // below the diagonal: triangle 0 of the quad
        if (triangle_color[i] > 0) { ++coloured; distinct.insert(triangle_color[i]); CHECK(painted_side); }
        else                       { ++plain; CHECK_FALSE(painted_side); }
    }
    CHECK(coloured > 0);
    CHECK(plain > 0);
    CHECK(distinct.size() >= 2); // the texture has four colours across the quad, so one side sees several
}

// Per chart of an unwrap: whether it is a topological disk (V - E + F = 1, one boundary loop), the only shape a
// single island can be laid flat from.
static std::vector<bool> unwrap_charts_are_disks(const PatchUnwrap &u)
{
    std::vector<std::set<int>>                 verts(size_t(u.chart_count));
    std::vector<std::set<std::pair<int, int>>> edges(size_t(u.chart_count));
    std::vector<int>                           faces(size_t(u.chart_count), 0);
    for (const stl_triangle_vertex_indices &tri : u.indices) {
        const int c = u.vertex_chart[size_t(tri[0])];
        ++faces[size_t(c)];
        for (int i = 0; i < 3; ++i) {
            verts[size_t(c)].insert(tri[i]);
            edges[size_t(c)].insert({ std::min(tri[i], tri[(i + 1) % 3]), std::max(tri[i], tri[(i + 1) % 3]) });
        }
    }
    // Boundary loops per chart: flood the boundary edges' vertices.
    std::vector<std::vector<int>> boundary_adj(u.uvs.size());
    for (const auto &[a, b] : u.boundary_edges) {
        boundary_adj[size_t(a)].push_back(b);
        boundary_adj[size_t(b)].push_back(a);
    }
    std::vector<int>  loops(size_t(u.chart_count), 0);
    std::vector<bool> seen(u.uvs.size(), false);
    for (size_t v = 0; v < u.uvs.size(); ++v) {
        if (seen[v] || boundary_adj[v].empty())
            continue;
        ++loops[size_t(u.vertex_chart[v])];
        std::vector<int> stack{ int(v) };
        seen[v] = true;
        while (!stack.empty()) {
            const int x = stack.back();
            stack.pop_back();
            for (const int y : boundary_adj[size_t(x)])
                if (!seen[size_t(y)]) {
                    seen[size_t(y)] = true;
                    stack.push_back(y);
                }
        }
    }
    std::vector<bool> disks(size_t(u.chart_count));
    for (int c = 0; c < u.chart_count; ++c)
        disks[size_t(c)] = int(verts[size_t(c)].size()) - int(edges[size_t(c)].size()) + faces[size_t(c)] == 1 &&
                           loops[size_t(c)] == 1;
    return disks;
}

TEST_CASE("TextureDisplacement: the default pipeline bakes an unwrap (LSCM) layer", "[TextureDisplacement]")
{
    // The one-run pipeline samples per point, and an unwrap has no per-point formula, so unwrap layers
    // used to be dropped from it altogether: the bake moved nothing, and the job then cleared the paint
    // as if it had baked - "paint, unwrap, bake, and the painted region just disappears".
    const indexed_triangle_set cube = its_make_cube(10., 10., 10.);
    TextureDisplacementLayer   layer;
    layer.slot              = 0;
    layer.image_data        = make_flat_gray_png(255);
    layer.depth_mm          = 0.5f;
    layer.tiling_scale      = 10.f;
    layer.projection_method = TextureProjectionMethod::LSCM;

    TextureDisplacementFacetsData facets;
    facets[0] = paint_whole_mesh(cube);

    TextureDisplacementOptions options;
    options.pipeline_v2        = true;
    options.v2_refine_mm       = 2.f;
    options.v2_max_triangles_k = 0;
    const indexed_triangle_set out = build_texture_displacement(cube, { layer }, facets, options);
    REQUIRE(!out.vertices.empty());

    // A flat white texture at depth 0.5 pushes the faces out by 0.5, so the bounding box grows on
    // every side (face interiors move the full depth; only corners, moving along their blended normal,
    // move less). Before the fix it stayed exactly [0, 10].
    Vec3f lo = out.vertices.front(), hi = lo;
    for (const Vec3f &v : out.vertices) {
        lo = lo.cwiseMin(v);
        hi = hi.cwiseMax(v);
    }
    CHECK(lo.x() < -0.4f);
    CHECK(hi.z() > 10.4f);
}

TEST_CASE("Per-corner LSCM UVs give each triangle its own island's placement", "[TextureDisplacement]")
{
    // compute_lscm_uvs() has to collapse a seam vertex onto one chart, because displacement is
    // per vertex. Everything that samples per *triangle* must not: a cube corner belongs to three
    // faces, so the collapse handed a triangle at an unjoined seam a neighbouring island's placement.
    // That showed up as one visibly skewed triangle per face, and as every island's texture following
    // the lowest-numbered island whenever it was dragged.
    const indexed_triangle_set cube   = its_make_cube(10., 10., 10.);
    const PatchUnwrap          unwrap = compute_patch_unwrap(cube, 30.f, 0.f);
    REQUIRE(unwrap.chart_count == 6);
    // The map back to the patch's own triangle order, without which there are no per-corner UVs.
    REQUIRE(unwrap.source_face.size() == unwrap.indices.size());

    TextureDisplacementLayer layer;
    layer.projection_method   = TextureProjectionMethod::LSCM;
    layer.lscm_seam_angle_deg = 30.f;
    layer.islands             = compute_connected_net(unwrap);
    REQUIRE(layer.islands.size() == 6);
    // Move one island by hand. Its neighbours must stay exactly where they were.
    layer.islands[0].offset += Vec2f(37.f, -19.f);

    const std::vector<Vec2f> corner = compute_lscm_corner_uvs(cube, layer);
    REQUIRE(corner.size() == cube.indices.size() * 3);

    for (size_t t = 0; t < unwrap.indices.size(); ++t) {
        const size_t f = size_t(unwrap.source_face[t]);
        REQUIRE(f < cube.indices.size());
        // A triangle lies in exactly one chart, so any of its corners names that chart.
        const int c = unwrap.vertex_chart[size_t(unwrap.indices[t][0])];
        for (int k = 0; k < 3; ++k) {
            const Vec2f want = apply_island_transform(unwrap.uvs[size_t(unwrap.indices[t][k])], c, unwrap, layer.islands);
            CHECK_THAT(corner[f * 3 + size_t(k)].x(), WithinAbs(want.x(), 1e-4));
            CHECK_THAT(corner[f * 3 + size_t(k)].y(), WithinAbs(want.y(), 1e-4));
        }
    }

    // And the per-vertex path must disagree somewhere - otherwise this test proves nothing, because
    // the bug it guards against is precisely that the two were the same thing.
    const std::vector<Vec2f> per_vertex = compute_lscm_uvs(cube, layer);
    REQUIRE(per_vertex.size() == cube.vertices.size());
    bool differs = false;
    for (size_t f = 0; f < cube.indices.size() && !differs; ++f)
        for (int k = 0; k < 3; ++k)
            if ((corner[f * 3 + size_t(k)] - per_vertex[size_t(cube.indices[f][k])]).norm() > 1e-3f)
                differs = true;
    CHECK(differs);
}

TEST_CASE("The Cylindrical/Spherical patch frame is the bake's own, so a preview can share it", "[TextureDisplacement]")
{
    // The fast preview reconstructs these two projections in the fragment shader and needs the very
    // centroid and axis the bake wraps around - a patch-only normal average would sometimes quantize
    // to a different world axis and wrap the texture the other way round.
    const indexed_triangle_set cube    = its_make_cube(10., 10., 10.);
    const std::vector<Vec3f>   normals = texture_displacement_vertex_normals(cube);
    REQUIRE(normals.size() == cube.vertices.size());

    // Just the +X face: its average normal is +X, so the cylinder axis must be a world axis
    // perpendicular to it, and the centroid must sit on that face.
    indexed_triangle_set face;
    face.vertices = cube.vertices;
    for (const stl_triangle_vertex_indices &t : cube.indices) {
        const Vec3f n = (cube.vertices[size_t(t[1])] - cube.vertices[size_t(t[0])])
                            .cross(cube.vertices[size_t(t[2])] - cube.vertices[size_t(t[0])]);
        if (n.normalized().x() > 0.99f)
            face.indices.push_back(t);
    }
    REQUIRE(face.indices.size() == 2);

    Vec3f center, axis, average_normal;
    texture_displacement_patch_frame(face, normals, center, axis, average_normal);
    CHECK_THAT(center.x(), WithinAbs(10., 1e-4));
    CHECK_THAT(std::abs(axis.x()), WithinAbs(0., 1e-4)); // never the face's own normal direction
    CHECK_THAT(axis.norm(), WithinAbs(1., 1e-4));

    // An empty patch must not divide by zero; it falls back to +Z.
    texture_displacement_patch_frame(indexed_triangle_set{}, normals, center, axis, average_normal);
    CHECK_THAT(center.norm(), WithinAbs(0., 1e-6));
    CHECK_THAT(axis.z(), WithinAbs(1., 1e-6));
}

TEST_CASE("A cube unwraps into one island per face, laid out as a connected net", "[TextureDisplacement]")
{
    const indexed_triangle_set cube   = its_make_cube(10., 10., 10.);
    const PatchUnwrap          unwrap = compute_patch_unwrap(cube, 30.f, 0.f);
    // Two triangles per face, and every face edge but the diagonals is a 90 degree crease.
    REQUIRE(unwrap.chart_count == 6);

    const std::vector<TextureIsland> islands = compute_connected_net(unwrap);
    REQUIRE(islands.size() == 6);

    std::vector<std::array<Vec2f, 3>> placed;
    std::vector<int>                  placed_chart;
    for (const stl_triangle_vertex_indices &tri : unwrap.indices) {
        const int            c = unwrap.vertex_chart[size_t(tri[0])];
        std::array<Vec2f, 3> t;
        for (int k = 0; k < 3; ++k)
            t[size_t(k)] = apply_island_transform(unwrap.uvs[size_t(tri[k])], c, unwrap, islands);
        placed.push_back(t);
        placed_chart.push_back(c);
    }

    SECTION("no two faces overlap")
    {
        // A 10 mm cube: the six 100 mm2 faces laid flat without overlap cover 600 mm2, which triangles overlapping
        // anywhere could not add up to within their joint bounding box... so test it directly, by sampling.
        float lo_x = 1e9f, lo_y = 1e9f, hi_x = -1e9f, hi_y = -1e9f;
        for (const auto &t : placed)
            for (const Vec2f &p : t) {
                lo_x = std::min(lo_x, p.x());
                lo_y = std::min(lo_y, p.y());
                hi_x = std::max(hi_x, p.x());
                hi_y = std::max(hi_y, p.y());
            }
        const auto inside = [](const std::array<Vec2f, 3> &t, const Vec2f &p) {
            const auto cross = [](const Vec2f &a, const Vec2f &b) { return a.x() * b.y() - a.y() * b.x(); };
            const float d0 = cross(t[1] - t[0], p - t[0]), d1 = cross(t[2] - t[1], p - t[1]), d2 = cross(t[0] - t[2], p - t[2]);
            return (d0 > 1e-3f && d1 > 1e-3f && d2 > 1e-3f) || (d0 < -1e-3f && d1 < -1e-3f && d2 < -1e-3f);
        };
        int overlapping = 0;
        for (float x = lo_x + 0.13f; x < hi_x; x += 0.5f)
            for (float y = lo_y + 0.17f; y < hi_y; y += 0.5f) {
                int covering = 0;
                for (const auto &t : placed)
                    covering += inside(t, Vec2f(x, y)) ? 1 : 0;
                overlapping += covering > 1 ? 1 : 0;
            }
        CHECK(overlapping == 0);
    }

    SECTION("every face shares an edge with another face of the net")
    {
        std::vector<bool> joined(6, false);
        for (size_t i = 0; i < placed.size(); ++i)
            for (size_t j = 0; j < placed.size(); ++j) {
                if (placed_chart[i] == placed_chart[j])
                    continue;
                int shared_corners = 0;
                for (const Vec2f &p : placed[i])
                    for (const Vec2f &q : placed[j])
                        shared_corners += (p - q).norm() < 1e-3f ? 1 : 0;
                if (shared_corners >= 2)
                    joined[size_t(placed_chart[i])] = true;
            }
        for (int c = 0; c < 6; ++c)
            CHECK(joined[size_t(c)]);
    }
}

TEST_CASE("Unwrapping a closed cylinder cuts its side until every island is a disk", "[TextureDisplacement]")
{
    // The side is smooth, so the seam angle alone leaves it as one ring-shaped chart, which cannot be laid flat.
    const indexed_triangle_set cylinder = its_make_cylinder(5., 20., 2. * PI / 36.);
    const PatchUnwrap          unwrap   = compute_patch_unwrap(cylinder, 30.f, 0.f);
    REQUIRE(unwrap.chart_count >= 4); // two caps and at least two side pieces

    const std::vector<bool> disks = unwrap_charts_are_disks(unwrap);
    for (int c = 0; c < unwrap.chart_count; ++c)
        CHECK(disks[size_t(c)]);
}

TEST_CASE("A UV edit on one copy of a seam vertex leaves its other copies alone", "[TextureDisplacement]")
{
    const indexed_triangle_set cube   = its_make_cube(10., 10., 10.);
    PatchUnwrap                unwrap = compute_patch_unwrap(cube, 30.f, 0.f);

    // A cube corner has a copy in each of the three faces around it.
    const int                 edited_copy = 0;
    const int                 mesh_vertex = unwrap.source_vertex[size_t(edited_copy)];
    std::vector<int>          other_copies;
    for (size_t i = 0; i < unwrap.uvs.size(); ++i)
        if (int(i) != edited_copy && unwrap.source_vertex[i] == mesh_vertex)
            other_copies.push_back(int(i));
    REQUIRE_FALSE(other_copies.empty());

    const std::vector<Vec2f> before = unwrap.uvs;
    const Vec2f              target = before[size_t(edited_copy)] + Vec2f(3.f, -2.f);
    const std::vector<bool>  edited = apply_lscm_uv_overrides(unwrap, { { lscm_uv_override_key(edited_copy), target } });

    CHECK_THAT(unwrap.uvs[size_t(edited_copy)].x(), WithinAbs(target.x(), 1e-6));
    CHECK_THAT(unwrap.uvs[size_t(edited_copy)].y(), WithinAbs(target.y(), 1e-6));
    CHECK(edited[size_t(edited_copy)]);
    for (const int i : other_copies) {
        CHECK_THAT(unwrap.uvs[size_t(i)].x(), WithinAbs(before[size_t(i)].x(), 1e-6));
        CHECK_THAT(unwrap.uvs[size_t(i)].y(), WithinAbs(before[size_t(i)].y(), 1e-6));
        CHECK_FALSE(edited[size_t(i)]);
    }
}

TEST_CASE("TextureDisplacement: moving the model about the plate does not move the texture on it", "[TextureDisplacement]")
{
    // The bake projects in world millimetres but anchored at the volume's origin, so an instance
    // translated across the plate bakes exactly the relief the same instance bakes at the origin. The
    // classic path keeps the topology, so the comparison is vertex by vertex.
    const indexed_triangle_set cube = subdivide_mesh_uniform(its_make_cube(10.f, 10.f, 10.f), 1.f, 6);
    TextureDisplacementLayer   layer;
    layer.slot         = 0;
    layer.image_data   = make_checkerboard_png(16, 16);
    layer.tiling_scale = 4.f;
    layer.depth_mm     = 0.5f;
    TextureDisplacementFacetsData facets;
    facets[0] = paint_whole_mesh(cube);

    const indexed_triangle_set at_origin = build_texture_displacement(cube, { layer }, facets, classic_options());
    const indexed_triangle_set moved     = build_texture_displacement(cube, { layer }, facets, classic_options(), {}, nullptr,
                                                                      Transform3d(Eigen::Translation3d(123.4, -56.7, 0.0)));
    REQUIRE(moved.vertices.size() == at_origin.vertices.size());
    float max_diff = 0.f;
    for (size_t i = 0; i < moved.vertices.size(); ++i)
        max_diff = std::max(max_diff, (moved.vertices[i] - at_origin.vertices[i]).norm());
    CHECK(max_diff < 1e-3f);
}

// Longest edge over the shortest altitude: 1.15 for an equilateral triangle, 2 for a right isosceles
// one, unbounded for a needle.
static float triangle_aspect(const Vec3f &a, const Vec3f &b, const Vec3f &c)
{
    const float longest    = std::max({ (b - a).norm(), (c - b).norm(), (a - c).norm() });
    const float twice_area = (b - a).cross(c - a).norm();
    return twice_area > 0.f ? longest * longest / twice_area : std::numeric_limits<float>::infinity();
}

TEST_CASE("TextureDisplacement: baking a second face leaves the first face's relief untouched", "[TextureDisplacement]")
{
    // Paint the top of a cube and bake, then paint the front of the *result* and bake again, the way
    // the gizmo does (each bake replaces the mesh and clears the baked paint). The top is unpainted the
    // second time round, so it is excluded from refinement and pinned by the displacement: every
    // vertex of its relief must still be there, in the same number of triangles, and no needle may
    // appear on it.
    const indexed_triangle_set cube = subdivide_mesh_uniform(its_make_cube(20., 20., 20.), 2.f, 6);

    TextureDisplacementLayer layer;
    layer.slot         = 0;
    layer.image_data   = make_checkerboard_png(16, 16);
    layer.tiling_scale = 5.f;
    layer.depth_mm     = 0.5f;

    TextureDisplacementOptions options;
    options.pipeline_v2        = true;
    options.v2_refine_mm       = 0.5f;
    options.v2_max_triangles_k = 0; // no simplification

    // Paints exactly the triangles whose three corners satisfy `on_face` - no brush spill.
    const auto paint_where = [](const indexed_triangle_set &mesh, auto on_face) {
        const TriangleMesh tm(mesh);
        TriangleSelector   selector(tm);
        for (size_t f = 0; f < mesh.indices.size(); ++f) {
            const stl_triangle_vertex_indices &t = mesh.indices[f];
            if (on_face(mesh.vertices[size_t(t[0])]) && on_face(mesh.vertices[size_t(t[1])]) &&
                on_face(mesh.vertices[size_t(t[2])]))
                selector.set_facet(int(f), EnforcerBlockerType::ENFORCER);
        }
        return selector.serialize();
    };
    const auto on_top   = [](const Vec3f &v) { return v.z() > 19.9f; };
    const auto on_front = [](const Vec3f &v) { return v.y() < 0.1f; };

    // The top region of a result: its vertices, and its triangles' count and worst aspect ratio.
    struct TopRegion
    {
        std::vector<Vec3f> vertices;
        size_t             triangles  = 0;
        float              max_aspect = 0.f;
    };
    const auto top_region = [&on_top](const indexed_triangle_set &mesh) {
        TopRegion r;
        for (const Vec3f &v : mesh.vertices)
            if (on_top(v))
                r.vertices.push_back(v);
        for (const stl_triangle_vertex_indices &t : mesh.indices) {
            const Vec3f &a = mesh.vertices[size_t(t[0])], &b = mesh.vertices[size_t(t[1])], &c = mesh.vertices[size_t(t[2])];
            if (!on_top(a) || !on_top(b) || !on_top(c))
                continue;
            ++r.triangles;
            r.max_aspect = std::max(r.max_aspect, triangle_aspect(a, b, c));
        }
        return r;
    };

    TextureDisplacementFacetsData facets{};
    facets[0] = paint_where(cube, on_top);
    const indexed_triangle_set first = build_texture_displacement(cube, { layer }, facets, options);
    REQUIRE(first.indices.size() > cube.indices.size());
    const TopRegion top_before = top_region(first);
    REQUIRE(top_before.triangles > 0);
    // The relief really is there: the checkerboard raises part of the top by the full depth.
    float top_z_max = 0.f;
    for (const Vec3f &v : top_before.vertices)
        top_z_max = std::max(top_z_max, v.z());
    REQUIRE(top_z_max > 20.3f);
    REQUIRE(top_before.max_aspect < 20.f);

    // Only the front of the baked mesh is painted for the second bake.
    facets[0] = paint_where(first, on_front);
    REQUIRE_FALSE(facets[0].triangles_to_split.empty());

    const auto check_top_untouched = [&](const indexed_triangle_set &second) {
        REQUIRE_FALSE(second.indices.empty()); // an aborted bake returns {}
        const TopRegion top_after = top_region(second);

        // Every vertex of the first relief still exists, at the same place. O(n*m) over a few
        // thousand vertices each, restricted to the top region on both sides.
        size_t missing = 0;
        Vec3f  first_missing = Vec3f::Zero();
        for (const Vec3f &v : top_before.vertices) {
            bool found = false;
            for (const Vec3f &w : top_after.vertices)
                if ((w - v).squaredNorm() <= 1e-6f) { // within 1e-3 mm
                    found = true;
                    break;
                }
            if (!found) {
                if (missing == 0)
                    first_missing = v;
                ++missing;
            }
        }
        INFO("first vertex of the top relief missing from the second bake: " << first_missing.transpose());
        CHECK(missing == 0);

        // Nothing was added to or taken from the top either - its triangles are excluded from the
        // refinement, and a rim edge it shares with the front was already at the refine length.
        CHECK(top_after.triangles == top_before.triangles);

        // And no needles: the worst triangle on the top is no worse than after the first bake.
        INFO("worst top-face aspect ratio after the second bake: " << top_after.max_aspect
             << ", after the first: " << top_before.max_aspect);
        CHECK(top_after.max_aspect < 20.f);
    };

    SECTION("bake mode: no simplification")
    {
        check_top_untouched(build_texture_displacement(first, { layer }, facets, options));
    }

    SECTION("export mode: simplification and T-junction repair run over the excluded region too")
    {
        // A budget far below the mesh forces the decimation (the locked top alone is over it, so it
        // only harvests flat faces) and with it the repair pass - the two stages that walk every face,
        // excluded ones included.
        TextureDisplacementOptions export_options = options;
        export_options.v2_max_triangles_k         = 1;
        check_top_untouched(build_texture_displacement(first, { layer }, facets, export_options));
    }
}

TEST_CASE("TextureDisplacement: a brush stroke smaller than a triangle displaces only the stroke", "[TextureDisplacement]")
{
    // A plain 12-triangle cube. The selector splits the top triangle under a 3 mm spherical brush,
    // so the painted pieces are far smaller than the triangle. The one-run pipeline used to include
    // the whole source triangle: the entire top face rose.
    const indexed_triangle_set    cube = its_make_cube(20.f, 20.f, 20.f);
    TextureDisplacementFacetsData facets;
    facets[0] = paint_top_disc(cube, 20.f, Vec2f(10.f, 10.f), 3.f);
    REQUIRE(TriangleSelector::has_facets(facets[0], EnforcerBlockerType::ENFORCER));

    TextureDisplacementLayer layer;
    layer.slot         = 0;
    layer.image_data   = make_flat_gray_png(255, 8, 8); // uniform full height: every painted point rises
    layer.tiling_scale = 4.f;
    layer.depth_mm     = 0.5f;
    TextureDisplacementOptions options;
    options.pipeline_v2        = true;
    options.v2_refine_mm       = 0.5f;
    options.v2_max_triangles_k = 0;
    const indexed_triangle_set out = build_texture_displacement(cube, { layer }, facets, options);
    REQUIRE(out.indices.size() > cube.indices.size());

    size_t raised_inside = 0, raised_outside = 0, outside = 0;
    for (const Vec3f &v : out.vertices) {
        if (v.z() < 19.9f)
            continue; // not the top face
        const float r = (Vec2f(v.x(), v.y()) - Vec2f(10.f, 10.f)).norm();
        if (r < 2.f && v.z() > 20.3f)
            ++raised_inside;
        if (r > 4.5f) {
            ++outside;
            if (v.z() > 20.01f)
                ++raised_outside;
        }
    }
    CHECK(raised_inside > 0);   // the stroke itself is displaced
    CHECK(outside > 0);
    CHECK(raised_outside == 0); // the rest of the face, inside the same source triangle, is not
}

TEST_CASE("TextureDisplacement: a texture of flat colours is told apart from a continuous one", "[TextureDisplacement]")
{
    // The verdict that decides whether a layer may use filament mixes: a checkerboard is two colours,
    // a ramp spreads over every level.
    TextureDisplacementLayer checker;
    checker.image_data = make_checkerboard_png(32, 32);
    CHECK(analyze_texture_detail(checker).flat_colors);

    std::vector<uint8_t> ramp(64 * 64);
    for (size_t y = 0; y < 64; ++y)
        for (size_t x = 0; x < 64; ++x)
            ramp[y * 64 + x] = uint8_t((x * 4 + y) & 255);
    const boost::filesystem::path tmp_path = boost::filesystem::temp_directory_path()
        / boost::filesystem::unique_path("texdisp_test_%%%%%%%%.png");
    REQUIRE(Slic3r::png::write_gray_to_file(tmp_path.string(), 64, 64, ramp));
    std::vector<unsigned char> bytes;
    {
        std::ifstream ifs(tmp_path.string(), std::ios::binary);
        bytes.assign(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>());
    }
    boost::system::error_code ec;
    boost::filesystem::remove(tmp_path, ec);
    TextureDisplacementLayer gradient;
    gradient.image_data = std::make_shared<std::vector<unsigned char>>(std::move(bytes));
    const TextureDetail d = analyze_texture_detail(gradient);
    CHECK_FALSE(d.flat_colors);
    CHECK(d.flat_share < 0.85f);
}

TEST_CASE("Each layer's texture is sampled only on its own painted area", "[TextureDisplacement]")
{
    // Two layers with different textures and depths, one painted on the cube's top, one on its -X side.
    const indexed_triangle_set cube = its_make_cube(10., 10., 10.);
    const TriangleMesh         cube_mesh(cube);
    const auto paint_facing = [&](const Vec3f &dir) {
        TriangleSelector selector(cube_mesh);
        for (int f = 0; f < int(cube.indices.size()); ++f) {
            const stl_triangle_vertex_indices &t = cube.indices[size_t(f)];
            const Vec3f n = (cube.vertices[size_t(t[1])] - cube.vertices[size_t(t[0])])
                                .cross(cube.vertices[size_t(t[2])] - cube.vertices[size_t(t[0])])
                                .normalized();
            if (n.dot(dir) > 0.99f)
                selector.set_facet(f, EnforcerBlockerType::ENFORCER);
        }
        return selector.serialize();
    };
    TextureDisplacementFacetsData facets{};
    facets[0] = paint_facing(Vec3f::UnitZ());
    facets[1] = paint_facing(-Vec3f::UnitX());

    TextureDisplacementLayer top;
    top.slot         = 0;
    top.depth_mm     = 1.0f;
    top.tiling_scale = 5.0f;
    top.image_data   = make_flat_gray_png(255);
    TextureDisplacementLayer side = top;
    side.slot       = 1;
    side.depth_mm   = 0.5f;
    side.image_data = make_flat_gray_png(64);

    const HeightFieldSampler both      = make_combined_displacement_sampler(cube, { top, side }, facets);
    const HeightFieldSampler top_only  = make_combined_displacement_sampler(cube, { top }, facets);
    const HeightFieldSampler side_only = make_combined_displacement_sampler(cube, { side }, facets);
    REQUIRE(both);
    REQUIRE(top_only);
    REQUIRE(side_only);

    const Vec3f on_top(5.f, 5.f, 10.f), on_side(0.f, 5.f, 5.f), unpainted(5.f, 10.f, 5.f);
    CHECK_THAT(both(on_top, Vec3f::UnitZ()), WithinAbs(top_only(on_top, Vec3f::UnitZ()), 1e-5f));
    CHECK_THAT(both(on_side, -Vec3f::UnitX()), WithinAbs(side_only(on_side, -Vec3f::UnitX()), 1e-5f));
    CHECK_THAT(both(unpainted, Vec3f::UnitY()), WithinAbs(0.f, 1e-6f));
}

TEST_CASE("A budget met by collapsing flat faces alone removes no detail", "[TextureDisplacement]")
{
    // 12 * 4^5 = 12288 triangles on six flat faces, which can go down to 12 without moving the surface.
    const TextureBake::TriSoup cube = TextureBake::to_soup(subdivide_mesh_uniform(its_make_cube(20., 20., 20.), 0.f, 5));
    const TextureBake::DecimateResult res = TextureBake::decimate(cube, 1000, true, 0.005);
    CHECK(res.geometry.triangle_count() <= 1000);
    CHECK_FALSE(res.target_cost_detail);
}

TEST_CASE("A budget met only by flattening curvature removes detail", "[TextureDisplacement]")
{
    // A 10 degree sphere of radius 10 bulges about 0.04 mm out of each facet, far past a 0.005 mm tolerance.
    const TextureBake::TriSoup sphere = TextureBake::to_soup(its_make_sphere(10., PI / 18.));
    const TextureBake::DecimateResult res = TextureBake::decimate(sphere, 100, true, 0.005);
    CHECK(res.geometry.triangle_count() <= 100);
    CHECK(res.target_cost_detail);
}

TEST_CASE("Cancelling while flat faces are harvested past the budget stops the decimation", "[TextureDisplacement]")
{
    // 12 * 4^6 = 49152 triangles: the budget is met almost at once, and the harvest after it runs to
    // a dozen triangles over several times the two polling intervals a cancel takes to be seen.
    const TextureBake::TriSoup cube   = TextureBake::to_soup(subdivide_mesh_uniform(its_make_cube(20., 20., 20.), 0.f, 6));
    const size_t               target = cube.triangle_count() - 1000;
    const size_t full = TextureBake::decimate(cube, target, true, 0.005).geometry.triangle_count();

    // The Cancel button pressed once the bar is full: every call after the first to report 1.0 cancels.
    bool       full_seen = false;
    const auto cancel_when_full = [&full_seen](double f) {
        if (full_seen)
            return false;
        full_seen = f >= 1.0;
        return true;
    };
    const size_t canceled = TextureBake::decimate(cube, target, true, 0.005, {}, cancel_when_full).geometry.triangle_count();
    CHECK(full_seen);
    CHECK(canceled <= target);
    CHECK(canceled > full);
}

TEST_CASE("A second bake beside a first comes out as fine as a single bake", "[TextureDisplacement]")
{
    // A stroke in each of the cube's two top triangles, baked one after the other. The first bake used
    // to leave the unpainted ground fanned into slivers from a far corner, or re-triangulated by the
    // flat harvest into long triangles, and the second bake refined those shapes rather than the
    // cube's own grid: its stroke came out many times as dense, with walls off the texture's lines.
    const indexed_triangle_set cube = its_make_cube(20.f, 20.f, 20.f);
    const Vec2f                first_stroke(5.f, 15.f), second_stroke(15.f, 5.f);
    constexpr float            radius = 3.f;

    TextureDisplacementLayer layer;
    layer.slot         = 0;
    layer.image_data   = make_checkerboard_png(16, 16);
    layer.tiling_scale = 5.f;
    layer.depth_mm     = 0.5f;
    TextureDisplacementOptions options;
    options.pipeline_v2        = true;
    options.v2_refine_mm       = 0.5f;
    options.v2_max_triangles_k = 750; // export: the flat harvest runs
    const auto bake = [&](const indexed_triangle_set &mesh, const Vec2f &stroke) {
        TextureDisplacementFacetsData facets;
        facets[0] = paint_top_disc(mesh, 20.f, stroke, radius);
        return build_texture_displacement(mesh, { layer }, facets, options);
    };
    // Aspect ratios of the top-face triangles whose centroid's distance from `centre` passes `keep`.
    const auto top_aspects = [](const indexed_triangle_set &mesh, const Vec2f &centre, auto keep) {
        std::vector<float> out;
        for (const stl_triangle_vertex_indices &t : mesh.indices) {
            const Vec3f &a = mesh.vertices[size_t(t[0])], &b = mesh.vertices[size_t(t[1])], &c = mesh.vertices[size_t(t[2])];
            const Vec3f  g = (a + b + c) / 3.f;
            if (g.z() > 19.9f && keep((Vec2f(g.x(), g.y()) - centre).norm()))
                out.push_back(triangle_aspect(a, b, c));
        }
        return out;
    };
    const auto in_stroke = [&](float d) { return d < radius; };

    const indexed_triangle_set first = bake(cube, first_stroke);
    REQUIRE(first.indices.size() > cube.indices.size());
    // What the first bake leaves around its stroke is ground the second one refines: whole pieces of
    // the cube's grid, not slivers.
    const std::vector<float> ground = top_aspects(first, first_stroke, [&](float d) { return d > radius + 1.f; });
    REQUIRE(!ground.empty());
    const float worst = *std::max_element(ground.begin(), ground.end());
    INFO("worst aspect ratio on the ground the first bake left: " << worst);
    CHECK(worst < 6.f);

    const size_t single = top_aspects(bake(cube, second_stroke), second_stroke, in_stroke).size();
    const size_t second = top_aspects(bake(first, second_stroke), second_stroke, in_stroke).size();
    INFO("second stroke baked alone: " << single << " triangles, after the first: " << second);
    REQUIRE(single > 0);
    CHECK(second <= single * 5 / 4);
}

// A small planar mesh keeps the edge-distance contract visible at individual vertices: the painted
// cells are the 4x4 square [1, 5] x [1, 5], while the one-cell outer ring remains untouched.
static indexed_triangle_set make_texture_edge_grid()
{
    constexpr int side = 7;
    indexed_triangle_set mesh;
    mesh.vertices.reserve(side * side);
    for (int y = 0; y < side; ++y)
        for (int x = 0; x < side; ++x)
            mesh.vertices.emplace_back(float(x), float(y), 0.f);

    mesh.indices.reserve(2 * (side - 1) * (side - 1));
    for (int y = 0; y < side - 1; ++y)
        for (int x = 0; x < side - 1; ++x) {
            const int a = y * side + x;
            const int b = a + 1;
            const int d = a + side;
            const int c = d + 1;
            mesh.indices.push_back({a, b, c});
            mesh.indices.push_back({a, c, d});
        }
    return mesh;
}

static TriangleSelector::TriangleSplittingData paint_texture_edge_cells(const indexed_triangle_set &mesh)
{
    constexpr int side = 7;
    REQUIRE(mesh.vertices.size() == size_t(side * side));
    REQUIRE(mesh.indices.size() == size_t(2 * (side - 1) * (side - 1)));

    const TriangleMesh tm(mesh);
    TriangleSelector       selector(tm);
    for (int y = 1; y < 5; ++y)
        for (int x = 1; x < 5; ++x) {
            const int face = 2 * (y * (side - 1) + x);
            selector.set_facet(face, EnforcerBlockerType::ENFORCER);
            selector.set_facet(face + 1, EnforcerBlockerType::ENFORCER);
        }
    return selector.serialize();
}

TEST_CASE("Texture displacement fades a painted patch by edge distance", "[TextureDisplacement][TextureDisplacementEdgeSmoothing]")
{
    const indexed_triangle_set mesh = make_texture_edge_grid();
    TextureDisplacementFacetsData facets{};
    facets[0] = paint_texture_edge_cells(mesh);
    REQUIRE_FALSE(facets[0].triangles_to_split.empty());

    const auto settings = GENERATE(table<int, bool, float>({
        {0, false, 0.f}, // disabled control
        {1, true, 0.f},  // amount-zero control
        {2, true, 0.5f},
        {3, true, 1.f},
    }));
    const int   row            = std::get<0>(settings);
    const bool  edge_smoothing = std::get<1>(settings);
    const float amount         = std::get<2>(settings);
    CAPTURE(row, edge_smoothing, amount);

    // Fixed contract table, independent of patch_boundary_distance(): the disabled and
    // amount-zero controls raise every painted grid vertex to 2; amount .5 leaves the
    // boundary at 0, the non-center 3x3 interior at 1, and (3,3) at 2; amount 1
    // leaves the complete flat grid unchanged.
    std::array<float, 49> expected{};
    if (row <= 1) {
        for (int y = 1; y <= 5; ++y)
            for (int x = 1; x <= 5; ++x)
                expected[size_t(y * 7 + x)] = 2.f;
    } else if (row == 2) {
        for (int y = 2; y <= 4; ++y)
            for (int x = 2; x <= 4; ++x)
                expected[size_t(y * 7 + x)] = 1.f;
        expected[3 * 7 + 3] = 2.f;
    }

    TextureDisplacementLayer layer;
    layer.slot                  = 0;
    layer.depth_mm              = 2.f;
    layer.tiling_scale          = 1.f;
    layer.rotation_deg          = 0.f;
    layer.offset                = Vec2f::Zero();
    layer.invert                = false;
    layer.midlevel              = 0.f;
    layer.smoothing             = 0.f;
    layer.tile_enabled          = true;
    layer.tile_method           = TextureTileMethod::Repeat;
    layer.projection_method     = TextureProjectionMethod::Triplanar;
    layer.image_data            = make_flat_gray_png(255);
    layer.edge_smoothing        = edge_smoothing;
    layer.edge_smoothing_amount = amount;
    layer.blend_mode            = TextureBlendMode::Add;

    TextureDisplacementOptions options = classic_options();
    options.displace_border     = true;
    options.smooth_enabled      = false;
    options.smooth_strength     = 0.f;
    options.smooth_iterations   = 0;
    options.smooth_skip_border  = true;

    const indexed_triangle_set out = build_texture_displacement(mesh, {layer}, facets, options);
    REQUIRE(out.vertices.size() == mesh.vertices.size());
    REQUIRE(out.indices == mesh.indices);
    for (size_t i = 0; i < out.vertices.size(); ++i) {
        REQUIRE(std::isfinite(out.vertices[i].x()));
        REQUIRE(std::isfinite(out.vertices[i].y()));
        REQUIRE(std::isfinite(out.vertices[i].z()));
        CHECK_THAT(out.vertices[i].x(), WithinAbs(mesh.vertices[i].x(), 1e-6f));
        CHECK_THAT(out.vertices[i].y(), WithinAbs(mesh.vertices[i].y(), 1e-6f));
        CHECK_THAT(out.vertices[i].z(), WithinAbs(expected[i], 1e-4f));
    }
}

TEST_CASE("Texture displacement edge smoothing preserves a lower layer's neutral value", "[TextureDisplacement][TextureDisplacementEdgeSmoothing]")
{
    const indexed_triangle_set mesh = make_texture_edge_grid();
    TextureDisplacementFacetsData facets{};
    facets[0] = paint_whole_mesh(mesh);
    facets[1] = paint_texture_edge_cells(mesh);
    REQUIRE_FALSE(facets[1].triangles_to_split.empty());

    const auto blend_case = GENERATE(table<TextureBlendMode, float>({
        {TextureBlendMode::Multiply, 2.f},
        {TextureBlendMode::Divide, 0.5f},
    }));
    const TextureBlendMode blend_mode = std::get<0>(blend_case);
    const float partial_depth          = std::get<1>(blend_case);
    CAPTURE(blend_mode, partial_depth);

    TextureDisplacementLayer base;
    base.slot                  = 0;
    base.depth_mm              = 1.f;
    base.tiling_scale          = 1.f;
    base.rotation_deg          = 0.f;
    base.offset                = Vec2f::Zero();
    base.invert                = false;
    base.midlevel              = 0.f;
    base.smoothing             = 0.f;
    base.tile_enabled          = true;
    base.tile_method           = TextureTileMethod::Repeat;
    base.projection_method     = TextureProjectionMethod::Triplanar;
    base.image_data            = make_flat_gray_png(255);
    base.blend_mode            = TextureBlendMode::Add;
    base.edge_smoothing        = false;

    TextureDisplacementLayer partial = base;
    partial.slot                  = 1;
    partial.depth_mm              = partial_depth;
    partial.blend_mode            = blend_mode;
    partial.edge_smoothing        = true;
    partial.edge_smoothing_amount = 0.5f;

    TextureDisplacementOptions options = classic_options();
    options.displace_border    = true;
    options.smooth_enabled     = false;
    options.smooth_strength    = 0.f;
    options.smooth_iterations  = 0;
    options.smooth_skip_border = true;

    const indexed_triangle_set out = build_texture_displacement(mesh, {base, partial}, facets, options);
    REQUIRE(out.vertices.size() == mesh.vertices.size());
    REQUIRE(out.indices == mesh.indices);
    for (size_t i = 0; i < out.vertices.size(); ++i) {
        const int x = int(i % 7), y = int(i / 7);
        REQUIRE(std::isfinite(out.vertices[i].x()));
        REQUIRE(std::isfinite(out.vertices[i].y()));
        REQUIRE(std::isfinite(out.vertices[i].z()));
        CHECK_THAT(out.vertices[i].x(), WithinAbs(mesh.vertices[i].x(), 1e-6f));
        CHECK_THAT(out.vertices[i].y(), WithinAbs(mesh.vertices[i].y(), 1e-6f));
        // Fixed neutral-element profile: outside and painted boundary stay at the
        // accumulated base 1, the non-center 3x3 interior is 1.5, and the center is 2.
        float expected = 1.f;
        if (x >= 2 && x <= 4 && y >= 2 && y <= 4)
            expected = (x == 3 && y == 3) ? 2.f : 1.5f;
        CHECK_THAT(out.vertices[i].z(), WithinAbs(expected, 1e-4f));
    }
}

TEST_CASE("Small colored regions follow their strongest shared boundary", "[TextureDisplacement][TextureColorRegions]")
{
    // The central right triangle has area 6 and edge lengths 5, 3 and 4.  Its
    // color-1 neighbor shares only the length-5 edge; its two color-2 neighbors
    // share lengths 3 and 4, so their combined boundary length 7 must win.
    indexed_triangle_set mesh;
    mesh.vertices = {{0.f, 0.f, 0.f}, {4.f, 0.f, 0.f}, {0.f, 3.f, 0.f},
                     {4.f, 4.f, 0.f}, {-5.f, 1.f, 0.f}, {2.f, -4.f, 0.f}};
    mesh.indices  = {{0, 1, 2}, {1, 3, 2}, {2, 4, 0}, {0, 5, 1}};
    const auto triangle_area = [&mesh](size_t i) {
        const stl_triangle_vertex_indices &t = mesh.indices[i];
        return 0.5f * (mesh.vertices[size_t(t[1])] - mesh.vertices[size_t(t[0])])
                            .cross(mesh.vertices[size_t(t[2])] - mesh.vertices[size_t(t[0])])
                            .norm();
    };
    CHECK_THAT(triangle_area(0), WithinAbs(6.f, 1e-6f));
    CHECK(triangle_area(1) > 7.f);
    CHECK(triangle_area(2) > 7.f);
    CHECK(triangle_area(3) > 7.f);
    CHECK_THAT((mesh.vertices[1] - mesh.vertices[2]).norm(), WithinAbs(5.f, 1e-6f));
    CHECK_THAT((mesh.vertices[2] - mesh.vertices[0]).norm(), WithinAbs(3.f, 1e-6f));
    CHECK_THAT((mesh.vertices[0] - mesh.vertices[1]).norm(), WithinAbs(4.f, 1e-6f));

    const std::vector<int> seed = {0, 1, 2, 2};
    SECTION("zero threshold is disabled")
    {
        std::vector<int> colors = seed;
        merge_small_color_regions(mesh, colors, 0.f);
        CHECK(colors == seed);
    }
    SECTION("the strict area boundary preserves an equal-area component")
    {
        std::vector<int> colors = seed;
        merge_small_color_regions(mesh, colors, 6.f);
        CHECK(colors == seed);
    }
    SECTION("the summed shared boundary chooses color two")
    {
        std::vector<int> colors = seed;
        merge_small_color_regions(mesh, colors, 7.f);
        const std::vector<int> expected = {2, 1, 2, 2};
        CHECK(colors == expected);
    }
    SECTION("unpainted neighbors block a merge")
    {
        std::vector<int> colors = {0, -1, -1, -1};
        merge_small_color_regions(mesh, colors, 7.f);
        const std::vector<int> expected = {0, -1, -1, -1};
        CHECK(colors == expected);
    }
    SECTION("an unpainted center stays outside colored components")
    {
        std::vector<int> colors = {-1, 1, 2, 2};
        merge_small_color_regions(mesh, colors, 7.f);
        const std::vector<int> expected = {-1, 1, 2, 2};
        CHECK(colors == expected);
    }
    SECTION("connected faces contribute their combined area")
    {
        // The center and its left neighbor form one area-13.5 component.
        std::vector<int> colors = {0, 1, 0, 2};
        merge_small_color_regions(mesh, colors, 7.f);
        const std::vector<int> expected = {0, 1, 0, 2};
        CHECK(colors == expected);
    }
}

static DecodedHeightTexture make_texture_projection_contract_texture()
{
    DecodedHeightTexture texture;
    texture.width = 2;
    texture.height = 2;
    texture.pixels = {0, 85, 170, 255};
    texture.rgb = {
        255, 0,   0, // red
        0,   255, 0, // green
        0,   0,   255, // blue
        255, 255, 255, // white
    };
    return texture;
}

TEST_CASE("Cylindrical and spherical projection sample literal colour texels", "[TextureDisplacement][TextureProjectionColor]")
{
    const DecodedHeightTexture texture = make_texture_projection_contract_texture();
    const Vec3f patch_center(10.f, 20.f, 30.f);
    constexpr float pi = 3.14159265358979323846f;
    constexpr float half_pi = 1.57079632679489661923f;

    auto expect_sample = [&](const TextureDisplacementLayer &layer,
                             const Vec3f &position,
                             const Vec3f &patch_axis,
                             const std::array<float, 3> &expected_rgb,
                             float expected_height) {
        Vec3f colour(-1.f, -1.f, -1.f);
        REQUIRE(sample_layer_color(texture, layer, position, Vec3f::UnitZ(), colour,
                                            patch_center, patch_axis));
        CHECK_THAT(colour.x(), WithinAbs(expected_rgb[0], 1e-6f));
        CHECK_THAT(colour.y(), WithinAbs(expected_rgb[1], 1e-6f));
        CHECK_THAT(colour.z(), WithinAbs(expected_rgb[2], 1e-6f));
        CHECK_THAT(sample_layer_height(texture, layer, position, Vec3f::UnitZ(),
                                               patch_center, patch_axis),
                   WithinAbs(expected_height, 1e-6f));
    };

    TextureDisplacementLayer cylindrical;
    cylindrical.projection_method = TextureProjectionMethod::Cylindrical;
    cylindrical.tile_enabled = false;
    cylindrical.tiling_scale = pi;
    cylindrical.rotation_deg = 0.f;
    cylindrical.offset = Vec2f::Zero();

    // With an axis through patch_center, the selected positions land at UV (0,0), (.5,0),
    // and (.5,.5), which are literal texel corners in this 2x2 fixture.
    expect_sample(cylindrical, patch_center + Vec3f(0.f, 1.f, 0.f),
                  Vec3f::UnitZ(), {1.f, 0.f, 0.f}, 0.f);
    expect_sample(cylindrical, patch_center + Vec3f(1.f, 0.f, 0.f),
                  Vec3f(0.f, 0.f, 5.f), {0.f, 1.f, 0.f}, 85.f / 255.f);
    expect_sample(cylindrical, patch_center + Vec3f(1.f, 0.f, half_pi),
                  Vec3f::Zero(), {1.f, 1.f, 1.f}, 1.f);

    TextureDisplacementLayer spherical = cylindrical;
    spherical.projection_method = TextureProjectionMethod::Spherical;
    expect_sample(spherical, patch_center, Vec3f::UnitX(), {1.f, 0.f, 0.f}, 0.f);
    expect_sample(spherical, patch_center + Vec3f(1.f, 0.f, 0.f),
                  Vec3f::UnitX(), {1.f, 0.f, 0.f}, 0.f);
    expect_sample(spherical, patch_center + Vec3f(0.f, 1.f, 0.f),
                  Vec3f::UnitX(), {0.f, 1.f, 0.f}, 85.f / 255.f);
    expect_sample(spherical, patch_center + Vec3f(0.f, 0.f, 1.f),
                  Vec3f::UnitX(), {0.f, 0.f, 1.f}, 170.f / 255.f);

    // The south pole is outside the single non-tiled placement. The color API must report no
    // colour and preserve its caller-owned output, while the height API returns its neutral zero.
    Vec3f colour_sentinel(0.25f, 0.5f, 0.75f);
    CHECK_FALSE(sample_layer_color(texture, spherical,
                                           patch_center + Vec3f(0.f, 0.f, -1.f),
                                           Vec3f::UnitZ(), colour_sentinel, patch_center,
                                           Vec3f::UnitX()));
    CHECK_THAT(colour_sentinel.x(), WithinAbs(0.25f, 1e-6f));
    CHECK_THAT(colour_sentinel.y(), WithinAbs(0.5f, 1e-6f));
    CHECK_THAT(colour_sentinel.z(), WithinAbs(0.75f, 1e-6f));
    CHECK_THAT(sample_layer_height(texture, spherical,
                                           patch_center + Vec3f(0.f, 0.f, -1.f),
                                           Vec3f::UnitZ(), patch_center,
                                           Vec3f::UnitX()),
               WithinAbs(0.f, 1e-6f));
}

TEST_CASE("Projective colour projection enforces depth and untiled bounds", "[TextureDisplacement][TextureProjectionColor]")
{
    const DecodedHeightTexture texture = make_texture_projection_contract_texture();
    TextureDisplacementLayer layer;
    layer.projection_method = TextureProjectionMethod::ViewProjected;
    layer.view_project_projective = true;
    layer.tile_enabled = false;
    // These ordinary placement fields must not affect a projective matrix's already-normalized UV.
    layer.tiling_scale = 0.125f;
    layer.rotation_deg = 37.f;
    layer.offset = Vec2f(0.31f, -0.27f);
    // w=z+1; the origin maps to the white texel at UV (.5,.5), z=1 has w=2 and maps to
    // the green texel at UV (.5,0), y=2 maps outside at v=1.5, and z=-2 is behind.
    layer.view_project_matrix = {0.f, 0.f, 0.5f, 0.5f,
                                 0.f, 0.5f, -0.5f, 0.5f,
                                 0.f, 0.f, 1.f, 1.f};

    Vec3f inside(-1.f, -1.f, -1.f);
    REQUIRE(sample_layer_color(texture, layer, Vec3f::Zero(), Vec3f::UnitZ(), inside));
    CHECK_THAT(inside.x(), WithinAbs(1.f, 1e-6f));
    CHECK_THAT(inside.y(), WithinAbs(1.f, 1e-6f));
    CHECK_THAT(inside.z(), WithinAbs(1.f, 1e-6f));
    CHECK_THAT(sample_layer_height(texture, layer, Vec3f::Zero(), Vec3f::UnitZ()),
               WithinAbs(1.f, 1e-6f));

    Vec3f perspective_inside(-1.f, -1.f, -1.f);
    REQUIRE(sample_layer_color(texture, layer, Vec3f(0.f, 0.f, 1.f),
                                       Vec3f::UnitZ(), perspective_inside));
    CHECK_THAT(perspective_inside.x(), WithinAbs(0.f, 1e-6f));
    CHECK_THAT(perspective_inside.y(), WithinAbs(1.f, 1e-6f));
    CHECK_THAT(perspective_inside.z(), WithinAbs(0.f, 1e-6f));
    CHECK_THAT(sample_layer_height(texture, layer, Vec3f(0.f, 0.f, 1.f),
                                           Vec3f::UnitZ()),
               WithinAbs(85.f / 255.f, 1e-6f));

    Vec3f behind_sentinel(0.2f, 0.4f, 0.6f);
    CHECK_FALSE(sample_layer_color(texture, layer, Vec3f(0.f, 0.f, -2.f),
                                           Vec3f::UnitZ(), behind_sentinel));
    CHECK_THAT(behind_sentinel.x(), WithinAbs(0.2f, 1e-6f));
    CHECK_THAT(behind_sentinel.y(), WithinAbs(0.4f, 1e-6f));
    CHECK_THAT(behind_sentinel.z(), WithinAbs(0.6f, 1e-6f));
    CHECK_THAT(sample_layer_height(texture, layer, Vec3f(0.f, 0.f, -2.f),
                                           Vec3f::UnitZ()),
               WithinAbs(0.f, 1e-6f));

    Vec3f outside_sentinel(0.7f, 0.8f, 0.9f);
    CHECK_FALSE(sample_layer_color(texture, layer, Vec3f(0.f, 2.f, 0.f),
                                           Vec3f::UnitZ(), outside_sentinel));
    CHECK_THAT(outside_sentinel.x(), WithinAbs(0.7f, 1e-6f));
    CHECK_THAT(outside_sentinel.y(), WithinAbs(0.8f, 1e-6f));
    CHECK_THAT(outside_sentinel.z(), WithinAbs(0.9f, 1e-6f));
    CHECK_THAT(sample_layer_height(texture, layer, Vec3f(0.f, 2.f, 0.f),
                                           Vec3f::UnitZ()),
               WithinAbs(0.f, 1e-6f));
}

namespace texture_anchor_contracts {

TriangleSelector::TriangleSplittingData select_facets(const indexed_triangle_set &mesh,
                                                       const std::vector<int> &facet_ids)
{
    const TriangleMesh triangle_mesh(mesh);
    TriangleSelector selector(triangle_mesh);
    for (const int facet_id : facet_ids)
        selector.set_facet(facet_id, EnforcerBlockerType::ENFORCER);
    return selector.serialize();
}

void check_vec3(const Vec3f &actual, const Vec3f &expected, float tolerance)
{
    CHECK_THAT(actual.x(), WithinAbs(expected.x(), tolerance));
    CHECK_THAT(actual.y(), WithinAbs(expected.y(), tolerance));
    CHECK_THAT(actual.z(), WithinAbs(expected.z(), tolerance));
}

TEST_CASE("Painted anchors use the analytic area-weighted patch centroid and normal",
          "[TextureDisplacement][TextureDisplacementAnchor]")
{
    const bool equal_area = GENERATE(false, true);
    CAPTURE(equal_area);

    indexed_triangle_set mesh;
    if (equal_area) {
        // Both triangles have area 1.5 and +Z orientation.  This is a positive
        // control for the current unweighted implementation as well as a check
        // that the selector preserves both selected leaves.
        mesh.vertices = {
            { 0.f, 0.f, 0.f }, { 3.f, 0.f, 0.f }, { 0.f, 1.f, 0.f },
            { 10.f, 0.f, 0.f }, { 13.f, 0.f, 0.f }, { 10.f, 1.f, 0.f }
        };
        mesh.indices = { { 0, 1, 2 }, { 3, 4, 5 } };
    } else {
        // The first triangle has area 1.5, centroid (1, 1/3, 0), and +Z
        // normal.  The second has area 6, centroid (10, 4/3, 1), and +X
        // normal.  Disjoint facets keep the expected result independent of
        // topology or ordering conventions in the selector.
        mesh.vertices = {
            { 0.f, 0.f, 0.f }, { 3.f, 0.f, 0.f }, { 0.f, 1.f, 0.f },
            { 10.f, 0.f, 0.f }, { 10.f, 4.f, 0.f }, { 10.f, 0.f, 3.f }
        };
        mesh.indices = { { 0, 1, 2 }, { 3, 4, 5 } };
    }

    Vec3f anchor_position(99.f, 98.f, 97.f);
    Vec3f anchor_normal(-1.f, -2.f, -3.f);
    REQUIRE(compute_layer_paint_anchor(mesh, select_facets(mesh, {0, 1}),
                                       anchor_position, anchor_normal));

    if (equal_area) {
        // The two equal-area centroids average to (6, 1/3, 0), and both
        // oriented cross products point along +Z.
        check_vec3(anchor_position, Vec3f(6.f, 1.f / 3.f, 0.f), 1e-5f);
        check_vec3(anchor_normal, Vec3f::UnitZ(), 1e-5f);
    } else {
        // (1.5*(1,1/3,0) + 6*(10,4/3,1)) / 7.5.
        check_vec3(anchor_position, Vec3f(8.2f, 17.f / 15.f, 0.8f), 1e-5f);
        // Cross products are (0,0,3) and (12,0,0), so the normalized sum is (12,0,3)/sqrt(153).
        check_vec3(anchor_normal, Vec3f(12.f / std::sqrt(153.f), 0.f, 3.f / std::sqrt(153.f)), 1e-5f);
    }
}

TEST_CASE("Unpainted anchors preserve caller outputs",
          "[TextureDisplacement][TextureDisplacementAnchor]")
{
    const bool has_mesh = GENERATE(false, true);
    CAPTURE(has_mesh);

    indexed_triangle_set mesh;
    if (has_mesh)
        mesh = its_make_cube(10., 10., 10.);
    // Both rows have no selected facets.  The nonempty row verifies that the
    // valid base mesh itself does not turn an unpainted layer into a patch.
    TriangleSelector::TriangleSplittingData empty_facets;
    const Vec3f original_position(7.f, -8.f, 9.f);
    const Vec3f original_normal(-4.f, 5.f, -6.f);
    Vec3f anchor_position = original_position;
    Vec3f anchor_normal = original_normal;

    CHECK_FALSE(compute_layer_paint_anchor(mesh, empty_facets, anchor_position, anchor_normal));
    check_vec3(anchor_position, original_position, 0.f);
    check_vec3(anchor_normal, original_normal, 0.f);
}

TEST_CASE("Degenerate painted patches use the stable fallback normal",
          "[TextureDisplacement][TextureDisplacementAnchor]")
{
    // TriangleSelector::reset/push_triangle retains every indexed leaf,
    // including this collinear one; set_facet/serialize therefore provide a
    // valid selected patch for the source fallback branch to consume.
    indexed_triangle_set mesh;
    mesh.vertices = { { 0.f, 0.f, 0.f }, { 2.f, 0.f, 0.f }, { 1.f, 0.f, 0.f } };
    mesh.indices = { { 0, 1, 2 } };

    Vec3f anchor_position = Vec3f::Zero();
    Vec3f anchor_normal = Vec3f::Zero();
    REQUIRE(compute_layer_paint_anchor(mesh, select_facets(mesh, {0}),
                                       anchor_position, anchor_normal));
    check_vec3(anchor_position, Vec3f(1.f, 0.f, 0.f), 1e-5f);
    // The +Z result is source-specified by TextureDisplacement.cpp's
    // near-zero normal guard; it is not separately promised by the header.
    check_vec3(anchor_normal, Vec3f::UnitZ(), 1e-5f);
}


TEST_CASE("Painted anchor geometry ignores zero-area faces and winding cancellation",
          "[TextureDisplacement][TextureDisplacementAnchor]")
{
    const bool opposite_winding = GENERATE(false, true);
    CAPTURE(opposite_winding);

    indexed_triangle_set mesh;
    if (opposite_winding) {
        // Equal-area triangles with opposite winding retain their geometric centroid,
        // while the signed normal sum intentionally reaches the existing +Z fallback.
        mesh.vertices = {
            { 0.f, 0.f, 0.f }, { 3.f, 0.f, 0.f }, { 0.f, 1.f, 0.f },
            { 10.f, 0.f, 0.f }, { 10.f, 1.f, 0.f }, { 13.f, 0.f, 0.f }
        };
        mesh.indices = { { 0, 1, 2 }, { 3, 4, 5 } };
    } else {
        // The second face is far away but has exactly zero area. It must not move
        // the positive-area face's centroid or normal.
        mesh.vertices = {
            { 0.f, 0.f, 0.f }, { 3.f, 0.f, 0.f }, { 0.f, 1.f, 0.f },
            { 100.f, 100.f, 100.f }, { 102.f, 100.f, 100.f }, { 101.f, 100.f, 100.f }
        };
        mesh.indices = { { 0, 1, 2 }, { 3, 4, 5 } };
    }

    REQUIRE(mesh.vertices.size() == 6);
    REQUIRE(mesh.indices.size() == 2);
    const auto selected = select_facets(mesh, {0, 1});
    REQUIRE_FALSE(selected.triangles_to_split.empty());

    Vec3f anchor_position(99.f, 98.f, 97.f);
    Vec3f anchor_normal(-1.f, -2.f, -3.f);
    REQUIRE(compute_layer_paint_anchor(mesh, selected, anchor_position, anchor_normal));

    if (opposite_winding) {
        check_vec3(anchor_position, Vec3f(6.f, 1.f / 3.f, 0.f), 1e-5f);
        check_vec3(anchor_normal, Vec3f::UnitZ(), 1e-5f);
    } else {
        check_vec3(anchor_position, Vec3f(1.f, 1.f / 3.f, 0.f), 1e-5f);
        check_vec3(anchor_normal, Vec3f::UnitZ(), 1e-5f);
    }
}
} // namespace texture_anchor_contracts
TEST_CASE("Planar projection follows the dominant normal axis", "[TextureDisplacement][TexturePlacement]")
{
    const Vec3f position(1.25f, -2.5f, 3.75f);
    struct Row {
        Vec3f normal;
        Vec2f expected;
    };
    const std::array<Row, 4> rows{{
        { Vec3f(3.f, -1.f, 0.5f), Vec2f(-2.5f, 3.75f) },
        { Vec3f(-0.5f, 4.f, -1.f), Vec2f(1.25f, 3.75f) },
        { Vec3f(0.5f, -1.f, -5.f), Vec2f(1.25f, -2.5f) },
        { Vec3f(-5.f, 1.f, 0.5f), Vec2f(-2.5f, 3.75f) },
    }};

    for (size_t row = 0; row < rows.size(); ++row) {
        INFO("row=" << row);
        const Vec2f actual = project_planar(position, rows[row].normal);
        CHECK_THAT(actual.x(), WithinAbs(rows[row].expected.x(), 1e-6f));
        CHECK_THAT(actual.y(), WithinAbs(rows[row].expected.y(), 1e-6f));
    }
}

TEST_CASE("Texture UV transforms apply scale rotation aspect and offset", "[TextureDisplacement][TexturePlacement]")
{
    TextureDisplacementLayer layer;
    layer.tiling_scale = 2.f;
    layer.rotation_deg = 90.f;
    layer.offset       = Vec2f(0.1f, -0.2f);
    const Vec2f planar(2.f, 1.f);

    struct Row {
        float aspect;
        Vec2f expected;
    };
    const std::array<Row, 3> rows{{
        { 1.f, Vec2f(-0.4f, 0.8f) },
        { 2.f, Vec2f(-0.4f, 1.8f) },
        { 0.f, Vec2f(-0.4f, 0.8f) },
    }};
    for (size_t row = 0; row < rows.size(); ++row) {
        INFO("row=" << row);
        const Vec2f actual = apply_uv_transform(planar, layer, rows[row].aspect);
        CHECK_THAT(actual.x(), WithinAbs(rows[row].expected.x(), 1e-5f));
        CHECK_THAT(actual.y(), WithinAbs(rows[row].expected.y(), 1e-5f));
    }

    TextureDisplacementLayer fallback = layer;
    fallback.tiling_scale = 0.f;
    const Vec2f fallback_uv = apply_uv_transform(planar, fallback, 1.f);
    CHECK_THAT(fallback_uv.x(), WithinAbs(-0.9f, 1e-5f));
    CHECK_THAT(fallback_uv.y(), WithinAbs(1.8f, 1e-5f));
}

static TextureBake::TriSoup make_texture_bake_boundary_falloff_strip()
{
    TextureBake::TriSoup soup;
    const Vec3f normal(0.f, 0.f, 1.f);
    const auto append_triangle = [&](const Vec3f &a, const Vec3f &b, const Vec3f &c, float excluded) {
        soup.pos.push_back(a);
        soup.pos.push_back(b);
        soup.pos.push_back(c);
        soup.nrm.insert(soup.nrm.end(), 3, normal);
        soup.exclude_weight.insert(soup.exclude_weight.end(), 3, excluded);
    };
    const auto append_quad = [&](int x, float excluded) {
        const Vec3f p00(float(x), 0.f, 0.f);
        const Vec3f p10(float(x + 1), 0.f, 0.f);
        const Vec3f p11(float(x + 1), 1.f, 0.f);
        const Vec3f p01(float(x), 1.f, 0.f);
        append_triangle(p00, p10, p11, excluded);
        append_triangle(p00, p11, p01, excluded);
    };

    append_quad(0, 1.f);
    append_quad(1, 0.f);
    append_quad(2, 0.f);
    return soup;
}

TEST_CASE("Texture bake boundary falloff ramps an excluded planar seam", "[TextureDisplacement][TextureBakeBoundaryFalloff]")
{
    const TextureBake::TriSoup strip = make_texture_bake_boundary_falloff_strip();
    const TextureBake::HeightSampleFn sample =
        [](const Vec3f &, const Vec3f &, const Vec3f &) { return 1.f; };

    TextureBake::DisplaceSettings settings;
    settings.amplitude              = 1.f;
    settings.symmetric              = false;
    settings.bottom_angle_limit     = 0.f;
    settings.top_angle_limit        = 0.f;
    settings.no_downward_z           = false;
    settings.blend_normal_smoothing = 0;
    settings.boundary_falloff        = 2.f;

    TextureBake::DisplaceBounds bounds;
    bounds.min = Vec3f(0.f, 0.f, 0.f);
    bounds.max = Vec3f(3.f, 1.f, 0.f);

    const auto check_output = [&](const TextureBake::TriSoup &input_soup,
                                  const TextureBake::TriSoup &output,
                                  const std::array<float, 4> &expected_z) {
        REQUIRE(output.pos.size() == input_soup.pos.size());
        REQUIRE(output.nrm.size() == input_soup.nrm.size());
        REQUIRE(output.exclude_weight == input_soup.exclude_weight);

        float min_z = std::numeric_limits<float>::max();
        float max_z = std::numeric_limits<float>::lowest();
        for (size_t i = 0; i < input_soup.pos.size(); ++i) {
            const Vec3f &input  = input_soup.pos[i];
            const Vec3f &actual = output.pos[i];
            CHECK_THAT(actual.x(), WithinAbs(input.x(), 1e-6f));
            CHECK_THAT(actual.y(), WithinAbs(input.y(), 1e-6f));
            const size_t column = size_t(input.x());
            REQUIRE(column < expected_z.size());
            CHECK_THAT(actual.z(), WithinAbs(expected_z[column], 1e-5f));
            min_z = std::min(min_z, actual.z());
            max_z = std::max(max_z, actual.z());
        }
        CHECK(min_z >= -1e-6f);
        CHECK(max_z <= 1.f + 1e-6f);
    };

    const TextureBake::TriSoup ramp = TextureBake::apply_displacement(strip, sample, settings, bounds);
    check_output(strip, ramp, { 0.f, 0.f, 0.5f, 1.f });

    settings.boundary_falloff = 0.f;
    const TextureBake::TriSoup hard = TextureBake::apply_displacement(strip, sample, settings, bounds);
    check_output(strip, hard, { 0.f, 0.f, 1.f, 1.f });

    TextureBake::TriSoup unmasked = strip;
    std::fill(unmasked.exclude_weight.begin(), unmasked.exclude_weight.end(), 0.f);
    settings.boundary_falloff = 2.f;
    const TextureBake::TriSoup unmasked_output =
        TextureBake::apply_displacement(unmasked, sample, settings, bounds);
    check_output(unmasked, unmasked_output, { 1.f, 1.f, 1.f, 1.f });
}
