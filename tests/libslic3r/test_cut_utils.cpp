#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/CutUtils.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/libslic3r.h"

using namespace Slic3r;

namespace {

constexpr double COORD_EPSILON = 1e-4;

double mesh_volume(const ModelVolume *volume)
{
    TriangleMesh mesh = volume->mesh(); // volume() is not const
    return std::abs(static_cast<double>(mesh.volume()));
}

// A box spanning [0, x] x [0, y] x [0, z].
ModelObject *add_box(Model &model, double x, double y, double z)
{
    ModelObject *object = model.add_object();
    object->add_volume(make_cube(x, y, z));
    object->add_instance();
    return object;
}

const ModelObjectCutAttributes keep_both = ModelObjectCutAttribute::KeepUpper | ModelObjectCutAttribute::KeepLower;

} // namespace

namespace groove_contracts {
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

// A 3 mm deep, 1.5 mm wide groove with flat flaps, no taper and the default 0.1 mm tolerances.
Cut::Groove flat_groove()
{
    Cut::Groove groove;
    groove.depth = 3.f;
    groove.width = 1.5f;
    groove.flaps_angle = 0.f;
    groove.angle = 0.f;
    return groove;
}

// A groove with perpendicular flaps cuts a rectangular tongue, here 2 mm deep and 2 mm wide.
Cut::Groove perpendicular_groove()
{
    Cut::Groove groove;
    groove.depth = 2.f;
    groove.width = 2.f;
    groove.flaps_angle = float(PI / 2.);
    groove.angle = 0.f;
    groove.depth_tolerance = .1f;
    groove.width_tolerance = .1f;
    return groove;
}

struct GrooveHalves {
    double upper = 0.;
    double lower = 0.;
    // The part of the upper half below the cut plane is only the tongues: their spans across the groove
    // direction, their common extent along it, and their lowest point.
    std::vector<std::pair<double, double>> tongues;
    std::pair<double, double> length{0., 0.};
    double tongue_min_z = 0.;
};

// The volumes of the halves of a box [0, x] x [0, y] x [0, 20] cut at z = 10 by `count` perpendicular
// grooves whose tongues run along Y (or along X when `along_x`). The lower half keeps everything below the
// groove layer plus the layer outside the tongues; the upper half keeps everything above plus the
// tongues, trimmed by the depth tolerance at the tip and half the width tolerance on each side.
GrooveHalves expected_halves(double x, double y, const Cut::Groove &g, bool along_x = false, int count = 1, double gap = 0.)
{
    const double length = along_x ? x : y;
    const double across = along_x ? y : x;
    const double outside_layer = x * y * (10. - g.depth / 2.);
    GrooveHalves halves;
    halves.upper = outside_layer + count * (g.width - g.width_tolerance) * (g.depth - g.depth_tolerance) * length;
    halves.lower = outside_layer + (across - count * g.width) * g.depth * length;
    // Tongues of width (w - wt), spaced (gap + w) apart around the box centre, with their tip trimmed by dt.
    for (int i = 0; i < count; ++i) {
        const double centre = across / 2. + (i - .5 * (count - 1)) * (gap + g.width);
        halves.tongues.emplace_back(centre - .5 * (g.width - g.width_tolerance), centre + .5 * (g.width - g.width_tolerance));
    }
    halves.length = {0., length};
    halves.tongue_min_z = 10. - g.depth / 2. + g.depth_tolerance;
    return halves;
}

// Measures the tongues: the upper half's vertices below the cut plane, grouped across the groove direction
// wherever consecutive coordinates are more than 3 mm apart (the tests space tongues further apart than
// that and keep each narrower).
void measure_tongues(const ModelObject &upper, bool along_x, GrooveHalves &halves)
{
    const ModelVolume &volume = *upper.volumes.front();
    const Transform3d to_world = upper.instances.front()->get_matrix() * volume.get_matrix();
    std::vector<double> across;
    halves.length = {std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest()};
    halves.tongue_min_z = std::numeric_limits<double>::max();
    for (const Vec3f &v : volume.mesh().its.vertices) {
        const Vec3d p = to_world * v.cast<double>();
        if (p.z() >= 10.)
            continue;
        across.push_back(along_x ? p.y() : p.x());
        const double along = along_x ? p.x() : p.y();
        halves.length.first = std::min(halves.length.first, along);
        halves.length.second = std::max(halves.length.second, along);
        halves.tongue_min_z = std::min(halves.tongue_min_z, p.z());
    }
    std::sort(across.begin(), across.end());
    for (size_t i = 0; i < across.size(); ++i) {
        if (i == 0 || across[i] - across[i - 1] > 3.)
            halves.tongues.emplace_back(across[i], across[i]);
        halves.tongues.back().second = across[i];
    }
}

// Cuts the box [0, x] x [0, y] x [0, 20] at its centre with the grooves turned `angle` about Z and returns
// the half volumes. The upper half is the one whose tongue reaches below the cut plane.
GrooveHalves cut_box_with_groove(double x, double y, const Cut::Groove &groove, double angle = 0., int count = 1,
                                 float gap = 0.f, bool keep_as_parts = false)
{
    Model source;
    ModelObject *object = add_box(source, x, y, 20.);
    const Transform3d rotation = Geometry::rotation_transform(Vec3d(0., 0., angle));
    Cut cut(object, 0, Geometry::translation_transform(Vec3d(x / 2., y / 2., 10.)) * rotation, keep_both);
    const ModelObjectPtrs &result = cut.perform_with_groove(groove, rotation, count, gap, 10.f, keep_as_parts);
    REQUIRE(result.size() == 2);
    GrooveHalves halves;
    if (keep_as_parts) {
        // Both halves are parts of the first object; the second is only kept so the model deletes it.
        for (const ModelVolume *volume : result.front()->volumes) {
            CHECK(its_num_open_edges(volume->mesh().its) == 0);
            (volume->cut_info.is_from_upper ? halves.upper : halves.lower) += mesh_volume(volume);
        }
        return halves;
    }
    size_t uppers = 0;
    for (const ModelObject *part : result) {
        REQUIRE(part->volumes.size() == 1);
        const ModelVolume *volume = part->volumes.front();
        CHECK(its_num_open_edges(volume->mesh().its) == 0);
        // The cut halves carry their placement on the instance, so classify them in world space.
        const bool is_upper = part->instance_bounding_box(0).min.z() > COORD_EPSILON;
        uppers += is_upper;
        (is_upper ? halves.upper : halves.lower) = mesh_volume(volume);
        if (is_upper)
            measure_tongues(*part, std::abs(std::sin(angle)) > .5, halves);
    }
    CHECK(uppers == 1);
    return halves;
}

void check_halves(const GrooveHalves &actual, const GrooveHalves &expected)
{
    CHECK_THAT(actual.upper, WithinRel(expected.upper, 1e-5));
    CHECK_THAT(actual.lower, WithinRel(expected.lower, 1e-5));
    REQUIRE(actual.tongues.size() == expected.tongues.size());
    for (size_t i = 0; i < expected.tongues.size(); ++i) {
        CHECK_THAT(actual.tongues[i].first, WithinAbs(expected.tongues[i].first, 1e-4));
        CHECK_THAT(actual.tongues[i].second, WithinAbs(expected.tongues[i].second, 1e-4));
    }
    CHECK_THAT(actual.length.first, WithinAbs(expected.length.first, 1e-4));
    CHECK_THAT(actual.length.second, WithinAbs(expected.length.second, 1e-4));
    CHECK_THAT(actual.tongue_min_z, WithinAbs(expected.tongue_min_z, 1e-4));
}

// Keep-as-parts returns no separate upper object to measure the tongues of, so only volumes are compared.
void check_volumes(const GrooveHalves &actual, const GrooveHalves &expected)
{
    CHECK_THAT(actual.upper, WithinRel(expected.upper, 1e-5));
    CHECK_THAT(actual.lower, WithinRel(expected.lower, 1e-5));
}

TEST_CASE("Groove width for flat flaps adds a full-depth mouth on each side", "[CutUtils][CutUtilsGroove]")
{
    // 1.5 mm neck plus 3 mm on each side.
    CHECK_THAT(Cut::calculate_groove_width(flat_groove(), 10.f), WithinAbs(7.5, 1e-5));
}

TEST_CASE("Groove width uses the flap angle to reduce the projected mouth width", "[CutUtils][CutUtilsGroove]")
{
    Cut::Groove groove = flat_groove();
    groove.flaps_angle = float(PI / 6.);
    // Each 30 degree flap of depth 3 spans 3 / tan(30) horizontally.
    CHECK_THAT(Cut::calculate_groove_width(groove, 10.f), WithinAbs(1.5 + 2. * 3. / std::tan(PI / 6.), 1e-4));
}

TEST_CASE("Groove width grows with the taper angle in proportion to the radius", "[CutUtils][CutUtilsGroove]")
{
    Cut::Groove groove = flat_groove();
    groove.angle = .25f;
    // The preview plane reaches 1.125 radii each way, so each mouth widens by 1.125 r tan(angle).
    CHECK_THAT(Cut::calculate_groove_width(groove, 10.f), WithinAbs(7.5 + 2. * 1.125 * 10. * std::tan(.25), 1e-4));
    CHECK_THAT(Cut::calculate_groove_width(groove, 20.f), WithinAbs(7.5 + 2. * 1.125 * 20. * std::tan(.25), 1e-4));
}

TEST_CASE("Groove width tolerance fields do not change the cutting mouth", "[CutUtils][CutUtilsGroove]")
{
    Cut::Groove loose = flat_groove();
    loose.depth_tolerance = 2.f;
    loose.width_tolerance = 3.f;
    CHECK_THAT(Cut::calculate_groove_width(loose, 10.f), WithinAbs(7.5, 1e-5));
}

TEST_CASE("A perpendicular groove splits a box into a tongue half and a slot half", "[CutUtils][CutUtilsGroove]")
{
    // 30 x 20 box, tongue along Y: lower 600 * 9 + 28 * 2 * 20, upper 600 * 9 + 1.9 * 1.9 * 20.
    const Cut::Groove groove = perpendicular_groove();
    check_halves(cut_box_with_groove(30., 20., groove), expected_halves(30., 20., groove));
}

TEST_CASE("Groove depth tolerance shortens only the tongue", "[CutUtils][CutUtilsGroove]")
{
    Cut::Groove groove = perpendicular_groove();
    groove.depth_tolerance = .5f;
    check_halves(cut_box_with_groove(30., 20., groove), expected_halves(30., 20., groove));
}

TEST_CASE("Groove width tolerance narrows only the tongue", "[CutUtils][CutUtilsGroove]")
{
    Cut::Groove groove = perpendicular_groove();
    groove.width_tolerance = .5f;
    check_halves(cut_box_with_groove(30., 20., groove), expected_halves(30., 20., groove));
}

TEST_CASE("A groove turned a quarter about the cut normal runs its tongue along X", "[CutUtils][CutUtilsGroove]")
{
    // In the 30 x 20 box the tongue is 30 mm long instead of 20, so both halves change.
    const Cut::Groove groove = perpendicular_groove();
    check_halves(cut_box_with_groove(30., 20., groove, PI / 2.), expected_halves(30., 20., groove, true));
}

TEST_CASE("Multiple grooves cut a separate tongue for every requested slot", "[CutUtils][CutUtilsGroove]")
{
    // Two 2 mm grooves 4 mm apart in a 40 mm box: tongues at x = 16.05..17.95 and 22.05..23.95.
    const Cut::Groove groove = perpendicular_groove();
    check_halves(cut_box_with_groove(40., 20., groove, 0., 2, 4.f), expected_halves(40., 20., groove, false, 2, 4.));
}

TEST_CASE("Keep-as-parts returns both groove halves as parts of one object", "[CutUtils][CutUtilsGroove]")
{
    const Cut::Groove groove = perpendicular_groove();
    check_volumes(cut_box_with_groove(30., 20., groove, 0., 1, 0.f, true), expected_halves(30., 20., groove));
}

} // namespace groove_contracts
