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

BoundingBoxf3 world_bounding_box(const ModelVolume *volume)
{
    return volume->mesh().transformed_bounding_box(volume->get_matrix());
}

// The bounds of a cut result volume in world space: cut objects carry their placement on the instance.
BoundingBoxf3 placed_bounding_box(const ModelObject *part, const ModelVolume *volume)
{
    return volume->mesh().transformed_bounding_box(part->instances.front()->get_matrix() * volume->get_matrix());
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

TEST_CASE("Cut plane splits a box into two closed halves at the plane", "[CutUtils]")
{
    Model source_model;
    ModelObject *source_object = add_box(source_model, 20., 20., 20.);

    // The plane sits at z = 6, off centre, so the halves are told apart by size as well as position.
    Cut cut(source_object, 0, Geometry::translation_transform(Vec3d(0., 0., 6.)), keep_both);
    const ModelObjectPtrs &parts = cut.perform_with_plane();

    REQUIRE(parts.size() == 2);
    size_t lower_count = 0, upper_count = 0;
    for (const ModelObject *part : parts) {
        REQUIRE(part->volumes.size() == 1);
        const ModelVolume *volume = part->volumes.front();
        REQUIRE(volume->is_model_part());
        CHECK(its_num_open_edges(volume->mesh().its) == 0);

        const BoundingBoxf3 box = placed_bounding_box(part, volume);
        CHECK_THAT(box.size().x(), Catch::Matchers::WithinAbs(20., COORD_EPSILON));
        CHECK_THAT(box.size().y(), Catch::Matchers::WithinAbs(20., COORD_EPSILON));
        if (box.size().z() < 10.) {
            ++lower_count;
            CHECK_THAT(box.min.z(), Catch::Matchers::WithinAbs(0., COORD_EPSILON));
            CHECK_THAT(box.max.z(), Catch::Matchers::WithinAbs(6., COORD_EPSILON));
            CHECK_THAT(mesh_volume(volume), Catch::Matchers::WithinRel(20. * 20. * 6., 1e-5));
        } else {
            ++upper_count;
            CHECK_THAT(box.min.z(), Catch::Matchers::WithinAbs(6., COORD_EPSILON));
            CHECK_THAT(box.max.z(), Catch::Matchers::WithinAbs(20., COORD_EPSILON));
            CHECK_THAT(mesh_volume(volume), Catch::Matchers::WithinRel(20. * 20. * 14., 1e-5));
        }
    }
    CHECK(lower_count == 1);
    CHECK(upper_count == 1);
}

TEST_CASE("Cut plane honors a single retained side", "[CutUtils]")
{
    auto retained = [](ModelObjectCutAttribute side) {
        Model source_model;
        ModelObject *source_object = add_box(source_model, 20., 20., 20.);
        Cut cut(source_object, 0, Geometry::translation_transform(Vec3d(0., 0., 6.)), side);
        const ModelObjectPtrs &parts = cut.perform_with_plane();
        REQUIRE(parts.size() == 1);
        REQUIRE(parts.front()->volumes.size() == 1);
        CHECK(its_num_open_edges(parts.front()->volumes.front()->mesh().its) == 0);
        return mesh_volume(parts.front()->volumes.front());
    };
    CHECK_THAT(retained(ModelObjectCutAttribute::KeepUpper), Catch::Matchers::WithinRel(20. * 20. * 14., 1e-5));
    CHECK_THAT(retained(ModelObjectCutAttribute::KeepLower), Catch::Matchers::WithinRel(20. * 20. * 6., 1e-5));
}

TEST_CASE("Cut plane with no retained side returns no objects", "[CutUtils]")
{
    Model source_model;
    ModelObject *source_object = add_box(source_model, 10., 10., 10.);

    Cut cut(source_object, 0, Transform3d::Identity(), ModelObjectCutAttribute::KeepAsParts);
    CHECK(cut.perform_with_plane().empty());
}

TEST_CASE("Cut plane copies modifiers to both halves and splits an unprocessed dowel", "[CutUtils]")
{
    Model source_model;
    ModelObject *source_object = add_box(source_model, 20., 20., 20.);

    ModelVolume *modifier = source_object->add_volume(make_cube(8., 8., 8.), ModelVolumeType::PARAMETER_MODIFIER);
    modifier->set_offset(Vec3d(0., 0., 10.));
    ModelVolume *connector = source_object->add_volume(make_cube(4., 4., 4.), ModelVolumeType::NEGATIVE_VOLUME);
    connector->set_offset(Vec3d(0., 0., 10.));
    connector->cut_info = ModelVolume::CutInfo(CutConnectorType::Dowel, 0.1f, 0.2f, false);

    // add_volume() centres each mesh on its volume offset, so both volumes straddle the z = 10 cut plane.
    const BoundingBoxf3 modifier_box = world_bounding_box(modifier);
    CHECK_THAT(modifier_box.min.z(), Catch::Matchers::WithinAbs(6., COORD_EPSILON));
    CHECK_THAT(modifier_box.max.z(), Catch::Matchers::WithinAbs(14., COORD_EPSILON));
    const BoundingBoxf3 connector_source_box = world_bounding_box(connector);
    CHECK_THAT(connector_source_box.min.z(), Catch::Matchers::WithinAbs(8., COORD_EPSILON));
    CHECK_THAT(connector_source_box.max.z(), Catch::Matchers::WithinAbs(12., COORD_EPSILON));

    Cut cut(source_object, 0, Geometry::translation_transform(Vec3d(0., 0., 10.)), keep_both);
    const ModelObjectPtrs &parts = cut.perform_with_plane();

    REQUIRE(parts.size() == 2);
    bool found_lower_connector = false;
    bool found_upper_connector = false;
    for (const ModelObject *part : parts) {
        size_t modifiers = 0;
        const ModelVolume *part_connector = nullptr;
        for (const ModelVolume *volume : part->volumes) {
            if (volume->type() == ModelVolumeType::PARAMETER_MODIFIER)
                ++modifiers;
            if (volume->is_cut_connector())
                part_connector = volume;
        }
        CHECK(modifiers == 1);
        REQUIRE(part_connector != nullptr);
        CHECK(part_connector->is_negative_volume());
        CHECK(part_connector->cut_info.connector_type == CutConnectorType::Dowel);
        CHECK_THAT(part_connector->cut_info.radius_tolerance, Catch::Matchers::WithinAbs(0.1, 1e-6));
        CHECK_THAT(part_connector->cut_info.height_tolerance, Catch::Matchers::WithinAbs(0.2, 1e-6));

        // Before the split the dowel gets its tolerances: 4 mm grows by the 0.1 radius tolerance to 4.4 mm
        // across, its height by the 0.2 height tolerance to 4.8 mm about its centre (7.6..12.4), then it is
        // raised by half the height tolerance (7.7..12.5). The halves are cut at z = 10 and moved 0.05 mm
        // apart for the preview.
        const BoundingBoxf3 connector_box = placed_bounding_box(part, part_connector);
        CHECK_THAT(connector_box.size().x(), Catch::Matchers::WithinAbs(4.4, COORD_EPSILON));
        CHECK_THAT(connector_box.size().y(), Catch::Matchers::WithinAbs(4.4, COORD_EPSILON));
        if (connector_box.center().z() < 10.) {
            found_lower_connector = true;
            CHECK_THAT(connector_box.min.z(), Catch::Matchers::WithinAbs(7.7 + .05, COORD_EPSILON));
            CHECK_THAT(connector_box.max.z(), Catch::Matchers::WithinAbs(10. + .05, COORD_EPSILON));
            CHECK_THAT(mesh_volume(part_connector) * part_connector->get_scaling_factor().prod(),
                       Catch::Matchers::WithinRel(4.4 * 4.4 * 2.3, 1e-4));
        } else {
            found_upper_connector = true;
            CHECK_THAT(connector_box.min.z(), Catch::Matchers::WithinAbs(10. - .05, COORD_EPSILON));
            CHECK_THAT(connector_box.max.z(), Catch::Matchers::WithinAbs(12.5 - .05, COORD_EPSILON));
            CHECK_THAT(mesh_volume(part_connector) * part_connector->get_scaling_factor().prod(),
                       Catch::Matchers::WithinRel(4.4 * 4.4 * 2.5, 1e-4));
        }
    }

    CHECK(found_lower_connector);
    CHECK(found_upper_connector);
}

TEST_CASE("Contour cut distributes and merges selected model parts", "[CutUtils]")
{
    Model source_model;
    ModelObject *source_object = source_model.add_object();
    ModelVolume *upper_a = source_object->add_volume(make_cube(5., 10., 10.));
    upper_a->set_offset(Vec3d(-7.5, 0., 0.));
    ModelVolume *upper_b = source_object->add_volume(make_cube(5., 10., 10.));
    upper_b->set_offset(Vec3d(-2.5, 0., 0.));
    ModelVolume *lower = source_object->add_volume(make_cube(5., 10., 10.));
    lower->set_offset(Vec3d(5., 0., 0.));
    source_object->add_volume(make_cube(30., 12., 12.), ModelVolumeType::PARAMETER_MODIFIER);
    source_object->add_instance();

    Cut cut(source_object, 0, Transform3d::Identity(), keep_both);
    // The two selected parts form the upper object and the unselected one the lower; the last entry
    // marks the modifier.
    const std::vector<Cut::Part> selected_parts {
        { true,  false },
        { true,  false },
        { false, false },
        { true,  true  },
    };
    const ModelObjectPtrs &parts = cut.perform_by_contour(source_object, selected_parts, 0);

    REQUIRE(parts.size() == 2);
    size_t lower_parts = 0;
    for (const ModelObject *part : parts) {
        size_t model_parts = 0;
        bool has_modifier = false;
        double model_volume = 0.;
        BoundingBoxf3 model_box;
        for (const ModelVolume *volume : part->volumes) {
            if (volume->is_model_part()) {
                ++model_parts;
                model_volume += mesh_volume(volume);
                model_box.merge(world_bounding_box(volume));
            }
            if (volume->type() == ModelVolumeType::PARAMETER_MODIFIER)
                has_modifier = true;
        }
        // The two selected parts are merged into one.
        REQUIRE(model_parts == 1);
        REQUIRE(model_box.defined);
        const bool is_lower_part = model_box.min.x() > COORD_EPSILON;
        lower_parts += is_lower_part;
        // The modifier's original [0, 30] world-space box intersects only the
        // lower selected part ([2.5, 7.5]); distribution must use world bounds.
        CHECK(has_modifier == is_lower_part);
        const double expected_volume = is_lower_part ? 500. : 1000.;
        CHECK_THAT(model_volume, Catch::Matchers::WithinRel(expected_volume, 1e-5));
    }
    CHECK(lower_parts == 1);
}

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

TEST_CASE("Groove width adds the horizontal run of each angled flap", "[CutUtils][CutUtilsGroove]")
{
    // A flap of depth 3 at angle a runs 3 / tan(a) horizontally, so steeper flaps give a narrower mouth.
    // (A zero angle is special-cased as a flap one depth wide; see the flat-flap test.)
    Cut::Groove groove = flat_groove();
    groove.flaps_angle = float(PI / 6.);
    CHECK_THAT(Cut::calculate_groove_width(groove, 10.f), WithinAbs(1.5 + 2. * 3. / std::tan(PI / 6.), 1e-4)); // 11.89
    groove.flaps_angle = float(PI / 3.);
    CHECK_THAT(Cut::calculate_groove_width(groove, 10.f), WithinAbs(1.5 + 2. * 3. / std::tan(PI / 3.), 1e-4)); // 4.96
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
