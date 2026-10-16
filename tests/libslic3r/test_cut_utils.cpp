#include <array>
#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <vector>

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/CutUtils.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/libslic3r.h"
#include "test_utils.hpp"

using namespace Slic3r;

namespace {

constexpr double COORD_EPSILON = 1e-4;

double mesh_volume(const ModelVolume *volume)
{
    TriangleMesh mesh = volume->mesh();
    return std::abs(static_cast<double>(mesh.volume()));
}

BoundingBoxf3 world_bounding_box(const ModelVolume *volume)
{
    return volume->mesh().transformed_bounding_box(volume->get_matrix());
}

ModelObject *add_centered_cube(Model &model, double x, double y, double z)
{
    ModelObject *object = model.add_object();
    object->add_volume(make_cube(x, y, z));
    object->add_instance();
    return object;
}

} // namespace

TEST_CASE("Cut plane conserves a cube as two watertight halves", "[CutUtils]")
{
    Model source_model;
    ModelObject *source_object = add_centered_cube(source_model, 20., 20., 20.);
    const BoundingBoxf3 source_world_box = world_bounding_box(source_object->volumes.front());
    CHECK_THAT(source_world_box.min.z(), Catch::Matchers::WithinAbs(0., COORD_EPSILON));
    CHECK_THAT(source_world_box.max.z(), Catch::Matchers::WithinAbs(20., COORD_EPSILON));
    const double source_volume = mesh_volume(source_object->volumes.front());

    const ModelObjectCutAttributes attributes = ModelObjectCutAttribute::KeepUpper |
                                                 ModelObjectCutAttribute::KeepLower;
    const Transform3d cut_plane = Geometry::translation_transform(Vec3d(0., 0., 10.));
    Cut cut(source_object, 0, cut_plane, attributes);
    const ModelObjectPtrs &parts = cut.perform_with_plane();

    REQUIRE(parts.size() == 2);
    double split_volume = 0.;
    bool found_lower = false;
    bool found_upper = false;
    for (const ModelObject *part : parts) {
        REQUIRE(part != nullptr);
        REQUIRE(part->volumes.size() == 1);
        const ModelVolume *volume = part->volumes.front();
        REQUIRE(volume->is_model_part());
        REQUIRE_FALSE(volume->mesh().empty());
        CHECK(its_num_open_edges(volume->mesh().its) == 0);

        const BoundingBoxf3 box = world_bounding_box(volume);
        split_volume += mesh_volume(volume);
        if (box.max.z() <= 10. + COORD_EPSILON)
            found_lower = true;
        if (box.min.z() >= 10. - COORD_EPSILON)
            found_upper = true;
    }

    CHECK(found_lower);
    CHECK(found_upper);
    CHECK_THAT(split_volume, Catch::Matchers::WithinRel(source_volume, 1e-5));
}

TEST_CASE("Cut plane honors a single retained side", "[CutUtils]")
{
    Model source_model;
    ModelObject *source_object = add_centered_cube(source_model, 20., 20., 20.);

    const Transform3d cut_plane = Geometry::translation_transform(Vec3d(0., 0., 10.));
    Cut cut(source_object, 0, cut_plane, ModelObjectCutAttribute::KeepUpper);
    const ModelObjectPtrs &parts = cut.perform_with_plane();

    REQUIRE(parts.size() == 1);
    REQUIRE(parts.front()->volumes.size() == 1);
    const BoundingBoxf3 box = world_bounding_box(parts.front()->volumes.front());
    CHECK(box.min.z() >= 10. - COORD_EPSILON);
    CHECK(its_num_open_edges(parts.front()->volumes.front()->mesh().its) == 0);
}

TEST_CASE("Cut plane preserves modifiers and splits an unprocessed dowel", "[CutUtils]")
{
    Model source_model;
    ModelObject *source_object = add_centered_cube(source_model, 20., 20., 20.);

    ModelVolume *modifier = source_object->add_volume(make_cube(8., 8., 8.), ModelVolumeType::PARAMETER_MODIFIER);
    modifier->set_offset(Vec3d(0., 0., 10.));
    ModelVolume *connector = source_object->add_volume(make_cube(4., 4., 4.), ModelVolumeType::NEGATIVE_VOLUME);
    connector->set_offset(Vec3d(0., 0., 10.));
    connector->cut_info = ModelVolume::CutInfo(CutConnectorType::Dowel, 0.1f, 0.2f, false);

    const ModelObjectCutAttributes attributes = ModelObjectCutAttribute::KeepUpper |
                                                 ModelObjectCutAttribute::KeepLower;
    const Transform3d cut_plane = Geometry::translation_transform(Vec3d(0., 0., 10.));
    Cut cut(source_object, 0, cut_plane, attributes);
    const ModelObjectPtrs &parts = cut.perform_with_plane();

    REQUIRE(parts.size() == 2);
    bool found_lower_connector = false;
    bool found_upper_connector = false;
    for (const ModelObject *part : parts) {
        bool found_modifier = false;
        const ModelVolume *part_connector = nullptr;
        for (const ModelVolume *volume : part->volumes) {
            if (volume->type() == ModelVolumeType::PARAMETER_MODIFIER)
                found_modifier = true;
            if (volume->is_cut_connector())
                part_connector = volume;
        }
        CHECK(found_modifier);
        REQUIRE(part_connector != nullptr);
        CHECK(part_connector->is_negative_volume());
        CHECK(part_connector->cut_info.connector_type == CutConnectorType::Dowel);
        CHECK_THAT(part_connector->cut_info.radius_tolerance, Catch::Matchers::WithinAbs(0.1, 1e-6));
        CHECK_THAT(part_connector->cut_info.height_tolerance, Catch::Matchers::WithinAbs(0.2, 1e-6));

        const BoundingBoxf3 connector_box = world_bounding_box(part_connector);
        // CutUtils offsets the two preview halves by 0.05 mm in opposite
        // directions, so classify the connector halves by their centers.
        if (connector_box.center().z() < 10.)
            found_lower_connector = true;
        if (connector_box.center().z() > 10.)
            found_upper_connector = true;
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

    const ModelObjectCutAttributes attributes = ModelObjectCutAttribute::KeepUpper |
                                                 ModelObjectCutAttribute::KeepLower;
    Cut cut(source_object, 0, Transform3d::Identity(), attributes);
    const std::vector<Cut::Part> selected_parts {
        { true,  false },
        { true,  false },
        { false, false },
        { true,  true  },
    };
    const ModelObjectPtrs &parts = cut.perform_by_contour(source_object, selected_parts, 0);

    REQUIRE(parts.size() == 2);
    for (const ModelObject *part : parts) {
        size_t model_parts = 0;
        bool has_modifier = false;
        double model_volume = 0.;
        BoundingBoxf3 model_box;
        bool model_box_initialized = false;
        for (const ModelVolume *volume : part->volumes) {
            if (volume->is_model_part()) {
                ++model_parts;
                model_volume += mesh_volume(volume);
                if (!model_box_initialized) {
                    model_box = world_bounding_box(volume);
                    model_box_initialized = true;
                } else {
                    model_box.merge(world_bounding_box(volume));
                }
            }
            if (volume->type() == ModelVolumeType::PARAMETER_MODIFIER)
                has_modifier = true;
        }
        REQUIRE(model_parts == 1);
        REQUIRE(model_box_initialized);
        const bool is_lower_part = model_box.min.x() > COORD_EPSILON;
        // The modifier's original [0, 30] world-space box intersects only the
        // lower selected part ([2.5, 7.5]); distribution must use world bounds.
        CHECK(has_modifier == is_lower_part);
        const double expected_volume = is_lower_part ? 500. : 1000.;
        CHECK_THAT(model_volume, Catch::Matchers::WithinRel(expected_volume, 1e-5));
    }
}

TEST_CASE("Cut plane with no retained side returns no objects", "[CutUtils]")
{
    Model source_model;
    ModelObject *source_object = add_centered_cube(source_model, 10., 10., 10.);

    Cut cut(source_object, 0, Transform3d::Identity(), ModelObjectCutAttribute::KeepAsParts);
    CHECK(cut.perform_with_plane().empty());
}

namespace groove_contracts {
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

Cut::Groove base_groove()
{
    Cut::Groove groove;
    groove.depth = 2.f;
    groove.width = 2.f;
    groove.flaps_angle = 0.f;
    groove.angle = 0.f;
    groove.depth_tolerance = .1f;
    groove.width_tolerance = .1f;
    return groove;
}

void require_closed_nonempty(const ModelObject *object)
{
    REQUIRE(object != nullptr);
    REQUIRE(!object->volumes.empty());
    for (const ModelVolume *volume : object->volumes) {
        REQUIRE(volume != nullptr);
        REQUIRE(!volume->mesh().empty());
        CHECK(its_num_open_edges(volume->mesh().its) == 0);
        CHECK(std::isfinite(mesh_volume(volume)));
        CHECK(mesh_volume(volume) > 0.);
    }
}

double object_volume(const ModelObjectPtrs &objects)
{
    double total = 0.;
    for (const ModelObject *object : objects) {
        require_closed_nonempty(object);
        for (const ModelVolume *volume : object->volumes)
            total += mesh_volume(volume);
    }
    return total;
}

size_t object_triangle_count(const ModelObjectPtrs &objects)
{
    size_t total = 0;
    for (const ModelObject *object : objects)
        for (const ModelVolume *volume : object->volumes)
            total += volume->mesh().its.indices.size();
    return total;
}


TEST_CASE("Groove width for flat flaps includes the two full-depth mouths", "[CutUtils][CutUtilsGroove]")
{
    Cut::Groove groove = base_groove();
    CHECK_THAT(Cut::calculate_groove_width(groove, 10.f), WithinAbs(6.f, 1e-6));
}

TEST_CASE("Groove width grows with a positive taper angle and radius", "[CutUtils][CutUtilsGroove]")
{
    Cut::Groove groove = base_groove();
    groove.angle = .25f;
    const double small = Cut::calculate_groove_width(groove, 10.f);
    const double large = Cut::calculate_groove_width(groove, 20.f);
    CHECK(large > small);
    CHECK_THAT(large - small, WithinAbs(22.5 * std::tan(.25), .02));
}

TEST_CASE("Groove width uses the flap angle to reduce the projected mouth width", "[CutUtils][CutUtilsGroove]")
{
    Cut::Groove groove = base_groove();
    groove.flaps_angle = float(PI / 6.);
    const double expected = 2. + 2. * 2. * std::cos(PI / 6.) / std::sin(PI / 6.);
    CHECK_THAT(Cut::calculate_groove_width(groove, 10.f), WithinAbs(expected, 1e-5));
}

TEST_CASE("Groove width tolerance fields do not change the cutting mouth", "[CutUtils][CutUtilsGroove]")
{
    Cut::Groove first = base_groove();
    Cut::Groove second = first;
    second.depth_tolerance = 2.f;
    second.width_tolerance = 3.f;
    CHECK_THAT(Cut::calculate_groove_width(first, 10.f), WithinAbs(
        Cut::calculate_groove_width(second, 10.f), 1e-6));
}

TEST_CASE("Groove depth tolerance changes the retained cut geometry", "[CutUtils][CutUtilsGroove]")
{
    auto retained_volume = [](float depth_tolerance) {
        Model source;
        ModelObject *object = add_centered_cube(source, 20., 20., 20.);
        Cut::Groove groove = base_groove();
        groove.depth_tolerance = depth_tolerance;
        groove.flaps_angle = .35f;
        groove.angle = .2f;
        const ModelObjectCutAttributes attributes = ModelObjectCutAttribute::KeepUpper |
                                                     ModelObjectCutAttribute::KeepLower;
        Cut cut(object, 0, Geometry::translation_transform(Vec3d(10., 10., 10.)), attributes);
        const ModelObjectPtrs &result = cut.perform_with_groove(
            groove, Transform3d::Identity(), 1, 0.f, 10.f);
        REQUIRE(result.size() == 2);
        return object_volume(result);
    };
    const double nominal = retained_volume(.1f);
    const double wide = retained_volume(1.5f);
    CHECK(std::isfinite(nominal));
    CHECK(std::isfinite(wide));
    CHECK(std::abs(nominal - wide) > 1e-3);
}

TEST_CASE("A groove with perpendicular flaps leaves two closed nonempty sides with reduced total volume", "[CutUtils][CutUtilsGroove]")
{
    Model source;
    ModelObject *object = add_centered_cube(source, 20., 20., 20.);
    const double source_volume = mesh_volume(object->volumes.front());
    const ModelObjectCutAttributes attributes = ModelObjectCutAttribute::KeepUpper |
                                                 ModelObjectCutAttribute::KeepLower;
    Cut cut(object, 0, Geometry::translation_transform(Vec3d(10., 10., 10.)), attributes);
    Cut::Groove groove = base_groove();
    groove.flaps_angle = float(PI / 2.);
    const ModelObjectPtrs &result = cut.perform_with_groove(groove, Transform3d::Identity(), 1, 0.f, 10.f);
    REQUIRE(result.size() == 2);
    const double result_volume = object_volume(result);
    CHECK(result_volume < source_volume);
    CHECK(result_volume > .5 * source_volume);
}

TEST_CASE("Multiple grooves retain a closed finite result for every requested slot", "[CutUtils][CutUtilsGroove]")
{
    Model one_source;
    Model two_source;
    ModelObject *one_object = add_centered_cube(one_source, 40., 20., 20.);
    ModelObject *two_object = add_centered_cube(two_source, 40., 20., 20.);
    Cut::Groove groove = base_groove();
    groove.flaps_angle = .35f;
    groove.angle = .2f;
    const ModelObjectCutAttributes attributes = ModelObjectCutAttribute::KeepUpper |
                                                 ModelObjectCutAttribute::KeepLower;
    Cut one_cut(one_object, 0, Geometry::translation_transform(Vec3d(20., 10., 10.)), attributes);
    Cut two_cut(two_object, 0, Geometry::translation_transform(Vec3d(20., 10., 10.)), attributes);
    const ModelObjectPtrs &one = one_cut.perform_with_groove(groove, Transform3d::Identity(), 1, 0.f, 10.f);
    const ModelObjectPtrs &two = two_cut.perform_with_groove(groove, Transform3d::Identity(), 2, 2.f, 10.f);
    REQUIRE(one.size() == 2);
    REQUIRE(two.size() == 2);
    const double one_volume = object_volume(one);
    const double two_volume = object_volume(two);
    const size_t one_triangles = object_triangle_count(one);
    const size_t two_triangles = object_triangle_count(two);
    CHECK(std::isfinite(one_volume));
    CHECK(std::isfinite(two_volume));
    CHECK(one_volume > 0.);
    CHECK(two_volume > 0.);
    CHECK(two_volume < one_volume - 1e-3);
    CHECK(two_volume > .5 * 40. * 20. * 20.);
    CHECK(two_triangles != one_triangles);
}

TEST_CASE("Angled flaps produce a finite closed groove result", "[CutUtils][CutUtilsGroove]")
{
    Model source;
    ModelObject *object = add_centered_cube(source, 20., 20., 20.);
    const double source_volume = mesh_volume(object->volumes.front());
    Cut::Groove groove = base_groove();
    groove.flaps_angle = .35f;
    groove.angle = .2f;
    const ModelObjectCutAttributes attributes = ModelObjectCutAttribute::KeepUpper |
                                                 ModelObjectCutAttribute::KeepLower;
    Cut cut(object, 0, Geometry::translation_transform(Vec3d(10., 10., 10.)), attributes);
    const ModelObjectPtrs &result = cut.perform_with_groove(groove, Transform3d::Identity(), 1, 0.f, 10.f);
    REQUIRE(result.size() == 2);
    const double result_volume = object_volume(result);
    CHECK(std::isfinite(result_volume));
    CHECK(result_volume > 0.);
    CHECK(result_volume < source_volume);
}

TEST_CASE("Rotating a centered groove swaps its horizontal bounds and preserves volume", "[CutUtils][CutUtilsGroove]")
{
    auto signature = [](double angle) {
        Model source;
        ModelObject *object = add_centered_cube(source, 20., 20., 20.);
        const Transform3d rotation = Geometry::rotation_transform(Vec3d(0., 0., angle));
        const Transform3d center = Geometry::translation_transform(Vec3d(10., 10., 10.));
        const ModelObjectCutAttributes attributes = ModelObjectCutAttribute::KeepUpper |
                                                     ModelObjectCutAttribute::KeepLower;
        Cut cut(object, 0, center * rotation, attributes);
        Cut::Groove groove = base_groove();
        groove.flaps_angle = .35f;
        groove.angle = .2f;
        const ModelObjectPtrs &result = cut.perform_with_groove(groove, rotation, 1, 0.f, 10.f);
        REQUIRE(result.size() == 2);
        std::vector<std::array<double, 4>> shapes;
        for (const ModelObject *part : result) {
            require_closed_nonempty(part);
            REQUIRE(part->volumes.size() == 1);
            const ModelVolume *volume = part->volumes.front();
            const Vec3d size = volume->mesh().transformed_bounding_box(volume->get_matrix()).size();
            shapes.push_back({mesh_volume(volume), size.x(), size.y(), size.z()});
        }
        std::sort(shapes.begin(), shapes.end());
        return shapes;
    };
    const auto original = signature(0.);
    const auto rotated = signature(PI / 2.);
    REQUIRE(original.size() == rotated.size());
    for (size_t i = 0; i < original.size(); ++i) {
        CHECK_THAT(rotated[i][0], WithinRel(original[i][0], 1e-5));
        CHECK_THAT(rotated[i][1], WithinAbs(original[i][2], 1e-4));
        CHECK_THAT(rotated[i][2], WithinAbs(original[i][1], 1e-4));
        CHECK_THAT(rotated[i][3], WithinAbs(original[i][3], 1e-4));
    }
}

TEST_CASE("Keep-as-parts retains a nonempty closed groove result", "[CutUtils][CutUtilsGroove]")
{
    Model source;
    ModelObject *object = add_centered_cube(source, 20., 20., 20.);
    const ModelObjectCutAttributes attributes = ModelObjectCutAttribute::KeepUpper |
                                                 ModelObjectCutAttribute::KeepLower;
    Cut cut(object, 0, Geometry::translation_transform(Vec3d(10., 10., 10.)), attributes);
    Cut::Groove groove = base_groove();
    groove.flaps_angle = .35f;
    groove.angle = .2f;
    const ModelObjectPtrs &result = cut.perform_with_groove(groove, Transform3d::Identity(), 1, 0.f, 10.f, true);
    REQUIRE(result.size() == 2);
    const ModelObject *combined = result.front();
    require_closed_nonempty(combined);
    size_t upper_volumes = 0;
    size_t lower_volumes = 0;
    double upper_volume = 0.;
    double lower_volume = 0.;
    for (const ModelVolume *volume : combined->volumes) {
        if (volume->cut_info.is_from_upper) {
            ++upper_volumes;
            upper_volume += mesh_volume(volume);
        } else {
            ++lower_volumes;
            lower_volume += mesh_volume(volume);
        }
    }
    CHECK(upper_volumes > 0);
    CHECK(lower_volumes > 0);
    CHECK(upper_volume > 0.);
    CHECK(lower_volume > 0.);
    CHECK(upper_volume + lower_volume < 8000.);
    CHECK(upper_volume + lower_volume > 4000.);
}

} // namespace groove_contracts
