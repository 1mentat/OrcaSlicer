#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <cstddef>
#include <vector>

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/CutUtils.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/TriangleMesh.hpp"

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

        const BoundingBoxf3 box = world_bounding_box(volume);
        CHECK_THAT(box.size().x(), Catch::Matchers::WithinAbs(20., COORD_EPSILON));
        CHECK_THAT(box.size().y(), Catch::Matchers::WithinAbs(20., COORD_EPSILON));
        if (box.size().z() < 10.) {
            ++lower_count;
            CHECK_THAT(box.size().z(), Catch::Matchers::WithinAbs(6., COORD_EPSILON));
            CHECK_THAT(mesh_volume(volume), Catch::Matchers::WithinRel(20. * 20. * 6., 1e-5));
        } else {
            ++upper_count;
            CHECK_THAT(box.size().z(), Catch::Matchers::WithinAbs(14., COORD_EPSILON));
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
