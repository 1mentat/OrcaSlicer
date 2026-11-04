#include <algorithm>
#include <catch2/catch_all.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>
#include "test_utils.hpp"

#include <cstddef>
#include <cmath>
#include <iterator>
#include <libslic3r/BoundingBox.hpp>
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/TriangleMeshSlicer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/SLAPrint.hpp"
#include "libslic3r/libslic3r.h"
#include "libslic3r/SLA/SupportPoint.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/SLA/SpatIndex.hpp"
#include <limits>
#include "libslic3r/SLA/SupportPointGenerator.hpp"
#include "libslic3r/SLA/Clustering.hpp"
#include <vector>
#include "libslic3r/Line.hpp"
#include "libslic3r/SLA/Hollowing.hpp"

#include "sla_test_utils.hpp"

#include <optional>
#include <utility>

namespace Slic3r { namespace sla {

TEST_CASE("Overhanging point should be supported", "[SupGen]") {

    // Pyramid with 45 deg slope
    TriangleMesh mesh = make_pyramid(10.f, 10.f);
    mesh.rotate_y(float(PI));
    write_debug_obj("sla_supptgen/Pyramid.obj", mesh);

    sla::SupportPoints pts = calc_support_pts(mesh);

    // The overhang, which is the upside-down pyramid's edge
    Vec3f overh{0., 0., -10.};

    REQUIRE(!pts.empty());

    float dist = (overh - pts.front().pos).norm();

    for (const auto &pt : pts)
        dist = std::min(dist, (overh - pt.pos).norm());

    // Should require exactly one support point at the overhang
    REQUIRE(pts.size() > 0);
    REQUIRE(dist < 1.f);
}

double min_point_distance(const sla::SupportPoints &pts)
{
    sla::PointIndex index;

    for (size_t i = 0; i < pts.size(); ++i)
        index.insert(pts[i].pos.cast<double>(), i);

    auto d = std::numeric_limits<double>::max();
    index.foreach([&d, &index](const sla::PointIndexEl &el) {
        auto res = index.nearest(el.first, 2);
        for (const sla::PointIndexEl &r : res)
            if (r.second != el.second)
                d = std::min(d, (el.first - r.first).norm());
    });

    return d;
}

TEST_CASE("Overhanging horizontal surface should be supported", "[SupGen]") {
    double width = 10., depth = 10., height = 1.;

    TriangleMesh mesh = make_cube(width, depth, height);
    mesh.translate(0., 0., 5.); // lift up
    write_debug_obj("sla_supptgen/Cuboid.obj", mesh);

    sla::SupportPointGenerator::Config cfg;
    sla::SupportPoints pts = calc_support_pts(mesh, cfg);

    double mm2 = width * depth;

    REQUIRE(!pts.empty());
    REQUIRE(pts.size() * cfg.support_force() > mm2 * cfg.tear_pressure());
    REQUIRE(min_point_distance(pts) >= cfg.minimal_distance);
}

template<class M> auto&& center_around_bb(M &&mesh)
{
    auto bb = mesh.bounding_box();
    mesh.translate(-bb.center().template cast<float>());

    return std::forward<M>(mesh);
}

TEST_CASE("Overhanging edge should be supported", "[SupGen]") {
    float width = 10.f, depth = 10.f, height = 5.f;

    TriangleMesh mesh = make_prism(width, depth, height);
    mesh.rotate_y(float(PI)); // rotate on its back
    mesh.translate(0., 0., height);
    write_debug_obj("sla_supptgen/Prism.obj", mesh);

    sla::SupportPointGenerator::Config cfg;
    sla::SupportPoints pts = calc_support_pts(mesh, cfg);

    Linef3 overh{ {0.f, -depth / 2.f, 0.f}, {0.f, depth / 2.f, 0.f}};

    // Get all the points closer that 1 mm to the overhanging edge:
    sla::SupportPoints overh_pts; overh_pts.reserve(pts.size());

    std::copy_if(pts.begin(), pts.end(), std::back_inserter(overh_pts),
                 [&overh](const sla::SupportPoint &pt){
                     return line_alg::distance_to(overh, Vec3d{pt.pos.cast<double>()}) < 1.;
                 });

    REQUIRE(overh_pts.size() * cfg.support_force() > overh.length() * cfg.tear_pressure());
    double ddiff = min_point_distance(pts) - cfg.minimal_distance;
    REQUIRE(ddiff > - 0.1 * cfg.minimal_distance);
}

TEST_CASE("Hollowed cube should be supported from the inside", "[SupGen][Hollowed]") {
    constexpr float outer_size = 20.f;
    constexpr float wall_thickness = 2.5f;
    constexpr float cavity_size = outer_size - 2.f * wall_thickness;
    TriangleMesh mesh = make_cube(outer_size, outer_size, outer_size);

    // Use an explicit 2.5 mm wall shell for a meaningful cavity fixture.
    // The separate hollowing contract tests cover voxel generation; this test
    // exercises support placement on the resulting cavity ceiling.
    TriangleMesh cavity = make_cube(cavity_size, cavity_size, cavity_size);
    cavity.translate(wall_thickness, wall_thickness, wall_thickness);
    sla::swap_normals(cavity.its);
    mesh.merge(cavity);

    write_debug_obj("sla_supptgen/cube_hollowed.obj", mesh);

    auto bb = mesh.bounding_box();
    auto h  = float(bb.max.z() - bb.min.z());
    Vec3f mv = bb.center().cast<float>() - Vec3f{0.f, 0.f, 0.5f * h};
    mesh.translate(-mv);

    sla::SupportPointGenerator::Config cfg;
    sla::SupportPoints pts = calc_support_pts(mesh, cfg);
    sla::remove_bottom_points(pts, mesh.bounding_box().min.z() + EPSILON);

    constexpr float cavity_min_xy = -cavity_size / 2.f;
    constexpr float cavity_max_xy =  cavity_size / 2.f;
    constexpr float cavity_ceiling_z = outer_size - wall_thickness;
    const bool has_inside_ceiling_support = std::any_of(
        pts.begin(), pts.end(), [](const sla::SupportPoint &pt) {
            return pt.pos.x() > cavity_min_xy && pt.pos.x() < cavity_max_xy &&
                   pt.pos.y() > cavity_min_xy && pt.pos.y() < cavity_max_xy &&
                   std::abs(pt.pos.z() - cavity_ceiling_z) < 0.1f;
        });

    // The witness must be projected onto the ceiling inside the cavity, rather
    // than merely accepting any point on the cube's exterior or bottom.
    REQUIRE(has_inside_ceiling_support);
}

TEST_CASE("Two parallel plates should be supported", "[SupGen][Hollowed]")
{
    double width = 20., depth = 20., height = 1.;

    TriangleMesh mesh = center_around_bb(make_cube(width + 5., depth + 5., height));
    TriangleMesh mesh_high = center_around_bb(make_cube(width, depth, height));
    mesh_high.translate(0., 0., 10.); // lift up
    mesh.merge(mesh_high);

    write_debug_obj("sla_supptgen/parallel_plates.obj", mesh);

    sla::SupportPointGenerator::Config cfg;
    sla::SupportPoints pts = calc_support_pts(mesh, cfg);
    sla::remove_bottom_points(pts, mesh.bounding_box().min.z() + EPSILON);

    REQUIRE(!pts.empty());
}

namespace {

ClusteredPoints bounded_reject_all_cluster(const std::vector<unsigned> &indices)
{
    unsigned predicate_calls = 0;
    bool budget_exceeded = false;
    struct PredicateCallBudgetExceeded {};
    auto reject_every_pair = [&predicate_calls](const PointIndexEl &, const PointIndexEl &) {
        if (++predicate_calls > 32)
            throw PredicateCallBudgetExceeded{};
        return false;
    };

    ClusteredPoints clusters;
    try {
        clusters = cluster(
            indices,
            [](unsigned) { return Vec3d::Zero(); },
            reject_every_pair,
            2);
    } catch (const PredicateCallBudgetExceeded &) {
        budget_exceeded = true;
    }

    REQUIRE_FALSE(budget_exceeded);
    return clusters;
}

struct SlaSupportGeometryOverrides
{
    std::optional<double> base_diameter;
    std::optional<double> base_height;
    std::optional<double> head_penetration;
};

TriangleMesh build_sla_support_mesh(const SlaSupportGeometryOverrides &overrides)
{
    DynamicPrintConfig config;
    config.apply(SLAFullPrintConfig::defaults());
    config.set_key_value("printer_technology",
                         new ConfigOptionEnum<PrinterTechnology>(ptSLA));
    config.set_deserialize_strict({
        {"layer_height", 1.0},
        {"initial_layer_height", 1.0},
        {"supports_enable", true},
        {"pad_enable", false},
        {"hollowing_enable", false},
        {"support_head_width", 1.0},
        {"support_pillar_diameter", 1.0},
        {"support_base_safety_distance", 1.0},
        {"display_width", 50.0},
        {"display_height", 50.0},
        {"area_fill", 50.0},
        {"exposure_time", 2.0},
        {"initial_exposure_time", 3.0},
        {"min_exposure_time", 0.0},
        {"max_exposure_time", 100.0},
        {"min_initial_exposure_time", 0.0},
        {"max_initial_exposure_time", 100.0},
        {"fast_tilt_time", 1.0},
        {"slow_tilt_time", 2.0},
        {"support_object_elevation", 5.0},
    });
    if (overrides.base_diameter)
        config.set_deserialize_strict({{"support_base_diameter", *overrides.base_diameter}});
    if (overrides.base_height)
        config.set_deserialize_strict({{"support_base_height", *overrides.base_height}});
    if (overrides.head_penetration)
        config.set_deserialize_strict({{"support_head_penetration", *overrides.head_penetration}});

    Model model;
    ModelObject *model_object = model.add_object();
    TriangleMesh model_mesh = make_cube(10.0, 10.0, 4.0);
    model_mesh.translate(Vec3f(20.f, 20.f, 0.f));
    model_object->add_volume(std::move(model_mesh));
    model_object->sla_support_points.emplace_back(25.f, 25.f, 0.f, 0.5f, true);
    model_object->sla_points_status = PointsStatus::UserModified;
    model_object->add_instance();
    model_object->ensure_on_bed();

    SLAPrint print;
    const SLAPrint::ApplyStatus apply_status = print.apply(model, std::move(config));
    REQUIRE(apply_status != SLAPrint::APPLY_STATUS_UNCHANGED);
    REQUIRE(print.objects().size() == 1);
    const auto validation = print.validate().string;
    INFO("SLA support geometry settings validation: " << validation);
    REQUIRE(validation.empty());

    print.process();

    REQUIRE(print.finished());
    const SLAPrintObject *print_object = print.objects().front();
    REQUIRE(print_object->is_step_done(slaposSupportTree));
    TriangleMesh support_mesh = print_object->support_mesh();
    REQUIRE_FALSE(support_mesh.empty());
    REQUIRE(support_mesh.volume() > 0.f);
    return support_mesh;
}

double support_section_x_span_mm(const TriangleMesh &mesh, const float z)
{
    const Polygons section = slice_mesh(mesh.its, z, MeshSlicingParams{});
    REQUIRE_FALSE(section.empty());

    coord_t min_x = std::numeric_limits<coord_t>::max();
    coord_t max_x = std::numeric_limits<coord_t>::lowest();
    for (const Polygon &polygon : section)
        for (const Point &point : polygon.points) {
            min_x = std::min(min_x, point.x());
            max_x = std::max(max_x, point.x());
        }
    REQUIRE(min_x <= max_x);
    return unscaled<double>(max_x) - unscaled<double>(min_x);
}

double support_top_z_mm(const TriangleMesh &mesh)
{
    const BoundingBoxf3 bounds = mesh.bounding_box();
    REQUIRE(bounds.defined);
    return double(bounds.max.z());
}

} // namespace

TEST_CASE("SLA registered support dimensions change public support geometry",
          "[SLAPrint][SLASupportSettingsVariance]")
{
    // Bound clustering before the zero-diameter endpoint reaches SLAPrint.
    const ClusteredPoints progress_preflight = bounded_reject_all_cluster(std::vector<unsigned>{17});
    REQUIRE(progress_preflight.size() == 1);
    REQUIRE(progress_preflight.front().size() == 1);
    REQUIRE(progress_preflight.front().front() == 17);

    // Each variant changes exactly one registered setting from the same default
    // public-print fixture. The support mesh, rather than the private builder
    // configuration, is the measured output.
    const TriangleMesh defaults = build_sla_support_mesh({});
    const TriangleMesh base_diameter_min = build_sla_support_mesh({0.0, {}, {}});
    const TriangleMesh base_diameter_max = build_sla_support_mesh({30.0, {}, {}});
    const TriangleMesh base_height_min = build_sla_support_mesh({{}, 0.0, {}});
    const TriangleMesh base_height_rep = build_sla_support_mesh({{}, 2.0, {}});
    const TriangleMesh head_penetration_min = build_sla_support_mesh({{}, {}, 0.0});
    const TriangleMesh head_penetration_rep = build_sla_support_mesh({{}, {}, 0.8});

    const float section_height = 0.5f;
    const float ground_z = float(defaults.bounding_box().min.z());
    const float section_z = ground_z + section_height;

    const double base_min_span = support_section_x_span_mm(base_diameter_min, section_z);
    const double base_default_span = support_section_x_span_mm(defaults, section_z);
    const double base_max_span = support_section_x_span_mm(base_diameter_max, section_z);

    // The support pillar radius is 0.5 mm. At half of the default 1 mm base
    // height, a straight frustum section has radius halfway between the base
    // radius and pillar radius. The mesh's regular polygon introduces less than
    // 0.05 mm of span error at these radii.
    CHECK_THAT(base_min_span, Catch::Matchers::WithinAbs(1.0, 0.05));
    CHECK_THAT(base_default_span, Catch::Matchers::WithinAbs(2.5, 0.05));
    CHECK_THAT(base_max_span, Catch::Matchers::WithinAbs(15.5, 0.05));

    const double height_min_span = support_section_x_span_mm(base_height_min, section_z);
    const double height_default_span = support_section_x_span_mm(defaults, section_z);
    const double height_rep_span = support_section_x_span_mm(base_height_rep, section_z);
    CHECK_THAT(height_min_span, Catch::Matchers::WithinAbs(1.0, 0.05));
    CHECK_THAT(height_default_span, Catch::Matchers::WithinAbs(2.5, 0.05));
    CHECK_THAT(height_rep_span, Catch::Matchers::WithinAbs(3.25, 0.05));

    const double penetration_min_top = support_top_z_mm(head_penetration_min);
    const double penetration_default_top = support_top_z_mm(defaults);
    const double penetration_rep_top = support_top_z_mm(head_penetration_rep);
    CHECK_THAT(penetration_default_top - penetration_min_top, Catch::Matchers::WithinAbs(0.2, 0.05));
    CHECK_THAT(penetration_rep_top - penetration_min_top, Catch::Matchers::WithinAbs(0.8, 0.05));
}

TEST_CASE("Clustering returns a singleton when the predicate rejects every neighbor",
          "[SupGen][Clustering][Termination]")
{
    constexpr unsigned point_index = 17;
    const unsigned second_point_index = 29;
    const std::vector<std::vector<unsigned>> reject_all_inputs{
        {point_index},
        {point_index, second_point_index}
    };
    for (const std::vector<unsigned> &indices : reject_all_inputs) {
        const ClusteredPoints clusters = bounded_reject_all_cluster(indices);
        REQUIRE(clusters.size() == indices.size());
        std::vector<unsigned> clustered_indices;
        for (const ClusterEl &cluster : clusters) {
            REQUIRE(cluster.size() == 1);
            clustered_indices.push_back(cluster.front());
        }
        std::vector<unsigned> expected_indices = indices;
        std::sort(clustered_indices.begin(), clustered_indices.end());
        std::sort(expected_indices.begin(), expected_indices.end());
        CHECK(clustered_indices == expected_indices);
    }

    const std::vector<unsigned> connected_indices{point_index, second_point_index};
    auto accept_every_pair = [](const PointIndexEl &, const PointIndexEl &) { return true; };
    const ClusteredPoints connected_clusters = cluster(
        connected_indices,
        [](unsigned index) { return Vec3d(double(index), 0.0, 0.0); },
        accept_every_pair,
        2);

    REQUIRE(connected_clusters.size() == 1);
    REQUIRE(connected_clusters.front().size() == 2);
    const ClusterEl &connected_members = connected_clusters.front();
    CHECK(std::count(connected_members.begin(), connected_members.end(), point_index) == 1);
    CHECK(std::count(connected_members.begin(), connected_members.end(), second_point_index) == 1);
}

}} // namespace Slic3r::sla
