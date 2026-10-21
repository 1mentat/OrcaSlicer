#include <algorithm>
#include <catch2/catch_all.hpp>

#include <cstdlib>
#include <cstddef>
#include <memory>
#include <utility>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/libslic3r.h"
#include "libslic3r/Line.hpp"
#include "libslic3r/Polyline.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/BoundingBox.hpp"


using namespace Slic3r;

static inline Slic3r::Point3 random_point3(float LO=-50, float HI=50)
{
    Vec3f pt = Vec3f(LO, LO, LO) + (Vec3d(rand(), rand(), rand()) * (HI-LO) / RAND_MAX).cast<float>();
	return Point3(pt.cast<coord_t>());
}


// build a sample extrusion entity collection with random start and end points.
static Slic3r::ExtrusionPath random_path(size_t length = 20, float LO = -50, float HI = 50)
{
    ExtrusionPath t {erPerimeter, 1.0, 1.0, 1.0};
    for (size_t j = 0; j < length; ++ j)
        t.polyline.append(random_point3(LO, HI));
    return t;
}

static Slic3r::ExtrusionPaths random_paths(size_t count = 10, size_t length = 20, float LO = -50, float HI = 50)
{
    Slic3r::ExtrusionPaths p;
    for (size_t i = 0; i < count; ++ i)
        p.push_back(random_path(length, LO, HI));
    return p;
}

SCENARIO("Polygon flattening", "[ExtrusionEntity]") {
    srand(0xDEADBEEF); // consistent seed for test reproducibility.

    // Generate one specific random path set and save it for later comparison
    Slic3r::ExtrusionPaths nosort_path_set = random_paths();

    Slic3r::ExtrusionEntityCollection sub_nosort;
    sub_nosort.append(nosort_path_set);
    sub_nosort.no_sort = true;

    Slic3r::ExtrusionEntityCollection sub_sort;
    sub_sort.no_sort = false;
    sub_sort.append(random_paths());

    GIVEN("A Extrusion Entity Collection with a child that has one child that is marked as no-sort") {
        Slic3r::ExtrusionEntityCollection sample;
        Slic3r::ExtrusionEntityCollection output;

        sample.append(sub_sort);
        sample.append(sub_nosort);
        sample.append(sub_sort);

        WHEN("The EEC is flattened with default options (preserve_order=false)") {
			output = sample.flatten();
            THEN("The output EEC contains no Extrusion Entity Collections") {
                CHECK(std::count_if(output.entities.cbegin(), output.entities.cend(), [=](const ExtrusionEntity* e) {return e->is_collection();}) == 0);
            }
        }
        WHEN("The EEC is flattened with preservation (preserve_order=true)") {
			output = sample.flatten(true);
            THEN("The output EECs contains one EEC.") {
                CHECK(std::count_if(output.entities.cbegin(), output.entities.cend(), [=](const ExtrusionEntity* e) {return e->is_collection();}) == 1);
            }
            AND_THEN("The ordered EEC contains the same order of elements than the original") {
                // find the entity in the collection
                for (auto e : output.entities)
                    if (e->is_collection()) {
                        ExtrusionEntityCollection *temp = dynamic_cast<ExtrusionEntityCollection*>(e);
                        // check each Extrusion path against nosort_path_set to see if the first and last match the same
                        CHECK(nosort_path_set.size() == temp->entities.size());
                        for (size_t i = 0; i < nosort_path_set.size(); ++ i) {
                            CHECK(temp->entities[i]->first_point() == nosort_path_set[i].first_point());
                            CHECK(temp->entities[i]->last_point() == nosort_path_set[i].last_point());
                        }
                    }
            }
        }
    }
}

static ExtrusionPaths straight_path(const std::vector<double> &xs)
{
    ExtrusionPath path{erExternalPerimeter, 1.0, 0.45f, 0.2f};
    for (double x : xs)
        path.polyline.append(Point3::new_scale(x, 0., 0.));
    return {path};
}

TEST_CASE("Scarf ramp ends on the next loop vertex instead of leaving a short stub", "[ExtrusionEntity]")
{
    using Catch::Matchers::WithinAbs;
    // A 20 mm scarf in 10 steps: a remainder shorter than half a 2 mm step is snapped forward.
    const double slope_length = 20.;
    const double max_segment  = scale_(slope_length / 10);

    SECTION("a 0.09 mm remainder extends the ramp to the vertex") {
        ExtrusionPaths     paths = straight_path({0., 5., 10., 15., 20.09, 25., 30.});
        ExtrusionLoopSloped loop(paths, 0., slope_length, max_segment, 0.);
        REQUIRE(loop.starts.size() == 1);
        REQUIRE(loop.ends.size() == 1);
        REQUIRE(loop.paths.size() == 1);
        CHECK_THAT(unscale_(loop.starts.front().polyline.last_point().x()), WithinAbs(20.09, 1e-3));
        CHECK_THAT(unscale_(loop.ends.front().polyline.last_point().x()), WithinAbs(20.09, 1e-3));
        CHECK_THAT(unscale_(loop.paths.front().polyline.first_point().x()), WithinAbs(20.09, 1e-3));
        CHECK_THAT(unscale_(loop.paths.front().polyline.lines().front().length()), WithinAbs(4.91, 1e-3));
    }

    SECTION("a remainder longer than half a step keeps the exact scarf length") {
        ExtrusionPaths     paths = straight_path({0., 5., 10., 15., 21.5, 25., 30.});
        ExtrusionLoopSloped loop(paths, 0., slope_length, max_segment, 0.);
        REQUIRE(loop.starts.size() == 1);
        REQUIRE(loop.paths.size() == 1);
        CHECK_THAT(unscale_(loop.starts.front().polyline.last_point().x()), WithinAbs(20., 1e-3));
        CHECK_THAT(unscale_(loop.paths.front().polyline.first_point().x()), WithinAbs(20., 1e-3));
        CHECK_THAT(unscale_(loop.paths.front().polyline.lines().front().length()), WithinAbs(1.5, 1e-3));
    }

    SECTION("the ramp never grows by more than a millimetre, whatever the step size") {
        ExtrusionPaths     paths = straight_path({0., 5., 10., 15., 21.5, 25., 30.});
        ExtrusionLoopSloped loop(paths, 0., slope_length, scale_(slope_length), 0.); // a single 20 mm step
        REQUIRE(loop.paths.size() == 1);
        CHECK_THAT(unscale_(loop.starts.front().polyline.last_point().x()), WithinAbs(20., 1e-3));
    }

    SECTION("snapping onto the path's last vertex leaves no single-point flat path") {
        ExtrusionPaths     paths = straight_path({0., 5., 10., 15., 20.5});
        ExtrusionLoopSloped loop(paths, 0., slope_length, max_segment, 0.);
        REQUIRE(loop.starts.size() == 1);
        CHECK(loop.paths.empty());
        CHECK_THAT(unscale_(loop.starts.front().polyline.last_point().x()), WithinAbs(20.5, 1e-3));
    }
}

TEST_CASE("Scarf loop drops the micro segments the seam insertion leaves at both ends", "[ExtrusionEntity]")
{
    using Catch::Matchers::WithinAbs;
    const double slope_length = 20.;
    const double max_segment  = scale_(slope_length / 10);

    SECTION("a 3 um segment at each end of a single path is removed, the seam point stays") {
        ExtrusionPaths     paths = straight_path({0., 0.003, 5., 10., 15., 21.5, 25., 29.997, 30.});
        ExtrusionLoopSloped loop(paths, 0., slope_length, max_segment, 0.);
        REQUIRE(loop.starts.size() == 1);
        REQUIRE(loop.paths.size() == 1);
        const Polyline3 &start = loop.starts.front().polyline;
        CHECK_THAT(unscale_(start.first_point().x()), WithinAbs(0., 1e-4));
        CHECK_THAT(unscale_(start.lines().front().length()), WithinAbs(1.25, 1e-3)); // 5 mm halved twice
        const Polyline3 &flat = loop.paths.front().polyline;
        CHECK_THAT(unscale_(flat.last_point().x()), WithinAbs(30., 1e-4));
        CHECK_THAT(unscale_(flat.lines().back().length()), WithinAbs(5., 1e-3));
    }

    SECTION("a micro path of its own is dropped and the neighbour ends at the seam point") {
        ExtrusionPaths paths = straight_path({0., 0.003});
        ExtrusionPaths rest  = straight_path({0.003, 5., 10., 15., 21.5, 25., 30.});
        paths.push_back(rest.front());
        ExtrusionLoopSloped loop(paths, 0., slope_length, max_segment, 0.);
        REQUIRE(loop.starts.size() == 1);
        CHECK_THAT(unscale_(loop.starts.front().polyline.first_point().x()), WithinAbs(0., 1e-4));
        CHECK_THAT(unscale_(loop.starts.front().polyline.lines().front().length()), WithinAbs(1.25, 1e-3));
    }

    SECTION("a scarf covering the whole loop still ends at full flow after a trim") {
        // The caller sizes the scarf from the untrimmed loop: 10.003 mm here, 10 mm after the trim.
        ExtrusionPaths     paths = straight_path({0., 0.003, 5., 10.});
        ExtrusionLoopSloped loop(paths, 0., 10.003, max_segment, 0.);
        REQUIRE(loop.starts.size() == 1);
        CHECK(loop.paths.empty());
        CHECK_THAT(loop.starts.back().slope_end.e_ratio, WithinAbs(1., 1e-9));
        CHECK_THAT(unscale_(loop.starts.back().polyline.last_point().x()), WithinAbs(10., 1e-4));
    }

    SECTION("segments longer than the tolerance are kept") {
        ExtrusionPaths     paths = straight_path({0., 0.3, 5., 10., 15., 21.5, 25., 29.7, 30.});
        ExtrusionLoopSloped loop(paths, 0., slope_length, max_segment, 0.);
        REQUIRE(loop.paths.size() == 1);
        CHECK_THAT(unscale_(loop.starts.front().polyline.lines().front().length()), WithinAbs(0.3, 1e-3));
        CHECK_THAT(unscale_(loop.paths.front().polyline.lines().back().length()), WithinAbs(0.3, 1e-3));
    }
}


TEST_CASE("Multipath footprints follow each path width and keep the minimum flow", "[ExtrusionEntity]")
{
    using Catch::Matchers::WithinAbs;
    ExtrusionPath first{erExternalPerimeter, 0.07, 0.4f, 0.2f};
    first.polyline.append(Point3::new_scale(0., 0., 0.));
    first.polyline.append(Point3::new_scale(10., 0., 0.));
    ExtrusionPath second{erExternalPerimeter, 0.04, 0.8f, 0.2f};
    second.polyline.append(Point3::new_scale(10., 0., 0.));
    second.polyline.append(Point3::new_scale(20., 0., 0.));

    ExtrusionMultiPath multipath(ExtrusionPaths{first, second});
    CHECK_THAT(multipath.min_mm3_per_mm(), WithinAbs(0.04, 1e-9));

    const Polygons by_width = multipath.polygons_covered_by_width();
    const Polygons by_spacing = multipath.polygons_covered_by_spacing();
    REQUIRE(by_width.size() == 2);
    REQUIRE(by_spacing.size() == 2);

    auto ordered_extents = [](const Polygons &polygons) {
        std::vector<BoundingBox> extents;
        extents.reserve(polygons.size());
        for (const Polygon &polygon : polygons)
            extents.emplace_back(get_extents(polygon));
        std::sort(extents.begin(), extents.end(), [](const BoundingBox &lhs, const BoundingBox &rhs) {
            return lhs.center().x() < rhs.center().x();
        });
        return extents;
    };
    const std::vector<BoundingBox> width_extents = ordered_extents(by_width);
    const std::vector<BoundingBox> spacing_extents = ordered_extents(by_spacing);
    for (size_t i = 0; i < width_extents.size(); ++i) {
        REQUIRE(width_extents[i].defined);
        REQUIRE(spacing_extents[i].defined);
    }

    // Open butt caps preserve the two centerline intervals; the literal widths
    // determine their independent transverse bounds below.
    CHECK_THAT(unscale_(width_extents[0].min.x()), WithinAbs(0., 0.001));
    CHECK_THAT(unscale_(width_extents[0].max.x()), WithinAbs(10., 0.001));
    CHECK_THAT(unscale_(width_extents[1].min.x()), WithinAbs(10., 0.001));
    CHECK_THAT(unscale_(width_extents[1].max.x()), WithinAbs(20., 0.001));
    CHECK_THAT(unscale_(width_extents[0].center().x()), WithinAbs(5., 0.01));
    CHECK_THAT(unscale_(width_extents[1].center().x()), WithinAbs(15., 0.01));
    CHECK_THAT(unscale_(width_extents[0].min.y()), WithinAbs(-0.2, 0.001));
    CHECK_THAT(unscale_(width_extents[0].max.y()), WithinAbs(0.2, 0.001));
    CHECK_THAT(unscale_(width_extents[1].min.y()), WithinAbs(-0.4, 0.001));
    CHECK_THAT(unscale_(width_extents[1].max.y()), WithinAbs(0.4, 0.001));
    CHECK_THAT(unscale_(spacing_extents[0].center().x()), WithinAbs(5., 0.01));
    CHECK_THAT(unscale_(spacing_extents[1].center().x()), WithinAbs(15., 0.01));
    for (size_t i = 0; i < width_extents.size(); ++i) {
        CHECK(spacing_extents[i].min.y() > width_extents[i].min.y());
        CHECK(spacing_extents[i].max.y() < width_extents[i].max.y());
    }
}

TEST_CASE("Contoured path operations preserve vertex heights and geometry", "[ExtrusionEntity]")
{
    using Catch::Matchers::WithinAbs;
    const std::vector<Point3> source_points{
        Point3::new_scale(0., 0., 0.), Point3::new_scale(5., 1., 0.),
        Point3::new_scale(10., 0., 0.), Point3::new_scale(15., 2., 0.)};
    const std::vector<double> source_diffs{0.1, 0.2, 0.3, 0.4};
    ExtrusionPath metadata{erExternalPerimeter, 0.06, 0.45f, 0.2f};
    metadata.z_contoured = true;
    Polyline3 polyline;
    for (const Point3 &point : source_points)
        polyline.append(point);
    ExtrusionPathContoured path(std::move(polyline), metadata, std::vector<double>(source_diffs));

    // The same 100 mm tolerance removes the small zigzags from a plain path.
    ExtrusionPath plain{erExternalPerimeter, 0.06, 0.45f, 0.2f};
    for (const Point3 &point : source_points)
        plain.polyline.append(point);
    plain.simplify(scale_(100.));
    REQUIRE(plain.polyline.points.size() == 2);
    CHECK(plain.polyline.points.front() == Point3::new_scale(0., 0., 0.));
    CHECK(plain.polyline.points.back() == source_points.back());

    path.simplify(scale_(100.));
    path.simplify_by_fitting_arc(scale_(100.));
    auto check_pairs = [&](const ExtrusionPathContoured &candidate,
                           const std::vector<Point3> &expected_points,
                           const std::vector<double> &expected_diffs) {
        REQUIRE(candidate.polyline.points.size() == expected_points.size());
        REQUIRE(candidate.z_diffs.size() == expected_diffs.size());
        for (size_t i = 0; i < expected_points.size(); ++i) {
            CHECK(candidate.polyline.points[i] == expected_points[i]);
            CHECK_THAT(candidate.z_diffs[i], WithinAbs(expected_diffs[i], 1e-12));
        }
    };
    check_pairs(path, source_points, source_diffs);

    std::unique_ptr<ExtrusionEntity> cloned(path.clone());
    const auto *cloned_contour = dynamic_cast<const ExtrusionPathContoured *>(cloned.get());
    REQUIRE(cloned_contour != nullptr);
    check_pairs(*cloned_contour, source_points, source_diffs);

    const std::vector<Point3> reversed_points(source_points.rbegin(), source_points.rend());
    const std::vector<double> reversed_diffs(source_diffs.rbegin(), source_diffs.rend());
    path.reverse();
    check_pairs(path, reversed_points, reversed_diffs);

    std::unique_ptr<ExtrusionEntity> moved(path.clone_move());
    const auto *moved_contour = dynamic_cast<const ExtrusionPathContoured *>(moved.get());
    REQUIRE(moved_contour != nullptr);
    check_pairs(*moved_contour, reversed_points, reversed_diffs);
}

TEST_CASE("Extrusion loop clipping removes whole trailing paths before trimming the last path", "[ExtrusionEntity]")
{
    using Catch::Matchers::WithinAbs;
    auto make_path = [](Point from, Point to, double flow) {
        ExtrusionPath path{erExternalPerimeter, flow, 0.45f, 0.2f};
        path.polyline.append(Point3(from, coord_t(0)));
        path.polyline.append(Point3(to, coord_t(0)));
        return path;
    };
    ExtrusionPaths paths{
        make_path(Point::new_scale(0., 0.), Point::new_scale(10., 0.), 0.07),
        make_path(Point::new_scale(10., 0.), Point::new_scale(10., 8.), 0.04),
        make_path(Point::new_scale(10., 8.), Point::new_scale(0., 8.), 0.06),
        make_path(Point::new_scale(0., 8.), Point::new_scale(0., 0.), 0.05)};
    ExtrusionLoop loop(paths);
    ExtrusionPaths clipped;
    loop.clip_end(scale_(12.), &clipped);

    REQUIRE(clipped.size() == 3);
    CHECK_THAT(unscale_(clipped.back().length()), WithinAbs(6., 0.001));
    CHECK(clipped.back().first_point() == Point::new_scale(10., 8.));
    CHECK(clipped.back().last_point() == Point::new_scale(4., 8.));
    CHECK_THAT(unscale_(loop.length()), WithinAbs(36., 0.001));
    CHECK_THAT(loop.min_mm3_per_mm(), WithinAbs(0.04, 1e-9));
}

TEST_CASE("Scarf endpoint clipping removes complete segments before the partial boundary segment", "[ExtrusionEntity]")
{
    using Catch::Matchers::WithinAbs;
    ExtrusionPaths paths;
    const std::vector<double> xs{0., 5., 10., 15., 20., 30.};
    for (size_t i = 1; i < xs.size(); ++i) {
        ExtrusionPath path{erExternalPerimeter, 0.05, 0.45f, 0.2f};
        path.polyline.append(Point3::new_scale(xs[i - 1], 0., 0.));
        path.polyline.append(Point3::new_scale(xs[i], 0., 0.));
        paths.emplace_back(std::move(path));
    }
    ExtrusionLoopSloped scarf(paths, 0., 20., scale_(5.), 0.);
    REQUIRE(scarf.starts.size() == 4);
    REQUIRE(scarf.ends.size() == 4);
    REQUIRE(scarf.paths.size() == 1);

    scarf.clip_front(scale_(6.));
    scarf.clip_end(scale_(6.));

    REQUIRE(scarf.starts.size() == 3);
    REQUIRE(scarf.ends.size() == 3);
    CHECK(scarf.starts.front().polyline.first_point() == Point3::new_scale(6., 0., 0.));
    CHECK(scarf.starts.front().polyline.last_point() == Point3::new_scale(10., 0., 0.));
    CHECK(scarf.ends.back().polyline.first_point() == Point3::new_scale(10., 0., 0.));
    CHECK(scarf.ends.back().polyline.last_point() == Point3::new_scale(14., 0., 0.));
    CHECK_THAT(unscale_(scarf.paths.front().length()), WithinAbs(10., 0.001));
}

TEST_CASE("A scarf seam gap stops the final ramp before its non-extruding tail", "[ExtrusionEntity]")
{
    using Catch::Matchers::WithinAbs;
    ExtrusionPaths paths = straight_path({0., 5., 10.});
    ExtrusionLoopSloped scarf(paths, scale_(2.), 10., scale_(5.), 0.);
    REQUIRE(scarf.starts.size() == 1);
    REQUIRE(scarf.ends.size() == 1);
    CHECK(scarf.ends.front().polyline.first_point() == Point3::new_scale(0., 0., 0.));
    CHECK(scarf.ends.front().polyline.last_point() == Point3::new_scale(8., 0., 0.));
    CHECK_THAT(scarf.ends.front().slope_end.e_ratio, WithinAbs(0.2, 1e-9));
}

TEST_CASE("A seam gap covering the last scarf segment removes that end segment", "[ExtrusionEntity]")
{
    ExtrusionPaths paths = straight_path({0., 5., 10.});
    ExtrusionLoopSloped scarf(paths, scale_(10.), 10., scale_(5.), 0.);
    REQUIRE(scarf.starts.size() == 1);
    CHECK(scarf.ends.empty());
}

TEST_CASE("Loop smoothness distinguishes ordinary corners from a narrow pointed wall", "[ExtrusionEntity]")
{
    ExtrusionPath square_path{erExternalPerimeter, 0.05, 0.45f, 0.2f};
    for (const Point3 &point : std::vector<Point3>{
             Point3::new_scale(0., 0., 0.), Point3::new_scale(10., 0., 0.),
             Point3::new_scale(10., 10., 0.), Point3::new_scale(0., 10., 0.),
             Point3::new_scale(0., 0., 0.)})
        square_path.polyline.append(point);
    ExtrusionLoop square(square_path);

    ExtrusionPath pointed_path{erExternalPerimeter, 0.05, 0.45f, 0.2f};
    for (const Point3 &point : std::vector<Point3>{
             Point3::new_scale(0., 0., 0.), Point3::new_scale(10., 0., 0.),
             Point3::new_scale(10., 10., 0.), Point3::new_scale(5.2, 10., 0.),
             Point3::new_scale(5., 5., 0.), Point3::new_scale(4.8, 10., 0.),
             Point3::new_scale(0., 10., 0.), Point3::new_scale(0., 0., 0.)})
        pointed_path.polyline.append(point);
    ExtrusionLoop pointed(pointed_path);

    CHECK(square.is_smooth(0.2, 0.025));
    CHECK_FALSE(pointed.is_smooth(0.2, 0.025));
}
