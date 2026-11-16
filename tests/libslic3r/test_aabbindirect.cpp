#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <igl/Hit.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <cstddef>
#include <limits>
#include <libslic3r/TriangleMesh.hpp>
#include <libslic3r/AABBTreeIndirect.hpp>
#include <vector>
#include "libslic3r/Point.hpp"

using namespace Slic3r;

TEST_CASE("Building a tree over a box, ray caster and closest query", "[AABBIndirect]")
{
    TriangleMesh tmesh = make_cube(1., 1., 1.);

    auto tree = AABBTreeIndirect::build_aabb_tree_over_indexed_triangle_set(tmesh.its.vertices, tmesh.its.indices);
    REQUIRE(! tree.empty());

    igl::Hit<float> hit;
	bool intersected = AABBTreeIndirect::intersect_ray_first_hit(
		tmesh.its.vertices, tmesh.its.indices,
		tree,
		Vec3d(0.5, 0.5, -5.),
		Vec3d(0., 0., 1.),
		hit);

    REQUIRE(intersected);
    REQUIRE(hit.t == Catch::Approx(5.));

    std::vector<igl::Hit<float>> hits;
	bool intersected2 = AABBTreeIndirect::intersect_ray_all_hits(
		tmesh.its.vertices, tmesh.its.indices,
		tree,
        Vec3d(0.3, 0.5, -5.),
		Vec3d(0., 0., 1.),
		hits);
    REQUIRE(intersected2);
    REQUIRE(hits.size() == 2);
    REQUIRE(hits.front().t == Catch::Approx(5.));
    REQUIRE(hits.back().t == Catch::Approx(6.));

    size_t hit_idx;
    Vec3d  closest_point;
    double squared_distance = AABBTreeIndirect::squared_distance_to_indexed_triangle_set(
		tmesh.its.vertices, tmesh.its.indices,
		tree,
        Vec3d(0.3, 0.5, -5.),
		hit_idx, closest_point);
    REQUIRE(squared_distance == Catch::Approx(5. * 5.));
    REQUIRE(closest_point.x() == Catch::Approx(0.3));
    REQUIRE(closest_point.y() == Catch::Approx(0.5));
    REQUIRE(closest_point.z() == Catch::Approx(0.));

    squared_distance = AABBTreeIndirect::squared_distance_to_indexed_triangle_set(
		tmesh.its.vertices, tmesh.its.indices,
		tree,
        Vec3d(0.3, 0.5, 5.),
		hit_idx, closest_point);
    REQUIRE(squared_distance == Catch::Approx(4. * 4.));
    REQUIRE(closest_point.x() == Catch::Approx(0.3));
    REQUIRE(closest_point.y() == Catch::Approx(0.5));
    REQUIRE(closest_point.z() == Catch::Approx(1.));
}

namespace {

// The closed unit box [-1,1]^3 the ray/slab cases below are built around.
Eigen::AlignedBox3d closed_unit_box()
{
    return Eigen::AlignedBox3d(Eigen::Vector3d(-1., -1., -1.), Eigen::Vector3d(1., 1., 1.));
}

// The reciprocal direction is derived exactly as the AABB tree derives it
// (RayIntersector::invdir), so that a signed zero direction is preserved.
Vec3d reciprocal_direction(const Vec3d &dir)
{
    return Vec3d(dir.cwiseInverse());
}

// The slab test every node of the AABB tree is culled with. It is exercised
// directly, because the bounding boxes stored in the tree are inflated by
// SCALED_EPSILON and therefore cannot pin an exact closed-box boundary.
bool ray_hits_box(const Vec3d &origin, const Vec3d &dir, const Eigen::AlignedBox3d &box, double t0, double t1)
{
    return Slic3r::AABBTreeIndirect::detail::ray_box_intersect_invdir(origin, reciprocal_direction(dir), box, t0, t1);
}

} // namespace

TEST_CASE("A ray parallel to an axis on a closed box face hits the box", "[AABBIndirect]")
{
    const Eigen::AlignedBox3d box = closed_unit_box();
    const double t_inf = std::numeric_limits<double>::infinity();

    SECTION("parallel to X, lying on the closed min-X face")
    {
        // The ray travels along +Y on the closed min-X face of the box, so it
        // crosses the box from Y=-1 to Y=1, i.e. it hits for t in [-1, 1].
        const Vec3d origin(-1., 0., 0.);
        const Vec3d dir(0., 1., 0.5);
        REQUIRE(ray_hits_box(origin, dir, box, 0., t_inf));
        REQUIRE(ray_hits_box(origin, dir, box, 0.5, 1.5));
        REQUIRE_FALSE(ray_hits_box(origin, dir, box, 2., 3.));
    }

    SECTION("parallel to X, lying on the closed max-X face")
    {
        // The ray travels along +Y on the closed max-X face of the box and
        // hits for t in [-1, 1] as well.
        const Vec3d origin(1., 0., 0.);
        const Vec3d dir(0., 1., 0.5);
        REQUIRE(ray_hits_box(origin, dir, box, 0., t_inf));
        REQUIRE(ray_hits_box(origin, dir, box, 0.5, 1.5));
        REQUIRE_FALSE(ray_hits_box(origin, dir, box, 2., 3.));
    }

    SECTION("parallel to Y, lying on the closed min-Y face")
    {
        // The ray travels along +X on the closed min-Y face of the box and
        // hits for t in [-1, 1].
        const Vec3d origin(0., -1., 0.);
        const Vec3d dir(1., 0., 0.5);
        REQUIRE(ray_hits_box(origin, dir, box, 0., t_inf));
        REQUIRE(ray_hits_box(origin, dir, box, 0.5, 1.5));
        REQUIRE_FALSE(ray_hits_box(origin, dir, box, 2., 3.));
    }

    SECTION("parallel to Y, lying on the closed max-Y face")
    {
        const Vec3d origin(0., 1., 0.);
        const Vec3d dir(1., 0., 0.5);
        REQUIRE(ray_hits_box(origin, dir, box, 0., t_inf));
        REQUIRE(ray_hits_box(origin, dir, box, 0.5, 1.5));
        REQUIRE_FALSE(ray_hits_box(origin, dir, box, 2., 3.));
    }

    SECTION("parallel to Z, lying on the closed min-Z face")
    {
        // The ray travels along +X on the closed min-Z face of the box and
        // hits for t in [-1, 1].
        const Vec3d origin(0., 0., -1.);
        const Vec3d dir(1., 0.5, 0.);
        REQUIRE(ray_hits_box(origin, dir, box, 0., t_inf));
        REQUIRE(ray_hits_box(origin, dir, box, 0.5, 1.5));
        REQUIRE_FALSE(ray_hits_box(origin, dir, box, 2., 3.));
    }

    SECTION("parallel to Z, lying on the closed max-Z face")
    {
        const Vec3d origin(0., 0., 1.);
        const Vec3d dir(1., 0.5, 0.);
        REQUIRE(ray_hits_box(origin, dir, box, 0., t_inf));
        REQUIRE(ray_hits_box(origin, dir, box, 0.5, 1.5));
        REQUIRE_FALSE(ray_hits_box(origin, dir, box, 2., 3.));
    }
}

TEST_CASE("A ray parallel to X with a negative zero direction hits the closed box face", "[AABBIndirect]")
{
    const Eigen::AlignedBox3d box = closed_unit_box();
    const double t_inf = std::numeric_limits<double>::infinity();

    SECTION("negative zero direction on the closed min-X face")
    {
        // A negative zero in the X direction is still a direction parallel to
        // the X axis, so the ray lies on the closed min-X face of the box.
        const Vec3d origin(-1., 0., 0.);
        const Vec3d dir(-0.0, 1., 0.5);
        REQUIRE(std::signbit(dir.x()));
        REQUIRE(ray_hits_box(origin, dir, box, 0., t_inf));
        REQUIRE(ray_hits_box(origin, dir, box, 0.5, 1.5));
        REQUIRE_FALSE(ray_hits_box(origin, dir, box, 2., 3.));
    }

    SECTION("negative zero direction on the closed max-X face")
    {
        const Vec3d origin(1., 0., 0.);
        const Vec3d dir(-0.0, 1., 0.5);
        REQUIRE(std::signbit(dir.x()));
        REQUIRE(ray_hits_box(origin, dir, box, 0., t_inf));
        REQUIRE(ray_hits_box(origin, dir, box, 0.5, 1.5));
        REQUIRE_FALSE(ray_hits_box(origin, dir, box, 2., 3.));
    }
}

TEST_CASE("A ray parallel to X hits the box only when it is within the X range", "[AABBIndirect]")
{
    const Eigen::AlignedBox3d box = closed_unit_box();
    const double t_inf = std::numeric_limits<double>::infinity();

    SECTION("the ray is strictly inside the X range of the box")
    {
        const Vec3d origin(-0.5, 0., 0.);
        const Vec3d dir(0., 1., 0.5);
        REQUIRE(ray_hits_box(origin, dir, box, 0., t_inf));
        REQUIRE(ray_hits_box(origin, dir, box, 0.5, 1.5));
        REQUIRE_FALSE(ray_hits_box(origin, dir, box, 2., 3.));
    }

    SECTION("the ray is strictly outside the X range of the box")
    {
        // The ray never enters the X slab of the box, so it misses it.
        const Vec3d origin(-2., 0., 0.);
        const Vec3d dir(0., 1., 0.5);
        REQUIRE_FALSE(ray_hits_box(origin, dir, box, 0., t_inf));
        REQUIRE_FALSE(ray_hits_box(origin, dir, box, 0.5, 1.5));
    }

    SECTION("the ray is strictly outside the X range of the box on the far side")
    {
        const Vec3d origin(2., 0., 0.);
        const Vec3d dir(0., 1., 0.5);
        REQUIRE_FALSE(ray_hits_box(origin, dir, box, 0., t_inf));
        REQUIRE_FALSE(ray_hits_box(origin, dir, box, 0.5, 1.5));
    }
}

TEST_CASE("A ray parallel to X on a closed box face misses the box when it moves away from it", "[AABBIndirect]")
{
    const Eigen::AlignedBox3d box = closed_unit_box();
    const double t_inf = std::numeric_limits<double>::infinity();

    // The ray lies on the closed min-X face of the box, but it travels along
    // +Y away from the box, so it never enters it.
    const Vec3d origin(-1., 5., 0.);
    const Vec3d dir(0., 1., 0.5);
    REQUIRE_FALSE(ray_hits_box(origin, dir, box, 0., t_inf));
}

TEST_CASE("A ray that is not parallel to an axis hits the box within its slab interval", "[AABBIndirect]")
{
    const Eigen::AlignedBox3d box = closed_unit_box();
    const double t_inf = std::numeric_limits<double>::infinity();

    SECTION("the ray crosses the box")
    {
        // Each slab is crossed between t=2 and t=3, so the box is hit exactly
        // for t in [2, 3].
        const Vec3d origin(-5., -5., -5.);
        const Vec3d dir(2., 2., 2.);
        REQUIRE(ray_hits_box(origin, dir, box, 0., t_inf));
        REQUIRE(ray_hits_box(origin, dir, box, 2.5, 2.5));
        REQUIRE_FALSE(ray_hits_box(origin, dir, box, 0.5, 1.5));
        REQUIRE_FALSE(ray_hits_box(origin, dir, box, 3.5, 4.5));
    }

    SECTION("the ray travels away from the box")
    {
        const Vec3d origin(5., 5., 5.);
        const Vec3d dir(1., 1., 1.);
        REQUIRE_FALSE(ray_hits_box(origin, dir, box, 0., t_inf));
    }
}
