#include <cstddef>
#include <cstdint>
#include <vector>

#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>

#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/ExPolygonsIndex.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"

using namespace Slic3r;

namespace {

ExPolygons indexed_shapes()
{
    ExPolygons shapes;
    shapes.emplace_back(ExPolygon(
        Points{{0, 0}, {40, 0}, {40, 30}, {0, 30}},
        Points{{5, 5}, {5, 12}, {18, 12}, {18, 5}}));
    shapes.back().holes.emplace_back(Points{{25, 15}, {25, 24}, {35, 24}, {35, 15}});

    shapes.emplace_back(ExPolygon(Points{{100, 0}, {130, 0}, {140, 20}, {120, 35}, {100, 20}}));
    shapes.back().holes.emplace_back(Points{{110, 8}, {118, 8}, {118, 15}, {110, 15}});
    return shapes;
}

std::vector<Point> flatten_points(const ExPolygons &shapes)
{
    std::vector<Point> points;
    for (const ExPolygon &shape : shapes) {
        points.insert(points.end(), shape.contour.points.begin(), shape.contour.points.end());
        for (const Polygon &hole : shape.holes)
            points.insert(points.end(), hole.points.begin(), hole.points.end());
    }
    return points;
}

} // namespace

TEST_CASE("ExPolygon point indexes round-trip across islands and holes", "[ExPolygonsIndex]")
{
    const ExPolygons       shapes = indexed_shapes();
    const ExPolygonsIndices indices(shapes);
    const std::vector<Point> points = flatten_points(shapes);

    REQUIRE(indices.get_count() == points.size());
    REQUIRE(indices.get_count() == count_points(shapes));

    for (uint32_t flat_index = 0; flat_index < indices.get_count(); ++flat_index) {
        CAPTURE(flat_index);
        const ExPolygonsIndex id = indices.cvt(flat_index);
        REQUIRE(id.expolygons_index < shapes.size());
        REQUIRE(id.polygon_index < shapes[id.expolygons_index].num_contours());
        const Polygon &polygon = shapes[id.expolygons_index].contour_or_hole(id.polygon_index);
        REQUIRE(id.point_index < polygon.points.size());
        CHECK(polygon.points[id.point_index] == points[flat_index]);
        CHECK(indices.cvt(id) == flat_index);
        CHECK(id.is_contour() == (id.polygon_index == 0));
        CHECK(id.is_hole() == (id.polygon_index != 0));
        if (id.is_hole())
            CHECK(id.hole_index() == id.polygon_index - 1);
    }
}

TEST_CASE("ExPolygon point indexes identify the end of every contour and hole", "[ExPolygonsIndex]")
{
    const ExPolygons       shapes = indexed_shapes();
    const ExPolygonsIndices indices(shapes);

    uint32_t flat_index = 0;
    for (size_t expolygon_index = 0; expolygon_index < shapes.size(); ++expolygon_index) {
        const ExPolygon &shape = shapes[expolygon_index];
        for (size_t polygon_index = 0; polygon_index < shape.num_contours(); ++polygon_index) {
            const Polygon &polygon = shape.contour_or_hole(polygon_index);
            REQUIRE(polygon.points.size() >= 3);
            for (size_t point_index = 0; point_index < polygon.points.size(); ++point_index, ++flat_index) {
                const ExPolygonsIndex id{static_cast<uint32_t>(expolygon_index),
                                         static_cast<uint32_t>(polygon_index),
                                         static_cast<uint32_t>(point_index)};
                CAPTURE(expolygon_index, polygon_index, point_index);
                CHECK(indices.cvt(id) == flat_index);
                CHECK(indices.is_last_point(id) == (point_index + 1 == polygon.points.size()));
            }
        }
    }
    REQUIRE(flat_index == indices.get_count());
}

TEST_CASE("An empty ExPolygon index has no flattened points", "[ExPolygonsIndex]")
{
    const ExPolygonsIndices indices(ExPolygons{});
    CHECK(indices.get_count() == 0);
}
