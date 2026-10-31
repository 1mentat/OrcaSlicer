#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/catch_message.hpp>
#include "libslic3r/Feature/Interlocking/VoxelUtils.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/libslic3r.h"

#include <cstdlib>
#include <cstddef>
#include <set>
#include <tuple>
#include <vector>

using namespace Slic3r;

namespace {

using Cell = std::tuple<coord_t, coord_t, coord_t>;

Cell cell_tuple(const GridPoint3 &cell)
{
    return {cell.x(), cell.y(), cell.z()};
}

ExPolygon box(double x0, double y0, double x1, double y1)
{
    return ExPolygon(Polygon::new_scale({{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}}));
}

} // namespace

TEST_CASE("Voxel line walking visits every crossed cell and can stop early", "[VoxelUtils]")
{
    const VoxelUtils voxels(Vec3crd(scale_(1.), scale_(1.), scale_(1.)));
    std::vector<Cell> crossed;
    const bool completed = voxels.walkLine(
        Vec3crd(scale_(0.2), scale_(0.2), scale_(0.2)),
        Vec3crd(scale_(3.2), scale_(0.2), scale_(0.2)),
        [&](GridPoint3 cell) {
            crossed.push_back(cell_tuple(cell));
            return crossed.size() < 2;
        });

    CHECK_FALSE(completed);
    REQUIRE(crossed.size() == 2);
    CHECK(crossed[0] == Cell{0, 0, 0});
    CHECK(crossed[1] == Cell{1, 0, 0});

    CHECK(voxels.toGridPoint(Vec3crd(scale_(1.2), scale_(2.2), scale_(3.2))) ==
          GridPoint3(1, 2, 3));
    CHECK(voxels.toLowerCorner(GridPoint3(-1, 2, 3)) ==
          Vec3crd(-scale_(1.), scale_(2.), scale_(3.)));
}

TEST_CASE("Voxel line walking reports a single cell for a zero-length segment", "[VoxelUtils]")
{
    const VoxelUtils voxels(Vec3crd(scale_(1.), scale_(1.), scale_(1.)));
    std::vector<Cell> cells;
    REQUIRE(voxels.walkLine(
        Vec3crd(scale_(1.2), scale_(2.2), scale_(3.2)),
        Vec3crd(scale_(1.2), scale_(2.2), scale_(3.2)),
        [&](GridPoint3 cell) {
            cells.push_back(cell_tuple(cell));
            return true;
        }));
    REQUIRE(cells.size() == 1);
    CHECK(cells.front() == Cell{1, 2, 3});
}

TEST_CASE("Voxel polygon walking follows every boundary edge", "[VoxelUtils]")
{
    const VoxelUtils voxels(Vec3crd(scale_(1.), scale_(1.), scale_(1.)));
    const ExPolygon square = box(0., 0., 2., 2.);
    std::vector<Cell> visits;
    REQUIRE(voxels.walkPolygons(square, scale_(4.), [&](GridPoint3 cell) {
        visits.push_back(cell_tuple(cell));
        return true;
    }));

    REQUIRE(visits.size() >= 8);
    std::set<Cell> unique(visits.begin(), visits.end());
    CHECK(unique.size() >= 8);
    for (const Cell &cell : unique)
        CHECK(std::get<2>(cell) == 4);
}

TEST_CASE("Voxel area walking samples only interior cells", "[VoxelUtils]")
{
    const VoxelUtils voxels(Vec3crd(scale_(1.), scale_(1.), scale_(1.)));
    const ExPolygon square = box(0., 0., 4., 4.);
    std::set<Cell> cells;
    REQUIRE(voxels.walkAreas(square, scale_(2.), [&](GridPoint3 cell) {
        cells.insert(cell_tuple(cell));
        return true;
    }));

    REQUIRE_FALSE(cells.empty());
    for (const Cell &cell : cells) {
        CHECK(std::get<0>(cell) >= 0);
        CHECK(std::get<0>(cell) < 4);
        CHECK(std::get<1>(cell) >= 0);
        CHECK(std::get<1>(cell) < 4);
        CHECK(std::get<2>(cell) == 2);
    }
}

TEST_CASE("Voxel dilation kernels have bounded unique neighborhoods", "[VoxelUtils]")
{
    const DilationKernel cube(GridPoint3(3, 3, 3), DilationKernel::Type::CUBE);
    const DilationKernel diamond(GridPoint3(3, 3, 3), DilationKernel::Type::DIAMOND);
    const DilationKernel prism(GridPoint3(3, 3, 3), DilationKernel::Type::PRISM);

    REQUIRE(cube.relative_cells_.size() == 27);
    CHECK(diamond.relative_cells_.size() > 0);
    CHECK(diamond.relative_cells_.size() < cube.relative_cells_.size());
    CHECK(prism.relative_cells_.size() > diamond.relative_cells_.size());
    CHECK(prism.relative_cells_.size() < cube.relative_cells_.size());

    for (const DilationKernel &kernel : {cube, diamond, prism}) {
        std::set<Cell> cells;
        for (const GridPoint3 &cell : kernel.relative_cells_) {
            cells.insert(cell_tuple(cell));
            CHECK(std::abs(cell.x()) <= 1);
            CHECK(std::abs(cell.y()) <= 1);
            CHECK(std::abs(cell.z()) <= 1);
        }
        CHECK(cells.size() == kernel.relative_cells_.size());
        CHECK(cells.count(Cell{0, 0, 0}) == 1);
    }
}

TEST_CASE("Dilated voxel polygon walking expands each visited boundary cell", "[VoxelUtils]")
{
    const VoxelUtils voxels(Vec3crd(scale_(1.), scale_(1.), scale_(1.)));
    const ExPolygon square = box(0., 0., 2., 2.);
    const DilationKernel kernel(GridPoint3(3, 3, 1), DilationKernel::Type::CUBE);
    std::set<Cell> cells;
    REQUIRE(voxels.walkDilatedPolygons(square, scale_(4.), kernel, [&](GridPoint3 cell) {
        cells.insert(cell_tuple(cell));
        return true;
    }));

    REQUIRE(cells.size() > 8);
    for (const Cell &cell : cells)
        CHECK(std::get<2>(cell) == 4);
}

TEST_CASE("Voxel traversal crosses each grid axis in both directions", "[VoxelUtils]")
{
    const int axis = GENERATE(0, 1, 2);
    const int direction = GENERATE(-1, 1);
    CAPTURE(axis, direction);
    const VoxelUtils voxels(Vec3crd::Constant(scale_(1.)));
    const Vec3crd start = Vec3crd::Constant(scale_(0.5));
    Vec3crd end = start;
    end[axis] += direction * scale_(3.);
    std::vector<GridPoint3> visited;
    REQUIRE(voxels.walkLine(start, end, [&](GridPoint3 cell) {
        visited.push_back(cell);
        return true;
    }));
    REQUIRE(visited.size() == 4);
    for (size_t i = 0; i < visited.size(); ++i) {
        GridPoint3 expected = GridPoint3::Zero();
        expected[axis] = direction * coord_t(i);
        CHECK(visited[i] == expected);
    }
}
