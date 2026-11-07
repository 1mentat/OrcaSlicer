#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include "libslic3r/libslic3r.h"
#include "libslic3r/Line.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/GCode/ConflictChecker.hpp"

#include <cfloat>
#include <cstdint>
#include <map>
#include <optional>
#include <utility>
#include <vector>

using namespace Slic3r;

namespace {

// The grid cells of a line, as the conflict check rasterizes it.
std::vector<std::pair<int64_t, int64_t>> rasterize(const Line &line)
{
    const int64_t d     = scale_(1);
    auto          cell  = [d](const Point &p) { return std::make_pair(int64_t(p.x() / d), int64_t(p.y() / d)); };
    auto          cur   = cell(line.a);
    const auto    last  = cell(line.b);
    const Point   ray   = line.b - line.a;
    const double  stepX = ray.x() >= 0 ? 1 : -1;
    const double  stepY = ray.y() >= 0 ? 1 : -1;
    double        nextX = (cur.first + stepX) * d;
    double        nextY = (cur.second + stepY) * d;
    if (stepX < 0) nextX += d;
    if (stepY < 0) nextY += d;
    double       tx  = ray.x() != 0 ? (nextX - line.a.x()) / ray.x() : DBL_MAX;
    double       ty  = ray.y() != 0 ? (nextY - line.a.y()) / ray.y() : DBL_MAX;
    const double tdx = ray.x() != 0 ? double(d) / ray.x() * stepX : DBL_MAX;
    const double tdy = ray.y() != 0 ? double(d) / ray.y() * stepY : DBL_MAX;
    std::vector<std::pair<int64_t, int64_t>> res{cur};
    while (last != cur) {
        if (last.first == cur.first) {
            for (int64_t i = cur.second; i != last.second; i += int64_t(stepY)) {
                cur.second += int64_t(stepY);
                res.push_back(cur);
            }
            break;
        }
        if (last.second == cur.second) {
            for (int64_t i = cur.first; i != last.first; i += int64_t(stepX)) {
                cur.first += int64_t(stepX);
                res.push_back(cur);
            }
            break;
        }
        if (tx < ty) {
            cur.first += int64_t(stepX);
            tx += tdx;
        } else {
            cur.second += int64_t(stepY);
            ty += tdy;
        }
        res.push_back(cur);
    }
    return res;
}

// Reference: test every line against all earlier lines sharing a grid cell and return the first conflict.
ConflictComputeOpt find_conflict_in_all_cells(const LineWithIDs &lines)
{
    std::map<std::pair<int64_t, int64_t>, std::vector<int>> cells;
    for (int i = 0; i < int(lines.size()); ++i)
        for (const auto &c : rasterize(lines[i]._line)) {
            for (int j : cells[c])
                if (auto res = ConflictChecker::line_intersect(lines[i], lines[j]); res.has_value())
                    return res;
            cells[c].push_back(i);
        }
    return {};
}

} // namespace

TEST_CASE("Conflicts between objects are the ones found by testing all lines sharing a grid cell", "[ConflictChecker]")
{
    // Three objects: two crowded near the origin, where their lines meet, and one 30 mm away whose lines only meet each other.
    const int objects[3] = {0, 1, 2};
    const int n          = GENERATE(20, 200, 2000);
    const int spread_mm  = GENERATE(2, 20);
    uint32_t  h          = 777u + uint32_t(n) + uint32_t(spread_mm);
    auto      rnd        = [&h](int64_t range) {
        h = h * 1664525u + 1013904223u;
        return int64_t((h >> 8) % uint32_t(range)) - range / 2;
    };
    LineWithIDs lines;
    for (int i = 0; i < n; ++i) {
        const int    obj    = int((h >> 4) % 3);
        const int64_t span   = scale_(spread_mm);
        const Point  origin = obj == 2 ? Point(scale_(30.), scale_(30.)) : Point(0, 0);
        const Point  a      = origin + Point(rnd(span), rnd(span));
        const Point  b      = a + Point(rnd(scale_(3.)), rnd(scale_(3.)));
        lines.emplace_back(Line(a, b), &objects[obj], i % 7 == 0 ? ExtrusionRole::erSupportMaterial : ExtrusionRole::erPerimeter);
    }

    const ConflictComputeOpt expected = find_conflict_in_all_cells(lines);
    const ConflictComputeOpt found    = ConflictChecker::find_inter_of_lines(lines);
    REQUIRE(found.has_value() == expected.has_value());
    if (expected.has_value()) {
        CHECK(found->_obj1 == expected->_obj1);
        CHECK(found->_obj2 == expected->_obj2);
    }
}
