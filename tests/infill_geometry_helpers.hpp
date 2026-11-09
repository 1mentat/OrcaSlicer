#pragma once

#include "libslic3r/Point.hpp"
#include "libslic3r/Polyline.hpp"
#include "libslic3r/libslic3r.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <vector>

namespace infill_setting_geometry_contracts {

inline std::vector<double> long_axis_line_positions(const Slic3r::Polylines &paths, bool vertical, bool reverse_normal = false)
{
    std::vector<double> positions;
    for (const Slic3r::Polyline &path : paths)
        for (std::size_t i = 1; i < path.points.size(); ++i) {
            const Slic3r::Point &a = path.points[i - 1];
            const Slic3r::Point &b = path.points[i];
            const double dx = Slic3r::unscale<double>(std::abs(b.x() - a.x()));
            const double dy = Slic3r::unscale<double>(std::abs(b.y() - a.y()));
            if (vertical ? (dx <= 0.02 && dy >= 6.0) : (dy <= 0.02 && dx >= 6.0))
                positions.push_back(Slic3r::unscale<double>((vertical ? a.x() : a.y()) * (reverse_normal ? -1 : 1)));
        }
    std::sort(positions.begin(), positions.end());
    positions.erase(std::unique(positions.begin(), positions.end(), [](double a, double b) {
        return std::abs(a - b) <= 0.04;
    }), positions.end());
    return positions;
}

} // namespace infill_setting_geometry_contracts
