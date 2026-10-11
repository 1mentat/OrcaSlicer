#include "../ClipperUtils.hpp"
#include "../Layer.hpp"
#include "../Polyline.hpp"
#include "libslic3r/Surface.hpp"
#include <vector>
#include "libslic3r/AABBTreeIndirect.hpp"
#include <cstddef>
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/libslic3r.h"
#include <cassert>
#include "libslic3r/Polygon.hpp"

#include "RetractWhenCrossingPerimeters.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/SurfaceCollection.hpp"

namespace Slic3r {

bool RetractWhenCrossingPerimeters::travel_inside_internal_regions(const Layer &layer, const Polyline &travel)
{
    if (m_layer != &layer) {
        // Update cache.
        m_layer = &layer;
        m_internal_islands.clear();
        m_aabbtree_internal_islands.clear();
        // Collect expolygons of internal slices.
        for (const LayerRegion *layerm : layer.regions())
            for (const Surface &surface : layerm->get_slices().surfaces)
                if (surface.is_internal())
                    m_internal_islands.emplace_back(&surface.expolygon);
        // Calculate bounding boxes of internal slices.
        std::vector<AABBTreeIndirect::BoundingBoxWrapper> bboxes;
        bboxes.reserve(m_internal_islands.size());
        for (size_t i = 0; i < m_internal_islands.size(); ++ i)
            bboxes.emplace_back(i, get_extents(*m_internal_islands[i]));
        // Build AABB tree over bounding boxes of internal slices.
        m_aabbtree_internal_islands.build_modify_input(bboxes);
    }

    BoundingBox           bbox_travel = get_extents(travel);
    AABBTree::BoundingBox bbox_travel_eigen{ bbox_travel.min, bbox_travel.max };
    int result = -1;
    bbox_travel.offset(SCALED_EPSILON);
    // A travel inside an island lies inside the island's bounding box, and so inside the boxes of all tree nodes above it.
    // A travel reaching out of a box by more than a rounding error has a part outside the island.
    const Point travel_min = Point(bbox_travel_eigen.min()) + Point(coord_t(SCALED_EPSILON), coord_t(SCALED_EPSILON));
    const Point travel_max = Point(bbox_travel_eigen.max()) - Point(coord_t(SCALED_EPSILON), coord_t(SCALED_EPSILON));
    AABBTreeIndirect::traverse(m_aabbtree_internal_islands, 
        [&travel_min, &travel_max](const AABBTree::Node &node) {
            return node.bbox.min().x() <= travel_min.x() && node.bbox.min().y() <= travel_min.y() &&
                   node.bbox.max().x() >= travel_max.x() && node.bbox.max().y() >= travel_max.y();
        },
        [&travel, &bbox_travel, &result, &islands = m_internal_islands](const AABBTree::Node &node) {
            assert(node.is_leaf());
            assert(node.is_valid());
            Polygons clipped = ClipperUtils::clip_clipper_polygons_with_subject_bbox(*islands[node.idx], bbox_travel);
            if (diff_pl(travel, clipped).empty()) {
                // Travel path is completely inside an "internal" island. Don't retract.
                result = int(node.idx);
                // Stop traversal.
                return false;
            }
            // Continue traversal.
            return true;
        });
    return result != -1;
}

} // namespace Slic3r
