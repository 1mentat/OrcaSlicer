#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "libslic3r/BoundingBox.hpp"
// Test file for Arachne wall generation
// 
// Tests for duplicate/coinciding wall segment detection in Arachne output.
// 
// This test reproduces an issue where Arachne generates duplicate extrusion
// segments at certain min_bead_width settings. The test uses a polygon with
// an outer rectangle (0,0)-(20,20) and an inner cutout (0.5,0.5)-(19.5,19.5).
//
// With precise_outer_wall enabled and min_bead_width at 50% (0.20mm), Arachne
// generates two separate closed contours that share a coinciding edge at y=19.75.
// At 60% (0.24mm), Arachne handles this differently and avoids the duplicate.
//
// Parameters are based on "0.28mm Extra Draft @BBL X1C" profile with:
// - 0.4mm nozzle, 0.28mm layer height
// - outer_wall_line_width: 0.42mm, inner_wall_line_width: 0.45mm
// - wall_loops: 2, precise_outer_wall: enabled

#include <catch2/catch_all.hpp>
#include <catch2/matchers/catch_matchers.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/catch_message.hpp>
#include "libslic3r/Arachne/WallToolPaths.hpp"
#include "libslic3r/Arachne/SkeletalTrapezoidation.hpp"
#include "libslic3r/Arachne/utils/ExtrusionLine.hpp"
#include "libslic3r/Arachne/BeadingStrategy/BeadingStrategyFactory.hpp"
#include "libslic3r/Arachne/BeadingStrategy/BeadingStrategy.hpp"
#include "libslic3r/Feature/FuzzySkin/FuzzySkin.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Point.hpp"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <cstddef>
#include "libslic3r/libslic3r.h"
#include <vector>
#include <utility>
#include <math.h>
#include "libslic3r/Arachne/utils/ExtrusionJunction.hpp"
#include "libslic3r/PerimeterGenerator.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Layer.hpp"

using namespace Slic3r;
using namespace Slic3r::Arachne;

namespace {

// Represents a segment with direction-independent comparison
struct Segment {
    Point from;
    Point to;
    size_t inset_idx;
    
    // Normalize segment so that the "smaller" point comes first
    // This allows direction-independent comparison
    Segment normalized() const {
        if (from < to || (from.x() == to.x() && from.y() < to.y())) {
            return *this;
        }
        return {to, from, inset_idx};
    }
    
    bool operator<(const Segment& other) const {
        auto a = normalized();
        auto b = other.normalized();
        if (a.inset_idx != b.inset_idx) return a.inset_idx < b.inset_idx;
        if (a.from != b.from) return a.from < b.from;
        return a.to < b.to;
    }
    
    bool operator==(const Segment& other) const {
        auto a = normalized();
        auto b = other.normalized();
        return a.inset_idx == b.inset_idx && a.from == b.from && a.to == b.to;
    }
};

// Check if two points are approximately equal within tolerance
bool points_approx_equal(const Point& a, const Point& b, coord_t tolerance) {
    return std::abs(a.x() - b.x()) <= tolerance && std::abs(a.y() - b.y()) <= tolerance;
}

// Check if two segments are approximately equal (direction-independent)
bool segments_approx_equal(const Segment& a, const Segment& b, coord_t tolerance) {
    if (a.inset_idx != b.inset_idx) return false;
    
    // Check both directions
    bool same_dir = points_approx_equal(a.from, b.from, tolerance) && 
                    points_approx_equal(a.to, b.to, tolerance);
    bool reverse_dir = points_approx_equal(a.from, b.to, tolerance) && 
                       points_approx_equal(a.to, b.from, tolerance);
    return same_dir || reverse_dir;
}

// Extract all segments from toolpaths (all inset indices)
std::vector<Segment> extract_all_segments(const std::vector<VariableWidthLines>& toolpaths) {
    std::vector<Segment> segments;
    
    for (const auto& inset : toolpaths) {
        for (const auto& line : inset) {
            if (line.junctions.size() < 2) continue;
            
            for (size_t i = 0; i + 1 < line.junctions.size(); ++i) {
                segments.push_back({
                    line.junctions[i].p,
                    line.junctions[i + 1].p,
                    line.inset_idx
                });
            }
        }
    }
    
    return segments;
}

// Find duplicate segments within tolerance
std::vector<std::pair<Segment, Segment>> find_duplicate_segments(
    const std::vector<Segment>& segments, 
    coord_t tolerance) 
{
    std::vector<std::pair<Segment, Segment>> duplicates;
    
    for (size_t i = 0; i < segments.size(); ++i) {
        for (size_t j = i + 1; j < segments.size(); ++j) {
            if (segments_approx_equal(segments[i], segments[j], tolerance)) {
                duplicates.emplace_back(segments[i], segments[j]);
            }
        }
    }
    
    return duplicates;
}

// Create params matching "0.28mm Extra Draft @BBL X1C" profile
WallToolPathsParams make_bbl_x1c_028_params(int min_bead_width_percent) {
    constexpr double nozzle_diameter = 0.4;
    
    WallToolPathsParams params;
    params.min_bead_width = float(min_bead_width_percent / 100.0 * nozzle_diameter);
    params.min_feature_size = float(0.25 * nozzle_diameter);
    params.wall_transition_filter_deviation = float(0.25 * nozzle_diameter);
    params.wall_transition_length = float(1.0 * nozzle_diameter);
    params.wall_transition_angle = 10.0f;
    params.wall_distribution_count = 1;
    params.min_length_factor = 0.5f;
    params.is_top_or_bottom_layer = false;
    return params;
}

// Run Arachne wall generation test with specified min_bead_width percentage
// Returns the number of duplicate segments found
size_t run_arachne_test(int min_bead_width_percent) {
    constexpr double layer_height = 0.28;
    constexpr double ext_perimeter_width_mm = 0.42;
    constexpr double perimeter_width_mm = 0.45;
    
    // Spacing calculation: width - height * (1 - PI/4)
    constexpr double spacing_factor = 1.0 - 0.25 * M_PI;
    double ext_perimeter_spacing_mm = ext_perimeter_width_mm - layer_height * spacing_factor;
    double perimeter_spacing_mm = perimeter_width_mm - layer_height * spacing_factor;
    
    coord_t ext_perimeter_width = scaled<coord_t>(ext_perimeter_width_mm);
    coord_t ext_perimeter_spacing = scaled<coord_t>(ext_perimeter_spacing_mm);
    coord_t perimeter_spacing = scaled<coord_t>(perimeter_spacing_mm);
    
    coord_t bead_width_0 = ext_perimeter_spacing;
    coord_t bead_width_x = perimeter_spacing;
    size_t  inset_count  = 2;
    
    // precise_outer_wall enabled
    float precise_offset = -float(ext_perimeter_width - ext_perimeter_spacing);
    coord_t wall_0_inset = -coord_t(ext_perimeter_width / 2 - ext_perimeter_spacing / 2);
    
    auto params = make_bbl_x1c_028_params(min_bead_width_percent);
    
    // Test polygon: outer rectangle with inner cutout creating 0.5mm frame
    Polygon outer_raw;
    outer_raw.points.emplace_back(Point::new_scale(0.0, 0.0));
    outer_raw.points.emplace_back(Point::new_scale(20.0, 0.0));
    outer_raw.points.emplace_back(Point::new_scale(20.0, 20.0));
    outer_raw.points.emplace_back(Point::new_scale(0.0, 20.0));
    
    Polygon inner_raw;
    inner_raw.points.emplace_back(Point::new_scale(0.5, 0.5));
    inner_raw.points.emplace_back(Point::new_scale(0.5, 19.5));
    inner_raw.points.emplace_back(Point::new_scale(19.5, 19.5));
    inner_raw.points.emplace_back(Point::new_scale(19.5, 0.5));
    
    ExPolygon input_expolygon;
    input_expolygon.contour = outer_raw;
    input_expolygon.holes.push_back(inner_raw);
    
    ExPolygons offset_result = offset_ex(input_expolygon, precise_offset);
    Polygons outline;
    for (const auto& expoly : offset_result) {
        outline.push_back(expoly.contour);
        for (const auto& hole : expoly.holes) {
            outline.push_back(hole);
        }
    }
    
    WallToolPaths wallToolPaths(outline, bead_width_0, bead_width_x, 
                                 inset_count, wall_0_inset, 
                                 layer_height, params);
    auto toolpaths = wallToolPaths.getToolPaths();
    
    auto all_segments = extract_all_segments(toolpaths);
    auto duplicates = find_duplicate_segments(all_segments, scaled<coord_t>(0.1));
    
    return duplicates.size();
}

} // anonymous namespace

TEST_CASE("Arachne wall generation - 50% min_bead_width", "[Arachne]") {
    size_t duplicates = run_arachne_test(50);
    REQUIRE(duplicates == 0);
}

TEST_CASE("Arachne wall generation - 60% min_bead_width", "[Arachne]") {
    size_t duplicates = run_arachne_test(60);
    REQUIRE(duplicates == 0);
}

// Regression test for #14376 ("Fuzzy skin artifacting" — a surface bulge at a fixed height).
//
// PR #14031 changed WideningBeadingStrategy::compute() to take the thin-wall single-bead
// branch whenever thickness < getTransitionThickness(1). That branch emits a single bead at
// the full wall thickness and ignores the requested bead_count. When the skeletal graph asks
// for 2 beads at a thickness inside the 1<->2 transition band (between the inner wall width and
// getTransitionThickness(1)), the request was collapsed into one over-wide bead — an
// over-extruded line that shows up as a bulge on curved surfaces at a deterministic height.
//
// Profile mirrors the reporter's project ("0.20mm Standard @BBL X1C", 0.4mm nozzle):
//   outer 0.42mm / inner 0.45mm, min_bead_width 85% (0.34mm), 2 walls (max_bead_count 4).
// For these numbers wall_split_middle_threshold = 2*0.34/0.42 - 1 = 0.619, so
// getTransitionThickness(1) = (1 + 0.619) * 0.42 = 0.68mm. A 0.5mm-thick wall therefore sits
// in the transition band: alpha produced 2 beads here, beta collapses it to 1 fat bead.
TEST_CASE("Arachne widening keeps two beads in transition band (#14376)", "[Arachne]") {

    // Widths in mm; the scaled coord_t values and the thresholds below are both derived from
    // these so a width change cannot silently desync the transition-band math.
    const double outer_mm = 0.42, inner_mm = 0.45, min_bead_mm = 0.34; // min_bead = 85% of 0.4mm nozzle

    const coord_t outer_width = scaled<coord_t>(outer_mm);
    const coord_t inner_width = scaled<coord_t>(inner_mm);
    const coord_t min_bead_width = scaled<coord_t>(min_bead_mm);
    const coord_t min_feature_size = scaled<coord_t>(0.10); // 25% of 0.4mm nozzle
    const coord_t transition_length = scaled<coord_t>(0.40);
    const coord_t max_bead_count = 4; // 2 * wall_loops

    // Same derivation as WallToolPaths.cpp.
    const double split_middle_threshold = std::clamp(2.0 * min_bead_mm / outer_mm - 1.0, 0.01, 0.99);
    const double add_middle_threshold = std::clamp(min_bead_mm / inner_mm, 0.01, 0.99);

    auto strategy = BeadingStrategyFactory::makeStrategy(
        outer_width, inner_width, transition_length,
        /*transitioning_angle*/ float(M_PI / 4.0), /*print_thin_walls*/ true,
        min_bead_width, min_feature_size,
        split_middle_threshold, add_middle_threshold,
        max_bead_count, /*outer_wall_offset*/ 0, /*inward_distributed_center_wall_count*/ 1);

    // A wall thickness inside the 1<->2 bead transition band (inner_width < t < transition).
    const coord_t thickness = scaled<coord_t>(0.50);
    REQUIRE(thickness > inner_width);
    REQUIRE(thickness < strategy->getTransitionThickness(1));

    // When the graph requests 2 beads, the strategy must produce 2 beads — not collapse them
    // into a single full-thickness (bulge) bead.
    const BeadingStrategy::Beading beading = strategy->compute(thickness, 2);
    REQUIRE(beading.bead_widths.size() == 2);

    // And neither bead may be over-wide: a single collapsed bead would be ~0.5mm (the full
    // thickness), well above the configured wall widths.
    for (const coord_t w : beading.bead_widths)
        CHECK(w <= inner_width);
}

namespace {
// Exposes the protected static interpolate() for a focused unit test.
struct InterpolateProbe : SkeletalTrapezoidation {
    using SkeletalTrapezoidation::interpolate;
};
} // anonymous namespace

// interpolate() indexes the merged beading with an index derived from `left`. The merged beading
// follows the thicker of left/right, so when the thicker side has fewer insets the index runs past
// its end.
TEST_CASE("Beading interpolation tolerates a thicker side with fewer insets", "[Arachne][Regression]") {
    using Beading = BeadingStrategy::Beading;

    // Thicker side (right) has fewer insets, so the merged beading holds only 2 toolpath locations.
    const coord_t w = scaled<coord_t>(0.42);
    Beading left;
    left.total_thickness = scaled<coord_t>(1.0);
    left.bead_widths = { w, w, w, w };
    left.toolpath_locations = { scaled<coord_t>(0.1), scaled<coord_t>(0.3), scaled<coord_t>(0.5), scaled<coord_t>(0.7) };
    left.left_over = 0;

    Beading right;
    right.total_thickness = scaled<coord_t>(2.0);
    right.bead_widths = { w, w };
    right.toolpath_locations = { scaled<coord_t>(0.1), scaled<coord_t>(0.3) };
    right.left_over = 0;

    // Just past left's location [2] (0.5), so the derived index is 2, past the end of the 2-inset merged beading.
    const coord_t switching_radius = scaled<coord_t>(0.6);

    Beading result;
    REQUIRE_NOTHROW(result = InterpolateProbe::interpolate(left, 0.5, right, switching_radius));

    // With the guard the adjustment is skipped, so the result is the plain interpolation.
    const Beading expected = InterpolateProbe::interpolate(left, 0.5, right);
    REQUIRE(result.toolpath_locations.size() == expected.toolpath_locations.size());
    REQUIRE(result.bead_widths.size() == expected.bead_widths.size());
    for (size_t i = 0; i < expected.toolpath_locations.size(); ++i) {
        CHECK(result.toolpath_locations[i] == expected.toolpath_locations[i]);
        CHECK(result.bead_widths[i] == expected.bead_widths[i]);
    }
}

namespace {

// Closed 20 mm square loop at a uniform width.
Arachne::ExtrusionJunctions square_loop(coord_t width)
{
    const coord_t s = scaled<coord_t>(20.);
    return {{Point(0, 0), width, 0}, {Point(s, 0), width, 0}, {Point(s, s), width, 0}, {Point(0, s), width, 0}, {Point(0, 0), width, 0}};
}

FuzzySkinConfig thick_fuzzy_config(FuzzySkinMode mode, NoiseType noise_type, double thickness_mm)
{
    FuzzySkinConfig cfg{};
    cfg.type              = FuzzySkinType::All;
    cfg.thickness         = scaled<coord_t>(thickness_mm);
    cfg.point_distance    = scaled<coord_t>(0.3);
    cfg.fuzzy_first_layer = true;
    cfg.noise_type        = noise_type;
    cfg.noise_scale       = 1.0;
    cfg.noise_octaves     = 4;
    cfg.noise_persistence = 0.5;
    cfg.mode              = mode;
    cfg.layer_id          = 5;
    return cfg;
}

} // namespace

// Extrusion and Combined mode add noise to each junction's width. A junction narrower than
// height * (1 - PI/4) makes Flow::rounded_rectangle_extrusion_spacing() throw and fails the slice.
// The fuzz thickness is 3x the line width so the clamp is hit on every run regardless of RNG seed.
// Ridged multifractal is covered because its output is not bounded to [-1, 1], so it scales past
// the configured thickness; the floor has to hold for any noise value, not just an in-range one.
TEST_CASE("Fuzzy skin extrusion width is floored at the minimum the flow accepts", "[Arachne][FuzzySkin]") {
    using namespace Slic3r::Feature::FuzzySkin;

    const double layer_height = GENERATE(0.08, 0.2, 0.28);
    const auto   mode         = GENERATE(FuzzySkinMode::Extrusion, FuzzySkinMode::Combined);
    const auto   noise_type   = GENERATE(NoiseType::Classic, NoiseType::Perlin, NoiseType::Billow, NoiseType::RidgedMulti, NoiseType::Voronoi);
    CAPTURE(layer_height, int(mode), int(noise_type));

    const double line_width_mm = 0.42;
    auto         loop          = square_loop(scaled<coord_t>(line_width_mm));
    fuzzy_extrusion_line(loop, /*slice_z*/ 1.0, layer_height, thick_fuzzy_config(mode, noise_type, 3 * line_width_mm));

    REQUIRE(loop.size() > 100);

    const auto   narrowest    = std::min_element(loop.begin(), loop.end(), [](const auto& a, const auto& b) { return a.w < b.w; });
    const double narrowest_mm = unscaled<double>(narrowest->w);
    const double floor_mm     = layer_height * (1. - 0.25 * PI);
    CAPTURE(narrowest_mm, floor_mm);

    CHECK(narrowest_mm < line_width_mm); // the clamp was exercised
    CHECK(narrowest_mm > floor_mm);
    CHECK_NOTHROW(Flow::rounded_rectangle_extrusion_spacing(float(narrowest_mm), float(layer_height)));
}

// Displacement mode only moves points; widths must pass through unchanged.
TEST_CASE("Fuzzy skin displacement mode leaves widths untouched", "[Arachne][FuzzySkin]") {
    using namespace Slic3r::Feature::FuzzySkin;

    const coord_t width = scaled<coord_t>(0.42);
    auto          loop  = square_loop(width);
    fuzzy_extrusion_line(loop, /*slice_z*/ 1.0, /*layer_height*/ 0.2, thick_fuzzy_config(FuzzySkinMode::Displacement, NoiseType::Classic, 1.26));

    REQUIRE(loop.size() > 100);
    CHECK(std::all_of(loop.begin(), loop.end(), [width](const auto& j) { return j.w == width; }));
}

namespace arachne_wall_transition_contracts {

struct ConfiguredWall {
    PrintConfig       print;
    PrintObjectConfig object;
};

ConfiguredWall wall_config(double transition_length_percent,
                           double filter_deviation_percent,
                           double transition_angle,
                           int    distribution_count,
                           double maximum_resolution_mm,
                           double maximum_deviation_mm)
{
    ConfiguredWall config;
    config.print.nozzle_diameter.values = {0.4};
    config.object.layer_height.value = 0.2;
    config.object.min_feature_size.value = 25.;
    config.object.min_bead_width.value = 50.;
    config.object.min_length_factor.value = 0.5;
    config.object.wall_transition_length.value = transition_length_percent;
    config.object.wall_transition_filter_deviation.value = filter_deviation_percent;
    config.object.wall_transition_angle.value = transition_angle;
    config.object.wall_distribution_count.value = distribution_count;
    config.object.wall_maximum_resolution.value = maximum_resolution_mm;
    config.object.wall_maximum_deviation.value = maximum_deviation_mm;
    return config;
}

WallToolPathsParams params_for(const ConfiguredWall &config)
{
    return make_paths_params(1, config.object, config.print);
}

Polygon plus_outline()
{
    constexpr double fixture_scale = 0.3;
    Polygon outline;
    outline.points.emplace_back(Point::new_scale(0., 0.));
    outline.points.emplace_back(Point::new_scale(24. * fixture_scale, 0.));
    outline.points.emplace_back(Point::new_scale(24. * fixture_scale, 8. * fixture_scale));
    outline.points.emplace_back(Point::new_scale(16. * fixture_scale, 8. * fixture_scale));
    outline.points.emplace_back(Point::new_scale(16. * fixture_scale, 16. * fixture_scale));
    outline.points.emplace_back(Point::new_scale(24. * fixture_scale, 16. * fixture_scale));
    outline.points.emplace_back(Point::new_scale(24. * fixture_scale, 24. * fixture_scale));
    outline.points.emplace_back(Point::new_scale(0., 24. * fixture_scale));
    outline.points.emplace_back(Point::new_scale(0., 16. * fixture_scale));
    outline.points.emplace_back(Point::new_scale(8. * fixture_scale, 16. * fixture_scale));
    outline.points.emplace_back(Point::new_scale(8. * fixture_scale, 8. * fixture_scale));
    outline.points.emplace_back(Point::new_scale(0., 8. * fixture_scale));
    return outline;
}

Polygon round_outline(double radius_mm = 12.)
{
    Polygon outline;
    constexpr int points = 256;
    for (int i = 0; i < points; ++i) {
        const double angle = 2. * M_PI * double(i) / double(points);
        outline.points.emplace_back(Point::new_scale(radius_mm + radius_mm * std::cos(angle),
                                                       radius_mm + radius_mm * std::sin(angle)));
    }
    return outline;
}

Polygon shallow_bump_outline(bool bumped)
{
    Polygon outline;
    outline.points.emplace_back(Point::new_scale(0., 0.));
    outline.points.emplace_back(Point::new_scale(20., 0.));
    outline.points.emplace_back(Point::new_scale(20., 10.));
    if (bumped) {
        outline.points.emplace_back(Point::new_scale(10.1, 10.));
        outline.points.emplace_back(Point::new_scale(10., 10.02));
        outline.points.emplace_back(Point::new_scale(9.9, 10.));
    }
    outline.points.emplace_back(Point::new_scale(0., 10.));
    return outline;
}

Polygon transition_wedge_outline()
{
    Polygon wedge;
    wedge.points.emplace_back(Point::new_scale(0., 0.));
    wedge.points.emplace_back(Point::new_scale(20., -5.359));
    wedge.points.emplace_back(Point::new_scale(20., 5.359));
    return wedge;
}

struct WallSummary {
    size_t lines = 0;
    size_t junctions = 0;
    double length_mm = 0.;
    double width_area_mm2 = 0.;
    double width_squared_length_mm3 = 0.;
    double minimum_width_mm = 1e9;
    double maximum_width_mm = 0.;
    double maximum_y_mm = -std::numeric_limits<double>::infinity();
    double width_span_mm = 0.;
    bool finite = true;
    bool inside_fixture = true;
};

WallSummary summarize(const std::vector<VariableWidthLines> &toolpaths,
                      const BoundingBox &fixture_bounds)
{
    const double fixture_min_x = unscaled<double>(fixture_bounds.min.x()) - 0.05;
    const double fixture_max_x = unscaled<double>(fixture_bounds.max.x()) + 0.05;
    const double fixture_min_y = unscaled<double>(fixture_bounds.min.y()) - 0.05;
    const double fixture_max_y = unscaled<double>(fixture_bounds.max.y()) + 0.05;
    WallSummary summary;
    for (const VariableWidthLines &inset : toolpaths) {
        for (const ExtrusionLine &line : inset) {
            if (line.empty())
                continue;
            ++summary.lines;
            summary.junctions += line.junctions.size();
            for (const ExtrusionJunction &junction : line.junctions) {
                const double x = unscaled<double>(junction.p.x());
                const double y = unscaled<double>(junction.p.y());
                const double w = unscaled<double>(junction.w);
                summary.minimum_width_mm = std::min(summary.minimum_width_mm, w);
                summary.maximum_width_mm = std::max(summary.maximum_width_mm, w);
                summary.maximum_y_mm = std::max(summary.maximum_y_mm, y);
                summary.finite &= std::isfinite(x) && std::isfinite(y) && std::isfinite(w);
                summary.inside_fixture &= x >= fixture_min_x && x <= fixture_max_x &&
                                           y >= fixture_min_y && y <= fixture_max_y;
            }
            for (size_t i = 1; i < line.junctions.size(); ++i) {
                const double segment_mm = unscaled<double>((line.junctions[i].p - line.junctions[i - 1].p).cast<double>().norm());
                const double mean_width_mm = 0.5 * (unscaled<double>(line.junctions[i].w) +
                                                     unscaled<double>(line.junctions[i - 1].w));
                summary.length_mm += segment_mm;
                summary.width_area_mm2 += segment_mm * mean_width_mm;
                const double w0 = unscaled<double>(line.junctions[i - 1].w);
                const double w1 = unscaled<double>(line.junctions[i].w);
                // Integral of the squared, linearly interpolated width along the segment.
                // This distinguishes width redistribution even when total material is conserved.
                summary.width_squared_length_mm3 += segment_mm * (w0 * w0 + w0 * w1 + w1 * w1) / 3.;
            }
        }
    }
    summary.width_span_mm = summary.maximum_width_mm - summary.minimum_width_mm;
    return summary;
}

WallSummary generate(const Polygon &outline, const WallToolPathsParams &params)
{
    const double spacing_factor = 1. - 0.25 * M_PI;
    const coord_t bead_width_0 = scaled<coord_t>(0.42 - 0.2 * spacing_factor);
    const coord_t bead_width_x = scaled<coord_t>(0.45 - 0.2 * spacing_factor);
    const Polygons outlines{outline};
    WallToolPaths walls(outlines, bead_width_0, bead_width_x, 4, 0, 0.2, params);
    return summarize(walls.getToolPaths(), outline.bounding_box());
}

} // namespace arachne_wall_transition_contracts

TEST_CASE("Arachne wall settings map to the active wall-path controls", "[ArachneWallTransitionMapping]")
{
    using namespace arachne_wall_transition_contracts;
    const ConfiguredWall config = wall_config(250., 75., 37., 3, 0.06, 0.012);
    const WallToolPathsParams params = params_for(config);

    CHECK_THAT(params.wall_transition_length, Catch::Matchers::WithinAbs(1.0, 1e-6));
    CHECK_THAT(params.wall_transition_filter_deviation, Catch::Matchers::WithinAbs(0.3, 1e-6));
    CHECK_THAT(params.wall_transition_angle, Catch::Matchers::WithinAbs(37., 1e-6));
    CHECK(params.wall_distribution_count == 3);
    CHECK(params.wall_maximum_resolution == scaled<coord_t>(0.06));
    CHECK(params.wall_maximum_deviation == scaled<coord_t>(0.012));
}

TEST_CASE("Arachne wall transitions preserve the scaled plus outline while changing wall geometry", "[ArachneWallTransition]")
{
    using namespace arachne_wall_transition_contracts;
    const Polygon outline = plus_outline();
    const WallSummary baseline = generate(outline, params_for(wall_config(100., 25., 10., 1, 0.5, 0.025)));
    const WallSummary distributed = generate(outline, params_for(wall_config(100., 25., 10., 3, 0.5, 0.025)));
    const WallSummary long_transition = generate(outline, params_for(wall_config(300., 25., 10., 3, 0.5, 0.025)));

    for (const WallSummary &row : {baseline, distributed, long_transition}) {
        REQUIRE(row.lines > 0);
        CHECK(row.finite);
        CHECK(row.inside_fixture);
        CHECK(row.minimum_width_mm > 0.05);
        CHECK(row.width_span_mm > 0.05);
        CHECK(row.length_mm > 0.5);
    }

    const bool distribution_changed = std::abs(baseline.width_squared_length_mm3 - distributed.width_squared_length_mm3) > 1e-4 ||
                                      std::abs(baseline.maximum_width_mm - distributed.maximum_width_mm) > 1e-4 ||
                                      std::abs(baseline.width_area_mm2 - distributed.width_area_mm2) > 1e-3;
    CHECK(distribution_changed);
    const bool transition_changed = std::abs(distributed.width_squared_length_mm3 - long_transition.width_squared_length_mm3) > 1e-4 ||
                                    std::abs(distributed.maximum_width_mm - long_transition.maximum_width_mm) > 1e-4 ||
                                    std::abs(distributed.width_area_mm2 - long_transition.width_area_mm2) > 1e-3;
    CHECK(transition_changed);


}

TEST_CASE("Arachne wall resolution preserves a rounded wall while simplifying its sampling", "[ArachneWallResolution]")
{
    using namespace arachne_wall_transition_contracts;
    const Polygon outline = round_outline();
    const WallSummary fine = generate(outline, params_for(wall_config(100., 25., 10., 1, 0.005, 0.005)));
    const WallSummary coarse_resolution = generate(outline, params_for(wall_config(100., 25., 10., 1, 0.5, 0.005)));
    const WallSummary coarse_deviation = generate(outline, params_for(wall_config(100., 25., 10., 1, 0.005, 0.05)));
    const WallSummary coarse = generate(outline, params_for(wall_config(100., 25., 10., 1, 0.5, 0.05)));

    for (const WallSummary &row : {fine, coarse_resolution, coarse_deviation, coarse}) {
        REQUIRE(row.lines == 4);
        CHECK(row.finite);
        CHECK(row.inside_fixture);
        CHECK(row.minimum_width_mm > 0.05);
        CHECK(row.length_mm > 1.);
    }
    CHECK(fine.junctions > coarse_resolution.junctions);
    CHECK(coarse_deviation.junctions > coarse.junctions);
    CHECK(params_for(wall_config(100., 25., 10., 1, 0.5, 0.005)).wall_maximum_deviation == scaled<coord_t>(0.005));
    CHECK(params_for(wall_config(100., 25., 10., 1, 0.5, 0.05)).wall_maximum_deviation == scaled<coord_t>(0.05));

    // At radius 4 mm, a six-edge shortcut has about 0.0108 mm sagitta, between
    // the registered 0.005 and 0.05 mm deviation endpoints.
    const Polygon deviation_outline = round_outline(4.);
    const WallSummary strict_deviation = generate(deviation_outline, params_for(wall_config(100., 25., 10., 1, 0.5, 0.005)));
    const WallSummary permissive_deviation = generate(deviation_outline, params_for(wall_config(100., 25., 10., 1, 0.5, 0.05)));
    for (const WallSummary &row : {strict_deviation, permissive_deviation}) {
        REQUIRE(row.lines == 4);
        CHECK(row.finite);
        CHECK(row.inside_fixture);
        CHECK(row.minimum_width_mm > 0.05);
        CHECK(row.length_mm > 1.);
    }
    CHECK(strict_deviation.junctions > permissive_deviation.junctions);

    // Four convex circular tracks: their circumference changes by at most
    // 2*pi times a radial change. Include both rows' deviation budgets.
    const double length_budget_mm = 4. * 2. * M_PI * (0.005 + 0.05);
    CHECK_THAT(fine.length_mm, Catch::Matchers::WithinAbs(coarse.length_mm, length_budget_mm));
    REQUIRE(fine.width_area_mm2 > 0.);
    REQUIRE(coarse.width_area_mm2 > 0.);
}

TEST_CASE("Arachne wall deviation preserves a shallow boundary feature", "[Arachne][ArachneWallResolution]")
{
    using namespace arachne_wall_transition_contracts;
    const Polygon bump_outline = shallow_bump_outline(true);
    const Polygon flat_outline = shallow_bump_outline(false);
    const WallToolPathsParams strict_params = params_for(wall_config(100., 25., 10., 1, 0.5, 0.005));
    const WallToolPathsParams permissive_params = params_for(wall_config(100., 25., 10., 1, 0.5, 0.05));
    const WallSummary strict_bump = generate(bump_outline, strict_params);
    const WallSummary permissive_bump = generate(bump_outline, permissive_params);
    const WallSummary flat_control = generate(flat_outline, strict_params);

    CAPTURE(strict_bump.maximum_y_mm, permissive_bump.maximum_y_mm, flat_control.maximum_y_mm,
            strict_bump.minimum_width_mm, permissive_bump.minimum_width_mm, flat_control.minimum_width_mm);
    for (const WallSummary &row : {strict_bump, permissive_bump, flat_control}) {
        REQUIRE(row.lines == 4);
        CHECK(row.finite);
        CHECK(row.inside_fixture);
        CHECK(row.minimum_width_mm > 0.05);
        CHECK(row.length_mm > 1.);
    }
    CHECK(strict_bump.maximum_y_mm > flat_control.maximum_y_mm + 0.005);
    CHECK_THAT(permissive_bump.maximum_y_mm,
               Catch::Matchers::WithinAbs(flat_control.maximum_y_mm, 0.005));
}

TEST_CASE("Arachne one millimeter transition keeps low-angle wedge widths positive",
          "[Arachne][ArachneWallTransitionLength]")
{
    using namespace arachne_wall_transition_contracts;
    const WallToolPathsParams params = params_for(wall_config(250., 25., 1., 1, 0.005, 0.005));
    const WallSummary summary = generate(transition_wedge_outline(), params);

    CHECK_THAT(params.wall_transition_length, Catch::Matchers::WithinAbs(1., 1e-6));
    REQUIRE(summary.lines > 0);
    CHECK(summary.finite);
    CHECK(summary.inside_fixture);
    CHECK(summary.minimum_width_mm > 0.);
}

// Known baseline defect: the 1 mm transition at the high-angle endpoint currently
// produces a negative generated width. Keep the positive-width contract executing.
// Catch2's !shouldfail makes an unexpected pass fail too, requiring this marker to
// be removed with the fix. This case is diagnostic evidence, not validated behavior.
TEST_CASE("Arachne one millimeter high-angle transition keeps wedge widths positive",
          "[Arachne][ArachneWallTransitionKnownDefect][KnownBaselineDefect][!shouldfail]")
{
    using namespace arachne_wall_transition_contracts;
    const WallToolPathsParams params = params_for(wall_config(250., 25., 59., 1, 0.005, 0.005));
    const WallSummary summary = generate(transition_wedge_outline(), params);

    CHECK_THAT(params.wall_transition_length, Catch::Matchers::WithinAbs(1., 1e-6));
    REQUIRE(summary.lines > 0);
    CHECK(summary.finite);
    CHECK(summary.inside_fixture);
    CHECK(summary.minimum_width_mm > 0.);
}

namespace arachne_skeletal_public_contracts {

Polygon polygon(std::initializer_list<std::pair<double, double>> coordinates)
{
    Polygon shape;
    for (const auto &[x, y] : coordinates)
        shape.points.emplace_back(Point::new_scale(x, y));
    return shape;
}

struct Summary {
    size_t paths = 0;
    size_t open_odd_paths = 0;
    size_t positive_width_junctions = 0;
    size_t varying_width_paths = 0;
    size_t witness_junctions = 0;
    double length_mm = 0.;
    bool finite = true;
    bool inside_outline_or_margin = true;
};

bool inside_outline_or_margin(const Polygon &shape, const Point &point, coord_t margin)
{
    if (shape.contains(point))
        return true;
    const Point projection = shape.point_projection(point);
    return (projection - point).cast<double>().norm() <= double(margin);
}

Summary generate(const Polygon &shape, bool filter_outermost_central_edges,
                 const BoundingBox *witness_bounds = nullptr)
{
    const coord_t outer_width = scaled<coord_t>(0.42);
    const coord_t inner_width = scaled<coord_t>(0.45);
    const coord_t min_bead_width = scaled<coord_t>(0.20);
    const coord_t min_feature_size = scaled<coord_t>(0.10);
    const coord_t transition_length = scaled<coord_t>(0.40);
    auto strategy = BeadingStrategyFactory::makeStrategy(
        outer_width, inner_width, transition_length, float(M_PI / 4.), true,
        min_bead_width, min_feature_size, 0.5, 0.5, 4, 0, 1);

    SkeletalTrapezoidation trapezoidation(Polygons{shape}, *strategy,
        strategy->getTransitioningAngle(), scaled<coord_t>(0.25),
        scaled<coord_t>(0.40), scaled<coord_t>(0.10), transition_length);
    std::vector<VariableWidthLines> paths;
    trapezoidation.generateToolpaths(paths, filter_outermost_central_edges);

    const coord_t margin = scaled<coord_t>(0.02);
    Summary result;
    for (const VariableWidthLines &inset : paths) {
        for (const ExtrusionLine &line : inset) {
            if (line.junctions.empty())
                continue;
            ++result.paths;
            if (line.is_odd && line.front().p != line.back().p)
                ++result.open_odd_paths;
            double path_minimum_width_mm = std::numeric_limits<double>::infinity();
            double path_maximum_width_mm = 0.;
            for (size_t index = 0; index < line.junctions.size(); ++index) {
                const ExtrusionJunction &junction = line.junctions[index];
                const double x = unscaled<double>(junction.p.x());
                const double y = unscaled<double>(junction.p.y());
                const double width = unscaled<double>(junction.w);
                result.finite &= std::isfinite(x) && std::isfinite(y) && std::isfinite(width);
                result.inside_outline_or_margin &= inside_outline_or_margin(shape, junction.p, margin);
                if (witness_bounds != nullptr && witness_bounds->contains(junction.p))
                    ++result.witness_junctions;
                if (junction.w > 0) {
                    ++result.positive_width_junctions;
                    path_minimum_width_mm = std::min(path_minimum_width_mm, width);
                    path_maximum_width_mm = std::max(path_maximum_width_mm, width);
                }
                if (index > 0) {
                    const double segment_mm = unscaled<double>(
                        (junction.p - line.junctions[index - 1].p).cast<double>().norm());
                    result.length_mm += segment_mm;
                }
            }
            if (path_maximum_width_mm > path_minimum_width_mm + 0.01)
                ++result.varying_width_paths;
        }
    }
    return result;
}

} // namespace arachne_skeletal_public_contracts

TEST_CASE("Arachne outer-central filtering loops sharp tapered walls", "[Arachne][SkeletalPublicAPI]")
{
    using namespace arachne_skeletal_public_contracts;
    // A long wedge gives the public filter a real acute outer corner and a wide region
    // where the beading strategy can emit multiple walls.
    const Polygon wedge = polygon({{0., 0.}, {30., 0.}, {15., 40.}});
    const Summary unfiltered = generate(wedge, false);
    const Summary filtered = generate(wedge, true);

    REQUIRE(unfiltered.paths > 0);
    REQUIRE(filtered.paths > 0);
    CHECK(unfiltered.finite);
    CHECK(filtered.finite);
    CHECK(unfiltered.inside_outline_or_margin);
    CHECK(filtered.inside_outline_or_margin);
    CHECK(unfiltered.open_odd_paths > filtered.open_odd_paths);
}

TEST_CASE("Arachne skeletal paths follow a concave variable-width outline", "[Arachne][SkeletalPublicAPI]")
{
    using namespace arachne_skeletal_public_contracts;
    // Two broad lobes joined by a narrower central neck force beading to propagate
    // through a concave branch while keeping the expected extent easy to check.
    const Polygon necked = polygon({
        {0., 0.}, {24., 0.}, {24., 8.}, {12.35, 8.}, {12.35, 16.}, {24., 16.},
        {24., 24.}, {0., 24.}, {0., 16.}, {11.65, 16.}, {11.65, 8.}, {0., 8.}
    });
    const BoundingBox neck_bounds(Point::new_scale(11.60, 8.0), Point::new_scale(12.40, 16.0));
    const Summary first = generate(necked, false, &neck_bounds);
    const Summary repeated = generate(necked, false);

    REQUIRE(first.paths > 0);
    REQUIRE(first.positive_width_junctions > 0);
    REQUIRE(first.varying_width_paths > 0);
    REQUIRE(first.witness_junctions > 0);
    CHECK(first.finite);
    CHECK(first.inside_outline_or_margin);
    CHECK_THAT(first.length_mm, Catch::Matchers::WithinAbs(repeated.length_mm, 0.001));
    CHECK(first.paths == repeated.paths);
    CHECK(first.positive_width_junctions == repeated.positive_width_junctions);
    CHECK(first.varying_width_paths == repeated.varying_width_paths);
}

TEST_CASE("Arachne factory wrappers preserve nonlinear width thresholds", "[Arachne][SkeletalPublicAPI]")
{
    const coord_t min_output_width = scaled<coord_t>(0.28);
    const coord_t min_feature_width = scaled<coord_t>(0.10);

    for (const coord_t outer_wall_offset : {coord_t(0), scaled<coord_t>(0.02)}) {
        CAPTURE(unscaled<double>(outer_wall_offset));
        const auto widened = BeadingStrategyFactory::makeStrategy(
            scaled<coord_t>(0.42), scaled<coord_t>(0.45), scaled<coord_t>(0.40),
            float(M_PI / 4.0), true, min_output_width, min_feature_width,
            0.5, 0.5, 8, outer_wall_offset, 2);
        const std::vector<coord_t> thresholds = widened->getNonlinearThicknesses(1);
        CHECK(std::find(thresholds.begin(), thresholds.end(), min_output_width) != thresholds.end());
        CHECK(widened->getNonlinearThicknesses(9).empty());

        const auto without_widening = BeadingStrategyFactory::makeStrategy(
            scaled<coord_t>(0.42), scaled<coord_t>(0.45), scaled<coord_t>(0.40),
            float(M_PI / 4.0), false, min_output_width, min_feature_width,
            0.5, 0.5, 8, outer_wall_offset, 2);
        CHECK(without_widening->getNonlinearThicknesses(1).empty());
    }
}

TEST_CASE("Arachne taper preserves the output-width floor at its analytic thickness", "[Arachne][SkeletalPublicAPI]")
{
    constexpr double length_mm = 30.0;
    constexpr double half_base_width_mm = 3.0;
    constexpr double min_output_width_mm = 0.28;
    const double half_slope = half_base_width_mm / length_mm;
    const double expected_x = (min_output_width_mm / 2.0) *
        std::sqrt(1.0 + half_slope * half_slope) / half_slope;
    const Polygon taper = arachne_skeletal_public_contracts::polygon({
        {0.0, 0.0}, {length_mm, -half_base_width_mm}, {length_mm, half_base_width_mm}
    });

    auto strategy = BeadingStrategyFactory::makeStrategy(
        scaled<coord_t>(0.42), scaled<coord_t>(0.45), scaled<coord_t>(0.40),
        float(M_PI / 4.0), true, scaled<coord_t>(min_output_width_mm), scaled<coord_t>(0.10),
        0.5, 0.5, 8, 0, 2);
    const coord_t transition_length = scaled<coord_t>(0.40);
    SkeletalTrapezoidation trapezoidation(
        Polygons{taper}, *strategy, strategy->getTransitioningAngle(),
        scaled<coord_t>(0.25), scaled<coord_t>(0.40), scaled<coord_t>(0.10), transition_length);
    std::vector<VariableWidthLines> toolpaths;
    trapezoidation.generateToolpaths(toolpaths);

    // Sample the public piecewise-linear width at the analytic local thickness.
    // The contract is the width at that location, not a particular junction encoding.
    double nearest_axis_error = std::numeric_limits<double>::infinity();
    double nearest_width_mm = 0.0;
    for (const VariableWidthLines &inset : toolpaths) {
        for (const ExtrusionLine &line : inset) {
            for (size_t i = 1; i < line.junctions.size(); ++i) {
                const ExtrusionJunction &a = line.junctions[i - 1];
                const ExtrusionJunction &b = line.junctions[i];
                const double x_a = unscaled<double>(a.p.x());
                const double x_b = unscaled<double>(b.p.x());
                if (x_a == x_b)
                    continue;
                const double t = (expected_x - x_a) / (x_b - x_a);
                if (t < 0.0 || t > 1.0)
                    continue;
                const double y = unscaled<double>(a.p.y()) +
                    t * (unscaled<double>(b.p.y()) - unscaled<double>(a.p.y()));
                const double axis_error = std::abs(y);
                if (axis_error < nearest_axis_error) {
                    nearest_axis_error = axis_error;
                    nearest_width_mm = unscaled<double>(a.w) +
                        t * (unscaled<double>(b.w) - unscaled<double>(a.w));
                }
            }
        }
    }
    CAPTURE(expected_x, nearest_axis_error, nearest_width_mm);
    REQUIRE(std::isfinite(nearest_axis_error));
    CHECK(nearest_axis_error <= 0.02);
    CHECK_THAT(nearest_width_mm, Catch::Matchers::WithinAbs(min_output_width_mm, 0.01));
}


namespace arachne_wide_neck_contracts {

struct RegionMeasure {
    double length_mm = 0.;
    double width_area_mm2 = 0.;
};

struct Summary {
    size_t paths = 0;
    size_t zero_width_junctions = 0;
    size_t negative_width_junctions = 0;
    size_t nonfinite_width_junctions = 0;
    size_t positive_width_segments = 0;
    bool finite = true;
    bool nonnegative_widths = true;
    bool inside_outline_or_margin = true;
    double width_squared_length_mm3 = 0.;
    RegionMeasure left_lobe;
    RegionMeasure neck;
    RegionMeasure right_lobe;
};

std::vector<VariableWidthLines> generate(const Polygon &outline, double transition_length_mm)
{
    const coord_t transition_length = scaled<coord_t>(transition_length_mm);
    auto strategy = BeadingStrategyFactory::makeStrategy(
        scaled<coord_t>(0.42), scaled<coord_t>(0.45), transition_length,
        float(M_PI / 4.0), true, scaled<coord_t>(0.28), scaled<coord_t>(0.10),
        0.5, 0.5, 8, 0, 2);
    SkeletalTrapezoidation trapezoidation(
        Polygons{outline}, *strategy, strategy->getTransitioningAngle(),
        scaled<coord_t>(0.25), scaled<coord_t>(0.40), scaled<coord_t>(0.10),
        transition_length);
    std::vector<VariableWidthLines> paths;
    trapezoidation.generateToolpaths(paths);
    return paths;
}

Summary summarize(const Polygon &outline, const std::vector<VariableWidthLines> &paths)
{
    Summary result;
    const coord_t margin = scaled<coord_t>(0.02);
    const auto add = [](RegionMeasure &region, double length_mm, double from_width_mm, double to_width_mm) {
        region.length_mm += length_mm;
        region.width_area_mm2 += length_mm * 0.5 * (from_width_mm + to_width_mm);
    };

    for (const VariableWidthLines &inset : paths) {
        for (const ExtrusionLine &line : inset) {
            if (line.junctions.size() < 2)
                continue;
            ++result.paths;
            for (const ExtrusionJunction &junction : line.junctions) {
                const double x = unscaled<double>(junction.p.x());
                const double y = unscaled<double>(junction.p.y());
                const double width = unscaled<double>(junction.w);
                const bool finite_width = std::isfinite(width);
                result.finite &= std::isfinite(x) && std::isfinite(y) && finite_width;
                result.nonnegative_widths &= finite_width && width >= 0.;
                if (!finite_width)
                    ++result.nonfinite_width_junctions;
                else if (width == 0.)
                    ++result.zero_width_junctions;
                else if (width < 0.)
                    ++result.negative_width_junctions;
                if (!outline.contains(junction.p)) {
                    const Point projection = outline.point_projection(junction.p);
                    const double distance = unscaled<double>((projection - junction.p).cast<double>().norm());
                    result.inside_outline_or_margin &= distance <= unscaled<double>(margin);
                }
            }

            for (size_t index = 1; index < line.junctions.size(); ++index) {
                const ExtrusionJunction &from = line.junctions[index - 1];
                const ExtrusionJunction &to = line.junctions[index];
                const double from_x = unscaled<double>(from.p.x());
                const double from_y = unscaled<double>(from.p.y());
                const double to_x = unscaled<double>(to.p.x());
                const double to_y = unscaled<double>(to.p.y());
                const double length_mm = std::hypot(to_x - from_x, to_y - from_y);
                const double midpoint_x = 0.5 * (from_x + to_x);
                const double midpoint_y = 0.5 * (from_y + to_y);
                const double from_width_mm = unscaled<double>(from.w);
                const double to_width_mm = unscaled<double>(to.w);
                // Skeletal output can retain zero-width contour markers. They are not
                // extrusion, so exclude segments whose two endpoints both have zero width.
                if (from_width_mm == 0. && to_width_mm == 0.)
                    continue;
                ++result.positive_width_segments;
                const double width_squared_mean =
                    (from_width_mm * from_width_mm + from_width_mm * to_width_mm + to_width_mm * to_width_mm) / 3.;
                result.width_squared_length_mm3 += length_mm * width_squared_mean;

                if (midpoint_x >= -11.0 && midpoint_x <= -7.0 && std::abs(midpoint_y) <= 3.25)
                    add(result.left_lobe, length_mm, from_width_mm, to_width_mm);
                if (midpoint_x >= -3.0 && midpoint_x <= 3.0 && std::abs(midpoint_y) <= 0.40)
                    add(result.neck, length_mm, from_width_mm, to_width_mm);
                if (midpoint_x >= 7.0 && midpoint_x <= 11.0 && std::abs(midpoint_y) <= 3.25)
                    add(result.right_lobe, length_mm, from_width_mm, to_width_mm);
            }
        }
    }
    return result;
}

} // namespace arachne_wide_neck_contracts

TEST_CASE("Arachne variable-width walls remain balanced through broad lobes and a narrow neck", "[Arachne][SkeletalPublicAPI]")
{
    using namespace arachne_wide_neck_contracts;
    // The two 8 mm tall lobes taper through 2 mm shoulders into a 1 mm wide,
    // 8 mm long neck. This large bead-count drop and the concave shoulders put
    // transition propagation on both sides of the neck without depending on a
    // particular generated junction or path ordering.
    const Polygon outline = arachne_skeletal_public_contracts::polygon({
        {-12., -4.}, {-6., -4.}, {-4., -0.5}, {4., -0.5}, {6., -4.}, {12., -4.},
        {12., 4.}, {6., 4.}, {4., 0.5}, {-4., 0.5}, {-6., 4.}, {-12., 4.}
    });
    const auto short_paths = generate(outline, 0.40);
    const auto long_paths = generate(outline, 2.40);
    const Summary short_transition = summarize(outline, short_paths);
    const Summary long_transition = summarize(outline, long_paths);

    CAPTURE(short_transition.left_lobe.length_mm, short_transition.neck.length_mm,
            short_transition.right_lobe.length_mm, long_transition.left_lobe.length_mm,
            long_transition.neck.length_mm, long_transition.right_lobe.length_mm,
            short_transition.zero_width_junctions, short_transition.negative_width_junctions,
            short_transition.nonfinite_width_junctions, long_transition.zero_width_junctions,
            long_transition.negative_width_junctions, long_transition.nonfinite_width_junctions,
            short_transition.width_squared_length_mm3, long_transition.width_squared_length_mm3);
    REQUIRE(long_transition.paths > 0);
    REQUIRE(short_transition.positive_width_segments > 0);
    REQUIRE(long_transition.positive_width_segments > 0);
    REQUIRE(long_transition.left_lobe.length_mm > 0.5);
    REQUIRE(short_transition.neck.length_mm > 4.0);
    REQUIRE(long_transition.neck.length_mm > 4.0);
    REQUIRE(long_transition.right_lobe.length_mm > 0.5);
    CHECK(short_transition.finite);
    CHECK(long_transition.finite);
    CHECK(short_transition.nonnegative_widths);
    CHECK(long_transition.nonnegative_widths);
    CHECK(short_transition.inside_outline_or_margin);
    CHECK(long_transition.inside_outline_or_margin);
    CHECK_THAT(long_transition.left_lobe.length_mm,
               Catch::Matchers::WithinRel(long_transition.right_lobe.length_mm, 0.05));
    CHECK_THAT(long_transition.left_lobe.width_area_mm2,
               Catch::Matchers::WithinRel(long_transition.right_lobe.width_area_mm2, 0.05));
    CHECK(std::abs(short_transition.width_squared_length_mm3 -
                   long_transition.width_squared_length_mm3) > 1e-3);
}


TEST_CASE("Arachne wall transition-angle boundaries change tapered-wall geometry", "[Arachne][ArachneWallTransitionAngle]")
{
    using namespace arachne_wall_transition_contracts;
    Polygon wedge;
    wedge.points.emplace_back(Point::new_scale(0., 0.));
    wedge.points.emplace_back(Point::new_scale(20., -5.359));
    wedge.points.emplace_back(Point::new_scale(20., 5.359));

    const auto low_angle_config = wall_config(100., 25., 1., 1, 0.005, 0.005);
    const auto high_angle_config = wall_config(100., 25., 59., 1, 0.005, 0.005);
    const WallToolPathsParams low_angle_params = params_for(low_angle_config);
    const WallToolPathsParams high_angle_params = params_for(high_angle_config);
    CHECK_THAT(low_angle_params.wall_transition_angle, Catch::Matchers::WithinAbs(1., 1e-6));
    CHECK_THAT(high_angle_params.wall_transition_angle, Catch::Matchers::WithinAbs(59., 1e-6));

    const WallSummary low_angle = generate(wedge, low_angle_params);
    const WallSummary high_angle = generate(wedge, high_angle_params);
    for (const WallSummary &row : {low_angle, high_angle}) {
        REQUIRE(row.lines > 0);
        CHECK(row.finite);
        CHECK(row.inside_fixture);
        CHECK(row.minimum_width_mm > 0.05);
        CHECK(row.length_mm > 1.);
    }
    CHECK(std::abs(low_angle.width_squared_length_mm3 - high_angle.width_squared_length_mm3) > 1e-4);
}

TEST_CASE("Arachne wall transition length honors its zero boundary", "[Arachne][ArachneWallTransitionLength]")
{
    using namespace arachne_wall_transition_contracts;
    const Polygon outline = plus_outline();
    const auto zero_config = wall_config(0., 25., 10., 3, 0.005, 0.005);
    const auto normal_config = wall_config(100., 25., 10., 3, 0.005, 0.005);
    const WallToolPathsParams zero_params = params_for(zero_config);
    const WallToolPathsParams normal_params = params_for(normal_config);
    CHECK_THAT(zero_params.wall_transition_length, Catch::Matchers::WithinAbs(0., 1e-6));
    CHECK_THAT(normal_params.wall_transition_length, Catch::Matchers::WithinAbs(0.4, 1e-6));

    const WallSummary zero_length = generate(outline, zero_params);
    const WallSummary normal_length = generate(outline, normal_params);
    for (const WallSummary &row : {zero_length, normal_length}) {
        REQUIRE(row.lines > 0);
        CHECK(row.finite);
        CHECK(row.inside_fixture);
        CHECK(row.minimum_width_mm > 0.05);
        CHECK(row.width_span_mm > 0.05);
        CHECK(row.length_mm > 0.5);
    }
    CHECK(std::abs(zero_length.width_squared_length_mm3 - normal_length.width_squared_length_mm3) > 1e-4);
}
