#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/ObjColorUtils.hpp"
#include "libslic3r/Color.hpp"
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <set>
#include <vector>

using namespace Slic3r;
using Catch::Matchers::WithinAbs;
namespace {
struct RngRestore {
    uint64_t state = cv::theRNG().state;
    ~RngRestore() { cv::theRNG().state = state; }
};

using Bgr = cv::Vec3b;
using Rgba = cv::Vec4b;

void require_vec(const Bgr &actual, const Bgr &expected)
{
    for (int i = 0; i < 3; ++i)
        CHECK(actual[i] == expected[i]);
}

} // namespace

TEST_CASE("ObjColorUtils counts unique BGR colors and honors requested cardinality", "[ObjColorUtils][ObjColorUtilsExtended]")
{
    QuantKMeans quant;
    cv::Mat pixels(4, 1, CV_8UC3);
    pixels.at<Bgr>(0) = Bgr(1, 2, 3);
    pixels.at<Bgr>(1) = Bgr(1, 2, 3);
    pixels.at<Bgr>(2) = Bgr(4, 5, 6);
    pixels.at<Bgr>(3) = Bgr(7, 8, 9);
    CHECK(quant.compute_num_colors(pixels) == 3);
    CHECK(quant.more_than_request(pixels, 3));
    CHECK_FALSE(quant.more_than_request(pixels, 4));
    CHECK(quant.is_in(Bgr(4, 5, 6), {Bgr(1, 2, 3), Bgr(4, 5, 6)}));
    CHECK_FALSE(quant.is_in(Bgr(4, 5, 7), {Bgr(1, 2, 3), Bgr(4, 5, 6)}));
    CHECK_FALSE(quant.is_in(Bgr(0, 0, 0), {Bgr(1, 2, 3), Bgr(4, 5, 6)}));
}

TEST_CASE("ObjColorUtils flatten_vector performs independent normalized-to-byte conversion", "[ObjColorUtils][ObjColorUtilsExtended]")
{
    QuantKMeans quant;
    const std::vector<RGBA> values{{0.0f, 0.5f, 1.0f, 0.1f}, {0.25f, 0.75f, 0.125f, 1.0f}};
    const cv::Mat flattened = quant.flatten_vector(values);
    REQUIRE(flattened.rows == 2);
    REQUIRE(flattened.cols == 1);
    REQUIRE(flattened.type() == CV_8UC3);
    require_vec(flattened.at<Bgr>(0), Bgr(0, 127, 255));
    require_vec(flattened.at<Bgr>(1), Bgr(63, 191, 31));
}

TEST_CASE("ObjColorUtils flatten_alpha removes only pixels below the alpha threshold", "[ObjColorUtils][ObjColorUtilsExtended]")
{
    QuantKMeans quant(10);
    cv::Mat rgba(3, 1, CV_8UC4);
    rgba.at<Rgba>(0) = Rgba(10, 20, 30, 9);
    rgba.at<Rgba>(1) = Rgba(40, 50, 60, 10);
    rgba.at<Rgba>(2) = Rgba(70, 80, 90, 255);
    const cv::Mat flattened = quant.flatten_alpha(rgba);
    REQUIRE(flattened.rows == 2);
    REQUIRE(flattened.cols == 1);
    REQUIRE(flattened.type() == CV_8UC3);
    require_vec(flattened.at<Bgr>(0), Bgr(40, 50, 60));
    require_vec(flattened.at<Bgr>(1), Bgr(70, 80, 90));
}

TEST_CASE("ObjColorUtils color-space conversion has known BGR HSV roundtrip", "[ObjColorUtils][ObjColorUtilsExtended]")
{
    QuantKMeans quant;
    cv::Mat bgr(1, 1, CV_8UC3), hsv, roundtrip;
    bgr.at<Bgr>(0) = Bgr(0, 0, 255);
    quant.convert_color_space(bgr, hsv, 1);
    REQUIRE(hsv.rows == 1);
    REQUIRE(hsv.cols == 1);
    REQUIRE(hsv.type() == CV_8UC3);
    require_vec(hsv.at<Bgr>(0), Bgr(0, 255, 255));
    quant.convert_color_space(hsv, roundtrip, 1, true);
    REQUIRE(roundtrip.rows == 1);
    REQUIRE(roundtrip.cols == 1);
    REQUIRE(roundtrip.type() == CV_8UC3);
    require_vec(roundtrip.at<Bgr>(0), Bgr(0, 0, 255));
}

TEST_CASE("ObjColorUtils repeat_center distinguishes distinct and duplicate centers", "[ObjColorUtils][ObjColorUtilsExtended]")
{
    QuantKMeans quant;
    cv::Mat distinct(2, 1, CV_32FC3), repeated(2, 1, CV_32FC3);
    distinct.at<cv::Vec3f>(0) = cv::Vec3f(0.f, 0.f, 0.f);
    distinct.at<cv::Vec3f>(1) = cv::Vec3f(255.f, 0.f, 0.f);
    repeated.at<cv::Vec3f>(0) = cv::Vec3f(12.f, 34.f, 56.f);
    repeated.at<cv::Vec3f>(1) = cv::Vec3f(12.f, 34.f, 56.f);
    CHECK_FALSE(quant.repeat_center(2, distinct, 0));
    CHECK(quant.repeat_center(2, repeated, 0));
}

TEST_CASE("ObjColorUtils image application replaces every pixel with its cluster center", "[ObjColorUtils][ObjColorUtilsExtended]")
{
    RngRestore restore;
    cv::theRNG().state = 12345;
    QuantKMeans quant;
    // Four shades in two clusters: each pixel must become its cluster mean, which no input pixel equals.
    cv::Mat input(2, 2, CV_8UC3), output(2, 2, CV_8UC3, cv::Scalar(0, 0, 0));
    input.at<Bgr>(0, 0) = Bgr(0, 0, 250);
    input.at<Bgr>(0, 1) = Bgr(0, 0, 254);
    input.at<Bgr>(1, 0) = Bgr(250, 0, 0);
    input.at<Bgr>(1, 1) = Bgr(254, 0, 0);
    quant.apply(input, output, 2, 0);
    REQUIRE(output.rows == input.rows);
    REQUIRE(output.cols == input.cols);
    REQUIRE(output.type() == CV_8UC3);
    require_vec(output.at<Bgr>(0, 0), Bgr(0, 0, 252));
    require_vec(output.at<Bgr>(0, 1), Bgr(0, 0, 252));
    require_vec(output.at<Bgr>(1, 0), Bgr(252, 0, 0));
    require_vec(output.at<Bgr>(1, 1), Bgr(252, 0, 0));
}

TEST_CASE("ObjColorUtils alpha application preserves transparent pixels and alpha bytes", "[ObjColorUtils][ObjColorUtilsExtended]")
{
    RngRestore restore;
    cv::theRNG().state = 54321;
    QuantKMeans quant(10);
    // One transparent pixel and four opaque shades in two clusters.
    cv::Mat input(1, 5, CV_8UC4), output(1, 5, CV_8UC4, cv::Scalar(0, 0, 0, 0));
    input.at<Rgba>(0, 0) = Rgba(5, 6, 7, 0);
    input.at<Rgba>(0, 1) = Rgba(0, 0, 250, 255);
    input.at<Rgba>(0, 2) = Rgba(0, 0, 254, 200);
    input.at<Rgba>(0, 3) = Rgba(250, 0, 0, 128);
    input.at<Rgba>(0, 4) = Rgba(254, 0, 0, 64);
    quant.apply_aplha(input, output, 2, 0);
    REQUIRE(output.rows == input.rows);
    REQUIRE(output.cols == input.cols);
    REQUIRE(output.type() == CV_8UC4);
    CHECK(output.at<Rgba>(0, 0) == input.at<Rgba>(0, 0));
    CHECK(output.at<Rgba>(0, 1) == Rgba(0, 0, 252, 255));
    CHECK(output.at<Rgba>(0, 2) == Rgba(0, 0, 252, 200));
    CHECK(output.at<Rgba>(0, 3) == Rgba(252, 0, 0, 128));
    CHECK(output.at<Rgba>(0, 4) == Rgba(252, 0, 0, 64));
}

TEST_CASE("ObjColorUtils vector clustering retains two exact palette regions", "[ObjColorUtils][ObjColorUtilsExtended]")
{
    RngRestore restore;
    cv::theRNG().state = 13579;
    QuantKMeans quant;
    std::vector<RGBA> colors{{1.f, 0.f, 0.f, 1.f}, {1.f, 0.f, 0.f, 1.f},
                             {0.f, 0.f, 1.f, 1.f}, {0.f, 0.f, 1.f, 1.f}};
    std::vector<RGBA> centers;
    std::vector<int> labels;
    quant.apply(colors, centers, labels, 2, 2, 0);
    REQUIRE(centers.size() == 2);
    REQUIRE(labels.size() == colors.size());
    std::set<std::array<int, 3>> center_bytes;
    for (const RGBA &center : centers)
        center_bytes.insert({int(std::lround(center[0] * 255.f)), int(std::lround(center[1] * 255.f)), int(std::lround(center[2] * 255.f))});
    CHECK(center_bytes.count({255, 0, 0}) == 1);
    CHECK(center_bytes.count({0, 0, 255}) == 1);
    CHECK(labels[0] == labels[1]);
    CHECK(labels[2] == labels[3]);
    CHECK(labels[0] != labels[2]);
    for (std::size_t i = 0; i < labels.size(); ++i) {
        REQUIRE(labels[i] >= 0);
        REQUIRE(static_cast<std::size_t>(labels[i]) < centers.size());
        CHECK_THAT(centers[static_cast<std::size_t>(labels[i])][0], WithinAbs(colors[i][0], 1e-6));
        CHECK_THAT(centers[static_cast<std::size_t>(labels[i])][1], WithinAbs(colors[i][1], 1e-6));
        CHECK_THAT(centers[static_cast<std::size_t>(labels[i])][2], WithinAbs(colors[i][2], 1e-6));
    }
}

TEST_CASE("ObjColorUtils requested clusters collapse to the number of distinct samples", "[ObjColorUtils][ObjColorUtilsExtended]")
{
    RngRestore restore;
    cv::theRNG().state = 24680;
    QuantKMeans quant;
    const std::vector<RGBA> colors{{0.f, 1.f, 0.f, 1.f}, {0.f, 1.f, 0.f, 1.f}, {1.f, 1.f, 0.f, 1.f}};
    std::vector<RGBA> centers;
    std::vector<int> labels;
    quant.apply(colors, centers, labels, 4, 4, 0);
    REQUIRE(centers.size() == 2);
    REQUIRE(labels.size() == colors.size());
    CHECK(labels[0] == labels[1]);
    CHECK(labels[0] != labels[2]);
    for (std::size_t i = 0; i < labels.size(); ++i) {
        REQUIRE(labels[i] >= 0);
        REQUIRE(static_cast<std::size_t>(labels[i]) < centers.size());
        CHECK_THAT(centers[static_cast<std::size_t>(labels[i])][0], WithinAbs(colors[i][0], 1e-6));
        CHECK_THAT(centers[static_cast<std::size_t>(labels[i])][1], WithinAbs(colors[i][1], 1e-6));
        CHECK_THAT(centers[static_cast<std::size_t>(labels[i])][2], WithinAbs(colors[i][2], 1e-6));
    }
}

TEST_CASE("ObjColorUtils automatic cluster count finds every distinct color", "[ObjColorUtils][ObjColorUtilsExtended]")
{
    RngRestore restore;
    cv::theRNG().state = 11235;
    QuantKMeans quant;
    // num_cluster < 1 selects the cluster count by k-means score; three exact colors need three clusters.
    const std::vector<RGBA> colors{{1.f, 0.f, 0.f, 1.f}, {1.f, 0.f, 0.f, 1.f}, {0.f, 1.f, 0.f, 1.f},
                                   {0.f, 1.f, 0.f, 1.f}, {0.f, 0.f, 1.f, 1.f}, {0.f, 0.f, 1.f, 1.f}};
    std::vector<RGBA> centers;
    std::vector<int> labels;
    quant.apply(colors, centers, labels, -1, 4, 0);
    REQUIRE(centers.size() == 3);
    REQUIRE(labels.size() == colors.size());
    CHECK(labels[0] == labels[1]);
    CHECK(labels[2] == labels[3]);
    CHECK(labels[4] == labels[5]);
    CHECK(std::set<int>{labels[0], labels[2], labels[4]}.size() == 3);
    for (std::size_t i = 0; i < labels.size(); ++i) {
        REQUIRE(labels[i] >= 0);
        REQUIRE(static_cast<std::size_t>(labels[i]) < centers.size());
        for (int channel = 0; channel < 3; ++channel)
            CHECK_THAT(centers[static_cast<std::size_t>(labels[i])][channel], WithinAbs(colors[i][channel], 1e-6));
    }
}

TEST_CASE("ObjColorUtils obj_color_deal_algo reconstructs two exact palette regions", "[ObjColorUtils][ObjColorUtilsExtended]")
{
    RngRestore restore;
    cv::theRNG().state = 97531;
    std::vector<RGBA> colors{{1.f, 0.f, 0.f, 1.f}, {1.f, 0.f, 0.f, 1.f},
                             {0.f, 0.f, 1.f, 1.f}, {0.f, 0.f, 1.f, 1.f}};
    std::vector<RGBA> centers;
    std::vector<int> labels;
    char cluster_number = 2;
    REQUIRE(obj_color_deal_algo(colors, centers, labels, cluster_number, 3));
    REQUIRE(centers.size() == 2);
    REQUIRE(labels.size() == colors.size());
    CHECK(labels[0] == labels[1]);
    CHECK(labels[2] == labels[3]);
    CHECK(labels[0] != labels[2]);
    float reconstruction_error = 0.f;
    for (std::size_t i = 0; i < labels.size(); ++i) {
        REQUIRE(labels[i] >= 0);
        REQUIRE(static_cast<std::size_t>(labels[i]) < centers.size());
        // The byte-quantized Lab conversion and inverse may shift an RGB
        // channel by a few byte steps; tolerate at most three here.
        constexpr float byte_tolerance = 3.f / 255.f + 1e-6f;
        CHECK_THAT(centers[static_cast<std::size_t>(labels[i])][0], WithinAbs(colors[i][0], byte_tolerance));
        CHECK_THAT(centers[static_cast<std::size_t>(labels[i])][1], WithinAbs(colors[i][1], byte_tolerance));
        CHECK_THAT(centers[static_cast<std::size_t>(labels[i])][2], WithinAbs(colors[i][2], byte_tolerance));
        for (int channel = 0; channel < 3; ++channel) {
            const float delta = centers[static_cast<std::size_t>(labels[i])][channel] - colors[i][channel];
            reconstruction_error += delta * delta;
        }
    }
    // The two exact input colors form the unique two-center optimum.  This
    // aggregate error also rejects a center that is individually plausible
    // but assigned to the wrong palette region.
    CHECK(reconstruction_error < 0.001f);
}

TEST_CASE("ObjColorUtils empty image helpers report zero colors without fabricating clusters", "[ObjColorUtils][ObjColorUtilsExtended]")
{
    QuantKMeans quant;
    const cv::Mat pixels = cv::Mat::zeros(0, 1, CV_8UC3);
    CHECK(quant.compute_num_colors(pixels) == 0);
    CHECK_FALSE(quant.more_than_request(pixels, 1));
    CHECK_FALSE(quant.is_in(Bgr(1, 2, 3), {}));
}
