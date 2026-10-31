#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>

#include "libslic3r/PNGReadWrite.hpp"

#include "test_utils.hpp"

using namespace Slic3r;

// libpng reports a corrupt or truncated file by longjmp()ing out of the decoder, so the decoders have
// to come back with false rather than crash or hand back a half filled image.
namespace {

// A real PNG, produced by the writer next door, so the bytes are a file libpng accepts.
std::vector<uint8_t> encoded_png(size_t w, size_t h)
{
    std::vector<uint8_t> pixels(w * h);
    for (size_t i = 0; i < pixels.size(); ++ i)
        pixels[i] = uint8_t((i * 7) % 256);

    const boost::filesystem::path path = boost::filesystem::temp_directory_path() /
                                         boost::filesystem::unique_path("png_rw_%%%%%%%%.png");
    REQUIRE(png::write_gray_to_file(path.string(), w, h, pixels));
    std::vector<uint8_t> bytes;
    {
        std::ifstream ifs(path.string(), std::ios::binary);
        bytes.assign(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>());
    }
    boost::system::error_code ec;
    boost::filesystem::remove(path, ec);
    REQUIRE(bytes.size() > 64);
    return bytes;
}

png::ReadBuf buf_of(const std::vector<uint8_t> &bytes, size_t size)
{
    return png::ReadBuf{ bytes.data(), size };
}

std::vector<uint8_t> read_bytes(const std::string &path)
{
    std::ifstream ifs(path, std::ios::binary);
    std::vector<uint8_t> bytes{std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>()};
    REQUIRE_FALSE(bytes.empty());
    return bytes;
}

png::ImageGreyscale read_gray_file(const std::string &path)
{
    const std::vector<uint8_t> bytes = read_bytes(path);
    png::ImageGreyscale image;
    REQUIRE(png::decode_png(buf_of(bytes, bytes.size()), image));
    return image;
}

png::ImageColorscale read_color_file(const std::string &path)
{
    const std::vector<uint8_t> bytes = read_bytes(path);
    png::ImageColorscale image;
    REQUIRE(png::decode_colored_png(buf_of(bytes, bytes.size()), image));
    return image;
}

} // namespace

TEST_CASE("A whole PNG decodes", "[PNG]") {
    const std::vector<uint8_t> bytes = encoded_png(24, 16);

    png::ImageGreyscale grey;
    REQUIRE(png::decode_png(buf_of(bytes, bytes.size()), grey));
    CHECK(grey.cols == 24);
    CHECK(grey.rows == 16);
    CHECK(grey.buf.size() == 24 * 16);
}

TEST_CASE("A truncated PNG is refused instead of crashing", "[PNG]") {
    const std::vector<uint8_t> bytes = encoded_png(64, 64);

    // Cut past the signature: inside the header, and inside the pixel data. Not in the trailing
    // chunks - decode_png() does not read those, so a file missing only its IEND still decodes, and
    // that is the pre-existing contract rather than anything this change touches.
    const size_t size = GENERATE_COPY(size_t(16), size_t(40), bytes.size() / 2, bytes.size() * 3 / 4);
    REQUIRE(size < bytes.size());

    png::ImageGreyscale grey;
    CHECK_FALSE(png::decode_png(buf_of(bytes, size), grey));

    png::ImageColorscale colour;
    CHECK_FALSE(png::decode_colored_png(buf_of(bytes, size), colour));
}

TEST_CASE("A PNG whose body is garbage is refused", "[PNG]") {
    std::vector<uint8_t> bytes = encoded_png(32, 32);
    // Keep the signature, scribble over everything after it.
    for (size_t i = 8; i < bytes.size(); ++ i)
        bytes[i] = uint8_t(0xA5);

    png::ImageGreyscale grey;
    CHECK_FALSE(png::decode_png(buf_of(bytes, bytes.size()), grey));

    png::ImageColorscale colour;
    CHECK_FALSE(png::decode_colored_png(buf_of(bytes, bytes.size()), colour));
}

// Each writer has const char *, std::string + pointer and std::string + vector overloads; all three must
// produce the same image. Scaling repeats every pixel scale x scale times without smoothing.
TEST_CASE("Grayscale PNG writer overloads preserve pixels and nearest-neighbor scaling", "[PNG]") {
    constexpr size_t width = 2;
    constexpr size_t height = 3;
    const std::vector<uint8_t> pixels{11, 37, 83, 129, 177, 241};
    const std::vector<uint8_t> scaled_pixels{
        11, 11, 37, 37, 11, 11, 37, 37,
        83, 83, 129, 129, 83, 83, 129, 129,
        177, 177, 241, 241, 177, 177, 241, 241
    };

    ScopedTemporaryFile by_char(".png"), by_pointer(".png"), by_vector(".png");
    REQUIRE(png::write_gray_to_file(by_char.string().c_str(), width, height, pixels.data()));
    REQUIRE(png::write_gray_to_file(by_pointer.string(), width, height, pixels.data()));
    REQUIRE(png::write_gray_to_file(by_vector.string(), width, height, pixels));
    for (const ScopedTemporaryFile *file : {&by_char, &by_pointer, &by_vector}) {
        const png::ImageGreyscale image = read_gray_file(file->string());
        CHECK(image.cols == width);
        CHECK(image.rows == height);
        CHECK(image.buf == pixels);
    }

    ScopedTemporaryFile scaled_char(".png"), scaled_pointer(".png"), scaled_vector(".png"), scale_one(".png");
    REQUIRE(png::write_gray_to_file_scaled(scaled_char.string().c_str(), width, height, pixels.data(), 2));
    REQUIRE(png::write_gray_to_file_scaled(scaled_pointer.string(), width, height, pixels.data(), 2));
    REQUIRE(png::write_gray_to_file_scaled(scaled_vector.string(), width, height, pixels, 2));
    REQUIRE(png::write_gray_to_file_scaled(scale_one.string(), width, height, pixels, 1));
    for (const ScopedTemporaryFile *file : {&scaled_char, &scaled_pointer, &scaled_vector}) {
        const png::ImageGreyscale image = read_gray_file(file->string());
        CHECK(image.cols == width * 2);
        CHECK(image.rows == height * 2);
        CHECK(image.buf == scaled_pixels);
    }
    const png::ImageGreyscale unscaled = read_gray_file(scale_one.string());
    CHECK(unscaled.cols == width);
    CHECK(unscaled.rows == height);
    CHECK(unscaled.buf == pixels);
}

// decode_colored_png() hands rows back last row first, so a top-down RGB image decodes upside down.
TEST_CASE("RGB PNG writer overloads preserve channels and decoded row order", "[PNG]") {
    constexpr size_t width = 2;
    constexpr size_t height = 3;
    const std::vector<uint8_t> pixels{
        1, 2, 3, 4, 5, 6,
        7, 8, 9, 10, 11, 12,
        13, 14, 15, 16, 17, 18
    };
    const std::vector<uint8_t> decoded_bottom_up{
        13, 14, 15, 16, 17, 18,
        7, 8, 9, 10, 11, 12,
        1, 2, 3, 4, 5, 6
    };
    const std::vector<uint8_t> scaled_bottom_up{
        13, 14, 15, 13, 14, 15, 16, 17, 18, 16, 17, 18,
        13, 14, 15, 13, 14, 15, 16, 17, 18, 16, 17, 18,
        7, 8, 9, 7, 8, 9, 10, 11, 12, 10, 11, 12,
        7, 8, 9, 7, 8, 9, 10, 11, 12, 10, 11, 12,
        1, 2, 3, 1, 2, 3, 4, 5, 6, 4, 5, 6,
        1, 2, 3, 1, 2, 3, 4, 5, 6, 4, 5, 6
    };

    ScopedTemporaryFile by_char(".png"), by_pointer(".png"), by_vector(".png");
    REQUIRE(png::write_rgb_to_file(by_char.string().c_str(), width, height, pixels.data()));
    REQUIRE(png::write_rgb_to_file(by_pointer.string(), width, height, pixels.data()));
    REQUIRE(png::write_rgb_to_file(by_vector.string(), width, height, pixels));
    for (const ScopedTemporaryFile *file : {&by_char, &by_pointer, &by_vector}) {
        const png::ImageColorscale image = read_color_file(file->string());
        CHECK(image.cols == width);
        CHECK(image.rows == height);
        CHECK(image.bytes_per_pixel == 3);
        CHECK(image.buf == decoded_bottom_up);
    }

    ScopedTemporaryFile scaled_char(".png"), scaled_pointer(".png"), scaled_vector(".png"), scale_one(".png");
    REQUIRE(png::write_rgb_to_file_scaled(scaled_char.string().c_str(), width, height, pixels.data(), 2));
    REQUIRE(png::write_rgb_to_file_scaled(scaled_pointer.string(), width, height, pixels.data(), 2));
    REQUIRE(png::write_rgb_to_file_scaled(scaled_vector.string(), width, height, pixels, 2));
    REQUIRE(png::write_rgb_to_file_scaled(scale_one.string().c_str(), width, height, pixels.data(), 1));
    for (const ScopedTemporaryFile *file : {&scaled_char, &scaled_pointer, &scaled_vector}) {
        const png::ImageColorscale image = read_color_file(file->string());
        CHECK(image.cols == width * 2);
        CHECK(image.rows == height * 2);
        CHECK(image.bytes_per_pixel == 3);
        CHECK(image.buf == scaled_bottom_up);
    }
    const png::ImageColorscale unscaled = read_color_file(scale_one.string());
    CHECK(unscaled.cols == width);
    CHECK(unscaled.rows == height);
    CHECK(unscaled.bytes_per_pixel == 3);
    CHECK(unscaled.buf == decoded_bottom_up);
}

TEST_CASE("Colored PNG decoding rejects a non-PNG signature", "[PNG]") {
    // A valid RGB PNG whose only defect is its signature, so nothing but the signature check refuses it.
    ScopedTemporaryFile file(".png");
    const std::vector<uint8_t> pixels{1, 2, 3};
    REQUIRE(png::write_rgb_to_file(file.string(), 1, 1, pixels));
    std::vector<uint8_t> bytes = read_bytes(file.string());

    png::ImageColorscale image;
    REQUIRE(png::decode_colored_png(buf_of(bytes, bytes.size()), image));
    bytes[0] = 0;
    CHECK_FALSE(png::decode_colored_png(buf_of(bytes, bytes.size()), image));
}

TEST_CASE("PNG writing fails beneath a missing temporary child directory", "[PNG]") {
    ScopedTemporaryDir temporary_root("orca_png_writer_failure");
    const boost::filesystem::path absent_parent = temporary_root.path() / "missing-child";
    const std::vector<uint8_t> pixel{42};
    CHECK_FALSE(png::write_gray_to_file((absent_parent / "image.png").string(), 1, 1, pixel));
    CHECK_FALSE(boost::filesystem::exists(absent_parent));
}
