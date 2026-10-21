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

png::ImageGreyscale read_gray_file(const std::string &path)
{
    std::vector<uint8_t> bytes;
    {
        std::ifstream ifs(path, std::ios::binary);
        bytes.assign(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>());
    }
    REQUIRE_FALSE(bytes.empty());
    png::ImageGreyscale image;
    REQUIRE(png::decode_png(buf_of(bytes, bytes.size()), image));
    return image;
}

png::ImageColorscale read_color_file(const std::string &path)
{
    std::vector<uint8_t> bytes;
    {
        std::ifstream ifs(path, std::ios::binary);
        bytes.assign(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>());
    }
    REQUIRE_FALSE(bytes.empty());
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


TEST_CASE("Grayscale PNG writer overloads preserve pixels and nearest-neighbor scaling", "[PNG]") {
    constexpr size_t width = 2;
    constexpr size_t height = 3;
    const std::vector<uint8_t> pixels{11, 37, 83, 129, 177, 241};
    const std::vector<uint8_t> scaled_pixels{
        11, 11, 37, 37, 11, 11, 37, 37,
        83, 83, 129, 129, 83, 83, 129, 129,
        177, 177, 241, 241, 177, 177, 241, 241
    };

    ScopedTemporaryFile char_writer(".png");
    ScopedTemporaryFile pointer_writer(".png");
    ScopedTemporaryFile vector_writer(".png");
    const std::string char_path = char_writer.string();
    const std::string pointer_path = pointer_writer.string();
    const std::string vector_path = vector_writer.string();
    REQUIRE(png::write_gray_to_file(char_path.c_str(), width, height, pixels.data()));
    REQUIRE(png::write_gray_to_file(pointer_path, width, height, pixels.data()));
    REQUIRE(png::write_gray_to_file(vector_path, width, height, pixels));

    for (const std::string &path : {char_path, pointer_path, vector_path}) {
        const png::ImageGreyscale image = read_gray_file(path);
        CHECK(image.cols == width);
        CHECK(image.rows == height);
        CHECK(image.buf == pixels);
    }

    ScopedTemporaryFile scaled_char_writer(".png");
    ScopedTemporaryFile scaled_pointer_writer(".png");
    ScopedTemporaryFile scaled_vector_writer(".png");
    ScopedTemporaryFile scale_one_writer(".png");
    const std::string scaled_char_path = scaled_char_writer.string();
    const std::string scaled_pointer_path = scaled_pointer_writer.string();
    const std::string scaled_vector_path = scaled_vector_writer.string();
    const std::string scale_one_path = scale_one_writer.string();
    REQUIRE(png::write_gray_to_file_scaled(scaled_char_path.c_str(), width, height, pixels.data(), 2));
    REQUIRE(png::write_gray_to_file_scaled(scaled_pointer_path, width, height, pixels.data(), 2));
    REQUIRE(png::write_gray_to_file_scaled(scaled_vector_path, width, height, pixels, 2));
    REQUIRE(png::write_gray_to_file_scaled(scale_one_path, width, height, pixels, 1));

    for (const std::string &path : {scaled_char_path, scaled_pointer_path, scaled_vector_path}) {
        const png::ImageGreyscale image = read_gray_file(path);
        CHECK(image.cols == width * 2);
        CHECK(image.rows == height * 2);
        CHECK(image.buf == scaled_pixels);
    }
    const png::ImageGreyscale scale_one = read_gray_file(scale_one_path);
    CHECK(scale_one.cols == width);
    CHECK(scale_one.rows == height);
    CHECK(scale_one.buf == pixels);
}

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

    ScopedTemporaryFile char_writer(".png");
    ScopedTemporaryFile pointer_writer(".png");
    ScopedTemporaryFile vector_writer(".png");
    const std::string char_path = char_writer.string();
    const std::string pointer_path = pointer_writer.string();
    const std::string vector_path = vector_writer.string();
    REQUIRE(png::write_rgb_to_file(char_path.c_str(), width, height, pixels.data()));
    REQUIRE(png::write_rgb_to_file(pointer_path, width, height, pixels.data()));
    REQUIRE(png::write_rgb_to_file(vector_path, width, height, pixels));

    for (const std::string &path : {char_path, pointer_path, vector_path}) {
        const png::ImageColorscale image = read_color_file(path);
        CHECK(image.cols == width);
        CHECK(image.rows == height);
        CHECK(image.bytes_per_pixel == 3);
        CHECK(image.buf == decoded_bottom_up);
    }

    ScopedTemporaryFile scaled_char_writer(".png");
    ScopedTemporaryFile scaled_pointer_writer(".png");
    ScopedTemporaryFile scaled_vector_writer(".png");
    ScopedTemporaryFile scale_one_writer(".png");
    const std::string scaled_char_path = scaled_char_writer.string();
    const std::string scaled_pointer_path = scaled_pointer_writer.string();
    const std::string scaled_vector_path = scaled_vector_writer.string();
    const std::string scale_one_path = scale_one_writer.string();
    REQUIRE(png::write_rgb_to_file_scaled(scaled_char_path.c_str(), width, height, pixels.data(), 2));
    REQUIRE(png::write_rgb_to_file_scaled(scaled_pointer_path, width, height, pixels.data(), 2));
    REQUIRE(png::write_rgb_to_file_scaled(scaled_vector_path, width, height, pixels, 2));
    REQUIRE(png::write_rgb_to_file_scaled(scale_one_path.c_str(), width, height, pixels.data(), 1));

    for (const std::string &path : {scaled_char_path, scaled_pointer_path, scaled_vector_path}) {
        const png::ImageColorscale image = read_color_file(path);
        CHECK(image.cols == width * 2);
        CHECK(image.rows == height * 2);
        CHECK(image.bytes_per_pixel == 3);
        CHECK(image.buf == scaled_bottom_up);
    }
    const png::ImageColorscale scale_one = read_color_file(scale_one_path);
    CHECK(scale_one.cols == width);
    CHECK(scale_one.rows == height);
    CHECK(scale_one.bytes_per_pixel == 3);
    CHECK(scale_one.buf == decoded_bottom_up);
}

TEST_CASE("Colored PNG decoding rejects a non-PNG signature", "[PNG]") {
    const std::vector<uint8_t> invalid_signature(8, 0);
    png::ImageColorscale image;
    CHECK_FALSE(png::decode_colored_png(buf_of(invalid_signature, invalid_signature.size()), image));
}

TEST_CASE("PNG writing fails beneath a missing temporary child directory", "[PNG]") {
    ScopedTemporaryDir temporary_root("orca_png_writer_failure");
    const boost::filesystem::path absent_parent = temporary_root.path() / "missing-child";
    const boost::filesystem::path absent_file = absent_parent / "image.png";
    const std::vector<uint8_t> pixel{42};
    CHECK_FALSE(png::write_gray_to_file(absent_file.string(), 1, 1, pixel));
    CHECK_FALSE(boost::filesystem::exists(absent_parent));
}
