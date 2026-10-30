#include <catch2/catch_all.hpp>

#include <cstdint>
#include <fstream>
#include <iterator>
#include <vector>

#include <boost/filesystem.hpp>

#include "libslic3r/PNGReadWrite.hpp"
#include "libslic3r/GCode/Thumbnails.hpp"

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

TEST_CASE("A JPEG thumbnail larger than the raw pixel buffer is returned complete", "[Thumbnails]")
{
    // A 2x2 RGBA image is 16 bytes of pixels, but any JPEG stream (headers and tables
    // alone) is several hundred bytes, so the encoder has to grow its output buffer.
    Slic3r::ThumbnailData data;
    data.set(2, 2);
    for (size_t i = 0; i < data.pixels.size(); i += 4) {
        data.pixels[i + 0] = 200;
        data.pixels[i + 1] = 100;
        data.pixels[i + 2] = 50;
        data.pixels[i + 3] = 255;
    }
    const auto jpg = Slic3r::GCodeThumbnails::compress_thumbnail(data, Slic3r::GCodeThumbnailsFormat::JPG);
    REQUIRE(jpg != nullptr);
    REQUIRE(jpg->size > data.pixels.size());
    const auto *bytes = static_cast<const unsigned char *>(jpg->data);
    // Start-of-image and end-of-image markers.
    CHECK(bytes[0] == 0xFF);
    CHECK(bytes[1] == 0xD8);
    CHECK(bytes[jpg->size - 2] == 0xFF);
    CHECK(bytes[jpg->size - 1] == 0xD9);
}
