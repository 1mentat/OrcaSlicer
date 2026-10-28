#include <charconv>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <jpeglib.h>
#include <boost/beast/core/detail/base64.hpp>
#include <csetjmp>
#include <cmath>
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_message.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "libslic3r/Config.hpp"
#include "libslic3r/GCode/ThumbnailData.hpp"
#include "libslic3r/GCode/Thumbnails.hpp"
#include "libslic3r/PNGReadWrite.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "test_utils.hpp"
#include <qoi/qoi.h>
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

namespace thumbnail_encoding_contracts {
using namespace Slic3r;

namespace {

ThumbnailData make_pattern(unsigned int width = 3, unsigned int height = 2)
{
    ThumbnailData data;
    data.set(width, height);
    for (unsigned int y = 0; y < height; ++y) {
        for (unsigned int x = 0; x < width; ++x) {
            const size_t i = 4 * (size_t(y) * width + x);
            data.pixels[i + 0] = static_cast<unsigned char>(20 + 30 * x);
            data.pixels[i + 1] = static_cast<unsigned char>(40 + 40 * y);
            data.pixels[i + 2] = static_cast<unsigned char>(80 + 20 * x + 10 * y);
            data.pixels[i + 3] = static_cast<unsigned char>(255 - 40 * (x + y));
        }
    }
    return data;
}

std::pair<GCodeThumbnails::GCodeThumbnailDefinitionsList, ThumbnailErrors> parse(const std::string &value,
                                                                                    std::string_view default_ext = "PNG")
{
    return GCodeThumbnails::make_and_check_thumbnail_list(value, default_ext);
}

TEST_CASE("ThumbnailData set, reset, and validity preserve the four-channel invariant", "[Thumbnails][ThumbnailsExtended][ThumbnailData]")
{
    ThumbnailData data;
    CHECK_FALSE(data.is_valid());
    data.set(0, 4);
    CHECK(data.width == 0);
    CHECK(data.height == 0);
    CHECK(data.pixels.empty());

    data.set(2, 3);
    REQUIRE(data.is_valid());
    CHECK(data.pixels.size() == 24);
    CHECK(std::all_of(data.pixels.begin(), data.pixels.end(), [](unsigned char value) { return value == 255; }));

    data.pixels.pop_back();
    CHECK_FALSE(data.is_valid());
    data.reset();
    CHECK_FALSE(data.is_valid());
    CHECK(data.pixels.empty());
}

TEST_CASE("ThumbnailData load_from makes an owned pixel copy", "[Thumbnails][ThumbnailsExtended][ThumbnailData]")
{
    ThumbnailData source = make_pattern(2, 1);
    ThumbnailData copy;
    copy.load_from(source);
    REQUIRE(copy.is_valid());
    CHECK(copy.width == source.width);
    CHECK(copy.height == source.height);
    CHECK(copy.pixels == source.pixels);
    source.pixels[0] = 0;
    CHECK(copy.pixels[0] != source.pixels[0]);
}

TEST_CASE("Thumbnail parser accepts case-insensitive formats and preserves dimensions", "[Thumbnails][ThumbnailsExtended][Parser]")
{
    auto [list, errors] = parse("320x240/png,128x64/JPG,64x32/qOi");
    REQUIRE(errors == ThumbnailErrors());
    REQUIRE(list.size() == 3);
    CHECK(list[0].first == GCodeThumbnailsFormat::PNG);
    CHECK(list[1].first == GCodeThumbnailsFormat::JPG);
    CHECK(list[2].first == GCodeThumbnailsFormat::QOI);
    CHECK(list[0].second == Vec2d(320, 240));
    CHECK(list[2].second == Vec2d(64, 32));
}

TEST_CASE("Thumbnail parser applies the default extension and reports independent errors", "[Thumbnails][ThumbnailsExtended][Parser]")
{
    auto [defaults, default_errors] = parse("16x8/,20x10", "QOI");
    REQUIRE(default_errors == ThumbnailErrors());
    REQUIRE(defaults.size() == 2);
    CHECK(defaults[0].first == GCodeThumbnailsFormat::QOI);
    CHECK(defaults[1].first == GCodeThumbnailsFormat::QOI);

    auto [invalid, errors] = parse("0x8/PNG,1000x20/PNG,4x5/BMP,not-a-thumbnail");
    CHECK(invalid.size() == 1);
    CHECK(invalid[0].first == GCodeThumbnailsFormat::PNG);
    CHECK(errors.has(ThumbnailError::OutOfRange));
    CHECK(errors.has(ThumbnailError::InvalidExt));
    CHECK(errors.has(ThumbnailError::InvalidVal));
    CHECK(!GCodeThumbnails::get_error_string(errors).empty());
}

TEST_CASE("Thumbnail parser reads the configured thumbnails string", "[Thumbnails][ThumbnailsExtended][Parser][Config]")
{
    DynamicPrintConfig config;
    config.set_key_value("thumbnails", new ConfigOptionString("40x30/PNG,20x10/QOI"));
    auto [list, errors] = GCodeThumbnails::make_and_check_thumbnail_list(config);
    REQUIRE(errors == ThumbnailErrors());
    REQUIRE(list.size() == 2);
    CHECK(list[0].second == Vec2d(40, 30));
    CHECK(list[1].first == GCodeThumbnailsFormat::QOI);
}

TEST_CASE("PNG thumbnail compression decodes exact dimensions and pixels", "[Thumbnails][ThumbnailsExtended][Compression][PNG]")
{
    ThumbnailData data;
    data.set(2, 2);
    data.pixels = {
        0x12, 0x34, 0x56, 0x78, 0x90, 0xab, 0xcd, 0xef,
        0x21, 0x43, 0x65, 0x87, 0xfe, 0xdc, 0xba, 0x98,
    };
    auto encoded = GCodeThumbnails::compress_thumbnail(data, GCodeThumbnailsFormat::PNG);
    REQUIRE(encoded != nullptr);
    REQUIRE(encoded->tag() == "thumbnail");
    png::ImageColorscale decoded;
    REQUIRE(png::decode_colored_png(png::ReadBuf{encoded->data, encoded->size}, decoded));
    REQUIRE(decoded.cols == 2);
    REQUIRE(decoded.rows == 2);
    REQUIRE(decoded.bytes_per_pixel == 4);
    REQUIRE(decoded.buf.size() == data.pixels.size());
    CHECK(decoded.buf == data.pixels);
}

TEST_CASE("Unknown thumbnail format values use the PNG compression fallback", "[Thumbnails][ThumbnailsExtended][Compression][PNG]")
{
    const ThumbnailData data = make_pattern(2, 2);
    auto encoded = GCodeThumbnails::compress_thumbnail(data, static_cast<GCodeThumbnailsFormat>(99));
    REQUIRE(encoded != nullptr);
    CHECK(encoded->tag() == "thumbnail");
    png::ImageColorscale decoded;
    REQUIRE(png::decode_colored_png(png::ReadBuf{encoded->data, encoded->size}, decoded));
    CHECK(decoded.cols == 2);
    CHECK(decoded.rows == 2);
    CHECK(decoded.bytes_per_pixel == 4);
    CHECK(decoded.buf == data.pixels);
}

TEST_CASE("QOI thumbnail compression preserves dimensions and vertical image order", "[Thumbnails][ThumbnailsExtended][Compression][QOI]")
{
    ThumbnailData data;
    data.set(2, 2);
    // The encoder intentionally flips rows for firmware conventions. Give each source row
    // a distinct color so the orientation is an observable contract.
    for (size_t i = 0; i < 8; i += 4) {
        data.pixels[i + 0] = 255;
        data.pixels[i + 1] = 0;
        data.pixels[i + 2] = 0;
        data.pixels[i + 3] = 255;
    }
    for (size_t i = 8; i < 16; i += 4) {
        data.pixels[i + 0] = 0;
        data.pixels[i + 1] = 0;
        data.pixels[i + 2] = 255;
        data.pixels[i + 3] = 255;
    }
    auto encoded = GCodeThumbnails::compress_thumbnail(data, GCodeThumbnailsFormat::QOI);
    REQUIRE(encoded != nullptr);
    REQUIRE(encoded->tag() == "thumbnail_QOI");
    qoi_desc desc{};
    std::unique_ptr<unsigned char, decltype(&std::free)> decoded(
        static_cast<unsigned char *>(qoi_decode(encoded->data, int(encoded->size), &desc, 4)), &std::free);
    REQUIRE(decoded != nullptr);
    REQUIRE(desc.width == 2);
    REQUIRE(desc.height == 2);
    std::vector<unsigned char> expected(data.pixels.size());
    for (unsigned int y = 0; y < data.height; ++y)
        std::copy_n(data.pixels.data() + y * data.width * 4, data.width * 4,
                    expected.data() + (data.height - y - 1) * data.width * 4);
    CHECK(std::equal(expected.begin(), expected.end(), decoded.get()));
}

TEST_CASE("BTT thumbnail compression emits RGB565 rows with firmware line endings", "[Thumbnails][ThumbnailsExtended][Compression][BTT]")
{
    ThumbnailData data;
    data.set(2, 2);
    std::fill(data.pixels.begin(), data.pixels.end(), 0);
    for (size_t i = 0; i < data.pixels.size(); i += 4) {
        data.pixels[i + 0] = 255;
        data.pixels[i + 3] = 255;
    }
    auto encoded = GCodeThumbnails::compress_thumbnail(data, GCodeThumbnailsFormat::BTT_TFT);
    REQUIRE(encoded != nullptr);
    REQUIRE(encoded->tag() == "thumbnail_BIQU");
    const std::string rows(static_cast<const char *>(encoded->data), encoded->size);
    CHECK(rows.find(";f800f800\r\n;f800f800\r\n") == 0);
}

TEST_CASE("BTT thumbnail compression premultiplies transparent pixels", "[Thumbnails][ThumbnailsExtended][Compression][BTT]")
{
    ThumbnailData data;
    data.set(2, 1);
    data.pixels = { 255, 0, 0, 0, 255, 0, 0, 255 };
    auto encoded = GCodeThumbnails::compress_thumbnail(data, GCodeThumbnailsFormat::BTT_TFT);
    REQUIRE(encoded != nullptr);
    const std::string row(static_cast<const char *>(encoded->data), encoded->size);
    // Alpha premultiplication maps transparent red to RGB565 black; opaque red
    // remains 0xf800.
    CHECK(row.find(";0000f800\r\n") == 0);
}

TEST_CASE("ColPic thumbnail compression returns its printable encoded payload", "[Thumbnails][ThumbnailsExtended][Compression][ColPic]")
{
    const ThumbnailData data = make_pattern(4, 3);
    auto encoded = GCodeThumbnails::compress_thumbnail(data, GCodeThumbnailsFormat::ColPic);
    REQUIRE(encoded != nullptr);
    REQUIRE(encoded->tag() == "thumbnail_QIDI");
    REQUIRE(encoded->size > 4);
    const auto *bytes = static_cast<const unsigned char *>(encoded->data);
    CHECK(std::any_of(bytes, bytes + encoded->size, [](unsigned char value) { return value != 0; }));
    CHECK(std::all_of(bytes, bytes + encoded->size, [](unsigned char value) {
        return value == 0 || (value >= 48 && value <= 126);
    }));
}

TEST_CASE("Thumbnail export writes distinct blocks and invokes cancellation after each image", "[Thumbnails][ThumbnailsExtended][Export]")
{
    const ThumbnailData data = make_pattern(2, 2);
    ThumbnailsGeneratorCallback callback = [&](const ThumbnailsParams &) {
        return ThumbnailsList{data};
    };
    const auto [list, errors] = parse("2x2/PNG,2x2/QOI,2x2/BTT_TFT");
    REQUIRE(errors == ThumbnailErrors());
    std::string output;
    int callbacks = 0;
    GCodeThumbnails::export_thumbnails_to_file(callback, 7, list,
        [&](const char *text) { output += text; },
        [&]() { ++callbacks; });
    CHECK(callbacks == 3);
    CHECK(output.find("thumbnail begin 2x2") != std::string::npos);
    CHECK(output.find("thumbnail_QOI begin 2x2") != std::string::npos);
    CHECK(output.find(";00020002\r\n") != std::string::npos);
}

TEST_CASE("Thumbnail export skips invalid data and propagates cancellation", "[Thumbnails][ThumbnailsExtended][Export]")
{
    ThumbnailData invalid;
    invalid.set(3, 3);
    invalid.pixels.pop_back();
    ThumbnailsGeneratorCallback callback = [&](const ThumbnailsParams &) {
        return ThumbnailsList{invalid};
    };
    const auto [list, errors] = parse("2x2/PNG");
    REQUIRE(errors == ThumbnailErrors());
    std::string output;
    int callbacks = 0;
    GCodeThumbnails::export_thumbnails_to_file(callback, 0, list,
        [&](const char *text) { output += text; },
        [&]() { ++callbacks; });
    CHECK(output.empty());
    CHECK(callbacks == 0);

    const ThumbnailData valid = make_pattern(2, 2);
    callback = [&](const ThumbnailsParams &) { return ThumbnailsList{valid}; };
    CHECK_THROWS(GCodeThumbnails::export_thumbnails_to_file(callback, 0, list,
        [&](const char *text) { output += text; },
        [&]() { throw std::runtime_error("thumbnail cancellation"); }));
    CHECK(callbacks == 0);
    CHECK(output.find("THUMBNAIL_BLOCK_START") != std::string::npos);
}

} // namespace

} // namespace thumbnail_encoding_contracts

namespace thumbnail_jpeg_contracts {
using namespace Slic3r;

namespace {

struct JpegTestError
{
    jpeg_error_mgr public_error;
    jmp_buf jump;
};

extern "C" void jpeg_test_error_exit(j_common_ptr common)
{
    auto *error = reinterpret_cast<JpegTestError *>(common->err);
    longjmp(error->jump, 1);
}

struct JpegDecodeResult
{
    bool complete = false;
    bool pixels_match = false;
    unsigned width = 0;
    unsigned height = 0;
    unsigned components = 0;
    unsigned rows = 0;
};

struct JpegDecodeState
{
    jpeg_decompress_struct info;
    JpegTestError error;
    bool created;
    JpegDecodeResult result;
};

// Keep this helper C-like after setjmp: a libjpeg error uses longjmp, so no
// non-trivial C++ object is created after the jump point and jpeg_destroy is
// explicit on both error and success paths.
JpegDecodeResult decode_jpeg(const unsigned char *bytes, size_t size,
                             unsigned expected_width, unsigned expected_height)
{
    // libjpeg reports malformed input with longjmp. Keep every state value
    // that is changed after setjmp on the heap so the jump cannot make an
    // automatic C++ value indeterminate. This state is POD and is deleted on
    // both paths below.
    JpegDecodeState *state = new JpegDecodeState{};
    state->info.err = jpeg_std_error(&state->error.public_error);
    state->error.public_error.error_exit = jpeg_test_error_exit;
    if (setjmp(state->error.jump) != 0) {
        if (state->created)
            jpeg_destroy_decompress(&state->info);
        delete state;
        return {};
    }

    jpeg_create_decompress(&state->info);
    state->created = true;
    jpeg_mem_src(&state->info, const_cast<unsigned char *>(bytes), size);
    if (jpeg_read_header(&state->info, TRUE) != JPEG_HEADER_OK)
        goto cleanup;
    state->result.width = state->info.image_width;
    state->result.height = state->info.image_height;
    state->result.components = state->info.num_components;
    if (state->result.width != expected_width || state->result.height != expected_height ||
        state->result.components != 3 || !jpeg_start_decompress(&state->info))
        goto cleanup;
    {
        unsigned char row[4096];
        const size_t row_size = size_t(state->info.output_width) * state->info.output_components;
        if (row_size > sizeof(row))
            goto cleanup;
        state->result.pixels_match = true;
        while (state->info.output_scanline < state->info.output_height) {
            JSAMPROW row_ptr = row;
            if (jpeg_read_scanlines(&state->info, &row_ptr, 1) != 1) {
                state->result.pixels_match = false;
                goto cleanup;
            }
            for (size_t x = 0; x < state->info.output_width; ++x) {
                state->result.pixels_match = state->result.pixels_match &&
                    std::abs(int(row[3 * x + 0]) - 40) <= 12 &&
                    std::abs(int(row[3 * x + 1]) - 120) <= 12 &&
                    std::abs(int(row[3 * x + 2]) - 220) <= 12;
            }
            ++state->result.rows;
        }
    }
    state->result.complete = jpeg_finish_decompress(&state->info) != 0;

cleanup:
    jpeg_destroy_decompress(&state->info);
    JpegDecodeResult result = state->result;
    delete state;
    return result;
}

} // namespace

TEST_CASE("JPEG thumbnail output has a complete decodable stream", "[Thumbnails][ThumbnailJpegRegression]")
{
    for (const auto [width, height] : { std::pair<unsigned, unsigned>{ 4, 3 },
                                        std::pair<unsigned, unsigned>{ 64, 48 } }) {
        ThumbnailData data;
        data.set(width, height);
        for (size_t i = 0; i < data.pixels.size(); i += 4) {
            data.pixels[i + 0] = 40;
            data.pixels[i + 1] = 120;
            data.pixels[i + 2] = 220;
            data.pixels[i + 3] = 255;
        }

        auto encoded = GCodeThumbnails::compress_thumbnail(data, GCodeThumbnailsFormat::JPG);
        REQUIRE(encoded != nullptr);
        REQUIRE(encoded->tag() == "thumbnail_JPG");
        REQUIRE(encoded->data != nullptr);
        REQUIRE(encoded->size > 4);
        const auto *bytes = static_cast<const unsigned char *>(encoded->data);
        REQUIRE(bytes[0] == 0xff);
        REQUIRE(bytes[1] == 0xd8);
        REQUIRE(bytes[encoded->size - 2] == 0xff);
        REQUIRE(bytes[encoded->size - 1] == 0xd9);

        const JpegDecodeResult decoded = decode_jpeg(bytes, encoded->size, width, height);
        CHECK(decoded.width == width);
        CHECK(decoded.height == height);
        CHECK(decoded.components == 3);
        CHECK(decoded.rows == height);
        CHECK(decoded.pixels_match);
        CHECK(decoded.complete);
    }
}

TEST_CASE("Configured thumbnail format reaches encoded G-code export",
          "[Thumbnails][ThumbnailFormatSetting][ThumbnailGCodeExport]")
{
    const std::vector<std::string> expected_names = {"PNG", "JPG", "QOI", "BTT_TFT", "COLPIC"};
    const auto &registered_names = ConfigOptionEnum<GCodeThumbnailsFormat>::get_enum_names();
    REQUIRE(registered_names == expected_names);

    for (size_t index = 0; index < registered_names.size(); ++index) {
        INFO("selected thumbnail format: " << registered_names[index]);
        const auto format = static_cast<GCodeThumbnailsFormat>(index);
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.set_deserialize_strict({
            {"thumbnails", "4x3/"},
            {"thumbnails_format", registered_names[index].c_str()},
        });
        config.handle_legacy_composite();

        CHECK(config.opt_string("thumbnails") == "4x3/" + registered_names[index]);
        const auto [definitions, errors] = GCodeThumbnails::make_and_check_thumbnail_list(config);
        REQUIRE(errors == ThumbnailErrors());
        REQUIRE(definitions.size() == 1);
        REQUIRE(definitions.front().first == format);
        CHECK(definitions.front().second == Vec2d(4, 3));

        ThumbnailData image;
        image.set(4, 3);
        for (size_t pixel = 0; pixel < image.pixels.size(); pixel += 4) {
            image.pixels[pixel + 0] = 40;
            image.pixels[pixel + 1] = 120;
            image.pixels[pixel + 2] = 220;
            image.pixels[pixel + 3] = 255;
        }
        if (format == GCodeThumbnailsFormat::BTT_TFT) {
            // RGB565 0x0020 is one of the firmware colors that must be emitted as black.
            for (unsigned int y = 0; y < image.height; ++y) {
                const size_t pixel = 4 * (size_t(y) * image.width);
                image.pixels[pixel + 0] = 0;
                image.pixels[pixel + 1] = 4;
                image.pixels[pixel + 2] = 0;
            }
        } else if (format == GCodeThumbnailsFormat::ColPic) {
            // The ColPic format substitutes a documented background color for transparent pixels.
            const size_t pixel = 4 * (size_t(image.height - 1) * image.width);
            image.pixels[pixel + 0] = 255;
            image.pixels[pixel + 1] = 0;
            image.pixels[pixel + 2] = 0;
            image.pixels[pixel + 3] = 0;
        }

        ThumbnailsGeneratorCallback callback = [image](const ThumbnailsParams &) {
            return ThumbnailsList{image};
        };
        std::string output;
        GCodeThumbnails::export_thumbnails_to_file(callback, 7, definitions,
            [&](const char *text) { output += text; }, []() {});

        if (format == GCodeThumbnailsFormat::BTT_TFT) {
            std::string expected = ";00040003\r\n";
            for (int row = 0; row < 3; ++row)
                expected += ";00002bdb2bdb2bdb\r\n";
            expected += "; bigtree thumbnail end\r\n\r\n";
            CHECK(output == expected);
            continue;
        }

        if (format == GCodeThumbnailsFormat::ColPic) {
            const size_t marker = output.find(";gimage:");
            REQUIRE(marker != std::string::npos);
            const size_t payload_begin = marker + std::string(";gimage:").size();
            const size_t payload_end = output.find('\n', payload_begin);
            REQUIRE(payload_end != std::string::npos);
            const std::string text = output.substr(payload_begin, payload_end - payload_begin);
            REQUIRE_FALSE(text.empty());
            REQUIRE(text.size() % 4 == 0);

            std::vector<unsigned char> encoded;
            for (size_t i = 0; i < text.size(); i += 4) {
                int values[4] = {};
                for (size_t j = 0; j < 4; ++j) {
                    const unsigned char ch = static_cast<unsigned char>(text[i + j]);
                    values[j] = ch == '~' ? 44 : int(ch) - 48;
                    REQUIRE(values[j] >= 0);
                    REQUIRE(values[j] < 64);
                }
                encoded.push_back(static_cast<unsigned char>((values[0] << 2) | (values[1] >> 4)));
                encoded.push_back(static_cast<unsigned char>((values[1] << 4) | (values[2] >> 2)));
                encoded.push_back(static_cast<unsigned char>((values[2] << 6) | values[3]));
            }
            REQUIRE(encoded.size() >= 32);
            const auto read_le16 = [&](size_t offset) {
                return std::uint16_t(encoded[offset + 0]) |
                       (std::uint16_t(encoded[offset + 1]) << 8);
            };
            const auto read_le32 = [&](size_t offset) {
                return std::uint32_t(encoded[offset + 0]) |
                       (std::uint32_t(encoded[offset + 1]) << 8) |
                       (std::uint32_t(encoded[offset + 2]) << 16) |
                       (std::uint32_t(encoded[offset + 3]) << 24);
            };
            CHECK(encoded[0] == 3);                 // ColPic stream version.
            CHECK(read_le32(4) == 4);               // Encoded width.
            CHECK(read_le32(8) == 3);               // Encoded height.
            CHECK(read_le32(12) == 0x05DDC33C);     // ColPic header marker.

            const size_t palette_bytes = read_le32(16);
            const size_t color_data_bytes = read_le32(20);
            REQUIRE(palette_bytes > 0);
            REQUIRE(palette_bytes % 2 == 0);
            REQUIRE(palette_bytes <= encoded.size() - 32);
            const size_t color_data_begin = 32 + palette_bytes;
            REQUIRE(color_data_bytes <= encoded.size() - color_data_begin);
            const size_t color_data_end = color_data_begin + color_data_bytes;
            const size_t palette_size = palette_bytes / 2;
            std::vector<std::uint16_t> palette;
            palette.reserve(palette_size);
            for (size_t i = 0; i < palette_size; ++i)
                palette.push_back(read_le16(32 + 2 * i));

            // ColPic color data stores a five-bit palette slot and three-bit run length.
            // A zero run code takes its length from the following byte; code seven changes palette bank.
            std::vector<std::uint16_t> decoded_pixels;
            size_t cursor = color_data_begin;
            size_t palette_bank = 0;
            while (cursor < color_data_end) {
                const unsigned char token = encoded[cursor++];
                const size_t code = token >> 5;
                const size_t slot = token & 0x1f;
                if (code == 7) {
                    palette_bank = slot;
                    continue;
                }
                const size_t run = code == 0 ? (cursor < color_data_end ? encoded[cursor++] : 0) : code;
                const size_t palette_index = palette_bank * 32 + slot;
                REQUIRE(run > 0);
                REQUIRE(palette_index < palette.size());
                REQUIRE(decoded_pixels.size() + run <= 12);
                decoded_pixels.insert(decoded_pixels.end(), run, palette[palette_index]);
            }
            REQUIRE(decoded_pixels.size() == 12);
            CHECK(decoded_pixels.front() == 0x2989); // Transparent-pixel fallback RGB565.
            CHECK(std::all_of(decoded_pixels.begin() + 1, decoded_pixels.end(),
                              [](std::uint16_t pixel) { return pixel == 0x2bdb; }));
            continue;
        }

        const char *tag = format == GCodeThumbnailsFormat::PNG ? "thumbnail" :
                          format == GCodeThumbnailsFormat::JPG ? "thumbnail_JPG" : "thumbnail_QOI";
        const std::string begin = std::string("; ") + tag + " begin 4x3 ";
        const size_t header = output.find(begin);
        REQUIRE(header != std::string::npos);
        const size_t header_end = output.find('\n', header);
        REQUIRE(header_end != std::string::npos);
        size_t declared_size = 0;
        const char *size_begin = output.data() + header + begin.size();
        const char *size_end = output.data() + header_end;
        const auto parsed_size = std::from_chars(size_begin, size_end, declared_size);
        REQUIRE(parsed_size.ec == std::errc{});
        REQUIRE(parsed_size.ptr == size_end);
        const size_t block_end = output.find(std::string("; ") + tag + " end", header_end);
        REQUIRE(block_end != std::string::npos);
        std::string base64;
        for (size_t pos = header_end + 1; pos < block_end;) {
            size_t line_end = output.find('\n', pos);
            if (line_end == std::string::npos || line_end > block_end)
                line_end = block_end;
            if (line_end - pos >= 2 && output[pos] == ';' && output[pos + 1] == ' ')
                base64.append(output, pos + 2, line_end - pos - 2);
            pos = line_end + 1;
        }
        REQUIRE_FALSE(base64.empty());
        REQUIRE(base64.size() % 4 == 0);
        const size_t padding = base64.back() == '=' ?
            (base64.size() >= 2 && base64[base64.size() - 2] == '=' ? 2 : 1) : 0;
        std::vector<unsigned char> payload(boost::beast::detail::base64::decoded_size(base64.size()));
        const auto [payload_size, consumed] = boost::beast::detail::base64::decode(
            payload.data(), base64.data(), base64.size());
        REQUIRE(consumed == base64.size() - padding);
        REQUIRE(payload_size == (base64.size() / 4) * 3 - padding);
        REQUIRE(declared_size == base64.size());
        payload.resize(payload_size);

        if (format == GCodeThumbnailsFormat::PNG) {
            png::ImageColorscale decoded;
            REQUIRE(png::decode_colored_png(png::ReadBuf{payload.data(), payload.size()}, decoded));
            CHECK(decoded.cols == 4);
            CHECK(decoded.rows == 3);
            CHECK(decoded.bytes_per_pixel == 4);
            CHECK(decoded.buf == image.pixels);
        } else if (format == GCodeThumbnailsFormat::JPG) {
            const JpegDecodeResult decoded = decode_jpeg(payload.data(), payload.size(), 4, 3);
            CHECK(decoded.width == 4);
            CHECK(decoded.height == 3);
            CHECK(decoded.rows == 3);
            CHECK(decoded.pixels_match);
            CHECK(decoded.complete);
        } else {
            qoi_desc desc{};
            std::unique_ptr<unsigned char, decltype(&std::free)> decoded(
                static_cast<unsigned char *>(qoi_decode(payload.data(), int(payload.size()), &desc, 4)), &std::free);
            REQUIRE(decoded != nullptr);
            REQUIRE(desc.width == 4);
            REQUIRE(desc.height == 3);
            CHECK(std::equal(image.pixels.begin(), image.pixels.end(), decoded.get()));
        }
    }
}

} // namespace thumbnail_jpeg_contracts
