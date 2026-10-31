#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <ios>

#include "libslic3r/Exception.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/Format/ZipperArchiveImport.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Zipper.hpp"

#include "test_utils.hpp"

#include <boost/property_tree/ptree.hpp>

#include <cstdint>
#include <fstream>
#include <initializer_list>
#include <string>
#include <utility>
#include <vector>

using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace zipper_archive_import_contracts {

namespace {

void write_archive(const std::string &path,
                   std::initializer_list<std::pair<std::string, std::string>> entries)
{
    Zipper zipper(path, Zipper::NO_COMPRESSION);
    for (const auto &entry : entries) {
        zipper.add_entry(entry.first);
        zipper << entry.second;
    }
    zipper.finalize();
}

DynamicPrintConfig fallback_profile(double layer_height, double initial_layer_height)
{
    DynamicPrintConfig profile;
    profile.set_key_value("layer_height", new ConfigOptionFloat(layer_height));
    profile.set_key_value("initial_layer_height", new ConfigOptionFloat(initial_layer_height));
    return profile;
}

} // namespace

TEST_CASE("Zipper archive import filters and orders entries while parsing metadata",
          "[ZipperArchiveImport]")
{
    ScopedTemporaryFile archive(".zip");
    write_archive(archive.string(), {
        {"config.ini", "layerHeight = 0.25\n"},
        {"prusaslicer.ini", "layer_height = 0.2\ninitial_layer_height = 0.3\n"},
        {"Models/secret.bin", "secret"},
        {"models/A.bin", "alpha"},
    });

    const ZipperArchive all_models = read_zipper_archive(archive.string(), {"models"}, {});
    CHECK(all_models.config.empty());
    CHECK(all_models.profile.empty());
    REQUIRE(all_models.entries.size() == 2);
    CHECK(all_models.entries[0].fname == "models/a.bin");
    CHECK(all_models.entries[1].fname == "models/secret.bin");
    const std::vector<uint8_t> alpha{'a', 'l', 'p', 'h', 'a'};
    const std::vector<uint8_t> secret{'s', 'e', 'c', 'r', 'e', 't'};
    CHECK(all_models.entries[0].buf == alpha);
    CHECK(all_models.entries[1].buf == secret);

    const ZipperArchive filtered = read_zipper_archive(archive.string(), {"models"}, {"secret"});
    REQUIRE(filtered.entries.size() == 1);
    CHECK(filtered.entries[0].fname == "models/a.bin");

    const ZipperArchive metadata = read_zipper_archive(archive.string(), {"ini"}, {});
    CHECK(metadata.entries.empty());
    CHECK(metadata.config.get<std::string>("layerHeight") == "0.25");
    CHECK(metadata.profile.get<std::string>("layer_height") == "0.2");
    CHECK(metadata.profile.get<std::string>("initial_layer_height") == "0.3");

    const ZipperArchive no_match = read_zipper_archive(archive.string(), {"missing"}, {});
    CHECK(no_match.entries.empty());
    CHECK(no_match.config.empty());
    CHECK(no_match.profile.empty());
}

TEST_CASE("Zipper archive import rejects a missing archive", "[ZipperArchiveImport]")
{
    ScopedTemporaryFile missing(".zip");
    REQUIRE_THROWS_AS(read_zipper_archive(missing.string(), {"models"}, {}), FileIOError);
}

TEST_CASE("Zipper archive import rejects malformed archive bytes", "[ZipperArchiveImport]")
{
    ScopedTemporaryFile malformed(".zip");
    std::ofstream out(malformed.string(), std::ios::binary);
    REQUIRE(out.good());
    out << "not a ZIP archive";
    out.close();

    REQUIRE_THROWS_AS(read_zipper_archive(malformed.string(), {"models"}, {}), FileIOError);
}

TEST_CASE("Zipper archive import uses a complete profile and replaces the output profile",
          "[ZipperArchiveImport]")
{
    ScopedTemporaryFile archive(".zip");
    write_archive(archive.string(), {
        {"prusaslicer.ini", "layer_height = 0.2\ninitial_layer_height = 0.3\n"},
    });
    const ZipperArchive imported = read_zipper_archive(archive.string(), {"ini"}, {});

    DynamicPrintConfig profile_out = fallback_profile(0.4, 0.45);
    auto extracted = extract_profile(imported, profile_out);

    CHECK(extracted.second.empty());
    CHECK_FALSE(extracted.first.empty());
    CHECK_FALSE(profile_out.empty());
    CHECK_THAT(extracted.first.opt_float("layer_height"), WithinAbs(0.2, 1e-12));
    CHECK_THAT(extracted.first.opt_float("initial_layer_height"), WithinAbs(0.3, 1e-12));
    CHECK_THAT(profile_out.opt_float("layer_height"), WithinAbs(0.2, 1e-12));
    CHECK_THAT(profile_out.opt_float("initial_layer_height"), WithinAbs(0.3, 1e-12));
}

TEST_CASE("Zipper archive import recovers layer height when the profile is absent",
          "[ZipperArchiveImport]")
{
    ScopedTemporaryFile archive(".zip");
    write_archive(archive.string(), {
        {"config.ini", "jobDir = Fixture\nlayerHeight = 0.25\n"},
    });
    const ZipperArchive imported = read_zipper_archive(archive.string(), {"ini"}, {});

    DynamicPrintConfig profile_out = fallback_profile(0.4, 0.45);
    auto extracted = extract_profile(imported, profile_out);

    CHECK(extracted.second.empty());
    CHECK_FALSE(extracted.first.empty());
    CHECK(extracted.first.has("layer_height"));
    CHECK(extracted.first.has("initial_layer_height"));
    CHECK_THAT(extracted.first.opt_float("layer_height"), WithinAbs(0.25, 1e-12));
    CHECK_THAT(extracted.first.opt_float("initial_layer_height"), WithinAbs(0.25, 1e-12));
    CHECK(profile_out.empty());
}

TEST_CASE("Zipper archive import preserves fallback when layer height metadata is invalid",
          "[ZipperArchiveImport]")
{
    ScopedTemporaryFile archive(".zip");
    write_archive(archive.string(), {
        {"config.ini", "layerHeight = invalid\n"},
    });
    const ZipperArchive imported = read_zipper_archive(archive.string(), {"ini"}, {});

    DynamicPrintConfig profile_out = fallback_profile(0.4, 0.45);
    auto extracted = extract_profile(imported, profile_out);

    CHECK(extracted.second.empty());
    CHECK_THAT(extracted.first.opt_float("layer_height"), WithinAbs(0.4, 1e-12));
    CHECK_THAT(extracted.first.opt_float("initial_layer_height"), WithinAbs(0.45, 1e-12));
    CHECK(profile_out.empty());
}

TEST_CASE("Zipper archive import preserves fallback when layer height metadata is absent",
          "[ZipperArchiveImport]")
{
    ScopedTemporaryFile archive(".zip");
    write_archive(archive.string(), {
        {"payload.bin", "payload"},
    });
    const ZipperArchive imported = read_zipper_archive(archive.string(), {"payload"}, {});

    DynamicPrintConfig profile_out = fallback_profile(0.4, 0.45);
    auto extracted = extract_profile(imported, profile_out);

    CHECK(extracted.second.empty());
    CHECK_THAT(extracted.first.opt_float("layer_height"), WithinAbs(0.4, 1e-12));
    CHECK_THAT(extracted.first.opt_float("initial_layer_height"), WithinAbs(0.45, 1e-12));
    CHECK(profile_out.empty());
}

} // namespace zipper_archive_import_contracts
