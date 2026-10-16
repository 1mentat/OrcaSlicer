#include <vector>
#include <set>
#include <array>
#include "libslic3r/Exception.hpp"
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/directory.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/file_status.hpp>
#include <catch2/catch_all.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/catch_message.hpp>
#include "libslic3r/Utils.hpp"

#include "test_utils.hpp"

#include <boost/filesystem.hpp>

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <fstream>
#include <ios>
#include <iterator>
#include <string>
#include <thread>
#include <system_error>
#include <utility>

#ifndef _WIN32
#include <unistd.h>     // getuid
#endif

using namespace Slic3r;

TEST_CASE("per_user_temp_dir composes a per-user temp root", "[utils]") {
    const std::string base = "/tmp";

    SECTION("an empty id returns base unchanged") {
        REQUIRE(per_user_temp_dir(base, "") == base);
    }
    SECTION("a non-empty id is appended at the top level") {
        REQUIRE(per_user_temp_dir(base, "1000") == base + "/orcaslicer_1000");
    }
    SECTION("distinct ids produce distinct roots") {
        REQUIRE(per_user_temp_dir(base, "1000") != per_user_temp_dir(base, "1001"));
    }
}

TEST_CASE("per_user_temp_id follows the platform contract", "[utils]") {
    const std::string id = per_user_temp_id();

    SECTION("stable across calls") {
        REQUIRE(per_user_temp_id() == id);
    }
#ifdef _WIN32
    SECTION("empty on Windows (its temp dir is already per-user)") {
        REQUIRE(id.empty());
    }
#else
    SECTION("the current uid on Linux/macOS") {
        REQUIRE_FALSE(id.empty());
        REQUIRE(id == std::to_string(static_cast<unsigned long>(::getuid())));
    }
#endif
}

// The end-to-end contract callers depend on: the temp root is left alone on
// Windows and isolated per user on Linux/macOS.
TEST_CASE("per-user temp root is unchanged on Windows, isolated elsewhere", "[utils]") {
    const std::string base = "/tmp";
    const std::string root = per_user_temp_dir(base, per_user_temp_id());
#ifdef _WIN32
    REQUIRE(root == base);
#else
    REQUIRE(root != base);
    REQUIRE_THAT(root, Catch::Matchers::StartsWith(base + "/orcaslicer_"));
#endif
}

TEST_CASE("write_file_atomically replaces the target and leaves no temporary file", "[utils]") {
    ScopedTemporaryDir dir;
    const boost::filesystem::path target = dir.path() / "preset.json";

    REQUIRE_FALSE(write_file_atomically(target.string(), "first"));
    REQUIRE_FALSE(write_file_atomically(target.string(), "second"));

    std::string content;
    load_string_file(target, content);
    REQUIRE(content == "second");
    size_t entries = 0;
    for (auto &entry : boost::filesystem::directory_iterator(dir.path())) {
        (void) entry;
        ++entries;
    }
    REQUIRE(entries == 1);
}

TEST_CASE("write_file_atomically reports a missing directory and writes nothing", "[utils]") {
    ScopedTemporaryDir dir;
    const boost::filesystem::path target = dir.path() / "missing" / "preset.json";

    const std::error_code ec = write_file_atomically(target.string(), "x");
    REQUIRE(ec == std::errc::no_such_file_or_directory);
    REQUIRE_FALSE(boost::filesystem::exists(target));
}

#ifndef _WIN32
// The read-only bit on a directory stops file creation only on POSIX.
TEST_CASE("write_file_atomically writes in place when no temporary can be created beside an existing target", "[utils]") {
    if (::geteuid() == 0)
        SKIP("a read-only directory does not stop root");
    ScopedTemporaryDir dir;
    const boost::filesystem::path target = dir.path() / "preset.json";
    REQUIRE_FALSE(write_file_atomically(target.string(), "first"));
    boost::filesystem::permissions(dir.path(), boost::filesystem::owner_read | boost::filesystem::owner_exe);

    const std::error_code replaced = write_file_atomically(target.string(), "second");
    const std::error_code created  = write_file_atomically((dir.path() / "new.json").string(), "x");
    // Restored before any assertion, so a failure never leaves an unremovable directory behind.
    boost::filesystem::permissions(dir.path(), boost::filesystem::owner_all);

    REQUIRE_FALSE(replaced);
    REQUIRE(created == std::errc::permission_denied);
    std::string content;
    load_string_file(target, content);
    REQUIRE(content == "second");
}
#endif

TEST_CASE("write_file_atomically keeps bytes intact in binary mode", "[utils]") {
    ScopedTemporaryDir dir;
    const boost::filesystem::path target = dir.path() / "blob.bin";
    const std::string bytes("a\r\nb\0c", 6);

    REQUIRE_FALSE(write_file_atomically(target.string(), bytes, /*binary=*/true));
    REQUIRE(boost::filesystem::file_size(target) == bytes.size());
}

#ifndef _WIN32
TEST_CASE("write_file_atomically writes through a symlink and keeps the target's permissions", "[utils]") {
    ScopedTemporaryDir dir;
    const boost::filesystem::path real = dir.path() / "real.json";
    const boost::filesystem::path link = dir.path() / "link.json";
    REQUIRE_FALSE(write_file_atomically(real.string(), "first"));
    boost::filesystem::permissions(real, boost::filesystem::owner_read | boost::filesystem::owner_write);
    boost::filesystem::create_symlink(real, link);

    REQUIRE_FALSE(write_file_atomically(link.string(), "second"));

    REQUIRE(boost::filesystem::is_symlink(boost::filesystem::symlink_status(link)));
    std::string content;
    load_string_file(real, content);
    REQUIRE(content == "second");

    REQUIRE_FALSE(write_file_atomically(real.string(), "third"));
    const auto perms = boost::filesystem::status(real).permissions() & boost::filesystem::all_all;
    REQUIRE(perms == (boost::filesystem::owner_read | boost::filesystem::owner_write));
}
#endif

TEST_CASE("write_file_atomically survives two threads writing one target", "[utils][Parallel]") {
    ScopedTemporaryDir dir;
    const boost::filesystem::path target = dir.path() / "shared.json";
    const std::string a(20000, 'a'), b(20000, 'b');

    std::thread other([&] {
        for (int i = 0; i < 50; ++i)
            write_file_atomically(target.string(), a);
    });
    for (int i = 0; i < 50; ++i)
        write_file_atomically(target.string(), b);
    other.join();

    std::string content;
    load_string_file(target, content);
    const bool whole = content == a || content == b;
    REQUIRE(whole);
    // No temporary may be left; a scanner on Windows may briefly hold the old
    // file under another name, so only the temporaries are counted.
    size_t temporaries = 0;
    for (auto &entry : boost::filesystem::directory_iterator(dir.path()))
        if (entry.path().extension() == ".tmp")
            ++temporaries;
    REQUIRE(temporaries == 0);
}

TEST_CASE("copy_file reports the OS error when the destination cannot be written", "[utils]") {
    ScopedTemporaryFile source(".txt");
    {
        std::ofstream ofs(source.string(), std::ios::binary);
        ofs << "orca";
    }
    REQUIRE(boost::filesystem::exists(source.path()));

    // A directory that was never created, so the copy fails on every platform.
    const boost::filesystem::path destination = source.path().parent_path() / "orca-missing-dir" / "copy.txt";
    REQUIRE_FALSE(boost::filesystem::exists(destination.parent_path()));

    std::string error_message;
    REQUIRE(copy_file(source.string(), destination.string(), error_message) == FAIL_COPY_FILE);
    REQUIRE_FALSE(error_message.empty());

#ifdef _WIN32
    // The Windows branch formats GetLastError() itself. Writing that as
    // "Error: " + errCode adds an integer to a string literal, which indexes into the
    // literal instead of appending and runs off its end for any code above 7.
    const std::string prefix = "Error: ";
    REQUIRE(error_message.rfind(prefix, 0) == 0);

    const std::string code = error_message.substr(prefix.size());
    REQUIRE_FALSE(code.empty());
    REQUIRE(std::all_of(code.begin(), code.end(), [](unsigned char c) { return std::isdigit(c) != 0; }));
#endif // _WIN32
}

TEST_CASE("A resolved input path still names the same file after the working directory changes", "[utils]") {
    ScopedTemporaryFile model(".3mf");
    { std::ofstream out(model.string()); out << "3mf"; }
    const std::string name = model.path().filename().string();

    // Resolve the bare name from the directory holding the file, then move away from it. The guard
    // restores the directory the test started in, wherever this leaves it.
    ScopedWorkingDirectory cwd(model.path().parent_path());
    const std::string resolved = resolve_cli_input_path(name);
    boost::filesystem::current_path(boost::filesystem::path(TEST_DATA_DIR));

    REQUIRE(boost::filesystem::exists(resolved));
    REQUIRE(boost::filesystem::equivalent(resolved, model.path()));
    // Control: the bare name finds nothing from here, so resolving it this late would have failed.
    REQUIRE_FALSE(boost::filesystem::exists(name));
}

TEST_CASE("resolve_cli_input_path completes a relative path against the working directory", "[utils]") {
    ScopedWorkingDirectory cwd(boost::filesystem::temp_directory_path());
    // Read back rather than reusing temp_directory_path(): changing to it resolves any symlink.
    const boost::filesystem::path here = boost::filesystem::current_path();

    SECTION("a bare name") {
        REQUIRE(resolve_cli_input_path("model.3mf") == (here / "model.3mf").make_preferred().string());
    }
    SECTION("a ./ prefix is dropped") {
        REQUIRE(resolve_cli_input_path("./model.3mf") == (here / "model.3mf").make_preferred().string());
    }
    SECTION("a ../ traversal is collapsed") {
        REQUIRE(resolve_cli_input_path("../model.3mf") == (here.parent_path() / "model.3mf").make_preferred().string());
    }
}

TEST_CASE("resolve_cli_input_path leaves inputs that must not be completed unchanged", "[utils]") {
    SECTION("an absolute path") {
        const boost::filesystem::path absolute = (boost::filesystem::temp_directory_path() / "model.3mf").make_preferred();
        REQUIRE(resolve_cli_input_path(absolute.string()) == absolute.string());
    }
#ifdef _WIN32
    // Every absolute form Windows accepts opens today, so each must come back byte for byte:
    // normalizing them would rewrite the forward slashes and rebuild the \\?\ and UNC prefixes.
    SECTION("an absolute Windows path of any form") {
        for (const std::string absolute : {R"(C:\models\model.3mf)",
                                           R"(C:/models/model.3mf)",
                                           R"(\\server\share\model.3mf)",
                                           R"(\\?\C:\models\model.3mf)"})
            REQUIRE(resolve_cli_input_path(absolute) == absolute);
    }
#endif
    // These are downloaded rather than opened, and completing one would produce a path, not a URL.
    SECTION("a custom open protocol URL") {
        for (const std::string url : {"orcaslicer://open/?file=https://example.com/model.3mf",
                                      "prusaslicer://open/?file=https://example.com/model.3mf",
                                      "bambustudio://open/?file=https://example.com/model.3mf",
                                      "cura://open/?file=https://example.com/model.3mf"})
            REQUIRE(resolve_cli_input_path(url) == url);
    }
    SECTION("an empty argument") {
        REQUIRE(resolve_cli_input_path("").empty());
    }
}

TEST_CASE("sanitize_file_basename keeps only a plain file name from an untrusted name", "[Utils]") {
    const std::string unicode = "\xe6\xa8\xa1\xe5\x9e\x8b \xc3\xa9t\xc3\xa9.3mf"; // UTF-8 CJK and accented Latin
    const auto [input, expected] = GENERATE_COPY(table<std::string, std::string>({
        {"normal.3mf", "normal.3mf"},
        {"../../x.3mf", "x.3mf"},
        {"..\\..\\x.3mf", "x.3mf"},
        {"C:\\x.3mf", "x.3mf"},
        {"C:x.3mf", "C_x.3mf"},
        {"/etc/x", "x"},
        {"a/b\\c.gcode", "c.gcode"},
        {"x:stream", "x_stream"}, // no NTFS alternate data stream
        {"x.", "x."},
        {".3mf", ".3mf"},
        {unicode, unicode},
    }));
    CAPTURE(input);
    CHECK(sanitize_file_basename(input) == expected);
}

TEST_CASE("sanitize_file_basename rejects names that do not name a file", "[Utils]") {
    const std::string input = GENERATE(as<std::string>{}, "", ".", "..", "../..", "dir/", "..\\", " ", ". .", "...");
    CAPTURE(input);
    CHECK(sanitize_file_basename(input).empty());
}

namespace {
void touch(const boost::filesystem::path &path) { std::ofstream(path.string()) << "existing"; }
std::string file_contents(const boost::filesystem::path &path)
{
    std::ifstream file(path.string());
    return std::string(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
}
} // namespace

TEST_CASE("find_unused_filename keeps a name nothing uses", "[Utils]") {
    ScopedTemporaryDir dir;
    std::string name;
    REQUIRE(find_unused_filename(dir.path(), "model.3mf", {}, name));
    CHECK(name == "model.3mf");
}

TEST_CASE("find_unused_filename versions a name an existing file uses", "[Utils]") {
    ScopedTemporaryDir dir;
    touch(dir.path() / "model.3mf");
    std::string name;
    REQUIRE(find_unused_filename(dir.path(), "model.3mf", {}, name));
    CHECK(name == "model(1).3mf");
}

TEST_CASE("find_unused_filename versions a name that maps onto an existing file once sanitized", "[Utils]") {
    ScopedTemporaryDir dir;
    touch(dir.path() / "my_model.3mf");
    const std::string input = GENERATE(as<std::string>{}, "my?model.3mf", "my:model.3mf", "my*model.3mf");
    CAPTURE(input);
    std::string name;
    REQUIRE(find_unused_filename(dir.path(), input, {}, name));
    CHECK(name == "my_model(1).3mf");
    CHECK(file_contents(dir.path() / "my_model.3mf") == "existing");
}

TEST_CASE("find_unused_filename treats the marker of another download as used", "[Utils]") {
    ScopedTemporaryDir dir;
    touch(download_marker_path(dir.path(), "model.3mf"));
    std::string name;
    REQUIRE(find_unused_filename(dir.path(), "model.3mf", {}, name));
    CHECK(name == "model(1).3mf");
}

TEST_CASE("find_unused_filename ignores the marker of the download asking", "[Utils]") {
    ScopedTemporaryDir dir;
    const boost::filesystem::path own_marker = download_marker_path(dir.path(), "model.3mf");
    touch(own_marker);
    std::string name;
    REQUIRE(find_unused_filename(dir.path(), "model.3mf", own_marker, name));
    CHECK(name == "model.3mf");
}

TEST_CASE("find_unused_filename gives up after 999 versions", "[Utils]") {
    ScopedTemporaryDir dir;
    touch(dir.path() / "model.3mf");
    for (int version = 1; version < 999; ++version)
        touch(dir.path() / ("model(" + std::to_string(version) + ").3mf"));
    std::string name;
    REQUIRE(find_unused_filename(dir.path(), "model.3mf", {}, name));
    CHECK(name == "model(999).3mf");

    touch(dir.path() / name);
    REQUIRE_FALSE(find_unused_filename(dir.path(), "model.3mf", {}, name));
    CHECK(name == "model(999).3mf");
}

TEST_CASE("is_path_within_root accepts a root given with a trailing separator", "[utils]") {
    ScopedTemporaryDir tmp;
    const std::string root = tmp.path().string();
    const std::string with_separator = GENERATE_COPY(root + "/", root + std::string(1, static_cast<char>(boost::filesystem::path::preferred_separator)));

    CAPTURE(with_separator);
    CHECK(is_path_within_root("vendor.json", with_separator));
    CHECK(is_path_within_root("vendor/machine/printer.json", with_separator));
    CHECK_FALSE(is_path_within_root("../vendor.json", with_separator));
}

TEST_CASE("is_path_within_root treats Windows-specific name forms the same on every platform", "[utils]") {
    ScopedTemporaryDir tmp;

    SECTION("names ending in dots or spaces stay inside the root") {
        const std::string name = GENERATE(std::string("name."), std::string("name "), std::string("dir./file.json"), std::string("dir /file.json"));
        CAPTURE(name);
        CHECK(is_path_within_root(name, tmp.path()));
    }
    SECTION("drive-relative names are rejected") {
        const std::string name = GENERATE(std::string("C:x"), std::string("c:x/y.json"), std::string("C:"));
        CAPTURE(name);
        CHECK_FALSE(is_path_within_root(name, tmp.path()));
    }
}

TEST_CASE("is_path_within_root rejects a name with an embedded NUL", "[utils]") {
    ScopedTemporaryDir tmp;
    // The filesystem calls stop at the NUL, so they would act on a different path than the one checked.
    const std::string name = GENERATE(std::string("..\0", 3), std::string("..\0x/file.json", 14), std::string("sub/..\0x", 8),
                                      std::string("file.json\0", 10), std::string("\0file.json", 10));
    CAPTURE(name.size());
    CHECK_FALSE(is_path_within_root(name, tmp.path()));
}

TEST_CASE("is_symlink_target_within_root accepts relative targets that stay inside the root", "[utils]") {
    ScopedTemporaryDir tmp;
    const auto [link, target] = GENERATE(std::make_pair(std::string("Versions/Current"), std::string("A")),
                                         std::make_pair(std::string("Foo.framework/Foo"), std::string("Versions/Current/Foo")),
                                         std::make_pair(std::string("libfoo.so"), std::string("libfoo.so.1")),
                                         std::make_pair(std::string("a/b/link"), std::string("c/d")));
    CAPTURE(link, target);
    CHECK(is_symlink_target_within_root(link, target, tmp.path()));
}

TEST_CASE("is_symlink_target_within_root rejects absolute targets and targets that climb out", "[utils]") {
    ScopedTemporaryDir tmp;
    const std::string outside = (tmp.path().parent_path() / "outside").generic_string();
    const auto [link, target] = GENERATE_COPY(std::make_pair(std::string("sub/link"), outside),
                                              std::make_pair(std::string("sub/link"), std::string("/etc/passwd")),
                                              std::make_pair(std::string("sub/link"), std::string("\\outside")),
                                              std::make_pair(std::string("sub/link"), std::string("C:/outside")),
                                              std::make_pair(std::string("sub/link"), std::string("C:outside")),
                                              std::make_pair(std::string("sub/link"), std::string("")),
                                              std::make_pair(std::string("link"), std::string("..")),
                                              std::make_pair(std::string("link"), std::string("../outside")),
                                              std::make_pair(std::string("sub/link"), std::string("../../outside")),
                                              std::make_pair(std::string("sub/link"), std::string("x/../../../outside")),
                                              std::make_pair(std::string("sub/link"), std::string("..\\..\\outside")),
                                              // symlink() stops at the NUL, so this target would be created as "..".
                                              std::make_pair(std::string("link"), std::string("..\0", 3)));
    CAPTURE(link, target);
    CHECK_FALSE(is_symlink_target_within_root(link, target, tmp.path()));
}

#ifndef _WIN32
TEST_CASE("is_symlink_target_within_root rejects a target that passes through a symlink leading out", "[utils]") {
    ScopedTemporaryDir tmp;
    const boost::filesystem::path root    = tmp.path() / "root";
    const boost::filesystem::path outside = tmp.path() / "outside";
    boost::filesystem::create_directories(root);
    boost::filesystem::create_directories(outside);
    boost::filesystem::create_symlink(outside, root / "out");

    CHECK_FALSE(is_symlink_target_within_root("link", "out/lib.so", root));
    CHECK(is_symlink_target_within_root("link", "in/lib.so", root));
}
#endif

TEST_CASE("is_absolute_path_within_root accepts only entries inside the root", "[utils]") {
    namespace fs = boost::filesystem;
    ScopedTemporaryDir outer;
    const fs::path root = outer.path() / "Auxiliaries";
    fs::create_directories(root / "Others");
    const fs::path inside = root / "Others" / "note.txt";
    const fs::path outside = outer.path() / "secret.txt";
    std::ofstream(inside.string()) << "inside";
    std::ofstream(outside.string()) << "outside";

    SECTION("a file inside the root") {
        REQUIRE(is_absolute_path_within_root(inside, root));
    }
    SECTION("a path inside the root whose file does not exist yet") {
        REQUIRE(is_absolute_path_within_root(root / "Others" / "missing.txt", root));
    }
    SECTION("the root itself") {
        REQUIRE_FALSE(is_absolute_path_within_root(root, root));
    }
    SECTION("a parent-directory escape spelled under the root") {
        REQUIRE_FALSE(is_absolute_path_within_root(root / "Others" / ".." / ".." / "secret.txt", root));
    }
    SECTION("an absolute path elsewhere") {
        REQUIRE_FALSE(is_absolute_path_within_root(outside, root));
    }
    SECTION("a sibling directory sharing the root's name as a prefix") {
        const fs::path sibling = outer.path() / "Auxiliaries2" / "note.txt";
        REQUIRE_FALSE(is_absolute_path_within_root(sibling, root));
    }
    SECTION("a relative path") {
        REQUIRE_FALSE(is_absolute_path_within_root(fs::path("Others") / "note.txt", root));
    }
    SECTION("an empty path") {
        REQUIRE_FALSE(is_absolute_path_within_root(fs::path(), root));
    }
#ifndef _WIN32
    // Creating symlinks on Windows needs elevated rights or developer mode.
    SECTION("a symlink inside the root that points outside") {
        const fs::path link = root / "Others" / "link.txt";
        fs::create_symlink(outside, link);
        REQUIRE_FALSE(is_absolute_path_within_root(link, root));
    }
#endif
}

TEST_CASE("is_safe_to_open_file_name accepts plain documents, images and models", "[utils]") {
    const std::string safe = GENERATE(as<std::string>{},
        "Manual.pdf", "BOM.xlsx", "BOM.csv", "guide.docx", "notes.txt", "README.md", "photo.JPG", "render.png",
        "assembly.step", "part.stl", "project.3mf", "drawing.dxf", "build.mp4", "setup.exe.pdf", ".pdf");
    INFO(safe);
    CHECK(is_safe_to_open_file_name(safe));
}

TEST_CASE("is_safe_to_open_file_name rejects programs and anything it does not know", "[utils]") {
    const std::string unsafe = GENERATE(as<std::string>{},
        "setup.exe", "SETUP.EXE", "Manual.pdf.exe", "run.bat", "shortcut.lnk", "site.url", "script.ps1", "help.chm",
        "tool.jar", "script.py", "Install.command", "install.sh", "launcher.desktop", "Printer.AppImage",
        // Documents that can carry macros or scripts.
        "BOM.xls", "BOM.xlsm", "guide.doc", "guide.docm", "sheet.ods", "page.html", "logo.svg", "bundle.zip",
        // No extension, an unknown one, or a name the desktop would read differently.
        "readme", "pdf", "data.xyz", "", "...", "Manual.pdf.", "Manual.pdf ", "setup.exe:note.txt", "dir.pdf/readme");
    INFO(unsafe);
    CHECK_FALSE(is_safe_to_open_file_name(unsafe));
}

TEST_CASE("ascii_iequals compares ASCII letters regardless of case", "[Utils]") {
    CHECK(ascii_iequals("set_velocity_limit", "SET_VELOCITY_LIMIT"));
    CHECK(ascii_iequals("G28", "g28"));
    CHECK(ascii_iequals("", ""));
    CHECK_FALSE(ascii_iequals("G28", "G29"));
    CHECK_FALSE(ascii_iequals("G2", "G28"));
    CHECK_FALSE(ascii_iequals("G28", "G2"));
    // Non-letters 0x20 apart are not equal.
    CHECK_FALSE(ascii_iequals("[", "{"));
    CHECK_FALSE(ascii_iequals("@", "`"));
}

namespace utils_io_contracts {
using namespace Slic3r;
namespace fs = boost::filesystem;

void write_bytes(const fs::path& p, const std::string& s)
{
    std::ofstream out(p.string(), std::ios::binary);
    out.write(s.data(), static_cast<std::streamsize>(s.size()));
    REQUIRE(out.good());
}
std::string read_bytes(const fs::path& p)
{
    std::ifstream in(p.string(), std::ios::binary);
    REQUIRE(in.good());
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}
void check_bytes(const fs::path& path, const std::string& expected)
{
    const std::string actual = read_bytes(path);
    CHECK(actual == expected);
}
void matches(const std::string& pattern, const std::set<int>& one_based, int last = 20)
{
    for (int n = 1; n <= last; ++n) {
        INFO("pattern=" << pattern << ", layer=" << n);
        CHECK(check_layer_id_pattern(pattern, n - 1) == (one_based.count(n) != 0));
    }
}
struct DirectoryState
{
    std::string var = var_dir(), resources = resources_dir(), temp = temporary_dir(), local = localization_dir();
    std::string shapes = sys_shapes_dir(), gcodes = custom_gcodes_dir(), data = data_dir();
    ~DirectoryState()
    {
        set_var_dir(var);
        set_resources_dir(resources);
        set_temporary_dir(temp);
        set_local_dir(local);
        set_sys_shapes_dir(shapes);
        set_custom_gcodes_dir(gcodes);
        set_data_dir(data);
    }
};

TEST_CASE("Layer patterns use one-based interval starts and repeat bounded spans", "[Utils][UtilsIOContracts]")
{
    matches("5", {5, 10, 15, 20});
    matches("5#2", {5, 6, 10, 11, 15, 16, 20});
    matches("5#", {5, 10, 15, 20});
    matches("1", {1, 2, 3, 4, 5}, 5);
    matches("3#5", {3, 4, 5, 6, 7, 8, 9, 10}, 10);
}
TEST_CASE("Layer patterns strip whitespace and surrounding quote forms", "[Utils][UtilsIOContracts]")
{
    for (const std::string& p : {" 5 # 2 ", "\t\"5#2\"\r\n", "' 5 # 2 '"})
        matches(p, {5, 6, 10, 11, 15, 16, 20});
    matches("\"1, 7, 9\"", {1, 7, 9});
}
TEST_CASE("Layer lists keep independent ranges and do not repeat their entries", "[Utils][UtilsIOContracts]")
{
    matches("1,7,9", {1, 7, 9});
    matches("5,9#2,18", {5, 9, 10, 18});
    matches(",,3#,7#3,,12,", {3, 7, 8, 9, 12});
}
TEST_CASE("Layer lists ignore malformed entries without losing subsequent valid ranges", "[Utils][UtilsIOContracts]")
{
    matches("bad,3,0#2,-1#2,5#0,6#-1,8#2,9999999999999999999999", {3, 8, 9});
    matches("bad,#2,5#bad", {});
}
TEST_CASE("Layer patterns reject empty, negative, nonnumeric and nonpositive intervals", "[Utils][UtilsIOContracts]")
{
    for (const std::string& p : {"", " ", "\"\"", "''", "bad", "#2", "5#bad", "0", "-2", "2#0", "2#-1", "9999999999999999999999"})
        matches(p, {});
    CHECK_FALSE(check_layer_id_pattern("1", -1));
}
TEST_CASE("Decimal memory formatting rounds MB and preserves comma groups", "[Utils][UtilsIOContracts]")
{
    const std::vector<std::pair<size_t, std::string>>
        rows{{0, "0MB"},       {499999, "0MB"},      {500000, "1MB"},        {1499999, "1MB"},
             {1500000, "2MB"}, {999499999, "999MB"}, {999500000, "1,000MB"}, {1002003000000ULL, "1,002,003MB"}};
    for (const auto& [n, s] : rows) {
        CAPTURE(n);
        CHECK(format_memsize_MB(n) == s);
    }
}
TEST_CASE("Binary memory formatting switches units at powers of 1024", "[Utils][UtilsIOContracts]")
{
    const std::vector<std::pair<size_t, std::string>> rows{{0, "0 bytes"},
                                                           {1023, "1023 bytes"},
                                                           {1024, "1024 bytes (1.0KB)"},
                                                           {1048576, "1048576 bytes (1.0MB)"},
                                                           {1073741824ULL, "1073741824 bytes (1.0GB)"},
                                                           {1099511627776ULL, "1099511627776 bytes (1.0TB)"}};
    for (const auto& [n, s] : rows) {
        CAPTURE(n);
        CHECK(format_memsize(n) == s);
    }
    CHECK(format_memsize(1536, 2) == "1536 bytes (1.50KB)");
    CHECK(format_memsize(1572864, 0) == "1572864 bytes (2MB)");
    CHECK(format_memsize(1610612736ULL, 2) == "1610612736 bytes (1.50GB)");
    CHECK(format_memsize(1649267441664ULL, 3) == "1649267441664 bytes (1.500TB)");
}
TEST_CASE("Logging level names map supported severities and the unknown numeric fallback", "[Utils][UtilsIOContracts]")
{
    const std::array<std::string, 6> names{"fatal", "error", "warning", "info", "debug", "trace"};
    for (unsigned i = 0; i < names.size(); ++i) {
        CHECK(level_string_to_boost(names[i]) == i);
        CHECK(get_string_logging_level(i) == names[i]);
    }
    CHECK(get_string_logging_level(99) == "error");
}
TEST_CASE("UTF8 stepping spans ASCII and complete modern multibyte characters", "[Utils][UtilsIOContracts]")
{
    const std::string s = "A\xc3\xa9\xe6\xa8\xa1\xf0\x9f\x90\x8bZ";
    const std::vector<std::pair<size_t, size_t>> rows{{0, 1}, {1, 2}, {3, 3}, {6, 4}, {10, 1}};
    for (const auto& [pos, len] : rows) {
        CHECK(get_utf8_sequence_length(s, pos) == len);
        CHECK(get_utf8_sequence_length(s.data() + pos, s.size() - pos) == len);
    }
}
TEST_CASE("UTF8 stepping clamps truncated sequences to available bytes", "[Utils][UtilsIOContracts]")
{
    for (const auto& [lead, width] : std::vector<std::pair<unsigned char, size_t>>{{0xc3, 2}, {0xe6, 3}, {0xf0, 4}, {0xf8, 5}, {0xfc, 6}}) {
        // This helper measures byte spans, including legacy five/six-byte headers; it is not a Unicode validator.
        std::string s(width, char(0x80));
        s[0] = char(lead);
        for (size_t available = 1; available <= width; ++available) {
            CAPTURE(lead, width, available);
            CHECK(get_utf8_sequence_length(s.data(), available) == available);
        }
    }
}
TEST_CASE("UTF8 stepping from a continuation byte stops before a new character", "[Utils][UtilsIOContracts]")
{
    const std::string ascii_end  = "\x80\xbf\x81Z";
    const std::string header_end = "\x80\xbf\xc3\xa9";
    CHECK(get_utf8_sequence_length(ascii_end, 0) == 3);
    CHECK(get_utf8_sequence_length(ascii_end.data(), 2) == 2);
    CHECK(get_utf8_sequence_length(header_end, 0) == 2);
    CHECK(get_utf8_sequence_length("\xfe", 1) == 1);
    CHECK(get_utf8_sequence_length("\xff", 1) == 1);
}
TEST_CASE("File verification distinguishes absent inputs, length differences and changed bytes", "[Utils][UtilsIOContracts]")
{
    ScopedTemporaryDir d;
    const auto a = d.path() / "a", b = d.path() / "b";
    CHECK(check_copy(a.string(), b.string()) == FAIL_CHECK_ORIGIN_NOT_OPENED);
    write_bytes(a, "abc");
    CHECK(check_copy(a.string(), b.string()) == FAIL_CHECK_TARGET_NOT_OPENED);
    write_bytes(b, "ab");
    CHECK(check_copy(a.string(), b.string()) == FAIL_FILES_DIFFERENT);
    write_bytes(b, "abd");
    CHECK(check_copy(a.string(), b.string()) == FAIL_FILES_DIFFERENT);
    write_bytes(b, "abc");
    CHECK(check_copy(a.string(), b.string()) == SUCCESS);
    write_bytes(a, "");
    write_bytes(b, "");
    CHECK(check_copy(a.string(), b.string()) == SUCCESS);
}
TEST_CASE("File verification detects changes after its first eight MiB chunk", "[Utils][UtilsIOContracts]")
{
    ScopedTemporaryDir d;
    const auto a = d.path() / "a", b = d.path() / "b";
    std::string data(8 * 1024 * 1024 + 17, 'x');
    data.front() = 'A';
    data.back()  = 'Z';
    write_bytes(a, data);
    write_bytes(b, data);
    CHECK(check_copy(a.string(), b.string()) == SUCCESS);
    data.back() = 'Y';
    write_bytes(b, data);
    CHECK(check_copy(a.string(), b.string()) == FAIL_FILES_DIFFERENT);
}
TEST_CASE("Verified copies preserve binary bytes and replace an existing destination", "[Utils][UtilsIOContracts]")
{
    ScopedTemporaryDir d;
    const auto a = d.path() / "a", b = d.path() / "b";
    const std::string data("x\0y\r\nz", 6);
    write_bytes(a, data);
    write_bytes(b, "obsolete");
    std::string error;
    CHECK(copy_file(a.string(), b.string(), error, true) == SUCCESS);
    check_bytes(b, data);
    CHECK_FALSE(fs::exists(b.string() + ".tmp"));
}
TEST_CASE("Framework copying recurses through directories and overwrites ordinary files", "[Utils][UtilsIOContracts]")
{
    ScopedTemporaryDir d;
    const auto src = d.path() / "src", dst = d.path() / "dst";
    fs::create_directories(src / "sub");
    fs::create_directories(dst);
    write_bytes(src / "top", "top");
    write_bytes(src / "sub" / "child", "nested");
    write_bytes(dst / "top", "old");
    REQUIRE(copy_framework(src.string(), dst.string()));
    check_bytes(dst / "top", "top");
    check_bytes(dst / "sub" / "child", "nested");
    CHECK_FALSE(copy_framework((src / "top").string(), (d.path() / "bad").string()));
    write_bytes(d.path() / "blocked", "file");
    CHECK_FALSE(copy_framework(src.string(), (d.path() / "blocked").string()));
}
#ifndef _WIN32
TEST_CASE("Framework copying preserves relative symlink targets", "[Utils][UtilsIOContracts]")
{
    ScopedTemporaryDir d;
    const auto src = d.path() / "src", dst = d.path() / "dst";
    fs::create_directories(src / "Versions" / "A");
    write_bytes(src / "Versions" / "A" / "binary", "payload");
    fs::create_symlink("A", src / "Versions" / "Current");
    REQUIRE(copy_framework(src.string(), dst.string()));
    CHECK(fs::is_symlink(fs::symlink_status(dst / "Versions" / "Current")));
    CHECK(fs::read_symlink(dst / "Versions" / "Current").string() == "A");
    check_bytes(dst / "Versions" / "Current" / "binary", "payload");
}
#endif
TEST_CASE("Recursive directory copies exclude selected filenames and distinguish replacement from merge", "[Utils][UtilsIOContracts]")
{
    ScopedTemporaryDir d;
    const auto src = d.path() / "src", dst = d.path() / "dst";
    fs::create_directories(src / "sub");
    fs::create_directories(dst / "sub");
    write_bytes(src / "keep.json", "new");
    write_bytes(src / "ignore.png", "image");
    write_bytes(src / "sub" / "child", "child");
    write_bytes(dst / "stale", "stale");
    write_bytes(dst / "sub" / "old", "old");
    copy_directory_recursively(src, dst, [](const std::string& name) { return name == "ignore.png"; }, true);
    check_bytes(dst / "keep.json", "new");
    check_bytes(dst / "sub" / "child", "child");
    CHECK(fs::exists(dst / "stale"));
    CHECK(fs::exists(dst / "sub" / "old"));
    CHECK_FALSE(fs::exists(dst / "ignore.png"));
    copy_directory_recursively(src, dst, {}, false);
    CHECK_FALSE(fs::exists(dst / "stale"));
    CHECK_FALSE(fs::exists(dst / "sub" / "old"));
    check_bytes(dst / "ignore.png", "image");
}
TEST_CASE("Recursive directory copying reports an unwritable child destination", "[Utils][UtilsIOContracts]")
{
    ScopedTemporaryDir d;
    const auto src = d.path() / "src", dst = d.path() / "dst";
    fs::create_directories(src);
    fs::create_directories(dst / "file.txt" / "occupied");
    write_bytes(src / "file.txt", "payload");
    CHECK_THROWS_AS(copy_directory_recursively(src, dst, {}, true), CriticalException);
}
TEST_CASE("File classification checks case-insensitive extensions and regular-file type", "[Utils][UtilsIOContracts]")
{
    ScopedTemporaryDir d;
    for (const std::string& name : {"settings.INI", "cache.IdX", "shape.STL"})
        write_bytes(d.path() / name, "data");
    fs::create_directories(d.path() / "folder.ini");
    CHECK(is_ini_file(fs::directory_entry(d.path() / "settings.INI")));
    CHECK_FALSE(is_ini_file(fs::directory_entry(d.path() / "cache.IdX")));
    CHECK_FALSE(is_ini_file(fs::directory_entry(d.path() / "folder.ini")));
    CHECK(is_idx_file(fs::directory_entry(d.path() / "cache.IdX")));
    CHECK_FALSE(is_idx_file(fs::directory_entry(d.path() / "settings.INI")));
    CHECK(is_gallery_file(fs::directory_entry(d.path() / "shape.STL"), ".stl"));
    CHECK_FALSE(is_gallery_file(fs::directory_entry(d.path() / "folder.ini"), ".ini"));
    CHECK(is_gallery_file(std::string("nested/shape.STL"), ".stl"));
    CHECK_FALSE(is_gallery_file(std::string("shape.stl.backup"), ".stl"));
    CHECK(is_img_file("image.PNG"));
    CHECK(is_img_file("image.SvG"));
    CHECK_FALSE(is_img_file("image.svg.json"));
}
TEST_CASE("Configured resource paths restore globals and distinguish all shape roots", "[Utils][UtilsIOContracts]")
{
    ScopedTemporaryDir d;
    DirectoryState restore;
    set_var_dir((d.path() / "var").string());
    set_resources_dir((d.path() / "res").string());
    set_data_dir((d.path() / "data" / "nested").string());
    set_local_dir((d.path() / "locale").string());
    set_sys_shapes_dir((d.path() / "shapes").string());
    set_custom_gcodes_dir((d.path() / "gcodes").string());
    set_temporary_dir((d.path() / "temp").string());
    CHECK(fs::is_directory(data_dir()));
    CHECK(localization_dir() == (d.path() / "locale").string());
    CHECK(custom_gcodes_dir() == (d.path() / "gcodes").string());
    CHECK(temporary_dir() == (d.path() / "temp").string());
    CHECK(var("missing.png") == (d.path() / "var" / "missing.png").make_preferred().string());
    const auto existing = d.path() / "existing.png";
    write_bytes(existing, "image");
    CHECK(var(existing.string()) == existing.string());
    CHECK(custom_shapes_dir() == (fs::path(data_dir()) / "shapes").string());
    CHECK(handy_models_dir() == (d.path() / "res" / "handy_models").string());
    CHECK(is_shapes_dir(sys_shapes_dir()));
    CHECK(is_shapes_dir(custom_shapes_dir()));
    CHECK(is_shapes_dir(handy_models_dir()));
    CHECK_FALSE(is_shapes_dir((d.path() / "unrelated").string()));
}
TEST_CASE("MD5 file fingerprints match standard vectors across the 64 KiB read boundary", "[Utils][UtilsIOContracts]")
{
    ScopedTemporaryDir d;
    std::string file = (d.path() / "data").string(), digest;
    for (const auto& [data, expected] :
         std::vector<std::pair<std::string, std::string>>{{"", "D41D8CD98F00B204E9800998ECF8427E"},
                                                          {"abc", "900150983CD24FB0D6963F7D28E17F72"},
                                                          {std::string(65537, 'a'), "B3C6FC238E908636E53AABD5AD830CF7"}}) {
        write_bytes(file, data);
        REQUIRE(bbl_calc_md5(file, digest));
        CHECK(digest == expected);
    }
}
TEST_CASE("Perl-compatible path helpers preserve the parent and final extension", "[Utils][UtilsIOContracts]")
{
    const auto p = fs::path("models") / "nested" / "part.v2.3mf";
    const auto s = p.string();
    CHECK(PerlUtils::path_to_filename(s.c_str()) == "part.v2.3mf");
    CHECK(PerlUtils::path_to_stem(s.c_str()) == "part.v2");
    CHECK(PerlUtils::path_to_extension(s.c_str()) == ".3mf");
    CHECK(PerlUtils::path_to_parent_path(s.c_str()) == p.parent_path().string());
}

TEST_CASE("Framework copying propagates a failure from a nested directory", "[Utils][UtilsFrameworkFailure]")
{
    ScopedTemporaryDir d;
    const auto src = d.path() / "src", dst = d.path() / "dst";
    fs::create_directories(src / "nested");
    fs::create_directories(dst);
    write_bytes(src / "nested" / "payload", "source");
    write_bytes(dst / "nested", "occupied by a file");
    CHECK_FALSE(copy_framework(src.string(), dst.string()));
    check_bytes(dst / "nested", "occupied by a file");
}
} // namespace utils_io_contracts
