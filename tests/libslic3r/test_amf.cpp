#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/Config.hpp"
#include "libslic3r/Format/AMF.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/Utils.hpp"
#include "libslic3r/miniz_extension.hpp"

#include "test_utils.hpp"

#include <boost/nowide/fstream.hpp>
#include <miniz.h>

#include <array>
#include <ios>
#include <string>
#include <utility>
#include <vector>

using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace {

// The smallest AMF the loader accepts: one object holding one volume, a tetrahedron. The metadata
// element of the object is dropped in verbatim, so a test can hand the parser a malformed one.
std::string amf_with_object_metadata(const std::string &object_metadata)
{
    return "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
           "<amf unit=\"millimeter\">\n"
           " <object id=\"0\">\n"
           "  " + object_metadata + "\n"
           "  <mesh>\n"
           "   <vertices>\n"
           "    <vertex><coordinates><x>0</x><y>0</y><z>0</z></coordinates></vertex>\n"
           "    <vertex><coordinates><x>1</x><y>0</y><z>0</z></coordinates></vertex>\n"
           "    <vertex><coordinates><x>0</x><y>1</y><z>0</z></coordinates></vertex>\n"
           "    <vertex><coordinates><x>0</x><y>0</y><z>1</z></coordinates></vertex>\n"
           "   </vertices>\n"
           "   <volume>\n"
           "    <triangle><v1>0</v1><v2>2</v2><v3>1</v3></triangle>\n"
           "    <triangle><v1>0</v1><v2>1</v2><v3>3</v3></triangle>\n"
           "    <triangle><v1>0</v1><v2>3</v2><v3>2</v3></triangle>\n"
           "    <triangle><v1>1</v1><v2>2</v2><v3>3</v3></triangle>\n"
           "   </volume>\n"
           "  </mesh>\n"
           " </object>\n"
           "</amf>\n";
}

void write_file(const std::string &path, const std::string &content)
{
    boost::nowide::ofstream f(path, std::ios::binary);
    f << content;
}

bool load(const std::string &path, Model &model, bool *inches = nullptr)
{
    DynamicPrintConfig        config;
    ConfigSubstitutionContext substitutions(ForwardCompatibilitySubstitutionRule::Disable);
    return load_amf(path.c_str(), &config, &substitutions, &model, inches);
}

void replace(std::string &s, const std::string &a, const std::string &b)
{
    const auto at = s.find(a);
    REQUIRE(at != std::string::npos);
    s.replace(at, a.size(), b);
}

void archive(const std::string &path, const std::vector<std::pair<std::string, std::string>> &entries)
{
    mz_zip_archive zip{};
    REQUIRE(open_zip_writer(&zip, path));
    ScopeGuard close([&] { close_zip_writer(&zip); });
    for (const auto &[name, text] : entries)
        REQUIRE(mz_zip_writer_add_mem(&zip, name.c_str(), text.data(), text.size(), MZ_BEST_COMPRESSION));
    REQUIRE(mz_zip_writer_finalize_archive(&zip));
}

std::string document(const std::string &instances = "")
{
    auto s = amf_with_object_metadata("<metadata type=\"name\">Tetra &amp; café</metadata>");
    replace(s, "<volume>", "<volume><metadata type=\"name\">Volume Ω</metadata>");
    if (!instances.empty())
        replace(s, "</amf>", "<constellation id=\"9\">" + instances + "</constellation></amf>");
    return s;
}

std::string instance(const std::string &fields, const std::string &id = "0")
{
    return "<instance objectid=\"" + id + "\">" + fields + "</instance>";
}

void geometry(const Model &model, const std::string &path)
{
    INFO("loaded from " << path);
    REQUIRE(model.objects.size() == 1);
    const auto *o = model.objects.front();
    REQUIRE(o->volumes.size() == 1);
    const auto *v = o->volumes.front();
    CHECK(o->name == "Tetra & café");
    CHECK(v->name == "Volume Ω");
    CHECK(v->mesh().facets_count() == 4);
    CHECK(v->mesh().its.vertices.size() == 4);
    CHECK(v->source.input_file == path);
    CHECK_THAT(its_volume(v->mesh().its), WithinAbs(1. / 6., 1e-6));
    const auto box = v->mesh().bounding_box();
    for (int i = 0; i < 3; ++i)
        CHECK_THAT(box.size()[i], WithinAbs(1., 1e-6));
}

} // namespace

TEST_CASE("An AMF object metadata element with no type attribute is rejected", "[AMF]")
{
    ScopedTemporaryFile tmp(".amf");

    SECTION("with the attribute the file loads")
    {
        write_file(tmp.string(), amf_with_object_metadata("<metadata type=\"name\">tetra</metadata>"));

        Model model;
        REQUIRE(load(tmp.string(), model));
        CHECK(model.objects.size() == 1);
    }

    SECTION("without it the load fails instead of reading a null attribute")
    {
        write_file(tmp.string(), amf_with_object_metadata("<metadata>tetra</metadata>"));

        Model model;
        CHECK_FALSE(load(tmp.string(), model));
    }
}

TEST_CASE("AMF archives preserve named tetrahedron geometry and source identity", "[AMF][AMFArchive]")
{
    const auto name = GENERATE(std::string("model.amf"), std::string("folder/MODEL.AMF"));
    ScopedTemporaryFile file(".zip.amf");
    archive(file.string(), {{"readme.txt", "unrelated"}, {name, document()}});
    Model m;
    bool inches = true;
    REQUIRE(load(file.string(), m, &inches));
    CHECK_FALSE(inches);
    geometry(m, file.string());
}

TEST_CASE("AMF text and compressed input agree on the declared inch flag", "[AMF][AMFArchive]")
{
    const bool zip = GENERATE(false, true);
    auto xml       = document();
    replace(xml, "millimeter", "inch");
    ScopedTemporaryFile file(".amf");
    if (zip)
        archive(file.string(), {{"unit.amf", xml}});
    else
        write_file(file.string(), xml);
    Model m;
    bool inches = false;
    REQUIRE(load(file.string(), m, &inches));
    CHECK(inches);
    geometry(m, file.string());
}

TEST_CASE("AMF archives reject malformed embedded XML", "[AMF][AMFArchive]")
{
    const auto xml = GENERATE(std::string("<amf>"), std::string("<wrong/>"), std::string("<amf><object/></amf>"));
    ScopedTemporaryFile f(".amf");
    archive(f.string(), {{"bad.amf", xml}});
    Model m;
    CHECK_FALSE(load(f.string(), m));
}

TEST_CASE("AMF archives reject empty model entries", "[AMF][AMFArchive]")
{
    ScopedTemporaryFile f(".amf");
    archive(f.string(), {{"empty.amf", ""}});
    Model m;
    CHECK_FALSE(load(f.string(), m));
}

TEST_CASE("AMF input rejects a damaged ZIP container", "[AMF][AMFArchive]")
{
    ScopedTemporaryFile f(".amf");
    write_file(f.string(), "PK invalid archive");
    Model m;
    CHECK_FALSE(load(f.string(), m));
}

TEST_CASE("AMF archives stream model XML across extraction chunks", "[AMF][AMFArchive]")
{
    // The padding comes first, so the whole object (names, vertices, triangles) is only seen if the
    // parser is fed the extraction chunks after the first one (miniz hands out at most 64 KiB at a time).
    auto xml = document();
    const std::string root = "<amf unit=\"millimeter\">\n";
    replace(xml, root, root + "<metadata type=\"long\">" + std::string(150000, 'x') + "</metadata>\n");
    REQUIRE(xml.find("<object") > 150000);
    ScopedTemporaryFile f(".amf");
    archive(f.string(), {{"large.amf", xml}});
    Model m;
    REQUIRE(load(f.string(), m));
    geometry(m, f.string());
}

TEST_CASE("AMF constellation offsets retain all three coordinates", "[AMF][AMFArchive]")
{
    const bool zip = GENERATE(false, true);
    const auto xml = document(instance("<deltax>2.5</deltax><deltay>-3.25</deltay><deltaz>4.75</deltaz>"));
    ScopedTemporaryFile f(".amf");
    if (zip)
        archive(f.string(), {{"model.amf", xml}});
    else
        write_file(f.string(), xml);
    Model m;
    REQUIRE(load(f.string(), m));
    REQUIRE(m.objects.size() == 1);
    REQUIRE(m.objects[0]->instances.size() == 1);
    const auto p = m.objects[0]->instances[0]->get_offset();
    CHECK_THAT(p.x(), WithinAbs(2.5, 1e-6));
    CHECK_THAT(p.y(), WithinAbs(-3.25, 1e-6));
    CHECK_THAT(p.z(), WithinAbs(4.75, 1e-6));
}

TEST_CASE("AMF constellation rotations retain independent axes", "[AMF][AMFArchive]")
{
    ScopedTemporaryFile f(".amf");
    archive(f.string(), {{"model.amf", document(instance("<rx>0.25</rx><ry>-0.5</ry><rz>1.25</rz>"))}});
    Model m;
    REQUIRE(load(f.string(), m));
    REQUIRE(m.objects.size() == 1);
    REQUIRE(m.objects[0]->instances.size() == 1);
    const auto p = m.objects[0]->instances[0]->get_rotation();
    CHECK_THAT(p.x(), WithinAbs(0.25, 1e-6));
    CHECK_THAT(p.y(), WithinAbs(-0.5, 1e-6));
    CHECK_THAT(p.z(), WithinAbs(1.25, 1e-6));
}

TEST_CASE("AMF constellation axis scales and mirrors remain distinct", "[AMF][AMFArchive]")
{
    ScopedTemporaryFile f(".amf");
    archive(f.string(), {{"model.amf", document(instance("<scalex>2</scalex><scaley>3</scaley><scalez>4</scalez><mirrorx>-1</"
                                                         "mirrorx><mirrory>1</mirrory><mirrorz>-1</mirrorz>"))}});
    Model m;
    REQUIRE(load(f.string(), m));
    REQUIRE(m.objects.size() == 1);
    REQUIRE(m.objects[0]->instances.size() == 1);
    const auto *i     = m.objects[0]->instances[0];
    const auto scale  = i->get_scaling_factor();
    const auto matrix = i->get_matrix();
    for (int j = 0; j < 3; ++j) {
        CHECK_THAT(scale[j], WithinAbs(j + 2., 1e-6));
        for (int k = 0; k < 3; ++k)
            CHECK_THAT(matrix(j, k), WithinAbs(j == k ? (j + 2.) * (j == 1 ? 1. : -1.) : 0., 1e-6));
    }
}

TEST_CASE("AMF uniform scale expands to every axis", "[AMF]")
{
    ScopedTemporaryFile f(".amf");
    write_file(f.string(), document(instance("<scale>2.5</scale>")));
    Model m;
    REQUIRE(load(f.string(), m));
    REQUIRE(m.objects.size() == 1);
    REQUIRE(m.objects[0]->instances.size() == 1);
    const auto s = m.objects[0]->instances[0]->get_scaling_factor();
    for (int j = 0; j < 3; ++j)
        CHECK_THAT(s[j], WithinAbs(2.5, 1e-6));
}

TEST_CASE("AMF constellation copies preserve independent placement and printability", "[AMF][AMFArchive]")
{
    ScopedTemporaryFile f(".amf");
    archive(f.string(), {{"model.amf", document(instance("<deltax>2</deltax><printable>1</printable>") +
                                                instance("<deltax>9</deltax><printable>0</printable>"))}});
    Model m;
    REQUIRE(load(f.string(), m));
    REQUIRE(m.objects.size() == 2);
    for (const auto *o : m.objects) {
        REQUIRE(o->instances.size() == 1);
        REQUIRE(o->volumes.size() == 1);
        CHECK(o->volumes[0]->mesh().facets_count() == 4);
    }
    CHECK_THAT(m.objects[0]->instances[0]->get_offset().x(), WithinAbs(2., 1e-6));
    CHECK_THAT(m.objects[1]->instances[0]->get_offset().x(), WithinAbs(9., 1e-6));
    CHECK(m.objects[0]->instances[0]->printable);
    CHECK_FALSE(m.objects[1]->instances[0]->printable);
    CHECK(m.objects[0]->id() != m.objects[1]->id());
}

TEST_CASE("AMF missing constellation targets do not duplicate known geometry", "[AMF][AMFArchive]")
{
    ScopedTemporaryFile f(".amf");
    archive(f.string(), {{"model.amf", document(instance("<deltax>12</deltax>", "missing"))}});
    Model m;
    REQUIRE(load(f.string(), m));
    geometry(m, f.string());
    CHECK(m.objects[0]->instances.empty());
}

TEST_CASE("AMF empty constellation instances preserve uninstanced geometry", "[AMF]")
{
    ScopedTemporaryFile f(".amf");
    write_file(f.string(), document(instance("")));
    Model m;
    REQUIRE(load(f.string(), m));
    geometry(m, f.string());
}

TEST_CASE("AMF material attributes retain Unicode and entity decoding", "[AMF][AMFArchive]")
{
    auto xml = document();
    replace(xml, "</amf>",
            "<material id=\"m1\"><metadata type=\"name\">PLA &amp; café</metadata><metadata "
            "type=\"color\">#123456</metadata></material></amf>");
    ScopedTemporaryFile f(".amf");
    archive(f.string(), {{"model.amf", xml}});
    Model m;
    REQUIRE(load(f.string(), m));
    REQUIRE(m.materials.size() == 1);
    REQUIRE(m.materials.count("m1") == 1);
    CHECK(m.materials.at("m1")->attributes.at("name") == "PLA & café");
    CHECK(m.materials.at("m1")->attributes.at("color") == "#123456");
}

TEST_CASE("AMF archives reject triangle references outside the vertex list", "[AMF][AMFArchive]")
{
    const auto index = GENERATE(std::string("-1"), std::string("4"), std::string("99"));
    auto xml         = document();
    replace(xml, "<v1>0</v1>", "<v1>" + index + "</v1>");
    ScopedTemporaryFile f(".amf");
    archive(f.string(), {{"model.amf", xml}});
    Model m;
    CHECK_FALSE(load(f.string(), m));
}

TEST_CASE("AMF archives reject a volume without triangles", "[AMF][AMFArchive]")
{
    auto xml     = document();
    const auto a = xml.find("<triangle>");
    const auto z = xml.rfind("</triangle>");
    REQUIRE(a != std::string::npos);
    REQUIRE(z != std::string::npos);
    xml.erase(a, z + 11 - a);
    ScopedTemporaryFile f(".amf");
    archive(f.string(), {{"model.amf", xml}});
    Model m;
    CHECK_FALSE(load(f.string(), m));
}

TEST_CASE("AMF archived object metadata requires its type attribute", "[AMF][AMFArchive]")
{
    auto xml = document();
    replace(xml, "<metadata type=\"name\">", "<metadata>");
    ScopedTemporaryFile f(".amf");
    archive(f.string(), {{"model.amf", xml}});
    Model m;
    CHECK_FALSE(load(f.string(), m));
}

TEST_CASE("AMF normalizes inward triangle winding to positive volume", "[AMF][AMFArchive]")
{
    auto xml = document();
    const std::array<std::pair<std::string, std::string>, 4> faces{{{"0</v1><v2>2</v2><v3>1", "0</v1><v2>1</v2><v3>2"},
                                                                    {"0</v1><v2>1</v2><v3>3", "0</v1><v2>3</v2><v3>1"},
                                                                    {"0</v1><v2>3</v2><v3>2", "0</v1><v2>2</v2><v3>3"},
                                                                    {"1</v1><v2>2</v2><v3>3", "1</v1><v2>3</v2><v3>2"}}};
    for (const auto &[a, b] : faces)
        replace(xml, a, b);
    ScopedTemporaryFile f(".amf");
    archive(f.string(), {{"model.amf", xml}});
    Model m;
    REQUIRE(load(f.string(), m));
    geometry(m, f.string());
}
