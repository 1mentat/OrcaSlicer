#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/Config.hpp"
#include "libslic3r/Exception.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Format/AMF.hpp"
#include "libslic3r/Format/ProjectStrategy.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/Utils.hpp"
#include "libslic3r/miniz_extension.hpp"
#include "libslic3r/libslic3r.h"

#include "test_utils.hpp"

#include <boost/nowide/fstream.hpp>
#include <miniz.h>

#include <array>
#include <ios>
#include <string>
#include <utility>
#include <boost/filesystem/path.hpp>

#include <algorithm>
#include <cstddef>
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

TEST_CASE("AMF empty constellation instances still load the object geometry", "[AMF]")
{
    ScopedTemporaryFile f(".amf");
    write_file(f.string(), document(instance("")));
    Model m;
    REQUIRE(load(f.string(), m));
    geometry(m, f.string());
    CHECK(m.objects[0]->instances.empty());
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

TEST_CASE("AMF archives stream model XML across extraction chunks (original M033 trailing-metadata fixture)", "[AMF][AMFArchive]")
{
    auto xml = document();
    replace(xml, "</amf>", "<metadata type=\"long\">" + std::string(150000, 'x') + "</metadata></amf>");
    ScopedTemporaryFile f(".amf");
    archive(f.string(), {{"large.amf", xml}});
    Model m;
    bool inches = false;
    REQUIRE(load(f.string(), m, &inches));
    geometry(m, f.string());
}

namespace {

// Reuse the tetrahedron fixture, giving object and volume deliberately different names.
std::string named_amf_object(int id, const std::string &object_name, const std::string &volume_name)
{
    std::string xml = amf_with_object_metadata("<metadata type=\"name\">" + object_name + "</metadata>");
    const size_t begin = xml.find(" <object");
    xml = xml.substr(begin, xml.find(" </object>", begin) + std::string(" </object>").size() - begin);
    const std::string id_attribute = "id=\"0\"";
    xml.replace(xml.find(id_attribute), id_attribute.size(), "id=\"" + std::to_string(id) + "\"");
    xml.insert(xml.find("<volume>") + std::string("<volume>").size(),
               "<metadata type=\"name\">" + volume_name + "</metadata>");
    return xml;
}

bool load_amf_with_options(const std::string &path, Model &model, LoadStrategy options)
{
    DynamicPrintConfig config;
    ConfigSubstitutionContext substitutions(ForwardCompatibilitySubstitutionRule::Disable);
    return load_amf(path.c_str(), &config, &substitutions, &model, nullptr, options);
}

// Read the part's actual geometry in assembly coordinates, including importer centering.
BoundingBoxf3 amf_part_bounds(const ModelVolume &volume)
{
    TriangleMesh mesh(volume.mesh());
    mesh.transform(volume.get_matrix());
    return mesh.bounding_box();
}

} // namespace

TEST_CASE("AMF assembly parts retain their volume names and instance transforms", "[AMF][OriginalNames]")
{
    ScopedTemporaryFile tmp(".amf");
    write_file(tmp.string(), "<amf unit=\"millimeter\">" +
        named_amf_object(1, "ParentA", "LeafA") + named_amf_object(2, "ParentB", "LeafB") +
        "<constellation id=\"10\">"
        "<instance objectid=\"1\"/>"
        "<instance objectid=\"2\"><deltax>10</deltax><deltaz>5</deltaz><scale>2</scale></instance>"
        "</constellation></amf>");

    Model model;
    REQUIRE(load_amf_with_options(tmp.string(), model,
        LoadStrategy::LoadModel | LoadStrategy::AssembleParts | LoadStrategy::KeepOriginalNames));
    REQUIRE(model.objects.size() == 1);
    const auto &parts = model.objects.front()->volumes;
    REQUIRE(parts.size() == 2);
    CHECK(parts[0]->name == "LeafA");
    CHECK(parts[1]->name == "LeafB");
    const BoundingBoxf3 first = amf_part_bounds(*parts[0]);
    const BoundingBoxf3 second = amf_part_bounds(*parts[1]);
    CHECK_THAT(first.min.x(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    CHECK_THAT(first.min.z(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    CHECK_THAT(second.min.x(), Catch::Matchers::WithinAbs(10.0, 1e-6));
    CHECK_THAT(second.min.z(), Catch::Matchers::WithinAbs(5.0, 1e-6));
    CHECK_THAT(second.size().x(), Catch::Matchers::WithinAbs(2.0, 1e-6));
    CHECK_THAT(second.size().z(), Catch::Matchers::WithinAbs(2.0, 1e-6));
}

TEST_CASE("AMF constellations preserve separate groups and repeated object occurrences", "[AMF][OriginalNames]")
{
    ScopedTemporaryFile tmp(".amf");
    // The same definition is referenced twice in one group and again in another.
    write_file(tmp.string(), "<amf unit=\"millimeter\">" +
        named_amf_object(1, "SharedParent", "SharedLeaf") + named_amf_object(2, "Standalone", "Alone") +
        "<constellation id=\"10\"><instance objectid=\"1\"/>"
        "<instance objectid=\"1\"><deltax>10</deltax></instance></constellation>"
        "<constellation id=\"20\"><instance objectid=\"1\"><deltax>30</deltax></instance></constellation>"
        "</amf>");

    Model model;
    REQUIRE(load_amf_with_options(tmp.string(), model,
        LoadStrategy::LoadModel | LoadStrategy::AssembleParts | LoadStrategy::KeepOriginalNames));
    REQUIRE(model.objects.size() == 3);
    size_t repeated_groups = 0;
    size_t distant_groups = 0;
    size_t standalone_objects = 0;
    for (const ModelObject *object : model.objects) {
        if (object->name == "Standalone") {
            ++standalone_objects;
            REQUIRE(object->volumes.size() == 1);
            CHECK(object->volumes.front()->name == "Alone");
        } else if (object->volumes.size() == 2) {
            ++repeated_groups;
            CHECK(object->volumes[0]->name == "SharedLeaf");
            CHECK(object->volumes[1]->name == "SharedLeaf");
            CHECK_THAT(amf_part_bounds(*object->volumes[1]).min.x() -
                       amf_part_bounds(*object->volumes[0]).min.x(), Catch::Matchers::WithinAbs(10.0, 1e-6));
        } else {
            ++distant_groups;
            REQUIRE(object->volumes.size() == 1);
            CHECK(object->volumes.front()->name == "SharedLeaf");
            REQUIRE(object->instances.size() == 1);
            CHECK_THAT(object->instance_bounding_box(0).min.x(), Catch::Matchers::WithinAbs(30.0, 1e-6));
        }
    }
    CHECK(repeated_groups == 1);
    CHECK(distant_groups == 1);
    CHECK(standalone_objects == 1);
}

TEST_CASE("AMF assembly structure is independent of original name selection", "[AMF][OriginalNames]")
{
    const bool keep_names = GENERATE(false, true);
    ScopedTemporaryFile tmp(".amf");
    write_file(tmp.string(), "<amf unit=\"millimeter\">" +
        named_amf_object(1, "ParentA", "LeafA") + named_amf_object(2, "ParentB", "LeafB") +
        "<constellation id=\"3\"><instance objectid=\"1\"/><instance objectid=\"2\"/></constellation></amf>");
    LoadStrategy options = LoadStrategy::LoadModel | LoadStrategy::AssembleParts;
    if (keep_names)
        options = options | LoadStrategy::KeepOriginalNames;
    Model model;
    REQUIRE(load_amf_with_options(tmp.string(), model, options));
    REQUIRE(model.objects.size() == 1);
    REQUIRE(model.objects.front()->volumes.size() == 2);
    CHECK(model.objects.front()->volumes[0]->name == (keep_names ? "LeafA" : "ParentA"));
    CHECK(model.objects.front()->volumes[1]->name == (keep_names ? "LeafB" : "ParentB"));
}

TEST_CASE("AMF assemblies without placements are left for the GUI to centre on the bed", "[AMF][OriginalNames]")
{
    // CAD exporters list occurrences without deltas; the legacy loader then creates no instance either.
    const bool merged = GENERATE(false, true);
    ScopedTemporaryFile tmp(".amf");
    write_file(tmp.string(), "<amf unit=\"millimeter\">" +
        named_amf_object(1, "ParentA", "LeafA") + named_amf_object(2, "ParentB", "LeafB") +
        "<constellation id=\"3\"><instance objectid=\"1\"/>" +
        std::string(merged ? "<instance objectid=\"2\"/>" : "") + "</constellation></amf>");
    Model model;
    REQUIRE(load_amf_with_options(tmp.string(), model,
        LoadStrategy::LoadModel | LoadStrategy::AssembleParts | LoadStrategy::KeepOriginalNames));
    // Unreferenced object 2 of the single-occurrence case stays a separate object, as before.
    REQUIRE(model.objects.size() == (merged ? 1 : 2));
    CHECK(model.objects.front()->volumes.size() == (merged ? 2 : 1));
    CHECK(model.objects.front()->instances.empty());
}

TEST_CASE("Unplaced excluded AMF assemblies keep their excluded instance", "[AMF][OriginalNames]")
{
    // Without an instance the GUI would add a printable one and print the excluded part.
    ScopedTemporaryFile tmp(".amf");
    write_file(tmp.string(), "<amf unit=\"millimeter\">" +
        named_amf_object(1, "ParentA", "LeafA") + named_amf_object(2, "ParentB", "LeafB") +
        "<constellation id=\"3\"><instance objectid=\"1\"><printable>0</printable></instance>"
        "<instance objectid=\"2\"><printable>0</printable></instance></constellation></amf>");
    Model model;
    REQUIRE(load_amf_with_options(tmp.string(), model,
        LoadStrategy::LoadModel | LoadStrategy::AssembleParts | LoadStrategy::KeepOriginalNames));
    REQUIRE(model.objects.size() == 1);
    REQUIRE(model.objects.front()->instances.size() == 1);
    CHECK_FALSE(model.objects.front()->instances.front()->printable);
}

TEST_CASE("Multi-part detection compares unplaced AMF groups at zero height", "[AMF][OriginalNames]")
{
    // The GUI checks before it adds instances to unplaced objects, so they have no height on the bed yet;
    // as before, they compare at zero and their mesh heights are ignored.
    const bool placed = GENERATE(false, true);
    const bool raised = GENERATE(false, true);
    const std::string placement = placed ? "<deltax>10</deltax>" : "";
    // A placed object is raised by its placement; the GUI's centering moves the volumes, not the instance, so the rise stays.
    const std::string raised_placement = placement + (placed && raised ? "<deltaz>10</deltaz>" : "");
    ScopedTemporaryFile tmp(".amf");
    write_file(tmp.string(), "<amf unit=\"millimeter\">" +
        named_amf_object(1, "ParentA", "LeafA") + named_amf_object(2, "ParentB", "LeafB") +
        "<constellation id=\"3\"><instance objectid=\"1\">" + placement + "</instance></constellation>"
        "<constellation id=\"4\"><instance objectid=\"2\">" + raised_placement + "</instance></constellation></amf>");
    Model model;
    REQUIRE(load_amf_with_options(tmp.string(), model,
        LoadStrategy::LoadModel | LoadStrategy::AssembleParts | LoadStrategy::KeepOriginalNames));
    REQUIRE(model.objects.size() == 2);
    // An unplaced object can only be raised in its mesh.
    if (raised && !placed) {
        model.objects[1]->volumes.front()->translate(Vec3d(0., 0., 10.));
        model.objects[1]->invalidate_bounding_box();
    }
    CHECK(model.looks_like_multipart_object() == (placed && raised));
    // The check must not place the objects; the GUI still centers unplaced ones later.
    CHECK(model.objects[0]->instances.empty() == !placed);
    CHECK(model.objects[1]->instances.empty() == !placed);
}

TEST_CASE("Multi-part detection of mixed and centered unplaced AMF objects", "[AMF][OriginalNames]")
{
    // Either object comes first, so the result does not depend on which one sets the reference height.
    const bool swapped = GENERATE(false, true);
    const size_t first = swapped ? 1 : 0;
    const size_t second = swapped ? 0 : 1;
    // A placed object stands at its placement, and an unplaced one compares at zero, so they differ.
    SECTION("mixed placement asks") {
        const std::string unplaced = "<instance objectid=\"1\"></instance>";
        const std::string placed = "<instance objectid=\"2\"><deltax>10</deltax><deltaz>10</deltaz></instance>";
        ScopedTemporaryFile tmp(".amf");
        write_file(tmp.string(), "<amf unit=\"millimeter\">" +
            named_amf_object(1, "ParentA", "LeafA") + named_amf_object(2, "ParentB", "LeafB") +
            "<constellation id=\"3\">" + (swapped ? placed : unplaced) + "</constellation>"
            "<constellation id=\"4\">" + (swapped ? unplaced : placed) + "</constellation></amf>");
        Model model;
        REQUIRE(load_amf_with_options(tmp.string(), model,
            LoadStrategy::LoadModel | LoadStrategy::AssembleParts | LoadStrategy::KeepOriginalNames));
        REQUIRE(model.objects.size() == 2);
        CHECK(model.looks_like_multipart_object());
        // The unplaced object comes from the first group unless swapped.
        CHECK(model.objects[first]->instances.empty());
        CHECK(model.objects[second]->instances.size() == 1);
    }
    // The reviewer's case: unplaced objects of different heights after the GUI's rotation, which centers them.
    SECTION("unplaced objects of different heights do not ask") {
        ScopedTemporaryFile tmp(".amf");
        write_file(tmp.string(), "<amf unit=\"millimeter\">" +
            named_amf_object(1, "ParentA", "LeafA") + named_amf_object(2, "ParentB", "LeafB") + "</amf>");
        Model model;
        REQUIRE(load_amf_with_options(tmp.string(), model, LoadStrategy::LoadModel | LoadStrategy::KeepOriginalNames));
        REQUIRE(model.objects.size() == 2);
        model.objects[first]->volumes.front()->scale(Vec3d(1., 1., 4.));
        for (ModelObject *object : model.objects) {
            object->invalidate_bounding_box();
            object->rotate(0., Axis::Z);
        }
        CHECK_FALSE(model.looks_like_multipart_object());
        CHECK(model.objects[0]->instances.empty());
        CHECK(model.objects[1]->instances.empty());
    }
}

TEST_CASE("AMF assemblies with any placement keep their position from the file", "[AMF][OriginalNames]")
{
    ScopedTemporaryFile tmp(".amf");
    write_file(tmp.string(), "<amf unit=\"millimeter\">" +
        named_amf_object(1, "ParentA", "LeafA") + named_amf_object(2, "ParentB", "LeafB") +
        "<constellation id=\"3\"><instance objectid=\"1\"/>"
        "<instance objectid=\"2\"><deltax>10</deltax></instance></constellation></amf>");
    Model model;
    REQUIRE(load_amf_with_options(tmp.string(), model,
        LoadStrategy::LoadModel | LoadStrategy::AssembleParts | LoadStrategy::KeepOriginalNames));
    REQUIRE(model.objects.size() == 1);
    REQUIRE(model.objects.front()->instances.size() == 1);
    CHECK_THAT(model.objects.front()->instance_bounding_box(0).min.x(), Catch::Matchers::WithinAbs(0.0, 1e-6));
}

TEST_CASE("Legacy AMF loading keeps independent object definitions", "[AMF][OriginalNames]")
{
    ScopedTemporaryFile tmp(".amf");
    write_file(tmp.string(), "<amf unit=\"millimeter\">" +
        named_amf_object(1, "ParentA", "LeafA") + named_amf_object(2, "ParentB", "LeafB") +
        "<constellation id=\"3\"><instance objectid=\"1\"/><instance objectid=\"2\"/></constellation></amf>");
    Model model;
    REQUIRE(load(tmp.string(), model));
    REQUIRE(model.objects.size() == 2);
    CHECK(model.objects[0]->name == "ParentA");
    CHECK(model.objects[0]->volumes.front()->name == "LeafA");
    CHECK(model.objects[1]->name == "ParentB");
    CHECK(model.objects[1]->volumes.front()->name == "LeafB");
}

TEST_CASE("AMF mixed printable occurrences retain their individual printability", "[AMF][OriginalNames]")
{
    ScopedTemporaryFile tmp(".amf");
    // Combining these occurrences into one printable instance would print an excluded part.
    write_file(tmp.string(), "<amf unit=\"millimeter\">" + named_amf_object(1, "Parent", "Leaf") +
        "<constellation id=\"10\">"
        "<instance objectid=\"1\"><printable>1</printable></instance>"
        "<instance objectid=\"1\"><deltax>10</deltax><printable>0</printable></instance>"
        "</constellation></amf>");

    Model model;
    REQUIRE(load_amf_with_options(tmp.string(), model,
        LoadStrategy::LoadModel | LoadStrategy::AssembleParts | LoadStrategy::KeepOriginalNames));
    REQUIRE(model.objects.size() == 2);
    size_t printable = 0;
    size_t excluded = 0;
    for (const ModelObject *object : model.objects) {
        REQUIRE(object->volumes.size() == 1);
        REQUIRE(object->instances.size() == 1);
        CHECK(object->volumes.front()->name == "Leaf");
        const BoundingBoxf3 bounds = object->instance_bounding_box(0);
        if (object->instances.front()->printable) {
            ++printable;
            CHECK_THAT(bounds.min.x(), Catch::Matchers::WithinAbs(0.0, 1e-6));
        } else {
            ++excluded;
            CHECK_THAT(bounds.min.x(), Catch::Matchers::WithinAbs(10.0, 1e-6));
        }
    }
    CHECK(printable == 1);
    CHECK(excluded == 1);
}

TEST_CASE("AMF constellations containing only excluded occurrences stay excluded", "[AMF][OriginalNames]")
{
    ScopedTemporaryFile tmp(".amf");
    write_file(tmp.string(), "<amf unit=\"millimeter\">" +
        named_amf_object(1, "ParentA", "LeafA") + named_amf_object(2, "ParentB", "LeafB") +
        "<constellation id=\"10\">"
        "<instance objectid=\"1\"><printable>0</printable></instance>"
        "<instance objectid=\"2\"><deltax>10</deltax><printable>0</printable></instance>"
        "</constellation></amf>");
    Model model;
    REQUIRE(load_amf_with_options(tmp.string(), model,
        LoadStrategy::LoadModel | LoadStrategy::AssembleParts | LoadStrategy::KeepOriginalNames));
    REQUIRE(model.objects.size() == 1);
    REQUIRE(model.objects.front()->volumes.size() == 2);
    REQUIRE(model.objects.front()->instances.size() == 1);
    CHECK_FALSE(model.objects.front()->instances.front()->printable);
}

TEST_CASE("AMF assembly import skips unresolved constellation references like the legacy loader", "[AMF][OriginalNames]")
{
    ScopedTemporaryFile tmp(".amf");
    write_file(tmp.string(), "<amf unit=\"millimeter\">" + named_amf_object(1, "Parent", "Leaf") +
        named_amf_object(2, "Other", "OtherLeaf") +
        "<constellation id=\"10\"><instance objectid=\"1\"/><instance objectid=\"999\"/>"
        "<instance objectid=\"2\"><deltax>10</deltax></instance></constellation></amf>");
    Model model;
    REQUIRE(load_amf_with_options(tmp.string(), model,
        LoadStrategy::LoadModel | LoadStrategy::AssembleParts | LoadStrategy::KeepOriginalNames));
    REQUIRE(model.objects.size() == 1);
    CHECK(model.objects.front()->volumes.size() == 2);
}

TEST_CASE("AMF assembly import rejects cyclic constellation references", "[AMF][OriginalNames]")
{
    ScopedTemporaryFile tmp(".amf");
    write_file(tmp.string(), "<amf unit=\"millimeter\">" + named_amf_object(1, "Parent", "Leaf") +
        "<constellation id=\"10\"><instance objectid=\"20\"/></constellation>"
        "<constellation id=\"20\"><instance objectid=\"10\"/><instance objectid=\"1\"/></constellation></amf>");
    Model model;
    CHECK_FALSE(load_amf_with_options(tmp.string(), model,
        LoadStrategy::LoadModel | LoadStrategy::AssembleParts | LoadStrategy::KeepOriginalNames));
}

TEST_CASE("AMF nested constellations flatten into one assembly with composed placements", "[AMF][OriginalNames]")
{
    ScopedTemporaryFile tmp(".amf");
    // Group 20 is placed twice inside group 10; its own occurrence offset adds to each placement.
    write_file(tmp.string(), "<amf unit=\"millimeter\">" +
        named_amf_object(1, "Base", "BaseBody") + named_amf_object(2, "Pin", "PinBody") +
        "<constellation id=\"10\"><instance objectid=\"1\"/>"
        "<instance objectid=\"20\"><deltax>10</deltax></instance>"
        "<instance objectid=\"20\"><deltax>20</deltax></instance></constellation>"
        "<constellation id=\"20\"><instance objectid=\"2\"><deltaz>5</deltaz></instance></constellation></amf>");
    Model model;
    REQUIRE(load_amf_with_options(tmp.string(), model,
        LoadStrategy::LoadModel | LoadStrategy::AssembleParts | LoadStrategy::KeepOriginalNames));
    // The nested group is not an assembly of its own.
    REQUIRE(model.objects.size() == 1);
    const auto &parts = model.objects.front()->volumes;
    REQUIRE(parts.size() == 3);
    CHECK(parts[0]->name == "BaseBody");
    CHECK(parts[1]->name == "PinBody");
    CHECK(parts[2]->name == "PinBody");
    CHECK_THAT(amf_part_bounds(*parts[1]).min.x() - amf_part_bounds(*parts[0]).min.x(), Catch::Matchers::WithinAbs(10.0, 1e-6));
    CHECK_THAT(amf_part_bounds(*parts[2]).min.x() - amf_part_bounds(*parts[0]).min.x(), Catch::Matchers::WithinAbs(20.0, 1e-6));
    CHECK_THAT(amf_part_bounds(*parts[1]).min.z() - amf_part_bounds(*parts[0]).min.z(), Catch::Matchers::WithinAbs(5.0, 1e-6));
}

TEST_CASE("AMF nested placements apply the parent transform to the child placement", "[AMF][OriginalNames]")
{
    ScopedTemporaryFile tmp(".amf");
    // The nested group is mirrored in X; the child's own +5 shift must be mirrored with it.
    write_file(tmp.string(), "<amf unit=\"millimeter\">" +
        named_amf_object(1, "Base", "BaseBody") + named_amf_object(2, "Pin", "PinBody") +
        "<constellation id=\"10\"><instance objectid=\"1\"/>"
        "<instance objectid=\"20\"><mirrorx>-1</mirrorx></instance></constellation>"
        "<constellation id=\"20\"><instance objectid=\"2\"><deltax>5</deltax></instance></constellation></amf>");
    Model model;
    REQUIRE(load_amf_with_options(tmp.string(), model,
        LoadStrategy::LoadModel | LoadStrategy::AssembleParts | LoadStrategy::KeepOriginalNames));
    REQUIRE(model.objects.size() == 1);
    REQUIRE(model.objects.front()->volumes.size() == 2);
    // parent * child puts the pin at x in [-6, -5]; child * parent would give [4, 5].
    CHECK_THAT(amf_part_bounds(*model.objects.front()->volumes[1]).max.x() -
               amf_part_bounds(*model.objects.front()->volumes[0]).min.x(), Catch::Matchers::WithinAbs(-5.0, 1e-6));
}

TEST_CASE("AMF nested printability multiplies into the top-level group", "[AMF][OriginalNames]")
{
    ScopedTemporaryFile tmp(".amf");
    write_file(tmp.string(), "<amf unit=\"millimeter\">" +
        named_amf_object(1, "Base", "BaseBody") + named_amf_object(2, "Pin", "PinBody") +
        "<constellation id=\"10\"><instance objectid=\"1\"/>"
        "<instance objectid=\"20\"><printable>0</printable></instance></constellation>"
        "<constellation id=\"20\"><instance objectid=\"2\"/></constellation></amf>");
    Model model;
    REQUIRE(load_amf_with_options(tmp.string(), model,
        LoadStrategy::LoadModel | LoadStrategy::AssembleParts | LoadStrategy::KeepOriginalNames));
    // A non-printable nested occurrence makes the group mixed, so its bodies stay separate objects.
    REQUIRE(model.objects.size() == 2);
    CHECK(model.objects[0]->instances.front()->printable);
    CHECK_FALSE(model.objects[1]->instances.front()->printable);
}

TEST_CASE("AMF assembly import loads files with duplicate or object-like constellation ids", "[AMF][OriginalNames]")
{
    ScopedTemporaryFile tmp(".amf");
    // Group 1 shares its id with object 1, and two groups share id 10: references still resolve to the object.
    write_file(tmp.string(), "<amf unit=\"millimeter\">" + named_amf_object(1, "Parent", "Leaf") +
        "<constellation id=\"10\"><instance objectid=\"1\"/></constellation>"
        "<constellation id=\"10\"><instance objectid=\"1\"><deltax>10</deltax></instance></constellation>"
        "<constellation id=\"1\"><instance objectid=\"1\"><deltax>20</deltax></instance></constellation></amf>");
    Model model;
    REQUIRE(load_amf_with_options(tmp.string(), model,
        LoadStrategy::LoadModel | LoadStrategy::AssembleParts | LoadStrategy::KeepOriginalNames));
    // Each single-occurrence group stays one object with its own placement.
    CHECK(model.objects.size() == 3);
}

TEST_CASE("AMF nesting that only multiplies work or depth is rejected", "[AMF][OriginalNames]")
{
    const bool deep = GENERATE(false, true);
    // The order of declaration must not let a deep chain slip past the cycle check.
    const bool leaf_first = GENERATE(false, true);
    CAPTURE(deep, leaf_first);
    ScopedTemporaryFile tmp(".amf");
    // Doubling chain: every group references the next twice and the last one reaches no body.
    // Deep chain: one reference per level, deeper than any real assembly.
    const int levels = deep ? 400 : 30;
    std::vector<std::string> chain;
    for (int level = 0; level < levels; ++level) {
        const std::string next = level + 1 < levels ? std::to_string(1000 + level + 1) : std::string("999");
        chain.push_back("<constellation id=\"" + std::to_string(1000 + level) + "\"><instance objectid=\"" + next + "\"/>" +
                        (deep ? std::string() : "<instance objectid=\"" + next + "\"/>") + "</constellation>");
    }
    if (leaf_first)
        std::reverse(chain.begin(), chain.end());
    std::string groups;
    for (const std::string &group : chain)
        groups += group;
    write_file(tmp.string(), "<amf unit=\"millimeter\">" + named_amf_object(1, "Parent", "Leaf") + groups + "</amf>");
    Model model;
    CHECK_FALSE(load_amf_with_options(tmp.string(), model,
        LoadStrategy::LoadModel | LoadStrategy::AssembleParts | LoadStrategy::KeepOriginalNames));
}

namespace {

// An internal DTD subset has to precede the root element, so splice it in ahead of the root tag.
std::string amf_with_dtd(const std::string &declarations, const std::string &object_metadata)
{
    std::string xml = amf_with_object_metadata(object_metadata);
    xml.insert(xml.find("<amf"), "<!DOCTYPE amf [\n" + declarations + "\n]>\n");
    return xml;
}

} // namespace

TEST_CASE("AMF rejects an XML entity amplification bomb", "[AMF][Security]")
{
    ScopedTemporaryFile tmp(".amf");
    // Seven levels expand to 10^7 characters, past expat's 8 MiB output and 100x amplification thresholds, yet a
    // build without the guard still parses that in well under a second instead of exhausting CI memory as ten would.
    std::string declarations = "<!ENTITY e1 \"AAAAAAAAAA\">";
    for (int level = 2; level <= 7; ++level) {
        std::string body;
        for (int repeat = 0; repeat < 10; ++repeat)
            body += "&e" + std::to_string(level - 1) + ";";
        declarations += "\n<!ENTITY e" + std::to_string(level) + " \"" + body + "\">";
    }
    write_file(tmp.string(), amf_with_dtd(declarations, "<metadata type=\"name\">&e7;</metadata>"));

    Model model;
    CHECK_FALSE(load(tmp.string(), model));
}

TEST_CASE("AMF still expands an ordinary internal entity", "[AMF][Security]")
{
    // The amplification cap must not turn entity support off: ordinary AMF files keep parsing.
    ScopedTemporaryFile tmp(".amf");
    write_file(tmp.string(), amf_with_dtd("<!ENTITY name \"hello\">", "<metadata type=\"name\">&name;</metadata>"));

    Model model;
    REQUIRE(load(tmp.string(), model));
    REQUIRE(model.objects.size() == 1);
    CHECK(model.objects.front()->name == "hello");
}

TEST_CASE("AMF assemblies are named after the file, with suffixes for later groups", "[AMF][OriginalNames]")
{
    ScopedTemporaryFile tmp(".amf");
    write_file(tmp.string(), "<amf unit=\"millimeter\">" +
        named_amf_object(1, "First", "FirstBody") + named_amf_object(2, "Second", "SecondBody") +
        "<constellation id=\"10\"><instance objectid=\"1\"/><instance objectid=\"2\"><deltax>10</deltax></instance></constellation>"
        "<constellation id=\"20\"><instance objectid=\"1\"/><instance objectid=\"2\"><deltax>10</deltax></instance></constellation>"
        "</amf>");
    Model model;
    REQUIRE(load_amf_with_options(tmp.string(), model,
        LoadStrategy::LoadModel | LoadStrategy::AssembleParts | LoadStrategy::KeepOriginalNames));
    REQUIRE(model.objects.size() == 2);
    const std::string file_name = boost::filesystem::path(tmp.string()).filename().string();
    CHECK(model.objects[0]->name == file_name);
    CHECK(model.objects[1]->name == file_name + "_2");
    // The first body name is a part name, never the assembly name.
    CHECK(model.objects[0]->volumes.front()->name == "FirstBody");
}

TEST_CASE("AMF preserves equal valid names belonging to different bodies", "[AMF][OriginalNames]")
{
    ScopedTemporaryFile tmp(".amf");
    write_file(tmp.string(), "<amf unit=\"millimeter\">" +
        named_amf_object(1, "ParentA", "Same") + named_amf_object(2, "ParentB", "Same") +
        "<constellation id=\"10\"><instance objectid=\"1\"/>"
        "<instance objectid=\"2\"><deltaz>7</deltaz></instance></constellation></amf>");
    Model model;
    REQUIRE(load_amf_with_options(tmp.string(), model,
        LoadStrategy::LoadModel | LoadStrategy::AssembleParts | LoadStrategy::KeepOriginalNames));
    REQUIRE(model.objects.size() == 1);
    REQUIRE(model.objects.front()->volumes.size() == 2);
    CHECK(model.objects.front()->volumes[0]->name == "Same");
    CHECK(model.objects.front()->volumes[1]->name == "Same");
    CHECK_THAT(amf_part_bounds(*model.objects.front()->volumes[1]).min.z() -
               amf_part_bounds(*model.objects.front()->volumes[0]).min.z(), Catch::Matchers::WithinAbs(7.0, 1e-6));
}

TEST_CASE("AMF standalone multipart objects keep their volume metadata and definition order", "[AMF][OriginalNames]")
{
    ScopedTemporaryFile tmp(".amf");
    std::string multipart = named_amf_object(9, "FirstDefinition", "FirstPart");
    const size_t volume_begin = multipart.find("<volume>");
    std::string second_volume = multipart.substr(volume_begin,
        multipart.find("</volume>", volume_begin) + std::string("</volume>").size() - volume_begin);
    second_volume.replace(second_volume.find("FirstPart"), std::string("FirstPart").size(), "SecondPart");
    multipart.insert(multipart.find("</mesh>"), second_volume);
    // Numeric IDs deliberately disagree with file order; neither object belongs to a constellation.
    write_file(tmp.string(), "<amf unit=\"millimeter\">" + multipart +
        named_amf_object(1, "SecondDefinition", "LastPart") + "</amf>");
    Model model;
    REQUIRE(load_amf_with_options(tmp.string(), model,
        LoadStrategy::LoadModel | LoadStrategy::AssembleParts | LoadStrategy::KeepOriginalNames));
    REQUIRE(model.objects.size() == 2);
    CHECK(model.objects[0]->name == "FirstDefinition");
    REQUIRE(model.objects[0]->volumes.size() == 2);
    CHECK(model.objects[0]->volumes[0]->name == "FirstPart");
    CHECK(model.objects[0]->volumes[1]->name == "SecondPart");
    CHECK(model.objects[1]->name == "SecondDefinition");
    REQUIRE(model.objects[1]->volumes.size() == 1);
    CHECK(model.objects[1]->volumes.front()->name == "LastPart");
}

TEST_CASE("Unnamed AMF assembly parts use the existing filename fallback", "[AMF][OriginalNames]")
{
    const bool keep_names = GENERATE(false, true);
    ScopedTemporaryFile tmp(".amf");
    std::string objects = named_amf_object(1, "", "") + named_amf_object(2, "", "");
    const std::string empty_metadata = "<metadata type=\"name\"></metadata>";
    // No name metadata at either level: this matches the user's unmodified CAD AMF export.
    for (size_t pos = objects.find(empty_metadata); pos != std::string::npos; pos = objects.find(empty_metadata))
        objects.erase(pos, empty_metadata.size());
    write_file(tmp.string(), "<amf unit=\"millimeter\">" + objects +
        "<constellation id=\"10\"><instance objectid=\"1\"/>"
        "<instance objectid=\"2\"><deltax>10</deltax></instance></constellation></amf>");
    LoadStrategy options = LoadStrategy::LoadModel | LoadStrategy::AssembleParts;
    if (keep_names)
        options = options | LoadStrategy::KeepOriginalNames;
    Model model;
    REQUIRE(load_amf_with_options(tmp.string(), model, options));
    REQUIRE(model.objects.size() == 1);
    REQUIRE(model.objects.front()->volumes.size() == 2);
    const std::string fallback = boost::filesystem::path(tmp.string()).filename().string();
    CHECK(model.objects.front()->name == fallback);
    CHECK(model.objects.front()->volumes[0]->name == fallback);
    CHECK(model.objects.front()->volumes[1]->name == fallback);
    // Later splitting existing bodies into objects must retain these same ordinary names.
    ModelObjectPtrs separated;
    model.objects.front()->split(&separated, false);
    REQUIRE(separated.size() == 2);
    CHECK(separated[0]->name == fallback);
    CHECK(separated[1]->name == fallback);
}

TEST_CASE("AMF assemblies have no part settings of their own", "[AMF][OriginalNames]")
{
    ScopedTemporaryFile tmp(".amf");
    write_file(tmp.string(), "<amf unit=\"millimeter\">" +
        named_amf_object(0, "ParentA", "LeafA") + named_amf_object(1, "ParentB", "LeafB") +
        "<constellation id=\"1\"><instance objectid=\"0\"/><instance objectid=\"0\"><deltax>10</deltax></instance>"
        "<instance objectid=\"1\"><deltax>20</deltax></instance></constellation></amf>");
    Model model;
    REQUIRE(load_amf_with_options(tmp.string(), model,
        LoadStrategy::LoadModel | LoadStrategy::AssembleParts | LoadStrategy::KeepOriginalNames));
    REQUIRE(model.objects.size() == 1);
    ModelObject& object = *model.objects.front();
    REQUIRE(object.volumes.size() == 3);
    // Merging copies the object's filament to every part; repeating it is not a setting of the part.
    CHECK_FALSE(object.has_individual_part_settings());
    object.imported_parts_hidden = true;
    CHECK(object.hides_imported_parts());

    object.volumes[1]->config.set("extruder", 2);
    CHECK(object.has_individual_part_settings());
    CHECK_FALSE(object.hides_imported_parts());
}
