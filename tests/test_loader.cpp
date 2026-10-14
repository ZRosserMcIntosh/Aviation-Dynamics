// Aviation Dynamics — JSON parser, aircraft-definition loader and validator.
// SPDX-License-Identifier: MIT
#include <fstream>
#include <sstream>

#include "avd/aircraft.hpp"
#include "avd/aircraft_loader.hpp"
#include "avd/json.hpp"
#include "avd_test.hpp"

using namespace avd;

namespace {

std::string referenceText() {
    std::ifstream f(std::string(AVD_DATA_DIR) + "/aircraft/tupi_t180.json");
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

bool anyContains(const std::vector<std::string>& v, const std::string& s) {
    for (const auto& e : v)
        if (e.find(s) != std::string::npos) return true;
    return false;
}

std::string replaceOnce(std::string s, const std::string& from, const std::string& to) {
    const auto p = s.find(from);
    if (p != std::string::npos) s.replace(p, from.size(), to);
    return s;
}

}  // namespace

TEST_CASE("json: parses the grammar and reports positions") {
    const auto r = json::parse(R"({"a": [1, -2.5e2, true, null, "xé"], "b": {"c": false}})");
    CHECK(r.ok());
    CHECK(r.value.find("a")->items().size() == 5);
    CHECK_NEAR(r.value.find("a")->items()[1].asNumber(), -250, 0);
    CHECK(r.value.find("a")->items()[4].asString() == "x\xc3\xa9");
    const auto bad = json::parse("{\n  \"a\": [1, 2,,]\n}");
    CHECK(!bad.ok());
    CHECK(bad.error.find("line 2") != std::string::npos);
    CHECK(!json::parse(R"({"a": 1, "a": 2})").ok());  // duplicate keys rejected
    CHECK(!json::parse("[1] x").ok());
}

TEST_CASE("loader: the reference aircraft loads and validates") {
    const LoadResult r = loadAircraftFromJson(referenceText());
    for (const auto& e : r.errors) std::cerr << "  " << e << "\n";
    CHECK(r.ok());
    const AircraftDefinition& d = r.definition;
    CHECK(d.id == "tupi_t180");
    CHECK(d.surfaces.size() == 3);
    CHECK(d.engines.size() == 1);
    CHECK(d.fuel.tanks.size() == 2);
    CHECK_NEAR(d.engines[0].ratedPower, 134200, 1e-6);
    CHECK_NEAR(d.surfaces[0].panels[0].dihedral, 1.73 * kPi / 180, 1e-12);
    CHECK_NEAR(d.fuel.tanks[0].capacity, 0.106, 1e-12);
    CHECK(!d.provenance.empty());
    CHECK(!d.sources.empty());
}

TEST_CASE("loader: unknown keys, wrong types and missing fields are errors with paths") {
    const std::string base = referenceText();
    const auto typo = loadAircraftFromJson(replaceOnce(base, "\"rated_rpm\"", "\"rated_rmp\""));
    CHECK(anyContains(typo.errors, "engines[0].rated_rmp: unknown key"));
    CHECK(anyContains(typo.errors, "engines[0].rated_rpm: required"));
    const auto type = loadAircraftFromJson(replaceOnce(base, "\"empty_kg\": 754.3", "\"empty_kg\": \"heavy\""));
    CHECK(anyContains(type.errors, "mass.empty_kg: must be a number"));
    const auto vec = loadAircraftFromJson(replaceOnce(base, "\"empty_cg_m\": [-1.006, 0.0, -0.20]", "\"empty_cg_m\": [1, 2]"));
    CHECK(anyContains(vec.errors, "mass.empty_cg_m"));
    const auto chan = loadAircraftFromJson(replaceOnce(base, "\"channel\": \"rudder\"", "\"channel\": \"tail wheel\""));
    CHECK(anyContains(chan.errors, "channel: must be one of"));
}

TEST_CASE("validator: physically impossible values are rejected") {
    const AircraftDefinition good = loadAircraftFromJson(referenceText()).definition;
    CHECK(good.validate().empty());
    auto d = good;
    d.emptyMass = -5;
    CHECK(anyContains(d.validate(), "emptyMass: must be > 0"));
    d = good;
    d.emptyInertiaDiag = {100, 100, 1000};  // violates the triangle inequality
    CHECK(anyContains(d.validate(), "triangle inequality"));
    d = good;
    d.engines[0].propeller.diameter = std::nan("");
    CHECK(anyContains(d.validate(), "engines[0].propeller.diameter"));
    d = good;
    d.surfaces[1].downwashSource = 2;  // must be an earlier surface
    CHECK(anyContains(d.validate(), "surfaces[1].downwashSource"));
    d = good;
    d.fuel.tanks[0].initial = 1.0;  // more than capacity
    CHECK(anyContains(d.validate(), "fuel.tanks[0].initial"));
    d = good;
    d.schemaVersion = 2;
    CHECK(anyContains(d.validate(), "schemaVersion"));
    // The Aircraft constructor refuses invalid definitions.
    bool threw = false;
    try {
        d = good;
        d.gear[0].stiffness = 0;
        Aircraft ac(d);
    } catch (const std::invalid_argument& e) {
        threw = std::string(e.what()).find("gear[0].stiffness") != std::string::npos;
    }
    CHECK(threw);
}
