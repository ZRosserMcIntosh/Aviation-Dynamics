// Aviation Dynamics — validation against published data: the Cessna 172S
// POH for the Tupi T-180, published and representative G650ER figures for
// the Top G.
// SPDX-License-Identifier: MIT
//
// The rows, their sources and tolerances live in src/scenarios/validation.cpp
// (shared with `avdemo validate` and the docs generator). These tests guard
// calibration: they are evidence of agreement with a handful of published
// performance points, not of manufacturer-level fidelity. A failing row is
// reported, never tuned away here: see docs/validation/*.md.
//
// With AVD_VALIDATION_JSON=<path> in the environment the rows that ran are
// written there at exit (tools/update_validation_docs.py reads it; the file
// says whether every row ran, so a filtered run cannot rewrite the docs).
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <set>

#include "avd/units.hpp"
#include "avd_test.hpp"
#include "scenarios.hpp"
#include "validation.hpp"

using namespace avd;
using namespace avd::scenarios;
using namespace avd::units;
namespace val = avd::scenarios::validation;

namespace {

struct Sink {
    val::Rows rows;
    ~Sink() {
        const char* path = std::getenv("AVD_VALIDATION_JSON");
        if (!path || rows.empty()) return;
        std::set<std::string> have;
        for (const val::Row& r : rows) have.insert(r.id);
        bool complete = true;
        for (const std::string& id : val::allIds()) complete = complete && have.count(id);
        std::ofstream f(path);
        val::writeJson(f, rows, complete);
        std::cerr << "validation report (" << rows.size() << " rows" << (complete ? "" : ", INCOMPLETE") << "): " << path << "\n";
    }
};
Sink& sink() {
    static Sink s;
    return s;
}

// Assert every row of a group and keep it for the report.
void expect(val::Rows rows) {
    for (const val::Row& r : rows) {
        if (!r.pass()) {
            ++avdtest::failures();
            std::cerr << "  FAILED validation row " << r.id << ": " << r.quantity << " = " << r.value << " " << r.unit
                      << ", published " << r.published << " (band " << r.low << ".." << r.high << " " << r.unit << ")\n";
        } else if (r.kind != val::Row::Kind::Info) {
            std::cout << "    " << r.id << ": " << r.value << " " << r.unit << " vs " << r.published << "\n";
        }
        sink().rows.push_back(r);
    }
}

const AircraftDefinition& ref() {
    static const AircraftDefinition d = loadReference();
    return d;
}

}  // namespace

TEST_CASE("validation: gross weight loading is inside the published CG envelope") {
    Aircraft ac(ref());
    ac.setStationMass(0, 170);
    ac.setStationMass(1, ref().maxTakeoffMass - ac.massProperties().mass);
    const Real m = ac.massProperties().mass, x = ac.massProperties().cg.x;
    CHECK_NEAR(m, ref().maxTakeoffMass, 0.5);
    CHECK(x <= ref().cgForwardLimitVsMass(m));  // structural x is negative aft
    CHECK(x >= ref().cgAftLimitVsMass(m));
    // Inertia totals near the commonly quoted C172 values (Estimated, +-25%).
    CHECK_RANGE(ac.massProperties().inertia.m[0][0], 1285 * 0.75, 1285 * 1.25);
    CHECK_RANGE(ac.massProperties().inertia.m[1][1], 1825 * 0.75, 1825 * 1.25);
    CHECK_RANGE(ac.massProperties().inertia.m[2][2], 2667 * 0.75, 2667 * 1.25);
}

TEST_CASE("validation: stall speeds (POH 53 KCAS flaps up, 48 KCAS flaps 30) and the warning margin") {
    expect(val::c172Stalls());
}

TEST_CASE("validation: maximum level speed at sea level (POH 126 KTAS) below the red line") {
    expect(val::c172MaxLevelSpeed());
}

TEST_CASE("validation: full-throttle cruise at 8000 ft (POH 124 KTAS at 75% power)") {
    expect(val::c172FullThrottle8000());
}

TEST_CASE("validation: rate of climb at sea level, 4,000, 8,000 and 12,000 ft; time, fuel and distance to 8,000 ft (POH Figure 5-7)") {
    expect(val::c172ClimbRates());
}

TEST_CASE("validation: cruise table at 2,000 and 8,000 ft, 55/65/75 % power (POH Figure 5-8)") {
    expect(val::c172CruiseTable());
}

TEST_CASE("validation: best glide at 68 KIAS is about 9:1 (POH maximum glide, 1.5 NM per 1,000 ft)") {
    expect(val::c172BestGlide());
}

TEST_CASE("validation: short-field take-off (POH 960 ft ground roll, 1630 ft over 50 ft)") {
    expect(val::c172Takeoff());
}

TEST_CASE("validation: static rpm within the published full-throttle static range (2300-2420)") {
    expect(val::c172StaticRpm());
}

TEST_CASE("validation: landing roll with a pilot's braking technique (POH 575 ft)") {
    // +-25%: the POH roll is a demonstrated landing with an unpublished
    // technique, and the nose-high roll puts the full-flap wing just past its
    // stall in ground effect, a regime no data here constrains.
    const val::Rows rows = val::c172LandingRoll();
    expect(rows);
    // The tyre-limited bound stops sooner, and harder braking never lengthens the roll.
    CHECK(rows[1].value < rows[0].value);
    BrakingTechnique soft = BrakingTechnique::pilot();
    soft.effort = 0.4;
    CHECK(mToFt(landingRollTest(ref(), 50, kDefaultDt, soft)) > rows[0].value);
}

TEST_CASE("validation: Top G time to climb to FL410 at mid weight (representative 20-25 min, +-25 %)") {
    expect(val::topGTimeToClimb());
}

TEST_CASE("validation: Top G high-speed cruise Mach 0.90 at FL410 is available; FL450 Mach 0.85 fuel flow") {
    expect(val::topGMaxCruise());
}

TEST_CASE("validation: Top G Vref at 27.5 t and at the maximum landing weight (representative 115-125 KIAS)") {
    expect(val::topGVref());
}

TEST_CASE("validation: Top G landing distance from 50 ft at the maximum landing weight (published 3,000 ft)") {
    expect(val::topGLandingDistance());
}

TEST_CASE("validation: the report lists every row the docs expect") {
    // allIds() is the contract between the rows and tools/update_validation_docs.py.
    std::set<std::string> ids;
    for (const std::string& id : val::allIds()) CHECK(ids.insert(id).second);  // no duplicates
    std::set<std::string> ran;
    for (const val::Row& r : sink().rows) ran.insert(r.id);
    // Only meaningful when the whole group ran (an unfiltered run); a filtered
    // run has fewer rows and the JSON is then marked incomplete.
    for (const std::string& id : ran) CHECK(ids.count(id) == 1);
}
