// Aviation Dynamics — tests: the São Paulo region's destination airports,
// their ground, their ILSs and GPS approach routes.
// SPDX-License-Identifier: MIT
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

#include "airports.hpp"
#include "avd/gps.hpp"
#include "avd/json.hpp"
#include "avd/units.hpp"
#include "avd_test.hpp"

using namespace avd;
using namespace avd::scenarios;
using namespace avd::units;

namespace {

const Airport& airport(const std::string& icao) {
    for (const Airport& a : saoPauloRegionAirports())
        if (a.icao == icao) return a;
    static const Airport none;
    return none;
}

}  // namespace

TEST_CASE("destinations: region airports carry their published runways and elevations") {
    const auto& list = saoPauloRegionAirports();
    CHECK(list.size() == 6);
    CHECK(list[0].icao == "SBSP" && list[1].icao == "SBMT");
    // Guarulhos: 3,700 x 45 m (10L/28R) and 3,000 x 45 m (10R/28L), 2,461 ft;
    // 10L runs about 074 deg true.
    const Airport& gr = airport("SBGR");
    CHECK(gr.runways.size() == 4);
    CHECK_NEAR(gr.runways[0].length, 3700, 40);
    CHECK_NEAR(gr.runways[2].length, 3000, 40);
    CHECK_NEAR(gr.elevation, 750.1, 0.5);
    CHECK_NEAR(radToDeg(gr.runways[0].headingTrue), 74, 1.5);
    CHECK_NEAR(radToDeg(wrapPi(gr.runways[1].headingTrue - gr.runways[0].headingTrue)), 180, 1e-6);
    // Viracopos 15/33: 3,240 m, a 300 ft displaced threshold on 33.
    const Airport& kp = airport("SBKP");
    CHECK_NEAR(kp.runways[0].length, 3240, 40);
    CHECK_NEAR(kp.runways[1].displaced, 91.4, 0.1);
    for (const Airport& a : list) {
        CHECK(a.runways.size() % 2 == 0);
        CHECK(!a.source.empty());
        for (const IlsDef& i : a.ils) CHECK(!i.source.empty() && i.khz >= 108100 && i.khz <= 111950);
    }
}

TEST_CASE("destinations: one ground holds every field at its own elevation, blended between") {
    const auto& list = saoPauloRegionAirports();
    const auto ground = makeRegionGround(list);
    CHECK(ground->fields().size() == list.size());
    Real lo = 1e9, hi = -1e9;
    for (const Airport& a : list) {
        lo = std::min(lo, a.elevation);
        hi = std::max(hi, a.elevation);
        // On each runway: asphalt at the field elevation, level.
        const RunwayDef& r = a.runways[0];
        const Vec3 mid = runwayPoint(r, 0.5 * r.length);
        const GroundHit h = ground->below({mid.x, mid.y, -3000});
        CHECK(h.hit);
        CHECK_NEAR(-h.point.z, a.elevation, 1e-6);
        CHECK(h.surface.id == surfaces::kAsphalt.id);
        CHECK_NEAR(h.normal.z, -1.0, 1e-9);
        // Beside it, still in the field: grass, same level.
        const Vec3 side = runwayPoint(r, 0.5 * r.length, 200);
        const GroundHit g = ground->below({side.x, side.y, -3000});
        CHECK(g.surface.id == surfaces::kGrass.id);
        CHECK_NEAR(-g.point.z, a.elevation, 1e-6);
    }
    // Between Congonhas (802 m) and Guarulhos (750 m): in between, continuous
    // and gently sloped (under 2 %).
    const Vec3 a = runwayPoint(list[0].runways[0], 900), b = runwayPoint(airport("SBGR").runways[0], 1800);
    Real prev = -ground->terrainDown(a.x, a.y), maxSlope = 0;
    for (int k = 1; k <= 400; ++k) {
        const Vec3 p = a + (b - a) * (k / 400.0);
        const Real e = -ground->terrainDown(p.x, p.y);
        CHECK_RANGE(e, lo - 1e-6, hi + 1e-6);
        maxSlope = std::max(maxSlope, std::abs(e - prev) / (length(b - a) / 400));
        prev = e;
    }
    CHECK(maxSlope < 0.02);
    // A slanted ray finds the blended terrain.
    const Vec3 o{(a.x + b.x) / 2, (a.y + b.y) / 2, -1500};
    const Vec3 dir = normalized(Vec3{1, 1, 1});
    const GroundHit h = ground->raycast(o, dir, 1e5);
    CHECK(h.hit);
    CHECK_NEAR(h.point.z, ground->terrainDown(h.point.x, h.point.y), 0.05);
}

TEST_CASE("destinations: every ILS of the region is centred on its own runway's 3 deg path") {
    const NavEnvironment& env = saoPauloNavEnvironment();
    const LocalFrame f = saoPauloWorldFrame();
    int count = 0;
    for (const Airport& a : saoPauloRegionAirports()) {
        for (const IlsDef& i : a.ils) {
            const RunwayDef& r = a.runways[static_cast<std::size_t>(i.runway)];
            // 6 NM out on the extended centreline, on the glidepath from the GS antenna.
            const Real out = 6 * 1852;
            Geodetic g = f.geodeticFromNed(runwayPoint(r, r.displaced - out));
            // The glidepath is an angle at the antenna: over a curved Earth the
            // path stands d^2 / 2R higher (10 m at 6 NM) than a flat one.
            const Real d = out + 300;
            g.height = a.elevation + 2 + d * std::tan(degToRad(3)) + d * d / (2 * 6.371e6);
            const NavIndication n = evaluateNav(*env.database, i.khz, 0, g);
            CHECK(n.valid && n.localizer && n.ident == i.ident);
            CHECK_NEAR(n.cdi, 0, 0.03);
            CHECK(n.glideslopeValid);
            CHECK_NEAR(n.glideslope, 0, 0.05);
            // Close in (0.4 NM, 55 m left of the centreline): the glideslope's
            // coverage is about the localizer course, so it is still received.
            Geodetic near = f.geodeticFromNed(runwayPoint(r, r.displaced - 740, -55));
            near.height = a.elevation + 2 + 1040 * std::tan(degToRad(3));
            CHECK(evaluateNav(*env.database, i.khz, 0, near).glideslopeValid);
            // The localizer is directional: nothing on the reciprocal's final.
            Geodetic back = f.geodeticFromNed(runwayPoint(r, r.length + out));
            back.height = g.height;
            const NavIndication nb = evaluateNav(*env.database, i.khz, 0, back);
            CHECK(!(nb.valid && nb.ident == i.ident && nb.glideslopeValid));
            ++count;
        }
    }
    CHECK(count == 8);  // SBSP 2, SBMT 1 (synthetic), SBGR 4, SBKP 1
}

TEST_CASE("gps: an approach route from the side gets an initial fix; flown, it sequences to the runway on the path") {
    const Airport& gr = airport("SBGR");
    const RunwayDef& r = gr.runways[0];  // 10L
    // From São José dos Campos, east of Guarulhos: arriving opposite to the
    // 10L final course.
    const Airport& sp = airport("SBSJ");
    int faf = -1;
    const auto route = approachRoute(gr, 0, {}, faf, &sp.runways[0]);
    CHECK(route.size() == 6);  // RW16, DP16, IA10L, IF10L, FF10L, RW10L
    CHECK(route.front().ident == "RW16" && route.back().ident == "RW10L");
    // From Congonhas, south-west: within 60 deg of the final course, no IAF.
    int faf2 = -1;
    CHECK(approachRoute(gr, 0, {}, faf2, &saoPauloRegionAirports()[0].runways[0]).size() == 5);
    CHECK(route[faf].ident == "FF10L");
    CHECK(route[2].ident == "IA10L");
    const Vec3 thr = landingThreshold(r);
    CHECK_NEAR(length(route[faf].position - thr), 5 * 1852, 1);
    CHECK_NEAR(length(route[faf - 1].position - thr), 10 * 1852, 1);
    CHECK_NEAR(route[faf].altitudeMsl - gr.elevation, 15.24 + 5 * 1852 * std::tan(0.0524), 0.5);

    GpsNavigator gps;
    gps.setRoute(route, faf);
    // A point-mass airplane at 70 m/s steering the GPS course with an intercept
    // proportional to the cross-track error (30 deg max, 3 deg/s turns) and,
    // once the vertical path is valid, holding it.
    Vec3 p = landingThreshold(sp.runways[0]);
    Real hdg = sp.runways[0].headingTrue, alt = sp.elevation + 10;
    const Real v = 70, dt = 0.5;
    int lastActive = 1, maxActive = 1;
    bool monotonic = true;
    Real worstFinalXtk = 0, worstFinalVdev = 0;
    for (int i = 0; i < 2 * 7200; ++i) {
        const GpsIndication& g = gps.update(p, alt, v);
        CHECK(g.valid);
        if (g.active < lastActive) monotonic = false;
        lastActive = g.active;
        maxActive = std::max(maxActive, g.active);
        const Real want = g.desiredTrack - std::clamp(g.crossTrack / 1000, -0.52, 0.52);
        hdg += std::clamp(wrapPi(want - hdg), -0.026, 0.026);
        p = p + Vec3{std::cos(hdg), std::sin(hdg), 0} * (v * dt);
        if (g.verticalValid) alt -= std::clamp(g.verticalDeviation, -3.0, 3.0) + v * dt * std::tan(0.0524);
        else alt = std::min(alt + 5 * dt, route[faf - 1].altitudeMsl);
        if (g.active == faf + 1 && g.distanceToWaypoint < 3 * 1852) {
            worstFinalXtk = std::max(worstFinalXtk, std::abs(g.crossTrack));
            worstFinalVdev = std::max(worstFinalVdev, std::abs(g.verticalDeviation));
            CHECK(g.mode == "APR");
        }
        if (g.active == static_cast<int>(route.size()) - 1 && g.distanceToWaypoint < 100) break;
    }
    CHECK(monotonic);
    CHECK(maxActive == static_cast<int>(route.size()) - 1);
    CHECK(length(Vec3{p.x - thr.x, p.y - thr.y, 0}) < 150);
    CHECK(worstFinalXtk < 30);
    CHECK(worstFinalVdev < 10);
}

TEST_CASE("gps: CDI sense and full scale; vertical needle sense") {
    std::vector<RouteWaypoint> route{{"A", {0, 0, 0}}, {"B", {100000, 0, 0}}, {"C", {110000, 0, -700}, -1, true}};
    GpsNavigator gps;
    gps.setRoute(route, 1);
    // 1 NM east (right) of a northbound course, 50 NM out: en route, ±2 NM.
    GpsIndication g = gps.update({10000, 1852, -1000}, 1000, 60);
    CHECK(g.mode == "ENR");
    CHECK_NEAR(g.fullScale, 2 * 1852, 1e-6);
    CHECK_NEAR(g.crossTrack, 1852, 1e-6);
    CHECK_NEAR(g.cdi, -0.5, 1e-6);  // course to the left: needle left
    CHECK(!g.verticalValid);
    // Inside 30 NM: terminal, ±1 NM; 1 NM before the FAF: 0.65 NM.
    g = gps.update({60000, -900, -1000}, 1000, 60);
    CHECK(g.mode == "TERM" && std::abs(g.fullScale - 1852) < 1e-6 && g.cdi > 0);
    g = gps.update({100000 - 1852, 0, -1000}, 1000, 60);
    CHECK(g.mode == "APR");
    CHECK_NEAR(g.fullScale, 0.65 * 1852, 1);
    // Past the FAF: high on the path -> needle down; 3 deg to 700 m + 15.24 m.
    g = gps.update({105000, 0, -1200}, 1200, 60);
    CHECK(g.active == 2 && g.verticalValid);
    CHECK_NEAR(g.verticalDeviation, 1200 - (700 + 15.24 + 5000 * std::tan(0.0524)), 1e-6);
    CHECK(g.vdi < 0);
}

TEST_CASE("destinations: every Brazilian airport record loads with its runways at their published lengths") {
    std::ifstream in(std::string(AVD_DATA_DIR) + "/world/airports_br.json");
    std::stringstream ss;
    ss << in.rdbuf();
    const json::ParseResult pr = json::parse(ss.str());
    CHECK(pr.ok());
    const json::Value* list = pr.value.find("airports");
    CHECK(list && list->isArray() && list->items().size() > 1000);
    const LocalFrame f = saoPauloWorldFrame();
    int ends = 0, ils = 0, bad = 0;
    for (const json::Value& rec : list->items()) {
        const Airport a = airportFromRecord(rec, f);
        const json::Value* rw = rec.find("rw");
        if (a.icao.empty() || a.runways.size() != 2 * rw->items().size()) ++bad;
        for (std::size_t k = 0; k < rw->items().size(); ++k) {
            const json::Value& r = rw->items()[k];
            if (std::abs(a.runways[2 * k].length - r.find("len")->asNumber() * 0.3048) > 1) ++bad;
        }
        // Every runway end points where its number says (magnetic heading /
        // 10, with Brazil's variation of 12-24 deg W: the regional 21.5 deg
        // estimate within 15 deg).
        for (const RunwayDef& r : a.runways) {
            const Real fromNumber = std::atof(r.ident.c_str()) * 10 - 21.5;
            if (std::abs(radToDeg(wrapPi(r.headingTrue - degToRad(fromNumber)))) > 15) ++bad;
        }
        ends += static_cast<int>(a.runways.size());
        ils += static_cast<int>(a.ils.size());
    }
    CHECK(bad == 0);
    CHECK(ends > 2000);
    CHECK(ils >= 10);
}

TEST_CASE("destinations: far airports (Galeão, Brasília) carry ILSs centred on their own runways") {
    std::ifstream in(std::string(AVD_DATA_DIR) + "/world/airports_br.json");
    std::stringstream ss;
    ss << in.rdbuf();
    const json::ParseResult pr = json::parse(ss.str());
    const json::Value* list = pr.value.find("airports");
    const LocalFrame f = saoPauloWorldFrame();
    int checked = 0;
    for (const json::Value& rec : list->items()) {
        const std::string icao = rec.find("icao")->asString();
        if (icao != "SBGL" && icao != "SBBR") continue;
        const Airport a = airportFromRecord(rec, f);
        NavDatabase db;
        addIlsNavaids(db, a, f);
        CHECK(!a.ils.empty());
        for (const IlsDef& i : a.ils) {
            const RunwayDef& r = a.runways[static_cast<std::size_t>(i.runway)];
            const Real d = 6 * 1852 + 300;
            Geodetic g = f.geodeticFromNed(runwayPoint(r, r.displaced - 6 * 1852));
            g.height = a.elevation + 2 + d * std::tan(degToRad(3)) + d * d / (2 * 6.371e6);
            const NavIndication n = evaluateNav(db, i.khz, 0, g);
            CHECK(n.valid && n.localizer && n.ident == i.ident);
            CHECK_NEAR(n.cdi, 0, 0.03);
            CHECK(n.glideslopeValid);
            CHECK_NEAR(n.glideslope, 0, 0.05);
            ++checked;
        }
        // A route from Congonhas: the GPS sequences it from the runway.
        int faf = -1;
        const auto route = approachRoute(a, a.ils[0].runway, {}, faf, &saoPauloRegionAirports()[0].runways[0]);
        CHECK(route.size() >= 5 && faf == static_cast<int>(route.size()) - 2);
    }
    CHECK(checked == 7);  // SBGL 3, SBBR 4
}
