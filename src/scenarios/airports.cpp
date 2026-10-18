// Aviation Dynamics — scenarios.
// SPDX-License-Identifier: MIT
#include "airports.hpp"

#include "avd/units.hpp"

namespace avd::scenarios {

namespace {
Geodetic deg(Real lat, Real lon, Real h = 0) { return {units::degToRad(lat), units::degToRad(lon), h}; }
}  // namespace

LocalFrame saoPauloWorldFrame() { return LocalFrame(deg(-23.5611, -46.6558, 0.0)); }

Airport campoDeMarte(const LocalFrame& frame) {
    Airport a;
    a.icao = "SBMT";
    a.name = "Campo de Marte";
    a.elevation = 722.0;
    a.magneticVariation = units::degToRad(-21.5);  // ~21-22 deg W in 2026 (estimated)
    const Vec3 thr12 = frame.nedFromGeodetic(deg(-23.508333, -46.6425, 0));
    const Vec3 thr30 = frame.nedFromGeodetic(deg(-23.510333, -46.631333, 0));
    const Vec3 d = thr30 - thr12;
    const Real heading = std::atan2(d.y, d.x);
    const Vec3 dir{std::cos(heading), std::sin(heading), 0};
    RunwayDef r12;
    r12.ident = "12";
    r12.headingTrue = heading;
    r12.length = 1600;
    r12.width = 45;
    // Paved end of runway 12: 300 m before its displaced threshold.
    r12.threshold = thr12 - dir * 300.0;
    r12.threshold.z = -a.elevation;
    r12.surface = surfaces::kAsphalt;
    a.runways.push_back(r12);
    RunwayDef r30 = r12;
    r30.ident = "30";
    r30.headingTrue = wrapPi(heading + kPi);
    r30.threshold = r12.threshold + dir * r12.length;
    r30.threshold.z = -a.elevation;
    a.runways.push_back(r30);
    a.ilsKhz = 110300;
    return a;
}

Airport congonhas(const LocalFrame& frame) {
    Airport a;
    a.icao = "SBSP";
    a.name = "Congonhas";
    a.elevation = 802.0;
    a.magneticVariation = units::degToRad(-21.5);
    const Vec3 thr35 = frame.nedFromGeodetic(deg(-23.634333, -46.651167, 0));
    const Vec3 thr17 = frame.nedFromGeodetic(deg(-23.621, -46.6605, 0));
    const Vec3 d = thr17 - thr35;
    const Real heading = std::atan2(d.y, d.x);
    const Vec3 dir{std::cos(heading), std::sin(heading), 0};
    RunwayDef r35;
    r35.ident = "35L";
    r35.headingTrue = heading;
    r35.length = 1940;
    r35.width = 45;
    // Paved end of 35L: 61 m before its displaced threshold.
    r35.threshold = thr35 - dir * 61.0;
    r35.threshold.z = -a.elevation;
    r35.surface = surfaces::kAsphalt;
    a.runways.push_back(r35);
    RunwayDef r17 = r35;
    r17.ident = "17R";
    r17.headingTrue = wrapPi(heading + kPi);
    r17.threshold = r35.threshold + dir * r35.length;
    r17.threshold.z = -a.elevation;
    a.runways.push_back(r17);
    a.ilsKhz = 110500;
    return a;
}

std::unique_ptr<AirfieldGround> makeAirfieldGround(const Airport& a, const LocalFrame&) {
    auto g = std::make_unique<AirfieldGround>(-a.elevation, surfaces::kGrass);
    // One paved strip covers both directions.
    if (!a.runways.empty()) g->addRunway(a.runways[0]);
    return g;
}

const NavEnvironment& saoPauloNavEnvironment() {
    static NavDatabase db = [] {
        NavDatabase d;
        const LocalFrame f = saoPauloWorldFrame();
        const Airport a = campoDeMarte(f);
        const Real var = units::degToRad(-21.5);
        const std::string src = "synthetic test facility (Aviation Dynamics); not a real-world navaid";
        Navaid vor;
        vor.type = Navaid::Type::Vor;
        vor.ident = "TSP";
        vor.name = "Test VOR Paulista (synthetic)";
        vor.frequencyKhz = 113700;
        vor.position = f.origin();
        vor.position.height = 830;
        vor.magneticVariation = var;
        vor.rangeNm = 40;
        vor.synthetic = true;
        vor.source = src;
        d.add(vor);
        // Localizer antenna 300 m beyond the far end of runway 12.
        auto geo = [&](const Vec3& ned, Real h) {
            Geodetic g = f.geodeticFromNed(ned);
            g.height = h;
            return g;
        };
        // Localizer antenna 300 m beyond the far end; glideslope 300 m in from
        // the paved end, 120 m right of the centreline; 3 degrees.
        auto ils = [&](const Airport& ap, const std::string& ident, const std::string& name) {
            const RunwayDef& rw = ap.runways[0];
            Navaid loc;
            loc.type = Navaid::Type::Localizer;
            loc.ident = ident;
            loc.name = name;
            loc.frequencyKhz = ap.ilsKhz;
            loc.position = geo(runwayPoint(rw, rw.length + 300), ap.elevation + 2);
            loc.course = rw.headingTrue;
            // Width giving 700 ft (213 m) full-scale span at the threshold.
            loc.courseWidth = 2 * std::atan2(106.7, rw.length + 300);
            loc.rangeNm = 18;
            loc.synthetic = true;
            loc.source = src;
            d.add(loc);
            Navaid gs = loc;
            gs.type = Navaid::Type::Glideslope;
            gs.ident = ident + " GS";
            gs.position = geo(runwayPoint(rw, 300, 120), ap.elevation + 2);
            gs.glidepathAngle = units::degToRad(3.0);
            gs.rangeNm = 10;
            d.add(gs);
        };
        ils(a, "ITMT", "Test ILS Campo de Marte RWY 12 (synthetic)");
        ils(congonhas(f), "ITCG", "Test ILS Congonhas RWY 35L (synthetic)");
        return d;
    }();
    static NavEnvironment env = [] {
        NavEnvironment e;
        e.field.declination = units::degToRad(-21.5);
        e.field.inclination = units::degToRad(-38.0);
        e.database = &db;
        return e;
    }();
    return env;
}

Vec3 runwayPoint(const RunwayDef& r, Real along, Real across) {
    const Real c = std::cos(r.headingTrue), s = std::sin(r.headingTrue);
    return {r.threshold.x + along * c - across * s, r.threshold.y + along * s + across * c,
            r.threshold.z - r.slope * along};
}

}  // namespace avd::scenarios
