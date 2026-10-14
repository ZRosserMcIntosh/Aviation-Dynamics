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
    return a;
}

std::unique_ptr<AirfieldGround> makeAirfieldGround(const Airport& a, const LocalFrame&) {
    auto g = std::make_unique<AirfieldGround>(-a.elevation, surfaces::kGrass);
    // One paved strip covers both directions.
    if (!a.runways.empty()) g->addRunway(a.runways[0]);
    return g;
}

Vec3 runwayPoint(const RunwayDef& r, Real along, Real across) {
    const Real c = std::cos(r.headingTrue), s = std::sin(r.headingTrue);
    return {r.threshold.x + along * c - across * s, r.threshold.y + along * s + across * c,
            r.threshold.z - r.slope * along};
}

}  // namespace avd::scenarios
