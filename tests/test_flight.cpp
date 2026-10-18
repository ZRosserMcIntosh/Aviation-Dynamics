// Aviation Dynamics — aircraft-level flight dynamics tests.
// SPDX-License-Identifier: MIT
#include <cstring>

#include "airports.hpp"
#include "avd/units.hpp"
#include "avd_test.hpp"
#include "scenarios.hpp"

using namespace avd;
using namespace avd::scenarios;
using namespace avd::units;

namespace {

const AircraftDefinition& ref() {
    static const AircraftDefinition d = loadReference();
    return d;
}

struct Rig {
    Aircraft ac{ref()};
    SimpleWeather atm;
    FlatGround ground{0};
    TestPilot pilot;
    TrimResult trim;
    explicit Rig(Real kias = 100, Real alt = 1500, int flaps = 0, const SimpleWeather::Params* wp = nullptr) {
        if (wp) atm = SimpleWeather(*wp);
        startTrimmedFlight(ac, {kias, alt, flaps}, atm, ground, &pilot, &trim);
    }
    void fly(Real seconds, bool usePilot, Real dt = kDefaultDt) {
        const int n = static_cast<int>(std::lround(seconds / dt));
        for (int i = 0; i < n; ++i) {
            const PilotInput in = usePilot ? pilot.update(ac, dt) : hands;
            ac.step(dt, in, ground, atm);
        }
    }
    PilotInput hands;  // fixed controls when not using the pilot
};

}  // namespace

TEST_CASE("flight: trim converges and holds level flight hands-off") {
    Rig r(100, 1500);
    CHECK(r.trim.converged);
    CHECK_RANGE(radToDeg(r.trim.alpha), -2.0, 6.0);
    r.hands = r.trim.input;
    r.hands.pitch = 0;  // trimmed through the trim wheel
    const Real alt0 = r.ac.altitudeMsl(), v0 = r.ac.telemetry().trueAirspeed;
    Real maxDev = 0;
    r.ac.step(kDefaultDt, r.hands, r.ground, r.atm);
    for (int i = 0; i < 20 * 240; ++i) {
        r.ac.step(kDefaultDt, r.hands, r.ground, r.atm);
        maxDev = std::max(maxDev, std::abs(r.ac.altitudeMsl() - alt0));
    }
    // Slow phugoid/spiral drift only: no divergence in 20 s.
    CHECK(maxDev < 25);
    CHECK_NEAR(r.ac.telemetry().trueAirspeed, v0, 3.0);
    CHECK(std::abs(r.ac.telemetry().roll) < degToRad(10));
    CHECK(!r.ac.faulted());
}

TEST_CASE("flight: short period is well damped, phugoid is slow and lightly damped") {
    Rig r(100, 1500);
    r.hands = r.trim.input;
    r.hands.pitch = 0;
    // 0.5 s nose-up elevator pulse.
    PilotInput pulse = r.hands;
    pulse.pitch = 0.3;
    for (int i = 0; i < 120; ++i) r.ac.step(kDefaultDt, pulse, r.ground, r.atm);
    // Short period: pitch rate is back near zero within ~2 s.
    for (int i = 0; i < 480; ++i) r.ac.step(kDefaultDt, r.hands, r.ground, r.atm);
    CHECK(std::abs(r.ac.telemetry().omega.y) < degToRad(2.0));
    // Phugoid: find successive airspeed maxima.
    std::vector<Real> peaks;
    Real prev = r.ac.telemetry().trueAirspeed, prev2 = prev;
    for (int i = 0; i < 120 * 240 && peaks.size() < 3; ++i) {
        r.ac.step(kDefaultDt, r.hands, r.ground, r.atm);
        const Real v = r.ac.telemetry().trueAirspeed;
        if (i % 24 == 0) {
            if (prev > prev2 && prev > v) peaks.push_back(r.ac.time());
            prev2 = prev;
            prev = v;
        }
    }
    CHECK(peaks.size() >= 2);
    if (peaks.size() >= 2) {
        // Lanchester estimate: T ~ pi sqrt(2) V / g ~ 23 s at 51 m/s.
        CHECK_RANGE(peaks[1] - peaks[0], 15.0, 40.0);
    }
}

TEST_CASE("flight: coordinated 30 deg turn gives n = 1/cos(phi) and the right turn rate") {
    Rig r(95, 1500);
    r.pilot.lateral = TestPilot::Lateral::Bank;
    r.pilot.targetBank = degToRad(30);
    r.pilot.controls.throttle = std::min(1.0, r.trim.throttle + 0.12);
    r.fly(25, true);
    const AircraftTelemetry& t = r.ac.telemetry();
    CHECK_NEAR(t.roll, degToRad(30), degToRad(2));
    CHECK_NEAR(t.loadFactor, 1 / std::cos(degToRad(30)), 0.06);
    const Real expectedRate = kStandardGravity * std::tan(degToRad(30)) / t.trueAirspeed;
    const Real psiDot = std::atan2(t.velocityNed.y, t.velocityNed.x);
    r.fly(1, true);
    const Real psiDot2 = std::atan2(r.ac.telemetry().velocityNed.y, r.ac.telemetry().velocityNed.x);
    CHECK_NEAR(wrapPi(psiDot2 - psiDot), expectedRate, expectedRate * 0.1);
    CHECK(std::abs(radToDeg(t.beta)) < 2.0);  // coordinated
}

TEST_CASE("flight: the aircraft can stall, and the stall warning precedes it") {
    const StallResult s = stallTest(ref(), 0);
    CHECK(s.stalled);
    CHECK(s.warningIas > s.minIas + ktToMs(4));
    CHECK(s.alphaMax > degToRad(12));
    CHECK(s.altitudeLost > 10);  // a real stall loses height
}

TEST_CASE("flight: no artificial speed limit; VNE exceedance is reported, not prevented") {
    Rig r(120, 3000);
    r.pilot.vertical = TestPilot::Vertical::Pitch;
    r.pilot.targetPitch = degToRad(-35);
    r.pilot.controls.throttle = 1.0;
    Real maxIas = 0;
    bool overspeedEvent = false;
    for (int i = 0; i < 25 * 240; ++i) {
        r.ac.step(kDefaultDt, r.pilot.update(r.ac, kDefaultDt), r.ground, r.atm);
        maxIas = std::max(maxIas, r.ac.instruments().readings().indicatedAirspeed);
        for (const FlightEvent& e : r.ac.drainEvents())
            if (e.type == FlightEventType::Overspeed) overspeedEvent = true;
        if (r.ac.altitudeMsl() < 800) break;
    }
    CHECK(msToKt(maxIas) > 170);  // beyond VNE (163 KIAS)
    CHECK(overspeedEvent);
    CHECK(r.ac.telemetry().mach < 0.6);  // the physics, not a cap, decides: far from Mach 1
}

TEST_CASE("flight: simulation is bit-for-bit deterministic") {
    auto run = [] {
        SimpleWeather::Params wp;
        wp.windSpeed = 6;
        wp.turbulence.intensity = 1.5;
        Rig r(100, 1500, 0, &wp);
        r.pilot.lateral = TestPilot::Lateral::Bank;
        r.pilot.targetBank = 0.3;
        r.fly(20, true);
        return r.ac.state();
    };
    const DynamicState a = run(), b = run();
    CHECK(std::memcmp(&a.body, &b.body, sizeof(a.body)) == 0);
}

TEST_CASE("flight: fixed-step results converge (120 Hz vs 960 Hz)") {
    auto run = [](Real dt) {
        Rig r(100, 1500);
        r.hands = r.trim.input;
        r.hands.pitch = 0;
        PilotInput in = r.hands;
        in.pitch = 0.2;
        in.roll = 0.3;
        const int pulse = static_cast<int>(1.0 / dt), total = static_cast<int>(10.0 / dt);
        for (int i = 0; i < total; ++i) r.ac.step(dt, i < pulse ? in : r.hands, r.ground, r.atm);
        return r.ac.telemetry();
    };
    const AircraftTelemetry lo = run(1.0 / 120), hi = run(1.0 / 960);
    CHECK_NEAR(lo.altitudeMsl, hi.altitudeMsl, 0.5);
    CHECK_NEAR(lo.trueAirspeed, hi.trueAirspeed, 0.1);
    CHECK_NEAR(lo.roll, hi.roll, degToRad(0.5));
    CHECK_NEAR(lo.heading, hi.heading, degToRad(0.5));
}

TEST_CASE("flight: wind changes ground speed and track, not airspeed") {
    SimpleWeather::Params wp;
    wp.windSpeed = 10;
    wp.windFromDirection = 0;       // headwind for a northbound aircraft
    wp.profile = WindProfile::Uniform;   // uniform
    Rig r(100, 1500, 0, &wp);
    r.ac.setFlightState({r.ac.state().body.position, r.ac.state().body.velocity + Vec3{-10, 0, 0},
                         r.ac.state().body.attitude, {}});
    r.fly(10, true);
    const AircraftTelemetry& t = r.ac.telemetry();
    CHECK_NEAR(t.groundSpeed, t.trueAirspeed - 10, 0.6);
    // Crosswind from the west: the aircraft drifts east while heading north.
    SimpleWeather::Params cw;
    cw.windSpeed = 10;
    cw.windFromDirection = -kPi / 2;
    cw.profile = WindProfile::Uniform;
    Rig c(100, 1500, 0, &cw);
    c.ac.setFlightState({c.ac.state().body.position, c.ac.state().body.velocity + Vec3{0, 10, 0},
                         c.ac.state().body.attitude, {}});
    c.pilot.lateral = TestPilot::Lateral::Heading;
    c.pilot.targetHeading = 0;
    c.fly(10, true);
    CHECK_NEAR(c.ac.telemetry().velocityNed.y, 10, 1.0);
    CHECK(std::abs(c.ac.telemetry().beta) < degToRad(2));  // it weathervanes into the air, not the ground
}

TEST_CASE("flight: turbulence disturbs the aircraft through the air, deterministically") {
    SimpleWeather::Params wp;
    wp.turbulence.intensity = 2.5;
    Rig r(100, 1500, 0, &wp);
    r.hands = r.trim.input;
    r.hands.pitch = 0;
    Real maxP = 0, maxNz = 0;
    for (int i = 0; i < 20 * 240; ++i) {
        r.ac.step(kDefaultDt, r.hands, r.ground, r.atm);
        maxP = std::max(maxP, std::abs(r.ac.telemetry().omega.x));
        maxNz = std::max(maxNz, std::abs(r.ac.telemetry().loadFactor - 1));
    }
    CHECK(maxP > degToRad(2));
    CHECK(maxNz > 0.05);
    CHECK(!r.ac.faulted());
}

TEST_CASE("flight: the wind itself is bumpy near the ground and smooth above the boundary layer") {
    // Same 15 kt surface wind, no turbulence class set: the mechanical
    // turbulence of the surface layer shakes the airplane at 500 ft and is
    // gone at 5000 ft. The test pilot holds wings level and altitude.
    auto bumps = [](Real altitude, Real wind) {
        SimpleWeather::Params wp;
        wp.windSpeed = ktToMs(wind);
        Rig r(100, altitude, 0, &wp);
        r.pilot.lateral = TestPilot::Lateral::WingsLevel;
        Real maxNz = 0, minAlt = 1e9;
        for (int i = 0; i < 20 * 240; ++i) {
            r.ac.step(kDefaultDt, r.pilot.update(r.ac, kDefaultDt), r.ground, r.atm);
            if (i > 240) maxNz = std::max(maxNz, std::abs(r.ac.telemetry().loadFactor - 1));
            minAlt = std::min(minAlt, r.ac.altitudeMsl());
        }
        CHECK(minAlt > altitude - 60);
        return maxNz;
    };
    const Real calmLow = bumps(ftToM(500), 0), low = bumps(ftToM(500), 15);
    const Real calmHigh = bumps(ftToM(5000), 0), high = bumps(ftToM(5000), 15);
    CHECK(low > calmLow + 0.1);
    CHECK(low < 1.0);  // light-to-moderate mechanical turbulence, not a structural event
    CHECK_NEAR(high, calmHigh, 0.01);
}

TEST_CASE("flight: crossing a microburst -- headwind gain, then tailwind loss and sink") {
    // The classic encounter: airspeed rises in the outflow headwind, then the
    // downdraft and the tailwind take it away. A 15 m/s microburst (outflow
    // ~11 m/s and downdraft ~12 m/s at this height) 6 km ahead of an airplane
    // at 1000 ft, 100 KIAS, pilot holding wings level and altitude with
    // fixed power: more sink than a C172 can out-climb.
    SimpleWeather::Params wp;
    wp.turbulence.mechanical = false;
    Rig probe(100, ftToM(1000), 0, &wp);
    const Vec3 p0 = probe.ac.telemetry().positionNed;
    const Real hdg = probe.ac.telemetry().heading;
    Microburst m;
    m.peakOutflow = 15;
    m.north = p0.x + 6000 * std::cos(hdg);
    m.east = p0.y + 6000 * std::sin(hdg);
    m.startTime = -100;  // already mature when the airplane is trimmed
    m.growthTime = 1;
    m.matureTime = 1000;
    wp.microbursts.push_back(m);
    Rig r(100, ftToM(1000), 0, &wp);
    r.pilot.lateral = TestPilot::Lateral::WingsLevel;
    const Real v0 = r.ac.telemetry().trueAirspeed, alt0 = r.ac.altitudeMsl();
    Real vMax = v0, vMin = v0, minAlt = alt0, tMax = 0, tMin = 0, maxSink = 0;
    for (int i = 0; i < 200 * 240; ++i) {
        r.ac.step(kDefaultDt, r.pilot.update(r.ac, kDefaultDt), r.ground, r.atm);
        const AircraftTelemetry& t = r.ac.telemetry();
        if (t.trueAirspeed > vMax) { vMax = t.trueAirspeed; tMax = t.time; }
        if (t.trueAirspeed < vMin) { vMin = t.trueAirspeed; tMin = t.time; }
        minAlt = std::min(minAlt, r.ac.altitudeMsl());
        maxSink = std::max(maxSink, t.air.wind.z);
    }
    // The outflow headwind grows slowly (~0.1 m/s^2) against the drag time
    // constant: a modest gain (~5 kt). Then downdraft and tailwind together:
    // a large loss of airspeed and height (~55 kt, ~250 m).
    CHECK(msToKt(vMax - v0) > 3);    // headwind side
    CHECK(msToKt(v0 - vMin) > 30);   // tailwind and downdraft side
    CHECK(tMax < tMin);              // in that order
    CHECK(maxSink > 6);              // the downdraft (NED +z)
    CHECK(alt0 - minAlt > 100);      // more sink than a C172 can out-climb
    CHECK(!r.ac.faulted());
}

TEST_CASE("flight: a vertical 1-cosine gust gives the Pratt gust load factor") {
    // FAR 23/25 discrete gust: Delta n = Kg rho0 Ve a U / (2 W/S), with the
    // gust alleviation factor Kg = 0.88 mu / (5.3 + mu), mu = 2 (W/S) /
    // (rho c a g). Lift slope from Helmbold's relation for the wing aspect
    // ratio. A 30 ft (9.1 m/s) gust over H = 12.5 MAC, hands off.
    SimpleWeather::Params wp;
    wp.turbulence.mechanical = false;
    Rig probe(100, 1500, 0, &wp);
    const Vec3 p0 = probe.ac.telemetry().positionNed;
    const Real hdg = probe.ac.telemetry().heading;
    const AircraftDefinition& g = ref();
    const Real s = g.referenceArea, c = g.referenceChord, ar = sq(g.referenceSpan) / s;
    const Real u = 5.0;  // m/s up: moderate, keeps the wing well below the stall
    DiscreteGust gust;
    gust.heading = hdg + kPi;  // front faces the airplane...
    gust.frontSpeed = 0;       // ...fixed in space, 300 m ahead
    gust.origin = p0 + Vec3{std::cos(hdg), std::sin(hdg), 0} * 300.0;
    gust.length = 12.5 * c;
    gust.amplitude = {0, 0, -u};
    wp.gusts.push_back(gust);
    Rig r(100, 1500, 0, &wp);
    r.hands = r.trim.input;
    r.hands.pitch = 0;
    Real dnMax = 0;
    for (int i = 0; i < 15 * 240; ++i) {
        r.ac.step(kDefaultDt, r.hands, r.ground, r.atm);
        dnMax = std::max(dnMax, r.ac.telemetry().loadFactor - 1);
    }
    const Real rho = r.ac.telemetry().air.state.density, v = r.ac.telemetry().trueAirspeed;
    const Real ws = r.ac.massProperties().mass * kStandardGravity / s;
    const Real a = kTwoPi * ar / (2 + std::sqrt(ar * ar + 4));
    const Real mu = 2 * ws / (rho * c * a * kStandardGravity);
    const Real kg = 0.88 * mu / (5.3 + mu);
    const Real pratt = kg * rho * v * a * u / (2 * ws);
    // The strips are quasi-steady (no Küssner lift growth, which Kg partly
    // accounts for), so the sim reads ~15-20% above Pratt: known gap,
    // docs/architecture/wind.md.
    CHECK_RANGE(dnMax / pratt, 1.0, 1.35);
}

TEST_CASE("flight: a thermal lifts the airplane through the air") {
    // A strong explicit thermal (w* 3 m/s, zi 1800 m) on the track at
    // 3000 ft AGL. Holding airspeed at fixed power, the airplane climbs
    // through the core and sinks in the ring around it.
    SimpleWeather::Params wp;
    wp.turbulence.mechanical = false;
    Rig probe(100, ftToM(3000), 0, &wp);
    const Vec3 p0 = probe.ac.telemetry().positionNed;
    const Real hdg = probe.ac.telemetry().heading;
    wp.thermals.convectiveVelocity = 3;
    wp.thermals.depth = 1800;
    wp.thermals.field = false;
    wp.thermals.thermals.push_back({p0.x + 1500 * std::cos(hdg), p0.y + 1500 * std::sin(hdg), 1.0, -1e9, 0});
    wp.thermals.convectiveTurbulence = false;  // would mask the single core
    Rig r(100, ftToM(3000), 0, &wp);
    r.pilot.lateral = TestPilot::Lateral::WingsLevel;
    Real maxVs = 0, minVs = 0, maxUp = 0;
    for (int i = 0; i < 60 * 240; ++i) {
        r.ac.step(kDefaultDt, r.pilot.update(r.ac, kDefaultDt), r.ground, r.atm);
        maxVs = std::max(maxVs, r.ac.telemetry().verticalSpeed);
        minVs = std::min(minVs, r.ac.telemetry().verticalSpeed);
        maxUp = std::max(maxUp, -r.ac.telemetry().air.wind.z);
    }
    const Real wPeak = -ThermalField(wp.thermals, {}).thermalVelocity(0, 0, ftToM(3000), 1).z;
    CHECK_NEAR(maxUp, wPeak, 0.3 * wPeak);  // flew through the core
    CHECK(maxVs > 0.4 * wPeak);              // and was carried up
    CHECK(minVs < -0.15);                    // then the (broad, weaker) sink ring
    CHECK(!r.ac.faulted());
}

TEST_CASE("flight: fuel burn reduces mass exactly and moves the CG forward") {
    Rig r(100, 1500);
    const Real m0 = r.ac.massProperties().mass, f0 = r.ac.fuel().totalQuantity() + r.ac.fuel().lineQuantity();
    const Real cg0 = r.ac.massProperties().cg.x;
    Real burned = 0;
    for (int i = 0; i < 120 * 240; ++i) {
        r.ac.step(kDefaultDt, r.pilot.update(r.ac, kDefaultDt), r.ground, r.atm);
        burned += r.ac.telemetry().engines[0].engine.fuelFlow * kDefaultDt;
    }
    const Real f1 = r.ac.fuel().totalQuantity() + r.ac.fuel().lineQuantity();
    CHECK_NEAR((f0 - f1) * ref().fuel.density, burned, burned * 0.01);
    CHECK_NEAR(m0 - r.ac.massProperties().mass, burned, burned * 0.01);
    CHECK_RANGE(burned, 0.6, 1.4);  // ~7-10 gph for two minutes
    // Fuel arm (48 in) is aft of the loaded CG: burning it moves the CG forward.
    CHECK(r.ac.massProperties().cg.x > cg0);
}

TEST_CASE("flight: lateral fuel imbalance needs aileron, and the airframe does not jump") {
    Rig r(100, 1500);
    const Vec3 datum0 = r.ac.datumPosition();
    r.ac.fuel().setTankQuantity(0, 0.010);  // left tank nearly empty
    r.ac.refreshMass();
    const Vec3 datum1 = r.ac.datumPosition();
    CHECK_NEAR(length(datum1 - datum0), 0, 1e-9);  // CG moved inside the airframe, airframe did not move
    CHECK(r.ac.massProperties().cg.y > 0.05);      // CG to the right (heavy right wing)
    r.pilot.lateral = TestPilot::Lateral::WingsLevel;
    r.fly(15, true);
    CHECK(r.ac.telemetry().controls.aileron < -0.02);  // holding the heavy right wing up
}

TEST_CASE("flight: frame rebasing for long-distance flight preserves the trajectory") {
    Rig a(100, 1500), b(100, 1500);
    a.fly(5, false);
    b.fly(5, false);
    // Re-anchor b 15 km east: its world position must not change.
    const Geodetic before = b.ac.frame().geodeticFromNed(b.ac.state().body.position);
    const LocalFrame nf(b.ac.frame().geodeticFromNed({0, 15000, 0}));
    b.ac.rebase(nf);
    const Geodetic after = b.ac.frame().geodeticFromNed(b.ac.state().body.position);
    CHECK_NEAR(after.latitude, before.latitude, 1e-12);
    CHECK_NEAR(after.longitude, before.longitude, 1e-12);
    CHECK_NEAR(after.height, before.height, 1e-6);
    // Attitude and velocity are unchanged in Earth-fixed axes (local roll/pitch
    // change by the ~0.13 deg curvature angle between the tangent planes).
    const Vec3 noseA = a.ac.frame().nedToEcef() * a.ac.state().body.attitude.rotate({1, 0, 0});
    const Vec3 noseB = b.ac.frame().nedToEcef() * b.ac.state().body.attitude.rotate({1, 0, 0});
    CHECK_NEAR(length(noseA - noseB), 0, 1e-9);
    const Vec3 va = a.ac.frame().nedToEcef() * a.ac.state().body.velocity;
    const Vec3 vb = b.ac.frame().nedToEcef() * b.ac.state().body.velocity;
    CHECK_NEAR(length(va - vb), 0, 1e-9);
    // Both keep flying the same physics afterwards (gravity follows the new vertical).
    a.fly(5, false);
    b.fly(5, false);
    CHECK_NEAR(a.ac.altitudeMsl(), b.ac.altitudeMsl(), 20.0);
}
