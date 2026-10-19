// Aviation Dynamics — the generated jets (tools/jetgen): each loads, trims in
// cruise and on the approach, and meets the published figures it was
// calibrated to (the maximum cruise speed at full thrust; the approach speed
// where one is published), with the approach speed it predicts elsewhere.
// SPDX-License-Identifier: MIT
#include <cmath>

#include "avd/aero.hpp"
#include "avd/units.hpp"
#include "avd_test.hpp"
#include "scenarios.hpp"

using namespace avd;
using namespace avd::units;
using namespace avd::scenarios;

namespace {

struct Type {
    const char* id;
    Real cruiseKtas, flightLevel;       // published maximum cruise
    Real vrefKias, vrefLb, vrefFactor;  // published approach speed, its weight, Vref / Vs (0: none published)
    Real vrefTolerance;                 // fraction: calibrated types tight, predicted ones loose
};
const Type kTypes[] = {
    {"phenom300", 464, 300, 104, 17042, 1.3, 0.08},  // predicted (weight of the published Vref not stated)
    {"cj4", 451, 300, 112, 15660, 1.3, 0.02},        // calibrated
    {"learjet45", 465, 330, 0, 0, 0, 0},
    {"crj700", 473, 370, 135, 67000, 1.23, 0.02},    // calibrated
    {"b738", 460, 350, 0, 0, 0, 0},
};

TrimResult trimAt(Aircraft& ac, Real altFt, Real tas, int flap = 0, bool gear = false) {
    SimpleWeather atm;
    ac.setGearImmediately(gear);
    TrimRequest req;
    req.altitude = ftToM(altFt);
    req.tas = tas;
    req.flapDetent = flap;
    return trim(ac, req, atm);
}

}  // namespace

TEST_CASE("generated jets: each trims at its published maximum cruise speed on about full thrust") {
    for (const Type& t : kTypes) {
        Aircraft ac(loadReference(t.id));
        const TrimResult r = trimAt(ac, t.flightLevel * 100, ktToMs(t.cruiseKtas));
        CHECK(r.converged);
        CHECK_RANGE(r.throttle, 0.85, 1.08);
        CHECK_RANGE(radToDeg(r.alpha), -1.0, 5.0);
    }
}

TEST_CASE("generated jets: approach speeds against the published ones; a stable approach at Vref + 5") {
    for (const Type& t : kTypes) {
        const AircraftDefinition d = loadReference(t.id);
        Aircraft ac(d);
        const AeroModel model(d);
        const Real cl = trimmedMaxLiftCoefficient(model, d.referenceArea, ac.massProperties().cg, d.controls.flapDetents.back(), 0.2);
        CHECK_RANGE(cl, 1.4, 3.2);
        if (t.vrefKias > 0) {
            const Real w = t.vrefLb * kPound * kStandardGravity;
            const Real vs = std::sqrt(2 * w / (1.225 * d.referenceArea * cl));
            CHECK_NEAR(msToKt(vs) * t.vrefFactor, t.vrefKias, t.vrefTolerance * t.vrefKias);
        }
        // The approach at the spawn weight: Vref (1.23 Vs) + 5 kt, landing flap, gear down.
        const Real vs = std::sqrt(2 * ac.massProperties().mass * kStandardGravity / (1.225 * d.referenceArea * cl));
        const TrimResult r = trimAt(ac, 1500, 1.23 * vs + ktToMs(5), static_cast<int>(d.controls.flapDetents.size()) - 1, true);
        CHECK(r.converged);
        CHECK_RANGE(radToDeg(r.alpha), -2.0, 9.0);
    }
}

TEST_CASE("generated jets: the take-off roll tracks straight hands off (no ground loop), then lifts off and climbs") {
    // With the nose tyre as stiff as the Top G's per newton, a shorter
    // wheelbase put the nose tyre's destabilising side force within reach of
    // Vr: the small jets yawed off the runway near rotation without rudder.
    for (const Type& t : kTypes) {
        const AircraftDefinition d = loadReference(t.id);
        Aircraft ac(d);
        SimpleWeather atm;
        FlatGround ground(0);
        const std::size_t to = d.controls.flapDetents.size() / 2;
        ac.setEngineRunning(true);
        ac.setFlapsImmediately(static_cast<int>(to));
        ac.placeOnGround(0, 0, 0, ground, atm);
        ac.setCockpitControl("brakes.parking", 0);
        const AeroModel model(d);
        const Real cl = trimmedMaxLiftCoefficient(model, d.referenceArea, ac.massProperties().cg, d.controls.flapDetents[to], 0.2);
        const Real vr = 1.12 * std::sqrt(2 * ac.massProperties().mass * kStandardGravity / (1.225 * d.referenceArea * cl));
        const Real h0 = ac.telemetry().heading;
        PilotInput in;
        in.throttle = 1;
        Real headingAtVr = -1;
        bool climbed = false;
        for (int i = 0; i < 240 * 90 && !climbed; ++i) {
            const AircraftTelemetry& tm = ac.telemetry();
            const Real ias = ac.instruments().readings().indicatedAirspeed;
            if (ias > vr) {
                if (headingAtVr < 0) headingAtVr = std::abs(radToDeg(wrapPi(tm.heading - h0)));
                in.pitch = clamp(5.0 * (degToRad(10) - tm.pitch) - 1.5 * tm.omega.y, -1.0, 1.0);
            }
            ac.step(1.0 / 240, in, ground, atm);
            climbed = ac.telemetry().heightAgl > ftToM(35) + 2.5;
        }
        CHECK(headingAtVr >= 0 && headingAtVr < 3.0);
        CHECK(climbed);
        CHECK(!ac.faulted());
    }
}
