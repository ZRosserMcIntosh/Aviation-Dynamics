// Aviation Dynamics — turbofan, retractable gear, speedbrake and the Top G
// (G650ER-class) reference jet.
// SPDX-License-Identifier: MIT
#include "avd/units.hpp"
#include "avd_test.hpp"
#include "scenarios.hpp"

using namespace avd;
using namespace avd::scenarios;
using namespace avd::units;

namespace {

const AircraftDefinition& jet() {
    static const AircraftDefinition d = loadReference("top_g");
    return d;
}

constexpr Real kDt = 1.0 / 240;

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

TEST_CASE("turbofan: Mattingly thrust lapse, spool lag and fuel flow") {
    TurbofanDef d;
    d.staticThrust = 75000;
    d.lapseMachCoefficient = 0.49;
    d.throttleRatio = 1.0;
    const TurbofanEngine e(d);
    // Static, sea level ISA: the full rating.
    CHECK_NEAR(e.availableThrust(isa::kSeaLevelPressure, isa::kSeaLevelTemperature, 0), 75000, 1);
    // FL350, M0.8: delta0 (1 - 0.49 sqrt(M)) by hand.
    const StandardAtmosphere atm;
    const auto a = atm.at(ftToM(35000));
    const Real k = 1 + 0.2 * 0.64;
    const Real expect = 75000 * a.pressure / isa::kSeaLevelPressure * std::pow(k, 3.5) * (1 - 0.49 * std::sqrt(0.8));
    CHECK_NEAR(e.availableThrust(a.pressure, a.temperature, 0.8), expect, 1);
    // Hot day beyond the throttle ratio: less thrust.
    CHECK(e.availableThrust(isa::kSeaLevelPressure, isa::kSeaLevelTemperature + 30, 0) < 0.85 * 75000);
    // Spool-up from idle to 95 % of take-off thrust in about the defined acceleration time.
    TurbofanState s;
    TurbofanInputs in;
    in.fuelOn = in.fuelAtEngine = true;
    e.setSteady(s, in);
    CHECK(s.running);
    CHECK_NEAR(s.n1, d.idleN1, 1e-9);
    in.throttle = 1;
    Real t = 0;
    while (s.thrustFraction < 0.95 && t < 30) {
        e.step(s, in, 0.01);
        t += 0.01;
    }
    CHECK_NEAR(t, d.accelerationTime, 0.3);
    CHECK(s.fuelFlow > 0.5);  // ~1.1e-5 kg/(N s) x 71 kN
    // Fuel off: the engine runs down.
    in.fuelOn = false;
    for (int i = 0; i < 3000; ++i) e.step(s, in, 0.01);
    CHECK(!s.running);
    CHECK(s.thrust == 0);
    CHECK(s.n1 < 0.05);
}

TEST_CASE("top g: definition loads; cruise at Mach 0.85 is available at FL450 with sensible fuel burn") {
    Aircraft ac(jet());
    CHECK(ac.turbofanCount() == 2);
    const StandardAtmosphere atm;
    const TrimResult tr = trimAt(ac, 45000, 0.85 * atm.at(ftToM(45000)).speedOfSound);
    CHECK(tr.converged);
    CHECK(tr.throttle < 0.98);
    // Published long-range fuel burn of this class ~2,600-3,400 lb/h.
    CHECK(tr.fuelFlow * 3600 > lbToKg(2200) && tr.fuelFlow * 3600 < lbToKg(3800));
    CHECK(tr.theta > degToRad(0) && tr.theta < degToRad(4));
}

TEST_CASE("top g: approach speeds and stall margin are near the published class values") {
    // 1-g minimum speed with landing flap (Vref ~ 1.23 Vs1g ~ 110-120 KIAS at mid weights).
    const StandardAtmosphere atm;
    Real minKias = 0;
    for (Real k = 140; k > 70; k -= 2) {
        Aircraft ac(jet());
        const TrimResult tr = trimAt(ac, 5000, ktToMs(k) * std::sqrt(1.225 / atm.at(ftToM(5000)).density), 3, true);
        bool stalled = false;
        for (int i = ac.aeroModel().firstStrip(0); i < ac.aeroModel().endStrip(0); ++i)
            stalled = stalled || ac.aero().strips[i].alpha > ac.aero().strips[i].stallAlpha;
        if (!tr.converged || stalled) break;
        minKias = k;
    }
    CHECK(minKias > 82 && minKias < 102);
    // Trimmed on the approach with an ordinary pitch attitude and elevator inside its travel.
    Aircraft ac(jet());
    const TrimResult tr = trimAt(ac, 2000, ktToMs(125) * std::sqrt(1.225 / atm.at(ftToM(2000)).density), 3, true);
    CHECK(tr.converged);
    CHECK(std::abs(tr.elevator) < 0.5);
    CHECK(tr.theta > degToRad(0) && tr.theta < degToRad(6));
}

TEST_CASE("top g: take-off at a mid weight, then gear retraction cleans it up") {
    Aircraft ac(jet());
    SimpleWeather atm;
    FlatGround ground(0);
    ac.setEngineRunning(true);
    ac.setFlapsImmediately(1);
    ac.placeOnGround(0, 0, 0, ground, atm);
    ac.setCockpitControl("brakes.parking", 0);
    PilotInput in;
    in.throttle = 1;
    const Real x0 = ac.state().body.position.x;
    Real to35 = -1, liftoffIas = -1;
    for (int i = 0; i < 240 * 60 && to35 < 0; ++i) {
        const AircraftTelemetry& t = ac.telemetry();
        const Real ias = ac.instruments().readings().indicatedAirspeed;
        if (msToKt(ias) > 125) in.pitch = clamp(5.0 * (degToRad(10) - t.pitch) - 1.5 * t.omega.y, -1.0, 1.0);
        ac.step(kDt, in, ground, atm);
        if (liftoffIas < 0 && !ac.telemetry().onGround && msToKt(ias) > 80) liftoffIas = msToKt(ias);
        if (ac.telemetry().heightAgl > ftToM(35) + 1.4) to35 = ac.state().body.position.x - x0;
    }
    // ~1,000 m to 35 ft at 33.7 t, sea level ISA (published 1,920 m balanced field at 47 t).
    CHECK(to35 > 700 && to35 < 1400);
    CHECK(liftoffIas > 115 && liftoffIas < 150);
    // Gear up: legs stop touching anything, drag falls.
    ac.setCockpitControl("gear.lever", 0);
    for (int i = 0; i < 240 * 10; ++i) {
        in.pitch = clamp(3.0 * (degToRad(8) - ac.telemetry().pitch) - 1.0 * ac.telemetry().omega.y, -1.0, 1.0);
        ac.step(kDt, in, ground, atm);
    }
    CHECK_NEAR(ac.gearPosition(), 0.0, 1e-9);
    CHECK(ac.telemetry().verticalSpeed > ftToM(2000) / 60);
}

TEST_CASE("top g: fuel control switches start the engines from cold through the starter and light-off") {
    Aircraft ac(jet());
    SimpleWeather atm;
    FlatGround ground(0);
    ac.placeOnGround(0, 0, 0, ground, atm);
    CHECK(!ac.turbofanState(0).running);
    ac.setCockpitControl("electrical.battery", 1);
    ac.setCockpitControl("electrical.alternator", 1);  // generator switches
    ac.setCockpitControl("engine1.fuel", 1);
    PilotInput idle;
    bool started = false;
    Real t = 0;
    for (; t < 90 && !started; t += kDt) {
        ac.step(kDt, idle, ground, atm);
        started = ac.turbofanState(0).running;
    }
    CHECK(started);
    CHECK(t > 20 && t < 60);
    CHECK(!ac.turbofanState(1).running);
    CHECK(ac.electrical().alternatorOnline());  // the starter-generator now generates
    // Cut-off: it runs down.
    ac.setCockpitControl("engine1.fuel", 0);
    for (int i = 0; i < 240 * 5; ++i) ac.step(kDt, idle, ground, atm);
    CHECK(!ac.turbofanState(0).running);
}

TEST_CASE("top g: speedbrakes add drag and spoil lift; swept-wing drag rises past the critical Mach") {
    const StandardAtmosphere atm;
    Aircraft a(jet()), b(jet());
    const Real tas = ktToMs(280) * std::sqrt(1.225 / atm.at(ftToM(20000)).density);
    const TrimResult t0 = trimAt(a, 20000, tas);
    {
        // Deploy the panels in flight (they move at their own rate).
        const TrimResult tb = trimAt(b, 20000, tas);
        b.setCockpitControl("speedbrake.lever", 2);
        SimpleWeather w;
        FlatGround g(0);
        for (int i = 0; i < 240 * 3; ++i) b.step(kDt, tb.input, g, w);
    }
    CHECK_NEAR(b.speedbrakePosition(), 1.0, 1e-6);
    const TrimResult t1 = trimAt(b, 20000, tas);
    CHECK(t1.thrust > t0.thrust * 1.25);   // more drag to hold speed
    CHECK(t1.theta > t0.theta);            // more alpha for the spoiled lift
    // Drag rise: thrust to hold M0.92 vs M0.80 at FL390 grows faster than q.
    Aircraft c(jet()), d(jet());
    const Real a39 = atm.at(ftToM(39000)).speedOfSound;
    const TrimResult m80 = trimAt(c, 39000, 0.80 * a39), m92 = trimAt(d, 39000, 0.92 * a39);
    const Real cd80 = m80.thrust / sq(0.80), cd92 = m92.thrust / sq(0.92);
    CHECK(cd92 > cd80 * 1.08);
}
