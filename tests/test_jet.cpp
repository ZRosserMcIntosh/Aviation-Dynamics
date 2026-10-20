// Aviation Dynamics — turbofan, retractable gear, speedbrake and the Top G
// (G650ER-class) reference jet.
// SPDX-License-Identifier: MIT
#include <vector>

#include "avd/assists.hpp"
#include "avd/loading.hpp"
#include "avd/wake.hpp"
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

TEST_CASE("turbofan: FADEC ratings cap N1 (fractions at sea level, converging at altitude) and the automatic schedule") {
    const AircraftDefinition& d = jet();
    const TurbofanDef& td = d.turbofans[0];
    const TurbofanEngine e(td);
    const Real p0 = isa::kSeaLevelPressure, T0 = isa::kSeaLevelTemperature;
    // Sea level static: the definition's fractions, and N1 limits in order.
    const Real to = e.ratingThrust(ThrustRating::Takeoff, p0, T0, 0);
    CHECK_NEAR(to, td.staticThrust, 1);
    CHECK_NEAR(e.ratingThrust(ThrustRating::MaxContinuous, p0, T0, 0) / to, td.maxContinuousFraction, 1e-6);
    CHECK_NEAR(e.ratingThrust(ThrustRating::MaxClimb, p0, T0, 0) / to, td.maxClimbFraction, 1e-6);
    CHECK_NEAR(e.ratingThrust(ThrustRating::MaxCruise, p0, T0, 0) / to, td.maxCruiseFraction, 1e-6);
    const Real n1Clb = e.ratingN1(ThrustRating::MaxClimb, p0, T0, 0);
    CHECK(n1Clb < 1 && n1Clb > e.ratingN1(ThrustRating::MaxCruise, p0, T0, 0));
    CHECK_NEAR(e.thrustFraction(n1Clb), td.maxClimbFraction, 1e-6);
    // At FL410 / Mach 0.85 the take-off rating sits far below its temperature
    // limit and the lower ratings converge on it (never above it).
    const StandardAtmosphere atm;
    const auto a = atm.at(ftToM(41000));
    const Real toHigh = e.ratingThrust(ThrustRating::Takeoff, a.pressure, a.temperature, 0.85);
    const Real crzHigh = e.ratingThrust(ThrustRating::MaxCruise, a.pressure, a.temperature, 0.85);
    CHECK(crzHigh <= toHigh + 1e-9);
    CHECK(crzHigh / toHigh > 0.95);
    CHECK(e.ratingThrust(ThrustRating::MaxClimb, a.pressure, a.temperature, 0.85) >= crzHigh);
    // The fuel control: the lever commands N1 below the limit, the limit above it.
    TurbofanInputs in;
    in.rating = ThrustRating::MaxClimb;
    in.throttle = 0.5;
    CHECK_NEAR(e.commandedN1(in), e.commandedN1(0.5), 1e-12);
    in.throttle = 1;
    CHECK_NEAR(e.commandedN1(in), n1Clb, 1e-12);
    // The aircraft's automatic schedule: take-off thrust from lift-off for
    // the rating's time, then the climb rating caps N1 at its limit for the
    // flight condition reached. A forced rating applies at once.
    SimpleWeather atmo;
    const FlatGround ground(1e7);
    auto limitN1 = [&](const Aircraft& ac, ThrustRating r) {
        const AirState air = atmo.atmosphere().at(ac.altitudeMsl());
        return ac.turbofan(0).ratingN1(r, air.pressure, air.temperature, length(ac.state().body.velocity) / air.speedOfSound);
    };
    auto climb = [&](Aircraft& ac, const TrimResult& tr, ThrustMode mode, Real seconds) {
        PilotInput in1 = tr.input;
        in1.thrustMode = mode;
        for (Real t = 0; t < seconds; t += kDt) {
            in1.pitch = tr.input.pitch + clamp(2.0 * (tr.theta - ac.telemetry().pitch) - 0.5 * ac.telemetry().omega.y, -0.3, 0.3);
            ac.step(kDt, in1, ground, atmo);
        }
    };
    TrimRequest req;
    req.altitude = ftToM(10000);
    req.tas = tasFromCas(ktToMs(250), atmo.atmosphere().at(ftToM(10000)));
    req.throttle = 1;
    req.solveFlightPath = true;
    {
        Aircraft ac(d);
        const TrimResult tr = trim(ac, req, atmo);
        CHECK(tr.converged);
        CHECK(ac.activeThrustRating() == ThrustRating::Takeoff);
        climb(ac, tr, ThrustMode::Auto, td.takeoffRatingTime - 5);
        CHECK(ac.activeThrustRating() == ThrustRating::Takeoff);
        CHECK_NEAR(ac.turbofanState(0).n1, 1.0, 1e-6);
        climb(ac, tr, ThrustMode::Auto, 25);
        CHECK(ac.activeThrustRating() == ThrustRating::MaxClimb);
        CHECK(ac.turbofanState(0).n1 <= limitN1(ac, ThrustRating::MaxClimb) + 1e-3);
    }
    {
        // Forced climb rating at 10,000 ft: well below take-off N1.
        Aircraft ac(d);
        const TrimResult tr = trim(ac, req, atmo);
        climb(ac, tr, ThrustMode::Climb, 15);
        CHECK(ac.activeThrustRating() == ThrustRating::MaxClimb);
        CHECK(ac.turbofanState(0).n1 < 0.97);
        CHECK_NEAR(ac.turbofanState(0).n1, limitN1(ac, ThrustRating::MaxClimb), 2e-3);
        CHECK(ac.turbofanState(0).thrustFraction < 0.95);  // about 0.91 of take-off at 250 KIAS, 10,000 ft
        climb(ac, tr, ThrustMode::Cruise, 15);
        CHECK(ac.activeThrustRating() == ThrustRating::MaxCruise);
        CHECK(ac.turbofanState(0).n1 < limitN1(ac, ThrustRating::MaxClimb));
    }
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

TEST_CASE("trimmed CLmax gives stall speeds near the published values for the single and the Top G") {
    auto vs = [](const AircraftDefinition& d, Real massKg, Real flapDeg) {
        Aircraft ac(d);
        const Real cl = trimmedMaxLiftCoefficient(ac.aeroModel(), d.referenceArea, ac.massProperties().cg,
                                                  degToRad(flapDeg), 0.15);
        CHECK(cl > 0.8);
        return msToKt(std::sqrt(2 * massKg * 9.80665 / (1.225 * d.referenceArea * cl)));
    };
    // Top G at 33.7 t: clean 1-g minimum ~118-126 KCAS in the trim sweep; landing flap ~92.
    const Real jetClean = vs(jet(), 33700, 0), jetLand = vs(jet(), 33700, 39);
    std::printf("    Top G Vs1g clean %.1f kt, flaps 39 %.1f kt\n", jetClean, jetLand);
    CHECK(jetClean > 105 && jetClean < 135);
    CHECK(jetLand > 82 && jetLand < 102);
    CHECK(jetLand < vs(jet(), 33700, 20));
    // The single (tupi_t180) at 1,157 kg: calibrated to POH 53 KCAS flaps up, 48 KCAS flaps 30.
    const AircraftDefinition single = loadReference("tupi_t180");
    const Real cClean = vs(single, 1156.7, 0), cLand = vs(single, 1156.7, 30);
    std::printf("    single Vs1g clean %.1f kt, flaps 30 %.1f kt\n", cClean, cLand);
    CHECK(cClean > 48 && cClean < 58);
    CHECK(cLand > 42 && cLand < 53);
}

TEST_CASE("top g: swept-wing lift is normal to the swept span, so induced drag is real (L/D 11-19)") {
    // Regression: lift taken normal to the unswept lateral axis tilted it
    // forward by ~sin(alpha) sin^2(sweep) and cancelled most induced drag
    // (L/D ~100 at approach speeds, idle-thrust level flight at 130 kt).
    for (const Real kt : {130.0, 200.0, 250.0}) {
        Aircraft ac(jet());
        const TrimResult tr = trimAt(ac, 5000, ktToMs(kt) * 1.08);
        CHECK(tr.converged);
        Real thrust = 0;
        for (int i = 0; i < ac.turbofanCount(); ++i) thrust += ac.turbofanState(i).thrust;
        const Real ld = ac.telemetry().mass * 9.80665 / thrust;
        std::printf("    %.0f kt: L/D %.1f\n", kt, ld);
        CHECK(ld > 10.5 && ld < 19);
    }
}

TEST_CASE("top g: a double flame-out at cruise windmills below idle, drops the generators, and relights") {
    const StandardAtmosphere std0;
    Aircraft ac(jet());
    SimpleWeather atm;
    FlatGround ground(0);
    const TrimResult tr = trimAt(ac, 33000, 0.72 * std0.at(ftToM(33000)).speedOfSound);
    CHECK(tr.converged);
    for (const char* c : {"electrical.battery", "electrical.alternator", "engine1.fuel", "engine2.fuel"}) ac.setCockpitControl(c, 1);
    PilotInput in;
    in.throttle = tr.throttle;
    auto fly = [&](Real seconds, Real pitchDeg) {
        for (int i = 0; i < static_cast<int>(seconds / kDt); ++i) {
            const AircraftTelemetry& t = ac.telemetry();
            in.pitch = clamp(tr.elevator + 2.0 * (degToRad(pitchDeg) - t.pitch) - 0.8 * t.omega.y, -1.0, 1.0);
            ac.step(kDt, in, ground, atm);
        }
    };
    fly(5, radToDeg(tr.theta));
    CHECK(ac.electrical().alternatorOnline());
    ac.setCockpitControl("engine1.fuel", 0);
    ac.setCockpitControl("engine2.fuel", 0);
    fly(20, -2);
    CHECK(ac.turbofanState(0).itt - 273.15 < 350);  // the fire is out: ITT falls fast
    fly(70, -2);
    for (int i = 0; i < 2; ++i) {
        const TurbofanState& e = ac.turbofanState(i);
        CHECK(!e.running);
        CHECK(e.n2 > 0.12 && e.n2 < 0.3);       // windmilling core, well below idle
        CHECK(e.n1 > 0.15 && e.n1 < 0.32);
        CHECK(e.oilPressure < 6894.76 * 25);   // below the oil-pressure red line
    }
    CHECK(!ac.electrical().alternatorOnline());  // windmilling does not hold the bus
    ac.setCockpitControl("engine1.fuel", 1);
    ac.setCockpitControl("engine2.fuel", 1);
    bool both = false;
    for (Real t = 0; t < 60 && !both; t += 0.5) {
        fly(0.5, -2);
        both = ac.turbofanState(0).running && ac.turbofanState(1).running;
    }
    CHECK(both);
}

TEST_CASE("top g: the stick pusher keeps a full-aft-stick idle stall out of the T-tail deep stall") {
    Aircraft ac(jet());
    SimpleWeather atm;
    FlatGround ground(0);
    const TrimResult tr = trimAt(ac, 15000, ktToMs(200) * 1.27);
    CHECK(tr.converged);
    PilotInput in;
    in.throttle = 0;
    const Real h0 = ac.telemetry().altitudeMsl;
    Real maxAlpha = 0, warnIas = 0;
    bool pushed = false;
    int pullStart = -1;
    for (int i = 0; pullStart < 0 ? i < 240 * 240 : i < pullStart + 240 * 45; ++i) {
        const AircraftTelemetry& t = ac.telemetry();
        if (warnIas > 0 && pullStart < 0) pullStart = i;
        if (warnIas == 0) {
            // Idle, held level: a slow 1-g deceleration to the warning, the
            // pilot trimming the stick force away as the speed falls.
            const Real thetaCmd = t.alpha + clamp(0.01 * (h0 - t.altitudeMsl), -0.08, 0.08);
            in.pitch = clamp(tr.elevator + 2.5 * (thetaCmd - t.pitch) - 0.8 * t.omega.y, -1.0, 1.0);
            in.trimRate = clamp(4 * in.pitch, -1.0, 1.0);
        } else {
            in.trimRate = 0;
            in.pitch = 1;  // then held fully back
        }
        // Wings held level with the ailerons.
        in.roll = clamp(-2.0 * t.roll - 0.5 * t.omega.x, -1.0, 1.0);
        ac.step(kDt, in, ground, atm);
        maxAlpha = std::max(maxAlpha, ac.telemetry().alpha);
        pushed = pushed || ac.telemetry().stickPusher;
        if (warnIas == 0 && ac.telemetry().stallWarning) warnIas = msToKt(ac.instruments().readings().indicatedAirspeed);
    }
    std::printf("    warning at %.0f KIAS, max alpha %.1f deg\n", warnIas, radToDeg(maxAlpha));
    CHECK(pushed);
    CHECK(maxAlpha < degToRad(25));
    CHECK(warnIas > 112 && warnIas < 125);  // a few knots above the ~112 KIAS clean stall
}

TEST_CASE("top g: Q/E steer the nose wheel by tiller below 5 kt, by pedals above") {
    // The yaw input through the assist layer (Simulation preset: the tiller
    // handover is an input binding, not an assist). Taxi up to the target
    // speed, hold it with thrust and brakes, then full right yaw input for 8 s.
    auto taxiTurn = [](const AircraftDefinition& def, Real targetKt, Real& steer, bool& onTiller) {
        Aircraft ac(def);
        SimpleWeather atm;
        FlatGround ground(0);
        ac.setEngineRunning(true);
        ac.placeOnGround(0, 0, 0, ground, atm);
        ac.setCockpitControl("brakes.parking", 0);
        FlightAssist assist(makePreset(AssistPreset::Simulation));
        PilotInput raw;
        Real rate = 0;
        int turning = -1;  // steps since the yaw input went in
        for (int i = 0; i < 240 * 60 && turning < 240 * 8; ++i) {
            const Real gs = msToKt(ac.telemetry().groundSpeed);
            if (turning < 0 && gs >= targetKt - 0.3) turning = 0;
            raw.throttle = gs < targetKt - 0.5 ? 0.4 : 0.0;
            raw.brakeLeft = raw.brakeRight = clamp((gs - targetKt) * 0.3, 0.0, 1.0);
            raw.yaw = turning >= 0 ? 1.0 : 0.0;
            ac.step(kDt, assist.apply(raw, ac, kDt), ground, atm);
            if (turning >= 0 && ++turning > 240 * 5) rate = std::max(rate, ac.telemetry().omega.z);
        }
        steer = ac.telemetry().gear[0].steerAngle;
        onTiller = assist.report().yawOnTiller;
        return rate;
    };
    AircraftDefinition pedalsOnly = jet();
    pedalsOnly.gear[0].tillerSteerAngle = 0;
    Real steerT, steerP, steerFast;
    bool tillerT, tillerP, tillerFast;
    const Real tiller = taxiTurn(jet(), 3, steerT, tillerT);
    const Real pedals = taxiTurn(pedalsOnly, 3, steerP, tillerP);
    taxiTurn(jet(), 9, steerFast, tillerFast);
    CHECK(tillerT);
    CHECK_NEAR(steerT, degToRad(80), 1e-6);
    CHECK(!tillerP);
    CHECK_NEAR(steerP, degToRad(10), 1e-6);
    CHECK(tiller > 3 * pedals);  // a tight pivot, not a wide pedal turn
    // Above 5 kt the yaw input is back on the pedals (and the rudder).
    CHECK(!tillerFast);
    CHECK(std::abs(steerFast) <= degToRad(10) + 1e-6);
}

TEST_CASE("top g: auto-trim holds FL410 after a pitch input, and near the stall gives up height, not speed") {
    // Trimmed, hands-off start with the trimmed elevator in the trim wheel.
    auto start = [](Aircraft& ac, Real altFt, Real kias, int flap, bool gear) {
        SimpleWeather atm;
        FlatGround ground(0);
        const TrimResult tr =
            trimAt(ac, altFt, tasFromCas(ktToMs(kias), atm.atmosphere().at(ftToM(altFt)).density), flap, gear);
        ac.setState(tr.state);
        ac.prepareContext(tr.input, ground, atm);
        ac.settleDownwash();
        ac.setElevatorTrim(tr.trim);
        return tr;
    };
    AssistConfig cfg = makePreset(AssistPreset::Simulation);
    cfg.levelHold = true;
    SimpleWeather atm;
    FlatGround ground(0);
    {
        // Climbing turn, then hands off with 30% less thrust at FL410.
        Aircraft ac(jet());
        const TrimResult tr = start(ac, 41000, 240, 0, false);
        FlightAssist assist(cfg);
        PilotInput raw = tr.input;
        Real hMin = 1e9, hMax = -1e9;
        for (int i = 0; i < 240 * 120; ++i) {
            raw.roll = i < 240 * 1.5 ? 0.25 : 0.0;
            raw.pitch = i < 240 * 2 ? 0.15 : 0.0;
            raw.throttle = i < 240 * 2 ? tr.input.throttle : 0.7 * tr.input.throttle;
            ac.step(kDt, assist.apply(raw, ac, kDt), ground, atm);
            if (i > 240 * 40) {
                hMin = std::min(hMin, ac.telemetry().altitudeMsl);
                hMax = std::max(hMax, ac.telemetry().altitudeMsl);
            }
        }
        CHECK(assist.report().levelHold);
        CHECK(!assist.report().underspeed);
        CHECK(hMax - hMin < ftToM(20));  // held while the speed bleeds off
        CHECK(std::abs(ac.telemetry().roll) < degToRad(1));
    }
    {
        // On the approach with too little thrust to hold altitude: the hold
        // trades height for speed and the wing never stalls.
        Aircraft ac(jet());
        const TrimResult tr = start(ac, 5000, 150, 2, true);
        FlightAssist assist(cfg);
        PilotInput raw = tr.input;
        raw.throttle = 0.5 * tr.input.throttle;
        bool underspeed = false;
        int stalled = 0;
        for (int i = 0; i < 240 * 150; ++i) {
            ac.step(kDt, assist.apply(raw, ac, kDt), ground, atm);
            underspeed = underspeed || assist.report().underspeed;
            stalled = std::max(stalled, ac.telemetry().stalledStrips);
        }
        CHECK(underspeed);
        CHECK(stalled == 0);
        CHECK(ac.telemetry().verticalSpeed < 0);  // descending to keep flying speed
        CHECK(ac.telemetry().altitudeMsl > ftToM(1000));
    }
}

TEST_CASE("top g: an engine starved with its fuel switch at RUN winds down instead of hanging at idle") {
    Aircraft ac(jet());
    SimpleWeather atm;
    FlatGround ground(0);
    ac.setEngineRunning(true);
    ac.placeOnGround(0, 0, 0, ground, atm);
    for (int t = 0; t < 2; ++t) ac.fuel().setTankQuantity(t, jet().fuel.tanks[t].unusable);  // dry tanks
    PilotInput idle;
    for (int i = 0; i < 240 * 120; ++i) ac.step(kDt, idle, ground, atm);
    const TurbofanState& e = ac.turbofanState(0);
    CHECK(!e.running);
    CHECK(!e.starting);                 // the no-light-off start was aborted
    CHECK(e.n2 < 0.3);                  // run down to the starter's speed at most
    CHECK(e.thrust <= 0 && e.thrust > -20);  // unlit: no thrust, only the windmilling drag of whatever air moves past
    CHECK(e.itt < atm.atmosphere().at(0).temperature + 60);
    // Fuel back, and the battery recharged (as ground power would): the starter
    // waits until the fuel switch is cycled, then starts it.
    for (int t = 0; t < 2; ++t) ac.fuel().setTankQuantity(t, 0.5 * jet().fuel.tanks[t].capacity);
    ac.electrical().setBatteryCharge(1.0);
    for (int i = 0; i < 240 * 10; ++i) ac.step(kDt, idle, ground, atm);
    CHECK(!ac.turbofanState(0).running);
    ac.setCockpitControl("engine1.fuel", 0);
    ac.step(kDt, idle, ground, atm);
    ac.setCockpitControl("engine1.fuel", 1);
    for (int i = 0; i < 240 * 60 && !ac.turbofanState(0).running; ++i) ac.step(kDt, idle, ground, atm);
    CHECK(ac.turbofanState(0).running);
}

TEST_CASE("top g: the gear handle is locked down while the weight is on the wheels") {
    Aircraft ac(jet());
    SimpleWeather atm;
    FlatGround ground(0);
    ac.setEngineRunning(true);
    ac.placeOnGround(0, 0, 0, ground, atm);
    PilotInput idle;
    ac.step(kDt, idle, ground, atm);
    CHECK(!ac.setCockpitControl("gear.lever", 0));
    for (int i = 0; i < 240 * 5; ++i) ac.step(kDt, idle, ground, atm);
    CHECK_NEAR(ac.gearPosition(), 1.0, 1e-12);
    CHECK(ac.cockpitControl("gear.lever") == 1);
}

TEST_CASE("top g: the load sheet checks the zero-fuel, landing and ramp weights") {
    Aircraft ac(jet());
    LoadSheet ls = computeLoadSheet(ac);
    CHECK(ls.flags == 0);
    CHECK_NEAR(ac.massProperties().mass, ls.grossMass, 1e-6);
    // Full cabin and baggage: past the maximum zero-fuel weight (60,500 lb).
    ac.setStationMass(1, 2200);
    ac.setStationMass(2, 1134);
    ls = computeLoadSheet(ac);
    CHECK((ls.flags & kOverMaxZeroFuel) != 0);
    // Full tanks on top: over the ramp and take-off weights, and the landing weight.
    for (int t = 0; t < 2; ++t) ac.fuel().setTankQuantity(t, jet().fuel.tanks[t].capacity);
    ac.refreshMass();
    ls = computeLoadSheet(ac);
    CHECK((ls.flags & kOverMaxRamp) != 0);
    CHECK((ls.flags & kOverMaxTakeoff) != 0);
    CHECK((ls.flags & kOverMaxLanding) != 0);
    CHECK_NEAR(ls.fuelUsable, 2 * (13664.0 - 40.0) * 1e-3 * jet().fuel.density, 1e-6);
}

TEST_CASE("ground: wet-runway friction follows 25.109 and collapses at the hydroplaning speed") {
    SurfaceProperties dry = surfaces::kAsphalt, wet = dry, flooded = dry;
    wet.waterDepth = 0.001;
    flooded.waterDepth = 0.008;
    const Real psi = 6894.757;
    CHECK_NEAR(wetTyreFriction(dry, ktToMs(100), 50 * psi), 0.8, 1e-12);
    // 50 psi at 100 kt: -0.0350 + 0.306 - 0.851 + 0.883.
    CHECK_NEAR(wetTyreFriction(wet, ktToMs(100), 50 * psi), 0.303, 1e-9);
    CHECK_NEAR(wetTyreFriction(wet, ktToMs(100), 200 * psi), -0.0331 + 0.252 - 0.658 + 0.692, 1e-9);
    CHECK_NEAR(wetTyreFriction(wet, 0, 50 * psi), 0.8, 1e-12);   // never above dry
    CHECK_NEAR(msToKt(hydroplaningSpeed(38 * psi)), 9 * std::sqrt(38.0), 1e-9);
    CHECK_NEAR(wetTyreFriction(flooded, hydroplaningSpeed(190 * psi) * 1.01, 190 * psi), 0.05, 1e-12);
    CHECK(wetTyreFriction(flooded, ktToMs(60), 190 * psi) < wetTyreFriction(wet, ktToMs(60), 190 * psi));
    SurfaceProperties grass = surfaces::kGrass;
    grass.waterDepth = 0.001;
    CHECK_NEAR(wetTyreFriction(grass, 10, 0), 0.6 * surfaces::kGrass.friction, 1e-12);
}

TEST_CASE("top g: a wet runway lengthens the stop; standing water above the hydroplaning speed far more") {
    // The tyres alone: no reversers or ground spoilers (those do not care about water).
    static const AircraftDefinition tyresOnly = [] {
        AircraftDefinition d = jet();
        for (TurbofanDef& t : d.turbofans) t.reverser.present = false;
        d.controls.speedbrake.groundSpoilers = false;
        return d;
    }();
    auto stop = [](Real waterDepth, Real kt) {
        Aircraft ac(tyresOnly);
        SimpleWeather atm;
        SurfaceProperties s = surfaces::kAsphalt;
        s.waterDepth = waterDepth;
        FlatGround ground(0, s);
        ac.setEngineRunning(true);
        ac.placeOnGround(0, 0, 0, ground, atm);
        ac.setCockpitControl("brakes.parking", 0);
        DynamicState st = ac.state();
        st.body.velocity = {ktToMs(kt), 0, 0};
        ac.setState(st);
        PilotInput in;
        in.brakeLeft = in.brakeRight = 1;
        const Real x0 = ac.state().body.position.x;
        for (int i = 0; i < 240 * 90; ++i) {
            ac.step(kDt, in, ground, atm);
            if (i > 240 && ac.telemetry().groundSpeed < 0.5) break;
        }
        return ac.state().body.position.x - x0;
    };
    // From 100 kt (below the ~124 kt hydroplaning speed of 190 psi tyres).
    const Real dry = stop(0, 100), wet = stop(0.001, 100), flooded = stop(0.008, 100);
    CHECK(wet > 1.3 * dry);
    CHECK(flooded > 1.1 * wet);
    // From 140 kt the tyres ride on standing water until they slow below it.
    const Real wetFast = stop(0.001, 140), floodedFast = stop(0.008, 140);
    CHECK(floodedFast > wetFast + 150);
}

TEST_CASE("top g: ground spoilers and thrust reversers deploy on touchdown, reverse to 60 KIAS, then stow") {
    auto roll = [](bool automatics, std::vector<int>* phases, Real* stowKias, Real* spoilerMax) {
        AircraftDefinition d = jet();
        if (!automatics) {
            for (TurbofanDef& t : d.turbofans) t.reverser.present = false;
            d.controls.speedbrake.groundSpoilers = false;
        }
        Aircraft ac(d);
        SimpleWeather atm;
        FlatGround ground(0, surfaces::kAsphalt);
        ac.setEngineRunning(true);
        ac.placeOnGround(0, 0, 0, ground, atm);
        ac.setCockpitControl("brakes.parking", 0);
        // At rest with the throttles at idle: nothing deploys.
        PilotInput in;
        for (int i = 0; i < 240; ++i) ac.step(kDt, in, ground, atm);
        CHECK(ac.reverserPosition(0) == 0);
        CHECK(!ac.groundSpoilersActive());
        // The landing roll from 120 kt, no wheel brakes.
        DynamicState st = ac.state();
        st.body.velocity = {ktToMs(120), 0, 0};
        ac.setState(st);
        const Real x0 = ac.state().body.position.x;
        Real to70 = 0;
        for (int i = 0; i < 240 * 120; ++i) {
            ac.step(kDt, in, ground, atm);
            if (phases && (phases->empty() || phases->back() != ac.reverserPhase(0))) {
                phases->push_back(ac.reverserPhase(0));
                if (ac.reverserPhase(0) == 2 && stowKias) *stowKias = msToKt(ac.telemetry().trueAirspeed);
            }
            if (spoilerMax) *spoilerMax = std::max(*spoilerMax, ac.speedbrakePosition());
            if (to70 == 0 && ac.telemetry().groundSpeed < ktToMs(70)) to70 = ac.state().body.position.x - x0;
            if (ac.telemetry().groundSpeed < 5) break;
        }
        CHECK(ac.reverserPosition(0) < 0.05);  // stowed at taxi speed
        return to70;
    };
    std::vector<int> phases;
    Real stowKias = 0, spoilerMax = 0;
    const Real with = roll(true, &phases, &stowKias, &spoilerMax), without = roll(false, nullptr, nullptr, nullptr);
    // Deploy, reverse thrust, reverse idle at 60 KIAS, stow, stowed.
    CHECK(phases.size() >= 4);
    phases.resize(4, -1);
    CHECK(phases[0] == 1);
    CHECK(phases[1] == 2);
    CHECK(phases[2] == 3);
    CHECK(phases[3] == 0);
    CHECK_RANGE(stowKias, 55, 61);
    CHECK(spoilerMax > 0.99);
    // From 120 to 70 kt without brakes (where reversers matter: below 60 kt
    // they are stowed) the reversers and spoilers save a great deal of runway.
    CHECK_RANGE(with / without, 0.15, 0.7);

    // A landing (after flight) deploys them with the throttles short of idle,
    // too: the automation retards them; a take-off roll at that setting does not.
    Aircraft ac(jet());
    SimpleWeather atm;
    FlatGround ground(0, surfaces::kAsphalt);
    ac.setEngineRunning(true);
    ac.placeOnGround(0, 0, 0, ground, atm);
    ac.setCockpitControl("brakes.parking", 0);
    DynamicState st = ac.state();
    st.body.position.z -= 8;
    st.body.velocity = {ktToMs(125), 0, 1.6};
    ac.setState(st);
    PilotInput in;
    in.throttle = 0.3;
    bool deployed = false;
    for (int i = 0; i < 240 * 12 && !deployed; ++i) {
        ac.step(kDt, in, ground, atm);
        deployed = ac.reverserPhase(0) == 1 && ac.groundSpoilersActive();
    }
    CHECK(deployed);
}

TEST_CASE("top g: spoiler panels dump lift where they are and shed their own vortices; gear drag pitches nose down") {
    Aircraft ac(jet());
    const AeroModel& m = ac.aeroModel();
    CHECK(m.hasSpoilers());
    AeroOutputs out;
    m.prepare(out);
    AeroInputs in;
    in.cg = ac.massProperties().cg;
    const Real v = 128, a = 0.03;
    in.velocityBody = {v * std::cos(a), 0, v * std::sin(a)};
    auto lift = [&](Real sb) {
        in.controls.speedbrake = sb;
        for (int k = 0; k < 3; ++k) m.evaluate(in, out);
        return out.force.x * std::sin(a) - out.force.z * std::cos(a);
    };
    const Real clean = lift(0);
    // The strips behind the panels lose their lift and separate (red tufts);
    // the root, with no panels, stays attached. (The outer wing loses lift
    // too: past the panels' outer edge the circulation steps back up, and
    // that edge's vortex washes the outer wing down like an inboard tip.)
    std::vector<Real> clClean(static_cast<std::size_t>(m.stripCount()));
    for (int i = 0; i < m.stripCount(); ++i) clClean[static_cast<std::size_t>(i)] = out.strips[static_cast<std::size_t>(i)].cl;
    const Real dumped = lift(1);
    CHECK(dumped < 0.8 * clean);
    for (int i = m.firstStrip(0); i < m.endStrip(0); ++i) {
        const Real y = std::abs(m.stripGeometry(i).position.y);
        if (y > 5 && y < 10) {
            CHECK(out.strips[i].cl < 0.7 * clClean[static_cast<std::size_t>(i)]);
            CHECK(out.strips[i].attached < 0.5);
        }
        if (y < 3.8) CHECK(out.strips[i].attached > 0.99);
    }
    // The wake: a clean wing sheds one rolled-up vortex a side; with the
    // spoilers out the lift steps down at their edges, so it sheds more.
    auto vortices = [&](Real sb) {
        Aircraft a2(jet());
        SimpleWeather atm;
        FlatGround ground(0);
        const TrimResult tr = trimAt(a2, 10000, ktToMs(250) * 1.16);
        Wake w(a2.aeroModel());
        PilotInput pin = tr.input;
        a2.setCockpitControl("speedbrake.lever", sb > 0 ? 2 : 0);
        for (int i = 0; i < 240 * 4; ++i) {
            a2.step(kDt, pin, ground, atm);
            w.update(a2, atm, &ground);
        }
        return w.pair(1.0).vortices;
    };
    CHECK(vortices(1) > vortices(0));
    // Gear drag below the CG: nose down when it comes out.
    Aircraft up(jet()), down(jet());
    up.setGearImmediately(false);
    down.setGearImmediately(true);
    SimpleWeather atm;
    FlatGround ground(0);
    const TrimResult tu = trimAt(up, 5000, ktToMs(200) * 1.08, 1, false);
    const TrimResult td = trimAt(down, 5000, ktToMs(200) * 1.08, 1, true);
    CHECK(tu.converged && td.converged);
    CHECK(td.trim > tu.trim);  // more nose-up trim needed against the gear's nose-down drag moment
}
