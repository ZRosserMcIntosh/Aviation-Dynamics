// Aviation Dynamics — engine-out asymmetry (split throttles, windmilling drag,
// minimum control speed), generators and the battery, hydraulics (power
// gating, free-fall gear, accumulator brakes) and the sensed engine channels.
// SPDX-License-Identifier: MIT
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <vector>

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
const AircraftDefinition& single() {
    static const AircraftDefinition d = loadReference("tupi_t180");
    return d;
}

constexpr Real kDt = 1.0 / 240;

// Trimmed level flight at a pressure altitude and CAS, flap detent and gear.
TrimResult trimJet(Aircraft& ac, Real altFt, Real kcas, int flap = 0, bool gear = false) {
    SimpleWeather atm;
    ac.setGearImmediately(gear);
    TrimRequest req;
    req.altitude = ftToM(altFt);
    req.tas = tasFromCas(ktToMs(kcas), atm.atmosphere().at(ftToM(altFt)).density);
    req.flapDetent = flap;
    return trim(ac, req, atm);
}

// Fly `seconds` holding a pitch attitude with the elevator, wings held level
// (or at a bank) with the ailerons; the yaw channel as given.
void hold(Aircraft& ac, PilotInput& in, const TrimResult& tr, Real seconds, Real pitchDeg, const GroundQuery& g,
          const AtmosphereField& atm, Real bankDeg = 0) {
    for (int i = 0; i < static_cast<int>(seconds / kDt); ++i) {
        const AircraftTelemetry& t = ac.telemetry();
        in.pitch = clamp(tr.elevator + 2.0 * (degToRad(pitchDeg) - t.pitch) - 0.8 * t.omega.y, -1.0, 1.0);
        in.roll = clamp(-2.0 * (t.roll - degToRad(bankDeg)) - 0.5 * t.omega.x, -1.0, 1.0);
        ac.step(kDt, in, g, atm);
    }
}

bool hasEvent(const std::vector<FlightEvent>& ev, FlightEventType t, int index = -2) {
    for (const auto& e : ev)
        if (e.type == t && (index == -2 || e.index == index)) return true;
    return false;
}

}  // namespace

TEST_CASE("engine out: split throttles yaw toward the retarded engine; the levers are reported") {
    SimpleWeather atm;
    FlatGround ground(0);
    for (const int retarded : {1, 0}) {
        Aircraft ac(jet());
        ac.setYawDamper(false);
        const TrimResult tr = trimJet(ac, 10000, 200);
        CHECK(tr.converged);
        PilotInput in = tr.input;
        in.splitThrottles = true;
        in.throttles.fill(tr.throttle);
        in.throttles[static_cast<std::size_t>(retarded)] = 0;  // that engine to idle
        const Real heading0 = ac.telemetry().heading;
        hold(ac, in, tr, 8, radToDeg(tr.theta), ground, atm);
        const AircraftTelemetry& t = ac.telemetry();
        // Right engine idle: the live left engine yaws the nose right, and
        // wings level it settles into a steady sideslip with the air coming
        // from the left (beta < 0), the heading having swung right.
        const Real sign = retarded == 1 ? 1.0 : -1.0;
        const Real turned = sign * wrapPi(t.heading - heading0);
        std::printf("    engine %d idle: heading %+.1f deg in 8 s, beta %+.1f deg, N1 %.0f / %.0f %%\n", retarded + 1,
                    radToDeg(wrapPi(t.heading - heading0)), radToDeg(t.beta), ac.turbofanState(0).n1 * 100,
                    ac.turbofanState(1).n1 * 100);
        CHECK(turned > degToRad(2));
        CHECK(sign * t.beta < -degToRad(0.5));
        CHECK(ac.turbofanState(retarded).n1 < ac.turbofanState(1 - retarded).n1 - 0.2);
        CHECK_NEAR(t.throttles[static_cast<std::size_t>(1 - retarded)], tr.throttle, 1e-12);
        CHECK_NEAR(t.throttles[static_cast<std::size_t>(retarded)], 0, 1e-12);
    }
    // Without the split flag `throttle` still drives every engine.
    Aircraft ac(jet());
    const TrimResult tr = trimJet(ac, 10000, 200);
    PilotInput in = tr.input;
    in.throttles.fill(0);
    in.splitThrottles = false;
    ac.step(kDt, in, ground, atm);
    CHECK_NEAR(ac.telemetry().throttles[0], tr.throttle, 1e-12);
    CHECK_NEAR(ac.telemetry().throttles[1], tr.throttle, 1e-12);
}

TEST_CASE("engine out: an unlit turbofan windmills with a drag of q x CdA (default 0.12 x the fan disc)") {
    const TurbofanEngine e(jet().turbofans[0]);
    CHECK_NEAR(e.fanDiameter(), 1.27, 1e-9);
    CHECK_NEAR(e.windmillDragArea(), 0.152, 1e-9);  // declared
    TurbofanDef d;
    d.reverser.cowlRadius = 0.7;
    const TurbofanEngine dflt(d);
    CHECK_NEAR(dflt.windmillDragArea(), 0.12 * kPi * 0.49, 1e-9);  // from the cowl radius
    TurbofanState s;  // not running, not starting: unlit
    TurbofanInputs in;
    const Real v = ktToMs(150);
    in.mach = v / 340.294;
    e.evaluate(s, in);
    const Real q = 0.5 * isa::kSeaLevelDensity * v * v;
    std::printf("    windmilling drag at 150 KTAS, sea level: %.0f N (q %.0f Pa, CdA %.3f m^2)\n", -s.thrust, q,
                e.windmillDragArea());
    CHECK(s.thrust < 0);
    CHECK_NEAR(s.thrust, -q * 0.152, 3.0);
    CHECK(s.fuelFlow == 0);
    in.mach = 0;
    e.evaluate(s, in);
    CHECK(s.thrust == 0);  // at rest, no drag
}

namespace {

// Steady engine-out lateral-directional balance at one speed and bank: the
// sideslip, aileron and rudder that zero the rolling and yawing moments and
// the lateral acceleration (take-off flap, gear up, 2,000 ft, left engine at
// take-off thrust, right engine unlit). The rudder may exceed its travel:
// that is the answer when it does.
struct LateralBalance {
    bool converged = false;
    Real beta = 0, aileron = 0, rudder = 0;
    Real tas = 0;
};

LateralBalance engineOutBalance(const AircraftDefinition& def, Real kcas, Real bankDeg) {
    SimpleWeather atm;
    FlatGround ground(0);
    LateralBalance out;
    Aircraft ac(def);
    ac.setYawDamper(false);
    const TrimResult tr = trimJet(ac, 2000, kcas, 1, false);
    if (!tr.converged) return out;
    for (const char* c : {"electrical.battery", "electrical.alternator", "engine1.fuel"}) ac.setCockpitControl(c, 1);
    ac.setCockpitControl("engine2.fuel", 0);
    ac.step(kDt, tr.input, ground, atm);  // the right engine is now unlit (its spool speeds do not matter)
    const Real v = tr.tas, alpha = tr.alpha, phi = degToRad(bankDeg);
    const Real m = ac.massProperties().mass;
    const Real rho = atm.atmosphere().at(ftToM(2000)).density;
    const Real norm = 0.5 * rho * v * v * def.referenceArea * def.referenceSpan;
    out.tas = v;
    auto residual = [&](const Real x[3], Real r[3]) {
        DynamicState st = tr.state;
        st.body.attitude = Quat::fromEuler(phi, alpha, 0);
        st.body.velocity = st.body.attitude.rotate({v * std::cos(alpha) * std::cos(x[0]), v * std::sin(x[0]), v * std::sin(alpha) * std::cos(x[0])});
        st.body.omega = {};
        PilotInput in = tr.input;
        in.splitThrottles = true;
        in.throttles = {1.0, 0.0, 0.0, 0.0};
        in.roll = x[1];
        in.yaw = x[2];
        ac.setState(st);
        ac.prepareContext(in, ground, atm);
        const Aircraft::Derivatives d = ac.evaluate(st);
        r[0] = d.momentBody.x / norm;
        r[1] = d.momentBody.z / norm;
        r[2] = (d.forceBody.y / m + kStandardGravity * std::cos(alpha) * std::sin(phi)) / kStandardGravity;
    };
    auto normOf = [](const Real r[3]) { return std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]); };
    auto det3 = [](const Real a[3][3]) {
        return a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
               a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
    };
    Real x[3] = {0, 0, 0}, r0[3];
    residual(x, r0);
    for (int it = 0; it < 30 && !out.converged; ++it) {
        if (std::getenv("AVD_VMCA_DEBUG"))
            std::printf("      it %d: beta %.4f ail %.3f rud %.3f  r %.5f %.5f %.5f\n", it, x[0], x[1], x[2], r0[0], r0[1], r0[2]);
        if (normOf(r0) < 2e-4) { out.converged = true; break; }
        Real J[3][3];
        for (int j = 0; j < 3; ++j) {
            Real xp[3] = {x[0], x[1], x[2]};
            const Real h = j == 0 ? 1e-3 : 1e-2;
            xp[j] += h;
            Real rp[3];
            residual(xp, rp);
            for (int i = 0; i < 3; ++i) J[i][j] = (rp[i] - r0[i]) / h;
        }
        const Real det = det3(J);
        if (std::abs(det) < 1e-18) break;
        Real dx[3];
        for (int j = 0; j < 3; ++j) {
            Real Jj[3][3];
            for (int i = 0; i < 3; ++i)
                for (int k = 0; k < 3; ++k) Jj[i][k] = k == j ? -r0[i] : J[i][k];
            dx[j] = det3(Jj) / det;
        }
        // Backtracking: the step (or a fraction of it) must lower the residual.
        bool accepted = false;
        for (Real scale = 1.0; scale > 0.05 && !accepted; scale *= 0.5) {
            Real xn[3], rn[3];
            for (int j = 0; j < 3; ++j) xn[j] = x[j] + clamp(scale * dx[j], -0.25, 0.25);
            xn[2] = clamp(xn[2], -3.0, 3.0);
            residual(xn, rn);
            if (normOf(rn) < normOf(r0)) {
                for (int j = 0; j < 3; ++j) { x[j] = xn[j]; r0[j] = rn[j]; }
                accepted = true;
            }
        }
        if (!accepted) break;
    }
    out.beta = x[0];
    out.aileron = x[1];
    out.rudder = x[2];
    return out;
}

}  // namespace

TEST_CASE("engine out: the rudder demand grows as the speed falls; the minimum control speed is reported") {
    // Wings level, from 150 KCAS down to where a steady solution still exists
    // (the wing near its stall ends the sweep). V2 is taken as 1.13 Vs1g at
    // take-off flap. The rudder demand scales with 1/V^2 (constant engine
    // moment against a q-scaled rudder), so the speed at full travel is
    // extrapolated from the lowest solved point when it is not reached.
    const Real vsTo = [] {
        Aircraft ac(jet());
        const Real cl = trimmedMaxLiftCoefficient(ac.aeroModel(), jet().referenceArea, ac.massProperties().cg,
                                                  jet().controls.flapDetents[1], 0.15);
        return msToKt(std::sqrt(2 * ac.massProperties().mass * 9.80665 / (1.225 * jet().referenceArea * cl)));
    }();
    const Real v2 = 1.13 * vsTo;
    std::printf("    Vs1g take-off flap %.0f KCAS, V2 ~ %.0f KCAS\n", vsTo, v2);
    std::vector<Real> speeds;
    for (Real k = 150; k >= 0.98 * vsTo; k -= 5) speeds.push_back(k);
    speeds.push_back(v2);
    std::sort(speeds.begin(), speeds.end(), [](Real a, Real b) { return a > b; });
    Real prev = 0, lowest = -1, lowestRudder = 0, rudderAtV2 = 0, vmca = -1;
    for (const Real kcas : speeds) {
        const LateralBalance b = engineOutBalance(jet(), kcas, 0);
        if (!b.converged) {
            std::printf("    %3.0f KCAS: no steady solution (the wing is at its stall)\n", kcas);
            break;
        }
        std::printf("    %3.0f KCAS: rudder %+.2f, aileron %+.2f, beta %+.1f deg\n", kcas, b.rudder, b.aileron, radToDeg(b.beta));
        CHECK(b.rudder < 0);  // left rudder against the live left engine
        if (lowest > 0) CHECK(std::abs(b.rudder) > std::abs(prev));  // grows as the speed falls
        if (kcas == v2) rudderAtV2 = b.rudder;
        if (std::abs(b.rudder) >= 1.0) {
            vmca = lowest > 0 ? lowest + (kcas - lowest) * (1.0 - std::abs(prev)) / (std::abs(b.rudder) - std::abs(prev)) : kcas;
            break;
        }
        prev = b.rudder;
        lowest = kcas;
        lowestRudder = b.rudder;
    }
    CHECK(lowest > 0 && lowest <= v2 + 1);
    const bool reached = vmca > 0;
    if (!reached) vmca = lowest * std::sqrt(std::abs(lowestRudder));  // 1/V^2 trend
    std::printf("    minimum control speed, wings level: %s%.0f KCAS (%s the %.0f KCAS take-off-flap stall)\n",
                reached ? "" : "about ", vmca, vmca < vsTo ? "below" : "above", vsTo);
    CHECK_RANGE(vmca, 40, 135);
    // Bank into the live engine: 5 deg over-compensates on this fin (the
    // sideslip it needs yaws the nose left harder than the engine yaws it
    // right, so the rudder goes the other way); a degree or two is the
    // zero-rudder bank.
    const LateralBalance b5 = engineOutBalance(jet(), v2, -5);
    if (b5.converged)
        std::printf("    at V2 with 5 deg of bank into the live engine: rudder %+.2f, beta %+.1f deg\n", b5.rudder, radToDeg(b5.beta));
    CHECK(!b5.converged || b5.rudder > rudderAtV2);
    // At V2 the airplane is controllable with rudder to spare, and the
    // windmilling engine's drag is a measurable part of the demand.
    CHECK(rudderAtV2 < 0 && rudderAtV2 > -1.0);
    AircraftDefinition noWindmill = jet();
    for (TurbofanDef& e : noWindmill.turbofans) e.windmillDragArea = 0;
    const LateralBalance without = engineOutBalance(noWindmill, v2, 0);
    std::printf("    at V2, wings level: rudder %+.3f with the windmilling drag, %+.3f without\n", rudderAtV2, without.rudder);
    CHECK(without.converged);
    CHECK(std::abs(rudderAtV2) > std::abs(without.rudder));
}

TEST_CASE("generators: generator 1 failure leaves the bus on generator 2; per-generator switches and events") {
    Aircraft ac(jet());
    SimpleWeather atm;
    FlatGround g(0);
    ac.setEngineRunning(true);
    ac.placeOnGround(0, 0, 0, g, atm);
    PilotInput idle;
    CHECK(ac.electrical().generatorCount() == 2);
    CHECK(ac.electrical().generatorOnline(0) && ac.electrical().generatorOnline(1));
    CHECK_NEAR(ac.electrical().generatorCapacity(), 600, 1e-9);
    ac.drainEvents();
    ac.electrical().setGeneratorFailed(0, true);
    for (int i = 0; i < 240 * 2; ++i) ac.step(kDt, idle, g, atm);
    CHECK(!ac.electrical().generatorOnline(0));
    CHECK(ac.electrical().generatorOnline(1));
    CHECK(ac.electrical().alternatorOnline());
    CHECK_NEAR(ac.electrical().busVoltage(), 28, 1e-9);
    CHECK_NEAR(ac.electrical().generatorCapacity(), 300, 1e-9);
    {
        const auto ev = ac.drainEvents();
        CHECK(hasEvent(ev, FlightEventType::AlternatorOffline, 0));
        CHECK(!hasEvent(ev, FlightEventType::AlternatorOffline, 1));
    }
    // Generator 2's own switch: now the battery carries the bus; the master
    // reads OFF while any generator is off, and sets them all back on.
    CHECK(ac.cockpitControl("electrical.generator2") == 1);
    ac.setCockpitControl("electrical.generator2", 0);
    for (int i = 0; i < 240; ++i) ac.step(kDt, idle, g, atm);
    CHECK(!ac.electrical().alternatorOnline());
    CHECK(ac.electrical().busVoltage() > 20 && ac.electrical().busVoltage() < 26.5);
    CHECK(ac.electrical().batteryCurrent() > 10);
    CHECK(ac.cockpitControl("electrical.alternator") == 0);
    ac.setCockpitControl("electrical.alternator", 1);
    for (int i = 0; i < 240; ++i) ac.step(kDt, idle, g, atm);
    CHECK(ac.electrical().generatorOnline(1));
    CHECK(!ac.electrical().generatorOnline(0));  // still failed
    CHECK(hasEvent(ac.drainEvents(), FlightEventType::AlternatorOnline, 1));
}

TEST_CASE("dual flame-out: generators drop, the battery carries the essentials, hydraulics fall to free fall") {
    Aircraft ac(jet());
    SimpleWeather atm;
    FlatGround ground(0);
    const TrimResult tr = trimJet(ac, 15000, 250);
    CHECK(tr.converged);
    for (const char* c : {"electrical.battery", "electrical.alternator", "engine1.fuel", "engine2.fuel"}) ac.setCockpitControl(c, 1);
    PilotInput in = tr.input;
    hold(ac, in, tr, 8, radToDeg(tr.theta), ground, atm);
    CHECK(ac.hydraulicSystemCount() == 2);
    CHECK(ac.hydraulicPressure(0) > 0.95 && ac.hydraulicPressure(1) > 0.95);
    CHECK_NEAR(ac.telemetry().gearPower, 1, 1e-9);
    CHECK(ac.electrical().generatorOnline(0) && ac.electrical().generatorOnline(1));
    ac.drainEvents();
    ac.setCockpitControl("engine1.fuel", 0);
    ac.setCockpitControl("engine2.fuel", 0);
    hold(ac, in, tr, 30, -2, ground, atm);
    CHECK(!ac.turbofanState(0).running && !ac.turbofanState(1).running);
    CHECK(!ac.electrical().alternatorOnline());
    CHECK(hasEvent(ac.drainEvents(), FlightEventType::AlternatorOffline, 1));
    CHECK(ac.electrical().busVoltage() >= ac.electrical().busMinimum());  // the battery
    CHECK(ac.electrical().batteryCurrent() > 20);                          // avionics, AHRS, gauges
    CHECK(ac.hydraulicPressure(0) < 0.05 && ac.hydraulicPressure(1) < 0.05);  // windmilling cores pump nothing; the electric pump needs a generator
    CHECK_NEAR(ac.telemetry().gearPower, 0, 0.05);
    CHECK(ac.instruments().state().engines[0].n1 > 10);  // the indication is alive on the battery
    // Normal gear, flaps and speedbrake: the levers move, nothing else does.
    ac.setCockpitControl("gear.lever", 1);
    ac.setCockpitControl("flaps.lever", 1);
    ac.setCockpitControl("speedbrake.lever", 1);
    hold(ac, in, tr, 10, -2, ground, atm);
    CHECK_NEAR(ac.gearPosition(), 0, 1e-9);
    CHECK_NEAR(ac.flapAngle(), 0, 1e-9);
    CHECK_NEAR(ac.speedbrakePosition(), 0, 1e-9);
    // Emergency release: the gear falls at 35 % of its normal rate (7 s).
    ac.setCockpitControl("gear.emergency", 1);
    hold(ac, in, tr, 10, -2, ground, atm);
    CHECK_RANGE(ac.gearPosition(), 0.4, 0.6);
    hold(ac, in, tr, 15, -2, ground, atm);
    CHECK_NEAR(ac.gearPosition(), 1, 1e-9);
    CHECK(ac.telemetry().gearEmergency);
    // Relight engine 2 (a windmilling core relights): system B comes back,
    // the flaps run on it.
    ac.setCockpitControl("engine2.fuel", 1);
    bool lit = false;
    for (Real t = 0; t < 60 && !lit; t += 0.5) {
        hold(ac, in, tr, 0.5, -2, ground, atm);
        lit = ac.turbofanState(1).running;
    }
    CHECK(lit);
    hold(ac, in, tr, 8, -2, ground, atm);
    CHECK(ac.hydraulicPressure(1) > 0.9);
    CHECK(ac.flapAngle() > degToRad(5));
    CHECK(ac.electrical().generatorOnline(1));
}

TEST_CASE("generated jets: two generators and two hydraulic systems each; a dual flame-out stops the gear") {
    // The template's electrical and hydraulics blocks reach every generated
    // jet (tools/jetgen). Regression: the generated jets carried one aggregate
    // alternator and no hydraulics block, so a dual flame-out left the gear
    // and flaps working.
    SimpleWeather atm;
    FlatGround ground(0);
    for (const char* id : {"phenom300", "cj4", "learjet45", "crj700", "b738"}) {
        std::printf("    %s\n", id);
        Aircraft ac(loadReference(id));
        const TrimResult tr = trimJet(ac, 15000, 250);
        CHECK(tr.converged);
        for (const char* c : {"electrical.battery", "electrical.alternator", "engine1.fuel", "engine2.fuel"}) ac.setCockpitControl(c, 1);
        PilotInput in = tr.input;
        hold(ac, in, tr, 8, radToDeg(tr.theta), ground, atm);
        CHECK(ac.electrical().generatorCount() == 2);
        CHECK(ac.electrical().generatorOnline(0) && ac.electrical().generatorOnline(1));
        CHECK(ac.hydraulicSystemCount() == 2);
        CHECK(ac.hydraulicPressure(0) > 0.95 && ac.hydraulicPressure(1) > 0.95);
        CHECK_NEAR(ac.telemetry().gearPower, 1, 1e-9);
        ac.setCockpitControl("engine1.fuel", 0);
        ac.setCockpitControl("engine2.fuel", 0);
        hold(ac, in, tr, 30, -2, ground, atm);
        CHECK(!ac.turbofanState(0).running && !ac.turbofanState(1).running);
        CHECK(!ac.electrical().alternatorOnline());
        CHECK(ac.electrical().busVoltage() >= ac.electrical().busMinimum());  // the battery carries the essentials
        CHECK(ac.hydraulicPressure(0) < 0.05 && ac.hydraulicPressure(1) < 0.05);
        CHECK_NEAR(ac.telemetry().gearPower, 0, 0.05);  // the gear actuator's rate is zero
        ac.setCockpitControl("gear.lever", 1);
        hold(ac, in, tr, 10, -2, ground, atm);
        CHECK_NEAR(ac.gearPosition(), 0, 1e-9);
    }
}

TEST_CASE("hydraulics: with both engines off the brakes hold for the accumulator's applications, then fade") {
    Aircraft ac(jet());
    SimpleWeather atm;
    FlatGround g(0);
    ac.placeOnGround(0, 0, 0, g, atm);  // cold and dark: the parking brake set on arrival spent one
    ac.setCockpitControl("brakes.parking", 0);
    ac.setCockpitControl("electrical.battery", 1);
    CHECK(ac.brakeAccumulatorApplications() == 5);
    const Quat q = ac.state().body.attitude;
    auto roll = [&](Real brake, Real seconds) {
        DynamicState s = ac.state();
        s.body.velocity = q.rotate({15, 0, 0});
        s.tyreLongitudinal.fill(0);
        ac.setState(s);
        PilotInput in;
        in.brakeLeft = in.brakeRight = brake;
        for (int i = 0; i < static_cast<int>(seconds / kDt); ++i) ac.step(kDt, in, g, atm);
        return (15 - ac.telemetry().groundSpeed) / seconds;  // mean deceleration m/s^2
    };
    const Real coasting = roll(0, 2);
    std::printf("    coasting %.2f m/s^2", coasting);
    for (int k = 0; k < 5; ++k) {
        const Real d = roll(1, 2);
        std::printf(", application %d: %.2f", k + 1, d);
        CHECK(d > coasting + 1.0);
        roll(0, 1);  // released: the next press is a new application
        CHECK(ac.brakeAccumulatorApplications() == 4 - k);
    }
    const Real spent = roll(1, 2);
    std::printf(", spent: %.2f m/s^2\n", spent);
    CHECK(ac.brakeAccumulatorApplications() == 0);
    CHECK(spent < coasting + 0.3);
    CHECK_NEAR(ac.telemetry().brakePower, 0, 1e-9);
    // Engines running: the systems pressurise and recharge the accumulator.
    ac.setEngineRunning(true);
    PilotInput idle;
    for (int i = 0; i < 240 * 6; ++i) ac.step(kDt, idle, g, atm);
    CHECK(ac.hydraulicPressure(0) > 0.9);
    CHECK(ac.brakeAccumulatorApplications() == 6);
    CHECK_NEAR(ac.telemetry().brakePower, 1, 1e-9);
}

TEST_CASE("electrical: the electric fuel pump draws its breaker's current, through its breaker only") {
    for (const bool isJet : {true, false}) {
        Aircraft ac(isJet ? jet() : single());
        SimpleWeather atm;
        FlatGround g(0);
        ac.placeOnGround(0, 0, 0, g, atm);
        ac.setCockpitControl("electrical.battery", 1);
        PilotInput in;
        for (int i = 0; i < 240; ++i) ac.step(kDt, in, g, atm);
        const Real i0 = ac.electrical().batteryCurrent();
        ac.setCockpitControl("fuel.auxpump", 1);
        for (int i = 0; i < 240; ++i) ac.step(kDt, in, g, atm);
        const Real declared = isJet ? 8.0 : 4.0;
        CHECK_NEAR(ac.electrical().batteryCurrent() - i0, declared, 1e-9);
        ac.setCockpitControl("electrical.load.fuel_pump", 0);  // breaker pulled
        for (int i = 0; i < 240; ++i) ac.step(kDt, in, g, atm);
        CHECK_NEAR(ac.electrical().batteryCurrent(), i0, 1e-9);
        ac.setCockpitControl("fuel.auxpump", 0);
        ac.setCockpitControl("electrical.load.fuel_pump", 1);
        for (int i = 0; i < 240; ++i) ac.step(kDt, in, g, atm);
        CHECK_NEAR(ac.electrical().batteryCurrent(), i0, 1e-9);
    }
}

TEST_CASE("electrical: a battery run flat raises BatteryDepleted once and the bus dies") {
    Aircraft ac(single());
    SimpleWeather atm;
    FlatGround g(0);
    ac.placeOnGround(0, 0, 0, g, atm);
    ac.setCockpitControl("electrical.battery", 1);
    ac.electrical().setBatteryCharge(0.0005);  // ~23 A s: seconds at the panel's 1 A (avionics off)
    ac.drainEvents();
    PilotInput in;
    int depleted = 0;
    for (int i = 0; i < 240 * 60; ++i) {
        ac.step(kDt, in, g, atm);
        for (const auto& e : ac.drainEvents()) depleted += e.type == FlightEventType::BatteryDepleted;
    }
    CHECK(depleted == 1);
    CHECK(ac.electrical().batteryDepleted());
    CHECK_NEAR(ac.electrical().busVoltage(), 0, 0);
    CHECK(ac.instruments().state().annunciators == 0);
}

TEST_CASE("indications: sensed N1 lags a slam acceleration and blanks without gauge power") {
    Aircraft ac(jet());
    SimpleWeather atm;
    FlatGround g(0);
    ac.setEngineRunning(true);
    ac.placeOnGround(0, 0, 0, g, atm);
    const CockpitState& cs = ac.instruments().state();
    CHECK(cs.engineCount == 2);
    CHECK_NEAR(cs.engines[0].n1, ac.turbofanState(0).n1 * 100, 0.5);
    CHECK_NEAR(cs.engines[1].n2, ac.turbofanState(1).n2 * 100, 0.5);
    CHECK_NEAR(cs.engines[0].itt, ac.turbofanState(0).itt, 2);
    PilotInput in;
    in.throttle = 0.7;
    for (int i = 0; i < 240; ++i) ac.step(kDt, in, g, atm);
    const Real truth1 = ac.turbofanState(0).n1 * 100, sensed1 = cs.engines[0].n1;
    std::printf("    1 s into the slam: N1 truth %.1f %%, indicated %.1f %%\n", truth1, sensed1);
    CHECK(truth1 > jet().turbofans[0].idleN1 * 100 + 3);
    CHECK(sensed1 < truth1 - 1.0);
    CHECK(sensed1 > jet().turbofans[0].idleN1 * 100);
    for (int i = 0; i < 240 * 15; ++i) ac.step(kDt, in, g, atm);
    CHECK_NEAR(cs.engines[0].n1, ac.turbofanState(0).n1 * 100, 0.3);
    CHECK_NEAR(cs.engines[1].itt, ac.turbofanState(1).itt, 10);  // 3 s sender lag on a still-rising ITT
    CHECK_NEAR(cs.engines[0].fuelFlow, ac.turbofanState(0).fuelFlow, 0.01);
    CHECK_NEAR(cs.engines[1].oilPressure, ac.turbofanState(1).oilPressure, 1e4);
    // Engine indication breaker pulled: every channel blank, the engines still run.
    ac.setCockpitControl("electrical.load.engine_gauges", 0);
    for (int i = 0; i < 240 * 4; ++i) ac.step(kDt, in, g, atm);
    CHECK(ac.turbofanState(0).n1 > 0.5);
    CHECK(cs.engines[0].n1 < 0.5 && cs.engines[1].n2 < 0.5);  // needles down within seconds
    for (int i = 0; i < 240 * 20; ++i) ac.step(kDt, in, g, atm);
    CHECK(cs.engines[0].itt < 5 && cs.engines[0].fuelFlow < 1e-3 && cs.engines[0].oilPressure < 500);  // the lagged senders too
    // The single's tach is mechanical: it reads with the master off.
    Aircraft single_(single());
    single_.setEngineRunning(true, 2200);
    single_.placeOnGround(0, 0, 0, g, atm);
    single_.setCockpitControl("electrical.alternator", 0);
    single_.setCockpitControl("electrical.battery", 0);
    PilotInput cruise;
    cruise.throttle = 0.6;
    for (int i = 0; i < 240 * 4; ++i) single_.step(kDt, cruise, g, atm);
    const CockpitState& sc = single_.instruments().state();
    CHECK(sc.engineCount == 1);
    CHECK(sc.engines[0].n1 > 1500);
    CHECK_NEAR(sc.engines[0].n1, sc.tachometerRpm, 1e-6);
    CHECK(sc.engines[0].oilPressure < 100);  // electric sender: blank
}

TEST_CASE("battery: the single's ammeter settles to a few amps in cruise") {
    Aircraft ac(single());
    SimpleWeather atm;
    FlatGround g(0);
    TestPilot pilot;
    startTrimmedFlight(ac, {100, 1500, 0}, atm, g, &pilot);
    const int n = avdtest::kOptimisedBuild ? 240 * 600 : 240 * 60;
    for (int i = 0; i < n; ++i) ac.step(kDefaultDt, pilot.update(ac, kDefaultDt), g, atm);
    const Real amps = ac.instruments().state().ammeter;
    std::printf("    ammeter after %d min of cruise: %+.1f A (battery %.1f %%)\n", n / (240 * 60), amps,
                ac.electrical().batteryCharge() * 100);
    CHECK_RANGE(amps, 0, 5);
    CHECK(ac.electrical().alternatorOnline());
}

TEST_CASE("battery: VOLTS lights within two minutes of an alternator failure at normal loads") {
    Aircraft ac(single());
    SimpleWeather atm;
    FlatGround g(0);
    TestPilot pilot;
    startTrimmedFlight(ac, {100, 1500, 0}, atm, g, &pilot);
    for (int i = 0; i < 240 * 5; ++i) ac.step(kDefaultDt, pilot.update(ac, kDefaultDt), g, atm);
    ac.electrical().setAlternatorFailed(true);
    for (int i = 0; i < 240 * 2; ++i) ac.step(kDefaultDt, pilot.update(ac, kDefaultDt), g, atm);
    const CockpitState& cs = ac.instruments().state();
    // The surface charge left by the alternator holds the bus above 24.5 V at first.
    CHECK(!(cs.annunciators & kAnnVolts));
    CHECK(cs.ammeter < -5 && cs.ammeter > -10);  // the panel's ~7 A on the battery
    Real t = 2;
    while (!(cs.annunciators & kAnnVolts) && t < 300) {
        ac.step(kDefaultDt, pilot.update(ac, kDefaultDt), g, atm);
        t += kDefaultDt;
    }
    std::printf("    VOLTS lamp %.0f s after the alternator failed (bus %.2f V)\n", t, cs.voltmeter);
    CHECK(t < 120);
    CHECK(t > 10);
}

TEST_CASE("indications: the jet (no vacuum system) never shows LOW VAC; the bus minimum is the definition's") {
    Aircraft ac(jet());
    SimpleWeather atm;
    FlatGround g(0);
    ac.placeOnGround(0, 0, 0, g, atm);
    ac.setCockpitControl("electrical.battery", 1);
    PilotInput in;
    for (int i = 0; i < 240 * 2; ++i) ac.step(kDt, in, g, atm);
    const CockpitState& cs = ac.instruments().state();
    CHECK(cs.annunciators & kAnnOilPressure);  // engines stopped: lamps are alive
    CHECK(!(cs.annunciators & kAnnLowVacuum));
    CHECK_NEAR(ac.electrical().busMinimum(), 18, 1e-9);
    ElectricalDef twelve;
    twelve.batteryVoltage = 12;
    CHECK_NEAR(twelve.busMinimum(), 9, 1e-9);
}
