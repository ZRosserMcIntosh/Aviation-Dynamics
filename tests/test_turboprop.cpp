// Aviation Dynamics — the free-turbine turboprop (PT6A class), the
// constant-speed / feathering / reversing propeller, and the King Air 350.
// Published comparisons: docs/validation/king-air-350-validation.md.
// SPDX-License-Identifier: MIT
#include "avd/units.hpp"
#include "avd_test.hpp"
#include "scenarios.hpp"

using namespace avd;
using namespace avd::scenarios;
using namespace avd::units;

namespace {

const AircraftDefinition& kingAir() {
    static const AircraftDefinition d = loadReference("king_air_350");
    return d;
}

constexpr Real kDt = 1.0 / 240;

// Prop lever position for a governed rpm.
Real propLeverFor(Real rpm) {
    const TurbopropDef& e = kingAir().turboprops[0];
    return (rpm - e.minGovernedRpm) / (e.ratedPropRpm - e.minGovernedRpm);
}

TrimResult trimLevel(Aircraft& ac, Real altFt, Real tas, int flap = 0, bool gear = false, Real propRpm = 1700) {
    SimpleWeather atm;
    ac.setGearImmediately(gear);
    TrimRequest req;
    req.altitude = ftToM(altFt);
    req.tas = tas;
    req.flapDetent = flap;
    req.propLever = propLeverFor(propRpm);
    return trim(ac, req, atm);
}

// MTOW: full-ish wing tanks and a cabin load (15,000 lb).
void loadToMtow(Aircraft& ac) {
    ac.fuel().setTankQuantity(0, 0.9);
    ac.fuel().setTankQuantity(1, 0.9);
    ac.refreshMass();
    ac.setStationMass(1, ac.stationMass(1) + kingAir().maxTakeoffMass - ac.massProperties().mass);
}

Real ias(const Aircraft& ac) { return ac.instruments().readings().indicatedAirspeed; }

}  // namespace

TEST_CASE("turboprop: PT6A-60A flat rating to the torque limit, the ITT-limited lapse, take-off Ng/ITT/fuel") {
    const TurbopropDef& d = kingAir().turboprops[0];
    const TurbopropEngine e(d);
    const StandardAtmosphere atm;
    auto shp = [&](Real ft, Real dT, Real mach) {
        AtmosphereConditions c;
        c.temperatureDeviation = dT;
        const AirState a = StandardAtmosphere(c).at(ftToM(ft));
        return e.availablePower(a.pressure, a.temperature, mach) / kHorsepower;
    };
    // Flat rated: 1,050 shp at sea level up to ISA+20 and at 10,000 ft;
    // ITT-limited above ~15,000 ft in the climb, and in the FL240 cruise
    // about the published maximum-cruise power class.
    std::cerr << "  available shp: SL " << shp(0, 0, 0) << ", SL ISA+20 " << shp(0, 20, 0) << ", 10,000 ft " << shp(10000, 0, 0.3)
              << ", 18,000 ft " << shp(18000, 0, 0.35) << ", FL240 M0.5 " << shp(24000, 0, 0.5) << ", FL300 M0.5 " << shp(30000, 0, 0.5) << "\n";
    CHECK_NEAR(shp(0, 0, 0), 1050, 1);
    CHECK_NEAR(shp(0, 20, 0), 1050, 1);
    CHECK_NEAR(shp(10000, 0, 0.3), 1050, 1);
    CHECK(shp(18000, 0, 0.35) < 1000);
    CHECK_RANGE(shp(24000, 0, 0.5), 650, 900);
    CHECK(shp(30000, 0, 0.5) < shp(24000, 0, 0.5));
    // Take-off at sea level, static, ISA, the propeller at 1,700 rpm: torque
    // limited with Ng, ITT and fuel flow in the PT6A-60A's class, SFC the published 0.548.
    TurbopropState s;
    TurbopropInputs in;
    in.condition = ConditionLever::LowIdle;
    in.fuelAtEngine = true;
    in.powerLever = 1;
    e.setSteady(s, in, d.ratedOmega());
    const Real sfc = s.fuelFlow / s.shaftPower * kHorsepower * 3600 / kPound;
    std::cerr << "  take-off: Ng " << s.ng * 100 << " %, ITT " << s.itt - 273.15 << " degC, torque " << s.torqueFraction * 100
              << " %, " << s.shaftPower / kHorsepower << " shp, " << s.fuelFlow * 3600 / kPound << " lb/h, SFC " << sfc << "\n";
    CHECK_NEAR(s.torqueFraction, 1.0, 1e-3);
    CHECK_RANGE(s.ng, 0.88, 1.0);
    CHECK_RANGE(s.itt - 273.15, 700, 820);
    CHECK_NEAR(sfc, 0.548, 0.01);
    // Low idle: ~100 lb/h, ITT ~600 degC; high idle is faster and burns more.
    in.powerLever = 0;
    e.setSteady(s, in, 1000 * kTwoPi / 60);
    CHECK_NEAR(s.ng, d.ngLowIdle, 1e-9);
    CHECK_NEAR(s.fuelFlow * 3600 / kPound, 100, 2);
    const Real lowIdleFlow = s.fuelFlow;
    in.condition = ConditionLever::HighIdle;
    e.setSteady(s, in, 1000 * kTwoPi / 60);
    CHECK_NEAR(s.ng, d.ngHighIdle, 1e-9);
    CHECK(s.fuelFlow > lowIdleFlow);
    // Free power turbine: at the same gas power the torque rises as Np falls
    // (twice the design torque at rest).
    CHECK_NEAR(e.turbineTorque(d.ratedPower, 0), 2 * d.ratedTorque(), 1e-6 * d.ratedTorque());
    CHECK(e.turbineTorque(d.ratedPower, 0.8 * d.ratedOmega()) > e.turbineTorque(d.ratedPower, d.ratedOmega()));
}

TEST_CASE("turboprop: variable-pitch BEM tables; the fixed-pitch path is unchanged") {
    // Fixed pitch: a zero blade rotation is the old solution bit for bit (the
    // golden traces check the whole C172 on top of this).
    const AircraftDefinition c172 = loadReference("tupi_t180");
    const PropellerDef& fp = c172.engines[0].propeller;
    for (Real v : {0.0, 30.0, 55.0}) {
        Real t0, q0, t1, q1;
        Propeller::bem(fp, v, 2400 * kTwoPi / 60, 1.1, t0, q0, 340);
        Propeller::bem(fp, v, 2400 * kTwoPi / 60, 1.1, t1, q1, 340, 0.0);
        CHECK(t0 == t1);
        CHECK(q0 == q1);
    }
    const PropellerDef& pd = kingAir().turboprops[0].propeller;
    const Propeller p(pd);
    const Real w = 1700 * kTwoPi / 60;
    // On a table slice the interpolation is the BEM solution.
    for (Real angle : {pd.fineBladeAngle, degToRad(30.0), degToRad(45.0)}) {
        Real t, q;
        Propeller::bem(pd, 120, w, 0.8, t, q, 320, angle - pd.geometricBladeAngle());
        const PropellerForces f = p.evaluate(120, w, 0.8, 320, angle);
        CHECK_NEAR(f.thrust, t, 0.02 * std::abs(t) + 20);
        CHECK_NEAR(f.torque, q, 0.02 * std::abs(q) + 5);
    }
    // In cruise thrust and torque grow with blade angle; feathered, the
    // stopped propeller only drags; in reverse, the thrust is negative.
    const PropellerForces lo = p.evaluate(140, w, 0.7, 320, degToRad(30));
    const PropellerForces hi = p.evaluate(140, w, 0.7, 320, degToRad(40));
    CHECK(hi.thrust > lo.thrust);
    CHECK(hi.torque > lo.torque);
    CHECK_RANGE(hi.efficiency, 0.8, 0.92);
    const PropellerForces feathered = p.evaluate(70, 0, 1.2, 340, pd.featherBladeAngle);
    CHECK(feathered.thrust < 0);
    CHECK(feathered.thrust > -600);
    const PropellerForces reverse = p.evaluate(40, w, 1.2, 340, pd.reverseBladeAngle);
    CHECK(reverse.thrust < -3000);
    std::cerr << "  cruise eta " << hi.efficiency << "; feathered at 70 m/s " << feathered.thrust << " N; reverse at 40 m/s "
              << reverse.thrust << " N\n";
}

TEST_CASE("king air: static take-off thrust and torque, then a cold start through the starter and light-off") {
    Aircraft ac(kingAir());
    SimpleWeather atm;
    FlatGround ground(0);
    ac.setEngineRunning(true);
    ac.placeOnGround(0, 0, 0, ground, atm);
    // Ground idle: the propellers at the flight-idle stop below the governed speed.
    PilotInput in;
    for (int i = 0; i < 240 * 3; ++i) ac.step(kDt, in, ground, atm);
    const AircraftTelemetry& t = ac.telemetry();
    std::cerr << "  low idle: Np " << t.engines[0].rpm << " rpm, Ng " << t.turboprops[0].ng * 100 << " %, ITT "
              << t.turboprops[0].itt - 273.15 << " degC, thrust " << t.turboprops[0].thrust << " N\n";
    CHECK_RANGE(t.engines[0].rpm, 700, 1300);
    CHECK_NEAR(t.turboprops[0].bladeAngle, kingAir().turboprops[0].propeller.fineBladeAngle, 1e-9);
    // Take-off power on the brakes (parking brake set).
    in.throttle = 1;
    for (int i = 0; i < 240 * 12; ++i) ac.step(kDt, in, ground, atm);
    for (int k = 0; k < 2; ++k) {
        const TurbopropState& s = t.turboprops[static_cast<std::size_t>(k)];
        std::cerr << "  take-off power, engine " << k + 1 << ": torque " << s.torqueFraction * 100 << " % ("
                  << s.torque / (kPoundForce * kFoot) << " ft lbf), Np " << t.engines[static_cast<std::size_t>(k)].rpm << ", Ng "
                  << s.ng * 100 << " %, ITT " << s.itt - 273.15 << " degC, " << s.fuelFlow * 3600 / kPound << " lb/h, thrust "
                  << s.thrust / 1000 << " kN (" << s.thrust / kPoundForce << " lbf), blade " << radToDeg(s.bladeAngle) << " deg\n";
        CHECK_NEAR(s.torqueFraction, 1.0, 0.02);
        CHECK_NEAR(t.engines[static_cast<std::size_t>(k)].rpm, 1700, 10);
        CHECK(s.itt < kingAir().turboprops[0].ittLimit);
        CHECK(s.ng < kingAir().turboprops[0].ngMax);
        CHECK_RANGE(s.thrust, 11000, 17500);  // ~2.5-3.7 lbf/shp, the class of a 105 in four-blade at 1,050 shp
        CHECK_RANGE(s.fuelFlow * 3600 / kPound, 520, 640);
    }

    // Cold and dark: battery, generators, starter; condition lever to LOW
    // IDLE at light-off speed. The starter-generator then takes the bus.
    Aircraft b(kingAir());
    b.placeOnGround(0, 0, 0, ground, atm);
    CHECK(b.turbopropState(0).feathered);
    b.setCockpitControl("electrical.battery", 1);
    b.setCockpitControl("electrical.alternator", 1);
    b.setCockpitControl("engine1.starter", 1);
    PilotInput idle;
    Real tRun = -1, ittPeak = 0;
    for (Real tt = 0; tt < 90 && tRun < 0; tt += kDt) {
        b.step(kDt, idle, ground, atm);
        if (b.turbopropState(0).ng >= 0.13 && b.cockpitControl("engine1.condition") == 0) b.setCockpitControl("engine1.condition", 1);
        ittPeak = std::max(ittPeak, b.turbopropState(0).itt);
        if (b.turbopropState(0).running) tRun = tt;
    }
    for (int i = 0; i < 240 * 10; ++i) b.step(kDt, idle, ground, atm);
    std::cerr << "  start: idle after " << tRun << " s, ITT peak " << ittPeak - 273.15 << " degC, Np " << b.telemetry().engines[0].rpm
              << " rpm\n";
    CHECK_RANGE(tRun, 20, 45);
    CHECK_RANGE(ittPeak - 273.15, 650, 1000);  // the PT6's start limit is 1,000 degC
    CHECK(b.cockpitControl("engine1.starter") == 0);  // released at idle
    CHECK(b.electrical().alternatorOnline());
    CHECK(!b.turbopropState(0).feathered);  // out of feather on the engine's oil
    CHECK(!b.turbopropState(1).running);
    // Shut down: Ng runs down and the propeller goes to feather as the oil goes.
    b.setCockpitControl("engine1.condition", 0);
    for (int i = 0; i < 240 * 40; ++i) b.step(kDt, idle, ground, atm);
    CHECK(!b.turbopropState(0).running);
    CHECK(b.turbopropState(0).feathered);
    CHECK(b.telemetry().engines[0].rpm < 50);
}

TEST_CASE("king air: the governor holds Np as the power changes, and follows the propeller lever") {
    Aircraft ac(kingAir());
    SimpleWeather atm;
    const FlatGround ground(1e7);
    const TrimResult tr = trimLevel(ac, 5000, ktToMs(180));
    CHECK(tr.converged);
    ac.setElevatorTrim(tr.trim);
    PilotInput in = tr.input;
    in.pitch = 0;
    const Real blade0 = ac.turbopropState(0).bladeAngle;
    auto run = [&](Real secs, Real& npMin, Real& npMax) {
        for (int i = 0; i < static_cast<int>(secs * 240); ++i) {
            in.roll = clamp(-2.0 * ac.telemetry().roll - 0.5 * ac.telemetry().omega.x, -1.0, 1.0);
            ac.step(kDt, in, ground, atm);
            npMin = std::min(npMin, ac.telemetry().engines[0].rpm);
            npMax = std::max(npMax, ac.telemetry().engines[0].rpm);
        }
    };
    Real lo = 1e9, hi = 0;
    in.throttle = 1;  // part power to the torque limit: Ng spools, the blades take the power
    run(10, lo, hi);
    std::cerr << "  power up: Np " << lo << " .. " << hi << " rpm, torque " << ac.turbopropState(0).torqueFraction * 100
              << " %, blade " << radToDeg(blade0) << " -> " << radToDeg(ac.turbopropState(0).bladeAngle) << " deg\n";
    CHECK(hi < 1700 * 1.02);
    CHECK(lo > 1700 * 0.98);
    CHECK_NEAR(ac.telemetry().engines[0].rpm, 1700, 5);
    CHECK(ac.turbopropState(0).bladeAngle > blade0 + degToRad(3));
    CHECK(ac.turbopropState(0).torqueFraction > 0.95);
    lo = 1e9, hi = 0;
    in.throttle = 0.2;
    run(10, lo, hi);
    std::cerr << "  power down: Np " << lo << " .. " << hi << " rpm\n";
    CHECK(lo > 1700 * 0.98);
    CHECK_NEAR(ac.telemetry().engines[0].rpm, 1700, 5);
    // Propeller levers back to 1,500 rpm: the governor coarsens the blades.
    in.throttle = tr.throttle;
    in.propLever = propLeverFor(1500);
    lo = 1e9, hi = 0;
    run(12, lo, hi);
    std::cerr << "  lever to 1,500 rpm: Np " << ac.telemetry().engines[0].rpm << ", blade " << radToDeg(ac.turbopropState(0).bladeAngle)
              << " deg\n";
    CHECK_NEAR(ac.telemetry().engines[0].rpm, 1500, 8);
    CHECK_NEAR(ac.telemetry().engines[1].rpm, 1500, 8);
}

TEST_CASE("king air: engine failure, windmilling against feathered drag; autofeather at take-off power") {
    SimpleWeather atm;
    const FlatGround ground(1e7);
    // Same flight condition, the left engine cut: the windmilling propeller
    // (the governor holding Np on a free turbine: the zero-torque blade
    // angle) against the feathered one.
    auto deadEngineThrust = [&](bool feather) {
        Aircraft ac(kingAir());
        const TrimResult tr = trimLevel(ac, 5000, ktToMs(140));
        ac.setCockpitControl("propeller.autofeather", 0);
        ac.setCockpitControl("engine1.condition", 0);
        if (feather) ac.setCockpitControl("engine1.feather", 1);
        // The engines run on at the trimmed flight condition (the airframe held there).
        const RigidBodyState held = ac.state().body;
        for (int i = 0; i < 240 * 25; ++i) {
            DynamicState s = ac.state();
            s.body = held;
            ac.setState(s);
            ac.step(kDt, tr.input, ground, atm);
        }
        const TurbopropState& s = ac.turbopropState(0);
        const Real tas = ac.telemetry().trueAirspeed;
        const Real q = 0.5 * ac.telemetry().air.state.density * tas * tas;
        std::cerr << "  " << (feather ? "feathered" : "windmilling") << ": Np " << ac.telemetry().engines[0].rpm << " rpm, Ng "
                  << s.ng * 100 << " %, blade " << radToDeg(s.bladeAngle) << " deg, thrust " << s.thrust << " N at "
                  << msToKt(tas) << " KTAS (drag area " << -s.thrust / q << " m^2)\n";
        return std::make_pair(s.thrust / q, ac.telemetry().engines[0].rpm);
    };
    const auto windmill = deadEngineThrust(false);
    const auto feathered = deadEngineThrust(true);
    CHECK(windmill.first < 0);
    CHECK(feathered.first < 0);
    CHECK(windmill.first < 2 * feathered.first);  // the windmilling drag area at least twice the feathered one
    CHECK(windmill.second > 1500);                  // windmilling at the governed speed: no feather on its own
    CHECK(feathered.second < 100);

    // Take-off: autofeather armed (power levers up, both engines making
    // torque); the left engine fails at 110 KIAS in the climb and its
    // propeller feathers within seconds without the pilot.
    Aircraft ac(kingAir());
    const TrimResult tr = trimLevel(ac, 1000, ktToMs(120), 1);
    ac.setElevatorTrim(tr.trim);
    PilotInput in = tr.input;
    in.pitch = 0;
    in.throttle = 1;
    for (int i = 0; i < 240 * 6; ++i) ac.step(kDt, in, ground, atm);
    CHECK(ac.autofeatherArmed());
    ac.setCockpitControl("engine1.condition", 0);
    Real tFeather = -1;
    for (Real t = 0; t < 20 && tFeather < 0; t += kDt) {
        in.roll = clamp(-2.0 * ac.telemetry().roll - 0.5 * ac.telemetry().omega.x, -1.0, 1.0);
        in.yaw = clamp(-1.5 * ac.telemetry().beta - 0.5 * ac.telemetry().omega.z, -1.0, 1.0);
        ac.step(kDt, in, ground, atm);
        if (ac.turbopropState(0).feathered) tFeather = t;
    }
    std::cerr << "  autofeather: feathered " << tFeather << " s after the failure\n";
    CHECK_RANGE(tFeather, 2, 15);
    CHECK(!ac.turbopropState(1).feathered);
    CHECK(ac.turbopropState(1).torqueFraction > 0.9);
}

TEST_CASE("king air: engine-out control: rudder, aileron and sideslip for straight flight at 5 deg of bank, and the Vmca they give") {
    // Published Vmca 94 KIAS: take-off power on the right engine, the left
    // feathered, up to 5 deg of bank into the live engine. At each speed the
    // engines are run to their state (left cut off and feathered, right at
    // take-off power) at the trimmed flight condition, then the lateral
    // balance is solved: no side acceleration, no roll or yaw acceleration.
    SimpleWeather atm;
    const FlatGround ground(1e7);
    struct Balance {
        bool ok = false;
        Real rudder = 0, aileron = 0, beta = 0;
    };
    auto balance = [&](Real kias) {
        Aircraft ac(kingAir());
        ac.setStationMass(1, 150);  // light: the certification's worst case
        const Real rho = atm.atmosphere().at(ftToM(3000)).density;
        const Real tas = ktToMs(kias) * std::sqrt(1.225 / rho);
        const TrimResult tr = trimLevel(ac, 3000, tas);
        ac.setYawDamper(false);
        ac.setCockpitControl("propeller.autofeather", 0);
        ac.setCockpitControl("engine1.condition", 0);
        ac.setCockpitControl("engine1.feather", 1);
        PilotInput in = tr.input;
        in.splitThrottles = true;
        in.throttles = {0, 1, 0, 0};
        const RigidBodyState held = ac.state().body;
        for (int i = 0; i < 240 * 15; ++i) {  // the engines at the flight condition, the airframe held
            DynamicState s = ac.state();
            s.body = held;
            ac.setState(s);
            ac.step(kDt, in, ground, atm);
        }
        DynamicState base = ac.state();
        base.body = held;
        auto residual = [&](const Real x[3], Real r[3]) {
            DynamicState s = base;
            const Real b = x[2];
            const Vec3 vBody{tas * std::cos(tr.alpha) * std::cos(b), tas * std::sin(b), tas * std::sin(tr.alpha) * std::cos(b)};
            s.body.attitude = Quat::fromEuler(degToRad(5), tr.theta, 0);
            s.body.velocity = s.body.attitude.rotate(vBody);
            s.body.omega = {};
            PilotInput p = in;
            p.yaw = x[0];
            p.roll = x[1];
            ac.setState(s);
            ac.prepareContext(p, ground, atm);
            const Aircraft::Derivatives d = ac.evaluate(s);
            const Vec3 a = s.body.attitude.inverseRotate(d.body.velocity);
            r[0] = a.y / kStandardGravity;
            r[1] = d.body.omega.x;
            r[2] = d.body.omega.z;
        };
        Real x[3] = {0.3, 0, 0}, r[3];
        Balance out;
        for (int it = 0; it < 40; ++it) {
            residual(x, r);
            if (std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]) < 1e-7) {
                out.ok = std::abs(x[0]) < 0.999 && std::abs(x[1]) < 0.999;
                break;
            }
            Real J[3][3];
            for (int k = 0; k < 3; ++k) {
                Real xp[3] = {x[0], x[1], x[2]}, rp[3];
                xp[k] += 1e-4;
                residual(xp, rp);
                for (int m = 0; m < 3; ++m) J[m][k] = (rp[m] - r[m]) / 1e-4;
            }
            // Cramer's rule.
            auto det = [](Real a[3][3]) {
                return a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
                       a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
            };
            const Real D = det(J);
            if (std::abs(D) < 1e-14) break;  // a control at its stop: no balance
            for (int k = 0; k < 3; ++k) {
                Real A[3][3];
                for (int m = 0; m < 3; ++m)
                    for (int n = 0; n < 3; ++n) A[m][n] = n == k ? -r[m] : J[m][n];
                x[k] += clamp(det(A) / D, -0.3, 0.3);
            }
            x[0] = clamp(x[0], -1.0, 1.0);
            x[1] = clamp(x[1], -1.0, 1.0);
        }
        out.rudder = x[0];
        out.aileron = x[1];
        out.beta = x[2];
        std::cerr << "  " << kias << " KIAS, left engine feathered, right " << ac.turbopropState(1).torqueFraction * 100
                  << " % torque, " << ac.turbopropState(1).thrust / 1000 << " kN: " << (out.ok ? "balanced" : "NO BALANCE")
                  << ", rudder " << out.rudder * 100 << " %, aileron " << out.aileron * 100 << " %, sideslip "
                  << radToDeg(out.beta) << " deg\n";
        return out;
    };
    const Balance fast = balance(120);
    CHECK(fast.ok);
    CHECK(fast.rudder > 0 && fast.rudder < 0.85);  // right rudder against the dead left engine
    // Vmca: the lowest speed that still balances, in 2 kt steps.
    Real vmc = 0;
    for (Real kias = 110; kias >= 70; kias -= 2) {
        if (!balance(kias).ok) {
            vmc = kias + 2;
            break;
        }
    }
    std::cerr << "  Vmca ~ " << vmc << " KIAS (published 94)\n";
    CHECK_RANGE(vmc, 84, 104);
}

TEST_CASE("king air: beta and reverse on the landing roll decelerate; no beta in flight") {
    SimpleWeather atm;
    FlatGround ground(0);
    auto rollout = [&](Real reverse) {
        Aircraft ac(kingAir());
        ac.setEngineRunning(true);
        ac.setFlapsImmediately(2);
        ac.placeOnGround(0, 0, 0, ground, atm);
        ac.setCockpitControl("brakes.parking", 0);
        // Rolling at 90 kt on the ground (a landing roll), power levers idle.
        DynamicState s = ac.state();
        s.body.velocity = {ktToMs(90), 0, 0};
        for (std::size_t g = 0; g < 3; ++g) s.wheelOmega[g] = ktToMs(90) / kingAir().gear[g].wheelRadius;
        ac.setState(s);
        PilotInput in;
        in.reverse = reverse;
        for (int i = 0; i < 240 * 6; ++i) {
            in.yaw = clamp(-3.0 * ac.telemetry().heading - 1.0 * ac.telemetry().omega.z, -1.0, 1.0);
            ac.step(kDt, in, ground, atm);
        }
        const TurbopropState& t = ac.turbopropState(0);
        std::cerr << "  reverse " << reverse << ": " << msToKt(ac.telemetry().groundSpeed) << " kt after 6 s, blade "
                  << radToDeg(t.bladeAngle) << " deg, Ng " << t.ng * 100 << " %, thrust " << t.thrust << " N, beta " << t.beta << "\n";
        return std::make_pair(ac.telemetry().groundSpeed, t.bladeAngle);
    };
    const auto idle = rollout(0);
    const auto beta = rollout(0.4);
    const auto full = rollout(1.0);
    CHECK(beta.first < idle.first - ktToMs(5));
    CHECK(full.first < beta.first - ktToMs(3));
    CHECK_NEAR(full.second, kingAir().turboprops[0].propeller.reverseBladeAngle, degToRad(1));
    // Average deceleration over 6 s with full reverse and no brakes: > 0.12 g.
    CHECK((ktToMs(90) - full.first) / 6 > 0.12 * kStandardGravity);

    // Airborne the flight-idle gate holds: the reverse lever does nothing.
    Aircraft ac(kingAir());
    const TrimResult tr = trimLevel(ac, 3000, ktToMs(150));
    PilotInput in = tr.input;
    in.throttle = 0;
    in.reverse = 1;
    for (int i = 0; i < 240 * 3; ++i) ac.step(kDt, in, FlatGround(1e7), atm);
    CHECK(!ac.turbopropState(0).beta);
    CHECK(ac.turbopropState(0).bladeAngle >= kingAir().turboprops[0].propeller.fineBladeAngle - 1e-9);
}

TEST_CASE("king air: maximum cruise at FL240 and the sea-level climb at MTOW against the published figures") {
    SimpleWeather atm;
    // Max cruise power (levers to the ITT limit), 1,500 rpm, FL240 ISA, mid
    // weight: the published maximum cruise is ~303-312 KTAS around FL240-FL280.
    {
        Aircraft ac(kingAir());
        ac.setGearImmediately(false);
        TrimRequest req;
        req.altitude = ftToM(24000);
        req.tas = ktToMs(290);
        req.throttle = 1;
        req.solveTas = true;
        req.propLever = propLeverFor(1500);
        const TrimResult r = trim(ac, req, atm);
        const TurbopropState& s = ac.turbopropState(0);
        std::cerr << "  FL240 max cruise at " << ac.massProperties().mass / kPound << " lb: " << msToKt(r.tas) << " KTAS, "
                  << s.shaftPower / kHorsepower << " shp/engine, torque " << s.torqueFraction * 100 << " %, ITT " << s.itt - 273.15
                  << " degC, " << r.fuelFlow / 2 * 3600 / kPound << " lb/h/engine, prop eta " << r.propEfficiency << "\n";
        CHECK(r.converged);
        CHECK_RANGE(msToKt(r.tas), 290, 320);
        CHECK_RANGE(r.fuelFlow / 2 * 3600 / kPound, 280, 420);
        CHECK_NEAR(s.itt, kingAir().turboprops[0].ittLimit, 2);  // ITT-limited at altitude
    }
    // Sea level ISA, MTOW, clean, take-off power at 1,700 rpm, 140 KIAS:
    // published rate of climb 2,731 fpm.
    {
        Aircraft ac(kingAir());
        loadToMtow(ac);
        ac.setGearImmediately(false);
        TrimRequest req;
        req.altitude = 0;
        req.tas = ktToMs(140);
        req.throttle = 1;
        req.solveFlightPath = true;
        const TrimResult r = trim(ac, req, atm);
        const Real fpm = msToFpm(r.tas * std::sin(r.flightPath));
        std::cerr << "  sea-level climb at " << ac.massProperties().mass / kPound << " lb: " << fpm << " fpm, "
                  << ac.turbopropState(0).torqueFraction * 100 << " % torque\n";
        CHECK(r.converged);
        CHECK_RANGE(fpm, 2300, 3200);
    }
}

TEST_CASE("king air: trims across its envelope; stall speeds in the published class") {
    SimpleWeather atm;
    struct Point {
        Real ft, kias;
        int flap;
        bool gear;
        Real rpm;
    };
    const Point pts[] = {{0, 130, 0, false, 1700},   {0, 250, 0, false, 1700},   {10000, 200, 0, false, 1600},
                         {20000, 180, 0, false, 1500}, {28000, 160, 0, false, 1500}, {33000, 140, 0, false, 1500},
                         {2000, 150, 1, true, 1700},  {1500, 110, 2, true, 1700}};
    for (const Point& p : pts) {
        Aircraft ac(kingAir());
        const AirState a = atm.atmosphere().at(ftToM(p.ft));
        const Real tas = tasFromCas(ktToMs(p.kias), a);
        const TrimResult r = trimLevel(ac, p.ft, tas, p.flap, p.gear, p.rpm);
        std::cerr << "  " << p.ft << " ft " << p.kias << " KIAS flap " << p.flap << (p.gear ? " gear" : "") << ": "
                  << msToKt(tas) << " KTAS M" << tas / a.speedOfSound << ", lever " << r.throttle << ", alpha " << radToDeg(r.alpha)
                  << ", trim " << r.trim << ", elevator " << r.elevator << ", torque " << ac.turbopropState(0).torqueFraction * 100
                  << " %, Np " << r.rpm << "\n";
        CHECK(r.converged);
        CHECK_RANGE(r.throttle, 0.0, 0.999);
        CHECK_RANGE(radToDeg(r.alpha), -4.0, 11.0);  // nose low with landing flap, as the King Air flies
        CHECK(std::abs(r.trim) < 0.95);
        CHECK_NEAR(r.rpm, p.rpm, 2);
    }
    // 1-g stall speeds at MTOW from the trimmed CLmax: published landing
    // configuration ~81 KIAS (VSO), clean ~100 KIAS class.
    const AircraftDefinition& d = kingAir();
    Aircraft ac(d);
    const AeroModel model(d);
    const Real w = d.maxTakeoffMass * kStandardGravity;
    auto vs = [&](int detent) {
        const Real cl = trimmedMaxLiftCoefficient(model, d.referenceArea, ac.massProperties().cg, d.controls.flapDetents[detent], 0.15);
        return std::make_pair(msToKt(std::sqrt(2 * w / (1.225 * d.referenceArea * cl))), cl);
    };
    const auto clean = vs(0), approach = vs(1), landing = vs(2);
    std::cerr << "  stall at MTOW: clean " << clean.first << " KIAS (CLmax " << clean.second << "), approach " << approach.first
              << " (" << approach.second << "), landing " << landing.first << " (" << landing.second << ")\n";
    CHECK_RANGE(clean.first, 92, 110);
    CHECK(approach.first < clean.first);
    CHECK_RANGE(landing.first, 75, 88);
}

TEST_CASE("king air: take-off at MTOW, all engines, to 50 ft") {
    Aircraft ac(kingAir());
    loadToMtow(ac);
    SimpleWeather atm;
    FlatGround ground(0);
    ac.setEngineRunning(true);
    ac.setFlapsImmediately(1);
    ac.placeOnGround(0, 0, 0, ground, atm);
    ac.setCockpitControl("brakes.parking", 0);
    PilotInput in;
    in.throttle = 1;
    const Real x0 = ac.datumPosition().x;
    Real roll = -1, to50 = -1, liftoff = 0;
    for (int i = 0; i < 240 * 60 && to50 < 0; ++i) {
        const AircraftTelemetry& t = ac.telemetry();
        // Rotate at Vr ~ 100 KIAS toward 10 deg, the rudder holding the centreline.
        if (msToKt(ias(ac)) > 100) in.pitch = clamp(4.0 * (degToRad(10) - t.pitch) - 1.5 * t.omega.y, -1.0, 1.0);
        in.yaw = clamp(-3.0 * t.heading - 1.0 * t.omega.z, -1.0, 1.0);
        ac.step(kDt, in, ground, atm);
        if (roll < 0 && !ac.telemetry().onGround) {
            roll = ac.datumPosition().x - x0;
            liftoff = msToKt(ias(ac));
        }
        if (ac.telemetry().heightAgl > ftToM(50) + 1.45) to50 = ac.datumPosition().x - x0;
    }
    std::cerr << "  " << ac.massProperties().mass / kPound << " lb, SL ISA, approach flap: ground roll " << mToFt(roll)
              << " ft, lift-off " << liftoff << " KIAS, to 50 ft " << mToFt(to50) << " ft\n";
    // The published 3,300 ft is the take-off field length (an engine failure
    // at V1 included); the all-engines distance to 50 ft is well inside it.
    CHECK_RANGE(mToFt(to50), 1600, 3000);
    CHECK_RANGE(liftoff, 95, 115);
    CHECK(std::abs(radToDeg(ac.telemetry().heading)) < 3);
}
