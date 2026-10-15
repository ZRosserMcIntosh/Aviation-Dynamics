// Aviation Dynamics — scenarios.
// SPDX-License-Identifier: MIT
#include "scenarios.hpp"

#include <cstdio>
#include <cstdlib>
#include <stdexcept>

#include "avd/units.hpp"

namespace avd::scenarios {

using namespace units;

AircraftDefinition loadReference(const std::string& id) {
    const LoadResult r = loadAircraftFromFile(std::string(AVD_DATA_DIR) + "/aircraft/" + id + ".json");
    if (!r.ok()) {
        std::string msg = "cannot load aircraft '" + id + "':";
        for (const auto& e : r.errors) msg += "\n  " + e;
        throw std::runtime_error(msg);
    }
    return r.definition;
}

// ---------------------------------------------------------------- stall ----

namespace {

void loadToMaxGross(Aircraft& ac) {
    // Two front occupants + one rear (mid CG) to 2550 lb with full fuel.
    const Real remaining = ac.definition().maxTakeoffMass - ac.massProperties().mass + ac.stationMass(0) + ac.stationMass(1);
    ac.setStationMass(0, std::min(170.0, remaining));
    ac.setStationMass(1, std::max(0.0, remaining - 170.0));
}

}  // namespace

StallResult stallTest(const AircraftDefinition& def, int flap, Real dt) {
    StallResult out;
    Aircraft ac(def, LocalFrame{});
    loadToMaxGross(ac);
    SimpleWeather atm;
    FlatGround ground(0);
    TestPilot pilot;
    const Real startKias = flap == 0 ? 70 : 62;
    startTrimmedFlight(ac, {startKias, 1500, flap}, atm, ground, &pilot);
    pilot.vertical = TestPilot::Vertical::Airspeed;
    pilot.controls.throttle = 0.0;
    pilot.targetPitch = 0.0;
    Real target = ktToMs(startKias);
    Real minCas = 1e9, minIas = 1e9;
    Real stallTime = -1, altAtStall = 0, minAlt = 1e9;
    for (int i = 0; i < static_cast<int>(120 / dt); ++i) {
        const Real t = i * dt;
        if (stallTime < 0) target = std::max(target - ktToMs(1.0) * dt, ktToMs(20));
        pilot.targetIas = target;
        PilotInput in = pilot.update(ac, dt);
        if (stallTime >= 0 && t > stallTime + 1.0) {
            // Recovery: nose down, full power.
            in.pitch = -0.3;
            in.throttle = 1.0;
        }
        ac.step(dt, in, ground, atm);
        const AircraftTelemetry& tel = ac.telemetry();
        const CockpitState& r = ac.instruments().readings();
        if (stallTime < 0) {
            minCas = std::min(minCas, r.calibratedAirspeed);
            minIas = std::min(minIas, r.indicatedAirspeed);
            out.alphaMax = std::max(out.alphaMax, tel.alpha);
            if (r.stallHorn && out.warningIas == 0) out.warningIas = r.indicatedAirspeed;
            // Stall break: CL max passed -- a sustained nose drop or wing
            // stall spreading, with the pilot still asking for less speed.
            const bool noseDrop = tel.omega.y < -0.12;
            if (ac.aero().surfaces[0].stalledStrips >= 6 || (noseDrop && tel.alpha > 0.15)) {
                stallTime = t;
                altAtStall = tel.altitudeMsl;
                out.stalled = true;
            }
        } else {
            minAlt = std::min(minAlt, tel.altitudeMsl);
            if (t > stallTime + 8) break;
        }
    }
    out.minCas = minCas;
    out.minIas = minIas;
    out.altitudeLost = out.stalled ? altAtStall - minAlt : 0;
    return out;
}

// -------------------------------------------------------------- takeoff ----

TakeoffResult takeoffTest(const AircraftDefinition& def, Real elevation, Real dT, Real rh, Real headwind,
                          Real rotateKias, Real dt) {
    TakeoffResult out;
    Aircraft ac(def, LocalFrame{});
    loadToMaxGross(ac);
    SimpleWeather::Params wp;
    wp.conditions.temperatureDeviation = dT;
    wp.conditions.relativeHumidity = rh;
    wp.groundElevation = elevation;
    wp.windSpeed = headwind;
    wp.windFromDirection = 0;  // runway heads north: headwind blows from the north
    SimpleWeather atm(wp);
    AirfieldGround ground(-elevation);
    RunwayDef rw;
    rw.threshold = {0, 0, -elevation};
    rw.headingTrue = 0;
    rw.length = 3000;
    rw.width = 45;
    ground.addRunway(rw);

    ac.setEngineRunning(true, 1000);
    ac.setFlapsImmediately(1);  // 10 deg
    ac.placeOnGround(40, 0, 0, ground, atm);
    ac.setCockpitControl("brakes.parking", 1);
    PilotInput in;
    in.mixture = bestPowerMixture(def, atm.atmosphere().at(elevation).density);
    in.throttle = 1.0;
    for (int i = 0; i < static_cast<int>(3.0 / dt); ++i) ac.step(dt, in, ground, atm);
    ac.setCockpitControl("brakes.parking", 0);
    const Vec3 start = ac.datumPosition();
    const Real cgHeight0 = ac.telemetry().altitudeMsl - elevation;
    TestPilot pilot;
    pilot.controls = in;
    pilot.lateral = TestPilot::Lateral::WingsLevel;
    pilot.vertical = TestPilot::Vertical::Manual;
    bool rotated = false, airborne = false;
    Real airborneTime = 0;
    for (int i = 0; i < static_cast<int>(90 / dt); ++i) {
        const AircraftTelemetry& t = ac.telemetry();
        const Real ias = ac.instruments().readings().indicatedAirspeed;
        if (!rotated && ias >= ktToMs(rotateKias)) {
            rotated = true;
            pilot.vertical = TestPilot::Vertical::Pitch;
            pilot.targetPitch = degToRad(10);
            pilot.resetIntegrators();
        }
        if (airborne && pilot.vertical != TestPilot::Vertical::Airspeed) {
            // Climb at 56 KIAS from lift-off (POH short-field technique).
            pilot.vertical = TestPilot::Vertical::Airspeed;
            pilot.targetIas = ktToMs(56);
            pilot.targetPitch = degToRad(12);
        }
        PilotInput c = pilot.update(ac, dt);
        if (t.onGround) c.yaw = clamp(-3.0 * t.heading - 0.8 * t.omega.z, -1.0, 1.0);  // keep the centreline
        if (!rotated) c.pitch = 0;
        ac.step(dt, c, ground, atm);
        const AircraftTelemetry& t2 = ac.telemetry();
        if (!airborne && !t2.onGround) {
            airborneTime += dt;
            if (airborneTime > 0.3) {
                airborne = true;
                out.liftoffIas = ias;
            }
            if (out.groundRoll == 0) out.groundRoll = length(ac.datumPosition() - start);
        } else if (!airborne) {
            airborneTime = 0;
            out.groundRoll = 0;
        }
        if (std::getenv("AVD_TRACE_TAKEOFF") && i % 60 == 0)
            std::fprintf(stderr, "t=%5.1f d=%6.0f ft ias=%5.1f cas=%5.1f alpha=%5.1f h=%5.1f ft pitch=%5.1f vs=%5.0f gnd=%d\n", i * dt,
                         mToFt(length(ac.datumPosition() - start)), msToKt(ac.instruments().readings().indicatedAirspeed),
                         msToKt(ac.instruments().truth().calibratedAirspeed), radToDeg(t2.alpha),
                         mToFt(t2.altitudeMsl - elevation - cgHeight0), radToDeg(t2.pitch), msToFpm(t2.verticalSpeed), t2.onGround);
        if (airborne && t2.altitudeMsl - elevation - cgHeight0 >= ftToM(50)) {
            const Vec3 p = ac.datumPosition() - start;
            out.distanceTo50ft = std::hypot(p.x, p.y);
            out.success = true;
            break;
        }
    }
    return out;
}

Real landingRollTest(const AircraftDefinition& def, Real touchdownKias, Real dt, BrakingTechnique technique) {
    Aircraft ac(def, LocalFrame{});
    loadToMaxGross(ac);
    SimpleWeather atm;
    FlatGround ground(0);
    ac.setEngineRunning(true, 800);
    ac.setFlapsImmediately(3);
    ac.placeOnGround(0, 0, 0, ground, atm);
    ac.setCockpitControl("brakes.parking", 0);
    DynamicState s = ac.state();
    s.body.velocity = {ktToMs(touchdownKias), 0, 0};
    ac.setState(s);
    const Vec3 start = ac.datumPosition();
    PilotInput in;
    in.throttle = 0;
    in.pitch = 1.0;
    for (int i = 0; i < static_cast<int>(60 / dt); ++i) {
        const AircraftTelemetry& t = ac.telemetry();
        in.yaw = clamp(-3.0 * t.heading - 0.8 * t.omega.z, -1.0, 1.0);
        const Real since = i * dt - technique.delay;
        const Real ramp = technique.rampTime > 0 ? clamp(since / technique.rampTime, 0.0, 1.0) : (since >= 0 ? 1.0 : 0.0);
        in.brakeLeft = in.brakeRight = technique.effort * ramp;
        ac.step(dt, in, ground, atm);
        if (ac.telemetry().groundSpeed < 0.2) break;
    }
    return length(ac.datumPosition() - start);
}

}  // namespace avd::scenarios
