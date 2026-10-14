// Aviation Dynamics — interacting systems, cockpit, procedures, ground handling,
// assists and demonstration scenarios (A: fuel starvation, C: engine shutdown).
// SPDX-License-Identifier: MIT
#include <algorithm>
#include <cstring>
#include <sstream>

#include "airports.hpp"
#include "avd/assists.hpp"
#include "avd/procedures.hpp"
#include "avd/telemetry.hpp"
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

bool hasEvent(const std::vector<FlightEvent>& ev, FlightEventType t) {
    for (const auto& e : ev)
        if (e.type == t) return true;
    return false;
}

}  // namespace

// ---------------------------------------------------------------- ground ----

TEST_CASE("ground: static equilibrium on the gear and sensible load split") {
    Aircraft ac(ref());
    SimpleWeather atm;
    FlatGround g(0);
    ac.placeOnGround(0, 0, 0, g, atm);
    const AircraftTelemetry& t = ac.telemetry();
    const Real w = ac.massProperties().mass * kStandardGravity;
    const Real sum = t.gear[0].normalForce + t.gear[1].normalForce + t.gear[2].normalForce;
    CHECK_NEAR(sum, w, 0.02 * w);
    CHECK_RANGE(t.gear[0].normalForce / w, 0.15, 0.40);           // nose wheel share
    CHECK_NEAR(t.gear[1].normalForce, t.gear[2].normalForce, 0.02 * w);
    CHECK(std::abs(t.pitch) < degToRad(3));
    CHECK(t.onGround);
    // Propeller ground clearance ~ 11 in (POH).
    const Vec3 tip = ac.state().body.position + ac.state().body.attitude.rotate(ref().contacts[0].position - ac.massProperties().cg);
    CHECK_RANGE(mToFt(-tip.z) * 12, 7.0, 15.0);
}

TEST_CASE("ground: parking brake holds full power; brakes released, it accelerates") {
    Aircraft ac(ref());
    SimpleWeather atm;
    FlatGround g(0);
    ac.setEngineRunning(true, 1000);
    ac.placeOnGround(0, 0, 0, g, atm);
    ac.setCockpitControl("brakes.parking", 1);
    PilotInput in;
    in.throttle = 1;
    const Vec3 p0 = ac.datumPosition();
    for (int i = 0; i < 10 * 240; ++i) ac.step(kDefaultDt, in, g, atm);
    CHECK(length(ac.datumPosition() - p0) < 0.2);
    ac.setCockpitControl("brakes.parking", 0);
    for (int i = 0; i < 5 * 240; ++i) ac.step(kDefaultDt, in, g, atm);
    CHECK(ac.telemetry().groundSpeed > 8);
}

TEST_CASE("ground: nose-wheel steering follows its geometry; differential braking turns tighter") {
    // Walking-pace taxi (~3 m/s): full right pedal gives ~10 deg nose-wheel
    // steering; the kinematic yaw rate is V tan(delta) / wheelbase.
    auto turn = [](Real yaw, Real brakeRight, Real& speed) {
        Aircraft ac(ref());
        SimpleWeather atm;
        FlatGround g(0);
        ac.setEngineRunning(true, 1000);
        ac.placeOnGround(0, 0, 0, g, atm);
        ac.setCockpitControl("brakes.parking", 0);
        PilotInput in;
        in.throttle = 0.15;
        for (int i = 0; i < 4 * 240; ++i) ac.step(kDefaultDt, in, g, atm);
        in.yaw = yaw;
        in.brakeRight = brakeRight;
        for (int i = 0; i < 3 * 240; ++i) ac.step(kDefaultDt, in, g, atm);
        speed = ac.telemetry().groundSpeed;
        return ac.telemetry().omega.z;
    };
    Real v0, v1, v2;
    const Real straight = turn(0, 0, v0), steer = turn(1, 0, v1), brake = turn(1, 0.5, v2);
    const Real wheelbase = ref().gear[0].attach.x - ref().gear[1].attach.x;
    const Real kinematic = v1 * std::tan(ref().gear[0].maxSteerAngle) / wheelbase;
    CHECK_RANGE(v1, 1.5, 6.0);
    CHECK_NEAR(steer, kinematic, 0.3 * kinematic);
    // Neutral pedals: mild left-turning tendency from the slipstream swirl.
    CHECK_RANGE(straight, -0.06, 0.005);
    CHECK(v2 / brake < v1 / steer);  // plus right brake: smaller turn radius
}

TEST_CASE("ground: steering hard at speed can overturn a tricycle") {
    // Full steering at ~17 kt demands ~0.75 g: beyond the nose/main overturn
    // limit, the outside wheel lifts. Physics, not a scripted event.
    Aircraft ac(ref());
    SimpleWeather atm;
    FlatGround g(0);
    ac.setEngineRunning(true, 1000);
    ac.placeOnGround(0, 0, 0, g, atm);
    ac.setCockpitControl("brakes.parking", 0);
    PilotInput in;
    in.throttle = 0.35;
    for (int i = 0; i < 6 * 240; ++i) ac.step(kDefaultDt, in, g, atm);
    in.yaw = 1;
    Real maxRoll = 0;
    bool wheelLifted = false;
    for (int i = 0; i < 4 * 240; ++i) {
        ac.step(kDefaultDt, in, g, atm);
        maxRoll = std::max(maxRoll, std::abs(ac.telemetry().roll));
        wheelLifted = wheelLifted || !ac.telemetry().gear[2].contact || !ac.telemetry().gear[1].contact;
    }
    CHECK(wheelLifted);
    CHECK(maxRoll > degToRad(5));
}

// ----------------------------------------------------- systems interplay ----

TEST_CASE("systems: battery master off in flight kills electric devices, not the engine") {
    Aircraft ac(ref());
    SimpleWeather atm;
    FlatGround g(0);
    TestPilot pilot;
    startTrimmedFlight(ac, {100, 1500, 0}, atm, g, &pilot);
    ac.setCockpitControl("electrical.alternator", 0);
    ac.setCockpitControl("electrical.battery", 0);
    ac.setCockpitControl("flaps.lever", 1);  // try to lower flaps
    for (int i = 0; i < 10 * 240; ++i) ac.step(kDefaultDt, pilot.update(ac, kDefaultDt), g, atm);
    CHECK_NEAR(ac.electrical().busVoltage(), 0, 0);
    CHECK_NEAR(ac.flapAngle(), 0, 1e-12);                           // electric flaps do not move
    CHECK_NEAR(ac.instruments().readings().turnRate, 0, 0);         // electric gyro dead
    CHECK_NEAR(ac.instruments().readings().fuelGauge[0], 0, 0.002); // gauges at zero
    CHECK(ac.telemetry().engines[0].engine.running);                // magnetos are self-powered
    CHECK(ac.instruments().readings().tachometerRpm > 2000);        // mechanical tach still works
    // Power back: flaps run.
    ac.setCockpitControl("electrical.battery", 1);
    for (int i = 0; i < 5 * 240; ++i) ac.step(kDefaultDt, pilot.update(ac, kDefaultDt), g, atm);
    CHECK(ac.flapAngle() > degToRad(9));
}

TEST_CASE("scenario C (piston): fuel shutoff in cruise stops the engine through the fuel system") {
    Aircraft ac(ref());
    SimpleWeather atm;
    FlatGround g(0);
    TestPilot pilot;
    startTrimmedFlight(ac, {100, 2000, 0}, atm, g, &pilot);
    pilot.vertical = TestPilot::Vertical::Airspeed;  // the pilot holds 75 KIAS glide
    pilot.targetIas = ktToMs(75);
    ac.drainEvents();
    CHECK(ac.setCockpitControl("fuel.shutoff", 0));
    Real stopTime = -1;
    for (int i = 0; i < 90 * 240; ++i) {
        ac.step(kDefaultDt, pilot.update(ac, kDefaultDt), g, atm);
        if (stopTime < 0 && !ac.telemetry().engines[0].engine.firing) stopTime = ac.time();
    }
    const auto ev = ac.drainEvents();
    CHECK(hasEvent(ev, FlightEventType::EngineStopped));
    CHECK(hasEvent(ev, FlightEventType::FuelStarvation));
    CHECK(stopTime > 0);
    // The engine ran on line fuel for a while (no instant scripted cut).
    CHECK(stopTime - ev.front().time > 5);
    CHECK(ac.telemetry().engines[0].prop.thrust < 0);  // windmilling drag
    CHECK(ac.telemetry().verticalSpeed < -2);          // gliding down
    // Restore fuel: the windmilling engine relights.
    ac.setCockpitControl("fuel.shutoff", 1);
    pilot.controls.throttle = 0.6;
    pilot.targetIas = ktToMs(80);
    for (int i = 0; i < 20 * 240; ++i) ac.step(kDefaultDt, pilot.update(ac, kDefaultDt), g, atm);
    CHECK(ac.telemetry().engines[0].engine.running);
    CHECK(hasEvent(ac.drainEvents(), FlightEventType::EngineStarted));
}

TEST_CASE("systems: magnetos off and idle cut-off stop the engine; magnetos on relight it") {
    Aircraft ac(ref());
    SimpleWeather atm;
    FlatGround g(0);
    TestPilot pilot;
    startTrimmedFlight(ac, {100, 2000, 0}, atm, g, &pilot);
    ac.setCockpitControl("engine1.ignition", 0);
    for (int i = 0; i < 2 * 240; ++i) ac.step(kDefaultDt, pilot.update(ac, kDefaultDt), g, atm);
    CHECK(!ac.telemetry().engines[0].engine.firing);
    ac.setCockpitControl("engine1.ignition", 3);
    for (int i = 0; i < 2 * 240; ++i) ac.step(kDefaultDt, pilot.update(ac, kDefaultDt), g, atm);
    CHECK(ac.telemetry().engines[0].engine.running);
    pilot.controls.mixture = 0.0;  // idle cut-off
    for (int i = 0; i < 2 * 240; ++i) ac.step(kDefaultDt, pilot.update(ac, kDefaultDt), g, atm);
    CHECK(!ac.telemetry().engines[0].engine.firing);
    CHECK_NEAR(ac.telemetry().engines[0].engine.fuelFlow, 0, 0);
}

TEST_CASE("scenario A: fuel exhaustion from real consumption, then a glide") {
    Aircraft ac(ref());
    SimpleWeather atm;
    FlatGround g(0);
    TestPilot pilot;
    // 2 litres of usable fuel per tank above the unusable quantity.
    for (int t = 0; t < 2; ++t) ac.fuel().setTankQuantity(t, ref().fuel.tanks[t].unusable + 0.002);
    ac.refreshMass();
    startTrimmedFlight(ac, {100, 1500, 0}, atm, g, &pilot);
    const Real m0 = ac.massProperties().mass;
    Real burned = 0, quitTime = -1;
    for (int i = 0; i < 1200 * 240; ++i) {
        ac.step(kDefaultDt, pilot.update(ac, kDefaultDt), g, atm);
        burned += ac.telemetry().engines[0].engine.fuelFlow * kDefaultDt;
        if (!ac.telemetry().engines[0].engine.firing) { quitTime = ac.time(); break; }
    }
    CHECK(quitTime > 0);
    // ~4.25 L at ~9 gph (34 L/h) is ~7-8 minutes.
    CHECK_RANGE(quitTime / 60, 5.0, 11.0);
    CHECK_NEAR(burned, (0.004 + ref().fuel.lineVolume) * ref().fuel.density, 0.05);
    CHECK_NEAR(m0 - ac.massProperties().mass, burned, 0.05);
    CHECK_NEAR(ac.fuel().tankQuantity(0), ref().fuel.tanks[0].unusable, 1e-6);  // unusable remains
    CHECK(hasEvent(ac.drainEvents(), FlightEventType::FuelStarvation));
}

// ------------------------------------------------------ cockpit & procedures ----

TEST_CASE("cockpit: controls are discoverable, labelled and operate real state") {
    Aircraft ac(ref());
    const int i = ac.cockpitControlIndex("fuel.shutoff");
    CHECK(i >= 0);
    CHECK(ac.cockpitControls()[i].label == "Fuel Shutoff Valve");
    CHECK(ac.cockpitControl("fuel.shutoff") == 1);
    CHECK(ac.cycleCockpitControl("fuel.shutoff"));
    CHECK(!ac.fuel().shutoffOpen());
    CHECK(ac.setCockpitControl("fuel.selector", 0));
    CHECK(ac.fuel().selector() == 0);
    CHECK(ac.setCockpitControl("electrical.battery", 1));
    CHECK(ac.electrical().batterySwitch());
    CHECK(!ac.setCockpitControl("fuel.selector", 7));  // no such position
    CHECK(!ac.setCockpitControl("no.such.control", 0));
    bool sawEvent = false;
    for (const auto& e : ac.drainEvents()) sawEvent = sawEvent || e.type == FlightEventType::CockpitControlChanged;
    CHECK(sawEvent);
}

TEST_CASE("procedures: Quick Start from cold and dark works through the real systems") {
    Aircraft ac(ref());
    SimpleWeather atm;
    FlatGround g(0);
    ac.placeOnGround(0, 0, 0, g, atm);
    ac.setColdAndDark();
    CHECK(!ac.telemetry().engines[0].engine.running);
    ProcedureRunner run;
    run.start(quickStartPistonSingle(ac));
    for (int i = 0; i < 60 * 240 && run.status() == ProcedureRunner::Status::Running; ++i) {
        PilotInput in;
        in.throttle = 0.1;
        run.update(ac, in, kDefaultDt);
        ac.step(kDefaultDt, in, g, atm);
    }
    CHECK(run.status() == ProcedureRunner::Status::Done);
    CHECK(ac.telemetry().engines[0].engine.running);
    CHECK(ac.electrical().alternatorOnline());
    CHECK_RANGE(ac.telemetry().engines[0].rpm, 700, 1400);
}

TEST_CASE("procedures: Quick Start fails for real reasons (flat battery, fuel valve shut)") {
    {
        Aircraft ac(ref());
        SimpleWeather atm;
        FlatGround g(0);
        ac.placeOnGround(0, 0, 0, g, atm);
        ac.setColdAndDark();
        ac.electrical().setBatteryCharge(0.01);
        ProcedureRunner run;
        run.start(quickStartPistonSingle(ac));
        for (int i = 0; i < 60 * 240 && run.status() == ProcedureRunner::Status::Running; ++i) {
            PilotInput in;
            run.update(ac, in, kDefaultDt);
            ac.step(kDefaultDt, in, g, atm);
        }
        CHECK(run.status() == ProcedureRunner::Status::Failed);
        CHECK(!ac.telemetry().engines[0].engine.running);
    }
}

TEST_CASE("procedures: with empty tanks the engine starts on line fuel, then starves") {
    // Emergent, not scripted: the primed line holds ~0.25 L, enough to start
    // and run briefly; then the engine quits and a starvation event fires.
    Aircraft ac(ref());
    SimpleWeather atm;
    FlatGround g(0);
    ac.placeOnGround(0, 0, 0, g, atm);
    ac.setColdAndDark();
    for (int t = 0; t < 2; ++t) ac.fuel().setTankQuantity(t, ref().fuel.tanks[t].unusable);
    ac.refreshMass();
    ProcedureRunner run;
    run.start(quickStartPistonSingle(ac));
    bool started = false, starved = false;
    Real stopTime = -1;
    for (int i = 0; i < 180 * 240; ++i) {
        PilotInput in;
        in.throttle = 0.12;
        run.update(ac, in, kDefaultDt);
        ac.step(kDefaultDt, in, g, atm);
        started = started || ac.telemetry().engines[0].engine.running;
        for (const auto& e : ac.drainEvents()) starved = starved || e.type == FlightEventType::FuelStarvation;
        if (started && !ac.telemetry().engines[0].engine.firing) { stopTime = ac.time(); break; }
    }
    CHECK(started);
    CHECK(starved);
    CHECK(stopTime > 0);
    CHECK_NEAR(ac.fuel().tankQuantity(0), ref().fuel.tanks[0].unusable, 1e-12);
}

// ------------------------------------------------------------------ assists ----

TEST_CASE("assists: never change the definition; Simulation preset is a pass-through") {
    Aircraft a(ref()), b(ref());
    SimpleWeather atm;
    FlatGround g(0);
    TestPilot pa, pb;
    startTrimmedFlight(a, {100, 1500, 0}, atm, g, &pa);
    startTrimmedFlight(b, {100, 1500, 0}, atm, g, &pb);
    FlightAssist sim(makePreset(AssistPreset::Simulation));
    PilotInput raw;
    raw.throttle = 0.7;
    raw.mixture = 0.8;
    raw.roll = 0.2;
    for (int i = 0; i < 5 * 240; ++i) {
        a.step(kDefaultDt, raw, g, atm);
        b.step(kDefaultDt, sim.apply(raw, b, kDefaultDt), g, atm);
    }
    CHECK(std::memcmp(&a.state().body, &b.state().body, sizeof(RigidBodyState)) == 0);
    CHECK(!sim.report().active());
    // Every preset leaves the aircraft definition untouched (assists only see const Aircraft&).
    for (AssistPreset p : {AssistPreset::Arcade, AssistPreset::Sport}) {
        FlightAssist fa(makePreset(p));
        fa.apply(raw, a, kDefaultDt);
        CHECK(a.definition().surfaces[0].section.clMax == ref().surfaces[0].section.clMax);
    }
}

TEST_CASE("assists: Arcade coordinates turns and levels the wings through the controls") {
    auto run = [](AssistPreset preset) {
        Aircraft ac(ref());
        SimpleWeather atm;
        FlatGround g(0);
        TestPilot tp;
        TrimResult tr;
        startTrimmedFlight(ac, {100, 1500, 0}, atm, g, &tp, &tr);
        FlightAssist assist(makePreset(preset));
        PilotInput raw = tr.input;
        raw.pitch = 0;
        Real maxBeta = 0;
        for (int i = 0; i < 6 * 240; ++i) {
            raw.roll = i < 2 * 240 ? 0.6 : 0.0;  // roll in, then let go
            ac.step(kDefaultDt, assist.apply(raw, ac, kDefaultDt), g, atm);
            maxBeta = std::max(maxBeta, std::abs(ac.telemetry().beta));
        }
        return std::pair<Real, Real>(maxBeta, std::abs(ac.telemetry().roll));
    };
    const auto sim = run(AssistPreset::Simulation), arcade = run(AssistPreset::Arcade);
    CHECK(arcade.first < sim.first);       // less sideslip
    CHECK(arcade.second < degToRad(10));   // wings levelled after release
    CHECK(sim.second > arcade.second);
}

TEST_CASE("assists: stall prevention is a labelled aid acting only on stick input") {
    Aircraft ac(ref());
    SimpleWeather atm;
    FlatGround g(0);
    TestPilot tp;
    TrimResult tr;
    startTrimmedFlight(ac, {70, 1500, 0}, atm, g, &tp, &tr);
    AssistConfig c;
    c.stallPrevention = true;
    FlightAssist assist(c);
    PilotInput raw = tr.input;
    raw.throttle = 0;
    raw.pitch = 1.0;  // full back stick
    int stalled = 0;
    for (int i = 0; i < 15 * 240; ++i) {
        ac.step(kDefaultDt, assist.apply(raw, ac, kDefaultDt), g, atm);
        stalled = std::max(stalled, ac.aero().surfaces[0].stalledStrips);
    }
    CHECK(stalled <= 4);
    CHECK(assist.report().pitchLimited > 0);
}

// -------------------------------------------------------- environment ----

TEST_CASE("performance: hot and high and humid take-offs need more runway") {
    const TakeoffResult sl = takeoffTest(ref(), 0, 0);
    const TakeoffResult hot = takeoffTest(ref(), 722, 20);  // Campo de Marte on a hot day
    const TakeoffResult humid = takeoffTest(ref(), 0, 15, 1.0);
    const TakeoffResult dryWarm = takeoffTest(ref(), 0, 15, 0.0);
    CHECK(sl.success && hot.success && humid.success);
    CHECK_RANGE(hot.groundRoll / sl.groundRoll, 1.3, 1.9);
    // Humidity matters, but only by its density contribution (~1-2% density).
    CHECK(humid.groundRoll > dryWarm.groundRoll);
    CHECK(humid.groundRoll < dryWarm.groundRoll * 1.06);
    // A 10 kt headwind shortens the ground roll substantially.
    const TakeoffResult head = takeoffTest(ref(), 0, 0, 0, ktToMs(10));
    CHECK(head.groundRoll < 0.85 * sl.groundRoll);
}

TEST_CASE("events and telemetry: touchdown is reported with sink rate; CSV is written") {
    Aircraft ac(ref());
    SimpleWeather atm;
    FlatGround g(0);
    ac.setEngineRunning(true, 1000);
    ac.setFlapsImmediately(3);
    RigidBodyState s;
    s.position = {0, 0, -3.0};
    s.velocity = {28, 0, 1.5};  // 1.5 m/s sink
    s.attitude = Quat::fromEuler(0, degToRad(4), 0);
    ac.setFlightState(s);
    ac.setCockpitControl("brakes.parking", 0);
    std::ostringstream csv;
    TelemetryRecorder rec(csv, 0.1);
    rec.writeHeader();
    PilotInput in;
    for (int i = 0; i < 7 * 240; ++i) {  // touchdown, a small balloon, settle
        ac.step(kDefaultDt, in, g, atm);
        rec.record(ac);
    }
    bool touchdown = false;
    for (const auto& e : ac.drainEvents())
        if (e.type == FlightEventType::Touchdown && !touchdown) {
            touchdown = true;
            CHECK_RANGE(e.value, 0.8, 3.0);  // first contact at ~1.5 m/s sink
        }
    CHECK(touchdown);
    CHECK(ac.telemetry().onGround);
    const std::string out = csv.str();
    CHECK(out.rfind("time_s,", 0) == 0);
    CHECK(std::count(out.begin(), out.end(), '\n') > 30);
}

TEST_CASE("world: Campo de Marte runway geometry from the São Paulo World anchor") {
    const LocalFrame f = saoPauloWorldFrame();
    const Airport a = campoDeMarte(f);
    CHECK(a.runways.size() == 2);
    // ~5.7 km north and ~1.8 km east of MASP, runway 12 true heading ~101 deg.
    CHECK_RANGE(a.runways[0].threshold.x, 5000, 6500);
    CHECK_RANGE(a.runways[0].threshold.y, 1000, 2500);
    CHECK_NEAR(radToDeg(a.runways[0].headingTrue), 101, 3);
    // Magnetic 122 deg less 21.5 deg W variation ~ 100.5 deg true.
    CHECK_NEAR(radToDeg(a.runways[0].headingTrue), 122 + radToDeg(a.magneticVariation), 3);
    auto ground = makeAirfieldGround(a, f);
    const GroundHit h = ground->below(runwayPoint(a.runways[0], 800) + Vec3{0, 0, -100});
    CHECK(h.hit);
    CHECK_NEAR(h.point.z, -722, 1e-6);
    CHECK(h.surface.id == surfaces::kAsphalt.id);
    CHECK(ground->below(runwayPoint(a.runways[0], 800, 60) + Vec3{0, 0, -100}).surface.id == surfaces::kGrass.id);
}
