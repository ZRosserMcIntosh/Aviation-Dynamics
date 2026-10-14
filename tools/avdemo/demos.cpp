// Aviation Dynamics — avdemo flight scenarios.
// SPDX-License-Identifier: MIT
//
// Every scenario flies the same Aircraft the game uses. The pilot is a script
// (TestPilot + a small phase machine) that only moves controls and switches.
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <memory>
#include <string>

#include "airports.hpp"
#include "avd/procedures.hpp"
#include "avd/telemetry.hpp"
#include "avd/units.hpp"
#include "scenarios.hpp"

using namespace avd;
using namespace avd::scenarios;
using namespace avd::units;

namespace {

constexpr Real kDt = kDefaultDt;

const char* eventText(FlightEventType t) {
    switch (t) {
        case FlightEventType::EngineStarted: return "engine running";
        case FlightEventType::EngineStopped: return "ENGINE STOPPED";
        case FlightEventType::FuelStarvation: return "FUEL STARVATION at the engine inlet";
        case FlightEventType::StallWarningOn: return "stall warning horn ON";
        case FlightEventType::StallWarningOff: return "stall warning horn off";
        case FlightEventType::Touchdown: return "touchdown";
        case FlightEventType::Liftoff: return "lift-off";
        case FlightEventType::Overspeed: return "OVERSPEED (VNE exceeded)";
        case FlightEventType::OverG: return "LIMIT LOAD EXCEEDED";
        case FlightEventType::FlapOverspeed: return "flaps extended above VFE";
        case FlightEventType::PropStrike: return "PROPELLER STRIKE";
        case FlightEventType::HardPointContact: return "airframe touched the ground";
        case FlightEventType::BatteryDepleted: return "battery depleted";
        case FlightEventType::AlternatorOffline: return "alternator OFFLINE";
        case FlightEventType::AlternatorOnline: return "alternator online";
        case FlightEventType::CockpitControlChanged: return "";
    }
    return "?";
}

void log(const Aircraft& ac, const std::string& msg) {
    std::cout << "  t=" << std::fixed << std::setprecision(1) << std::setw(6) << ac.time() << " s | " << msg << "\n";
}

void drain(Aircraft& ac) {
    for (const FlightEvent& e : ac.drainEvents()) {
        std::ostringstream os;
        os << std::fixed;
        if (e.type == FlightEventType::CockpitControlChanged) {
            const CockpitControl& c = ac.cockpitControls()[e.index];
            os << "[cockpit] " << c.label << " -> " << c.positions[static_cast<int>(e.value)];
        } else {
            os << eventText(e.type);
            if (e.type == FlightEventType::Touchdown) os << " (sink " << std::setprecision(0) << msToFpm(e.value) << " fpm)";
            if (e.type == FlightEventType::Liftoff) os << " at " << std::setprecision(0) << msToKt(e.value) << " KTAS";
            if (e.type == FlightEventType::EngineStopped || e.type == FlightEventType::EngineStarted)
                os << " (" << std::setprecision(0) << e.value << " rpm)";
            if (e.type == FlightEventType::HardPointContact || e.type == FlightEventType::PropStrike)
                os << " (" << ac.definition().contacts[e.index].name << ")";
        }
        std::cout << "  t=" << std::fixed << std::setprecision(1) << std::setw(6) << e.time << " s | " << os.str() << "\n";
    }
}

std::string status(const Aircraft& ac, Real fieldElevation) {
    const AircraftTelemetry& t = ac.telemetry();
    const InstrumentReadings& r = ac.instruments().readings();
    std::ostringstream os;
    os << std::fixed << std::setprecision(0) << msToKt(r.indicatedAirspeed) << " KIAS, " << mToFt(t.altitudeMsl)
       << " ft MSL (" << mToFt(t.altitudeMsl - fieldElevation) << " AGL), VS " << msToFpm(t.verticalSpeed) << " fpm, hdg "
       << std::setw(3) << std::lround(radToDeg(t.heading) + 360) % 360 << " T, " << t.engines[0].rpm << " rpm, fuel "
       << std::setprecision(1) << t.fuelMass << " kg, mass " << std::setprecision(0) << t.mass << " kg";
    return os.str();
}

std::unique_ptr<std::ofstream> openCsv(const char* path, std::unique_ptr<TelemetryRecorder>& rec) {
    if (!path) return nullptr;
    auto f = std::make_unique<std::ofstream>(path);
    rec = std::make_unique<TelemetryRecorder>(*f, 0.25);
    rec->writeHeader();
    return f;
}

}  // namespace

// ---------------------------------------------------------------- flight ----

int flightDemo(const char* csv) {
    const AircraftDefinition def = loadReference();
    const LocalFrame frame = saoPauloWorldFrame();
    const Airport sbmt = campoDeMarte(frame);
    auto ground = makeAirfieldGround(sbmt, frame);
    SimpleWeather::Params wp;
    wp.groundElevation = sbmt.elevation;
    wp.conditions.temperatureDeviation = 10;  // a warm São Paulo morning (ISA+10)
    wp.conditions.relativeHumidity = 0.7;
    wp.windSpeed = 4;                         // light easterly
    wp.windFromDirection = degToRad(110);
    wp.turbulence.intensity = 0.6;
    SimpleWeather atm(wp);
    Aircraft ac(def, frame);
    ac.instruments().setAltimeterSetting(atm.atmosphere().conditions().seaLevelPressure);
    const RunwayDef& rw = sbmt.runways[0];
    const Real elev = sbmt.elevation;
    std::unique_ptr<TelemetryRecorder> rec;
    auto file = openCsv(csv, rec);

    std::cout << "# Scripted flight: " << def.displayName << " at " << sbmt.name << " (" << sbmt.icao << "), runway "
              << rw.ident << ", " << rw.length << " m, elevation " << elev << " m\n";
    std::cout << "# Weather: ISA+10, RH 70%, wind 110/4 m/s, light turbulence. World frame anchored at MASP.\n\n";

    // Line up at the start of runway 12, cold and dark.
    const Vec3 start = runwayPoint(rw, 30);
    ac.placeOnGround(start.x, start.y, rw.headingTrue, *ground, atm);
    ac.setColdAndDark();
    log(ac, "cold and dark on runway " + rw.ident + ": " + status(ac, elev));

    // 1. Quick Start (performed through the cockpit, step by step).
    ProcedureRunner run;
    run.start(quickStartPistonSingle(ac));
    int lastStep = -1;
    while (run.status() == ProcedureRunner::Status::Running && ac.time() < 120) {
        if (run.currentStep() != lastStep) {
            lastStep = run.currentStep();
            log(ac, "Quick Start: " + run.message());
        }
        PilotInput in;
        run.update(ac, in, kDt);
        ac.step(kDt, in, *ground, atm);
        drain(ac);
    }
    if (run.status() != ProcedureRunner::Status::Done) {
        log(ac, "Quick Start FAILED: " + run.message());
        return 1;
    }
    for (int i = 0; i < 20 * 240; ++i) {  // warm-up at 1000 rpm
        PilotInput in;
        in.throttle = 0.12;
        ac.step(kDt, in, *ground, atm);
    }
    log(ac, "engine warm, alternator online: " + status(ac, elev));

    // 2. Take-off: flaps 10, full throttle, mixture leaned for the field
    // density (the POH technique above 3000 ft density altitude).
    ac.setCockpitControl("flaps.lever", 1);
    for (int i = 0; i < 6 * 240; ++i) ac.step(kDt, PilotInput{0, 0, 0, 0.12, 1.0}, *ground, atm);
    ac.setCockpitControl("brakes.parking", 0);
    TestPilot pilot;
    pilot.controls.throttle = 1.0;
    pilot.controls.mixture = bestPowerMixture(def, ac.telemetry().air.state.density);
    pilot.lateral = TestPilot::Lateral::Track;
    pilot.trackOrigin = rw.threshold;
    pilot.targetHeading = rw.headingTrue;
    pilot.vertical = TestPilot::Vertical::Manual;
    log(ac, "take-off roll, flaps 10, full throttle (density altitude " +
                std::to_string(std::lround(mToFt(ac.telemetry().air.state.densityAltitude))) + " ft)");
    enum class Phase { Roll, Climb, Stall, Recover, Downwind, Base, Final, Flare, Rollout, Done };
    Phase phase = Phase::Roll;
    Real phaseTime = 0, stallStart = 0, speedIntegral = 0;
    const Real practiceAlt = elev + 900;  // stall practice ~3000 ft above the field
    const Real circuitAlt = elev + 300;   // ~1000 ft circuit
    const Real touchdownAlong = 300;      // aim point down runway 12
    const Vec3 aim = runwayPoint(rw, touchdownAlong);
    const Real glide = degToRad(3.5);
    int lastPrint = -1;
    auto setPhase = [&](Phase p, const std::string& msg) {
        phase = p;
        phaseTime = 0;
        log(ac, msg + ": " + status(ac, elev));
    };
    while (phase != Phase::Done && ac.time() < 2400) {
        const AircraftTelemetry& t = ac.telemetry();
        const Real ias = ac.instruments().readings().indicatedAirspeed;
        const Real agl = t.altitudeMsl - elev;
        const Vec3 rel = t.positionNed - aim;
        const Real ch = std::cos(rw.headingTrue), sh = std::sin(rw.headingTrue);
        const Real along = rel.x * ch + rel.y * sh;    // + beyond the aim point
        const Real across = -rel.x * sh + rel.y * ch;  // + right of the centreline
        switch (phase) {
            case Phase::Roll:
                if (ias > ktToMs(52)) {
                    pilot.vertical = TestPilot::Vertical::Pitch;
                    pilot.targetPitch = degToRad(9);
                }
                if (!t.onGround && agl > 15) {
                    pilot.vertical = TestPilot::Vertical::Airspeed;
                    pilot.targetIas = ktToMs(74);
                    pilot.targetPitch = degToRad(8);
                    setPhase(Phase::Climb, "climbing at Vy 74 KIAS on the runway heading");
                }
                break;
            case Phase::Climb:
                if (agl > 100 && ac.flapDetent() != 0) ac.setCockpitControl("flaps.lever", 0);
                if (t.altitudeMsl > practiceAlt) {
                    pilot.vertical = TestPilot::Vertical::Airspeed;
                    pilot.controls.throttle = 0;
                    pilot.targetIas = ktToMs(74);
                    pilot.targetPitch = 0;
                    stallStart = ac.time();
                    setPhase(Phase::Stall, "power-off stall practice: idle, slowing 1 kt/s");
                }
                break;
            case Phase::Stall:
                pilot.targetIas = std::max(ktToMs(74) - ktToMs(1.0) * (ac.time() - stallStart), ktToMs(30));
                if (ac.aero().surfaces[0].stalledStrips >= 6 || (t.omega.y < -0.12 && t.alpha > 0.15)) {
                    pilot.vertical = TestPilot::Vertical::Pitch;
                    pilot.targetPitch = degToRad(-5);
                    pilot.controls.throttle = 1.0;
                    setPhase(Phase::Recover, "STALL BREAK -- recovering: nose down, full power");
                }
                break;
            case Phase::Recover:
                if (phaseTime > 2 && ias > ktToMs(70)) {
                    // Descend to circuit height while turning downwind (left-hand
                    // circuit: reciprocal track 1.5 km left of the runway).
                    pilot.vertical = TestPilot::Vertical::Altitude;
                    pilot.targetAltitude = circuitAlt;
                    pilot.targetPitch = degToRad(0);
                    pilot.controls.throttle = 0.45;
                    pilot.lateral = TestPilot::Lateral::Track;
                    pilot.targetHeading = wrapPi(rw.headingTrue + kPi);
                    pilot.trackOrigin = runwayPoint(rw, 0, -1500);
                    setPhase(Phase::Downwind, "recovered; descending to circuit height, turning downwind");
                }
                break;
            case Phase::Downwind:
                if (std::abs(t.altitudeMsl - circuitAlt) < 30) pilot.controls.throttle = 0.6;
                if (along < -6000) {
                    pilot.targetHeading = rw.headingTrue;
                    pilot.trackOrigin = rw.threshold;
                    pilot.controls.throttle = 0.45;
                    ac.setCockpitControl("flaps.lever", 1);
                    setPhase(Phase::Base, "turning base to intercept the runway 12 centreline, flaps 10");
                }
                break;
            case Phase::Base:
                if (std::abs(across) < 150 && std::abs(wrapPi(t.heading - rw.headingTrue)) < 0.25) {
                    ac.setCockpitControl("flaps.lever", 3);
                    pilot.vertical = TestPilot::Vertical::Altitude;
                    pilot.targetPitch = degToRad(-2);
                    setPhase(Phase::Final, "established on final, flaps 30, 65 KIAS, " +
                                               std::to_string(std::lround(-along)) + " m to the aim point");
                }
                break;
            case Phase::Final: {
                // Pitch tracks the moving glide-path altitude (with the path's
                // sink rate fed forward); power holds 65 KIAS.
                pilot.targetAltitude = elev + 1.0 + std::max(-along, 0.0) * std::tan(glide);
                pilot.verticalSpeedFeedForward = -t.groundSpeed * std::tan(glide);
                speedIntegral = clamp(speedIntegral + (ktToMs(65) - ias) * kDt * 0.02, -0.3, 0.3);
                pilot.controls.throttle = clamp(0.3 + 0.08 * (ktToMs(65) - ias) + speedIntegral, 0.0, 1.0);
                if (agl < 5) {
                    pilot.verticalSpeedFeedForward = 0;
                    pilot.vertical = TestPilot::Vertical::Pitch;
                    pilot.targetPitch = degToRad(5);
                    pilot.controls.throttle = 0;
                    setPhase(Phase::Flare, "flare, power idle");
                }
                break;
            }
            case Phase::Flare:
                if (t.onGround && phaseTime > 0.5) {
                    pilot.vertical = TestPilot::Vertical::Manual;
                    pilot.controls.brakeLeft = pilot.controls.brakeRight = 0.6;
                    setPhase(Phase::Rollout, "on the runway, braking");
                }
                break;
            case Phase::Rollout:
                if (t.groundSpeed < 0.5) {
                    ac.setCockpitControl("brakes.parking", 1);
                    ac.setCockpitControl("flaps.lever", 0);
                    Real down;
                    Vec3 n;
                    SurfaceProperties surf;
                    ground->surfaceAt(t.positionNed.x, t.positionNed.y, down, n, surf);
                    setPhase(Phase::Done, "stopped " + std::to_string(std::lround(along)) + " m past the aim point, " +
                                              std::to_string(std::lround(across)) + " m off the centreline, on " +
                                              (surf.id == surfaces::kAsphalt.id ? "the runway (asphalt)" : "grass"));
                }
                break;
            case Phase::Done: break;
        }
        PilotInput in = pilot.update(ac, kDt);
        if (t.onGround) {
            in.yaw = clamp(-3.0 * wrapPi(t.heading - rw.headingTrue) - 0.8 * t.omega.z, -1.0, 1.0);
            if (phase == Phase::Rollout) in.pitch = 0.6;
        }
        ac.step(kDt, in, *ground, atm);
        phaseTime += kDt;
        if (rec) rec->record(ac);
        drain(ac);
        const int sec = static_cast<int>(ac.time());
        if (sec % 30 == 0 && sec != lastPrint && phase != Phase::Roll) {
            lastPrint = sec;
            log(ac, status(ac, elev));
        }
    }
    const Real used = def.fuel.tanks[0].initial * 2 * def.fuel.density - ac.telemetry().fuelMass;
    std::cout << "\nFlight complete. Fuel used " << std::setprecision(1) << used << " kg ("
              << m3ToGal(used / def.fuel.density) << " US gal); the aircraft is " << used
              << " kg lighter than at engine start (mass and CG followed the burn).\n";
    if (csv) std::cout << "Telemetry: " << csv << "\n";
    return phase == Phase::Done ? 0 : 1;
}

// ---------------------------------------------------- scenario A: fuel ----

int starvationDemo(const char* csv) {
    const AircraftDefinition def = loadReference();
    const LocalFrame frame = saoPauloWorldFrame();
    const Airport sbmt = campoDeMarte(frame);
    auto ground = makeAirfieldGround(sbmt, frame);
    SimpleWeather::Params wp;
    wp.groundElevation = sbmt.elevation;
    SimpleWeather atm(wp);
    Aircraft ac(def, frame);
    std::unique_ptr<TelemetryRecorder> rec;
    auto file = openCsv(csv, rec);
    // Departure with 3 litres of usable fuel per tank.
    for (int t = 0; t < 2; ++t) ac.fuel().setTankQuantity(t, def.fuel.tanks[t].unusable + 0.003);
    ac.refreshMass();
    TestPilot pilot;
    startTrimmedFlight(ac, {95, sbmt.elevation + 600, 0}, atm, *ground, &pilot);
    pilot.lateral = TestPilot::Lateral::Heading;
    pilot.targetHeading = degToRad(330);
    std::cout << "# Scenario A: fuel exhaustion. " << def.displayName << " cruising over São Paulo with "
              << "3 L usable per tank (+ unusable fuel).\n";
    log(ac, status(ac, sbmt.elevation));
    const Real m0 = ac.massProperties().mass;
    Real quit = -1, burned = 0;
    while (ac.time() < 3600) {
        ac.step(kDt, pilot.update(ac, kDt), *ground, atm);
        burned += ac.telemetry().engines[0].engine.fuelFlow * kDt;
        if (rec) rec->record(ac);
        drain(ac);
        if (quit < 0 && !ac.telemetry().engines[0].engine.firing) {
            quit = ac.time();
            log(ac, "engine quit: " + status(ac, sbmt.elevation));
            pilot.vertical = TestPilot::Vertical::Airspeed;  // best glide ~68 KIAS (POH)
            pilot.targetIas = ktToMs(68);
            pilot.targetPitch = 0;
            pilot.controls.throttle = 0;
        }
        if (quit < 0 && static_cast<int>(ac.time() / kDt) % static_cast<int>(60 / kDt) == 0) log(ac, status(ac, sbmt.elevation));
        if (quit > 0 && (ac.telemetry().onGround || ac.time() > quit + 300)) break;
    }
    const AircraftTelemetry& t = ac.telemetry();
    std::cout << "\nFuel burned " << std::setprecision(2) << burned << " kg in " << std::setprecision(1) << quit / 60
              << " min; mass fell by " << std::setprecision(2) << m0 - ac.massProperties().mass << " kg. Unusable fuel left: "
              << ac.fuel().tankQuantity(0) * 1000 << " + " << ac.fuel().tankQuantity(1) * 1000 << " L.\n";
    std::cout << "Glide: " << std::setprecision(0) << (ac.time() - quit) << " s to " << (t.onGround ? "touchdown" : "the end of the run")
              << ", windmilling at " << t.engines[0].rpm << " rpm; vacuum gyro " << std::setprecision(0)
              << ac.vacuum().gyroSpin() * 100 << "% speed.\n";
    return quit > 0 ? 0 : 1;
}

// -------------------------------------------- scenario C: fuel shutoff ----

int shutdownDemo() {
    const AircraftDefinition def = loadReference();
    SimpleWeather atm;
    FlatGround ground(0);
    Aircraft ac(def);
    TestPilot pilot;
    startTrimmedFlight(ac, {105, 2000, 0}, atm, ground, &pilot);
    std::cout << "# Scenario C (single-engine analogue): the player looks at a knob and presses F.\n";
    auto line = [&](const std::string& what) {
        const AircraftTelemetry& t = ac.telemetry();
        const InstrumentReadings& r = ac.instruments().readings();
        std::ostringstream os;
        os << what << std::fixed << std::setprecision(0) << " | rpm " << r.tachometerRpm << ", fuel flow "
           << std::setprecision(1) << m3ToGal(t.engines[0].engine.fuelFlow / def.fuel.density) * 3600 << " gph, thrust "
           << std::setprecision(0) << t.engines[0].prop.thrust << " N, " << msToKt(r.indicatedAirspeed) << " KIAS, VS "
           << msToFpm(t.verticalSpeed) << " fpm, bus " << std::setprecision(1) << r.voltmeter << " V, suction "
           << paToInHg(ac.vacuum().suction()) << " inHg";
        log(ac, os.str());
    };
    for (int i = 0; i < 10 * 240; ++i) ac.step(kDt, pilot.update(ac, kDt), ground, atm);
    drain(ac);
    line("cruise, altitude hold");
    const int idx = ac.cockpitControlIndex("fuel.shutoff");
    log(ac, "Press F to interact -- " + ac.cockpitControls()[idx].label);
    ac.cycleCockpitControl("fuel.shutoff");
    drain(ac);
    bool gliding = false;
    for (int s = 0; s < 70; ++s) {
        for (int i = 0; i < 240; ++i) {
            if (!gliding && !ac.telemetry().engines[0].engine.firing) {
                // Only now does the pilot notice: best-glide attitude.
                gliding = true;
                pilot.vertical = TestPilot::Vertical::Airspeed;
                pilot.targetIas = ktToMs(68);
                pilot.targetPitch = 0;
            }
            ac.step(kDt, pilot.update(ac, kDt), ground, atm);
        }
        drain(ac);
        if (s % 10 == 9) line("valve closed");
    }
    log(ac, "Press F to interact -- " + ac.cockpitControls()[idx].label);
    ac.cycleCockpitControl("fuel.shutoff");
    pilot.controls.throttle = 0.8;
    pilot.targetIas = ktToMs(85);
    for (int s = 0; s < 20; ++s) {
        for (int i = 0; i < 240; ++i) ac.step(kDt, pilot.update(ac, kDt), ground, atm);
        drain(ac);
        if (s % 5 == 4) line("valve open");
    }
    std::cout << "\nNothing above is scripted except the two key presses and the pilot holding airspeed: the line\n"
                 "drained, combustion stopped, the propeller windmilled, and fuel pressure brought it back.\n";
    return ac.telemetry().engines[0].engine.running ? 0 : 1;
}
