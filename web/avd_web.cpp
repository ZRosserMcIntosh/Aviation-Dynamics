// Aviation Dynamics — WebAssembly bindings for the browser flight demo.
// SPDX-License-Identifier: MIT
//
// A thin C ABI over avd::Aircraft so a web page can fly the real core. The page
// only renders and reads input. World positions are returned in the São Paulo
// World frame (ENU metres from MASP); see web/README.md for the buffer layout.
#include <algorithm>
#include <cstring>
#include <memory>
#include <string>

#include "airports.hpp"
#include "scenarios.hpp"
#include "avd/aircraft_loader.hpp"
#include "avd/assists.hpp"
#include "avd/procedures.hpp"
#include "avd/units.hpp"
#include "avd/world_frame.hpp"

#if defined(__EMSCRIPTEN__)
#include <emscripten/emscripten.h>
#define AVD_EXPORT extern "C" EMSCRIPTEN_KEEPALIVE
#else
#define AVD_EXPORT extern "C" __attribute__((visibility("default")))
#endif

using namespace avd;
using namespace avd::units;

namespace {

constexpr Real kDt = 1.0 / 240.0;
constexpr int kStateSize = 64;

struct Sim {
    AircraftDefinition def;
    LocalFrame frame = scenarios::saoPauloWorldFrame();
    scenarios::Airport airport;
    std::unique_ptr<AirfieldGround> ground;
    std::unique_ptr<SimpleWeather> weather;
    SimpleWeatherParams weatherParams;
    std::unique_ptr<Aircraft> ac;
    Real trimThrottle = 0, trimMixture = 1;  // controls of the last trimmed spawn
    FlightAssist assist{makePreset(AssistPreset::Sport)};
    bool levelHold = false;  // straight-and-level hold, on top of any preset
    ProcedureRunner procedure;
    PilotInput raw;
    Real accumulator = 0;
    std::string events;
    int jetStart = 0;  // jet Quick Start: 0 idle, 1 starting engine 1, 2 engine 2, 3 done
    // Trimmed CLmax cache for the low-speed awareness band and V-speeds.
    std::unique_ptr<AeroModel> clmaxModel;
    Real clmaxKey = -1, clmax = 0, clmaxTakeoff = 0, clmaxLanding = 0;
};

Sim* g = nullptr;
double g_state[kStateSize];
double g_cockpit[96];
double g_jet[64];
constexpr int kFlowStrips = 64, kFlowStride = 16;
double g_flow[16 + kFlowStrips * kFlowStride + 4 * 10];
char* g_text = nullptr;
int g_textCap = 0;
std::vector<std::string> g_errors;

int putText(const std::string& s) {
    if (static_cast<int>(s.size()) + 1 > g_textCap) {
        std::free(g_text);
        g_textCap = static_cast<int>(s.size()) + 1024;
        g_text = static_cast<char*>(std::malloc(static_cast<std::size_t>(g_textCap)));
    }
    std::memcpy(g_text, s.data(), s.size());
    g_text[s.size()] = 0;
    return static_cast<int>(s.size());
}

const char* eventText(FlightEventType t) {
    switch (t) {
        case FlightEventType::EngineStarted: return "Engine running";
        case FlightEventType::EngineStopped: return "ENGINE STOPPED";
        case FlightEventType::FuelStarvation: return "FUEL STARVATION";
        case FlightEventType::StallWarningOn: return "";
        case FlightEventType::StallWarningOff: return "";
        case FlightEventType::Touchdown: return "Touchdown";
        case FlightEventType::Liftoff: return "Lift-off";
        case FlightEventType::Overspeed: return g && !g->def.turbofans.empty() ? "OVERSPEED: above VMO" : "OVERSPEED: above VNE";
        case FlightEventType::MachOverspeed: return "OVERSPEED: above MMO";
        case FlightEventType::GearOverspeed: return "Gear extended above VLE";
        case FlightEventType::OverG: return "LIMIT LOAD EXCEEDED";
        case FlightEventType::FlapOverspeed: return "Flaps above VFE";
        case FlightEventType::PropStrike: return "PROPELLER STRIKE";
        case FlightEventType::HardPointContact: return "Airframe ground contact";
        case FlightEventType::BatteryDepleted: return "Battery depleted";
        case FlightEventType::AlternatorOffline: return g && !g->def.turbofans.empty() ? "Generators offline" : "Alternator offline";
        case FlightEventType::AlternatorOnline: return g && !g->def.turbofans.empty() ? "Generators online" : "Alternator online";
        case FlightEventType::CockpitControlChanged: return "";
    }
    return "";
}

void collectEvents() {
    for (const FlightEvent& e : g->ac->drainEvents()) {
        std::string s = eventText(e.type);
        if (e.type == FlightEventType::CockpitControlChanged) {
            const CockpitControl& c = g->ac->cockpitControls()[static_cast<std::size_t>(e.index)];
            s = c.label + " -> " + c.positions[static_cast<std::size_t>(e.value)];
        }
        if (e.type == FlightEventType::Touchdown)
            s += " (" + std::to_string(static_cast<int>(msToFpm(e.value))) + " fpm)";
        if (e.type == FlightEventType::HardPointContact || e.type == FlightEventType::PropStrike)
            s += " (" + g->def.contacts[static_cast<std::size_t>(e.index)].name + ")";
        if (!s.empty()) g->events += s + "\n";
    }
    if (g->events.size() > 4096) g->events.erase(0, g->events.size() - 4096);
}

}  // namespace

// Text buffer shared with JavaScript (UTF-8, NUL-terminated).
AVD_EXPORT char* avd_text(int capacity) {
    if (capacity > g_textCap) putText(std::string(static_cast<std::size_t>(capacity), ' '));
    return g_text;
}

// Load an aircraft definition (JSON already written to avd_text). Returns the
// number of errors; read them with avd_error_text(i).
AVD_EXPORT int avd_load(int length) {
    const LoadResult r = loadAircraftFromJson(std::string(g_text, static_cast<std::size_t>(length)));
    g_errors = r.errors;
    if (!r.ok()) return static_cast<int>(g_errors.size());
    delete g;
    g = new Sim;
    g->def = r.definition;
    g->airport = scenarios::campoDeMarte(g->frame);
    g->ground = scenarios::makeAirfieldGround(g->airport, g->frame);
    SimpleWeather::Params wp;
    wp.groundElevation = g->airport.elevation;
    g->weather = std::make_unique<SimpleWeather>(wp);
    g->ac = std::make_unique<Aircraft>(g->def, g->frame);
    g->ac->setNavEnvironment(&scenarios::saoPauloNavEnvironment());
    return 0;
}

AVD_EXPORT int avd_error_text(int i) {
    return i >= 0 && i < static_cast<int>(g_errors.size()) ? putText(g_errors[static_cast<std::size_t>(i)]) : 0;
}

AVD_EXPORT void avd_set_weather(double windSpeed, double windFromDeg, double turbulence, double tempDev, double humidity) {
    if (!g) return;
    SimpleWeather::Params wp;
    wp.groundElevation = g->airport.elevation;
    wp.windSpeed = windSpeed;
    wp.windFromDirection = degToRad(windFromDeg);
    wp.turbulence.intensity = turbulence;
    wp.conditions.temperatureDeviation = tempDev;
    wp.conditions.relativeHumidity = humidity;
    wp.cloudBase = g->weatherParams.cloudBase;
    wp.cloudTop = g->weatherParams.cloudTop;
    wp.cloudLiquidWater = g->weatherParams.cloudLiquidWater;
    g->weatherParams = wp;
    g->weather = std::make_unique<SimpleWeather>(wp);
}

// One cloud layer (m MSL) with its liquid water content (g/m^3); top <= base
// clears it. Supercooled (an icing layer) where the air is below 0 C.
AVD_EXPORT void avd_set_cloud(double baseMsl, double topMsl, double liquidWater) {
    if (!g) return;
    g->weatherParams.cloudBase = baseMsl;
    g->weatherParams.cloudTop = topMsl;
    g->weatherParams.cloudLiquidWater = liquidWater;
    g->weather = std::make_unique<SimpleWeather>(g->weatherParams);
}

// 0 cold and dark on runway 12, 1 engine running on runway 12, 2 on final
// for runway 12 (3 km), 3 cruising at 3500 ft over the city.
namespace {

// A fresh aircraft for a reset.
void newAircraft() {
    g->ac = std::make_unique<Aircraft>(g->def, g->frame);
    g->ac->setNavEnvironment(&scenarios::saoPauloNavEnvironment());
    g->procedure = ProcedureRunner{};
    g->assist.reset();
    g->accumulator = 0;
    g->events.clear();
    g->jetStart = 0;
    g->trimThrottle = 0;
    g->trimMixture = 1;
}

// Trimmed airborne spawn: solve the free-air trim in calm air at the spawn
// altitude (throttle solved for the flight path), rotate the trimmed
// north-heading state onto `heading` and place it at `posNed` / `altMsl`;
// the air-relative velocity gets the wind. Hands-off on arrival: the trimmed
// elevator goes into the trim wheel.
void placeTrimmed(Vec3 posNed, Real altMsl, Real heading, Real kias, int flapDetent, Real path) {
    g->ac->setEngineRunning(true, 2000);
    // Retractable gear: down on the approach (landing flap), up otherwise.
    g->ac->setGearImmediately(flapDetent >= 2);
    SimpleWeatherParams calm = g->weatherParams;
    calm.windSpeed = 0;
    calm.turbulence.intensity = 0;
    const SimpleWeather trimAir(calm);
    scenarios::TrimRequest req;
    req.altitude = altMsl;
    req.flapDetent = flapDetent;
    req.tas = scenarios::tasFromCas(ktToMs(kias), trimAir.atmosphere().at(altMsl));
    req.flightPath = path;
    const scenarios::TrimResult tr = scenarios::trim(*g->ac, req, trimAir);
    DynamicState s = tr.state;
    const Quat toHeading = Quat::fromEuler(0, 0, heading);
    s.body.attitude = toHeading * s.body.attitude;
    s.body.attitude.normalize();
    posNed.z = -altMsl;
    s.body.position = posNed;
    s.body.velocity = toHeading.rotate(s.body.velocity) + g->weather->sample(posNed, altMsl, 0).wind;
    g->ac->setState(s);
    g->ac->setCockpitControl("brakes.parking", 0);
    // The air sample and ground planes must describe the new position
    // before the downwash and the instruments are initialised from them.
    g->ac->prepareContext(tr.input, *g->ground, *g->weather);
    g->ac->settleDownwash();
    Real elevUp = 0.4;
    for (const auto& sd : g->def.surfaces)
        for (const auto& pd : sd.panels)
            for (const auto& c : pd.controls)
                if (c.channel == ControlChannel::Elevator) elevUp = c.maxNegative;
    g->ac->setElevatorTrim(tr.elevator * elevUp / g->def.controls.elevatorTrimRange);
    g->trimThrottle = tr.input.throttle;
    g->trimMixture = tr.input.mixture;
}

// Instruments and radios after any reset.
void finishReset(bool seedInstruments) {
    // One step (1/240 s) at the trimmed controls so telemetry, the air sample
    // and every derived value describe the new state before the host reads it.
    PilotInput in;
    in.throttle = g->trimThrottle;
    in.mixture = g->trimMixture;
    g->ac->step(kDt, in, *g->ground, *g->weather);
    g->ac->instruments().setAltimeterSetting(g->weather->atmosphere().conditions().seaLevelPressure);
    if (seedInstruments) g->ac->resetInstrumentsToTruth();
    // Radios: NAV 1 on the (synthetic) runway 12 localizer, the test VOR in standby.
    InstrumentPanel& p = g->ac->instruments();
    p.setNavStandby(0, 110300);
    p.swapNav(0);
    p.setNavStandby(0, 113700);
    p.setObs(degToRad(122));
    p.setHeadingBug(degToRad(122));
    p.setComStandby(0, 132800);
    p.setTransponderMode(TransponderMode::Altitude);
    g->ac->drainEvents();
}

}  // namespace

AVD_EXPORT void avd_reset(int mode) {
    if (!g) return;
    newAircraft();
    const RunwayDef& rw = g->airport.runways[0];
    if (mode <= 1) {
        const Vec3 p = scenarios::runwayPoint(rw, 30);
        if (mode == 1) g->ac->setEngineRunning(true, 1000);
        g->ac->placeOnGround(p.x, p.y, rw.headingTrue, *g->ground, *g->weather);
        if (mode == 0) g->ac->setColdAndDark();
    } else {
        // Mode 2: on the 3 deg final for runway 12 (65 KIAS, flaps 20), exactly
        // on the glide path, which starts at the glideslope antenna 300 m beyond
        // the paved end (scenarios/airports.cpp). Mode 3: level cruise, 100 KIAS.
        // Jets: final at 125 KIAS (about Vref + 10) with landing flap and the gear down, cruise at 250 KIAS.
        const bool final = mode == 2;
        const bool jet = g->ac->turbofanCount() > 0;
        const int landingFlap = jet ? static_cast<int>(g->def.controls.flapDetents.size()) - 1 : 2;
        const Real dist = final ? (jet ? -6000.0 : -3000.0) : -8000.0;
        const Real path = final ? degToRad(-3.0) : 0.0;
        const Real alt = g->airport.elevation + (final ? (-dist + 300.0) * std::tan(-path) : (jet ? 1500.0 : 850.0));
        const Real kias = final ? (jet ? 125.0 : 65.0) : (jet ? 250.0 : 100.0);
        placeTrimmed(scenarios::runwayPoint(rw, dist), alt, rw.headingTrue, kias, final ? landingFlap : 0, path);
    }
    finishReset(mode >= 1);
}

// Trimmed, level, hands-off spawn anywhere in the world: world ENU position
// (m east and north of the world anchor), height above the field elevation
// (m), true heading (deg), indicated airspeed (kt) and flap detent.
AVD_EXPORT void avd_reset_air(double east, double north, double heightAboveField, double headingDeg, double kias,
                              int flapDetent) {
    if (!g) return;
    newAircraft();
    const Vec3 ned = world::nedFromEnu({east, north, 0});
    const AircraftDefinition& d = g->def;
    const bool jet = !d.turbofans.empty();
    // Keep the request flyable: below the type's ceiling, no faster than Vmo
    // (or Mmo at height), flaps no further than the speed allows, and at least
    // 1.25 Vs for that configuration and weight.
    const Real height = std::clamp(heightAboveField, 30.0, jet ? 14500.0 : 4500.0);
    const Real altMsl = g->airport.elevation + height;
    const std::vector<Real>& det = d.controls.flapDetents;
    int flap = std::clamp(flapDetent, 0, static_cast<int>(det.size()) - 1);
    Real v = std::clamp(kias, 55.0, d.limits.neverExceedIas > 0 ? msToKt(d.limits.neverExceedIas) : 350.0);
    const AirState air = StandardAtmosphere().at(altMsl);
    while (d.limits.maxOperatingMach > 0 && v > 60 &&
           scenarios::tasFromCas(ktToMs(v), air) / air.speedOfSound > d.limits.maxOperatingMach - 0.03)
        v -= 1;
    auto vfe = [&](int k) {
        if (det[static_cast<std::size_t>(k)] <= 0) return 1e9;
        const Real ms = !d.limits.flapExtendedIasVsFlap.empty() ? d.limits.flapExtendedIasVsFlap(det[static_cast<std::size_t>(k)])
                                                                 : d.limits.flapExtendedIas;
        return ms > 0 ? msToKt(ms) : 1e9;
    };
    while (flap > 0 && v > vfe(flap)) --flap;
    // Jets at height: no slower than Mach 0.72 (behind the drag curve otherwise).
    if (jet && altMsl > 9000)
        while (v < 300 && scenarios::tasFromCas(ktToMs(v), air) / air.speedOfSound < 0.72) v += 1;
    AeroModel am(d);
    const Real cl = trimmedMaxLiftCoefficient(am, d.referenceArea, g->ac->massProperties().cg, det[static_cast<std::size_t>(flap)], 0.2);
    if (cl > 0.1) v = std::max(v, 1.25 * msToKt(std::sqrt(2 * g->ac->massProperties().mass * 9.80665 / (1.225 * d.referenceArea * cl))));
    placeTrimmed(ned, altMsl, degToRad(headingDeg), v, flap, 0.0);
    finishReset(true);
}

// Compass correction card from a compass swing: what the compass reads (deg)
// on magnetic headings 0, 30, ... 330 ("FOR" -> "STEER").
AVD_EXPORT double* avd_compass_card() {
    std::fill(std::begin(g_state), std::end(g_state), 0.0);
    if (!g) return g_state;
    const std::array<Real, 12> sw = g->ac->instruments().compassSwing(scenarios::saoPauloNavEnvironment().field);
    for (int k = 0; k < 12; ++k) g_state[k] = std::fmod(radToDeg(sw[static_cast<std::size_t>(k)]) + 360.0, 360.0);
    return g_state;
}

// Throttle and mixture of the last trimmed spawn (modes 2 and 3, avd_reset_air), so the
// host's levers start where the trim put them.
AVD_EXPORT double avd_trim_throttle() { return g ? g->trimThrottle : 0.0; }
AVD_EXPORT double avd_trim_mixture() { return g ? g->trimMixture : 1.0; }

AVD_EXPORT void avd_set_input(double pitch, double roll, double yaw, double throttle, double mixture, double brakeLeft,
                              double brakeRight, double trimRate, int starter) {
    if (!g) return;
    g->raw.pitch = pitch;
    g->raw.roll = roll;
    g->raw.yaw = yaw;
    g->raw.throttle = throttle;
    g->raw.mixture = mixture;
    g->raw.brakeLeft = brakeLeft;
    g->raw.brakeRight = brakeRight;
    g->raw.trimRate = trimRate;
    g->raw.starterHeld = starter != 0;
}

// 0 Arcade, 1 Sport, 2 Simulation.
AVD_EXPORT void avd_set_assist(int preset) {
    if (!g) return;
    const AssistPreset p = preset == 0 ? AssistPreset::Arcade : (preset == 1 ? AssistPreset::Sport : AssistPreset::Simulation);
    AssistConfig c = makePreset(p);
    c.levelHold = g->levelHold;
    g->assist.setConfig(c);
}

// Straight-and-level hold (auto-trim that keeps the airplane straight and
// level when the stick is released), with any preset. Kept across presets.
AVD_EXPORT void avd_set_level_hold(int on) {
    if (!g) return;
    g->levelHold = on != 0;
    AssistConfig c = g->assist.config();
    c.levelHold = g->levelHold;
    g->assist.setConfig(c);
}

AVD_EXPORT void avd_quick_start() {
    if (!g) return;
    if (g->ac->turbofanCount() > 0) {
        // Jet: power up, then the fuel control switches one engine at a time
        // (each start runs the starter, light-off and spool-up in the core).
        Aircraft& ac = *g->ac;
        ac.setCockpitControl("electrical.battery", 1);
        ac.setCockpitControl("electrical.alternator", 1);
        ac.setCockpitControl("electrical.avionics", 1);
        ac.setCockpitControl("fuel.auxpump", 1);
        ac.setCockpitControl("electrical.load.beacon", 1);
        ac.setCockpitControl("engine1.fuel", 1);
        g->jetStart = 1;
        return;
    }
    g->procedure.start(quickStartPistonSingle(*g->ac));
}

namespace {
void updateJetStart() {
    if (!g || g->jetStart == 0 || g->jetStart == 3) return;
    Aircraft& ac = *g->ac;
    const int n = ac.turbofanCount();
    const int i = g->jetStart - 1;
    if (i < n && ac.turbofanState(i).running) {
        if (i + 1 < n) {
            ac.setCockpitControl("engine" + std::to_string(i + 2) + ".fuel", 1);
            g->jetStart = i + 2;
        } else {
            ac.setCockpitControl("electrical.load.nav_lights", 1);
            ac.setCockpitControl("electrical.load.strobes", 1);
            g->jetStart = 3;
        }
    }
}
}  // namespace

// Procedure status: 0 idle, 1 running, 2 done, 3 failed; message in the text buffer.
AVD_EXPORT int avd_procedure_status() {
    if (!g) return 0;
    if (g->jetStart > 0) {
        const int i = std::min(g->jetStart, g->ac->turbofanCount()) - 1;
        putText(g->jetStart == 3 ? "Both engines running." : "Starting engine " + std::to_string(i + 1) + "...");
        return g->jetStart == 3 ? 2 : 1;
    }
    putText(g->procedure.message());
    return static_cast<int>(g->procedure.status());
}

// Airflow (for the flow visualisation): what the strip aerodynamics computed
// this step. Header: [0] strips, [1] engines, [2] TAS m/s, [3] alpha, [4] beta
// (rad), [5] density, [6..8] CG (structural frame), [9] speed of sound.
// Strips from 16, 16 doubles each: quarter-chord position xyz, span axis xyz,
// normal (lift side) axis xyz, chord, area, cl, alpha, q, stall alpha, surface.
// Engines after the strips, 10 doubles each: position xyz, thrust axis xyz,
// radius, slipstream / jet velocity increment (m/s), thrust (N), kind (0
// propeller, 1 turbofan). Structural frame: x forward, y right, z down.
AVD_EXPORT double* avd_flow() {
    for (double& v : g_flow) v = 0;
    if (!g) return g_flow;
    const Aircraft& ac = *g->ac;
    const AeroModel& am = ac.aeroModel();
    const AeroOutputs& ao = ac.aero();
    const int n = std::min(am.stripCount(), kFlowStrips);
    g_flow[0] = n;
    g_flow[2] = ao.airspeed; g_flow[3] = ao.alpha; g_flow[4] = ao.beta;
    g_flow[5] = ac.telemetry().air.state.density;
    const Vec3 cg = ac.massProperties().cg;
    g_flow[6] = cg.x; g_flow[7] = cg.y; g_flow[8] = cg.z;
    g_flow[9] = ac.telemetry().air.state.speedOfSound;
    for (int i = 0; i < n; ++i) {
        const AeroModel::StripGeometry sg = am.stripGeometry(i);
        const StripTelemetry& t = ao.strips[static_cast<std::size_t>(i)];
        double* o = g_flow + 16 + i * kFlowStride;
        o[0] = sg.position.x; o[1] = sg.position.y; o[2] = sg.position.z;
        o[3] = sg.spanAxis.x; o[4] = sg.spanAxis.y; o[5] = sg.spanAxis.z;
        o[6] = sg.normalAxis.x; o[7] = sg.normalAxis.y; o[8] = sg.normalAxis.z;
        o[9] = sg.chord; o[10] = sg.area; o[11] = t.cl; o[12] = t.alpha; o[13] = t.q; o[14] = t.stallAlpha;
        o[15] = sg.surface;
    }
    double* e = g_flow + 16 + kFlowStrips * kFlowStride;
    int ne = 0;
    for (int i = 0; i < ac.engineCount() && ne < 4; ++i, ++ne) {
        const SlipstreamState& ss = ac.slipstream(i);
        double* o = e + ne * 10;
        o[0] = ss.discCentre.x; o[1] = ss.discCentre.y; o[2] = ss.discCentre.z;
        o[3] = ss.axis.x; o[4] = ss.axis.y; o[5] = ss.axis.z;
        o[6] = ss.radius; o[7] = ss.axialIncrement; o[8] = ac.telemetry().engines[static_cast<std::size_t>(i)].prop.thrust; o[9] = 0;
    }
    for (int i = 0; i < ac.turbofanCount() && ne < 4; ++i, ++ne) {
        const TurbofanDef& d = g->def.turbofans[static_cast<std::size_t>(i)];
        const TurbofanState& ts = ac.turbofanState(i);
        double* o = e + ne * 10;
        const Real rNozzle = 0.45;
        const Real rho = std::max(ac.telemetry().air.state.density, 0.05);
        const Real v0 = ao.airspeed;
        // Fully expanded jet: T = mdot (vj - v0), mdot = rho A vj.
        const Real A = kPi * rNozzle * rNozzle;
        const Real vj = 0.5 * (v0 + std::sqrt(v0 * v0 + 4 * std::max(ts.thrust, 0.0) / (rho * A)));
        o[0] = d.position.x; o[1] = d.position.y; o[2] = d.position.z;
        const Vec3 ax = normalized(d.thrustAxis);
        o[3] = ax.x; o[4] = ax.y; o[5] = ax.z;
        o[6] = rNozzle; o[7] = std::max(vj - v0, 0.0); o[8] = ts.thrust; o[9] = 1;
    }
    g_flow[1] = ne;
    return g_flow;
}

// Attitude indicator caging knob (square the gyro to the case; use in level flight).
AVD_EXPORT void avd_ai_cage() {
    if (g) g->ac->instruments().cageAttitude(g->ac->state().body.attitude);
}

// Jet systems (32 doubles): [0] turbofans, [1] gear position 0..1, [2] gear
// lever (1 down), [3] speedbrake position 0..1, [4] speedbrake detent, [5] Mach,
// [6] fuel kg, [7] fuel flow kg/h; engines at 8 + 8 i: N1 %, N2 %, ITT degC,
// fuel flow kg/h, thrust kN, running, starting, fuel switch; [24] MMO,
// [25] VMO kt, [26] TAS kt, [27] flap deg, [28] trim, [29] weight on wheels,
// [30], [31] oil pressure psi (engines 1, 2).
AVD_EXPORT double* avd_jet() {
    for (double& v : g_jet) v = 0;
    if (!g) return g_jet;
    const Aircraft& ac = *g->ac;
    const AircraftTelemetry& t = ac.telemetry();
    g_jet[0] = ac.turbofanCount();
    g_jet[1] = ac.gearPosition();
    g_jet[2] = ac.cockpitControl("gear.lever") != 0;
    g_jet[3] = ac.speedbrakePosition();
    g_jet[4] = std::max(ac.cockpitControl("speedbrake.lever"), 0);
    g_jet[5] = t.mach;
    g_jet[6] = t.fuelMass;
    Real ff = 0;
    for (int i = 0; i < std::min(ac.turbofanCount(), 2); ++i) {
        const TurbofanState& e = ac.turbofanState(i);
        double* o = g_jet + 8 + 8 * i;
        o[0] = e.n1 * 100; o[1] = e.n2 * 100; o[2] = e.itt - 273.15; o[3] = e.fuelFlow * 3600; o[4] = e.thrust / 1000;
        o[5] = e.running; o[6] = e.starting; o[7] = ac.cockpitControl("engine" + std::to_string(i + 1) + ".fuel") == 1;
        g_jet[30 + i] = e.oilPressure / 6894.757;
        ff += e.fuelFlow;
    }
    g_jet[7] = ff * 3600;
    g_jet[24] = g->def.limits.maxOperatingMach;
    g_jet[25] = msToKt(g->def.limits.neverExceedIas);
    g_jet[26] = msToKt(t.trueAirspeed);
    g_jet[27] = radToDeg(t.flapAngle);
    g_jet[28] = t.elevatorTrim;
    g_jet[29] = t.onGround;
    // Low-speed awareness and V-speeds: 1-g stall CAS from the trimmed CLmax at
    // this weight (current flap and Mach; take-off and landing flap at low Mach).
    const std::vector<Real>& det = g->def.controls.flapDetents;
    if (!g->clmaxModel) {
        g->clmaxModel = std::make_unique<AeroModel>(g->def);
        const Vec3 cg0 = ac.massProperties().cg;
        g->clmaxTakeoff = trimmedMaxLiftCoefficient(*g->clmaxModel, g->def.referenceArea, cg0, det[det.size() / 2], 0.2);
        g->clmaxLanding = trimmedMaxLiftCoefficient(*g->clmaxModel, g->def.referenceArea, cg0, det.back(), 0.2);
    }
    const Real key = std::round(radToDeg(t.flapAngle) * 2) * 1000 + std::round(std::max(t.mach, 0.2) * 20);
    if (key != g->clmaxKey) {
        g->clmaxKey = key;
        g->clmax = trimmedMaxLiftCoefficient(*g->clmaxModel, g->def.referenceArea, ac.massProperties().cg, t.flapAngle,
                                             std::max(t.mach, 0.2));
    }
    auto vs = [&](Real cl) { return cl > 0.1 ? msToKt(std::sqrt(2 * t.mass * 9.80665 / (1.225 * g->def.referenceArea * cl))) : 0.0; };
    g_jet[32] = vs(g->clmax);
    g_jet[33] = vs(g->clmaxTakeoff);
    g_jet[34] = vs(g->clmaxLanding);
    g_jet[35] = t.mass;
    const Vec3 vn = t.velocityNed;
    g_jet[36] = radToDeg(std::atan2(-vn.z, std::max(std::hypot(vn.x, vn.y), 0.5)));
    g_jet[37] = std::fmod(radToDeg(std::atan2(vn.y, vn.x)) + 360.0, 360.0);
    const Vec3 w = t.air.wind;  // air velocity, NED: the wind blows toward it
    g_jet[38] = std::fmod(radToDeg(std::atan2(-w.y, -w.x)) + 360.0, 360.0);
    g_jet[39] = msToKt(std::hypot(w.x, w.y));
    g_jet[40] = mToFt(t.heightAgl);
    g_jet[41] = kToCelsius(t.air.state.temperature);
    g_jet[42] = t.loadFactor;
    g_jet[43] = radToDeg(t.alpha);
    g_jet[44] = ac.electrical().alternatorOnline();
    g_jet[45] = t.busVoltage;
    g_jet[46] = t.stallWarning;
    const StructuralLimitsDef& lim = g->def.limits;
    g_jet[47] = t.flapAngle < degToRad(0.5) ? 0.0
                : msToKt(!lim.flapExtendedIasVsFlap.empty() ? lim.flapExtendedIasVsFlap(t.flapAngle) : lim.flapExtendedIas);
    g_jet[48] = msToKt(g->def.controls.gear.maxExtendedIas);
    g_jet[49] = det.size() > 2 ? radToDeg(det[det.size() / 2]) : 0.0;
    g_jet[50] = radToDeg(det.back());
    g_jet[51] = t.stickPusher;
    return g_jet;
}

// Advance by wall-clock seconds at a fixed 240 Hz (at most 0.25 s per call).
AVD_EXPORT int avd_advance(double seconds) {
    if (!g) return 0;
    g->accumulator += std::clamp(seconds, 0.0, 0.25);
    int n = 0;
    while (g->accumulator >= kDt) {
        PilotInput in = g->assist.apply(g->raw, *g->ac, kDt);
        g->procedure.update(*g->ac, in, kDt);
        g->ac->step(kDt, in, *g->ground, *g->weather);
        g->accumulator -= kDt;
        ++n;
    }
    updateJetStart();
    collectEvents();
    return n;
}

// Event lines since the last call (in the text buffer); returns the length.
AVD_EXPORT int avd_events() {
    if (!g) return 0;
    const int n = putText(g->events);
    g->events.clear();
    return n;
}

AVD_EXPORT int avd_cockpit_count() { return g ? static_cast<int>(g->ac->cockpitControls().size()) : 0; }

// "id|label|pos0;pos1;...|current" in the text buffer.
AVD_EXPORT int avd_cockpit_text(int i) {
    if (!g || i < 0 || i >= avd_cockpit_count()) return 0;
    const CockpitControl& c = g->ac->cockpitControls()[static_cast<std::size_t>(i)];
    std::string s = c.id + "|" + c.label + "|";
    for (std::size_t k = 0; k < c.positions.size(); ++k) s += (k ? ";" : "") + c.positions[k];
    s += "|" + std::to_string(g->ac->cockpitControl(c.id));
    return putText(s);
}

AVD_EXPORT int avd_cockpit_cycle(int i) {
    if (!g || i < 0 || i >= avd_cockpit_count()) return 0;
    return g->ac->cycleCockpitControl(g->ac->cockpitControls()[static_cast<std::size_t>(i)].id) ? 1 : 0;
}

// Runway 12/30 geometry in the world frame: [east, north, elevation, heading_true_deg, length, width].
AVD_EXPORT double* avd_runway() {
    if (!g) return g_state;
    const RunwayDef& r = g->airport.runways[0];
    const Vec3 e = world::enuFromNed(r.threshold);
    g_state[0] = e.x;
    g_state[1] = e.y;
    g_state[2] = g->airport.elevation;
    g_state[3] = radToDeg(r.headingTrue);
    g_state[4] = r.length;
    g_state[5] = r.width;
    return g_state;
}

AVD_EXPORT double* avd_state() {
    std::fill(std::begin(g_state), std::end(g_state), 0.0);
    if (!g) return g_state;
    const Aircraft& ac = *g->ac;
    const AircraftTelemetry& t = ac.telemetry();
    const CockpitState& r = ac.instruments().readings();
    const EngineTelemetry& e = t.engines[0];
    const Vec3 enu = world::enuFromNed(ac.datumPosition());
    const Quat q = world::enuFluFromNedFrd(ac.state().body.attitude);
    double* s = g_state;
    s[0] = enu.x; s[1] = enu.y; s[2] = enu.z;
    s[3] = q.w; s[4] = q.x; s[5] = q.y; s[6] = q.z;
    s[7] = radToDeg(t.roll); s[8] = radToDeg(t.pitch); s[9] = std::fmod(radToDeg(t.heading) + 360.0, 360.0);
    s[10] = msToKt(r.indicatedAirspeed); s[11] = msToKt(t.trueAirspeed); s[12] = msToKt(t.groundSpeed);
    s[13] = mToFt(r.indicatedAltitude); s[14] = mToFt(t.altitudeMsl - g->airport.elevation); s[15] = msToFpm(r.verticalSpeed);
    s[16] = radToDeg(t.alpha); s[17] = radToDeg(t.beta); s[18] = t.loadFactor;
    s[19] = r.tachometerRpm; s[20] = paToInHg(e.engine.manifoldPressure);
    s[21] = m3ToGal(r.fuelFlow / g->def.fuel.density) * 3600; s[22] = r.fuelGauge[0] * 1000; s[23] = r.fuelGauge[1] * 1000;
    s[24] = g->raw.throttle; s[25] = g->raw.mixture;
    s[26] = radToDeg(t.flapAngle); s[27] = t.elevatorTrim; s[28] = t.onGround; s[29] = r.stallHorn;
    s[30] = ac.turbofanCount() > 0 ? ac.turbofanState(0).running || ac.turbofanState(std::min(1, ac.turbofanCount() - 1)).running
                                   : e.engine.running; s[31] = r.voltmeter; s[32] = ac.electrical().alternatorOnline(); s[33] = paToInHg(ac.vacuum().suction());
    s[34] = ac.electrical().batteryCharge();
    s[35] = radToDeg(r.attitudeRoll); s[36] = radToDeg(r.attitudePitch); s[37] = std::fmod(radToDeg(r.headingIndicator) + 360.0, 360.0);
    s[38] = radToDeg(r.turnRate); s[39] = r.slipBall;
    s[40] = t.mass; s[41] = t.cg.x; s[42] = t.time; s[43] = t.stalledStrips; s[44] = r.oilPressure / 6894.76;
    s[45] = t.controls.aileron; s[46] = t.controls.elevator; s[47] = t.controls.rudder; s[48] = ac.faulted();
    s[49] = ac.parkingBrake(); s[50] = e.prop.thrust; s[51] = t.mach; s[52] = ac.fuel().totalQuantity() * 1000;
    s[53] = mToFt(t.altitudeMsl); s[54] = g->assist.report().active();
    s[55] = g->assist.report().levelHold; s[56] = g->assist.report().yawOnTiller; s[57] = ac.hasTiller();
    return s;
}

// ---- Cockpit instruments (display units) -----------------------------------
// See web/README.md for the layout. Displayed values come only from the
// instrument models; [48..] is truth for the debug overlay.
AVD_EXPORT double* avd_cockpit() {
    std::fill(std::begin(g_cockpit), std::end(g_cockpit), 0.0);
    if (!g) return g_cockpit;
    const InstrumentPanel& ip = g->ac->instruments();
    const CockpitState& c = ip.state();
    const InstrumentTruth& t = ip.truth();
    auto hdg = [](Real r) { return std::fmod(radToDeg(r) + 360.0, 360.0); };
    auto f = [](Real k) { return k > 0 ? (k - 273.15) * 9.0 / 5.0 + 32.0 : 0.0; };
    double* o = g_cockpit;
    o[0] = msToKt(c.indicatedAirspeed); o[1] = mToFt(c.indicatedAltitude); o[2] = paToInHg(c.altimeterSetting);
    o[3] = msToFpm(c.verticalSpeed); o[4] = radToDeg(c.attitudePitch); o[5] = radToDeg(c.attitudeRoll);
    o[6] = c.attitudeTumbled; o[7] = hdg(c.headingIndicator); o[8] = hdg(c.headingBug);
    o[9] = radToDeg(c.turnCoordinatorDeflection); o[10] = radToDeg(c.turnRate); o[11] = c.turnCoordinatorFlag;
    o[12] = radToDeg(c.slipBall); o[13] = hdg(c.magneticHeading); o[14] = c.tachometerRpm;
    o[15] = c.oilPressure / 6894.757; o[16] = f(c.oilTemperature); o[17] = f(c.egt);
    o[18] = m3ToGal(c.fuelFlow / g->def.fuel.density) * 3600; o[19] = m3ToGal(c.fuelGauge[0]); o[20] = m3ToGal(c.fuelGauge[1]);
    o[21] = c.ammeter; o[22] = c.voltmeter; o[23] = paToInHg(c.suction); o[24] = c.annunciators; o[25] = c.stallHorn;
    o[26] = kToCelsius(c.outsideAirTemperature); o[27] = c.clock; o[28] = radToDeg(c.flapIndicator);
    o[29] = c.trimIndicator; o[30] = c.panelLights; o[31] = c.panelLightDimmer;
    o[32] = c.navActiveKhz[0] / 1000; o[33] = c.navStandbyKhz[0] / 1000; o[34] = c.comActiveKhz[0] / 1000;
    o[35] = c.comStandbyKhz[0] / 1000; o[36] = hdg(c.obs); o[37] = c.nav1.valid; o[38] = c.nav1.cdi;
    o[39] = c.nav1.toFrom; o[40] = c.nav1.glideslopeValid; o[41] = c.nav1.glideslope; o[42] = c.nav1.localizer;
    o[43] = c.avionicsOn; o[44] = c.squawk; o[45] = static_cast<int>(c.transponderMode);
    o[46] = c.reportedAltitudeFt; o[47] = c.altitudeReported;
    o[48] = msToKt(t.trueAirspeed); o[49] = msToKt(t.calibratedAirspeed); o[50] = msToKt(t.groundSpeed);
    o[51] = mToFt(t.altitudeMsl); o[52] = mToFt(t.pressureAltitude); o[53] = msToFpm(t.verticalSpeed);
    o[54] = radToDeg(t.pitch); o[55] = radToDeg(t.roll); o[56] = hdg(t.trueHeading); o[57] = hdg(t.magneticHeading);
    o[58] = radToDeg(t.omegaBody.x); o[59] = radToDeg(t.omegaBody.y); o[60] = radToDeg(t.omegaBody.z);
    o[61] = radToDeg(t.slipAngle); o[62] = t.attitudeGyroSpin; o[63] = t.headingGyroSpin; o[64] = t.turnGyroSpin;
    o[65] = msToKt(t.equivalentAirspeed); o[66] = mToFt(t.densityAltitude); o[67] = mToFt(t.heightAboveGround);
    o[68] = radToDeg(g->ac->telemetry().alpha);
    o[69] = ip.faults().pitotBlocked; o[70] = ip.faults().staticBlocked; o[71] = g->ac->vacuum().pumpFailed();
    // Pitot icing (truth): ice as a fraction of what closes the inlet, heater warm-up, cloud water (g/m^3).
    o[72] = ip.truth().pitotIce / g->def.panel.pitotProbe.iceToBlockInlet; o[73] = ip.truth().pitotHeat;
    o[74] = g->weather ? g->weather->sample(g->ac->state().body.position, g->ac->altitudeMsl(), 0).liquidWaterContent : 0.0;
    return o;
}

// Dial markings from the aircraft's panel data (display units).
AVD_EXPORT double* avd_markings() {
    std::fill(std::begin(g_cockpit), std::end(g_cockpit), 0.0);
    if (!g) return g_cockpit;
    const InstrumentPanelDef& p = g->def.panel;
    auto f = [](Real k) { return k > 0 ? (k - 273.15) * 9.0 / 5.0 + 32.0 : 0.0; };
    double* o = g_cockpit;
    o[0] = msToKt(p.airspeed.whiteArc.low); o[1] = msToKt(p.airspeed.whiteArc.high);
    o[2] = msToKt(p.airspeed.greenArc.low); o[3] = msToKt(p.airspeed.greenArc.high);
    o[4] = msToKt(p.airspeed.yellowArc.low); o[5] = msToKt(p.airspeed.yellowArc.high);
    o[6] = msToKt(p.airspeed.redLine); o[7] = msToKt(p.airspeed.dialMin); o[8] = msToKt(p.airspeed.dialMax);
    o[9] = p.engine.tachometerGreen.low; o[10] = p.engine.tachometerGreen.high; o[11] = p.engine.tachometerRedLine;
    o[12] = p.engine.tachometerMax;
    o[13] = p.engine.oilPressureGreen.low / 6894.757; o[14] = p.engine.oilPressureGreen.high / 6894.757;
    o[15] = p.engine.oilPressureRedLow / 6894.757; o[16] = p.engine.oilPressureRedHigh / 6894.757;
    o[17] = f(p.engine.oilTemperatureGreen.low); o[18] = f(p.engine.oilTemperatureGreen.high);
    o[19] = f(p.engine.oilTemperatureRedLine);
    o[20] = paToInHg(p.engine.suctionGreen.low); o[21] = paToInHg(p.engine.suctionGreen.high);
    o[22] = m3ToGal(g->def.fuel.tanks.empty() ? 0 : g->def.fuel.tanks[0].capacity - g->def.fuel.tanks[0].unusable);
    o[23] = msToFpm(p.vsi.dialRange);
    o[24] = p.avionics.navComRadios; o[25] = p.avionics.glideslope; o[26] = p.avionics.transponder;
    o[27] = radToDeg(p.turnCoordinator.standardRateDeflection);
    o[28] = p.engine.exhaustGasTemperature; o[29] = p.engine.fuelFlow; o[30] = p.engine.ammeter;
    return o;
}

AVD_EXPORT void avd_altimeter_adjust(int hundredthsInHg) { if (g) g->ac->instruments().adjustAltimeterSetting(hundredthsInHg); }
AVD_EXPORT void avd_dg_adjust(double deg) { if (g) g->ac->instruments().adjustHeadingIndicator(degToRad(deg)); }
AVD_EXPORT void avd_dg_sync() { if (g) g->ac->instruments().syncHeadingIndicatorToCompass(); }
AVD_EXPORT void avd_heading_bug(double deg) { if (g) g->ac->instruments().setHeadingBug(degToRad(deg)); }
AVD_EXPORT void avd_obs_adjust(double deg) { if (g) g->ac->instruments().adjustObs(degToRad(deg)); }
AVD_EXPORT void avd_nav_tune(int steps50khz) { if (g) g->ac->instruments().tuneNavStandby(0, steps50khz); }
AVD_EXPORT void avd_nav_swap() { if (g) g->ac->instruments().swapNav(0); }
AVD_EXPORT void avd_com_tune(int channels) { if (g) g->ac->instruments().tuneComStandby(0, channels); }
AVD_EXPORT void avd_com_swap() { if (g) g->ac->instruments().swapCom(0); }
AVD_EXPORT void avd_squawk(int code) { if (g) g->ac->instruments().setSquawk(code); }
AVD_EXPORT void avd_xpdr_mode(int m) {
    if (g && m >= 0 && m <= 3) g->ac->instruments().setTransponderMode(static_cast<TransponderMode>(m));
}
AVD_EXPORT void avd_dimmer(double v) { if (g) g->ac->instruments().setPanelLightDimmer(v); }
// Failure injection: 0 pitot (drain open), 1 static port, 2 vacuum pump, 3 alternator.
AVD_EXPORT void avd_failure(int which, int on) {
    if (!g) return;
    auto& f = g->ac->instruments().faults();
    switch (which) {
        case 0: f.pitotBlocked = on != 0; break;
        case 1: f.staticBlocked = on != 0; break;
        case 2: g->ac->vacuum().setPumpFailed(on != 0); break;
        case 3: g->ac->electrical().setAlternatorFailed(on != 0); break;
        default: break;
    }
}
