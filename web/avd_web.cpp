// Aviation Dynamics — WebAssembly bindings for the browser flight demo.
// SPDX-License-Identifier: MIT
//
// A thin C ABI over avd::Aircraft so a web page can fly the real core. The page
// only renders and reads input. World positions are returned in the São Paulo
// World frame (ENU metres from MASP); see web/README.md for the buffer layout.
#include <cstring>
#include <memory>
#include <string>

#include "airports.hpp"
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
    std::unique_ptr<Aircraft> ac;
    FlightAssist assist{makePreset(AssistPreset::Sport)};
    ProcedureRunner procedure;
    PilotInput raw;
    Real accumulator = 0;
    std::string events;
};

Sim* g = nullptr;
double g_state[kStateSize];
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
        case FlightEventType::Overspeed: return "OVERSPEED: above VNE";
        case FlightEventType::OverG: return "LIMIT LOAD EXCEEDED";
        case FlightEventType::FlapOverspeed: return "Flaps above VFE";
        case FlightEventType::PropStrike: return "PROPELLER STRIKE";
        case FlightEventType::HardPointContact: return "Airframe ground contact";
        case FlightEventType::BatteryDepleted: return "Battery depleted";
        case FlightEventType::AlternatorOffline: return "Alternator offline";
        case FlightEventType::AlternatorOnline: return "Alternator online";
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
    g->weather = std::make_unique<SimpleWeather>(wp);
}

// 0 cold and dark on runway 12, 1 engine running on runway 12, 2 on final
// for runway 12 (3 km), 3 cruising at 3500 ft over the city.
AVD_EXPORT void avd_reset(int mode) {
    if (!g) return;
    g->ac = std::make_unique<Aircraft>(g->def, g->frame);
    g->procedure = ProcedureRunner{};
    g->accumulator = 0;
    g->events.clear();
    const RunwayDef& rw = g->airport.runways[0];
    if (mode <= 1) {
        const Vec3 p = scenarios::runwayPoint(rw, 30);
        if (mode == 1) g->ac->setEngineRunning(true, 1000);
        g->ac->placeOnGround(p.x, p.y, rw.headingTrue, *g->ground, *g->weather);
        if (mode == 0) g->ac->setColdAndDark();
    } else {
        const bool final = mode == 2;
        g->ac->setEngineRunning(true, 2000);
        if (final) g->ac->setFlapsImmediately(2);
        const Real dist = final ? -3000.0 : -8000.0;
        Vec3 p = scenarios::runwayPoint(rw, dist);
        p.z = -(g->airport.elevation + (final ? 3000 * std::tan(degToRad(3.5)) : 850.0));
        const Real v = ktToMs(final ? 70.0 : 100.0);
        RigidBodyState s;
        s.position = p;
        s.velocity = {v * std::cos(rw.headingTrue), v * std::sin(rw.headingTrue), final ? v * std::tan(degToRad(3.5)) : 0.0};
        s.attitude = Quat::fromEuler(0, final ? 0.0 : degToRad(2), rw.headingTrue);
        g->ac->setFlightState(s);
        g->ac->setCockpitControl("brakes.parking", 0);
        g->ac->settleDownwash();
    }
    g->ac->instruments().setAltimeterSetting(g->weather->atmosphere().conditions().seaLevelPressure);
    g->ac->drainEvents();
}

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
    g->assist.setConfig(makePreset(p));
}

AVD_EXPORT void avd_quick_start() {
    if (g) g->procedure.start(quickStartPistonSingle(*g->ac));
}

// Procedure status: 0 idle, 1 running, 2 done, 3 failed; message in the text buffer.
AVD_EXPORT int avd_procedure_status() {
    if (!g) return 0;
    putText(g->procedure.message());
    return static_cast<int>(g->procedure.status());
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
    const InstrumentReadings& r = ac.instruments().readings();
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
    s[30] = e.engine.running; s[31] = r.voltmeter; s[32] = ac.electrical().alternatorOnline(); s[33] = paToInHg(ac.vacuum().suction());
    s[34] = ac.electrical().batteryCharge();
    s[35] = radToDeg(r.attitudeRoll); s[36] = radToDeg(r.attitudePitch); s[37] = std::fmod(radToDeg(r.headingIndicator) + 360.0, 360.0);
    s[38] = radToDeg(r.turnRate); s[39] = r.slipBall;
    s[40] = t.mass; s[41] = t.cg.x; s[42] = t.time; s[43] = t.stalledStrips; s[44] = r.oilPressure / 6894.76;
    s[45] = t.controls.aileron; s[46] = t.controls.elevator; s[47] = t.controls.rudder; s[48] = ac.faulted();
    s[49] = ac.parkingBrake(); s[50] = e.prop.thrust; s[51] = t.mach; s[52] = ac.fuel().totalQuantity() * 1000;
    s[53] = mToFt(t.altitudeMsl); s[54] = g->assist.report().active();
    return s;
}
