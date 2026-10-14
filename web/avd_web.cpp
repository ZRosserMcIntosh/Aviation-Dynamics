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
    ProcedureRunner procedure;
    PilotInput raw;
    Real accumulator = 0;
    std::string events;
};

Sim* g = nullptr;
double g_state[kStateSize];
double g_cockpit[96];
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
    g->weatherParams = wp;
    g->weather = std::make_unique<SimpleWeather>(wp);
}

// 0 cold and dark on runway 12, 1 engine running on runway 12, 2 on final
// for runway 12 (3 km), 3 cruising at 3500 ft over the city.
AVD_EXPORT void avd_reset(int mode) {
    if (!g) return;
    g->ac = std::make_unique<Aircraft>(g->def, g->frame);
    g->ac->setNavEnvironment(&scenarios::saoPauloNavEnvironment());
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
        // Trimmed spawns: solve the free-air trim (calm air at the spawn
        // altitude), then place the trimmed state on the 3 deg final for
        // runway 12 (65 KIAS, flaps 20, throttle solved) or in level cruise
        // over the city (100 KIAS, throttle solved). Hands-off on arrival.
        const bool final = mode == 2;
        g->ac->setEngineRunning(true, 2000);
        const Real dist = final ? -3000.0 : -8000.0;
        const Real path = final ? degToRad(-3.0) : 0.0;
        Vec3 p = scenarios::runwayPoint(rw, dist);
        // On final: exactly on the glide path, which starts at the glideslope
        // antenna 300 m beyond the paved end (scenarios/airports.cpp).
        const Real alt = g->airport.elevation + (final ? (-dist + 300.0) * std::tan(-path) : 850.0);
        SimpleWeatherParams calm = g->weatherParams;
        calm.windSpeed = 0;
        calm.turbulence.intensity = 0;
        const SimpleWeather trimAir(calm);
        scenarios::TrimRequest req;
        req.altitude = alt;
        req.flapDetent = final ? 2 : 0;
        req.tas = scenarios::tasFromCas(ktToMs(final ? 65.0 : 100.0), trimAir.atmosphere().at(alt).density);
        req.flightPath = path;
        const scenarios::TrimResult tr = scenarios::trim(*g->ac, req, trimAir);
        // Rotate the trimmed (north-heading) state onto the runway heading and
        // move it to the spawn point; the air-relative velocity gets the wind.
        DynamicState s = tr.state;
        const Quat toHeading = Quat::fromEuler(0, 0, rw.headingTrue);
        s.body.attitude = toHeading * s.body.attitude;
        s.body.attitude.normalize();
        p.z = -alt;
        s.body.position = p;
        s.body.velocity = toHeading.rotate(s.body.velocity) + g->weather->sample(p, alt, 0).wind;
        g->ac->setState(s);
        g->ac->setCockpitControl("brakes.parking", 0);
        // The air sample and ground planes must describe the new position
        // before the downwash and the instruments are initialised from them.
        g->ac->prepareContext(tr.input, *g->ground, *g->weather);
        g->ac->settleDownwash();
        // The trimmed elevator goes into the trim wheel: neutral stick holds it.
        Real elevUp = 0.4;
        for (const auto& sd : g->def.surfaces)
            for (const auto& pd : sd.panels)
                for (const auto& c : pd.controls)
                    if (c.channel == ControlChannel::Elevator) elevUp = c.maxNegative;
        g->ac->setElevatorTrim(tr.elevator * elevUp / g->def.controls.elevatorTrimRange);
        g->trimThrottle = tr.input.throttle;
        g->trimMixture = tr.input.mixture;
    }
    g->ac->instruments().setAltimeterSetting(g->weather->atmosphere().conditions().seaLevelPressure);
    if (mode >= 1) g->ac->resetInstrumentsToTruth();
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

// Throttle and mixture of the last trimmed spawn (modes 2 and 3), so the
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
