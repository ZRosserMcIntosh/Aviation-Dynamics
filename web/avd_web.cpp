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
#include "avd/loading.hpp"
#include "avd/procedures.hpp"
#include "avd/units.hpp"
#include "avd/wake.hpp"
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
    // Weather controls; the field is rebuilt from them (rebuildWeather).
    struct {
        double windSpeed = 0, windFromDeg = 120, turbulence = 0, tempDev = 0, humidity = 0.6;
        double cloudBase = 0, cloudTop = 0, cloudLiquidWater = 0;
        int severity = 0;
        double convectiveVelocity = 0, convectiveDepth = 1500;
        int hazard = 0;
        double hazardTime = 0;
    } wx;
    std::unique_ptr<Aircraft> ac;
    Real trimThrottle = 0, trimMixture = 1;  // controls of the last trimmed spawn
    FlightAssist assist{makePreset(AssistPreset::Sport)};
    bool levelHold = false;  // straight-and-level hold, on top of any preset
    ProcedureRunner procedure;
    PilotInput raw;
    Real accumulator = 0;
    Vec3 windAvg;              // the wind at the aircraft, low-passed (3 s) as avionics show it
    bool windAvgValid = false;
    Real runwayWater = 0;      // m of water on the airfield
    std::string events;
    int jetStart = 0;  // jet Quick Start: 0 idle, 1 starting engine 1, 2 engine 2, 3 done
    // Trimmed CLmax cache for the low-speed awareness band and V-speeds.
    std::unique_ptr<AeroModel> clmaxModel;
    Real clmaxKey = -1, clmax = 0, clmaxTakeoff = 0, clmaxLanding = 0;
    // Planned load (kg per station, m^3 per tank), applied to every new spawn.
    std::vector<Real> loadStations, loadTanks;
    // The aircraft's wake: shed into the air, and felt when flown into (its
    // own gates older than kOwnWakeAge s: after a 360, a go-around).
    std::unique_ptr<Wake> wake;
    std::unique_ptr<WakeField> air;  // the weather with the wake in it
    bool wakeOn = true;
};
constexpr Real kOwnWakeAge = 4.0;

Sim* g = nullptr;
double g_state[kStateSize];
double g_cockpit[96];
double g_jet[64];
double g_load[256];
// Airport for spawns: 0 Campo de Marte (SBMT, runway 12), 1 Congonhas (SBSP,
// runway 35L). Kept across aircraft loads.
int g_airportId = 1;

scenarios::Airport makeAirport(int id, const LocalFrame& f) {
    return id == 0 ? scenarios::campoDeMarte(f) : scenarios::congonhas(f);
}

void rebuildWeather();

void buildAirport() {
    g->airport = makeAirport(g_airportId, g->frame);
    g->ground = scenarios::makeAirfieldGround(g->airport, g->frame);
    g->ground->setWaterDepth(g->runwayWater);
    rebuildWeather();  // the wind surroundings and hazards are placed around it
}

// The planned load onto an aircraft (stations and tanks), mass refreshed.
void applyLoad(Aircraft& ac) {
    for (std::size_t i = 0; i < g->loadStations.size(); ++i) ac.setStationMass(static_cast<int>(i), g->loadStations[i]);
    for (std::size_t i = 0; i < g->loadTanks.size(); ++i) ac.fuel().setTankQuantity(static_cast<int>(i), g->loadTanks[i]);
    ac.refreshMass();
}
constexpr int kFlowStrips = 64, kFlowStride = 20, kFlowEngine = 14;
double g_flow[16 + kFlowStrips * kFlowStride + 4 * kFlowEngine];
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

// The São Paulo wind surroundings plus the page's settings and hazard.
// Ambient eddy dissipation rate (m^2/s^3) for the wake's decay, from the
// weather settings: calm air, the surface wind's mechanical turbulence (u* ~
// 0.08 U10, eps = u*^3 / (kappa z) at ~100 m), the clear-air class, the RMS
// intensity (eps ~ sigma^3 / L, L ~ 300 m) and convection (0.4 w*^3 / zi).
Real eddyDissipation() {
    const auto& w = g->wx;
    static constexpr Real kClass[4] = {0, 2e-3, 1.5e-2, 8e-2};
    Real eps = 1e-4 + std::pow(0.08 * w.windSpeed, 3) / (0.4 * 100) + kClass[std::clamp(w.severity, 0, 3)];
    eps += std::pow(w.turbulence, 3) / 300;
    if (w.convectiveVelocity > 0) eps += 0.4 * std::pow(w.convectiveVelocity, 3) / std::max(w.convectiveDepth, 100.0);
    return eps;
}

void rebuildWeather() {
    const auto& w = g->wx;
    SimpleWeather::Params wp;
    scenarios::addSaoPauloWindEnvironment(wp, g->airport);
    wp.windSpeed = w.windSpeed;
    wp.windFromDirection = degToRad(w.windFromDeg);
    wp.turbulence.intensity = w.turbulence;
    wp.turbulence.severity = static_cast<TurbulenceSeverity>(std::clamp(w.severity, 0, 3));
    wp.conditions.temperatureDeviation = w.tempDev;
    wp.conditions.relativeHumidity = w.humidity;
    wp.cloudBase = w.cloudBase;
    wp.cloudTop = w.cloudTop;
    wp.cloudLiquidWater = w.cloudLiquidWater;
    wp.thermals.convectiveVelocity = w.convectiveVelocity;
    wp.thermals.depth = w.convectiveDepth;
    const RunwayDef& rw = g->airport.runways[0];
    if (w.hazard == 1) {
        // Mature microburst 2 km before the runway 12 threshold, on final.
        Microburst m;
        const Vec3 c = scenarios::runwayPoint(rw, -2000);
        m.north = c.x;
        m.east = c.y;
        m.peakOutflow = 12;
        m.startTime = w.hazardTime - 200;
        m.growthTime = 60;
        m.matureTime = 900;
        m.decayTime = 300;
        wp.microbursts.push_back(m);
    } else if (w.hazard == 2) {
        // Thunderstorm gust front from the south-west, over the field in ~5 min.
        WindShift f;
        const Vec3 c = scenarios::runwayPoint(rw, 0.5 * rw.length);
        f.heading = degToRad(45);
        f.north = c.x - 3000 * std::cos(f.heading);
        f.east = c.y - 3000 * std::sin(f.heading);
        f.speed = 10;
        f.startTime = w.hazardTime;
        f.newSpeed = 14;
        f.newFromDirection = degToRad(225);
        f.depth = 1000;
        f.noseUpdraft = 2;
        wp.windShifts.push_back(f);
    }
    g->weatherParams = wp;
    g->weather = std::make_unique<SimpleWeather>(wp);
    if (g->air) g->air->setBase(*g->weather);
    if (g->wake) g->wake->setEddyDissipation(eddyDissipation());
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
    buildAirport();
    for (const PayloadStationDef& s : g->def.stations) g->loadStations.push_back(s.initial);
    for (const FuelTankDef& t : g->def.fuel.tanks) g->loadTanks.push_back(t.initial);
    g->ac = std::make_unique<Aircraft>(g->def, g->frame);
    g->ac->setNavEnvironment(&scenarios::saoPauloNavEnvironment());
    applyLoad(*g->ac);
    return 0;
}

AVD_EXPORT int avd_error_text(int i) {
    return i >= 0 && i < static_cast<int>(g_errors.size()) ? putText(g_errors[static_cast<std::size_t>(i)]) : 0;
}

// Surface wind (m/s, from deg true), extra turbulence RMS (m/s), ISA
// deviation (K), humidity (0..1). The wind brings its own mechanical
// turbulence (rougher over the city than over the airfield).
AVD_EXPORT void avd_set_weather(double windSpeed, double windFromDeg, double turbulence, double tempDev, double humidity) {
    if (!g) return;
    g->wx.windSpeed = windSpeed;
    g->wx.windFromDeg = windFromDeg;
    g->wx.turbulence = turbulence;
    g->wx.tempDev = tempDev;
    g->wx.humidity = humidity;
    rebuildWeather();
}

// One cloud layer (m MSL) with its liquid water content (g/m^3); top <= base
// clears it. Supercooled (an icing layer) where the air is below 0 C.
AVD_EXPORT void avd_set_cloud(double baseMsl, double topMsl, double liquidWater) {
    if (!g) return;
    g->wx.cloudBase = baseMsl;
    g->wx.cloudTop = topMsl;
    g->wx.cloudLiquidWater = liquidWater;
    rebuildWeather();
}

// Turbulence class (0 none, 1 light, 2 moderate, 3 severe; MIL-HDBK-1797)
// and convection: thermals with w* (m/s, 0 none) in a layer zi deep (m AGL).
AVD_EXPORT void avd_set_wind_detail(int severity, double convectiveVelocity, double convectiveDepth) {
    if (!g) return;
    g->wx.severity = severity;
    g->wx.convectiveVelocity = convectiveVelocity;
    g->wx.convectiveDepth = convectiveDepth;
    rebuildWeather();
}

// Wind hazard: 0 none, 1 microburst on final runway 12, 2 gust front from
// the south-west. Placed relative to the current simulation time.
AVD_EXPORT void avd_set_wind_hazard(int kind) {
    if (!g) return;
    g->wx.hazard = kind;
    g->wx.hazardTime = g->ac ? g->ac->telemetry().time : 0.0;
    rebuildWeather();
}

// On the runway in use (avd_set_airport): 0 cold and dark, 1 engine running, 2 on
// the 3 deg final (3 km; jets 6 km), 3 cruising over the city.
namespace {

// A fresh aircraft for a reset.
void newAircraft() {
    g->ac = std::make_unique<Aircraft>(g->def, g->frame);
    // The new aircraft's clock starts at zero: re-anchor a hazard to it.
    if (g->wx.hazard != 0) {
        g->wx.hazardTime = 0;
        rebuildWeather();
    }
    g->ac->setNavEnvironment(&scenarios::saoPauloNavEnvironment());
    applyLoad(*g->ac);
    g->procedure = ProcedureRunner{};
    g->assist.reset();
    g->accumulator = 0;
    g->events.clear();
    g->jetStart = 0;
    g->trimThrottle = 0;
    g->trimMixture = 1;
    g->windAvgValid = false;
    WakeParams wp;
    wp.maxBinsPerSurface = 12;  // the near wake for display and encounters; fewer segments
    wp.eddyDissipation = eddyDissipation();
    g->wake = std::make_unique<Wake>(g->ac->aeroModel(), wp);
    g->air = std::make_unique<WakeField>(*g->weather);
    g->air->add(*g->wake, kOwnWakeAge);
}

// The air the aircraft flies in: the weather, with its wake when that is on.
const AtmosphereField& flightAir() {
    if (g->wakeOn && g->air) return *g->air;
    return *g->weather;
}

// Trimmed airborne spawn: solve the free-air trim in calm air at the spawn
// altitude (throttle solved for the flight path), rotate the trimmed
// north-heading state onto the heading that makes good `track` in the mean
// wind there (crabbed into it), and place it at `posNed` / `altMsl`; the
// air-relative velocity gets the wind. `path` is the flight-path angle over
// the ground (a 3 deg glide path stays 3 deg in a headwind). Hands-off on
// arrival: the trimmed elevator goes into the trim wheel.
void placeTrimmed(Vec3 posNed, Real altMsl, Real track, Real kias, int flapDetent, Real path) {
    g->ac->setEngineRunning(true, 2000);
    // Retractable gear: down on the approach (landing flap), up otherwise.
    g->ac->setGearImmediately(flapDetent >= 2);
    // Trim in still air: no mean wind, turbulence, thermals or hazards (the
    // spawn then gets the wind at its position on top of the trimmed state).
    SimpleWeatherParams calm = g->weatherParams;
    calm.windSpeed = 0;
    calm.turbulence = TurbulenceParams{};
    calm.turbulence.mechanical = false;
    calm.thermals.convectiveVelocity = 0;
    calm.microbursts.clear();
    calm.windShifts.clear();
    calm.gusts.clear();
    const SimpleWeather trimAir(calm);
    scenarios::TrimRequest req;
    req.altitude = altMsl;
    req.flapDetent = flapDetent;
    req.tas = scenarios::tasFromCas(ktToMs(kias), trimAir.atmosphere().at(altMsl));
    // The mean wind the pilot plans with (no turbulence or hazards): crab
    // angle and the air-mass flight path that give `track` and `path` over
    // the ground (wind correction angle asin(crosswind / horizontal TAS)).
    posNed.z = -altMsl;
    SimpleWeatherParams mean = calm;
    mean.windSpeed = g->weatherParams.windSpeed;
    const Vec3 w = SimpleWeather(mean).sample(posNed, altMsl, 0).wind;
    const Real cross = -w.x * std::sin(track) + w.y * std::cos(track);  // from the left: +
    const Real along = w.x * std::cos(track) + w.y * std::sin(track);
    Real heading = track, airPath = path;
    for (int it = 0; it < 4; ++it) {
        const Real vh = req.tas * std::cos(airPath);
        heading = track - std::asin(clamp(cross / vh, -0.9, 0.9));
        const Real gs = vh * std::cos(heading - track) + along;
        airPath = std::asin(clamp((gs * std::tan(path) + w.z) / req.tas, -0.5, 0.5));  // w.z: down
    }
    req.flightPath = airPath;
    const scenarios::TrimResult tr = scenarios::trim(*g->ac, req, trimAir);
    DynamicState s = tr.state;
    const Quat toHeading = Quat::fromEuler(0, 0, heading);
    s.body.attitude = toHeading * s.body.attitude;
    s.body.attitude.normalize();
    s.body.position = posNed;
    s.body.velocity = toHeading.rotate(s.body.velocity) + g->weather->sample(posNed, altMsl, 0).wind;
    g->ac->setState(s);
    g->ac->setCockpitControl("brakes.parking", 0);
    // The air sample and ground planes must describe the new position
    // before the downwash and the instruments are initialised from them.
    g->ac->prepareContext(tr.input, *g->ground, *g->weather);
    g->ac->settleDownwash();
    g->ac->setElevatorTrim(tr.trim);
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
    // Radios: NAV 1 on the (synthetic) localizer of the runway in use, the test
    // VOR in standby; OBS and heading bug on the runway's magnetic course.
    InstrumentPanel& p = g->ac->instruments();
    p.setNavStandby(0, g->airport.ilsKhz ? g->airport.ilsKhz : 113700);
    p.swapNav(0);
    p.setNavStandby(0, 113700);
    const Real course = std::round(radToDeg(g->airport.runways[0].headingTrue - g->airport.magneticVariation));
    p.setObs(degToRad(course));
    p.setHeadingBug(degToRad(course));
    p.setComStandby(0, 132800);
    p.setTransponderMode(TransponderMode::Altitude);
    g->ac->drainEvents();
    if (g->wake) g->wake->reset();  // a new start leaves nothing in the air
}

}  // namespace

// The take-off flap: the largest detent up to 20 deg (the middle one if none
// is), as the PFD's V-speeds and the runway hint assume.
std::size_t takeoffDetent(const std::vector<Real>& det) {
    std::size_t k = 0;
    for (std::size_t i = 1; i < det.size(); ++i)
        if (det[i] <= degToRad(20.5)) k = i;
    return k > 0 ? k : det.size() / 2;
}

// Trimmed maximum lift coefficients (clean is per flight condition; take-off
// and landing flap at low Mach), computed once per aircraft.
void ensureClmaxModel() {
    if (g->clmaxModel) return;
    const std::vector<Real>& det = g->def.controls.flapDetents;
    g->clmaxModel = std::make_unique<AeroModel>(g->def);
    const Vec3 cg0 = g->ac->massProperties().cg;
    g->clmaxTakeoff = trimmedMaxLiftCoefficient(*g->clmaxModel, g->def.referenceArea, cg0, det[takeoffDetent(det)], 0.2);
    g->clmaxLanding = trimmedMaxLiftCoefficient(*g->clmaxModel, g->def.referenceArea, cg0, det.back(), 0.2);
}
Real jetApproachKias() {
    ensureClmaxModel();
    const Real m = g->ac->massProperties().mass;
    const Real vs = std::sqrt(2 * m * 9.80665 / (1.225 * g->def.referenceArea * std::max(g->clmaxLanding, 0.5)));
    return std::clamp(msToKt(1.23 * vs) + 5.0, 90.0, 180.0);
}

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
        // Mode 2: on the 3 deg final for the runway in use (65 KIAS, flaps 20), exactly
        // on the glide path, which starts at the glideslope antenna 300 m beyond
        // the paved end (scenarios/airports.cpp). Mode 3: level cruise, 100 KIAS.
        // Jets: final at 125 KIAS (about Vref + 10) with landing flap and the gear down, cruise at 250 KIAS.
        const bool final = mode == 2;
        const bool jet = g->ac->turbofanCount() > 0;
        const int landingFlap = jet ? static_cast<int>(g->def.controls.flapDetents.size()) - 1 : 2;
        const Real dist = final ? (jet ? -6000.0 : -3000.0) : -8000.0;
        const Real path = final ? degToRad(-3.0) : 0.0;
        const Real alt = g->airport.elevation + (final ? (-dist + 300.0) * std::tan(-path) : (jet ? 1500.0 : 850.0));
        // Jets fly the final at their own approach speed: Vref (1.23 times the
        // landing-flap stall speed at this weight) + 5 kt.
        const Real kias = final ? (jet ? jetApproachKias() : 65.0) : (jet ? 250.0 : 100.0);
        placeTrimmed(scenarios::runwayPoint(rw, dist), alt, rw.headingTrue, kias, final ? landingFlap : 0, path);
    }
    finishReset(mode >= 1);
}

// Trimmed, level, hands-off spawn anywhere in the world: world ENU position
// (m east and north of the world anchor), height above the field elevation
// (m), true track (deg; the spawn crabs into the mean wind to make it good),
// indicated airspeed (kt) and flap detent.
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
// Strips from 16, 20 doubles each: quarter-chord position xyz, span axis xyz,
// normal (lift side) axis xyz, chord, area, cl, alpha, q, stall alpha, surface,
// attached fraction, circulation, zero-lift alpha (with the flaps), cd.
// Engines after the strips, 14 doubles each: position xyz, thrust axis xyz,
// radius, slipstream / jet velocity increment (m/s), thrust (N), kind (0
// propeller, 1 turbofan), propeller rate rad/s (signed: right-handed about the
// thrust axis), blades, swirl m/s at 0.7 R, hub radius fraction (turbofans:
// reverser position, efflux speed m/s, cowl radius, cascades forward of the
// nozzle m). Structural frame: x forward, y right, z down.
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
    const Vec3 w = ac.state().body.omega;  // body rates p, q, r (rad/s)
    g_flow[10] = w.x; g_flow[11] = w.y; g_flow[12] = w.z;
    for (int i = 0; i < n; ++i) {
        const AeroModel::StripGeometry sg = am.stripGeometry(i);
        const StripTelemetry& t = ao.strips[static_cast<std::size_t>(i)];
        double* o = g_flow + 16 + i * kFlowStride;
        o[0] = sg.position.x; o[1] = sg.position.y; o[2] = sg.position.z;
        o[3] = sg.spanAxis.x; o[4] = sg.spanAxis.y; o[5] = sg.spanAxis.z;
        o[6] = sg.normalAxis.x; o[7] = sg.normalAxis.y; o[8] = sg.normalAxis.z;
        o[9] = sg.chord; o[10] = sg.area; o[11] = t.cl; o[12] = t.alpha; o[13] = t.q; o[14] = t.stallAlpha;
        o[15] = sg.surface;
        o[16] = t.attached; o[17] = t.gamma; o[18] = t.zeroLiftAlpha; o[19] = t.cd;
    }
    double* e = g_flow + 16 + kFlowStrips * kFlowStride;
    int ne = 0;
    for (int i = 0; i < ac.engineCount() && ne < 4; ++i, ++ne) {
        const SlipstreamState& ss = ac.slipstream(i);
        double* o = e + ne * kFlowEngine;
        const EngineTelemetry& et = ac.telemetry().engines[static_cast<std::size_t>(i)];
        const PropellerDef& pd = g->def.engines[static_cast<std::size_t>(i)].propeller;
        o[0] = ss.discCentre.x; o[1] = ss.discCentre.y; o[2] = ss.discCentre.z;
        o[3] = ss.axis.x; o[4] = ss.axis.y; o[5] = ss.axis.z;
        o[6] = ss.radius; o[7] = ss.axialIncrement; o[8] = et.prop.thrust; o[9] = 0;
        o[10] = et.rpm * kTwoPi / 60 * (ss.clockwiseFromBehind ? 1.0 : -1.0);
        o[11] = pd.blades; o[12] = ss.swirlVelocity; o[13] = pd.hubRadiusFraction;
    }
    for (int i = 0; i < ac.turbofanCount() && ne < 4; ++i, ++ne) {
        const TurbofanDef& d = g->def.turbofans[static_cast<std::size_t>(i)];
        const TurbofanState& ts = ac.turbofanState(i);
        double* o = e + ne * kFlowEngine;
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
        // Reverser: position, efflux speed through the cascades (the fan
        // stream, about the jet's speed relative to the nacelle), cowl radius
        // and the cascades' distance forward of the nozzle.
        if (d.reverser.present) {
            o[10] = ac.reverserPosition(i); o[11] = vj;
            o[12] = d.reverser.cowlRadius; o[13] = d.reverser.cascadeForward;
        }
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
// [30], [31] oil pressure psi (engines 1, 2); ... [52..55] thrust reversers
// (position, phase) engines 1, 2, [56] ground spoilers out, [57] reversers fitted.
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
    ensureClmaxModel();
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
    g_jet[49] = det.size() > 2 ? radToDeg(det[takeoffDetent(det)]) : 0.0;
    g_jet[50] = radToDeg(det.back());
    g_jet[51] = t.stickPusher;
    // Thrust reversers (engines 1, 2: position 0..1, phase 0 stowed, 1 reverse
    // thrust, 2 reverse idle, 3 stowing) and the ground spoilers.
    for (int i = 0; i < std::min(ac.turbofanCount(), 2); ++i) {
        g_jet[52 + 2 * i] = ac.reverserPosition(i);
        g_jet[53 + 2 * i] = ac.reverserPhase(i);
    }
    g_jet[56] = ac.groundSpoilersActive();
    g_jet[57] = g->def.turbofans.empty() ? 0.0 : g->def.turbofans[0].reverser.present;
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
        g->ac->step(kDt, in, *g->ground, flightAir());
        if (g->wakeOn) g->wake->update(*g->ac, *g->weather, g->ground.get());
        g->accumulator -= kDt;
        ++n;
        const Vec3 w = g->ac->telemetry().air.wind;
        if (!g->windAvgValid) g->windAvg = w;
        g->windAvg = g->windAvg + (w - g->windAvg) * (kDt / 3.0);
        g->windAvgValid = true;
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

// Runway in use, world frame: [east, north, elevation, heading_true_deg, length, width]
// (east/north of its paved start). Names: avd_airport_text().
// Choose the spawn airport (0 SBMT, 1 SBSP); takes effect for the next reset.
AVD_EXPORT void avd_set_airport(int id) {
    g_airportId = id == 0 ? 0 : 1;
    if (g) buildAirport();
}

// "ICAO|name|runway in use|reciprocal" in the text buffer.
AVD_EXPORT int avd_airport_text() {
    if (!g) return 0;
    const scenarios::Airport& a = g->airport;
    return putText(a.icao + "|" + a.name + "|" + a.runways[0].ident + "|" + (a.runways.size() > 1 ? a.runways[1].ident : ""));
}

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
    s[58] = g->assist.report().underspeed;
    // Wind (smoothed), track and drift, as a navigation display shows them.
    const Vec3 w = g->windAvgValid ? g->windAvg : t.air.wind;
    const Real ws = std::hypot(w.x, w.y);
    s[59] = ws > 0.05 ? std::fmod(radToDeg(std::atan2(-w.y, -w.x)) + 720.0, 360.0) : 0.0;  // from, true
    s[60] = msToKt(ws);
    const Vec3 v = ac.state().body.velocity;
    const Real track = std::hypot(v.x, v.y) > 1.0 ? std::atan2(v.y, v.x) : t.heading;
    s[61] = std::fmod(radToDeg(track) + 360.0, 360.0);
    s[62] = radToDeg(wrapPi(track - t.heading));  // drift, + right
    s[63] = g->runwayWater * 1000;
    return s;
}

// ---- Runway condition and the wind around the aircraft --------------------
// Water on the airfield (mm): 0 dry, under 3 wet, 3 and more standing water.
AVD_EXPORT void avd_set_runway_water(double mm) {
    if (!g) return;
    g->runwayWater = std::clamp(std::isfinite(mm) ? mm : 0.0, 0.0, 50.0) * 1e-3;
    g->ground->setWaterDepth(g->runwayWater);
}

double g_windGrid[3 * 16 * 12 * 8];
// The wind field around the aircraft for the airflow view: a grid of nx * ny
// * nz points (at most 16 x 12 x 8) spanning +-hx, +-hy, +-hz metres about the
// CG along the body axes (x forward, y right, z down), x fastest. Each entry
// is the wind there minus the wind at the CG, in body axes (m/s): the eddies,
// thermals and shear the free stream at the CG does not carry.
AVD_EXPORT double* avd_wind_grid(double hx, double hy, double hz, int nx, int ny, int nz) {
    std::fill(std::begin(g_windGrid), std::end(g_windGrid), 0.0);
    if (!g) return g_windGrid;
    nx = std::clamp(nx, 2, 16), ny = std::clamp(ny, 2, 12), nz = std::clamp(nz, 2, 8);
    const Aircraft& ac = *g->ac;
    const Quat& q = ac.state().body.attitude;
    const Vec3 c = ac.state().body.position;
    const Real alt = ac.altitudeMsl(), t = ac.time();
    // Each value is averaged over the grid spacing: eddies smaller than the
    // grid would alias into kinks between the nodes.
    const Real extent = std::min({2 * hx / (nx - 1), 2 * hy / (ny - 1), 2 * hz / (nz - 1)});
    const Vec3 w0 = g->weather->sample(c, alt, t, extent).wind;
    double* o = g_windGrid;
    for (int k = 0; k < nz; ++k)
        for (int j = 0; j < ny; ++j)
            for (int i = 0; i < nx; ++i) {
                const Vec3 b{hx * (2.0 * i / (nx - 1) - 1), hy * (2.0 * j / (ny - 1) - 1), hz * (2.0 * k / (nz - 1) - 1)};
                Vec3 p = c + q.rotate(b);
                // Below the (flat) airfield: the air just above the surface.
                const Real lift = g->airport.elevation + 1 - (alt - (p.z - c.z));
                if (lift > 0) p.z -= lift;
                const Vec3 d = q.inverseRotate(g->weather->sample(p, alt - (p.z - c.z), t, extent).wind - w0);
                *o++ = d.x;
                *o++ = d.y;
                *o++ = d.z;
            }
    return g_windGrid;
}

// ---- Wake ------------------------------------------------------------------
// Shed and felt (on, the default) or neither (off).
AVD_EXPORT void avd_set_wake(int on) {
    if (!g) return;
    g->wakeOn = on != 0;
    if (!g->wakeOn && g->wake) g->wake->reset();
}

// The wake's vortex segments (see web/README.md): [0] count, [1] the largest
// |circulation| (m^2/s), [2] ground height (ENU up, m; -1e9 without), then
// kWakeStride doubles per segment: end a (ENU, m), end b, circulation (m^2/s,
// right-handed about a -> b), Lamb-Oseen core radius (m), age (s). Bound,
// attached, shed and ground-image segments; at most kWakeMax.
constexpr int kWakeMax = 4096, kWakeStride = 9, kWakeHeader = 8;
double g_wake[kWakeHeader + kWakeMax * kWakeStride];
std::vector<WakeSegment> g_wakeSegments;
AVD_EXPORT double* avd_wake() {
    g_wake[0] = g_wake[1] = 0;
    g_wake[2] = -1e9;
    if (!g || !g->wake || !g->wakeOn) return g_wake;
    g->wake->segments(g_wakeSegments);
    const GroundHit h = g->ground->below(g->ac->state().body.position);
    if (h.hit) g_wake[2] = -h.point.z;
    int n = 0;
    Real gmax = 0;
    for (const WakeSegment& w : g_wakeSegments) {
        if (n >= kWakeMax) break;
        const Vec3 a = world::enuFromNed(w.a), b = world::enuFromNed(w.b);
        double* o = g_wake + kWakeHeader + n * kWakeStride;
        o[0] = a.x; o[1] = a.y; o[2] = a.z;
        o[3] = b.x; o[4] = b.y; o[5] = b.z;
        o[6] = w.gamma; o[7] = w.core; o[8] = w.age;
        gmax = std::max(gmax, std::abs(w.gamma));
        ++n;
    }
    g_wake[0] = n;
    g_wake[1] = gmax;
    return g_wake;
}

// The wind (ENU, m/s, the air's own velocity) on an nx x ny x nz grid over
// the box centre (cE, cN, cU) +- (hE, hN, hU), ENU metres, x fastest. Each
// value is averaged over the grid spacing (smaller eddies would alias).
double g_windBox[3 * 24 * 24 * 12];
AVD_EXPORT double* avd_wind_box(double cE, double cN, double cU, double hE, double hN, double hU, int nx, int ny, int nz) {
    std::fill(std::begin(g_windBox), std::end(g_windBox), 0.0);
    if (!g) return g_windBox;
    nx = std::clamp(nx, 2, 24), ny = std::clamp(ny, 2, 24), nz = std::clamp(nz, 2, 12);
    const Real t = g->ac->time();
    // MSL altitude of NED z = 0.
    const Real msl0 = g->ac->altitudeMsl() + g->ac->state().body.position.z;
    const Real extent = std::min({2 * hE / (nx - 1), 2 * hN / (ny - 1), 2 * hU / (nz - 1)});
    double* o = g_windBox;
    for (int k = 0; k < nz; ++k)
        for (int j = 0; j < ny; ++j)
            for (int i = 0; i < nx; ++i) {
                const Vec3 enu{cE + hE * (2.0 * i / (nx - 1) - 1), cN + hN * (2.0 * j / (ny - 1) - 1), cU + hU * (2.0 * k / (nz - 1) - 1)};
                Vec3 p = world::nedFromEnu(enu);
                const Real lift = g->airport.elevation + 1 - (msl0 - p.z);
                if (lift > 0) p.z -= lift;  // below the airfield: the air just above it
                const Vec3 w = world::enuFromNed(g->weather->sample(p, msl0 - p.z, t, extent).wind);
                *o++ = w.x;
                *o++ = w.y;
                *o++ = w.z;
            }
    return g_windBox;
}

// Bluff bodies in the airflow (airflow view): extended retractable gear legs,
// deployed spoiler panels, and the drag items given a shape (wing struts,
// fixed gear and fairings). [0] count, then 10 doubles a body: position
// (structural, m), axis (the body's long axis), size (diameter or projected
// height, m), length along the axis (m), drag area (m^2), kind (0 gear,
// 1 spoiler, 2 strut or other fixed item).
double g_bodies[1 + 32 * 10];
AVD_EXPORT double* avd_bodies() {
    std::fill(std::begin(g_bodies), std::end(g_bodies), 0.0);
    if (!g) return g_bodies;
    const Aircraft& ac = *g->ac;
    int n = 0;
    auto put = [&](const Vec3& p, const Vec3& a, Real size, Real length, Real cda, int kind) {
        if (n >= 32) return;
        double* o = g_bodies + 1 + n * 10;
        o[0] = p.x; o[1] = p.y; o[2] = p.z; o[3] = a.x; o[4] = a.y; o[5] = a.z;
        o[6] = size; o[7] = length; o[8] = cda; o[9] = kind;
        ++n;
    };
    const GearRetractionDef& gc = g->def.controls.gear;
    const Real down = gc.retractable ? ac.gearPosition() : 1.0;
    if (gc.retractable && down > 0.05 && !g->def.gear.empty()) {
        // Retractable: its drag area shared over the legs.
        for (const GearDef& gd : g->def.gear) {
            const Real d = 2 * gd.wheelRadius;
            put(gd.attach + Vec3{0, 0, 0.6 * gd.length * down}, {0, 0, 1}, d, gd.length * down, gc.dragArea / static_cast<Real>(g->def.gear.size()) * down, 0);
        }
    }
    // Drag items with a shape (fixed gear is one of these).
    for (const DragItemDef& di : g->def.dragItems) {
        if (di.bodySize <= 0 || di.bodyLength <= 0) continue;
        const bool gear = di.name.find("gear") != std::string::npos || di.name.find("wheel") != std::string::npos;
        put(di.position, normalized(di.bodyAxis), di.bodySize, di.bodyLength, di.dragArea, gear ? 0 : 2);
    }
    std::array<AeroModel::BluffBody, 24> sp{};
    const int ns = ac.aeroModel().spoilerBodies(ac.speedbrakePosition(), sp.data(), 24);
    for (int i = 0; i < ns; ++i) put(sp[static_cast<std::size_t>(i)].position, sp[static_cast<std::size_t>(i)].axis, sp[static_cast<std::size_t>(i)].size, sp[static_cast<std::size_t>(i)].length, sp[static_cast<std::size_t>(i)].dragArea, 1);
    g_bodies[0] = n;
    return g_bodies;
}

// ---- Weight and balance ---------------------------------------------------
// The planned load is kept across resets and applied to every new spawn; the
// host restarts to fly it. Placards and limits are reported, never enforced.
AVD_EXPORT void avd_set_station(int i, double kg) {
    if (g && i >= 0 && i < static_cast<int>(g->loadStations.size()) && std::isfinite(kg)) g->loadStations[static_cast<std::size_t>(i)] = std::max(kg, 0.0);
}
// Litres in tank i (clamped to the tank's capacity: it cannot hold more).
AVD_EXPORT void avd_set_tank(int i, double litres) {
    if (g && i >= 0 && i < static_cast<int>(g->loadTanks.size()) && std::isfinite(litres))
        g->loadTanks[static_cast<std::size_t>(i)] = std::clamp(litres * 1e-3, 0.0, g->def.fuel.tanks[static_cast<std::size_t>(i)].capacity);
}

// Load sheet (see web/README.md for the layout): planned != 0 for the planned
// load (what the next spawn carries), 0 for the aircraft as it is now.
AVD_EXPORT double* avd_loadsheet(int planned) {
    std::fill(std::begin(g_load), std::end(g_load), 0.0);
    if (!g) return g_load;
    std::unique_ptr<Aircraft> tmp;
    if (planned) {
        tmp = std::make_unique<Aircraft>(g->def, g->frame);
        applyLoad(*tmp);
    }
    const Aircraft& ac = planned ? *tmp : *g->ac;
    const AircraftDefinition& d = g->def;
    const LoadSheet ls = computeLoadSheet(ac);
    double* o = g_load;
    o[0] = ls.basicEmptyMass; o[1] = ls.basicEmptyCg.x; o[2] = ls.payload; o[3] = ls.zeroFuelMass; o[4] = ls.zeroFuelCg.x;
    o[5] = ls.fuelTotal; o[6] = ls.fuelUsable; o[7] = ls.grossMass; o[8] = ls.cg.x;
    o[9] = ls.cgForwardLimit; o[10] = ls.cgAftLimit; o[11] = ls.flags;
    o[12] = d.maxRampMass; o[13] = d.maxTakeoffMass; o[14] = d.maxLandingMass; o[15] = d.maxZeroFuelMass;
    o[16] = d.fuel.density * 1e-3;
    const int ns = static_cast<int>(d.stations.size()), nt = static_cast<int>(d.fuel.tanks.size());
    const int ng = static_cast<int>(d.stationGroupLimits.size());
    const int nf = static_cast<int>(d.cgForwardLimitVsMass.xs().size()), na = static_cast<int>(d.cgAftLimitVsMass.xs().size());
    o[17] = ns; o[18] = nt; o[19] = ng; o[20] = nf; o[21] = na;
    o[22] = ls.fuelTotal - ls.fuelUsable;
    int k = 24;
    auto put = [&](double v) { if (k < 256) o[k++] = v; };
    for (int i = 0; i < ns; ++i) {
        const PayloadStationDef& st = d.stations[static_cast<std::size_t>(i)];
        put(ac.stationMass(i)); put(st.maxMass); put(st.position.x);
    }
    for (int i = 0; i < nt; ++i) {
        const FuelTankDef& t = d.fuel.tanks[static_cast<std::size_t>(i)];
        put(ac.fuel().tankQuantity(i) * 1e3); put(t.capacity * 1e3); put(t.unusable * 1e3); put(t.position.x);
    }
    for (int i = 0; i < ng; ++i) {
        const StationGroupLimitDef& gr = d.stationGroupLimits[static_cast<std::size_t>(i)];
        Real m = 0;
        for (int s : gr.stations) m += ac.stationMass(s);
        put(m); put(gr.maxMass);
    }
    for (int i = 0; i < nf; ++i) { put(d.cgForwardLimitVsMass.xs()[static_cast<std::size_t>(i)]); put(d.cgForwardLimitVsMass.ys()[static_cast<std::size_t>(i)]); }
    for (int i = 0; i < na; ++i) { put(d.cgAftLimitVsMass.xs()[static_cast<std::size_t>(i)]); put(d.cgAftLimitVsMass.ys()[static_cast<std::size_t>(i)]); }
    return o;
}

// Names for the load sheet in the text buffer: "station;station|tank;tank|group;group".
AVD_EXPORT int avd_loadsheet_names() {
    if (!g) return 0;
    std::string s;
    for (std::size_t i = 0; i < g->def.stations.size(); ++i) s += (i ? ";" : "") + g->def.stations[i].name;
    s += "|";
    for (std::size_t i = 0; i < g->def.fuel.tanks.size(); ++i) s += (i ? ";" : "") + g->def.fuel.tanks[i].name;
    s += "|";
    for (std::size_t i = 0; i < g->def.stationGroupLimits.size(); ++i) s += (i ? ";" : "") + g->def.stationGroupLimits[i].name;
    return putText(s);
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
