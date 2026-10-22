// Aviation Dynamics — scenarios: golden-trace regression scenarios.
// SPDX-License-Identifier: MIT
#include "golden.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <istream>
#include <ostream>
#include <sstream>
#include <stdexcept>

#include "airports.hpp"
#include "approach_pilot.hpp"
#include "avd/aero.hpp"
#include "avd/units.hpp"
#include "scenarios.hpp"

namespace avd::scenarios::golden {

using namespace units;

// ------------------------------------------------------------- channels ----

const std::array<Channel, kChannels>& channels() {
    // Tolerances: tight enough that a physics change shows, loose enough for
    // another compiler's last-bit differences over a few minutes of flight.
    static const std::array<Channel, kChannels> c = {{
        {"t_s", 1e-6, 0},            // 0
        {"north_m", 10.0, 0},        // 1  position NED
        {"east_m", 10.0, 0},         // 2
        {"down_m", 3.048, 0},        // 3  (10 ft)
        {"tas_kt", 0.5, 0},          // 4
        {"cas_kt", 0.5, 0},          // 5
        {"gs_kt", 0.5, 0},           // 6
        {"alt_msl_ft", 10.0, 0},     // 7
        {"vs_fpm", 30.0, 0},         // 8
        {"alpha_deg", 0.3, 0},       // 9
        {"beta_deg", 0.3, 0},        // 10
        {"roll_deg", 0.3, 0, true},  // 11
        {"pitch_deg", 0.3, 0},       // 12
        {"heading_deg", 0.3, 0, true},  // 13
        {"p_dps", 0.5, 0},           // 14 body rates
        {"q_dps", 0.5, 0},           // 15
        {"r_dps", 0.5, 0},           // 16
        {"nz", 0.02, 0},             // 17 load factor
        {"elevator", 0.02, 0},       // 18 control deflections (as the aero model sees them)
        {"aileron", 0.02, 0},        // 19
        {"rudder", 0.02, 0},         // 20
        {"throttle", 0.02, 0},       // 21 the pilot's lever
        {"thrust1_n", 5.0, 0.02},    // 22 per engine: 2 % of the value (+5 N)
        {"thrust2_n", 5.0, 0.02},    // 23
        {"n1_1_pct", 1.0, 0},        // 24 turbofan N1 %; piston: rpm as % of the rated rpm
        {"n1_2_pct", 1.0, 0},        // 25
        {"fuel_kg", 0.5, 0},         // 26
        {"gear", 0.01, 0},           // 27 0 up .. 1 down
        {"flap_deg", 0.3, 0},        // 28
        {"on_ground", 0.5, 0},       // 29
    }};
    return c;
}

namespace {

constexpr Real kDt = kDefaultDt;
constexpr int kStepsPerSample = 24;  // 240 Hz -> 10 Hz

// Calm, still air over the field: no wind, no turbulence of any kind, so a
// run is a pure function of the physics and the script.
SimpleWeather::Params calmAt(Real elevation) {
    SimpleWeather::Params wp;
    wp.groundElevation = elevation;
    wp.windSpeed = 0;
    wp.turbulence = TurbulenceParams{};
    wp.turbulence.intensity = 0;
    wp.turbulence.mechanical = false;
    return wp;
}

class Recorder {
public:
    explicit Recorder(const std::string& name) { trace_.scenario = name; }
    // Call once per step after ac.step(); samples every kStepsPerSample.
    void step(const Aircraft& ac, const PilotInput& in) {
        if (++count_ % kStepsPerSample != 0) return;
        const AircraftTelemetry& t = ac.telemetry();
        const InstrumentTruth& tr = ac.instruments().truth();
        Sample s{};
        s[0] = t.time;
        s[1] = t.positionNed.x;
        s[2] = t.positionNed.y;
        s[3] = t.positionNed.z;
        s[4] = msToKt(t.trueAirspeed);
        s[5] = msToKt(tr.calibratedAirspeed);
        s[6] = msToKt(t.groundSpeed);
        s[7] = mToFt(t.altitudeMsl);
        s[8] = msToFpm(t.verticalSpeed);
        s[9] = radToDeg(t.alpha);
        s[10] = radToDeg(t.beta);
        s[11] = radToDeg(t.roll);
        s[12] = radToDeg(t.pitch);
        s[13] = std::fmod(radToDeg(t.heading) + 360.0, 360.0);
        s[14] = radToDeg(t.omega.x);
        s[15] = radToDeg(t.omega.y);
        s[16] = radToDeg(t.omega.z);
        s[17] = t.loadFactor;
        s[18] = t.controls.elevator;
        s[19] = t.controls.aileron;
        s[20] = t.controls.rudder;
        s[21] = in.throttle;
        const AircraftDefinition& d = ac.definition();
        if (ac.turbofanCount() > 0) {
            for (int i = 0; i < std::min(ac.turbofanCount(), 2); ++i) {
                s[22 + static_cast<std::size_t>(i)] = t.turbofans[static_cast<std::size_t>(i)].thrust;
                s[24 + static_cast<std::size_t>(i)] = t.turbofans[static_cast<std::size_t>(i)].n1 * 100;
            }
        } else {
            for (int i = 0; i < std::min(ac.engineCount(), 2); ++i) {
                s[22 + static_cast<std::size_t>(i)] = t.engines[static_cast<std::size_t>(i)].prop.thrust;
                s[24 + static_cast<std::size_t>(i)] = t.engines[static_cast<std::size_t>(i)].rpm / d.engines[static_cast<std::size_t>(i)].ratedRpm * 100;
            }
        }
        s[26] = t.fuelMass;
        s[27] = t.gearPosition;
        s[28] = radToDeg(t.flapAngle);
        s[29] = t.onGround ? 1.0 : 0.0;
        trace_.rows.push_back(s);
    }
    Trace take() { return std::move(trace_); }

private:
    Trace trace_;
    long count_ = 0;
};

const Airport& airport(const char* icao) {
    for (const Airport& a : saoPauloRegionAirports())
        if (a.icao == icao) return a;
    throw std::invalid_argument(std::string("no airport ") + icao);
}

// Rudder that keeps the runway heading on the ground (as takeoffTest does).
Real centreline(const AircraftTelemetry& t, Real runwayHeading) {
    return clamp(-3.0 * wrapPi(t.heading - runwayHeading) - 0.8 * t.omega.z, -1.0, 1.0);
}

// ---- 172: take-off from Congonhas 35L to 500 ft AGL ----
Trace c172Takeoff() {
    Recorder rec("c172_takeoff_sbsp");
    const AircraftDefinition def = loadReference("tupi_t180");
    const LocalFrame frame = saoPauloWorldFrame();
    const Airport& ap = airport("SBSP");
    const RunwayDef& rw = ap.runways[0];
    auto ground = makeAirfieldGround(ap, frame);
    SimpleWeather atm(calmAt(ap.elevation));
    Aircraft ac(def, frame);
    ac.setEngineRunning(true, 1000);
    ac.setFlapsImmediately(1);  // flaps 10
    const Vec3 start = runwayPoint(rw, 30);
    ac.placeOnGround(start.x, start.y, rw.headingTrue, *ground, atm);
    ac.setCockpitControl("brakes.parking", 0);
    TestPilot pilot;
    pilot.controls.throttle = 1.0;
    pilot.controls.mixture = bestPowerMixture(def, atm.atmosphere().at(ap.elevation).density);
    pilot.lateral = TestPilot::Lateral::Track;
    pilot.trackOrigin = rw.threshold;
    pilot.targetHeading = rw.headingTrue;
    pilot.vertical = TestPilot::Vertical::Manual;
    bool rotated = false, climbing = false;
    for (int i = 0; i < static_cast<int>(180 / kDt); ++i) {
        const AircraftTelemetry& t = ac.telemetry();
        const Real ias = ac.instruments().readings().indicatedAirspeed;
        if (!rotated && ias >= ktToMs(55)) {
            rotated = true;
            pilot.vertical = TestPilot::Vertical::Pitch;
            pilot.targetPitch = degToRad(8);
            pilot.resetIntegrators();
        }
        if (!climbing && rotated && !t.onGround && t.altitudeMsl - ap.elevation > 15) {
            climbing = true;
            pilot.vertical = TestPilot::Vertical::Airspeed;  // Vy
            pilot.targetIas = ktToMs(74);
            pilot.targetPitch = degToRad(8);
        }
        if (climbing && t.altitudeMsl - ap.elevation > ftToM(300) && ac.flapDetent() != 0) ac.setCockpitControl("flaps.lever", 0);
        PilotInput in = pilot.update(ac, kDt);
        if (t.onGround) in.yaw = centreline(t, rw.headingTrue);
        if (!rotated) in.pitch = 0;
        ac.step(kDt, in, *ground, atm);
        rec.step(ac, in);
        if (ac.telemetry().altitudeMsl - ap.elevation >= ftToM(500)) break;
    }
    return rec.take();
}

// ---- 172: power-off stall and recovery from 3,000 ft ----
Trace c172Stall() {
    Recorder rec("c172_stall_3000ft");
    const AircraftDefinition def = loadReference("tupi_t180");
    Aircraft ac(def, LocalFrame{});
    SimpleWeather atm(calmAt(0));
    FlatGround ground(0);
    TestPilot pilot;
    startTrimmedFlight(ac, {70, ftToM(3000), 0}, atm, ground, &pilot);
    pilot.vertical = TestPilot::Vertical::Airspeed;
    pilot.controls.throttle = 0.0;
    pilot.targetPitch = 0.0;
    Real target = ktToMs(70), stallTime = -1, highAlphaSince = -1;
    int phase = 0;  // 0 decelerating, 1 released (break), 2 nose down, 3 climbing away
    for (int i = 0; i < static_cast<int>(150 / kDt); ++i) {
        const Real t = i * kDt;
        if (stallTime < 0) {
            target = std::max(target - ktToMs(1.0) * kDt, ktToMs(20));
            pilot.targetIas = target;
        }
        PilotInput in;
        if (stallTime < 0) {
            in = pilot.update(ac, kDt);
        } else {
            // Recovery as taught: release the back pressure and apply full
            // power at the break, let the nose fall to about 5 deg below the
            // horizon, then climb away at 70 KIAS.
            if (phase == 0) {
                phase = 1;
                pilot.controls.throttle = 1.0;
            }
            if (phase == 1) {
                in.pitch = 0;
                in.throttle = 1.0;
                if (t > stallTime + 1.5) {
                    phase = 2;
                    pilot.resetIntegrators();
                    pilot.vertical = TestPilot::Vertical::Pitch;
                    pilot.targetPitch = degToRad(-5);
                }
            } else {
                if (phase == 2 && (ac.instruments().readings().indicatedAirspeed > ktToMs(65) || t > stallTime + 6)) {
                    phase = 3;
                    pilot.resetIntegrators();
                    pilot.vertical = TestPilot::Vertical::Airspeed;
                    pilot.targetIas = ktToMs(70);
                    pilot.targetPitch = degToRad(5);
                }
                in = pilot.update(ac, kDt);
            }
        }
        ac.step(kDt, in, ground, atm);
        rec.step(ac, in);
        const AircraftTelemetry& tel = ac.telemetry();
        // The stall: the break (the nose dropping, or most of the wing
        // stalled), or a mushing stall that does not break (the wing holding
        // past 16.5 deg for 2 s, sinking with the nose up). Which one the
        // stall is turns on the last bits of the solution at the break.
        highAlphaSince = tel.alpha > degToRad(16.5) ? (highAlphaSince < 0 ? t : highAlphaSince) : -1;
        if (stallTime < 0 && (ac.aero().surfaces[0].stalledStrips >= 6 || (tel.omega.y < -0.12 && tel.alpha > 0.15) ||
                              (highAlphaSince >= 0 && t - highAlphaSince > 2)))
            stallTime = t;
        if (stallTime >= 0 && t > stallTime + 20) break;
    }
    return rec.take();
}

// ---- 172: trimmed cruise, stick free in pitch, wings held level, 5 minutes ----
// (Hands off entirely the 172 enters its spiral; the wing leveller keeps the
// run a pitch-axis record: the phugoid, the fuel burn and the trim.)
Trace c172Cruise() {
    Recorder rec("c172_cruise_trim");
    const AircraftDefinition def = loadReference("tupi_t180");
    Aircraft ac(def, LocalFrame{});
    SimpleWeather atm(calmAt(0));
    FlatGround ground(0);
    TestPilot pilot;
    startTrimmedFlight(ac, {100, ftToM(4500), 0}, atm, ground, &pilot);
    pilot.vertical = TestPilot::Vertical::Manual;  // the trimmed elevator is in the trim wheel
    pilot.lateral = TestPilot::Lateral::WingsLevel;
    for (int i = 0; i < static_cast<int>(300 / kDt); ++i) {
        const PilotInput in = pilot.update(ac, kDt);
        ac.step(kDt, in, ground, atm);
        rec.step(ac, in);
    }
    return rec.take();
}

// ---- Blind ILS approach to a stop (the blind-landing test's set-up, calm) ----
Trace ilsApproach(const char* traceName, const char* id, const char* icao, Real startNm) {
    Recorder rec(traceName);
    const AircraftDefinition def = loadReference(id);
    const LocalFrame frame = saoPauloWorldFrame();
    const Airport& ap = airport(icao);
    const RunwayDef& rw = ap.runways[0];
    const auto ground = makeRegionGround(saoPauloRegionAirports());
    SimpleWeatherParams wp;
    addSaoPauloWindEnvironment(wp, ap);
    wp.windSpeed = 0;
    wp.turbulence = TurbulenceParams{};
    wp.turbulence.intensity = 0;
    wp.turbulence.mechanical = false;
    const SimpleWeather wx(wp);
    Aircraft ac(def, frame);
    ac.setNavEnvironment(&saoPauloNavEnvironment());
    const bool jet = ac.turbofanCount() > 0;
    const int landing = static_cast<int>(def.controls.flapDetents.size()) - 1;
    const Vec3 onRw = runwayPoint(rw, rw.displaced + 400);
    ac.setEngineRunning(true, 1000);
    ac.placeOnGround(onRw.x, onRw.y, rw.headingTrue, *ground, wx);
    ac.step(kDt, PilotInput{}, *ground, wx);
    ac.instruments().setAltimeterSetting(wx.atmosphere().conditions().seaLevelPressure);
    ac.resetInstrumentsToTruth();
    ac.step(kDt, PilotInput{}, *ground, wx);
    const Real raZero = ac.telemetry().heightAgl;
    const Real baroZero = ac.instruments().readings().indicatedAltitude - ap.elevation;
    const AeroModel model(def);
    const Real cl = trimmedMaxLiftCoefficient(model, def.referenceArea, ac.massProperties().cg, def.controls.flapDetents.back(), 0.2);
    const Real vs = std::sqrt(2 * ac.massProperties().mass * kStandardGravity / (isa::kSeaLevelDensity * def.referenceArea * cl));
    const Real vapp = jet ? 1.23 * vs + ktToMs(5) : ktToMs(65);
    const Real out = startNm * kNauticalMile;
    const Real altIf = ap.elevation + 15.24 + out * std::tan(degToRad(3));
    ac.setGearImmediately(true);
    ac.setFlapsImmediately(landing);
    TrimRequest req;
    req.altitude = altIf;
    req.flapDetent = landing;
    req.tas = tasFromCas(vapp, wx.atmosphere().at(altIf));
    const TrimResult tr = trim(ac, req, wx);
    DynamicState s = tr.state;
    const Quat toHeading = Quat::fromEuler(0, 0, rw.headingTrue);
    s.body.attitude = toHeading * s.body.attitude;
    s.body.attitude.normalize();
    Vec3 p = runwayPoint(rw, rw.displaced - out);
    p.z = -altIf;
    s.body.position = p;
    s.body.velocity = toHeading.rotate(s.body.velocity);
    ac.setState(s);
    ac.prepareContext(tr.input, *ground, wx);
    ac.settleDownwash();
    ac.setElevatorTrim(tr.trim);
    ac.step(kDt, tr.input, *ground, wx);
    ac.instruments().setAltimeterSetting(wx.atmosphere().conditions().seaLevelPressure);
    ac.resetInstrumentsToTruth();

    ApproachPilot pilot;
    const Real course = rw.headingTrue - ap.magneticVariation;
    pilot.courseMagnetic = course;
    pilot.fieldElevation = ap.elevation;
    pilot.runwayLength = rw.length - rw.displaced;
    pilot.approachIas = vapp;
    pilot.interceptAltitude = altIf;
    pilot.radioAltimeter = jet;
    pilot.raZero = jet ? raZero : baroZero;
    pilot.pitchRef = tr.theta;
    pilot.baseThrottle = tr.input.throttle;
    pilot.controls = tr.input;
    for (const IlsDef& i : ap.ils)
        if (i.runway == 0) {
            ac.instruments().setNavStandby(0, i.khz);
            ac.instruments().swapNav(0);
        }
    ac.instruments().setObs(course);
    for (int i = 0; i < static_cast<int>(900 / kDt) && pilot.phase() != ApproachPilot::Phase::Stopped; ++i) {
        const PilotInput in = pilot.update(ac, kDt);
        ac.step(kDt, in, *ground, wx);
        rec.step(ac, in);
        if (ac.faulted()) break;
    }
    return rec.take();
}

// ---- Top G: take-off from Congonhas 35L to 1,500 ft AGL ----
Trace topGTakeoff() {
    Recorder rec("topg_takeoff_sbsp");
    const AircraftDefinition def = loadReference("top_g");
    const LocalFrame frame = saoPauloWorldFrame();
    const Airport& ap = airport("SBSP");
    const RunwayDef& rw = ap.runways[0];
    auto ground = makeAirfieldGround(ap, frame);
    SimpleWeather atm(calmAt(ap.elevation));
    Aircraft ac(def, frame);
    ac.setEngineRunning(true);
    ac.setFlapsImmediately(1);  // flaps 10
    const Vec3 start = runwayPoint(rw, 30);
    ac.placeOnGround(start.x, start.y, rw.headingTrue, *ground, atm);
    ac.setCockpitControl("brakes.parking", 0);
    TestPilot pilot;  // the lateral axis: the runway track, then wings level and coordinated
    pilot.controls.throttle = 1.0;
    pilot.lateral = TestPilot::Lateral::Track;
    pilot.trackOrigin = rw.threshold;
    pilot.targetHeading = rw.headingTrue;
    pilot.vertical = TestPilot::Vertical::Manual;
    bool rotated = false, gearUp = false;
    for (int i = 0; i < static_cast<int>(180 / kDt); ++i) {
        const AircraftTelemetry& t = ac.telemetry();
        const Real ias = ac.instruments().readings().indicatedAirspeed;
        const Real agl = t.altitudeMsl - ap.elevation;
        if (!rotated && ias >= ktToMs(125)) rotated = true;
        if (rotated && !gearUp && !t.onGround && agl > 15) {
            gearUp = true;
            ac.setCockpitControl("gear.lever", 0);
            pilot.lateral = TestPilot::Lateral::WingsLevel;  // off the runway: wings level, coordinated
        }
        if (gearUp && agl > ftToM(1000) && ac.flapDetent() != 0) ac.setCockpitControl("flaps.lever", 0);
        PilotInput in = pilot.update(ac, kDt);
        // Rotation and initial climb at a 12 deg attitude, with rate damping
        // (the attitude law the jet tests use, tests/test_jet.cpp); the
        // airplane accelerates through 200 KIAS on its way to 1,500 ft.
        if (rotated) in.pitch = clamp(5.0 * (degToRad(12) - t.pitch) - 1.5 * t.omega.y, -1.0, 1.0);
        else in.pitch = 0;
        if (t.onGround) in.yaw = centreline(t, rw.headingTrue);
        ac.step(kDt, in, *ground, atm);
        rec.step(ac, in);
        if (ac.telemetry().altitudeMsl - ap.elevation >= ftToM(1500)) break;
    }
    return rec.take();
}

// ---- Top G: trimmed cruise at FL410, Mach 0.85, hands off, 3 minutes ----
Trace topGCruise() {
    Recorder rec("topg_cruise_fl410");
    const AircraftDefinition def = loadReference("top_g");
    Aircraft ac(def, LocalFrame{});
    SimpleWeather atm(calmAt(0));
    FlatGround ground(0);
    ac.setGearImmediately(false);
    TrimRequest req;
    req.altitude = ftToM(41000);
    req.tas = 0.85 * atm.atmosphere().at(req.altitude).speedOfSound;
    const TrimResult tr = trim(ac, req, atm);
    ac.prepareContext(tr.input, ground, atm);
    ac.setElevatorTrim(tr.trim);
    ac.resetInstrumentsToTruth();
    PilotInput in = tr.input;
    in.pitch = tr.elevator;  // the stick beyond the stabiliser's travel (normally 0)
    for (int i = 0; i < static_cast<int>(180 / kDt); ++i) {
        ac.step(kDt, in, ground, atm);
        rec.step(ac, in);
    }
    return rec.take();
}

struct Entry {
    const char* name;
    const char* about;
    Trace (*fn)();
};
Trace c172Ils() { return ilsApproach("c172_ils_sbsp", "tupi_t180", "SBSP", 6); }
Trace topGIls() { return ilsApproach("topg_ils_sbgr", "top_g", "SBGR", 10); }

const Entry kEntries[] = {
    {"c172_takeoff_sbsp", "Tupi T-180: take-off from Congonhas 35L, flaps 10, rotate 55 KIAS, Vy 74 KIAS to 500 ft AGL", c172Takeoff},
    {"c172_stall_3000ft", "Tupi T-180: power-off stall from 70 KIAS at 3,000 ft (1 kt/s), recovery with full power, climb away", c172Stall},
    {"c172_cruise_trim", "Tupi T-180: trimmed 100 KIAS at 4,500 ft, stick free in pitch, wings held level, 5 minutes", c172Cruise},
    {"c172_ils_sbsp", "Tupi T-180: ILS approach to Congonhas 35L from 6 NM (ApproachPilot) to a stop", c172Ils},
    {"topg_takeoff_sbsp", "Top G: take-off from Congonhas 35L, flaps 10, rotate at 125 KIAS to 12 deg, gear up, climb to 1,500 ft AGL", topGTakeoff},
    {"topg_cruise_fl410", "Top G: trimmed Mach 0.85 at FL410, hands off for 3 minutes", topGCruise},
    {"topg_ils_sbgr", "Top G: ILS approach to Guarulhos from 10 NM (ApproachPilot) to a stop", topGIls},
};

}  // namespace

std::vector<std::string> scenarioNames() {
    std::vector<std::string> v;
    for (const Entry& e : kEntries) v.emplace_back(e.name);
    return v;
}

std::string describe(const std::string& name) {
    for (const Entry& e : kEntries)
        if (name == e.name) return e.about;
    return "";
}

Trace run(const std::string& name) {
    for (const Entry& e : kEntries)
        if (name == e.name) return e.fn();
    throw std::invalid_argument("unknown golden scenario '" + name + "'");
}

// ------------------------------------------------------------------ csv ----

void writeCsv(std::ostream& out, const Trace& t) {
    out << "# avd-golden/1 scenario=" << t.scenario << " rate_hz=10 channels=" << kChannels << "\n";
    out << "# " << describe(t.scenario) << "\n";
    const auto& ch = channels();
    for (int i = 0; i < kChannels; ++i) out << (i ? "," : "") << ch[static_cast<std::size_t>(i)].name;
    out << "\n";
    char buf[32];
    for (const Sample& s : t.rows) {
        for (int i = 0; i < kChannels; ++i) {
            std::snprintf(buf, sizeof buf, "%.9g", s[static_cast<std::size_t>(i)]);
            out << (i ? "," : "") << buf;
        }
        out << "\n";
    }
}

bool readCsv(std::istream& in, Trace& t, std::string& error) {
    t = Trace{};
    std::string line;
    bool header = false;
    std::size_t lineNo = 0;
    while (std::getline(in, line)) {
        ++lineNo;
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty()) continue;
        if (line[0] == '#') {
            const auto k = line.find("scenario=");
            if (k != std::string::npos) {
                const auto e = line.find(' ', k);
                t.scenario = line.substr(k + 9, e == std::string::npos ? std::string::npos : e - k - 9);
            }
            continue;
        }
        if (!header) {
            header = true;
            std::string expect;
            for (int i = 0; i < kChannels; ++i) expect += std::string(i ? "," : "") + channels()[static_cast<std::size_t>(i)].name;
            if (line != expect) {
                error = "line " + std::to_string(lineNo) + ": channel header differs from this build's (" + std::to_string(kChannels) + " channels)";
                return false;
            }
            continue;
        }
        Sample s{};
        std::stringstream ss(line);
        std::string cell;
        int n = 0;
        while (std::getline(ss, cell, ',')) {
            if (n >= kChannels) break;
            char* end = nullptr;
            s[static_cast<std::size_t>(n)] = std::strtod(cell.c_str(), &end);
            if (end == cell.c_str()) {
                error = "line " + std::to_string(lineNo) + ": not a number: '" + cell + "'";
                return false;
            }
            ++n;
        }
        if (n != kChannels) {
            error = "line " + std::to_string(lineNo) + ": " + std::to_string(n) + " values, expected " + std::to_string(kChannels);
            return false;
        }
        t.rows.push_back(s);
    }
    if (!header) {
        error = "no channel header";
        return false;
    }
    return true;
}

// -------------------------------------------------------------- compare ----

Comparison compare(const Trace& g, const Trace& f) {
    Comparison c;
    c.goldenRows = g.rows.size();
    c.freshRows = f.rows.size();
    const std::size_t n = std::min(g.rows.size(), f.rows.size());
    const auto& ch = channels();
    bool ok = g.rows.size() == f.rows.size();
    for (std::size_t k = 0; k < kChannels; ++k) {
        ChannelDelta& d = c.channel[k];
        d.tolerance = ch[k].tolerance;
        for (std::size_t i = 0; i < n; ++i) {
            Real delta = f.rows[i][k] - g.rows[i][k];
            if (ch[k].angleDeg) delta = std::remainder(delta, 360.0);
            const Real tol = ch[k].tolerance + ch[k].relative * std::abs(g.rows[i][k]);
            if (std::abs(delta) > d.worst) {
                d.worst = std::abs(delta);
                d.worstTime = g.rows[i][0];
                d.tolerance = tol;
            }
            if (std::abs(delta) > tol && d.firstBeyond < 0) d.firstBeyond = g.rows[i][0];
        }
        if (d.firstBeyond >= 0) {
            ok = false;
            if (c.firstDivergence < 0 || d.firstBeyond < c.firstDivergence) c.firstDivergence = d.firstBeyond;
        }
    }
    c.ok = ok;
    return c;
}

void writeReport(std::ostream& out, const Comparison& c) {
    const auto& ch = channels();
    out << std::fixed;
    if (c.goldenRows != c.freshRows)
        out << "  rows: golden " << c.goldenRows << ", this run " << c.freshRows << " (" << std::setprecision(1)
            << (static_cast<Real>(c.goldenRows) - 1) * kSampleInterval << " s vs " << (static_cast<Real>(c.freshRows) - 1) * kSampleInterval << " s)\n";
    if (c.firstDivergence >= 0) out << "  first divergence at t = " << std::setprecision(1) << c.firstDivergence << " s\n";
    out << "  channel        worst |delta|    at t (s)   band        status\n";
    for (std::size_t k = 1; k < kChannels; ++k) {
        const ChannelDelta& d = c.channel[k];
        out << "  " << std::left << std::setw(13) << ch[k].name << std::right << std::setw(14) << std::setprecision(4) << d.worst
            << std::setw(12) << std::setprecision(1) << d.worstTime << std::setw(11) << std::setprecision(3) << d.tolerance
            << "   " << (d.firstBeyond >= 0 ? "BEYOND from t = " + ([&] { std::ostringstream o; o << std::fixed << std::setprecision(1) << d.firstBeyond; return o.str(); })() : "ok")
            << "\n";
    }
}

// ----------------------------------------------------------------- paths ----

namespace {
std::string g_goldenDirOverride;
}  // namespace

void setGoldenDirectory(const std::string& dir) { g_goldenDirOverride = dir; }

std::string goldenDirectory() {
    if (!g_goldenDirOverride.empty()) return g_goldenDirOverride;
    if (const char* env = std::getenv("AVD_GOLDEN_DIR"); env && *env) return env;
#ifdef AVD_GOLDEN_DIR
    return AVD_GOLDEN_DIR;
#else
    return "tests/golden";
#endif
}

std::string goldenPath(const std::string& name) { return goldenDirectory() + "/" + name + ".csv"; }

}  // namespace avd::scenarios::golden
