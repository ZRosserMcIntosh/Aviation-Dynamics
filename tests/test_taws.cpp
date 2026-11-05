// Aviation Dynamics — tests: terrain awareness and ground proximity warning
// (avd/taws.hpp). A steep descent toward the ground, level flight into the
// Pico do Jaraguá on the region's real terrain, a normal ILS approach to
// Guarulhos, the same approach with the gear up, and the radio altimeter over
// the hills.
// SPDX-License-Identifier: MIT
#include <array>
#include <cmath>
#include <cstdio>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include "airports.hpp"
#include "approach_pilot.hpp"
#include "avd/aero.hpp"
#include "avd/taws.hpp"
#include "avd/units.hpp"
#include "avd_test.hpp"
#include "scenarios.hpp"

using namespace avd;
using namespace avd::scenarios;
using namespace avd::units;

namespace {

// The region's runways for the terrain database (both ends of each).
std::vector<TawsRunway> regionRunways() {
    std::vector<TawsRunway> r;
    for (const Airport& a : saoPauloRegionAirports())
        for (const RunwayDef& rw : a.runways) r.push_back({rw.threshold, rw.headingTrue, rw.length});
    return r;
}

const Airport& airport(const char* icao) {
    for (const Airport& a : saoPauloRegionAirports())
        if (a.icao == icao) return a;
    return saoPauloRegionAirports()[0];
}

// What the box said and when.
struct Log {
    struct Said {
        Real time, ra;
        TawsVoice voice;
    };
    std::vector<Said> said;
    unsigned count = 0;
    std::array<Real, static_cast<int>(TawsAlert::Count)> first{};  // time each alert first came on (-1 never)
    std::array<Real, static_cast<int>(TawsAlert::Count)> firstRa{};
    Log() { first.fill(-1); }
    void record(const Aircraft& ac) {
        const TawsStatus& s = ac.taws().status();
        for (int a = 1; a < static_cast<int>(TawsAlert::Count); ++a)
            if (s.isActive(static_cast<TawsAlert>(a)) && first[static_cast<std::size_t>(a)] < 0) {
                first[static_cast<std::size_t>(a)] = ac.time();
                firstRa[static_cast<std::size_t>(a)] = s.radioAltitude;
            }
        if (s.voiceCount != count) {
            count = s.voiceCount;
            said.push_back({ac.time(), s.radioAltitude, s.voice});
        }
    }
    Real at(TawsAlert a) const { return first[static_cast<std::size_t>(a)]; }
    int indexOf(TawsVoice v) const {
        for (std::size_t i = 0; i < said.size(); ++i)
            if (said[i].voice == v) return static_cast<int>(i);
        return -1;
    }
    std::string phrases() const {
        std::string s;
        TawsVoice last = TawsVoice::None;
        int repeats = 0;
        for (const Said& x : said) {
            if (x.voice == last) {
                ++repeats;
                continue;
            }
            if (repeats) s += " x" + std::to_string(repeats + 1);
            char b[96];
            std::snprintf(b, sizeof b, "%s[%.0fs %.0fft]", s.empty() ? "" : " | ", x.time, mToFt(x.ra));
            s += b;
            s += tawsVoiceText(x.voice);
            last = x.voice;
            repeats = 0;
        }
        if (repeats) s += " x" + std::to_string(repeats + 1);
        return s;
    }
};

// A jet trimmed level at `alt` m MSL and `kias` on `heading` at (north, east), clean.
struct Spawn {
    std::unique_ptr<Aircraft> ac;
    TrimResult tr;
};
Spawn spawnLevel(const AircraftDefinition& def, const SimpleWeather& wx, Real north, Real east, Real alt, Real kias, Real heading,
                 int flap = 0, bool gear = false) {
    Spawn s;
    s.ac = std::make_unique<Aircraft>(def, saoPauloWorldFrame());
    Aircraft& ac = *s.ac;
    ac.setEngineRunning(true, 1000);
    ac.setGearImmediately(gear);
    ac.setFlapsImmediately(flap);
    TrimRequest req;
    req.altitude = alt;
    req.flapDetent = flap;
    req.tas = tasFromCas(ktToMs(kias), wx.atmosphere().at(alt));
    s.tr = trim(ac, req, wx);
    DynamicState st = s.tr.state;
    const Quat toHeading = Quat::fromEuler(0, 0, heading);
    st.body.attitude = toHeading * st.body.attitude;
    st.body.attitude.normalize();
    st.body.position = {north, east, -alt};
    st.body.velocity = toHeading.rotate(st.body.velocity);
    ac.setState(st);
    ac.setElevatorTrim(s.tr.trim);
    return s;
}

// A simple pitch / wings-level hold around the trimmed controls.
PilotInput hold(const Aircraft& ac, const PilotInput& base, Real pitchCmd) {
    const AircraftTelemetry& t = ac.telemetry();
    PilotInput in = base;
    in.pitch = clamp(base.pitch + 2.0 * (pitchCmd - t.pitch) - 0.6 * ac.state().body.omega.y, -1.0, 1.0);
    in.roll = clamp(-1.5 * t.roll - 0.3 * ac.state().body.omega.x, -1.0, 1.0);
    return in;
}

}  // namespace

TEST_CASE("TAWS: a steep descent toward the ground -- SINK RATE, then PULL UP (Mode 1)") {
    const AircraftDefinition def = loadReference("top_g");
    const SimpleWeather wx{SimpleWeatherParams{}};
    const FlatGround ground(0);
    // 250 kt at 2,400 ft over flat ground, then nose down to a 4,000+ fpm descent.
    Spawn s = spawnLevel(def, wx, 0, 0, ftToM(2400) + 2, 250, 0);
    Aircraft& ac = *s.ac;
    CHECK(ac.taws().tawsClass() == TawsClass::A);
    Log log;
    Real pullUpRa = -1, sinkRa = -1, maxSink = 0;
    for (int i = 0; i < 240 * 60 && !ac.telemetry().onGround; ++i) {
        const Real cmd = ac.time() < 3 ? s.tr.theta - degToRad(4) * ac.time() : s.tr.theta - degToRad(12);
        ac.step(1.0 / 240, hold(ac, s.tr.input, cmd), ground, wx);
        log.record(ac);
        maxSink = std::max(maxSink, -msToFpm(ac.telemetry().verticalSpeed));
        if (ac.taws().status().radioAltitude < ftToM(300) && ac.time() > 5) break;  // the test ends here; no need to crash
    }
    sinkRa = log.firstRa[static_cast<int>(TawsAlert::SinkRate)];
    pullUpRa = log.firstRa[static_cast<int>(TawsAlert::PullUpSinkRate)];
    std::cerr << "  dive: max sink " << maxSink << " fpm; SINK RATE at " << log.at(TawsAlert::SinkRate) << " s (" << mToFt(sinkRa)
              << " ft), PULL UP at " << log.at(TawsAlert::PullUpSinkRate) << " s (" << mToFt(pullUpRa) << " ft)\n  voice: " << log.phrases() << "\n";
    CHECK(maxSink > 4000);
    CHECK(log.at(TawsAlert::SinkRate) > 0 && log.at(TawsAlert::PullUpSinkRate) > log.at(TawsAlert::SinkRate));
    CHECK(mToFt(sinkRa) > 1200 && mToFt(pullUpRa) > 600);
    const int sinkSaid = log.indexOf(TawsVoice::SinkRate), pullSaid = log.indexOf(TawsVoice::PullUp);
    CHECK(sinkSaid >= 0 && pullSaid > sinkSaid);
    // Once PULL UP is on, it is all the voice says.
    for (std::size_t i = static_cast<std::size_t>(std::max(pullSaid, 0)); i < log.said.size(); ++i)
        CHECK(log.said[i].voice == TawsVoice::PullUp || log.said[i].voice == TawsVoice::TerrainAheadPullUp);
    // Nothing for the configuration: no gear and no flaps alerts above 500 ft... and Mode 3 never armed (no take-off).
    CHECK(log.at(TawsAlert::DontSink) < 0);
}

TEST_CASE("TAWS: level flight toward the Pico do Jaraguá on the real terrain -- CAUTION TERRAIN, then TERRAIN AHEAD PULL UP, tens of seconds out") {
    const auto ground = makeSaoPauloGround();
    CHECK(ground->terrainCount() == 2);
    const AircraftDefinition def = loadReference("top_g");
    const SimpleWeather wx{SimpleWeatherParams{}};
    // 220 kt at 3,500 ft MSL (1,067 m), 12 km south-south-east of the summit (1,132 m on the grid), headed for it.
    const Vec3 peak = saoPauloMapNed(-23.4592, -46.7669);
    const Real hdg = degToRad(-20);
    const Real north = peak.x - 12000 * std::cos(hdg), east = peak.y - 12000 * std::sin(hdg);
    Spawn s = spawnLevel(def, wx, north, east, 1067, 220, hdg);
    Aircraft& ac = *s.ac;
    ac.taws().setRunways(regionRunways());
    ac.step(1.0 / 240, s.tr.input, *ground, wx);
    ac.resetInstrumentsToTruth();
    Log log;
    Real impact = -1, minClear = 1e9;
    for (int i = 0; i < 240 * 150; ++i) {
        ac.step(1.0 / 240, hold(ac, s.tr.input, s.tr.theta + 0.0005 * (1067 - ac.altitudeMsl()) - 0.004 * ac.telemetry().verticalSpeed), *ground, wx);
        log.record(ac);
        const Vec3 p = ac.state().body.position;
        const Real clear = ground->terrainDown(p.x, p.y) - p.z;
        minClear = std::min(minClear, clear);
        if (clear < 5 || ac.telemetry().onGround) {
            impact = ac.time();
            break;
        }
    }
    const Real caution = log.at(TawsAlert::CautionTerrain), warning = log.at(TawsAlert::TerrainAheadPullUp);
    std::cerr << "  toward Jaragua at 1,067 m: CAUTION TERRAIN at " << caution << " s, TERRAIN AHEAD PULL UP at " << warning
              << " s, the hill at " << impact << " s (" << (impact - caution) << " s / " << (impact - warning)
              << " s of warning); phase " << static_cast<int>(ac.taws().status().phase) << "\n  voice: " << log.phrases() << "\n";
    CHECK(impact > 0);
    CHECK(caution > 0 && warning > caution);
    CHECK(impact - caution > 30);
    CHECK(impact - warning > 18);
    const int c = log.indexOf(TawsVoice::CautionTerrain), w = log.indexOf(TawsVoice::TerrainAheadPullUp);
    CHECK(c >= 0 && w > c);
    // On the region's 200 m grid alone the summit is rounded off ~100 m lower,
    // and the city grid is what brings the hill up to the path: the same flight
    // at 1,067 m passes over the coarse summit (1,033 m) with 34 m to spare.
    RegionGround coarse = *makeRegionGround(saoPauloRegionAirports());
    coarse.setTerrain(saoPauloTerrain());
    CHECK(-coarse.terrainDown(peak.x, peak.y) < 1067);
    CHECK(-ground->terrainDown(peak.x, peak.y) > 1100);
}

namespace {

// Fly `id` down the ILS to Guarulhos 10L from 10 NM (the 172 from 6 NM) on
// the region's real terrain, gear down or up; the TAWS log to touchdown (or
// to 100 ft with the gear up).
Log ilsApproach(const char* id, bool gearDown, Real& touchdownTime) {
    const AircraftDefinition def = loadReference(id);
    const LocalFrame frame = saoPauloWorldFrame();
    const Airport& ap = airport("SBGR");
    const RunwayDef& rw = ap.runways[0];
    const auto ground = makeSaoPauloGround();
    SimpleWeatherParams wp;
    addSaoPauloWindEnvironment(wp, ap);
    wp.windSpeed = 0;
    wp.turbulence = TurbulenceParams{};
    wp.turbulence.mechanical = false;
    const SimpleWeather wx(wp);
    Aircraft ac(def, frame);
    ac.setNavEnvironment(&saoPauloNavEnvironment());
    ac.taws().setRunways(regionRunways());
    const bool jet = ac.turbofanCount() > 0;
    const int landing = static_cast<int>(def.controls.flapDetents.size()) - 1;
    const Vec3 onRw = runwayPoint(rw, rw.displaced + 400);
    ac.setEngineRunning(true, 1000);
    ac.placeOnGround(onRw.x, onRw.y, rw.headingTrue, *ground, wx);
    ac.step(1.0 / 240, PilotInput{}, *ground, wx);
    ac.instruments().setAltimeterSetting(wx.atmosphere().conditions().seaLevelPressure);
    ac.resetInstrumentsToTruth();
    ac.step(1.0 / 240, PilotInput{}, *ground, wx);
    const Real raZero = ac.telemetry().heightAgl;
    const Real baroZero = ac.instruments().readings().indicatedAltitude - ap.elevation;
    // The radio altimeter reads ~0 with the wheels on the runway.
    CHECK(std::abs(ac.taws().status().radioAltitude) < 0.5);

    const AeroModel model(def);
    const Real cl = trimmedMaxLiftCoefficient(model, def.referenceArea, ac.massProperties().cg, def.controls.flapDetents.back(), 0.2);
    const Real vs = std::sqrt(2 * ac.massProperties().mass * kStandardGravity / (1.225 * def.referenceArea * cl));
    const Real vapp = jet ? 1.23 * vs + ktToMs(5) : ktToMs(65);
    const Real out = (jet ? 10 : 6) * 1852.0;
    const Real altIf = ap.elevation + 15.24 + out * std::tan(degToRad(3));
    ac.setGearImmediately(gearDown);
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
    ac.step(1.0 / 240, tr.input, *ground, wx);
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
    Log log;
    touchdownTime = -1;
    for (int i = 0; i < 240 * 600 && pilot.phase() != ApproachPilot::Phase::Stopped; ++i) {
        const PilotInput in = pilot.update(ac, 1.0 / 240);
        ac.step(1.0 / 240, in, *ground, wx);
        log.record(ac);
        if (ac.telemetry().onGround) {
            touchdownTime = ac.time();
            break;
        }
        if (!gearDown && ac.taws().status().radioAltitudeValid && ac.taws().status().radioAltitude < ftToM(100)) break;
    }
    return log;
}

}  // namespace

TEST_CASE("TAWS: a normal ILS approach on the real terrain gives the callouts and nothing else; gear up, TOO LOW GEAR") {
    Real td = -1;
    const Log ils = ilsApproach("top_g", true, td);
    std::cerr << "  Top G ILS 10L SBGR, gear down: touchdown at " << td << " s\n  voice: " << ils.phrases() << "\n";
    CHECK(td > 0);
    for (int a = 1; a < static_cast<int>(TawsAlert::Count); ++a) {
        const TawsAlert al = static_cast<TawsAlert>(a);
        if (al == TawsAlert::AltitudeCallout || al == TawsAlert::Minimums) continue;
        if (ils.at(al) >= 0) std::cerr << "    unexpected " << tawsAlertText(al) << " at " << ils.at(al) << " s\n";
        CHECK(ils.at(al) < 0);
    }
    // The callouts in order, each once.
    const TawsVoice expected[] = {TawsVoice::Callout1000, TawsVoice::Callout500, TawsVoice::Minimums, TawsVoice::Callout100,
                                  TawsVoice::Callout50, TawsVoice::Callout40, TawsVoice::Callout30, TawsVoice::Callout20, TawsVoice::Callout10};
    int prev = -1;
    for (TawsVoice v : expected) {
        const int k = ils.indexOf(v);
        if (k <= prev) std::cerr << "    callout " << tawsVoiceText(v) << " missing or out of order\n";
        CHECK(k > prev);
        prev = k;
    }
    CHECK(ils.said.size() == std::size(expected));

    // The same approach with the gear left up: TOO LOW GEAR below 500 ft, and
    // said; no gear warning above it.
    Real td2 = -1;
    const Log up = ilsApproach("top_g", false, td2);
    const Real ra = up.firstRa[static_cast<int>(TawsAlert::TooLowGear)];
    std::cerr << "  gear up: TOO LOW GEAR at " << up.at(TawsAlert::TooLowGear) << " s, " << mToFt(ra) << " ft\n  voice: " << up.phrases() << "\n";
    CHECK(up.at(TawsAlert::TooLowGear) > 0);
    CHECK_RANGE(mToFt(ra), 450, 500);
    CHECK(up.indexOf(TawsVoice::TooLowGear) >= 0);
    CHECK(up.indexOf(TawsVoice::Callout500) < 0);  // the callouts are for a configured approach

    // The 172 (Class B): FIVE HUNDRED and nothing else.
    Real td3 = -1;
    const Log b = ilsApproach("tupi_t180", true, td3);
    std::cerr << "  172 (Class B) ILS: voice: " << b.phrases() << "\n";
    CHECK(td3 > 0);
    CHECK(b.said.size() == 1 && b.indexOf(TawsVoice::Callout500) == 0);
}

TEST_CASE("TAWS: the radio altimeter reads the height over the region's real terrain") {
    const auto ground = makeSaoPauloGround();
    const AircraftDefinition def = loadReference("top_g");
    const SimpleWeather wx{SimpleWeatherParams{}};
    // Over the Jaraguá summit, over the Cantareira and over the Pinheiros floor, level at 1,400 m (the Pinheiros floor 2,200 ft below, inside the radio altimeter's range).
    const Real lat[] = {-23.4592, -23.4164, -23.5865}, lon[] = {-46.7669, -46.6233, -46.6955};
    for (int k = 0; k < 3; ++k) {
        const Vec3 p = saoPauloMapNed(lat[k], lon[k]);
        Spawn s = spawnLevel(def, wx, p.x, p.y, 1400, 200, 0, 0, true);
        Aircraft& ac = *s.ac;
        ac.step(1.0 / 240, s.tr.input, *ground, wx);
        ac.step(1.0 / 240, s.tr.input, *ground, wx);
        const Vec3 q = ac.state().body.position;
        const Real terrain = -ground->terrainDown(q.x, q.y);
        const TawsStatus& st = ac.taws().status();
        // The wheels' height above the hill: the CG's less the CG-to-wheel
        // drop (~2 m); on a slope the nearest return is the perpendicular to
        // it (h cos(slope)), as a radio altimeter's beam finds it.
        const Real expected = ac.altitudeMsl() - terrain;
        Vec3 nrm;
        Real down = 0;
        SurfaceProperties surf;
        ground->surfaceAt(q.x, q.y, down, nrm, surf);
        const Real cosSlope = -nrm.z;
        std::cerr << "  over " << lat[k] << ", " << lon[k] << ": terrain " << terrain << " m, slope " << radToDeg(std::acos(cosSlope))
                  << " deg, RA " << st.radioAltitude << " m (CG " << expected << " m above)\n";
        CHECK(st.radioAltitudeValid);
        CHECK(std::abs(st.radioAltitude - expected * cosSlope) < 6);
    }
}

TEST_CASE("TAWS: the envelopes driven directly -- Modes 2 to 5, bank angle, Class B, the voice's priority") {
    const FlatGround ground(0);
    // Steady inputs at radio altitude h (ft), the aircraft over flat ground at 0.
    auto at = [](Real hFt, Real vsFpm, Real kias) {
        TawsInputs in;
        in.dt = 0.05;
        in.position = {0, 0, -ftToM(hFt)};
        in.velocity = {ktToMs(kias), 0, -fpmToMs(vsFpm)};
        in.heightAgl = ftToM(hFt);
        in.calibratedAirspeed = ktToMs(kias);
        return in;
    };
    auto fresh = [](TawsClass c) {
        TawsMonitor m;
        m.setClass(c);
        m.setTerrainInhibit(true);  // FLTA would see the flat ground below a low pass: the GPWS modes alone here
        return m;
    };
    auto run = [&](TawsMonitor& m, TawsInputs in, Real seconds) {
        for (Real t = 0; t < seconds; t += in.dt) m.update(in, ground);
        return m.status();
    };
    // Mode 2A: the ground coming up at 4,000 fpm under level flight at 1,000 ft:
    // TERRAIN TERRAIN, then PULL UP.
    {
        TawsMonitor m = fresh(TawsClass::A);
        TawsInputs in = at(1100, 0, 250);
        in.gearDown = false;
        m.update(in, ground);
        bool terrain = false, pullUp = false;
        for (Real h = 1100; h > 600; h -= fpmToMs(4000) * in.dt / units::kFoot) {
            in.heightAgl = ftToM(h);
            m.update(in, ground);
            terrain = terrain || m.status().isActive(TawsAlert::TerrainTerrain);
            pullUp = pullUp || (terrain && m.status().isActive(TawsAlert::PullUpClosure));
        }
        CHECK(terrain && pullUp);
        CHECK(m.status().voice == TawsVoice::PullUp || m.status().voice == TawsVoice::None);
    }
    // Mode 3: after lift-off, a climb to 400 ft, then sinking: DON'T SINK once the loss passes ~40 ft.
    {
        TawsMonitor m = fresh(TawsClass::A);
        TawsInputs in = at(0, 0, 140);
        in.onGround = true;
        m.update(in, ground);
        in.onGround = false;
        in.landingFlaps = false;
        for (Real h = 0; h < 400; h += 2000.0 / 60 * in.dt) m.update(at(h, 2000, 140), ground);
        CHECK(m.status().takeoffPhase);
        Real lossAtAlert = -1;
        for (Real h = 400; h > 300; h -= 1000.0 / 60 * in.dt) {
            TawsInputs d = at(h, -1000, 140);
            m.update(d, ground);
            if (lossAtAlert < 0 && m.status().isActive(TawsAlert::DontSink)) lossAtAlert = mToFt(m.status().altitudeLoss);
        }
        std::cerr << "  Mode 3: DON'T SINK after " << lossAtAlert << " ft lost from 400 ft\n";
        CHECK_RANGE(lossAtAlert, 30, 45);
    }
    // Mode 4: gear down, flaps up, 140 kt at 200 ft: TOO LOW FLAPS; gear up at 200 kt and 550 ft: TOO LOW TERRAIN.
    {
        TawsMonitor m = fresh(TawsClass::A);
        TawsInputs in = at(200, 0, 140);
        CHECK(run(m, in, 1).isActive(TawsAlert::TooLowFlaps));
        TawsMonitor m2 = fresh(TawsClass::A);
        in = at(550, 0, 200);
        in.gearDown = false;
        const TawsStatus s = run(m2, in, 1);
        CHECK(s.isActive(TawsAlert::TooLowTerrain) && !s.isActive(TawsAlert::TooLowGear));
        TawsMonitor m3 = fresh(TawsClass::A);
        in = at(600, 0, 200);
        in.gearDown = false;
        CHECK(run(m3, in, 1).active == 0);
    }
    // Mode 5: 1.6 dots low at 500 ft, GLIDESLOPE (soft); 1 dot, nothing; pegged at 200 ft, loud.
    {
        TawsMonitor m = fresh(TawsClass::A);
        TawsInputs in = at(500, -700, 140);
        in.landingFlaps = true;
        in.glideslopeValid = true;
        in.glideslope = 0.8;
        TawsStatus s = run(m, in, 1);
        CHECK(s.isActive(TawsAlert::Glideslope) && !s.glideslopeLoud);
        in.glideslope = 0.5;
        CHECK(!run(m, in, 0.2).isActive(TawsAlert::Glideslope));
        in = at(200, -700, 140);
        in.landingFlaps = in.glideslopeValid = true;
        in.glideslope = 1.0;
        s = run(m, in, 0.2);
        CHECK(s.isActive(TawsAlert::Glideslope) && s.glideslopeLoud);
    }
    // Bank angle: 40 deg at 1,000 ft, 30 deg not; 20 deg at 50 ft (the limit 19 deg there).
    {
        TawsMonitor m = fresh(TawsClass::B);
        TawsInputs in = at(1000, 0, 100);
        in.roll = degToRad(40);
        CHECK(run(m, in, 0.2).isActive(TawsAlert::BankAngle));
        in.roll = degToRad(30);
        CHECK(!run(m, in, 0.2).isActive(TawsAlert::BankAngle));
        in = at(50, 0, 70);
        in.roll = degToRad(20);
        CHECK(run(m, in, 0.2).isActive(TawsAlert::BankAngle));
    }
    // Class B: no Modes 2, 4 or 5 (gear up at 200 ft, the glideslope pegged, the ground rushing up): nothing.
    {
        TawsMonitor m = fresh(TawsClass::B);
        TawsInputs in = at(200, 0, 120);
        in.gearDown = false;
        in.glideslopeValid = true;
        in.glideslope = 1;
        CHECK(run(m, in, 1).active == 0);
    }
    // The voice: with PULL UP on, SINK RATE (also on) is not spoken in its gaps.
    {
        TawsMonitor m = fresh(TawsClass::A);
        TawsInputs in = at(1000, -5000, 250);
        in.gearDown = false;
        unsigned count = 0;
        int sink = 0, pull = 0;
        for (int i = 0; i < 100; ++i) {
            m.update(in, ground);
            if (m.status().voiceCount != count) {
                count = m.status().voiceCount;
                sink += m.status().voice == TawsVoice::SinkRate;
                pull += m.status().voice == TawsVoice::PullUp;
            }
        }
        CHECK(m.status().isActive(TawsAlert::SinkRate) && m.status().isActive(TawsAlert::PullUpSinkRate));
        CHECK(pull >= 3 && sink == 0);
    }
}
