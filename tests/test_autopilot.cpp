// Aviation Dynamics — tests: the autopilot (scenarios::Autopilot): heading
// select, altitude hold and capture, vertical speed, the GPS route, the
// autothrottle and the coupled approach (APR: ILS and GPS) to the decision
// altitude, on the 172 and the jets, from the instruments alone.
// SPDX-License-Identifier: MIT
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "airports.hpp"
#include "autopilot.hpp"
#include "avd/aero.hpp"
#include "avd/units.hpp"
#include "avd_test.hpp"
#include "scenarios.hpp"

using namespace avd;
using namespace avd::scenarios;
using namespace avd::units;

namespace {

constexpr Real kDt = 1.0 / 240;

// Trimmed level at `altM` and `kias`, heading north, the altimeter on the
// field's setting (standard here) and the instruments erected.
struct Flight {
    Aircraft ac;
    SimpleWeather atm;
    FlatGround ground{0};
    PilotInput pilot;
    Flight(const char* id, Real altM, Real kias) : ac(loadReference(id)) {
        TrimRequest req;
        req.altitude = altM;
        req.tas = tasFromCas(ktToMs(kias), atm.atmosphere().at(altM));
        const TrimResult tr = trim(ac, req, atm);
        DynamicState s = tr.state;
        s.body.position = {0, 0, -altM};
        ac.setState(s);
        ac.prepareContext(tr.input, ground, atm);
        ac.settleDownwash();
        ac.setElevatorTrim(tr.trim);
        pilot = tr.input;
        ac.step(kDt, pilot, ground, atm);
        ac.instruments().setAltimeterSetting(atm.atmosphere().conditions().seaLevelPressure);
        ac.resetInstrumentsToTruth();
    }
};

}  // namespace

TEST_CASE("autopilot: heading select turns the short way at no more than 25 deg of bank; altitude and speed held") {
    for (const char* id : {"tupi_t180", "top_g", "b738"}) {
        const bool jet = std::string(id) != "tupi_t180";
        const Real alt = ftToM(jet ? 10000 : 4500), kias = jet ? 250 : 100;
        Flight f(id, alt, kias);
        Autopilot ap;
        ap.lateral = Autopilot::Lateral::Heading;
        ap.vertical = Autopilot::Vertical::Altitude;
        ap.autothrottle = true;
        ap.selectedAltitude = f.ac.instruments().readings().indicatedAltitude;
        ap.selectedIas = ktToMs(kias);
        ap.headingBug = degToRad(120);  // from north: a right turn
        ap.engage(f.ac, f.pilot);
        Real maxBank = 0, maxTrueBank = 0, worstAlt = 0, worstIas = 0, firstBank = 0;
        for (int i = 0; i < 240 * 150; ++i) {
            f.ac.step(kDt, ap.update(f.ac, f.pilot, kDt), f.ground, f.atm);
            const CockpitState& r = f.ac.instruments().readings();
            // The bank it flies is the attitude indicator's: on the jets an
            // AHRS, the true bank; on the 172 its vacuum gyro, which reads the
            // turn a degree or two shallow (the true bank is reported).
            maxBank = std::max(maxBank, std::abs(r.attitudeRoll));
            maxTrueBank = std::max(maxTrueBank, std::abs(f.ac.telemetry().roll));
            if (i == 240 * 5) firstBank = f.ac.telemetry().roll;
            worstAlt = std::max(worstAlt, std::abs(r.indicatedAltitude - ap.selectedAltitude));
            if (i > 240 * 30) worstIas = std::max(worstIas, std::abs(msToKt(r.indicatedAirspeed) - kias));
        }
        const Real hdgErr = radToDeg(std::abs(wrapPi(f.ac.instruments().readings().headingIndicator - ap.headingBug)));
        std::cerr << "  " << id << ": heading off by " << hdgErr << " deg, bank at most " << radToDeg(maxBank) << " deg (true " << radToDeg(maxTrueBank) << "), altitude within "
                  << mToFt(worstAlt) << " ft, IAS within " << worstIas << " kt\n";
        CHECK(firstBank > 0.05);  // the short way: right
        CHECK(hdgErr < 2);
        CHECK(radToDeg(maxBank) < 26.5);
        // The AHRS has no turn error: the airplane really banks no more than 25 deg.
        if (jet) CHECK(radToDeg(maxTrueBank) < 26.0);
        else CHECK(radToDeg(maxTrueBank) < 20.0);  // the 172: a rate-one turn (15 deg at 100 kt), plus its gyro's error
        CHECK(mToFt(worstAlt) < 100);
        CHECK(worstIas < 6);
        CHECK(!f.ac.faulted());
    }
}

TEST_CASE("autopilot: vertical speed to a selected altitude, captured without overshooting, then held") {
    for (const char* id : {"tupi_t180", "top_g", "crj700"}) {
        const bool jet = std::string(id) != "tupi_t180";
        const Real alt = ftToM(jet ? 8000 : 3000), kias = jet ? 240 : 95;
        for (const Real dir : {1.0, -1.0}) {
            Flight f(id, alt, kias);
            Autopilot ap;
            ap.lateral = Autopilot::Lateral::Roll;
            ap.vertical = Autopilot::Vertical::VerticalSpeed;
            ap.autothrottle = true;
            ap.selectedIas = ktToMs(kias);
            const Real start = f.ac.instruments().readings().indicatedAltitude;
            ap.selectedAltitude = start + dir * ftToM(jet ? 3000 : 1000);
            ap.selectedVs = dir * fpmToMs(jet ? 1500 : 500);
            ap.engage(f.ac, f.pilot);
            Real overshoot = 0, worstVs = 0, settled = -1;
            for (int i = 0; i < 240 * 240; ++i) {
                f.ac.step(kDt, ap.update(f.ac, f.pilot, kDt), f.ground, f.atm);
                const CockpitState& r = f.ac.instruments().readings();
                overshoot = std::max(overshoot, dir * (r.indicatedAltitude - ap.selectedAltitude));
                if (ap.vertical == Autopilot::Vertical::VerticalSpeed && i > 240 * 20)
                    worstVs = std::max(worstVs, std::abs(f.ac.telemetry().verticalSpeed - ap.selectedVs));
                if (settled < 0 && ap.vertical == Autopilot::Vertical::Altitude) settled = i * kDt;
            }
            const Real off = f.ac.instruments().readings().indicatedAltitude - ap.selectedAltitude;
            std::cerr << "  " << id << (dir > 0 ? " climb" : " descent") << ": rate within " << msToFpm(worstVs) << " fpm, overshoot "
                      << mToFt(overshoot) << " ft, ALT at " << settled << " s, then " << mToFt(off) << " ft off\n";
            CHECK(ap.vertical == Autopilot::Vertical::Altitude);
            CHECK(msToFpm(worstVs) < 300);
            CHECK(mToFt(overshoot) < 100);
            CHECK(std::abs(mToFt(off)) < 30);
            CHECK(!f.ac.faulted());
        }
    }
}

TEST_CASE("autopilot: NAV flies the GPS route, joining from the side and sequencing the turns") {
    for (const char* id : {"tupi_t180", "top_g"}) {
        const bool jet = std::string(id) != "tupi_t180";
        const Real alt = ftToM(jet ? 9000 : 4000), kias = jet ? 230 : 100;
        Flight f(id, alt, kias);
        // North 1 km off a route that runs north, then turns east.
        const Real leg = jet ? 25000 : 10000;
        GpsNavigator gps;
        std::vector<RouteWaypoint> route{{"A", {-5000, 1000, -alt}}, {"B", {leg, 1000, -alt}}, {"C", {leg, 1000 + leg, -alt}}};
        gps.setRoute(route);
        gps.activate(1);
        Autopilot ap;
        ap.lateral = Autopilot::Lateral::Nav;
        ap.vertical = Autopilot::Vertical::Altitude;
        ap.autothrottle = true;
        ap.selectedAltitude = f.ac.instruments().readings().indicatedAltitude;
        ap.selectedIas = ktToMs(kias);
        ap.gps = &gps.indication();
        ap.engage(f.ac, f.pilot);
        Real joined = -1, worstOnLeg1 = 0, worstOnLeg2 = 0;
        bool sequenced = false;
        const Real v = tasFromCas(ktToMs(kias), f.atm.atmosphere().at(alt));
        const int steps = static_cast<int>((leg * 1.6 + 6000) / v * 240);
        for (int i = 0; i < steps; ++i) {
            const AircraftTelemetry& t = f.ac.telemetry();
            gps.update(f.ac.state().body.position, t.altitudeMsl, t.groundSpeed);
            f.ac.step(kDt, ap.update(f.ac, f.pilot, kDt), f.ground, f.atm);
            const GpsIndication& gi = gps.indication();
            if (joined < 0 && std::abs(gi.crossTrack) < 50) joined = i * kDt;
            if (gi.active == 1 && joined >= 0) worstOnLeg1 = std::max(worstOnLeg1, std::abs(gi.crossTrack));
            if (std::getenv("AP_TRACE") && i % 1200 == 0)
                std::cerr << "    t " << i * kDt << " leg " << gi.active << " xtk " << gi.crossTrack << " dist " << gi.distanceToWaypoint
                          << " bank " << radToDeg(f.ac.telemetry().roll) << " cmd " << radToDeg(ap.bankCommand()) << "\n";
            if (gi.active == 2) {
                sequenced = true;
                // Past the turn's roll-out (a 25 deg turn at 230 kt has a 3 km radius).
                if (gi.distanceToWaypoint < leg - (jet ? 9000 : 4000)) worstOnLeg2 = std::max(worstOnLeg2, std::abs(gi.crossTrack));
            }
        }
        std::cerr << "  " << id << ": on the leg after " << joined << " s, then within " << worstOnLeg1 << " m; after the turn within "
                  << worstOnLeg2 << " m\n";
        CHECK(joined > 0 && joined < 150);
        CHECK(worstOnLeg1 < 60);
        CHECK(sequenced);
        CHECK(worstOnLeg2 < 150);
        CHECK(!f.ac.faulted());
    }
}

TEST_CASE("autopilot: VS away from the selected altitude flies the rate selected; it does not capture behind it") {
    Flight f("tupi_t180", ftToM(5400), 100);
    Autopilot ap;
    ap.lateral = Autopilot::Lateral::Roll;
    ap.vertical = Autopilot::Vertical::VerticalSpeed;
    ap.autothrottle = true;
    ap.selectedIas = ktToMs(100);
    const Real start = f.ac.instruments().readings().indicatedAltitude;
    ap.selectedAltitude = start - ftToM(1000);  // below, with VS +500: a climb away from it
    ap.selectedVs = fpmToMs(500);
    ap.engage(f.ac, f.pilot);
    Real worstVs = 0;
    for (int i = 0; i < 240 * 60; ++i) {
        f.ac.step(kDt, ap.update(f.ac, f.pilot, kDt), f.ground, f.atm);
        if (i > 240 * 15) worstVs = std::max(worstVs, std::abs(f.ac.telemetry().verticalSpeed - ap.selectedVs));
    }
    const Real climbed = f.ac.instruments().readings().indicatedAltitude - start;
    std::cerr << "  172, VS +500 with the altitude 1,000 ft below: " << mToFt(climbed) << " ft in 60 s, rate within " << msToFpm(worstVs)
              << " fpm, mode " << static_cast<int>(ap.vertical) << "\n";
    CHECK(ap.vertical == Autopilot::Vertical::VerticalSpeed);
    CHECK_RANGE(mToFt(climbed), 350.0, 650.0);
    CHECK(msToFpm(worstVs) < 100);
}

TEST_CASE("autopilot: the autothrottle holds its speed whatever lever it engaged on, and from a later SPD press") {
    for (const Real lever : {1.0, 0.0}) {
        for (const Real vsFpm : {0.0, -2000.0, 2000.0}) {
            if ((lever > 0.5 && vsFpm > 0) || (lever < 0.5 && vsFpm < 0)) continue;  // the hard cases: full levers down, idle up
            Flight f("top_g", ftToM(10000), 250);
            PilotInput levers = f.pilot;
            levers.throttle = lever;
            Autopilot ap;
            ap.lateral = Autopilot::Lateral::Roll;
            ap.vertical = vsFpm == 0 ? Autopilot::Vertical::Altitude : Autopilot::Vertical::VerticalSpeed;
            ap.selectedAltitude = f.ac.instruments().readings().indicatedAltitude + (vsFpm > 0 ? 1 : -1) * ftToM(vsFpm == 0 ? 0 : 6000);
            ap.selectedVs = fpmToMs(vsFpm);
            ap.autothrottle = true;
            ap.selectedIas = ktToMs(250);
            ap.engage(f.ac, levers);
            Real worst = 0;
            for (int i = 0; i < 240 * 120; ++i) {
                f.ac.step(kDt, ap.update(f.ac, levers, kDt), f.ground, f.atm);
                if (i > 240 * 40) worst = std::max(worst, std::abs(msToKt(f.ac.instruments().readings().indicatedAirspeed) - 250));
            }
            std::cerr << "  Top G, levers at " << lever << ", " << vsFpm << " fpm: IAS within " << worst << " kt of 250 after 40 s\n";
            CHECK(worst < 4);
        }
    }
    // The autopilot on without SPD (the pilot's levers in), then SPD pressed
    // with the levers at full: the base is taken then, not at the engagement.
    Flight f("top_g", ftToM(10000), 250);
    Autopilot ap;
    ap.lateral = Autopilot::Lateral::Roll;
    ap.vertical = Autopilot::Vertical::Altitude;
    ap.selectedAltitude = f.ac.instruments().readings().indicatedAltitude;
    ap.selectedIas = ktToMs(250);
    ap.engage(f.ac, f.pilot);
    for (int i = 0; i < 240 * 10; ++i) f.ac.step(kDt, ap.update(f.ac, f.pilot, kDt), f.ground, f.atm);
    PilotInput full = f.pilot;
    full.throttle = 1.0;
    ap.autothrottle = true;
    Real worst = 0;
    for (int i = 0; i < 240 * 120; ++i) {
        f.ac.step(kDt, ap.update(f.ac, full, kDt), f.ground, f.atm);
        if (i > 240 * 40) worst = std::max(worst, std::abs(msToKt(f.ac.instruments().readings().indicatedAirspeed) - 250));
    }
    std::cerr << "  Top G, SPD pressed later with the levers at full: IAS within " << worst << " kt\n";
    CHECK(worst < 4);
}

TEST_CASE("autopilot: PIT selected with the autopilot on holds the attitude at its selection") {
    Flight f("top_g", ftToM(8000), 240);
    Autopilot ap;
    ap.lateral = Autopilot::Lateral::Roll;
    ap.vertical = Autopilot::Vertical::VerticalSpeed;
    ap.selectedAltitude = f.ac.instruments().readings().indicatedAltitude + ftToM(5000);
    ap.selectedVs = fpmToMs(2000);
    ap.autothrottle = true;
    ap.selectedIas = ktToMs(240);
    ap.engage(f.ac, f.pilot);
    const Real engagedAt = f.ac.instruments().readings().attitudePitch;
    for (int i = 0; i < 240 * 30; ++i) f.ac.step(kDt, ap.update(f.ac, f.pilot, kDt), f.ground, f.atm);
    const Real selectedAt = f.ac.instruments().readings().attitudePitch;
    ap.vertical = Autopilot::Vertical::Pitch;
    for (int i = 0; i < 240 * 20; ++i) f.ac.step(kDt, ap.update(f.ac, f.pilot, kDt), f.ground, f.atm);
    const Real held = f.ac.instruments().readings().attitudePitch;
    std::cerr << "  pitch at engagement " << radToDeg(engagedAt) << " deg, at PIT " << radToDeg(selectedAt) << ", held " << radToDeg(held) << "\n";
    CHECK(selectedAt - engagedAt > degToRad(1.5));  // the climb raised it
    CHECK_NEAR(held, selectedAt, degToRad(0.7));
}

namespace {

const Airport& airportById(const char* icao) {
    for (const Airport& a : saoPauloRegionAirports())
        if (a.icao == icao) return a;
    return saoPauloRegionAirports()[0];
}

struct Approach {
    bool reachedDa = false, disconnected = false, latCaptured = false, vertCaptured = false;
    Real worstLatDots = 0, worstVertDots = 0, worstIas = 0, latDotsAtDa = 0, vertDotsAtDa = 0, iasAtDa = 0, heightAtDa = 0;
    Real latCaptureNm = 0, vertCaptureNm = 0;
};

// `id` coupled (APR) to runway end `rwy` of `icao` from `startNm` out, level
// `belowPathFt` under the 3 deg path, `offsetM` right of the extended
// centreline on a 30 deg intercept heading (ILS), or on the GPS approach
// route; the decision altitude 200 ft above the threshold; the autothrottle
// on the approach speed (the 172: 80 KIAS, flaps 10; a jet: 1.23 VS landing
// flap + 5 kt).
Approach flyApproach(const char* id, const char* icao, int rwy, bool ils, Real startNm, Real belowPathFt, Real offsetM) {
    const AircraftDefinition def = loadReference(id);
    const Airport& ap = airportById(icao);
    const RunwayDef& rw = ap.runways[static_cast<std::size_t>(rwy)];
    const auto ground = makeRegionGround(saoPauloRegionAirports());
    SimpleWeatherParams wp;
    addSaoPauloWindEnvironment(wp, ap);
    wp.windSpeed = 0;
    const SimpleWeather wx(wp);
    Aircraft ac(def, saoPauloWorldFrame());
    ac.setNavEnvironment(&saoPauloNavEnvironment());
    const bool jet = ac.turbofanCount() > 0;
    const int flap = jet ? static_cast<int>(def.controls.flapDetents.size()) - 1 : 1;
    const AeroModel model(def);
    const Real cl = trimmedMaxLiftCoefficient(model, def.referenceArea, ac.massProperties().cg, def.controls.flapDetents.back(), 0.2);
    const Real vstall = std::sqrt(2 * ac.massProperties().mass * kStandardGravity / (1.225 * def.referenceArea * cl));
    const Real vapp = jet ? 1.23 * vstall + ktToMs(5) : ktToMs(80);
    const Real out = startNm * 1852;
    const Real alt = ap.elevation + 15.24 + out * std::tan(degToRad(3)) - ftToM(belowPathFt);
    const Real course = rw.headingTrue - ap.magneticVariation;
    const Real hdgTrue = rw.headingTrue + (ils ? degToRad(-30) : 0.0);  // from the right of the centreline, turning left onto it
    ac.setGearImmediately(true);
    ac.setFlapsImmediately(flap);
    TrimRequest req;
    req.altitude = alt;
    req.flapDetent = flap;
    req.tas = tasFromCas(vapp, wx.atmosphere().at(alt));
    const TrimResult tr = trim(ac, req, wx);
    DynamicState s = tr.state;
    const Quat toHeading = Quat::fromEuler(0, 0, hdgTrue);
    s.body.attitude = toHeading * s.body.attitude;
    s.body.attitude.normalize();
    Vec3 p = runwayPoint(rw, rw.displaced - out, ils ? offsetM : 0.0);
    p.z = -alt;
    s.body.position = p;
    s.body.velocity = toHeading.rotate(s.body.velocity);
    ac.setState(s);
    ac.prepareContext(tr.input, *ground, wx);
    ac.settleDownwash();
    ac.setElevatorTrim(tr.trim);
    ac.step(kDt, tr.input, *ground, wx);
    ac.instruments().setAltimeterSetting(wx.atmosphere().conditions().seaLevelPressure);
    ac.resetInstrumentsToTruth();
    PilotInput pilot = tr.input;

    Autopilot apl;
    GpsNavigator gps;
    apl.magneticVariation = ap.magneticVariation;
    apl.fieldElevation = ap.elevation;
    apl.runwayLength = rw.length - rw.displaced;
    apl.decisionAltitude = ap.elevation + ftToM(200);
    apl.approachCourse = course;
    apl.headingBug = wrapPi(hdgTrue - ap.magneticVariation);
    if (ils) {
        for (const IlsDef& i : ap.ils)
            if (i.runway == rwy) {
                ac.instruments().setNavStandby(0, i.khz);
                ac.instruments().swapNav(0);
            }
        ac.instruments().setObs(course);
        apl.approachSource = Autopilot::ApproachSource::Ils;
    } else {
        int faf = -1;
        auto route = approachRoute(ap, rwy, ac.state().body.position, faf);
        route.insert(route.begin(), RouteWaypoint{"PPOS", ac.state().body.position, -1, false});
        gps.setRoute(route, faf + 1);
        gps.activate(faf + (out > 5.2 * 1852 ? 1 : 2));
        apl.approachSource = Autopilot::ApproachSource::Gps;
        apl.gps = &gps.indication();
    }
    apl.lateral = Autopilot::Lateral::Approach;
    apl.vertical = Autopilot::Vertical::Altitude;
    apl.selectedAltitude = ac.instruments().readings().indicatedAltitude;
    apl.autothrottle = true;
    apl.selectedIas = vapp;
    apl.engage(ac, pilot);

    Approach A;
    const Vec3 thr = landingThreshold(rw);
    for (int i = 0; i < 240 * 900; ++i) {
        const AircraftTelemetry& t = ac.telemetry();
        if (!ils) gps.update(ac.state().body.position, t.altitudeMsl, t.groundSpeed);
        const PilotInput in = apl.update(ac, pilot, kDt);
        ac.step(kDt, in, *ground, wx);
        const Vec3 q = ac.state().body.position;
        const Real toGo = -((q.x - thr.x) * std::cos(rw.headingTrue) + (q.y - thr.y) * std::sin(rw.headingTrue)) / 1852;
        const Real iasErr = std::abs(msToKt(ac.instruments().readings().indicatedAirspeed - vapp));
        if (!A.latCaptured && apl.approachState() >= Autopilot::ApproachState::LateralCaptured) A.latCaptured = true, A.latCaptureNm = toGo;
        if (!A.vertCaptured && apl.approachState() >= Autopilot::ApproachState::VerticalCaptured) A.vertCaptured = true, A.vertCaptureNm = toGo;
        // Established: 1 NM after the glidepath capture, to DA.
        if (A.vertCaptured && toGo < A.vertCaptureNm - 1 && apl.approachState() == Autopilot::ApproachState::VerticalCaptured) {
            A.worstLatDots = std::max(A.worstLatDots, std::abs(apl.lateralDots()));
            A.worstVertDots = std::max(A.worstVertDots, std::abs(apl.verticalDots()));
            A.worstIas = std::max(A.worstIas, iasErr);
        }
        if (apl.disconnected()) {
            A.reachedDa = true;
            A.disconnected = apl.lateral == Autopilot::Lateral::Off && apl.vertical == Autopilot::Vertical::Off;
            A.latDotsAtDa = apl.lateralDots();
            A.vertDotsAtDa = apl.verticalDots();
            A.iasAtDa = iasErr;
            A.heightAtDa = mToFt(ac.instruments().readings().indicatedAltitude - ap.elevation);
            break;
        }
        if (ac.faulted() || t.onGround) break;
    }
    std::printf("    %-9s %s %-3s %s %4.1f NM: LOC/LNAV captured %.1f NM out, GS/GP %.1f NM; then within %.2f / %.2f dots, IAS %.1f kt; "
                "DA (%.0f ft): %+.2f / %+.2f dots, %.1f kt off%s\n",
                id, icao, rw.ident.c_str(), ils ? "ILS " : "RNAV", startNm, A.latCaptureNm, A.vertCaptureNm, A.worstLatDots, A.worstVertDots,
                A.worstIas, A.heightAtDa, A.latDotsAtDa, A.vertDotsAtDa, A.iasAtDa, A.disconnected ? ", disconnected" : "");
    return A;
}

void checkApproach(const Approach& A) {
    CHECK(A.latCaptured && A.vertCaptured);
    CHECK(A.reachedDa && A.disconnected);
    // CAT I: within about a dot of the localizer and glideslope, on speed.
    CHECK(A.worstLatDots < 1.0);
    CHECK(A.worstVertDots < 1.0);
    CHECK(std::abs(A.latDotsAtDa) < 0.5);
    CHECK(std::abs(A.vertDotsAtDa) < 0.5);
    CHECK(A.worstIas < 5);
    CHECK_RANGE(A.heightAtDa, 150.0, 210.0);
}

}  // namespace

TEST_CASE("autopilot: APR couples the SBGR 10L ILS from 10 NM: LOC, then GS from below, flown to DA, where it disconnects") {
    for (const char* id : {"tupi_t180", "top_g"}) checkApproach(flyApproach(id, "SBGR", 0, true, 10, 500, 1500));
}

TEST_CASE("autopilot: APR on an RNAV (GPS) approach: LNAV, then the glidepath from level, to DA") {
    for (const char* id : {"tupi_t180", "top_g"}) checkApproach(flyApproach(id, "SBSJ", 0, false, 7, 300, 0));
}
