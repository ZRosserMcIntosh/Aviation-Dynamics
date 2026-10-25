// Aviation Dynamics — tests: blind landings. In zero visibility the only
// guidance is the instruments: an instrument-only pilot (ApproachPilot: the
// ILS needles or the GPS course and path, attitude, heading, airspeed,
// vertical speed and the radio or barometric altimeter; no truth state)
// flies each aircraft from the intermediate fix to a stop, on ILS and RNAV
// approaches, calm and in a light crosswind (fog forms in light winds).
// Touchdown and rollout are checked against the touchdown zone and the
// runway: on the CAT III lateral limit (27 ft), under the 10 ft/s design
// sink rate, stopped on the runway.
// SPDX-License-Identifier: MIT
#include <cmath>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "airports.hpp"
#include "approach_pilot.hpp"
#include "avd/aero.hpp"
#include "avd/units.hpp"
#include "avd_test.hpp"
#include "scenarios.hpp"

using namespace avd;
using namespace avd::scenarios;
using namespace avd::units;

namespace {

struct Landing {
    bool touched = false, stopped = false, faulted = false;
    Real along = 0, lateral = 0, sinkFpm = 0, rollout = 0, stopLateral = 0, maxBankLow = 0;
    Real lateralAt200 = 0, verticalAt200 = 0;
    Real seatG = 0, dri = 0;  // the dummy in the pilot's seat
    bool gearCollapsed = false;
};

const Airport& airport(const char* icao) {
    for (const Airport& a : saoPauloRegionAirports())
        if (a.icao == icao) return a;
    return saoPauloRegionAirports()[0];
}

// Fly `id` blind onto runway end `rwy` of `icao` from `startNm` out, on the
// ILS (if `ils`) or the GPS approach; wind from `windFrom` (deg true) at
// `windKt` with turbulence `turb` (m/s RMS).
Landing fly(const char* id, const char* icao, int rwy, bool ils, Real startNm, Real windKt = 0, Real windFrom = 0,
            Real turb = 0) {
    if (const char* only = std::getenv("BLIND_ONLY"); only && std::string(only).find(id) == std::string::npos) {
        Landing skip;
        skip.touched = skip.stopped = true;
        skip.along = 300;
        return skip;
    }
    const AircraftDefinition def = loadReference(id);
    const LocalFrame frame = saoPauloWorldFrame();
    const Airport& ap = airport(icao);
    const RunwayDef& rw = ap.runways[static_cast<std::size_t>(rwy)];
    const auto ground = makeRegionGround(saoPauloRegionAirports());
    SimpleWeatherParams wp;
    addSaoPauloWindEnvironment(wp, ap);
    wp.windSpeed = ktToMs(windKt);
    wp.windFromDirection = degToRad(windFrom);
    wp.turbulence.intensity = turb;
    const SimpleWeather wx(wp);
    SimpleWeatherParams calm = wp;
    calm.windSpeed = 0;
    calm.turbulence = TurbulenceParams{};
    calm.turbulence.mechanical = false;
    const SimpleWeather calmAir(calm);

    Aircraft ac(def, frame);
    ac.setNavEnvironment(&saoPauloNavEnvironment());
    const bool jet = ac.turbofanCount() > 0;
    const int landing = static_cast<int>(def.controls.flapDetents.size()) - 1;
    // The radio and barometric altimeters' readings on the runway: touchdown.
    const Vec3 onRw = runwayPoint(rw, rw.displaced + 400);
    ac.setEngineRunning(true, 1000);
    ac.placeOnGround(onRw.x, onRw.y, rw.headingTrue, *ground, wx);
    ac.step(1.0 / 240, PilotInput{}, *ground, wx);
    ac.instruments().setAltimeterSetting(wx.atmosphere().conditions().seaLevelPressure);
    ac.resetInstrumentsToTruth();
    ac.step(1.0 / 240, PilotInput{}, *ground, wx);
    const Real raZero = ac.telemetry().heightAgl;
    const Real baroZero = ac.instruments().readings().indicatedAltitude - ap.elevation;

    // Approach speed: 1.23 Vs (landing flap, this weight) + 5 kt; the 172 at its book 65 KIAS.
    const AeroModel model(def);
    const Real cl = trimmedMaxLiftCoefficient(model, def.referenceArea, ac.massProperties().cg, def.controls.flapDetents.back(), 0.2);
    const Real vs = std::sqrt(2 * ac.massProperties().mass * kStandardGravity / (1.225 * def.referenceArea * cl));
    const Real vapp = jet ? 1.23 * vs + ktToMs(5) : ktToMs(65);

    // Trimmed, level at the intercept altitude, on the final course, landing configuration.
    const Real out = startNm * 1852;
    const Real altIf = ap.elevation + 15.24 + out * std::tan(degToRad(3));
    ac.setGearImmediately(true);
    ac.setFlapsImmediately(landing);
    TrimRequest req;
    req.altitude = altIf;
    req.flapDetent = landing;
    req.tas = tasFromCas(vapp, calmAir.atmosphere().at(altIf));
    const TrimResult tr = trim(ac, req, calmAir);
    DynamicState s = tr.state;
    const Quat toHeading = Quat::fromEuler(0, 0, rw.headingTrue);
    s.body.attitude = toHeading * s.body.attitude;
    s.body.attitude.normalize();
    Vec3 p = runwayPoint(rw, rw.displaced - out);
    p.z = -altIf;
    s.body.position = p;
    s.body.velocity = toHeading.rotate(s.body.velocity) + wx.sample(p, altIf, 0).wind;
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
    pilot.radioAltimeter = jet;  // the 172 has none: barometric height
    pilot.raZero = jet ? raZero : baroZero;
    pilot.pitchRef = tr.theta;
    pilot.baseThrottle = tr.input.throttle;
    pilot.controls = tr.input;
    GpsNavigator gps;
    if (ils) {
        for (const IlsDef& i : ap.ils)
            if (i.runway == rwy) {
                ac.instruments().setNavStandby(0, i.khz);
                ac.instruments().swapNav(0);
            }
        ac.instruments().setObs(course);
    } else {
        int faf = -1;
        auto route = approachRoute(ap, rwy, ac.state().body.position, faf);
        route.insert(route.begin(), RouteWaypoint{"PPOS", ac.state().body.position, -1, false});
        gps.setRoute(route, faf + 1);
        gps.activate(faf + (out > 5.2 * 1852 ? 1 : 2));
        pilot.source = ApproachPilot::Source::Gps;
        pilot.gps = &gps.indication();
    }

    Landing L;
    const Vec3 thr = landingThreshold(rw);
    const Real c = std::cos(rw.headingTrue), sn = std::sin(rw.headingTrue);
    for (int i = 0; i < 240 * 900 && pilot.phase() != ApproachPilot::Phase::Stopped; ++i) {
        const AircraftTelemetry& t = ac.telemetry();
        if (!ils) gps.update(ac.state().body.position, t.altitudeMsl, t.groundSpeed);
        const PilotInput in = pilot.update(ac, 1.0 / 240);
        ac.step(1.0 / 240, in, *ground, wx);
        const Vec3 q = ac.state().body.position;
        const Real along = (q.x - thr.x) * c + (q.y - thr.y) * sn, lateral = -(q.x - thr.x) * sn + (q.y - thr.y) * c;
        const Real h = ac.telemetry().heightAgl - raZero;
        if (h < ftToM(200) && !L.touched) L.maxBankLow = std::max(L.maxBankLow, std::abs(ac.telemetry().roll));
        if (L.lateralAt200 == 0 && h < ftToM(200) && along < 0) {
            L.lateralAt200 = lateral;
            const Real pathAlt = ap.elevation + 2 + (-along + 300) * std::tan(degToRad(3));  // from the GS antenna
            L.verticalAt200 = ac.telemetry().altitudeMsl - pathAlt;
        }
        if (!L.touched && ac.telemetry().onGround) {
            L.touched = true;
            L.along = along;
            L.lateral = lateral;
            L.sinkFpm = -msToFpm(ac.telemetry().verticalSpeed);
        }
        if (std::getenv("BLIND_TRACE") && std::string(std::getenv("BLIND_TRACE")) == id && i % 240 == 0)
            std::printf("      t %3d %-9s along %7.0f lat %+6.1f h %6.1f ias %5.1f vs %6.0f pitch %5.1f roll %5.1f hdg %5.1f y %+6.1f z %+6.1f thr %.2f elev %+.2f ztrue %+6.1f gs %+.2f\n",
                        i / 240, std::array<const char*, 5>{"icpt", "final", "flare", "roll", "stop"}[static_cast<std::size_t>(pilot.phase())],
                        along, lateral, h, msToKt(ac.instruments().readings().indicatedAirspeed), msToFpm(ac.telemetry().verticalSpeed),
                        radToDeg(ac.telemetry().pitch), radToDeg(ac.telemetry().roll), radToDeg(ac.instruments().readings().headingIndicator),
                        pilot.lateralMetres(), pilot.verticalMetres(), in.throttle, in.pitch,
                        ac.telemetry().altitudeMsl - (ap.elevation + 2 + (-along + 300) * std::tan(degToRad(3))), ac.instruments().readings().nav1.glideslope);
        if (ac.faulted()) { L.faulted = true; break; }
        L.rollout = along;
        L.stopLateral = lateral;
    }
    L.stopped = pilot.phase() == ApproachPilot::Phase::Stopped;
    L.seatG = ac.occupant().peakG;
    L.dri = ac.occupant().dri;
    for (int g = 0; g < static_cast<int>(def.gear.size()); ++g) L.gearCollapsed = L.gearCollapsed || ac.gearFailed(g);
    std::printf("    %-9s %s %-3s %s %4.1f NM wind %2.0f kt turb %.1f: TD %5.0f m past thr, %+5.1f m off, %4.0f fpm; stop %5.0f m, %+5.1f m; "
                "at 200 ft %+5.1f m / %+5.1f m; bank<200ft %4.1f deg; seat %.1f g, DRI %.1f%s\n",
                id, icao, rw.ident.c_str(), ils ? "ILS " : "RNAV", startNm, windKt, turb, L.along, L.lateral, L.sinkFpm,
                L.rollout, L.stopLateral, L.lateralAt200, L.verticalAt200, radToDeg(L.maxBankLow), L.seatG, L.dri, L.faulted ? " FAULT" : "");
    return L;
}

void checkLanding(const Landing& L, const RunwayDef& rw) {
    CHECK(L.touched && !L.faulted && L.stopped);
    CHECK_RANGE(L.along, 60.0, 914.0);                // the touchdown zone (3,000 ft)
    CHECK(std::abs(L.lateral) < 8.2);                 // within 27 ft of the centreline (AC 120-28D)
    CHECK(L.sinkFpm < 600);                            // under 10 ft/s, the design sink rate
    CHECK(L.rollout < rw.length - rw.displaced);       // stopped on the runway
    CHECK(std::abs(L.stopLateral) < 0.5 * rw.width - 3);
    // The pilot's seat rode an ordinary landing: the gear intact, the spine far from its limit (DRI 18).
    CHECK(!L.gearCollapsed);
    CHECK(L.dri < 5);
}

}  // namespace

TEST_CASE("blind landings: ILS to a stop in zero visibility, every aircraft, calm") {
    const Airport& gr = airport("SBGR");
    for (const char* id : {"tupi_t180", "top_g", "phenom300", "cj4", "learjet45", "crj700", "b738"}) {
        const bool jet = std::string(id) != "tupi_t180";
        checkLanding(fly(id, "SBGR", 0, true, jet ? 10 : 6), gr.runways[0]);
    }
}

TEST_CASE("blind landings: ILS in a crosswind; RNAV (GPS) approaches") {
    // Congonhas 35L (1,940 m, short for jets) with a 6 kt fog breeze from 060,
    // nearly straight across, and the mechanical turbulence it makes over the
    // city.
    const Airport& sp = airport("SBSP");
    for (const char* id : {"tupi_t180", "top_g", "crj700", "b738"}) {
        const bool jet = std::string(id) != "tupi_t180";
        checkLanding(fly(id, "SBSP", 0, true, jet ? 10 : 6, 6, 60), sp.runways[0]);
    }
    // 10 kt across: the two largest jets, flaring on the radio altimeter's rate.
    for (const char* id : {"crj700", "b738"}) checkLanding(fly(id, "SBSP", 0, true, 10, 10, 60), sp.runways[0]);
    // GPS approaches (São José dos Campos 16 has no ILS; Guarulhos 28L flown on GPS).
    const Airport& sj = airport("SBSJ");
    for (const char* id : {"tupi_t180", "top_g", "crj700"}) {
        const bool jet = std::string(id) != "tupi_t180";
        checkLanding(fly(id, "SBSJ", 0, false, jet ? 10 : 6), sj.runways[0]);
    }
    checkLanding(fly("b738", "SBGR", 3, false, 10, 6, 200), airport("SBGR").runways[3]);
}
