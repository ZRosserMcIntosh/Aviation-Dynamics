// Aviation Dynamics — tests: the autopilot (scenarios::Autopilot): heading
// select, altitude hold and capture, vertical speed, the GPS route and the
// autothrottle, on the 172 and the jets, from the instruments alone.
// SPDX-License-Identifier: MIT
#include <algorithm>
#include <cmath>
#include <string>

#include "autopilot.hpp"
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
            // The bank it flies is the attitude indicator's (the gyro reads a
            // jet's 25 deg turn about 2 deg shallow: the true bank is reported).
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
