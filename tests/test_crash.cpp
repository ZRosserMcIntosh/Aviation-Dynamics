// Aviation Dynamics — tests: the airframe's limits and its occupants: a crash
// dummy in the pilot's seat, the landing gear collapsing in a hard impact,
// the structure past its ultimate load factor.
// SPDX-License-Identifier: MIT
#include <algorithm>
#include <cmath>
#include <string>

#include "avd/units.hpp"
#include "avd_test.hpp"
#include "scenarios.hpp"

using namespace avd;
using namespace avd::scenarios;
using namespace avd::units;

namespace {

// Settled on flat ground, then lifted `drop` m and let fall, wings level, to
// strike it at `impact` m/s.
void dropOnto(Aircraft& ac, const FlatGround& g, const SimpleWeather& atm, Real drop, Real impact) {
    ac.placeOnGround(0, 0, 0, g, atm);
    for (int i = 0; i < 240; ++i) ac.step(kDefaultDt, PilotInput{}, g, atm);
    DynamicState s = ac.state();
    s.body.position.z -= drop;
    s.body.velocity = {0, 0, std::sqrt(std::max(impact * impact - 2 * 9.80665 * drop, 0.0))};
    s.body.omega = {};
    ac.setState(s);
}

bool collapsedAny(const Aircraft& ac) {
    for (int i = 0; i < static_cast<int>(ac.definition().gear.size()); ++i)
        if (ac.gearFailed(i)) return true;
    return false;
}

}  // namespace

TEST_CASE("crash: the dummy in the pilot's seat reads 1 g sitting on the ramp") {
    Aircraft ac(loadReference("tupi_t180"));
    SimpleWeather atm;
    FlatGround g(0);
    ac.placeOnGround(0, 0, 0, g, atm);
    for (int i = 0; i < 240 * 3; ++i) ac.step(kDefaultDt, PilotInput{}, g, atm);
    const OccupantReport& o = ac.occupant();
    std::cerr << "  at rest: peak " << o.peakG << " g, DRI " << o.dri << ", HIC " << o.hic << ", lumbar " << o.lumbarN << " N\n";
    CHECK_RANGE(o.peakG, 0.95, 1.2);
    CHECK_RANGE(o.dri, 0.95, 1.3);
    CHECK(o.hic < 1);
    CHECK(!o.hicExceeded() && !o.lumbarExceeded() && !o.driExceeded());
    CHECK(!collapsedAny(ac) && !ac.structurallyFailed());
}

TEST_CASE("crash: a firm drop is absorbed; past the reserve-energy sink rate the gear collapses onto the belly") {
    for (const char* id : {"tupi_t180", "top_g"}) {
        SimpleWeather atm;
        FlatGround g(0);
        // Firm: 2.5 m/s (490 fpm). Hard: 6 m/s (1,180 fpm), past the 12-14 ft/s the gear is rated to absorb.
        Aircraft firm(loadReference(id)), hard(loadReference(id));
        dropOnto(firm, g, atm, 0.3, 2.5);
        dropOnto(hard, g, atm, 0.3, 6.0);
        bool collapseEvent = false, bellyContact = false;
        for (int i = 0; i < 240 * 4; ++i) {
            firm.step(kDefaultDt, PilotInput{}, g, atm);
            hard.step(kDefaultDt, PilotInput{}, g, atm);
            for (const FlightEvent& e : hard.drainEvents()) {
                collapseEvent = collapseEvent || e.type == FlightEventType::GearCollapse;
                bellyContact = bellyContact || e.type == FlightEventType::HardPointContact || e.type == FlightEventType::PropStrike;
            }
        }
        const OccupantReport &f = firm.occupant(), &h = hard.occupant();
        std::cerr << "  " << id << ": firm (2.5 m/s) peak " << f.peakG << " g, DRI " << f.dri << ", collapsed " << collapsedAny(firm)
                  << "; hard (6 m/s) peak " << h.peakG << " g, DRI " << h.dri << ", HIC " << h.hic << ", lumbar " << h.lumbarN
                  << " N, collapsed " << collapsedAny(hard) << ", on its hard points " << bellyContact << "\n";
        CHECK(!collapsedAny(firm));
        CHECK(collapsedAny(hard) && collapseEvent);
        CHECK(bellyContact);
        CHECK(h.dri > f.dri && h.peakG > f.peakG);
        CHECK(!hard.faulted());
    }
}

TEST_CASE("crash: full aft stick at 100 kt stalls short of the ultimate load; at 155 kt the structure fails") {
    auto pull = [](Real kias, Real& worstNz) {
        Aircraft ac(loadReference("tupi_t180"));
        SimpleWeather atm;
        TrimRequest req;
        req.altitude = 2000;
        req.tas = tasFromCas(ktToMs(kias), atm.atmosphere().at(2000.0));
        const TrimResult tr = trim(ac, req, atm);
        DynamicState s = tr.state;
        s.body.position = {0, 0, -2000};
        ac.setState(s);
        FlatGround g(0);
        ac.prepareContext(tr.input, g, atm);
        ac.settleDownwash();
        ac.setElevatorTrim(tr.trim);
        PilotInput in = tr.input;
        in.pitch = 1;
        worstNz = 0;
        for (int i = 0; i < 240 * 3; ++i) {
            ac.step(kDefaultDt, in, g, atm);
            worstNz = std::max(worstNz, ac.telemetry().loadFactor);
        }
        return ac.structurallyFailed();
    };
    Real n100, n155;
    const bool f100 = pull(100, n100), f155 = pull(155, n155);
    std::cerr << "  full aft stick: at 100 kt " << n100 << " g (failed " << f100 << "), at 155 kt " << n155 << " g (failed " << f155
              << "); ultimate 5.7 g\n";
    CHECK(!f100);
    CHECK(n100 < 5.7);
    CHECK(f155);
}
