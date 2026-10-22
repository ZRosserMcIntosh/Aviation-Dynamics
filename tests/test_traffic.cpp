// Aviation Dynamics — traffic tests: the line pilot that flies the other
// aircraft of a scene, and a near miss on final (a jet that lines up on the
// wrong runway passes over a light aircraft, whose path its wake then sinks
// into).
// SPDX-License-Identifier: MIT
#include "avd/units.hpp"
#include "avd/wake.hpp"
#include "avd_test.hpp"
#include "line_pilot.hpp"
#include "scenarios.hpp"

using namespace avd;
using namespace avd::scenarios;
using namespace avd::units;

namespace {

const AircraftDefinition& light() {
    static const AircraftDefinition d = loadReference("tupi_t180");
    return d;
}
const AircraftDefinition& jet() {
    static const AircraftDefinition d = loadReference("b738");
    return d;
}

// Trimmed on a straight descending line: at `pos` (NED), flying `course`
// (true, rad) down `path` (rad) at `kias` with flap `detent` and the gear down;
// the pilot set up to fly that line.
void startOnLine(Aircraft& ac, LinePilot& pilot, const Vec3& pos, Real course, Real path, Real kias, int detent,
                 const AtmosphereField& atm, const GroundQuery& ground) {
    ac.setEngineRunning(true, 2000);
    ac.setGearImmediately(true);
    TrimRequest req;
    req.altitude = -pos.z;
    req.flapDetent = detent;
    req.tas = tasFromCas(ktToMs(kias), StandardAtmosphere().at(-pos.z).density);
    req.flightPath = path;
    const TrimResult tr = trim(ac, req, atm);
    DynamicState s = tr.state;
    const Quat toCourse = Quat::fromEuler(0, 0, course);
    s.body.attitude = toCourse * s.body.attitude;
    s.body.attitude.normalize();
    s.body.position = pos;
    s.body.velocity = toCourse.rotate(s.body.velocity);
    ac.setState(s);
    ac.prepareContext(tr.input, ground, atm);
    ac.settleDownwash();
    ac.setElevatorTrim(tr.trim);
    ac.resetInstrumentsToTruth();
    pilot.origin = pos;
    pilot.course = course;
    pilot.path = path;
    pilot.targetIas = ktToMs(kias);
    pilot.baseThrottle = tr.input.throttle;
    pilot.pitchRef = tr.theta;
    pilot.controls = tr.input;
    pilot.controls.pitch = 0;
}

SimpleWeather calmAir() {
    SimpleWeather::Params wp;
    wp.turbulence.mechanical = false;
    return SimpleWeather(wp);
}

}  // namespace

TEST_CASE("traffic: the line pilot flies a 737 down a 3 deg glide path, then goes around") {
    const SimpleWeather atm = calmAir();
    FlatGround ground{0};
    Aircraft ac(jet());
    LinePilot pilot;
    const int flap = static_cast<int>(jet().controls.flapDetents.size()) - 1;
    startOnLine(ac, pilot, {0, 0, -600}, degToRad(350), degToRad(-3), 145, flap, atm, ground);
    Real worstAbove = 0, worstRight = 0, worstIas = 0;
    for (int i = 0; i < 240 * 60; ++i) {
        ac.step(kDefaultDt, pilot.update(ac, kDefaultDt), ground, atm);
        if (i > 240 * 10) {
            worstAbove = std::max(worstAbove, std::abs(pilot.above()));
            worstRight = std::max(worstRight, std::abs(pilot.right()));
            worstIas = std::max(worstIas, std::abs(msToKt(ac.instruments().readings().indicatedAirspeed) - 145));
        }
    }
    std::cerr << "  737 on the line: within " << worstAbove << " m of the path, " << worstRight << " m of the course, "
              << worstIas << " kt of the speed\n";
    CHECK(worstAbove < 8);
    CHECK(worstRight < 8);
    CHECK(worstIas < 6);
    const Real gaAlt = ac.telemetry().altitudeMsl;
    pilot.goAround();
    for (int i = 0; i < 240 * 20; ++i) ac.step(kDefaultDt, pilot.update(ac, kDefaultDt), ground, atm);
    std::cerr << "  go-around: " << ac.telemetry().altitudeMsl - gaAlt << " m up in 20 s, climbing at "
              << ac.telemetry().verticalSpeed << " m/s\n";
    CHECK(ac.telemetry().verticalSpeed > 2);
    CHECK(ac.telemetry().altitudeMsl - gaAlt > 100);  // 5 m/s and more on average
    CHECK(std::abs(radToDeg(ac.telemetry().roll)) < 5);
}

TEST_CASE("traffic: a 737 overtaking a light aircraft on final: its wake rolls it off to one side, sinks it below") {
    const SimpleWeather atm = calmAir();
    FlatGround ground{0};
    const Real course = degToRad(350), path = degToRad(-3);
    const Vec3 fwd{std::cos(course), std::sin(course), 0}, left{std::sin(course), -std::cos(course), 0};
    // The light aircraft on its final at 400 m; the jet 600 m behind, `above`
    // m above its path and `aside` m to its left, 145 kt against 65; it goes
    // around 900 m past the light aircraft's start. Worst bank (deg) and most
    // height lost below the path (m) in the next minute.
    auto encounter = [&](bool withWake, Real above, Real aside, Real* closest, Real* sunk) {
        Aircraft light172(light()), jet737(jet());
        LinePilot p172, p737;
        const Vec3 at172{0, 0, -400};
        startOnLine(light172, p172, at172, course, path, 65, 2, atm, ground);
        const Vec3 at737 = at172 - fwd * 600.0 + left * aside + Vec3{0, 0, -600 * std::tan(-path) - above};
        startOnLine(jet737, p737, at737, course, path, 145, static_cast<int>(jet().controls.flapDetents.size()) - 1, atm,
                    ground);
        p737.goAroundAt = 600 + 900;
        WakeParams wp;
        wp.maxBinsPerSurface = 12;
        Wake wake(jet737.aeroModel(), wp);
        WakeField air(atm);
        if (withWake) air.add(wake, 0);
        Real worst = 0;
        *closest = 1e9;
        *sunk = 0;
        for (int i = 0; i < 240 * 60; ++i) {
            jet737.step(kDefaultDt, p737.update(jet737, kDefaultDt), ground, atm);
            wake.update(jet737, atm, &ground);
            light172.step(kDefaultDt, p172.update(light172, kDefaultDt), ground, air);
            *closest = std::min(*closest, length(jet737.state().body.position - light172.state().body.position));
            worst = std::max(worst, std::abs(radToDeg(light172.telemetry().roll)));
            *sunk = std::max(*sunk, -p172.above());
        }
        return worst;
    };
    Real closeCalm, sunkCalm, close, sunk, closeOver, sunkOver;
    const Real calm = encounter(false, 20, 20, &closeCalm, &sunkCalm);
    const Real hit = encounter(true, 20, 20, &close, &sunk);
    std::cerr << "  20 m above, 20 m aside: closest " << close << " m; the 172 banks " << hit << " deg and sinks " << sunk
              << " m below its path in the 737's wake (" << calm << " deg, " << sunkCalm << " m without it)\n";
    CHECK(close < 35);
    CHECK(calm < 5);
    CHECK(sunkCalm < 5);
    CHECK(hit > 45);
    // Straight over it: between the vortices the downwash, not a roll.
    const Real over = encounter(true, 20, 0, &closeOver, &sunkOver);
    std::cerr << "  straight over it: the 172 banks " << over << " deg and sinks " << sunkOver << " m below its path\n";
    CHECK(over < 10);
    CHECK(sunkOver > 10);
}
