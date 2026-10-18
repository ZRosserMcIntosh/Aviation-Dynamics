// Aviation Dynamics — wake vortex tests: Biot-Savart, pair descent, ground
// effect, core growth and decay, an aircraft's own wake, wake encounters.
// SPDX-License-Identifier: MIT
#include "avd/units.hpp"
#include "avd/wake.hpp"
#include "avd_test.hpp"
#include "scenarios.hpp"

using namespace avd;
using namespace avd::scenarios;
using namespace avd::units;

namespace {

// Still air, standard sea-level state.
struct StillAir final : AtmosphereField {
    AirSample sample(const Vec3&, Real, Real) const override { return {}; }
};

const AircraftDefinition& c172() {
    static const AircraftDefinition d = loadReference();
    return d;
}

// A straight pair along +x, centred on x = 0 (the generator flew toward +x,
// so aft is -x): one gate every `dx` m over `length` m, all at time 0.
void seedLine(Wake& w, Real b0, Real gamma, Real z, Real groundZ = 1e9, Real length = 1600, Real dx = 4) {
    const int n = static_cast<int>(length / dx);
    for (int i = 0; i <= n; ++i)
        w.seedPair(0, {-0.5 * length + i * dx, 0, z}, {-1, 0, 0}, {0, 1, 0}, b0, gamma, groundZ);
}

// Advance in small steps to `t`.
void run(Wake& w, const AtmosphereField& air, Real from, Real to, Real dt = 0.05) {
    for (Real t = from; t < to; t += dt) w.advanceTo(std::min(t + dt, to), air);
}

}  // namespace

TEST_CASE("wake: a long straight segment induces gamma / (2 pi r), right-handed") {
    const Vec3 v = vortexSegmentVelocity({0, 0, -2}, {-5000, 0, 0}, {5000, 0, 0}, 10, 0.05);
    // Circulation along +x: at a point above (-z) the air moves toward +y.
    CHECK_NEAR(v.y, 10 / (kTwoPi * 2), 1e-3);
    CHECK_NEAR(v.x, 0, 1e-9);
    CHECK_NEAR(v.z, 0, 1e-9);
    // Lamb-Oseen core: solid-body inside, finite on the axis.
    const Vec3 c = vortexSegmentVelocity({0, 0, -0.01}, {-5000, 0, 0}, {5000, 0, 0}, 10, 1.0);
    CHECK(length(c) < 0.05);
    CHECK(length(vortexSegmentVelocity({0, 0, 0}, {-5000, 0, 0}, {5000, 0, 0}, 10, 1.0)) == 0);
}

TEST_CASE("wake: a lifting pair sinks at gamma / (2 pi b0) and its field is the Biot-Savart pair") {
    const StillAir air;
    Wake w;
    const Real b0 = 8.6, gamma = 20, z0 = -1000;
    seedLine(w, b0, gamma, z0);
    const Real w0 = gamma / (kTwoPi * b0);
    // Down between the vortices: 2 gamma / (pi b0) from the two lines.
    const Vec3 mid{0, 0, z0};
    const Vec3 u = w.velocity(mid);
    CHECK_NEAR(u.z, 2 * gamma / (kPi * b0), 0.03 * 2 * gamma / (kPi * b0));
    CHECK(std::abs(u.y) < 0.02 && std::abs(u.x) < 0.02);
    // Outside the pair: upwash.
    CHECK(w.velocity(mid + Vec3{0, 1.5 * b0, 0}).z < 0);
    // Sink for 10 s.
    run(w, air, 0, 10);
    const Wake::PairInfo p = w.pair(10);
    CHECK(p.valid);
    const Real sink = 0.5 * (p.left.z + p.right.z) - z0;
    CHECK_NEAR(sink / p.age, w0 * p.gamma / gamma, 0.03 * w0);
    CHECK_NEAR(p.spacing, b0, 0.01 * b0);  // out of ground effect the spacing holds
}

TEST_CASE("wake: near the ground the pair stops sinking and spreads apart (image vortices)") {
    const StillAir air;
    Wake w;
    const Real b0 = 30, gamma = 400;  // a heavy jet's pair
    seedLine(w, b0, gamma, -0.6 * b0, 0, 3000, 10);
    const Real t0 = kTwoPi * b0 * b0 / gamma;
    run(w, air, 0, 2.5 * t0);
    const Wake::PairInfo p = w.pair(2.5 * t0);
    CHECK(p.valid);
    // Inviscid image theory: x^2 y^2 / (x^2 + y^2) is conserved, so a pair
    // released at 0.6 b0 levels off near 0.38 b0 and the vortices separate.
    CHECK_RANGE(p.height / b0, 0.3, 0.5);
    CHECK(p.spacing > 1.4 * b0);
}

TEST_CASE("wake: cores grow with Squire's eddy viscosity; turbulence brings rapid decay sooner") {
    const StillAir air;
    WakeParams calm, rough;
    calm.eddyDissipation = 1e-5;
    rough.eddyDissipation = 1e-2;
    Wake a(calm), b(rough);
    const Real b0 = 8.6, gamma = 20;
    seedLine(a, b0, gamma, -1000);
    seedLine(b, b0, gamma, -1000);
    const Real t0 = kTwoPi * b0 * b0 / gamma;
    run(a, air, 0, 2 * t0, 0.2);
    run(b, air, 0, 2 * t0, 0.2);
    const Wake::PairInfo pa = a.pair(2 * t0), pb = b.pair(2 * t0);
    CHECK(pa.valid && pb.valid);
    const Real rc0 = calm.rolledCore * b0;
    CHECK_NEAR(sq(pa.core) - sq(rc0), 4 * 1.25643 * (calm.kinematicViscosity + calm.squire * gamma) * pa.age, 1e-6);
    // Calm: still most of its strength after 2 t0; moderate turbulence: mostly gone.
    CHECK(pa.gamma / pa.gamma0 > 0.85);
    CHECK(pb.gamma / pb.gamma0 < 0.5);
}

TEST_CASE("wake: a C172 in level flight sheds a rolled-up pair that carries its weight and sinks") {
    Aircraft ac(c172());
    SimpleWeather::Params wp;
    wp.turbulence.mechanical = false;
    const SimpleWeather atm(wp);
    FlatGround ground{0};
    TrimResult tr;
    startTrimmedFlight(ac, {100, 1500, 0}, atm, ground, nullptr, &tr);
    Wake wake(ac.aeroModel());
    PilotInput in = tr.input;
    in.pitch = 0;
    for (int i = 0; i < 20 * 240; ++i) {
        ac.step(kDefaultDt, in, ground, atm);
        wake.update(ac, atm, &ground);
    }
    CHECK(wake.gateCount() > 20);
    const Wake::PairInfo p = wake.pair(10);
    CHECK(p.valid);
    CHECK(p.rolled > 0.999);
    // The wing's span: from the vortex spacing, b0 = (pi / 4) b for elliptic
    // loading; a tapered real wing sheds a little wider.
    const Real span = ac.aeroModel().surfaceSpan(0);
    CHECK_RANGE(p.spacing / span, 0.72, 0.9);
    // rho V gamma0 b0: the wing's lift, about the weight.
    const Real rho = ac.telemetry().air.state.density, v = ac.telemetry().trueAirspeed;
    const Real weight = ac.massProperties().mass * 9.80665;
    CHECK_RANGE(rho * v * p.gamma0 * p.spacing / weight, 0.88, 1.15);
    // Sinking at gamma / (2 pi b0) between 6 and 14 s old.
    const Wake::PairInfo a = wake.pair(6), b = wake.pair(14);
    const Real rate = (0.5 * (b.left.z + b.right.z) - 0.5 * (a.left.z + a.right.z)) / (b.age - a.age);
    const Real w0 = p.gamma / (kTwoPi * p.spacing);
    // (the pair is not quite the same air at the two ages: the generator
    // shed a = 6 s and b = 14 s at different places; both started level)
    CHECK_RANGE(rate / w0, 0.6, 1.4);
    // The field between the vortices points down.
    const Vec3 mid = (p.left + p.right) * 0.5;
    CHECK(wake.velocity(mid).z > 0.5 * 2 * p.gamma / (kPi * p.spacing));
    // Moved somewhere it could not have flown: a new start clears the wake.
    DynamicState s = ac.state();
    s.body.position += Vec3{3000, 0, 0};
    ac.setState(s);
    wake.update(ac, atm, &ground);
    CHECK(wake.gateCount() <= 4);
}

TEST_CASE("wake: flying into a heavy jet's vortex rolls a light aircraft hard") {
    SimpleWeather::Params wp;
    wp.turbulence.mechanical = false;
    const SimpleWeather atm(wp);
    FlatGround ground{0};
    auto rollAfter = [&](bool withWake) {
        Aircraft ac(c172());
        TrimResult tr;
        startTrimmedFlight(ac, {100, 1500, 0}, atm, ground, nullptr, &tr);
        const Vec3 p = ac.state().body.position;
        // A heavy's pair (gamma 400, b0 50 m) lying along the flight path,
        // its right vortex 3 m to the left of the C172's CG... in front of it.
        Wake w;
        const Real heading = ac.telemetry().heading;
        const Vec3 fwd{std::cos(heading), std::sin(heading), 0}, right{-std::sin(heading), std::cos(heading), 0};
        const Vec3 rightVortex = p - right * 3.0;
        const Vec3 centre = rightVortex - right * 25.0;
        for (int i = 0; i <= 200; ++i)
            w.seedPair(0, centre + fwd * (-1000.0 + i * 10), fwd * -1.0, right, 50, 400);
        WakeField field(atm);
        if (withWake) field.add(w, 0);
        PilotInput in = tr.input;
        in.pitch = 0;
        for (int i = 0; i < 240; ++i) ac.step(kDefaultDt, in, ground, field);
        return std::abs(radToDeg(ac.telemetry().roll));
    };
    const Real calm = rollAfter(false), hit = rollAfter(true);
    CHECK(calm < 2);
    CHECK(hit > 25);
}
