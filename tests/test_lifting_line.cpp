// Aviation Dynamics — nonlinear lifting line: classical results (Prandtl's
// elliptic wing, Glauert's rectangular wing, roll damping), swept-wing tip
// loading, ground effect and winglets.
// SPDX-License-Identifier: MIT
#include <cmath>

#include "avd/aero.hpp"
#include "avd/units.hpp"
#include "avd_test.hpp"

using namespace avd;
using namespace avd::units;

namespace {

// An inviscid, unstallable section: lift only, 2 pi per rad.
SectionPolar idealSection() {
    SectionPolar p;
    p.liftSlope = kTwoPi;
    p.zeroLiftAlpha = 0;
    p.clMax = 5;
    p.clMin = -5;
    p.cdMin = 0;
    p.cdQuadratic = 0;
    p.clAtCdMin = 0;
    p.cm0 = 0;
    return p;
}

constexpr Real kHalfPiT = kPi / 2;

// Half span b/2 in panels of one strip each, chord c(eta) at the panel ends.
template <class Chord>
AircraftDefinition wing(Real halfSpan, int panels, Chord chord, Real sweepDeg = 0, bool lifting = true) {
    AircraftDefinition d;
    LiftingSurfaceDef w;
    w.name = "wing";
    w.section = idealSection();
    w.liftingLine = lifting;
    w.spanEfficiency = 1;  // the ideal wing: induced drag from the planform alone
    w.groundEffect = true;
    for (int i = 0; i < panels; ++i) {
        // Cosine spacing: finer towards the tip, where the loading changes fastest.
        const Real t0 = 1 - std::cos(kHalfPiT * i / panels), t1 = 1 - std::cos(kHalfPiT * (i + 1) / panels);
        PanelDef p;
        p.span = halfSpan * (t1 - t0);
        p.rootQuarterChord = {-halfSpan * t0 * std::tan(degToRad(sweepDeg)), halfSpan * t0, 0};
        p.rootChord = chord(t0);
        p.tipChord = chord(t1);
        p.sweep = degToRad(sweepDeg);
        p.strips = 1;
        p.mirrored = true;
        w.panels.push_back(p);
    }
    d.surfaces.push_back(w);
    return d;
}
struct Polar {
    Real cl = 0, cdi = 0, area = 0, span = 0;
};
Polar at(const AeroModel& m, Real alpha, Real height = 1e9, Vec3 omega = {}) {
    AeroOutputs out;
    m.prepare(out);
    AeroInputs in;
    const Real v = 50, q = 0.5 * 1.225 * v * v;
    in.velocityBody = {v * std::cos(alpha), 0, v * std::sin(alpha)};
    in.omegaBody = omega;
    in.heightAboveGround = height;
    if (height < 1e8) in.groundUpBody = {std::sin(alpha), 0, -std::cos(alpha)};
    for (int k = 0; k < 3; ++k) m.evaluate(in, out);  // warm start, converged
    Polar p;
    for (int i = 0; i < m.stripCount(); ++i) p.area += m.stripGeometry(i).area;
    const Vec3 f = out.force;
    p.cl = (f.x * std::sin(alpha) - f.z * std::cos(alpha)) / (q * p.area);
    p.cdi = -(f.x * std::cos(alpha) + f.z * std::sin(alpha)) / (q * p.area);
    p.span = m.surfaceSpan(0);
    return p;
}

}  // namespace

TEST_CASE("lifting line: Prandtl's elliptic wing (CLa = 2 pi AR / (AR + 2), e = 1)") {
    for (Real ar : {6.0, 10.0}) {
        // Elliptic chord, b = 10 m: c0 = 4 b / (pi AR).
        const Real b = 10, c0 = 4 * b / (kPi * ar);
        const AeroModel m(wing(b / 2, 24, [&](Real t) { return std::max(c0 * std::sqrt(std::max(1 - t * t, 0.0)), 0.02 * c0); }));
        const Polar p = at(m, degToRad(4));
        const Real arModel = sq(p.span) / p.area;
        const Real expect = kTwoPi * arModel / (arModel + 2);
        CHECK_NEAR(p.cl / degToRad(4), expect, 0.03 * expect);
        // Induced drag CL^2 / (pi AR e) with e ~ 1.
        const Real e = sq(p.cl) / (kPi * arModel * p.cdi);
        CHECK_RANGE(e, 0.95, 1.03);
    }
}

TEST_CASE("lifting line: Glauert's rectangular wing, AR 8 (CLa ~4.9 per rad, e ~0.95)") {
    const AeroModel m(wing(8, 24, [](Real) { return 2.0; }));
    const Polar p = at(m, degToRad(4));
    CHECK_RANGE(p.cl / degToRad(4), 4.75, 5.05);
    const Real e = sq(p.cl) / (kPi * 8 * p.cdi);
    CHECK_RANGE(e, 0.9, 0.99);
    // Strip theory with the Helmbold slope gives the same lift but no span loading.
    const AeroModel strip(wing(8, 24, [](Real) { return 2.0; }, 0, false));
    CHECK_NEAR(at(strip, degToRad(4)).cl, p.cl, 0.06 * p.cl);
}

TEST_CASE("lifting line: roll damping of an elliptic wing (Clp = -pi AR / (4 (AR + 4)))") {
    const Real b = 10, ar = 8, c0 = 4 * b / (kPi * ar);
    const AeroModel m(wing(b / 2, 24, [&](Real t) { return std::max(c0 * std::sqrt(std::max(1 - t * t, 0.0)), 0.02 * c0); }));
    AeroOutputs out;
    m.prepare(out);
    AeroInputs in;
    const Real v = 50, q = 0.5 * 1.225 * v * v, p = 0.2;
    in.velocityBody = {v, 0, 0};
    in.omegaBody = {p, 0, 0};
    for (int k = 0; k < 3; ++k) m.evaluate(in, out);
    Real area = 0;
    for (int i = 0; i < m.stripCount(); ++i) area += m.stripGeometry(i).area;
    const Real arModel = sq(m.surfaceSpan(0)) / area;
    const Real clp = out.moment.x / (q * area * b) / (p * b / (2 * v));
    CHECK_NEAR(clp, -kPi * arModel / (4 * (arModel + 4)), 0.06 * kPi * arModel / (4 * (arModel + 4)));
}

TEST_CASE("lifting line: a swept wing loads its tips (and stalls there first); the ground cuts induced drag") {
    // 35 deg swept, untapered, untwisted, AR 8.
    const AeroModel m(wing(8, 24, [](Real) { return 2.0; }, 35));
    AeroOutputs out;
    m.prepare(out);
    AeroInputs in;
    in.velocityBody = {50 * std::cos(0.07), 0, 50 * std::sin(0.07)};
    for (int k = 0; k < 3; ++k) m.evaluate(in, out);
    // Strips of the right half from root to tip: the section cl climbs outboard.
    Real clRoot = 0, clOut = 0;
    for (int i = 0; i < m.stripCount(); ++i) {
        const AeroModel::StripGeometry g = m.stripGeometry(i);
        if (g.side < 0) continue;
        const Real eta = g.position.y / 8;
        if (eta < 0.15) clRoot = std::max(clRoot, out.strips[i].cl);
        if (eta > 0.7 && eta < 0.9) clOut = std::max(clOut, out.strips[i].cl);
    }
    CHECK(clOut > 1.1 * clRoot);
    // Ground effect: at h = b / 10 the induced drag at equal lift falls to
    // roughly McCormick's (16 h/b)^2 / (1 + (16 h/b)^2) = 0.72 of free air.
    const AeroModel r(wing(8, 24, [](Real) { return 2.0; }));
    const Polar free = at(r, degToRad(4)), near = at(r, degToRad(4), 1.6);
    const Real kFree = free.cdi / sq(free.cl), kNear = near.cdi / sq(near.cl);
    CHECK_RANGE(kNear / kFree, 0.55, 0.85);
    CHECK(near.cl > free.cl);  // and lifts more at the same angle
}

TEST_CASE("lifting line: a winglet cuts the induced drag at equal lift") {
    auto build = [](bool winglet) {
        AircraftDefinition d = wing(8, 20, [](Real t) { return 2.4 - 1.4 * t; }, 25);
        if (winglet) {
            PanelDef w;
            const PanelDef& tip = d.surfaces[0].panels.back();
            w.rootQuarterChord = {tip.rootQuarterChord.x - tip.span * std::tan(tip.sweep), 8, 0};
            w.span = 0.35;           // lateral extent
            w.dihedral = degToRad(75);  // ~1.3 m tall
            w.sweep = degToRad(40);
            w.rootChord = 1.0;
            w.tipChord = 0.45;
            w.strips = 3;
            w.mirrored = true;
            d.surfaces[0].panels.push_back(w);
        }
        return d;
    };
    const AeroModel plain(build(false)), winglet(build(true));
    const Polar a = at(plain, degToRad(5)), b = at(winglet, degToRad(5));
    // Induced drag at equal lift (both referenced to the plain wing's area).
    const Real kA = a.cdi / sq(a.cl), kB = (b.cdi * b.area) / sq(b.cl * b.area) * a.area;
    CHECK(kB < 0.93 * kA);
}
