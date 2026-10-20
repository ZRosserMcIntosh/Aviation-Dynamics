// Aviation Dynamics — transonic behaviour of the Top G (G650ER-class) at
// altitude: Mach tuck, the buffet boundary and the elevator going heavy
// toward Mmo. The section model is in aero.cpp (normal-Mach clmax knee,
// aerodynamic-centre travel past Mcrit, hinged-control effectiveness loss).
// SPDX-License-Identifier: MIT
#include <cstdio>

#include "avd/aero.hpp"
#include "avd/units.hpp"
#include "avd_test.hpp"
#include "scenarios.hpp"

using namespace avd;
using namespace avd::scenarios;
using namespace avd::units;

namespace {

const AircraftDefinition& jet() {
    static const AircraftDefinition d = loadReference("top_g");
    return d;
}

// The bare aero model in steady wings-level flight at an altitude: lift and
// pitching moment coefficients at a Mach number, angle of attack and
// longitudinal control settings (no thrust, no rotation).
struct Longitudinal {
    const AeroModel& model;
    const Real refArea, chord;
    Real density = 1.225, speedOfSound = 340.3;
    AeroInputs in;
    AeroOutputs out;

    Longitudinal(const Aircraft& ac, Real altFt)
        : model(ac.aeroModel()), refArea(jet().referenceArea), chord(jet().referenceChord) {
        const StandardAtmosphere atm;
        const auto a = atm.at(ftToM(altFt));
        density = a.density;
        speedOfSound = a.speedOfSound;
        in.density = density;
        in.speedOfSound = speedOfSound;
        in.cg = ac.massProperties().cg;
        model.prepare(out);
    }
    struct Point {
        Real cl = 0, cm = 0, q = 0;
    };
    Point at(Real mach, Real alpha, Real elevator, Real stabilizer) {
        const Real v = mach * speedOfSound;
        in.velocityBody = {v * std::cos(alpha), 0, v * std::sin(alpha)};
        in.controls.elevator = elevator;
        in.controls.stabilizer = stabilizer;
        for (int k = 0; k < 3; ++k) model.evaluate(in, out);  // warm start, converged
        Point p;
        p.q = 0.5 * density * v * v;
        const Real lift = out.force.x * std::sin(alpha) - out.force.z * std::cos(alpha);
        p.cl = lift / (p.q * refArea);
        p.cm = out.moment.y / (p.q * refArea * chord);
        return p;
    }
    // Angle of attack for a lift coefficient (bisection; CL rises with alpha below the stall).
    Real alphaFor(Real cl, Real mach, Real elevator, Real stabilizer) {
        Real lo = degToRad(-4), hi = degToRad(14);
        for (int k = 0; k < 40; ++k) {
            const Real mid = 0.5 * (lo + hi);
            if (at(mach, mid, elevator, stabilizer).cl < cl) lo = mid;
            else hi = mid;
        }
        return 0.5 * (lo + hi);
    }
    // Elevator that zeroes the pitching moment at an angle of attack (bisection).
    Real elevatorFor(Real mach, Real alpha, Real stabilizer) {
        Real lo = -1, hi = 1;
        const Real mlo = at(mach, alpha, lo, stabilizer).cm;
        for (int k = 0; k < 40; ++k) {
            const Real mid = 0.5 * (lo + hi), m = at(mach, alpha, mid, stabilizer).cm;
            if ((m > 0) == (mlo > 0)) lo = mid;
            else hi = mid;
        }
        return 0.5 * (lo + hi);
    }
};

}  // namespace

TEST_CASE("transonic: trimmed at FL410, the Top G tucks nose down above Mcrit and the elevator per g rises") {
    Aircraft ac(jet());
    ac.setGearImmediately(false);
    SimpleWeather w;
    const StandardAtmosphere atm;
    TrimRequest req;
    req.altitude = ftToM(41000);
    req.tas = 0.80 * atm.at(req.altitude).speedOfSound;
    const TrimResult tr = trim(ac, req, w);
    CHECK(tr.converged);
    Longitudinal L(ac, 41000);
    const Real weight = ac.massProperties().mass * kStandardGravity;
    // The trimmed stabiliser (and whatever stick the trim left) held fixed,
    // the speed raised: at each Mach the 1-g angle of attack and the
    // out-of-trim moment there (+ nose up). A conventional airplane's speed
    // stability gives a nose-up moment as it accelerates; Mach tuck turns it
    // nose down once the shocks move the wing's lift aft.
    const Real stab = tr.trim * (tr.trim > 0 ? jet().controls.stabilizerNoseUp : jet().controls.stabilizerNoseDown);
    const Real stick = tr.elevator;
    // Swept to just past Mmo (0.925): beyond ~M0.94 the 1-g point at this
    // weight and altitude is itself inside the high-speed buffet, where the
    // lift is no longer monotonic in alpha.
    Real cm80 = 0, cmPrev = 0, cmMin = 1e9, onset = 0, epg80 = 0, epg92 = 0, cm92 = 0;
    std::printf("    M     alpha1g  Cm1g     elev/g\n");
    for (int i = 0; i <= 13; ++i) {
        const Real mach = 0.80 + 0.01 * i;
        const Real cl1 = weight / (L.at(mach, 0.03, stick, stab).q * jet().referenceArea);
        const Real a1 = L.alphaFor(cl1, mach, stick, stab);
        const Real cm = L.at(mach, a1, stick, stab).cm;
        // Elevator per g: the stick to trim at 1 g and at 1.3 g (alpha for the
        // higher lift at the trimmed stick, then the stick that trims there).
        const Real e1 = L.elevatorFor(mach, a1, stab);
        const Real a13 = L.alphaFor(1.3 * cl1, mach, e1, stab);
        const Real e13 = L.elevatorFor(mach, a13, stab);
        const Real epg = (e13 - e1) / 0.3;
        std::printf("    %.2f  %5.2f    %+.4f  %+.3f\n", mach, radToDeg(a1), cm, epg);
        if (i == 0) { cm80 = cm; epg80 = epg; }
        if (i > 0 && onset == 0 && cm < cmPrev - 1e-4) onset = mach;
        if (mach > 0.919 && mach < 0.921) { cm92 = cm; epg92 = epg; }
        cmMin = std::min(cmMin, cm);
        cmPrev = cm;
    }
    std::printf("    tuck onset M %.2f, Cm1g M0.80 %+.4f -> M0.92 %+.4f (min %+.4f), elevator/g %.3f -> %.3f\n", onset, cm80, cm92, cmMin,
                epg80, epg92);
    CHECK_RANGE(onset, 0.80, 0.90);       // past Mcrit (~0.80 at cruise CL), well before Mmo 0.925
    CHECK(cm92 < cm80 - 0.01);            // nose down relative to the trimmed point
    CHECK(cm92 < 0);                      // and nose down outright: a push is needed to hold it
    CHECK(epg80 > 0);                     // stable: aft stick for more g
    CHECK(epg92 > 1.3 * epg80);           // and heavier near Mmo
}

TEST_CASE("transonic: the Top G's 1-g CLmax at M0.85 is well below the low-speed value (buffet boundary)") {
    Aircraft ac(jet());
    const Vec3 cg = ac.massProperties().cg;
    const Real low = trimmedMaxLiftCoefficient(ac.aeroModel(), jet().referenceArea, cg, 0, 0.15);
    const Real m50 = trimmedMaxLiftCoefficient(ac.aeroModel(), jet().referenceArea, cg, 0, 0.50);
    const Real m85 = trimmedMaxLiftCoefficient(ac.aeroModel(), jet().referenceArea, cg, 0, 0.85);
    std::printf("    trimmed CLmax: M0.15 %.3f, M0.50 %.3f, M0.85 %.3f (%.0f %% of low speed)\n", low, m50, m85, 100 * m85 / low);
    CHECK(low > 1.1 && low < 1.4);        // the clean wing's ~1.24 (test_jet's stall speeds)
    CHECK(m50 > 0.9 * low);               // barely past the knee: the low-speed wing
    CHECK(m85 < 0.76 * low);              // at cruise Mach ~a quarter gone: buffet, not the low-speed stall
    // Of the class: a 1.3-g buffet margin at M0.85 means CL ~0.9 / 1.3 ~0.7 at 1 g.
    CHECK(m85 > 0.75);
}

TEST_CASE("transonic: elevator effectiveness at M0.92 is a fraction of that at M0.5") {
    Aircraft ac(jet());
    Longitudinal L(ac, 41000);
    auto power = [&](Real mach) {
        const Real a = degToRad(2);
        return (L.at(mach, a, 0.5, 0).cm - L.at(mach, a, -0.5, 0).cm);  // dCm over a unit of stick
    };
    const Real p50 = power(0.5), p80 = power(0.8), p92 = power(0.92);
    std::printf("    dCm per stick: M0.5 %.4f, M0.8 %.4f, M0.92 %.4f (%.0f %% of M0.5)\n", p50, p80, p92, 100 * p92 / p50);
    CHECK(p50 > 0);                       // stick aft (+) pitches nose up
    CHECK(p92 / p50 > 0.1 && p92 / p50 < 0.5);
    CHECK(p80 / p50 > 0.7);               // the cruise Mach keeps most of it
}
