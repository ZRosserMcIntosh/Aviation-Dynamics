// Aviation Dynamics — ground effect on the whole airplane: lift, induced drag
// and pitching moment at a fixed attitude and airspeed as the runway comes up.
// SPDX-License-Identifier: MIT
#include <cstdio>

#include "avd/units.hpp"
#include "avd_test.hpp"
#include "scenarios.hpp"

using namespace avd;
using namespace avd::scenarios;
using namespace avd::units;

namespace {

struct GePoint {
    Real wheelHeight = 0, wingHeight = 0;
    Real lift = 0, drag = 0, pitchMoment = 0;  // aerodynamic only, N and N m (flight-path axes; + nose up)
    Real inducedK = 0;                         // dD/d(L^2) over +-1 deg of pitch: the lift-dependent drag factor
    Real tailDownwash = 0;                     // rad
    Real qS = 0;
};

// The airplane trimmed in free air (gear down, flaps at `flap`, level at
// `kias`), then held at the same attitude, airspeed and air density with its
// lowest wheel `h` above a flat runway: the runway is moved, not the airplane.
std::vector<GePoint> groundSweep(const AircraftDefinition& def, int flap, Real kias, const std::vector<Real>& heights) {
    Aircraft ac(def);
    SimpleWeather atm;
    ac.setGearImmediately(true);
    TrimRequest req;
    req.altitude = 300;
    req.tas = tasFromCas(ktToMs(kias), atm.atmosphere().at(300.0));
    req.flapDetent = flap;
    const TrimResult tr = trim(ac, req, atm);
    CHECK(tr.converged);
    ac.setQuasiSteadyDownwash(true);
    const Vec3 cg = ac.massProperties().cg;
    const Quat q0 = tr.state.body.attitude;
    Real lowest = -1e9;  // the lowest tyre (full extension) below the CG at the trimmed attitude
    for (const GearDef& gd : def.gear) lowest = std::max(lowest, q0.rotate(gd.attach + Vec3{0, 0, gd.length} - cg).z);
    const Real wingBelowCg = q0.rotate(ac.aeroModel().surfaceCentroid(0) - cg).z;
    // Forces at the trimmed velocity, the attitude pitched by dTheta.
    auto at = [&](Real h, Real dTheta, GePoint& p) {
        DynamicState s = tr.state;
        s.body.attitude = q0 * Quat::fromAxisAngle({0, 1, 0}, dTheta);
        const FlatGround ground(s.body.position.z + lowest + h);
        ac.setState(s);
        ac.prepareContext(tr.input, ground, atm);
        ac.evaluate(s);
        const Vec3 fWorld = s.body.attitude.rotate(ac.aero().force);
        p.drag = -dot(fWorld, normalized(s.body.velocity));  // level flight: drag along -V, lift up
        p.lift = -fWorld.z;
        p.pitchMoment = ac.aero().moment.y;
        p.tailDownwash = ac.aero().surfaces[1].downwash;
        p.wingHeight = lowest + h - wingBelowCg;
        p.qS = ac.aero().dynamicPressure * def.referenceArea;
    };
    std::vector<GePoint> out;
    for (Real h : heights) {
        GePoint p, lo, hi;
        at(h, degToRad(-1.0), lo);
        at(h, degToRad(+1.0), hi);
        at(h, 0, p);
        p.wheelHeight = h;
        p.inducedK = (hi.drag - lo.drag) / (sq(hi.lift) - sq(lo.lift));
        out.push_back(p);
    }
    return out;
}

}  // namespace

TEST_CASE("ground effect: lift rises, induced drag falls by McCormick's factor and the nose pitches down as the runway comes up") {
    struct Case {
        const char* id;
        int flap;
        Real kias;
        Real maxLiftGain;
        bool lowWing;  // the high wing's tail downwash falls less than its CL rises
    };
    for (const Case& c : {Case{"top_g", 3, 125, 0.12, true}, Case{"tupi_t180", 3, 62, 0.08, false}}) {
        const AircraftDefinition def = loadReference(c.id);
        const std::vector<GePoint> r = groundSweep(def, c.flap, c.kias, {300, 30, 10, 5, 2});
        const GePoint& free = r[0];
        const Real b = def.referenceSpan, chord = def.referenceChord;
        auto mcCormick = [&](Real h) {
            const Real x = sq(16 * h / b);
            return x / (1 + x);
        };
        for (const GePoint& p : r)
            std::fprintf(stderr,
                         "  %-9s wheels %5.1f m (wing %5.1f m, h/b %.2f): lift %+5.2f %%, drag %+5.2f %%, induced factor %.3f (McCormick %.3f), "
                         "dCm %+.4f, tail downwash %.2f deg\n",
                         c.id, p.wheelHeight, p.wingHeight, p.wingHeight / b, 100 * (p.lift / free.lift - 1), 100 * (p.drag / free.drag - 1),
                         p.inducedK / free.inducedK, mcCormick(p.wingHeight), (p.pitchMoment - free.pitchMoment) / (free.qS * chord),
                         radToDeg(p.tailDownwash));
        // Every step down gains lift.
        for (std::size_t k = 1; k < r.size(); ++k) CHECK(r[k].lift > r[k - 1].lift - 1e-6 * free.lift);
        const GePoint& g2 = r.back();
        // A few per cent of lift at 2 m (the bound vortex's image made it a
        // 4-10 % LOSS on the Top G), more than at 10 m.
        CHECK_RANGE(g2.lift / free.lift - 1, 0.01, c.maxLiftGain);
        CHECK(g2.lift > r[2].lift);
        // The lift-dependent drag factor falls between the classical factors:
        // McCormick's (16h/b)^2 / (1 + (16h/b)^2) and Wieselsberger's
        // 1 - (1 - 1.32 h/b) / (1.05 + 7.4 h/b) (fitted for h/b 0.033-0.25;
        // Hoerner, "Fluid-Dynamic Lift", ch. 9; McCormick ch. 9), which brackets it.
        const Real hb = g2.wingHeight / b;
        const Real wieselsberger = 1 - (1 - 1.32 * std::min(hb, 0.25)) / (1.05 + 7.4 * std::min(hb, 0.25));
        CHECK(g2.inducedK < free.inducedK);
        CHECK_RANGE(g2.inducedK / free.inducedK, wieselsberger - 0.05, mcCormick(g2.wingHeight) + 0.05);
        CHECK(g2.drag < free.drag);
        // A nose-down change (less downwash at the tail), a few hundredths of Cm.
        CHECK_RANGE((g2.pitchMoment - free.pitchMoment) / (free.qS * chord), -0.08, 0.0);
        if (c.lowWing) CHECK(g2.tailDownwash < free.tailDownwash);
    }
}
