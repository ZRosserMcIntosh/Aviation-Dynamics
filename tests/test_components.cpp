// Aviation Dynamics — component tests: aerodynamics, propulsion, systems, instruments.
// SPDX-License-Identifier: MIT
#include "avd/aero.hpp"
#include "avd/instruments.hpp"
#include "avd/propulsion.hpp"
#include "avd/systems.hpp"
#include "avd/units.hpp"
#include "avd_test.hpp"
#include "scenarios.hpp"

using namespace avd;
using namespace avd::units;

namespace {
const AircraftDefinition& ref() {
    static const AircraftDefinition d = scenarios::loadReference();
    return d;
}

AeroOutputs evalAero(const AeroModel& m, const Vec3& vBody, const Vec3& omega = {}, const ControlDeflections& c = {},
                     Real heightAgl = 1e9) {
    AeroOutputs out;
    m.prepare(out);
    AeroInputs in;
    in.velocityBody = vBody;
    in.omegaBody = omega;
    in.cg = {-1.05, 0, -0.15};
    in.controls = c;
    in.heightAboveGround = heightAgl;
    m.evaluate(in, out);
    return out;
}

Vec3 flow(Real v, Real alpha, Real beta = 0) {
    return {v * std::cos(alpha) * std::cos(beta), v * std::sin(beta), v * std::sin(alpha) * std::cos(beta)};
}
}  // namespace

// ---------------------------------------------------------------- section ----

TEST_CASE("aero: section polar spans -180..180 with stall and post-stall") {
    SectionPolar p;
    p.zeroLiftAlpha = degToRad(-2);
    p.clMax = 1.5;
    const Real a = 6.0;
    CHECK_NEAR(evaluateSection(p, p.zeroLiftAlpha, a).cl, 0, 1e-12);
    const Real slope = (evaluateSection(p, 0.05, a).cl - evaluateSection(p, 0.0, a).cl) / 0.05;
    CHECK_NEAR(slope, a, 1e-9);
    // Peak near clMax, then a lift break.
    Real peak = 0, peakAlpha = 0;
    for (Real al = 0; al < 0.6; al += 0.001) {
        const Real cl = evaluateSection(p, al, a).cl;
        if (cl > peak) { peak = cl; peakAlpha = al; }
    }
    CHECK_NEAR(peak, 1.5, 0.05);
    CHECK(evaluateSection(p, peakAlpha + 0.15, a).cl < 0.85 * peak);
    CHECK(evaluateSection(p, peakAlpha + 0.15, a).stalled);
    // Flat plate at 90 deg: no lift, maximum drag.
    const SectionResult s90 = evaluateSection(p, kPi / 2, a);
    CHECK_NEAR(s90.cl, 0, 1e-6);
    CHECK_NEAR(s90.cd, p.flatPlateCd, 0.01);
    // Inverted flight works (negative stall), reversed flow is finite.
    CHECK(evaluateSection(p, degToRad(-10), a).cl < -0.7);
    for (Real al = -kPi; al <= kPi; al += 0.01) {
        const SectionResult s = evaluateSection(p, al, a);
        CHECK(std::isfinite(s.cl) && std::isfinite(s.cd) && std::isfinite(s.cm) && s.cd > 0);
    }
    // A flap shifts lift up and raises clMax.
    CHECK(evaluateSection(p, 0.0, a, -0.1, 0.4).cl > evaluateSection(p, 0.0, a).cl + 0.5);
}

TEST_CASE("aero: finite-wing relations (Helmbold slope, flap tau, ground effect)") {
    // AR 7.5 unswept, a2d = 2 pi * 0.95: ~4.75 /rad.
    CHECK_RANGE(liftCurveSlope3D(7.5, 0, 2 * kPi * 0.95), 4.5, 5.0);
    CHECK(liftCurveSlope3D(7.5, 0.6, 6.0) < liftCurveSlope3D(7.5, 0, 6.0));  // sweep reduces slope
    CHECK(liftCurveSlope3D(7.5, 0, 6.0, 0.6) > liftCurveSlope3D(7.5, 0, 6.0));  // compressibility raises it
    CHECK_NEAR(flapEffectiveness(0.25), 0.60, 0.02);  // classic thin-airfoil value
    CHECK_NEAR(flapEffectiveness(1.0), 1.0, 1e-9);
    CHECK_NEAR(groundEffectFactor(1e9, 11), 1.0, 1e-9);
    CHECK_NEAR(groundEffectFactor(11.0 / 8, 11), 0.8, 1e-9);  // h = b/8: 4/5
    CHECK(groundEffectFactor(0.5, 11) < 0.4);
}

// ------------------------------------------------------------ airframe ----

namespace {
// A lone untwisted, untapered wing of aspect ratio 8 at a quarter-chord sweep
// (deg), with an optional full-span Fowler flap.
AircraftDefinition sweptWing(Real sweepDeg, Real chordExtension = 0) {
    AircraftDefinition d;
    LiftingSurfaceDef w;
    w.name = "wing";
    w.section.zeroLiftAlpha = degToRad(-3);
    w.section.clMax = 1.6;
    w.section.cdMin = 0.008;
    w.section.clAtCdMin = 0.2;
    PanelDef p;
    p.span = 8;
    p.rootChord = p.tipChord = 2;
    p.sweep = degToRad(sweepDeg);
    p.strips = 12;
    p.mirrored = true;
    if (chordExtension > 0) {
        ControlSurfaceDef f;
        f.name = "flap";
        f.channel = ControlChannel::Flap;
        f.chordFraction = 0.25;
        f.maxPositive = degToRad(30);
        f.chordExtension = chordExtension;
        p.controls.push_back(f);
    }
    w.panels.push_back(p);
    w.groundEffect = false;
    d.surfaces.push_back(w);
    return d;
}

struct WingPolar {
    Real alpha0 = 0, slope = 0, clMax = 0, cd0 = 0;
};
// Streamwise CL(alpha) of a lone wing at 60 m/s (lift and drag on the
// free-stream q and the reference area of the planform).
WingPolar wingPolar(const AircraftDefinition& d, Real flap = 0) {
    const AeroModel m(d);
    AeroOutputs out;
    m.prepare(out);
    AeroInputs in;
    in.controls.flap = flap;
    const Real v = 60, q = 0.5 * 1.225 * v * v;
    Real area = 0;
    for (int i = 0; i < m.stripCount(); ++i) area += m.stripGeometry(i).area;
    auto at = [&](Real a, Real& cd) {
        in.velocityBody = {v * std::cos(a), 0, v * std::sin(a)};
        m.evaluate(in, out);
        cd = -(out.force.x * std::cos(a) + out.force.z * std::sin(a)) / (q * area);
        return (out.force.x * std::sin(a) - out.force.z * std::cos(a)) / (q * area);
    };
    WingPolar w;
    Real cd = 0;
    const Real c1 = at(degToRad(-1), cd), c2 = at(degToRad(1), cd);
    w.slope = (c2 - c1) / degToRad(2);
    w.alpha0 = degToRad(-1) - c1 / w.slope;
    for (Real a = 0; a < 30; a += 0.1) w.clMax = std::max(w.clMax, at(degToRad(a), cd));
    at(w.alpha0, w.cd0);
    return w;
}
}  // namespace

TEST_CASE("aero: simple sweep theory - zero-lift angle and profile drag hold, CLmax falls as cos(sweep)") {
    const WingPolar w0 = wingPolar(sweptWing(0)), w30 = wingPolar(sweptWing(30)), w45 = wingPolar(sweptWing(45));
    // An untwisted wing's zero-lift angle is its section's at any sweep.
    CHECK_NEAR(radToDeg(w0.alpha0), -3, 0.05);
    CHECK_NEAR(radToDeg(w30.alpha0), -3, 0.05);
    CHECK_NEAR(radToDeg(w45.alpha0), -3, 0.05);
    // The lift slope is the finite-wing (Helmbold) slope for the sweep.
    const AeroModel m30(sweptWing(30));
    CHECK_NEAR(w30.slope, m30.surfaceLiftSlope(0), 0.02 * w30.slope);
    // Profile drag at zero lift: skin friction does not care about sweep.
    CHECK_NEAR(w30.cd0, w0.cd0, 0.05 * w0.cd0);
    CHECK_NEAR(w45.cd0, w0.cd0, 0.08 * w0.cd0);
    // CLmax ~ clmax cos(sweep) (Raymer 12.15 without the 0.9 span factor;
    // DATCOM 4.1.3.4 gives 0.8-0.9 at 30 deg).
    CHECK_RANGE(w30.clMax / w0.clMax, 0.82, 0.9);
    CHECK_RANGE(w45.clMax / w0.clMax, 0.66, 0.76);
}

TEST_CASE("aero: a Fowler flap's chord extension adds lift; post-stall drag is the finite wing's") {
    const WingPolar plain = wingPolar(sweptWing(0, 1e-9), degToRad(30)), fowler = wingPolar(sweptWing(0, 0.15), degToRad(30));
    CHECK_RANGE(fowler.clMax / plain.clMax, 1.12, 1.16);
    // Flat-plate drag of an AR 8 wing at 90 deg: Viterna's 1.11 + 0.018 AR, not the 2-D 1.9.
    CHECK_NEAR(finiteWingMaxDrag(8), 1.254, 1e-9);
    const AeroModel m(sweptWing(0));
    AeroOutputs out;
    m.prepare(out);
    AeroInputs in;
    in.velocityBody = {0, 0, 30};
    m.evaluate(in, out);
    Real area = 0;
    for (int i = 0; i < m.stripCount(); ++i) area += m.stripGeometry(i).area;
    const Real cn = -out.force.z / (0.5 * 1.225 * 900 * area);
    CHECK_RANGE(cn, 1.15, 1.35);
}

TEST_CASE("aero: reference airframe static stability and damping signs") {
    const AeroModel m(ref());
    const Real v = 50;
    // Symmetric flight: no lateral force or moments (no slipstream here).
    const AeroOutputs sym = evalAero(m, flow(v, 0.05));
    CHECK_NEAR(sym.force.y, 0, 1e-6);
    CHECK_NEAR(sym.moment.x, 0, 1e-6);
    CHECK_NEAR(sym.moment.z, 0, 1e-6);
    CHECK(sym.force.z < 0);  // lift acts up (-Z body)
    // Cm_alpha < 0 about a mid-envelope CG.
    const Real m1 = evalAero(m, flow(v, 0.02)).moment.y, m2 = evalAero(m, flow(v, 0.10)).moment.y;
    CHECK(m2 < m1);
    // Weathercock: sideslip from the right yaws the nose right (Cn_beta > 0).
    const AeroOutputs sl = evalAero(m, flow(v, 0.05, 0.08));
    CHECK(sl.moment.z > 0);
    CHECK(sl.force.y < 0);   // side force opposes the slip
    CHECK(sl.moment.x < 0);  // dihedral effect: Cl_beta < 0
    // Damping: roll rate, pitch rate and yaw rate are opposed.
    CHECK(evalAero(m, flow(v, 0.05), {0.5, 0, 0}).moment.x < -1000);
    CHECK(evalAero(m, flow(v, 0.05), {0, 0.3, 0}).moment.y < m1);
    CHECK(evalAero(m, flow(v, 0.05), {0, 0, 0.3}).moment.z < -100);
}

TEST_CASE("aero: control surfaces act in the commanded sense; ailerons show adverse yaw") {
    const AeroModel m(ref());
    const Vec3 vb = flow(45, 0.06);
    const AeroOutputs base = evalAero(m, vb);
    ControlDeflections c;
    c.aileron = 0.5;
    const AeroOutputs ail = evalAero(m, vb, {}, c);
    CHECK(ail.moment.x > base.moment.x + 500);  // roll right
    // Adverse yaw is defined in stability axes (body x rotated by alpha into
    // the flight path): N_s = N_b cos(a) - L_b sin(a).
    const Real a = 0.06;
    auto nStab = [a](const AeroOutputs& o) { return o.moment.z * std::cos(a) - o.moment.x * std::sin(a); };
    CHECK(nStab(ail) < nStab(base) - 50);  // adverse: nose left
    c = {};
    c.elevator = 0.5;
    CHECK(evalAero(m, vb, {}, c).moment.y > base.moment.y + 500);  // nose up
    c = {};
    c.rudder = 0.5;
    CHECK(evalAero(m, vb, {}, c).moment.z > base.moment.z + 200);  // nose right
    c = {};
    c.flap = degToRad(30);
    const AeroOutputs fl = evalAero(m, vb, {}, c);
    CHECK(-fl.force.z > -base.force.z * 1.3);  // more lift
    CHECK(fl.force.x < base.force.x);          // more drag
}

TEST_CASE("aero: ground effect, stall asymmetry and component loss") {
    const AeroModel m(ref());
    const Vec3 vb = flow(30, 0.10);
    const AeroOutputs free = evalAero(m, vb), ge = evalAero(m, vb, {}, {}, 1.2);
    CHECK(-ge.force.z > -free.force.z);  // more lift near the ground
    CHECK(ge.force.x > free.force.x);    // less induced drag
    // Rolling at high alpha: the descending (right) wing stalls first -> autorotation tendency.
    const AeroOutputs roll = evalAero(m, flow(28, degToRad(15)), {0.8, 0, 0});
    int right = 0, left = 0;
    for (int i = m.firstStrip(0); i < m.endStrip(0); ++i) {
        // Strip order: panel0 right(3) left(3), panel1 right(5) left(5).
        const int k = i - m.firstStrip(0);
        const bool isRight = (k < 3) || (k >= 6 && k < 11);
        if (roll.strips[i].stalled) (isRight ? right : left)++;
    }
    CHECK(right > left);
    // Losing the left wing removes its lift and produces a large rolling moment.
    AeroModel damaged(ref());
    damaged.setSurfaceAttached(0, -1, false);
    const AeroOutputs d = evalAero(damaged, flow(50, 0.05));
    const AeroOutputs h = evalAero(m, flow(50, 0.05));
    CHECK(-d.force.z < 0.65 * -h.force.z);
    CHECK(d.moment.x < -5000);  // rolls toward the missing (left) wing
}

// ----------------------------------------------------------- propulsion ----

TEST_CASE("propeller: BEM thrust/torque across static, cruise, windmill and stopped") {
    const Propeller p(ref().engines[0].propeller);
    const Real w = rpmToRadS(2350);
    const PropellerForces st = p.evaluate(0, w, 1.225);
    CHECK_RANGE(st.thrust, 2200, 3600);       // static thrust of a 180 hp fixed-pitch prop
    CHECK_RANGE(st.torque * w, 80e3, 140e3);  // absorbs most of the engine's power
    const PropellerForces cr = p.evaluate(60, rpmToRadS(2500), 0.96);
    CHECK(cr.thrust < st.thrust);
    CHECK_RANGE(cr.efficiency, 0.75, 0.90);
    // High advance ratio at low rpm: windmilling (negative thrust, negative torque).
    const PropellerForces wm = p.evaluate(60, rpmToRadS(800), 1.225);
    CHECK(wm.thrust < 0);
    CHECK(wm.torque < 0);
    // Stopped propeller: drag only, finite.
    const PropellerForces sp = p.evaluate(50, 0, 1.225);
    CHECK(sp.thrust < 0);
    CHECK(std::isfinite(sp.torque));
    // Tables are smooth and finite over the advance-angle domain.
    for (Real b = -0.3; b < 1.6; b += 0.01) CHECK(std::isfinite(p.thrustTable()(b, 0.4)) && std::isfinite(p.torqueTable()(b, 0.4)));
}

TEST_CASE("propeller: swirl and tip compressibility behave physically") {
    const PropellerDef& def = ref().engines[0].propeller;
    const Propeller p(def);
    // Static figure of merit T^1.5 / (P sqrt(2 rho A)) of a cruise-pitched
    // fixed-pitch propeller: ~0.35-0.55 (a C172 O-320 test measured ~0.37).
    const Real w = rpmToRadS(2350);
    const PropellerForces st = p.evaluate(0, w, 1.225);
    const Real fm = std::pow(st.thrust, 1.5) / (st.torque * w * std::sqrt(2 * 1.225 * p.discArea()));
    CHECK_RANGE(fm, 0.35, 0.60);
    // Same advance angle, faster tips: the drag rise costs torque per unit
    // thrust (efficiency falls) once the tips pass their critical Mach.
    const Real beta = 0.35;
    const Real eLow = p.thrustTable()(beta, 0.4) / p.torqueTable()(beta, 0.4);
    const Real eHigh = p.thrustTable()(beta, 0.85) / p.torqueTable()(beta, 0.85);
    CHECK(eHigh < 0.95 * eLow);
    // Without thickness data the propeller is incompressible.
    PropellerDef inc = def;
    inc.thicknessVsRadius = Table1D{};
    const Propeller pi(inc);
    CHECK_NEAR(pi.thrustTable()(beta, 0.4), pi.thrustTable()(beta, 0.85), 1e-12);
    // The coupled axial + swirl solution converges at static (direct solver).
    Real t, q;
    Propeller::bem(def, 0, w, 1.225, t, q);
    CHECK(t > 0 && q > 0);
    // ... and where the tip drag rise dominates the torque (tip Mach ~1.4,
    // the M = 1.0 table column), from slight reverse flow to climb speed:
    // thrust falls and torque rises smoothly with axial speed. An
    // under-relaxed fixed point on the swirl oscillated here.
    const Real omegaHigh = 503.8;
    Real tPrev = 1e30, qPrev = 0;
    for (Real v = -12; v <= 20; v += 2) {
        Propeller::bem(def, v, omegaHigh, 1.0, t, q, 340.294);
        CHECK(t < tPrev);
        CHECK(q > qPrev);
        tPrev = t;
        qPrev = q;
    }
    // Identical definitions share the solved tables (cache) bit for bit.
    const Propeller p2(def);
    CHECK(p2.thrustTable()(beta, 0.6) == p.thrustTable()(beta, 0.6));
}

TEST_CASE("piston engine: rated power, altitude lapse, mixture, ignition and fuel") {
    const PistonEngine e(ref().engines[0]);
    PistonEngineInputs in;
    in.throttle = 1;
    in.magnetos = Magnetos::Both;
    in.fuelAtEngine = true;
    in.mixture = scenarios::bestPowerMixture(ref(), 1.225);
    const Real wr = rpmToRadS(2700);
    const PistonEngineState sl = e.evaluate(in, wr);
    CHECK_NEAR(sl.brakePower, 134200, 1500);  // calibrated: rated power at rated rpm
    CHECK(sl.firing);
    // BSFC at best power: 0.42-0.52 lb/hp/h; fuel flow ~13-16 US gal/h.
    const Real bsfc = kgToLb(sl.fuelFlow * 3600) / wToHp(sl.brakePower);
    CHECK_RANGE(bsfc, 0.42, 0.54);
    CHECK_RANGE(m3ToGal(sl.fuelFlow / 720.0) * 3600, 12.0, 17.0);
    // 8000 ft full throttle: ~70-78% (normally aspirated lapse).
    const AirState a8 = StandardAtmosphere().at(ftToM(8000));
    PistonEngineInputs hi = in;
    hi.ambientPressure = a8.pressure;
    hi.ambientTemperature = a8.temperature;
    hi.ambientDensity = a8.density;
    hi.mixture = scenarios::bestPowerMixture(ref(), a8.density);
    CHECK_RANGE(e.evaluate(hi, wr).brakePower / sl.brakePower, 0.68, 0.80);
    // Temperature at a pressure altitude: the heated charge damps the power's
    // sensitivity to ambient temperature from the ideal-gas 1/T toward the
    // sqrt(T_std/T) of Lycoming's manuals; the exponent follows from the
    // definition's charge heating. Sea level, ISA+20, full throttle, rated rpm.
    const PistonEngineDef& d = ref().engines[0];
    PistonEngineInputs hot = in;
    hot.ambientTemperature = isa::kSeaLevelTemperature + 20;
    hot.ambientDensity = isa::kSeaLevelDensity * isa::kSeaLevelTemperature / hot.ambientTemperature;
    hot.mixture = scenarios::bestPowerMixture(ref(), hot.ambientDensity);
    const Real hotRatio = e.evaluate(hot, wr).brakePower / sl.brakePower;
    const Real theta = hot.ambientTemperature / isa::kSeaLevelTemperature;
    const Real eps = d.chargeHeating / (d.chargeWallTemperature - isa::kSeaLevelTemperature);
    const Real n = (1 - eps) * isa::kSeaLevelTemperature / (isa::kSeaLevelTemperature + d.chargeHeating);
    CHECK_RANGE(n, 0.5, 0.85);
    CHECK_NEAR(hotRatio, 1.132 * std::pow(theta, -n) - 0.132, 0.01);
    CHECK(hotRatio > 1.132 / theta - 0.132 + 0.01);                 // damped against the ideal gas
    CHECK(hotRatio < 1.132 / std::sqrt(theta) - 0.132);             // but not below the sqrt rule
    // The heating is normalised out at sea-level ISA: the rated point is untouched.
    CHECK_NEAR(e.chargeTemperature(isa::kSeaLevelTemperature), isa::kSeaLevelTemperature, 1e-9);
    // Full rich at altitude is too rich: less power than best-power mixture.
    PistonEngineInputs rich = hi;
    rich.mixture = 1.0;
    CHECK(e.evaluate(rich, wr).brakePower < e.evaluate(hi, wr).brakePower);
    // Single magneto: small rpm-drop-sized power loss.
    PistonEngineInputs mag = in;
    mag.magnetos = Magnetos::Left;
    CHECK_RANGE(e.evaluate(mag, wr).brakePower / sl.brakePower, 0.94, 0.99);
    // Idle cut-off, magnetos off, no fuel: no combustion.
    PistonEngineInputs cut = in;
    cut.mixture = 0;
    CHECK(!e.evaluate(cut, wr).firing);
    CHECK_NEAR(e.evaluate(cut, wr).fuelFlow, 0, 0);
    PistonEngineInputs off = in;
    off.magnetos = Magnetos::Off;
    CHECK(!e.evaluate(off, wr).firing);
    CHECK(e.evaluate(off, wr).brakeTorque < 0);  // friction only
    PistonEngineInputs dry = in;
    dry.fuelAtEngine = false;
    CHECK(!e.evaluate(dry, wr).firing);
    // Starter cranks against friction.
    PistonEngineInputs crank = in;
    crank.magnetos = Magnetos::Start;
    crank.starterPowered = true;
    const PistonEngineState cs = e.evaluate(crank, rpmToRadS(100));
    CHECK(cs.starterTorque + cs.brakeTorque > 0);
}

TEST_CASE("piston engine: induction model (throttle orifice, idle orifice, residual gas, pumping)") {
    const PistonEngineDef& d = ref().engines[0];
    const PistonEngine e(d);
    const Real p0 = isa::kSeaLevelPressure, T0 = isa::kSeaLevelTemperature;
    const Real wr = rpmToRadS(d.ratedRpm), wi = rpmToRadS(d.idleRpm);
    // Full throttle at rated rpm: the definition's intake loss; at a lower
    // rpm the drop shrinks with the square of the air flow.
    CHECK_NEAR(e.manifoldPressure(1, wr, p0, T0) / p0, 1 - d.intakeLossRatio, 1e-6);
    CHECK(e.manifoldPressure(1, rpmToRadS(1800), p0, T0) > e.manifoldPressure(1, wr, p0, T0));
    CHECK(e.manifoldPressure(1, rpmToRadS(1800), p0, T0) < p0);
    // Closed throttle: the idle orifice gives the definition's MAP at the idle
    // rpm, and above it the choked flow is fixed, so MAP falls with rpm
    // toward the residual-gas floor (ambient / compression ratio).
    CHECK_NEAR(e.manifoldPressure(0, wi, p0, T0) / p0, d.idleManifoldRatio, 1e-6);
    const Real m1200 = e.manifoldPressure(0, rpmToRadS(1200), p0, T0), m2400 = e.manifoldPressure(0, rpmToRadS(2400), p0, T0);
    CHECK(m1200 < d.idleManifoldRatio * p0);
    CHECK(m2400 < m1200);
    CHECK(m2400 > p0 / d.compressionRatio);
    // Fixed choked air flow: eta_v MAP omega is the same at 1,200 and 2,400 rpm.
    const Real flow1200 = e.volumetricEfficiency(m1200, p0, rpmToRadS(1200)) * m1200 * 1200;
    const Real flow2400 = e.volumetricEfficiency(m2400, p0, rpmToRadS(2400)) * m2400 * 2400;
    CHECK_NEAR(flow2400 / flow1200, 1.0, 0.01);
    // Stopped engine: no pumping, MAP = ambient. Half lever: well below ambient at cruise rpm.
    CHECK_NEAR(e.manifoldPressure(0.5, 0, p0, T0), p0, 1.0);
    CHECK_RANGE(e.manifoldPressure(0.5, rpmToRadS(2300), p0, T0) / p0, 0.65, 0.85);
    // Volumetric efficiency is the definition's at the rated point and falls
    // both with speed (port losses) and with the exhaust/intake ratio (residual gas).
    CHECK_NEAR(e.volumetricEfficiency((1 - d.intakeLossRatio) * p0, p0, wr), d.volumetricEfficiency, 1e-9);
    CHECK(e.volumetricEfficiency((1 - d.intakeLossRatio) * p0, p0, rpmToRadS(1800)) > d.volumetricEfficiency);
    CHECK(e.volumetricEfficiency(0.3 * p0, p0, wr) < 0.8 * d.volumetricEfficiency);
    // Windmilled at 1,500 rpm with the throttle closed the firing engine is a
    // brake (indicated power below friction plus pumping); dead, more so.
    PistonEngineInputs in;
    in.throttle = 0;
    in.magnetos = Magnetos::Both;
    in.fuelAtEngine = true;
    in.mixture = 1;
    const PistonEngineState idle = e.evaluate(in, rpmToRadS(1500));
    CHECK(idle.firing);
    CHECK(idle.brakeTorque < 0);
    in.mixture = 0;
    CHECK(e.evaluate(in, rpmToRadS(1500)).brakeTorque < idle.brakeTorque);
    // Rated power still calibrated with the pumping loop included.
    in.mixture = scenarios::bestPowerMixture(ref(), 1.225);
    in.throttle = 1;
    CHECK_NEAR(e.evaluate(in, wr).brakePower, d.ratedPower, 1.0);
}

// -------------------------------------------------------------- systems ----

TEST_CASE("fuel system: selector, usable fuel, shutoff valve, line and leaks") {
    FuelSystem f(ref().fuel);
    const Real demand = 0.01;  // kg/s
    f.setSelector(0);         // LEFT
    for (int i = 0; i < 1000; ++i) f.step(0.1, demand, true);
    CHECK(f.tankQuantity(0) < f.tankQuantity(1));
    CHECK_NEAR(f.tankQuantity(1), ref().fuel.tanks[1].initial, 1e-12);
    // BOTH draws evenly.
    FuelSystem b(ref().fuel);
    b.setSelector(1);
    for (int i = 0; i < 1000; ++i) b.step(0.1, demand, true);
    CHECK_NEAR(b.tankQuantity(0), b.tankQuantity(1), 1e-9);
    // Mass is conserved: fuel removed from tanks + line = fuel delivered.
    CHECK_NEAR((2 * 0.106 + ref().fuel.lineVolume - b.totalQuantity() - b.lineQuantity()) * 720, demand * 100, 1e-6);
    // Unusable fuel stays in the tank; the line then drains and the engine starves.
    FuelSystem s(ref().fuel);
    s.setTankQuantity(0, 0.0057);  // 5.7 L: just above the unusable 5.68 L
    s.setSelector(0);
    int stepsToStarve = 0;
    while (s.fuelAtEngine() && stepsToStarve < 100000) { s.step(0.1, demand, true); ++stepsToStarve; }
    CHECK(!s.fuelAtEngine());
    CHECK_NEAR(s.tankQuantity(0), ref().fuel.tanks[0].unusable, 1e-9);
    CHECK(s.starving());
    // Shutoff valve: the engine runs only on line fuel, then starves.
    FuelSystem v(ref().fuel);
    v.setShutoffOpen(false);
    Real t = 0;
    while (v.fuelAtEngine() && t < 100) { v.step(0.1, demand, true); t += 0.1; }
    CHECK_NEAR(t, ref().fuel.lineVolume * 720 / demand, 0.2);
    // Leak drains even with valves closed.
    FuelSystem l(ref().fuel);
    l.setLeakRate(1, 1e-4);
    l.step(10, 0, false);
    CHECK_NEAR(l.tankQuantity(1), 0.106 - 1e-3, 1e-12);
}

TEST_CASE("electrical system: master switches, alternator, starter load and depletion") {
    ElectricalSystem e(ref().electrical, 900);
    e.setAvionicsSwitch(true);
    e.step(0.1, {});
    CHECK_NEAR(e.busVoltage(), 0, 0);  // cold and dark
    e.setBatterySwitch(true);
    e.step(0.1, {});
    CHECK_RANGE(e.busVoltage(), 24, 26);
    // Starter current sags the bus.
    ElectricalInputs crank;
    crank.starterCurrent = 150;
    e.step(0.1, crank);
    CHECK(e.busVoltage() < 22);
    CHECK(e.busVoltage() > 16);
    // Alternator needs its switch, field voltage and rpm.
    ElectricalInputs run;
    run.alternatorRpm = 2000;
    e.step(0.1, run);
    CHECK(!e.alternatorOnline());
    e.setAlternatorSwitch(true);
    e.step(0.1, run);
    CHECK(e.alternatorOnline());
    CHECK_NEAR(e.busVoltage(), 28, 1e-9);
    CHECK(e.batteryCurrent() < 0);  // charging
    // Alternator failure: the battery carries the loads until it is flat.
    e.setAlternatorFailed(true);
    Real t = 0;
    while (e.busVoltage() > 18 && t < 4 * 3600) { e.step(1, run); t += 1; }
    CHECK_RANGE(t / 60, 20, 200);  // 12.75 Ah at ~10.5 A of loads
    const int tc = e.loadIndex("turn_coordinator");
    CHECK(tc >= 0);
    CHECK(!e.loadPowered(tc));
    // A battery that cannot hold the starter above 16 V cannot crank.
    ElectricalSystem weak(ref().electrical, 900);
    weak.setBatterySwitch(true);
    weak.setBatteryCharge(0.03);
    weak.step(0.1, crank);
    CHECK(weak.busVoltage() < 16);
}

TEST_CASE("vacuum system: suction follows the engine-driven pump") {
    VacuumSystem v(ref().vacuum);
    v.step(0.1, 2400);
    CHECK_NEAR(v.suction(), inHgToPa(5.0), 1);
    v.step(0.1, 750);
    CHECK_NEAR(v.suction(), inHgToPa(5.0) * 0.5, 1);
    v.setPumpFailed(true);
    v.step(0.1, 2400);
    CHECK(v.suction() == 0);
}
