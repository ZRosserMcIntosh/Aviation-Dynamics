// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
//
// References:
//  - Strip theory / uniform-downwash lifting line: Anderson, "Fundamentals of
//    Aerodynamics", ch. 5; McCormick, "Aerodynamics, Aeronautics and Flight
//    Mechanics" (ground effect factor (16h/b)^2 / (1 + (16h/b)^2)).
//  - 3-D lift slope: Helmbold equation as given in USAF DATCOM 4.1.3.2.
//  - Post-stall: Viterna & Corrigan (1982), NASA CP-2230 (finite-wing CDmax
//    = 1.11 + 0.018 AR).
//  - Swept strips: simple sweep theory (the section works on the flow normal
//    to its span; Anderson ch. 10, Raymer §12.4.1: CLmax ~ clmax cos(sweep)),
//    flap increments with the hinge-line sweep (DATCOM 6.1.4.1).
//  - Flap effectiveness tau and pitching moment: thin-airfoil theory (Abbott &
//    von Doenhoff, "Theory of Wing Sections", §8).
//  - Drag rise: Korn equation + Lock's fourth-power law (Mason, "Transonic
//    aerodynamics of airfoils and wings", Virginia Tech notes).
//  - Slender-body Munk moment: Munk (1924), NACA TR-184.
//  - Transonic section behaviour: aerodynamic-centre travel from c/4 toward
//    c/2 through the transonic range and the loss of flap effectiveness once
//    the shock stands ahead of the hinge (Hilton, "High-Speed Aerodynamics",
//    1951; Raymer §12.4.2 on the pitch-down "tuck"); the buffet boundary as a
//    clmax falling with the normal Mach number above a knee (Obert,
//    "Aerodynamic Design of Transport Aircraft", ch. 20: CL_buffet vs M).
//  - Tail in the stalled wing's wake (deep stall): Taylor & Ray, "A
//    systematic study of the factors contributing to post-stall longitudinal
//    stability of T-tail transport configurations", AIAA 65-737; the wake
//    centreline from the mid-chord along alpha less the downwash, the
//    separated region spreading aft, reduced tail q inside it.
#include "avd/aero.hpp"

#include <stdexcept>

#include "avd/atmosphere.hpp"
#include "avd/error.hpp"

namespace avd {

namespace {

constexpr Real kHalfPi = kPi / 2;

struct Viterna {
    Real a1, a2, b1, b2;
    Viterna(Real alphaS, Real clS, Real cdS, Real cdMax) {
        const Real s = std::sin(alphaS), c = std::cos(alphaS);
        b1 = cdMax;
        a1 = b1 / 2;
        b2 = (cdS - cdMax * s * s) / c;
        a2 = (clS - cdMax * s * c) * s / (c * c);
    }
    // alpha in [alphaS, pi/2]
    Real cl(Real a) const {
        const Real s = std::max(std::sin(a), 1e-6), c = std::cos(a);
        return a1 * std::sin(2 * a) + a2 * c * c / s;
    }
    Real cd(Real a) const { return b1 * std::sin(a) * std::sin(a) + b2 * std::cos(a); }
};

// Forward-flow half: alpha in [-pi/2, pi/2].
SectionResult forwardSection(const SectionPolar& p, Real alpha, Real a, Real alpha0, Real clMaxE, Real clMinE, Real cdMax) {
    SectionResult r;
    const Real clLin = a * (alpha - alpha0);
    const Real alphaSp = alpha0 + clMaxE / a;
    const Real alphaSm = alpha0 + clMinE / a;
    auto attachedCd = [&](Real cl) { return p.cdMin + p.cdQuadratic * sq(cl - p.clAtCdMin); };
    const Real w = p.stallWidth;

    if (alpha >= alphaSp) {
        const Real clAtt = std::min(clLin, clMaxE);
        const Real cdS = attachedCd(clMaxE);
        const Real as = std::min(alphaSp, kHalfPi - 0.05);
        const Viterna v(as, clMaxE * (1 - p.postStallClDrop), cdS, cdMax);
        const Real av = std::max(alpha, as);
        const Real blend = 1 - smoothstep(alphaSp, alphaSp + w, alpha);
        r.cl = blend * clAtt + (1 - blend) * v.cl(av);
        r.cd = blend * attachedCd(clAtt) + (1 - blend) * v.cd(av);
        r.stalled = alpha > alphaSp + 0.5 * w;
        r.cm = blend;  // temporarily: attached weight
        return r;
    }
    if (alpha <= alphaSm) {
        const Real clAtt = std::max(clLin, clMinE);
        const Real cdS = attachedCd(clMinE);
        const Real as = std::min(-alphaSm, kHalfPi - 0.05);
        const Viterna v(as, -clMinE * (1 - p.postStallClDrop), cdS, cdMax);
        const Real av = std::max(-alpha, as);
        const Real blend = 1 - smoothstep(-alphaSm, -alphaSm + w, -alpha);
        r.cl = blend * clAtt - (1 - blend) * v.cl(av);
        r.cd = blend * attachedCd(clAtt) + (1 - blend) * v.cd(av);
        r.stalled = alpha < alphaSm - 0.5 * w;
        r.cm = blend;
        return r;
    }
    r.cl = clLin;
    r.cd = attachedCd(clLin);
    r.cm = 1;
    return r;
}

}  // namespace

Real finiteWingMaxDrag(Real aspectRatio) { return 1.11 + 0.018 * clamp(aspectRatio, 0.0, 50.0); }

SectionResult evaluateSection(const SectionPolar& p, Real alpha, Real a, Real dAlpha0, Real dClMax, Real cdMax) {
    alpha = wrapPi(alpha);
    if (cdMax <= 0) cdMax = p.flatPlateCd;
    SectionResult r;
    const Real dCl = -a * dAlpha0;
    if (std::abs(alpha) <= kHalfPi) {
        const Real alpha0 = p.zeroLiftAlpha + dAlpha0;
        r = forwardSection(p, alpha, a, alpha0, p.clMax + dClMax, p.clMin + dCl, cdMax);
    } else {
        // Reversed flow: the trailing edge leads. Treat as a weaker symmetric
        // section (common practice for wind-turbine/rotor polars).
        const Real ar = alpha > 0 ? alpha - kPi : alpha + kPi;
        SectionPolar rev = p;
        rev.zeroLiftAlpha = 0;
        rev.clAtCdMin = 0;
        r = forwardSection(rev, ar, a, 0, 0.7 * p.clMax, -0.7 * p.clMax, cdMax);
        r.cl *= 0.7;
        r.cm = 0;  // attached weight 0: use the centre-of-pressure model below
        r.stalled = true;
    }
    // Pitching moment about the quarter chord: section cm0 when attached,
    // centre of pressure moving from c/4 to c/2 (90 deg) to 3c/4 (180 deg).
    const Real attachedWeight = r.cm;
    const Real cn = r.cl * std::cos(alpha) + r.cd * std::sin(alpha);
    const Real xcp = 0.25 + 0.5 * std::abs(alpha) / kPi;
    const Real cmPost = -cn * (xcp - 0.25);
    r.cm = attachedWeight * p.cm0 + (1 - attachedWeight) * cmPost;
    r.attached = attachedWeight;
    return r;
}

Real flapEffectiveness(Real cf) {
    cf = clamp(cf, 0.0, 1.0);
    const Real theta = std::acos(clamp(2 * cf - 1, -1.0, 1.0));
    return 1 - (theta - std::sin(theta)) / kPi;
}

Real liftCurveSlope3D(Real ar, Real sweep, Real a2d, Real beta) {
    const Real kappa = a2d / kTwoPi;
    const Real t = std::tan(sweep);
    return kTwoPi * ar / (2 + std::sqrt(sq(ar * beta / kappa) * (1 + t * t / (beta * beta)) + 4));
}

Real groundEffectFactor(Real h, Real b) {
    if (b <= 0 || h <= 0) return h <= 0 ? 0.05 : 1.0;
    const Real x = sq(16 * h / b);
    return std::max(x / (1 + x), 0.05);
}

AeroModel::AeroModel(const AircraftDefinition& def) : fuselages_(def.fuselages), dragItems_(def.dragItems) {
    if (def.surfaces.size() > 8) AVD_FAIL(std::invalid_argument, "at most 8 lifting surfaces");
    for (std::size_t s = 0; s < def.surfaces.size(); ++s) {
        Surface surf;
        surf.def = def.surfaces[s];
        surf.firstStrip = static_cast<int>(strips_.size());
        surfaces_.push_back(surf);
        Real sweepArea = 0, spanSum = 0;
        bool anyVertical = false, anyMirrored = false;
        for (const PanelDef& p : def.surfaces[s].panels) {
            addPanel(static_cast<int>(s), p, +1, spanSum);
            if (p.mirrored) addPanel(static_cast<int>(s), p, -1, spanSum);
            const Real area = 0.5 * (p.rootChord + p.tipChord) * (p.vertical ? p.span : p.span / std::cos(p.dihedral)) * (p.mirrored ? 2 : 1);
            sweepArea += p.sweep * area;
            spanSum += p.span;
            anyVertical = anyVertical || p.vertical;
            anyMirrored = anyMirrored || p.mirrored;
        }
        Surface& S = surfaces_.back();
        S.endStrip = static_cast<int>(strips_.size());
        Vec3 c, nrm;
        for (int i = S.firstStrip; i < S.endStrip; ++i) {
            S.area += strips_[i].area;
            c += strips_[i].position * strips_[i].area;
            nrm += strips_[i].normalAxis * strips_[i].area;
        }
        S.centroid = S.area > 0 ? c / S.area : Vec3{};
        if (length(nrm) > 1e-6) S.meanNormal = normalized(nrm);
        S.sweep = S.area > 0 ? sweepArea / S.area : 0;
        S.horizontal = !anyVertical;
        S.span = anyMirrored ? 2 * spanSum : spanSum;
        // A fin on a fuselage behaves like a surface of ~1.55x its geometric
        // aspect ratio (end-plate effect of fuselage and tailplane; DATCOM 5.3).
        S.aspectRatio = S.area > 0 ? sq(S.span) / S.area * (anyVertical && !anyMirrored ? 1.55 : 1.0) : 0;
        S.liftSlope = liftCurveSlope3D(S.aspectRatio, S.sweep, S.def.section.liftSlope);
        S.meanChord = S.span > 0 ? S.area / S.span : 0;
        // Sweep stall progression: the section's lift limits scaled along the
        // span by 1 - k sin(sweep) (eta - 1/2), on the strips of the surface's
        // main plane (a winglet's boundary layer is its own).
        if (S.def.sweepStallGradient != 0 && spanSum > 0)
            for (int i = S.firstStrip; i < S.endStrip; ++i) {
                Strip& st = strips_[static_cast<std::size_t>(i)];
                if (dot(st.normalAxis, S.meanNormal) <= 0.7071) continue;
                const Real sinSweep = std::sqrt(std::max(1 - st.cosSweep * st.cosSweep, 0.0));
                st.stallLimitFactor = clamp(1 - S.def.sweepStallGradient * sinSweep * (st.spanFromRoot / spanSum - 0.5), 0.3, 3.0);
            }
    }
    for (std::size_t si = 0; si < surfaces_.size(); ++si) {
        Surface& S = surfaces_[si];
        const int n = S.endStrip - S.firstStrip;
        // Spanwise neighbours: strips sharing a bound-vortex end.
        for (int i = S.firstStrip; i < S.endStrip; ++i)
            for (int j = S.firstStrip; j < S.endStrip; ++j) {
                if (i == j) continue;
                if (length(strips_[i].boundA - strips_[j].boundB) < 1e-3) strips_[i].neighbourA = j;
                if (length(strips_[i].boundB - strips_[j].boundA) < 1e-3) strips_[i].neighbourB = j;
            }
        llOffset_[si + 1] = llOffset_[si] + (S.def.liftingLine ? n * n : 0);
        if (!S.def.liftingLine) continue;
        S.controlSmoothing.assign(static_cast<std::size_t>(n * n), 0);
        for (int i = 0; i < n; ++i) {
            const Strip& a = strips_[static_cast<std::size_t>(S.firstStrip + i)];
            const Real sigma = 1.0 * a.chord;
            Real sum = 0;
            for (int j = 0; j < n; ++j) {
                const Strip& b = strips_[static_cast<std::size_t>(S.firstStrip + j)];
                if (b.side != a.side) continue;
                const Real w = std::exp(-dot(b.position - a.position, b.position - a.position) / (sigma * sigma));
                S.controlSmoothing[static_cast<std::size_t>(i * n + j)] = w;
                sum += w;
            }
            for (int j = 0; j < n; ++j) S.controlSmoothing[static_cast<std::size_t>(i * n + j)] /= sum;
        }
    }
    for (Surface& S : surfaces_) {
        const int src = S.def.downwashSource;
        if (src >= 0 && src < static_cast<int>(surfaces_.size())) {
            const Surface& W = surfaces_[src];
            S.downwashGradient = S.def.downwashGradient >= 0 ? S.def.downwashGradient
                                                             : 2 * W.liftSlope / (kPi * W.aspectRatio);
            // The separated wake of the stalled wing keeps about the deflection
            // it had at the stall (its normal force does not collapse), rather
            // than the falling CL's: the downwash at the finite wing's CLmax,
            // ~0.9 clmax cos(sweep) (Raymer §12.4.1).
            const Real clRef = 0.9 * W.def.section.clMax * std::cos(W.sweep);
            S.wakeDownwash = S.def.downwashAtZeroLift + S.downwashGradient * clRef / std::max(W.liftSlope, 0.1);
        }
    }
}

void AeroModel::addPanel(int si, const PanelDef& p, int side, Real spanOffset) {
    const Real ys = side;  // mirror sign
    Vec3 tipOffset;
    Vec3 chord0{1, 0, 0}, normal0;
    if (p.vertical) {
        tipOffset = {-p.span * std::tan(p.sweep), 0, -p.span};
        normal0 = {0, 1, 0};
    } else {
        tipOffset = {-p.span * std::tan(p.sweep), p.span, -p.span * std::tan(p.dihedral)};
        normal0 = {0, -std::sin(p.dihedral), -std::cos(p.dihedral)};
    }
    Vec3 root = p.rootQuarterChord;
    if (side < 0) {
        root.y = -root.y;
        tipOffset.y = -tipOffset.y;
        if (!p.vertical) normal0.y = -normal0.y;
    }
    const Vec3 spanAxis = normalized(tipOffset);
    for (int i = 0; i < p.strips; ++i) {
        const Real t = (i + 0.5) / p.strips;
        Strip s;
        s.surface = si;
        s.side = side;
        s.position = root + tipOffset * t;
        s.chord = lerp(p.rootChord, p.tipChord, t);
        s.spanFromRoot = spanOffset + t * p.span;
        // Area on the panel's true span (a canted winglet's height counts, not
        // its lateral projection); vertical panels' span is their height.
        s.area = s.chord * (p.vertical ? p.span : p.span / std::cos(p.dihedral)) / p.strips;
        s.width = length(tipOffset) / p.strips;
        const Real inc = lerp(p.rootIncidence, p.tipIncidence, t);
        s.chordAxis = chord0 * std::cos(inc) + normal0 * std::sin(inc);
        s.normalAxis = normal0 * std::cos(inc) - chord0 * std::sin(inc);
        s.liftAxis = cross(s.chordAxis, s.normalAxis);
        s.spanAxis = spanAxis;
        const Real cs = dot(s.chordAxis, spanAxis);
        s.cosSweep = std::sqrt(std::max(1 - cs * cs, 0.04));
        s.chordNormal = normalized(s.chordAxis - spanAxis * cs);
        s.normalNormal = normalized(cross(s.chordNormal, spanAxis));
        if (dot(s.normalNormal, s.normalAxis) < 0) s.normalNormal = -s.normalNormal;
        const Vec3 la = dot(spanAxis, s.liftAxis) >= 0 ? spanAxis : -spanAxis;
        s.boundA = s.position - la * (0.5 * s.width);
        s.boundB = s.position + la * (0.5 * s.width);
        for (const ControlSurfaceDef& c : p.controls) {
            if (t < c.spanStart || t > c.spanEnd || s.controlCount >= 3) continue;
            StripControl sc;
            sc.channel = c.channel;
            sc.sideSign = c.antisymmetric ? ys : 1.0;
            sc.maxPositive = c.maxPositive;
            sc.maxNegative = c.maxNegative;
            sc.tau = flapEffectiveness(c.chordFraction);
            const Real theta = std::acos(clamp(2 * c.chordFraction - 1, -1.0, 1.0));
            sc.cmFactor = -0.5 * std::sin(theta) * (1 - std::cos(theta));
            sc.effectiveness = c.effectivenessVsDeflection;
            sc.clMaxIncrementRatio = c.clMaxIncrementRatio;
            sc.profileDragPerRad2 = c.profileDragPerRad2;
            sc.chordExtension = c.chordExtension;
            sc.chordFraction = c.chordFraction;
            if (c.channel == ControlChannel::Spoiler) hasSpoilers_ = true;
            s.controls[s.controlCount++] = sc;
        }
        strips_.push_back(s);
    }
}

void AeroModel::setSurfaceAttached(int surface, int side, bool attached) {
    const Surface& S = surfaces_[surface];
    for (int i = S.firstStrip; i < S.endStrip; ++i)
        if (side == 0 || strips_[i].side == side) strips_[i].attached = attached;
}

Vec3 AeroModel::localFlow(const Surface& S, const Strip& st, const Vec3& r, const AeroInputs& in) const {
    // Strip velocity relative to the local air, body axes.
    Vec3 v = in.velocityBody + cross(in.omegaBody, r) - (in.windBody + in.windGradientBody * r);
    if (S.def.slipstreamExposure <= 0) return v;
    for (int k = 0; k < in.slipstreamCount; ++k) {
        if (S.def.slipstreamEngine >= 0 && k != S.def.slipstreamEngine) continue;
        const SlipstreamState& ss = in.slipstreams[k];
        const Vec3 d = st.position - ss.discCentre;
        const Real along = dot(d, ss.axis);
        if (along > 0) continue;  // ahead of the disc
        const Vec3 radial = d - ss.axis * along;
        const Real rr = length(radial);
        if (rr > ss.radius * 1.05) continue;
        // Air accelerated aft by the disc and swirled with the rotation.
        v += ss.axis * (ss.axialIncrement * S.def.slipstreamExposure);
        const Vec3 spin = ss.axis * (ss.clockwiseFromBehind ? 1.0 : -1.0);
        if (rr > 1e-3) v -= normalized(cross(spin, radial)) * (ss.swirlVelocity * S.def.slipstreamExposure);
    }
    return v;
}

Vec3 AeroModel::slipstreamAt(const Vec3& position, const AeroInputs& in) const {
    // Axial slipstream increment at a point (body axes), zero outside the
    // contracted wake cylinder or ahead of the disc.
    Vec3 dv;
    for (int k = 0; k < in.slipstreamCount; ++k) {
        const SlipstreamState& ss = in.slipstreams[k];
        const Vec3 d = position - ss.discCentre;
        const Real along = dot(d, ss.axis);
        if (along > 0) continue;
        if (length(d - ss.axis * along) > ss.radius) continue;
        dv += ss.axis * ss.axialIncrement;
    }
    return dv;
}

int AeroModel::spoilerBodies(Real speedbrake, BluffBody* out, int max) const {
    int n = 0;
    ControlDeflections d;
    d.speedbrake = speedbrake;
    for (const Strip& st : strips_) {
        if (!st.attached) continue;
        for (int c = 0; c < st.controlCount && n < max; ++c) {
            const StripControl& sc = st.controls[c];
            if (sc.channel != ControlChannel::Spoiler) continue;
            const Real sd = std::sin(std::max(controlDeflection(sc, d), 0.0));
            if (sd < 0.02) continue;
            BluffBody& b = out[n++];
            b.position = st.position - st.chordAxis * (0.3 * st.chord) + st.normalAxis * (0.06 * st.chord);
            b.axis = st.spanAxis;
            b.size = sc.chordFraction * st.chord * sd;
            b.length = st.width;
            b.dragArea = 0.45 * sc.chordFraction * sd * st.area;
        }
    }
    return n;
}

std::array<Real, 6> AeroModel::controlEffects(const Strip& st, const ControlDeflections& d) const {
    // Zero-lift shift, clmax increment per unit lift slope, cm, cd, chord extension.
    std::array<Real, 6> e{};
    for (int c = 0; c < st.controlCount; ++c) {
        const StripControl& sc = st.controls[c];
        const Real delta = controlDeflection(sc, d);
        if (sc.channel == ControlChannel::Spoiler) {
            // A spoiler separates the flow behind it: most of the section's
            // lift is lost (~0.95 sin delta, at most 85 %), the aft loading
            // with it (nose-up section moment), and it adds pressure drag on
            // its projected height, well below a free flat plate's since the
            // panel sits in the boundary layer (DATCOM 6.1.5.1 order).
            const Real sd = std::sin(std::max(delta, 0.0));
            const Real f = std::min(0.95 * sd, 0.85);
            e[5] = 1 - (1 - e[5]) * (1 - f);
            e[2] += 0.06 * f;
            e[3] += 0.45 * sc.chordFraction * sd;
            continue;
        }
        const Real eta = sc.effectiveness.empty() ? 1.0 : sc.effectiveness(std::abs(delta));
        const Real dA = -sc.tau * eta * delta;
        e[0] += dA;
        e[1] += sc.clMaxIncrementRatio * -dA;
        e[2] += sc.cmFactor * eta * delta;
        e[3] += sc.profileDragPerRad2 * delta * delta;
        if (sc.chordExtension > 0 && delta > 0 && sc.maxPositive > 0) e[4] += sc.chordExtension * std::min(delta / sc.maxPositive, 1.0);
    }
    return e;
}

Real AeroModel::controlDeflection(const StripControl& c, const ControlDeflections& d) const {
    Real u = 0;
    switch (c.channel) {
        case ControlChannel::Aileron: u = -d.aileron * c.sideSign; break;
        case ControlChannel::Elevator: u = -d.elevator; break;
        case ControlChannel::Rudder: u = -d.rudder; break;
        case ControlChannel::Flap: return clamp(d.flap, -c.maxNegative, c.maxPositive);
        case ControlChannel::Spoiler: return clamp(d.speedbrake, 0.0, 1.0) * c.maxPositive;
        case ControlChannel::None: return 0;
    }
    u = clamp(u, -1.0, 1.0);  // mechanical stops
    return u >= 0 ? u * c.maxPositive : u * c.maxNegative;
}

void AeroModel::prepare(AeroOutputs& out) const {
    out.strips.assign(strips_.size(), {});
    out.llFlow.assign(strips_.size(), {});
    out.llInduced.assign(strips_.size(), {});
    out.llInfluence.assign(static_cast<std::size_t>(llOffset_[surfaces_.size()]), {});
    int nMax = 0;
    for (const Surface& S : surfaces_)
        if (S.def.liftingLine) nMax = std::max(nMax, S.endStrip - S.firstStrip);
    out.llJacobian.assign(static_cast<std::size_t>(nMax * nMax), 0);
    out.llResidual.assign(static_cast<std::size_t>(nMax), 0);
    out.llGamma.assign(static_cast<std::size_t>(nMax), 0);
    out.llSeparated.assign(static_cast<std::size_t>(nMax), 0);
    out.llGrad.assign(static_cast<std::size_t>(nMax), {});
    out.llControl.assign(strips_.size(), {});
    out.llLu.assign(static_cast<std::size_t>(llOffset_[surfaces_.size()]), 0);
    out.llPivot.assign(strips_.size(), 0);
    out.llValid.fill(false);
    out.llLuValid.fill(false);
}

namespace {


// Air velocity at p induced by a horseshoe vortex of unit circulation: bound
// from a to b, trailing legs to infinity along u (Phillips & Snyder 2000, eq. 4).
Vec3 horseshoe(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& u) {
    const Vec3 r1 = p - a, r2 = p - b;
    const Real n1 = length(r1), n2 = length(r2);
    Vec3 v;
    const Real tiny = 1e-9;
    const Real d2 = n2 * (n2 - dot(u, r2));
    if (d2 > tiny) v += cross(u, r2) / d2;
    const Real db = n1 * n2 * (n1 * n2 + dot(r1, r2));
    if (db > tiny) v += cross(r1, r2) * ((n1 + n2) / db);
    const Real d1 = n1 * (n1 - dot(u, r1));
    if (d1 > tiny) v -= cross(u, r1) / d1;
    return v / (4 * kPi);
}

// LU factorisation with partial pivoting in place (J n x n, row-major; piv:
// the row swapped into each column), and the solve of J x = r with it (r
// overwritten by x). Factor returns false for a singular matrix.
bool luFactor(Real* J, int* piv, int n) {
    for (int c = 0; c < n; ++c) {
        int p = c;
        for (int i = c + 1; i < n; ++i)
            if (std::abs(J[i * n + c]) > std::abs(J[p * n + c])) p = i;
        if (std::abs(J[p * n + c]) < 1e-12) return false;
        piv[c] = p;
        if (p != c)
            for (int k = 0; k < n; ++k) std::swap(J[c * n + k], J[p * n + k]);
        for (int i = c + 1; i < n; ++i) {
            const Real f = J[i * n + c] / J[c * n + c];
            J[i * n + c] = f;
            if (f == 0) continue;
            for (int k = c + 1; k < n; ++k) J[i * n + k] -= f * J[c * n + k];
        }
    }
    return true;
}
void luSolve(const Real* LU, const int* piv, Real* r, int n) {
    // Every row swap first (the multipliers moved with their rows), then the
    // forward substitution: the elimination's own operations, in its order.
    for (int c = 0; c < n; ++c)
        if (piv[c] != c) std::swap(r[c], r[piv[c]]);
    for (int c = 0; c < n; ++c)
        for (int i = c + 1; i < n; ++i) {
            const Real f = LU[i * n + c];
            if (f != 0) r[i] -= f * r[c];
        }
    for (int i = n - 1; i >= 0; --i) {
        Real sum = r[i];
        for (int k = i + 1; k < n; ++k) sum -= LU[i * n + k] * r[k];
        r[i] = sum / LU[i * n + i];
    }
}

}  // namespace

template <class GammaOf>
void AeroModel::solveLiftingLine(int si, int n0, int n, const AeroInputs& in, AeroOutputs& out, GammaOf&& gammaOf) const {
    const Surface& S = surfaces_[static_cast<std::size_t>(si)];
    const Vec3* vFree = out.llFlow.data() + n0;
    Vec3* induced = out.llInduced.data() + n0;
    for (int k = 0; k < n; ++k) induced[k] = {};
    // The trailing legs run downstream along the surface's mean flow.
    Vec3 sum;
    for (int k = 0; k < n; ++k) sum += vFree[k];
    // At (near) rest the mean flow has no meaningful direction (a slipstream
    // alone, a gust); with the air from behind the lifting-line picture does
    // not apply: the legs then trail straight aft along the body.
    const Vec3 aft{-1, 0, 0};
    Vec3 u = length(sum) > 3.0 * n ? normalized(-sum) : aft;
    if (dot(u, aft) < 0.2) u = aft;
    // Ground images (a flat ground below, within a few spans).
    const bool ground = S.def.groundEffect && dot(in.groundUpBody, in.groundUpBody) > 0.5 && in.heightAboveGround < 3 * S.span;
    const Real h = ground ? in.heightAboveGround : 1e9;
    // Near the ground the wake cannot trail down through it: legs parallel
    // to the ground (and so to their images), blending in below one span.
    if (ground) {
        const Vec3 flat = u - in.groundUpBody * dot(u, in.groundUpBody);
        if (length(flat) > 0.1) {
            const Real k = smoothstep(S.span, 0.25 * S.span, h);
            u = normalized(u * (1 - k) + normalized(flat) * k);
        }
    }
    Vec3* V = out.llInfluence.data() + llOffset_[static_cast<std::size_t>(si)];
    const std::size_t s = static_cast<std::size_t>(si);
    const bool rebuild = !out.llValid[s] || dot(u, out.llDir[s]) < 0.99999 ||
                         (h < 1e8 || out.llHeight[s] < 1e8 ? std::abs(h - out.llHeight[s]) > std::max(0.01 * std::min(h, out.llHeight[s]), 0.02) : false);
    if (rebuild) {
        const Vec3 up = in.groundUpBody;
        const Vec3 ref = surfaces_[0].centroid;
        auto mirror = [&](const Vec3& p) { return p - up * (2 * (h + dot(p - ref, up))); };
        const Vec3 ui = u - up * (2 * dot(u, up));
        for (int i = 0; i < n; ++i) {
            const Vec3& p = strips_[static_cast<std::size_t>(n0 + i)].position;
            for (int j = 0; j < n; ++j) {
                const Strip& sj = strips_[static_cast<std::size_t>(n0 + j)];
                Vec3 v = horseshoe(p, sj.boundA, sj.boundB, u);
                if (h < 1e8) v -= horseshoe(p, mirror(sj.boundA), mirror(sj.boundB), ui);
                V[i * n + j] = v;
            }
        }
        out.llDir[s] = u;
        out.llHeight[s] = h;
        out.llValid[s] = true;
        out.llLuValid[s] = false;
    }
    // Newton on R_i = G_i - g_i(v_i - sum_j V_ij G_j), from the last solution.
    Real* G = out.llGamma.data();
    Real* R = out.llResidual.data();
    Real* J = out.llJacobian.data();
    Real* sep = out.llSeparated.data();
    constexpr Real kViscosity = 0.5;
    Vec3* grad = out.llGrad.data();
    for (int k = 0; k < n; ++k) G[k] = strips_[static_cast<std::size_t>(n0 + k)].attached ? out.strips[static_cast<std::size_t>(n0 + k)].gamma : 0;
    auto flowAt = [&](int i) {
        Vec3 w = vFree[i];
        for (int j = 0; j < n; ++j) w -= V[i * n + j] * G[j];
        return w;
    };
    std::array<Vec3, 64> wSave{};
    int staleIters = 0;       // steps made with the Jacobian in hand
    bool lastNewton = true;   // the last step's Jacobian was built for it (the start: as before)
    Real prevR = 0;
    // A warm start converges in a few iterations; a cold one from the
    // transonic regime, where the un-induced sections sit past their reduced
    // lift limits, needs the room (24 left the forces 7 % short at M0.92).
    for (int it = 0; it < 64; ++it) {
        Real rMax = 0, gMax = 1e-3;
        bool separated = false;
        for (int i = 0; i < n; ++i) {
            if (!strips_[static_cast<std::size_t>(n0 + i)].attached) {
                R[i] = G[i];
                continue;
            }
            const Vec3 w = flowAt(i);
            wSave[static_cast<std::size_t>(std::min(i, 63))] = w;
            Real att = 1;
            const Real g = gammaOf(i, w, &att);
            // Artificial viscosity where the flow has separated (Chattot
            // 2004): a stalled strip's circulation is pulled toward its
            // neighbours', so the loss of its lift cannot feed the runaway
            // upwash of a single strip (the lifting line has no unique
            // post-stall solution without it). Attached flow is untouched.
            sep[i] = kViscosity * (1 - att);
            separated = separated || sep[i] > 0;
            const Strip& sti = strips_[static_cast<std::size_t>(n0 + i)];
            const Real ga = sti.neighbourA >= 0 ? G[sti.neighbourA - n0] : 0, gb = sti.neighbourB >= 0 ? G[sti.neighbourB - n0] : 0;
            R[i] = G[i] - g - sep[i] * (ga + gb - 2 * G[i]);
            rMax = std::max(rMax, std::abs(R[i]));
            gMax = std::max({gMax, std::abs(G[i]), std::abs(g)});
        }
        out.llIterations = it;
        // Tight: the result must not depend on the warm start (trim and
        // linearisation difference the forces). After a chord step (a
        // Jacobian kept from before) the convergence is linear, not
        // quadratic: it goes on to 1e-12, about where a Newton step from 1e-9
        // lands, so that keeping the Jacobian does not loosen the solution.
        if (lastNewton ? rMax < 1e-9 * gMax + 1e-10 : rMax < 1e-14 * gMax + 1e-14) break;
        // The Jacobian is the costly part (three more section evaluations a
        // strip and a factorisation). From one solve to the next it hardly
        // changes, so the last one is kept while the residual still falls
        // fast with it (a chord iteration); it is rebuilt at once when it does
        // not. Only in attached flow, whose solution is unique: once a strip
        // separates, every step is a full Newton step as it always was (the
        // post-stall solution leans on the viscosity, and the path to it
        // matters).
        Real* LU = out.llLu.data() + llOffset_[s];
        int* piv = out.llPivot.data() + n0;
        if (separated || staleIters >= 8 || (staleIters > 0 && rMax > 0.2 * prevR)) out.llLuValid[s] = false;
        prevR = rMax;
        if (!out.llLuValid[s]) {
            // Jacobian dR_i/dG_j = delta_ij + grad(g_i) . V_ij (g's gradient
            // in the strip's flow by finite differences).
            for (int i = 0; i < n; ++i) {
                if (!strips_[static_cast<std::size_t>(n0 + i)].attached) {
                    grad[i] = {};
                    continue;
                }
                const Vec3 w = i < 64 ? wSave[static_cast<std::size_t>(i)] : flowAt(i);
                const Real g = gammaOf(i, w, nullptr);
                const Real dh = 1e-3 * std::max(length(w), 1.0);
                grad[i] = Vec3{gammaOf(i, w + Vec3{dh, 0, 0}, nullptr) - g, gammaOf(i, w + Vec3{0, dh, 0}, nullptr) - g, gammaOf(i, w + Vec3{0, 0, dh}, nullptr) - g} / dh;
            }
            for (int i = 0; i < n; ++i) {
                for (int j = 0; j < n; ++j) J[i * n + j] = (i == j ? 1.0 : 0.0) + dot(grad[i], V[i * n + j]);
                if (sep[i] > 0) {
                    const Strip& sti = strips_[static_cast<std::size_t>(n0 + i)];
                    J[i * n + i] += 2 * sep[i];
                    if (sti.neighbourA >= 0) J[i * n + sti.neighbourA - n0] -= sep[i];
                    if (sti.neighbourB >= 0) J[i * n + sti.neighbourB - n0] -= sep[i];
                }
            }
            if (luFactor(J, piv, n)) {
                std::copy(J, J + n * n, LU);
                out.llLuValid[s] = true;
            }
            staleIters = 0;
        }
        lastNewton = staleIters == 0;
        if (out.llLuValid[s]) {
            ++staleIters;
            luSolve(LU, piv, R, n);
            Real stepMax = 0;
            for (int i = 0; i < n; ++i) stepMax = std::max(stepMax, std::abs(R[i]));
            // Damped where the polar's negative post-stall slope would overshoot.
            const Real scale = stepMax > 0.5 * gMax + 1 ? (0.5 * gMax + 1) / stepMax : 1.0;
            for (int i = 0; i < n; ++i) G[i] -= scale * R[i];
        } else {
            for (int i = 0; i < n; ++i) G[i] -= 0.5 * R[i];
        }
    }
    for (int i = 0; i < n; ++i) {
        Vec3 a;
        for (int j = 0; j < n; ++j) a += V[i * n + j] * G[j];
        induced[i] = a;
        out.strips[static_cast<std::size_t>(n0 + i)].gamma = G[i];
    }
}

void AeroModel::evaluate(const AeroInputs& in, AeroOutputs& out) const {
    out.force = {};
    out.moment = {};
    const Real rho = in.density;

    // CG airflow summary.
    const Vec3 vAirCg = in.velocityBody - in.windBody;
    const Real vt = length(vAirCg);
    out.airspeed = vt;
    out.mach = vt / in.speedOfSound;
    out.dynamicPressure = 0.5 * rho * vt * vt;
    out.alpha = std::atan2(vAirCg.z, vAirCg.x);
    out.beta = vt > 1e-6 ? std::asin(clamp(vAirCg.y / vt, -1.0, 1.0)) : 0.0;
    // Prandtl-Glauert factor. The 0.9 cap is a numerical guard against the
    // linear theory's singularity (a model limit, not a speed limit); drag
    // rise below carries the physics of the transonic regime.
    const Real beta = std::sqrt(1 - std::min(sq(out.mach), 0.81));

    for (std::size_t si = 0; si < surfaces_.size(); ++si) {
        const Surface& S = surfaces_[si];
        SurfaceOutput& so = out.surfaces[si];
        so = {};
        // Ground effect on induced flow.
        const Real ge = S.def.groundEffect && S.horizontal ? groundEffectFactor(in.heightAboveGround, S.span) : 1.0;
        const Real arEff = S.aspectRatio / ge;
        const Real a3 = liftCurveSlope3D(arEff, S.sweep, S.def.section.liftSlope, beta);
        // Downwash from the source surface: the linear estimate here; the
        // stalled wake (below, once the strips' flow is known) may relax it,
        // and eps (lagged if available) is settled before any strip is evaluated.
        Real eps = 0;
        Real epsLinear = 0, srcGe = 1;
        const int src = S.def.downwashSource;
        if (src >= 0) {
            const Real clSrc = out.surfaces[src].cl;
            const Real aSrc = surfaces_[src].liftSlope;
            srcGe = groundEffectFactor(in.heightAboveGround, surfaces_[src].span);
            epsLinear = (S.def.downwashAtZeroLift + S.downwashGradient * clSrc / aSrc) * srcGe;
            so.downwash = epsLinear;
        }

        const bool ll = S.def.liftingLine;
        // Induced angle from the strip's own loading (local lifting-line
        // approximation): d(cl * alpha_i)/dcl = 2 alpha_i, which is what makes
        // aileron yaw adverse and asymmetric stall yaw into the dropping wing.
        // With the lifting line the induced flow is solved instead.
        const Real inducedPerCl = ll ? 0.0 : 1.0 / (kPi * arEff * S.def.spanEfficiency);
        // With the lifting line the planform's own induced drag is solved; the
        // surface's Oswald e also carries fuselage interference and the
        // viscous drag due to lift: the remainder (1/e - 1) / (pi AR) CL^2.
        const Real liftDragK = ll ? std::max(1 / S.def.spanEfficiency - 1, 0.0) / (kPi * S.aspectRatio) : 0.0;
        // Finite-wing post-stall drag (Viterna), never above the 2-D flat plate.
        const Real cdMax = std::min(S.def.section.flatPlateCd, finiteWingMaxDrag(S.aspectRatio));

        // Section evaluation of strip i at its velocity relative to the air v
        // (body axes; with the lifting line, including the induced flow).
        struct StripEval {
            Real alpha = 0, cs = 1, an = 0, q = 0, qFree = 0, cdWave = 0, extension = 0, stallAlpha = 0, zeroLiftAlpha = 0;
            Real gamma = 0;  // bound circulation the section's lift implies (Kutta-Joukowski)
            Vec3 vPerp;
            SectionResult sec;
            bool active = false;
        };
        auto evalStrip = [&](const Strip& st, const Vec3& v) {
            StripEval e;
            const Vec3 vPerp = v - st.spanAxis * dot(v, st.spanAxis);
            const Real vp2 = dot(vPerp, vPerp);
            if (vp2 < 1e-8) return e;
            e.active = true;
            e.vPerp = vPerp;
            // Simple sweep theory: the section works on the flow normal to its
            // span. Its streamwise-defined polar maps there with the sweep:
            // angles, slope and lift limits over cos(sweep), so that q_n cl_n
            // = q CL (linear range unchanged), an untwisted wing's zero-lift
            // angle is the section's at any sweep, and CLmax ~ clmax cos(sweep).
            const Real cs = st.cosSweep;
            e.cs = cs;
            Real alpha = std::atan2(-dot(vPerp, st.normalNormal), dot(vPerp, st.chordNormal));
            if (S.horizontal) alpha -= eps / cs;
            if (S.def.trimmableStabilizer) alpha -= in.controls.stabilizer / cs;  // leading edge down: nose up
            SectionPolar sp = S.def.section;
            sp.zeroLiftAlpha /= cs;
            // The span station's lift limits (sweep stall progression), then
            // the sweep mapping.
            sp.clMax *= st.stallLimitFactor;
            sp.clMin *= st.stallLimitFactor;
            sp.clMax /= cs;
            sp.clMin /= cs;
            sp.clAtCdMin /= cs;
            // Lifting line: the normal section's own 2-D slope (Prandtl-Glauert
            // on the normal Mach number); the finite span is in the induced
            // flow. Strip theory: the surface's 3-D slope.
            const Real an = ll ? S.def.section.liftSlope / std::sqrt(1 - std::min(vp2 / sq(in.speedOfSound), 0.81)) : a3 / cs;
            e.an = an;
            // Transonic section behaviour on the normal Mach number mn (the
            // free-stream equivalent m = mn / cos(sweep) is what Korn's
            // equation is written for). Above the knee the section's lift
            // limit falls (shock-induced separation: the buffet boundary);
            // the hinged controls lose effectiveness as the shock moves to
            // the hinge line, by the zero-lift Mdd so that the control's own
            // lift does not feed back into it.
            const Real mn = std::sqrt(vp2) / in.speedOfSound;
            const Real m = mn / cs;
            const Real mdd0 = S.def.kornFactor / cs - S.def.thicknessRatio / (cs * cs);
            const Real ctlEff = mdd0 > 0.05 ? std::max(S.def.machControlFloor, 1 - std::pow(m / mdd0, S.def.machControlExponent)) : 1.0;
            if (mn > S.def.machClMaxKnee) {
                const Real k = std::max(0.35, 1 - S.def.machClMaxFactor * sq(mn - S.def.machClMaxKnee));
                sp.clMax *= k;
                sp.clMin *= k;
            }
            // Controls on this strip (deflections normal to the hinge line),
            // spread spanwise over half a chord on a lifting-line surface.
            const std::array<Real, 6> ctl = ll ? out.llControl[static_cast<std::size_t>(&st - strips_.data())] : controlEffects(st, in.controls);
            const Real dAlpha0 = ctl[0] * ctlEff, dClMax = ctl[1] * an * ctlEff, dCm = ctl[2] * ctlEff, dCd = ctl[3], extension = ctl[4], spoiled = ctl[5];
            // The separated section's normal force (Viterna's CDmax sets the
            // post-stall level) follows the same span factor: the inboard
            // separation of a swept wing is drained outboard (thin, a low base
            // pressure), the outboard is where the dead air piles up and the
            // tip relieves it. Never above the 2-D flat plate.
            const Real cdMaxStrip = std::min(S.def.section.flatPlateCd, cdMax * st.stallLimitFactor);
            e.sec = evaluateSection(sp, alpha, an, dAlpha0, dClMax, cdMaxStrip);
            e.sec.cm += dCm;
            e.sec.cd += dCd;
            if (spoiled > 0) {
                e.sec.cl *= 1 - spoiled;
                e.sec.attached = std::min(e.sec.attached, 1 - spoiled);
            }
            // Spoilers / lift dump on the main wing: a fraction of the
            // section lift is spoiled (the drag is carried by a drag area).
            if (si == 0 && in.controls.spoiler > 0) e.sec.cl *= 1 - in.controls.spoiler;
            // Compressibility drag rise (Korn + Lock), on the normal flow:
            // Korn's equation is written for the free-stream Mach number and
            // the streamwise lift coefficient (cl_streamwise = cl_n cos^2), and
            // Lock's increment is streamwise (normal: divide by cos^2).
            const Real mdd = mdd0 - std::abs(e.sec.cl) / (10 * cs);
            const Real mcrit = mdd - 0.1077;  // (0.1/80)^(1/3)
            e.cdWave = m > mcrit ? 20 * std::pow(m - mcrit, 4) / (cs * cs) : 0;
            // Mach tuck. Through the drag rise (Mcrit to just past Mdd) a
            // rear-loaded section's aft loading grows: the cm0 shift. From
            // about Mdd, where the shock is strong and moving aft, the force
            // point travels from c/4 toward c/2 (reached about M 1): the
            // lift's moment about the quarter chord, where the lifting line
            // keeps its control points. Attached flow only; the separated
            // section's centre of pressure is in evaluateSection.
            if (m > mcrit && mcrit < 0.99) {
                const Real fCm0 = smoothstep(mcrit, mdd + 0.05, m) * e.sec.attached;
                const Real fAc = mdd - 0.05 < 0.99 ? smoothstep(mdd - 0.05, 1.0, m) * e.sec.attached : 0.0;
                e.sec.cm += fCm0 * S.def.machCm0Shift - fAc * S.def.machAcShift * e.sec.cl;
            }
            e.alpha = alpha;
            e.q = 0.5 * rho * vp2;
            e.qFree = 0.5 * rho * dot(v, v);
            e.extension = extension;
            e.stallAlpha = (sp.zeroLiftAlpha + dAlpha0 + (sp.clMax + dClMax) / an) * cs;
            e.zeroLiftAlpha = (sp.zeroLiftAlpha + dAlpha0) * cs;
            // Kutta-Joukowski on the (swept) bound vortex: the strip's lift
            // q S cl = rho |V x l| G, so G = V_perp cl (S / width) / 2.
            e.gamma = 0.5 * std::sqrt(vp2) * e.sec.cl * st.area * (1 + extension) / std::max(st.width, 1e-6);
            return e;
        };

        // Free flow at every strip (no induced flow yet).
        const int n0 = S.firstStrip, n = S.endStrip - S.firstStrip;
        if (ll) {
            std::array<Real, 6>* raw = out.llControl.data() + n0;
            std::array<std::array<Real, 6>, 64> local{};
            for (int k = 0; k < n && k < 64; ++k) local[static_cast<std::size_t>(k)] = controlEffects(strips_[static_cast<std::size_t>(n0 + k)], in.controls);
            for (int i = 0; i < n && i < 64; ++i) {
                std::array<Real, 6> sm{};
                for (int j = 0; j < n && j < 64; ++j) {
                    const Real w = S.controlSmoothing[static_cast<std::size_t>(i * n + j)];
                    if (w == 0) continue;
                    for (int q = 0; q < 5; ++q) sm[static_cast<std::size_t>(q)] += w * local[static_cast<std::size_t>(j)][static_cast<std::size_t>(q)];
                }
                // A spoiler's separation is local, with a sharp edge (whose
                // vortex is the point): not smoothed.
                sm[5] = local[static_cast<std::size_t>(i)][5];
                raw[i] = sm;
            }
        }
        // The stalled wing's wake over a horizontal tail. The separated
        // fraction of the wing within the tail's span sets the band: the
        // separated region spans the chord's projection across the flow (the
        // shear layer off the leading edge to the trailing edge), so its
        // centreline leaves the mid-chord along the flow (alpha less the
        // downwash, in body axes) with a half-thickness of c/2 sin(alpha - eps)
        // plus the spreading aft, both scaled by that fraction. A strip inside
        // sees the wake's reduced dynamic pressure; the surface's downwash
        // relaxes toward the wake's own instead of following the collapsing CL
        // (which made a stalled wing's tail MORE effective: the reverse of
        // blanketing).
        bool wake = false;
        Real wakeFraction = 0, wakeHalfBase = 0, wakeTanAlpha = 0, wakeXte = 0, wakeZte = 0;
        if (src >= 0 && S.horizontal && S.def.wakeBlanking && surfaces_[static_cast<std::size_t>(src)].horizontal) {
            const Surface& W = surfaces_[static_cast<std::size_t>(src)];
            const Real yMax = 0.5 * S.span + 0.5 * W.meanChord;
            Real sepArea = 0, area = 0;
            for (int i = W.firstStrip; i < W.endStrip; ++i) {
                const Strip& ws = strips_[static_cast<std::size_t>(i)];
                if (std::abs(ws.position.y) > yMax || !ws.attached) continue;
                const StripTelemetry& wt = out.strips[static_cast<std::size_t>(i)];
                if (wt.q <= 0) continue;
                // Separated by the strip's own section weight, or by its
                // free-stream angle (incidence, twist, rotation; no induced
                // flow) further past its stall angle than attached flow's
                // largest induced angle, ~2 clmax / (pi AR): the lifting
                // line's regularised post-stall solution holds a root strip
                // attached under its stalled neighbours' induced flow long
                // after the wing has let go (17 deg past its stall at 30 deg
                // alpha), while a flapped wing at its 1-g CLmax is genuinely
                // held attached by 4-6 deg of induced downwash.
                const Vec3& v = out.llFlow[static_cast<std::size_t>(i)];
                const Vec3 vp = v - ws.spanAxis * dot(v, ws.spanAxis);
                Real sep = 1 - wt.attached;
                if (dot(vp, vp) > 1e-8) {
                    const Real aFree = std::atan2(-dot(vp, ws.normalNormal), dot(vp, ws.chordNormal)) * ws.cosSweep;
                    const Real width = W.def.section.stallWidth * ws.cosSweep;
                    const Real induced = 2 * W.def.section.clMax / (kPi * std::max(W.aspectRatio, 1.0));
                    sep = std::max(sep, smoothstep(wt.stallAlpha + induced, wt.stallAlpha + induced + width, aFree));
                }
                sepArea += ws.area * sep;
                area += ws.area;
            }
            wakeFraction = area > 0 ? clamp(sepArea / area, 0.0, 1.0) : 0.0;
            if (wakeFraction > 1e-3) {
                wake = true;
                const Real deflection = clamp(out.alpha - std::max(epsLinear, 0.0), -1.0, 1.0);
                wakeXte = W.centroid.x - 0.25 * W.meanChord;  // mid-chord (the centroid is on the quarter-chord line)
                wakeZte = W.centroid.z;
                wakeHalfBase = 0.5 * W.meanChord * std::abs(std::sin(deflection));
                wakeTanAlpha = std::tan(deflection);
            }
        }
        Vec3* vFree = out.llFlow.data() + n0;
        Real immersion = 0, immersionArea = 0;
        for (int k = 0; k < n; ++k) {
            const Strip& st = strips_[static_cast<std::size_t>(n0 + k)];
            vFree[k] = st.attached ? localFlow(S, st, st.position - in.cg, in) : Vec3{};
            if (!wake || !st.attached) continue;
            immersionArea += st.area;
            const Real dx = wakeXte - st.position.x;  // distance aft of the wing's mid-chord
            if (dx <= 0) continue;
            // Air flowing aft and up (z down) at alpha carries the wake up by dx tan(alpha - eps).
            const Real zWake = wakeZte - dx * wakeTanAlpha;
            // The thickness and the strength both grow with the separated
            // fraction: a wing just past its stall sheds a thin wake (a low
            // tail with landing flap sits in the wing's plane and must not
            // lose its elevator at the 1-g stall), a fully stalled one the
            // whole separated region.
            const Real half = wakeFraction * (wakeHalfBase + S.def.wakeSpread * dx);
            const Real w = half > 1e-6 ? wakeFraction * (1 - smoothstep(0.5, 1.0, std::abs(st.position.z - zWake) / half)) : 0.0;
            if (w <= 0) continue;
            vFree[k] = vFree[k] * std::sqrt(1 - w * (1 - S.def.wakeDynamicPressureRatio));
            immersion += w * st.area;
        }
        if (wake && immersionArea > 0) {
            so.wakeImmersion = immersion / immersionArea;
            so.downwash = lerp(epsLinear, S.wakeDownwash * srcGe, so.wakeImmersion);
        }
        if (src >= 0) eps = in.downwashLagged ? in.downwashLagged[si] : so.downwash;
        if (ll)
            solveLiftingLine(static_cast<int>(si), n0, n, in, out, [&](int k, const Vec3& v, Real* attached) {
                const StripEval e = evalStrip(strips_[static_cast<std::size_t>(n0 + k)], v);
                if (attached) *attached = e.active ? e.sec.attached : 1;
                return e.gamma;
            });

        Real liftSum = 0, qaFree = 0, alphaSum = 0, areaSum = 0, separatedSum = 0;
        for (int i = S.firstStrip; i < S.endStrip; ++i) {
            const Strip& st = strips_[i];
            StripTelemetry& tel = out.strips[i];
            const Real gammaPrev = tel.gamma;
            tel = {};
            tel.gamma = gammaPrev;
            if (!st.attached) { tel.gamma = 0; continue; }
            const Vec3 r = st.position - in.cg;
            // With the lifting line, the strip's flow includes the induced
            // velocity of every horseshoe (and its ground image).
            const Vec3 v = vFree[i - n0] - (ll ? out.llInduced[static_cast<std::size_t>(i)] : Vec3{});
            const StripEval e = evalStrip(st, v);
            if (!e.active) continue;
            const SectionResult& sec = e.sec;
            const Real cs = e.cs, q = e.q, qFree = e.qFree;
            const Vec3& vPerp = e.vPerp;
            // A Fowler flap's chord extension: more lifting area.
            const Real area = st.area * (1 + e.extension), chord = st.chord * (1 + e.extension);
            // Angles reported streamwise-equivalent (normal-plane angle x
            // cos(sweep)), so margins in radians mean the same on every wing.
            tel.alpha = e.alpha * cs;
            tel.cl = sec.cl;
            tel.cd = sec.cd + e.cdWave;
            tel.cm = sec.cm;
            tel.q = q;
            tel.flowPerp = vPerp;
            tel.stalled = sec.stalled;
            tel.attached = sec.attached;
            tel.stallAlpha = e.stallAlpha;
            tel.zeroLiftAlpha = e.zeroLiftAlpha;
            if (ll) tel.gamma = e.gamma;
            liftSum += q * area * sec.cl;
            qaFree += qFree * area;
            alphaSum += e.alpha * cs * st.area;
            areaSum += st.area;
            separatedSum += (1 - sec.attached) * st.area;
            if (sec.stalled) ++so.stalledStrips;

            // Forces. Lift is normal to both the strip's (spanwise-free) flow
            // and its swept span axis; the unswept lateral axis would tilt it
            // forward by ~sin(alpha) sin^2(sweep) and cancel induced drag.
            // With the lifting line the flow already carries the induced
            // velocity, so the lift tilts back by the induced angle: that is
            // the induced drag.
            Vec3 dragDir = normalized(-vPerp);
            const Vec3 la = dot(st.spanAxis, st.liftAxis) >= 0 ? st.spanAxis : -st.spanAxis;
            if (S.horizontal && eps != 0) {
                // Downwash tilts the local flow (rotate about the span axis).
                const Vec3 nrm = normalized(cross(la, dragDir));
                dragDir = normalized(dragDir * std::cos(eps / cs) + nrm * std::sin(eps / cs));
            }
            const Vec3 liftDir = normalized(cross(dragDir, la));
            // Induced angle in the normal plane from the streamwise CL (cl_n
            // cos^2), fading as the flow separates (a stalled strip sheds its
            // vorticity rather than trailing it).
            const Real ai = sec.cl * cs * cs * (inducedPerCl + liftDragK) * sec.attached;
            const Real qa = q * area;
            // Attached profile drag is skin friction and pressure drag of the
            // whole local flow (spanwise flow included); the separated part,
            // the wave drag and the induced drag act on the normal flow.
            const Real cdFriction = sec.attached * sec.cd;
            const Real cdNormal = (1 - sec.attached) * sec.cd + e.cdWave;
            const Vec3 f = liftDir * (qa * sec.cl) + dragDir * (qa * (cdNormal + sec.cl * ai)) +
                           normalized(-v) * (qFree * area * cdFriction);
            out.force += f;
            out.moment += cross(r, f) + st.liftAxis * (qa * chord * sec.cm);
        }
        // The surface's streamwise lift coefficient (on the free-stream q),
        // the input of the downwash it makes.
        so.cl = qaFree > 0 ? liftSum / qaFree : 0;
        so.alphaMean = areaSum > 0 ? alphaSum / areaSum : 0;
        so.separated = areaSum > 0 ? separatedSum / areaSum : 0;
    }

    // Fuselage: Munk moment and crossflow drag (free stream; slender-body
    // terms are not applied to the slipstream).
    for (const FuselageDef& fu : fuselages_) {
        const Vec3 r = fu.referencePoint - in.cg;
        const Vec3 v = in.velocityBody + cross(in.omegaBody, r) - (in.windBody + in.windGradientBody * r);
        const Real v2 = dot(v, v);
        if (v2 < 1e-8) continue;
        const Real q = 0.5 * rho * v2;
        const Real af = std::atan2(v.z, v.x);
        const Real bf = std::atan2(v.y, v.x);
        out.moment.y += q * fu.volume * fu.munkFactor * std::sin(2 * af);
        out.moment.z += -q * fu.volume * fu.munkFactor * std::sin(2 * bf);
        const Vec3 vc{0, v.y, v.z};
        const Vec3 fc = vc * (-0.5 * rho * fu.crossflowCd * fu.crossflowArea * length(vc));
        out.force += fc;
        out.moment += cross(r, fc);
    }
    // Parasite drag items (in the slipstream they see its extra dynamic
    // pressure: "scrubbing" drag, which grows with thrust).
    for (const DragItemDef& d : dragItems_) {
        const Vec3 r = d.position - in.cg;
        Vec3 v = in.velocityBody + cross(in.omegaBody, r) - (in.windBody + in.windGradientBody * r);
        v += slipstreamAt(d.position, in);
        Real area = d.dragArea;
        if (d.criticalMach > 0) {
            const Real m = std::min(length(v) / in.speedOfSound, 1.3);
            if (m > d.criticalMach) area *= 1 + 60 * sq(m - d.criticalMach);
        }
        const Vec3 f = v * (-0.5 * rho * area * length(v));
        out.force += f;
        out.moment += cross(r, f);
    }
}

Real trimmedMaxLiftCoefficient(const AeroModel& model, Real refArea, const Vec3& cg, Real flap, Real mach) {
    const Real a = 340.3, v = std::max(mach, 0.15) * a;
    AeroInputs in;
    in.density = isa::kSeaLevelDensity;
    in.speedOfSound = a;
    in.cg = cg;
    in.controls.flap = flap;
    AeroOutputs out;
    model.prepare(out);
    const Real q = 0.5 * in.density * v * v;
    // Lift and pitching moment at alpha with the elevator at e (-1..1).
    auto eval = [&](Real alpha, Real e, Real& lift) {
        in.velocityBody = {v * std::cos(alpha), 0, v * std::sin(alpha)};
        in.controls.elevator = e;
        model.evaluate(in, out);
        lift = out.force.x * std::sin(alpha) - out.force.z * std::cos(alpha);
        return out.moment.y;
    };
    Real best = 0;
    for (Real alphaDeg = 0; alphaDeg <= 30.0; alphaDeg += 0.25) {
        const Real alpha = alphaDeg * kTwoPi / 360.0;
        Real lift = 0, liftLo = 0, liftHi = 0;
        const Real mLo = eval(alpha, -1, liftLo), mHi = eval(alpha, 1, liftHi);
        if ((mLo > 0) == (mHi > 0)) {
            // Cannot trim here: the elevator is out of authority (nose-up) or the
            // pitch-down limit is reached; stop once the stick is fully back.
            if (mHi < 0) break;
            continue;
        }
        Real lo = -1, hi = 1, mlo = mLo;
        for (int k = 0; k < 30; ++k) {
            const Real mid = 0.5 * (lo + hi), m = eval(alpha, mid, lift);
            if ((m > 0) == (mlo > 0)) { lo = mid; mlo = m; } else hi = mid;
        }
        eval(alpha, 0.5 * (lo + hi), lift);
        const Real cl = lift / (q * refArea);
        if (cl > best) best = cl;
        else if (cl < best - 0.15) break;  // well past the peak
    }
    return best;
}

}  // namespace avd
