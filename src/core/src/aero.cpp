// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
//
// References:
//  - Strip theory / uniform-downwash lifting line: Anderson, "Fundamentals of
//    Aerodynamics", ch. 5; McCormick, "Aerodynamics, Aeronautics and Flight
//    Mechanics" (ground effect factor (16h/b)^2 / (1 + (16h/b)^2)).
//  - 3-D lift slope: Helmbold equation as given in USAF DATCOM 4.1.3.2.
//  - Post-stall: Viterna & Corrigan (1982), NASA CP-2230.
//  - Flap effectiveness tau and pitching moment: thin-airfoil theory (Abbott &
//    von Doenhoff, "Theory of Wing Sections", §8).
//  - Drag rise: Korn equation + Lock's fourth-power law (Mason, "Transonic
//    aerodynamics of airfoils and wings", Virginia Tech notes).
//  - Slender-body Munk moment: Munk (1924), NACA TR-184.
#include "avd/aero.hpp"

#include <stdexcept>

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
SectionResult forwardSection(const SectionPolar& p, Real alpha, Real a, Real alpha0, Real clMaxE, Real clMinE) {
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
        const Viterna v(as, clMaxE * (1 - p.postStallClDrop), cdS, p.flatPlateCd);
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
        const Viterna v(as, -clMinE * (1 - p.postStallClDrop), cdS, p.flatPlateCd);
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

SectionResult evaluateSection(const SectionPolar& p, Real alpha, Real a, Real dAlpha0, Real dClMax) {
    alpha = wrapPi(alpha);
    SectionResult r;
    const Real dCl = -a * dAlpha0;
    if (std::abs(alpha) <= kHalfPi) {
        const Real alpha0 = p.zeroLiftAlpha + dAlpha0;
        r = forwardSection(p, alpha, a, alpha0, p.clMax + dClMax, p.clMin + dCl);
    } else {
        // Reversed flow: the trailing edge leads. Treat as a weaker symmetric
        // section (common practice for wind-turbine/rotor polars).
        const Real ar = alpha > 0 ? alpha - kPi : alpha + kPi;
        SectionPolar rev = p;
        rev.zeroLiftAlpha = 0;
        rev.clAtCdMin = 0;
        r = forwardSection(rev, ar, a, 0, 0.7 * p.clMax, -0.7 * p.clMax);
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
            addPanel(static_cast<int>(s), p, +1);
            if (p.mirrored) addPanel(static_cast<int>(s), p, -1);
            const Real area = 0.5 * (p.rootChord + p.tipChord) * p.span * (p.mirrored ? 2 : 1);
            sweepArea += p.sweep * area;
            spanSum += p.span;
            anyVertical = anyVertical || p.vertical;
            anyMirrored = anyMirrored || p.mirrored;
        }
        Surface& S = surfaces_.back();
        S.endStrip = static_cast<int>(strips_.size());
        Vec3 c;
        for (int i = S.firstStrip; i < S.endStrip; ++i) {
            S.area += strips_[i].area;
            c += strips_[i].position * strips_[i].area;
        }
        S.centroid = S.area > 0 ? c / S.area : Vec3{};
        S.sweep = S.area > 0 ? sweepArea / S.area : 0;
        S.horizontal = !anyVertical;
        S.span = anyMirrored ? 2 * spanSum : spanSum;
        // A fin on a fuselage behaves like a surface of ~1.55x its geometric
        // aspect ratio (end-plate effect of fuselage and tailplane; DATCOM 5.3).
        S.aspectRatio = S.area > 0 ? sq(S.span) / S.area * (anyVertical && !anyMirrored ? 1.55 : 1.0) : 0;
        S.liftSlope = liftCurveSlope3D(S.aspectRatio, S.sweep, S.def.section.liftSlope);
    }
    for (Surface& S : surfaces_) {
        const int src = S.def.downwashSource;
        if (src >= 0 && src < static_cast<int>(surfaces_.size())) {
            const Surface& W = surfaces_[src];
            S.downwashGradient = S.def.downwashGradient >= 0 ? S.def.downwashGradient
                                                             : 2 * W.liftSlope / (kPi * W.aspectRatio);
        }
    }
}

void AeroModel::addPanel(int si, const PanelDef& p, int side) {
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
        s.area = s.chord * p.span / p.strips;
        const Real inc = lerp(p.rootIncidence, p.tipIncidence, t);
        s.chordAxis = chord0 * std::cos(inc) + normal0 * std::sin(inc);
        s.normalAxis = normal0 * std::cos(inc) - chord0 * std::sin(inc);
        s.liftAxis = cross(s.chordAxis, s.normalAxis);
        s.spanAxis = spanAxis;
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

Real AeroModel::controlDeflection(const StripControl& c, const ControlDeflections& d) const {
    Real u = 0;
    switch (c.channel) {
        case ControlChannel::Aileron: u = -d.aileron * c.sideSign; break;
        case ControlChannel::Elevator: u = -d.elevator; break;
        case ControlChannel::Rudder: u = -d.rudder; break;
        case ControlChannel::Flap: return clamp(d.flap, -c.maxNegative, c.maxPositive);
        case ControlChannel::None: return 0;
    }
    u = clamp(u, -1.0, 1.0);  // mechanical stops
    return u >= 0 ? u * c.maxPositive : u * c.maxNegative;
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
        // Downwash from the source surface (lagged if available).
        Real eps = 0;
        if (S.def.downwashSource >= 0) {
            const int src = S.def.downwashSource;
            const Real clSrc = out.surfaces[src].cl;
            const Real aSrc = surfaces_[src].liftSlope;
            const Real srcGe = groundEffectFactor(in.heightAboveGround, surfaces_[src].span);
            so.downwash = (S.def.downwashAtZeroLift + S.downwashGradient * clSrc / aSrc) * srcGe;
            eps = in.downwashLagged ? in.downwashLagged[si] : so.downwash;
        }

        // Pass 1: strip coefficients; accumulate per-side lift for induced angle.
        Real liftSide[2] = {0, 0}, qaSide[2] = {0, 0}, alphaSum = 0, areaSum = 0;
        for (int i = S.firstStrip; i < S.endStrip; ++i) {
            const Strip& st = strips_[i];
            StripTelemetry& tel = out.strips[i];
            tel = {};
            if (!st.attached) continue;
            const Vec3 r = st.position - in.cg;
            const Vec3 v = localFlow(S, st, r, in);
            const Vec3 vPerp = v - st.spanAxis * dot(v, st.spanAxis);
            const Real vp2 = dot(vPerp, vPerp);
            if (vp2 < 1e-8) continue;
            Real alpha = std::atan2(-dot(vPerp, st.normalAxis), dot(vPerp, st.chordAxis));
            if (S.horizontal) alpha -= eps;

            // Controls on this strip.
            Real dAlpha0 = 0, dClMax = 0, dCm = 0, dCd = 0;
            for (int c = 0; c < st.controlCount; ++c) {
                const StripControl& sc = st.controls[c];
                const Real delta = controlDeflection(sc, in.controls);
                const Real eta = sc.effectiveness.empty() ? 1.0 : sc.effectiveness(std::abs(delta));
                const Real dA = -sc.tau * eta * delta;
                dAlpha0 += dA;
                dClMax += sc.clMaxIncrementRatio * (-a3 * dA);
                dCm += sc.cmFactor * eta * delta;
                dCd += sc.profileDragPerRad2 * delta * delta;
            }
            SectionResult sec = evaluateSection(S.def.section, alpha, a3, dAlpha0, dClMax);
            sec.cm += dCm;
            sec.cd += dCd;
            // Compressibility drag rise (Korn + Lock).
            const Real m = std::sqrt(vp2) / in.speedOfSound;
            const Real cs = std::cos(S.sweep);
            const Real mdd = S.def.kornFactor / cs - S.def.thicknessRatio / (cs * cs) -
                             std::abs(sec.cl) / (10 * cs * cs * cs);
            const Real mcrit = mdd - 0.1077;  // (0.1/80)^(1/3)
            if (m > mcrit) sec.cd += 20 * std::pow(m - mcrit, 4);

            const Real q = 0.5 * rho * vp2;
            tel.alpha = alpha;
            tel.cl = sec.cl;
            tel.cd = sec.cd;
            tel.cm = sec.cm;
            tel.q = q;
            tel.flowPerp = vPerp;
            tel.stalled = sec.stalled;
            tel.stallAlpha = S.def.section.zeroLiftAlpha + dAlpha0 + (S.def.section.clMax + dClMax) / a3;
            const int sideIdx = st.side > 0 ? 0 : 1;
            liftSide[sideIdx] += q * st.area * sec.cl;
            qaSide[sideIdx] += q * st.area;
            alphaSum += alpha * st.area;
            areaSum += st.area;
            if (sec.stalled) ++so.stalledStrips;
        }
        const Real qaTot = qaSide[0] + qaSide[1];
        so.cl = qaTot > 0 ? (liftSide[0] + liftSide[1]) / qaTot : 0;
        so.alphaMean = areaSum > 0 ? alphaSum / areaSum : 0;
        // Induced angle from the strip's own loading (local lifting-line
        // approximation): d(cl * alpha_i)/dcl = 2 alpha_i, which is what makes
        // aileron yaw adverse and asymmetric stall yaw into the dropping wing.
        const Real inducedPerCl = 1.0 / (kPi * arEff * S.def.spanEfficiency);
        // Pass 2: forces and moments.
        for (int i = S.firstStrip; i < S.endStrip; ++i) {
            const Strip& st = strips_[i];
            const StripTelemetry& tel = out.strips[i];
            if (!st.attached || tel.q <= 0) continue;
            const Vec3 r = st.position - in.cg;
            Vec3 dragDir = normalized(-tel.flowPerp);
            if (S.horizontal && eps != 0) {
                // Downwash tilts the local flow (rotate about the lift axis).
                const Vec3 n = normalized(cross(st.liftAxis, dragDir));
                dragDir = normalized(dragDir * std::cos(eps) + n * std::sin(eps));
            }
            const Vec3 liftDir = normalized(cross(dragDir, st.liftAxis));
            const Real ai = tel.cl * inducedPerCl;
            const Real qa = tel.q * st.area;
            const Vec3 f = liftDir * (qa * tel.cl) + dragDir * (qa * (tel.cd + tel.cl * ai));
            out.force += f;
            out.moment += cross(r, f) + st.liftAxis * (qa * st.chord * tel.cm);
        }
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
        const Vec3 f = v * (-0.5 * rho * d.dragArea * length(v));
        out.force += f;
        out.moment += cross(r, f);
    }
}

}  // namespace avd
