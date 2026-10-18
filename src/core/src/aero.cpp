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

        // Induced angle from the strip's own loading (local lifting-line
        // approximation): d(cl * alpha_i)/dcl = 2 alpha_i, which is what makes
        // aileron yaw adverse and asymmetric stall yaw into the dropping wing.
        const Real inducedPerCl = 1.0 / (kPi * arEff * S.def.spanEfficiency);
        // Finite-wing post-stall drag (Viterna), never above the 2-D flat plate.
        const Real cdMax = std::min(S.def.section.flatPlateCd, finiteWingMaxDrag(S.aspectRatio));
        Real liftSum = 0, qaFree = 0, alphaSum = 0, areaSum = 0;
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
            // Simple sweep theory: the section works on the flow normal to its
            // span. Its streamwise-defined polar maps there with the sweep:
            // angles, slope and lift limits over cos(sweep), so that q_n cl_n
            // = q CL (linear range unchanged), an untwisted wing's zero-lift
            // angle is the section's at any sweep, and CLmax ~ clmax cos(sweep).
            const Real cs = st.cosSweep;
            Real alpha = std::atan2(-dot(vPerp, st.normalNormal), dot(vPerp, st.chordNormal));
            if (S.horizontal) alpha -= eps / cs;
            SectionPolar sp = S.def.section;
            sp.zeroLiftAlpha /= cs;
            sp.clMax /= cs;
            sp.clMin /= cs;
            sp.clAtCdMin /= cs;
            const Real an = a3 / cs;

            // Controls on this strip (deflections normal to the hinge line).
            Real dAlpha0 = 0, dClMax = 0, dCm = 0, dCd = 0, extension = 0;
            for (int c = 0; c < st.controlCount; ++c) {
                const StripControl& sc = st.controls[c];
                const Real delta = controlDeflection(sc, in.controls);
                const Real eta = sc.effectiveness.empty() ? 1.0 : sc.effectiveness(std::abs(delta));
                const Real dA = -sc.tau * eta * delta;
                dAlpha0 += dA;
                dClMax += sc.clMaxIncrementRatio * (-an * dA);
                dCm += sc.cmFactor * eta * delta;
                dCd += sc.profileDragPerRad2 * delta * delta;
                if (sc.chordExtension > 0 && delta > 0 && sc.maxPositive > 0)
                    extension += sc.chordExtension * std::min(delta / sc.maxPositive, 1.0);
            }
            SectionResult sec = evaluateSection(sp, alpha, an, dAlpha0, dClMax, cdMax);
            sec.cm += dCm;
            sec.cd += dCd;
            // Spoilers / lift dump on the main wing: a fraction of the
            // section lift is spoiled (the drag is carried by a drag area).
            if (si == 0 && in.controls.spoiler > 0) sec.cl *= 1 - in.controls.spoiler;
            // Compressibility drag rise (Korn + Lock), on the normal flow:
            // Korn's equation is written for the free-stream Mach number and
            // the streamwise lift coefficient (cl_streamwise = cl_n cos^2), and
            // Lock's increment is streamwise (normal: divide by cos^2).
            const Real m = std::sqrt(vp2) / in.speedOfSound / cs;
            const Real mdd = S.def.kornFactor / cs - S.def.thicknessRatio / (cs * cs) - std::abs(sec.cl) / (10 * cs);
            const Real mcrit = mdd - 0.1077;  // (0.1/80)^(1/3)
            const Real cdWave = m > mcrit ? 20 * std::pow(m - mcrit, 4) / (cs * cs) : 0;

            const Real q = 0.5 * rho * vp2;
            const Real qFree = 0.5 * rho * dot(v, v);
            // A Fowler flap's chord extension: more lifting area.
            const Real area = st.area * (1 + extension), chord = st.chord * (1 + extension);
            // Angles reported streamwise-equivalent (normal-plane angle x
            // cos(sweep)), so margins in radians mean the same on every wing.
            tel.alpha = alpha * cs;
            tel.cl = sec.cl;
            tel.cd = sec.cd + cdWave;
            tel.cm = sec.cm;
            tel.q = q;
            tel.flowPerp = vPerp;
            tel.stalled = sec.stalled;
            tel.attached = sec.attached;
            tel.stallAlpha = (sp.zeroLiftAlpha + dAlpha0 + (sp.clMax + dClMax) / an) * cs;
            liftSum += q * area * sec.cl;
            qaFree += qFree * area;
            alphaSum += alpha * cs * st.area;
            areaSum += st.area;
            if (sec.stalled) ++so.stalledStrips;

            // Forces. Lift is normal to both the strip's (spanwise-free) flow
            // and its swept span axis; the unswept lateral axis would tilt it
            // forward by ~sin(alpha) sin^2(sweep) and cancel induced drag.
            Vec3 dragDir = normalized(-vPerp);
            const Vec3 la = dot(st.spanAxis, st.liftAxis) >= 0 ? st.spanAxis : -st.spanAxis;
            if (S.horizontal && eps != 0) {
                // Downwash tilts the local flow (rotate about the span axis).
                const Vec3 n = normalized(cross(la, dragDir));
                dragDir = normalized(dragDir * std::cos(eps / cs) + n * std::sin(eps / cs));
            }
            const Vec3 liftDir = normalized(cross(dragDir, la));
            // Induced angle in the normal plane from the streamwise CL (cl_n
            // cos^2), fading as the flow separates (a stalled strip sheds its
            // vorticity rather than trailing it).
            const Real ai = sec.cl * cs * cs * inducedPerCl * sec.attached;
            const Real qa = q * area;
            // Attached profile drag is skin friction and pressure drag of the
            // whole local flow (spanwise flow included); the separated part,
            // the wave drag and the induced drag act on the normal flow.
            const Real cdFriction = sec.attached * sec.cd;
            const Real cdNormal = (1 - sec.attached) * sec.cd + cdWave;
            const Vec3 f = liftDir * (qa * sec.cl) + dragDir * (qa * (cdNormal + sec.cl * ai)) +
                           normalized(-v) * (qFree * area * cdFriction);
            out.force += f;
            out.moment += cross(r, f) + st.liftAxis * (qa * chord * sec.cm);
        }
        // The surface's streamwise lift coefficient (on the free-stream q),
        // the input of the downwash it makes.
        so.cl = qaFree > 0 ? liftSum / qaFree : 0;
        so.alphaMean = areaSum > 0 ? alphaSum / areaSum : 0;
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
    in.density = 1.225;
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
