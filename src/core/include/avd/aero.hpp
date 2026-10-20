// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
//
// Component build-up aerodynamics (strip theory).
//
// Each lifting surface is split into spanwise strips. Every strip sees its own
// local airflow: aircraft velocity, rotation (omega x r), the atmosphere's
// spatially varying wind, wing downwash (with a transport lag) and propeller
// slipstream. Its section polar covers the full -180..180 deg range (attached
// lift, a smooth stall, Viterna-Corrigan post-stall, reversed flow), so
// stall/spin asymmetry, roll and pitch damping, dihedral effect and
// weathercock stability emerge from geometry instead of global derivatives.
// Removing a strip (damage, separation) removes exactly its contribution.
//
// Finite-wing effects use the uniform-downwash lifting-line approximation:
// strips use the surface's 3-D lift-curve slope (Helmbold/DATCOM) and the
// lift vector is tilted by the induced angle CL/(pi AR e), reduced in ground
// effect (McCormick). See docs/architecture/flight-dynamics.md.
#pragma once

#include <array>
#include <vector>

#include "avd/aircraft_def.hpp"
#include "avd/atmosphere.hpp"
#include "avd/math.hpp"

namespace avd {

struct SectionResult {
    Real cl = 0, cd = 0, cm = 0;
    bool stalled = false;
    Real attached = 1;  // weight of the attached-flow polar (1 attached, 0 separated)
};

// Section coefficients at angle of attack alpha (any value, rad) with a
// zero-lift shift dAlpha0 (from flaps/controls) and clMax increment dClMax.
// cdMax: the post-stall drag at 90 deg (Viterna's CDmax); <= 0 uses the
// polar's 2-D flat-plate value.
SectionResult evaluateSection(const SectionPolar& p, Real alpha, Real liftSlope, Real dAlpha0 = 0,
                              Real dClMax = 0, Real cdMax = 0);

// Viterna & Corrigan's finite-wing CDmax = 1.11 + 0.018 AR (AR <= 50).
Real finiteWingMaxDrag(Real aspectRatio);

// Thin-airfoil flap effectiveness tau(cf/c).
Real flapEffectiveness(Real chordFraction);

// Helmbold/DATCOM 3-D lift-curve slope for aspect ratio AR, sweep (rad),
// 2-D slope a2d and Prandtl-Glauert factor beta = sqrt(1 - M^2).
Real liftCurveSlope3D(Real aspectRatio, Real sweep, Real a2d, Real beta = 1.0);

// McCormick ground-effect factor on induced drag, h = height of the wing
// above ground, b = span. 1 out of ground effect, -> 0 at the ground.
Real groundEffectFactor(Real height, Real span);

// Pilot-sense control positions in [-1, 1] plus actual flap angle.
struct ControlDeflections {
    Real aileron = 0;   // + roll right
    Real elevator = 0;  // + nose up (stick aft); includes trim offset in rad-equivalent
    Real rudder = 0;    // + yaw right
    Real flap = 0;      // rad, trailing edge down
    Real spoiler = 0;   // fraction of main-wing lift spoiled (speedbrake / lift dump)
    Real stabilizer = 0;  // rad, trimmable horizontal stabiliser, + nose up (leading edge down)
    Real speedbrake = 0;  // 0..1, spoiler panels' deployment
};

// Propeller slipstream as seen by downstream surfaces.
struct SlipstreamState {
    Vec3 discCentre;    // structural frame
    Vec3 axis{1, 0, 0}; // body, unit, thrust direction
    Real radius = 0;
    Real axialIncrement = 0;  // m/s added to the far-wake axial velocity
    Real swirlVelocity = 0;   // m/s tangential at 0.7 R, sign along rotation
    bool clockwiseFromBehind = true;
};

struct AeroInputs {
    Vec3 velocityBody;         // CG velocity relative to the ground, body axes
    Vec3 omegaBody;            // rad/s
    Vec3 windBody;             // air velocity at the CG, body axes
    Mat3 windGradientBody = Mat3::zero();  // d(wind)/d(position), body axes
    Real density = isa::kSeaLevelDensity;
    Real speedOfSound = 340.3;
    Vec3 cg;                   // structural frame
    ControlDeflections controls;
    Real heightAboveGround = 1e9;  // of the wing reference (m)
    Vec3 groundUpBody;             // unit normal of the ground (up), body axes; zero: no ground images
    const SlipstreamState* slipstreams = nullptr;
    int slipstreamCount = 0;
    const Real* downwashLagged = nullptr;  // per surface, rad (size = surfaces)
};

struct SurfaceOutput {
    Real cl = 0;               // surface lift coefficient on its own area
    Real alphaMean = 0;        // area-weighted mean strip alpha
    Real downwash = 0;         // instantaneous downwash it generates on others' strips (rad)
    int stalledStrips = 0;
    Real separated = 0;        // area-weighted separated fraction (1 - attached) of its strips
    Real wakeImmersion = 0;    // 0..1: how far the surface sits in its downwash source's stalled wake
};

// Section values are in the plane normal to the strip's span (simple sweep
// theory): cl, cd and q are those of the normal flow; alpha and stallAlpha
// are the normal-plane angles times cos(sweep) (streamwise-equivalent).
struct StripTelemetry {
    Real alpha = 0;
    Real stallAlpha = 0;       // positive stall angle incl. control effects
    Real zeroLiftAlpha = 0;    // zero-lift angle incl. control effects (flaps)
    Real cl = 0, cd = 0, cm = 0;
    Real q = 0;                // local dynamic pressure (0 = strip inactive)
    Vec3 flowPerp;             // strip velocity relative to air, spanwise part removed (body)
    bool stalled = false;
    Real attached = 1;         // attached-flow weight of the section polar (tufts, buffet)
    Real gamma = 0;            // bound circulation (m^2/s; lifting-line surfaces)
};

struct AeroOutputs {
    Vec3 force;                // body axes, N
    Vec3 moment;               // about the CG, body axes, N m
    Real alpha = 0, beta = 0;  // at the CG (rad)
    Real airspeed = 0;         // true airspeed at the CG (m/s)
    Real mach = 0;
    Real dynamicPressure = 0;
    std::array<SurfaceOutput, 8> surfaces{};
    std::vector<StripTelemetry> strips;  // sized at construction; no allocation per step
    // Lifting-line working storage (sized by prepare()).
    std::vector<Vec3> llFlow, llInduced;   // per strip: free flow, induced air velocity
    std::vector<Vec3> llInfluence;         // per surface, n x n: air velocity at i per unit circulation of j
    std::vector<Real> llJacobian, llResidual, llGamma, llSeparated;
    std::vector<Vec3> llGrad;
    // Per strip: zero-lift shift, clmax increment / slope, cm, cd, chord
    // extension, spoiled lift fraction.
    std::vector<std::array<Real, 6>> llControl;
    std::array<Vec3, 8> llDir{};           // flow direction the influence was built for
    std::array<Real, 8> llHeight{};
    std::array<bool, 8> llValid{};
    int llIterations = 0;                  // Newton iterations of the last solve (diagnostics)
};

class AeroModel {
public:
    explicit AeroModel(const AircraftDefinition& def);

    // Hot path: no allocation (outputs.strips must be sized via prepare()).
    void evaluate(const AeroInputs& in, AeroOutputs& out) const;
    void prepare(AeroOutputs& out) const;

    int surfaceCount() const { return static_cast<int>(surfaces_.size()); }
    int stripCount() const { return static_cast<int>(strips_.size()); }
    // First/one-past-last strip index of a surface (for stall-warning vanes).
    int firstStrip(int surface) const { return surfaces_[surface].firstStrip; }
    int endStrip(int surface) const { return surfaces_[surface].endStrip; }
    Real surfaceArea(int s) const { return surfaces_[s].area; }
    Real surfaceSpan(int s) const { return surfaces_[s].span; }
    Real surfaceAspectRatio(int s) const { return surfaces_[s].aspectRatio; }
    Real surfaceLiftSlope(int s) const { return surfaces_[s].liftSlope; }
    Real downwashGradient(int s) const { return surfaces_[s].downwashGradient; }
    int downwashSource(int s) const { return surfaces_[s].def.downwashSource; }
    Vec3 surfaceCentroid(int s) const { return surfaces_[s].centroid; }
    // A strip of the surface's main plane (within 45 deg of its area-weighted
    // normal): a winglet or other canted panel is not, and does not stand in
    // for the wing where a stall angle is scanned (stall warning, pusher,
    // assist margins); its own stall is still in the forces.
    bool stripIsPrimary(int i) const {
        const Strip& s = strips_[static_cast<std::size_t>(i)];
        return dot(s.normalAxis, surfaces_[static_cast<std::size_t>(s.surface)].meanNormal) > 0.7071;
    }
    // Strip geometry for visualisation and analysis (structural frame).
    struct StripGeometry {
        int surface = 0, side = 1;
        Vec3 position, chordAxis, normalAxis, spanAxis;
        Real area = 0, chord = 0;
        Real width = 0;  // length along spanAxis (m)
        unsigned controls = 0;  // bit (1 << channel) for each control surface on the strip
    };
    StripGeometry stripGeometry(int i) const {
        const Strip& s = strips_[static_cast<std::size_t>(i)];
        unsigned mask = 0;
        for (int c = 0; c < s.controlCount; ++c) mask |= 1u << static_cast<unsigned>(s.controls[static_cast<std::size_t>(c)].channel);
        return {s.surface, s.side, s.position, s.chordAxis, s.normalAxis, s.spanAxis, s.area, s.chord, s.width, mask};
    }

    // Detach (or re-attach) every strip of a surface on one side (+1 right, -1
    // left, 0 both). Used by the damage system and tests.
    void setSurfaceAttached(int surface, int side, bool attached);

private:
    struct StripControl {
        ControlChannel channel;
        Real sideSign;          // +1 or -1 (antisymmetric ailerons)
        Real maxPositive, maxNegative;
        Real tau;               // thin-airfoil effectiveness
        Real cmFactor;          // dcm/ddelta (thin airfoil)
        Table1D effectiveness;  // eta(|delta|), empty = 1
        Real clMaxIncrementRatio = 0.6;
        Real profileDragPerRad2 = 0.9;
        Real chordExtension = 0;  // Fowler: chord added at full positive deflection (fraction of c)
        Real chordFraction = 0.25;
    };
    struct Strip {
        int surface;
        int side;               // +1 right/upper, -1 mirrored
        Vec3 position;          // quarter chord, structural frame
        Vec3 chordAxis;         // forward along the chord (incidence applied)
        Vec3 normalAxis;        // positive-lift side (incidence applied)
        Vec3 spanAxis;          // geometric spanwise unit vector
        Vec3 liftAxis;          // cross(chord, normal): lift = cross(-dragDir, liftAxis)
        // The section plane normal to the span (simple sweep theory): chord
        // and normal axes in it, and the cosine of the local sweep.
        Vec3 chordNormal, normalNormal;
        Real cosSweep = 1;
        Vec3 boundA, boundB;    // bound vortex ends at the quarter chord (A -> B along the lift axis)
        int neighbourA = -1, neighbourB = -1;  // strips sharing the A / B edge (same surface)
        Real area, chord;
        Real width;             // length along spanAxis
        bool attached = true;
        int controlCount = 0;
        std::array<StripControl, 3> controls{};
    };
    struct Surface {
        LiftingSurfaceDef def;
        int firstStrip = 0, endStrip = 0;
        Real area = 0, span = 0, aspectRatio = 0, sweep = 0;
        Real liftSlope = 0;     // incompressible 3-D slope, per rad
        Real downwashGradient = 0;
        Real wakeDownwash = 0;  // downwash the source's stalled wake holds (rad), wake blanking
        Real meanChord = 0;     // area / span
        Vec3 centroid;
        Vec3 meanNormal{0, 0, -1};  // area-weighted strip normal (unit): the surface's main plane
        bool horizontal = true;
        // Lifting line: spanwise smoothing of the control effects (n x n,
        // rows summing to 1). A flap's lift carries past its end over about
        // half a chord; a step at a strip edge would put a line vortex half a
        // strip from the neighbouring control point.
        std::vector<Real> controlSmoothing;
    };

    void addPanel(int surfaceIndex, const PanelDef& panel, int side);
    Real controlDeflection(const StripControl& c, const ControlDeflections& d) const;
    std::array<Real, 6> controlEffects(const Strip& st, const ControlDeflections& d) const;
public:
    bool hasSpoilers() const { return hasSpoilers_; }
    // Deployed spoiler panels as bluff bodies (airflow view): centre on the
    // upper surface, span axis, projected height and drag area (m^2), for a
    // speedbrake deployment 0..1. Returns how many were written (<= max).
    struct BluffBody {
        Vec3 position, axis;
        Real size = 0, length = 0, dragArea = 0;
    };
    int spoilerBodies(Real speedbrake, BluffBody* out, int max) const;
private:
    bool hasSpoilers_ = false;
    Vec3 localFlow(const Surface& S, const Strip& st, const Vec3& r, const AeroInputs& in) const;
    Vec3 slipstreamAt(const Vec3& position, const AeroInputs& in) const;
    // Nonlinear lifting line (Phillips & Snyder 2000) over strips [n0, n0+n)
    // of surface si: fills out.llInduced. gammaOf(k, v, attached) is the
    // circulation the section of strip k carries at strip velocity v (and its
    // attached-flow weight).
    template <class GammaOf>
    void solveLiftingLine(int si, int n0, int n, const AeroInputs& in, AeroOutputs& out, GammaOf&& gammaOf) const;
    std::array<int, 9> llOffset_{};  // start of each surface's influence block

    std::vector<Surface> surfaces_;
    std::vector<Strip> strips_;
    std::vector<FuselageDef> fuselages_;
    std::vector<DragItemDef> dragItems_;
};

// Highest lift coefficient (on refArea) the airframe reaches in steady 1-g
// flight with the elevator trimming the pitching moment about cg, for a flap
// angle (rad) and Mach number: the basis of stall and V-speeds. Sweeps alpha
// out of ground effect with no slipstream; elevator saturating limits it.
Real trimmedMaxLiftCoefficient(const AeroModel& model, Real refArea, const Vec3& cg, Real flap, Real mach);

}  // namespace avd
