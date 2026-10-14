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
#include "avd/math.hpp"

namespace avd {

struct SectionResult {
    Real cl = 0, cd = 0, cm = 0;
    bool stalled = false;
};

// Section coefficients at angle of attack alpha (any value, rad) with a
// zero-lift shift dAlpha0 (from flaps/controls) and clMax increment dClMax.
SectionResult evaluateSection(const SectionPolar& p, Real alpha, Real liftSlope, Real dAlpha0 = 0,
                              Real dClMax = 0);

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
    Real density = 1.225;
    Real speedOfSound = 340.3;
    Vec3 cg;                   // structural frame
    ControlDeflections controls;
    Real heightAboveGround = 1e9;  // of the wing reference (m)
    const SlipstreamState* slipstreams = nullptr;
    int slipstreamCount = 0;
    const Real* downwashLagged = nullptr;  // per surface, rad (size = surfaces)
};

struct SurfaceOutput {
    Real cl = 0;               // surface lift coefficient on its own area
    Real alphaMean = 0;        // area-weighted mean strip alpha
    Real downwash = 0;         // instantaneous downwash it generates on others' strips (rad)
    int stalledStrips = 0;
};

struct StripTelemetry {
    Real alpha = 0;
    Real stallAlpha = 0;       // positive stall angle incl. control effects
    Real cl = 0, cd = 0, cm = 0;
    Real q = 0;                // local dynamic pressure (0 = strip inactive)
    bool stalled = false;
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
};

class AeroModel {
public:
    explicit AeroModel(const AircraftDefinition& def);

    // Hot path: no allocation (outputs.strips must be sized via prepare()).
    void evaluate(const AeroInputs& in, AeroOutputs& out) const;
    void prepare(AeroOutputs& out) const { out.strips.assign(strips_.size(), {}); }

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
    };
    struct Strip {
        int surface;
        int side;               // +1 right/upper, -1 mirrored
        Vec3 position;          // quarter chord, structural frame
        Vec3 chordAxis;         // forward along the chord (incidence applied)
        Vec3 normalAxis;        // positive-lift side (incidence applied)
        Vec3 spanAxis;          // geometric spanwise unit vector
        Vec3 liftAxis;          // cross(chord, normal): lift = cross(-dragDir, liftAxis)
        Real area, chord;
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
        Vec3 centroid;
        bool horizontal = true;
    };

    void addPanel(int surfaceIndex, const PanelDef& panel, int side);
    Real controlDeflection(const StripControl& c, const ControlDeflections& d) const;
    Vec3 localFlow(const Surface& S, const Strip& st, const Vec3& r, const AeroInputs& in) const;
    Vec3 slipstreamAt(const Vec3& position, const AeroInputs& in) const;

    std::vector<Surface> surfaces_;
    std::vector<Strip> strips_;
    std::vector<FuselageDef> fuselages_;
    std::vector<DragItemDef> dragItems_;
};

}  // namespace avd
