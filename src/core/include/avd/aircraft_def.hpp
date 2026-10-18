// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
//
// AircraftDefinition: plain data describing one physical aircraft. It is the
// only place aircraft-specific numbers live; the engine contains no per-type
// code paths. Definitions are normally loaded from JSON (avd/aircraft_loader.hpp,
// schema in docs/architecture/aircraft-definition-schema.md) and are untrusted
// input: validate() checks every field before an Aircraft is built.
//
// Frames: positions are in the structural body frame (origin at the datum
// declared by the definition, axes FRD: X forward, Y right, Z down). Angles are
// radians, masses kg, lengths m, everything SI. Assistance presets never touch
// this struct (docs/decisions/0004-assistance-is-not-physics.md).
#pragma once

#include <string>
#include <vector>

#include "avd/math.hpp"
#include "avd/panel_def.hpp"
#include "avd/table.hpp"

namespace avd {

// Where a number came from (docs/architecture/fidelity-and-provenance.md).
enum class Provenance {
    Published,     // 1. published / measured (POH, TCDS, manufacturer data)
    Calibrated,    // 2. engineering model calibrated to published performance
    Estimated,     // 3. reasoned approximation (data unavailable)
    Gameplay,      // 4. explicitly artificial gameplay value
};

struct SourceRef {
    std::string id;        // e.g. "C172S-POH-R5"
    std::string citation;  // human-readable reference
};

struct ProvenanceNote {
    std::string field;     // dotted path in the definition
    Provenance kind = Provenance::Estimated;
    std::string source;    // SourceRef id
    std::string note;
};

// 2-D section aerodynamics, defined over the full -180..180 deg range:
// attached-flow linear lift with a smooth stall into a flat-plate model.
struct SectionPolar {
    Real liftSlope = 2 * kPi * 0.95;  // per rad (2-D)
    Real zeroLiftAlpha = 0;           // rad
    Real clMax = 1.5;                 // positive stall
    Real clMin = -1.1;                // negative stall
    Real cdMin = 0.008;               // minimum profile drag
    Real cdQuadratic = 0.006;         // profile drag rise: cd = cdMin + k (cl - clCdMin)^2
    Real clAtCdMin = 0.2;
    Real cm0 = -0.05;                 // about the quarter chord
    Real stallWidth = 0.07;           // rad, width of the stall transition (sharpness)
    Real postStallClDrop = 0.25;      // fraction of clMax lost immediately post-stall
    Real flatPlateCd = 1.9;           // cd at 90 deg (finite-aspect flat plate ~1.2-2.0)
};

enum class ControlChannel { Aileron, Elevator, Rudder, Flap, None };

// A plain flap-type control surface occupying part of a panel's span.
struct ControlSurfaceDef {
    std::string name;
    ControlChannel channel = ControlChannel::None;
    Real spanStart = 0, spanEnd = 1;  // fraction of the panel span (inboard -> outboard)
    Real chordFraction = 0.25;        // cf/c
    Real maxPositive = 0.35;          // rad, trailing edge down (positive deflection)
    Real maxNegative = 0.35;          // rad, trailing edge up (magnitude)
    // Ailerons deflect antisymmetrically on mirrored panels.
    bool antisymmetric = false;
    // Flap effectiveness correction for large deflections (DATCOM-style
    // eta(delta)); empty = ideal thin-airfoil effectiveness.
    Table1D effectivenessVsDeflection;
    Real clMaxIncrementRatio = 0.6;   // dclmax / dcl (flaps raise clmax less than cl)
    Real profileDragPerRad2 = 0.3;    // extra section cd per rad^2 of deflection
};

// One trapezoidal panel. A wing is a list of panels; `mirrored` panels are
// reflected about the XZ plane (Y -> -Y).
struct PanelDef {
    Vec3 rootQuarterChord;            // structural frame
    Real span = 1;                    // panel span along its own spanwise axis, m
    Real rootChord = 1, tipChord = 1;
    Real sweep = 0;                   // quarter-chord sweep, rad
    Real dihedral = 0;                // rad (for a vertical panel: ignored)
    Real rootIncidence = 0, tipIncidence = 0;  // rad, positive nose up
    bool mirrored = true;
    bool vertical = false;            // fin: span runs up (-Z)
    int strips = 4;
    std::vector<ControlSurfaceDef> controls;
};

struct LiftingSurfaceDef {
    std::string name;
    std::vector<PanelDef> panels;
    SectionPolar section;
    Real spanEfficiency = 0.8;        // Oswald e for the surface's induced drag
    // Downwash from another surface (index into surfaces), e.g. wing on tail:
    // epsilon = downwashAtZeroLift + downwashGradient * (alpha_source - alpha0_source).
    int downwashSource = -1;
    Real downwashGradient = -1;       // < 0: estimate as 2 CLa/(pi AR) of the source
    Real downwashAtZeroLift = 0;
    // Fraction of the surface bathed in propeller slipstream (0..1) and the
    // engine whose slipstream it is.
    Real slipstreamExposure = 0;
    int slipstreamEngine = -1;
    bool groundEffect = true;
    // Compressibility (subsonic model: Prandtl-Glauert lift slope + Korn/Lock
    // drag rise). Valid up to the drag-divergence regime; supersonic
    // aerodynamics are a separate model (Milestone 7).
    Real thicknessRatio = 0.12;
    Real kornFactor = 0.87;           // 0.87 conventional, ~0.95 supercritical
};

// Slender-body fuselage: Munk moment (destabilising pitch/yaw) and crossflow
// side force; parasite drag is declared separately as drag items.
struct FuselageDef {
    std::string name = "fuselage";
    Vec3 referencePoint;              // structural frame, roughly the volume centroid
    Real volume = 0;                  // m^3
    Real munkFactor = 0.8;            // (k2 - k1) apparent-mass factor
    Real crossflowArea = 0;           // planform/side area for crossflow drag, m^2
    Real crossflowCd = 1.0;
};

struct DragItemDef {
    std::string name;
    Vec3 position;
    Real dragArea = 0;                // Cd*A, m^2 (along the local airflow)
    // Transonic drag rise of a body (fuselage, nacelles): above this Mach the
    // drag area grows by 1 + 60 (M - Mcrit)^2, ~2.4x at Mcrit + 0.15. 0 = none.
    Real criticalMach = 0;
};

// ---- Propulsion -----------------------------------------------------------

struct PropellerDef {
    Real diameter = 1.9;              // m
    int blades = 2;
    Real pitch = 1.5;                 // m, geometric pitch (constant along the blade)
    Real hubRadiusFraction = 0.15;
    Table1D chordVsRadius;            // chord (m) vs r/R
    // Thickness ratio t/c vs r/R for the compressibility drag rise of the
    // blade elements (Korn + Lock, as on the wing). Empty: incompressible.
    Table1D thicknessVsRadius;
    Real kornFactor = 0.87;           // 0.87 conventional sections, 0.95 supercritical
    SectionPolar section;             // blade airfoil
    Real inertia = 1.6;               // kg m^2 (propeller alone)
    int bemElements = 24;
};

struct PistonEngineDef {
    std::string name = "engine";
    Vec3 position;                    // propeller hub, structural frame
    Vec3 thrustAxis{1, 0, 0};         // unit, body
    bool clockwiseFromBehind = true;  // rotation sense seen from the cockpit
    Real ratedPower = 134226;         // W, brake, sea-level ISA, full throttle, best power
    Real ratedRpm = 2700;
    Real displacement = 5.916e-3;     // m^3
    Real volumetricEfficiency = 0.85;
    Real frictionMepAtRated = 1.6e5;  // Pa, friction+pumping mean effective pressure
    Real frictionMepAtZero = 0.6e5;   // Pa
    Real idleManifoldRatio = 0.33;    // MAP / ambient at closed throttle
    Real intakeLossRatio = 0.03;      // MAP loss at full throttle
    Real engineInertia = 0.6;         // kg m^2 crankshaft etc. (propeller separate)
    Real starterTorque = 60;          // N m at the crankshaft
    Real starterCurrent = 150;        // A
    Real minimumFiringRpm = 120;
    Real singleMagnetoPowerFactor = 0.965;
    Real mixtureRichPhi = 1.25;       // equivalence ratio at full-rich, sea level
    Real fuelHeatingValue = 43.5e6;   // J/kg (avgas lower heating value)
    Real stoichiometricFar = 0.0676;  // fuel/air mass ratio
    Table1D powerVsPhi;               // relative power vs equivalence ratio (1 at best power)
    Real pFactorGain = 0.12;          // asymmetric disk loading, Estimated
    Real slipstreamSwirlFactor = 1.0; // multiplier on momentum-theory swirl
    Real alternatorMinRpm = 900;
    // Oil system (Estimated): positive-displacement pump pressure grows with
    // rpm and oil viscosity until the relief valve opens.
    Real oilPumpPressurePerRpm = 310;         // Pa per rpm at 190 degF oil
    Real oilReliefPressure = 5.5e5;           // Pa (~80 psi)
    Real oilViscosityTemperatureScale = 38.9; // K: pressure x e when oil is this much colder
    int fuelFeed = 0;                 // index of the fuel feed this engine draws from
    PropellerDef propeller;
};

// Two-spool high-bypass turbofan (docs/architecture/propulsion.md). Net
// thrust at full throttle lapses with altitude and Mach number after Mattingly
// ("Elements of Gas Turbine Propulsion", eq. 2.54a, high bypass ratio):
//   F/F0 = delta0 (1 - 0.49 sqrt(M))                       theta0 <= TR
//        = delta0 (1 - 0.49 sqrt(M) - 3 (theta0 - TR)/(1.5 + M))  theta0 > TR
// with delta0, theta0 the free-stream total pressure and temperature ratios.
// Part throttle follows the fan speed N1; the spools lag the throttle, and
// fuel flow is thrust times a TSFC that rises with Mach number.
struct TurbofanDef {
    std::string name = "engine";
    Vec3 position;                    // thrust line point (nozzle exit), structural frame
    Vec3 thrustAxis{1, 0, 0};         // unit, body
    Real staticThrust = 75000;        // N, sea-level ISA, take-off rating, M = 0
    Real bypassRatio = 4.4;
    Real throttleRatio = 1.0;         // TR: theta0 at which the engine reaches its temperature limit
    Real lapseMachCoefficient = 0.49; // the 0.49 in the lapse law
    Real idleN1 = 0.22;               // fan speed, fraction of take-off N1
    Real idleN2 = 0.6;                // core speed, fraction
    Real idleThrustFraction = 0.035;  // of the available thrust, at idle N1
    Real thrustExponent = 1.8;        // thrust fraction vs normalised N1 above idle
    Real accelerationTime = 5.0;      // s, idle to 95 % thrust
    Real decelerationTime = 4.0;      // s, take-off to idle
    Real startTime = 35.0;            // s, starter engaged to stabilised idle
    Real tsfcStatic = 1.10e-5;        // kg/(N s) at M = 0, sea level (0.39 lb/lbf/h)
    Real tsfcMachSlope = 0.9;         // TSFC grows by this fraction per unit Mach
    Real idleFuelFlow = 0.045;        // kg/s, sea-level ground idle
    Real ittIdle = 720;               // K
    Real ittTakeoff = 1150;           // K
    Real oilPressureIdle = 2.4e5;     // Pa
    Real oilPressureMax = 4.1e5;      // Pa
    int fuelFeed = 0;
};

// Retractable landing gear and speedbrakes (both optional).
struct GearRetractionDef {
    bool retractable = false;
    Real transitTime = 8.0;           // s, either way
    Real dragArea = 0;                // m^2 of CdA, all legs down and doors open
    Real maxExtendedIas = 0;          // m/s (VLE), exceedance reporting
};

struct SpeedbrakeDef {
    bool present = false;
    int detents = 3;                  // RET, ..., FULL
    Real dragArea = 0;                // m^2 of CdA fully deployed
    Real liftLossFraction = 0;        // wing lift spoiled when fully deployed (lift dump)
    Real rate = 0.5;                  // deployment fraction per second
};

// ---- Fuel -----------------------------------------------------------------

struct FuelTankDef {
    std::string name;
    Vec3 position;                    // CG of fuel (fixed in M1; sloshing later)
    Real capacity = 0;                // m^3 total
    Real unusable = 0;                // m^3 that cannot be drawn
    Real initial = 0;                 // m^3
};

// A selector with named positions, each opening a set of tanks.
struct FuelSelectorPosition {
    std::string name;
    std::vector<int> tanks;
};

struct FuelSystemDef {
    Real density = 720.0;             // kg/m^3 (avgas 100LL at 15 degC ~0.72 kg/L)
    std::vector<FuelTankDef> tanks;
    std::vector<FuelSelectorPosition> selector;
    int initialSelector = 0;
    Real lineVolume = 0.5e-3;         // m^3 between tanks and engine (starvation delay)
    bool hasShutoffValve = true;
    bool requiresAuxPumpForStart = false;
};

// ---- Electrical (single-bus light aircraft architecture in M1) ----------

struct ElectricalLoadDef {
    std::string name;
    std::string label;                // cockpit switch/breaker label (default: name)
    Real current = 1;                 // A when on
    Real minimumVoltage = 18;
    bool onAvionicsBus = false;
    bool initiallyOn = true;          // switch position on a fresh airframe
};

struct ElectricalDef {
    Real batteryVoltage = 24.0;       // open circuit, full
    Real batteryCapacity = 13.0 * 3600;  // A s
    Real batteryResistance = 0.05;    // ohm
    Real alternatorVoltage = 28.0;
    Real alternatorMaxCurrent = 60;
    std::vector<ElectricalLoadDef> loads;
};

// ---- Landing gear and contact --------------------------------------------

struct GearDef {
    std::string name;
    Vec3 attach;                      // strut top, structural frame
    Real length = 0.5;                // attach -> tyre contact at full extension (m), along +Z body
    Real stiffness = 6e4;             // N/m (strut+tyre lumped)
    Real preload = 0;                 // N
    Real dampingCompression = 3e3;    // N s/m
    Real dampingRebound = 6e3;
    Real maxCompression = 0.25;       // m; beyond this the stiffness rises 20x (bottoming)
    Real wheelRadius = 0.2;
    Real rollingResistance = 0.02;    // coefficient on a paved surface
    Real maxBrakeForceCoefficient = 0;  // brake force capacity / normal load (0 = no brake)
    int brakeGroup = -1;              // 0 = left, 1 = right
    Real maxSteerAngle = 0;           // rad, by the rudder pedals
    // Tiller (nose-wheel steering handwheel on transport aircraft): wheel angle
    // at full tiller (0 = no tiller). Tiller and pedal commands add, limited
    // to the larger of the two authorities.
    Real tillerSteerAngle = 0;        // rad
    // Slew-rate limit of a powered steering actuator (0 = direct mechanical
    // linkage, the wheel follows the command at once).
    Real steerRate = 0;               // rad/s
    Real castorAngle = 0;             // rad, free castoring range beyond pedal steering
    Real corneringStiffness = 4e4;    // N/rad
    Real relaxationLength = 0.3;      // m
    Real longitudinalStiffness = 2e5; // N/m (bristle model for braking/stiction)
};

// Hard points that should never touch the ground (tail, wingtips, prop tip,
// belly): stiff contact with high friction and an event when touched.
struct ContactPointDef {
    std::string name;
    Vec3 position;
    Real stiffness = 2e5;
    Real damping = 1e4;
    Real friction = 0.5;
};

// ---- Flight controls -------------------------------------------------------

struct FlightControlsDef {
    // Flap lever detents (rad of flap deflection); the flap motor moves the
    // surface toward the selected detent at `flapRate` if powered.
    std::vector<Real> flapDetents{0.0};
    Real flapRate = 0.06;             // rad/s
    bool flapsElectric = true;
    Real flapMotorCurrent = 10;       // A
    Real elevatorTrimRange = 0.2;     // rad of equivalent elevator offset at full trim
    Real elevatorTrimRate = 0.1;      // trim fraction per second (manual wheel)
    // Fixed rigging (ground-adjustable tabs, aileron rigging), as pilot-sense
    // control offsets -1..1. Light aircraft are rigged to fly hands-off near
    // cruise power despite engine torque and slipstream; Calibrated.
    Real riggingAileron = 0;
    Real riggingRudder = 0;
    // Rudder travel limiter (transport/business jets): full travel up to this
    // EAS, then travel scaled by (V/EAS)^2 so the yaw authority stays roughly
    // constant with dynamic pressure (floor 15 %). 0 = none.
    Real rudderLimiterIas = 0;        // m/s
    GearRetractionDef gear;
    SpeedbrakeDef speedbrake;
};

// ---- Sensors & instruments --------------------------------------------------

struct PitotStaticDef {
    Vec3 pitotPosition;
    Vec3 staticPosition;
    // Static-source pressure error coefficient dp/q vs local angle of attack
    // (rad) for flaps-up; calibrated against the published airspeed
    // calibration table.
    Table1D staticErrorVsAlpha;
    Real vsiTimeConstant = 3.0;       // s
};

struct StallWarningDef {
    bool present = true;
    int surface = 0;                  // lifting surface carrying the vane/reed
    int strip = -1;                   // strip index (-1 = middle)
    Real alphaMargin = 0.07;          // rad below the strip's stall angle
    // Stick pusher (T-tail jets: keeps the airplane out of a locked-in deep
    // stall): airborne, at this margin below the vane strip's stall angle the
    // elevator is pushed to at most `pusherElevator` until the angle of attack
    // is 5 deg lower again. 0 = no pusher.
    Real pusherMargin = 0;            // rad
    Real pusherElevator = -0.5;       // elevator command (-1..1, nose down)
};

struct VacuumDef {
    bool present = true;
    Real regulatedSuction = 16900;    // Pa (5.0 inHg)
    Real rpmForRegulated = 1500;
    Real gyroSpinUpTime = 20;         // s, rotor spin-up time constant
    Real gyroSpinDownTime = 180;      // s, rotor spin-down time constant (bearing friction)
};

// ---- Structural limits (M1: exceedance reporting only) --------------------

struct StructuralLimitsDef {
    Real neverExceedIas = 0;          // m/s (VNE, indicated)
    Real maxStructuralCruiseIas = 0;  // m/s (VNO)
    Real maneuveringIas = 0;          // m/s (VA at max gross)
    Real flapExtendedIas = 0;         // m/s (VFE, full flap)
    Table1D flapExtendedIasVsFlap;    // optional VFE (m/s) vs flap angle (rad), e.g. 110 kt at 10 deg
    Real limitLoadPositive = 3.8;     // g, flaps up
    Real limitLoadNegative = -1.52;
    Real limitLoadFlaps = 3.0;
    Real maxOperatingMach = 0;        // MMO (0 = none), exceedance reporting
};

// ---- Loading stations --------------------------------------------------

struct PayloadStationDef {
    std::string name;
    Vec3 position;
    Real maxMass = 0;
    Real initial = 0;
};

struct AircraftDefinition {
    int schemaVersion = 1;
    std::string id;                   // stable simulation id
    std::string displayName;          // fictional public name
    std::string manufacturer;         // fictional
    std::string referenceAircraft;    // engineering reference type and variant
    std::string description;
    std::vector<SourceRef> sources;
    std::vector<ProvenanceNote> provenance;

    // Mass & balance
    Real emptyMass = 0;
    Vec3 emptyCg;
    Vec3 emptyInertiaDiag;            // Ixx, Iyy, Izz about the empty CG
    Real emptyIxz = 0;
    Real maxTakeoffMass = 0;
    Real maxLandingMass = 0;
    std::vector<PayloadStationDef> stations;
    // CG envelope: forward/aft limit (structural x) vs mass.
    Table1D cgForwardLimitVsMass, cgAftLimitVsMass;

    // Geometry & aero
    Real referenceArea = 0;           // m^2
    Real referenceSpan = 0;           // m
    Real referenceChord = 0;          // m (MAC)
    std::vector<LiftingSurfaceDef> surfaces;
    std::vector<FuselageDef> fuselages;
    std::vector<DragItemDef> dragItems;

    std::vector<PistonEngineDef> engines;
    std::vector<TurbofanDef> turbofans;
    FuelSystemDef fuel;
    ElectricalDef electrical;
    std::vector<GearDef> gear;
    std::vector<ContactPointDef> contacts;
    FlightControlsDef controls;
    PitotStaticDef pitotStatic;
    StallWarningDef stallWarning;
    VacuumDef vacuum;
    StructuralLimitsDef limits;
    InstrumentPanelDef panel;

    // Returns human-readable errors (empty = valid). Every message starts with
    // the dotted field path, e.g. "engines[0].propeller.diameter: must be > 0".
    std::vector<std::string> validate() const;
};

const char* toString(ControlChannel c);
const char* toString(Provenance p);

}  // namespace avd
