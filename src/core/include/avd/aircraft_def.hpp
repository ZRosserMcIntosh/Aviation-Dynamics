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

// Spoiler: a panel on the upper surface raised by the speedbrake (lift dump);
// max_positive is its full deflection, chord_fraction its chord.
enum class ControlChannel { Aileron, Elevator, Rudder, Flap, Spoiler, None };

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
    // Fowler flaps move aft as they deflect: chord added at full positive
    // deflection, as a fraction of the chord (0 for plain and split flaps).
    Real chordExtension = 0;
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
    // Transonic section behaviour, on the Mach number normal to the span
    // (simple sweep theory, as the Korn drag rise). The defaults are an
    // ordinary subsonic section's; below M ~0.4 they change nothing.
    //  - clmax x max(0.35, 1 - k (Mn - knee)^2) above the knee: shock-induced
    //    separation, the buffet boundary at altitude (Mn = M cos(sweep)).
    //  - The section's force point moves aft from c/4 toward c/2 between
    //    about Mdd (Korn, with the lift coefficient) and M 1: Mach tuck.
    //  - Hinged controls lose effectiveness by (1 - (M/Mdd0)^n), floored, Mdd0
    //    the zero-lift Korn Mdd: the shock ahead of the hinge line.
    Real machClMaxKnee = 0.4;         // normal Mach
    Real machClMaxFactor = 2.0;       // k, per (normal Mach)^2: 0.92 at Mn 0.6, 0.68 at 0.8, 0.5 at 0.9
    Real machAcShift = 0.25;          // chord fraction at M 1 (c/4 -> c/2)
    //  - Optional cm0 shift through the drag rise (Mcrit = Mdd - 0.1077 to
    //    Mdd + 0.05): a rear-loaded supercritical section's aft loading grows
    //    with Mach (more nose-down); 0 for conventional sections.
    Real machCm0Shift = 0;            // added to cm0 past the drag rise (negative: nose down)
    Real machControlExponent = 10;    // n
    Real machControlFloor = 0.2;      // effectiveness kept at and above Mdd
    // A horizontal tail with a downwash source sits in that wing's separated
    // wake when the wing is stalled: a band about the wake centreline (from
    // the mid-chord, deflected by alpha less the downwash) of half-thickness
    // c/2 sin(alpha - eps) plus the spreading aft, thickness and strength
    // both scaled by the wing's separated fraction; inside it the tail sees reduced dynamic pressure
    // and a wake-fixed downwash (the T-tail deep stall). The spreading is a
    // turbulent wake's (~9 deg half-angle); the core's q ratio the ~55 %
    // velocity deficit a few chords behind a stalled wing (Taylor & Ray, AIAA
    // 65-737: the tail's contribution nearly lost in the wake core). Off:
    // the downwash model alone.
    bool wakeBlanking = true;
    Real wakeSpread = 0.16;           // half-thickness per metre aft
    Real wakeDynamicPressureRatio = 0.2;  // q ratio at the band's centre
    // Span loading from a nonlinear lifting line (horseshoe vortices solved
    // together, ground images) instead of independent strips with the
    // Helmbold slope: tip stall, swept-wing pitch-up, winglets, roll damping.
    bool liftingLine = false;
    bool trimmableStabilizer = false;  // its incidence follows the stabiliser trim
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
    // Optional shape, for the airflow view's bluff-body wakes (no effect on
    // the forces): long axis (structural), length and thickness across the
    // flow, m. A wing strut, a gear leg and its wheel fairing.
    Vec3 bodyAxis{0, 0, 1};
    Real bodyLength = 0, bodySize = 0;
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
    Real compressionRatio = 8.5;      // geometric; sets the residual-gas loss of volumetric efficiency at low MAP
    Real volumetricEfficiency = 0.85; // referred to manifold density, at full throttle and rated rpm
    // Inlet-port and valve flow losses grow with the square of the flow, so
    // the trapped charge falls with speed: eta_v x (1 - loss (rpm/rated)^2),
    // normalised to 1 at rated rpm (Heywood 6.2.4). Sets how much the
    // full-throttle torque rises below rated rpm (Lycoming's sea-level curve:
    // about +9 % from 2,700 to 1,800 rpm).
    Real volumetricEfficiencySpeedLoss = 0.08;
    Real frictionMepAtRated = 1.6e5;  // Pa, mechanical friction (rubbing, accessories) mean effective pressure
    Real frictionMepAtZero = 0.6e5;   // Pa (pumping, exhaust minus manifold pressure, is separate)
    // Induction: a butterfly throttle plate (effective area 1 - cos of the
    // plate angle, lever-linear) in series with the engine's pumping. The
    // closed plate leaves an idle orifice sized so that MAP = idleManifoldRatio
    // x ambient at idleRpm (the idle stop a mechanic sets); the wide-open
    // plate is sized so that MAP = (1 - intakeLossRatio) x ambient at rated
    // rpm (about 1 inHg). Between, MAP follows from rpm and lever: closed
    // throttle is choked, so the air flow is fixed and MAP falls as 1/rpm
    // when the propeller windmills the engine.
    Real idleManifoldRatio = 0.33;    // MAP / ambient at closed throttle at idleRpm
    Real idleRpm = 650;               // rpm the idle orifice is sized at
    Real intakeLossRatio = 0.03;      // MAP loss at full throttle, rated rpm
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
//
// FADEC thrust ratings (TurbofanDef::*Fraction): the fuel control caps the
// commanded N1 at the active rating's limit; the lever still commands N1
// below it. Take-off (TO/GA) is time-limited after lift-off, then maximum
// continuous (one engine out) or maximum climb apply, and maximum cruise in
// level flight (Aircraft schedules this in ThrustMode::Auto).
enum class ThrustRating { Takeoff = 0, MaxContinuous = 1, MaxClimb = 2, MaxCruise = 3 };

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
    // Thrust ratings as fractions of the take-off thrust at sea level, static,
    // ISA (Estimated: typical of modern business-jet turbofans, maximum
    // continuous 0.90-0.93, maximum climb 0.85-0.90, maximum cruise
    // 0.80-0.87). The ratings are turbine-temperature limits: the flat-rated
    // take-off thrust is held up to the throttle ratio by raising T_t4, and
    // each lower rating holds a fixed fraction of the temperature-limited
    // thrust (Mattingly's temperature branch, slope 3/(1.5 + M) per unit
    // theta0, on both sides of TR). They therefore give these fractions at
    // sea level and converge on the flat-rated line in the cold air at
    // altitude, where the take-off rating itself sits well below its
    // temperature limit (TurbofanEngine::ratingThrust).
    Real maxContinuousFraction = 0.92;
    Real maxClimbFraction = 0.88;
    Real maxCruiseFraction = 0.84;
    Real takeoffRatingTime = 300;           // s of take-off thrust after lift-off (all engines)
    Real takeoffRatingTimeEngineOut = 600;  // s with an engine out
    // Fan diameter, m (0: twice the reverser's cowl radius). Sets the default
    // windmilling drag area.
    Real fanDiameter = 0;
    // Unlit engine: the windmilling fan and core are a drag along the thrust
    // line, D = q CdA. Default (< 0): 0.12 x the fan disc area (Estimated,
    // Torenbeek App. G: windmilling drag ~0.1 q x inlet area for a high-bypass
    // turbofan); the engine-out yaw comes from this plus the live engine.
    Real windmillDragArea = -1;       // m^2
    // Thrust reverser (optional): fan-flow reverser (cascade type), deployed
    // and stowed automatically on the landing roll.
    struct Reverser {
        bool present = false;
        Real effectiveness = 0.4;     // reverse force as a fraction of the forward thrust at the same N1
        Real transitTime = 2.0;       // s, stowed to deployed (and back)
        Real reverseN1 = 0.8;         // fan speed commanded in full reverse (fraction of take-off)
        Real stowIas = 30.87;         // m/s (60 KIAS): reverse idle below this, then stow
        Real cascadeForward = 1.4;    // m forward of the nozzle exit: where the cascades are
        Real cowlRadius = 0.7;        // m, nacelle radius at the cascades
    } reverser;
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
    // (dragArea and liftLossFraction are the lumped model for aircraft
    // without spoiler panels; with spoiler controls on the wing the panels
    // carry the drag and the lift loss where they are.)
    Real rate = 0.5;                  // deployment fraction per second
    // Ground spoilers: every panel to full on main-wheel contact with the
    // throttles at idle above armSpeed (also a rejected take-off); stowed
    // when the throttles come up or below taxi speed.
    bool groundSpoilers = false;
    Real groundArmSpeed = 15.4;       // m/s ground speed (30 kt)
    Real groundStowSpeed = 7.7;       // m/s (15 kt)
    Real groundRate = 1.5;            // deployment fraction per second
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
    // On the essential bus: stays powered through a load shed (automatic on
    // battery power with `autoLoadShed`, or the pilot's "electrical.load_shed").
    bool essential = false;
};

// An engine-driven generator (alternator, starter-generator) on the main bus.
// Declared generators share the bus in parallel (their ratings add); each
// comes on line when its own engine is fast enough and has its own switch
// (cockpit control "electrical.generator<N>") and failure. Without a list the
// definition has one alternator rated `alternatorMaxCurrent`, driven by the
// first piston engine or by the fastest turbofan core.
struct GeneratorDef {
    std::string name;                 // e.g. "GEN 1"
    int engine = 0;                   // engine (piston or turbofan index) that drives it
    Real maxCurrent = 300;            // A at the regulated voltage
    // Turbofan starter-generators: core speed (fraction of take-off N2) from
    // which the generator holds the bus; a windmilling core (~20 %) does not.
    Real cutInN2 = 0.45;
};

struct ElectricalDef {
    Real batteryVoltage = 24.0;       // nominal (12 cells); the loaded plateau is ~1.017 x this
    Real batteryCapacity = 13.0 * 3600;  // A s
    Real batteryResistance = 0.05;    // ohm
    Real alternatorVoltage = 28.0;    // regulated bus voltage with a generator on line
    Real alternatorMaxCurrent = 60;   // A, the single alternator when `generators` is empty
    std::vector<GeneratorDef> generators;
    // Bus voltage below which bus-powered devices (gauges, annunciators,
    // actuators, pumps) stop: < 0 means 0.75 x batteryVoltage (18 V on a
    // 24 V system, 9 V on a 12 V one).
    Real busMinimumVoltage = -1;
    std::vector<ElectricalLoadDef> loads;
    // Bus controller: with no generator on line and the battery carrying the
    // bus for `loadShedDelay`, the non-essential loads are shed (they draw
    // nothing and read unpowered) until a generator comes back on line. A
    // light aircraft (false) sheds by hand: cockpit control "electrical.load_shed".
    bool autoLoadShed = false;
    Real loadShedDelay = 2.0;         // s on the battery alone before the automatic shed
    Real busMinimum() const { return busMinimumVoltage >= 0 ? busMinimumVoltage : 0.75 * batteryVoltage; }
};

// ---- Hydraulics (optional; absent = every actuator moves unconditionally) ----
//
// Each system is pressurised by engine-driven pumps (full pressure from
// about 40 % of idle core speed up; a windmilling core gives none) and/or an
// electric pump on the main bus. Pressure follows its source through a
// first-order lag. An actuator draws on one or more systems (the best of
// them): below 20 % pressure it does not move, from there its rate grows
// linearly to the full rate at full pressure. Brakes keep an accumulator good
// for a number of applications once every supplying system is down.
struct HydraulicElectricPumpDef {
    Real current = 0;                 // A drawn from the bus while running (0 = no electric pump)
    // The pump contactor is held by generator power (the battery cannot carry
    // a 3,000 psi pump for long): off with every generator off line.
    bool generatorOnly = true;
};

struct HydraulicSystemDef {
    std::string name = "A";
    std::vector<int> engines;         // engine-driven pumps: engine indices (piston or turbofan)
    HydraulicElectricPumpDef electricPump;
    Real nominalPressure = 2.068e7;   // Pa (3,000 psi), for display
    Real riseTime = 2.0;              // s, pressure time constant toward its source
};

struct HydraulicsDef {
    bool present = false;
    std::vector<HydraulicSystemDef> systems;
    // Supplying systems (indices) of each actuator; an empty list means that
    // actuator is not hydraulic (moves unconditionally, as without the block).
    std::vector<int> gear, flaps, speedbrake, steering, brakes;
    int brakeAccumulatorApplications = 6;   // full brake applications with every brake system down
    // Emergency (free-fall) extension: the gear lowers at this fraction of
    // its normal rate with the uplocks released, whatever the pressure.
    Real freeFallRateFraction = 0.35;
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
    Real maxBrakeForceCoefficient = 0;  // brake force / static load; sets brakeTorque when that is 0 (0 = no brake)
    int brakeGroup = -1;              // 0 = left, 1 = right
    Real maxSteerAngle = 0;           // rad, by the rudder pedals
    // Tiller (nose-wheel steering handwheel on transport aircraft): wheel angle
    // at full tiller (0 = no tiller). Tiller and pedal commands add, limited
    // to the larger of the two authorities.
    Real tillerSteerAngle = 0;        // rad
    // Slew-rate limit of a powered steering actuator (0 = direct mechanical
    // linkage, the wheel follows the command at once).
    Real steerRate = 0;               // rad/s
    // Castoring (swivelling) wheel: the wheel's heading is its own state,
    // turned by the tyre's side force acting on the mechanical trail of the
    // swivel axis toward the direction it rolls in, and held toward the pedal
    // command by the steering linkage (a spring that yields at a breakout
    // torque: the C172's steering bungees). castorAngle is the free swivel
    // range beyond the pedal steering (hard stops at +-(maxSteer + castor)).
    // castorTrail = 0 means a rigidly steered wheel (hydraulic steering).
    Real castorAngle = 0;             // rad, free castoring range beyond pedal steering
    Real castorTrail = 0;             // m, mechanical trail of the swivel axis (0 = rigid steering)
    Real steerLinkageStiffness = 0;   // N m/rad, pedal linkage spring (0 with a trail = free castor, no linkage)
    Real steerLinkageBreakout = 0;    // N m, torque at which the linkage yields (0 = never: the spring alone)
    // Tyre: cornering stiffness at the leg's static load N_ref, with the usual
    // load sensitivity C_alpha = c1 N (1 - c2 N / N_ref) (Pacejka's pKy form;
    // the stiffness peaks at N_ref / (2 c2)). c1 follows from the value at N_ref.
    Real corneringStiffness = 4e4;    // N/rad at the static load
    Real corneringLoadSensitivity = 0.25;  // c2 (0 = constant stiffness)
    Real staticLoad = 0;              // N, design static load N_ref (0 = from the geometry at MTOW)
    Real relaxationLength = 0.3;      // m
    Real longitudinalStiffness = 2e5; // N/m (bristle model for braking/stiction)
    Real tyrePressure = 0;            // Pa, inflation (wet-runway friction, hydroplaning); 0 = unknown
    // Wheel spin: I_w omega_dot = r F_x - T_brake - T_rolling. The brakes are a
    // torque capacity (full pedal), so a lightly loaded wheel locks; a locked
    // or skidding tyre slides at slidingFrictionRatio x the peak friction.
    Real wheelInertia = 0;            // kg m^2 about the axle (0 = estimated from the static load and radius)
    Real brakeTorque = 0;             // N m at full pedal (0 = maxBrakeForceCoefficient x static load x radius)
    Real slidingFrictionRatio = 0.75; // mu_slide / mu_peak
    // Anti-skid: 0 = none (the wheel can lock); otherwise the fraction of the
    // peak braking force the system holds by modulating the brake torque.
    Real antiSkidEfficiency = 0;
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
    // Trimmable horizontal stabiliser (jets): the trim moves the whole
    // tailplane's incidence instead of offsetting the elevator, rad of travel
    // each way (0: elevator-offset trim). The tailplane surface is marked
    // trimmable_stabilizer.
    Real stabilizerNoseUp = 0, stabilizerNoseDown = 0;
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
    // Yaw damper (swept-wing jets, whose Dutch roll is lightly damped or
    // divergent at low speed without it): rudder = -gain * (yaw rate less the
    // coordinated-turn rate for the bank, through a washout filter), the gain scheduled as
    // (reference EAS / EAS)^2 within 0.25..4, limited to +-authority of full
    // travel, in flight above 60 KIAS. gain 0 = none.
    struct YawDamperDef {
        Real gain = 0;            // rudder fraction per rad/s of yaw rate at the reference EAS
        Real referenceIas = 82;   // m/s
        Real washout = 3;         // s
        Real authority = 0.3;     // fraction of full rudder travel
    } yawDamper;
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
    // An angle-of-attack computer (jets) calibrated to the wing's real stall:
    // it protects the strip closest to its stall anywhere on the surface,
    // not one vane's strip (a swept wing stalls outboard first).
    bool computed = false;
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
    Real maxMass = 0;                 // placard limit (a limit of approval, not a physical stop)
    Real initial = 0;
};

// A placarded limit on the sum of several stations (e.g. two baggage areas).
struct StationGroupLimitDef {
    std::string name;
    std::vector<int> stations;
    Real maxMass = 0;
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
    // True when the published empty mass already includes the unusable fuel
    // (and full oil), as a POH "standard empty weight" does; the tanks carry
    // that fuel, so the airframe item is reduced by it rather than counting it twice.
    bool emptyIncludesUnusableFuel = false;
    Real maxRampMass = 0;             // 0 = not published
    Real maxTakeoffMass = 0;
    Real maxLandingMass = 0;
    Real maxZeroFuelMass = 0;         // 0 = not published / not applicable
    std::vector<PayloadStationDef> stations;
    std::vector<StationGroupLimitDef> stationGroupLimits;
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
    HydraulicsDef hydraulics;
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
