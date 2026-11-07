// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
//
// Piston engine + fixed-pitch propeller.
//
// The propeller's thrust and torque come from a blade-element-momentum (BEM)
// analysis of its geometry, run once at load time over the full range of
// advance angles (static thrust, climb, cruise, windmilling and a stopped
// propeller). At run time the tables are indexed by the advance angle
// beta = atan2(V, 0.7 Omega R), which stays well defined at Omega = 0.
//
// The piston engine computes air mass flow from manifold pressure, rpm and
// intake density; indicated power from fuel/air equivalence ratio; friction
// and pumping from an fmep model; and fuel flow from the metered mixture. Its
// indicated efficiency is calibrated once so that the definition's rated
// power is produced at sea level, rated rpm, full throttle, best-power
// mixture. Power lapse with altitude, rich/lean behaviour and fuel flow follow
// from the model rather than from separate curves.
// See docs/architecture/propulsion.md.
#pragma once

#include "avd/aircraft_def.hpp"
#include "avd/atmosphere.hpp"
#include "avd/table.hpp"

namespace avd {

struct PropellerForces {
    Real thrust = 0;    // N along the thrust axis
    Real torque = 0;    // N m absorbed (positive opposes rotation)
    Real efficiency = 0;
    Real advanceRatio = 0;
};

class Propeller {
public:
    explicit Propeller(const PropellerDef& def);
    // `axialSpeed`: aircraft speed relative to the air along the thrust axis.
    // `speedOfSound` sets the blade Mach numbers (compressibility).
    PropellerForces evaluate(Real axialSpeed, Real omega, Real density, Real speedOfSound = 340.294) const;
    // Variable-pitch propeller at blade angle `bladeAngle` (rad at 0.75 R):
    // the tables at the neighbouring blade angles, interpolated linearly
    // (PropellerDef::variablePitch; a fixed-pitch propeller ignores the angle).
    PropellerForces evaluate(Real axialSpeed, Real omega, Real density, Real speedOfSound, Real bladeAngle) const;
    // Blade angles (rad at 0.75 R) the variable-pitch tables are solved at.
    const std::vector<Real>& bladeAngles() const { return angles_; }
    Real radius() const { return 0.5 * def_.diameter; }
    Real discArea() const { return kPi * radius() * radius(); }
    const PropellerDef& def() const { return def_; }
    // Normalised coefficients vs advance angle (rad) and the Mach number of
    // W: T = cT 0.5 rho W^2 A, Q = cQ 0.5 rho W^2 A R, W^2 = V^2 + (0.7 Omega R)^2.
    const Table2D& thrustTable() const { return cT_; }
    const Table2D& torqueTable() const { return cQ_; }

    // One BEM solution (exposed for tests). Returns thrust and torque.
    // speedOfSound <= 0: incompressible.
    // `pitchOffset`: blade rotation in the hub (rad, added to the twist at
    // every station; 0 for a fixed-pitch propeller).
    static void bem(const PropellerDef& def, Real v, Real omega, Real rho, Real& thrust, Real& torque,
                    Real speedOfSound = 0, Real pitchOffset = 0);

private:
    void buildVariablePitch();
    PropellerDef def_;
    Table2D cT_, cQ_;
    // Variable pitch: one table pair per blade angle.
    std::vector<Real> angles_;
    std::vector<Table2D> vcT_, vcQ_;
};

enum class Magnetos { Off = 0, Right = 1, Left = 2, Both = 3, Start = 4 };

struct PistonEngineInputs {
    Real throttle = 0;           // 0..1
    Real mixture = 1;            // 0 (idle cut-off) .. 1 (full rich)
    Magnetos magnetos = Magnetos::Off;
    bool starterPowered = false; // key at START and starter has bus voltage
    bool fuelAtEngine = false;   // fuel system delivers fuel to the injectors
    Real ambientPressure = isa::kSeaLevelPressure;
    Real ambientTemperature = isa::kSeaLevelTemperature;
    Real ambientDensity = isa::kSeaLevelDensity;
    // Induction icing and carb heat (avd/icing.hpp; defaults: none): the
    // fraction of the throttle's flow area the ice leaves, the intake air's
    // temperature rise through the carb heat muff (less dense, the mixture
    // richer), and the power left with meltwater going through the cylinders.
    Real throttleAreaFactor = 1;
    Real inductionTemperatureRise = 0;  // K
    Real powerFactor = 1;
};

struct PistonEngineState {
    Real omega = 0;              // rad/s (crankshaft = propeller, direct drive)
    bool firing = false;
    bool running = false;        // firing and above idle-ish speed
    Real manifoldPressure = 0;   // Pa
    Real airMassFlow = 0;        // kg/s inducted
    Real equivalenceRatio = 0;
    Real fuelFlow = 0;           // kg/s
    Real brakePower = 0;         // W
    Real brakeTorque = 0;        // N m on the shaft (net of friction), excl. starter
    Real starterTorque = 0;
    Real egt = 300, cht = 300, oilTemperature = 300;  // K (lumped, Estimated)
    Real oilPressure = 0;        // Pa
};

class PistonEngine {
public:
    explicit PistonEngine(const PistonEngineDef& def);
    const PistonEngineDef& def() const { return def_; }

    // Compute torque, fuel flow and combustion state at shaft speed `omega`.
    // Pure function of inputs (used inside the integrator stages).
    PistonEngineState evaluate(const PistonEngineInputs& in, Real omega) const;
    // First-order thermal states (called once per step).
    void updateThermal(PistonEngineState& s, Real dt, Real coolingAirspeed) const;

    Real totalInertia() const { return def_.engineInertia + def_.propeller.inertia; }
    Real indicatedEfficiency() const { return etaIndicated_; }

    // Induction model (exposed for tests). Manifold pressure (Pa) at a
    // throttle lever and shaft speed: the balance of the throttle plate's
    // compressible orifice flow (choked below 0.528 x ambient) against the
    // engine's pumping. Closed, the idle orifice is choked at any speed above
    // a crawl, so the air flow is fixed and MAP falls as 1/rpm (less the
    // residual-gas floor at ambient / compression ratio).
    // `areaFactor` scales the throttle's flow area (carburettor ice).
    Real manifoldPressure(Real throttle, Real omega, Real ambientPressure, Real ambientTemperature, Real areaFactor = 1) const;
    // Effective throttle flow area Cd A (m^2) at a lever position.
    Real throttleArea(Real throttle) const;
    // Volumetric efficiency referred to manifold density: the exhaust
    // residual left at ambient pressure re-expands and displaces fresh charge,
    // eta_v = eta_v,wot (r_c - p_amb / MAP) / (r_c - p_amb / MAP_wot) (Heywood
    // 6.2), times the port-flow loss with speed (1 - loss (omega/omega_R)^2),
    // both normalised to the definition's value at full throttle, rated rpm.
    Real volumetricEfficiency(Real manifoldPressure, Real ambientPressure, Real omega) const;
    // Temperature (K) at which the charge fills the cylinder: ambient plus
    // the induction-tract heating (Heywood 6.2.3), which grows as the ambient
    // falls below the tract's wall temperature; see PistonEngineDef::chargeHeating.
    Real chargeTemperature(Real ambientTemperature) const;
    // Mechanical friction torque (N m), rubbing and accessories; pumping
    // (ambient minus manifold pressure over the displacement) is added in evaluate().
    Real frictionTorque(Real omega) const;

private:
    PistonEngineDef def_;
    Real etaIndicated_ = 0.3;
    Real throttleAreaFull_ = 1e-3, throttleAreaIdle_ = 1e-4;  // Cd A, m^2
    Real volumetricNorm_ = 1;                                 // (r_c - 1/(1 - intake loss)) / (r_c - 1)
};

// ---- Turbofan (see TurbofanDef) --------------------------------------------

struct TurbofanInputs {
    Real throttle = 0;            // 0 idle .. 1 take-off
    ThrustRating rating = ThrustRating::Takeoff;  // active FADEC rating: caps the commanded N1
    bool fuelOn = false;          // fuel control switch at RUN
    bool fuelAtEngine = false;    // the fuel system delivers fuel
    bool starterPowered = false;  // bus power for the starter and igniters
    Real ambientPressure = isa::kSeaLevelPressure;
    Real ambientTemperature = isa::kSeaLevelTemperature;
    Real mach = 0;
};

struct TurbofanState {
    Real n1 = 0, n2 = 0;          // fan and core speed, fraction of take-off
    bool running = false;         // lit and at or above idle
    bool starting = false;        // start sequence in progress
    Real startTimer = 0;          // s since the start sequence began
    bool startAborted = false;    // no light-off: the starter waits for the fuel switch to be cycled
    Real thrust = 0;              // N, net, along the thrust axis (negative: windmilling drag, unlit)
    Real thrustFraction = 0;      // of the available take-off thrust
    Real ratingThrust = 0;        // N available under the active rating at this flight condition
    Real fuelFlow = 0;            // kg/s
    Real itt = 288;               // K, inter-turbine temperature
    Real oilPressure = 0;         // Pa
};

class TurbofanEngine {
public:
    explicit TurbofanEngine(const TurbofanDef& def);
    const TurbofanDef& def() const { return def_; }
    // Fan speed the lever alone asks for (idle to take-off N1, linear).
    Real commandedN1(Real throttle) const { return def_.idleN1 + (1 - def_.idleN1) * clamp(throttle, 0.0, 1.0); }
    // Fan speed the fuel control schedules: the lever's N1 capped at the
    // active rating's limit for the flight condition.
    Real commandedN1(const TurbofanInputs& in) const;
    // Take-off thrust available at the flight condition (Mattingly lapse,
    // flat-rated to the throttle ratio).
    Real availableThrust(Real pressure, Real temperature, Real mach) const;
    // Thrust the rating allows at the flight condition (<= availableThrust):
    // a fixed fraction of the temperature-limited thrust, which is the
    // Mattingly temperature branch continued below the throttle ratio.
    Real ratingThrust(ThrustRating rating, Real pressure, Real temperature, Real mach) const;
    // The N1 limit of a rating at the flight condition (1 for take-off).
    Real ratingN1(ThrustRating rating, Real pressure, Real temperature, Real mach) const;
    // Inverse of thrustFraction above idle.
    Real n1ForThrustFraction(Real fraction) const;
    // Fan diameter (m) and the drag area (m^2 of CdA) of the unlit, windmilling
    // engine: thrust = -q CdA (TurbofanDef::windmillDragArea or its default).
    Real fanDiameter() const;
    Real windmillDragArea() const;
    // Thrust fraction of the available thrust at fan speed n1 (lit engine).
    Real thrustFraction(Real n1) const;
    // Thrust, fuel flow, ITT and oil pressure for the spool speeds in `s`
    // at the flight condition (pure; spool speeds unchanged).
    void evaluate(TurbofanState& s, const TurbofanInputs& in) const;
    // Spool dynamics and the start / shut-down sequence over dt.
    void step(TurbofanState& s, const TurbofanInputs& in, Real dt) const;
    // Lit and stabilised at the commanded fan speed (trim, spawns).
    void setSteady(TurbofanState& s, const TurbofanInputs& in) const;
    Real n2FromN1(Real n1) const;
    Real n1FromN2(Real n2) const;

private:
    TurbofanDef def_;
    Real tauUp_ = 1.0;            // s, acceleration time-constant scale (calibrated)
};

// ---- Turboprop (see TurbopropDef) ------------------------------------------

// Condition lever: fuel cut-off, low idle (ground) or high idle (flight, and
// a faster-accelerating gas generator).
enum class ConditionLever { CutOff = 0, LowIdle = 1, HighIdle = 2 };

struct TurbopropInputs {
    Real powerLever = 0;          // 0 idle .. 1 maximum (the forward stop: the torque / ITT limit)
    Real reverse = 0;             // 0 .. 1 below idle: beta, then reverse (ground only; the caller gates it)
    ConditionLever condition = ConditionLever::CutOff;
    bool fuelAtEngine = false;
    bool starter = false;         // starter engaged and powered
    Real npSelected = 1;          // the propeller lever's governed speed, fraction of rated Np
    Real ambientPressure = isa::kSeaLevelPressure;
    Real ambientTemperature = isa::kSeaLevelTemperature;
    Real mach = 0;
};

struct TurbopropState {
    Real ng = 0;                  // gas generator speed, fraction of 100 %
    Real np = 0;                  // propeller speed, fraction of rated (mirrors the integrated shaft speed)
    bool lit = false;             // burning
    bool running = false;         // lit and at or above idle
    bool starting = false;        // lit below idle, accelerating
    Real gasPower = 0;            // W the gas generator delivers to the power turbine at rated Np
    Real torque = 0;              // N m, power-turbine output at the propeller shaft (- dragging)
    Real torqueFraction = 0;      // of the rated torque (the torque gauge, 1 = 100 %)
    Real shaftPower = 0;          // W
    Real itt = 288;               // K
    Real fuelFlow = 0;            // kg/s
    Real oilPressure = 0;         // Pa
    Real oilTemperature = 288;    // K
    Real ngCommand = 0;           // the fuel control's Ng target
    // Propeller.
    Real bladeAngle = 0;          // rad at 0.75 R
    bool feathered = false;       // at (or within 1 deg of) the feather stop
    bool beta = false;            // blade angle set by the power lever (ground beta / reverse)
    Real npSet = 0;               // selected Np, fraction of rated (the governor's set point)
    Real thrust = 0;              // N (from the last recorded evaluation)
    Real propTorque = 0;          // N m absorbed
};

class TurbopropEngine {
public:
    explicit TurbopropEngine(const TurbopropDef& def);
    const TurbopropDef& def() const { return def_; }
    // Compressor-face total pressure and temperature ratios (ram rise).
    static void inletRatios(Real pressure, Real temperature, Real mach, Real& delta0, Real& theta0);
    // Gas power (W at rated Np) at a gas generator speed and flight condition.
    Real gasPower(Real ng, Real delta0, Real theta0) const;
    // ITT (K) of a lit engine at Ng.
    Real itt(Real ng, Real theta0) const;
    // Inverses for the fuel control's limiters: Ng that gives `power` (W of
    // gas power) and Ng at which the ITT reaches `ittK`.
    Real ngForGasPower(Real power, Real delta0, Real theta0) const;
    Real ngForItt(Real ittK, Real theta0) const;
    // Power-turbine torque (N m) at the propeller shaft for the gas power and
    // shaft speed (rad/s), friction included.
    Real turbineTorque(Real gasPower, Real omega) const;
    // Shaft power available at the flight condition with the propeller at its
    // rated speed: min(torque limit, ITT limit, Ng limit).
    Real availablePower(Real pressure, Real temperature, Real mach) const;
    // Fuel flow (kg/s) of the lit engine at Ng.
    Real fuelFlow(Real ng, Real delta0, Real theta0) const;
    // The fuel control's Ng target for the inputs at shaft speed omega.
    Real commandedNg(const TurbopropInputs& in, Real omega) const;
    Real idleNg(ConditionLever c) const { return c == ConditionLever::HighIdle ? def_.ngHighIdle : def_.ngLowIdle; }
    // Gas generator: start, light-off, spool dynamics, shut-down over dt
    // (the shaft speed omega, rad/s, from the integrator).
    void step(TurbopropState& s, const TurbopropInputs& in, Real omega, Real dt) const;
    // Gas power, torque, ITT, fuel flow and oil at the state's Ng (pure).
    void evaluate(TurbopropState& s, const TurbopropInputs& in, Real omega) const;
    // Lit and stabilised at the commanded Ng (trim, spawns).
    void setSteady(TurbopropState& s, const TurbopropInputs& in, Real omega) const;
    // Propeller governor and pitch-change mechanism over dt: feather (selected,
    // or no governor oil), the power lever's blade angle in beta, else the
    // governor holding the selected Np between the flight-idle stop and
    // feather. omega, omegaDot: shaft speed (rad/s) and its rate.
    void stepPropeller(TurbopropState& s, bool feather, bool betaActive, Real betaAngle, Real omega, Real omegaDot,
                       Real dt) const;
    // Blade angle the power lever sets in beta / reverse (reverse 0..1).
    Real betaBladeAngle(Real reverse) const;

private:
    TurbopropDef def_;
    Real willansA_ = 0, willansB_ = 0;  // corrected fuel flow = A + B Pr (corrected gas power / Pr)^n
    Real willansPr_ = 1;                // Pr: gas power at the flat rating, sea level static ISA (W)
};

}  // namespace avd
