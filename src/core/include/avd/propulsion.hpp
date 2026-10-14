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
    Real radius() const { return 0.5 * def_.diameter; }
    Real discArea() const { return kPi * radius() * radius(); }
    const PropellerDef& def() const { return def_; }
    // Normalised coefficients vs advance angle (rad) and the Mach number of
    // W: T = cT 0.5 rho W^2 A, Q = cQ 0.5 rho W^2 A R, W^2 = V^2 + (0.7 Omega R)^2.
    const Table2D& thrustTable() const { return cT_; }
    const Table2D& torqueTable() const { return cQ_; }

    // One BEM solution (exposed for tests). Returns thrust and torque.
    // speedOfSound <= 0: incompressible.
    static void bem(const PropellerDef& def, Real v, Real omega, Real rho, Real& thrust, Real& torque,
                    Real speedOfSound = 0);

private:
    PropellerDef def_;
    Table2D cT_, cQ_;
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
};

struct PistonEngineState {
    Real omega = 0;              // rad/s (crankshaft = propeller, direct drive)
    bool firing = false;
    bool running = false;        // firing and above idle-ish speed
    Real manifoldPressure = 0;   // Pa
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

private:
    Real frictionTorque(Real omega) const;
    PistonEngineDef def_;
    Real etaIndicated_ = 0.3;
};

}  // namespace avd
