// Aviation Dynamics — scenarios.
// SPDX-License-Identifier: MIT
//
// Reproducible flight-test scenarios shared by the test suite and avdemo:
// trim, POH-comparable performance measurements, and flight procedures.
// Every scenario uses the same Aircraft and the same physics the game uses;
// only the pilot is scripted.
#pragma once

#include <string>
#include <vector>

#include "avd/aircraft.hpp"
#include "avd/aircraft_loader.hpp"
#include "avd/atmosphere.hpp"
#include "avd/ground.hpp"
#include "test_pilot.hpp"

namespace avd::scenarios {

inline constexpr Real kDefaultDt = 1.0 / 240.0;

// Load a definition from data/aircraft/<id>.json (throws on errors).
AircraftDefinition loadReference(const std::string& id = "tupi_t180");

// Mixture lever for best-power equivalence ratio at density rho.
Real bestPowerMixture(const AircraftDefinition& def, Real rho);

struct TrimRequest {
    Real tas = 50;             // m/s (initial guess when solveTas)
    Real altitude = 0;         // m MSL
    int flapDetent = 0;
    Real throttle = -1;        // fixed if >= 0, otherwise solved
    Real flightPath = 0;       // rad, fixed unless solveFlightPath
    bool solveFlightPath = false;
    bool solveTas = false;
    Real mixture = -1;         // < 0: best power
};

struct TrimResult {
    bool converged = false;
    Real theta = 0, alpha = 0, elevator = 0, throttle = 0, rpm = 0, tas = 0, flightPath = 0;
    Real brakePower = 0, thrust = 0, propEfficiency = 0, fuelFlow = 0;
    Real residual = 0;
    DynamicState state;
    PilotInput input;
};

// Free-air longitudinal trim (u_dot = w_dot = q_dot = shaft_dot = 0), wings
// level, out of ground effect. Leaves the aircraft in the trimmed state with
// lag states settled.
TrimResult trim(Aircraft& ac, const TrimRequest& req, const AtmosphereField& atm);

// ---- POH-comparable performance measurements ----
struct StallResult {
    Real minCas = 0, minIas = 0;   // m/s
    Real warningIas = 0;           // IAS when the stall warning first sounded
    Real alphaMax = 0;             // rad
    bool stalled = false;
    Real altitudeLost = 0;         // m, from stall break to recovery start (+ is loss)
};
// 1 kt/s deceleration at idle, wings level (FAR 23.49 style), flaps at detent.
StallResult stallTest(const AircraftDefinition& def, int flapDetent, Real dt = kDefaultDt);

struct TakeoffResult {
    Real groundRoll = 0;           // m, brake release to lift-off
    Real distanceTo50ft = 0;       // m
    Real liftoffIas = 0;
    bool success = false;
};
// POH short-field technique: flaps 10, full throttle against brakes, release,
// rotate at `rotateKias`, climb at 56 KIAS. `elevation` m MSL, ISA + dT.
TakeoffResult takeoffTest(const AircraftDefinition& def, Real elevation, Real temperatureDeviation,
                          Real relativeHumidity = 0, Real headwind = 0, Real rotateKias = 49, Real dt = kDefaultDt);

// How the pilot brakes after touchdown. The default is the tyre-limited
// physics check (full pedal from the instant of touchdown); `pilot()` is a
// human technique without anti-skid (estimated values, see
// docs/validation/c172s-validation.md).
struct BrakingTechnique {
    Real delay = 0;      // s from touchdown to the first pedal pressure (nose lowering, reaction)
    Real rampTime = 0;   // s to build pedal pressure to `effort`
    Real effort = 1.0;   // sustained pedal, 0..1 (a pilot without anti-skid keeps margin)
    static BrakingTechnique pilot() { return {1.0, 1.0, 0.7}; }
};

// Ground roll from touchdown (`touchdownKias`, flaps 30, idle) to stop with
// full nose-up elevator and the given braking technique.
Real landingRollTest(const AircraftDefinition& def, Real touchdownKias = 47, Real dt = kDefaultDt,
                     BrakingTechnique technique = {});

// ---- Flight helpers ----
// An airborne aircraft trimmed at `kias`, `altitude` with the engine running.
struct FlightSetup {
    Real kias = 100;
    Real altitude = 1500;
    int flapDetent = 0;
};
void startTrimmedFlight(Aircraft& ac, const FlightSetup& s, const AtmosphereField& atm, const GroundQuery& ground,
                        TestPilot* pilot = nullptr, TrimResult* result = nullptr);

// Convert IAS (~CAS) to TAS at density rho (incompressible; < 1% error below 150 kt).
inline Real tasFromCas(Real cas, Real rho) { return cas * std::sqrt(isa::kSeaLevelDensity / rho); }
// Compressible CAS -> TAS (what the airspeed indicator is calibrated for): the
// impact pressure of CAS at sea level, then the Mach number giving that impact
// pressure at this static pressure.
inline Real tasFromCas(Real cas, const AirState& air) {
    constexpr Real a0 = 340.294;
    const Real qc = isa::kSeaLevelPressure * (std::pow(1 + 0.2 * (cas / a0) * (cas / a0), 3.5) - 1);
    const Real mach = std::sqrt(5 * (std::pow(qc / air.pressure + 1, 2.0 / 7.0) - 1));
    return mach * air.speedOfSound;
}

}  // namespace avd::scenarios
