// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
//
// Icing: ice on the lifting surfaces, the propeller and the windshield from
// supercooled cloud water and freezing rain, its aerodynamic effect, the ice
// protection that removes it, and carburettor ice. See docs/architecture/weather.md.
//
// Accretion. Droplets reach a body by inertia: the fraction of those in its
// path that strike it (the collection efficiency) grows with the modified
// inertia parameter K0 of Langmuir & Blodgett (1946), here through the fit of
// Finstad, Lozowski & Gates (1988) for a cylinder. A leading edge is a
// cylinder of its own diameter (2 x the nose radius, 1.1 (t/c)^2 c for a
// NACA 4-digit section): ~0.65 at the stagnation line of a 0.5 m chord at
// 67 m/s in 20 um drops, as LEWICE gives (Wright 2008, NASA/CR-2008-214255).
// The ice at the stagnation line grows at
//   dk/dt = beta0 LWC V n / rho_ice,
// n the freezing fraction: all of the water freezes on a surface at -6 C
// or colder (rime below about -8 C: milky, rho 880 kg/m^3), part of it
// nearer 0 C where the latent heat cannot all be carried away (glaze, clear,
// horns, 917 kg/m^3), none at or above 0 C. The surface is at the recovery temperature T + 0.85
// V^2 / (2 cp): a jet at 250 kt is 6 K warmer than the air. Smaller chords
// collect better and the same ice is a larger share of their chord: tails
// ice first. Out of the cloud ice melts above 0 C (forced convection at the
// leading edge, Froessling's stagnation-line heat transfer) and sublimates
// slowly below it in unsaturated air (the Lewis analogy).
//
// Effect (Bragg, Broeren & Blumenthal 2005, "Iced-airfoil aerodynamics",
// Prog. Aerosp. Sci. 41; Lynch & Khodadoust 2001, Prog. Aerosp. Sci. 37):
// on the section polar of each strip, by the ice height over the strip's
// chord k/c: clmax (and |clmin|) lose 1 - f = L x/(1 + x), x = sqrt((k/c) /
// 2e-4), L 0.45 for rime and up to 0.6 for glaze horns past 1 % of chord
// (19 % at k/c 1e-4 roughness, 31 % at 1e-3, 39 % for a 1 % rime shape, ~55 %
// for 2 % horns); the lift slope a quarter of that less; the drag cd0 up by
// (0.02 + 0.025 glaze) x'/(1 + x'), x' = sqrt((k/c)/3e-3) (+0.005 for
// roughness, +0.012 for a 1 % rime shape, +0.03 for horns), and its rise
// with lift steeper. With the slope nearly kept, the stall angle falls by about
// (1 - f) clmax / a: 4-6 deg on the C172's wing. A tail's clmin falls the
// same way: tailplane stall with the flaps down.
#pragma once

#include <array>
#include <string>
#include <vector>

#include "avd/aircraft_def.hpp"
#include "avd/atmosphere.hpp"
#include "avd/math.hpp"

namespace avd {

class AeroModel;

inline constexpr int kIcingMaxSurfaces = 8;
inline constexpr int kIcingMaxEngines = 4;
inline constexpr int kIcingMaxSwitches = 4;

// Ice on a lifting surface: its height at the leading edge and the share of
// it that is glaze (clear ice, horns) rather than rime.
struct SurfaceIce {
    Real thickness = 0;  // m
    Real glaze = 0;      // 0 rime .. 1 glaze
};

// What ice of a height k/c does to a section polar (see above).
struct IceSectionEffect {
    Real clMaxFactor = 1;        // on clmax and clmin
    Real liftSlopeFactor = 1;
    Real cdIncrement = 0;        // on cd0
    Real cdQuadraticFactor = 1;  // on the drag rise with lift
};
IceSectionEffect iceSectionEffect(Real heightOverChord, Real glaze);

namespace icing {
inline constexpr Real kLatentFusion = 3.34e5;   // J/kg
inline constexpr Real kLatentSublimation = 2.834e6;
inline constexpr Real kCpAir = 1005;            // J/(kg K)
inline constexpr Real kRimeDensity = 880, kGlazeDensity = 917;  // kg/m^3
// Total collection efficiency of a cylinder of `diameter` (m) at `speed`
// (m/s) for droplets of `droplet` diameter (m) in air of that density and
// viscosity (Langmuir & Blodgett's K0; Finstad et al. 1988, the A term).
Real collectionEfficiency(Real diameter, Real speed, Real droplet, Real airDensity, Real viscosity);
// Leading-edge radius (m) of a NACA 4-digit-like section.
inline Real leadingEdgeRadius(Real chord, Real thicknessRatio) { return 1.1019 * thicknessRatio * thicknessRatio * chord; }
// Surface (recovery) temperature, K.
inline Real recoveryTemperature(Real staticTemperature, Real speed) { return staticTemperature + 0.85 * speed * speed / (2 * kCpAir); }
// Freezing fraction (1 at -6 C and colder, 0 at 0 C) and the glaze share
// (rime at -8 C and colder, glaze from -2 C) at a surface temperature.
inline Real freezingFraction(Real surfaceTemperature) { return clamp((273.15 - surfaceTemperature) / 6.0, 0.0, 1.0); }
inline Real glazeShare(Real surfaceTemperature) { return clamp((surfaceTemperature - 273.15 + 8.0) / 6.0, 0.0, 1.0); }
inline Real iceDensity(Real glaze) { return kRimeDensity + (kGlazeDensity - kRimeDensity) * clamp(glaze, 0.0, 1.0); }
// Stagnation-line heat transfer coefficient (W/m^2/K) of a cylinder,
// Froessling: Nu = 1.14 Re^0.5 Pr^0.4.
Real stagnationHeatTransfer(Real diameter, Real speed, Real airDensity, Real viscosity);
// Saturation vapour pressure over ice (Buck 1981), Pa.
Real saturationVapourPressureIce(Real temperature);

// The growth (>= 0) and the melting or sublimation (>= 0) of the ice at a
// leading edge of that diameter, m/s, and the glaze share of new ice.
struct AccretionRates {
    Real growth = 0, loss = 0;
    Real glaze = 0;
    Real supercooledWater = 0;  // kg/m^3 met that can freeze (cloud + freezing rain)
};
AccretionRates leadingEdgeRates(const AirSample& air, Real speed, Real diameter);

// Carburettor: the throat/plate temperature (K), the ice building (kg/s)
// and melting (kg/s) for the intake air, its flow (kg/s), fuel/air ratio,
// MAP / ambient and the carb heat's temperature rise (K).
struct CarburettorRates {
    Real temperature = 288;
    Real accretion = 0, melt = 0;
};
CarburettorRates carburettorRates(const CarburettorDef& d, const AirSample& air, Real airMassFlow, Real fuelAirRatio,
                                  Real manifoldRatio, Real heatRise);
// Icing intensity class from the accretion rate (m/s) on an unprotected
// wing leading edge: 0 none, 1 trace (< 1/4 in/h), 2 light (to 1 in/h),
// 3 moderate (to 3 in/h), 4 severe (FMH-1 / AIM 7-1-21 rate classes).
int intensityClass(Real accretionRate);
}  // namespace icing

// Everything the cockpit and the outside view may show about ice.
struct IcingStatus {
    int surfaceCount = 0;
    std::array<SurfaceIce, kIcingMaxSurfaces> surfaces{};
    std::array<Real, kIcingMaxSurfaces> clMaxLoss{};       // fraction, at the surface's mean chord
    std::array<Real, kIcingMaxSurfaces> protectionHeat{};  // thermal anti-ice warm-up 0..1 (boots: 1 while inflated)
    std::array<Real, kIcingMaxEngines> propellerIce{};     // m at 0.4 R
    std::array<Real, kIcingMaxEngines> propellerThrustLoss{};
    std::array<Real, kIcingMaxEngines> carburettorIce{};   // kg
    std::array<Real, kIcingMaxEngines> carburettorBlockage{};  // fraction of the throttle area the ice takes
    std::array<Real, kIcingMaxEngines> carburettorTemperature{};  // K at the throat (0 without a carburettor)
    std::array<Real, kIcingMaxEngines> carburettorHeat{};  // carb heat warm-up 0..1
    std::array<Real, kIcingMaxEngines> roughness{};        // 0..1 rough running (meltwater, a blocked carburettor)
    Real windshieldIce = 0;       // m
    Real accretionRate = 0;       // m/s on the unprotected wing leading edge (the intensity)
    int intensity = 0;            // icing::intensityClass of it
    Real supercooledWater = 0;    // kg/m^3 met that freezes (cloud + freezing rain)
    bool iceDetected = false;     // the ICE annunciator (detector fitted)
    bool bootsInflating = false;
};

// The ice state of one airframe (per surface, propeller, carburettor,
// windshield), advanced once per step by the aircraft.
class IcingModel {
public:
    void configure(const AircraftDefinition& def, const AeroModel& aero);
    void reset();

    struct EngineInputs {
        Real omega = 0;            // rad/s propeller
        Real axialSpeed = 0;       // m/s through the disc
        Real throttle = 0;         // lever 0..1
        Real manifoldRatio = 1;    // MAP / ambient
        Real airMassFlow = 0;      // kg/s
        Real fuelFlow = 0;         // kg/s
        Real powerFraction = 0;    // of rated
        bool carbHeat = false;
    };
    struct Inputs {
        AirSample air;
        Real trueAirspeed = 0;
        std::array<bool, kIcingMaxSwitches> switches{};  // per switchName(i)
        Real bleed = 0;            // 0..1 bleed heat the cores supply
        std::array<EngineInputs, kIcingMaxEngines> engines{};
    };
    void update(Real dt, const Inputs& in);

    int switchCount() const { return static_cast<int>(switches_.size()); }
    const std::string& switchName(int i) const { return switches_[static_cast<std::size_t>(i)]; }
    // Thermal protection of switch i draws engine bleed.
    bool switchIsThermal(int i) const { return switchThermal_[static_cast<std::size_t>(i)]; }

    // Per surface (AeroInputs::ice), and whether any surface carries ice.
    const SurfaceIce* surfaces() const { return status_.surfaces.data(); }
    bool iced() const { return iced_; }
    Real propellerThrustFactor(int e) const { return propThrust_[static_cast<std::size_t>(e)]; }
    Real propellerTorqueFactor(int e) const { return propTorque_[static_cast<std::size_t>(e)]; }
    Real carburettorAreaFactor(int e) const { return 1 - status_.carburettorBlockage[static_cast<std::size_t>(e)]; }
    Real inductionTemperatureRise(int e) const { return heatRise_[static_cast<std::size_t>(e)]; }
    Real enginePowerFactor(int e) const { return powerFactor_[static_cast<std::size_t>(e)]; }
    const IcingStatus& status() const { return status_; }

    // Tests and scenarios: ice put on directly.
    void setSurfaceIce(int surface, Real thickness, Real glaze = 0);
    void setCarburettorIce(int engine, Real kg);

private:
    void refreshEffects();
    struct SurfaceGeometry {
        Real chord = 1, diameter = 0.05;  // mean chord, leading-edge diameter
        int protection = -1;              // switch index, -1 none
        bool boots = false;
    };
    struct PropGeometry {
        bool present = false;
        Real radius = 1, chord = 0.1, diameter = 0.01, ratedOmega = 280;
    };
    std::vector<SurfaceGeometry> geo_;
    std::array<PropGeometry, kIcingMaxEngines> prop_{};
    std::array<CarburettorDef, kIcingMaxEngines> carb_{};
    std::array<Real, kIcingMaxEngines> ratedPower_{};
    std::vector<std::string> switches_;
    std::vector<bool> switchThermal_;
    IceProtectionDef protection_;
    bool iced_ = false;
    std::array<Real, kIcingMaxEngines> propThrust_{1, 1, 1, 1}, propTorque_{1, 1, 1, 1};
    std::array<Real, kIcingMaxEngines> heatRise_{}, powerFactor_{1, 1, 1, 1};
    std::array<Real, kIcingMaxSwitches> bootTimer_{};
    Real detectorTimer_ = 0;
    IcingStatus status_;
};

}  // namespace avd
