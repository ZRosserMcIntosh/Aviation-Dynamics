// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
//
// Atmosphere: thermodynamic state (U.S. Standard Atmosphere 1976 to 86 km,
// with temperature and sea-level-pressure deviations and humidity) plus a
// world-space, time-dependent air-motion field (wind, gusts, turbulence).
//
// Aircraft never fake turbulence with camera shake: they sample the air
// velocity and its spatial gradient at their position, and the aerodynamic
// model applies the local airflow to each lifting strip. See
// docs/architecture/atmosphere-and-weather.md.
#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include "avd/math.hpp"

namespace avd {

namespace isa {
inline constexpr Real kSeaLevelPressure = 101325.0;     // Pa
inline constexpr Real kSeaLevelTemperature = 288.15;    // K
inline constexpr Real kSeaLevelDensity = 1.225;         // kg/m^3
inline constexpr Real kGasConstantDryAir = 287.05287;   // J/(kg K), USSA-1976 value
inline constexpr Real kGasConstantVapour = 461.495;     // J/(kg K)
inline constexpr Real kGamma = 1.4;
inline constexpr Real kEarthRadius = 6356766.0;         // m, USSA-1976 geopotential radius
inline constexpr Real kMaxGeopotentialAltitude = 84852.0;

Real geopotentialFromGeometric(Real z);
Real geometricFromGeopotential(Real h);
}  // namespace isa

// Deviations from the standard day. All thermodynamic profiles keep the ISA
// lapse-rate structure; a temperature deviation shifts the whole profile and
// the pressure is integrated hydrostatically with the actual temperature.
struct AtmosphereConditions {
    Real temperatureDeviation = 0;                  // K, e.g. ISA+20
    Real seaLevelPressure = isa::kSeaLevelPressure; // Pa (MSL pressure, ~QNH)
    Real relativeHumidity = 0;                      // 0..1 (applies below the tropopause)
};

// Thermodynamic state of the air at a point.
struct AirState {
    Real pressure = isa::kSeaLevelPressure;  // Pa (static)
    Real temperature = isa::kSeaLevelTemperature;  // K (static, dry-bulb)
    Real density = isa::kSeaLevelDensity;    // kg/m^3 (moist air)
    Real vapourPressure = 0;                 // Pa
    Real relativeHumidity = 0;               // 0..1
    Real speedOfSound = 340.294;             // m/s
    Real dynamicViscosity = 1.7894e-5;       // Pa s (Sutherland)
    Real pressureAltitude = 0;               // m, ISA altitude with this static pressure
    Real densityAltitude = 0;                // m, ISA altitude with this density
};

// Complete air sample: thermodynamics plus motion. Velocities are in the local
// NED frame (m/s); `windGradient` is d(wind_i)/d(x_j) (1/s), used to apply
// spatially varying air motion across the airframe.
struct AirSample {
    AirState state;
    Vec3 wind;            // mean + gust + turbulence air velocity
    Mat3 windGradient = Mat3::zero();
    Real precipitationRate = 0;  // mm/h liquid-equivalent (hook; consumers in M5)
    Real liquidWaterContent = 0; // g/m^3 cloud liquid water (icing hook; M5)
};

class StandardAtmosphere {
public:
    explicit StandardAtmosphere(const AtmosphereConditions& c = {});
    const AtmosphereConditions& conditions() const { return c_; }

    // State at geometric altitude above mean sea level (m).
    AirState at(Real geometricAltitude) const;

    // Standard-day helpers.
    static Real isaPressure(Real geometricAltitude);
    static Real isaTemperature(Real geometricAltitude);
    static Real isaDensity(Real geometricAltitude);
    // Inverse of the standard pressure profile (geopotential, troposphere/strat).
    static Real pressureAltitudeFromPressure(Real pressure);
    static Real densityAltitudeFromDensity(Real density);

private:
    AtmosphereConditions c_;
    std::array<Real, 8> basePressure_{};  // at each layer base, with deviations
};

// Saturation vapour pressure over liquid water (Buck 1996), T in K -> Pa.
Real saturationVapourPressure(Real temperature);

// ---------------------------------------------------------------------------
// Air-motion field interface. Implementations must be deterministic and
// thread-safe for concurrent const calls (multiple aircraft, server).
// `positionNed` is in the shared local frame; `altitudeMsl` is supplied so the
// field need not know the frame anchor.
class AtmosphereField {
public:
    virtual ~AtmosphereField() = default;
    virtual AirSample sample(const Vec3& positionNed, Real altitudeMsl, Real time) const = 0;
};

// Frozen, divergence-free turbulence synthesised from Fourier modes with a von
// Kármán energy spectrum (random-Fourier-mode synthesis, e.g. Kraichnan 1970).
// The field lives in world space and is advected by the mean wind, so every
// aircraft sees the same air at the same place and time.
class TurbulenceField {
public:
    struct Params {
        Real intensity = 0;          // RMS of each velocity component, m/s
        Real scaleLength = 533.0;    // von Kármán length scale L, m (1750 ft)
        int modes = 48;
        Real minWavelength = 4.0;    // m; shorter eddies are below airframe scale
        std::uint64_t seed = 1;
    };
    TurbulenceField() = default;
    explicit TurbulenceField(const Params& p);
    const Params& params() const { return p_; }
    bool active() const { return p_.intensity > 0 && !modes_.empty(); }
    // Velocity (NED) and gradient at a position already shifted into the
    // frozen-turbulence frame.
    void evaluate(const Vec3& x, Vec3& velocity, Mat3& gradient) const;

private:
    struct Mode {
        Vec3 k;        // wave vector, rad/m
        Vec3 dir;      // unit velocity direction, perpendicular to k
        Real amplitude;
        Real phase;
    };
    Params p_;
    std::vector<Mode> modes_;
};

// Parameters for SimpleWeather.
struct SimpleWeatherParams {
    AtmosphereConditions conditions;
    Real windSpeed = 0;              // m/s at referenceHeight AGL
    Real windFromDirection = 0;      // rad, true, direction the wind blows FROM
    Real referenceHeight = 10.0;     // m AGL (standard anemometer height)
    Real boundaryLayerExponent = 1.0 / 7.0;  // power law; 0 = uniform
    Real boundaryLayerTop = 600.0;   // m AGL; constant above
    Real groundElevation = 0;        // m MSL of the surface the profile refers to
    Real updraft = 0;                // m/s uniform vertical air velocity (+ up)
    TurbulenceField::Params turbulence;
};

// Standard atmosphere + surface wind with a boundary-layer profile + optional
// world-space turbulence. Deterministic; suitable for offline play and tests.
class SimpleWeather final : public AtmosphereField {
public:
    using Params = SimpleWeatherParams;
    SimpleWeather() : SimpleWeather(Params{}) {}
    explicit SimpleWeather(const Params& p);
    const Params& params() const { return p_; }
    const StandardAtmosphere& atmosphere() const { return atm_; }
    AirSample sample(const Vec3& positionNed, Real altitudeMsl, Real time) const override;
    Vec3 meanWindAt(Real heightAgl) const;

private:
    Params p_;
    StandardAtmosphere atm_;
    TurbulenceField turbulence_;
};

}  // namespace avd
