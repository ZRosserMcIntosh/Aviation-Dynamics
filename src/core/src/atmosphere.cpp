// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
//
// References:
//  - U.S. Standard Atmosphere, 1976 (NOAA/NASA/USAF), layers 0-84.852 km.
//  - Buck, A. L. (1981, updated 1996): saturation vapour pressure over water.
//  - Moist-air density: rho = (p - e)/(Rd T) + e/(Rv T) (ideal-gas mixture).
//  - Sutherland's law with USSA-1976 constants (beta = 1.458e-6, S = 110.4 K).
//  - von Kármán spectrum: E(k) ∝ (kL)^4 / (1 + (kL)^2)^(17/6) (MIL-HDBK-1797).
#include "avd/atmosphere.hpp"

#include <cstdint>

namespace avd {

namespace {

struct Layer {
    Real base;   // geopotential m
    Real lapse;  // K/m
    Real temp;   // K at base (standard day)
};
constexpr std::array<Layer, 8> kLayers{{{0, -0.0065, 288.15},
                                        {11000, 0.0, 216.65},
                                        {20000, 0.001, 216.65},
                                        {32000, 0.0028, 228.65},
                                        {47000, 0.0, 270.65},
                                        {51000, -0.0028, 270.65},
                                        {71000, -0.002, 214.65},
                                        {84852, 0.0, 186.946}}};
constexpr Real kG0 = kStandardGravity;
constexpr Real kR = isa::kGasConstantDryAir;

int layerIndex(Real h) {
    int i = 0;
    while (i + 1 < static_cast<int>(kLayers.size()) && h >= kLayers[i + 1].base) ++i;
    return i;
}

// Pressure at geopotential altitude h within layer i, given the layer-base
// pressure and a temperature deviation.
Real layerPressure(int i, Real pb, Real h, Real dT) {
    const Layer& L = kLayers[i];
    const Real tb = L.temp + dT;
    const Real dh = h - L.base;
    if (L.lapse == 0) return pb * std::exp(-kG0 * dh / (kR * tb));
    return pb * std::pow((tb + L.lapse * dh) / tb, -kG0 / (kR * L.lapse));
}

std::array<Real, 8> basePressures(Real p0, Real dT) {
    std::array<Real, 8> pb{};
    pb[0] = p0;
    for (int i = 1; i < 8; ++i) pb[i] = layerPressure(i - 1, pb[i - 1], kLayers[i].base, dT);
    return pb;
}

const std::array<Real, 8>& standardBasePressures() {
    static const std::array<Real, 8> pb = basePressures(isa::kSeaLevelPressure, 0);
    return pb;
}

// SplitMix64: tiny, portable, deterministic across platforms.
struct SplitMix {
    std::uint64_t s;
    std::uint64_t next() {
        std::uint64_t z = (s += 0x9E3779B97F4A7C15ull);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return z ^ (z >> 31);
    }
    Real uniform() { return static_cast<Real>(next() >> 11) * (1.0 / 9007199254740992.0); }
};

}  // namespace

namespace isa {
Real geopotentialFromGeometric(Real z) { return kEarthRadius * z / (kEarthRadius + z); }
Real geometricFromGeopotential(Real h) { return kEarthRadius * h / (kEarthRadius - h); }
}  // namespace isa

Real saturationVapourPressure(Real temperature) {
    const Real tc = temperature - 273.15;
    return 611.21 * std::exp((18.678 - tc / 234.5) * (tc / (257.14 + tc)));
}

StandardAtmosphere::StandardAtmosphere(const AtmosphereConditions& c)
    : c_(c), basePressure_(basePressures(c.seaLevelPressure, c.temperatureDeviation)) {}

Real StandardAtmosphere::isaTemperature(Real z) {
    const Real h = isa::geopotentialFromGeometric(z);
    const Layer& L = kLayers[layerIndex(h)];
    return L.temp + L.lapse * (h - L.base);
}

Real StandardAtmosphere::isaPressure(Real z) {
    const Real h = isa::geopotentialFromGeometric(z);
    const int i = layerIndex(h);
    return layerPressure(i, standardBasePressures()[i], h, 0);
}

Real StandardAtmosphere::isaDensity(Real z) { return isaPressure(z) / (kR * isaTemperature(z)); }

Real StandardAtmosphere::pressureAltitudeFromPressure(Real p) {
    const auto& pb = standardBasePressures();
    int i = 0;
    while (i + 1 < 8 && p <= pb[i + 1]) ++i;
    const Layer& L = kLayers[i];
    Real h;
    if (L.lapse == 0)
        h = L.base - kR * L.temp / kG0 * std::log(p / pb[i]);
    else
        h = L.base + L.temp / L.lapse * (std::pow(p / pb[i], -kR * L.lapse / kG0) - 1.0);
    return isa::geometricFromGeopotential(h);
}

Real StandardAtmosphere::densityAltitudeFromDensity(Real rho) {
    const auto& pb = standardBasePressures();
    auto baseRho = [&](int i) { return pb[i] / (kR * kLayers[i].temp); };
    int i = 0;
    while (i + 1 < 8 && rho <= baseRho(i + 1)) ++i;
    const Layer& L = kLayers[i];
    const Real rb = baseRho(i);
    Real h;
    if (L.lapse == 0)
        h = L.base - kR * L.temp / kG0 * std::log(rho / rb);
    else
        h = L.base + L.temp / L.lapse * (std::pow(rho / rb, 1.0 / (-kG0 / (kR * L.lapse) - 1.0)) - 1.0);
    return isa::geometricFromGeopotential(h);
}

AirState StandardAtmosphere::at(Real z) const {
    AirState s;
    // The model domain ends at 84.852 km geopotential; above it the top layer
    // is continued isothermally. This is a model-domain extension, not a limit.
    const Real h = isa::geopotentialFromGeometric(z);
    const int i = layerIndex(h);
    const Layer& L = kLayers[i];
    s.temperature = L.temp + L.lapse * (h - L.base) + c_.temperatureDeviation;
    s.pressure = layerPressure(i, basePressure_[i], h, c_.temperatureDeviation);

    // Humidity: specified relative humidity, applied in the troposphere only.
    const Real rh = h < 11000 ? clamp(c_.relativeHumidity, 0.0, 1.0) : 0.0;
    s.relativeHumidity = rh;
    s.vapourPressure = std::min(rh * saturationVapourPressure(s.temperature), 0.5 * s.pressure);
    const Real e = s.vapourPressure;
    s.density = (s.pressure - e) / (kR * s.temperature) + e / (isa::kGasConstantVapour * s.temperature);

    // Speed of sound from the virtual temperature (gamma held at 1.4: the
    // vapour-induced change in gamma is < 0.1% at tropical humidity).
    const Real tv = s.pressure / (s.density * kR);
    s.speedOfSound = std::sqrt(isa::kGamma * kR * tv);
    s.dynamicViscosity = 1.458e-6 * std::pow(s.temperature, 1.5) / (s.temperature + 110.4);
    s.pressureAltitude = pressureAltitudeFromPressure(s.pressure);
    s.densityAltitude = densityAltitudeFromDensity(s.density);
    return s;
}

// ---------------------------------------------------------------------------

TurbulenceField::TurbulenceField(const Params& p) : p_(p) {
    if (p.intensity <= 0 || p.modes <= 0 || p.scaleLength <= 0) return;
    SplitMix rng{p.seed};
    const Real L = p.scaleLength;
    const Real kMin = 0.05 / L;
    const Real kMax = kTwoPi / std::max(p.minWavelength, 0.5);
    const int n = p.modes;
    const Real ratio = std::pow(kMax / kMin, 1.0 / n);
    Real energySum = 0;
    modes_.reserve(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        const Real k0 = kMin * std::pow(ratio, i), k1 = k0 * ratio;
        const Real k = std::sqrt(k0 * k1);
        const Real kl = k * L;
        const Real e = std::pow(kl, 4) / std::pow(1 + kl * kl, 17.0 / 6.0) * (k1 - k0);  // ∝ E(k) dk
        // Random direction for k (uniform on the sphere) and a random unit
        // velocity direction perpendicular to it (divergence-free mode).
        const Real cz = 2 * rng.uniform() - 1, az = kTwoPi * rng.uniform();
        const Real sz = std::sqrt(1 - cz * cz);
        const Vec3 khat{sz * std::cos(az), sz * std::sin(az), cz};
        const Vec3 helper = std::abs(khat.z) < 0.9 ? Vec3{0, 0, 1} : Vec3{1, 0, 0};
        const Vec3 e1 = normalized(cross(khat, helper));
        const Vec3 e2 = cross(khat, e1);
        const Real ang = kTwoPi * rng.uniform();
        modes_.push_back({khat * k, e1 * std::cos(ang) + e2 * std::sin(ang), e, kTwoPi * rng.uniform()});
        energySum += e;
    }
    // Normalise: mean of |u|^2 = sum a^2 / 2 = 3 sigma^2.
    for (Mode& m : modes_) m.amplitude = std::sqrt(m.amplitude / energySum * 6.0) * p.intensity;
}

void TurbulenceField::evaluate(const Vec3& x, Vec3& velocity, Mat3& gradient) const {
    velocity = {};
    gradient = Mat3::zero();
    for (const Mode& m : modes_) {
        const Real arg = dot(m.k, x) + m.phase;
        const Real c = std::cos(arg), s = std::sin(arg);
        velocity += m.dir * (m.amplitude * c);
        const Vec3 d = m.dir * (-m.amplitude * s);
        const Real dv[3] = {d.x, d.y, d.z}, kv[3] = {m.k.x, m.k.y, m.k.z};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) gradient.m[i][j] += dv[i] * kv[j];
    }
}

SimpleWeather::SimpleWeather(const Params& p) : p_(p), atm_(p.conditions), turbulence_(p.turbulence) {}

Vec3 SimpleWeather::meanWindAt(Real agl) const {
    // Wind blows FROM windFromDirection: the air velocity points the other way.
    const Real dir = p_.windFromDirection + kPi;
    Real factor = 1.0;
    if (p_.boundaryLayerExponent > 0) {
        const Real hz = clamp(agl, 0.0, p_.boundaryLayerTop);
        factor = std::pow(std::max(hz, 0.5) / p_.referenceHeight, p_.boundaryLayerExponent);
        if (agl <= 0) factor = std::pow(0.5 / p_.referenceHeight, p_.boundaryLayerExponent);
    }
    const Real v = p_.windSpeed * factor;
    return {v * std::cos(dir), v * std::sin(dir), -p_.updraft};
}

AirSample SimpleWeather::sample(const Vec3& positionNed, Real altitudeMsl, Real time) const {
    AirSample s;
    s.state = atm_.at(altitudeMsl);
    if (p_.cloudTop > p_.cloudBase && altitudeMsl >= p_.cloudBase && altitudeMsl <= p_.cloudTop)
        s.liquidWaterContent = p_.cloudLiquidWater;
    const Real agl = altitudeMsl - p_.groundElevation;
    s.wind = meanWindAt(agl);
    // Vertical shear of the boundary layer: d(wind)/d(down) = -d(wind)/d(agl).
    if (p_.boundaryLayerExponent > 0 && agl > 0.5 && agl < p_.boundaryLayerTop) {
        const Real dvdh = p_.boundaryLayerExponent / agl;
        s.windGradient.m[0][2] = -s.wind.x * dvdh;
        s.windGradient.m[1][2] = -s.wind.y * dvdh;
    }
    if (turbulence_.active()) {
        // Frozen turbulence advected with the reference-height mean wind.
        const Vec3 adv = meanWindAt(p_.referenceHeight);
        Vec3 u;
        Mat3 g;
        turbulence_.evaluate(positionNed - adv * time, u, g);
        // Attenuate the vertical component near the ground (it must vanish at
        // the surface); a smooth ramp over the first 30 m.
        const Real ramp = smoothstep(0.0, 30.0, agl);
        u.z *= ramp;
        for (int j = 0; j < 3; ++j) g.m[2][j] *= ramp;
        s.wind += u;
        s.windGradient = s.windGradient + g;
    }
    return s;
}

}  // namespace avd
