// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
//
// References:
//  - U.S. Standard Atmosphere, 1976 (NOAA/NASA/USAF), layers 0-84.852 km.
//  - Buck, A. L. (1981, updated 1996): saturation vapour pressure over water.
//  - Moist-air density: rho = (p - e)/(Rd T) + e/(Rv T) (ideal-gas mixture).
//  - Sutherland's law with USSA-1976 constants (beta = 1.458e-6, S = 110.4 K).
//  - Wind and turbulence: see wind.cpp.
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

namespace {
BoundaryLayerParams boundaryLayerParams(const SimpleWeatherParams& p) {
    BoundaryLayerParams b;
    b.windSpeed = p.windSpeed;
    b.windFromDirection = p.windFromDirection;
    b.referenceHeight = p.referenceHeight;
    b.profile = p.profile;
    b.surfaceRoughness = p.surfaceRoughness;
    b.powerLawExponent = p.powerLawExponent;
    b.top = p.boundaryLayerTop;
    b.veer = p.veer;
    b.aloft = p.windsAloft;
    return b;
}

TurbulenceParams turbulenceParams(const SimpleWeatherParams& p) {
    TurbulenceParams t = p.turbulence;
    if (p.thermals.convectiveVelocity > 0 && p.thermals.convectiveTurbulence) {
        t.convectiveVelocity = p.thermals.convectiveVelocity;
        t.convectiveDepth = p.thermals.depth;
    }
    return t;
}
}  // namespace

// Representative surface u* (enables the mechanical turbulence): the
// strongest surface wind any source can produce over the reference surface.
Real representativeFrictionVelocity(const SimpleWeatherParams& p, const BoundaryLayer& bl) {
    Real v = p.stations.empty() ? p.windSpeed : 0.0;
    for (const WindStation& s : p.stations)
        for (Real sp : s.speeds) v = std::max(v, sp);
    for (const WindShift& w : p.windShifts) v = std::max(v, w.newSpeed);
    Real zMax = p.surfaceRoughness;
    for (const RoughnessPatch& r : p.roughnessPatches) zMax = std::max(zMax, r.roughness);
    return bl.frictionVelocity({v, 0, zMax});
}

SimpleWeather::SimpleWeather(const Params& p)
    : p_(p),
      spatial_(!p.roughnessPatches.empty() || !p.stations.empty() || !p.windShifts.empty() || !p.obstacles.empty()),
      atm_(p.conditions),
      boundaryLayer_(boundaryLayerParams(p)),
      turbulence_(turbulenceParams(p), boundaryLayer_.surfaceFlowHeading(),
                  representativeFrictionVelocity(p, BoundaryLayer(boundaryLayerParams(p)))),
      // Thermals drift with the mean wind in the middle of the convective layer.
      thermals_(p.thermals, boundaryLayer_.wind(0.5 * p.thermals.depth, p.groundElevation + 0.5 * p.thermals.depth)) {}

Vec3 SimpleWeather::meanWindAt(Real agl) const {
    return boundaryLayer_.wind(agl, p_.groundElevation + agl) + Vec3{0, 0, -p_.updraft};
}

Vec3 SimpleWeather::meanWindAt(const Vec3& positionNed, Real altitudeMsl, Real time) const {
    return localWind(positionNed, altitudeMsl - p_.groundElevation, altitudeMsl, time).wind;
}

Real SimpleWeather::roughnessAt(Real north, Real east) const {
    Real lz = std::log(std::max(p_.surfaceRoughness, 1e-5));
    for (const RoughnessPatch& r : p_.roughnessPatches) {
        const Real d = std::hypot(north - r.north, east - r.east);
        const Real half = 0.5 * std::max(r.transition, 1e-3);
        const Real w = 1 - smoothstep(r.radius - half, r.radius + half, d);
        lz = lerp(lz, std::log(std::max(r.roughness, 1e-5)), w);
    }
    return std::exp(lz);
}

SurfaceWind SimpleWeather::surfaceWindAt(Real north, Real east, Real time) const {
    SurfaceWind sw{p_.windSpeed, p_.windFromDirection, roughnessAt(north, east)};
    if (p_.stations.empty()) return sw;
    // Average the reports as vectors, weighted 1 / (d^2 + L^2).
    Real sn = 0, se = 0, sumW = 0;
    const Real l2 = sq(std::max(p_.stationInfluence, 1.0));
    for (const WindStation& st : p_.stations) {
        Real sp, from;
        st.at(time, sp, from);
        const Real w = 1 / (sq(north - st.north) + sq(east - st.east) + l2);
        sn += w * sp * std::cos(from);
        se += w * sp * std::sin(from);
        sumW += w;
    }
    sw.speed = std::hypot(sn, se) / sumW;
    sw.fromDirection = sw.speed > 1e-9 ? std::atan2(se, sn) : p_.windFromDirection;
    return sw;
}

SimpleWeather::LocalWind SimpleWeather::localWind(const Vec3& pos, Real agl, Real msl, Real time) const {
    LocalWind lw;
    const SurfaceWind sw = surfaceWindAt(pos.x, pos.y, time);
    lw.wind = boundaryLayer_.wind(agl, msl, sw);
    Real uStar = boundaryLayer_.frictionVelocity(sw);
    for (const WindShift& f : p_.windShifts) {
        const Real b = f.behind(pos, agl, time);
        const SurfaceWind nw{f.newSpeed, f.newFromDirection, sw.roughness};
        if (b > 0) lw.wind = lw.wind + (boundaryLayer_.wind(agl, msl, nw) - lw.wind) * b;
        uStar = lerp(uStar, boundaryLayer_.frictionVelocity(nw), f.behind(pos, 0, time));
        lw.wind.z -= f.updraft(pos, agl, time);
    }
    Real extra2 = 0;
    for (const Obstacle& o : p_.obstacles) {
        const Real roof = o.baseElevation + o.height;
        Real extra;
        lw.wind += o.velocity(pos, msl, boundaryLayer_.wind(roof - p_.groundElevation, roof, sw), extra);
        extra2 += extra * extra;
    }
    lw.wind.z -= p_.updraft;
    lw.turbulence = {uStar, std::sqrt(extra2)};
    return lw;
}

AirSample SimpleWeather::sample(const Vec3& positionNed, Real altitudeMsl, Real time, Real extent) const {
    AirSample s;
    s.state = atm_.at(altitudeMsl);
    if (p_.cloudTop > p_.cloudBase && altitudeMsl >= p_.cloudBase && altitudeMsl <= p_.cloudTop)
        s.liquidWaterContent = p_.cloudLiquidWater;
    const Real agl = altitudeMsl - p_.groundElevation;
    // Frozen turbulence is advected with the reference-height mean wind.
    const Vec3 adv = boundaryLayer_.wind(p_.referenceHeight, p_.groundElevation + p_.referenceHeight);
    if (!spatial_) {
        s.wind = boundaryLayer_.wind(agl, altitudeMsl) + Vec3{0, 0, -p_.updraft};
        // Vertical shear: d(wind)/d(down) = -d(wind)/d(height).
        const Vec3 dvdh = boundaryLayer_.shear(agl, altitudeMsl);
        s.windGradient.m[0][2] = -dvdh.x;
        s.windGradient.m[1][2] = -dvdh.y;
        if (turbulence_.active()) {
            Vec3 u;
            Mat3 g;
            turbulence_.evaluate(positionNed - adv * time, agl, altitudeMsl, u, g, extent);
            s.wind += u;
            s.windGradient = s.windGradient + g;
        }
    } else {
        // The mean wind and the turbulence statistics vary in space: both
        // gradients from the same seven local evaluations (point, +-step on
        // each axis; down = minus height).
        const Real d = 0.02;
        const Vec3 off[7] = {{0, 0, 0}, {d, 0, 0}, {-d, 0, 0}, {0, d, 0}, {0, -d, 0}, {0, 0, d}, {0, 0, -d}};
        LocalWind lw[7];
        TurbulenceLocal tl[7];
        for (int k = 0; k < 7; ++k) {
            lw[k] = localWind(positionNed + off[k], agl - off[k].z, altitudeMsl - off[k].z, time);
            tl[k] = lw[k].turbulence;
        }
        s.wind = lw[0].wind;
        for (int j = 0; j < 3; ++j) {
            const Vec3 dv = (lw[2 * j + 1].wind - lw[2 * j + 2].wind) / (2 * d);
            s.windGradient.m[0][j] = dv.x;
            s.windGradient.m[1][j] = dv.y;
            s.windGradient.m[2][j] = dv.z;
        }
        if (turbulence_.active()) {
            Vec3 u;
            Mat3 g;
            turbulence_.evaluate(positionNed - adv * time, agl, altitudeMsl, tl, d, u, g, extent);
            s.wind += u;
            s.windGradient = s.windGradient + g;
        }
    }
    // Discrete events, thermals and ridges: analytic fields, gradient by
    // central differences.
    if (!p_.gusts.empty() || !p_.microbursts.empty() || thermals_.active() || !p_.ridges.empty()) {
        auto events = [&](const Vec3& x) {
            const Real dz = x.z - positionNed.z;
            const Real h = agl - dz;
            Vec3 v;
            for (const DiscreteGust& g : p_.gusts) v += g.velocity(x, time, adv);
            for (const Microburst& m : p_.microbursts) v += m.velocity(x, h, time);
            v += thermals_.velocity(x, h, time);
            for (const Ridge& r : p_.ridges) {
                const Real crest = r.baseElevation + r.height;
                v += r.velocity(x, altitudeMsl - dz, boundaryLayer_.wind(crest - p_.groundElevation, crest));
            }
            return v;
        };
        Vec3 v;
        Mat3 g;
        sampleWithGradient(events, positionNed, v, g);
        s.wind += v;
        s.windGradient = s.windGradient + g;
    }
    return s;
}

}  // namespace avd
