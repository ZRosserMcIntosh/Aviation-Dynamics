// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
//
// References:
//  - Neutral surface layer: U(z) = (u*/kappa) ln(z/z0); roughness lengths
//    from Wieringa (1992) / WMO-No. 8 (Davenport classification).
//  - Surface-layer turbulence in neutral conditions: sigma_u : sigma_v :
//    sigma_w = 2.4 : 1.9 : 1.25 u* (Panofsky & Dutton 1984). MIL-HDBK-1797's
//    low-altitude sigma_w = 0.1 W20 is the same relation over open terrain.
//  - MIL-F-8785C / MIL-HDBK-1797 turbulence: low altitude (< 1000 ft AGL)
//    L_w = h, L_u = L_v = h / (0.177 + 0.000823 h)^1.2,
//    sigma_u / sigma_w = sigma_v / sigma_w = 1 / (0.177 + 0.000823 h)^0.4
//    (h in ft); medium/high altitude (> 2000 ft) isotropic, L = 2500 ft
//    (von Kármán), sigma from the probability-of-exceedance curves; linear
//    interpolation in between.
//  - Convective boundary layer: Lenschow, Wyngaard & Pennell (1980),
//    sigma_w^2 / w*^2 = 1.8 (z/zi)^(2/3) (1 - 0.8 z/zi)^2; horizontal
//    sigma ~ 0.6 w* (Caughey & Palmer 1979).
//  - von Kármán spectrum: E(k) ∝ (k/k_e)^4 / (1 + (k/k_e)^2)^(17/6),
//    k_e = 1 / (1.339 L).
#include "avd/wind.hpp"

#include <algorithm>

#include "avd/table.hpp"

namespace avd {

namespace {

constexpr Real kFt = 0.3048;
constexpr Real kKt = 1852.0 / 3600.0;

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

Vec3 flowVector(Real speed, Real fromDirection) {
    const Real to = fromDirection + kPi;
    return {speed * std::cos(to), speed * std::sin(to), 0};
}

// MIL-F-8785C medium/high-altitude RMS (ft/s) by altitude (ft) for the
// probability-of-exceedance curves 1e-2 (light), 1e-3 (moderate) and
// 1e-5 (severe).
const Table1D& highAltitudeSigma(TurbulenceSeverity s) {
    static const std::vector<Real> alt{500, 1750, 3750, 7500, 15000, 25000, 35000, 45000, 55000, 65000, 75000, 80000};
    static const Table1D light(alt, {6.6, 6.9, 7.4, 6.7, 4.6, 2.7, 0.4, 0, 0, 0, 0, 0});
    static const Table1D moderate(alt, {8.6, 9.6, 10.6, 10.1, 8.0, 6.6, 5.0, 4.2, 2.7, 0, 0, 0});
    static const Table1D severe(alt, {15.6, 17.6, 23.0, 23.6, 22.1, 20.0, 16.0, 15.1, 12.1, 7.9, 6.2, 5.1});
    return s == TurbulenceSeverity::Light ? light : s == TurbulenceSeverity::Moderate ? moderate : severe;
}

}  // namespace

// ================================================================ mean wind ==

BoundaryLayer::BoundaryLayer(const BoundaryLayerParams& p) : p_(p), aloft_(p.aloft) {
    p_.surfaceRoughness = std::max(p_.surfaceRoughness, 1e-5);
    p_.referenceHeight = std::max(p_.referenceHeight, 0.1);
    p_.top = std::max(p_.top, p_.referenceHeight);
    std::sort(aloft_.begin(), aloft_.end(),
              [](const WindLevel& a, const WindLevel& b) { return a.altitudeMsl < b.altitudeMsl; });
}

Real BoundaryLayer::speedFactor(Real h) const {
    h = std::min(h, p_.top);
    switch (p_.profile) {
        case WindProfile::Uniform:
            return 1.0;
        case WindProfile::PowerLaw:
            return std::pow(std::max(h, 0.5) / p_.referenceHeight, p_.powerLawExponent);
        case WindProfile::LogLaw: {
            // Below a couple of roughness lengths the log law has no meaning
            // (the flow is among the roughness elements): hold it there.
            const Real z0 = p_.surfaceRoughness;
            const Real zMin = std::max(0.5, 2 * z0);
            const Real zRef = std::max(p_.referenceHeight, 1.01 * zMin);
            return std::log(std::max(h, zMin) / z0) / std::log(zRef / z0);
        }
    }
    return 1.0;
}

Vec3 BoundaryLayer::wind(Real h, Real altitudeMsl) const {
    // Ekman turning: most of it in the lower part of the layer, none above.
    auto direction = [&](Real hh) {
        const Real zeta = clamp(hh / p_.top, 0.0, 1.0);
        return p_.windFromDirection + p_.veer * (1 - sq(1 - zeta));
    };
    if (h <= p_.top || aloft_.empty()) return flowVector(p_.windSpeed * speedFactor(h), direction(h));
    // Above the boundary layer: from its top wind to the reported levels,
    // interpolating speed and direction (shortest turn) by altitude.
    const Real topMsl = altitudeMsl - h + p_.top;
    WindLevel lo{topMsl, p_.windSpeed * speedFactor(p_.top), direction(p_.top)};
    for (const WindLevel& hi : aloft_) {
        if (hi.altitudeMsl <= topMsl) continue;
        if (altitudeMsl <= hi.altitudeMsl) {
            const Real t = (altitudeMsl - lo.altitudeMsl) / std::max(hi.altitudeMsl - lo.altitudeMsl, 1e-6);
            const Real dir = lo.fromDirection + wrapPi(hi.fromDirection - lo.fromDirection) * t;
            return flowVector(lerp(lo.speed, hi.speed, t), dir);
        }
        lo = hi;
    }
    return flowVector(lo.speed, lo.fromDirection);
}

Vec3 BoundaryLayer::shear(Real h, Real altitudeMsl) const {
    const Real d = 0.01;
    const Real lo = std::max(h - d, 0.0);
    const Real hi = lo + 2 * d;
    return (wind(hi, altitudeMsl + (hi - h)) - wind(lo, altitudeMsl + (lo - h))) / (hi - lo);
}

Real BoundaryLayer::frictionVelocity() const {
    const Real z0 = p_.surfaceRoughness;
    return kKarman * p_.windSpeed / std::log(std::max(p_.referenceHeight, 2 * z0) / z0);
}

// =============================================================== turbulence ==

TurbulenceField::TurbulenceField(const Params& p) : p_(p) {
    if (p.intensity <= 0 || p.modes <= 0 || p.scaleLength <= 0) return;
    SplitMix rng{p.seed};
    const Real ke = 1.0 / (1.339 * p.scaleLength);
    const Real kMin = 0.05 * ke;
    const Real kMax = std::max(kTwoPi / std::max(p.minWavelength, 0.25), 4 * kMin);
    const int n = p.modes;
    const Real ratio = std::pow(kMax / kMin, 1.0 / n);
    Real energySum = 0;
    modes_.reserve(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        const Real k0 = kMin * std::pow(ratio, i), k1 = k0 * ratio;
        const Real k = std::sqrt(k0 * k1);
        const Real kl = k / ke;
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

AtmosphericTurbulence::AtmosphericTurbulence(const TurbulenceParams& p, Real flowHeading, Real frictionVelocity)
    : p_(p), frictionVelocity_(p.mechanical ? std::max(frictionVelocity, 0.0) : 0.0) {
    const Real c = std::cos(flowHeading), s = std::sin(flowHeading);
    axes_ = Mat3::zero();
    axes_.m[0][0] = c;  axes_.m[1][0] = s;   // u: along the surface flow
    axes_.m[0][1] = -s; axes_.m[1][1] = c;   // v: to its right
    axes_.m[2][2] = 1;                       // w: down
    active_ = p.severity != TurbulenceSeverity::None || p.intensity > 0 || frictionVelocity_ > 0 ||
              p.convectiveVelocity > 0;
    if (!active_) return;
    bank_.reserve(kScales);
    for (int i = 0; i < kScales; ++i) {
        TurbulenceField::Params fp;
        fp.intensity = 1;
        fp.scaleLength = kScale0 * std::pow(2.0, i);
        fp.modes = p.modesPerScale;
        fp.minWavelength = p.minWavelength;
        fp.seed = p.seed * 0x9E3779B97F4A7C15ull + static_cast<std::uint64_t>(i) * 0xD1B54A32D192ED03ull + 1;
        bank_.emplace_back(fp);
    }
}

TurbulenceStatistics AtmosphericTurbulence::statistics(Real agl, Real altitudeMsl) const {
    TurbulenceStatistics st;
    if (!active_) return st;
    const Real h = std::max(agl / kFt, 10.0);  // ft; MIL-HDBK-1797 lower bound
    const Real w20 = p_.severity == TurbulenceSeverity::None ? 0.0 : 15.0 * static_cast<int>(p_.severity) * kKt;
    // Low-altitude model (vertical RMS sets the level; the horizontal
    // components grow toward the ground and their scales shrink).
    const Real sigmaWLow = std::sqrt(sq(1.25 * frictionVelocity_) + sq(0.1 * w20) + sq(p_.intensity));
    auto low = [&](Real hf, Vec3& sigma, Vec3& length) {
        const Real a = 0.177 + 0.000823 * hf;
        const Real ratio = 1 / std::pow(a, 0.4);
        sigma = {ratio * sigmaWLow, ratio * sigmaWLow, sigmaWLow};
        const Real lu = hf / std::pow(a, 1.2) * kFt;
        length = {lu, lu, hf * kFt};
    };
    Real sigmaHigh = p_.intensity;
    if (p_.severity != TurbulenceSeverity::None)
        sigmaHigh = std::sqrt(sq(highAltitudeSigma(p_.severity)(altitudeMsl / kFt) * kFt) + sq(p_.intensity));
    const Real lHigh = 2500 * kFt;
    if (h <= 1000) {
        low(h, st.sigma, st.length);
    } else if (h >= 2000) {
        st.sigma = {sigmaHigh, sigmaHigh, sigmaHigh};
        st.length = {lHigh, lHigh, lHigh};
    } else {
        Vec3 s1, l1;
        low(1000, s1, l1);
        const Real t = (h - 1000) / 1000;
        st.sigma = s1 + (Vec3{sigmaHigh, sigmaHigh, sigmaHigh} - s1) * t;
        st.length = l1 + (Vec3{lHigh, lHigh, lHigh} - l1) * t;
    }
    // Convective boundary layer, root-sum-square.
    if (p_.convectiveVelocity > 0 && agl > 0 && agl < p_.convectiveDepth) {
        const Real ws = p_.convectiveVelocity, zeta = agl / p_.convectiveDepth;
        const Real sw2 = 1.8 * ws * ws * std::pow(zeta, 2.0 / 3.0) * sq(1 - 0.8 * zeta);
        const Real sh2 = sq(0.6 * ws);
        st.sigma = {std::sqrt(sq(st.sigma.x) + sh2), std::sqrt(sq(st.sigma.y) + sh2), std::sqrt(sq(st.sigma.z) + sw2)};
    }
    // The vertical component vanishes at the surface.
    st.sigma.z *= smoothstep(0.0, 3.0, agl);
    return st;
}

void AtmosphericTurbulence::coefficients(Real agl, Real altitudeMsl, std::array<std::array<Real, kScales>, 3>& c) const {
    const TurbulenceStatistics st = statistics(agl, altitudeMsl);
    const Real sig[3] = {st.sigma.x, st.sigma.y, st.sigma.z}, len[3] = {st.length.x, st.length.y, st.length.z};
    for (int i = 0; i < 3; ++i) {
        c[i].fill(0);
        const Real t = clamp(std::log2(std::max(len[i], 1e-6) / kScale0), 0.0, kScales - 1.0);
        const int i0 = std::min(static_cast<int>(t), kScales - 2);
        const Real f = t - i0;
        // Two independent unit fields blended with cos/sin weights keep the
        // variance exactly sigma^2 between scales.
        c[i][i0] = sig[i] * std::cos(0.5 * kPi * f);
        c[i][i0 + 1] = sig[i] * std::sin(0.5 * kPi * f);
    }
}

void AtmosphericTurbulence::evaluate(const Vec3& x, Real agl, Real altitudeMsl, Vec3& velocity, Mat3& gradient) const {
    velocity = {};
    gradient = Mat3::zero();
    if (!active_) return;
    // Height derivative of the coefficients (intensity and scale change
    // with height): adds the d/dz column of the gradient.
    const Real d = 0.25;
    std::array<std::array<Real, kScales>, 3> c, cp, cm;
    coefficients(agl, altitudeMsl, c);
    coefficients(agl + d, altitudeMsl + d, cp);
    coefficients(agl - d, altitudeMsl - d, cm);
    const Mat3 at = axes_.transposed();
    Vec3 vt;              // velocity in turbulence axes
    Mat3 gt = Mat3::zero();  // rows: turbulence axes, columns: NED derivatives
    Real vtv[3] = {0, 0, 0};
    for (int k = 0; k < kScales; ++k) {
        bool used = false;
        for (int i = 0; i < 3; ++i) used = used || c[i][k] != 0 || cp[i][k] != 0 || cm[i][k] != 0;
        if (!used) continue;
        Vec3 f;
        Mat3 g;
        bank_[static_cast<std::size_t>(k)].evaluate(x, f, g);
        const Vec3 a = at * f;
        const Mat3 ga = at * g;
        const Real av[3] = {a.x, a.y, a.z};
        for (int i = 0; i < 3; ++i) {
            vtv[i] += c[i][k] * av[i];
            for (int j = 0; j < 3; ++j) gt.m[i][j] += c[i][k] * ga.m[i][j];
            // NED z is down: d/dz = -d/dh.
            gt.m[i][2] -= (cp[i][k] - cm[i][k]) / (2 * d) * av[i];
        }
    }
    vt = {vtv[0], vtv[1], vtv[2]};
    velocity = axes_ * vt;
    gradient = axes_ * gt;
}

}  // namespace avd
