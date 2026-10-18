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
//  - Discrete gusts: FAR 25.341 (1-cosine), MIL-F-8785C (ramp).
//  - Microburst: Oseguera & Bowles, NASA TM-100632 (1988); hazard index
//    F-factor: Bowles (1990), FAA AC 120-41 / TDWR.
//  - Thermals: Allen, NASA TM-2006-214250 (2006), mean updraft and radius.
//  - Mountain waves: Queney (1948); Durran, "Mountain waves and downslope
//    winds" (1990), linear hydrostatic flow over a Witch of Agnesi.
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

// ======================================================== discrete events ==

Vec3 DiscreteGust::velocity(const Vec3& x, Real t, const Vec3& advection) const {
    if (t < startTime || length <= 0) return {};
    const Vec3 n{std::cos(heading), std::sin(heading), 0};
    const Real c = frontSpeed >= 0 ? frontSpeed : dot(advection, n);
    // Distance behind the front (into the gust).
    const Real xi = c * (t - startTime) - dot(x - origin, n);
    if (xi <= 0) return {};
    Real f = 0;
    if (shape == Shape::OneMinusCosine) {
        f = xi < 2 * length ? 0.5 * (1 - std::cos(kPi * xi / length)) : 0.0;
    } else {
        const Real hold = std::max(holdLength, 0.0);
        if (xi < length) f = 0.5 * (1 - std::cos(kPi * xi / length));
        else if (xi < length + hold) f = 1;
        else if (xi < 2 * length + hold) f = 0.5 * (1 + std::cos(kPi * (xi - length - hold) / length));
    }
    return amplitude * f;
}

namespace {
// Peak of (1 - exp(-x^2)) / x, at x = 1.1209.
constexpr Real kRadialPeak = 0.638161;
Real outflowShape(Real z, Real zs, Real eps) { return std::exp(-z / zs) - std::exp(-z / eps); }
}  // namespace

Real Microburst::peakOutflowHeight() const {
    const Real zs = outflowDecayHeight, eps = surfaceLayerDepth;
    return zs * eps * std::log(zs / eps) / (zs - eps);
}

Real Microburst::lambda() const {
    if (radius <= 0 || outflowDecayHeight <= surfaceLayerDepth || surfaceLayerDepth <= 0) return 0;
    const Real g = outflowShape(peakOutflowHeight(), outflowDecayHeight, surfaceLayerDepth);
    return 2 * peakOutflow / (radius * kRadialPeak * g);
}

Real Microburst::strength(Real t) const {
    const Real a = t - startTime;
    if (a <= 0) return 0;
    if (a < growthTime) return smoothstep(0.0, growthTime, a);
    if (a < growthTime + matureTime) return 1;
    return 1 - smoothstep(growthTime + matureTime, growthTime + matureTime + std::max(decayTime, 1e-6), a);
}

Vec3 Microburst::velocity(const Vec3& x, Real z, Real t) const {
    const Real s = strength(t);
    if (s <= 0 || z < 0) return {};
    const Real lam = s * lambda();
    const Real age = t - startTime;
    const Real dx = x.x - (north + driftNorth * age), dy = x.y - (east + driftEast * age);
    const Real q = (dx * dx + dy * dy) / (radius * radius);
    const Real zs = outflowDecayHeight, eps = surfaceLayerDepth;
    // u / r = lambda R^2 / 2 (1 - e^-q) / r^2 = lambda / 2 (1 - e^-q) / q.
    const Real radialOverR = 0.5 * lam * (q > 1e-12 ? -std::expm1(-q) / q : 1.0) * outflowShape(z, zs, eps);
    const Real wUp = -lam * std::exp(-q) * (zs * -std::expm1(-z / zs) - eps * -std::expm1(-z / eps));
    return {radialOverR * dx, radialOverR * dy, -wUp};
}

// ============================================================ vertical air ==

namespace {
// Updraft zero crossing of exp(-q) - exp(-q/4)/4: q = 4 ln 4 / 3.
const Real kThermalZeroRadius = std::sqrt(4 * std::log(4.0) / 3);
// Mean of the shape over r < r2 as a fraction of A.
Real thermalMeanFraction() {
    const Real q0 = sq(kThermalZeroRadius);
    return (std::exp(-q0 / 4) - std::exp(-q0)) / q0;
}
std::uint64_t mix(std::uint64_t a, std::uint64_t b) {
    SplitMix m{a * 0x9E3779B97F4A7C15ull ^ (b + 0x632BE59BD9B4E019ull)};
    return m.next();
}
Real unit(std::uint64_t h) { return static_cast<Real>(h >> 11) * (1.0 / 9007199254740992.0); }
}  // namespace

ThermalField::ThermalField(const ThermalParams& p, const Vec3& drift) : p_(p), drift_{drift.x, drift.y, 0} {
    p_.depth = std::max(p_.depth, 100.0);
    spacing_ = p_.spacing > 0 ? p_.spacing : std::sqrt(p_.depth * outerRadius(0.5 * p_.depth) / 0.6);
}

Real ThermalField::meanUpdraft(Real z) const {
    const Real zi = p_.depth;
    if (z <= 0 || z >= zi / 1.1) return 0;
    // Allen's profile, rounded off at the surface (the inflow layer) so the
    // convergence below the core stays finite.
    return p_.convectiveVelocity * std::cbrt(z / zi) * (1 - 1.1 * z / zi) * -std::expm1(-z / (0.05 * zi));
}

Real ThermalField::outerRadius(Real z) const {
    const Real zi = p_.depth;
    const Real zz = clamp(z, 0.0, zi);
    return std::max(10.0, 0.102 * std::cbrt(zz / zi) * (1 - 0.25 * zz / zi) * zi);
}

Vec3 ThermalField::thermalVelocity(Real dn, Real de, Real z, Real s) const {
    if (s == 0 || z <= 0 || z >= p_.depth / 1.1) return {};
    // Stream function Psi = A R^2/2 (e4 - e1); A and R vary with height.
    auto ar = [&](Real zz, Real& a, Real& rr) {
        rr = outerRadius(zz) / kThermalZeroRadius;
        a = s * meanUpdraft(zz) / thermalMeanFraction();
    };
    Real a, rr;
    ar(z, a, rr);
    const Real r2 = dn * dn + de * de;
    const Real q = r2 / (rr * rr);
    const Real e1 = std::exp(-q), e4 = std::exp(-q / 4);
    if (q > 64) return {};
    const Real wUp = a * (e1 - 0.25 * e4);
    // dPsi/dz = 1/2 (A R^2)' (e4 - e1) + 1/2 A R^2 R' (r^2 / R^3)(e4/2 - 2 e1).
    const Real h = 0.5;
    Real ap, rp, am, rm;
    ar(z + h, ap, rp);
    ar(std::max(z - h, 1e-6), am, rm);
    const Real dz = z + h - std::max(z - h, 1e-6);
    const Real dAR2 = (ap * rp * rp - am * rm * rm) / dz;
    const Real dR = (rp - rm) / dz;
    const Real e4m1 = std::expm1(-q / 4) - std::expm1(-q);  // e4 - e1 without cancellation
    const Real dPsi = 0.5 * dAR2 * e4m1 + 0.5 * a * rr * rr * dR * (r2 / (rr * rr * rr)) * (0.5 * e4 - 2 * e1);
    // u_r = -(1/r) dPsi/dz; as a vector: -(dPsi/dz) / r^2 * (dn, de). On the
    // axis Psi -> 3/8 A r^2, so the limit is -3/8 dA/dz.
    const Real k = r2 > 1e-8 ? -dPsi / r2 : -0.375 * (ap - am) / dz;
    return {k * dn, k * de, -wUp};
}

Vec3 ThermalField::velocity(const Vec3& x, Real z, Real t) const {
    if (!active() || z <= 0 || z >= p_.depth / 1.1) return {};
    const Vec3 xf = x - drift_ * t;
    Vec3 v;
    for (const Thermal& th : p_.thermals) {
        Real s = th.strength;
        if (th.lifetime > 0) {
            const Real ph = (t - th.startTime) / th.lifetime;
            s *= ph > 0 && ph < 1 ? sq(std::sin(kPi * ph)) : 0.0;
        } else if (t < th.startTime) {
            s = 0;
        }
        v += thermalVelocity(xf.x - th.north, xf.y - th.east, z, s);
    }
    if (p_.field) {
        const Real d = spacing_;
        const auto ci = static_cast<long long>(std::floor(xf.x / d)), cj = static_cast<long long>(std::floor(xf.y / d));
        for (long long i = ci - 1; i <= ci + 1; ++i)
            for (long long j = cj - 1; j <= cj + 1; ++j) {
                const std::uint64_t cell = mix(mix(p_.seed, static_cast<std::uint64_t>(i)), static_cast<std::uint64_t>(j));
                // Each cell runs successive thermals with a random phase;
                // a new one starts at a new place when the last dies.
                const Real life = std::max(p_.lifetime, 1.0);
                const Real ph = t / life + unit(mix(cell, 1));
                const auto n = static_cast<std::uint64_t>(static_cast<long long>(std::floor(ph)));
                const std::uint64_t th = mix(cell, n + 17);
                const Real s = (0.6 + 0.8 * unit(mix(th, 2))) * sq(std::sin(kPi * (ph - std::floor(ph))));
                const Real cn = (static_cast<Real>(i) + 0.15 + 0.7 * unit(mix(th, 3))) * d;
                const Real ce = (static_cast<Real>(j) + 0.15 + 0.7 * unit(mix(th, 4))) * d;
                v += thermalVelocity(xf.x - cn, xf.y - ce, z, s);
            }
    }
    return v;
}

Vec3 Ridge::velocity(const Vec3& x, Real altitudeMsl, const Vec3& crestWind) const {
    const Vec3 along{std::cos(axis), std::sin(axis), 0};
    Vec3 n{-along.y, along.x, 0};
    Real u = dot(crestWind, n);
    if (u < 0) { n = -n; u = -u; }
    const Real z = altitudeMsl - baseElevation;
    if (z < 0 || u < 1.0 || height <= 0 || halfWidth <= 0 || stability <= 0) return {};
    const Vec3 d{x.x - north, x.y - east, 0};
    const Real xc = dot(d, n);       // across, downwind positive
    const Real yc = dot(d, along);   // along the crest
    // Fades: light cross-wind, ridge ends, absorbing layer aloft.
    const Real ends = 1 - smoothstep(0.5 * length, 0.5 * length + halfWidth, std::abs(yc));
    const Real top = 1 - smoothstep(waveTop - 1000, waveTop, altitudeMsl);
    const Real fade = smoothstep(1.0, 3.0, u) * ends * top;
    if (fade <= 0) return {};
    const Real a = halfWidth;
    const Real h0 = std::min(height, 0.8 * u / stability);  // blocking limit
    const Real l = stability / u;
    const Real c = std::cos(l * z), s = std::sin(l * z);
    const Real den = xc * xc + a * a;
    const Real detadx = h0 * a * ((xc * xc - a * a) * s - 2 * a * xc * c) / (den * den);
    const Real detadz = h0 * a * l * (-a * s - xc * c) / den;
    const Real wUp = u * detadx * fade;
    const Real du = -u * detadz * fade;
    return {n.x * du, n.y * du, -wUp};
}

Real windShearHazardFactor(Real tailwindRate, Real verticalAirVelocity, Real airspeed) {
    return tailwindRate / kStandardGravity - verticalAirVelocity / std::max(airspeed, 1.0);
}

}  // namespace avd
