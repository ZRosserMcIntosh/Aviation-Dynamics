// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
//
// References: NIMA TR8350.2 (WGS-84 definition, normal gravity); Bowring (1976)
// for the geodetic latitude iteration start.
#include "avd/geodesy.hpp"

#include <cmath>

namespace avd {
namespace wgs84 {

Vec3 geodeticToEcef(const Geodetic& g) {
    const Real sl = std::sin(g.latitude), cl = std::cos(g.latitude);
    const Real n = kSemiMajor / std::sqrt(1.0 - kE2 * sl * sl);
    return {(n + g.height) * cl * std::cos(g.longitude), (n + g.height) * cl * std::sin(g.longitude),
            (n * (1.0 - kE2) + g.height) * sl};
}

Geodetic ecefToGeodetic(const Vec3& e) {
    Geodetic g;
    g.longitude = std::atan2(e.y, e.x);
    const Real p = std::sqrt(e.x * e.x + e.y * e.y);
    if (p < 1e-9) {  // on the polar axis
        g.latitude = e.z >= 0 ? kPi / 2 : -kPi / 2;
        g.height = std::abs(e.z) - kSemiMinor;
        return g;
    }
    const Real ep2 = kE2 / (1.0 - kE2);
    // Bowring's parametric-latitude start, then Newton-like refinement.
    Real beta = std::atan2(e.z * kSemiMajor, p * kSemiMinor);
    Real lat = std::atan2(e.z + ep2 * kSemiMinor * std::pow(std::sin(beta), 3),
                          p - kE2 * kSemiMajor * std::pow(std::cos(beta), 3));
    for (int i = 0; i < 4; ++i) {
        beta = std::atan2((1.0 - kFlattening) * std::sin(lat), std::cos(lat));
        lat = std::atan2(e.z + ep2 * kSemiMinor * std::pow(std::sin(beta), 3),
                         p - kE2 * kSemiMajor * std::pow(std::cos(beta), 3));
    }
    const Real sl = std::sin(lat);
    const Real n = kSemiMajor / std::sqrt(1.0 - kE2 * sl * sl);
    g.latitude = lat;
    g.height = std::abs(std::cos(lat)) > 1e-6 ? p / std::cos(lat) - n : std::abs(e.z) / std::abs(sl) - n * (1.0 - kE2);
    return g;
}

Real normalGravity(Real latitude, Real height) {
    // Somigliana closed form (TR8350.2 eq. 4-1) with the second-order free-air
    // correction (eq. 4-3).
    constexpr Real ge = 9.7803253359, k = 0.00193185265241;
    constexpr Real m = 0.00344978650684;
    const Real s2 = std::sin(latitude) * std::sin(latitude);
    const Real g0 = ge * (1.0 + k * s2) / std::sqrt(1.0 - kE2 * s2);
    const Real a = kSemiMajor;
    return g0 * (1.0 - 2.0 / a * (1.0 + kFlattening + m - 2.0 * kFlattening * s2) * height +
                 3.0 / (a * a) * height * height);
}

void transverseMercator(const Geodetic& g, Real lon0, Real k0, Real& east, Real& north) {
    // Karney (2011), "Transverse Mercator with an accuracy of a few nanometers", eqs. 9-12,
    // with Krüger's alpha coefficients to n^6.
    const Real n = kFlattening / (2.0 - kFlattening), n2 = n * n, n3 = n2 * n, n4 = n3 * n, n5 = n4 * n, n6 = n5 * n;
    const Real A = kSemiMajor / (1 + n) * (1 + n2 / 4 + n4 / 64 + n6 / 256);
    const Real alpha[7] = {0,
                           n / 2 - 2 * n2 / 3 + 5 * n3 / 16 + 41 * n4 / 180 - 127 * n5 / 288 + 7891 * n6 / 37800,
                           13 * n2 / 48 - 3 * n3 / 5 + 557 * n4 / 1440 + 281 * n5 / 630 - 1983433 * n6 / 1935360,
                           61 * n3 / 240 - 103 * n4 / 140 + 15061 * n5 / 26880 + 167603 * n6 / 181440,
                           49561 * n4 / 161280 - 179 * n5 / 168 + 6601661 * n6 / 7257600,
                           34729 * n5 / 80640 - 3418889 * n6 / 1995840,
                           212378941 * n6 / 319334400};
    const Real e = std::sqrt(kE2), phi = g.latitude, lam = g.longitude - lon0;
    // Conformal latitude via tau' (Karney eq. 7).
    const Real tau = std::tan(phi), sigma = std::sinh(e * std::atanh(e * tau / std::sqrt(1 + tau * tau)));
    const Real taup = tau * std::sqrt(1 + sigma * sigma) - sigma * std::sqrt(1 + tau * tau);
    const Real xip = std::atan2(taup, std::cos(lam)), etap = std::asinh(std::sin(lam) / std::sqrt(taup * taup + std::cos(lam) * std::cos(lam)));
    Real xi = xip, eta = etap;
    for (int j = 1; j <= 6; ++j) {
        xi += alpha[j] * std::sin(2 * j * xip) * std::cosh(2 * j * etap);
        eta += alpha[j] * std::cos(2 * j * xip) * std::sinh(2 * j * etap);
    }
    east = k0 * A * eta;
    north = k0 * A * xi;
}

}  // namespace wgs84

LocalFrame::LocalFrame(const Geodetic& origin) : origin_(origin), originEcef_(wgs84::geodeticToEcef(origin)) {
    const Real sl = std::sin(origin.latitude), cl = std::cos(origin.latitude);
    const Real so = std::sin(origin.longitude), co = std::cos(origin.longitude);
    // Columns: North, East, Down unit vectors in ECEF.
    const Vec3 n{-sl * co, -sl * so, cl};
    const Vec3 e{-so, co, 0};
    const Vec3 d{-cl * co, -cl * so, -sl};
    nedToEcef_.m[0][0] = n.x; nedToEcef_.m[0][1] = e.x; nedToEcef_.m[0][2] = d.x;
    nedToEcef_.m[1][0] = n.y; nedToEcef_.m[1][1] = e.y; nedToEcef_.m[1][2] = d.y;
    nedToEcef_.m[2][0] = n.z; nedToEcef_.m[2][1] = e.z; nedToEcef_.m[2][2] = d.z;
}

Vec3 LocalFrame::nedFromEcef(const Vec3& ecef) const { return nedToEcef_.transposed() * (ecef - originEcef_); }
Vec3 LocalFrame::ecefFromNed(const Vec3& ned) const { return originEcef_ + nedToEcef_ * ned; }

Quat quatFromMatrix(const Mat3& r) {
    const auto& m = r.m;
    const Real tr = m[0][0] + m[1][1] + m[2][2];
    Quat q;
    if (tr > 0) {
        const Real s = std::sqrt(tr + 1.0) * 2;
        q = {0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s};
    } else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
        const Real s = std::sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2;
        q = {(m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s};
    } else if (m[1][1] > m[2][2]) {
        const Real s = std::sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2;
        q = {(m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s};
    } else {
        const Real s = std::sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2;
        q = {(m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s};
    }
    q.normalize();
    return q;
}

}  // namespace avd
