// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
//
// WGS-84 geodesy and local tangent frames.
//
// The flight dynamics integrate in a local North-East-Down tangent frame
// anchored at a geodetic origin (a "flat Earth" frame valid near the anchor).
// For long-distance flight the host re-anchors the frame (`LocalFrame::rebase`
// + `rebaseState` in aircraft code) when the aircraft is far from the anchor,
// which keeps gravity aligned with the true local vertical and keeps
// coordinates small. Earth rotation (Coriolis, centripetal) is neglected; the
// largest omitted term is ~2*Omega*V ≈ 1.5e-4*V m/s^2 (≈0.04 m/s^2 at Mach 0.8).
// See docs/architecture/coordinate-systems.md.
#pragma once

#include "avd/math.hpp"

namespace avd {

struct Geodetic {
    Real latitude = 0;   // rad, positive north
    Real longitude = 0;  // rad, positive east
    Real height = 0;     // m above the WGS-84 ellipsoid
};

namespace wgs84 {
inline constexpr Real kSemiMajor = 6378137.0;
inline constexpr Real kFlattening = 1.0 / 298.257223563;
inline constexpr Real kSemiMinor = kSemiMajor * (1.0 - kFlattening);
inline constexpr Real kE2 = kFlattening * (2.0 - kFlattening);  // first eccentricity squared

Vec3 geodeticToEcef(const Geodetic& g);
Geodetic ecefToGeodetic(const Vec3& ecef);  // iterative (Bowring start), ~1e-9 m
// Somigliana normal gravity at the ellipsoid plus free-air gradient (m/s^2).
Real normalGravity(Real latitude, Real height);
}  // namespace wgs84

// A local tangent frame (NED) anchored at a geodetic origin.
class LocalFrame {
public:
    LocalFrame() : LocalFrame(Geodetic{}) {}
    explicit LocalFrame(const Geodetic& origin);

    const Geodetic& origin() const { return origin_; }

    Vec3 nedFromEcef(const Vec3& ecef) const;
    Vec3 ecefFromNed(const Vec3& ned) const;
    Vec3 nedFromGeodetic(const Geodetic& g) const { return nedFromEcef(wgs84::geodeticToEcef(g)); }
    Geodetic geodeticFromNed(const Vec3& ned) const { return wgs84::ecefToGeodetic(ecefFromNed(ned)); }

    // Rotation taking vectors expressed in this frame's NED axes into ECEF axes.
    const Mat3& nedToEcef() const { return nedToEcef_; }
    // Rotation taking vectors from this frame's NED axes into `other`'s NED axes.
    Mat3 rotationTo(const LocalFrame& other) const { return other.nedToEcef_.transposed() * nedToEcef_; }

private:
    Geodetic origin_;
    Vec3 originEcef_;
    Mat3 nedToEcef_;
};

// Quaternion from a proper rotation matrix (Shepperd's method).
Quat quatFromMatrix(const Mat3& r);

}  // namespace avd
