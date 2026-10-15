// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
//
// Boundary conversions between the flight-dynamics frames (local NED, body FRD)
// and the game world frame (ENU: X east, Y north, Z up, metres) used by the
// São Paulo World tiles and, with z up, compatible with OpenRoad Dynamics'
// world axes. Body FRD <-> a game "FLU" body (X forward, Y left, Z up, ISO 8855
// as in OpenRoad Dynamics) is also provided for renderers and the car engine.
//
//   NED -> ENU:  (n, e, d) -> (e, n, -d)      (180 deg about (1,1,0)/sqrt2, det = +1)
//   FRD -> FLU:  (x, y, z) -> (x, -y, -z)     (180 deg about X, det = +1)
//
// Both maps are proper rotations, so axial vectors (angular velocity, torque)
// transform exactly like polar vectors.
#pragma once

#include "avd/math.hpp"

namespace avd::world {

inline Vec3 enuFromNed(const Vec3& v) { return {v.y, v.x, -v.z}; }
inline Vec3 nedFromEnu(const Vec3& v) { return {v.y, v.x, -v.z}; }
inline Vec3 fluFromFrd(const Vec3& v) { return {v.x, -v.y, -v.z}; }
inline Vec3 frdFromFlu(const Vec3& v) { return {v.x, -v.y, -v.z}; }

// Orientation of the body expressed for a renderer: rotation taking FLU body
// vectors into ENU world vectors.  q_enu_flu = R_enu_ned * q_ned_frd * R_frd_flu.
inline Quat enuFluFromNedFrd(const Quat& qNedFrd) {
    // R_enu_ned: swap x/y, negate z. As a quaternion: 180 deg about (1,1,0)/sqrt2.
    const Real s = std::sqrt(0.5);
    const Quat enuNed{0, s, s, 0};
    const Quat frdFlu{0, 1, 0, 0};  // 180 deg about X
    return enuNed * qNedFrd * frdFlu;
}

// Game tile index for a 500 m São Paulo World tile containing (east, north):
// tiles are centred on multiples of 500 m (T_{i}_{j}, i = round(x/500)).
inline int tileIndex(Real metres, Real tileSize = 500.0) { return static_cast<int>(std::lround(metres / tileSize)); }

}  // namespace avd::world
