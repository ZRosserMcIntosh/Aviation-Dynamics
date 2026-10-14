// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
//
// Minimal linear algebra. Double precision throughout (see
// docs/architecture/coordinate-systems.md).
//
// Frame conventions used by the flight dynamics (aerospace standard, ISO 1151 /
// LN 9300): body axes are X forward, Y right, Z down (FRD); the local navigation
// frame is North-East-Down (NED). Attitude quaternions rotate body vectors into
// the local NED frame. The world/game frame (ENU, Z up) is converted at the
// boundary in avd/world_frame.hpp. SI units internally.
#pragma once

#include <algorithm>
#include <cmath>

namespace avd {

using Real = double;

inline constexpr Real kPi = 3.14159265358979323846;
inline constexpr Real kTwoPi = 2.0 * kPi;
inline constexpr Real kStandardGravity = 9.80665;  // m/s^2 (CGPM 1901)

struct Vec3 {
    Real x = 0, y = 0, z = 0;

    constexpr Vec3() = default;
    constexpr Vec3(Real x_, Real y_, Real z_) : x(x_), y(y_), z(z_) {}

    constexpr Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    constexpr Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    constexpr Vec3 operator-() const { return {-x, -y, -z}; }
    constexpr Vec3 operator*(Real s) const { return {x * s, y * s, z * s}; }
    constexpr Vec3 operator/(Real s) const { return {x / s, y / s, z / s}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    Vec3& operator*=(Real s) { x *= s; y *= s; z *= s; return *this; }
    constexpr bool operator==(const Vec3&) const = default;
};

constexpr Vec3 operator*(Real s, const Vec3& v) { return v * s; }
constexpr Real dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
constexpr Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline Real length(const Vec3& v) { return std::sqrt(dot(v, v)); }
inline Vec3 normalized(const Vec3& v) {
    const Real l = length(v);
    return l > 1e-15 ? v / l : Vec3{};
}
inline bool isFinite(const Vec3& v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

struct Mat3 {
    Real m[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};  // row-major

    static Mat3 zero() {
        Mat3 r;
        for (auto& row : r.m)
            for (Real& e : row) e = 0;
        return r;
    }
    static Mat3 diagonal(const Vec3& d) {
        Mat3 r = zero();
        r.m[0][0] = d.x; r.m[1][1] = d.y; r.m[2][2] = d.z;
        return r;
    }
    Vec3 operator*(const Vec3& v) const {
        return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
                m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
    }
    Mat3 operator*(const Mat3& o) const {
        Mat3 r = zero();
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                for (int k = 0; k < 3; ++k) r.m[i][j] += m[i][k] * o.m[k][j];
        return r;
    }
    Mat3 operator+(const Mat3& o) const {
        Mat3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) r.m[i][j] = m[i][j] + o.m[i][j];
        return r;
    }
    Mat3 operator*(Real s) const {
        Mat3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) r.m[i][j] = m[i][j] * s;
        return r;
    }
    Mat3 transposed() const {
        Mat3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) r.m[i][j] = m[j][i];
        return r;
    }
    Real determinant() const {
        return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
               m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
               m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    }
    // Inverse; returns identity-scaled garbage if singular, so callers that
    // accept untrusted data must validate positive-definiteness first.
    Mat3 inverse() const {
        const Real d = determinant();
        const Real id = d != 0 ? 1.0 / d : 0.0;
        Mat3 r;
        r.m[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * id;
        r.m[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * id;
        r.m[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * id;
        r.m[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * id;
        r.m[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * id;
        r.m[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * id;
        r.m[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * id;
        r.m[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * id;
        r.m[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * id;
        return r;
    }
};

struct Quat {
    Real w = 1, x = 0, y = 0, z = 0;

    static Quat fromAxisAngle(const Vec3& axis, Real angle) {
        const Vec3 a = normalized(axis);
        const Real s = std::sin(angle * 0.5);
        return {std::cos(angle * 0.5), a.x * s, a.y * s, a.z * s};
    }
    // Aerospace 3-2-1 Euler sequence (yaw psi, pitch theta, roll phi):
    // body -> NED rotation.
    static Quat fromEuler(Real roll, Real pitch, Real yaw) {
        const Real cr = std::cos(roll * 0.5), sr = std::sin(roll * 0.5);
        const Real cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
        const Real cy = std::cos(yaw * 0.5), sy = std::sin(yaw * 0.5);
        return {cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy};
    }
    Quat operator*(const Quat& q) const {
        return {w * q.w - x * q.x - y * q.y - z * q.z, w * q.x + x * q.w + y * q.z - z * q.y,
                w * q.y - x * q.z + y * q.w + z * q.x, w * q.z + x * q.y - y * q.x + z * q.w};
    }
    Quat operator+(const Quat& q) const { return {w + q.w, x + q.x, y + q.y, z + q.z}; }
    Quat operator*(Real s) const { return {w * s, x * s, y * s, z * s}; }
    Quat conjugate() const { return {w, -x, -y, -z}; }
    Real norm() const { return std::sqrt(w * w + x * x + y * y + z * z); }
    void normalize() {
        const Real n = norm();
        if (n > 0) { w /= n; x /= n; y /= n; z /= n; }
    }
    // Rotate a body-frame vector into the parent (NED) frame.
    Vec3 rotate(const Vec3& v) const {
        const Vec3 u{x, y, z};
        const Vec3 t = 2.0 * cross(u, v);
        return v + w * t + cross(u, t);
    }
    // Rotate a parent-frame vector into the body frame.
    Vec3 inverseRotate(const Vec3& v) const { return conjugate().rotate(v); }

    // Euler angles (3-2-1). Pitch is clamped to +-90 deg at the singularity;
    // the quaternion itself has no singularity.
    Real roll() const { return std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)); }
    Real pitch() const { return std::asin(std::clamp(2 * (w * y - z * x), -1.0, 1.0)); }
    Real yaw() const { return std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)); }

    Mat3 toMatrix() const {
        Mat3 r;
        r.m[0][0] = 1 - 2 * (y * y + z * z); r.m[0][1] = 2 * (x * y - w * z);     r.m[0][2] = 2 * (x * z + w * y);
        r.m[1][0] = 2 * (x * y + w * z);     r.m[1][1] = 1 - 2 * (x * x + z * z); r.m[1][2] = 2 * (y * z - w * x);
        r.m[2][0] = 2 * (x * z - w * y);     r.m[2][1] = 2 * (y * z + w * x);     r.m[2][2] = 1 - 2 * (x * x + y * y);
        return r;
    }
};

// Time derivative of a body->parent quaternion for body angular rate omega.
inline Quat quatDerivative(const Quat& q, const Vec3& omegaBody) {
    const Quat o{0, omegaBody.x, omegaBody.y, omegaBody.z};
    return (q * o) * 0.5;
}

inline Real clamp(Real v, Real lo, Real hi) { return std::min(std::max(v, lo), hi); }
inline Real sign(Real v) { return (v > 0) - (v < 0); }
inline Real lerp(Real a, Real b, Real t) { return a + (b - a) * t; }
inline Real sq(Real v) { return v * v; }
inline Real approach(Real current, Real target, Real maxDelta) {
    if (current < target) return std::min(current + maxDelta, target);
    return std::max(current - maxDelta, target);
}
// Wrap an angle to (-pi, pi].
inline Real wrapPi(Real a) {
    a = std::fmod(a + kPi, kTwoPi);
    if (a <= 0) a += kTwoPi;
    return a - kPi;
}
// Smooth 0..1 step between edge0 and edge1.
inline Real smoothstep(Real e0, Real e1, Real v) {
    const Real t = clamp((v - e0) / (e1 - e0), 0.0, 1.0);
    return t * t * (3 - 2 * t);
}

}  // namespace avd
