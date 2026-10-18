// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
#include "avd/ground.hpp"

#include <cmath>

namespace avd {

namespace {
constexpr Real kPsi = 6894.757;   // Pa
constexpr Real kKnot = 1852.0 / 3600.0;  // m/s
// 14 CFR 25.109(c)(1): mu = a (V/100)^3 + b (V/100)^2 + c (V/100) + d.
struct WetCurve { Real psi, a, b, c, d; };
constexpr WetCurve kWet[] = {
    {50, -0.0350, 0.306, -0.851, 0.883},
    {100, -0.0437, 0.320, -0.805, 0.804},
    {200, -0.0331, 0.252, -0.658, 0.692},
};
Real wetCurve(const WetCurve& w, Real v100) { return ((w.a * v100 + w.b) * v100 + w.c) * v100 + w.d; }
}  // namespace

Real hydroplaningSpeed(Real tyrePressure) {
    const Real psi = tyrePressure > 0 ? tyrePressure / kPsi : 50.0;
    return 9.0 * std::sqrt(psi) * kKnot;
}

Real wetTyreFriction(const SurfaceProperties& s, Real speed, Real tyrePressure) {
    if (s.waterDepth <= 0) return s.friction;
    const bool paved = s.id == 0 || s.id == 1;
    if (!paved) return 0.6 * s.friction;
    const Real psi = std::clamp(tyrePressure > 0 ? tyrePressure / kPsi : 50.0, 50.0, 200.0);
    // The cubics are fitted up to ~200 kt; hold the end value beyond.
    const Real v100 = std::min(std::abs(speed) / kKnot, 200.0) / 100.0;
    const int i = psi < 100 ? 0 : 1;
    const Real f = (psi - kWet[i].psi) / (kWet[i + 1].psi - kWet[i].psi);
    Real mu = std::min(s.friction, (1 - f) * wetCurve(kWet[i], v100) + f * wetCurve(kWet[i + 1], v100));
    if (s.waterDepth >= 0.003) {
        // Standing water: displacement and the water wedge under the tread
        // take ever more of the contact patch toward the hydroplaning speed.
        const Real vp = hydroplaningSpeed(tyrePressure);
        const Real r = std::abs(speed) / vp;
        if (r >= 1) return 0.05;
        mu *= 1 - 0.5 * r * r;
        if (r > 0.9) mu = mu + (0.05 - mu) * (r - 0.9) / 0.1;
    }
    return std::max(mu, 0.05);
}

GroundHit FlatGround::raycast(const Vec3& o, const Vec3& dir, Real maxDistance) const {
    GroundHit h;
    if (dir.z <= 1e-9) return h;  // plane faces up (-Z); only downward rays hit
    const Real t = (d_ - o.z) / dir.z;
    if (t < 0 || t > maxDistance) return h;
    h.hit = true;
    h.distance = t;
    h.point = o + dir * t;
    h.surface = s_;
    return h;
}

void AirfieldGround::surfaceAt(Real n, Real e, Real& down, Vec3& normal, SurfaceProperties& s) const {
    down = fieldDown_;
    normal = {0, 0, -1};
    s = field_;
    s.waterDepth = waterDepth_;
    for (const RunwayDef& r : runways_) {
        const Real ch = std::cos(r.headingTrue), sh = std::sin(r.headingTrue);
        const Real dn = n - r.threshold.x, de = e - r.threshold.y;
        const Real along = dn * ch + de * sh;
        const Real across = -dn * sh + de * ch;
        if (along < 0 || along > r.length || std::abs(across) > 0.5 * r.width) continue;
        down = r.threshold.z - r.slope * along;
        // Up-facing normal of z_down = d0 - slope * along: tilts back downhill.
        normal = normalized(Vec3{-r.slope * ch, -r.slope * sh, -1});
        s = r.surface;
        s.waterDepth = waterDepth_;
        return;
    }
}

GroundHit AirfieldGround::raycast(const Vec3& o, const Vec3& dir, Real maxDistance) const {
    // Piecewise-planar ground: march to the field plane, then refine on the
    // local plane (runway slopes are small, two iterations suffice).
    GroundHit h;
    if (dir.z <= 1e-9) return h;
    Real t = (fieldDown_ - o.z) / dir.z;
    for (int i = 0; i < 3; ++i) {
        const Vec3 p = o + dir * t;
        Real down;
        Vec3 nrm;
        SurfaceProperties s;
        surfaceAt(p.x, p.y, down, nrm, s);
        // Plane through (p.x, p.y, down) with normal nrm.
        const Vec3 p0{p.x, p.y, down};
        const Real denom = dot(dir, nrm);
        if (std::abs(denom) < 1e-9) break;
        t = dot(p0 - o, nrm) / denom;
        h.normal = nrm;
        h.surface = s;
    }
    if (t < 0 || t > maxDistance) return {};
    h.hit = true;
    h.distance = t;
    h.point = o + dir * t;
    return h;
}

}  // namespace avd
