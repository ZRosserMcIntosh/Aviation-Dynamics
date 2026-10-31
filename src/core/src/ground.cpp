// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
#include "avd/ground.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

#include "avd/units.hpp"

namespace avd {

namespace {
using units::kPsi;  // Pa
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

bool ElevationGrid::parse(const unsigned char* data, std::size_t size) {
    heights_.clear();
    constexpr std::size_t kHeader = 4 + 2 * 4 + 4 * 8;
    if (!data || size < kHeader || std::memcmp(data, "AVT1", 4) != 0) return false;
    std::int32_t nx = 0, ny = 0;
    double e0 = 0, n0 = 0, sp = 0, scale = 0;
    std::memcpy(&nx, data + 4, 4);
    std::memcpy(&ny, data + 8, 4);
    std::memcpy(&e0, data + 12, 8);
    std::memcpy(&n0, data + 20, 8);
    std::memcpy(&sp, data + 28, 8);
    std::memcpy(&scale, data + 36, 8);
    if (nx < 2 || ny < 2 || nx > 20000 || ny > 20000 || !(sp > 0) || !(scale > 0) || !std::isfinite(e0) || !std::isfinite(n0)) return false;
    const std::size_t count = static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny);
    if (size != kHeader + 2 * count) return false;
    heights_.resize(count);
    for (std::size_t k = 0; k < count; ++k) {
        std::int16_t v = 0;
        std::memcpy(&v, data + kHeader + 2 * k, 2);
        heights_[k] = static_cast<float>(v * scale);
    }
    nx_ = nx, ny_ = ny, east0_ = e0, north0_ = n0, spacing_ = sp;
    return true;
}

Real ElevationGrid::heightAt(Real north, Real east) const {
    if (heights_.empty()) return std::numeric_limits<Real>::quiet_NaN();
    const Real fx = (east - east0_) / spacing_, fy = (north - north0_) / spacing_;
    if (!(fx >= 0 && fy >= 0 && fx <= nx_ - 1 && fy <= ny_ - 1)) return std::numeric_limits<Real>::quiet_NaN();
    const int i = std::min(static_cast<int>(fx), nx_ - 2), j = std::min(static_cast<int>(fy), ny_ - 2);
    const Real u = fx - i, v = fy - j;
    const std::size_t k = static_cast<std::size_t>(j) * static_cast<std::size_t>(nx_) + static_cast<std::size_t>(i);
    const Real h00 = heights_[k], h10 = heights_[k + 1], h01 = heights_[k + static_cast<std::size_t>(nx_)], h11 = heights_[k + static_cast<std::size_t>(nx_) + 1];
    return (h00 * (1 - u) + h10 * u) * (1 - v) + (h01 * (1 - u) + h11 * u) * v;
}

Real ElevationGrid::inside(Real north, Real east) const {
    if (heights_.empty()) return -1e9;
    const Real w = (nx_ - 1) * spacing_, h = (ny_ - 1) * spacing_;
    const Real x = east - east0_, y = north - north0_;
    return std::min({x, w - x, y, h - y});
}

Real RegionGround::terrainDown(Real n, Real e) const {
    if (fields_.empty() && !terrain_) return 0;
    // Within a field's flat area: its elevation.
    const Field* nearest = nullptr;
    Real nearestGap = 1e18;
    for (const Field& f : fields_) {
        const Real gap = std::hypot(n - f.north, e - f.east) - f.radius;
        if (gap <= 0) return f.down;
        if (gap < nearestGap) nearestGap = gap, nearest = &f;
    }
    const Real edge = terrain_ ? terrain_->inside(n, e) : -1;
    if (edge <= 0) return blendDown(n, e);
    // The terrain, brought to the nearest field's elevation over 2 km beyond
    // its flat area, and to the fields' blend over the grid's last 5 km.
    Real down = -terrain_->heightAt(n, e);
    if (nearest) down += (nearest->down - down) * (1 - smoothstep(0.0, 2000.0, nearestGap));
    if (edge < 5000) down += (blendDown(n, e) - down) * (1 - smoothstep(0.0, 5000.0, edge));
    return down;
}

Real RegionGround::blendDown(Real n, Real e) const {
    if (fields_.empty()) return 0;
    Real wsum = 0, dsum = 0;
    for (const Field& f : fields_) {
        const Real d = std::max(std::hypot(n - f.north, e - f.east) - f.radius, 0.0);
        if (d <= 0) return f.down;
        const Real w = 1 / (d * d);
        wsum += w;
        dsum += w * f.down;
    }
    return dsum / wsum;
}

void RegionGround::surfaceAt(Real n, Real e, Real& down, Vec3& normal, SurfaceProperties& s) const {
    s = surfaces::kGrass;
    s.waterDepth = waterDepth_;
    for (const Field& f : fields_) {
        if (std::hypot(n - f.north, e - f.east) > f.radius) continue;
        down = f.down;
        normal = {0, 0, -1};
        for (const RunwayDef& r : f.runways) {
            const Real ch = std::cos(r.headingTrue), sh = std::sin(r.headingTrue);
            const Real dn = n - r.threshold.x, de = e - r.threshold.y;
            const Real along = dn * ch + de * sh, across = -dn * sh + de * ch;
            if (along < 0 || along > r.length || std::abs(across) > 0.5 * r.width) continue;
            down = r.threshold.z - r.slope * along;
            normal = normalized(Vec3{-r.slope * ch, -r.slope * sh, -1});
            s = r.surface;
            s.waterDepth = waterDepth_;
            return;
        }
        return;
    }
    // Between the fields: the blended terrain, its normal by central differences.
    constexpr Real h = 25;
    down = terrainDown(n, e);
    const Real dN = (terrainDown(n + h, e) - terrainDown(n - h, e)) / (2 * h);
    const Real dE = (terrainDown(n, e + h) - terrainDown(n, e - h)) / (2 * h);
    normal = normalized(Vec3{dN, dE, -1});
}

GroundHit RegionGround::raycast(const Vec3& o, const Vec3& dir, Real maxDistance) const {
    GroundHit h;
    if (terrain_ && std::hypot(dir.x, dir.y) > 1e-6) {
        // Real terrain: march along the ray, each step at most half the
        // height above the ground (and at least 5 m), then bisect the
        // crossing. Ridges steeper than 2:1 can be stepped through at the
        // 5 m floor only.
        auto above = [&](Real t) {
            const Vec3 p = o + dir * t;
            return terrainDown(p.x, p.y) - p.z;  // + above the ground
        };
        Real t0 = 0, a0 = above(0);
        if (a0 < 0) return h;
        bool crossed = false;
        Real t1 = 0;
        while (t0 < maxDistance) {
            t1 = std::min(t0 + std::max(0.5 * a0, 5.0), maxDistance);
            const Real a1 = above(t1);
            if (a1 <= 0) {
                crossed = true;
                break;
            }
            t0 = t1, a0 = a1;
            if (t1 >= maxDistance) break;
        }
        if (!crossed) return h;
        for (int i = 0; i < 30; ++i) {
            const Real tm = 0.5 * (t0 + t1);
            (above(tm) > 0 ? t0 : t1) = tm;
        }
        const Vec3 p = o + dir * t1;
        Real down;
        surfaceAt(p.x, p.y, down, h.normal, h.surface);
        h.hit = true;
        h.distance = t1;
        h.point = {p.x, p.y, down};
        return h;
    }
    // Gentle terrain (or a vertical ray): start on the plane of the terrain
    // under the origin and refine on the local tangent plane.
    if (dir.z <= 1e-9) return h;
    Real t = (terrainDown(o.x, o.y) - o.z) / dir.z;
    for (int i = 0; i < 6; ++i) {
        const Vec3 p = o + dir * t;
        Real down;
        Vec3 nrm;
        SurfaceProperties s;
        surfaceAt(p.x, p.y, down, nrm, s);
        const Real denom = dot(dir, nrm);
        if (std::abs(denom) < 1e-9) break;
        t = dot(Vec3{p.x, p.y, down} - o, nrm) / denom;
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
