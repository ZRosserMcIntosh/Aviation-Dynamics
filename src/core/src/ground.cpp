// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
#include "avd/ground.hpp"

namespace avd {

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
