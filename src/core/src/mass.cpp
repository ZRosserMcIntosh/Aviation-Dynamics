// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
#include "avd/mass.hpp"

namespace avd {

Mat3 pointInertia(Real m, const Vec3& r) {
    Mat3 t = Mat3::zero();
    const Real r2 = dot(r, r);
    const Real v[3] = {r.x, r.y, r.z};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) t.m[i][j] = m * ((i == j ? r2 : 0.0) - v[i] * v[j]);
    return t;
}

Mat3 inertiaTensor(Real ixx, Real iyy, Real izz, Real ixz) {
    Mat3 t = Mat3::diagonal({ixx, iyy, izz});
    t.m[0][2] = t.m[2][0] = -ixz;
    return t;
}

MassProperties combine(const std::vector<MassItem>& items) {
    MassProperties p;
    Vec3 moment;
    for (const MassItem& it : items) {
        p.mass += it.mass;
        moment += it.position * it.mass;
    }
    if (p.mass <= 0) return p;
    p.cg = moment / p.mass;
    Mat3 inertia = Mat3::zero();
    for (const MassItem& it : items) inertia = inertia + it.inertia + pointInertia(it.mass, it.position - p.cg);
    p.inertia = inertia;
    p.inverseInertia = inertia.inverse();
    return p;
}

}  // namespace avd
