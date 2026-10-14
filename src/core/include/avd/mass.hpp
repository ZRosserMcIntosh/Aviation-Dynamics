// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
//
// Mass properties. The aircraft is a set of mass items (empty airframe,
// fuel in each tank, occupants, baggage, cargo, detachable components), each
// with a mass, a location in the structural body frame, and an inertia tensor
// about its own centre of mass. Totals (mass, CG, inertia about the CG) are
// recomputed whenever any item changes, so fuel burn, fuel transfer and
// component separation change the physics exactly as they change the loading.
//
// Structural body frame: origin at the aircraft's reference datum, axes FRD.
#pragma once

#include <string>
#include <vector>

#include "avd/math.hpp"

namespace avd {

struct MassItem {
    std::string name;
    Real mass = 0;           // kg
    Vec3 position;           // m, CG of the item in the structural frame
    Mat3 inertia = Mat3::zero();  // kg m^2 about the item's own CG, FRD axes
};

struct MassProperties {
    Real mass = 0;
    Vec3 cg;                       // structural frame
    Mat3 inertia = Mat3::zero();   // about cg
    Mat3 inverseInertia = Mat3::zero();
};

// Inertia tensor of a point mass m at offset r from the reference point
// (parallel-axis term): m (|r|^2 I - r r^T).
Mat3 pointInertia(Real m, const Vec3& r);

// Inertia tensor from principal-ish moments and the xz product of inertia
// (aircraft convention: Ixz enters with a minus sign in the tensor).
Mat3 inertiaTensor(Real ixx, Real iyy, Real izz, Real ixz = 0);

MassProperties combine(const std::vector<MassItem>& items);

}  // namespace avd
