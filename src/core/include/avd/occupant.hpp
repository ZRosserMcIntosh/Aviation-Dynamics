// Aviation Dynamics — core.
// SPDX-License-Identifier: MIT
//
// A crash-test dummy in a seat: what the occupant of that seat goes through,
// from the airframe's motion. The seat's acceleration is the accelerometer's
// at the centre of gravity carried to the seat by the rotation (the angular
// acceleration's and the centripetal terms); the occupant is restrained and
// moves with the seat (no slack in the belts, no head strike modelled).
//
// - Peak resultant and spinal (eyeballs-down) acceleration, g.
// - The spinal Dynamic Response Index (Stech & Payne, AMRL-TR-66-157): the
//   spine and the mass above it as a damped spring, wn = 52.9 rad/s,
//   zeta = 0.224, driven by the seat's acceleration along the spine;
//   DRI = wn^2 * peak compression / g (1 sitting still). DRI 18 is about a
//   5 % risk of spinal injury (the ejection-seat design limit).
// - An estimate of the lumbar load: the ~45 kg of a 77 kg occupant above the
//   lumbar spine on that spring, 45 kg * g * DRI. The 14 CFR 23.562 / 25.562
//   seat test limit is 1,500 lbf (6.67 kN).
// - HIC(36): the head injury criterion over windows up to 36 ms, from the
//   seat's resultant acceleration (the restrained head moving with the torso;
//   a head strike would add to it). The 23.562 / 25.562 limit is 1,000.
#pragma once

#include <array>

#include "avd/math.hpp"

namespace avd {

struct OccupantReport {
    Real peakG = 0;          // resultant, g
    Real peakSpinalG = 0;    // eyeballs-down (seat pushing up the spine), g
    Real dri = 1;            // spinal Dynamic Response Index (1 at rest)
    Real lumbarN = 0;        // estimated lumbar compression, N
    Real hic = 0;            // HIC(36)
    // Against the seat certification limits (14 CFR 23.562 / 25.562) and DRI 18.
    bool hicExceeded() const { return hic > 1000; }
    bool lumbarExceeded() const { return lumbarN > 6672; }  // 1,500 lbf
    bool driExceeded() const { return dri > 18; }
};

class OccupantMonitor {
public:
    // One step: the accelerometer at the CG (specific force, body frame,
    // m/s^2), the body rates and their rate of change, and the seat's position
    // from the CG (body frame, m).
    void update(Real dt, const Vec3& specificForceCg, const Vec3& omega, const Vec3& omegaDot, const Vec3& seatFromCg);
    void reset();
    const OccupantReport& report() const { return report_; }
    // The seat's acceleration this step (specific force, body frame, m/s^2).
    const Vec3& seatSpecificForce() const { return seat_; }

private:
    OccupantReport report_;
    Vec3 seat_;
    Real delta_ = 0, deltaRate_ = 0;  // spinal compression (m) and its rate
    bool started_ = false;
    static constexpr int kWindow = 64;  // samples kept for HIC (36 ms at 1 kHz and slower)
    std::array<Real, kWindow> aG_{};    // resultant, g
    std::array<Real, kWindow> dt_{};
    int count_ = 0, head_ = 0;
};

}  // namespace avd
