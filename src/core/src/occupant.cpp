// Aviation Dynamics — core.
// SPDX-License-Identifier: MIT
#include "avd/occupant.hpp"

#include <algorithm>
#include <cmath>

namespace avd {

namespace {
constexpr Real kG = 9.80665;
constexpr Real kWn = 52.9, kZeta = 0.224;  // Stech & Payne's spinal model
constexpr Real kUpperBody = 45.0;          // kg above the lumbar spine (77 kg occupant)
}  // namespace

void OccupantMonitor::reset() {
    *this = OccupantMonitor{};
}

void OccupantMonitor::update(Real dt, const Vec3& f, const Vec3& w, const Vec3& wDot, const Vec3& r) {
    if (dt <= 0) return;
    // The seat's specific force: the CG's, plus the rotation's (tangential and centripetal) terms.
    seat_ = f + cross(wDot, r) + cross(w, cross(w, r));
    const Real a = length(seat_) / kG;
    // Along the spine (body z down): the seat pushing up compresses it.
    const Real spinal = -seat_.z;
    if (!started_) {
        // Settled at the start: the spring at its static compression.
        delta_ = spinal / (kWn * kWn);
        deltaRate_ = 0;
        started_ = true;
    }
    // The spine: delta'' + 2 zeta wn delta' + wn^2 delta = a_spinal (semi-implicit, substeps under 1 ms).
    const int n = std::max(1, static_cast<int>(std::ceil(dt / 0.001)));
    const Real h = dt / n;
    for (int i = 0; i < n; ++i) {
        deltaRate_ += (spinal - 2 * kZeta * kWn * deltaRate_ - kWn * kWn * delta_) * h;
        delta_ += deltaRate_ * h;
    }
    const Real dri = kWn * kWn * delta_ / kG;
    report_.peakG = std::max(report_.peakG, a);
    report_.peakSpinalG = std::max(report_.peakSpinalG, spinal / kG);
    report_.dri = std::max(report_.dri, dri);
    report_.lumbarN = std::max(report_.lumbarN, kUpperBody * kG * std::max(dri, 0.0));
    // HIC: every window ending now, up to 36 ms: T * (mean acceleration over T)^2.5.
    head_ = (head_ + 1) % kWindow;
    aG_[static_cast<std::size_t>(head_)] = a;
    dt_[static_cast<std::size_t>(head_)] = dt;
    count_ = std::min(count_ + 1, kWindow);
    Real T = 0, integral = 0;
    for (int k = 0; k < count_; ++k) {
        const std::size_t idx = static_cast<std::size_t>((head_ - k + kWindow) % kWindow);
        if (T + dt_[idx] > 0.036 + 1e-9) break;
        T += dt_[idx];
        integral += aG_[idx] * dt_[idx];
        report_.hic = std::max(report_.hic, T * std::pow(integral / T, 2.5));
    }
}

}  // namespace avd
