// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
#include "avd/gps.hpp"

#include <algorithm>
#include <cmath>

namespace avd {

namespace {
constexpr Real kNm = 1852.0;
Real horizontal(const Vec3& a, const Vec3& b) { return std::hypot(b.x - a.x, b.y - a.y); }
Real track(const Vec3& a, const Vec3& b) { return std::atan2(b.y - a.y, b.x - a.x); }
}  // namespace

void GpsNavigator::setRoute(std::vector<RouteWaypoint> route, int finalApproachFix, Real glidepath, Real thresholdCrossing) {
    route_ = std::move(route);
    faf_ = finalApproachFix < static_cast<int>(route_.size()) ? finalApproachFix : -1;
    glidepath_ = glidepath;
    tch_ = thresholdCrossing;
    active_ = route_.size() >= 2 ? 1 : (route_.empty() ? -1 : 0);
    direct_ = false;
    directPending_ = route_.size() == 1;
    ind_ = GpsIndication{};
}

void GpsNavigator::clear() { setRoute({}); }

void GpsNavigator::directTo(int i) {
    if (i < 0 || i >= static_cast<int>(route_.size())) return;
    active_ = i;
    directPending_ = true;
}

void GpsNavigator::activate(int i) {
    if (i < 1 || i >= static_cast<int>(route_.size())) return;
    active_ = i;
    direct_ = directPending_ = false;
}

const GpsIndication& GpsNavigator::update(const Vec3& pos, Real altitudeMsl, Real groundSpeed) {
    ind_ = GpsIndication{};
    if (active_ < 0 || route_.empty()) return ind_;
    if (directPending_) {
        directFrom_ = pos;
        direct_ = true;
        directPending_ = false;
    }
    const int last = static_cast<int>(route_.size()) - 1;
    auto from = [&]() { return direct_ || active_ == 0 ? directFrom_ : route_[static_cast<std::size_t>(active_ - 1)].position; };
    // Sequence: past the leg's end, or inside the turn anticipation distance
    // of a fly-by waypoint (a standard-rate-ish 25 deg bank turn at this speed).
    for (int k = 0; k < 2 && active_ < last; ++k) {
        const Vec3 a = from(), b = route_[static_cast<std::size_t>(active_)].position;
        const Real len = horizontal(a, b), t = track(a, b);
        const Real along = (pos.x - a.x) * std::cos(t) + (pos.y - a.y) * std::sin(t);
        Real anticipate = 0;
        if (!route_[static_cast<std::size_t>(active_)].flyover) {
            const Real turn = std::abs(wrapPi(track(b, route_[static_cast<std::size_t>(active_ + 1)].position) - t));
            const Real v = std::max(groundSpeed, 30.0);
            const Real r = v * v / (9.80665 * std::tan(0.4363));
            anticipate = std::min(r * std::tan(0.5 * std::min(turn, 2.2)), 5000.0);
        }
        if (along < len - anticipate) break;
        ++active_;
        direct_ = false;
    }
    const RouteWaypoint& to = route_[static_cast<std::size_t>(active_)];
    const Vec3 a = from();
    const Real t = horizontal(a, to.position) > 1 ? track(a, to.position) : track(pos, to.position);
    ind_.valid = true;
    ind_.active = active_;
    ind_.desiredTrack = std::fmod(t + 2 * kPi, 2 * kPi);
    ind_.crossTrack = -(pos.x - a.x) * std::sin(t) + (pos.y - a.y) * std::cos(t);
    ind_.distanceToWaypoint = horizontal(pos, to.position);
    ind_.bearingToWaypoint = std::fmod(track(pos, to.position) + 2 * kPi, 2 * kPi);
    Real remaining = ind_.distanceToWaypoint;
    for (int i = active_; i < last; ++i)
        remaining += horizontal(route_[static_cast<std::size_t>(i)].position, route_[static_cast<std::size_t>(i + 1)].position);
    ind_.distanceToDestination = remaining;
    // Full scale: en route 2 NM, terminal (within 30 NM of the destination)
    // 1 NM, narrowing to 0.3 NM over the 2 NM to the FAF, then angular (2 deg)
    // toward the threshold between 350 ft and 0.3 NM.
    const bool pastFaf = faf_ >= 0 && active_ > faf_;
    const bool toFaf = faf_ >= 0 && active_ == faf_;
    if (pastFaf) {
        ind_.fullScale = std::clamp(remaining * std::tan(0.0349), 107.0, 0.3 * kNm);
        ind_.mode = "APR";
    } else if (toFaf && ind_.distanceToWaypoint < 2 * kNm) {
        ind_.fullScale = kNm * (0.3 + 0.7 * ind_.distanceToWaypoint / (2 * kNm));
        ind_.mode = "APR";
    } else {
        ind_.fullScale = remaining < 30 * kNm ? kNm : 2 * kNm;
        ind_.mode = remaining < 30 * kNm ? "TERM" : "ENR";
    }
    ind_.cdi = std::clamp(-ind_.crossTrack / ind_.fullScale, -1.0, 1.0);
    // Vertical path: in approach mode (inside 2 NM of the FAF, and after it),
    // to the threshold crossing height.
    if (ind_.mode == "APR") {
        const RouteWaypoint& thr = route_.back();
        const Real pathAlt = -thr.position.z + tch_ + remaining * std::tan(glidepath_);
        ind_.verticalValid = true;
        ind_.verticalDeviation = altitudeMsl - pathAlt;
        const Real fs = std::clamp(remaining * std::tan(0.0122), 15.0, 150.0);  // 0.7 deg, as a glideslope
        ind_.vdi = std::clamp(-ind_.verticalDeviation / fs, -1.0, 1.0);
    }
    return ind_;
}

}  // namespace avd
