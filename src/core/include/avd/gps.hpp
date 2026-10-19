// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
//
// GPS navigation along a route: the active leg, cross-track error and the
// course deviation scaled the way panel GPS navigators scale it (Garmin
// GNS/GTN style: en route ±2 NM, terminal ±1 NM, narrowing to ±0.3 NM over
// the 2 NM before the final approach fix, then angular toward the runway),
// automatic sequencing with turn anticipation, and a vertical path from the
// final approach fix to the runway threshold (LPV/LNAV+V style).
//
// Geometry is in the local NED frame (flat Earth: altitude is -z above the
// frame origin's height, as the aircraft reports it). Good for a region of
// ~100 km; long routes need the host to rebase.
#pragma once

#include <string>
#include <vector>

#include "avd/math.hpp"

namespace avd {

struct RouteWaypoint {
    std::string ident;
    Vec3 position;          // NED m (z: -altitude of the point; used for the runway threshold)
    Real altitudeMsl = -1;  // published crossing altitude (m), < 0 none
    bool flyover = false;   // no turn anticipation (the runway threshold)
};

struct GpsIndication {
    bool valid = false;
    int active = -1;                  // index of the TO waypoint
    std::string mode;                 // "ENR", "TERM", "APR"
    Real desiredTrack = 0;            // rad true, of the active leg
    Real crossTrack = 0;              // m, + = aircraft right of course
    Real fullScale = 0;               // m, the CDI's full-scale deflection
    Real cdi = 0;                     // -1..1; + = needle right (course to the right: fly right)
    Real distanceToWaypoint = 0;      // m, horizontal
    Real bearingToWaypoint = 0;       // rad true
    Real distanceToDestination = 0;   // m along the rest of the route
    bool verticalValid = false;       // approach mode: inside 2 NM of the FAF, and after it
    Real verticalDeviation = 0;       // m, + = aircraft above the path
    Real vdi = 0;                     // -1..1; + = needle up (path above: fly up)
};

class GpsNavigator {
public:
    // `finalApproachFix` indexes the FAF (the vertical path runs from it to the
    // last waypoint, the threshold, crossed at `thresholdCrossing` m on a
    // `glidepath` rad path); -1: no vertical guidance.
    void setRoute(std::vector<RouteWaypoint> route, int finalApproachFix = -1, Real glidepath = 0.0524,
                  Real thresholdCrossing = 15.24);
    void clear();
    const std::vector<RouteWaypoint>& route() const { return route_; }
    int finalApproachFix() const { return faf_; }
    // Direct to waypoint `i` from the present position (next update).
    void directTo(int i);
    // Make waypoint `i` active on its own leg (from waypoint i - 1), e.g. when
    // joining the route already established on it.
    void activate(int i);
    // Sequence the active waypoint when its leg is flown, and evaluate.
    const GpsIndication& update(const Vec3& positionNed, Real altitudeMsl, Real groundSpeed);
    const GpsIndication& indication() const { return ind_; }

private:
    std::vector<RouteWaypoint> route_;
    int faf_ = -1;
    Real glidepath_ = 0.0524, tch_ = 15.24;
    int active_ = -1;
    bool directPending_ = false;
    Vec3 directFrom_;  // the FROM point of a direct-to leg
    bool direct_ = false;
    GpsIndication ind_;
};

}  // namespace avd
