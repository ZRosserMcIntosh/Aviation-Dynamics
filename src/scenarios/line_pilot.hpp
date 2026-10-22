// Aviation Dynamics — scenarios.
// SPDX-License-Identifier: MIT
//
// A pilot for traffic and scripted encounters: flies a straight line in
// space (a final approach course and glide path, or any straight path) on
// the truth state, holds an airspeed with the throttle, and goes around on
// cue: take-off thrust and a steady climb, the course held. The other
// aircraft in a scene (the jet that lines up on the wrong runway) and the
// aircraft a demonstration flies for the viewer use it.
//
// It is the scenario's TestPilot (attitude, track and coordination loops)
// with the line as its target: the altitude it holds is the line's at the
// aircraft's distance along it, the line's descent rate fed forward.
#pragma once

#include "avd/aircraft.hpp"
#include "test_pilot.hpp"

namespace avd::scenarios {

class LinePilot {
public:
    Vec3 origin;              // a point on the line, local NED (m)
    Real course = 0;          // rad true, the line's direction over the ground
    Real path = 0;            // rad, its flight path angle (- descending)
    Real targetIas = 0;       // m/s
    Real baseThrottle = 0;    // the trimmed throttle the airspeed loop works around
    Real pitchRef = 0;        // rad, the trimmed pitch attitude
    Real maxBank = 0.44;      // rad
    // Go around once this far along the line (m past `origin`), or at once
    // when goAround() is called.
    Real goAroundAt = 1e30;
    Real goAroundClimb = 0.087;  // rad, the go-around's flight path (5 deg)
    PilotInput controls;        // mixture, flaps etc. as configured

    PilotInput update(const Aircraft& ac, Real dt);
    void goAround() { goingAround_ = true; }
    bool goingAround() const { return goingAround_; }
    // Where it is against the line: m along it from `origin`, m right of it, m above it.
    Real along() const { return along_; }
    Real right() const { return right_; }
    Real above() const { return above_; }

private:
    TestPilot pilot_;
    Pid iasPid_{0.04, 0.01, 0.0, -0.4, 0.4};
    bool started_ = false, goingAround_ = false;
    Real along_ = 0, right_ = 0, above_ = 0, gaPitch_ = 0;
};

}  // namespace avd::scenarios
