// Aviation Dynamics — scenarios.
// SPDX-License-Identifier: MIT
#include "line_pilot.hpp"

#include <cmath>

namespace avd::scenarios {

PilotInput LinePilot::update(const Aircraft& ac, Real dt) {
    const AircraftTelemetry& t = ac.telemetry();
    if (!started_) {
        pilot_.lateral = TestPilot::Lateral::Bank;
        gaPitch_ = 0;
        started_ = true;
    }
    // Where it is against the line.
    const Vec3 d = t.positionNed - origin;
    const Real c = std::cos(course), s = std::sin(course);
    along_ = d.x * c + d.y * s;
    right_ = -d.x * s + d.y * c;
    // Height above the line in the local frame (whatever altitude its z = 0 is).
    above_ = (origin.z - along_ * std::tan(path)) - t.positionNed.z;
    if (along_ >= goAroundAt) goingAround_ = true;

    pilot_.controls = controls;
    // Lateral: back to the line over a lookahead of 6 s of flight, the
    // track's turn a first-order lag of about 7 s (a light single and an
    // airliner alike: the bank this asks for is v / g times the track's rate;
    // quicker, the roll loop's own lag set the 737 weaving).
    const Real v = std::max(std::hypot(t.velocityNed.x, t.velocityNed.y), 20.0);
    const Real trackErr = wrapPi(std::atan2(t.velocityNed.y, t.velocityNed.x) - course);
    const Real want = -std::atan(right_ / (6.0 * v));
    pilot_.targetBank = clamp(std::atan(v / 9.80665 * 0.15 * wrapPi(want - trackErr)), -maxBank, maxBank);
    Real throttle;
    if (!goingAround_) {
        // The line's altitude a second ahead, its descent rate fed forward;
        // the trimmed attitude is where the altitude loop starts from.
        pilot_.vertical = TestPilot::Vertical::Altitude;
        pilot_.targetAltitude = t.altitudeMsl - above_ + t.groundSpeed * std::tan(path);
        pilot_.verticalSpeedFeedForward = t.groundSpeed * std::tan(path);
        pilot_.targetPitch = pitchRef;
        throttle = clamp(baseThrottle + iasPid_.update(targetIas - ac.instruments().readings().indicatedAirspeed, dt), 0.0, 1.0);
        gaPitch_ = std::atan2(t.verticalSpeed, std::max(t.groundSpeed, 1.0));
    } else {
        // Go around: take-off thrust and a climb, its angle raised at 3 deg/s to
        // goAroundClimb (a flight path, not an attitude: holding the pitch or
        // the speed with it set the phugoid going), on the course.
        gaPitch_ = std::min(goAroundClimb, gaPitch_ + 0.052 * dt);
        pilot_.vertical = TestPilot::Vertical::FlightPath;
        pilot_.targetFlightPath = gaPitch_;
        throttle = 1;
    }
    PilotInput in = pilot_.update(ac, dt);
    in.throttle = throttle;
    if (goingAround_) in.thrustMode = ThrustMode::Takeoff;
    return in;
}

}  // namespace avd::scenarios
