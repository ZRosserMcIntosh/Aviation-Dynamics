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
    const Real want = -std::atan(right_ / (8.0 * v));
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
    // On a jet, the bank on the ailerons with the approach pilot's law
    // (ApproachPilot), its gain with the airspeed. The test pilot's roll loop
    // was tuned on the 172 and stays its pilot (the wake demonstration's 172
    // and its tests fly with it); its gain falls with dynamic pressure to a
    // floor that, at a jet's approach speed, is ~1 % of stick per degree of
    // bank error. The integral and the spiral mode then set the bank, it lagged
    // the line's demand, and the weave grew to hundreds of metres in two
    // minutes (737, CRJ700, Top G).
    if (ac.turbofanCount() > 0) {
        const Real ias = std::max(ac.instruments().readings().indicatedAirspeed, 25.0);
        const Real qs = clamp(sq(60.0 / ias), 0.4, 3.0);
        in.roll = clamp((1.6 * (pilot_.targetBank - t.roll) - 0.35 * t.omega.x) * std::min(qs, 1.5), -1.0, 1.0);
    }
    in.throttle = throttle;
    if (goingAround_) in.thrustMode = ThrustMode::Takeoff;
    return in;
}

}  // namespace avd::scenarios
