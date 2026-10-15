// Aviation Dynamics — scenarios.
// SPDX-License-Identifier: MIT
#include "test_pilot.hpp"

namespace avd::scenarios {

void TestPilot::resetIntegrators() {
    pitchPid_.reset();
    iasPid_.reset();
    altPid_.reset();
    rollPid_.reset();
    yawPid_.reset();
}

PilotInput TestPilot::update(const Aircraft& ac, Real dt) {
    PilotInput in = controls;
    const AircraftTelemetry& t = ac.telemetry();
    const CockpitState& r = ac.instruments().readings();

    // ---- Vertical: outer loops produce a pitch-attitude target ----
    Real pitchTarget = targetPitch;
    switch (vertical) {
        case Vertical::Airspeed:
            // Too slow -> lower the nose.
            pitchTarget = targetPitch - iasPid_.update(targetIas - r.indicatedAirspeed, dt);
            break;
        case Vertical::Altitude: {
            const Real vsTarget = verticalSpeedFeedForward + clamp((targetAltitude - t.altitudeMsl) * 0.15, -4.0, 4.0);
            pitchTarget = targetPitch + altPid_.update(vsTarget - t.verticalSpeed, dt) +
                          0.012 * (vsTarget - t.verticalSpeed);
            break;
        }
        case Vertical::FlightPath: {
            const Real gamma = std::atan2(t.verticalSpeed, std::max(t.groundSpeed, 1.0));
            pitchTarget = t.alpha + targetFlightPath + altPid_.update(targetFlightPath - gamma, dt) * 4;
            break;
        }
        default: break;
    }
    if (vertical != Vertical::Manual) {
        pitchTarget = clamp(pitchTarget, -0.35, 0.35);
        // Pitch attitude loop with rate damping (scaled with dynamic pressure).
        const Real qs = clamp(800.0 / std::max(t.dynamicPressure, 150.0), 0.4, 3.0);
        in.pitch = clamp((pitchPid_.update(pitchTarget - t.pitch, dt) - 0.6 * t.omega.y) * qs, -1.0, 1.0);
    }

    // ---- Lateral ----
    Real bankTarget = 0;
    bool lateralActive = true;
    switch (lateral) {
        case Lateral::Manual: lateralActive = false; break;
        case Lateral::WingsLevel: bankTarget = 0; break;
        case Lateral::Bank: bankTarget = targetBank; break;
        case Lateral::Heading:
            bankTarget = clamp(2.0 * wrapPi(targetHeading - t.heading), -maxBank, maxBank);
            break;
        case Lateral::Track: {
            const Vec3 d = t.positionNed - trackOrigin;
            const Real cross = -d.x * std::sin(targetHeading) + d.y * std::cos(targetHeading);  // + right of track
            const Real courseTrack = std::atan2(t.velocityNed.y, t.velocityNed.x);
            const Real intercept = clamp(-cross * 0.01, -0.5, 0.5);
            const Real wantCourse = targetHeading + intercept;
            bankTarget = clamp(2.5 * wrapPi(wantCourse - courseTrack), -maxBank, maxBank);
            break;
        }
    }
    if (lateralActive && !t.onGround) {
        const Real qs = clamp(800.0 / std::max(t.dynamicPressure, 150.0), 0.4, 3.0);
        in.roll = clamp((rollPid_.update(bankTarget - t.roll, dt) - 0.3 * t.omega.x) * qs, -1.0, 1.0);
    }
    if (coordinate && !t.onGround && t.trueAirspeed > 15) {
        const Real qs = clamp(800.0 / std::max(t.dynamicPressure, 150.0), 0.4, 3.0);
        in.yaw = clamp(yawPid_.update(t.beta, dt) * qs + controls.yaw, -1.0, 1.0);
    }
    return in;
}

}  // namespace avd::scenarios
