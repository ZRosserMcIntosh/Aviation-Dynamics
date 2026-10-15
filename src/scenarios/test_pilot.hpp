// Aviation Dynamics — scenarios.
// SPDX-License-Identifier: MIT
//
// Scripted test pilot for automated flight-test scenarios. It moves the
// controls through PilotInput exactly as a human would, using only what a
// pilot can perceive (instrument readings plus attitude). It is a test
// harness, not the aircraft autopilot (that is a modelled aircraft system,
// Milestone 4).
#pragma once

#include "avd/aircraft.hpp"
#include "avd/ground.hpp"

namespace avd::scenarios {

struct Pid {
    Real kp = 0, ki = 0, kd = 0;
    Real iMin = -1, iMax = 1;
    Real integral = 0, prev = 0;
    bool first = true;
    Real update(Real error, Real dt) {
        integral = clamp(integral + error * ki * dt, iMin, iMax);
        const Real deriv = first || dt <= 0 ? 0 : (error - prev) / dt;
        first = false;
        prev = error;
        return kp * error + integral + kd * deriv;
    }
    void reset() { integral = 0; first = true; }
};

class TestPilot {
public:
    enum class Vertical { Manual, Pitch, Airspeed, Altitude, FlightPath };
    enum class Lateral { Manual, WingsLevel, Heading, Bank, Track };

    Vertical vertical = Vertical::Manual;
    Lateral lateral = Lateral::WingsLevel;
    Real targetPitch = 0;        // rad
    Real targetIas = 0;          // m/s
    Real targetAltitude = 0;     // m MSL
    Real targetHeading = 0;      // rad true
    Real targetBank = 0;         // rad
    Real targetFlightPath = 0;   // rad (+ climb)
    Real verticalSpeedFeedForward = 0;  // m/s added to the Altitude-mode VS target (moving targets)
    Real maxBank = 0.44;         // rad (25 deg)
    bool coordinate = true;      // rudder for zero sideslip in the air
    // Track mode: follow a line through `trackOrigin` with heading `targetHeading`.
    Vec3 trackOrigin;
    PilotInput controls;         // throttle, mixture, brakes, flaps set by the scenario

    PilotInput update(const Aircraft& ac, Real dt);
    void resetIntegrators();

private:
    Pid pitchPid_{2.5, 1.5, 0.35, -0.6, 0.6};
    Pid iasPid_{0.020, 0.004, 0.0, -0.15, 0.15};
    Pid altPid_{0.010, 0.0005, 0.0, -0.05, 0.05};
    Pid rollPid_{1.6, 0.25, 0.25, -0.3, 0.3};
    Pid yawPid_{3.0, 3.0, 0.0, -0.6, 0.6};
};

}  // namespace avd::scenarios
