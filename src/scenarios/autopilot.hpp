// Aviation Dynamics — scenarios.
// SPDX-License-Identifier: MIT
//
// A two-axis autopilot with an autothrottle, flown from the cockpit
// instruments alone (attitude, heading, altimeter, airspeed, slip ball and
// the GPS), as a flight director and its servos would be:
//
// - Lateral: wings level (ROL), heading select (HDG: the bug, at most 25 deg
//   of bank, turning the short way), or the GPS route (NAV: the active leg's
//   desired track, intercepting at up to 30 deg).
// - Vertical: pitch hold (PIT, the attitude at engagement), altitude hold
//   (ALT), or vertical speed (VS) to the selected altitude, which it captures
//   (ALT*) and then holds.
// - Autothrottle (SPD): the selected indicated airspeed on the throttles;
//   off, the pilot's own levers stay in.
// - Pitch trim: run so the elevator servo is not left holding the airplane.
//
// The pilot's other controls (brakes, flaps, gear, mixture) pass through.
// No modelled servo or its torque limit; disengaging is the caller's
// business (a stick input, the button).
#pragma once

#include "avd/aircraft.hpp"
#include "avd/gps.hpp"
#include "test_pilot.hpp"

namespace avd::scenarios {

class Autopilot {
public:
    enum class Lateral { Off, Roll, Heading, Nav };
    enum class Vertical { Off, Pitch, Altitude, VerticalSpeed, AltitudeCapture };

    Lateral lateral = Lateral::Off;
    Vertical vertical = Vertical::Off;
    bool autothrottle = false;
    Real headingBug = 0;          // rad, on the heading indicator (magnetic)
    Real selectedAltitude = 0;    // m, indicated (the altimeter's setting applied)
    Real selectedVs = 0;          // m/s (+ up)
    Real selectedIas = 0;         // m/s
    Real maxBank = 0.44;          // rad (25 deg)
    Real magneticVariation = 0;   // rad (true = magnetic + variation): NAV turns the GPS track into a heading
    const GpsIndication* gps = nullptr;

    // Engage (or change modes): the pitch held, the trimmed throttle, the
    // integrators are taken from where the airplane is now.
    void engage(const Aircraft& ac, const PilotInput& pilot);
    PilotInput update(const Aircraft& ac, const PilotInput& pilot, Real dt);

    // Diagnostics: the commanded bank and pitch attitudes (rad), the vertical speed wanted (m/s).
    Real bankCommand() const { return bankCmd_; }
    Real pitchCommand() const { return pitchCmd_; }
    Real vsCommand() const { return vsCmd_; }

private:
    bool have_ = false;
    Real prevRoll_ = 0, prevPitch_ = 0, prevHdg_ = 0, altPrev_ = 0, iasPrev_ = 0;
    Real rollRate_ = 0, pitchRate_ = 0, hdgRate_ = 0, vs_ = 0, iasRate_ = 0, hdgErr_ = 0;
    Real bankCmd_ = 0, pitchCmd_ = 0, vsCmd_ = 0, pitchHold_ = 0, pitchRef_ = 0, throttleBase_ = 0;
    Real xtkPrev_ = 0, xtkRate_ = 0;
    Pid pitchPid_{3.2, 1.6, 0.0, -0.5, 0.5};
    Pid iasPid_{0.04, 0.006, 0.0, -0.4, 0.4};
};

}  // namespace avd::scenarios
