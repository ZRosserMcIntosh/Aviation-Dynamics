// Aviation Dynamics — scenarios.
// SPDX-License-Identifier: MIT
//
// A two-axis autopilot with an autothrottle, flown from the cockpit
// instruments alone (attitude, heading, altimeter, airspeed, slip ball, NAV 1
// and the GPS), as a flight director and its servos would be:
//
// - Lateral: wings level (ROL), heading select (HDG: the bug, at most 25 deg
//   of bank, turning the short way), the GPS route (NAV: the active leg's
//   desired track, intercepting at up to 30 deg), or the approach (APR).
// - Vertical: pitch hold (PIT, the attitude at engagement), altitude hold
//   (ALT), or vertical speed (VS) to the selected altitude, which it captures
//   (ALT*) and then holds; on an approach the glideslope or glidepath (GS).
// - Autothrottle (SPD): the selected indicated airspeed on the throttles;
//   off, the pilot's own levers stay in.
// - Pitch trim: run so the elevator servo is not left holding the airplane.
//
// APR, as a flight director's approach mode: armed, it keeps flying the
// lateral mode it had (the heading bug for an ILS intercept, the route for a
// GPS approach) and the vertical one (ALT, VS); the localizer (or the GPS's
// final course) is captured inside full scale, and then the glideslope (or
// the LPV-style glidepath), from below or level, once its needle is within a
// quarter of a dot. Coupled, it flies both down to the decision altitude,
// where the autopilot disconnects itself (lateral and vertical modes off,
// MINIMUMS): the pilot lands. It is not coupled below DA: it has no flare,
// alignment or rollout, and a CAT I autopilot is certified only to its
// minimum use height (the autoland, scenarios::ApproachPilot, is the CAT III
// path). The autothrottle stays in speed mode, as a real one does until the
// pilot disconnects it (or retards for the flare); the host shows MINIMUMS.
//
// Mode changes are noticed here (the previous modes are remembered):
// engaging the autothrottle takes its base from the lever then applied, a
// vertical mode engaged from off (or PIT selected) takes the attitude then,
// a lateral one the bank then, so nothing steps.
//
// The pilot's other controls (brakes, flaps, gear, mixture) pass through.
// No modelled servo or its torque limit; disengaging is the caller's
// business (a stick input, the button) except at DA.
#pragma once

#include "avd/aircraft.hpp"
#include "avd/gps.hpp"
#include "test_pilot.hpp"

namespace avd::scenarios {

class Autopilot {
public:
    enum class Lateral { Off, Roll, Heading, Nav, Approach };
    enum class Vertical { Off, Pitch, Altitude, VerticalSpeed, AltitudeCapture, GlidePath };
    enum class ApproachSource { Ils, Gps };
    // APR's progress: armed; the localizer (GPS final course) captured, the
    // glideslope (glidepath) armed; both captured; disconnected at DA.
    enum class ApproachState { Off, Armed, LateralCaptured, VerticalCaptured, Minimums };

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

    // APR: the approach flown, from the chart.
    ApproachSource approachSource = ApproachSource::Ils;
    Real decisionAltitude = 0;    // m on the altimeter (DA); reached coupled on the glidepath, it disconnects
    Real approachCourse = 0;      // rad magnetic, the ILS front course (the OBS / HSI course); GPS: the leg's own
    Real fieldElevation = 0;      // m, threshold elevation (the ILS needles into metres)
    Real runwayLength = 2000;     // m, threshold to the far end (the localizer's beam width)
    Real thresholdCrossing = 15.24;  // m (50 ft)

    // Engage (or change modes): the pitch held, the trimmed throttle, the
    // integrators are taken from where the airplane is now.
    void engage(const Aircraft& ac, const PilotInput& pilot);
    PilotInput update(const Aircraft& ac, const PilotInput& pilot, Real dt);

    // Diagnostics: the commanded bank and pitch attitudes (rad), the vertical speed wanted (m/s).
    Real bankCommand() const { return bankCmd_; }
    Real pitchCommand() const { return pitchCmd_; }
    Real vsCommand() const { return vsCmd_; }
    // APR's state and the deviations it flies, in dots (2 dots full scale;
    // + the needle right / up: fly right / up); the deviations in metres.
    ApproachState approachState() const { return apr_; }
    Real lateralDots() const { return latDots_; }
    Real verticalDots() const { return vertDots_; }
    Real lateralMetres() const { return y_; }
    Real verticalMetres() const { return z_; }
    // It disconnected itself at DA (cleared by engage()).
    bool disconnected() const { return apr_ == ApproachState::Minimums; }

private:
    void approachGuidance(const Aircraft& ac, Real dt);

    bool have_ = false;
    Real prevRoll_ = 0, prevPitch_ = 0, prevHdg_ = 0, altPrev_ = 0, iasPrev_ = 0;
    Real rollRate_ = 0, pitchRate_ = 0, hdgRate_ = 0, vs_ = 0, iasRate_ = 0, hdgErr_ = 0;
    Real bankCmd_ = 0, pitchCmd_ = 0, vsCmd_ = 0, pitchHold_ = 0, pitchRef_ = 0, throttleBase_ = 0;
    Real xtkPrev_ = 0, xtkRate_ = 0;
    // The modes last step: a change is a new engagement of that axis.
    Lateral lastLateral_ = Lateral::Off;
    Vertical lastVertical_ = Vertical::Off;
    bool lastAutothrottle_ = false;
    // APR.
    ApproachState apr_ = ApproachState::Off;
    bool latValid_ = false, vertValid_ = false, haveY_ = false;
    Real y_ = 0, z_ = 0, yPrev_ = 0, yRate_ = 0, latDots_ = 0, vertDots_ = 0, hdgLag_ = 0, drift_ = 0, finalCourse_ = 0;
    Pid pitchPid_{3.2, 1.6, 0.0, -0.5, 0.5};
    // The speed loop's integral may take the throttle anywhere from idle to
    // full from the base it engaged at (+-0.4 left a 1.0 base unable to slow
    // below 257 kt on a 250 kt selection), and finds it in about half a
    // minute (at 0.006 an idle base was still 4 kt slow after 40 s).
    Pid iasPid_{0.04, 0.012, 0.0, -1.0, 1.0};
};

}  // namespace avd::scenarios
