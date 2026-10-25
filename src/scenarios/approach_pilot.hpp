// Aviation Dynamics — scenarios.
// SPDX-License-Identifier: MIT
//
// A blind approach and landing flown on instruments alone, as a pilot (or a
// coupled approach and autoland) in fog would: the localizer and glideslope
// needles on NAV 1, or the GPS course and vertical path; the attitude,
// heading, airspeed and vertical speed indicators; the radio altimeter for
// the flare (or, without one, the barometric altimeter against the field
// elevation). Nothing outside the windscreen and no truth state.
//
// Needle deflections are turned into metres as an autopilot does, from the
// geometry the approach chart gives (the beam narrows toward the runway; the
// distance is estimated from the height on the 3 deg path), so the gains hold
// all the way down. The airspeed is held by the throttle.
#pragma once

#include "avd/aircraft.hpp"
#include "avd/atmosphere.hpp"
#include "avd/gps.hpp"
#include "test_pilot.hpp"

namespace avd::scenarios {

class ApproachPilot {
public:
    enum class Phase { Intercept, Final, Flare, Rollout, Stopped };
    enum class Source { Ils, Gps };

    // Set from the approach chart and the aircraft's book.
    Source source = Source::Ils;
    Real courseMagnetic = 0;        // rad, final approach course (the OBS / DTK)
    Real fieldElevation = 0;        // m, threshold elevation (the altimeter is set to QNH)
    Real runwayLength = 2000;       // m, for the localizer's beam width
    Real thresholdCrossing = 15.24; // m (50 ft)
    Real approachIas = 0;           // m/s target
    Real interceptAltitude = 0;     // m MSL held until the glidepath is captured
    bool radioAltimeter = true;     // flare on the radio altimeter (else barometric height)
    Real flareHeight = 0;           // m above the runway to start the flare (0: by type)
    Real idleHeight = 0;            // m to close the throttle (0: by type)
    Real raZero = 0;                // the height reading on the ground (radio altimeter or baro): touchdown
    Real pitchRef = 0;              // rad, pitch attitude the vertical-speed loop starts from (the trim)
    Real baseThrottle = 0;          // the trimmed throttle the airspeed loop works around
    PilotInput controls;            // flaps, gear, mixture etc. as configured

    // The GPS (Source::Gps): its indication, updated by the caller.
    const GpsIndication* gps = nullptr;

    PilotInput update(const Aircraft& ac, Real dt);
    Phase phase() const { return phase_; }
    // Diagnostics: the estimated deviations the pilot is flying (m; + right of course / above the path).
    Real lateralMetres() const { return y_; }
    Real verticalMetres() const { return z_; }

private:
    Phase phase_ = Phase::Intercept;
    Real y_ = 0, z_ = 0, yRate_ = 0, hdgRate_ = 0, prevHdg_ = 0, hdgLag_ = 0, drift_ = 0, pastThreshold_ = 0, prevRoll_ = 0, prevPitch_ = 0, prevY_ = 0, flareVs_ = 0;
    bool havePrev_ = false, havePitchTarget_ = false, haveRa_ = false;
    Real raPrev_ = 0, vsRa_ = 0;
    Real retardFrom_ = 0, flareMax_ = 0, vsFast_ = 0, lastThrottle_ = 1, flareInt_ = 0, flarePitch_ = 0, hdgErr_ = 0, rollRate_ = 0, pitchRate_ = 0, flareH_ = 12, pitchTarget_ = 0, altPrev_ = 0, vsFilt_ = 0, iasPrev_ = 0, iasRate_ = 0;
    bool haveAlt_ = false, haveIas_ = false, retarded_ = false;
    Pid pitchPid_{3.2, 1.6, 0.0, -0.5, 0.5};
    Pid vsPid_{0.015, 0.003, 0.0, -0.08, 0.08};
    Pid iasPid_{0.05, 0.006, 0.0, -0.4, 0.6};
};

}  // namespace avd::scenarios
