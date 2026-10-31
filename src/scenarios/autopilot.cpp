// Aviation Dynamics — scenarios.
// SPDX-License-Identifier: MIT
#include "autopilot.hpp"

#include <algorithm>
#include <cmath>

#include "avd/units.hpp"

namespace avd::scenarios {

void Autopilot::engage(const Aircraft& ac, const PilotInput& pilot) {
    const CockpitState& r = ac.instruments().readings();
    have_ = false;
    pitchHold_ = pitchRef_ = pitchCmd_ = r.attitudePitch;
    bankCmd_ = 0;
    vsCmd_ = 0;
    hdgErr_ = 0;
    throttleBase_ = pilot.splitThrottles ? pilot.throttles[0] : pilot.throttle;
    pitchPid_.reset();
    iasPid_.reset();
}

PilotInput Autopilot::update(const Aircraft& ac, const PilotInput& pilot, Real dt) {
    PilotInput in = pilot;
    const CockpitState& r = ac.instruments().readings();
    const Real pitch = r.attitudePitch, roll = r.attitudeRoll, hdg = r.headingIndicator;
    const Real alt = r.indicatedAltitude, ias = r.indicatedAirspeed;
    const Real xtk = gps && gps->valid ? gps->crossTrack : 0.0;
    if (!have_) {
        prevRoll_ = roll, prevPitch_ = pitch, prevHdg_ = hdg, altPrev_ = alt, iasPrev_ = ias, xtkPrev_ = xtk;
        rollRate_ = pitchRate_ = hdgRate_ = iasRate_ = xtkRate_ = 0;
        vs_ = r.verticalSpeed;
        have_ = true;
    }
    // Rates from the instruments, filtered: attitude 0.1 s, heading 0.3 s;
    // the vertical speed from the altimeter's rate (1 s: an air-data
    // computer's, not the VSI needle's lag); the speed trend 1 s.
    const Real k1 = std::min(dt / 0.1, 1.0);
    rollRate_ += ((roll - prevRoll_) / dt - rollRate_) * k1;
    pitchRate_ += ((pitch - prevPitch_) / dt - pitchRate_) * k1;
    hdgRate_ += (wrapPi(hdg - prevHdg_) / dt - hdgRate_) * std::min(dt / 0.3, 1.0);
    vs_ += ((alt - altPrev_) / dt - vs_) * std::min(dt / 1.0, 1.0);
    iasRate_ += ((ias - iasPrev_) / dt - iasRate_) * std::min(dt / 1.0, 1.0);
    xtkRate_ += ((xtk - xtkPrev_) / dt - xtkRate_) * std::min(dt / 2.0, 1.0);
    prevRoll_ = roll, prevPitch_ = pitch, prevHdg_ = hdg, altPrev_ = alt, iasPrev_ = ias, xtkPrev_ = xtk;
    // Control gains with dynamic pressure (down to a jet's cruise: 1/q all the way).
    const Real qs = clamp(sq(60.0 / std::max(ias, 25.0)), 0.1, 3.0);
    const Real v = std::max(ias, 25.0);

    // ---- Lateral ----
    if (lateral != Lateral::Off) {
        Real bank = 0;
        if (lateral == Lateral::Heading || lateral == Lateral::Nav) {
            Real want = headingBug;
            if (lateral == Lateral::Nav && gps && gps->valid) {
                // The leg's track, and back to it from the side (a 20 s
                // exponential, at most 30 deg off the track).
                const Real course = wrapPi(gps->desiredTrack - magneticVariation);
                want = course - clamp((xtk / 20.0 + xtkRate_) / v, -0.52, 0.52);
            }
            hdgErr_ += (wrapPi(want - hdg) - hdgErr_) * std::min(dt / 1.0, 1.0);
            // At most a rate-one turn (3 deg/s) or maxBank, whichever is less;
            // the heading rate damps the roll-out.
            const Real rateOne = std::atan(v * 0.0524 / 9.81);
            const Real limit = std::min(maxBank, std::max(rateOne, 0.26));
            bank = clamp(1.5 * hdgErr_ - 2.0 * std::min(v / 60.0, 1.2) * hdgRate_, -limit, limit);
        }
        bankCmd_ += clamp(bank - bankCmd_, -0.087 * dt, 0.087 * dt);  // 5 deg/s
        in.roll = clamp((1.6 * (bankCmd_ - roll) - 0.8 * rollRate_) * std::min(qs, 1.5), -1.0, 1.0);
        in.yaw = clamp(pilot.yaw + 0.4 * r.slipBall * qs, -1.0, 1.0);
    }

    // ---- Vertical ----
    if (vertical != Vertical::Off) {
        // Captured when the altitude law's own rate meets the selected one
        // (10 s of error at 0.1/s): no step in the commanded rate.
        if (vertical == Vertical::VerticalSpeed) {
            const Real err = selectedAltitude - alt;
            if ((selectedVs > 0 && err < 10 * selectedVs) || (selectedVs < 0 && err > 10 * selectedVs) || std::abs(err) < 3)
                vertical = Vertical::AltitudeCapture;
        }
        if (vertical == Vertical::AltitudeCapture && std::abs(selectedAltitude - alt) < 15 && std::abs(vs_) < 1) vertical = Vertical::Altitude;
        Real wantPitch = pitchHold_;
        if (vertical != Vertical::Pitch) {
            if (vertical == Vertical::VerticalSpeed) vsCmd_ = selectedVs;
            else vsCmd_ = clamp(0.1 * (selectedAltitude - alt), -std::max(std::abs(selectedVs), 5.0), std::max(std::abs(selectedVs), 5.0));
            // The pitch for the rate: a change of flight path is the rate
            // over the true airspeed (so the gains go as 1/V: fixed, they
            // were four times too stiff at a jet's 240 kt and swung the
            // airplane +-5,000 fpm); a slow integral finds the attitude (it
            // moves with speed and weight).
            const Real e = vsCmd_ - vs_;
            const Real tas = std::max(ac.telemetry().trueAirspeed, 25.0);
            pitchRef_ = clamp(pitchRef_ + 0.15 / tas * e * dt, -0.17, 0.26);
            wantPitch = pitchRef_ + 0.8 / tas * e;
        }
        wantPitch = clamp(wantPitch, -0.17, 0.26);  // -10 .. +15 deg
        pitchCmd_ += clamp(wantPitch - pitchCmd_, -0.035 * dt, 0.035 * dt);  // 2 deg/s
        in.pitch = clamp((pitchPid_.update(pitchCmd_ - pitch, dt) - 1.6 * pitchRate_) * qs, -1.0, 1.0);
        // The trim follows, so the servo is not left holding the elevator.
        in.trimRate = std::abs(in.pitch) > 0.02 ? clamp(2.0 * in.pitch, -1.0, 1.0) : 0.0;
    }

    // ---- Autothrottle ----
    if (autothrottle) {
        const Real thr = clamp(throttleBase_ + iasPid_.update(selectedIas - ias, dt) - 0.25 * iasRate_, 0.0, 1.0);
        in.throttle = thr;
        in.splitThrottles = false;
    }
    return in;
}

}  // namespace avd::scenarios
