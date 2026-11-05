// Aviation Dynamics — scenarios.
// SPDX-License-Identifier: MIT
#include "autopilot.hpp"

#include <algorithm>
#include <cmath>

#include "approach_pilot.hpp"
#include "avd/units.hpp"

namespace avd::scenarios {

namespace {
constexpr Real kGlidepath = 0.0524;  // 3 deg
Real leverOf(const PilotInput& p) { return p.splitThrottles ? p.throttles[0] : p.throttle; }
}  // namespace

void Autopilot::engage(const Aircraft& ac, const PilotInput& pilot) {
    const CockpitState& r = ac.instruments().readings();
    have_ = false;
    pitchHold_ = pitchRef_ = pitchCmd_ = r.attitudePitch;
    bankCmd_ = r.attitudeRoll;
    vsCmd_ = 0;
    hdgErr_ = 0;
    throttleBase_ = leverOf(pilot);
    pitchPid_.reset();
    iasPid_.reset();
    // Everything as if newly selected: update() takes each axis from here.
    lastLateral_ = Lateral::Off;
    lastVertical_ = Vertical::Off;
    lastAutothrottle_ = false;
    apr_ = ApproachState::Off;
}

// The approach's deviations from NAV 1's needles or the GPS, in metres and dots.
void Autopilot::approachGuidance(const Aircraft& ac, Real dt) {
    const CockpitState& r = ac.instruments().readings();
    latValid_ = vertValid_ = false;
    if (approachSource == ApproachSource::Ils) {
        const NavIndication& n = r.nav1;
        const Real range = rangeOnGlidepath(r.indicatedAltitude - fieldElevation, thresholdCrossing);
        const BeamDeviation d = ilsDeviation(n, range, runwayLength);
        latValid_ = d.lateralValid;
        vertValid_ = d.verticalValid;
        if (latValid_) y_ = d.lateral, latDots_ = 2 * n.cdi;
        if (vertValid_) z_ = d.vertical, vertDots_ = 2 * n.glideslope;
        finalCourse_ = approachCourse;
    } else if (gps && gps->valid) {
        latValid_ = true;
        y_ = gps->crossTrack;
        latDots_ = 2 * gps->cdi;
        finalCourse_ = wrapPi(gps->desiredTrack - magneticVariation);
        if (gps->verticalValid) {
            vertValid_ = true;
            z_ = gps->verticalDeviation;
            vertDots_ = 2 * gps->vdi;
        }
    }
    if (!latValid_) latDots_ = 0;
    if (!vertValid_) vertDots_ = 0;
    // The lateral speed off the course, filtered (0.5 s).
    if (latValid_) {
        if (!haveY_) yPrev_ = y_, yRate_ = 0, haveY_ = true;
        if (dt > 0) yRate_ += ((y_ - yPrev_) / dt - yRate_) * std::min(dt / 0.5, 1.0);
        yPrev_ = y_;
    } else {
        haveY_ = false;
    }
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
    const Real tas = std::max(ac.telemetry().trueAirspeed, 25.0);

    // ---- Mode changes: each axis newly engaged starts from where the airplane is ----
    if (autothrottle && !lastAutothrottle_) {
        // SPD pressed: the base is the lever applied now (the pilot's), not the
        // one at the autopilot's engagement (a full-power climb's 1.0 held
        // 257 kt level on a 250 kt selection).
        throttleBase_ = leverOf(pilot);
        iasPid_.reset();
    }
    if (vertical != Vertical::Off && lastVertical_ == Vertical::Off) {
        pitchHold_ = pitchRef_ = pitchCmd_ = pitch;
        pitchPid_.reset();
    } else if (vertical == Vertical::Pitch && lastVertical_ != Vertical::Pitch) {
        pitchHold_ = pitch;  // PIT: the attitude at its engagement
    }
    if (lateral != Lateral::Off && lastLateral_ == Lateral::Off) {
        bankCmd_ = roll;
        hdgErr_ = 0;
        if (apr_ == ApproachState::Minimums) apr_ = ApproachState::Off;
    }
    if (lateral == Lateral::Approach && (lastLateral_ != Lateral::Approach || apr_ == ApproachState::Off)) {
        apr_ = ApproachState::Armed;
        haveY_ = false;
        drift_ = 0;
        hdgLag_ = wrapPi(hdg - approachCourse);
    }
    if (lateral != Lateral::Approach && apr_ != ApproachState::Minimums) apr_ = ApproachState::Off;
    if (vertical == Vertical::GlidePath && apr_ != ApproachState::VerticalCaptured) vertical = Vertical::Altitude;  // GS only by capture

    // ---- APR: arm, capture, minimums ----
    if (lateral == Lateral::Approach) {
        approachGuidance(ac, dt);
        // The localizer (GPS final course) captured inside full scale.
        if (apr_ == ApproachState::Armed && latValid_ && std::abs(latDots_) < 1.8) apr_ = ApproachState::LateralCaptured;
        // The glideslope (glidepath) armed behind it: captured within a
        // quarter of a dot (from below or level, the vertical mode holding
        // the airplane there; or as the airplane descends through it).
        if (apr_ == ApproachState::LateralCaptured && vertValid_ && std::abs(vertDots_) < 0.25) {
            apr_ = ApproachState::VerticalCaptured;
            vertical = Vertical::GlidePath;
        }
        // Decision altitude, coupled: the autopilot disconnects (the
        // autothrottle stays in SPD); the pilot lands.
        if (apr_ == ApproachState::VerticalCaptured && alt <= decisionAltitude) {
            apr_ = ApproachState::Minimums;
            lateral = Lateral::Off;
            vertical = Vertical::Off;
        }
    }

    // ---- Lateral ----
    if (lateral != Lateral::Off) {
        Real bank = 0;
        const bool aprCaptured = lateral == Lateral::Approach && apr_ != ApproachState::Armed && latValid_;
        const bool navLike = lateral == Lateral::Nav || (lateral == Lateral::Approach && !aprCaptured && approachSource == ApproachSource::Gps);
        if (lateral == Lateral::Heading || lateral == Lateral::Nav || lateral == Lateral::Approach) {
            Real want = headingBug;
            if (navLike && gps && gps->valid) {
                // The leg's track, and back to it from the side (a 20 s
                // exponential, at most 30 deg off the track).
                const Real course = wrapPi(gps->desiredTrack - magneticVariation);
                want = course - clamp((xtk / 20.0 + xtkRate_) / v, -0.52, 0.52);
            }
            if (aprCaptured) {
                // The approach: back to the centreline (an 8 s exponential, at
                // most 30 deg off the course), less the drift (the track off
                // the course less the heading, the heading lagged 2 s as the
                // track lags it, filtered 3 s): the crab into the wind comes
                // from the drift, as the ApproachPilot flies it.
                hdgLag_ += (wrapPi(hdg - finalCourse_) - hdgLag_) * std::min(dt / 2.0, 1.0);
                const Real drift = std::asin(clamp(yRate_ / tas, -0.5, 0.5)) - hdgLag_;
                drift_ += (clamp(drift, -0.5, 0.5) - drift_) * std::min(dt / 3.0, 1.0);
                want = finalCourse_ - clamp((y_ / 8.0 + yRate_) / tas, -0.52, 0.52) - drift_;
            }
            hdgErr_ += (wrapPi(want - hdg) - hdgErr_) * std::min(dt / 1.0, 1.0);
            // At most a rate-one turn (3 deg/s) or maxBank, whichever is less;
            // the heading rate damps the roll-out.
            const Real rateOne = std::atan(v * 0.0524 / 9.81);
            // NAV may bank to 30 deg (an FMS's LNAV limit): the GPS begins a
            // fly-by turn for a 25 deg turn, so the autopilot needs the margin
            // to catch up with its own lags (held at 25 deg on the AHRS's true
            // bank, a Top G overshot the next leg by 300 m; the old gyro, reading
            // 3-5 deg shallow, had been banking it 28-30).
            const Real limit = std::min(navLike ? std::max(maxBank, 0.52) : maxBank, std::max(rateOne, 0.26));
            bank = clamp(1.5 * hdgErr_ - 2.0 * std::min(v / 60.0, 1.2) * hdgRate_, -limit, limit);
        }
        bankCmd_ += clamp(bank - bankCmd_, -0.087 * dt, 0.087 * dt);  // 5 deg/s
        in.roll = clamp((1.6 * (bankCmd_ - roll) - 0.8 * rollRate_) * std::min(qs, 1.5), -1.0, 1.0);
        in.yaw = clamp(pilot.yaw + 0.4 * r.slipBall * qs, -1.0, 1.0);
    }

    // ---- Vertical ----
    if (vertical != Vertical::Off) {
        // Captured when the altitude law's own rate meets the selected one
        // (10 s of error at 0.1/s): no step in the commanded rate. Only with
        // the selected altitude ahead in the direction of the selected rate:
        // behind it (VS +500 with the altitude 1,000 ft below) it flies the
        // rate selected, away from it, as a real autopilot does (it went
        // straight to ALT* and dived at 1,000 fpm).
        if (vertical == Vertical::VerticalSpeed) {
            const Real err = selectedAltitude - alt;
            const bool ahead = (selectedVs > 0 && err > 0) || (selectedVs < 0 && err < 0);
            if ((ahead && std::abs(err) < 10 * std::abs(selectedVs)) || std::abs(err) < 3) vertical = Vertical::AltitudeCapture;
        }
        if (vertical == Vertical::AltitudeCapture && std::abs(selectedAltitude - alt) < 15 && std::abs(vs_) < 1) vertical = Vertical::Altitude;
        Real wantPitch = pitchHold_;
        if (vertical != Vertical::Pitch) {
            if (vertical == Vertical::VerticalSpeed) vsCmd_ = selectedVs;
            else if (vertical == Vertical::GlidePath)
                // The 3 deg path's sink at the true airspeed, corrected by the
                // deviation (a 10 s exponential), as the ApproachPilot flies it.
                vsCmd_ = clamp(-tas * std::sin(kGlidepath) - (vertValid_ ? z_ / 10.0 : 0.0), -8.0, 2.0);
            else vsCmd_ = clamp(0.1 * (selectedAltitude - alt), -std::max(std::abs(selectedVs), 5.0), std::max(std::abs(selectedVs), 5.0));
            // The pitch for the rate: a change of flight path is the rate
            // over the true airspeed (so the gains go as 1/V: fixed, they
            // were four times too stiff at a jet's 240 kt and swung the
            // airplane +-5,000 fpm); a slow integral finds the attitude (it
            // moves with speed and weight).
            const Real e = vsCmd_ - vs_;
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
    lastLateral_ = lateral;
    lastVertical_ = vertical;
    lastAutothrottle_ = autothrottle;
    return in;
}

}  // namespace avd::scenarios
