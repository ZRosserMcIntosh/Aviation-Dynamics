// Aviation Dynamics — scenarios.
// SPDX-License-Identifier: MIT
#include "approach_pilot.hpp"

#include <cmath>

namespace avd::scenarios {

namespace {
constexpr Real kGlidepath = 0.0524;      // 3 deg
constexpr Real kGsFullScale = 0.0122;    // 0.7 deg
}  // namespace

BeamDeviation ilsDeviation(const NavIndication& n, Real range, Real runwayLength) {
    BeamDeviation d;
    if (n.valid && n.localizer) {
        d.lateralValid = true;
        d.lateral = -n.cdi * 106.7 * std::max(range + runwayLength + 300, 300.0) / (runwayLength + 300);
    }
    if (n.valid && n.glideslopeValid) {
        d.verticalValid = true;
        d.vertical = -n.glideslope * std::tan(kGsFullScale) * std::max(range + 300, 150.0);
    }
    return d;
}

Real rangeOnGlidepath(Real heightAboveThreshold, Real thresholdCrossing) {
    return std::max((heightAboveThreshold - thresholdCrossing) / std::tan(kGlidepath), 0.0);
}

PilotInput ApproachPilot::update(const Aircraft& ac, Real dt) {
    PilotInput in = controls;
    const CockpitState& r = ac.instruments().readings();
    const AircraftTelemetry& t = ac.telemetry();  // used only for the radio altimeter and weight on wheels
    const Real ias = r.indicatedAirspeed;
    const Real baroHeight = r.indicatedAltitude - fieldElevation;
    // Height of the wheels above the runway: the radio altimeter reads zero
    // at touchdown (raZero is its reading on the ground).
    const Real height = radioAltimeter ? t.heightAgl - raZero : baroHeight - raZero;
    const Real pitch = r.attitudePitch, roll = r.attitudeRoll;
    // Attitude rates from the attitude indicator, filtered (0.1 s).
    if (havePrev_) {
        const Real k = std::min(dt / 0.1, 1.0);
        rollRate_ += ((roll - prevRoll_) / dt - rollRate_) * k;
        pitchRate_ += ((pitch - prevPitch_) / dt - pitchRate_) * k;
    }
    const Real rollRate = rollRate_, pitchRate = pitchRate_;
    prevRoll_ = roll;
    prevPitch_ = pitch;
    const Real qs = clamp(sq(60.0 / std::max(ias, 25.0)), 0.4, 3.0);  // control gains with dynamic pressure
    // Vertical speed from the altimeter's rate of change, filtered (1 s), as
    // an air-data computer gives it: the VSI needle's lag would set the loop
    // oscillating.
    if (!haveAlt_) { altPrev_ = r.indicatedAltitude; haveAlt_ = true; }
    vsFilt_ += ((r.indicatedAltitude - altPrev_) / dt - vsFilt_) * std::min(dt / 1.0, 1.0);
    vsFast_ += ((r.indicatedAltitude - altPrev_) / dt - vsFast_) * std::min(dt / 0.3, 1.0);
    altPrev_ = r.indicatedAltitude;
    // The radio altimeter's rate of change, filtered (0.3 s), as an autoland's
    // flare computer takes it: the barometric one moves with the static
    // source's error as the flare raises the angle of attack, and read the
    // CRJ sinking 25 % faster than it was, which pulled it into a balloon.
    if (!haveRa_) { raPrev_ = t.heightAgl; haveRa_ = true; }
    vsRa_ += ((t.heightAgl - raPrev_) / dt - vsRa_) * std::min(dt / 0.3, 1.0);
    raPrev_ = t.heightAgl;
    // The flare works on the quicker estimate (0.3 s): the 1 s lag overshot it.
    const Real vsNow = phase_ == Phase::Flare ? (radioAltimeter ? vsRa_ : vsFast_) : vsFilt_;

    // ---- Deviations in metres (+ right of course, + above the path) ----
    bool latValid = false, vertValid = false;
    // Distance to the threshold from the height on the path; past it (below
    // the crossing height) the distance flown since, at the airspeed.
    if (phase_ != Phase::Intercept && baroHeight < thresholdCrossing) pastThreshold_ += ias * dt;
    const Real range = pastThreshold_ > 0 ? -pastThreshold_ : rangeOnGlidepath(baroHeight, thresholdCrossing);
    if (source == Source::Ils) {
        const BeamDeviation d = ilsDeviation(r.nav1, range, runwayLength);
        latValid = d.lateralValid;
        vertValid = d.verticalValid;
        if (latValid) y_ = d.lateral;
        if (vertValid) z_ = d.vertical;
    } else if (gps && gps->valid) {
        latValid = true;
        y_ = gps->crossTrack;
        if (gps->verticalValid) {
            vertValid = true;
            z_ = gps->verticalDeviation;
        }
    }
    // The lateral speed off the course and the heading's rate of change, filtered (0.5 s).
    if (havePrev_) {
        const Real k = std::min(dt / 0.5, 1.0);
        yRate_ += ((y_ - prevY_) / dt - yRate_) * k;
        hdgRate_ += (wrapPi(r.headingIndicator - prevHdg_) / dt - hdgRate_) * k;
    }
    const Real yRate = yRate_;
    prevY_ = y_;
    prevHdg_ = r.headingIndicator;
    havePrev_ = true;
    const Real tas = ias * std::sqrt(isa::kSeaLevelDensity / StandardAtmosphere().at(r.indicatedAltitude).density);
    const Real v = std::max(tas, 30.0);
    // The drift (track less heading): the track off the course from the
    // lateral speed, less the heading off the course; filtered (3 s). The
    // heading to fly is the track wanted less the drift: the crab into the
    // wind comes from it, not from an integral of the deviation (which had to
    // wind up to a 10 deg crab and lagged it 30 m off the course).
    // The heading is taken lagged (2 s) as the track lags it in a turn: unlagged,
    // a swing in heading read as drift and fed itself.
    hdgLag_ += (wrapPi(r.headingIndicator - courseMagnetic) - hdgLag_) * std::min(dt / 2.0, 1.0);
    if (latValid) {
        const Real drift = std::asin(clamp(yRate / v, -0.5, 0.5)) - hdgLag_;
        drift_ += (clamp(drift, -0.5, 0.5) - drift_) * std::min(dt / 3.0, 1.0);
    }

    // ---- Phase ----
    // The flare starts about 5 s from touchdown at the present sink rate (a
    // heavy jet sinks faster on its approach and needs the room), at least
    // 40 ft on the radio altimeter (15 ft on the barometric one).
    const Real flareAt = flareHeight > 0 ? flareHeight : std::max(radioAltimeter ? 12.0 : 5.0, -5.0 * vsNow);
    // The retard: at 10 ft on the radio altimeter (a 737's autoland retards
    // at 27 ft and is at idle at touchdown; begun at 20 ft, the speed it bled
    // off in the last metres sank the jets in at 280-430 fpm).
    const Real idleAt = idleHeight > 0 ? idleHeight : 3.0;
    if (phase_ == Phase::Intercept && vertValid && z_ > -8 && std::abs(y_) < 150) phase_ = Phase::Final;
    if (phase_ == Phase::Final && height < flareAt) {
        phase_ = Phase::Flare;
        flareVs_ = std::min(vsNow, -0.5);
        flareH_ = flareAt;
        flarePitch_ = flareMax_ = pitch;
    }
    if ((phase_ == Phase::Flare || phase_ == Phase::Final) && t.onGround) phase_ = Phase::Rollout;
    if (phase_ == Phase::Rollout && t.groundSpeed < 2) phase_ = Phase::Stopped;

    if (phase_ == Phase::Rollout || phase_ == Phase::Stopped) {
        // Nose down, idle, brakes; steer the centreline on the localizer (or GPS).
        // The nose is flown down from the touchdown attitude at 2 deg/s, back
        // pressure against its rate: let fall (the mains' spin-up and the
        // ground spoilers pitch it down) it struck at 20 deg/s, 3.9 g at a
        // jet's cockpit.
        in.throttle = 0;
        if (!derotating_) {
            derotPitch_ = pitch;
            derotating_ = true;
        }
        derotPitch_ = std::max(derotPitch_ - 0.035 * dt, -0.02);
        in.pitch = clamp(4.0 * (derotPitch_ - pitch) - 3.0 * pitchRate, -0.4, 1.0);
        in.roll = clamp(-1.5 * roll, -0.5, 0.5);
        // Steer a heading back to the centreline (at most 4 deg off the course), the heading rate damping it.
        const Real hc = courseMagnetic - (latValid ? clamp(0.008 * y_ + 0.04 * yRate, -0.07, 0.07) : 0.0);
        in.yaw = clamp(3.0 * wrapPi(hc - r.headingIndicator) - 1.5 * hdgRate_, -1.0, 1.0);
        const Real brake = phase_ == Phase::Stopped ? 1.0 : (ias < 50 ? 0.7 : 0.4);
        in.brakeLeft = in.brakeRight = brake;
        return in;
    }

    // ---- Lateral: steer to the course, intercepting the centreline ----
    // The track wanted: back to the centreline, damped by the lateral speed (at most 30 deg off the course).
    Real heading = courseMagnetic;
    if (latValid) heading = courseMagnetic - clamp((y_ / 8.0 + yRate) / v, -0.52, 0.52) - drift_;
    const Real decrabAt = radioAltimeter ? 3.0 : 2.0;
    const Real maxBank = height < 30 ? 0.08 : 0.35;
    // The heading error is filtered (1.5 s): the Dutch roll's yawing would
    // otherwise come back through the bank command and pump it (it did, on
    // the 172).
    hdgErr_ += (wrapPi(heading - r.headingIndicator) - hdgErr_) * std::min(dt / 1.5, 1.0);
    // The heading rate damps it: the lateral-speed term in the track wanted
    // feeds the heading back on itself (the lateral speed is the heading off
    // the track), stiffening this loop into a 25 s weave in any wind.
    Real bank = clamp(1.5 * hdgErr_ - 2.0 * std::min(v / 60.0, 1.2) * hdgRate_, -maxBank, maxBank);
    in.yaw = clamp(0.4 * r.slipBall * qs, -0.3, 0.3);  // step on the ball, gently (a stronger hand set a light single yawing)
    // Close to the runway the rudder holds the heading too (bank is limited
    // there, and a gust's yaw otherwise turned the airplane off its track).
    if (height < 30 && phase_ != Phase::Intercept)
        in.yaw = clamp((2.0 * wrapPi(heading - r.headingIndicator) - 1.5 * hdgRate_) * qs, -0.5, 0.5);
    if (phase_ == Phase::Flare && height < decrabAt) {
        // The decrab, in the last seconds (below 10 ft; 6 ft on the barometric
        // altimeter): the rudder lines the nose up with
        // the runway and a wing down into the wind holds the track (a
        // sideslip), so the wheels meet the runway rolling along it. (Begun at
        // the flare, the drift it let in carried a jet 30 m off the centreline.)
        in.yaw = clamp((3.0 * wrapPi(courseMagnetic - r.headingIndicator) - 1.5 * hdgRate_) * qs, -1.0, 1.0);
        bank = latValid ? clamp(-0.1 * (yRate + 0.25 * y_), -0.1, 0.1) : 0.0;
    }
    in.roll = clamp((1.6 * (bank - roll) - 0.35 * rollRate) * std::min(qs, 1.5), -1.0, 1.0);

    // ---- Vertical: the glidepath (or the intercept altitude), then the flare ----
    Real vsTarget;
    if (phase_ == Phase::Intercept) {
        vsTarget = clamp((interceptAltitude - r.indicatedAltitude) * 0.1, -5.0, 5.0);
    } else if (phase_ == Phase::Final) {
        vsTarget = clamp(-tas * std::sin(kGlidepath) - (vertValid ? z_ / 10.0 : 0.0), -8.0, 2.0);
    } else {
        // Flare (the autoland law): the sink rate in proportion to the height
        // (time constant set so it matches the sink at flare entry) plus a
        // small touchdown sink, so the path meets the runway gently instead of
        // levelling off above it.
        const Real touchdown = radioAltimeter ? 0.4 : 0.3;
        const Real tau = flareH_ / std::max(-flareVs_ - touchdown, 0.3);
        vsTarget = -(std::max(height, 0.0) / tau + touchdown);
    }
    // The pitch target moves at most 2 deg/s (3 in the flare): a step in it
    // (the glidepath capture) would otherwise kick a stiff airplane into an
    // oscillation.
    // In the flare a firmer law of its own, from the attitude at flare entry
    // (the glidepath loop is deliberately soft and would arrest the sink too late):
    // 0.025 rad per m/s of sink-rate error plus an integral that holds the
    // attitude once the sink matches (proportional alone let the nose back
    // down and dropped the airplane the last few metres), at most 7 deg above
    // the entry attitude.
    const Real flareIntPrev = flareInt_;
    if (phase_ == Phase::Flare) flareInt_ = clamp(flareInt_ + 0.015 * (vsTarget - vsNow) * dt, -0.05, 0.12);
    // Never more than 1 deg below the highest flare attitude yet: lowering the
    // nose in the flare is how a pilot (and this one did) drops it in. Once
    // the sink is arrested (slower than wanted) 2.3 deg below: held at 1, the
    // floor levelled a CRJ off at 20 ft and floated it 800 m.
    const bool arrested = vsNow > vsTarget + 0.3;
    if (phase_ == Phase::Flare) flareMax_ = std::max(flareMax_, pitchTarget_);
    Real floor = arrested ? std::max(flarePitch_ - 0.035, flareMax_ - 0.04) : std::max(flarePitch_, flareMax_ - 0.017);
    // Below 2 m (the radio altimeter's flare) the nose is never lowered at
    // all: there the wing loses lift as the ground comes up (see below) and
    // a nose let down 2 deg after a level-off at 6 ft dropped a 737 in.
    if (radioAltimeter && arrested && height < 2.0) floor = std::max(floor, flareMax_);
    // The radio altimeter's law is firmer, and firmer still in the last 3 m
    // (up to 11 times at the runway): the wing's lift falls about 10 % at a
    // fixed attitude from 10 m down to 2 m of wheel height in this model's
    // ground effect, the sink grows there whatever the attitude was, and only
    // a quick answer to it in the last seconds holds the touchdown under
    // 200 fpm. (Firmer all the way down, the flare arrested high in the
    // city's turbulence, floated and dropped a CRJ in at 700 fpm.)
    const Real flareK = radioAltimeter ? 0.03 * (1 + 10 * clamp((3.0 - height) / 3.0, 0.0, 1.0)) : 0.025;
    const Real rawFlare = flarePitch_ + flareK * (vsTarget - vsNow) + flareInt_;
    Real want = phase_ == Phase::Flare
                          ? clamp(rawFlare, floor, flarePitch_ + 0.12)
                          : clamp(pitchRef + vsPid_.update(vsTarget - vsNow, dt), -0.2, 0.25);
    // The stall warning (the shaker, the horn) caps the nose where it is:
    // nothing more is asked of the wing (in ground effect its stalling angle
    // falls; a Top G's flare reached its pusher at 7 deg of alpha).
    const bool shaker = phase_ == Phase::Flare && radioAltimeter && r.stallHorn;
    if (shaker) want = std::min(want, pitch - 0.005);
    // The integral is kept from winding against the limits (radio altimeter
    // law): held while the floor, the ceiling or the shaker holds the nose;
    // and while the floor holds it, set so the law stands at the floor, so
    // the first sign of a sink raises the nose from there (wound down in a
    // float, it left the attitude on the floor while a CRJ sank in at 500 fpm).
    if (phase_ == Phase::Flare && radioAltimeter) {
        const Real e = vsTarget - vsNow;
        if ((rawFlare <= floor && e < 0) || ((rawFlare >= flarePitch_ + 0.12 || shaker) && e > 0)) flareInt_ = flareIntPrev;
        if (rawFlare < floor) flareInt_ = std::min(flareInt_ + (floor - rawFlare), 0.15);
    }
    if (!havePitchTarget_) pitchTarget_ = pitch, havePitchTarget_ = true;
    const Real slew = (phase_ == Phase::Flare ? 0.052 : 0.035) * dt;
    pitchTarget_ += clamp(want - pitchTarget_, -slew, slew);
    // Three times the authority in the flare: close to the runway the tail
    // loses downwash (ground effect) and the nose wants to drop just as the
    // flare needs it held (twice was too soft for the 737 in a gust).
    const Real flareGain = phase_ == Phase::Flare ? 3.0 : 1.0;
    in.pitch = clamp(flareGain * (pitchPid_.update(pitchTarget_ - pitch, dt) - 1.6 * pitchRate) * qs, -1.0, 1.0);

    // ---- Airspeed on the throttle; idle in the flare ----
    // The speed trend (filtered 1 s) damps it, as an autothrottle's
    // acceleration term does: with the engines' spool lag the speed loop alone
    // swung +-6 kt, and at a fixed attitude every speed swing is a glidepath swing.
    if (!haveIas_) { iasPrev_ = ias; haveIas_ = true; }
    iasRate_ += ((ias - iasPrev_) / dt - iasRate_) * std::min(dt / 1.0, 1.0);
    iasPrev_ = ias;
    // In the flare the thrust never comes up again, and once retarded it stays at idle
    // (a balloon above the retard height brought it back to full).
    // The retard is gradual, to idle over 4 s as an autothrottle's is: a chop
    // took the thrust away just as the flare needed the speed.
    // Also once the flare has levelled the airplane off: thrust held in a
    // float carries it down the runway.
    if (height < idleAt || (phase_ == Phase::Flare && vsNow > -0.5)) retarded_ = true;
    Real thr = clamp(baseThrottle + iasPid_.update(approachIas - ias, dt) - 0.25 * iasRate_, 0.0, 1.0);
    if (phase_ == Phase::Flare) thr = std::min(thr, lastThrottle_);
    if (retarded_) thr = std::max(lastThrottle_ - dt * std::max(retardFrom_, 0.05) / 4.0, 0.0);
    else retardFrom_ = thr;
    in.throttle = lastThrottle_ = thr;
    return in;
}

}  // namespace avd::scenarios
