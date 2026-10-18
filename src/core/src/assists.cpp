// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
#include "avd/assists.hpp"

namespace avd {

AssistConfig makePreset(AssistPreset p) {
    AssistConfig c;
    switch (p) {
        case AssistPreset::Arcade:
            c.autoMixture = c.autoCoordination = c.autoTrim = c.wingLeveler = c.stallPrevention = true;
            c.taxiBrakeSteering = true;
            c.inputExpo = 0.5;
            break;
        case AssistPreset::Sport:
            c.autoMixture = c.autoCoordination = true;
            c.taxiBrakeSteering = true;
            c.inputExpo = 0.3;
            break;
        case AssistPreset::Simulation: break;
    }
    return c;
}

const char* toString(AssistPreset p) {
    switch (p) {
        case AssistPreset::Arcade: return "Arcade";
        case AssistPreset::Sport: return "Sport";
        case AssistPreset::Simulation: return "Simulation";
    }
    return "?";
}

namespace {

// Smallest margin (rad) between a main-wing strip's angle of attack and its
// stall angle; 1 when there is no wing or no airflow.
Real wingStallMargin(const Aircraft& ac) {
    if (ac.aeroModel().surfaceCount() == 0) return 1;
    const AeroOutputs& a = ac.aero();
    Real margin = 1;
    for (int i = ac.aeroModel().firstStrip(0); i < ac.aeroModel().endStrip(0); ++i)
        if (a.strips[i].q > 0) margin = std::min(margin, a.strips[i].stallAlpha - a.strips[i].alpha);
    return margin;
}

}  // namespace

PilotInput FlightAssist::apply(const PilotInput& raw, const Aircraft& ac, Real dt) {
    r_ = {};
    PilotInput o = raw;
    const AircraftTelemetry& t = ac.telemetry();
    const AircraftDefinition& d = ac.definition();

    auto shape = [&](Real x) {
        const Real e = clamp(c_.inputExpo, 0.0, 1.0);
        return (1 - e) * x + e * x * x * x;
    };
    o.pitch = shape(raw.pitch);
    o.roll = shape(raw.roll);
    o.yaw = shape(raw.yaw);

    // Yaw input to the tiller at walking pace (hysteresis: off above the
    // handover speed, back on below 80% of it).
    if (c_.tillerHandover && ac.hasTiller()) {
        if (!t.onGround || t.groundSpeed > c_.tillerHandoverSpeed) onTiller_ = false;
        else if (t.groundSpeed < 0.8 * c_.tillerHandoverSpeed) onTiller_ = true;
        if (onTiller_) {
            o.tiller = clamp(o.tiller + o.yaw, -1.0, 1.0);
            o.yaw = 0;
        }
    } else {
        onTiller_ = false;
    }
    r_.yawOnTiller = onTiller_;

    const bool airborne = !t.onGround && t.trueAirspeed > 15;
    const Real qRef = 0.5 * 1.225 * 50 * 50;
    const Real qScale = clamp(qRef / std::max(t.dynamicPressure, 200.0), 0.3, 3.0);

    if (c_.autoMixture && ac.engineCount() > 0) {
        // Lever for best-power equivalence ratio at the current density.
        const PistonEngineDef& e = d.engines[0];
        const Real rich = e.mixtureRichPhi * std::sqrt(isa::kSeaLevelDensity / std::max(t.air.state.density, 0.05));
        o.mixture = clamp(1.15 / rich, 0.06, 1.0);
        r_.mixtureSet = o.mixture;
    }
    if (c_.autoCoordination && airborne) {
        betaIntegral_ = clamp(betaIntegral_ + t.beta * dt, -0.5, 0.5);
        const Real add = clamp((4.0 * t.beta + 2.0 * betaIntegral_ + 0.3 * t.omega.z * 0) * qScale, -1.0, 1.0);
        o.yaw = clamp(o.yaw + add, -1.0, 1.0);
        r_.rudderAdded = add;
    } else {
        betaIntegral_ = 0;
    }
    const bool sticksFree = std::abs(raw.pitch) < 0.05 && std::abs(raw.roll) < 0.05 && std::abs(raw.trimRate) < 1e-3;
    const bool hold = c_.levelHold && airborne && t.heightAgl > c_.levelHoldMinHeight && sticksFree;
    if (hold) {
        if (!holdEngaged_) headingCaptured_ = altitudeCaptured_ = false;
        holdEngaged_ = true;
        // Lateral: roll the wings level, then hold the heading once nearly level
        // (heading error 1 deg -> 1 deg of bank, at most 10 deg).
        if (!headingCaptured_ && std::abs(t.roll) < 0.05) {
            holdHeading_ = t.heading;
            headingCaptured_ = true;
        }
        const Real bankCmd = headingCaptured_ ? clamp(wrapPi(holdHeading_ - t.heading), -0.175, 0.175) : 0.0;
        const Real rollAdd = clamp((-1.2 * (t.roll - bankCmd) - 0.4 * t.omega.x) * qScale, -0.5, 0.5);
        o.roll = clamp(o.roll + rollAdd, -1.0, 1.0);
        r_.rollAdded = rollAdd;
        // Vertical: arrest the climb or descent, capture the altitude once
        // nearly level (100 fpm), then a flight-path command back to it (at most
        // ~500 fpm). The stick takes the fast correction and the trim wheel
        // (below) integrates it away, as an autopilot servo and its trim servo do.
        if (!altitudeCaptured_ && std::abs(t.verticalSpeed) < 0.5) {
            holdAltitude_ = t.altitudeMsl;
            altitudeCaptured_ = true;
        }
        const Real tas = std::max(t.trueAirspeed, 15.0);
        const Real vsCmd = altitudeCaptured_ ? clamp(0.1 * (holdAltitude_ - t.altitudeMsl), -2.5, 2.5) : 0.0;
        const Real gamma = std::asin(clamp(t.verticalSpeed / tas, -1.0, 1.0));
        Real gammaCmd = std::asin(clamp(vsCmd / tas, -1.0, 1.0));
        // Underspeed protection: closer than ~4 deg to the wing's stall angle
        // the hold lowers the nose as far as it takes to keep that margin,
        // trading height for speed instead of pitching up into the stall. It
        // climbs back to the held altitude once there is power to do so.
        const Real margin = wingStallMargin(ac);
        r_.underspeed = margin < 0.07;
        if (r_.underspeed) gammaCmd = std::min(gammaCmd, gamma + 3.0 * (margin - 0.07));
        const Real gammaError = gammaCmd - gamma;
        const Real gammaRate = kStandardGravity / tas * (t.loadFactor * std::cos(t.roll) - std::cos(gamma));
        const Real pitchAdd = clamp((6.0 * gammaError - 2.0 * gammaRate - 1.5 * t.omega.y) * qScale, -0.3, 0.3);
        o.pitch = clamp(o.pitch + pitchAdd, -1.0, 1.0);
        r_.pitchAdded = pitchAdd;
    } else {
        holdEngaged_ = false;
    }
    r_.levelHold = hold;
    if (c_.wingLeveler && !hold && airborne && std::abs(raw.roll) < 0.05) {
        const Real add = clamp((-1.2 * t.roll - 0.4 * t.omega.x) * qScale, -0.5, 0.5);
        o.roll = clamp(o.roll + add, -1.0, 1.0);
        r_.rollAdded = add;
    }
    if (c_.stallPrevention && airborne && o.pitch > 0 && ac.aeroModel().surfaceCount() > 0) {
        const Real allowed = clamp((wingStallMargin(ac) - 0.03) / 0.05, 0.0, 1.0);
        const Real limited = std::min(o.pitch, allowed);
        r_.pitchLimited = o.pitch - limited;
        o.pitch = limited;
    }
    if (hold && altitudeCaptured_) {
        // Trim runs until the hold needs no elevator beyond the trim; not
        // during the arrest (it would wind up) and never nose-up in underspeed.
        const Real add = clamp(o.pitch * 2.0, -1.0, r_.underspeed ? 0.0 : 1.0);
        o.trimRate = clamp(o.trimRate + add, -1.0, 1.0);
        r_.trimRateAdded = add;
    } else if (c_.autoTrim && airborne && std::abs(raw.trimRate) < 1e-3) {
        // Trim follows sustained stick input; the player relaxes as it does.
        const Real add = clamp(o.pitch * 1.5, -1.0, 1.0);
        o.trimRate = clamp(o.trimRate + add, -1.0, 1.0);
        r_.trimRateAdded = add;
    }
    if (c_.taxiBrakeSteering && !onTiller_ && t.onGround && t.groundSpeed < 12) {
        const Real b = clamp(std::abs(raw.yaw) - 0.6, 0.0, 0.4) / 0.4 * 0.5;
        if (raw.yaw > 0) o.brakeRight = std::max(o.brakeRight, b);
        if (raw.yaw < 0) o.brakeLeft = std::max(o.brakeLeft, b);
        r_.brakeAdded = b;
    }
    return o;
}

}  // namespace avd
