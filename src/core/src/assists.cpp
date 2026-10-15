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
    if (c_.wingLeveler && airborne && std::abs(raw.roll) < 0.05) {
        const Real add = clamp((-1.2 * t.roll - 0.4 * t.omega.x) * qScale, -0.5, 0.5);
        o.roll = clamp(o.roll + add, -1.0, 1.0);
        r_.rollAdded = add;
    }
    if (c_.stallPrevention && airborne && o.pitch > 0 && ac.aeroModel().surfaceCount() > 0) {
        // Margin to the main wing's stall angle (mean strip alpha vs stall).
        const AeroOutputs& a = ac.aero();
        const int s0 = ac.aeroModel().firstStrip(0), s1 = ac.aeroModel().endStrip(0);
        Real margin = 1;
        for (int i = s0; i < s1; ++i)
            if (a.strips[i].q > 0) margin = std::min(margin, a.strips[i].stallAlpha - a.strips[i].alpha);
        const Real allowed = clamp((margin - 0.03) / 0.05, 0.0, 1.0);
        const Real limited = std::min(o.pitch, allowed);
        r_.pitchLimited = o.pitch - limited;
        o.pitch = limited;
    }
    if (c_.autoTrim && airborne && std::abs(raw.trimRate) < 1e-3) {
        // Trim follows sustained stick input; the player relaxes as it does.
        const Real add = clamp(o.pitch * 1.5, -1.0, 1.0);
        o.trimRate = clamp(o.trimRate + add, -1.0, 1.0);
        r_.trimRateAdded = add;
    }
    if (c_.taxiBrakeSteering && t.onGround && t.groundSpeed < 12) {
        const Real b = clamp(std::abs(raw.yaw) - 0.6, 0.0, 0.4) / 0.4 * 0.5;
        if (raw.yaw > 0) o.brakeRight = std::max(o.brakeRight, b);
        if (raw.yaw < 0) o.brakeLeft = std::max(o.brakeLeft, b);
        r_.brakeAdded = b;
    }
    return o;
}

}  // namespace avd
