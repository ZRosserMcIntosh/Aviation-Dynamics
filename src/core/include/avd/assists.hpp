// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
//
// Player assistance. Assists are control logic that turns the player's input
// into the aircraft's control positions — the same inputs a pilot could make.
// They never read or modify the AircraftDefinition, the aerodynamic data or
// any force model (docs/decisions/0004-assistance-is-not-physics.md), and every
// intervention is reported. An aircraft's own systems (e.g. a fly-by-wire
// envelope protection on an airliner) belong to the aircraft, not here.
//
// Labels: each aid states whether a real-world equivalent exists.
#pragma once

#include "avd/aircraft.hpp"

namespace avd {

enum class AssistPreset { Arcade, Sport, Simulation };

struct AssistConfig {
    // Sets the mixture for best power at the current density
    // (real equivalent: automatic mixture control on some engines).
    bool autoMixture = false;
    // Rudder that drives sideslip to zero in flight (gameplay aid; comparable
    // to a yaw damper/turn coordinator in function, not in implementation).
    bool autoCoordination = false;
    // Moves the trim wheel to take out sustained pitch input (gameplay aid).
    bool autoTrim = false;
    // Rolls wings-level when roll input is released (comparable to a
    // wing-leveler autopilot).
    bool wingLeveler = false;
    // Limits nose-up input near the critical angle of attack (gameplay aid;
    // the aircraft can still stall, e.g. in a gust or with power changes).
    bool stallPrevention = false;
    // Differential braking from the rudder input at taxi speeds (what pilots
    // do with toe brakes; gameplay convenience for gamepads).
    bool taxiBrakeSteering = false;
    // Input shaping exponent blend (0 = linear, 1 = cubic) for stick axes.
    Real inputExpo = 0;
};

AssistConfig makePreset(AssistPreset p);
const char* toString(AssistPreset p);

struct AssistReport {
    Real rudderAdded = 0;
    Real rollAdded = 0;
    Real pitchLimited = 0;   // amount removed from nose-up input
    Real trimRateAdded = 0;
    Real mixtureSet = -1;    // -1 = not touched
    Real brakeAdded = 0;
    bool active() const {
        return rudderAdded != 0 || rollAdded != 0 || pitchLimited != 0 || trimRateAdded != 0 || mixtureSet >= 0 ||
               brakeAdded != 0;
    }
};

class FlightAssist {
public:
    explicit FlightAssist(const AssistConfig& c = {}) : c_(c) {}
    const AssistConfig& config() const { return c_; }
    void setConfig(const AssistConfig& c) { c_ = c; }

    // Shapes the raw player input using the aircraft's last telemetry.
    PilotInput apply(const PilotInput& raw, const Aircraft& aircraft, Real dt);
    const AssistReport& report() const { return r_; }

private:
    AssistConfig c_;
    AssistReport r_;
    Real betaIntegral_ = 0;
};

}  // namespace avd
