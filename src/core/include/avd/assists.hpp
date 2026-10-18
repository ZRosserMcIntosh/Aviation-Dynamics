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
    // Straight-and-level hold: with the stick (and trim) released in flight,
    // levels the wings, stops any climb or descent, then holds the heading and
    // altitude it captures, running the trim wheel so the elevator needs no
    // stick (comparable to a two-axis autopilot in heading and altitude hold
    // with auto-trim). The player's stick always wins; releasing it
    // re-captures. Off below `levelHoldMinHeight` so it never fights a flare.
    // Underspeed protection: near the stall it gives up height (a shallow
    // descent) rather than speed, like modern autopilots; it does not move
    // the throttle, so the player still owns the power.
    bool levelHold = false;
    Real levelHoldMinHeight = 30;    // m above ground
    // Input binding, not an assist (on in every preset, a no-op unless the
    // aircraft has a tiller): on the ground below `tillerHandoverSpeed` the
    // yaw input (Q/E, pedal axis) turns the nose-wheel tiller instead of the
    // pedals, as a crew steers a heavy aircraft by hand at walking pace. Above
    // it the yaw input goes back to the pedals, whose steering authority is
    // small, and the rudder; it returns to the tiller below 80% of that speed.
    bool tillerHandover = true;
    Real tillerHandoverSpeed = 2.572;  // m/s ground speed (5 kt)
    // Input shaping exponent blend (0 = linear, 1 = cubic) for stick axes.
    Real inputExpo = 0;
};

AssistConfig makePreset(AssistPreset p);
const char* toString(AssistPreset p);

struct AssistReport {
    Real rudderAdded = 0;
    Real rollAdded = 0;
    Real pitchAdded = 0;     // straight-and-level hold
    Real pitchLimited = 0;   // amount removed from nose-up input
    Real trimRateAdded = 0;
    Real mixtureSet = -1;    // -1 = not touched
    Real brakeAdded = 0;
    bool levelHold = false;  // straight-and-level hold engaged
    bool underspeed = false; // ... and trading height for speed near the stall
    // The yaw input is on the tiller (an input binding; not counted as an assist).
    bool yawOnTiller = false;
    bool active() const {
        return rudderAdded != 0 || rollAdded != 0 || pitchAdded != 0 || pitchLimited != 0 || trimRateAdded != 0 ||
               mixtureSet >= 0 || brakeAdded != 0 || levelHold;
    }
};

class FlightAssist {
public:
    explicit FlightAssist(const AssistConfig& c = {}) : c_(c) {}
    const AssistConfig& config() const { return c_; }
    void setConfig(const AssistConfig& c) { c_ = c; }
    // Forget captured references and integrators (new aircraft or spawn).
    void reset() { *this = FlightAssist(c_); }

    // Shapes the raw player input using the aircraft's last telemetry.
    PilotInput apply(const PilotInput& raw, const Aircraft& aircraft, Real dt);
    const AssistReport& report() const { return r_; }

private:
    AssistConfig c_;
    AssistReport r_;
    Real betaIntegral_ = 0;
    // Straight-and-level hold references, captured when the stick is released.
    bool holdEngaged_ = false, headingCaptured_ = false, altitudeCaptured_ = false;
    Real holdAltitude_ = 0, holdHeading_ = 0;
    bool onTiller_ = false;
};

}  // namespace avd
