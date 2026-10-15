// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
//
// Procedure automation ("Quick Start"). A procedure is the same sequence of
// cockpit actions a pilot would perform, executed through the cockpit-control
// API and pilot inputs, step by step, waiting on real system state. It can
// therefore fail for real reasons (flat battery, no fuel, fuel valve shut by a
// failure) and reports why. It is not a teleport into a running state —
// Aircraft::setEngineRunning() is the explicit spawn shortcut for that.
#pragma once

#include <functional>
#include <string>
#include <vector>

#include "avd/aircraft.hpp"

namespace avd {

class ProcedureRunner {
public:
    enum class Status { Idle, Running, Done, Failed };

    struct Step {
        std::string description;
        std::function<void(Aircraft&, PilotInput&)> action;          // applied every tick while active
        std::function<bool(const Aircraft&, Real elapsed)> complete; // advance when true
        Real timeout = 30;                                           // s, failure if exceeded
        std::string failure;                                         // message on timeout
    };

    void start(std::vector<Step> steps);
    // Advance one tick: may operate switches and override `input` fields.
    void update(Aircraft& ac, PilotInput& input, Real dt);
    Status status() const { return status_; }
    int currentStep() const { return index_; }
    const std::string& message() const { return message_; }
    const std::vector<Step>& steps() const { return steps_; }

private:
    std::vector<Step> steps_;
    int index_ = 0;
    Real elapsed_ = 0;
    Status status_ = Status::Idle;
    std::string message_;
};

// Single-engine piston start: battery, fuel on, mixture rich, throttle cracked,
// ignition BOTH, crank until running, alternator and avionics on, warm idle.
std::vector<ProcedureRunner::Step> quickStartPistonSingle(const Aircraft& ac);

}  // namespace avd
