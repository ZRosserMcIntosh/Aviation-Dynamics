// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
//
// CSV telemetry recorder for debugging, plotting and regression analysis.
// Channels are documented in docs/validation/telemetry-channels.md.
#pragma once

#include <ostream>

#include "avd/aircraft.hpp"

namespace avd {

class TelemetryRecorder {
public:
    explicit TelemetryRecorder(std::ostream& out, Real interval = 0.1) : out_(out), interval_(interval) {}
    void writeHeader();
    // Records if at least `interval` has passed since the last row.
    void record(const Aircraft& ac);

private:
    std::ostream& out_;
    Real interval_;
    Real last_ = -1e9;
};

}  // namespace avd
