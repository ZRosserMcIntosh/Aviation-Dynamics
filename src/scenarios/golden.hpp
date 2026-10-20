// Aviation Dynamics — scenarios: golden-trace regression scenarios.
// SPDX-License-Identifier: MIT
//
// Named canonical flights, flown deterministically (calm air, scripted
// pilots, fixed 240 Hz step) and sampled at 10 Hz into a fixed set of
// channels. `tools/avdtrace --record` writes them to tests/golden/<name>.csv;
// tests/test_golden.cpp replays each scenario and compares it with its golden
// file channel by channel. A golden file changes only in a commit that says
// why (tests/golden/README.md).
#pragma once

#include <array>
#include <iosfwd>
#include <string>
#include <vector>

#include "avd/math.hpp"

namespace avd::scenarios::golden {

inline constexpr int kChannels = 30;
inline constexpr Real kSampleInterval = 0.1;  // s (10 Hz)

struct Channel {
    const char* name;      // CSV column
    Real tolerance;        // absolute, in the channel's unit
    Real relative;         // plus this fraction of the golden value's magnitude (thrust: 2 %)
    bool angleDeg = false; // compare on the circle
};
// Column order of every trace (see docs/validation/golden-traces.md).
const std::array<Channel, kChannels>& channels();

using Sample = std::array<Real, kChannels>;

struct Trace {
    std::string scenario;
    std::vector<Sample> rows;
};

// The canonical scenarios, in the order --record writes them.
std::vector<std::string> scenarioNames();
// One line of what the scenario does (for --list and the docs).
std::string describe(const std::string& name);
// Fly the scenario. Throws std::invalid_argument for an unknown name.
Trace run(const std::string& name);

void writeCsv(std::ostream& out, const Trace& t);
// False with `error` set on a malformed file.
bool readCsv(std::istream& in, Trace& t, std::string& error);

struct ChannelDelta {
    Real worst = 0;        // largest |delta| over the compared rows
    Real worstTime = 0;    // when
    Real tolerance = 0;    // the band applied at that row
    Real firstBeyond = -1; // first time the channel left its band (-1: never)
};
struct Comparison {
    bool ok = false;
    std::size_t goldenRows = 0, freshRows = 0;
    Real firstDivergence = -1;  // s; the first time any channel left its band
    std::array<ChannelDelta, kChannels> channel{};
};
Comparison compare(const Trace& goldenTrace, const Trace& fresh);
// The compact per-channel table (worst delta, time, band, status).
void writeReport(std::ostream& out, const Comparison& c);

// Where the goldens live: AVD_GOLDEN_DIR (compile-time: <repo>/tests/golden),
// overridable at run time by the AVD_GOLDEN_DIR environment variable.
std::string goldenDirectory();
// Overrides AVD_GOLDEN_DIR and the build-time default for this process
// (the recorder's --dir); an empty string clears the override.
void setGoldenDirectory(const std::string& dir);
std::string goldenPath(const std::string& name);

}  // namespace avd::scenarios::golden
