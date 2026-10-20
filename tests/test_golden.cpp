// Aviation Dynamics — golden-trace regression: each canonical scenario
// (src/scenarios/golden.cpp) is flown again and compared, channel by
// channel, with the recorded trace in tests/golden/<name>.csv.
// SPDX-License-Identifier: MIT
//
// A missing golden file SKIPS that scenario with a message (CI stays green
// until the traces are recorded with `./build/tools/avdtrace --record`). A
// mismatch prints the worst delta per channel and the time of the first
// divergence. Goldens change only in a commit that explains why
// (tests/golden/README.md).
//
// The traces are recorded on the reference platform (Linux x86-64, GCC,
// Release) and compared only there: another compiler, architecture or
// optimisation level differs in the last bits of a few operations (FMA
// contraction, libm, vectorised sums), and the contact-heavy scenarios
// amplify that past the tight bands (ADR 0003 disclaims cross-platform
// bit-identity). Elsewhere the cases SKIP; AVD_GOLDEN_FORCE=1 compares anyway.
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>

#include "avd_test.hpp"
#include "golden.hpp"

namespace g = avd::scenarios::golden;

namespace {

bool referencePlatform() {
#if defined(__linux__) && defined(__x86_64__) && defined(__GNUC__) && !defined(__clang__) && defined(NDEBUG)
    return true;
#else
    return false;
#endif
}

bool compareHere() {
    if (const char* f = std::getenv("AVD_GOLDEN_FORCE"); f && *f && *f != '0') return true;
    return referencePlatform();
}

void replay(const char* name) {
    if (!compareHere()) {
        std::cout << "  SKIPPED: golden traces are compared on the reference platform (Linux x86-64, GCC, Release); "
                     "set AVD_GOLDEN_FORCE=1 to compare here\n";
        return;
    }
    const std::string path = g::goldenPath(name);
    std::ifstream f(path);
    if (!f) {
        std::cout << "  SKIPPED: no golden trace at " << path << " (record it with ./build/tools/avdtrace --record)\n";
        return;
    }
    g::Trace golden;
    std::string err;
    if (!g::readCsv(f, golden, err)) {
        ++avdtest::failures();
        std::cerr << "  FAILED: unreadable golden " << path << ": " << err << "\n";
        return;
    }
    const g::Trace fresh = g::run(name);
    const g::Comparison c = g::compare(golden, fresh);
    if (!c.ok) {
        ++avdtest::failures();
        std::cerr << "  FAILED: " << name << " differs from " << path << "\n";
        g::writeReport(std::cerr, c);
    } else {
        std::cout << "  " << name << ": " << fresh.rows.size() << " rows match the golden trace\n";
    }
}

}  // namespace

TEST_CASE("golden: Tupi T-180 take-off from Congonhas 35L to 500 ft AGL") { replay("c172_takeoff_sbsp"); }
TEST_CASE("golden: Tupi T-180 power-off stall and recovery from 3,000 ft") { replay("c172_stall_3000ft"); }
TEST_CASE("golden: Tupi T-180 trimmed cruise, stick free, wings level, 5 minutes") { replay("c172_cruise_trim"); }
TEST_CASE("golden: Tupi T-180 ILS approach to Congonhas 35L to touchdown and stop") { replay("c172_ils_sbsp"); }
TEST_CASE("golden: Top G take-off from Congonhas 35L to 1,500 ft AGL") { replay("topg_takeoff_sbsp"); }
TEST_CASE("golden: Top G trimmed cruise at FL410, Mach 0.85, 3 minutes") { replay("topg_cruise_fl410"); }
TEST_CASE("golden: Top G ILS approach to Guarulhos to touchdown and stop") { replay("topg_ils_sbgr"); }

TEST_CASE("golden: the CSV round-trips and the comparison sees a change") {
    // A short synthetic trace: write, read back, compare with itself and with a perturbed copy.
    g::Trace t;
    t.scenario = "synthetic";
    for (int i = 0; i < 50; ++i) {
        g::Sample s{};
        s[0] = i * g::kSampleInterval;
        s[4] = 100 + 0.1 * i;    // tas_kt
        s[13] = 359.9 + 0.01 * i;  // heading_deg crosses 360 -> compares on the circle
        s[22] = 2000 + i;        // thrust1_n
        t.rows.push_back(s);
    }
    std::stringstream ss;
    g::writeCsv(ss, t);
    g::Trace back;
    std::string err;
    CHECK(g::readCsv(ss, back, err));
    CHECK(err.empty());
    CHECK(back.scenario == "synthetic");
    CHECK(back.rows.size() == t.rows.size());
    CHECK(g::compare(t, back).ok);
    g::Trace other = t;
    other.rows[30][4] += 0.6;  // tas beyond 0.5 kt at t = 3.0 s
    other.rows[40][13] -= 360;  // the same heading, written on the other side of 360
    other.rows[45][22] *= 1.015;  // thrust within 2 %
    const g::Comparison c = g::compare(t, other);
    CHECK(!c.ok);
    CHECK_NEAR(c.firstDivergence, 3.0, 1e-9);
    CHECK(c.channel[4].firstBeyond >= 0);
    CHECK(c.channel[13].firstBeyond < 0);
    CHECK(c.channel[22].firstBeyond < 0);
    // Every scenario name has a description and a golden path.
    for (const std::string& n : g::scenarioNames()) {
        CHECK(!g::describe(n).empty());
        CHECK(g::goldenPath(n).find(n + ".csv") != std::string::npos);
    }
    CHECK(g::scenarioNames().size() == 7);
}
