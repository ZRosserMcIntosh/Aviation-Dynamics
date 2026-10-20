// Aviation Dynamics — scenarios: validation rows against published data.
// SPDX-License-Identifier: MIT
//
// One source of truth for the comparisons with published performance
// figures: the tests (tests/test_validation.cpp) assert each row, `avdemo
// validate` prints them, and `tools/update_validation_docs.py` rewrites the
// tables in docs/ from the JSON both can write. The simulated values are
// never tuned to a row here; a row that fails stays failing until someone
// decides what to do about it (docs/validation/*.md).
#pragma once

#include <ostream>
#include <string>
#include <vector>

#include "avd/math.hpp"

namespace avd::scenarios::validation {

struct Row {
    enum class Kind { Tolerance, Range, Info };
    std::string id;         // stable, e.g. "c172s.climb.sl" (the docs and tests key on it)
    std::string aircraft;   // "tupi_t180" | "top_g"
    std::string quantity;   // "Rate of climb, sea level, 74 KIAS"
    std::string published;  // "730 fpm" (as printed in the source)
    std::string source;     // "C172S POH Section 5, Figure 5-7"
    std::string unit;       // "fpm"
    std::string note;       // method, caveats, estimated/representative
    Kind kind = Kind::Tolerance;
    Real reference = 0;     // the published value (Tolerance), or the band's midpoint (Range)
    Real value = 0;         // simulated
    Real tolerancePct = 0;  // Tolerance: symmetric band; Range: informational half-width in %
    Real low = 0, high = 0; // the pass band (set from reference and tolerancePct for Tolerance rows)
    Real errorPct() const { return reference != 0 ? (value - reference) / reference * 100 : 0; }
    bool pass() const { return kind == Kind::Info || (value >= low && value <= high); }
};
using Rows = std::vector<Row>;

// Builders.
Row tolerance(const std::string& id, const std::string& quantity, const std::string& published, Real reference,
              Real value, Real tolerancePct, const std::string& unit, const std::string& source,
              const std::string& note = "");
Row range(const std::string& id, const std::string& quantity, const std::string& published, Real low, Real high,
          Real value, const std::string& unit, const std::string& source, const std::string& note = "");
Row info(const std::string& id, const std::string& quantity, Real value, const std::string& unit,
         const std::string& note = "");

// ---- Tupi T-180 against the Cessna 172S POH (2,550 lb, mid CG, ISA, no wind) ----
Rows c172Stalls();                 // c172s.stall.clean, c172s.stall.flaps30, c172s.stall.warning.* (info)
Rows c172MaxLevelSpeed();          // c172s.vmax.sl (+ rpm)
Rows c172FullThrottle8000();       // c172s.cruise.8000.full (+ power, rpm, efficiency, fuel flow)
Rows c172ClimbRates();             // c172s.climb.{sl,4000,8000,12000}, c172s.climb.to8000.{time,fuel,distance}
Rows c172CruiseTable();            // c172s.cruise.{2000,8000}.{55,65,75}.{ktas,gph}
Rows c172BestGlide();              // c172s.glide.ratio, c172s.glide.nm_per_1000ft
Rows c172Takeoff();                // c172s.takeoff.{roll,over50ft,liftoff}
Rows c172LandingRoll();            // c172s.landing.roll, c172s.landing.tyre_limit (info)
Rows c172StaticRpm();              // c172s.static_rpm
Rows c172All();

// ---- Top G against published / representative Gulfstream G650ER figures ----
Rows topGTimeToClimb();            // top_g.climb.fl410.{time,fuel,distance}
Rows topGMaxCruise();              // top_g.cruise.fl410.m090.throttle (+ fuel flow), top_g.cruise.fl450.m085 (info)
Rows topGVref();                   // top_g.vref.27500kg, top_g.vref.mlw
Rows topGLandingDistance();        // top_g.landing.distance
Rows topGAll();

// Every row id the full report contains (the docs script checks a JSON is complete).
std::vector<std::string> allIds();

// Machine-readable report: {"generated": ..., "rows": [...]}.
void writeJson(std::ostream& out, const Rows& rows, bool complete);
// Markdown table (the same layout the docs carry between their markers).
void writeMarkdown(std::ostream& out, const Rows& rows);

}  // namespace avd::scenarios::validation
