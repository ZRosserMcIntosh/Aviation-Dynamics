// Aviation Dynamics — headless demonstration and validation report.
// SPDX-License-Identifier: MIT
//
//   avdemo validate [--json f]  comparison tables against published data (Markdown; JSON for the docs)
//   avdemo flight [csv]        scripted flight at Campo de Marte (SBMT): quick start,
//                              take-off, climb, cruise, stall, descent, landing
//   avdemo starvation [csv]    Scenario A: departure with little fuel until the engine quits
//   avdemo shutdown            Scenario C (piston analogue): fuel shutoff valve closed in cruise
//   avdemo cockpit             list interactive cockpit controls
//   avdemo bench               per-step CPU cost
//   avdemo instruments [csv]   truth vs instrument display (debug view, optional CSV)
#include <chrono>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>

#include "airports.hpp"
#include "avd/assists.hpp"
#include "avd/procedures.hpp"
#include "avd/instruments.hpp"
#include "avd/telemetry.hpp"
#include "avd/units.hpp"
#include "scenarios.hpp"
#include "validation.hpp"

using namespace avd;
using namespace avd::scenarios;
using namespace avd::units;
namespace val = avd::scenarios::validation;

namespace {

// The validation rows (src/scenarios/validation.cpp) as Markdown, and as
// JSON for tools/update_validation_docs.py when `--json <path>` is given.
int validate(const char* jsonPath) {
    const val::Rows c172 = val::c172All();
    const val::Rows topg = val::topGAll();
    std::cout << "## Tupi T-180 vs Cessna 172S POH (2,550 lb, ISA, mid CG)\n\n";
    val::writeMarkdown(std::cout, c172);
    std::cout << "\n## Top G vs published / representative Gulfstream G650ER figures\n\n";
    val::writeMarkdown(std::cout, topg);
    int outside = 0;
    val::Rows all = c172;
    all.insert(all.end(), topg.begin(), topg.end());
    for (const val::Row& r : all)
        if (!r.pass()) ++outside;
    std::cout << "\n" << all.size() << " rows, " << outside << " outside their band.\n";
    if (jsonPath) {
        std::ofstream f(jsonPath);
        val::writeJson(f, all, true);
        std::cout << "JSON: " << jsonPath << "\n";
    }
    return 0;
}

int bench() {
    const AircraftDefinition def = loadReference();
    SimpleWeather::Params wp;
    wp.turbulence.intensity = 1.0;
    SimpleWeather atm(wp);
    SimpleWeather calm;
    FlatGround ground(0);
    auto run = [&](const AtmosphereField& a, bool onGround) {
        Aircraft ac(def);
        TestPilot pilot;
        PilotInput in;
        if (onGround) {
            ac.setEngineRunning(true, 1000);
            ac.placeOnGround(0, 0, 0, ground, a);
            ac.setCockpitControl("brakes.parking", 0);
            in.throttle = 0.3;
        } else {
            startTrimmedFlight(ac, {100, 1500, 0}, a, ground, &pilot);
        }
        const int steps = 240 * 60;
        const auto t0 = std::chrono::steady_clock::now();
        for (int i = 0; i < steps; ++i) ac.step(kDefaultDt, onGround ? in : pilot.update(ac, kDefaultDt), ground, a);
        const auto t1 = std::chrono::steady_clock::now();
        return std::chrono::duration<double, std::micro>(t1 - t0).count() / steps;
    };
    const double cruise = run(calm, false), turb = run(atm, false), taxi = run(calm, true);
    std::cout << "## Per-step CPU cost (one core, this machine; 240 Hz fixed step, RK4 = 4+1 force evaluations)\n\n";
    std::cout << "| Case | us / step | ms per 60 fps frame (4 steps) | % of one core in real time |\n|---|---|---|---|\n";
    for (auto [name, us] : {std::pair<const char*, double>{"cruise, calm air", cruise}, {"cruise, world-space turbulence (48 modes)", turb},
                            {"taxi on the gear", taxi}})
        std::cout << "| " << name << " | " << std::fixed << std::setprecision(1) << us << " | " << std::setprecision(3)
                  << us * 4 / 1000 << " | " << std::setprecision(2) << us * 240 / 1e4 << " % |\n";
    return 0;
}

// Debug view: what the aircraft is doing vs what the panel shows, during a
// scripted climbing turn with a vacuum failure partway through.
int instrumentDebug(const char* csvPath) {
    const AircraftDefinition def = loadReference();
    SimpleWeather atm;
    FlatGround ground(0);
    Aircraft ac(def, saoPauloWorldFrame());
    ac.setNavEnvironment(&saoPauloNavEnvironment());
    TestPilot pilot;
    startTrimmedFlight(ac, {95, 1200, 0}, atm, ground, &pilot);
    std::unique_ptr<std::ofstream> file;
    std::unique_ptr<InstrumentRecorder> rec;
    if (csvPath) {
        file = std::make_unique<std::ofstream>(csvPath);
        rec = std::make_unique<InstrumentRecorder>(*file, 0.1);
        rec->writeHeader();
    }
    pilot.lateral = TestPilot::Lateral::Bank;
    pilot.targetBank = degToRad(20);
    pilot.controls.throttle = 1;
    pilot.vertical = TestPilot::Vertical::Airspeed;
    pilot.targetIas = ktToMs(80);
    std::cout << "## Truth vs display (climbing 20 deg turn; vacuum pump fails at t=60 s)\n\n"
              << "| t s | KTAS | KCAS | KIAS | alt ft | ind ft | VS fpm | VSI fpm | roll | AI roll | MH | DG | compass | TC dps | ball | AI rotor | OIL psi | ann |\n"
              << "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|\n";
    for (int i = 0; i <= 240 * 240; ++i) {
        if (i == 60 * 240) ac.vacuum().setPumpFailed(true);
        ac.step(kDefaultDt, pilot.update(ac, kDefaultDt), ground, atm);
        if (rec) rec->record(ac);
        if (i % (20 * 240) != 0) continue;
        const InstrumentTruth& t = ac.instruments().truth();
        const CockpitState& c = ac.instruments().state();
        auto hd = [](Real r) { return std::lround(std::fmod(radToDeg(r) + 360, 360.0)); };
        std::cout << std::fixed << std::setprecision(0) << "| " << ac.time() << " | " << msToKt(t.trueAirspeed) << " | "
                  << msToKt(t.calibratedAirspeed) << " | " << msToKt(c.indicatedAirspeed) << " | " << mToFt(t.altitudeMsl)
                  << " | " << mToFt(c.indicatedAltitude) << " | " << msToFpm(t.verticalSpeed) << " | "
                  << msToFpm(c.verticalSpeed) << " | " << std::setprecision(1) << radToDeg(t.roll) << " | "
                  << radToDeg(c.attitudeRoll) << " | " << hd(t.magneticHeading) << " | " << hd(c.headingIndicator) << " | "
                  << hd(c.magneticHeading) << " | " << radToDeg(c.turnRate) << " | " << radToDeg(c.slipBall) << " | "
                  << std::setprecision(2) << t.attitudeGyroSpin << " | " << std::setprecision(0) << paToPsi(c.oilPressure)
                  << " | " << (c.annunciators & kAnnLowVacuum ? "LOW VAC" : "-") << " |\n";
    }
    return 0;
}

void listCockpit() {
    Aircraft ac(loadReference());
    std::cout << "Interactive cockpit controls (" << ac.definition().displayName << "):\n";
    for (const CockpitControl& c : ac.cockpitControls()) {
        std::cout << "  Press F to interact — " << c.label << "  [" << c.id << "]  positions:";
        for (std::size_t i = 0; i < c.positions.size(); ++i)
            std::cout << (static_cast<int>(i) == c.position ? " *" : " ") << c.positions[i];
        std::cout << "\n";
    }
}

}  // namespace

int flightDemo(const char* csv);
int starvationDemo(const char* csv);
int shutdownDemo();

int main(int argc, char** argv) {
    const char* cmd = argc > 1 ? argv[1] : "validate";
    const char* csv = argc > 2 ? argv[2] : nullptr;
    try {
        if (!std::strcmp(cmd, "validate")) return validate(csv && !std::strcmp(csv, "--json") && argc > 3 ? argv[3] : nullptr);
        if (!std::strcmp(cmd, "flight")) return flightDemo(csv);
        if (!std::strcmp(cmd, "starvation")) return starvationDemo(csv);
        if (!std::strcmp(cmd, "shutdown")) return shutdownDemo();
        if (!std::strcmp(cmd, "cockpit")) { listCockpit(); return 0; }
        if (!std::strcmp(cmd, "bench")) return bench();
        if (!std::strcmp(cmd, "instruments")) return instrumentDebug(csv);
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
    std::cerr << "usage: avdemo [validate [--json file]|flight [csv]|starvation [csv]|shutdown|cockpit|bench|instruments [csv]]\n";
    return 2;
}

