// Aviation Dynamics — headless demonstration and validation report.
// SPDX-License-Identifier: MIT
//
//   avdemo validate            POH comparison table (Markdown)
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

using namespace avd;
using namespace avd::scenarios;
using namespace avd::units;

namespace {

void trimNote(const TrimResult& t) {
    if (!t.converged) std::cout << "| (trim did NOT converge, residual " << t.residual << ") | | | | |\n";
}

void row(const char* what, const char* poh, double sim, const char* unit, double ref, double tol) {
    const double err = (sim - ref) / ref * 100;
    std::cout << "| " << what << " | " << poh << " | " << std::fixed << std::setprecision(1) << sim << " " << unit
              << " | " << std::showpos << err << std::noshowpos << "% | " << (std::abs(err) <= tol ? "within" : "OUTSIDE")
              << " ±" << tol << "% |\n";
}

int validate() {
    const AircraftDefinition def = loadReference();
    SimpleWeather atm;
    std::cout << "## Tupi T-180 vs Cessna 172S POH (2550 lb, ISA, mid CG)\n\n";
    std::cout << "| Quantity | POH | Simulation | Error | Tolerance |\n|---|---|---|---|---|\n";

    const StallResult s0 = stallTest(def, 0);
    const StallResult s30 = stallTest(def, 3);
    row("Stall speed, flaps up, power off", "53 KCAS", msToKt(s0.minCas), "KCAS", 53, 6);
    row("Stall speed, flaps 30, power off", "48 KCAS", msToKt(s30.minCas), "KCAS", 48, 6);

    {
        Aircraft ac(def);
        ac.setStationMass(0, 170);
        ac.setStationMass(1, def.maxTakeoffMass - ac.massProperties().mass);
        TrimRequest r;
        r.altitude = ftToM(8000);
        r.tas = ktToMs(120);
        r.throttle = 1.0;
        r.solveTas = true;
        const TrimResult t = trim(ac, r, atm);
        trimNote(t);
        row("Full-throttle level speed, 8000 ft", "124 KTAS (75 % power)", msToKt(t.tas), "KTAS", 124, 5);
        std::cout << "|  ↳ power / rpm / prop efficiency / fuel flow | 75 %, ~2700 rpm, ~10.5 gph | " << std::setprecision(0)
                  << t.brakePower / def.engines[0].ratedPower * 100 << " % / " << t.rpm << " rpm / " << std::setprecision(2)
                  << t.propEfficiency << " / " << std::setprecision(1) << m3ToGal(t.fuelFlow / def.fuel.density) * 3600
                  << " gph | | |\n";
    }
    {
        Aircraft ac(def);
        ac.setStationMass(0, 170);
        ac.setStationMass(1, def.maxTakeoffMass - ac.massProperties().mass);
        TrimRequest r;
        r.altitude = 0;
        r.tas = ktToMs(120);
        r.throttle = 1.0;
        r.solveTas = true;
        const TrimResult t = trim(ac, r, atm);
        trimNote(t);
        row("Maximum level speed, sea level", "126 KTAS", msToKt(t.tas), "KTAS", 126, 5);
        std::cout << "|  ↳ rpm at max speed | ≤ 2700 (red line) | " << std::setprecision(0) << t.rpm << " rpm | | |\n";
    }
    {
        Aircraft ac(def);
        ac.setStationMass(0, 170);
        ac.setStationMass(1, def.maxTakeoffMass - ac.massProperties().mass);
        TrimRequest r;
        r.altitude = 0;
        r.tas = tasFromCas(ktToMs(74), 1.225);
        r.throttle = 1.0;
        r.solveFlightPath = true;
        const TrimResult t = trim(ac, r, atm);
        trimNote(t);
        row("Rate of climb, sea level, 74 KIAS", "730 fpm", msToFpm(t.tas * std::sin(t.flightPath)), "fpm", 730, 15);
    }
    const TakeoffResult to = takeoffTest(def, 0, 0, 0, 0, 51);
    row("Take-off ground roll (short field)", "960 ft", mToFt(to.groundRoll), "ft", 960, 20);
    std::cout << "|  ↳ known discrepancy | suspected optimistic static thrust (BEM without swirl losses) | | | |\n";
    row("Take-off over 50 ft obstacle", "1630 ft", mToFt(to.distanceTo50ft), "ft", 1630, 20);
    const Real lr = landingRollTest(def);
    std::cout << "| Landing ground roll, maximum braking | 575 ft (POH: demonstrated braking) | " << std::setprecision(0)
              << mToFt(lr) << " ft | " << std::showpos << std::setprecision(1) << (mToFt(lr) - 575) / 5.75
              << std::noshowpos << "% | known discrepancy, see docs/validation |\n";
    {
        Aircraft ac(def);
        SimpleWeather a2;
        AirfieldGround g(0);
        ac.setEngineRunning(true, 1000);
        ac.placeOnGround(0, 0, 0, g, a2);
        PilotInput in;
        in.throttle = 1;
        in.mixture = 1;
        for (int i = 0; i < 240 * 6; ++i) ac.step(1.0 / 240, in, g, a2);
        std::cout << "| Static rpm, full throttle | 2300-2420 rpm (POH run-up/static check) | " << std::setprecision(0)
                  << ac.telemetry().engines[0].rpm << " rpm | | |\n";
    }
    std::cout << "\nStall warning sounded at " << std::setprecision(1) << msToKt(s0.warningIas) << " KIAS (flaps up; stall at "
              << msToKt(s0.minIas) << " KIAS), " << msToKt(s30.warningIas) << " KIAS (flaps 30; stall at "
              << msToKt(s30.minIas) << " KIAS). POH: warning 5-10 kt above the stall.\n";
    std::cout << "Hot & high: SBMT elevation 722 m, ISA+20: ground roll " << std::setprecision(0)
              << mToFt(takeoffTest(def, 722, 20).groundRoll) << " ft (sea-level ISA: " << mToFt(to.groundRoll) << " ft).\n";
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
                  << std::setprecision(2) << t.attitudeGyroSpin << " | " << std::setprecision(0) << c.oilPressure / 6894.757
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
        if (!std::strcmp(cmd, "validate")) return validate();
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
    std::cerr << "usage: avdemo [validate|flight [csv]|starvation [csv]|shutdown|cockpit|bench|instruments [csv]]\n";
    return 2;
}

