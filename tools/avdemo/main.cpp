// Aviation Dynamics — headless demonstration and validation report.
// SPDX-License-Identifier: MIT
//
//   avdemo validate            POH comparison table (Markdown)
//   avdemo flight [csv]        scripted flight at Campo de Marte (SBMT): quick start,
//                              take-off, climb, cruise, stall, descent, landing
//   avdemo starvation [csv]    Scenario A: departure with little fuel until the engine quits
//   avdemo shutdown            Scenario C (piston analogue): fuel shutoff valve closed in cruise
//   avdemo cockpit             list interactive cockpit controls
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>

#include "airports.hpp"
#include "avd/assists.hpp"
#include "avd/procedures.hpp"
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
    const TakeoffResult to = takeoffTest(def, 0, 0);
    row("Take-off ground roll (short field)", "960 ft", mToFt(to.groundRoll), "ft", 960, 20);
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
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
    std::cerr << "usage: avdemo [validate|flight [csv]|starvation [csv]|shutdown|cockpit]\n";
    return 2;
}

