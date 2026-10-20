// Aviation Dynamics — scenarios: validation rows against published data.
// SPDX-License-Identifier: MIT
//
// Sources and provenance (also in docs/validation/c172s-validation.md and
// docs/validation/top-g-validation.md):
//   [POH]  Cessna 172S Pilot's Operating Handbook (Nav III), Section 5
//          Performance: Figure 5-6 (maximum rate of climb), 5-7 (time, fuel
//          and distance to climb), 5-8 (cruise performance), maximum glide;
//          Section 3 (maximum glide 68 KIAS). 2,550 lb, standard temperature,
//          recommended lean mixture for cruise, full throttle for climb.
//          Values are transcribed from the handbook tables; where the
//          handbook tabulates by rpm (cruise) the 55/65/75 % rows are its
//          nearest rows, interpolated in % BHP.
//   [G650] Gulfstream G650ER published figures (Gulfstream specification
//          sheet, brokers' data sheets) and class-typical values: these are
//          "representative, estimated" and carry wide bands. The Top G is a
//          fictional aircraft that flies as a G650ER.
#include "validation.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

#include "approach_pilot.hpp"
#include "avd/aero.hpp"
#include "avd/gps.hpp"
#include "avd/units.hpp"
#include "scenarios.hpp"

namespace avd::scenarios::validation {

using namespace units;

// ------------------------------------------------------------------ rows ----

Row tolerance(const std::string& id, const std::string& quantity, const std::string& published, Real reference,
              Real value, Real tolerancePct, const std::string& unit, const std::string& source, const std::string& note) {
    Row r;
    r.id = id;
    r.quantity = quantity;
    r.published = published;
    r.reference = reference;
    r.value = value;
    r.tolerancePct = tolerancePct;
    r.unit = unit;
    r.source = source;
    r.note = note;
    r.kind = Row::Kind::Tolerance;
    r.low = reference * (1 - tolerancePct / 100);
    r.high = reference * (1 + tolerancePct / 100);
    if (r.low > r.high) std::swap(r.low, r.high);
    return r;
}

Row range(const std::string& id, const std::string& quantity, const std::string& published, Real low, Real high,
          Real value, const std::string& unit, const std::string& source, const std::string& note) {
    Row r;
    r.id = id;
    r.quantity = quantity;
    r.published = published;
    r.reference = 0.5 * (low + high);
    r.value = value;
    r.tolerancePct = r.reference != 0 ? (high - low) / 2 / std::abs(r.reference) * 100 : 0;
    r.unit = unit;
    r.source = source;
    r.note = note;
    r.kind = Row::Kind::Range;
    r.low = low;
    r.high = high;
    return r;
}

Row info(const std::string& id, const std::string& quantity, Real value, const std::string& unit, const std::string& note) {
    Row r;
    r.id = id;
    r.quantity = quantity;
    r.value = value;
    r.unit = unit;
    r.note = note;
    r.kind = Row::Kind::Info;
    return r;
}

namespace {

constexpr const char* kPoh = "C172S POH Section 5";
constexpr const char* kC172 = "tupi_t180";
constexpr const char* kTopG = "top_g";

std::string fmt(Real v, int decimals = 1) {
    std::ostringstream os;
    os << std::fixed << std::setprecision(decimals) << v;
    return os.str();
}

const AircraftDefinition& c172Def() {
    static const AircraftDefinition d = loadReference(kC172);
    return d;
}
const AircraftDefinition& topGDef() {
    static const AircraftDefinition d = loadReference(kTopG);
    return d;
}

// Two front occupants and the rest in the rear seats to 2,550 lb with the
// data file's fuel (the loading tests/test_validation.cpp always used).
Aircraft grossC172() {
    Aircraft ac(c172Def());
    ac.setStationMass(0, 170);
    ac.setStationMass(1, c172Def().maxTakeoffMass - ac.massProperties().mass);
    return ac;
}

// The Top G at `mass` kg: the data file's crew and cabin, fuel split evenly.
Aircraft topGAt(Real mass) {
    Aircraft ac(topGDef());
    const AircraftDefinition& d = topGDef();
    Real onBoard = 0;
    for (std::size_t t = 0; t < d.fuel.tanks.size(); ++t) onBoard += ac.fuel().tankQuantity(static_cast<int>(t)) * d.fuel.density;
    const Real fuel = std::max(mass - (ac.massProperties().mass - onBoard), 0.0);
    const Real tanks = static_cast<Real>(d.fuel.tanks.size());
    for (std::size_t t = 0; t < d.fuel.tanks.size(); ++t)
        ac.fuel().setTankQuantity(static_cast<int>(t), std::min(fuel / tanks / d.fuel.density, d.fuel.tanks[t].capacity));
    ac.refreshMass();
    ac.setGearImmediately(false);  // clean (the Aircraft starts with the gear down)
    return ac;
}

// Take `kg` of fuel out of the tanks, evenly, and refresh the mass.
void burnFuel(Aircraft& ac, Real kg) {
    const AircraftDefinition& d = ac.definition();
    const Real tanks = static_cast<Real>(d.fuel.tanks.size());
    for (std::size_t k = 0; k < d.fuel.tanks.size(); ++k)
        ac.fuel().setTankQuantity(static_cast<int>(k), std::max(ac.fuel().tankQuantity(static_cast<int>(k)) - kg / tanks / d.fuel.density, 0.0));
    ac.refreshMass();
}

void tag(Rows& rows, const char* aircraft) {
    for (Row& r : rows)
        if (r.aircraft.empty()) r.aircraft = aircraft;
}

void append(Rows& out, Rows more) {
    for (Row& r : more) out.push_back(std::move(r));
}

}  // namespace

// ------------------------------------------------------------- C172S ----

Rows c172Stalls() {
    Rows rows;
    const StallResult s0 = stallTest(c172Def(), 0);
    const StallResult s30 = stallTest(c172Def(), 3);
    rows.push_back(tolerance("c172s.stall.clean", "Stall speed, flaps up, power off", "53 KCAS", 53, msToKt(s0.minCas), 6,
                             "KCAS", kPoh, "1 kt/s deceleration at idle, wings level; clean section clmax is the published NACA 2412 value (not fitted)"));
    rows.push_back(tolerance("c172s.stall.flaps30", "Stall speed, flaps 30, power off", "48 KCAS", 48, msToKt(s30.minCas), 6,
                             "KCAS", kPoh, "the flap clmax increment is calibrated to this row"));
    rows.push_back(range("c172s.stall.warning.clean", "Stall warning margin, flaps up", "5-10 kt above the stall", 5, 10,
                         msToKt(s0.warningIas - s0.minIas), "kt", kPoh, "horn on at " + fmt(msToKt(s0.warningIas)) + " KIAS, stall at " + fmt(msToKt(s0.minIas)) + " KIAS"));
    rows.push_back(range("c172s.stall.warning.flaps30", "Stall warning margin, flaps 30", "5-10 kt above the stall", 5, 10,
                         msToKt(s30.warningIas - s30.minIas), "kt", kPoh, "horn on at " + fmt(msToKt(s30.warningIas)) + " KIAS, stall at " + fmt(msToKt(s30.minIas)) + " KIAS"));
    tag(rows, kC172);
    return rows;
}

Rows c172MaxLevelSpeed() {
    Rows rows;
    Aircraft ac = grossC172();
    SimpleWeather atm;
    TrimRequest r;
    r.tas = ktToMs(120);
    r.throttle = 1;
    r.solveTas = true;
    const TrimResult t = trim(ac, r, atm);
    rows.push_back(tolerance("c172s.vmax.sl", "Maximum level speed, sea level", "126 KTAS", 126, t.converged ? msToKt(t.tas) : 0, 5,
                             "KTAS", kPoh, "full throttle, best power; " + fmt(t.rpm, 0) + " rpm"));
    rows.push_back(range("c172s.vmax.rpm", "Engine speed at the maximum level speed", "at most 2,700 rpm (red line)", 0, 2700,
                         t.rpm, "rpm", kPoh));
    tag(rows, kC172);
    return rows;
}

Rows c172FullThrottle8000() {
    Rows rows;
    Aircraft ac = grossC172();
    SimpleWeather atm;
    TrimRequest r;
    r.altitude = ftToM(8000);
    r.tas = ktToMs(120);
    r.throttle = 1;
    r.solveTas = true;
    const TrimResult t = trim(ac, r, atm);
    const Real pct = t.brakePower / c172Def().engines[0].ratedPower * 100;
    const Real gph = m3ToGal(t.fuelFlow / c172Def().fuel.density) * 3600;
    rows.push_back(tolerance("c172s.cruise.8000.full", "Full-throttle level speed, 8,000 ft", "124 KTAS (75 % power, 2,650 rpm)", 124,
                             t.converged ? msToKt(t.tas) : 0, 5, "KTAS", std::string(kPoh) + ", Figure 5-8",
                             fmt(pct, 0) + " % power, " + fmt(t.rpm, 0) + " rpm, propeller efficiency " + fmt(t.propEfficiency, 2) + ", " + fmt(gph) + " gph at best power"));
    rows.push_back(tolerance("c172s.cruise.8000.full.power", "Power at full throttle, 8,000 ft", "75 %", 75, pct, 8, "%",
                             std::string(kPoh) + ", Figure 5-8"));
    rows.push_back(range("c172s.cruise.8000.full.gph", "Fuel flow at full throttle, 8,000 ft", "about 10.5 gph (recommended lean)", 8.5, 12.0, gph,
                         "gph", std::string(kPoh) + ", Figure 5-8", "trimmed at best power (richer than the handbook's recommended lean)"));
    tag(rows, kC172);
    return rows;
}

Rows c172ClimbRates() {
    // The handbook's method: the climb integrated from the rate-of-climb
    // data at each altitude (Figure 5-7 is derived from Figure 5-6), here
    // from trims every 500 ft at the Vy schedule (74 KIAS at sea level to
    // 72 KIAS at 8,000 ft and above), full throttle, best power, the mass
    // falling with the fuel burned.
    Rows rows;
    Aircraft ac = grossC172();
    SimpleWeather atm;
    const AircraftDefinition& d = c172Def();
    struct Pt { Real altFt, roc, ff, vh; };
    std::vector<Pt> pts;
    for (Real h = 0; h <= 12000 + 1e-6; h += 500) {
        const Real kias = h < 8000 ? 74 - 2 * h / 8000 : 72;
        TrimRequest r;
        r.altitude = ftToM(h);
        r.tas = tasFromCas(ktToMs(kias), atm.atmosphere().at(ftToM(h)));
        r.throttle = 1;
        r.solveFlightPath = true;
        const TrimResult t = trim(ac, r, atm);
        pts.push_back({h, t.converged ? t.tas * std::sin(t.flightPath) : 0, t.fuelFlow, t.tas * std::cos(t.flightPath)});
        if (h + 500 <= 12000 && pts.back().roc > 0.1) {
            // Burn the fuel of this 500 ft segment before the next trim.
            const Real dt = ftToM(500) / pts.back().roc;
            const Real burn = t.fuelFlow * dt;  // kg
            burnFuel(ac, burn);
        }
    }
    auto at = [&](Real h) {
        for (const Pt& p : pts)
            if (std::abs(p.altFt - h) < 1) return p;
        return Pt{};
    };
    const struct { Real alt; Real poh; const char* id; } refs[] = {
        {0, 730, "c172s.climb.sl"}, {4000, 600, "c172s.climb.4000"}, {8000, 455, "c172s.climb.8000"}, {12000, 310, "c172s.climb.12000"}};
    for (const auto& x : refs) {
        std::ostringstream q;
        q << "Rate of climb, " << (x.alt == 0 ? std::string("sea level") : fmt(x.alt, 0) + " ft") << ", Vy";
        rows.push_back(tolerance(x.id, q.str(), fmt(x.poh, 0) + " fpm", x.poh, msToFpm(at(x.alt).roc), 15, "fpm",
                                 std::string(kPoh) + ", Figure 5-7 (2,550 lb, standard temperature)",
                                 x.alt == 0 ? "74 KIAS, full throttle" : "72-74 KIAS, full throttle, mixture leaned for best power"));
    }
    // Time, fuel and distance from sea level to 8,000 ft (trapezoids over the 500 ft segments).
    Real time = 0, fuel = 0, dist = 0;
    for (std::size_t i = 0; i + 1 < pts.size() && pts[i + 1].altFt <= 8000 + 1e-6; ++i) {
        const Real roc = 0.5 * (pts[i].roc + pts[i + 1].roc);
        if (roc <= 0.1) break;
        const Real dt = ftToM(pts[i + 1].altFt - pts[i].altFt) / roc;
        time += dt;
        fuel += 0.5 * (pts[i].ff + pts[i + 1].ff) * dt;
        dist += 0.5 * (pts[i].vh + pts[i + 1].vh) * dt;
    }
    const std::string src7 = std::string(kPoh) + ", Figure 5-7 (time, fuel and distance to climb; no start/taxi allowance)";
    rows.push_back(tolerance("c172s.climb.to8000.time", "Time to climb, sea level to 8,000 ft", "14 min", 14, time / 60, 15, "min", src7));
    rows.push_back(tolerance("c172s.climb.to8000.fuel", "Fuel to climb, sea level to 8,000 ft", "3.0 gal", 3.0, m3ToGal(fuel / d.fuel.density), 15, "gal", src7,
                             "the handbook adds 1.4 gal for start, taxi and take-off, not counted here"));
    rows.push_back(tolerance("c172s.climb.to8000.distance", "Distance to climb, sea level to 8,000 ft", "19 NM", 19, dist / kNauticalMile, 15, "NM", src7, "zero wind"));
    tag(rows, kC172);
    return rows;
}

Rows c172CruiseTable() {
    // Figure 5-8 tabulates by rpm; the 55/65/75 % rows are its nearest rows
    // interpolated in % BHP (recommended lean mixture, 2,550 lb, standard
    // temperature). The simulation trims at the equivalence ratio 1.0 (peak
    // EGT, the handbook's recommended lean) with the throttle bisected to the
    // target power; where full throttle cannot reach it (75 % at 8,000 ft) it
    // flies full throttle and says so. Recommended lean is 50 degF rich of
    // peak EGT (POH Section 4), taken as an equivalence ratio of 1.06.
    struct Ref { Real altFt, pct, ktas, gph; };
    const Ref refs[] = {{2000, 55, 100, 7.8}, {2000, 65, 109, 9.1}, {2000, 75, 116, 10.5},
                        {8000, 55, 106, 7.9}, {8000, 65, 116, 9.2}, {8000, 75, 124, 10.5}};
    Rows rows;
    SimpleWeather atm;
    const AircraftDefinition& d = c172Def();
    for (const Ref& x : refs) {
        Aircraft ac = grossC172();
        const Real rho = atm.atmosphere().at(ftToM(x.altFt)).density;
        // Recommended lean (POH Section 4: 50 degF rich of peak EGT), taken as phi = 1.06.
        const Real mixture = bestPowerMixture(d, rho) * (1.06 / 1.15);
        auto at = [&](Real throttle) {
            TrimRequest r;
            r.altitude = ftToM(x.altFt);
            r.tas = ktToMs(110);
            r.throttle = throttle;
            r.solveTas = true;
            r.mixture = mixture;
            return trim(ac, r, atm);
        };
        const Real target = x.pct / 100 * d.engines[0].ratedPower;
        TrimResult t = at(1.0);
        std::string note;
        if (t.brakePower > target) {
            Real lo = 0.2, hi = 1.0;
            for (int it = 0; it < 22; ++it) {
                const Real mid = 0.5 * (lo + hi);
                t = at(mid);
                if (!t.converged) { lo = mid; continue; }
                (t.brakePower > target ? hi : lo) = mid;
            }
            t = at(0.5 * (lo + hi));
        } else {
            note = "full throttle gives only " + fmt(t.brakePower / d.engines[0].ratedPower * 100, 0) + " % here; ";
        }
        note += fmt(t.rpm, 0) + " rpm, throttle " + fmt(t.throttle * 100, 0) + " %, mixture 50 degF rich of peak";
        const std::string base = "c172s.cruise." + fmt(x.altFt, 0) + "." + fmt(x.pct, 0);
        const std::string where = fmt(x.altFt, 0) + " ft, " + fmt(x.pct, 0) + " % power";
        rows.push_back(tolerance(base + ".ktas", "Cruise speed, " + where, fmt(x.ktas, 0) + " KTAS", x.ktas, t.converged ? msToKt(t.tas) : 0, 5, "KTAS",
                                 std::string(kPoh) + ", Figure 5-8 (interpolated in % BHP)", note));
        rows.push_back(tolerance(base + ".gph", "Fuel flow, " + where, fmt(x.gph) + " gph", x.gph, m3ToGal(t.fuelFlow / d.fuel.density) * 3600, 15, "gph",
                                 std::string(kPoh) + ", Figure 5-8 (interpolated in % BHP)", "recommended lean"));
    }
    tag(rows, kC172);
    return rows;
}

Rows c172BestGlide() {
    // Maximum glide: 68 KIAS, flaps up, propeller windmilling (throttle
    // closed, engine idling), zero wind: about 1.5 NM per 1,000 ft (9.1:1).
    // Reported with the propeller's thrust and the airframe's L/D so a
    // discrepancy can be placed (idle power, windmilling drag or airframe).
    Rows rows;
    Aircraft ac = grossC172();
    SimpleWeather atm;
    TrimRequest r;
    r.altitude = ftToM(3000);
    r.tas = tasFromCas(ktToMs(68), atm.atmosphere().at(ftToM(3000)));
    r.throttle = 0;
    r.solveFlightPath = true;
    const TrimResult t = trim(ac, r, atm);
    const Real ratio = t.converged && t.flightPath < 0 ? 1.0 / std::tan(-t.flightPath) : 0;
    rows.push_back(tolerance("c172s.glide.ratio", "Best glide ratio at 68 KIAS", "about 9.1:1 (1.5 NM per 1,000 ft)", 9.1, ratio, 15, ":1",
                             std::string(kPoh) + " (Maximum Glide) and Section 3 (68 KIAS)",
                             "throttle closed, engine idling at " + fmt(t.rpm, 0) + " rpm, 3,000 ft; sink " + fmt(-msToFpm(t.tas * std::sin(t.flightPath)), 0) + " fpm"));
    rows.push_back(info("c172s.glide.nm_per_1000ft", "Glide distance per 1,000 ft", ratio * 1000 * kFoot / kNauticalMile, "NM", "from the ratio above"));
    rows.push_back(info("c172s.glide.prop_thrust", "Propeller thrust in the glide (throttle closed)", t.thrust, "N",
                        "engine brake power " + fmt(wToHp(t.brakePower)) + " hp at " + fmt(t.rpm, 0) + " rpm; + is pulling"));
    {
        // The airframe alone: lift to drag at 68 KIAS with no propeller force (power-off L/D, mass 2,550 lb).
        Aircraft probe = grossC172();
        AeroInputs in;
        const AirState air = atm.atmosphere().at(ftToM(3000));
        in.density = air.density;
        in.speedOfSound = air.speedOfSound;
        in.cg = probe.massProperties().cg;
        const Real v = tasFromCas(ktToMs(68), air);
        const Real w = probe.massProperties().mass * kStandardGravity;
        // Alpha for L = W by bisection on the aero model alone.
        Real lo = 0, hi = degToRad(14), ld = 0;
        AeroOutputs out;
        probe.aeroModel().prepare(out);
        for (int it = 0; it < 40; ++it) {
            const Real a = 0.5 * (lo + hi);
            in.velocityBody = {v * std::cos(a), 0, v * std::sin(a)};
            probe.aeroModel().evaluate(in, out);
            const Real lift = -out.force.z * std::cos(a) + out.force.x * std::sin(a);
            const Real drag = -out.force.x * std::cos(a) - out.force.z * std::sin(a);
            ld = lift / drag;
            (lift < w ? lo : hi) = a;
        }
        rows.push_back(info("c172s.glide.airframe_ld", "Airframe lift/drag at 68 KIAS, no propeller force", ld, ":1", "aero model alone, L = W"));
    }
    tag(rows, kC172);
    return rows;
}

Rows c172Takeoff() {
    Rows rows;
    const TakeoffResult t = takeoffTest(c172Def(), 0, 0, 0, 0, 51);  // POH short field: lift off at 51 KIAS
    rows.push_back(tolerance("c172s.takeoff.roll", "Take-off ground roll (short field)", "960 ft", 960, t.success ? mToFt(t.groundRoll) : 0, 20, "ft",
                             std::string(kPoh) + " (short-field take-off, sea level, ISA)", "flaps 10, lift off at 51 KIAS, climb at 56 KIAS"));
    rows.push_back(tolerance("c172s.takeoff.over50ft", "Take-off distance over a 50 ft obstacle", "1,630 ft", 1630, t.success ? mToFt(t.distanceTo50ft) : 0, 20, "ft",
                             std::string(kPoh) + " (short-field take-off, sea level, ISA)"));
    rows.push_back(range("c172s.takeoff.liftoff", "Lift-off speed", "51 KIAS", 48, 55, msToKt(t.liftoffIas), "KIAS", kPoh));
    tag(rows, kC172);
    return rows;
}

Rows c172LandingRoll() {
    Rows rows;
    const Real roll = mToFt(landingRollTest(c172Def(), 50, kDefaultDt, BrakingTechnique::pilot()));
    const Real limit = mToFt(landingRollTest(c172Def(), 50));
    rows.push_back(tolerance("c172s.landing.roll", "Landing ground roll, pilot braking technique", "575 ft", 575, roll, 25, "ft",
                             std::string(kPoh) + " (short-field landing, sea level, ISA)",
                             "touchdown at 50 KCAS, 1 s to the pedals, 1 s ramp, 70 % sustained pedal (no anti-skid); see the discussion in the validation note"));
    rows.push_back(info("c172s.landing.tyre_limit", "Landing roll, tyre-limited bound (full pedal at touchdown)", limit, "ft", "physics check, not a handbook figure"));
    tag(rows, kC172);
    return rows;
}

Rows c172StaticRpm() {
    Rows rows;
    Aircraft ac(c172Def());
    SimpleWeather atm;
    FlatGround g(0);
    ac.setEngineRunning(true, 1000);
    ac.placeOnGround(0, 0, 0, g, atm);
    PilotInput in;
    in.throttle = 1;
    for (int i = 0; i < 6 * 240; ++i) ac.step(kDefaultDt, in, g, atm);
    rows.push_back(range("c172s.static_rpm", "Static rpm, full throttle", "2,300-2,420 rpm", 2300, 2420, ac.telemetry().engines[0].rpm, "rpm",
                         "C172S POH Section 4 (full-throttle static check)"));
    tag(rows, kC172);
    return rows;
}

Rows c172All() {
    Rows rows;
    append(rows, c172Stalls());
    append(rows, c172MaxLevelSpeed());
    append(rows, c172FullThrottle8000());
    append(rows, c172ClimbRates());
    append(rows, c172CruiseTable());
    append(rows, c172BestGlide());
    append(rows, c172Takeoff());
    append(rows, c172LandingRoll());
    append(rows, c172StaticRpm());
    return rows;
}

// ------------------------------------------------------------- Top G ----

Rows topGTimeToClimb() {
    // The climb from sea level to FL410 integrated from trims every 1,000 ft
    // at a representative schedule (250 KIAS to 10,000 ft, 300 KIAS to the
    // crossover, then Mach 0.80), take-off thrust, the data file's mid
    // weight (33.7 t), mass falling with the fuel burned.
    Rows rows;
    Aircraft ac = topGAt(33700);
    const Real m0 = ac.massProperties().mass;
    SimpleWeather atm;
    struct Pt { Real altFt, roc, ff, vh; };
    std::vector<Pt> pts;
    for (Real h = 0; h <= 41000 + 1e-6; h += 1000) {
        const AirState air = atm.atmosphere().at(ftToM(h));
        const Real kias = h < 10000 ? 250 : 300;
        Real tas = tasFromCas(ktToMs(kias), air);
        if (tas / air.speedOfSound > 0.80) tas = 0.80 * air.speedOfSound;
        TrimRequest r;
        r.altitude = ftToM(h);
        r.tas = tas;
        r.throttle = 1;
        r.solveFlightPath = true;
        const TrimResult t = trim(ac, r, atm);
        pts.push_back({h, t.converged ? t.tas * std::sin(t.flightPath) : 0, t.fuelFlow, t.tas * std::cos(t.flightPath)});
        if (h + 1000 <= 41000 && pts.back().roc > 0.1) {
            burnFuel(ac, t.fuelFlow * ftToM(1000) / pts.back().roc);
        }
    }
    Real time = 0, fuel = 0, dist = 0;
    bool reached = true;
    for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
        const Real roc = 0.5 * (pts[i].roc + pts[i + 1].roc);
        if (roc <= 0.1) { reached = false; break; }
        const Real dt = ftToM(pts[i + 1].altFt - pts[i].altFt) / roc;
        time += dt;
        fuel += 0.5 * (pts[i].ff + pts[i + 1].ff) * dt;
        dist += 0.5 * (pts[i].vh + pts[i + 1].vh) * dt;
    }
    const std::string src = "representative, estimated (G650ER class: FL410 as the initial cruise altitude in about 20-25 min)";
    rows.push_back(tolerance("top_g.climb.fl410.time", "Time to climb, sea level to FL410, mid weight", "about 22 min", 22, reached ? time / 60 : 999, 25, "min", src,
                             fmt(m0 / 1000) + " t at brake release; 250/300 KIAS, Mach 0.80; take-off thrust; rate of climb at FL410 " + fmt(msToFpm(pts.back().roc), 0) + " fpm"));
    rows.push_back(info("top_g.climb.fl410.fuel", "Fuel to climb to FL410", fuel, "kg", "integrated from the trims"));
    rows.push_back(info("top_g.climb.fl410.distance", "Distance to climb to FL410", dist / kNauticalMile, "NM", "zero wind"));
    rows.push_back(info("top_g.climb.sl.roc", "Rate of climb at sea level, 250 KIAS, take-off thrust", msToFpm(pts.front().roc), "fpm"));
    tag(rows, kTopG);
    return rows;
}

Rows topGMaxCruise() {
    Rows rows;
    SimpleWeather atm;
    const StandardAtmosphere isa;
    {
        Aircraft ac = topGAt(33700);
        TrimRequest r;
        r.altitude = ftToM(41000);
        r.tas = 0.90 * isa.at(ftToM(41000)).speedOfSound;
        const TrimResult t = trim(ac, r, atm);
        rows.push_back(range("top_g.cruise.fl410.m090", "Thrust needed for Mach 0.90 at FL410, mid weight (fraction of maximum)", "available (high-speed cruise Mach 0.90)", 0, 98,
                             t.converged ? t.throttle * 100 : 999, "%", "Gulfstream G650ER published high-speed cruise Mach 0.90",
                             "33.7 t; fuel flow " + fmt(t.fuelFlow * 3600, 0) + " kg/h, pitch " + fmt(radToDeg(t.theta)) + " deg"));
    }
    {
        Aircraft ac = topGAt(33700);
        TrimRequest r;
        r.altitude = ftToM(45000);
        r.tas = 0.85 * isa.at(ftToM(45000)).speedOfSound;
        const TrimResult t = trim(ac, r, atm);
        rows.push_back(range("top_g.cruise.fl450.m085.fuel", "Fuel flow at Mach 0.85, FL450, mid weight", "about 2,600-3,400 lb/h (class figures)", lbToKg(2200), lbToKg(3800),
                             t.converged ? t.fuelFlow * 3600 : 0, "kg/h", "published long-range cruise burn of the class", "throttle " + fmt(t.throttle * 100, 0) + " %"));
    }
    tag(rows, kTopG);
    return rows;
}

Rows topGVref() {
    // Vref = 1.23 Vs1g with the landing flap (14 CFR 25.125), sea level ISA,
    // from the trimmed maximum lift coefficient the cockpit V-speeds use.
    Rows rows;
    const AircraftDefinition& d = topGDef();
    auto vref = [&](Real mass) {
        Aircraft ac = topGAt(mass);
        const AeroModel model(d);
        const Real cl = trimmedMaxLiftCoefficient(model, d.referenceArea, ac.massProperties().cg, d.controls.flapDetents.back(), 0.2);
        return 1.23 * msToKt(std::sqrt(2 * ac.massProperties().mass * kStandardGravity / (isa::kSeaLevelDensity * d.referenceArea * cl)));
    };
    rows.push_back(range("top_g.vref.27500kg", "Vref (1.23 Vs1g, landing flap) at 27.5 t", "115-125 KIAS at 25-30 t (representative)", 115, 125, vref(27500), "KIAS",
                         "representative, estimated (G650ER class)", "sea level ISA; trimmed CLmax, flaps 39"));
    rows.push_back(range("top_g.vref.mlw", "Vref (1.23 Vs1g, landing flap) at the maximum landing weight, 37.9 t", "110-125 KIAS (representative; published values 110-121 kt)", 110, 125,
                         vref(d.maxLandingMass), "KIAS", "representative, estimated (G650ER class)", "sea level ISA; trimmed CLmax, flaps 39"));
    tag(rows, kTopG);
    return rows;
}

Rows topGLandingDistance() {
    // 14 CFR 25.125 style: from 50 ft over the threshold at Vref on a 3 deg
    // path to a full stop on a dry runway, maximum wheel braking from
    // touchdown, ground spoilers, no reverse-thrust credit; maximum landing
    // weight, sea level ISA, no wind. The instrument-only approach pilot
    // (ApproachPilot) flies the path on a geometric "GPS" indication and
    // flares; the brakes are overridden to full on the ground.
    Rows rows;
    AircraftDefinition d = topGDef();
    for (TurbofanDef& t : d.turbofans) t.reverser.present = false;  // no reverse credit
    Aircraft ac(d, LocalFrame{});
    {
        // Load to the maximum landing weight.
        Real onBoard = 0;
        for (std::size_t t = 0; t < d.fuel.tanks.size(); ++t) onBoard += ac.fuel().tankQuantity(static_cast<int>(t)) * d.fuel.density;
        const Real fuel = std::max(d.maxLandingMass - (ac.massProperties().mass - onBoard), 0.0);
        const Real tanks = static_cast<Real>(d.fuel.tanks.size());
        for (std::size_t t = 0; t < d.fuel.tanks.size(); ++t)
            ac.fuel().setTankQuantity(static_cast<int>(t), std::min(fuel / tanks / d.fuel.density, d.fuel.tanks[t].capacity));
        ac.refreshMass();
    }
    SimpleWeather atm;
    AirfieldGround ground(0);
    RunwayDef rw;
    rw.threshold = {0, 0, 0};
    rw.headingTrue = 0;
    rw.length = 4000;
    rw.width = 45;
    ground.addRunway(rw);
    const int landing = static_cast<int>(d.controls.flapDetents.size()) - 1;
    // Wheel height on the runway (the radio altimeter's zero).
    ac.setEngineRunning(true, 1000);
    ac.placeOnGround(500, 0, 0, ground, atm);
    ac.step(kDefaultDt, PilotInput{}, ground, atm);
    const Real raZero = ac.telemetry().heightAgl;
    // Vref from the trimmed CLmax at this weight.
    const AeroModel model(d);
    const Real cl = trimmedMaxLiftCoefficient(model, d.referenceArea, ac.massProperties().cg, d.controls.flapDetents.back(), 0.2);
    const Real vref = 1.23 * std::sqrt(2 * ac.massProperties().mass * kStandardGravity / (isa::kSeaLevelDensity * d.referenceArea * cl));
    // Trimmed on the 3 deg path 2,500 m before the threshold, landing configuration.
    const Real out = 2500;
    const Real alt = 15.24 + out * std::tan(degToRad(3));
    ac.setGearImmediately(true);
    ac.setFlapsImmediately(landing);
    TrimRequest req;
    req.altitude = alt;
    req.flapDetent = landing;
    req.tas = tasFromCas(vref, atm.atmosphere().at(alt));
    req.flightPath = degToRad(-3);
    const TrimResult tr = trim(ac, req, atm);
    DynamicState s = tr.state;
    s.body.position = {-out, 0, -alt};
    ac.setState(s);
    ac.prepareContext(tr.input, ground, atm);
    ac.settleDownwash();
    ac.setElevatorTrim(tr.trim);
    ac.step(kDefaultDt, tr.input, ground, atm);
    ac.resetInstrumentsToTruth();

    ApproachPilot pilot;
    pilot.source = ApproachPilot::Source::Gps;
    pilot.courseMagnetic = 0;
    pilot.fieldElevation = 0;
    pilot.runwayLength = rw.length;
    pilot.approachIas = vref;
    pilot.interceptAltitude = alt;
    pilot.radioAltimeter = true;
    pilot.raZero = raZero;
    pilot.pitchRef = tr.theta;
    pilot.baseThrottle = tr.input.throttle;
    pilot.controls = tr.input;
    GpsIndication gps;
    pilot.gps = &gps;
    Real x50 = 0, xStop = 0, xTouch = 0, speedAt50 = 0, sink = 0;
    bool at50 = false, touched = false, stopped = false;
    for (int i = 0; i < 240 * 300; ++i) {
        const AircraftTelemetry& t = ac.telemetry();
        const Vec3 p = ac.state().body.position;
        // The geometric path indication: the 3 deg path to the threshold crossing height.
        gps.valid = gps.verticalValid = true;
        gps.crossTrack = p.y;
        gps.verticalDeviation = t.altitudeMsl - (15.24 + std::max(-p.x, 0.0) * std::tan(degToRad(3)));
        PilotInput in = pilot.update(ac, kDefaultDt);
        if (t.onGround) {
            in.brakeLeft = in.brakeRight = 1.0;  // maximum braking from touchdown
            in.throttle = 0;
        }
        ac.step(kDefaultDt, in, ground, atm);
        const Real h = ac.telemetry().heightAgl - raZero;
        if (!at50 && h <= ftToM(50)) {
            at50 = true;
            x50 = ac.state().body.position.x;
            speedAt50 = ac.instruments().readings().indicatedAirspeed;
        }
        if (!touched && ac.telemetry().onGround) {
            touched = true;
            xTouch = ac.state().body.position.x;
            sink = -msToFpm(ac.telemetry().verticalSpeed);
        }
        if (touched && ac.telemetry().groundSpeed < 0.3) {
            stopped = true;
            xStop = ac.state().body.position.x;
            break;
        }
        if (ac.faulted()) break;
    }
    const Real distance = at50 && stopped ? xStop - x50 : 0;
    rows.push_back(tolerance("top_g.landing.distance", "Landing distance from 50 ft, maximum landing weight (37.9 t)", "3,000 ft (914 m), SL, ISA, MLW", 914, distance, 15, "m",
                             "Gulfstream G650ER specification sheet (landing distance, SL, ISA, MLW); brokers quote 2,450-3,000 ft",
                             "Vref " + fmt(msToKt(vref), 0) + " KIAS (" + fmt(msToKt(speedAt50), 0) + " at 50 ft), touchdown " + fmt(xTouch - x50, 0) + " m past the 50 ft point at " +
                                 fmt(sink, 0) + " fpm; full wheel braking from touchdown, ground spoilers, no reverse credit, dry runway"));
    tag(rows, kTopG);
    return rows;
}

Rows topGAll() {
    Rows rows;
    append(rows, topGTimeToClimb());
    append(rows, topGMaxCruise());
    append(rows, topGVref());
    append(rows, topGLandingDistance());
    return rows;
}

std::vector<std::string> allIds() {
    // Kept by hand so a JSON can be checked for completeness without running
    // the (slow) scenarios; the test "validation: the report is complete"
    // compares it to the rows actually produced.
    return {
        "c172s.stall.clean", "c172s.stall.flaps30", "c172s.stall.warning.clean", "c172s.stall.warning.flaps30",
        "c172s.vmax.sl", "c172s.vmax.rpm",
        "c172s.cruise.8000.full", "c172s.cruise.8000.full.power", "c172s.cruise.8000.full.gph",
        "c172s.climb.sl", "c172s.climb.4000", "c172s.climb.8000", "c172s.climb.12000",
        "c172s.climb.to8000.time", "c172s.climb.to8000.fuel", "c172s.climb.to8000.distance",
        "c172s.cruise.2000.55.ktas", "c172s.cruise.2000.55.gph", "c172s.cruise.2000.65.ktas", "c172s.cruise.2000.65.gph",
        "c172s.cruise.2000.75.ktas", "c172s.cruise.2000.75.gph", "c172s.cruise.8000.55.ktas", "c172s.cruise.8000.55.gph",
        "c172s.cruise.8000.65.ktas", "c172s.cruise.8000.65.gph", "c172s.cruise.8000.75.ktas", "c172s.cruise.8000.75.gph",
        "c172s.glide.ratio", "c172s.glide.nm_per_1000ft", "c172s.glide.prop_thrust", "c172s.glide.airframe_ld",
        "c172s.takeoff.roll", "c172s.takeoff.over50ft", "c172s.takeoff.liftoff",
        "c172s.landing.roll", "c172s.landing.tyre_limit",
        "c172s.static_rpm",
        "top_g.climb.fl410.time", "top_g.climb.fl410.fuel", "top_g.climb.fl410.distance", "top_g.climb.sl.roc",
        "top_g.cruise.fl410.m090", "top_g.cruise.fl450.m085.fuel",
        "top_g.vref.27500kg", "top_g.vref.mlw",
        "top_g.landing.distance",
    };
}

// ---------------------------------------------------------------- output ----

namespace {
std::string jsonEscape(const std::string& s) {
    std::string o;
    for (unsigned char c : s) {
        switch (c) {
            case '"': o += "\\\""; break;
            case '\\': o += "\\\\"; break;
            case '\n': o += "\\n"; break;
            case '\t': o += "\\t"; break;
            default:
                if (c < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof buf, "\\u%04x", c);
                    o += buf;
                } else {
                    o += static_cast<char>(c);
                }
        }
    }
    return o;
}
std::string num(Real v) {
    if (!std::isfinite(v)) return "null";
    std::ostringstream os;
    os << std::setprecision(10) << v;
    return os.str();
}
}  // namespace

void writeJson(std::ostream& out, const Rows& in, bool complete) {
    // Canonical order (allIds), whichever order the rows were produced in,
    // so the tests and `avdemo validate` write the same file.
    Rows rows = in;
    const std::vector<std::string> ids = allIds();
    auto rank = [&](const Row& r) {
        for (std::size_t i = 0; i < ids.size(); ++i)
            if (ids[i] == r.id) return i;
        return ids.size();
    };
    std::stable_sort(rows.begin(), rows.end(), [&](const Row& a, const Row& b) { return rank(a) < rank(b); });
    out << "{\n  \"format\": \"avd-validation/1\",\n  \"complete\": " << (complete ? "true" : "false") << ",\n  \"rows\": [\n";
    for (std::size_t i = 0; i < rows.size(); ++i) {
        const Row& r = rows[i];
        out << "    {\"id\": \"" << jsonEscape(r.id) << "\", \"aircraft\": \"" << jsonEscape(r.aircraft) << "\", \"quantity\": \""
            << jsonEscape(r.quantity) << "\", \"published\": \"" << jsonEscape(r.published) << "\", \"source\": \"" << jsonEscape(r.source)
            << "\", \"unit\": \"" << jsonEscape(r.unit) << "\", \"note\": \"" << jsonEscape(r.note) << "\", \"kind\": \""
            << (r.kind == Row::Kind::Tolerance ? "tolerance" : r.kind == Row::Kind::Range ? "range" : "info") << "\", \"reference\": "
            << num(r.reference) << ", \"value\": " << num(r.value) << ", \"tolerance_pct\": " << num(r.tolerancePct) << ", \"low\": "
            << num(r.low) << ", \"high\": " << num(r.high) << ", \"error_pct\": " << num(r.errorPct()) << ", \"pass\": "
            << (r.pass() ? "true" : "false") << "}" << (i + 1 < rows.size() ? "," : "") << "\n";
    }
    out << "  ]\n}\n";
}

void writeMarkdown(std::ostream& out, const Rows& rows) {
    out << "| Quantity | Published | Simulation | Error | Tolerance | Result |\n|---|---|---|---|---|---|\n";
    for (const Row& r : rows) {
        out << "| " << r.quantity << " | " << (r.published.empty() ? "—" : r.published) << " | " << std::fixed << std::setprecision(1) << r.value
            << " " << r.unit << " | ";
        if (r.kind == Row::Kind::Tolerance) out << std::showpos << r.errorPct() << std::noshowpos << " % | ±" << std::setprecision(0) << r.tolerancePct << " % | ";
        else if (r.kind == Row::Kind::Range) out << "— | " << std::setprecision(1) << r.low << "–" << r.high << " " << r.unit << " | ";
        else out << "— | — | ";
        out << (r.kind == Row::Kind::Info ? "info" : r.pass() ? "within" : "**OUTSIDE**") << " |\n";
    }
}

}  // namespace avd::scenarios::validation
