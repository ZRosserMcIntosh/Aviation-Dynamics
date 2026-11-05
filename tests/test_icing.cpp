// Aviation Dynamics — weather: airframe, propeller and carburettor icing, ice
// protection, rain on the runway and in the air, fronts that change the
// temperature and the altimeter setting.
// SPDX-License-Identifier: MIT
#include <cstdio>

#include "avd/aero.hpp"
#include "avd/icing.hpp"
#include "avd/units.hpp"
#include "avd_test.hpp"
#include "scenarios.hpp"

using namespace avd;
using namespace avd::scenarios;
using namespace avd::units;

namespace {

constexpr Real kDt = 1.0 / 240;

const AircraftDefinition& c172() {
    static const AircraftDefinition d = loadReference("tupi_t180");
    return d;
}

// An icing layer: 1000-2500 m stratus with 0.4 g/m^3, on an ISA-15 day
// (about -10 C at 1500 m).
SimpleWeather::Params icingLayer(Real lwc = 0.4, Real dT = -15) {
    SimpleWeather::Params wp;
    wp.conditions.temperatureDeviation = dT;
    wp.cloudBase = 1000;
    wp.cloudTop = 2500;
    wp.cloudLiquidWater = lwc;
    return wp;
}

struct Flight {
    Aircraft ac;
    SimpleWeather atm;
    FlatGround ground{0};
    TestPilot pilot;
    TrimResult trim;
    Flight(const AircraftDefinition& def, Real kias, Real alt, const SimpleWeather::Params& wp) : ac(def), atm(wp) {
        startTrimmedFlight(ac, {kias, alt, 0}, atm, ground, &pilot, &trim);
        pilot.vertical = TestPilot::Vertical::Altitude;  // hold the height on a fixed throttle
    }
    void fly(Real seconds) {
        const int n = static_cast<int>(std::lround(seconds / kDt));
        for (int i = 0; i < n; ++i) ac.step(kDt, pilot.update(ac, kDt), ground, atm);
    }
    Real kcas() const { return msToKt(ac.instruments().truth().calibratedAirspeed); }
    Real rpm() const { return ac.state().shaftOmega[0] * 60 / kTwoPi; }
    Real mapInHg() const { return ac.telemetry().engines[0].engine.manifoldPressure / kInHg; }
};

// 1-g stall CAS (kt) at a weight from a trimmed CLmax.
Real stallKcas(Real mass, Real area, Real cl) { return msToKt(std::sqrt(2 * mass * kStandardGravity / (isa::kSeaLevelDensity * area * cl))); }

}  // namespace

TEST_CASE("icing: droplet collection efficiency and accretion rates of the right order") {
    const AirState air = StandardAtmosphere({-15, isa::kSeaLevelPressure, 0}).at(1500);
    // Langmuir & Blodgett's cylinder: nothing below K0 = 1/8, about a third
    // at K0 ~ 1, most of the drops at large K0 (a thin probe), all of a
    // raindrop's.
    const Real eWing = icing::collectionEfficiency(0.05, 55, 20e-6, air.density, air.dynamicViscosity);
    const Real eProbe = icing::collectionEfficiency(0.006, 55, 20e-6, air.density, air.dynamicViscosity);
    const Real eBody = icing::collectionEfficiency(1.0, 55, 20e-6, air.density, air.dynamicViscosity);
    const Real eRain = icing::collectionEfficiency(0.05, 55, 1.2e-3, air.density, air.dynamicViscosity);
    std::cerr << "  collection efficiency at 55 m/s, 20 um: leading edge (5 cm) " << eWing << ", probe (6 mm) " << eProbe
              << ", 1 m body " << eBody << "; 1.2 mm rain on the edge " << eRain << "\n";
    CHECK_RANGE(eWing, 0.25, 0.55);
    CHECK(eProbe > 0.75);
    CHECK(eBody < 0.05);
    CHECK(eRain > 0.95);
    // LEWICE's stagnation-line beta0 ~ 0.65 for a 0.53 m NACA 0012 at 67 m/s, 20 um.
    const Real d0012 = 2 * icing::leadingEdgeRadius(0.5334, 0.12);
    const Real beta0 = icing::collectionEfficiency(d0012, 67, 20e-6, 1.0, 1.72e-5);
    std::cerr << "  NACA 0012, 0.53 m chord, 67 m/s: beta0 " << beta0 << " (LEWICE ~0.65)\n";
    CHECK_RANGE(beta0, 0.55, 0.75);

    // The C172's wing at 100 KTAS in 0.4 g/m^3 at -10 C: rime, about
    // beta0 LWC V / rho = 0.37 x 4e-4 x 51 / 880 ~ 0.5 mm/min (moderate, 1-3 in/h).
    AirSample s;
    s.state = air;
    s.liquidWaterContent = 0.4;
    const Real v = ktToMs(100);
    const Real dWing = 2 * icing::leadingEdgeRadius(1.47, 0.12);
    const icing::AccretionRates r = icing::leadingEdgeRates(s, v, dWing);
    const Real ts = icing::recoveryTemperature(air.temperature, v);
    const Real hand = icing::collectionEfficiency(dWing, v, 20e-6, air.density, air.dynamicViscosity) * 0.4e-3 * v *
                      icing::freezingFraction(ts) / icing::iceDensity(icing::glazeShare(ts));
    std::cerr << "  C172 wing, 100 KTAS, 0.4 g/m^3, " << kToCelsius(air.temperature) << " C: " << r.growth * 6e4
              << " mm/min (hand " << hand * 6e4 << "), glaze share " << r.glaze << ", class " << icing::intensityClass(r.growth) << "\n";
    CHECK_RANGE(r.growth * 6e4, 0.25, 1.0);
    CHECK_NEAR(r.growth, hand, 0.05 * hand);
    CHECK(r.glaze < 0.1);  // rime at -10 C
    CHECK(icing::intensityClass(r.growth) == 3);
    // The tail (a smaller, thinner leading edge) ices faster for its chord.
    const Real dTail = 2 * icing::leadingEdgeRadius(1.0, 0.10);
    const Real tail = icing::leadingEdgeRates(s, v, dTail).growth;
    std::cerr << "  k/c rate tail/wing: " << (tail / 1.0) / (r.growth / 1.47) << "\n";
    CHECK((tail / 1.0) > 1.5 * (r.growth / 1.47));
    // At -3.5 C it is glaze and slower (the water does not all freeze); at +3.5 C none.
    s.state = StandardAtmosphere({-12, isa::kSeaLevelPressure, 0}).at(1000);
    const icing::AccretionRates g = icing::leadingEdgeRates(s, v, dWing);
    std::cerr << "  at " << kToCelsius(s.state.temperature) << " C: " << g.growth * 6e4 << " mm/min, glaze " << g.glaze << "\n";
    CHECK(g.glaze > 0.8);
    CHECK(g.growth < 0.6 * r.growth);
    s.state = StandardAtmosphere({+5, isa::kSeaLevelPressure, 0}).at(1000);
    CHECK(icing::leadingEdgeRates(s, v, dWing).growth == 0);
    CHECK(icing::leadingEdgeRates(s, v, dWing).loss > 0);  // melting
    // A jet at 250 KTAS is heated 6-7 K by the air it slows: in -4 C air no ice.
    s.state = StandardAtmosphere({+7, isa::kSeaLevelPressure, 0}).at(3000);
    std::cerr << "  -4 C at 250 KTAS: surface " << kToCelsius(icing::recoveryTemperature(s.state.temperature, ktToMs(250))) << " C\n";
    CHECK(icing::leadingEdgeRates(s, ktToMs(250), dWing).growth == 0);
    // Out of cloud in dry air at -10 C the ice sublimates slowly (< 1 mm/h).
    s.state = StandardAtmosphere({-15, isa::kSeaLevelPressure, 0.3}).at(1500);
    s.liquidWaterContent = 0;
    const Real sub = icing::leadingEdgeRates(s, v, dWing).loss;
    std::cerr << "  sublimation at -10 C, 30 % RH: " << sub * 3.6e6 << " mm/h\n";
    CHECK_RANGE(sub * 3.6e6, 0.05, 2.0);
}

TEST_CASE("icing: an iced C172 wing loses trimmed CLmax and stalls several degrees earlier, as in the literature") {
    // Section level: Bragg, Broeren & Blumenthal (2005): roughness 15-35 %,
    // rime shapes 25-40 %, glaze horns 40-60 % of clmax.
    const IceSectionEffect rough = iceSectionEffect(2e-4, 0), rime = iceSectionEffect(0.008, 0), horn = iceSectionEffect(0.02, 1);
    std::cerr << "  clmax loss: roughness (k/c 2e-4) " << 1 - rough.clMaxFactor << ", rime (0.8 %) " << 1 - rime.clMaxFactor
              << ", glaze horns (2 %) " << 1 - horn.clMaxFactor << "; cd0 + " << rough.cdIncrement << " / " << rime.cdIncrement
              << " / " << horn.cdIncrement << "\n";
    CHECK_RANGE(1 - rough.clMaxFactor, 0.15, 0.35);
    CHECK_RANGE(1 - rime.clMaxFactor, 0.25, 0.42);
    CHECK_RANGE(1 - horn.clMaxFactor, 0.40, 0.62);
    CHECK(rime.cdIncrement > 0.01 && horn.cdIncrement > rime.cdIncrement);

    const AircraftDefinition& d = c172();
    AeroModel model(d);
    Aircraft ac(d);
    const Vec3 cg = ac.massProperties().cg;
    const Real clean = trimmedMaxLiftCoefficient(model, d.referenceArea, cg, 0, 0.2);
    std::array<SurfaceIce, 8> ice{};
    // A 10-minute moderate encounter: ~5 mm of rime on the wing, the same on the tail.
    ice[0] = {0.005, 0};
    ice[1] = {0.005, 0};
    ice[2] = {0.005, 0};
    const Real iced = trimmedMaxLiftCoefficient(model, d.referenceArea, cg, 0, 0.2, ice.data());
    const Real cleanFull = trimmedMaxLiftCoefficient(model, d.referenceArea, cg, d.controls.flapDetents.back(), 0.2);
    const Real icedFull = trimmedMaxLiftCoefficient(model, d.referenceArea, cg, d.controls.flapDetents.back(), 0.2, ice.data());
    const Real m = d.maxTakeoffMass;
    const Real vs = stallKcas(m, d.referenceArea, clean), vsIced = stallKcas(m, d.referenceArea, iced);
    std::cerr << "  trimmed CLmax clean " << clean << " -> iced " << iced << " (" << 100 * (1 - iced / clean) << " % loss); Vs1 "
              << vs << " -> " << vsIced << " KCAS; full flap " << cleanFull << " -> " << icedFull << " ("
              << 100 * (1 - icedFull / cleanFull) << " %)\n";
    CHECK_RANGE(1 - iced / clean, 0.15, 0.40);
    CHECK_RANGE(vsIced / vs, 1.08, 1.30);
    // With full flap the iced tail (its clmin down as much) runs out of the
    // download the flapped wing needs before the wing stalls: the trimmed
    // CLmax falls further (tailplane stall territory; NASA's Twin Otter
    // tailplane icing programme, Ratvasky et al. 2000, NASA/TM-2000-209908).
    CHECK_RANGE(1 - icedFull / cleanFull, 0.2, 0.6);

    // The wing's stall angle: the strips report it, and how much the ice took.
    AeroInputs in;
    in.velocityBody = {50, 0, 0};
    in.cg = cg;
    in.ice = ice.data();
    AeroOutputs out;
    model.prepare(out);
    model.evaluate(in, out);
    const int mid = (model.firstStrip(0) + model.endStrip(0)) / 2;
    const StripTelemetry& st = out.strips[static_cast<std::size_t>(mid)];
    std::cerr << "  wing strip stall angle " << radToDeg(st.stallAlpha) << " deg, " << radToDeg(st.iceStallShift)
              << " deg below the clean wing's\n";
    CHECK_RANGE(radToDeg(st.iceStallShift), 3.0, 8.0);
    // Zero ice changes nothing at all.
    std::array<SurfaceIce, 8> none{};
    CHECK(trimmedMaxLiftCoefficient(model, d.referenceArea, cg, 0, 0.2, none.data()) == clean);
}

TEST_CASE("icing: the C172 in a supercooled cloud picks up ice: drag and stall speed rise, the stall warning comes late; it melts in warm air") {
    const SimpleWeather::Params wp = icingLayer();
    Flight f(c172(), 100, 1500, wp);
    const Real ias0 = f.kcas();
    CHECK(!f.ac.icing().iced());
    f.fly(600);  // ten minutes in the layer, altitude held on the trimmed throttle
    const IcingStatus& st = f.ac.icing().status();
    const Real wing = st.surfaces[0].thickness, tail = st.surfaces[1].thickness;
    std::cerr << "  after 10 min at -10 C in 0.4 g/m^3: wing " << wing * 1e3 << " mm (clmax -" << 100 * st.clMaxLoss[0] << " %), tail "
              << tail * 1e3 << " mm (-" << 100 * st.clMaxLoss[1] << " %), propeller " << st.propellerIce[0] * 1e3 << " mm (thrust -"
              << 100 * st.propellerThrustLoss[0] << " %), windshield " << st.windshieldIce * 1e3 << " mm; class " << st.intensity
              << "; IAS " << ias0 << " -> " << f.kcas() << " KCAS\n";
    CHECK_RANGE(wing * 1e3, 2.0, 10.0);
    CHECK(st.clMaxLoss[1] > st.clMaxLoss[0]);  // the tail first
    CHECK_RANGE(st.clMaxLoss[0], 0.2, 0.42);
    CHECK(st.windshieldIce > 0);
    CHECK(f.kcas() < ias0 - 5);  // the drag of the ice on the same power
    CHECK(f.ac.icing().iced());
    // Its stall speed now (the iced airframe's trimmed CLmax) against clean.
    const AircraftDefinition& d = c172();
    AeroModel model(d);
    const Vec3 cg = f.ac.massProperties().cg;
    std::array<SurfaceIce, 8> ice{};
    for (int s = 0; s < 3; ++s) ice[static_cast<std::size_t>(s)] = st.surfaces[static_cast<std::size_t>(s)];
    const Real vsClean = stallKcas(f.ac.massProperties().mass, d.referenceArea, trimmedMaxLiftCoefficient(model, d.referenceArea, cg, 0, 0.2));
    const Real vsIced = stallKcas(f.ac.massProperties().mass, d.referenceArea,
                                  trimmedMaxLiftCoefficient(model, d.referenceArea, cg, 0, 0.2, ice.data()));
    std::cerr << "  Vs1 " << vsClean << " -> " << vsIced << " KCAS\n";
    CHECK(vsIced > vsClean + 4);
    // A slowdown to the stall: the iced wing lets go before its (fixed-angle) reed sounds for long.
    {
        Flight g(c172(), 100, 1500, icingLayer(0));
        g.ac.icing().setSurfaceIce(0, wing, st.surfaces[0].glaze);
        g.ac.icing().setSurfaceIce(1, tail, st.surfaces[1].glaze);
        g.ac.icing().setSurfaceIce(2, st.surfaces[2].thickness, 0);
        Flight h(c172(), 100, 1500, icingLayer(0));
        auto stallRun = [](Flight& x, Real& warnLead) {
            x.pilot.vertical = TestPilot::Vertical::Altitude;
            x.pilot.controls.throttle = 0;
            Real warnAt = -1, stallAt = -1;
            for (int i = 0; i < 240 * 90 && stallAt < 0; ++i) {
                x.ac.step(kDt, x.pilot.update(x.ac, kDt), x.ground, x.atm);
                if (warnAt < 0 && x.ac.telemetry().stallWarning) warnAt = x.ac.time();
                if (x.ac.telemetry().stalledStrips > 4) stallAt = x.ac.time();
            }
            warnLead = warnAt >= 0 && stallAt >= 0 ? stallAt - warnAt : -99;
            return x.kcas();
        };
        Real leadClean = 0, leadIced = 0;
        const Real breakClean = stallRun(h, leadClean), breakIced = stallRun(g, leadIced);
        std::cerr << "  idle slowdown: the stall breaks at " << breakClean << " KCAS clean (warning " << leadClean << " s before), "
                  << breakIced << " KCAS iced (warning " << leadIced << " s before)\n";
        CHECK(breakIced > breakClean + 3);
        CHECK(leadIced < leadClean);
    }
    // Into warm air (+5 C at 1500 m, out of cloud): the ice melts away.
    SimpleWeather::Params warm;
    warm.conditions.temperatureDeviation = 0;
    f.atm = SimpleWeather(warm);
    f.fly(900);
    std::cerr << "  15 min in +5 C clear air: wing " << f.ac.icing().status().surfaces[0].thickness * 1e3 << " mm\n";
    CHECK(f.ac.icing().status().surfaces[0].thickness < 0.3 * wing);
}

TEST_CASE("icing: thermal anti-ice keeps a jet's wing clean for a thrust cost; boots shed the tail's; protection off, it ices") {
    // CJ4: bleed-air wing, de-ice boots on the tail. FL100 at 200 KIAS in a
    // 0.5 g/m^3 layer at about -15 C (ISA-10 at 3000 m).
    SimpleWeather::Params wp = icingLayer(0.5, -10);
    wp.cloudBase = 2000;
    wp.cloudTop = 4000;
    const AircraftDefinition d = loadReference("cj4");
    auto run = [&](bool on) {
        Flight f(d, 200, 3000, wp);
        f.ac.setCockpitControl("anti_ice.wing", on ? 1 : 0);
        f.ac.setCockpitControl("anti_ice.tail", on ? 1 : 0);
        f.fly(300);
        return f.ac.icing().status();
    };
    const IcingStatus off = run(false);
    const IcingStatus on = run(true);
    std::cerr << "  CJ4 5 min at FL100 in 0.5 g/m^3: unprotected wing " << off.surfaces[0].thickness * 1e3 << " mm, tail "
              << off.surfaces[1].thickness * 1e3 << " mm; protected wing " << on.surfaces[0].thickness * 1e3 << " mm (heat "
              << on.protectionHeat[0] << "), tail " << on.surfaces[1].thickness * 1e3 << " mm; detector " << off.iceDetected << "\n";
    CHECK(off.surfaces[0].thickness > 1e-3);
    CHECK(off.surfaces[1].thickness > 1e-3);
    CHECK(off.iceDetected);
    CHECK(on.surfaces[0].thickness < 0.1e-3);
    CHECK(on.protectionHeat[0] > 0.9);
    CHECK(on.surfaces[1].thickness < 0.5 * off.surfaces[1].thickness);  // boots: a cycle's worth over the residual
    // The bleed costs thrust: the same throttle with the wing anti-ice on.
    {
        Flight f(d, 200, 3000, SimpleWeather::Params{});
        const Real a = f.ac.evaluate(f.ac.state()).forceBody.x;
        f.ac.setCockpitControl("anti_ice.wing", 1);
        const Real b = f.ac.evaluate(f.ac.state()).forceBody.x;
        Real thrust = 0;
        for (int i = 0; i < f.ac.turbofanCount(); ++i) thrust += f.ac.turbofanState(i).thrust;
        std::cerr << "  wing anti-ice bleed: net axial force " << a << " -> " << b << " N (thrust " << thrust << " N, "
                  << 100 * (a - b) / thrust << " %)\n";
        CHECK(f.ac.antiIceBleed());
        CHECK_NEAR((a - b) / thrust, d.iceProtection.bleedThrustLoss, 0.005);
    }
    // The C172 has no protection to switch on.
    Aircraft c(c172());
    CHECK(c.cockpitControlIndex("anti_ice.wing") < 0);
    CHECK(c.cockpitControlIndex("engine1.carb_heat") >= 0);
}

TEST_CASE("carburettor ice: the chart's risk band (FAA AC 20-113): cruise to about +20 C, glide to about +28 C, humid air only") {
    const CarburettorDef& cd = c172().engines[0].carburettor;
    CHECK(cd.present);
    // Cruise (MAP 0.75 ambient, FAR 0.08) and glide (closed throttle, MAP 0.3)
    // at 3000 ft, airflow 0.07 / 0.02 kg/s.
    auto rate = [&](Real tc, Real rh, Real map, Real mdot) {
        AtmosphereConditions c;
        const Real alt = ftToM(3000);
        c.temperatureDeviation = tc + 273.15 - StandardAtmosphere::isaTemperature(alt);
        c.relativeHumidity = rh;
        AirSample s;
        s.state = StandardAtmosphere(c).at(alt);
        return icing::carburettorRates(cd, s, mdot, 0.08, map, 0);
    };
    std::printf("    carburettor ice (mg/s) at RH 80 %%:  T(C)  cruise  glide   throat T cruise/glide\n");
    for (Real t : {-20.0, -10.0, -5.0, 0.0, 10.0, 20.0, 25.0, 30.0, 35.0}) {
        const auto cr = rate(t, 0.8, 0.75, 0.07), gl = rate(t, 0.8, 0.3, 0.02);
        std::printf("    %36.0f  %6.2f  %6.2f   %5.1f / %5.1f\n", t, cr.accretion * 1e6, gl.accretion * 1e6, kToCelsius(cr.temperature),
                    kToCelsius(gl.temperature));
    }
    CHECK(rate(10, 0.8, 0.75, 0.07).accretion > 2e-6);
    CHECK(rate(15, 0.8, 0.75, 0.07).accretion > 1e-6);
    CHECK(rate(-5, 0.8, 0.75, 0.07).accretion > 0.3e-6);
    CHECK(rate(27, 0.8, 0.75, 0.07).accretion == 0);   // cruise: none above the low twenties
    CHECK(rate(25, 0.8, 0.3, 0.02).accretion > 0);     // glide: still at +25
    CHECK(rate(35, 0.8, 0.3, 0.02).accretion == 0);
    CHECK(rate(-20, 0.8, 0.75, 0.07).accretion < 0.1 * rate(10, 0.8, 0.75, 0.07).accretion);  // ice crystals pass through
    CHECK(rate(10, 0.1, 0.75, 0.07).accretion < 0.1 * rate(10, 0.8, 0.75, 0.07).accretion);   // dry air: (almost) none
    // Carb heat warms the throat above freezing at cruise.
    AirSample s;
    s.state = StandardAtmosphere().at(ftToM(3000));
    CHECK(icing::carburettorRates(cd, s, 0.07, 0.08, 0.75, 0.8 * cd.heatRise).temperature > 273.15 + 5);
}

TEST_CASE("carburettor ice: in humid air the C172's MAP and rpm fall; not outside the band; carb heat clears it after a rough patch") {
    // Cruise at 3000 ft, +10 C (ISA+4), 85 % RH: the chart's "serious icing at cruise power".
    auto weather = [](Real tc, Real rh) {
        SimpleWeather::Params wp;
        const Real alt = ftToM(3000);
        wp.conditions.temperatureDeviation = tc + 273.15 - StandardAtmosphere::isaTemperature(alt);
        wp.conditions.relativeHumidity = rh;
        return wp;
    };
    Flight f(c172(), 100, ftToM(3000), weather(10, 0.85));
    f.fly(2);
    const Real rpm0 = f.rpm(), map0 = f.mapInHg();
    f.fly(900);
    const IcingStatus& st = f.ac.icing().status();
    const Real rpm1 = f.rpm(), map1 = f.mapInHg();
    std::cerr << "  15 min at +10 C, 85 % RH: carb ice " << st.carburettorIce[0] * 1e3 << " g (blockage " << st.carburettorBlockage[0]
              << ", throat " << kToCelsius(st.carburettorTemperature[0]) << " C); MAP " << map0 << " -> " << map1 << " inHg, rpm "
              << rpm0 << " -> " << rpm1 << "\n";
    CHECK(map1 < map0 - 1.0);
    CHECK(rpm1 < rpm0 - 80);
    CHECK(!f.ac.icing().iced());  // clear air: no airframe ice
    // Outside the band: a hot day, a dry day.
    for (const auto& [tc, rh] : {std::pair<Real, Real>{32, 0.85}, std::pair<Real, Real>{10, 0.1}}) {
        Flight g(c172(), 100, ftToM(3000), weather(tc, rh));
        g.fly(2);
        const Real m0 = g.mapInHg();
        g.fly(900);
        std::cerr << "  15 min at " << tc << " C, " << 100 * rh << " % RH: carb ice " << g.ac.icing().status().carburettorIce[0] * 1e3
                  << " g, MAP " << m0 << " -> " << g.mapInHg() << "\n";
        CHECK(g.mapInHg() > m0 - 0.3);
    }
    // Carb heat: an rpm drop as the warm, thin air comes in, a rough patch
    // as the ice melts through the engine, then the rpm comes back above
    // where the ice had taken it.
    f.ac.setCockpitControl("engine1.carb_heat", 1);
    Real minRpm = 1e9, maxRough = 0;
    for (int k = 0; k < 120 * 4; ++k) {
        f.fly(0.25);
        minRpm = std::min(minRpm, f.rpm());
        maxRough = std::max(maxRough, st.roughness[0]);
    }
    const Real rpmHeat = f.rpm();
    std::cerr << "  carb heat 2 min: ice " << st.carburettorIce[0] * 1e3 << " g, lowest rpm " << minRpm << ", roughness peak " << maxRough
              << ", rpm " << rpmHeat << " (heat rise " << f.ac.icing().inductionTemperatureRise(0) << " K)\n";
    CHECK(st.carburettorIce[0] < 1e-4);
    CHECK(maxRough > 0.2);
    CHECK(rpmHeat > rpm1 + 30);
    CHECK(rpmHeat < rpm0 - 10);  // heated air is thinner: some rpm short of the clean, cold engine
    // Carb heat off again, still in the band: the ice comes back.
    f.ac.setCockpitControl("engine1.carb_heat", 0);
    f.fly(30);
    const Real rpmCold = f.rpm();
    f.fly(270);
    std::cerr << "  carb heat off: rpm " << rpmCold << " after 30 s; 5 min later carb ice " << st.carburettorIce[0] * 1e3 << " g, rpm "
              << f.rpm() << ", MAP " << f.mapInHg() << " inHg at " << f.kcas() << " KCAS\n";
    CHECK(rpmCold > rpm1 + 200);
    CHECK(st.carburettorIce[0] > 0.3e-3);
}

TEST_CASE("carb heat: applied at full throttle on a dry day it costs the C172 75-150 rpm (POH run-up check: a drop)") {
    Aircraft ac(c172());
    SimpleWeather atm;
    FlatGround ground(0);
    ac.setEngineRunning(true, 1000);
    ac.placeOnGround(0, 0, 0, ground, atm);
    ac.setCockpitControl("brakes.parking", 1);
    PilotInput in;
    in.mixture = bestPowerMixture(c172(), atm.atmosphere().at(0).density);
    in.throttle = 1;
    for (int i = 0; i < 240 * 15; ++i) ac.step(kDt, in, ground, atm);
    const Real rpm0 = ac.state().shaftOmega[0] * 60 / kTwoPi;
    ac.setCockpitControl("engine1.carb_heat", 1);
    for (int i = 0; i < 240 * 20; ++i) ac.step(kDt, in, ground, atm);
    const Real rpm1 = ac.state().shaftOmega[0] * 60 / kTwoPi;
    std::cerr << "  static full throttle: " << rpm0 << " rpm, carb heat " << rpm1 << " rpm (drop " << rpm0 - rpm1 << ")\n";
    CHECK_RANGE(rpm0 - rpm1, 40, 160);
}

TEST_CASE("rain: wets the runway (Gallaway film), freezes in sub-zero air, cuts the visibility") {
    // Gallaway (1979) on a 45 m crowned runway.
    const Real w10 = rainWaterFilmDepth(10), w50 = rainWaterFilmDepth(50), w100 = rainWaterFilmDepth(100);
    std::cerr << "  water film: 10 mm/h " << w10 * 1e3 << " mm, 50 mm/h " << w50 * 1e3 << " mm, 100 mm/h " << w100 * 1e3 << " mm\n";
    CHECK_RANGE(w10 * 1e3, 0.4, 1.0);
    CHECK(w50 < 0.003);
    CHECK(w100 >= 0.003);  // standing water: hydroplaning
    CHECK(rainWaterFilmDepth(0) == 0);
    RunwayWetting rw;
    for (int i = 0; i < 600; ++i) rw.update(1, 10);
    std::cerr << "  after 10 min of 10 mm/h: " << rw.depth() * 1e3 << " mm\n";
    CHECK_NEAR(rw.depth(), w10, 0.1 * w10);
    for (int i = 0; i < 3600; ++i) rw.update(1, 0);
    std::cerr << "  an hour after it stopped: " << rw.depth() * 1e3 << " mm\n";
    CHECK(rw.depth() == 0);
    // The wet runway takes the friction (the 25.109 wet curve).
    SurfaceProperties s = surfaces::kAsphalt;
    s.waterDepth = w10;
    CHECK(wetTyreFriction(s, ktToMs(50), 0) < 0.6);

    // In the weather: below the cloud top, everywhere without a layer.
    SimpleWeather::Params wp;
    wp.rainRate = 10;
    wp.cloudBase = 600;
    wp.cloudTop = 3000;
    wp.cloudLiquidWater = 0.3;
    SimpleWeather w(wp);
    const AirSample below = w.sample({}, 300, 0), inCloud = w.sample({}, 1000, 0), above = w.sample({}, 3500, 0);
    CHECK(below.precipitationRate == 10 && inCloud.precipitationRate == 10 && above.precipitationRate == 0);
    const Real visRain = meteorologicalVisibility(below), visCloud = meteorologicalVisibility(inCloud);
    std::cerr << "  visibility: 10 mm/h rain " << visRain << " m, in the cloud " << visCloud << " m, clear "
              << meteorologicalVisibility(above) << " m\n";
    CHECK_RANGE(visRain, 2500, 6000);
    CHECK_RANGE(visCloud, 30, 200);
    CHECK_NEAR(rainLiquidWaterContent(1), 0.089, 1e-9);
    CHECK_RANGE(rainMedianVolumeDiameter(5) * 1e3, 1.1, 1.4);
    // Freezing rain: rain into -5 C air ices the C172 fast, and as glaze.
    SimpleWeather::Params fz;
    fz.conditions.temperatureDeviation = -16;
    fz.rainRate = 4;
    Flight f(c172(), 100, 600, fz);
    f.fly(120);
    const IcingStatus& st = f.ac.icing().status();
    std::cerr << "  2 min of 4 mm/h freezing rain at " << kToCelsius(f.ac.telemetry().air.state.temperature) << " C: wing "
              << st.surfaces[0].thickness * 1e3 << " mm, glaze share " << st.surfaces[0].glaze << "\n";
    CHECK(st.surfaces[0].thickness > 0.5e-3);
    CHECK(st.surfaces[0].glaze > 0.5);
}

TEST_CASE("front: crossing a moving front the OAT and QNH change; from high to low and warm to cold the old altimeter setting reads high") {
    SimpleWeather::Params wp;
    WindShift f;
    f.heading = 0;           // moving north
    f.north = -20000;        // 20 km south at t = 0
    f.speed = 10;
    f.newSpeed = 8;
    f.newFromDirection = degToRad(200);
    f.temperatureChange = -6;
    f.pressureChange = 400;  // +4 hPa behind
    f.pressureTrough = 150;
    f.airmassWidth = 4000;
    wp.windShifts.push_back(f);
    SimpleWeather w(wp);
    const Vec3 here{0, 0, 0};
    const AirSample ahead = w.sample(here, 1000, 0), behind = w.sample(here, 1000, 4000);
    std::cerr << "  at 1000 m: ahead " << kToCelsius(ahead.state.temperature) << " C, QNH " << w.qnhAt(here, 0) / 100 << " hPa; on the line QNH "
              << w.qnhAt(here, 2000) / 100 << " hPa; behind " << kToCelsius(behind.state.temperature) << " C, QNH "
              << w.qnhAt(here, 4000) / 100 << " hPa\n";
    CHECK_NEAR(behind.state.temperature - ahead.state.temperature, -6, 0.05);
    CHECK_NEAR(w.qnhAt(here, 4000) - w.qnhAt(here, 0), 400, 1);
    CHECK(w.qnhAt(here, 2000) < 0.5 * (w.qnhAt(here, 0) + w.qnhAt(here, 4000)) - 100);  // the trough at the line
    // Without an airmass the front changes the wind only.
    SimpleWeather::Params windOnly = wp;
    windOnly.windShifts[0].temperatureChange = windOnly.windShifts[0].pressureChange = windOnly.windShifts[0].pressureTrough = 0;
    CHECK(SimpleWeather(windOnly).sample(here, 1000, 4000).state.temperature == SimpleWeather().sample(here, 1000, 0).state.temperature);

    // A C172 flying south at 4500 ft into a deepening low behind a front
    // (-5 hPa, 6 K colder), the altimeter set ahead of it: both make the old
    // setting read high (true below indicated: "from high to low, look out
    // below"). Behind a cold front proper (+4 hPa, -6 K) the two nearly cancel.
    wp.windShifts[0].pressureChange = -500;
    wp.windShifts[0].pressureTrough = 0;
    Flight fl(c172(), 100, ftToM(4500), wp);
    fl.ac.instruments().setAltimeterSetting(w.qnhAt(fl.ac.state().body.position, 0));
    fl.pilot.targetHeading = kPi;
    fl.pilot.lateral = TestPilot::Lateral::Heading;
    const Real oat0 = fl.ac.telemetry().air.state.temperature;
    fl.fly(2);
    const Real errAhead = fl.ac.instruments().truth().altitudeMsl - fl.ac.instruments().state().indicatedAltitude;
    // The front reaches the airplane after ~ 20 km / (10 + 51) m/s ~ 330 s.
    fl.fly(600);
    const Real oat1 = fl.ac.telemetry().air.state.temperature;
    const Real errBehind = fl.ac.instruments().truth().altitudeMsl - fl.ac.instruments().state().indicatedAltitude;
    std::cerr << "  C172 through the front: OAT " << kToCelsius(oat0) << " -> " << kToCelsius(oat1) << " C; true minus indicated altitude "
              << mToFt(errAhead) << " -> " << mToFt(errBehind) << " ft on the old setting\n";
    CHECK(oat1 < oat0 - 4);
    // -5 hPa ~ 140 ft and the cold column ~ 2 % of the height (~90 ft), both reading high.
    CHECK_RANGE(mToFt(errBehind - errAhead), -320, -150);
}
