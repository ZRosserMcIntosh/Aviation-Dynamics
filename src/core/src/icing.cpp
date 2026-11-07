// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
//
// References (see also avd/icing.hpp):
//  - Langmuir, I. & Blodgett, K. B. (1946), "A mathematical investigation of
//    water droplet trajectories", AAF TR 5418: the modified inertia
//    parameter K0 = 1/8 + (lambda/lambda_s)(K - 1/8).
//  - Finstad, K. J., Lozowski, E. P. & Gates, E. M. (1988), "A computational
//    investigation of water droplet trajectories", J. Atmos. Oceanic Technol.
//    5: the fits of the cylinder collection efficiency and lambda/lambda_s.
//  - Messinger, B. L. (1953), "Equilibrium temperature of an unheated icing
//    surface", J. Aeronaut. Sci. 20: the freezing fraction (here a linear
//    band from 0 to -10 C on the recovery temperature).
//  - Bragg, M. B., Broeren, A. P. & Blumenthal, L. A. (2005), "Iced-airfoil
//    aerodynamics", Prog. Aerosp. Sci. 41: clmax, stall and drag of
//    roughness, streamwise (rime) and horn (glaze) ice.
//  - Neel, C. B. & Bright, L. G. (1950), NACA TN 2852 / RM A50H18: 10-19 %
//    loss of propeller efficiency in icing.
//  - FAA AC 20-113 and SAIB CE-09-35: the carburettor icing chart.
//  - Buck, A. L. (1981), J. Appl. Meteorol. 20: vapour pressure over ice.
#include "avd/icing.hpp"

#include <algorithm>
#include <cmath>

#include "avd/aero.hpp"

namespace avd {

namespace {

Real firstOrder(Real x, Real target, Real dt, Real tau) {
    return tau <= 0 ? target : x + (target - x) * (1 - std::exp(-dt / tau));
}

constexpr Real kInchPerHour = 0.0254 / 3600;  // m/s

}  // namespace

IceSectionEffect iceSectionEffect(Real kc, Real glaze) {
    IceSectionEffect e;
    if (!(kc > 0)) return e;
    glaze = clamp(glaze, 0.0, 1.0);
    const Real x = std::sqrt(kc / 2e-4);
    // Horns grow out of glaze once the ice stands about 1 % of the chord proud.
    const Real lossMax = 0.45 + 0.15 * glaze * smoothstep(0.003, 0.015, kc);
    const Real loss = lossMax * x / (1 + x);
    e.clMaxFactor = 1 - loss;
    e.liftSlopeFactor = 1 - 0.25 * loss;
    const Real xd = std::sqrt(kc / 3e-3);
    e.cdIncrement = (0.02 + 0.025 * glaze) * xd / (1 + xd);
    e.cdQuadraticFactor = 1 + loss;
    return e;
}

namespace icing {

Real collectionEfficiency(Real diameter, Real speed, Real droplet, Real airDensity, Real viscosity) {
    if (!(diameter > 0) || !(speed > 0.1) || !(droplet > 0) || !(viscosity > 0)) return 0;
    constexpr Real kWater = 1000;
    const Real k = kWater * droplet * droplet * speed / (9 * viscosity * diameter);
    const Real re = airDensity * speed * droplet / viscosity;
    const Real lambdaRatio = 1 / (0.8388 + 0.001483 * re + 0.1847 * std::sqrt(re));
    const Real k0 = 0.125 + lambdaRatio * (k - 0.125);
    if (k0 <= 0.125) return 0;
    const Real e = 1.066 * std::pow(k0, -0.00616) * std::exp(-1.103 * std::pow(k0, -0.688)) - 0.028;
    return clamp(e, 0.0, 1.0);
}

Real stagnationHeatTransfer(Real diameter, Real speed, Real airDensity, Real viscosity) {
    constexpr Real kConductivity = 0.0241, kPrandtl = 0.71;  // air near 0 C
    const Real d = std::max(diameter, 1e-3);
    const Real re = std::max(airDensity, 0.0) * std::max(speed, 0.0) * d / std::max(viscosity, 1e-6);
    // Natural convection keeps ~10 W/m^2/K at rest.
    return std::max(1.14 * std::sqrt(re) * std::pow(kPrandtl, 0.4) * kConductivity / d, 10.0);
}

Real saturationVapourPressureIce(Real temperature) {
    const Real tc = temperature - 273.15;
    return 611.15 * std::exp((23.036 - tc / 333.7) * (tc / (279.82 + tc)));
}

AccretionRates leadingEdgeRates(const AirSample& air, Real speed, Real diameter) {
    AccretionRates r;
    const AirState& st = air.state;
    const Real ts = recoveryTemperature(st.temperature, speed);
    const Real cloud = std::max(air.liquidWaterContent, 0.0) * 1e-3;  // kg/m^3
    const Real rain = rainLiquidWaterContent(air.precipitationRate) * 1e-3;
    if (st.temperature < 273.15) r.supercooledWater = cloud + rain;
    const Real n = freezingFraction(ts);
    if (n > 0 && cloud + rain > 0) {
        const Real beta = collectionEfficiency(diameter, speed, air.dropletDiameter, st.density, st.dynamicViscosity) * cloud +
                          collectionEfficiency(diameter, speed, rainMedianVolumeDiameter(air.precipitationRate), st.density,
                                               st.dynamicViscosity) * rain;
        r.glaze = glazeShare(ts);
        r.growth = beta * speed * n / iceDensity(r.glaze);
    }
    const Real h = stagnationHeatTransfer(diameter, speed, st.density, st.dynamicViscosity);
    const Real rho = 0.5 * (kRimeDensity + kGlazeDensity);
    if (ts > 273.15) {
        r.loss = h * (ts - 273.15) / (rho * kLatentFusion);  // melting in the airflow
    } else if (cloud + rain <= 0) {
        // Sublimation into unsaturated air: mass transfer h / (rho cp) by the
        // Lewis analogy, on the vapour density difference over ice.
        const Real rv = isa::kGasConstantVapour;
        const Real flux = h / (std::max(st.density, 0.05) * kCpAir) *
                          (saturationVapourPressureIce(ts) / (rv * ts) - st.vapourPressure / (rv * st.temperature));
        r.loss = std::max(flux, 0.0) / rho;
    }
    return r;
}

CarburettorRates carburettorRates(const CarburettorDef& d, const AirSample& air, Real mdot, Real far, Real manifoldRatio,
                                  Real heatRise) {
    CarburettorRates r;
    const AirState& st = air.state;
    const Real t = st.temperature + std::max(heatRise, 0.0);
    far = clamp(far, 0.0, 0.15);
    // Fuel evaporating in the venturi, and the throttle plate's jet (choked
    // below 0.528 of ambient): the plate meets part of its static drop.
    const Real dEvap = d.fuelEvaporation * far * d.fuelLatentHeat / (kCpAir * (1 + far));
    const Real x = clamp(manifoldRatio, 0.528, 1.0);
    const Real dPlate = d.throttleCooling * t * (1 - std::pow(x, 0.2857));
    r.temperature = t - dEvap - dPlate;
    if (!(mdot > 0)) return r;
    if (r.temperature < 273.15) {
        // Water the cooled air can no longer hold (saturation over ice at the
        // throat's pressure), plus cloud or rain water drawn in with it.
        const Real p = st.pressure, e = std::min(st.vapourPressure, 0.5 * p);
        const Real w = 0.622 * e / std::max(p - e, 1.0);
        const Real pc = x * p;
        const Real es = saturationVapourPressureIce(r.temperature);
        const Real ws = 0.622 * es / std::max(pc - es, 1.0);
        const Real liquid = (std::max(air.liquidWaterContent, 0.0) + rainLiquidWaterContent(air.precipitationRate)) * 1e-3 /
                            std::max(st.density, 0.05);
        const Real water = std::max(w - ws, 0.0) + liquid;
        // Below about -25 C at the throat (-10 C outside) the water arrives
        // as ice crystals that pass through rather than stick (the chart's
        // cold limit).
        const Real stick = clamp((r.temperature - 238.15) / 10.0, 0.0, 1.0);
        r.accretion = d.captureFraction * mdot * water * stick * clamp((273.15 - r.temperature) / 2.0, 0.0, 1.0);
    } else {
        r.melt = d.meltEfficiency * mdot * kCpAir * (r.temperature - 273.15) / kLatentFusion;
    }
    return r;
}

int intensityClass(Real rate) {
    if (!(rate > 0)) return 0;
    if (rate < 0.25 * kInchPerHour) return 1;
    if (rate < 1 * kInchPerHour) return 2;
    if (rate < 3 * kInchPerHour) return 3;
    return 4;
}

}  // namespace icing

void IcingModel::configure(const AircraftDefinition& def, const AeroModel& aero) {
    geo_.clear();
    switches_.clear();
    switchThermal_.clear();
    protection_ = def.iceProtection;
    const int n = std::min(aero.surfaceCount(), kIcingMaxSurfaces);
    for (int s = 0; s < n; ++s) {
        SurfaceGeometry g;
        const Real span = aero.surfaceSpan(s);
        g.chord = span > 0 ? std::max(aero.surfaceArea(s) / span, 0.05) : 1.0;
        g.diameter = 2 * icing::leadingEdgeRadius(g.chord, def.surfaces[static_cast<std::size_t>(s)].thicknessRatio);
        geo_.push_back(g);
    }
    for (const SurfaceIceProtectionDef& p : def.iceProtection.surfaces) {
        if (p.surface < 0 || p.surface >= n) continue;
        std::size_t k = 0;
        while (k < switches_.size() && switches_[k] != p.switchName) ++k;
        if (k == switches_.size()) {
            if (static_cast<int>(k) >= kIcingMaxSwitches) continue;
            switches_.push_back(p.switchName);
            switchThermal_.push_back(false);
        }
        SurfaceGeometry& g = geo_[static_cast<std::size_t>(p.surface)];
        g.protection = static_cast<int>(k);
        g.boots = p.type == IceProtectionType::Boots;
        if (!g.boots) switchThermal_[k] = true;
    }
    prop_ = {};
    auto propeller = [&](PropGeometry& pg, const PropellerDef& pd, Real ratedRpm) {
        pg.present = pd.diameter > 0;
        pg.radius = 0.5 * pd.diameter;
        pg.chord = pd.chordVsRadius.empty() ? 0.08 * pd.diameter : std::max(pd.chordVsRadius(0.4), 0.02);
        const Real tc = pd.thicknessVsRadius.empty() ? 0.12 : pd.thicknessVsRadius(0.4);
        pg.diameter = 2 * icing::leadingEdgeRadius(pg.chord, tc);
        pg.ratedOmega = ratedRpm * kTwoPi / 60;
    };
    for (std::size_t e = 0; e < def.engines.size() && e < static_cast<std::size_t>(kIcingMaxEngines); ++e) {
        const PistonEngineDef& ed = def.engines[e];
        propeller(prop_[e], ed.propeller, ed.ratedRpm);
        carb_[e] = ed.carburettor;
        ratedPower_[e] = ed.ratedPower;
    }
    // Turboprops' propellers (an airframe has piston engines or turboprops,
    // indexed alike from 0).
    if (def.engines.empty())
        for (std::size_t e = 0; e < def.turboprops.size() && e < static_cast<std::size_t>(kIcingMaxEngines); ++e)
            propeller(prop_[e], def.turboprops[e].propeller, def.turboprops[e].ratedPropRpm);
    // Electric propeller de-ice: a switch of its own.
    propSwitch_ = -1;
    if (def.iceProtection.propellerDeice && static_cast<int>(switches_.size()) < kIcingMaxSwitches) {
        propSwitch_ = static_cast<int>(switches_.size());
        switches_.push_back("prop");
        switchThermal_.push_back(false);
    }
    status_ = {};
    status_.surfaceCount = n;
    reset();
}

void IcingModel::reset() {
    const int n = status_.surfaceCount;
    status_ = {};
    status_.surfaceCount = n;
    bootTimer_ = {};
    propTimer_ = 0;
    detectorTimer_ = 0;
    refreshEffects();
}

void IcingModel::setSurfaceIce(int s, Real thickness, Real glaze) {
    if (s < 0 || s >= status_.surfaceCount) return;
    status_.surfaces[static_cast<std::size_t>(s)] = {std::max(thickness, 0.0), clamp(glaze, 0.0, 1.0)};
    refreshEffects();
}

void IcingModel::setCarburettorIce(int e, Real kg) {
    if (e < 0 || e >= kIcingMaxEngines) return;
    status_.carburettorIce[static_cast<std::size_t>(e)] = std::max(kg, 0.0);
}

void IcingModel::refreshEffects() {
    iced_ = false;
    for (int s = 0; s < status_.surfaceCount; ++s) {
        const SurfaceIce& ice = status_.surfaces[static_cast<std::size_t>(s)];
        iced_ = iced_ || ice.thickness > 0;
        const Real chord = s < static_cast<int>(geo_.size()) ? geo_[static_cast<std::size_t>(s)].chord : 1.0;
        status_.clMaxLoss[static_cast<std::size_t>(s)] = 1 - iceSectionEffect(ice.thickness / chord, ice.glaze).clMaxFactor;
    }
    for (std::size_t e = 0; e < static_cast<std::size_t>(kIcingMaxEngines); ++e) {
        const Real k = status_.propellerIce[e];
        if (!(k > 0) || !prop_[e].present) {
            propThrust_[e] = propTorque_[e] = 1;
            status_.propellerThrustLoss[e] = 0;
            continue;
        }
        // Blade-section ice (the inboard blade's; the outer blade sheds or
        // stays warm): thrust lost and torque gained growing with k/c to the
        // 10-19 % efficiency loss measured in icing (Neel & Bright), ~13 %
        // at the shedding size.
        const Real x = std::sqrt(k / prop_[e].chord / 5e-3), s = x / (1 + x);
        propThrust_[e] = 1 - 0.2 * s;
        propTorque_[e] = 1 + 0.08 * s;
        status_.propellerThrustLoss[e] = 0.2 * s;
    }
}

void IcingModel::update(Real dt, const Inputs& in) {
    if (!(dt > 0)) return;
    const AirSample& air = in.air;
    const Real v = std::max(in.trueAirspeed, 0.0);
    // Boots: each switch's cycle (inflated for the first 6 s of it).
    status_.bootsInflating = false;
    std::array<bool, kIcingMaxSwitches> shed{};
    for (std::size_t k = 0; k < switches_.size(); ++k) {
        if (static_cast<int>(k) == propSwitch_) continue;
        if (!in.switches[k]) {
            bootTimer_[k] = 0;
            continue;
        }
        if (bootTimer_[k] <= 0) {
            shed[k] = true;
            bootTimer_[k] = protection_.bootCycle;
        }
        bootTimer_[k] -= dt;
        bool hasBoots = false;
        for (const SurfaceGeometry& g : geo_) hasBoots = hasBoots || (g.boots && g.protection == static_cast<int>(k));
        if (hasBoots && bootTimer_[k] > protection_.bootCycle - 6) status_.bootsInflating = true;
    }
    status_.accretionRate = 0;
    status_.supercooledWater = 0;
    const bool water = air.liquidWaterContent > 0 || air.precipitationRate > 0;
    for (int s = 0; s < status_.surfaceCount; ++s) {
        const SurfaceGeometry& g = geo_[static_cast<std::size_t>(s)];
        SurfaceIce& ice = status_.surfaces[static_cast<std::size_t>(s)];
        Real& heat = status_.protectionHeat[static_cast<std::size_t>(s)];
        // Nothing to do on a clean edge in air without water (the usual case).
        if (!water && ice.thickness <= 0 && heat <= 0 && g.protection < 0) continue;
        const icing::AccretionRates r = icing::leadingEdgeRates(air, v, g.diameter);
        if (s == 0) {
            status_.accretionRate = r.growth;
            status_.supercooledWater = r.supercooledWater;
        }
        const bool on = g.protection >= 0 && in.switches[static_cast<std::size_t>(g.protection)];
        Real growth = r.growth, loss = r.loss;
        if (g.protection >= 0 && !g.boots) {
            // Bleed-air leading edge: warms up with the bleed it gets; a warm
            // edge runs wet (no ice), and melts what is there.
            heat = firstOrder(heat, on ? clamp(in.bleed, 0.0, 1.0) : 0.0, dt, protection_.heatTimeConstant);
            growth *= 1 - heat;
            loss += heat * protection_.heatFlux / (icing::iceDensity(ice.glaze) * icing::kLatentFusion);
        } else if (g.boots) {
            heat = on && status_.bootsInflating ? 1.0 : 0.0;
        }
        const Real added = growth * dt;
        if (added > 0) ice.glaze = (ice.glaze * ice.thickness + r.glaze * added) / (ice.thickness + added);
        ice.thickness = std::max(ice.thickness + added - loss * dt, 0.0);
        if (g.boots && shed[static_cast<std::size_t>(g.protection)]) ice.thickness = std::min(ice.thickness, protection_.bootResidual);
        if (ice.thickness <= 0) ice.glaze = 0;
    }
    // Windshield: a raked panel catching about half what a leading edge does
    // (a 0.1 m collector); heated ones stay clear.
    if (water || status_.windshieldIce > 0) {
        const icing::AccretionRates r = icing::leadingEdgeRates(air, v, 0.1);
        status_.windshieldIce = protection_.windshieldHeat ? 0.0 : std::max(status_.windshieldIce + (r.growth - r.loss) * dt, 0.0);
    }
    // Ice detector: a small vibrating probe (6 mm) ices at once; the
    // annunciator holds 60 s after the last detection.
    if (protection_.iceDetector) {
        detectorTimer_ = water && icing::leadingEdgeRates(air, v, 0.006).growth > 0 ? 60.0 : std::max(detectorTimer_ - dt, 0.0);
        status_.iceDetected = detectorTimer_ > 0;
    }
    status_.intensity = icing::intensityClass(status_.accretionRate);
    // Propeller de-ice timer: a cycle ends every propellerDeiceCycle seconds
    // with the switch on (the first after one heating period).
    bool propShed = false;
    if (propSwitch_ >= 0 && in.switches[static_cast<std::size_t>(propSwitch_)]) {
        propTimer_ += dt;
        if (propTimer_ >= protection_.propellerDeiceCycle) {
            propShed = true;
            propTimer_ = 0;
        }
    } else {
        propTimer_ = 0;
    }
    for (std::size_t e = 0; e < static_cast<std::size_t>(kIcingMaxEngines); ++e) {
        const EngineInputs& ei = in.engines[e];
        // Propeller: the inboard blade section (0.4 R) at its helical speed:
        // the kinetic heating (~10 K at 0.6 R in a C172's cruise) keeps the
        // outer blade clear in all but cold cloud, the inboard ices first;
        // ice sheds once centrifugal force beats its adhesion (about 2 % of
        // the chord at the rated speed, more at lower rpm).
        if (prop_[e].present && (water || status_.propellerIce[e] > 0)) {
            const PropGeometry& pg = prop_[e];
            const Real vr = std::hypot(ei.axialSpeed, 0.4 * ei.omega * pg.radius);
            const icing::AccretionRates r = icing::leadingEdgeRates(air, vr, pg.diameter);
            Real& k = status_.propellerIce[e];
            k = std::max(k + (r.growth - r.loss) * dt, 0.0);
            const Real shedAt = 0.02 * pg.chord * std::min(sq(pg.ratedOmega / std::max(ei.omega, 1.0)), 4.0);
            if (k > shedAt) k *= 0.2;
            // Electric de-ice: each heating cycle frees the boot's ice and
            // the blade throws it (once the blade turns).
            if (propShed && ei.omega > 0.2 * pg.ratedOmega) k = 0;
        }
        // Carburettor.
        heatRise_[e] = 0;
        powerFactor_[e] = 1;
        status_.roughness[e] = 0;
        const CarburettorDef& cd = carb_[e];
        if (!cd.present) continue;
        Real& heat = status_.carburettorHeat[e];
        heat = firstOrder(heat, ei.carbHeat ? 1.0 : 0.0, dt, cd.heatTimeConstant);
        // The muff's heat follows the exhaust's: less at low power.
        heatRise_[e] = heat * cd.heatRise * (0.3 + 0.7 * clamp(ei.powerFraction, 0.0, 1.0));
        const Real mdot = std::max(ei.airMassFlow, 0.0);
        const Real far = mdot > 1e-6 ? ei.fuelFlow / mdot : 0.0;
        const icing::CarburettorRates cr = icing::carburettorRates(cd, air, mdot, far, ei.manifoldRatio, heatRise_[e]);
        status_.carburettorTemperature[e] = cr.temperature;
        Real& m = status_.carburettorIce[e];
        const Real melted = std::min(cr.melt * dt, m);
        m = std::max(m + cr.accretion * dt - melted, 0.0);
        // The same ice takes a larger share of a nearly closed plate's gap.
        const Real plate = 1 - std::cos(clamp(ei.throttle, 0.0, 1.0) * 0.5 * kPi);
        status_.carburettorBlockage[e] = clamp(m / cd.iceToClose * (1 + 2 * (1 - plate)), 0.0, 0.95);
        // Meltwater through the cylinders: rough running and lost power.
        const Real meltWater = mdot > 1e-4 ? melted / dt / mdot : 0.0;  // kg per kg of air
        powerFactor_[e] = 1 - clamp(12 * meltWater, 0.0, 0.5);
        status_.roughness[e] = std::max(clamp(meltWater / 0.004, 0.0, 1.0), 0.5 * smoothstep(0.3, 0.8, status_.carburettorBlockage[e]));
    }
    refreshEffects();
}

}  // namespace avd
