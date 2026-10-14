// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
//
// References:
//  - BEM with Prandtl tip loss: Glauert; McCormick, "Aerodynamics,
//    Aeronautics and Flight Mechanics", ch. 6; momentum relation
//    dT = 4 pi r rho F (V + v) v dr. Swirl (a') is neglected (lightly loaded
//    GA propellers: ~1-2% effect), documented in docs/architecture/propulsion.md.
//  - Piston engine air flow and fmep: Heywood, "Internal Combustion Engine
//    Fundamentals" (volumetric efficiency, friction mean effective pressure,
//    equivalence-ratio effects on power).
//  - Carburettor/injector metering richens ~sqrt(rho0/rho) with altitude
//    (fuel metered by the square root of the air-flow pressure drop).
#include "avd/propulsion.hpp"

#include "avd/aero.hpp"

namespace avd {

namespace {

// Relative power at constant air flow vs equivalence ratio (best power ~1.15).
// Shape from Heywood fig. 15-x class data; Estimated.
Table1D defaultPowerVsPhi() {
    return Table1D({0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.15, 1.3, 1.5, 1.7, 1.9, 2.1},
                   {0.0, 0.35, 0.68, 0.84, 0.93, 0.975, 1.0, 0.985, 0.92, 0.80, 0.45, 0.0});
}

Real bestPowerPhi(const Table1D& t) {
    Real best = 1.15, bestP = -1;
    for (std::size_t i = 0; i < t.size(); ++i)
        if (t.ys()[i] > bestP) { bestP = t.ys()[i]; best = t.xs()[i]; }
    return best;
}

}  // namespace

// ---------------------------------------------------------------------------

void Propeller::bem(const PropellerDef& def, Real v, Real omega, Real rho, Real& thrust, Real& torque) {
    const Real R = 0.5 * def.diameter;
    const Real r0 = def.hubRadiusFraction * R;
    const int n = std::max(def.bemElements, 4);
    const Real dr = (R - r0) / n;
    const Real B = def.blades;
    thrust = torque = 0;
    for (int i = 0; i < n; ++i) {
        const Real r = r0 + (i + 0.5) * dr;
        const Real chord = def.chordVsRadius.empty() ? 0.1 * R : def.chordVsRadius(r / R);
        const Real theta = std::atan2(def.pitch, kTwoPi * r);
        const Real ut = omega * r;

        auto element = [&](Real vi, Real& dT, Real& dQ, Real& dTm) {
            const Real ua = v + vi;
            const Real phi = std::atan2(ua, ut);
            const Real w2 = ua * ua + ut * ut;
            const SectionResult s = evaluateSection(def.section, theta - phi, def.section.liftSlope);
            const Real k = B * 0.5 * rho * w2 * chord * dr;
            dT = k * (s.cl * std::cos(phi) - s.cd * std::sin(phi));
            dQ = k * (s.cl * std::sin(phi) + s.cd * std::cos(phi)) * r;
            const Real sphi = std::max(std::abs(std::sin(phi)), 1e-3);
            const Real f = B * (R - r) / (2 * r * sphi);
            const Real F = std::max(2 / kPi * std::acos(std::exp(-f)), 0.02);
            dTm = 4 * kPi * r * rho * F * ua * vi * dr;
        };

        Real dT = 0, dQ = 0, dTm = 0;
        if (ut < 1e-6) {
            // Stopped propeller: pure blade-element drag/lift, no induction.
            element(0, dT, dQ, dTm);
        } else {
            element(0, dT, dQ, dTm);
            const Real g0 = dT;  // residual at vi = 0 (dTm = 0)
            Real lo, hi;
            if (g0 > 0) {
                lo = 0;
                hi = ut + std::abs(v);
            } else {
                lo = v > 0 ? -0.45 * v : 0;
                hi = 0;
            }
            auto residual = [&](Real vi) {
                Real a, b, c;
                element(vi, a, b, c);
                return a - c;
            };
            Real glo = residual(lo), ghi = residual(hi);
            Real vi = 0;
            if (glo * ghi < 0) {
                for (int it = 0; it < 50; ++it) {
                    const Real mid = 0.5 * (lo + hi);
                    const Real gm = residual(mid);
                    if ((gm < 0) == (glo < 0)) { lo = mid; glo = gm; } else { hi = mid; }
                }
                vi = 0.5 * (lo + hi);
            }
            element(vi, dT, dQ, dTm);
        }
        thrust += dT;
        torque += dQ;
    }
}

Propeller::Propeller(const PropellerDef& def) : def_(def) {
    // Tables over advance angle beta = atan2(V, 0.7 Omega R), normalised by
    // 0.5 rho W^2 A (thrust) and 0.5 rho W^2 A R (torque).
    const Real R = radius(), A = discArea();
    std::vector<Real> b, ct, cq;
    for (Real deg = -20; deg <= 90.0001; deg += 1.0) {
        const Real beta = deg * kPi / 180;
        const Real w = 50.0;  // reference |W| (results are normalised)
        const Real v = w * std::sin(beta);
        const Real omega = std::max(w * std::cos(beta), 0.0) / (0.7 * R);
        Real t, q;
        bem(def_, v, omega, 1.0, t, q);
        const Real norm = 0.5 * 1.0 * w * w * A;
        b.push_back(beta);
        ct.push_back(t / norm);
        cq.push_back(q / (norm * R));
    }
    cT_ = Table1D(b, ct);
    cQ_ = Table1D(std::move(b), cq);
}

PropellerForces Propeller::evaluate(Real axialSpeed, Real omega, Real rho) const {
    PropellerForces f;
    const Real R = radius();
    const Real ut = 0.7 * std::max(omega, 0.0) * R;
    const Real w2 = axialSpeed * axialSpeed + ut * ut;
    if (w2 < 1e-10) return f;
    const Real beta = std::atan2(axialSpeed, ut);
    const Real q = 0.5 * rho * w2 * discArea();
    f.thrust = cT_(beta) * q;
    f.torque = cQ_(beta) * q * R;
    const Real n = omega / kTwoPi;
    f.advanceRatio = n > 1e-6 ? axialSpeed / (n * def_.diameter) : 1e9;
    const Real shaft = f.torque * omega;
    f.efficiency = shaft > 1e-6 ? f.thrust * axialSpeed / shaft : 0;
    return f;
}

// ---------------------------------------------------------------------------

PistonEngine::PistonEngine(const PistonEngineDef& def) : def_(def) {
    if (def_.powerVsPhi.empty()) def_.powerVsPhi = defaultPowerVsPhi();
    // Calibrate indicated efficiency: rated brake power at SL ISA, full
    // throttle, rated rpm, best-power mixture.
    const Real omegaR = def_.ratedRpm * kTwoPi / 60;
    const Real map = isa::kSeaLevelPressure * (1 - def_.intakeLossRatio);
    const Real mAir = def_.volumetricEfficiency * map / (isa::kGasConstantDryAir * isa::kSeaLevelTemperature) *
                      def_.displacement * omegaR / (4 * kPi);
    const Real pInd = def_.ratedPower + frictionTorque(omegaR) * omegaR;
    const Real chem = mAir * def_.stoichiometricFar * def_.fuelHeatingValue *
                      def_.powerVsPhi(bestPowerPhi(def_.powerVsPhi));
    etaIndicated_ = chem > 0 ? pInd / chem : 0.3;
}

Real PistonEngine::frictionTorque(Real omega) const {
    const Real omegaR = def_.ratedRpm * kTwoPi / 60;
    const Real fmep = def_.frictionMepAtZero + (def_.frictionMepAtRated - def_.frictionMepAtZero) * sq(omega / omegaR);
    return fmep * def_.displacement / (4 * kPi);
}

PistonEngineState PistonEngine::evaluate(const PistonEngineInputs& in, Real omega) const {
    PistonEngineState s;
    omega = std::max(omega, 0.0);
    s.omega = omega;
    const Real throttle = clamp(in.throttle, 0.0, 1.0);
    const Real ratio = def_.idleManifoldRatio + (1 - def_.intakeLossRatio - def_.idleManifoldRatio) * throttle;
    s.manifoldPressure = in.ambientPressure * ratio;

    const Real rhoIntake = s.manifoldPressure / (isa::kGasConstantDryAir * in.ambientTemperature);
    const Real airPerRad = def_.volumetricEfficiency * rhoIntake * def_.displacement / (4 * kPi);  // kg/rad
    const Real mAir = airPerRad * omega;

    // Metered mixture: lever below 5% is idle cut-off.
    const Real lever = clamp(in.mixture, 0.0, 1.0);
    const bool cutoff = lever < 0.05;
    const Real phi = cutoff ? 0.0
                            : def_.mixtureRichPhi * lever *
                                  std::sqrt(isa::kSeaLevelDensity / std::max(in.ambientDensity, 1e-3));
    s.equivalenceRatio = in.fuelAtEngine ? phi : 0.0;
    const bool fuelFlowing = in.fuelAtEngine && !cutoff;
    s.fuelFlow = fuelFlowing ? mAir * phi * def_.stoichiometricFar : 0.0;

    const bool ignition = in.magnetos != Magnetos::Off;
    const Real relPower = def_.powerVsPhi(s.equivalenceRatio);
    s.firing = ignition && fuelFlowing && relPower > 0 && omega > def_.minimumFiringRpm * kTwoPi / 60;
    s.running = s.firing && omega > 400 * kTwoPi / 60;

    Real indicatedTorque = 0;
    if (s.firing) {
        const Real mag = (in.magnetos == Magnetos::Both || in.magnetos == Magnetos::Start)
                             ? 1.0
                             : def_.singleMagnetoPowerFactor;
        // Torque = eta * (air per radian) * FAR_stoich * LHV * relPower.
        indicatedTorque = etaIndicated_ * airPerRad * def_.stoichiometricFar * def_.fuelHeatingValue * relPower * mag;
    }
    // Friction opposes rotation; smooth through zero speed.
    const Real friction = frictionTorque(omega) * std::tanh(omega / 2.0);
    s.brakeTorque = indicatedTorque - friction;
    s.brakePower = s.brakeTorque * omega;
    if (in.starterPowered && in.magnetos == Magnetos::Start) {
        const Real freeOmega = 450 * kTwoPi / 60;  // starter free-run speed at the crank
        s.starterTorque = def_.starterTorque * std::max(0.0, 1 - omega / freeOmega);
    }
    return s;
}

void PistonEngine::updateThermal(PistonEngineState& s, Real dt, Real coolingAirspeed) const {
    // Lumped first-order models (Estimated; for instruments and future
    // failures, not validated). Temperatures in K.
    const Real powerFrac = std::max(s.brakePower, 0.0) / def_.ratedPower;
    const Real phi = s.equivalenceRatio;
    const Real egtTarget = s.firing ? 700 + 450 * std::exp(-sq((phi - 1.0) / 0.35)) * (0.6 + 0.4 * powerFrac) : 300;
    s.egt += (egtTarget - s.egt) * std::min(dt / 5.0, 1.0);
    const Real cooling = 1 + coolingAirspeed / 30.0;
    const Real chtTarget = 300 + (s.firing ? 300 * (0.3 + powerFrac) / cooling * 1.6 : 0);
    s.cht += (chtTarget - s.cht) * std::min(dt / 120.0, 1.0);
    const Real oilTarget = 300 + (s.firing ? 90 * (0.4 + 0.6 * powerFrac) : 0);
    s.oilTemperature += (oilTarget - s.oilTemperature) * std::min(dt / 300.0, 1.0);
    // Oil pressure: pump delivery ~ rpm, pressure ~ delivery x viscosity
    // (exponential in temperature) until the relief valve opens; the relief
    // valve then regulates with a small droop. Pump priming lag ~1 s.
    const Real rpm = s.omega * 60 / kTwoPi;
    const Real visc = std::exp((360.9 - s.oilTemperature) / def_.oilViscosityTemperatureScale);
    const Real unrestricted = def_.oilPumpPressurePerRpm * rpm * visc;
    const Real relief = def_.oilReliefPressure;
    const Real target = unrestricted < relief ? unrestricted : std::min(relief + 0.25 * (unrestricted - relief), 1.5 * relief);
    s.oilPressure += (target - s.oilPressure) * std::min(dt / 0.8, 1.0);
}

}  // namespace avd
