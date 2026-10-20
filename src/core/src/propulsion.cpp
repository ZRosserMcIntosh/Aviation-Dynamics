// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
//
// References:
//  - BEM with Prandtl tip loss: Glauert; McCormick, "Aerodynamics,
//    Aeronautics and Flight Mechanics", ch. 6; momentum relation
//    dT = 4 pi r rho F (V + v) v dr, and the angular-momentum relation
//    dQ = 2 pi r^2 rho F (V + v) w dr for the far-wake swirl velocity w
//    (w = 2 a' Omega r, i.e. dQ = 4 pi r^3 rho F (V + v) Omega a' dr); the
//    blade sees half of it (tangential speed Omega r - w/2). Swirl matters
//    most at low advance ratio (take-off), where the blade torque is high.
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

// Everything the propeller tables depend on, as a cache key.
std::vector<Real> tableKey(const PropellerDef& d) {
    const SectionPolar& s = d.section;
    std::vector<Real> k{d.diameter, Real(d.blades), d.pitch, d.hubRadiusFraction, d.kornFactor, Real(d.bemElements),
                        s.liftSlope, s.zeroLiftAlpha, s.clMax, s.clMin, s.cdMin, s.cdQuadratic, s.clAtCdMin, s.cm0,
                        s.stallWidth, s.postStallClDrop, s.flatPlateCd};
    for (const Table1D* t : {&d.chordVsRadius, &d.thicknessVsRadius}) {
        k.push_back(Real(t->size()));
        k.insert(k.end(), t->xs().begin(), t->xs().end());
        k.insert(k.end(), t->ys().begin(), t->ys().end());
    }
    return k;
}

struct PropTables {
    std::vector<Real> key;
    Table2D cT, cQ;
};

Real bestPowerPhi(const Table1D& t) {
    Real best = 1.15, bestP = -1;
    for (std::size_t i = 0; i < t.size(); ++i)
        if (t.ys()[i] > bestP) { bestP = t.ys()[i]; best = t.xs()[i]; }
    return best;
}

}  // namespace

// ---------------------------------------------------------------------------

void Propeller::bem(const PropellerDef& def, Real v, Real omega, Real rho, Real& thrust, Real& torque,
                    Real speedOfSound) {
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

        Real tipLoss = 1;
        // wt: far-wake swirl velocity; the disk sees ut = Omega r - wt / 2.
        auto element = [&](Real vi, Real wt, Real& dT, Real& dQ, Real& dTm, Real& dQm) {
            const Real ua = v + vi;
            const Real ut = omega * r - 0.5 * wt;
            const Real phi = std::atan2(ua, ut);
            const Real w2 = ua * ua + ut * ut;
            // Compressibility: Prandtl-Glauert lift slope and Korn/Lock drag
            // rise at the element Mach number (tips reach M 0.7-0.8).
            Real slope = def.section.liftSlope, dragRise = 0;
            if (speedOfSound > 0 && !def.thicknessVsRadius.empty()) {
                const Real m = std::sqrt(w2) / speedOfSound;
                slope /= std::sqrt(std::max(1 - m * m, 0.09));
                const Real tc = def.thicknessVsRadius(r / R);
                SectionResult s0 = evaluateSection(def.section, theta - phi, slope);
                const Real mdd = def.kornFactor - tc - std::abs(s0.cl) / 10;
                const Real mcrit = mdd - 0.1077;  // (0.1/80)^(1/3)
                if (m > mcrit) dragRise = 20 * std::pow(m - mcrit, 4);
            }
            SectionResult s = evaluateSection(def.section, theta - phi, slope);
            s.cd += dragRise;
            const Real k = B * 0.5 * rho * w2 * chord * dr;
            dT = k * (s.cl * std::cos(phi) - s.cd * std::sin(phi));
            dQ = k * (s.cl * std::sin(phi) + s.cd * std::cos(phi)) * r;
            const Real sphi = std::max(std::abs(std::sin(phi)), 1e-3);
            const Real f = B * (R - r) / (2 * r * sphi);
            const Real F = std::max(2 / kPi * std::acos(std::exp(-f)), 0.02);
            dTm = 4 * kPi * r * rho * F * ua * vi * dr;
            dQm = 2 * kPi * r * r * rho * F * ua * wt * dr;
            tipLoss = F;
        };

        Real dT = 0, dQ = 0, dTm = 0, dQm = 0;
        const Real ut0 = omega * r;
        if (ut0 < 1e-6) {
            // Stopped propeller: pure blade-element drag/lift, no induction.
            element(0, 0, dT, dQ, dTm, dQm);
        } else {
            // Axial induction for a given swirl: bisection on the thrust
            // balance (blade element = momentum).
            auto solveAxial = [&](Real wt) {
                element(0, wt, dT, dQ, dTm, dQm);
                const Real g0 = dT;  // residual at vi = 0 (dTm = 0)
                Real lo, hi;
                if (g0 > 0) {
                    lo = 0;
                    hi = ut0 + std::abs(v);
                } else {
                    lo = v > 0 ? -0.45 * v : 0;
                    hi = 0;
                }
                auto residual = [&](Real vi) {
                    Real a, b, c, d;
                    element(vi, wt, a, b, c, d);
                    return a - c;
                };
                Real glo = residual(lo), ghi = residual(hi);
                Real vi = 0;
                if (glo * ghi < 0) {
                    // Illinois (modified regula falsi): bracketed, superlinear.
                    int side = 0;
                    const Real tol = 1e-7 * (1 + ut0);
                    for (int it = 0; it < 60 && hi - lo > tol; ++it) {
                        const Real x = (lo * ghi - hi * glo) / (ghi - glo);
                        const Real gx = residual(x);
                        if (gx == 0) { lo = hi = x; break; }
                        if ((gx < 0) == (glo < 0)) {
                            lo = x; glo = gx;
                            if (side == -1) ghi *= 0.5;
                            side = -1;
                        } else {
                            hi = x; ghi = gx;
                            if (side == 1) glo *= 0.5;
                            side = 1;
                        }
                        if (std::abs(gx) < 1e-9 * (1 + std::abs(glo) + std::abs(ghi))) { lo = hi = x; break; }
                    }
                    vi = 0.5 * (lo + hi);
                }
                return vi;
            };
            // Swirl from the torque balance (blade torque = wake angular
            // momentum, wt = 2 a' Omega r): bracketed root of
            // g(wt) = wt - dQ / (2 pi r^2 rho F ua dr) on [0, wMax], the axial
            // balance re-solved at each trial swirl, so both inductions
            // satisfy their momentum relations together. A fixed-point
            // iteration oscillates where drag rise dominates the torque (tip
            // Mach > 1) and as ua -> 0 (flow from behind); the bracket does not.
            // wMax keeps ut >= 0.4 Omega r. Windmilling (dQ < 0): no swirl.
            const Real wMax = 1.2 * ut0;
            Real vi = 0;
            auto swirlResidual = [&](Real wt) {
                vi = solveAxial(wt);
                element(vi, wt, dT, dQ, dTm, dQm);
                const Real ua = v + vi;
                if (ua <= 1e-3 || dQ <= 0) return wt;
                return wt - dQ / (2 * kPi * r * r * rho * tipLoss * ua * dr);
            };
            Real lo = 0, hi = wMax;
            Real glo = swirlResidual(lo);
            Real wt = 0;
            if (glo < 0) {
                Real ghi = swirlResidual(hi);
                wt = hi;  // momentum swirl beyond the cap: hold the cap
                if (ghi > 0) {
                    int side = 0;
                    const Real tol = 1e-6 * (1 + ut0);
                    for (int it = 0; it < 60 && hi - lo > tol; ++it) {
                        const Real x = (lo * ghi - hi * glo) / (ghi - glo);
                        const Real gx = swirlResidual(x);
                        if (gx == 0) { lo = hi = x; break; }
                        if (gx < 0) {
                            lo = x; glo = gx;
                            if (side == -1) ghi *= 0.5;
                            side = -1;
                        } else {
                            hi = x; ghi = gx;
                            if (side == 1) glo *= 0.5;
                            side = 1;
                        }
                    }
                    wt = 0.5 * (lo + hi);
                }
            }
            vi = solveAxial(wt);
            element(vi, wt, dT, dQ, dTm, dQm);
        }
        thrust += dT;
        torque += dQ;
    }
}

Propeller::Propeller(const PropellerDef& def) : def_(def) {
    // Solving the tables takes a few hundred ms; every aircraft built from
    // the same propeller definition shares them. Per thread, so no locking;
    // a trivially destructible pointer because wasi-sdk has no thread-exit
    // destructor hook (the small bounded cache lives as long as the thread).
    thread_local std::vector<PropTables>* cachePtr = nullptr;
    if (!cachePtr) cachePtr = new std::vector<PropTables>();
    std::vector<PropTables>& cache = *cachePtr;
    const std::vector<Real> key = tableKey(def_);
    for (const PropTables& t : cache)
        if (t.key == key) {
            cT_ = t.cT;
            cQ_ = t.cQ;
            return;
        }
    // Tables over advance angle beta = atan2(V, 0.7 Omega R), normalised by
    // 0.5 rho W^2 A (thrust) and 0.5 rho W^2 A R (torque).
    const Real R = radius(), A = discArea();
    // Second dimension: Mach number of W (compressibility at the blades).
    // Without thickness data the propeller is incompressible and one Mach
    // column is enough.
    const Real aRef = 340.294;
    std::vector<Real> b, mach, ct, cq;
    for (Real deg = -20; deg <= 90.0001; deg += 1.0) b.push_back(deg * kPi / 180);
    if (def_.thicknessVsRadius.empty()) mach = {0.0, 1.0};
    else mach = {0.0, 0.3, 0.4, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.9, 1.0};
    for (Real beta : b) {
        for (Real m : mach) {
            // Reference |W| (results are normalised); the incompressible
            // table uses 50 m/s as before.
            const Real w = def_.thicknessVsRadius.empty() || m <= 0 ? 50.0 : m * aRef;
            const Real v = w * std::sin(beta);
            const Real omega = std::max(w * std::cos(beta), 0.0) / (0.7 * R);
            Real t, q;
            bem(def_, v, omega, 1.0, t, q, def_.thicknessVsRadius.empty() || m <= 0 ? 0.0 : aRef);
            const Real norm = 0.5 * 1.0 * w * w * A;
            ct.push_back(t / norm);
            cq.push_back(q / (norm * R));
        }
    }
    cT_ = Table2D(b, mach, ct);
    cQ_ = Table2D(std::move(b), std::move(mach), cq);
    if (cache.size() >= 8) cache.erase(cache.begin());
    cache.push_back({key, cT_, cQ_});
}

PropellerForces Propeller::evaluate(Real axialSpeed, Real omega, Real rho, Real speedOfSound) const {
    PropellerForces f;
    const Real R = radius();
    const Real ut = 0.7 * std::max(omega, 0.0) * R;
    const Real w2 = axialSpeed * axialSpeed + ut * ut;
    if (w2 < 1e-10) return f;
    const Real beta = std::atan2(axialSpeed, ut);
    const Real q = 0.5 * rho * w2 * discArea();
    const Real mach = std::sqrt(w2) / std::max(speedOfSound, 1.0);
    f.thrust = cT_(beta, mach) * q;
    f.torque = cQ_(beta, mach) * q * R;
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

// ------------------------------------------------------------- turbofan ----

TurbofanEngine::TurbofanEngine(const TurbofanDef& def) : def_(def) {
    // Spools accelerate slowly near idle and quickly near the top (the core's
    // surge margin limits fuel scheduling at low speed): tau = tauUp (1.6 - n1).
    // Calibrate tauUp so that idle -> 95 % of take-off thrust takes the
    // definition's acceleration time.
    const Real f95 = (0.95 - def_.idleThrustFraction) / (1 - def_.idleThrustFraction);
    Real n95 = def_.idleN1 + (1 - def_.idleN1) * std::pow(f95, 1.0 / def_.thrustExponent);
    n95 = std::min(n95, 0.999);
    auto timeFor = [&](Real tau) {
        Real n = def_.idleN1, t = 0;
        const Real h = 0.002;
        while (n < n95 && t < 120) {
            n += (1.0 - n) / (tau * (1.6 - n)) * h;
            t += h;
        }
        return t;
    };
    Real lo = 0.01, hi = 20;
    for (int i = 0; i < 40; ++i) {
        const Real mid = 0.5 * (lo + hi);
        (timeFor(mid) < def_.accelerationTime ? lo : hi) = mid;
    }
    tauUp_ = 0.5 * (lo + hi);
}

Real TurbofanEngine::fanDiameter() const {
    return def_.fanDiameter > 0 ? def_.fanDiameter : 2 * def_.reverser.cowlRadius;
}

Real TurbofanEngine::windmillDragArea() const {
    if (def_.windmillDragArea >= 0) return def_.windmillDragArea;
    return 0.12 * kPi * sq(0.5 * fanDiameter());
}

Real TurbofanEngine::availableThrust(Real pressure, Real temperature, Real mach) const {
    const Real m = std::max(mach, 0.0);
    const Real k = 1 + 0.2 * m * m;
    const Real delta0 = pressure / isa::kSeaLevelPressure * std::pow(k, 3.5);
    const Real theta0 = temperature / isa::kSeaLevelTemperature * k;
    Real lapse = 1 - def_.lapseMachCoefficient * std::sqrt(m);
    if (theta0 > def_.throttleRatio) lapse -= 3 * (theta0 - def_.throttleRatio) / (1.5 + m);
    return def_.staticThrust * delta0 * std::max(lapse, 0.0);
}

Real TurbofanEngine::thrustFraction(Real n1) const {
    const Real i = def_.idleN1;
    if (n1 <= i) return def_.idleThrustFraction * sq(std::max(n1, 0.0) / i);
    const Real x = clamp((n1 - i) / (1 - i), 0.0, 1.2);
    return def_.idleThrustFraction + (1 - def_.idleThrustFraction) * std::pow(x, def_.thrustExponent);
}

Real TurbofanEngine::n2FromN1(Real n1) const {
    if (n1 <= def_.idleN1) return n1 * def_.idleN2 / def_.idleN1;
    return def_.idleN2 + (1 - def_.idleN2) * std::sqrt((n1 - def_.idleN1) / (1 - def_.idleN1));
}

Real TurbofanEngine::n1FromN2(Real n2) const {
    if (n2 <= def_.idleN2) return n2 * def_.idleN1 / def_.idleN2;
    return def_.idleN1 + (1 - def_.idleN1) * sq((n2 - def_.idleN2) / (1 - def_.idleN2));
}

void TurbofanEngine::evaluate(TurbofanState& s, const TurbofanInputs& in) const {
    const Real thetaAmb = in.ambientTemperature / isa::kSeaLevelTemperature;
    const Real deltaAmb = in.ambientPressure / isa::kSeaLevelPressure;
    const bool lit = s.running || (s.starting && s.n2 >= 0.2 && in.fuelOn && in.fuelAtEngine);
    if (lit) {
        s.thrustFraction = thrustFraction(s.n1);
        s.thrust = availableThrust(in.ambientPressure, in.ambientTemperature, in.mach) * s.thrustFraction;
        const Real tsfc = def_.tsfcStatic * (1 + def_.tsfcMachSlope * std::max(in.mach, 0.0)) * std::sqrt(thetaAmb);
        const Real idleFlow = def_.idleFuelFlow * deltaAmb * std::sqrt(thetaAmb);
        s.fuelFlow = std::max(tsfc * s.thrust, idleFlow * std::min(1.0, s.n2 / def_.idleN2));
        const Real tf = std::max(s.thrustFraction - def_.idleThrustFraction, 0.0) / (1 - def_.idleThrustFraction);
        s.itt = def_.ittIdle + (def_.ittTakeoff - def_.ittIdle) * tf + 0.7 * (in.ambientTemperature - isa::kSeaLevelTemperature);
        if (s.starting) s.itt += 150 * std::max(0.0, 1 - std::abs(s.n2 - 0.35) / 0.2);  // light-off peak
    } else {
        // Unlit: the windmilling fan and core are a drag along the thrust
        // line, D = q CdA with q = 1/2 gamma p M^2 (no density input needed).
        s.thrustFraction = 0;
        s.thrust = -0.7 * in.ambientPressure * sq(std::max(in.mach, 0.0)) * windmillDragArea();
        s.fuelFlow = 0;
    }
    s.oilPressure = s.n2 < 0.05 ? 0.0
                                : def_.oilPressureIdle * std::min(1.0, s.n2 / def_.idleN2) +
                                      (def_.oilPressureMax - def_.oilPressureIdle) *
                                          clamp((s.n2 - def_.idleN2) / (1 - def_.idleN2), 0.0, 1.0);
}

void TurbofanEngine::step(TurbofanState& s, const TurbofanInputs& in, Real dt) const {
    const bool fuel = in.fuelOn && in.fuelAtEngine;
    // Windmilling: ram air turns the fan (and, more slowly, the core) with the
    // engine unlit: ~25 % N1 and ~20 % N2 at cruise Mach.
    const Real windmillN1 = std::min(0.3, 0.32 * std::max(in.mach, 0.0));
    const Real windmillN2 = std::min(0.24, 0.28 * std::max(in.mach, 0.0));
    if (s.running && !fuel) {
        s.running = false;  // shut down or flamed out
        s.starting = false;
    }
    // Start: the starter cranks the core, or in flight a fast enough windmilling
    // core (>= 12 % N2) relights on its own.
    const bool windmillStart = s.n2 >= 0.12 && windmillN2 >= 0.12;
    if (!in.fuelOn) s.startAborted = false;  // fuel switch cycled: the starter may try again
    if (!s.running && in.fuelOn && !s.starting && ((in.starterPowered && !s.startAborted) || windmillStart) && s.n2 < def_.idleN2) {
        s.starting = true;
        s.startTimer = 0;
    }
    if (s.starting && !in.fuelOn) s.starting = false;
    if (s.running) {
        const Real cmd = commandedN1(in.throttle);
        const Real tau = cmd > s.n1 ? tauUp_ * (1.6 - s.n1) : def_.decelerationTime / 3.0;
        s.n1 += (cmd - s.n1) * std::min(dt / tau, 1.0);
        s.n2 = n2FromN1(s.n1);
    } else if (s.starting) {
        // Starter cranks the core to ~25 %, light-off from 20 % with fuel,
        // then the lit core accelerates to idle.
        // Without fire the core is only turned by the starter (to ~25 %) or the
        // ram air, so above that it runs down and the turbine cools; a start
        // that has not lit within the start time is aborted (FADEC no-light-off
        // protection), which also ends a "start" on an engine with no fuel.
        const Real crank = 0.25 / (0.35 * def_.startTime);
        const bool litNow = (s.n2 >= 0.2 || windmillStart) && fuel;
        s.startTimer += dt;
        if (litNow) {
            s.n2 = std::min(s.n2 + (def_.idleN2 - 0.2) / (0.65 * def_.startTime) * dt, def_.idleN2);
        } else if (in.starterPowered && s.n2 < 0.25) {
            s.n2 = std::min(s.n2 + crank * dt, 0.25);
        } else {
            const Real turned = std::max(in.starterPowered ? 0.25 : 0.0, windmillN2);
            s.n2 += (std::min(turned, s.n2) - s.n2) * std::min(dt / 6.0, 1.0);
            s.itt += (in.ambientTemperature - s.itt) * std::min(dt / 12.0, 1.0);
        }
        s.n1 = n1FromN2(s.n2);
        if (s.n2 >= def_.idleN2 - 1e-6) {
            s.running = true;
            s.starting = false;
            s.n1 = def_.idleN1;
        }
        if (!litNow && (!in.starterPowered || s.startTimer > def_.startTime)) {
            s.starting = false;
            s.startAborted = true;  // no automatic retry: cycle the fuel switch
        }
    } else {
        // Spooling down toward windmill speed; ITT falls quickly once the fire is out.
        s.n1 += (windmillN1 - s.n1) * std::min(dt / 6.0, 1.0);
        s.n2 += (windmillN2 - s.n2) * std::min(dt / 6.0, 1.0);
        s.itt += (in.ambientTemperature - s.itt) * std::min(dt / 12.0, 1.0);
    }
    evaluate(s, in);
}

void TurbofanEngine::setSteady(TurbofanState& s, const TurbofanInputs& in) const {
    s.running = true;
    s.starting = false;
    s.startAborted = false;
    s.n1 = commandedN1(in.throttle);
    s.n2 = n2FromN1(s.n1);
    evaluate(s, in);
}

}  // namespace avd
