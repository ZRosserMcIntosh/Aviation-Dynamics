// Aviation Dynamics — scenarios: free-air trim and trimmed-flight spawns.
// SPDX-License-Identifier: MIT
//
// Pure computation (no files, no exceptions), so the browser build compiles
// it too (web/build_wasi.sh).
#include "scenarios.hpp"

#include "avd/units.hpp"

namespace avd::scenarios {

using namespace units;

Real bestPowerMixture(const AircraftDefinition& def, Real rho) {
    if (def.engines.empty()) return 1;
    const Real rich = def.engines[0].mixtureRichPhi * std::sqrt(isa::kSeaLevelDensity / rho);
    return clamp(1.15 / rich, 0.06, 1.0);
}

// ------------------------------------------------------------------ trim ----

namespace {

enum Param { Theta, Elevator, Omega, Throttle, Gamma, Tas };

struct TrimVars {
    Real v[6];
};

bool solve4(Real a[4][4], Real b[4], Real x[4]) {
    int p[4] = {0, 1, 2, 3};
    for (int c = 0; c < 4; ++c) {
        int best = c;
        for (int r = c + 1; r < 4; ++r)
            if (std::abs(a[p[r]][c]) > std::abs(a[p[best]][c])) best = r;
        std::swap(p[c], p[best]);
        if (std::abs(a[p[c]][c]) < 1e-14) return false;
        for (int r = c + 1; r < 4; ++r) {
            const Real f = a[p[r]][c] / a[p[c]][c];
            for (int k = c; k < 4; ++k) a[p[r]][k] -= f * a[p[c]][k];
            b[p[r]] -= f * b[p[c]];
        }
    }
    for (int c = 3; c >= 0; --c) {
        Real s = b[p[c]];
        for (int k = c + 1; k < 4; ++k) s -= a[p[c]][k] * x[k];
        x[c] = s / a[p[c]][c];
    }
    return true;
}

}  // namespace

TrimResult trim(Aircraft& ac, const TrimRequest& req, const AtmosphereField& atm) {
    TrimResult res;
    // Free-air trim: out of ground effect and clear of the ground by definition.
    const FlatGround ground(1e7);
    ac.setEngineRunning(true, 2300);
    ac.setFlapsImmediately(req.flapDetent);
    ac.setElevatorTrim(0);
    ac.setCockpitControl("brakes.parking", 0);
    ac.setQuasiSteadyDownwash(true);

    int unknown[4] = {Theta, Elevator, Omega, Throttle};
    if (req.throttle >= 0) unknown[3] = req.solveTas ? Tas : Gamma;
    TrimVars x{{0.04, 0.0, 2300 * kTwoPi / 60, req.throttle >= 0 ? req.throttle : 0.6, req.flightPath, req.tas}};

    const Real rho = StandardAtmosphere().at(req.altitude).density;
    Real mixture = req.mixture >= 0 ? req.mixture : bestPowerMixture(ac.definition(), rho);

    auto build = [&](const TrimVars& t, DynamicState& s, PilotInput& in) {
        s = DynamicState{};
        s.body.position = {0, 0, ac.frame().origin().height - req.altitude};
        s.body.velocity = {t.v[Tas] * std::cos(t.v[Gamma]), 0, -t.v[Tas] * std::sin(t.v[Gamma])};
        s.body.attitude = Quat::fromEuler(0, t.v[Theta], 0);
        s.shaftOmega[0] = t.v[Omega];
        in = PilotInput{};
        in.pitch = t.v[Elevator];
        in.throttle = t.v[Throttle];
        in.mixture = mixture;
    };
    auto residual = [&](const TrimVars& t, Real r[4]) {
        DynamicState s;
        PilotInput in;
        build(t, s, in);
        ac.setState(s);
        ac.prepareContext(in, ground, atm);
        const Aircraft::Derivatives d = ac.evaluate(s);
        const Vec3 aBody = s.body.attitude.inverseRotate(d.body.velocity);
        r[0] = aBody.x / kStandardGravity;
        r[1] = aBody.z / kStandardGravity;
        r[2] = d.body.omega.y;
        r[3] = d.shaftAccel[0] / 100.0;
    };

    const Real step[6] = {1e-4, 1e-4, 1e-2, 1e-4, 1e-5, 1e-3};
    const Real maxStep[6] = {0.08, 0.2, 30, 0.15, 0.05, 3};
    Real r[4];
    for (int it = 0; it < 80; ++it) {
        residual(x, r);
        res.residual = std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2] + r[3] * r[3]);
        if (res.residual < 1e-7) { res.converged = true; break; }
        Real J[4][4];
        for (int j = 0; j < 4; ++j) {
            TrimVars xp = x;
            xp.v[unknown[j]] += step[unknown[j]];
            Real rp[4];
            residual(xp, rp);
            for (int i = 0; i < 4; ++i) J[i][j] = (rp[i] - r[i]) / step[unknown[j]];
        }
        Real b[4] = {-r[0], -r[1], -r[2], -r[3]}, dx[4];
        if (!solve4(J, b, dx)) break;
        // Backtracking line search on the residual norm.
        Real lambda = 1.0;
        TrimVars best = x;
        Real bestNorm = res.residual;
        for (int ls = 0; ls < 8; ++ls, lambda *= 0.5) {
            TrimVars xt = x;
            for (int j = 0; j < 4; ++j) {
                const int u = unknown[j];
                xt.v[u] += clamp(lambda * dx[j], -maxStep[u], maxStep[u]);
            }
            xt.v[Elevator] = clamp(xt.v[Elevator], -1.0, 1.0);
            xt.v[Throttle] = clamp(xt.v[Throttle], 0.0, 1.0);
            xt.v[Omega] = std::max(xt.v[Omega], 20.0);
            xt.v[Tas] = std::max(xt.v[Tas], 10.0);
            Real rt[4];
            residual(xt, rt);
            const Real n = std::sqrt(rt[0] * rt[0] + rt[1] * rt[1] + rt[2] * rt[2] + rt[3] * rt[3]);
            if (n < bestNorm) {
                best = xt;
                bestNorm = n;
                break;
            }
        }
        if (bestNorm >= res.residual) break;  // stalled
        x = best;
    }
    if (!res.converged && res.residual < 1e-4) res.converged = true;

    DynamicState s;
    PilotInput in;
    build(x, s, in);
    ac.setState(s);
    ac.prepareContext(in, ground, atm);
    ac.evaluate(s);
    ac.setQuasiSteadyDownwash(false);
    ac.settleDownwash();
    ac.setFlightState(s.body);
    DynamicState s2 = ac.state();
    s2.shaftOmega[0] = x.v[Omega];
    ac.setState(s2);
    ac.prepareContext(in, ground, atm);

    // Report engine/prop quantities at the trimmed point.
    res.theta = x.v[Theta];
    res.flightPath = x.v[Gamma];
    res.alpha = x.v[Theta] - x.v[Gamma];
    res.elevator = x.v[Elevator];
    res.throttle = x.v[Throttle];
    res.rpm = x.v[Omega] * 60 / kTwoPi;
    res.tas = x.v[Tas];
    res.state = ac.state();
    res.input = in;
    {
        const PistonEngine& e = ac.engine(0);
        const AirSample a = atm.sample(s.body.position, req.altitude, 0);
        PistonEngineInputs pin;
        pin.throttle = in.throttle;
        pin.mixture = in.mixture;
        pin.magnetos = Magnetos::Both;
        pin.fuelAtEngine = true;
        pin.ambientPressure = a.state.pressure;
        pin.ambientTemperature = a.state.temperature;
        pin.ambientDensity = a.state.density;
        const PistonEngineState es = e.evaluate(pin, x.v[Omega]);
        res.brakePower = es.brakePower;
        res.fuelFlow = es.fuelFlow;
        const PropellerForces pf = ac.propeller(0).evaluate(x.v[Tas] * std::cos(res.alpha), x.v[Omega], a.state.density);
        res.thrust = pf.thrust;
        res.propEfficiency = pf.efficiency;
    }
    return res;
}

void startTrimmedFlight(Aircraft& ac, const FlightSetup& fs, const AtmosphereField& atm, const GroundQuery& ground,
                        TestPilot* pilot, TrimResult* out) {
    const Real rho = StandardAtmosphere().at(fs.altitude).density;
    TrimRequest req;
    req.tas = tasFromCas(ktToMs(fs.kias), rho);
    req.altitude = fs.altitude;
    req.flapDetent = fs.flapDetent;
    const TrimResult tr = trim(ac, req, atm);
    ac.prepareContext(tr.input, ground, atm);
    if (out) *out = tr;
    if (pilot) {
        pilot->resetIntegrators();
        pilot->controls = tr.input;
        pilot->controls.pitch = 0;
        pilot->targetPitch = tr.theta;
        pilot->targetAltitude = fs.altitude;
        pilot->targetIas = ktToMs(fs.kias);
        pilot->targetHeading = 0;
        pilot->vertical = TestPilot::Vertical::Altitude;
        pilot->lateral = TestPilot::Lateral::WingsLevel;
    }
    // Put the trimmed elevator into the trim wheel so the pilot starts hands-off.
    Real elevUp = 0.4;
    for (const auto& s : ac.definition().surfaces)
        for (const auto& p : s.panels)
            for (const auto& c : p.controls)
                if (c.channel == ControlChannel::Elevator) elevUp = c.maxNegative;
    ac.setElevatorTrim(tr.elevator * elevUp / ac.definition().controls.elevatorTrimRange);
    ac.resetInstrumentsToTruth();
}

}  // namespace avd::scenarios
