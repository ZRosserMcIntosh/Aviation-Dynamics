// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
//
// Rigid-body equations (Stevens & Lewis, "Aircraft Control and Simulation",
// 3rd ed., §1.7, flat-Earth form, CG-referenced):
//   m dv/dt = R F_b + m g
//   I domega/dt = M_b - omega x (I omega + H_rotors)
//   dq/dt = 1/2 q (x) (0, omega)
// Variable mass: fuel leaves through the engine at negligible relative
// velocity, so thrust-like mass-flow terms vanish; the dI/dt omega term is
// neglected (|dI/dt| / I ~ 1e-5 /s for a light aircraft). Integrated with
// classical RK4 at a fixed step (see docs/decisions/0003-integrator.md).
#include "avd/aircraft.hpp"

#include <sstream>
#include <stdexcept>

#include "avd/error.hpp"

namespace avd {

namespace {

constexpr Real kMinStarterVoltage = 16.0;

std::string joinErrors(const std::vector<std::string>& e) {
    std::ostringstream os;
    os << "invalid aircraft definition:";
    for (const auto& s : e) os << "\n  " << s;
    return os.str();
}

Real spinSign(const PistonEngineDef& e) { return e.clockwiseFromBehind ? 1.0 : -1.0; }

Real saturate(Real v, Real limit) { return clamp(v, -limit, limit); }

// Tyre friction past the peak: the force falls from mu_peak to mu_slide as the
// bristle demand grows from 1 to 1 + kSlideWidth times the peak force, and its
// direction turns from the bristle's to the sliding velocity's.
constexpr Real kSlideWidth = 0.5;

// "engine<N>.ignition" -> N-1, or -1 (no exceptions on malformed ids).
int engineIndexFromId(const std::string& id) {
    if (id.rfind("engine", 0) != 0) return -1;
    int n = 0;
    std::size_t i = 6;
    for (; i < id.size() && id[i] >= '0' && id[i] <= '9'; ++i) n = n * 10 + (id[i] - '0');
    if (i == 6 || id.compare(i, std::string::npos, ".ignition") != 0) return -1;
    return n - 1;
}

// "engine<N>.fuel" -> N-1 (turbofan fuel control switch), or -1.
int turbofanIndexFromId(const std::string& id) {
    if (id.rfind("engine", 0) != 0) return -1;
    int n = 0;
    std::size_t i = 6;
    for (; i < id.size() && id[i] >= '0' && id[i] <= '9'; ++i) n = n * 10 + (id[i] - '0');
    if (i == 6 || id.compare(i, std::string::npos, ".fuel") != 0) return -1;
    return n - 1;
}

// "electrical.generator<N>" -> N-1, or -1.
int generatorIndexFromId(const std::string& id) {
    const std::string prefix = "electrical.generator";
    if (id.rfind(prefix, 0) != 0 || id.size() == prefix.size()) return -1;
    int n = 0;
    for (std::size_t i = prefix.size(); i < id.size(); ++i) {
        if (id[i] < '0' || id[i] > '9') return -1;
        n = n * 10 + (id[i] - '0');
    }
    return n - 1;
}

// Pump output of an engine-driven hydraulic pump: full from ~40 % of idle
// core speed up (Estimated); a windmilling core (~20 % N2) gives none.
Real turbofanPumpOutput(const TurbofanState& s, const TurbofanDef& d) {
    return clamp((s.n2 - 0.4 * d.idleN2) / (0.6 * d.idleN2), 0.0, 1.0);
}
Real pistonPumpOutput(Real rpm) { return clamp((rpm - 200.0) / 400.0, 0.0, 1.0); }

}  // namespace

Aircraft::Aircraft(const AircraftDefinition& def, const LocalFrame& frame)
    : def_(def),
      frame_(frame),
      fuel_(def.fuel),
      electrical_(def.electrical, def.engines.empty() ? 900 : def.engines[0].alternatorMinRpm),
      vacuum_(def.vacuum),
      instruments_(def.panel, def.pitotStatic.staticErrorVsAlpha, def.vacuum.regulatedSuction) {
    const auto errors = def.validate();
    if (!errors.empty()) AVD_FAIL(std::invalid_argument, joinErrors(errors));
    aeroModel_ = std::make_unique<AeroModel>(def_);
    aeroModel_->prepare(aeroOut_);
    for (const PistonEngineDef& e : def_.engines) {
        engines_.emplace_back(e);
        props_.emplace_back(e.propeller);
    }
    for (const TurbofanDef& e : def_.turbofans) turbofans_.emplace_back(e);
    stationMass_.resize(def_.stations.size());
    for (std::size_t i = 0; i < def_.stations.size(); ++i) stationMass_[i] = def_.stations[i].initial;
    magnetos_.fill(Magnetos::Off);
    refreshMass();
    prepareGear();
    antiSkid_.fill(1.0);
    rebuildCockpit();
    setColdAndDark();
}

// Per-leg tyre and wheel constants from the definition.
void Aircraft::prepareGear() {
    // Static load estimate: the legs ahead of the CG and those behind it share
    // the weight by the moment balance, evenly within each group, at MTOW.
    const Real W = (def_.maxTakeoffMass > 0 ? def_.maxTakeoffMass : mass_.mass) * kStandardGravity;
    const Real xcg = mass_.cg.x;
    Real xf = 0, xa = 0;
    int nf = 0, na = 0;
    for (const GearDef& g : def_.gear) {
        if (g.attach.x > xcg) { xf += g.attach.x; ++nf; }
        else { xa += g.attach.x; ++na; }
    }
    Real shareFront = 0.5;
    if (nf > 0 && na > 0) {
        xf /= nf;
        xa /= na;
        shareFront = clamp((xcg - xa) / (xf - xa), 0.05, 0.95);
    }
    for (std::size_t i = 0; i < def_.gear.size(); ++i) {
        const GearDef& g = def_.gear[i];
        GearRuntime& r = gearRt_[i];
        const Real legs = static_cast<Real>(def_.gear.size());
        const Real est = nf > 0 && na > 0 ? W * (g.attach.x > xcg ? shareFront / nf : (1 - shareFront) / na) : W / legs;
        r.staticLoad = g.staticLoad > 0 ? g.staticLoad : std::max(est, 1.0);
        r.corneringC1 = g.corneringStiffness / (r.staticLoad * (1 - g.corneringLoadSensitivity));
        // Wheel and tyre spin inertia: about 1.4 % of the mass the leg carries
        // (a C172 6.00-6 wheel and tyre ~6 kg; a 15 t bogie ~2 x 100 kg), as a
        // ring at 0.6 of the radius squared.
        r.inertia = g.wheelInertia > 0 ? g.wheelInertia : 0.6 * 0.014 * (r.staticLoad / kStandardGravity) * sq(g.wheelRadius);
        // RK4 stability of the wheel/bristle mode (natural frequency
        // r sqrt(kx / I_w), damping ratio ~0.3 from spinDamping plus the
        // relaxation): the classic RK4 region reaches |dt lambda| ~2.8 near the
        // imaginary axis, so keep dt omega_n <= 2 at the coarsest step this
        // model runs at, 1/120 s in placeOnGround() (1 at the 1/240 s flight
        // step). The floor only acts on a wheel lighter than that.
        r.inertia = std::max(r.inertia, g.longitudinalStiffness * sq(g.wheelRadius) / sq(2.0 * 120));
        r.brakeTorque = g.brakeTorque > 0 ? g.brakeTorque : g.maxBrakeForceCoefficient * r.staticLoad * g.wheelRadius;
        r.spinDamping = 0.6 * std::sqrt(g.longitudinalStiffness * r.inertia) / g.wheelRadius;
        r.heldDamping = 0.05 * g.longitudinalStiffness;
    }
}

Real Aircraft::corneringStiffness(std::size_t leg, Real normalForce) const {
    // C_alpha = c1 N (1 - c2 N / N_ref), held at its maximum (N = N_ref / 2 c2)
    // for heavier loads so an overloaded tyre never loses all its grip.
    const GearDef& g = def_.gear[leg];
    const GearRuntime& r = gearRt_[leg];
    const Real c2 = g.corneringLoadSensitivity;
    const Real n = c2 > 0 ? std::min(normalForce, r.staticLoad / (2 * c2)) : normalForce;
    return std::max(r.corneringC1 * n * (1 - c2 * n / r.staticLoad), 1.0);
}

// ------------------------------------------------------------------ mass ----

void Aircraft::refreshMass() {
    massItems_.clear();
    MassItem empty;
    empty.name = "empty airframe";
    empty.mass = def_.emptyMass;
    empty.position = def_.emptyCg;
    if (def_.emptyIncludesUnusableFuel) {
        // The tanks hold the unusable fuel; take it out of the airframe item.
        Vec3 moment = empty.position * empty.mass;
        for (const FuelTankDef& t : def_.fuel.tanks) {
            const Real m = t.unusable * def_.fuel.density;
            empty.mass -= m;
            moment = moment - t.position * m;
        }
        empty.position = moment * (1.0 / empty.mass);
    }
    empty.inertia = inertiaTensor(def_.emptyInertiaDiag.x, def_.emptyInertiaDiag.y, def_.emptyInertiaDiag.z, def_.emptyIxz);
    massItems_.push_back(empty);
    for (std::size_t i = 0; i < def_.stations.size(); ++i) {
        if (stationMass_[i] <= 0) continue;
        massItems_.push_back({def_.stations[i].name, stationMass_[i], def_.stations[i].position, Mat3::zero()});
    }
    fuel_.appendMassItems(massItems_);
    const Vec3 oldCg = mass_.cg;
    const bool hadMass = mass_.mass > 0;
    mass_ = combine(massItems_);
    if (hadMass) {
        // Keep the airframe continuous: the integrated point is the CG, so a CG
        // shift inside the airframe moves the integrated point, not the structure.
        const Vec3 d = mass_.cg - oldCg;
        s_.body.position += s_.body.attitude.rotate(d);
        s_.body.velocity += s_.body.attitude.rotate(cross(s_.body.omega, d));
    }
}

void Aircraft::setStationMass(int station, Real kg) {
    // No upper clamp: the placard is a limit of approval, not physics. An
    // overloaded station is flagged by the load sheet (avd/loading.hpp).
    stationMass_[station] = std::isfinite(kg) ? std::max(kg, 0.0) : 0.0;
    refreshMass();
}

Vec3 Aircraft::datumPosition() const { return s_.body.position - s_.body.attitude.rotate(mass_.cg); }

bool Aircraft::hasTiller() const {
    for (const GearDef& g : def_.gear)
        if (g.tillerSteerAngle > 0) return true;
    return false;
}

// -------------------------------------------------------------- cockpit ----

void Aircraft::rebuildCockpit() {
    cockpit_.clear();
    cockpit_.push_back({"electrical.battery", "Battery Master Switch", {"OFF", "ON"}, 0});
    const bool gens = electrical_.generatorCount() > 1;
    cockpit_.push_back({"electrical.alternator", gens ? "Generator Master (all generators)" : "Alternator Master Switch", {"OFF", "ON"}, 0});
    if (gens)
        for (int k = 0; k < electrical_.generatorCount(); ++k) {
            const std::string& n = electrical_.generator(k).name;
            cockpit_.push_back({"electrical.generator" + std::to_string(k + 1),
                                (n.empty() ? "Generator " + std::to_string(k + 1) : n) + " Switch", {"OFF", "ON"}, 0});
        }
    cockpit_.push_back({"electrical.avionics", "Avionics Master Switch", {"OFF", "ON"}, 0});
    cockpit_.push_back({"electrical.load_shed", "Load Shed (essential bus only)", {"NORMAL", "SHED"}, 0});
    for (std::size_t i = 0; i < def_.engines.size(); ++i) {
        const std::string n = "engine" + std::to_string(i + 1);
        cockpit_.push_back({n + ".ignition", "Engine " + std::to_string(i + 1) + " Ignition Switch (hold START to crank)",
                            {"OFF", "R", "L", "BOTH"}, 0});
    }
    for (std::size_t i = 0; i < def_.turbofans.size(); ++i) {
        const std::string n = "engine" + std::to_string(i + 1);
        cockpit_.push_back({n + ".fuel", "Engine " + std::to_string(i + 1) + " Fuel Control Switch (RUN starts the engine)",
                            {"CUTOFF", "RUN"}, 0});
    }
    if (!def_.fuel.selector.empty()) {
        CockpitControl c{"fuel.selector", "Fuel Selector Valve", {}, 0};
        for (const auto& p : def_.fuel.selector) c.positions.push_back(p.name);
        cockpit_.push_back(c);
    }
    if (def_.fuel.hasShutoffValve) cockpit_.push_back({"fuel.shutoff", "Fuel Shutoff Valve", {"OFF", "ON"}, 1});
    cockpit_.push_back({"fuel.auxpump", "Electric Fuel Pump", {"OFF", "ON"}, 0});
    {
        CockpitControl c{"flaps.lever", "Wing Flap Lever", {}, 0};
        for (Real d : def_.controls.flapDetents) c.positions.push_back(std::to_string(static_cast<int>(std::lround(d * 180 / kPi))) + " deg");
        cockpit_.push_back(c);
    }
    if (def_.controls.gear.retractable) {
        cockpit_.push_back({"gear.lever", "Landing Gear Lever", {"UP", "DOWN"}, 1});
        cockpit_.push_back({"gear.emergency", "Landing Gear Emergency Release (free fall)", {"NORMAL", "RELEASE"}, 0});
    }
    for (std::size_t k = 0; k < def_.hydraulics.systems.size(); ++k) {
        const HydraulicSystemDef& h = def_.hydraulics.systems[k];
        if (h.electricPump.current > 0)
            cockpit_.push_back({"hydraulics." + h.name + ".pump", "Hydraulic System " + h.name + " Electric Pump", {"OFF", "ON"}, 1});
    }
    if (def_.controls.speedbrake.present) {
        CockpitControl c{"speedbrake.lever", "Speedbrake Lever", {}, 0};
        const int n = def_.controls.speedbrake.detents;
        for (int k = 0; k < n; ++k)
            c.positions.push_back(k == 0 ? "RET" : (k == n - 1 ? "FULL" : std::to_string(k * 100 / (n - 1)) + " %"));
        cockpit_.push_back(c);
    }
    cockpit_.push_back({"brakes.parking", "Parking Brake", {"OFF", "SET"}, 0});
    for (const ElectricalLoadDef& l : def_.electrical.loads)
        cockpit_.push_back({"electrical.load." + l.name, l.label.empty() ? "Circuit Breaker: " + l.name : l.label,
                            {"OFF", "ON"}, l.initiallyOn ? 1 : 0});
}

void Aircraft::syncCockpitPositions() {
    for (CockpitControl& c : cockpit_) c.position = cockpitControl(c.id);
}

int Aircraft::cockpitControlIndex(const std::string& id) const {
    for (std::size_t i = 0; i < cockpit_.size(); ++i)
        if (cockpit_[i].id == id) return static_cast<int>(i);
    return -1;
}

int Aircraft::cockpitControl(const std::string& id) const {
    if (id == "electrical.battery") return electrical_.batterySwitch();
    if (id == "electrical.alternator") return electrical_.alternatorSwitch();
    if (id == "electrical.avionics") return electrical_.avionicsSwitch();
    if (id == "electrical.load_shed") return electrical_.manualLoadShed();
    if (id == "fuel.selector") return fuel_.selector();
    if (id == "fuel.shutoff") return fuel_.shutoffOpen();
    if (id == "fuel.auxpump") return fuel_.auxPumpOn();
    if (id == "flaps.lever") return flapDetent_;
    if (id == "brakes.parking") return parkingBrake_;
    if (id == "gear.lever") return def_.controls.gear.retractable ? (gearDown_ ? 1 : 0) : -1;
    if (id == "gear.emergency") return def_.controls.gear.retractable ? (gearEmergency_ ? 1 : 0) : -1;
    if (const int k = generatorIndexFromId(id); k >= 0) return k < electrical_.generatorCount() ? (electrical_.generatorSwitch(k) ? 1 : 0) : -1;
    if (id.rfind("hydraulics.", 0) == 0)
        for (std::size_t k = 0; k < def_.hydraulics.systems.size(); ++k)
            if (id == "hydraulics." + def_.hydraulics.systems[k].name + ".pump") return hydPumpOn_[k] ? 1 : 0;
    if (id == "speedbrake.lever") return def_.controls.speedbrake.present ? speedbrakeDetent_ : -1;
    if (const int t = turbofanIndexFromId(id); t >= 0) return t < turbofanCount() ? (turboFuel_[t] ? 1 : 0) : -1;
    if (const int e = engineIndexFromId(id); e >= 0 || id.rfind("engine", 0) == 0) {
        if (e < 0 || e >= engineCount()) return -1;
        const Magnetos m = magnetos_[e];
        return m == Magnetos::Start ? 3 : static_cast<int>(m);
    }
    if (id.rfind("electrical.load.", 0) == 0) {
        const int i = electrical_.loadIndex(id.substr(16));
        return i >= 0 ? static_cast<int>(electrical_.loadSwitch(i)) : -1;
    }
    return -1;
}

bool Aircraft::setCockpitControl(const std::string& id, int position) {
    const int idx = cockpitControlIndex(id);
    if (idx < 0 || position < 0 || position >= static_cast<int>(cockpit_[idx].positions.size())) return false;
    if (cockpitControl(id) == position) return true;  // already there: no change, no event
    // Weight on wheels: a solenoid locks the gear handle down on the ground.
    if (id == "gear.lever" && position == 0 && tel_.onGround) return false;
    if (id == "electrical.battery") electrical_.setBatterySwitch(position == 1);
    else if (id == "electrical.alternator") electrical_.setAlternatorSwitch(position == 1);
    else if (id == "electrical.avionics") electrical_.setAvionicsSwitch(position == 1);
    else if (id == "electrical.load_shed") electrical_.setLoadShed(position == 1);
    else if (id == "fuel.selector") fuel_.setSelector(position);
    else if (id == "fuel.shutoff") fuel_.setShutoffOpen(position == 1);
    else if (id == "fuel.auxpump") fuel_.setAuxPump(position == 1);
    else if (id == "flaps.lever") flapDetent_ = position;
    else if (id == "brakes.parking") parkingBrake_ = position == 1;
    else if (id == "gear.lever") gearDown_ = position == 1;
    else if (id == "gear.emergency") gearEmergency_ = position == 1;
    else if (generatorIndexFromId(id) >= 0) electrical_.setGeneratorSwitch(generatorIndexFromId(id), position == 1);
    else if (id.rfind("hydraulics.", 0) == 0) {
        for (std::size_t k = 0; k < def_.hydraulics.systems.size(); ++k)
            if (id == "hydraulics." + def_.hydraulics.systems[k].name + ".pump") hydPumpOn_[k] = position == 1;
    }
    else if (id == "speedbrake.lever") speedbrakeDetent_ = position;
    else if (turbofanIndexFromId(id) >= 0) turboFuel_[turbofanIndexFromId(id)] = position == 1;
    else if (engineIndexFromId(id) >= 0) magnetos_[engineIndexFromId(id)] = static_cast<Magnetos>(position);
    else if (id.rfind("electrical.load.", 0) == 0) electrical_.setLoadSwitch(electrical_.loadIndex(id.substr(16)), position == 1);
    cockpit_[idx].position = position;
    pushEvent(FlightEventType::CockpitControlChanged, position, idx);
    return true;
}

bool Aircraft::cycleCockpitControl(const std::string& id) {
    const int idx = cockpitControlIndex(id);
    if (idx < 0) return false;
    const int n = static_cast<int>(cockpit_[idx].positions.size());
    return setCockpitControl(id, (cockpitControl(id) + 1) % n);
}

// ------------------------------------------------------------ set-up ----

void Aircraft::setFlightState(const RigidBodyState& s) {
    s_ = DynamicState{};
    s_.body = s;
    s_.body.attitude.normalize();
    antiSkid_.fill(1.0);
    pusher_ = false;
    for (int i = 0; i < engineCount(); ++i) s_.shaftOmega[i] = engineState_[i].omega;
    faulted_ = false;
    updateTelemetry();
}

void Aircraft::setColdAndDark() {
    electrical_.setBatterySwitch(false);
    electrical_.setAlternatorSwitch(false);
    electrical_.setAvionicsSwitch(false);
    fuel_.setAuxPump(false);
    fuel_.setShutoffOpen(true);
    fuel_.setSelector(def_.fuel.initialSelector);
    for (int i = 0; i < engineCount(); ++i) {
        magnetos_[i] = Magnetos::Off;
        s_.shaftOmega[i] = 0;
        engineState_[i] = {};
        runningWas_[i] = false;
    }
    for (int i = 0; i < turbofanCount(); ++i) {
        turboFuel_[i] = false;
        turboState_[i] = {};
        runningWas_[i] = false;
    }
    gearDown_ = true;
    gearPos_ = 1;
    speedbrakeDetent_ = 0;
    speedbrakePos_ = 0;
    groundSpoilers_ = false;
    reverserPos_ = {};
    reverserPhase_ = {};
    reverserDone_ = false;
    airborneTime_ = 0;
    parkingBrake_ = true;
    for (std::size_t i = 0; i < def_.electrical.loads.size(); ++i)
        electrical_.setLoadSwitch(static_cast<int>(i), def_.electrical.loads[i].initiallyOn);
    hydPressure_ = {};
    hydPumpOn_.fill(true);
    hydPumpCurrent_ = 0;
    gearEmergency_ = false;
    brakeWasApplied_ = brakeHeld_ = false;
    brakeAccumulator_ = def_.hydraulics.present && !def_.hydraulics.brakes.empty() ? def_.hydraulics.brakeAccumulatorApplications : -1;
    gearSupply_ = flapSupply_ = speedbrakeSupply_ = steeringSupply_ = brakeSupply_ = 1;
    lever_ = {};
    instruments_.resetCold(sensorInputs());
    syncCockpitPositions();
}

void Aircraft::setFlapsImmediately(int detent) {
    const auto& d = def_.controls.flapDetents;
    flapDetent_ = std::clamp(detent, 0, static_cast<int>(d.size()) - 1);
    flapAngle_ = d[static_cast<std::size_t>(flapDetent_)];
    ctx_.controls.flap = flapAngle_;
    syncCockpitPositions();
}

void Aircraft::setEngineRunning(bool running, Real rpm) {
    if (!running) { setColdAndDark(); return; }
    electrical_.setBatterySwitch(true);
    electrical_.setAlternatorSwitch(true);
    electrical_.setAvionicsSwitch(true);
    fuel_.setShutoffOpen(true);
    for (int i = 0; i < engineCount(); ++i) {
        magnetos_[i] = Magnetos::Both;
        s_.shaftOmega[i] = rpm * kTwoPi / 60;
        engineState_[i].omega = s_.shaftOmega[i];
        engineState_[i].firing = engineState_[i].running = true;
        engineState_[i].egt = 900; engineState_[i].cht = 420; engineState_[i].oilTemperature = 360;
        runningWas_[i] = true;
    }
    for (int i = 0; i < turbofanCount(); ++i) {
        turboFuel_[i] = true;
        TurbofanInputs ti;
        ti.fuelOn = ti.fuelAtEngine = true;
        turbofans_[i].setSteady(turboState_[i], ti);
        turboState_[i].itt = def_.turbofans[i].ittIdle;
        runningWas_[i] = true;
    }
    vacuum_.step(0, rpm);
    // Engine-driven hydraulic pumps up to pressure.
    for (std::size_t k = 0; k < def_.hydraulics.systems.size(); ++k)
        hydPressure_[k] = def_.hydraulics.systems[k].engines.empty() ? 0.0 : 1.0;
    // Establish bus voltage (battery, then the generators' fields) for the spawn.
    ElectricalInputs ei;
    ei.alternatorRpm = turbofanCount() > 0 ? 0.0 : rpm;
    ei.generatorDrive.fill(2.0);
    electrical_.step(0, ei);
    electrical_.step(0, ei);
    for (int i = 0; i < engineCount(); ++i) {
        engineState_[i].oilPressure = def_.engines[i].oilReliefPressure;
    }
    instruments_.resetToTruth(sensorInputs());
    syncCockpitPositions();
}

void Aircraft::placeOnGround(Real north, Real east, Real heading, const GroundQuery& ground, const AtmosphereField& atm) {
    s_.body = {};
    s_.body.attitude = Quat::fromEuler(0, 0, heading);
    const GroundHit g = ground.below({north, east, -1e4});
    const Real groundDown = g.hit ? g.point.z : 0.0;
    Real lowest = -1e9;
    for (const GearDef& gd : def_.gear) lowest = std::max(lowest, gd.attach.z + gd.length - mass_.cg.z);
    const Vec3 cgOffsetNed = s_.body.attitude.rotate(mass_.cg);
    s_.body.position = {north + cgOffsetNed.x, east + cgOffsetNed.y, groundDown - lowest - 0.01};
    s_.tyreLateral.fill(0);
    s_.tyreLongitudinal.fill(0);
    s_.wheelOmega.fill(0);
    s_.wheelSteer.fill(0);
    antiSkid_.fill(1.0);
    // Settle on the gear with brakes set and controls neutral.
    const bool pb = parkingBrake_;
    parkingBrake_ = true;
    PilotInput idle;
    idle.mixture = 1;
    const Real savedTime = time_;
    for (int i = 0; i < 600; ++i) step(1.0 / 120, idle, ground, atm);
    s_.body.velocity = {};
    s_.body.omega = {};
    parkingBrake_ = pb;
    time_ = savedTime;
    events_.clear();
    occupant_.reset();  // the settling is not the occupant's
    wasOnGround_ = true;
    groundStateTimer_ = 0;
    syncCockpitPositions();
    updateTelemetry();
}

void Aircraft::resetInstrumentsToTruth() {
    // Refresh the specific force for the current state (pendulous sensors).
    const Derivatives d = derive(s_, ctx_, false);
    if (mass_.mass > 0) tel_.specificForceBody = d.forceBody / mass_.mass;
    instruments_.resetToTruth(sensorInputs());
}

void Aircraft::rebase(const LocalFrame& nf) {
    const Mat3 r = frame_.rotationTo(nf);
    const Vec3 ecef = frame_.ecefFromNed(s_.body.position);
    s_.body.position = nf.nedFromEcef(ecef);
    s_.body.velocity = r * s_.body.velocity;
    s_.body.attitude = quatFromMatrix(r) * s_.body.attitude;
    s_.body.attitude.normalize();
    frame_ = nf;
    updateTelemetry();
}

// ------------------------------------------------------------ context ----

void Aircraft::prepareContext(const PilotInput& in, const GroundQuery& ground, const AtmosphereField& atm) {
    buildContext(in, ground, atm, 0);
    if (turbofanSteady_ || !def_.hydraulics.present) return;
    // Spawns and trims leave the engine-driven pumps at pressure (the
    // steady spools are set inside buildContext when trimming).
    settleHydraulics();
}

void Aircraft::buildContext(const PilotInput& in, const GroundQuery& ground, const AtmosphereField& atm, Real dt) {
    Context& c = ctx_;
    const Quat& q = s_.body.attitude;
    const Vec3& pos = s_.body.position;
    const Real alt = frame_.origin().height - pos.z;
    // The air as the airframe sees it: turbulence shorter than the span is
    // averaged over it (the strips extrapolate the gradient linearly).
    c.air = atm.sample(pos, alt, time_, def_.referenceSpan);
    c.gravity = {0, 0, wgs84::normalGravity(frame_.origin().latitude, alt)};

    auto plane = [&](const Vec3& p) {
        GroundPlane gp;
        const GroundHit h = ground.below(p);
        if (h.hit) {
            gp.valid = true;
            gp.point = h.point;
            gp.normal = h.normal;
            gp.surface = h.surface;
        }
        return gp;
    };
    for (std::size_t i = 0; i < def_.gear.size(); ++i)
        c.gearGround[i] = plane(pos + q.rotate(def_.gear[i].attach - mass_.cg));
    for (std::size_t i = 0; i < def_.contacts.size(); ++i) {
        const Vec3 p = pos + q.rotate(def_.contacts[i].position - mass_.cg);
        GroundHit solid;
        if (ground.solidAt(p, solid)) {
            // Inside a building: the nearest wall or roof pushes the point out.
            GroundPlane gp;
            gp.valid = true;
            gp.point = solid.point;
            gp.normal = solid.normal;
            gp.surface = solid.surface;
            c.contactGround[i] = gp;
        } else {
            c.contactGround[i] = plane(p);
        }
    }
    if (aeroModel_->surfaceCount() > 0) {
        const Vec3 w = pos + q.rotate(aeroModel_->surfaceCentroid(0) - mass_.cg);
        const GroundPlane gp = plane(w);
        c.wingHeightAgl = gp.valid ? dot(w - gp.point, gp.normal) : 1e9;
        c.groundUpBody = gp.valid ? q.inverseRotate(gp.normal) : Vec3{};
    }

    // Flight controls. Trim acts as an equivalent elevator offset (the
    // stick-free trim-tab hinge-moment model is planned; see fidelity matrix).
    Real elevUp = 0.4;
    for (const auto& s : def_.surfaces)
        for (const auto& p : s.panels)
            for (const auto& cs : p.controls)
                if (cs.channel == ControlChannel::Elevator) elevUp = cs.maxNegative;
    c.controls.aileron = clamp(in.roll + def_.controls.riggingAileron, -1.0, 1.0);
    if (hasTrimmableStabilizer()) {
        // The trim moves the tailplane; the stick keeps the whole elevator.
        c.controls.stabilizer = trim_ * (trim_ > 0 ? def_.controls.stabilizerNoseUp : def_.controls.stabilizerNoseDown);
        c.controls.elevator = clamp(in.pitch, -1.0, 1.0);
    } else {
        c.controls.elevator = clamp(in.pitch + trim_ * def_.controls.elevatorTrimRange / elevUp, -1.0, 1.0);
    }
    if (pusher_) c.controls.elevator = std::min(c.controls.elevator, def_.stallWarning.pusherElevator);
    c.controls.rudder = clamp(in.yaw + def_.controls.riggingRudder + ydCommand_, -1.0, 1.0);
    if (def_.controls.rudderLimiterIas > 0) {
        const Real eas = length(s_.body.velocity - c.air.wind) * std::sqrt(c.air.state.density / isa::kSeaLevelDensity);
        if (eas > def_.controls.rudderLimiterIas)
            c.controls.rudder *= std::max(0.15, sq(def_.controls.rudderLimiterIas / eas));
    }
    c.controls.flap = flapAngle_;
    c.controls.speedbrake = speedbrakePos_;
    c.controls.spoiler = def_.controls.speedbrake.present && !aeroModel_->hasSpoilers() ? speedbrakePos_ * def_.controls.speedbrake.liftLossFraction : 0.0;
    // Wheel steering follows the pedals (not the rudder tab) plus the tiller,
    // through the steering actuator's slew limit where it is powered.
    for (std::size_t i = 0; i < def_.gear.size(); ++i) {
        const GearDef& g = def_.gear[i];
        const Real authority = std::max(g.maxSteerAngle, g.tillerSteerAngle);
        const Real cmd = clamp(g.maxSteerAngle * clamp(in.yaw, -1.0, 1.0) + g.tillerSteerAngle * clamp(in.tiller, -1.0, 1.0),
                               -authority, authority);
        if (g.steerRate > 0 && dt > 0) {
            // A powered actuator: its slew rate with the pressure it has;
            // unpressurised, the wheel castors back toward straight ahead.
            const Real rate = actuatorRate(steeringSupply_);
            if (rate > 0) steerAngle_[i] = approach(steerAngle_[i], cmd, g.steerRate * rate * dt);
            else steerAngle_[i] = approach(steerAngle_[i], 0.0, g.steerRate * dt);
        } else if (g.steerRate > 0) {
            steerAngle_[i] = actuatorRate(steeringSupply_) > 0 ? cmd : steerAngle_[i];
        } else {
            steerAngle_[i] = cmd;
        }
        c.steer[i] = steerAngle_[i];
    }
    // Brakes through the pressure they have (an accumulator-backed
    // application counts as full pressure; see stepHydraulics); the
    // anti-skid modulates each leg's torque below that.
    c.brakeLeft = (parkingBrake_ ? 1.0 : clamp(in.brakeLeft, 0.0, 1.0)) * brakeSupply_;
    c.brakeRight = (parkingBrake_ ? 1.0 : clamp(in.brakeRight, 0.0, 1.0)) * brakeSupply_;
    for (std::size_t i = 0; i < def_.gear.size(); ++i) c.brakeModulation[i] = antiSkid_[i];

    for (int i = 0; i < engineCount(); ++i) {
        PistonEngineInputs& e = c.engineIn[i];
        lever_[i] = clamp(in.engineThrottle(i), 0.0, 1.0);
        e.throttle = lever_[i];
        e.mixture = clamp(in.mixture, 0.0, 1.0);
        e.magnetos = (in.starterHeld && magnetos_[i] == Magnetos::Both) ? Magnetos::Start : magnetos_[i];
        e.fuelAtEngine = fuel_.fuelAtEngine();
        e.ambientPressure = c.air.state.pressure;
        e.ambientTemperature = c.air.state.temperature;
        e.ambientDensity = c.air.state.density;
        e.starterPowered = false;
    }
    const Real airspeed = length(s_.body.velocity - c.air.wind);
    // The FADEC rating: forced by the pilot's thrust mode, else the automatic
    // schedule (advanced in step()).
    switch (in.thrustMode) {
    case ThrustMode::Takeoff: activeRating_ = ThrustRating::Takeoff; break;
    case ThrustMode::MaxContinuous: activeRating_ = ThrustRating::MaxContinuous; break;
    case ThrustMode::Climb: activeRating_ = ThrustRating::MaxClimb; break;
    case ThrustMode::Cruise: activeRating_ = ThrustRating::MaxCruise; break;
    default: activeRating_ = autoRating_; break;
    }
    for (int i = 0; i < turbofanCount(); ++i) {
        TurbofanInputs& e = c.turboIn[i];
        lever_[i] = clamp(in.engineThrottle(i), 0.0, 1.0);
        e.throttle = lever_[i];
        e.rating = activeRating_;
        e.fuelOn = turboFuel_[i];
        e.fuelAtEngine = fuel_.fuelAtEngine();
        e.ambientPressure = c.air.state.pressure;
        e.ambientTemperature = c.air.state.temperature;
        e.mach = airspeed / c.air.state.speedOfSound;
        e.starterPowered = false;
        if (turbofanSteady_) {
            e.fuelOn = e.fuelAtEngine = true;
            turbofans_[i].setSteady(turboState_[i], e);
        }
    }
    if (dt == 0 && turbofanSteady_ && def_.hydraulics.present) settleHydraulics();
}

// --------------------------------------------------------- derivatives ----

Aircraft::Derivatives Aircraft::evaluate(const DynamicState& s) const { return derive(s, ctx_, false); }

Aircraft::Derivatives Aircraft::derive(const DynamicState& s, const Context& c, bool record) const {
    Derivatives d;
    const Quat& q = s.body.attitude;
    const Vec3 vBody = q.inverseRotate(s.body.velocity);
    const Vec3& w = s.body.omega;
    const Vec3 windBody = q.inverseRotate(c.air.wind);
    const Mat3 rm = q.toMatrix();
    const Mat3 gradBody = rm.transposed() * c.air.windGradient * rm;
    const Real rho = c.air.state.density;
    const Vec3& cg = mass_.cg;

    Vec3 force, moment, rotorH;

    // ---- Propulsion ----
    for (int i = 0; i < engineCount(); ++i) {
        const PistonEngineDef& ed = def_.engines[i];
        const Real spin = spinSign(ed);
        const PistonEngineState es = engines_[i].evaluate(c.engineIn[i], s.shaftOmega[i]);
        const Vec3 r = ed.position - cg;
        const Vec3 vDisc = vBody + cross(w, r) - (windBody + gradBody * r);
        const Vec3 axis = normalized(ed.thrustAxis);
        const Real vAx = dot(vDisc, axis);
        const PropellerForces pf = props_[i].evaluate(vAx, s.shaftOmega[i], rho, c.air.state.speedOfSound);
        const Vec3 thrust = axis * pf.thrust;
        force += thrust;
        moment += cross(r, thrust);
        // P-factor: thrust centre shifts toward the descending blade.
        const Vec3 vIn = vDisc - axis * vAx;
        const Real vMag = std::max(length(vDisc), 1.0);
        const Vec3 offset = cross(axis, vIn) * (-spin * ed.pFactorGain * props_[i].radius() / vMag);
        moment += cross(offset, thrust);
        // Engine torque reaction and rotor angular momentum.
        moment -= axis * (spin * (es.brakeTorque + es.starterTorque));
        rotorH += axis * (spin * engines_[i].totalInertia() * s.shaftOmega[i]);
        Real accel = (es.brakeTorque + es.starterTorque - pf.torque) / engines_[i].totalInertia();
        if (s.shaftOmega[i] <= 0 && accel < 0) accel = 0;  // a piston engine cannot turn backwards
        d.shaftAccel[i] = accel;
        // Slipstream for downstream surfaces (momentum theory).
        SlipstreamState& ss = slip_[i];
        const Real A = props_[i].discArea();
        const Real v0 = std::max(vAx, 0.0);
        ss.discCentre = ed.position;
        ss.axis = axis;
        ss.radius = props_[i].radius();
        ss.clockwiseFromBehind = ed.clockwiseFromBehind;
        ss.axialIncrement = pf.thrust > 0 ? std::sqrt(v0 * v0 + 2 * pf.thrust / (rho * A)) - v0 : 0.0;
        const Real mdot = std::max(rho * A * (v0 + 0.5 * ss.axialIncrement), 1.0);
        ss.swirlVelocity = ed.slipstreamSwirlFactor * std::max(pf.torque, 0.0) / (mdot * 0.7 * ss.radius);
        if (record) {
            // Keep the lumped thermal states, which only postStep advances.
            PistonEngineState n = es;
            n.egt = engineState_[i].egt;
            n.cht = engineState_[i].cht;
            n.oilTemperature = engineState_[i].oilTemperature;
            n.oilPressure = engineState_[i].oilPressure;
            engineState_[i] = n;
            propState_[i] = pf;
        }
    }

    // ---- Turbofans: net thrust on the thrust line (spool speeds from the step) ----
    for (int i = 0; i < turbofanCount(); ++i) {
        const TurbofanDef& ed = def_.turbofans[i];
        TurbofanInputs ti = c.turboIn[i];
        const Vec3 vAir = vBody - windBody;
        ti.mach = length(vAir) / c.air.state.speedOfSound;
        TurbofanState ts = turboState_[i];
        turbofans_[i].evaluate(ts, ti);
        const Vec3 r = ed.position - cg;
        // A deployed reverser turns the fan flow forward: the forward thrust
        // goes, and a fraction of it pushes the other way.
        const Real rev = ed.reverser.present ? reverserPos_[static_cast<std::size_t>(i)] : 0.0;
        const Vec3 thrust = normalized(ed.thrustAxis) * (ts.thrust * (1 - rev * (1 + ed.reverser.effectiveness)));
        force += thrust;
        moment += cross(r, thrust);
    }
    // ---- Landing gear and speedbrake drag areas ----
    // Where they are: extended gear drags below the CG (nose down), a lumped
    // speedbrake on the wing. Spoiler panels are in the aerodynamics instead.
    {
        const Vec3 vAir = vBody - windBody;
        const Real qv = 0.5 * rho * length(vAir);
        const Real gearCda = def_.controls.gear.dragArea * (def_.controls.gear.retractable ? gearPos_ : 0.0);
        if (gearCda > 0 && !def_.gear.empty()) {
            const Real share = gearCda / static_cast<Real>(def_.gear.size());
            for (const GearDef& gd : def_.gear) {
                const Vec3 at = gd.attach + Vec3{0, 0, 0.5 * gd.length * gearPos_} - cg;
                const Vec3 f = vAir * (-qv * share);
                force += f;
                moment += cross(at, f);
            }
        }
        if (def_.controls.speedbrake.present && !aeroModel_->hasSpoilers() && speedbrakePos_ > 0) {
            const Vec3 at = (aeroModel_->surfaceCount() > 0 ? aeroModel_->surfaceCentroid(0) : cg) - cg;
            const Vec3 f = vAir * (-qv * def_.controls.speedbrake.dragArea * speedbrakePos_);
            force += f;
            moment += cross(at, f);
        }
    }

    // ---- Aerodynamics ----
    AeroInputs ai;
    ai.velocityBody = vBody;
    ai.omegaBody = w;
    ai.windBody = windBody;
    ai.windGradientBody = gradBody;
    ai.density = rho;
    ai.speedOfSound = c.air.state.speedOfSound;
    ai.cg = cg;
    ai.controls = c.controls;
    ai.heightAboveGround = c.wingHeightAgl;
    ai.groundUpBody = c.groundUpBody;
    ai.slipstreams = slip_.data();
    ai.slipstreamCount = engineCount();
    ai.downwashLagged = quasiSteadyDownwash_ ? nullptr : downwashLag_.data();
    aeroModel_->evaluate(ai, aeroOut_);
    force += aeroOut_.force;
    moment += aeroOut_.moment;

    // ---- Landing gear ----
    // Each leg: an oleo (spring, two-sided damper, bottoming), a tyre with
    // lateral and longitudinal bristle states (relaxation-length model,
    // Pacejka "Tyre and Vehicle Dynamics" ch. 7), its wheel's spin and, for a
    // castoring leg, the wheel's own heading. Friction: a single ellipse
    // (circle) on the resultant bristle force at mu_peak N, falling to
    // mu_slide N as the tyre slides, the force then along the sliding velocity.
    for (std::size_t i = 0; i < def_.gear.size(); ++i) {
        const GearDef& g = def_.gear[i];
        const GearRuntime& gr = gearRt_[i];
        const GroundPlane& gp = c.gearGround[i];
        const bool castoring = g.castorTrail > 0;
        GearTelemetry gt;
        gt.wheelOmega = s.wheelOmega[i];
        gt.steerAngle = castoring ? s.wheelSteer[i] : c.steer[i];
        // Off the ground: the bristles relax, the wheel spins down on its
        // bearings (~20 s), a castoring wheel's centring cam lines it up.
        d.tyreLateralRate[i] = -s.tyreLateral[i] * 20;
        d.tyreLongitudinalRate[i] = -s.tyreLongitudinal[i] * 20;
        d.wheelAlpha[i] = -s.wheelOmega[i] * 0.05;
        d.wheelSteerRate[i] = castoring ? (c.steer[i] - s.wheelSteer[i]) * 5 : 0.0;
        const bool legDown = (!def_.controls.gear.retractable || gearPos_ > 0.98) && !gearFailed_[i];
        gt.failed = gearFailed_[i];
        if (gp.valid && legDown) {
            const Vec3 rb = g.attach - cg;
            const Vec3 pAttach = s.body.position + q.rotate(rb);
            const Vec3 a = q.rotate({0, 0, 1});
            const Vec3& n = gp.normal;
            const Real h = dot(pAttach + a * g.length - gp.point, n);
            const Real cosT = dot(a, -n);
            if (h < 0 && cosT > 0.3) {
                const Real x = -h / cosT;
                const Vec3 rc = rb + Vec3{0, 0, g.length - x};
                const Vec3 vPt = s.body.velocity + q.rotate(cross(w, rc));
                const Real xdot = -dot(vPt, n) / cosT;
                Real fs = g.preload + g.stiffness * x + (xdot > 0 ? g.dampingCompression : g.dampingRebound) * xdot;
                if (x > g.maxCompression) fs += 20 * g.stiffness * (x - g.maxCompression);
                const Real N = std::max(fs, 0.0);
                const Real steer = gt.steerAngle;
                Vec3 f = q.rotate({std::cos(steer), std::sin(steer), 0});
                f = normalized(f - n * dot(f, n));
                const Vec3 l = cross(n, f);  // the wheel's left (n up, f forward)
                const Real vLong = dot(vPt, f), vLat = dot(vPt, l);
                const Real vGround = std::hypot(vLong, vLat);
                // Peak friction at the contact point's ground speed (the wet
                // curve is in ground speed), and the sliding value.
                const Real muPeak = wetTyreFriction(gp.surface, vGround, g.tyrePressure);
                const Real muSlide = g.slidingFrictionRatio * muPeak;
                const Real kx = g.longitudinalStiffness;
                const Real ky = corneringStiffness(i, N) / g.relaxationLength;
                // Wheel spin: the longitudinal bristle sees the slip velocity.
                const Real r = g.wheelRadius;
                const Real omega = s.wheelOmega[i];
                const Real vSlip = vLong - r * omega;
                const Real brakeCmd = g.brakeGroup == 0 ? c.brakeLeft : (g.brakeGroup == 1 ? c.brakeRight : 0.0);
                const Real tBrake = brakeCmd * c.brakeModulation[i] * gr.brakeTorque;
                const Real tRoll = g.rollingResistance * gp.surface.rollingResistance * N * r;
                const Real tHold = tBrake + tRoll;  // what stops the wheel turning
                const Real omegaLock = 0.1 / r;     // below 0.1 m/s of rim speed the wheel can be held
                const Real fyRaw = -(ky * s.tyreLateral[i] + 0.02 * ky * vLat);
                // Friction ellipse on the resultant bristle demand; past the
                // peak the tyre slides: mu falls toward mu_slide and the force
                // turns from the bristle's direction to oppose the sliding velocity.
                auto combine = [&](Real fxRaw, Real& fx, Real& fy) {
                    fx = fxRaw;
                    fy = fyRaw;
                    const Real fMax = muPeak * N;
                    const Real mag = std::hypot(fx, fy);
                    if (fMax <= 1e-6 || mag <= 1e-9) {
                        if (fMax <= 1e-6) fx = fy = 0;
                        return;
                    }
                    const Real demand = mag / fMax;
                    if (demand <= 1) return;
                    const Real sl = std::min((demand - 1) / kSlideWidth, 1.0);
                    const Real muEff = muPeak - sl * (muPeak - muSlide);
                    const Real vs = std::max(std::hypot(vSlip, vLat), 0.1);
                    Real dx = (1 - sl) * fx / mag - sl * vSlip / vs;
                    Real dy = (1 - sl) * fy / mag - sl * vLat / vs;
                    Real dn = std::hypot(dx, dy);
                    if (dn < 1e-6) { dx = fx / mag; dy = fy / mag; dn = 1; }
                    fx = muEff * N * dx / dn;
                    fy = muEff * N * dy / dn;
                };
                Real fLong = 0, fLat = 0;
                combine(-(kx * s.tyreLongitudinal[i] + gr.spinDamping * vSlip), fLong, fLat);
                // I_w omega_dot = -r F_x - (T_brake + T_roll) sign(omega): the
                // ground's force on the tread turns the wheel, the brake and the
                // rolling resistance oppose its turning. A wheel at rest stays
                // held while the tread force is within what the brakes (or, with
                // them off, the rolling threshold) can hold; the bristle is then
                // the airframe's spring to the ground, with the heavier damping
                // that mode needs (the wheel's own fast mode is absent then).
                const Real tDrive = -r * fLong;
                const Real sgn = std::abs(omega) >= omegaLock ? sign(omega) : sign(tDrive);
                const Real alphaFree = (tDrive - tHold * sgn) / gr.inertia;
                // Coulomb stop: the wheel is held when it is as good as stopped,
                // or would stop within this step (a braked wheel loses tens of
                // rad/s per step; left to the stages it would overshoot into
                // reverse and chatter), and the tread force is within what the
                // brakes (or the rolling threshold) hold.
                const bool stopping = omega * alphaFree < 0 && std::abs(omega) <= std::abs(alphaFree) * stepDt_;
                const bool held = (std::abs(omega) < omegaLock || stopping) && std::abs(tDrive) <= tHold;
                if (held) {
                    // The sliding law still applies to a locked, skidding tyre;
                    // at rest the tread force is what the brakes hold.
                    combine(-(kx * s.tyreLongitudinal[i] + gr.heldDamping * vSlip), fLong, fLat);
                    fLong = saturate(fLong, tHold / r);
                    d.wheelAlpha[i] = -omega * 100;
                } else {
                    d.wheelAlpha[i] = alphaFree;
                }
                d.tyreLongitudinalRate[i] = vSlip - std::abs(vLong) * s.tyreLongitudinal[i] / g.relaxationLength;
                d.tyreLateralRate[i] = vLat - std::abs(vLong) * s.tyreLateral[i] / g.relaxationLength;
                // Castoring wheel: its heading relaxes (over about a trail plus
                // a relaxation length of rolling, at least 5/s so the pedals
                // move it at rest) toward the quasi-static balance of the
                // linkage spring (yielding at its breakout) against the tyre's
                // aligning moment on the trail, between the hard stops.
                if (castoring) {
                    const Vec3 f0 = normalized(q.rotate({1, 0, 0}) - n * dot(q.rotate({1, 0, 0}), n));
                    const Vec3 l0 = cross(n, f0);
                    // (l0 = n x f0 points to the wheel's left; steer is + to the right.)
                    const Real deltaV = std::atan2(-dot(vPt, l0), std::max(dot(vPt, f0), 0.2));
                    const Real cmd = c.steer[i];
                    // The aligning moment needs rolling: a wheel at rest has no
                    // direction to line up with (the pedals then just turn it).
                    const Real rolling = clamp(vGround / 0.5, 0.0, 1.0);
                    const Real tyreGain = g.castorTrail * ky * g.relaxationLength * rolling;  // trail x C_alpha
                    const Real tyreMax = g.castorTrail * muPeak * N * rolling;
                    const Real ks = g.steerLinkageStiffness, breakout = g.steerLinkageBreakout;
                    auto torque = [&](Real delta) {
                        Real m = saturate(tyreGain * (deltaV - delta), tyreMax);
                        if (ks > 0) {
                            const Real ms = ks * (cmd - delta);
                            m += breakout > 0 ? saturate(ms, breakout) : ms;
                        }
                        return m;  // + turns the wheel right; non-increasing in delta
                    };
                    const Real dMax = g.maxSteerAngle + g.castorAngle;
                    Real lo = -dMax, hi = dMax, deltaEq;
                    if (torque(lo) <= 0) deltaEq = lo;
                    else if (torque(hi) >= 0) deltaEq = hi;
                    else {
                        for (int it = 0; it < 24; ++it) {
                            const Real mid = 0.5 * (lo + hi);
                            (torque(mid) > 0 ? lo : hi) = mid;
                        }
                        deltaEq = 0.5 * (lo + hi);
                    }
                    const Real rate = clamp(vGround / (g.castorTrail + g.relaxationLength), 5.0, 60.0);
                    d.wheelSteerRate[i] = rate * (deltaEq - s.wheelSteer[i]);
                }
                const Vec3 fNed = n * N + f * fLong + l * fLat;
                const Vec3 fB = q.inverseRotate(fNed);
                force += fB;
                moment += cross(rc, fB);
                gt.contact = N > 0;
                gt.compression = x;
                gt.normalForce = N;
                gt.closureSpeed = xdot;
                gt.surfaceId = gp.surface.id;
                gt.rollingSpeed = vLong;
                gt.slipRatio = vSlip / std::max(std::abs(vLong), 1.0);
                gt.longitudinalForce = fLong;
                gt.lateralForce = fLat;
                gt.peakFriction = muPeak;
                gt.brakeTorque = tBrake;
                gt.held = held;
            }
        }
        if (record) gearTel_[i] = gt;
    }

    // ---- Hard points (tail, wingtips, propeller tip, belly) ----
    for (std::size_t i = 0; i < def_.contacts.size(); ++i) {
        const ContactPointDef& cp = def_.contacts[i];
        const GroundPlane& gp = c.contactGround[i];
        if (!gp.valid) continue;
        const Vec3 rb = cp.position - cg;
        const Vec3 p = s.body.position + q.rotate(rb);
        const Real h = dot(p - gp.point, gp.normal);
        if (h >= 0) continue;
        const Vec3 vPt = s.body.velocity + q.rotate(cross(w, rb));
        const Real vn = dot(vPt, gp.normal);
        const Real N = std::max(0.0, cp.stiffness * -h - cp.damping * vn);
        const Vec3 vt = vPt - gp.normal * vn;
        const Vec3 ft = vt * (-cp.friction * N / std::max(length(vt), 0.2));
        const Vec3 fB = q.inverseRotate(gp.normal * N + ft);
        force += fB;
        moment += cross(rb, fB);
        if (record && N > 0) contactTouched_[i] = true;
    }

    // ---- Equations of motion ----
    d.forceBody = force;
    d.momentBody = moment;
    d.body.position = s.body.velocity;
    d.body.velocity = q.rotate(force / mass_.mass) + c.gravity;
    d.body.omega = mass_.inverseInertia * (moment - cross(w, mass_.inertia * w + rotorH));
    const Quat qd = quatDerivative(q, w);
    d.body.attitude = qd;
    return d;
}

void Aircraft::settleDownwash() {
    // Converge the lag states onto the instantaneous downwash at this state.
    for (int it = 0; it < 4; ++it) {
        derive(s_, ctx_, false);
        for (int si = 0; si < aeroModel_->surfaceCount(); ++si) downwashLag_[si] = aeroOut_.surfaces[si].downwash;
    }
}

// --------------------------------------------------------------- step ----

namespace {
DynamicState advance(const DynamicState& s, const Aircraft::Derivatives& d, Real h) {
    DynamicState r = s;
    r.body.position += d.body.position * h;
    r.body.velocity += d.body.velocity * h;
    r.body.attitude = s.body.attitude + d.body.attitude * h;
    r.body.attitude.normalize();
    r.body.omega += d.body.omega * h;
    for (int i = 0; i < kMaxEngines; ++i) r.shaftOmega[i] += d.shaftAccel[i] * h;
    for (int i = 0; i < kMaxGear; ++i) {
        r.tyreLateral[i] += d.tyreLateralRate[i] * h;
        r.tyreLongitudinal[i] += d.tyreLongitudinalRate[i] * h;
        r.wheelOmega[i] += d.wheelAlpha[i] * h;
        r.wheelSteer[i] += d.wheelSteerRate[i] * h;
    }
    return r;
}
}  // namespace

Real Aircraft::trimForElevator(Real e) const {
    Real elevUp = 0.4, elevDown = 0.4, tau = 0.5;
    for (const auto& s : def_.surfaces)
        for (const auto& p : s.panels)
            for (const auto& c : p.controls)
                if (c.channel == ControlChannel::Elevator) {
                    elevUp = c.maxNegative;
                    elevDown = c.maxPositive;
                    tau = flapEffectiveness(c.chordFraction);
                }
    if (!hasTrimmableStabilizer()) return clamp(e * elevUp / std::max(def_.controls.elevatorTrimRange, 1e-6), -1.0, 1.0);
    // Same tail lift: stabiliser angle = elevator angle x its effectiveness.
    const Real angle = tau * e * (e > 0 ? elevUp : elevDown);
    const Real range = angle > 0 ? def_.controls.stabilizerNoseUp : def_.controls.stabilizerNoseDown;
    return clamp(angle / std::max(range, 1e-6), -1.0, 1.0);
}

void Aircraft::step(Real dt, const PilotInput& in, const GroundQuery& ground, const AtmosphereField& atm) {
    if (!(dt > 0) || faulted_) return;
    stepDt_ = dt;
    buildContext(in, ground, atm, dt);
    Context& c = ctx_;

    // ---- Systems at step start ----
    Real starterCurrent = 0;
    for (int i = 0; i < engineCount(); ++i)
        if (c.engineIn[i].magnetos == Magnetos::Start) starterCurrent += def_.engines[i].starterCurrent;
    const Real flapTarget = def_.controls.flapDetents.empty()
                                ? 0.0
                                : def_.controls.flapDetents[static_cast<std::size_t>(std::clamp(flapDetent_, 0, static_cast<int>(def_.controls.flapDetents.size()) - 1))];
    flapMoving_ = std::abs(flapTarget - flapAngle_) > 1e-6;
    const int flapLoad = electrical_.loadIndex("flaps");
    const int pumpLoad = electrical_.fuelPumpLoad();
    // Hydraulics from the engines and the bus as they stand at step start
    // (the electric pumps' current then loads the bus this step).
    stepHydraulics(dt, in);
    ElectricalInputs ei;
    // Each generator's drive: its piston engine's rpm against the alternator
    // cut-in, or its turbofan core against the starter-generator's cut-in N2
    // (a windmilling core does not hold the bus). A single alternator with no
    // declared generators takes the first piston engine, or the fastest core.
    for (int k = 0; k < electrical_.generatorCount(); ++k) {
        const GeneratorDef& gd = electrical_.generator(k);
        Real drive = 0;
        auto pistonDrive = [&](int e) { return s_.shaftOmega[e] * 60 / kTwoPi / def_.engines[static_cast<std::size_t>(e)].alternatorMinRpm; };
        if (def_.electrical.generators.empty()) {
            if (engineCount() > 0) drive = pistonDrive(0);
            for (int i = 0; i < turbofanCount(); ++i) drive = std::max(drive, turboState_[i].n2 / gd.cutInN2);
        } else if (engineCount() > 0) {
            if (gd.engine < engineCount()) drive = pistonDrive(gd.engine);
        } else if (gd.engine < turbofanCount()) {
            drive = turboState_[gd.engine].n2 / gd.cutInN2;
        }
        ei.generatorDrive[static_cast<std::size_t>(k)] = drive;
    }
    // Turbofan starter-generators crank while starting.
    for (int i = 0; i < turbofanCount(); ++i)
        if (turboState_[i].starting && turboState_[i].n2 < 0.45) starterCurrent += 250;
    ei.starterCurrent = starterCurrent;
    ei.flapMotorCurrent = flapMoving_ && def_.controls.flapsElectric ? def_.controls.flapMotorCurrent : 0;
    // The electric fuel pump draws its breaker's declared current while on
    // (5 A, a light aircraft's, where no breaker is declared).
    ei.auxPumpCurrent = fuel_.auxPumpOn() ? (pumpLoad >= 0 ? def_.electrical.loads[static_cast<std::size_t>(pumpLoad)].current : 5.0) : 0.0;
    ei.hydraulicPumpCurrent = hydPumpCurrent_;
    electrical_.step(dt, ei);
    for (int i = 0; i < engineCount(); ++i)
        c.engineIn[i].starterPowered = c.engineIn[i].magnetos == Magnetos::Start &&
                                       electrical_.busVoltage() >= kMinStarterVoltage;
    // ---- Ground spoilers and thrust reversers (automatic, on the landing roll) ----
    {
        bool mains = false;
        for (std::size_t g = 0; g < def_.gear.size(); ++g)
            mains = mains || (def_.gear[g].attach.x < mass_.cg.x && gearTel_[g].contact);
        const Vec3 v = s_.body.velocity;
        const Real groundSpeed = std::hypot(v.x, v.y);
        const Real ias = length(v - c.air.wind) * std::sqrt(c.air.state.density / isa::kSeaLevelDensity);
        // A landing: the main wheels on the ground after flight, the throttles
        // anywhere below the go-around range (the automation retards them).
        // On a take-off roll only idle counts (a rejected take-off).
        if (!mains && !tel_.onGround) airborneTime_ += dt;
        else if (groundSpeed < 7.7) airborneTime_ = 0;  // stopped: the next run is a take-off
        const bool landing = airborneTime_ > 2.0;
        // The most advanced lever: every throttle at idle arms the automation.
        Real lever = in.throttle;
        for (int i = 0; i < engineCount() + turbofanCount(); ++i) lever = i == 0 ? lever_[i] : std::max(lever, lever_[i]);
        // ---- FADEC rating schedule (ThrustMode::Auto) ----
        if (turbofanCount() > 0) {
            const bool airborne = !mains && !tel_.onGround;
            bool engineOut = false;
            for (int i = 0; i < turbofanCount(); ++i) engineOut = engineOut || !turboState_[i].running;
            // Take-off thrust from lift-off for the rating's time; a go-around
            // (levers to the stop with the gear or flaps out) starts it again.
            const bool goAround = airborne && lever >= 0.95 && leversLow_ && (gearDown_ || flapDetent_ > 0);
            if (!airborne || goAround) takeoffRatingTimer_ = 0;
            else takeoffRatingTimer_ += dt;
            leversLow_ = lever < 0.9;
            const TurbofanDef& td = def_.turbofans[0];
            const Real limit = engineOut ? td.takeoffRatingTimeEngineOut : td.takeoffRatingTime;
            // Climbing (> 300 fpm) keeps the climb rating; a minute without a
            // climb (level, or descending) selects maximum cruise.
            if (-s_.body.velocity.z > 1.5) levelTime_ = 0;
            else levelTime_ += dt;
            if (takeoffRatingTimer_ < limit) autoRating_ = ThrustRating::Takeoff;
            else if (engineOut) autoRating_ = ThrustRating::MaxContinuous;
            else if (levelTime_ < 60) autoRating_ = ThrustRating::MaxClimb;
            else autoRating_ = ThrustRating::MaxCruise;
        }
        const bool idle = lever <= (landing ? 0.4 : 0.08), advanced = lever > (landing ? 0.5 : 0.2);
        const SpeedbrakeDef& sb = def_.controls.speedbrake;
        if (sb.present && sb.groundSpoilers) {
            if (!groundSpoilers_ && mains && idle && groundSpeed > sb.groundArmSpeed) groundSpoilers_ = true;
            else if (groundSpoilers_ && (advanced || groundSpeed < sb.groundStowSpeed)) groundSpoilers_ = false;
        }
        if (!mains && !tel_.onGround) reverserDone_ = false;  // airborne: armed for the next landing
        for (int i = 0; i < turbofanCount(); ++i) {
            const TurbofanDef::Reverser& rv = def_.turbofans[i].reverser;
            if (!rv.present) continue;
            const std::size_t k = static_cast<std::size_t>(i);
            int& ph = reverserPhase_[k];
            Real& pos = reverserPos_[k];
            const TurbofanEngine& eng = turbofans_[i];
            switch (ph) {
            case 0:  // stowed: deploy on the main wheels, idle, still fast
                if (mains && idle && ias > rv.stowIas + 2.5 && turboState_[i].running && !reverserDone_) ph = 1;
                break;
            case 1:  // reverse thrust down to the stow speed
                if (advanced || !mains) { ph = 3; reverserDone_ = true; }
                else if (ias <= rv.stowIas) ph = 2;
                break;
            case 2:  // reverse idle, then stow
                if (advanced || turboState_[i].n1 < def_.turbofans[i].idleN1 + 0.05) { ph = 3; reverserDone_ = true; }
                break;
            default:  // stowing
                if (pos <= 0) ph = 0;
                break;
            }
            pos = approach(pos, ph == 1 || ph == 2 ? 1.0 : 0.0, dt / rv.transitTime);
            // The fuel control: reverse N1 only once fully deployed; idle while
            // the sleeves travel (the interlock), the lever's own setting once stowed.
            if (ph == 1 && pos > 0.95) c.turboIn[i].throttle = (rv.reverseN1 - eng.def().idleN1) / (1 - eng.def().idleN1);
            else if (ph != 0) c.turboIn[i].throttle = 0;
        }
    }
    for (int i = 0; i < turbofanCount(); ++i) {
        c.turboIn[i].starterPowered = electrical_.busVoltage() >= kMinStarterVoltage;
        if (!turbofanSteady_) turbofans_[i].step(turboState_[i], c.turboIn[i], dt);
    }
    // Actuators move at their rate times what their hydraulic supply allows
    // (1 without hydraulics). The emergency release lets the gear fall to
    // down and locked at a fraction of the normal rate whatever the pressure,
    // and it cannot be retracted until the release is reset.
    if (def_.controls.gear.retractable) {
        const Real normal = actuatorRate(gearSupply_) / def_.controls.gear.transitTime;
        if (gearEmergency_) {
            const Real freeFall = def_.hydraulics.freeFallRateFraction / def_.controls.gear.transitTime;
            gearPos_ = approach(gearPos_, 1.0, std::max(freeFall, gearDown_ ? normal : 0.0) * dt);
        } else {
            gearPos_ = approach(gearPos_, gearDown_ ? 1.0 : 0.0, normal * dt);
        }
    }
    if (def_.controls.speedbrake.present) {
        const Real lever = static_cast<Real>(speedbrakeDetent_) / std::max(def_.controls.speedbrake.detents - 1, 1);
        const Real target = groundSpoilers_ ? 1.0 : lever;
        const Real rate = (groundSpoilers_ ? def_.controls.speedbrake.groundRate : def_.controls.speedbrake.rate) * actuatorRate(speedbrakeSupply_);
        speedbrakePos_ = approach(speedbrakePos_, target, rate * dt);
        c.controls.speedbrake = speedbrakePos_;
        c.controls.spoiler = aeroModel_->hasSpoilers() ? 0.0 : speedbrakePos_ * def_.controls.speedbrake.liftLossFraction;
    }
    const bool flapPowered = !def_.controls.flapsElectric ||
                             (electrical_.powered() && (flapLoad < 0 || electrical_.loadSwitch(flapLoad)));
    if (flapPowered) flapAngle_ = approach(flapAngle_, flapTarget, def_.controls.flapRate * actuatorRate(flapSupply_) * dt);
    c.controls.flap = flapAngle_;
    trim_ = clamp(trim_ + in.trimRate * def_.controls.elevatorTrimRate * dt, -1.0, 1.0);

    // ---- RK4 ----
    const DynamicState s0 = s_;
    const Derivatives k1 = derive(s0, c, false);
    const Derivatives k2 = derive(advance(s0, k1, 0.5 * dt), c, false);
    const Derivatives k3 = derive(advance(s0, k2, 0.5 * dt), c, false);
    const Derivatives k4 = derive(advance(s0, k3, dt), c, false);
    DynamicState s1 = s0;
    const Real h6 = dt / 6;
    s1.body.position += (k1.body.position + k2.body.position * 2 + k3.body.position * 2 + k4.body.position) * h6;
    s1.body.velocity += (k1.body.velocity + k2.body.velocity * 2 + k3.body.velocity * 2 + k4.body.velocity) * h6;
    s1.body.attitude = s0.body.attitude +
                       (k1.body.attitude + k2.body.attitude * 2 + k3.body.attitude * 2 + k4.body.attitude) * h6;
    s1.body.attitude.normalize();
    s1.body.omega += (k1.body.omega + k2.body.omega * 2 + k3.body.omega * 2 + k4.body.omega) * h6;
    for (int i = 0; i < kMaxEngines; ++i)
        s1.shaftOmega[i] = std::max(0.0, s0.shaftOmega[i] + (k1.shaftAccel[i] + 2 * k2.shaftAccel[i] +
                                                             2 * k3.shaftAccel[i] + k4.shaftAccel[i]) * h6);
    for (int i = 0; i < kMaxGear; ++i) {
        s1.tyreLateral[i] += (k1.tyreLateralRate[i] + 2 * k2.tyreLateralRate[i] + 2 * k3.tyreLateralRate[i] +
                              k4.tyreLateralRate[i]) * h6;
        s1.tyreLongitudinal[i] += (k1.tyreLongitudinalRate[i] + 2 * k2.tyreLongitudinalRate[i] +
                                   2 * k3.tyreLongitudinalRate[i] + k4.tyreLongitudinalRate[i]) * h6;
        s1.wheelOmega[i] += (k1.wheelAlpha[i] + 2 * k2.wheelAlpha[i] + 2 * k3.wheelAlpha[i] + k4.wheelAlpha[i]) * h6;
        s1.wheelSteer[i] += (k1.wheelSteerRate[i] + 2 * k2.wheelSteerRate[i] + 2 * k3.wheelSteerRate[i] +
                             k4.wheelSteerRate[i]) * h6;
    }
    s_ = s1;
    postStep(dt, c, s0);
}

void Aircraft::postStep(Real dt, const Context& c, const DynamicState& prev) {
    // Numerical fault guard: never propagate NaN/Inf into the world.
    if (!isFinite(s_.body.position) || !isFinite(s_.body.velocity) || !isFinite(s_.body.omega) ||
        !std::isfinite(s_.body.attitude.w)) {
        s_ = prev;
        faulted_ = true;
        return;
    }
    // Recorded evaluation at the end-of-step state: telemetry, engine state,
    // gear loads, contact flags.
    for (bool& t : contactTouched_) t = false;
    const Derivatives fin = derive(s_, c, true);

    // Tyre bristles cannot exceed their friction limits: the same ellipse as
    // the force, wet-aware, allowed out to the fully sliding demand. A wheel
    // the brakes hold is exactly stopped. Then the anti-skid controllers.
    for (std::size_t i = 0; i < def_.gear.size(); ++i) {
        const GearDef& g = def_.gear[i];
        const GearRuntime& gr = gearRt_[i];
        const GearTelemetry& gt = gearTel_[i];
        if (!gt.contact) {
            antiSkid_[i] = 1;
            continue;
        }
        const Real kx = g.longitudinalStiffness;
        const Real ky = corneringStiffness(i, gt.normalForce) / g.relaxationLength;
        const Real fx = kx * s_.tyreLongitudinal[i], fy = ky * s_.tyreLateral[i];
        const Real lim = (1 + kSlideWidth) * gt.peakFriction * gt.normalForce;
        const Real mag = std::hypot(fx, fy);
        if (mag > lim && mag > 0) {
            const Real k = lim / mag;
            s_.tyreLongitudinal[i] *= k;
            s_.tyreLateral[i] *= k;
        }
        if (gt.held) s_.wheelOmega[i] = 0;
        // Anti-skid (a sampled slip-ratio controller, one update per step): it
        // holds the braking slip at the efficiency's fraction of the peak slip
        // (on the rising side of the curve the force is that fraction of the
        // peak force) by integrating the slip error into the torque let
        // through, and dumps the pressure quickly in a deep skid. It is given
        // the tyre's current peak slip (an ideal adaptive system); the
        // efficiency says how far short of the peak it works.
        if (g.antiSkidEfficiency > 0) {
            Real& m = antiSkid_[i];
            const Real brakeCmd = g.brakeGroup == 0 ? c.brakeLeft : (g.brakeGroup == 1 ? c.brakeRight : 0.0);
            if (brakeCmd < 0.02 || std::abs(gt.rollingSpeed) < 1.5) {
                m = 1;
            } else {
                const Real slipStiffness = kx * g.relaxationLength + gr.spinDamping * std::abs(gt.rollingSpeed);
                const Real kappaPeak = std::max(gt.peakFriction * gt.normalForce / slipStiffness, 0.005);
                const Real target = g.antiSkidEfficiency * kappaPeak;
                // The loop works in torque, not in lever fraction: a brake with
                // more capacity than the tyre can react gets proportionally
                // smaller steps, so the cycle depth does not grow with the
                // brake (the modulation is a fraction of brakeCmd * brakeTorque).
                const Real tyreTorque = gt.peakFriction * gt.normalForce * g.wheelRadius;
                const Real gain = std::min(1.0, tyreTorque / std::max(brakeCmd * gr.brakeTorque, 1.0));
                m += (target - gt.slipRatio) / target * gain * dt / 0.08;
                if (gt.slipRatio > 1.2 * target) m -= gain * dt / 0.03;
                m = clamp(m, 0.0, 1.0);
            }
        }
    }

    // Fuel burn -> mass, CG, inertia.
    Real demand = 0;
    bool rotating = false;
    for (int i = 0; i < engineCount(); ++i) {
        demand += engineState_[i].fuelFlow;
        rotating = rotating || s_.shaftOmega[i] > 50 * kTwoPi / 60;
    }
    for (int i = 0; i < turbofanCount(); ++i) {
        demand += turboState_[i].fuelFlow;
        rotating = rotating || turboState_[i].n2 > 0.1;  // engine-driven boost pump
    }
    const int pumpLoad = electrical_.fuelPumpLoad();
    const bool auxPump = fuel_.auxPumpOn() && (pumpLoad >= 0 ? electrical_.loadPowered(pumpLoad) : electrical_.powered());
    fuel_.step(dt, demand, rotating || auxPump);
    refreshMass();

    // Downwash transport lag.
    const Real tas = std::max(aeroOut_.airspeed, 5.0);
    for (int si = 0; si < aeroModel_->surfaceCount(); ++si) {
        const int src = aeroModel_->downwashSource(si);
        if (src < 0) continue;
        const Real lt = std::abs(aeroModel_->surfaceCentroid(src).x - aeroModel_->surfaceCentroid(si).x);
        const Real target = aeroOut_.surfaces[si].downwash;
        downwashLag_[si] += (target - downwashLag_[si]) * std::min(dt * tas / std::max(lt, 0.1), 1.0);
    }

    // Vacuum, engine temperatures.
    vacuum_.step(dt, engineCount() > 0 ? s_.shaftOmega[0] * 60 / kTwoPi : 0);
    for (int i = 0; i < engineCount(); ++i) engines_[i].updateThermal(engineState_[i], dt, aeroOut_.airspeed);

    time_ += dt;
    // Specific force for instruments/telemetry.
    tel_.specificForceBody = fin.forceBody / mass_.mass;
    updateTelemetry();

    // Instruments: pneumatic stall-warning vane, then the panel's sensors.
    bool vane = false;
    if (def_.stallWarning.present && aeroModel_->surfaceCount() > def_.stallWarning.surface) {
        const int first = aeroModel_->firstStrip(def_.stallWarning.surface);
        const int end = aeroModel_->endStrip(def_.stallWarning.surface);
        int idx = def_.stallWarning.strip >= 0 ? first + def_.stallWarning.strip : (first + end) / 2;
        if (def_.stallWarning.computed) {
            Real least = 1e9;
            for (int i = first; i < end; ++i) {
                const StripTelemetry& s = aeroOut_.strips[i];
                if (s.q > 30 && aeroModel_->stripIsPrimary(i) && s.stallAlpha - s.alpha < least) {
                    least = s.stallAlpha - s.alpha;
                    idx = i;
                }
            }
        }
        if (idx < end) {
            const StripTelemetry& st = aeroOut_.strips[idx];
            vane = st.q > 30 && st.alpha > st.stallAlpha - def_.stallWarning.alphaMargin;
            if (def_.stallWarning.pusherMargin > 0) {
                const Real onAt = st.stallAlpha - def_.stallWarning.pusherMargin;
                if (st.q > 30 && !tel_.onGround && st.alpha > onAt) pusher_ = true;
                else if (pusher_ && (st.alpha < onAt - 0.0873 || tel_.onGround)) pusher_ = false;  // 5 deg hysteresis
            }
        }
    }
    stallVane_ = vane;
    instruments_.update(dt, sensorInputs());
    tel_.stallWarning = vane;
    tel_.stickPusher = pusher_;

    // ---- Yaw damper: washed-out yaw rate to the rudder, for the next step ----
    {
        const auto& yd = def_.controls.yawDamper;
        // Turn compensation: the yaw rate of a coordinated turn at this bank
        // (g sin(roll) cos(pitch) / V) is left alone; only the rest is damped.
        // Without it the damper fought every turn entry, saturated, and left
        // the airplane skidding (10 deg of sideslip in a Learjet 45 rolling in).
        const Real r = tel_.omega.z - kStandardGravity * std::sin(tel_.roll) * std::cos(tel_.pitch) / std::max(tel_.trueAirspeed, 30.0);
        ydLowPass_ += (r - ydLowPass_) * std::min(dt / std::max(yd.washout, 0.1), 1.0);
        const Real eas = tel_.trueAirspeed * std::sqrt(tel_.air.state.density / isa::kSeaLevelDensity);
        const bool on = yd.gain > 0 && yawDamperOn_ && !tel_.onGround && eas > 30.9;
        if (on) {
            const Real schedule = clamp(sq(yd.referenceIas / eas), 0.25, 4.0);
            ydCommand_ = clamp(-yd.gain * schedule * (r - ydLowPass_), -yd.authority, yd.authority);
        } else {
            ydCommand_ = 0;
        }
        tel_.yawDamper = on;
        tel_.yawDamperRudder = ydCommand_;
    }

    // ---- Events ----
    for (int i = 0; i < engineCount(); ++i) {
        if (engineState_[i].running != runningWas_[i])
            pushEvent(engineState_[i].running ? FlightEventType::EngineStarted : FlightEventType::EngineStopped,
                      s_.shaftOmega[i] * 60 / kTwoPi, i);
        runningWas_[i] = engineState_[i].running;
    }
    for (int i = 0; i < turbofanCount(); ++i) {
        if (turboState_[i].running != runningWas_[i])
            pushEvent(turboState_[i].running ? FlightEventType::EngineStarted : FlightEventType::EngineStopped,
                      turboState_[i].n2 * 100, i);
        runningWas_[i] = turboState_[i].running;
    }
    if (fuel_.starving() && !starvingWas_) pushEvent(FlightEventType::FuelStarvation);
    starvingWas_ = fuel_.starving();
    if (vane != stallWarningWas_) pushEvent(vane ? FlightEventType::StallWarningOn : FlightEventType::StallWarningOff);
    stallWarningWas_ = vane;
    // Touchdown/lift-off: a state change must persist 0.25 s (skipping
    // wheels and a tyre momentarily unloading are not separate events). The
    // touchdown sink rate is the one at first contact.
    if (tel_.onGround != wasOnGround_) {
        if (groundStateTimer_ == 0 && tel_.onGround) lastSinkRate_ = prev.body.velocity.z;
        groundStateTimer_ += dt;
        if (groundStateTimer_ >= 0.25) {
            pushEvent(tel_.onGround ? FlightEventType::Touchdown : FlightEventType::Liftoff,
                      tel_.onGround ? lastSinkRate_ : tel_.trueAirspeed);
            wasOnGround_ = tel_.onGround;
            groundStateTimer_ = 0;
        }
    } else {
        groundStateTimer_ = 0;
    }
    const Real ias = instruments_.readings().indicatedAirspeed;
    const Real hyst = 1.0;  // m/s: limit exceedances clear 1 m/s below the limit
    const bool over = def_.limits.neverExceedIas > 0 &&
                      ias > def_.limits.neverExceedIas - (overspeedWas_ ? hyst : 0.0);
    if (over && !overspeedWas_) pushEvent(FlightEventType::Overspeed, ias);
    overspeedWas_ = over;
    const bool machOver = def_.limits.maxOperatingMach > 0 &&
                          tel_.mach > def_.limits.maxOperatingMach - (machOverWas_ ? 0.005 : 0.0);
    if (machOver && !machOverWas_ && !over) pushEvent(FlightEventType::MachOverspeed, tel_.mach);
    machOverWas_ = machOver;
    const Real nLim = flapAngle_ > 0.01 ? def_.limits.limitLoadFlaps : def_.limits.limitLoadPositive;
    const Real gHyst = overGWas_ ? 0.1 : 0.0;
    const bool overG = tel_.loadFactor > nLim - gHyst || tel_.loadFactor < def_.limits.limitLoadNegative + gHyst;
    if (overG && !overGWas_) pushEvent(FlightEventType::OverG, tel_.loadFactor);
    overGWas_ = overG;
    // Structure: past the ultimate load factor (1.5 x the limit, 14 CFR 23/25.303)
    // for 50 ms, it has failed.
    const bool overUltimate = tel_.loadFactor > 1.5 * nLim || tel_.loadFactor < 1.5 * def_.limits.limitLoadNegative;
    overUltimate_ = overUltimate ? overUltimate_ + dt : 0.0;
    if (!structuralFailure_ && overUltimate_ >= 0.05) {
        structuralFailure_ = true;
        pushEvent(FlightEventType::StructuralFailure, tel_.loadFactor);
    }
    // Landing gear: a leg struck the ground closing faster than it can absorb.
    for (std::size_t i = 0; i < def_.gear.size() && i < static_cast<std::size_t>(kMaxGear); ++i) {
        const GearDef& g = def_.gear[i];
        const Real limit = g.collapseSpeed > 0 ? g.collapseSpeed : (turbofanCount() > 0 ? 4.27 : 3.66);
        if (!gearFailed_[i] && gearTel_[i].contact && gearTel_[i].closureSpeed > limit) {
            gearFailed_[i] = true;
            gearTel_[i].failed = true;
            pushEvent(FlightEventType::GearCollapse, gearTel_[i].closureSpeed, static_cast<int>(i));
        }
    }
    // The crash-test dummy in the pilot's seat.
    {
        const Vec3 seat = def_.stations.empty() ? tel_.cg : def_.stations[0].position;
        const Vec3 wDot = (s_.body.omega - prev.body.omega) / dt;
        occupant_.update(dt, tel_.specificForceBody, s_.body.omega, wDot, seat - tel_.cg);
    }
    const Real vfe = !def_.limits.flapExtendedIasVsFlap.empty() ? def_.limits.flapExtendedIasVsFlap(flapAngle_)
                                                                : def_.limits.flapExtendedIas;
    const bool flapOver = flapAngle_ > 0.01 && vfe > 0 && ias > vfe - (flapOverspeedWas_ ? hyst : 0.0);
    if (flapOver && !flapOverspeedWas_) pushEvent(FlightEventType::FlapOverspeed, ias);
    flapOverspeedWas_ = flapOver;
    const Real vle = def_.controls.gear.maxExtendedIas;
    const bool gearOver = def_.controls.gear.retractable && vle > 0 && gearPos_ > 0.01 &&
                          ias > vle - (gearOverspeedWas_ ? hyst : 0.0);
    if (gearOver && !gearOverspeedWas_) pushEvent(FlightEventType::GearOverspeed, ias);
    gearOverspeedWas_ = gearOver;
    for (int k = 0; k < electrical_.generatorCount(); ++k) {
        const bool on = electrical_.generatorOnline(k);
        if (on != genOnlineWas_[static_cast<std::size_t>(k)])
            pushEvent(on ? FlightEventType::AlternatorOnline : FlightEventType::AlternatorOffline, 0, k);
        genOnlineWas_[static_cast<std::size_t>(k)] = on;
    }
    if (electrical_.batteryDepleted() && !batteryDepletedWas_ && electrical_.batterySwitch()) pushEvent(FlightEventType::BatteryDepleted);
    batteryDepletedWas_ = electrical_.batteryDepleted();
    // value: 1 the bus controller's automatic shed, 0 the pilot's.
    if (electrical_.loadShed() != loadShedWas_)
        pushEvent(electrical_.loadShed() ? FlightEventType::LoadShed : FlightEventType::LoadRestored, electrical_.autoLoadShed() ? 1 : 0);
    loadShedWas_ = electrical_.loadShed();
    for (std::size_t i = 0; i < def_.contacts.size(); ++i) {
        // Edge-triggered: one event per new contact.
        if (contactTouched_[i] && !contactWas_[i]) {
            const bool prop = def_.contacts[i].name.find("prop") != std::string::npos;
            // value: surface id struck (5 = building, see avd/world_ground.hpp).
            pushEvent(prop ? FlightEventType::PropStrike : FlightEventType::HardPointContact,
                      ctx_.contactGround[i].surface.id, static_cast<int>(i));
        }
        contactWas_[i] = contactTouched_[i];
    }
}

bool Aircraft::loadPowered(const std::string& name, bool fallback) const {
    const int i = electrical_.loadIndex(name);
    return i >= 0 ? electrical_.loadPowered(i) : fallback;
}

SensorInputs Aircraft::sensorInputs() const {
    SensorInputs in;
    const Quat& q = s_.body.attitude;
    const Vec3 vBody = q.inverseRotate(s_.body.velocity);
    const Vec3 windBody = q.inverseRotate(ctx_.air.wind);
    const Vec3 rPitot = def_.pitotStatic.pitotPosition - mass_.cg;
    in.time = time_;
    in.air = ctx_.air.state;
    in.pitotAirVelocityBody = vBody - windBody + cross(s_.body.omega, rPitot);
    in.staticPortAlpha = aeroOut_.alpha;
    in.trueAirspeed = aeroOut_.airspeed;
    in.groundVelocityNed = s_.body.velocity;
    in.attitude = q;
    in.omegaBody = s_.body.omega;
    // Specific force at the CG (the panel is ~1 m away; rotational terms at
    // that lever arm are below 0.01 g in normal manoeuvres).
    in.specificForceBody = tel_.specificForceBody;
    // Exact geodetic position (the NAV receiver needs metre-level bearings;
    // a single-radius approximation is ~0.7% off at 23.5 S) with the
    // flat-frame altitude consistent with the projected game world.
    in.position = frame_.geodeticFromNed(s_.body.position);
    in.position.height = altitudeMsl();
    in.heightAboveGround = ctx_.wingHeightAgl;
    in.suction = def_.vacuum.present ? vacuum_.suction() : 0.0;
    in.vacuumInstalled = def_.vacuum.present;
    in.busVoltage = electrical_.busVoltage();
    in.batteryCurrent = electrical_.batteryCurrent();
    in.alternatorOnline = electrical_.alternatorOnline();
    in.loadShed = electrical_.loadShed();
    const InstrumentPanelDef& p = def_.panel;
    const bool bus = electrical_.powered();
    in.turnCoordinatorPowered = loadPowered(p.turnCoordinator.electricalLoad, bus);
    in.attitudeElectricPowered = loadPowered(p.attitude.electricalLoad, bus);
    in.headingElectricPowered = loadPowered(p.heading.electricalLoad, bus);
    in.gaugesPowered = loadPowered(p.engine.gaugeLoad, bus);
    in.avionicsPowered = loadPowered(p.avionics.avionicsLoad, electrical_.avionicsBusVoltage() >= electrical_.busMinimum());
    in.annunciatorsPowered = bus;
    in.pitotHeatOn = loadPowered(p.pitotHeatLoad, false);
    in.liquidWaterContent = ctx_.air.liquidWaterContent * 1e-3;  // g/m^3 -> kg/m^3
    in.panelLightsOn = loadPowered(p.panelLightsLoad, false);
    if (engineCount() > 0) {
        const PistonEngineState& e = engineState_[0];
        in.engineRpm = s_.shaftOmega[0] * 60 / kTwoPi;
        in.oilPressure = e.oilPressure;
        in.oilTemperature = e.oilTemperature;
        in.egt = e.egt;
        in.cht = e.cht;
        in.fuelFlow = e.fuelFlow;
    } else if (turbofanCount() > 0) {
        // Engine gauges read engine 1: N2 % as "rpm", ITT as EGT, total fuel flow.
        const TurbofanState& t = turboState_[0];
        in.engineRpm = t.n2 * 100;
        in.oilPressure = t.oilPressure;
        in.egt = t.itt;
        in.fuelFlow = 0;
        for (int i = 0; i < turbofanCount(); ++i) in.fuelFlow += turboState_[i].fuelFlow;
    }
    // Every engine's senders.
    in.engineCount = std::min(engineCount() + turbofanCount(), 4);
    for (int i = 0; i < engineCount() && i < 4; ++i) {
        SensorInputs::EngineSense& es = in.engines[static_cast<std::size_t>(i)];
        const PistonEngineState& e = engineState_[i];
        es.n1 = s_.shaftOmega[i] * 60 / kTwoPi;
        es.n2 = 0;
        es.itt = e.egt;
        es.fuelFlow = e.fuelFlow;
        es.oilPressure = e.oilPressure;
        es.oilTemperature = e.oilTemperature;
        es.electric = false;  // mechanical tachometer
    }
    for (int i = 0; i < turbofanCount() && i < 4; ++i) {
        SensorInputs::EngineSense& es = in.engines[static_cast<std::size_t>(i)];
        const TurbofanState& t = turboState_[i];
        es.n1 = t.n1 * 100;
        es.n2 = t.n2 * 100;
        es.itt = t.itt;
        es.fuelFlow = t.fuelFlow;
        es.oilPressure = t.oilPressure;
        es.oilTemperature = 0;
        es.electric = true;
    }
    in.fuelDensity = def_.fuel.density;
    in.tankCount = std::min(static_cast<int>(def_.fuel.tanks.size()), 4);
    for (int i = 0; i < in.tankCount; ++i) {
        in.fuelQuantity[i] = fuel_.tankQuantity(i);
        in.fuelUnusable[i] = def_.fuel.tanks[i].unusable;
    }
    in.flapAngle = flapAngle_;
    in.elevatorTrim = trim_;
    in.stallVaneTriggered = stallVane_;
    in.nav = nav_;
    return in;
}

void Aircraft::pushEvent(FlightEventType t, Real value, int index) {
    if (events_.size() >= 256) events_.erase(events_.begin());
    events_.push_back({t, time_, value, index});
}

std::vector<FlightEvent> Aircraft::drainEvents() {
    std::vector<FlightEvent> out;
    out.swap(events_);
    return out;
}

void Aircraft::updateTelemetry() {
    AircraftTelemetry& t = tel_;
    const Quat& q = s_.body.attitude;
    t.time = time_;
    t.positionNed = s_.body.position;
    t.velocityNed = s_.body.velocity;
    t.altitudeMsl = altitudeMsl();
    t.heightAgl = ctx_.wingHeightAgl;
    t.roll = q.roll();
    t.pitch = q.pitch();
    t.heading = q.yaw();
    t.omega = s_.body.omega;
    t.trueAirspeed = aeroOut_.airspeed;
    t.groundSpeed = std::hypot(s_.body.velocity.x, s_.body.velocity.y);
    t.verticalSpeed = -s_.body.velocity.z;
    t.alpha = aeroOut_.alpha;
    t.beta = aeroOut_.beta;
    t.mach = aeroOut_.mach;
    t.dynamicPressure = aeroOut_.dynamicPressure;
    t.loadFactor = -t.specificForceBody.z / kStandardGravity;
    t.mass = mass_.mass;
    t.cg = mass_.cg;
    t.fuelMass = fuel_.totalQuantity() * def_.fuel.density;
    t.flapAngle = flapAngle_;
    t.elevatorTrim = trim_;
    t.onGround = false;
    t.stalledStrips = 0;
    for (int s = 0; s < aeroModel_->surfaceCount(); ++s) t.stalledStrips += aeroOut_.surfaces[s].stalledStrips;
    for (std::size_t i = 0; i < def_.gear.size(); ++i) {
        t.gear[i] = gearTel_[i];
        t.onGround = t.onGround || gearTel_[i].contact;
    }
    t.busVoltage = electrical_.busVoltage();
    t.air = ctx_.air;
    for (int i = 0; i < engineCount(); ++i) {
        t.engines[i].engine = engineState_[i];
        t.engines[i].prop = propState_[i];
        t.engines[i].rpm = s_.shaftOmega[i] * 60 / kTwoPi;
    }
    for (int i = 0; i < turbofanCount(); ++i) t.turbofans[i] = turboState_[i];
    t.throttles = lever_;
    t.gearPosition = gearPos_;
    t.speedbrake = speedbrakePos_;
    t.controls = ctx_.controls;
    t.hydraulicPressure = hydPressure_;
    t.gearPower = gearSupply_;
    t.flapPower = flapSupply_;
    t.speedbrakePower = speedbrakeSupply_;
    t.steeringPower = steeringSupply_;
    t.brakePower = brakeSupply_;
    t.gearEmergency = gearEmergency_;
    t.brakeAccumulator = brakeAccumulator_;
}

// ----------------------------------------------------------- hydraulics ----

Real Aircraft::hydraulicSupply(const std::vector<int>& systems) const {
    if (!def_.hydraulics.present || systems.empty()) return 1;
    Real best = 0;
    for (int k : systems) best = std::max(best, hydPressure_[static_cast<std::size_t>(k)]);
    return best;
}

Real Aircraft::hydraulicSource(std::size_t k, Real* electricCurrent) const {
    const HydraulicSystemDef& sys = def_.hydraulics.systems[k];
    Real source = 0;
    for (int e : sys.engines) {
        if (engineCount() > 0) source = std::max(source, pistonPumpOutput(s_.shaftOmega[e] * 60 / kTwoPi));
        else source = std::max(source, turbofanPumpOutput(turboState_[e], def_.turbofans[static_cast<std::size_t>(e)]));
    }
    const HydraulicElectricPumpDef& ep = sys.electricPump;
    if (ep.current > 0 && hydPumpOn_[k] && electrical_.powered() && (!ep.generatorOnly || electrical_.alternatorOnline())) {
        source = 1;
        if (electricCurrent) *electricCurrent += ep.current;
    }
    return source;
}

void Aircraft::settleHydraulics() {
    // A spawn or trim describes a settled state: each system at its source's pressure.
    for (std::size_t k = 0; k < def_.hydraulics.systems.size(); ++k) hydPressure_[k] = hydraulicSource(k, nullptr);
    gearSupply_ = hydraulicSupply(def_.hydraulics.gear);
    flapSupply_ = hydraulicSupply(def_.hydraulics.flaps);
    speedbrakeSupply_ = hydraulicSupply(def_.hydraulics.speedbrake);
    steeringSupply_ = hydraulicSupply(def_.hydraulics.steering);
}

void Aircraft::stepHydraulics(Real dt, const PilotInput& in) {
    const HydraulicsDef& h = def_.hydraulics;
    hydPumpCurrent_ = 0;
    if (!h.present) return;
    for (std::size_t k = 0; k < h.systems.size(); ++k) {
        const Real source = hydraulicSource(k, &hydPumpCurrent_);
        hydPressure_[k] += (source - hydPressure_[k]) * std::min(dt / h.systems[k].riseTime, 1.0);
    }
    gearSupply_ = hydraulicSupply(h.gear);
    flapSupply_ = hydraulicSupply(h.flaps);
    speedbrakeSupply_ = hydraulicSupply(h.speedbrake);
    steeringSupply_ = hydraulicSupply(h.steering);
    // Brakes: full effect with pressure; with every supplying system down the
    // accumulator gives a number of full applications (each press, or the
    // parking brake set, spends one), then nothing.
    if (h.brakes.empty()) {
        brakeSupply_ = 1;
        return;
    }
    const bool pressurised = actuatorRate(hydraulicSupply(h.brakes)) > 0;
    if (pressurised) brakeAccumulator_ = h.brakeAccumulatorApplications;  // recharged by the system
    const bool applied = parkingBrake_ || in.brakeLeft > 0.1 || in.brakeRight > 0.1;
    if (applied && !brakeWasApplied_) {
        brakeHeld_ = pressurised;
        if (!pressurised && brakeAccumulator_ > 0) {
            --brakeAccumulator_;
            brakeHeld_ = true;
        }
    }
    if (!applied) brakeHeld_ = false;
    brakeWasApplied_ = applied;
    brakeSupply_ = pressurised || brakeHeld_ ? 1.0 : 0.0;
}

}  // namespace avd
