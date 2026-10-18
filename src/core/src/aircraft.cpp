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
    rebuildCockpit();
    setColdAndDark();
}

// ------------------------------------------------------------------ mass ----

void Aircraft::refreshMass() {
    massItems_.clear();
    MassItem empty;
    empty.name = "empty airframe";
    empty.mass = def_.emptyMass;
    empty.position = def_.emptyCg;
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
    stationMass_[station] = clamp(kg, 0.0, def_.stations[station].maxMass);
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
    cockpit_.push_back({"electrical.alternator", "Alternator Master Switch", {"OFF", "ON"}, 0});
    cockpit_.push_back({"electrical.avionics", "Avionics Master Switch", {"OFF", "ON"}, 0});
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
    if (def_.controls.gear.retractable) cockpit_.push_back({"gear.lever", "Landing Gear Lever", {"UP", "DOWN"}, 1});
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
    if (id == "fuel.selector") return fuel_.selector();
    if (id == "fuel.shutoff") return fuel_.shutoffOpen();
    if (id == "fuel.auxpump") return fuel_.auxPumpOn();
    if (id == "flaps.lever") return flapDetent_;
    if (id == "brakes.parking") return parkingBrake_;
    if (id == "gear.lever") return def_.controls.gear.retractable ? (gearDown_ ? 1 : 0) : -1;
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
    if (id == "electrical.battery") electrical_.setBatterySwitch(position == 1);
    else if (id == "electrical.alternator") electrical_.setAlternatorSwitch(position == 1);
    else if (id == "electrical.avionics") electrical_.setAvionicsSwitch(position == 1);
    else if (id == "fuel.selector") fuel_.setSelector(position);
    else if (id == "fuel.shutoff") fuel_.setShutoffOpen(position == 1);
    else if (id == "fuel.auxpump") fuel_.setAuxPump(position == 1);
    else if (id == "flaps.lever") flapDetent_ = position;
    else if (id == "brakes.parking") parkingBrake_ = position == 1;
    else if (id == "gear.lever") gearDown_ = position == 1;
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
    parkingBrake_ = true;
    for (std::size_t i = 0; i < def_.electrical.loads.size(); ++i)
        electrical_.setLoadSwitch(static_cast<int>(i), def_.electrical.loads[i].initiallyOn);
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
    // Establish bus voltage (battery, then the alternator's field) for the spawn.
    ElectricalInputs ei;
    ei.alternatorRpm = turbofanCount() > 0 ? 2400.0 : rpm;
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
}

void Aircraft::buildContext(const PilotInput& in, const GroundQuery& ground, const AtmosphereField& atm, Real dt) {
    Context& c = ctx_;
    const Quat& q = s_.body.attitude;
    const Vec3& pos = s_.body.position;
    const Real alt = frame_.origin().height - pos.z;
    c.air = atm.sample(pos, alt, time_);
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
    }

    // Flight controls. Trim acts as an equivalent elevator offset (the
    // stick-free trim-tab hinge-moment model is planned; see fidelity matrix).
    Real elevUp = 0.4;
    for (const auto& s : def_.surfaces)
        for (const auto& p : s.panels)
            for (const auto& cs : p.controls)
                if (cs.channel == ControlChannel::Elevator) elevUp = cs.maxNegative;
    c.controls.aileron = clamp(in.roll + def_.controls.riggingAileron, -1.0, 1.0);
    c.controls.elevator = clamp(in.pitch + trim_ * def_.controls.elevatorTrimRange / elevUp, -1.0, 1.0);
    if (pusher_) c.controls.elevator = std::min(c.controls.elevator, def_.stallWarning.pusherElevator);
    c.controls.rudder = clamp(in.yaw + def_.controls.riggingRudder, -1.0, 1.0);
    if (def_.controls.rudderLimiterIas > 0) {
        const Real eas = length(s_.body.velocity - c.air.wind) * std::sqrt(c.air.state.density / isa::kSeaLevelDensity);
        if (eas > def_.controls.rudderLimiterIas)
            c.controls.rudder *= std::max(0.15, sq(def_.controls.rudderLimiterIas / eas));
    }
    c.controls.flap = flapAngle_;
    c.controls.spoiler = def_.controls.speedbrake.present ? speedbrakePos_ * def_.controls.speedbrake.liftLossFraction : 0.0;
    // Wheel steering follows the pedals (not the rudder tab) plus the tiller,
    // through the steering actuator's slew limit where it is powered.
    for (std::size_t i = 0; i < def_.gear.size(); ++i) {
        const GearDef& g = def_.gear[i];
        const Real authority = std::max(g.maxSteerAngle, g.tillerSteerAngle);
        const Real cmd = clamp(g.maxSteerAngle * clamp(in.yaw, -1.0, 1.0) + g.tillerSteerAngle * clamp(in.tiller, -1.0, 1.0),
                               -authority, authority);
        if (g.steerRate > 0 && dt > 0) steerAngle_[i] = approach(steerAngle_[i], cmd, g.steerRate * dt);
        else steerAngle_[i] = cmd;
        c.steer[i] = steerAngle_[i];
    }
    c.brakeLeft = parkingBrake_ ? 1.0 : clamp(in.brakeLeft, 0.0, 1.0);
    c.brakeRight = parkingBrake_ ? 1.0 : clamp(in.brakeRight, 0.0, 1.0);

    for (int i = 0; i < engineCount(); ++i) {
        PistonEngineInputs& e = c.engineIn[i];
        e.throttle = clamp(in.throttle, 0.0, 1.0);
        e.mixture = clamp(in.mixture, 0.0, 1.0);
        e.magnetos = (in.starterHeld && magnetos_[i] == Magnetos::Both) ? Magnetos::Start : magnetos_[i];
        e.fuelAtEngine = fuel_.fuelAtEngine();
        e.ambientPressure = c.air.state.pressure;
        e.ambientTemperature = c.air.state.temperature;
        e.ambientDensity = c.air.state.density;
        e.starterPowered = false;
    }
    const Real airspeed = length(s_.body.velocity - c.air.wind);
    for (int i = 0; i < turbofanCount(); ++i) {
        TurbofanInputs& e = c.turboIn[i];
        e.throttle = clamp(in.throttle, 0.0, 1.0);
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
        const Vec3 thrust = normalized(ed.thrustAxis) * ts.thrust;
        force += thrust;
        moment += cross(r, thrust);
    }
    // ---- Landing gear and speedbrake drag areas ----
    {
        const Real cda = def_.controls.gear.dragArea * (def_.controls.gear.retractable ? gearPos_ : 0.0) +
                         (def_.controls.speedbrake.present ? def_.controls.speedbrake.dragArea * speedbrakePos_ : 0.0);
        if (cda > 0) {
            const Vec3 vAir = vBody - windBody;
            force -= vAir * (0.5 * rho * length(vAir) * cda);
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
    ai.slipstreams = slip_.data();
    ai.slipstreamCount = engineCount();
    ai.downwashLagged = quasiSteadyDownwash_ ? nullptr : downwashLag_.data();
    aeroModel_->evaluate(ai, aeroOut_);
    force += aeroOut_.force;
    moment += aeroOut_.moment;

    // ---- Landing gear ----
    for (std::size_t i = 0; i < def_.gear.size(); ++i) {
        const GearDef& g = def_.gear[i];
        const GroundPlane& gp = c.gearGround[i];
        GearTelemetry gt;
        d.tyreLateralRate[i] = -s.tyreLateral[i] * 20;
        d.tyreLongitudinalRate[i] = -s.tyreLongitudinal[i] * 20;
        const bool legDown = !def_.controls.gear.retractable || gearPos_ > 0.98;
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
                const Real steer = c.steer[i];
                Vec3 f = q.rotate({std::cos(steer), std::sin(steer), 0});
                f = normalized(f - n * dot(f, n));
                const Vec3 l = cross(n, f);
                const Real vLong = dot(vPt, f), vLat = dot(vPt, l);
                const Real mu = gp.surface.friction;
                const Real brake = g.brakeGroup == 0 ? c.brakeLeft : (g.brakeGroup == 1 ? c.brakeRight : 0.0);
                const Real limLong = std::min(N * (g.rollingResistance * gp.surface.rollingResistance +
                                                   brake * g.maxBrakeForceCoefficient),
                                              mu * N);
                const Real kx = g.longitudinalStiffness;
                const Real fLong = -saturate(kx * s.tyreLongitudinal[i] + 0.05 * kx * vLong, limLong);
                const Real ky = g.corneringStiffness / g.relaxationLength;
                const Real fLat = -saturate(ky * s.tyreLateral[i] + 0.02 * ky * vLat, mu * N);
                d.tyreLongitudinalRate[i] = vLong;
                d.tyreLateralRate[i] = vLat - std::abs(vLong) * s.tyreLateral[i] / g.relaxationLength;
                const Vec3 fNed = n * N + f * fLong + l * fLat;
                const Vec3 fB = q.inverseRotate(fNed);
                force += fB;
                moment += cross(rc, fB);
                gt.contact = N > 0;
                gt.compression = x;
                gt.normalForce = N;
                gt.steerAngle = steer;
                gt.surfaceId = gp.surface.id;
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
    }
    return r;
}
}  // namespace

void Aircraft::step(Real dt, const PilotInput& in, const GroundQuery& ground, const AtmosphereField& atm) {
    if (!(dt > 0) || faulted_) return;
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
    const int pumpLoad = electrical_.loadIndex("fuel_pump");
    ElectricalInputs ei;
    ei.alternatorRpm = engineCount() > 0 ? s_.shaftOmega[0] * 60 / kTwoPi : 0;
    // Turbofan starter-generators: crank while starting, generate above ~45 % N2.
    for (int i = 0; i < turbofanCount(); ++i) {
        if (turboState_[i].starting && turboState_[i].n2 < 0.45) starterCurrent += 250;
        // Scaled so the generator comes on line at 45 % N2 (a windmilling core does not hold the bus).
        ei.alternatorRpm = std::max(ei.alternatorRpm, turboState_[i].n2 * 2000);
    }
    ei.starterCurrent = starterCurrent;
    ei.flapMotorCurrent = flapMoving_ && def_.controls.flapsElectric ? def_.controls.flapMotorCurrent : 0;
    ei.auxPumpCurrent = fuel_.auxPumpOn() ? 5.0 : 0.0;
    electrical_.step(dt, ei);
    for (int i = 0; i < engineCount(); ++i)
        c.engineIn[i].starterPowered = c.engineIn[i].magnetos == Magnetos::Start &&
                                       electrical_.busVoltage() >= kMinStarterVoltage;
    for (int i = 0; i < turbofanCount(); ++i) {
        c.turboIn[i].starterPowered = electrical_.busVoltage() >= kMinStarterVoltage;
        if (!turbofanSteady_) turbofans_[i].step(turboState_[i], c.turboIn[i], dt);
    }
    if (def_.controls.gear.retractable)
        gearPos_ = approach(gearPos_, gearDown_ ? 1.0 : 0.0, dt / def_.controls.gear.transitTime);
    if (def_.controls.speedbrake.present) {
        const Real target = static_cast<Real>(speedbrakeDetent_) / std::max(def_.controls.speedbrake.detents - 1, 1);
        speedbrakePos_ = approach(speedbrakePos_, target, def_.controls.speedbrake.rate * dt);
        c.controls.spoiler = speedbrakePos_ * def_.controls.speedbrake.liftLossFraction;
    }
    const bool flapPowered = !def_.controls.flapsElectric ||
                             (electrical_.powered(18) && (flapLoad < 0 || electrical_.loadSwitch(flapLoad)));
    if (flapPowered) flapAngle_ = approach(flapAngle_, flapTarget, def_.controls.flapRate * dt);
    c.controls.flap = flapAngle_;
    trim_ = clamp(trim_ + in.trimRate * def_.controls.elevatorTrimRate * dt, -1.0, 1.0);
    (void)pumpLoad;

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

    // Tyre bristles cannot exceed their friction limits.
    for (std::size_t i = 0; i < def_.gear.size(); ++i) {
        const GearDef& g = def_.gear[i];
        const GearTelemetry& gt = gearTel_[i];
        if (!gt.contact) continue;
        const Real mu = c.gearGround[i].surface.friction;
        const Real ky = g.corneringStiffness / g.relaxationLength;
        s_.tyreLateral[i] = saturate(s_.tyreLateral[i], mu * gt.normalForce / ky);
        const Real brake = g.brakeGroup == 0 ? c.brakeLeft : (g.brakeGroup == 1 ? c.brakeRight : 0.0);
        const Real lim = std::min(gt.normalForce * (g.rollingResistance * c.gearGround[i].surface.rollingResistance +
                                                    brake * g.maxBrakeForceCoefficient),
                                  mu * gt.normalForce);
        s_.tyreLongitudinal[i] = saturate(s_.tyreLongitudinal[i], lim / g.longitudinalStiffness);
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
    const bool auxPump = fuel_.auxPumpOn() && electrical_.powered(18);
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
        const int idx = def_.stallWarning.strip >= 0 ? first + def_.stallWarning.strip : (first + end) / 2;
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
    if (electrical_.alternatorOnline() != altOnlineWas_)
        pushEvent(electrical_.alternatorOnline() ? FlightEventType::AlternatorOnline : FlightEventType::AlternatorOffline);
    altOnlineWas_ = electrical_.alternatorOnline();
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
    in.busVoltage = electrical_.busVoltage();
    in.batteryCurrent = electrical_.batteryCurrent();
    in.alternatorOnline = electrical_.alternatorOnline();
    const InstrumentPanelDef& p = def_.panel;
    const bool bus = electrical_.powered(18);
    in.turnCoordinatorPowered = loadPowered(p.turnCoordinator.electricalLoad, bus);
    in.attitudeElectricPowered = loadPowered(p.attitude.electricalLoad, bus);
    in.headingElectricPowered = loadPowered(p.heading.electricalLoad, bus);
    in.gaugesPowered = loadPowered(p.engine.gaugeLoad, bus);
    in.avionicsPowered = loadPowered(p.avionics.avionicsLoad, electrical_.avionicsBusVoltage() >= 18);
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
    t.gearPosition = gearPos_;
    t.speedbrake = speedbrakePos_;
    t.controls = ctx_.controls;
}

}  // namespace avd
