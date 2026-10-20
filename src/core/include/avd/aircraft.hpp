// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
//
// Aircraft: one simulated airframe built from an AircraftDefinition. Owns its
// rigid-body state, systems, propulsion, aerodynamics, gear and instruments,
// and advances them with a fixed timestep. Single-threaded; different
// aircraft share no mutable state.
//
// One step (see docs/architecture/flight-dynamics.md):
//   1. Ground planes under each gear/contact point (one query per point per
//      step), air sample at the CG (with its spatial gradient).
//   2. Systems from the state at step start: electrical, flaps, trim, engine
//      controls, fuel at the engine inlet.
//   3. RK4 integration of the coupled state: CG position and velocity (local
//      NED), attitude quaternion (body FRD -> NED), body rates, engine/prop
//      shaft speeds, and tyre bristle deflections. Forces: aerodynamics
//      (strip build-up), propeller thrust/torque/P-factor/slipstream, engine
//      reaction and rotor gyroscopics, gear and hard-point contact, gravity.
//   4. Fuel burn and the resulting mass/CG/inertia update (datum kept
//      continuous), downwash lag, vacuum, engine temperatures, instruments,
//      events and telemetry.
#pragma once

#include <array>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "avd/aero.hpp"
#include "avd/aircraft_def.hpp"
#include "avd/atmosphere.hpp"
#include "avd/geodesy.hpp"
#include "avd/ground.hpp"
#include "avd/instruments.hpp"
#include "avd/mass.hpp"
#include "avd/propulsion.hpp"
#include "avd/systems.hpp"

namespace avd {

inline constexpr int kMaxEngines = 4;
inline constexpr int kMaxGear = 8;
inline constexpr int kMaxContacts = 16;

// Pilot (or assist) control positions, sampled every step. Persistent
// cockpit switches (master, magnetos, fuel valves, flap lever, parking brake)
// are changed through the cockpit-control API instead.
// Turbofan thrust-rating selection (the FADEC's N1 limit). Auto: take-off on
// the ground and for the definition's take-off time after lift-off (re-armed
// by a go-around: levers to the stop with the gear or flaps out), then
// maximum continuous with an engine out, maximum climb while climbing and
// maximum cruise once level for a minute. The others force one rating.
enum class ThrustMode { Auto = 0, Takeoff = 1, MaxContinuous = 2, Climb = 3, Cruise = 4 };

struct PilotInput {
    Real pitch = 0;        // -1..1, + stick aft (nose up)
    Real roll = 0;         // -1..1, + roll right
    Real yaw = 0;          // -1..1, + right pedal
    Real tiller = 0;       // -1..1, + steer right (nose-wheel tiller, where fitted)
    Real throttle = 0;     // 0..1, every engine
    // Split levers (asymmetric thrust, differential taxi): with
    // `splitThrottles` set, engine i follows throttles[i] and `throttle` is
    // ignored; code that sets only `throttle` drives every engine as before.
    std::array<Real, kMaxEngines> throttles{};
    bool splitThrottles = false;
    Real engineThrottle(int engine) const { return splitThrottles ? throttles[static_cast<std::size_t>(engine)] : throttle; }
    ThrustMode thrustMode = ThrustMode::Auto;  // turbofans: the rating that caps N1
    Real mixture = 1;      // 0 (idle cut-off) .. 1 (full rich)
    Real brakeLeft = 0, brakeRight = 0;  // 0..1 toe brakes
    Real trimRate = 0;     // -1..1, + nose-up trim wheel movement
    bool starterHeld = false;  // ignition key held at START
};

struct RigidBodyState {
    Vec3 position;         // CG, local NED (m)
    Vec3 velocity;         // CG, local NED (m/s)
    Quat attitude;         // body FRD -> NED
    Vec3 omega;            // body rates p, q, r (rad/s)
};

struct DynamicState {
    RigidBodyState body;
    std::array<Real, kMaxEngines> shaftOmega{};
    std::array<Real, kMaxGear> tyreLateral{};       // bristle deflection (m)
    std::array<Real, kMaxGear> tyreLongitudinal{};  // bristle deflection (m), driven by the slip velocity v - r omega
    std::array<Real, kMaxGear> wheelOmega{};        // wheel spin (rad/s, + rolling forward)
    std::array<Real, kMaxGear> wheelSteer{};        // castoring wheel's heading (rad; unused for rigidly steered legs)
};

enum class FlightEventType {
    EngineStarted, EngineStopped, FuelStarvation, StallWarningOn, StallWarningOff,
    Touchdown, Liftoff, Overspeed, OverG, FlapOverspeed, PropStrike, HardPointContact,
    BatteryDepleted, AlternatorOffline, AlternatorOnline, CockpitControlChanged,
    MachOverspeed, GearOverspeed,
    LoadShed, LoadRestored,  // non-essential loads shed (bus controller or pilot) / back
};

struct FlightEvent {
    FlightEventType type;
    Real time = 0;
    Real value = 0;        // event-specific (sink rate at touchdown m/s; surface id for contacts)
    int index = -1;        // engine/gear/contact/control index
};

// A cockpit control the player can look at and operate ("Press F to interact").
struct CockpitControl {
    std::string id;        // stable id, e.g. "fuel.shutoff"
    std::string label;     // e.g. "Fuel Shutoff Valve"
    std::vector<std::string> positions;
    int position = 0;
};

struct GearTelemetry {
    bool contact = false;
    Real compression = 0;
    Real normalForce = 0;
    Real steerAngle = 0;         // the wheel's heading (a castoring wheel's own, else the command)
    int surfaceId = 0;
    Real wheelOmega = 0;         // rad/s
    Real rollingSpeed = 0;       // m/s, contact point along the wheel heading
    Real slipRatio = 0;          // (v_long - r omega) / max(|v_long|, 1), + when braking
    Real longitudinalForce = 0;  // N on the airframe along the wheel heading (- braking)
    Real lateralForce = 0;       // N along n x f: to the wheel's left
    Real peakFriction = 0;       // mu_peak used (wet-aware)
    Real brakeTorque = 0;        // N m applied (after anti-skid modulation)
    bool held = false;           // the wheel is stopped and held by the brakes / rolling threshold
};

struct EngineTelemetry {
    PistonEngineState engine;
    PropellerForces prop;
    Real rpm = 0;
};

struct AircraftTelemetry {
    Real time = 0;
    Vec3 positionNed, velocityNed;
    Real altitudeMsl = 0, heightAgl = 0;
    Real roll = 0, pitch = 0, heading = 0;   // rad (true heading)
    Vec3 omega;
    Real trueAirspeed = 0, groundSpeed = 0, verticalSpeed = 0;  // m/s (+ up)
    Real alpha = 0, beta = 0, mach = 0, dynamicPressure = 0;
    Vec3 specificForceBody;                  // m/s^2 (accelerometer at CG)
    Real loadFactor = 0;                     // -specificForce.z / g
    Real mass = 0;
    Vec3 cg;
    Real fuelMass = 0;
    Real flapAngle = 0, elevatorTrim = 0;
    bool onGround = false;
    bool stallWarning = false;
    bool stickPusher = false;                // stall protection pushing the nose down
    bool yawDamper = false;                  // yaw damper engaged (in flight, fitted and switched on)
    Real yawDamperRudder = 0;                // its rudder command (fraction of full travel)
    int stalledStrips = 0;
    Real busVoltage = 0;
    AirSample air;
    std::array<EngineTelemetry, kMaxEngines> engines{};
    std::array<TurbofanState, kMaxEngines> turbofans{};
    std::array<Real, kMaxEngines> throttles{};  // lever position each engine followed this step
    Real gearPosition = 1;                   // 0 up and locked .. 1 down and locked
    Real speedbrake = 0;                     // 0 retracted .. 1 fully deployed
    // Hydraulics: pressure fraction of each system (1 = nominal), what each
    // actuator has (1 without hydraulics), the free-fall release, and the
    // brake accumulator's remaining applications (-1 without hydraulics).
    std::array<Real, 4> hydraulicPressure{};
    Real gearPower = 1, flapPower = 1, speedbrakePower = 1, steeringPower = 1, brakePower = 1;
    bool gearEmergency = false;
    int brakeAccumulator = -1;
    std::array<GearTelemetry, kMaxGear> gear{};
    ControlDeflections controls;
};

class Aircraft {
public:
    // Throws std::invalid_argument listing every validation error.
    explicit Aircraft(const AircraftDefinition& def, const LocalFrame& frame = LocalFrame{});

    const AircraftDefinition& definition() const { return def_; }
    const LocalFrame& frame() const { return frame_; }

    // ---- Initial conditions ------------------------------------------------
    // Airborne: CG position (NED), NED velocity, attitude, body rates.
    void setFlightState(const RigidBodyState& s);
    // On the ground: datum north/east, true heading; settles on the gear.
    void placeOnGround(Real north, Real east, Real heading, const GroundQuery& ground, const AtmosphereField& atm);
    // Convenience: systems configured as for a running engine (spawn
    // shortcut; the procedural Quick Start lives in avd/procedures.hpp).
    void setEngineRunning(bool running, Real rpm = 1000);
    // Spawn helper: gyros erect and at speed, needles at the current truth.
    // Reads the air from the current context: after setFlightState() call
    // prepareContext() (or step) first, or the pitot-static instruments are
    // seeded with the air at the previous position.
    void resetInstrumentsToTruth();
    void setColdAndDark();
    // Initial-condition setters (spawn/trim): flaps already at a detent, trim position.
    void setFlapsImmediately(int detent);
    void setGearImmediately(bool down) { gearDown_ = down; gearPos_ = down || !def_.controls.gear.retractable ? 1.0 : 0.0; syncCockpitPositions(); }
    void setElevatorTrim(Real trim) { trim_ = clamp(trim, -1.0, 1.0); }
    Real elevatorTrim() const { return trim_; }
    // True if any gear leg is steered by a tiller (GearDef::tillerSteerAngle).
    bool hasTiller() const;
    Real flapAngle() const { return flapAngle_; }

    // Re-anchor the local tangent frame (long-distance flight): position,
    // velocity and attitude are transformed exactly through ECEF so the
    // aircraft does not move; gravity then follows the new local vertical.
    void rebase(const LocalFrame& newFrame);

    // ---- Simulation --------------------------------------------------------
    void step(Real dt, const PilotInput& in, const GroundQuery& ground, const AtmosphereField& atm);

    // ---- State ---------------------------------------------------------------
    const DynamicState& state() const { return s_; }
    const AircraftTelemetry& telemetry() const { return tel_; }
    const MassProperties& massProperties() const { return mass_; }
    const AeroOutputs& aero() const { return aeroOut_; }
    const AeroModel& aeroModel() const { return *aeroModel_; }
    bool hasTrimmableStabilizer() const { return def_.controls.stabilizerNoseUp > 0 || def_.controls.stabilizerNoseDown > 0; }
    // Trim wheel position (-1..1) that gives the same pitching control as an
    // elevator (stick) input e: the elevator offset, or for a trimmable
    // stabiliser the tailplane angle making the same tail lift.
    Real trimForElevator(Real e) const;
    AeroModel& aeroModel() { return *aeroModel_; }
    const InstrumentPanel& instruments() const { return instruments_; }
    InstrumentPanel& instruments() { return instruments_; }
    // World navigation environment (magnetic field, radio facilities); shared,
    // owned by the host, may be null (then: no NAV signals, zero variation).
    void setNavEnvironment(const NavEnvironment* env) { nav_ = env; }
    const NavEnvironment* navEnvironment() const { return nav_; }
    Real time() const { return time_; }
    bool faulted() const { return faulted_; }
    Real altitudeMsl() const { return frame_.origin().height - s_.body.position.z; }
    // Datum (structural origin) in the local NED frame.
    Vec3 datumPosition() const;

    FuelSystem& fuel() { return fuel_; }
    const FuelSystem& fuel() const { return fuel_; }
    ElectricalSystem& electrical() { return electrical_; }
    const ElectricalSystem& electrical() const { return electrical_; }
    const VacuumSystem& vacuum() const { return vacuum_; }
    VacuumSystem& vacuum() { return vacuum_; }
    int engineCount() const { return static_cast<int>(engines_.size()); }
    const PistonEngine& engine(int i) const { return engines_[i]; }
    const Propeller& propeller(int i) const { return props_[i]; }
    int turbofanCount() const { return static_cast<int>(turbofans_.size()); }
    const TurbofanEngine& turbofan(int i) const { return turbofans_[i]; }
    const TurbofanState& turbofanState(int i) const { return turboState_[i]; }
    // The FADEC rating in force (ThrustMode::Auto's schedule, or the forced one).
    ThrustRating activeThrustRating() const { return activeRating_; }
    Real takeoffRatingTime() const { return takeoffRatingTimer_; }
    // Trim/spawn: turbofans lit and held at the commanded fan speed (no spool lag).
    void setTurbofansSteady(bool on) { turbofanSteady_ = on; }
    Real gearPosition() const { return gearPos_; }
    // Propeller slipstream of engine i at the last recorded evaluation (visualisation).
    const SlipstreamState& slipstream(int i) const { return slip_[i]; }
    Real speedbrakePosition() const { return speedbrakePos_; }
    // Thrust reverser of turbofan i: 0 stowed .. 1 deployed; the automatic
    // sequence's phase (0 stowed, 1 deploying / reverse thrust, 2 reverse
    // idle, 3 stowing).
    Real reverserPosition(int i) const { return reverserPos_[static_cast<std::size_t>(i)]; }
    int reverserPhase(int i) const { return reverserPhase_[static_cast<std::size_t>(i)]; }
    bool groundSpoilersActive() const { return groundSpoilers_; }
    // Hydraulics (HydraulicsDef): system pressure fraction, actuator supply.
    int hydraulicSystemCount() const { return static_cast<int>(def_.hydraulics.systems.size()); }
    Real hydraulicPressure(int i) const { return hydPressure_[static_cast<std::size_t>(i)]; }
    int brakeAccumulatorApplications() const { return brakeAccumulator_; }

    // ---- Loading -------------------------------------------------------------
    void setStationMass(int station, Real kg);
    Real stationMass(int station) const { return stationMass_[station]; }
    void refreshMass();  // after servicing changes (fuel/stations)

    // ---- Cockpit controls ----------------------------------------------------
    const std::vector<CockpitControl>& cockpitControls() const { return cockpit_; }
    int cockpitControlIndex(const std::string& id) const;
    bool setCockpitControl(const std::string& id, int position);
    bool cycleCockpitControl(const std::string& id);  // "press F"
    int cockpitControl(const std::string& id) const;

    // Direct switch state (used by the cockpit layer and procedures).
    Magnetos magnetos(int engine) const { return magnetos_[engine]; }
    int flapDetent() const { return flapDetent_; }
    bool parkingBrake() const { return parkingBrake_; }
    // The yaw damper switch (on by default; a fitted damper works in flight).
    void setYawDamper(bool on) { yawDamperOn_ = on; }
    bool yawDamperSwitch() const { return yawDamperOn_; }

    // ---- Events ----------------------------------------------------------------
    // Events since the last call (bounded queue; oldest dropped when full).
    std::vector<FlightEvent> drainEvents();

    // ---- Analysis hooks (trim, linearisation, tests) ---------------------------
    struct Derivatives {
        RigidBodyState body;   // position = velocity, velocity = accel, attitude = qdot, omega = alpha
        std::array<Real, kMaxEngines> shaftAccel{};
        std::array<Real, kMaxGear> tyreLateralRate{}, tyreLongitudinalRate{};
        std::array<Real, kMaxGear> wheelAlpha{}, wheelSteerRate{};
        Vec3 forceBody;        // total non-gravitational force
        Vec3 momentBody;
    };
    // Evaluate derivatives at an arbitrary state with the current controls and
    // environment of the last step (or initial ones).
    Derivatives evaluate(const DynamicState& s) const;
    // Prepare the environment/controls context without stepping (for trim).
    void prepareContext(const PilotInput& in, const GroundQuery& ground, const AtmosphereField& atm);
    void setState(const DynamicState& s) { s_ = s; }
    // Trim/linearisation use quasi-steady downwash (no transport lag);
    // settleDownwash() then initialises the lag states for a following run.
    void setQuasiSteadyDownwash(bool on) { quasiSteadyDownwash_ = on; }
    void settleDownwash();

private:
    struct GroundPlane {
        bool valid = false;
        Vec3 point, normal{0, 0, -1};
        SurfaceProperties surface;
    };
    struct Context {
        AirSample air;
        Vec3 gravity{0, 0, kStandardGravity};
        std::array<GroundPlane, kMaxGear> gearGround{};
        std::array<GroundPlane, kMaxContacts> contactGround{};
        Real wingHeightAgl = 1e9;
        Vec3 groundUpBody;  // ground normal (up) in body axes; zero without ground
        ControlDeflections controls;
        std::array<PistonEngineInputs, kMaxEngines> engineIn{};
        std::array<TurbofanInputs, kMaxEngines> turboIn{};
        Real brakeLeft = 0, brakeRight = 0;
        std::array<Real, kMaxGear> steer{};  // wheel steering command (rad)
        std::array<Real, kMaxGear> brakeModulation{};  // anti-skid: fraction of the pedal's torque let through
    };
    // Per-leg constants derived from the definition (see prepareGear()).
    struct GearRuntime {
        Real staticLoad = 1;       // N_ref
        Real corneringC1 = 0;      // C_alpha = c1 N (1 - c2 N / N_ref)
        Real inertia = 1;          // wheel spin inertia (kg m^2)
        Real brakeTorque = 0;      // N m at full pedal
        Real spinDamping = 0;      // N s/m, bristle damping on the slip velocity while the wheel turns
        Real heldDamping = 0;      // N s/m, while the wheel is held (the airframe's fore-aft mode)
    };
    void prepareGear();
    Real corneringStiffness(std::size_t leg, Real normalForce) const;

    void rebuildCockpit();
    void syncCockpitPositions();
    void buildContext(const PilotInput& in, const GroundQuery& ground, const AtmosphereField& atm, Real dt);
    Derivatives derive(const DynamicState& s, const Context& c, bool record) const;
    void postStep(Real dt, const Context& c, const DynamicState& prev);
    void pushEvent(FlightEventType t, Real value = 0, int index = -1);
    SensorInputs sensorInputs() const;
    bool loadPowered(const std::string& name, bool fallback) const;
    void updateTelemetry();
    // Pressure available to an actuator fed by `systems` (1 when the list is
    // empty or no hydraulics are fitted), and the rate factor it moves at.
    Real hydraulicSupply(const std::vector<int>& systems) const;
    static Real actuatorRate(Real supply) { return clamp((supply - 0.2) / 0.8, 0.0, 1.0); }
    void stepHydraulics(Real dt, const PilotInput& in);
    // Pressure a system's pumps deliver now (0..1); adds any electric pump's current.
    Real hydraulicSource(std::size_t system, Real* electricCurrent) const;
    void settleHydraulics();

    AircraftDefinition def_;
    LocalFrame frame_;
    std::unique_ptr<AeroModel> aeroModel_;
    std::vector<PistonEngine> engines_;
    std::vector<Propeller> props_;
    std::vector<TurbofanEngine> turbofans_;
    FuelSystem fuel_;
    ElectricalSystem electrical_;
    VacuumSystem vacuum_;
    InstrumentPanel instruments_;
    const NavEnvironment* nav_ = nullptr;

    DynamicState s_;
    MassProperties mass_;
    std::vector<MassItem> massItems_;
    std::vector<Real> stationMass_;
    Context ctx_;
    Real time_ = 0;
    Real stepDt_ = 1.0 / 240;  // the current step (the wheels' Coulomb stop looks one step ahead)
    bool faulted_ = false;

    // Controls & switches.
    std::array<Magnetos, kMaxEngines> magnetos_{};
    int flapDetent_ = 0;
    Real flapAngle_ = 0;
    Real trim_ = 0;  // -1..1
    bool parkingBrake_ = false;
    bool flapMoving_ = false;
    std::array<bool, kMaxEngines> turboFuel_{};     // fuel control switches at RUN
    std::array<TurbofanState, kMaxEngines> turboState_{};
    bool turbofanSteady_ = false;
    bool gearDown_ = true;
    Real gearPos_ = 1;
    int speedbrakeDetent_ = 0;
    Real speedbrakePos_ = 0;
    bool groundSpoilers_ = false;
    std::array<Real, kMaxEngines> reverserPos_{};
    std::array<int, kMaxEngines> reverserPhase_{};
    bool reverserDone_ = false;  // used on this ground run: not again until airborne
    Real airborneTime_ = 0;      // s off the ground since the last stop (a landing roll follows)
    // FADEC rating schedule (ThrustMode::Auto).
    ThrustRating activeRating_ = ThrustRating::Takeoff;
    ThrustRating autoRating_ = ThrustRating::Takeoff;
    Real takeoffRatingTimer_ = 0;  // s airborne on the take-off rating since lift-off or a go-around
    Real levelTime_ = 0;           // s without a climb (switches climb -> cruise rating)
    bool leversLow_ = true;        // levers were below the go-around position
    bool machOverWas_ = false;
    std::array<Real, kMaxGear> steerAngle_{};  // steering actuator position (rad)
    std::array<Real, kMaxEngines> lever_{};    // throttle each engine followed this step
    // Hydraulics.
    std::array<Real, 4> hydPressure_{};        // pressure fraction per system
    std::array<bool, 4> hydPumpOn_{};          // electric pump switches
    bool gearEmergency_ = false;               // free-fall release pulled
    int brakeAccumulator_ = -1;                // applications left (-1: not hydraulic)
    bool brakeWasApplied_ = false, brakeHeld_ = false;  // an application backed by the accumulator
    Real hydPumpCurrent_ = 0;                  // A, electric hydraulic pumps running
    Real gearSupply_ = 1, flapSupply_ = 1, speedbrakeSupply_ = 1, steeringSupply_ = 1, brakeSupply_ = 1;
    bool batteryDepletedWas_ = false;
    bool loadShedWas_ = false;
    std::array<bool, kMaxGenerators> genOnlineWas_{};
    std::array<GearRuntime, kMaxGear> gearRt_{};
    std::array<Real, kMaxGear> antiSkid_{};    // anti-skid modulation state per leg (1 = brakes as pedalled)

    // Derived/lagged.
    std::array<Real, 8> downwashLag_{};
    bool quasiSteadyDownwash_ = false;
    // Written by the recorded end-of-step evaluation (derive(..., record=true)).
    mutable std::array<PistonEngineState, kMaxEngines> engineState_{};
    mutable std::array<PropellerForces, kMaxEngines> propState_{};
    mutable std::array<SlipstreamState, kMaxEngines> slip_{};
    mutable AeroOutputs aeroOut_;
    mutable std::array<GearTelemetry, kMaxGear> gearTel_{};
    mutable bool contactTouched_[kMaxContacts] = {};
    bool contactWas_[kMaxContacts] = {};
    Real groundStateTimer_ = 0;  // debounce for touchdown/lift-off events
    bool pusher_ = false, gearOverspeedWas_ = false;
    bool yawDamperOn_ = true;          // the yaw damper switch
    Real ydLowPass_ = 0, ydCommand_ = 0;  // washout filter state, rudder command
    bool wasOnGround_ = false, stallWarningWas_ = false, overspeedWas_ = false, overGWas_ = false;
    bool flapOverspeedWas_ = false;
    std::array<bool, kMaxEngines> runningWas_{};
    bool starvingWas_ = false, stallVane_ = false;
    Real lastSinkRate_ = 0;

    std::vector<CockpitControl> cockpit_;
    std::vector<FlightEvent> events_;
    AircraftTelemetry tel_;
};

}  // namespace avd
