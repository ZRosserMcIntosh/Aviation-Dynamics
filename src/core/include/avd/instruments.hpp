// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
//
// Cockpit instrument system:
//
//   flight dynamics / engine / systems / environment  (authoritative truth)
//        -> SensorInputs        what physical sensors are exposed to
//        -> InstrumentPanel     sensor + instrument dynamics (fixed step)
//        -> CockpitState        renderer-ready snapshot (plain data)
//
// Instruments never compute aircraft state themselves: they measure pressures,
// carry their own gyro rotors, feel specific force, read senders and bus
// voltages, and add only physically justified instrument behaviour (lag,
// damping, calibration error, precession, mechanical limits, power loss).
// See docs/architecture/instruments-and-cockpit.md.
#pragma once

#include <array>
#include <string>

#include "avd/atmosphere.hpp"
#include "avd/math.hpp"
#include "avd/navigation.hpp"
#include "avd/panel_def.hpp"
#include "avd/table.hpp"

namespace avd {

// Critically damped (by default) second-order follower used for needles,
// gimbals and the inclinometer ball. Semi-implicit; stable for wn*dt < 1.
struct SecondOrder {
    Real x = 0, v = 0;
    void reset(Real value) { x = value; v = 0; }
    void update(Real target, Real dt, const NeedleDynamics& d);
    // Angle variant: tracks the shortest way round, output wrapped to (-pi, pi].
    void updateAngle(Real target, Real dt, const NeedleDynamics& d);
};

// Everything the instruments' sensors are physically exposed to, assembled by
// the Aircraft from its authoritative state each step.
struct SensorInputs {
    Real time = 0;
    AirState air;                  // static air at the aircraft (truth)
    Vec3 pitotAirVelocityBody;     // probe velocity relative to the local air (body)
    Real staticPortAlpha = 0;      // local angle of attack for the static-source error
    Real trueAirspeed = 0;         // CG, for truth/debug only
    Vec3 groundVelocityNed;        // for truth/debug only
    Quat attitude;                 // body FRD -> local NED (truth)
    Vec3 omegaBody;                // rad/s (truth, inertial)
    Vec3 specificForceBody;        // m/s^2 at the instrument panel (what pendulums feel)
    Geodetic position;             // aircraft, height MSL
    Real heightAboveGround = 0;    // m, truth/debug
    // Power and vacuum.
    Real suction = 0;              // Pa, vacuum manifold
    Real busVoltage = 0;           // V, main bus
    Real batteryCurrent = 0;       // A, + discharge
    bool alternatorOnline = false;
    bool turnCoordinatorPowered = false;
    bool attitudeElectricPowered = false;
    bool headingElectricPowered = false;
    bool gaugesPowered = false;    // engine/fuel gauges load
    bool avionicsPowered = false;  // NAV/COM, transponder
    bool annunciatorsPowered = false;
    bool pitotHeatOn = false;
    bool panelLightsOn = false;
    Real liquidWaterContent = 0;   // kg/m^3 cloud liquid water at the aircraft (icing)
    // Engine and fuel.
    Real engineRpm = 0;
    Real oilPressure = 0;          // Pa (engine model)
    Real oilTemperature = 0;       // K
    Real egt = 0, cht = 0;         // K
    Real fuelFlow = 0;             // kg/s
    Real fuelDensity = 720;
    int tankCount = 0;
    std::array<Real, 4> fuelQuantity{};  // m^3 (truth)
    std::array<Real, 4> fuelUnusable{};  // m^3
    // Configuration.
    Real flapAngle = 0, elevatorTrim = 0;
    bool stallVaneTriggered = false;
    const NavEnvironment* nav = nullptr;
};

enum Annunciator : unsigned {
    kAnnOilPressure = 1u << 0,
    kAnnLowVacuum = 1u << 1,
    kAnnVolts = 1u << 2,
    kAnnLowFuelLeft = 1u << 3,
    kAnnLowFuelRight = 1u << 4,
};

enum class TransponderMode { Off, Standby, On, Altitude };

// What the pilot sees (and what a renderer draws). SI units.
struct CockpitState {
    // Airspeed indicator.
    Real indicatedAirspeed = 0;    // m/s (needle)
    // Altimeter.
    Real indicatedAltitude = 0;    // m (needles), for the Kollsman setting
    Real altimeterSetting = 101325;  // Pa (Kollsman window)
    // Vertical speed indicator.
    Real verticalSpeed = 0;        // m/s (needle, limited by the dial)
    // Attitude indicator (as displayed by the gyro).
    Real attitudePitch = 0, attitudeRoll = 0;
    bool attitudeTumbled = false;
    // Heading indicator.
    Real headingIndicator = 0;     // rad (card)
    Real headingBug = 0;           // rad
    // Turn coordinator and inclinometer.
    Real turnRate = 0;             // rad/s equivalent shown by the miniature airplane
    Real turnCoordinatorDeflection = 0;  // rad (miniature airplane bank)
    bool turnCoordinatorFlag = true;     // power flag visible
    Real slipBall = 0;             // rad of ball travel (+ right)
    // Magnetic compass.
    Real magneticHeading = 0;      // rad (card reading)
    // Engine and systems gauges.
    Real tachometerRpm = 0;
    Real oilPressure = 0, oilTemperature = 0;  // Pa, K
    Real egt = 0, cht = 0;         // K
    Real fuelFlow = 0;             // kg/s
    std::array<Real, 4> fuelGauge{};  // m^3 usable indicated
    Real ammeter = 0;              // A (+ charging)
    Real voltmeter = 0;            // V
    Real suction = 0;              // Pa
    unsigned annunciators = 0;     // Annunciator bits (lamps lit)
    bool stallHorn = false;
    Real outsideAirTemperature = 0;  // K (probe)
    Real clock = 0;                // s since power-up of the clock
    Real flapIndicator = 0;        // rad
    Real trimIndicator = 0;        // -1..1
    bool panelLights = false;
    Real panelLightDimmer = 1;
    // Radios.
    std::array<Real, 2> navActiveKhz{}, navStandbyKhz{}, comActiveKhz{}, comStandbyKhz{};
    Real obs = 0;                  // rad, NAV 1 OBS
    NavIndication nav1;
    bool avionicsOn = false;
    int squawk = 1200;             // octal digits as a decimal number (e.g. 7000)
    TransponderMode transponderMode = TransponderMode::Standby;
    int reportedAltitudeFt = 0;    // encoder output (100 ft steps), valid in ALT
    bool altitudeReported = false;

    // ---- Kept for convenience/compatibility (same values as above) ----
    Real calibratedAirspeed = 0;   // m/s, from the undistorted pitot-static (truth)
};

// Truth alongside the displays, for the debug overlay and telemetry.
struct InstrumentTruth {
    Real trueAirspeed = 0, calibratedAirspeed = 0, equivalentAirspeed = 0, groundSpeed = 0;  // m/s
    Real altitudeMsl = 0, pressureAltitude = 0, densityAltitude = 0, heightAboveGround = 0;  // m
    Real verticalSpeed = 0;        // m/s
    Real pitch = 0, roll = 0, trueHeading = 0, magneticHeading = 0;  // rad
    Vec3 omegaBody;
    Real slipAngle = 0;            // rad, apparent-vertical lateral angle (ball equilibrium)
    Real staticPressureMeasured = 0, totalPressureMeasured = 0;
    Real attitudeGyroSpin = 0, headingGyroSpin = 0, turnGyroSpin = 0;
    Real pitotIce = 0;             // kg of ice on the pitot probe
    Real pitotHeat = 0;            // 0..1 heater effectiveness (warm-up)
    bool pitotIced = false;        // ice has closed the ram inlet
};

struct PitotStaticFaults {
    bool pitotBlocked = false;     // ram port iced/blocked, drain open: pressure bleeds to static
    bool pitotDrainBlocked = false;  // with pitotBlocked: total pressure trapped
    bool staticBlocked = false;    // static port blocked: static pressure trapped
};

class InstrumentPanel {
public:
    InstrumentPanel() = default;
    InstrumentPanel(const InstrumentPanelDef& def, const Table1D& staticErrorVsAlpha, Real regulatedSuction);

    const InstrumentPanelDef& def() const { return def_; }

    // ---- Fixed-step update ----
    void update(Real dt, const SensorInputs& in);
    // Spawn helpers: gyros spun up and erected, needles at the true values.
    void resetToTruth(const SensorInputs& in);
    // Cold airframe: gyros stopped and resting at their gimbal stops.
    void resetCold(const SensorInputs& in);

    const CockpitState& state() const { return s_; }
    const CockpitState& readings() const { return s_; }  // compatibility alias
    const InstrumentTruth& truth() const { return t_; }

    // ---- Pilot controls on the instruments ----
    void setAltimeterSetting(Real pascals);
    void adjustAltimeterSetting(int hundredthsInHg);
    Real altimeterSetting() const { return s_.altimeterSetting; }
    void syncHeadingIndicator(Real headingRad);   // the DG set knob
    void syncHeadingIndicatorToCompass() { syncHeadingIndicator(s_.magneticHeading); }
    // Compass swing: what the compass reads (rad) when the airplane is level
    // and unaccelerated on magnetic headings 0, 30, ... 330 deg. The panel's
    // correction card ("FOR ... STEER ...") is printed from this.
    std::array<Real, 12> compassSwing(const MagneticField& field) const;
    void adjustHeadingIndicator(Real deltaRad);
    void setHeadingBug(Real rad) { s_.headingBug = wrapPi(rad); }
    void setObs(Real rad) { s_.obs = wrapPi(rad); }
    void adjustObs(Real deltaRad) { setObs(s_.obs + deltaRad); }
    void setNavStandby(int radio, Real khz);
    void swapNav(int radio);
    void setComStandby(int radio, Real khz);
    void swapCom(int radio);
    void tuneComStandby(int radio, int channelSteps);  // +-1 = one channel spacing
    void tuneNavStandby(int radio, int steps50khz);
    void setSquawk(int code);                          // validated octal 0000-7777
    void setTransponderMode(TransponderMode m) { s_.transponderMode = m; }
    void setPanelLightDimmer(Real v) { s_.panelLightDimmer = clamp(v, 0.0, 1.0); }
    PitotStaticFaults& faults() { return faults_; }
    const PitotStaticFaults& faults() const { return faults_; }
    void setAttitudeGyroFailed(bool f) { aiFailed_ = f; }
    void setHeadingGyroFailed(bool f) { hiFailed_ = f; }

    // Standard relations (exposed for tests).
    static Real impactPressure(Real staticPressure, Real mach);
    static Real casFromImpactPressure(Real qc);
    static Real altitudeFromPressure(Real p, Real setting);

private:
    void updatePitotStatic(Real dt, const SensorInputs& in);
    void updateAttitude(Real dt, const SensorInputs& in);
    void updateHeading(Real dt, const SensorInputs& in);
    void updateTurnAndSlip(Real dt, const SensorInputs& in);
    void updateCompass(Real dt, const SensorInputs& in);
    void updateEngineAndSystems(Real dt, const SensorInputs& in);
    void updateRadios(const SensorInputs& in);
    Real spinTarget(GyroPower p, bool electricPowered, Real suction) const;

    InstrumentPanelDef def_;
    Table1D staticError_;
    Real regulatedSuction_ = 16932;
    PitotStaticFaults faults_;
    CockpitState s_;
    InstrumentTruth t_;
    bool initialised_ = false;

    // Pneumatics.
    Real trappedStatic_ = -1, trappedTotal_ = -1, bledTotal_ = -1;
    Real pitotIce_ = 0, pitotHeat_ = 0;
    Real vsiCase_ = 0;
    SecondOrder asiNeedle_, altNeedle_, vsiNeedle_;
    // Attitude gyro: spin-axis direction in local NED (ideally straight down).
    Vec3 aiAxis_{0, 0, 1};
    Real aiSpin_ = 0;
    bool aiFailed_ = false;
    // Directional gyro: horizontal spin-axis azimuth (true, rad) and card offset.
    Real dgAxisAzimuth_ = 0, dgCardOffset_ = 0, dgSpin_ = 0;
    bool hiFailed_ = false;
    // Turn coordinator.
    Real tcSpin_ = 0;
    SecondOrder tcGimbal_, ball_;
    // Compass: lagged apparent vertical (NED) and card.
    Vec3 compassVertical_{0, 0, 1};
    SecondOrder compassCard_;
    // Engine gauges.
    SecondOrder tachNeedle_, oilPressNeedle_;
    Real oatProbe_ = 288.15;
};

}  // namespace avd
