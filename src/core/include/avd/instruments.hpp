// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
//
// Analog flight instruments driven by sensor models, not by truth:
//  - airspeed, altimeter and VSI read pitot and static pressures (static-port
//    position error vs angle of attack; blockage freezes the trapped pressure),
//  - attitude and heading indicators are vacuum gyros that wander when the
//    rotor slows (pump failure, engine stopped),
//  - the turn coordinator is an electric gyro; fuel/oil/EGT gauges need bus
//    power; the tachometer is mechanical; the stall horn is pneumatic.
// See docs/architecture/instruments-and-cockpit.md.
#pragma once

#include "avd/atmosphere.hpp"
#include "avd/math.hpp"
#include "avd/table.hpp"

namespace avd {

struct InstrumentInputs {
    AirState air;                 // static air at the aircraft
    Real trueAirspeed = 0;        // m/s
    Real alpha = 0;               // rad (for static-source error)
    Real roll = 0, pitch = 0, heading = 0;  // truth (rad), sensed by the gyros
    Vec3 omegaBody;               // rad/s
    Vec3 specificForceBody;       // m/s^2
    Real engineRpm = 0;
    Real busVoltage = 0;
    Real vacuumGyroSpin = 1;      // 0..1
    bool turnCoordinatorPowered = false;
    Real fuelQuantity[4] = {0, 0, 0, 0};  // m^3 per tank (truth)
    int tankCount = 0;
    Real oilPressure = 0, oilTemperature = 0, egt = 0, cht = 0;
    Real fuelFlow = 0;            // kg/s
    bool stallVaneTriggered = false;
    Real magneticVariation = 0;   // rad, east positive (true = magnetic + var)
};

struct InstrumentReadings {
    Real indicatedAirspeed = 0;   // m/s (IAS)
    Real calibratedAirspeed = 0;  // m/s (CAS, from undistorted pitot/static)
    Real indicatedAltitude = 0;   // m, for the Kollsman setting
    Real verticalSpeed = 0;       // m/s, + up (lagged)
    Real attitudeRoll = 0, attitudePitch = 0;  // rad as displayed
    Real headingIndicator = 0;    // rad (directional gyro)
    Real magneticHeading = 0;     // rad (compass, no turning/accel errors in M1)
    Real turnRate = 0;            // rad/s as displayed (0 when unpowered)
    Real slipBall = 0;            // lateral/normal specific-force ratio
    Real tachometerRpm = 0;
    Real fuelGauge[4] = {0, 0, 0, 0};  // m^3 indicated
    Real oilPressure = 0, oilTemperature = 0, egt = 0, cht = 0;
    Real fuelFlow = 0;
    Real voltmeter = 0;
    bool stallHorn = false;
};

struct PitotStaticFaults {
    bool pitotBlocked = false;    // ram port blocked (drain open): reads like static
    bool staticBlocked = false;   // static port blocked: pressure trapped
};

class Instruments {
public:
    Instruments() = default;
    explicit Instruments(Table1D staticErrorVsAlpha, Real vsiTimeConstant)
        : staticError_(std::move(staticErrorVsAlpha)), vsiTau_(vsiTimeConstant) {}

    void setAltimeterSetting(Real pascals) { kollsman_ = pascals; }
    Real altimeterSetting() const { return kollsman_; }
    PitotStaticFaults& faults() { return faults_; }

    void update(Real dt, const InstrumentInputs& in);
    // Snap lagged/gyro states to truth (spawn with a working panel).
    void reset(const InstrumentInputs& in);
    const InstrumentReadings& readings() const { return r_; }

    // Standard relations (exposed for tests).
    static Real impactPressure(Real staticPressure, Real mach);
    static Real casFromImpactPressure(Real qc);
    static Real altitudeFromPressure(Real p, Real setting);

private:
    Table1D staticError_;
    Real vsiTau_ = 3.0;
    Real kollsman_ = isa::kSeaLevelPressure;
    PitotStaticFaults faults_;
    Real trappedStatic_ = -1, trappedTotal_ = -1;
    Real lastAltitude_ = 0;
    bool initialised_ = false;
    InstrumentReadings r_;
};

}  // namespace avd
