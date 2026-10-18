// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
//
// Instrument-panel configuration (the "panel" section of an aircraft
// definition): which instruments are installed, what powers them, their dial
// markings and the parameters of their physical sensor/instrument models.
// Aircraft-specific markings come from the POH; instrument-internal constants
// (gyro erection rates, damping) are typical-instrument estimates — see
// docs/aircraft/tupi-t180-panel.md for each value's provenance.
//
// Units: SI and radians internally (markings in m/s, Pa, K, m^3, A, V);
// conversions to knots/feet/inHg/degF happen only for display.
#pragma once

#include <string>
#include <vector>

#include "avd/math.hpp"
#include "avd/table.hpp"

namespace avd {

struct Range {
    Real low = 0, high = 0;
    bool valid() const { return high > low; }
};

// How a gyro instrument is driven.
enum class GyroPower { Vacuum, Electric };

struct NeedleDynamics {
    Real naturalFrequency = 8.0;  // rad/s
    Real damping = 1.0;           // 1 = critically damped (no overshoot)
};

struct AirspeedIndicatorDef {
    Range whiteArc, greenArc, yellowArc;  // m/s IAS (flap, normal, caution)
    Real redLine = 0;                     // m/s IAS (VNE)
    Real dialMax = 0;                     // m/s, top of the scale (needle pegs above)
    Real dialMin = 0;                     // m/s, lowest graduation (needle rests below)
    NeedleDynamics needle{6.0, 1.0};
    // Pitot tubes recover total pressure up to ~15-20 deg of misalignment.
    Real pitotInsensitiveAngle = 0.30;    // rad
};

struct AltimeterDef {
    Real settingMin = 95159;              // Pa (28.10 inHg)
    Real settingMax = 104978;             // Pa (31.00 inHg)
    Real settingDefault = 101325;         // Pa
    NeedleDynamics needle{10.0, 1.0};
};

struct VerticalSpeedIndicatorDef {
    Real capillaryTimeConstant = 3.0;     // s, case-pressure leak (rate settles in ~3 tau)
    Real dialRange = 10.16;               // m/s (2000 fpm), needle pegs beyond
    NeedleDynamics needle{8.0, 1.0};
};

struct AttitudeIndicatorDef {
    bool installed = true;
    GyroPower power = GyroPower::Vacuum;
    std::string electricalLoad;           // when power = Electric
    Real erectionRate = 0.00175;          // rad/s (~6 deg/min, pendulous-vane erection)
    Real pitchLimit = 1.48;               // rad (85 deg); beyond it the gyro tumbles
    Real rollLimit = 0;                   // rad; 0 = unlimited (360 deg roll, modern instruments)
    Real spinUpTime = 60.0;               // s, rotor time constant
    Real spinDownTime = 200.0;            // s, rotor coast-down time constant
    Real minimumUsefulSpin = 0.7;         // below this the pendulous erection is ineffective
};

struct HeadingIndicatorDef {
    bool installed = true;
    GyroPower power = GyroPower::Vacuum;
    std::string electricalLoad;
    Real mechanicalDrift = 2.9e-5;        // rad/s (6 deg/h, bearing friction/imbalance)
    Real gimbalLimit = 1.40;              // rad (80 deg) bank or pitch: tumbles beyond; 0 = unlimited
    Real slavingRate = 0;                 // rad/s toward magnetic heading (slaved HSI / AHRS); 0 = free gyro
    Real spinUpTime = 60.0;
    Real spinDownTime = 200.0;
};

struct TurnCoordinatorDef {
    bool installed = true;
    std::string electricalLoad = "turn_coordinator";
    Real cant = 0.524;                    // rad (30 deg gyro cant: senses roll rate too)
    Real standardRateDeflection = 0.349;  // rad of miniature-airplane bank at standard rate
    Real calibrationBank = 0.28;          // rad bank of the standard-rate calibration turn
    Real maxDeflection = 0.70;            // rad, mechanical stop
    Real spinUpTime = 20.0;
    Real spinDownTime = 60.0;
    NeedleDynamics gimbal{5.0, 1.0};
};

struct InclinometerDef {
    Real tubeLimit = 0.28;                // rad, ball against the tube end
    NeedleDynamics ball{5.0, 0.8};        // fluid-damped ball
};

struct MagneticCompassDef {
    bool installed = true;
    Real tiltLimit = 0.31;                // rad (18 deg) card tilt before it binds
    Real verticalTimeConstant = 0.5;      // s, pendulous card following apparent vertical
    NeedleDynamics card{2.0, 0.5};        // liquid-damped card (does swing)
    Table1D deviation;                    // deviation (rad) vs compass heading (rad); empty = none
    // The airframe's own residual magnetic field after compensation ("hard
    // iron": steel parts, magnetised fittings), body FRD, as a fraction of the
    // local total field strength. It adds to the earth's field at the card,
    // so deviation emerges and varies with heading (~atan(|h| / cos(dip))).
    Vec3 hardIronBody{0, 0, 0};
};

struct EngineGaugesDef {
    Range tachometerGreen;                // rpm
    Real tachometerRedLine = 0;           // rpm
    Real tachometerMax = 3500;            // rpm (dial)
    Range oilPressureGreen;               // Pa
    Real oilPressureRedLow = 0, oilPressureRedHigh = 0;  // Pa
    Range oilTemperatureGreen;            // K
    Real oilTemperatureRedLine = 0;       // K
    Range suctionGreen;                   // Pa
    bool exhaustGasTemperature = true;    // EGT gauge installed
    bool cylinderHeadTemperature = false; // CHT gauge installed
    bool fuelFlow = true;                 // fuel-flow gauge installed
    bool ammeter = true;                  // battery charge/discharge ammeter
    bool voltmeter = false;
    std::string gaugeLoad = "engine_gauges";  // electrical load powering the electric gauges
    Real fuelSenderTimeConstant = 5.0;    // s (float damping/slosh averaging)
    NeedleDynamics needle{4.0, 1.0};
};

struct AnnunciatorsDef {
    bool installed = true;
    Real oilPressureBelow = 0;            // Pa
    Real vacuumBelow = 0;                 // Pa
    Real voltsBelow = 0;                  // V
    Real lowFuelBelow = 0;                // m^3 usable per tank
};

struct OutsideAirTemperatureDef {
    bool installed = true;
    Real recoveryFactor = 0.8;            // ram-rise recovery of the probe
    Real timeConstant = 10.0;             // s
};

struct AvionicsDef {
    int navComRadios = 1;                 // NAV/COM sets installed
    bool glideslope = true;               // NAV 1 indicator has a glideslope needle
    bool transponder = true;
    bool altitudeEncoder = true;
    bool gps = false, adf = false, dme = false, autopilot = false;  // not modelled yet when true
    std::string avionicsLoad = "avionics";
    Real comSpacingKhz = 25;              // 25 kHz (8.33 optional)
};

// Heated pitot probe: icing from supercooled cloud water, and its heater.
struct PitotProbeDef {
    Real frontalArea = 1.1e-4;        // m^2, tip facing the flow (~12 mm tube)
    Real collectionEfficiency = 0.8;  // droplets caught (small cylinder, 50 m/s)
    Real iceToBlockInlet = 3e-4;      // kg of ice that closes the ram inlet (~3 mm over the tip)
    Real iceToBlockDrain = 6e-4;      // kg that also seals the drain hole
    Real heaterPower = 130;           // W
    Real heaterEfficiency = 0.5;      // fraction of the heat reaching the ice (rest to the airflow)
    Real heaterTimeConstant = 20;     // s, tube warm-up / cool-down
    Real recoveryFactor = 0.85;       // probe surface temperature rise as a fraction of V^2/(2 cp)
};

struct InstrumentPanelDef {
    std::string profile = "analog";       // e.g. "c172s_analog"
    AirspeedIndicatorDef airspeed;
    AltimeterDef altimeter;
    VerticalSpeedIndicatorDef vsi;
    AttitudeIndicatorDef attitude;
    HeadingIndicatorDef heading;
    TurnCoordinatorDef turnCoordinator;
    InclinometerDef inclinometer;
    MagneticCompassDef compass;
    EngineGaugesDef engine;
    AnnunciatorsDef annunciators;
    OutsideAirTemperatureDef oat;
    AvionicsDef avionics;
    PitotProbeDef pitotProbe;
    bool clock = true;
    std::string pitotHeatLoad = "pitot_heat";
    std::string panelLightsLoad = "panel_lights";
};

}  // namespace avd
