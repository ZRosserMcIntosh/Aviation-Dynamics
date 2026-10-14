// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
//
// References:
//  - FAA Pilot's Handbook of Aeronautical Knowledge (FAA-H-8083-25B) ch. 8 and
//    Instrument Flying Handbook (FAA-H-8083-15B) ch. 5: pitot-static system,
//    VSI calibrated leak and 6-9 s rate lag, vacuum gyros and erection, DG
//    precession and 15 min re-sync practice, turn coordinator (canted gyro),
//    inclinometer, magnetic compass dip, northerly turning and acceleration
//    errors, OAT probe ram rise.
//  - Subsonic impact pressure qc = p((1 + 0.2 M^2)^3.5 - 1); CAS relation and
//    the ISA altimeter law (ICAO Doc 7488).
// Every instrument-internal constant is a parameter in InstrumentPanelDef; values and
// their provenance for the reference aircraft are in docs/aircraft/tupi-t180-panel.md.
#include "avd/instruments.hpp"

#include "avd/units.hpp"

namespace avd {

namespace {

constexpr Real kStandardRate = kPi / 60;  // 3 deg/s
constexpr Real kEarthRate = 7.2921150e-5;  // rad/s

// Rotate unit vector v about unit axis k by angle a (Rodrigues).
Vec3 rotateAbout(const Vec3& v, const Vec3& k, Real a) {
    const Real c = std::cos(a), s = std::sin(a);
    return v * c + cross(k, v) * s + k * (dot(k, v) * (1 - c));
}

Real firstOrder(Real x, Real target, Real dt, Real tau) {
    return tau <= 0 ? target : x + (target - x) * (1 - std::exp(-dt / tau));
}

Real finiteOr(Real v, Real fallback) { return std::isfinite(v) ? v : fallback; }

}  // namespace

// ------------------------------------------------------------- dynamics ----

void SecondOrder::update(Real target, Real dt, const NeedleDynamics& d) {
    if (!std::isfinite(target)) return;
    const Real wn = d.naturalFrequency;
    // Sub-step so wn*h stays small whatever the caller's step.
    const int n = std::max(1, static_cast<int>(std::ceil(dt * wn / 0.2)));
    const Real h = dt / n;
    for (int i = 0; i < n; ++i) {
        const Real a = wn * wn * (target - x) - 2 * d.damping * wn * v;
        v += a * h;
        x += v * h;
    }
}

void SecondOrder::updateAngle(Real target, Real dt, const NeedleDynamics& d) {
    if (!std::isfinite(target)) return;
    // Track in a continuous frame around the current value, then wrap.
    const Real t = x + wrapPi(target - x);
    update(t, dt, d);
    x = wrapPi(x);
}

// ------------------------------------------------------- static relations ----

Real InstrumentPanel::impactPressure(Real p, Real mach) {
    const Real m = std::max(mach, 0.0);
    if (m <= 1.0) return p * (std::pow(1 + 0.2 * m * m, 3.5) - 1);
    // Rayleigh pitot formula (normal shock ahead of the probe).
    const Real r = std::pow(1.2 * m * m, 3.5) * std::pow(6.0 / (7 * m * m - 1), 2.5);
    return p * (r - 1);
}

Real InstrumentPanel::casFromImpactPressure(Real qc) {
    constexpr Real a0 = 340.294, p0 = isa::kSeaLevelPressure;
    return a0 * std::sqrt(std::max(0.0, 5 * (std::pow(std::max(qc, 0.0) / p0 + 1, 2.0 / 7.0) - 1)));
}

Real InstrumentPanel::altitudeFromPressure(Real p, Real setting) {
    return 44330.77 * (1 - std::pow(std::max(p, 1.0) / std::max(setting, 1.0), 0.190263));
}

// ----------------------------------------------------------------- panel ----

InstrumentPanel::InstrumentPanel(const InstrumentPanelDef& def, const Table1D& staticError, Real regulatedSuction)
    : def_(def), staticError_(staticError), regulatedSuction_(regulatedSuction) {
    s_.altimeterSetting = def.altimeter.settingDefault;
    s_.navActiveKhz = {110000, 110000};
    s_.navStandbyKhz = {108000, 108000};
    s_.comActiveKhz = {121500, 121500};
    s_.comStandbyKhz = {118000, 118000};
}

Real InstrumentPanel::spinTarget(GyroPower p, bool electricPowered, Real suction) const {
    if (p == GyroPower::Electric) return electricPowered ? 1.0 : 0.0;
    // Vacuum gyros reach rated speed at ~85% of regulated suction (the green
    // arc's lower end); the air jet torque falls with suction.
    return clamp(suction / (0.85 * regulatedSuction_), 0.0, 1.0);
}

void InstrumentPanel::update(Real dt, const SensorInputs& in) {
    if (!(dt > 0)) dt = 0;
    // ---- Truth (for the debug overlay and telemetry only) ----
    const Quat& q = in.attitude;
    t_.trueAirspeed = in.trueAirspeed;
    t_.equivalentAirspeed = in.trueAirspeed * std::sqrt(in.air.density / isa::kSeaLevelDensity);
    t_.groundSpeed = std::hypot(in.groundVelocityNed.x, in.groundVelocityNed.y);
    t_.altitudeMsl = in.position.height;
    t_.pressureAltitude = in.air.pressureAltitude;
    t_.densityAltitude = in.air.densityAltitude;
    t_.heightAboveGround = in.heightAboveGround;
    t_.verticalSpeed = -in.groundVelocityNed.z;
    t_.pitch = q.pitch();
    t_.roll = q.roll();
    t_.trueHeading = q.yaw();
    const MagneticField field = in.nav ? in.nav->field : MagneticField{};
    t_.magneticHeading = wrapPi(t_.trueHeading - field.declination);
    t_.omegaBody = in.omegaBody;

    updatePitotStatic(dt, in);
    updateAttitude(dt, in);
    updateHeading(dt, in);
    updateTurnAndSlip(dt, in);
    updateCompass(dt, in);
    updateEngineAndSystems(dt, in);
    updateRadios(in);
    initialised_ = true;
}

// ------------------------------------------------------------ pitot-static ----

void InstrumentPanel::updatePitotStatic(Real dt, const SensorInputs& in) {
    const Real p = in.air.pressure;
    const Vec3& v = in.pitotAirVelocityBody;  // probe motion relative to the air
    const Real vp = length(v);
    const Real a = in.air.speedOfSound;
    // Total pressure recovered by the probe: full up to the insensitive angle,
    // falling as cos^2 beyond it; nothing with the flow from behind.
    Real qcProbe = 0;
    if (vp > 1e-6 && v.x > 0) {
        const Real misalign = std::acos(clamp(v.x / vp, -1.0, 1.0));
        const Real excess = std::max(0.0, misalign - def_.airspeed.pitotInsensitiveAngle);
        qcProbe = impactPressure(p, vp / a) * sq(std::cos(std::min(excess, kPi / 2)));
    }
    const Real qInf = 0.5 * in.air.density * in.trueAirspeed * in.trueAirspeed;
    Real stat = p + (staticError_.empty() ? 0.0 : staticError_(in.staticPortAlpha) * qInf);
    Real total = p + qcProbe;
    if (faults_.staticBlocked) {
        if (trappedStatic_ < 0) trappedStatic_ = stat;
        stat = trappedStatic_;
    } else {
        trappedStatic_ = -1;
    }
    if (faults_.pitotBlocked && faults_.pitotDrainBlocked) {
        if (trappedTotal_ < 0) trappedTotal_ = total;
        total = trappedTotal_;
        bledTotal_ = -1;
    } else if (faults_.pitotBlocked) {
        // Ram inlet blocked, drain open: the tube bleeds down to static pressure.
        if (bledTotal_ < 0) bledTotal_ = total;
        bledTotal_ = firstOrder(bledTotal_, stat, dt, 8.0);
        total = bledTotal_;
        trappedTotal_ = -1;
    } else {
        trappedTotal_ = bledTotal_ = -1;
    }
    t_.staticPressureMeasured = stat;
    t_.totalPressureMeasured = total;
    t_.calibratedAirspeed = casFromImpactPressure(impactPressure(p, in.trueAirspeed / a));
    s_.calibratedAirspeed = t_.calibratedAirspeed;

    // Airspeed indicator: capsule deflection follows (total - static).
    const AirspeedIndicatorDef& ad = def_.airspeed;
    const Real iasTarget = casFromImpactPressure(std::max(total - stat, 0.0));
    if (!initialised_) asiNeedle_.reset(iasTarget);
    asiNeedle_.update(iasTarget, dt, ad.needle);
    s_.indicatedAirspeed = clamp(asiNeedle_.x, 0.0, ad.dialMax > 0 ? ad.dialMax : 1e9);

    // Altimeter: aneroid against the Kollsman reference.
    const Real hTarget = altitudeFromPressure(stat, s_.altimeterSetting);
    if (!initialised_) altNeedle_.reset(hTarget);
    altNeedle_.update(hTarget, dt, def_.altimeter.needle);
    s_.indicatedAltitude = altNeedle_.x;

    // VSI: the case pressure follows static through the calibrated leak; the
    // diaphragm reads the difference, scaled by the standard-atmosphere density
    // at that pressure (the instrument's calibration).
    const VerticalSpeedIndicatorDef& vd = def_.vsi;
    const Real hp = altitudeFromPressure(stat, isa::kSeaLevelPressure);
    const Real rhoStd = stat / (isa::kGasConstantDryAir * std::max(288.15 - 0.0065 * hp, 150.0));
    if (!initialised_) {
        // Start in the steady state of the current climb or descent: the case
        // leads static by tau * rho * g * w (a spawn in a steady descent reads
        // the descent at once, as the real instrument would).
        const Real wUp = finiteOr(-in.groundVelocityNed.z, 0.0);
        vsiCase_ = faults_.staticBlocked ? stat : stat + vd.capillaryTimeConstant * rhoStd * kStandardGravity * wUp;
    }
    vsiCase_ = firstOrder(vsiCase_, stat, dt, vd.capillaryTimeConstant);
    const Real rate = (vsiCase_ - stat) / (vd.capillaryTimeConstant * rhoStd * kStandardGravity);
    const Real vsTarget = clamp(finiteOr(rate, 0.0), -1.1 * vd.dialRange, 1.1 * vd.dialRange);
    if (!initialised_) vsiNeedle_.reset(vsTarget);
    vsiNeedle_.update(vsTarget, dt, vd.needle);
    s_.verticalSpeed = vsiNeedle_.x;

    // Transponder altitude encoder: its own aneroid on the same static line,
    // always referenced to 29.92 inHg, 100 ft resolution.
    const Real encoderFt = units::mToFt(altitudeFromPressure(stat, isa::kSeaLevelPressure));
    s_.reportedAltitudeFt = static_cast<int>(std::lround(encoderFt / 100.0)) * 100;
}

// -------------------------------------------------------- attitude gyro ----

void InstrumentPanel::updateAttitude(Real dt, const SensorInputs& in) {
    const AttitudeIndicatorDef& d = def_.attitude;
    if (!d.installed) return;
    const Quat& q = in.attitude;
    const Real target = aiFailed_ ? 0.0 : spinTarget(d.power, in.attitudeElectricPowered, in.suction);
    aiSpin_ = firstOrder(aiSpin_, target, dt, target > aiSpin_ ? d.spinUpTime : d.spinDownTime);
    t_.attitudeGyroSpin = aiSpin_;

    const Vec3 caseDown = q.rotate({0, 0, 1});
    if (aiSpin_ >= d.minimumUsefulSpin) {
        // Rigidity in space; pendulous vanes erect the rotor toward the
        // apparent vertical at a slow, nearly constant rate.
        const Vec3 fNed = q.rotate(in.specificForceBody);
        const Real fm = length(fNed);
        if (fm > 1.0) {
            const Vec3 apparentDown = fNed * (-1.0 / fm);
            const Vec3 e = cross(aiAxis_, apparentDown);
            const Real s = length(e);
            if (s > 1e-9) {
                const Real err = std::asin(clamp(s, 0.0, 1.0));
                const Real step = std::min(d.erectionRate * std::min(1.0, err / 0.05) * dt, err);
                aiAxis_ = normalized(rotateAbout(aiAxis_, e / s, step));
            }
        }
    } else {
        // Rotor too slow to be rigid: bearing friction drags the gimbals with
        // the case, so the display drifts toward the airframe's own "down".
        const Real drag = (1 - aiSpin_ / d.minimumUsefulSpin) * 0.05 * dt;
        const Vec3 e = cross(aiAxis_, caseDown);
        const Real s = length(e);
        if (s > 1e-9) aiAxis_ = normalized(rotateAbout(aiAxis_, e / s, std::min(drag, std::asin(clamp(s, 0.0, 1.0)))));
    }
    // Displayed attitude: the case (airframe) relative to the gyro's vertical.
    const Vec3 db = q.inverseRotate(aiAxis_);
    const Real pitch = -std::asin(clamp(db.x, -1.0, 1.0));
    const Real roll = std::atan2(db.y, db.z);
    // Gimbal stops: beyond them a spinning gyro tumbles.
    const bool beyond = std::abs(pitch) > d.pitchLimit || (d.rollLimit > 0 && std::abs(roll) > d.rollLimit);
    if (beyond && aiSpin_ >= d.minimumUsefulSpin) {
        aiAxis_ = normalized(aiAxis_ + q.rotate({1, 0, 0}) * 0.9);
        s_.attitudeTumbled = true;
    }
    if (s_.attitudeTumbled && dot(aiAxis_, Vec3{0, 0, 1}) > std::cos(0.09)) s_.attitudeTumbled = false;
    const Vec3 db2 = q.inverseRotate(aiAxis_);
    s_.attitudePitch = -std::asin(clamp(db2.x, -1.0, 1.0));
    s_.attitudeRoll = std::atan2(db2.y, db2.z);
}

// ------------------------------------------------------ directional gyro ----

void InstrumentPanel::updateHeading(Real dt, const SensorInputs& in) {
    const HeadingIndicatorDef& d = def_.heading;
    if (!d.installed) return;
    const Quat& q = in.attitude;
    const Real target = hiFailed_ ? 0.0 : spinTarget(d.power, in.headingElectricPowered, in.suction);
    dgSpin_ = firstOrder(dgSpin_, target, dt, target > dgSpin_ ? d.spinUpTime : d.spinDownTime);
    t_.headingGyroSpin = dgSpin_;
    // The spin axis is fixed in space: relative to local north it wanders with
    // the Earth's rotation (+Omega sin(lat)) plus mechanical drift, which grows
    // as the rotor slows; below ~30% the card is no longer held at all.
    Real drift = kEarthRate * std::sin(in.position.latitude) + d.mechanicalDrift / std::max(dgSpin_, 0.2);
    if (dgSpin_ < 0.3) drift += 0.02 * (1 - dgSpin_ / 0.3);
    // Gimbal limits exceeded: the gyro tumbles and the card spins.
    const bool beyond = std::abs(t_.roll) > d.gimbalLimit || std::abs(t_.pitch) > d.gimbalLimit;
    if (beyond && dgSpin_ > 0.3) drift += 1.5;
    dgAxisAzimuth_ = wrapPi(dgAxisAzimuth_ + drift * dt);
    // The card reads the spin axis in the case's own horizontal plane (gimbal
    // error in banked/pitched attitudes emerges from this projection).
    const Vec3 sb = q.inverseRotate({std::cos(dgAxisAzimuth_), std::sin(dgAxisAzimuth_), 0});
    const Real gimbal = std::atan2(sb.y, sb.x);
    s_.headingIndicator = wrapPi(dgCardOffset_ - gimbal);
}

void InstrumentPanel::syncHeadingIndicator(Real heading) {
    // Card offset chosen so the card now reads `heading` (the knob cages and
    // rotates the card, it does not move the aircraft).
    dgCardOffset_ = wrapPi(dgCardOffset_ + wrapPi(heading - s_.headingIndicator));
    s_.headingIndicator = wrapPi(heading);
}

void InstrumentPanel::adjustHeadingIndicator(Real delta) { syncHeadingIndicator(s_.headingIndicator + delta); }

// ------------------------------------------------ turn coordinator & ball ----

void InstrumentPanel::updateTurnAndSlip(Real dt, const SensorInputs& in) {
    const TurnCoordinatorDef& d = def_.turnCoordinator;
    if (d.installed) {
        const Real target = in.turnCoordinatorPowered ? 1.0 : 0.0;
        tcSpin_ = firstOrder(tcSpin_, target, dt, target > tcSpin_ ? d.spinUpTime : d.spinDownTime);
        t_.turnGyroSpin = tcSpin_;
        // Precession torque is proportional to rotor speed times the rate about
        // the canted sensitive axis (yaw and roll).
        const Real sensed = tcSpin_ * (in.omegaBody.z * std::cos(d.cant) + in.omegaBody.x * std::sin(d.cant));
        const Real gain = d.standardRateDeflection / (kStandardRate * std::cos(d.calibrationBank) * std::cos(d.cant));
        const Real deflTarget = clamp(sensed * gain, -d.maxDeflection, d.maxDeflection);
        tcGimbal_.update(deflTarget, dt, d.gimbal);
        tcGimbal_.x = clamp(tcGimbal_.x, -d.maxDeflection, d.maxDeflection);
        s_.turnCoordinatorDeflection = tcGimbal_.x;
        s_.turnRate = tcGimbal_.x / d.standardRateDeflection * kStandardRate;
        s_.turnCoordinatorFlag = !in.turnCoordinatorPowered || tcSpin_ < 0.6;
    }
    // Inclinometer: the ball sits where the apparent gravity (-specific force)
    // points in the lateral plane of the airframe; fluid-damped.
    const InclinometerDef& b = def_.inclinometer;
    const Vec3& f = in.specificForceBody;
    const Real angle = std::atan2(-f.y, -f.z);
    t_.slipAngle = angle;
    const Real target = clamp(angle, -b.tubeLimit, b.tubeLimit);
    if (!initialised_) ball_.reset(target);
    ball_.update(target, dt, b.ball);
    ball_.x = clamp(ball_.x, -b.tubeLimit, b.tubeLimit);
    s_.slipBall = ball_.x;
}

// ------------------------------------------------------- magnetic compass ----

namespace {
// Heading the card shows with its plane normal to `n` (NED, down), including
// the airframe's hard-iron field and any deviation table.
Real compassReading(const MagneticCompassDef& d, const MagneticField& field, const Quat& q, const Vec3& n) {
    const Vec3 b = field.directionNed() + q.rotate(d.hardIronBody);
    const Vec3 bp = b - n * dot(n, b);
    const Vec3 nose = q.rotate({1, 0, 0});
    const Vec3 lp = nose - n * dot(n, nose);
    Real heading = std::atan2(dot(cross(bp, lp), n), dot(bp, lp));
    if (!d.deviation.empty()) heading = wrapPi(heading + d.deviation(heading));
    return heading;
}
}  // namespace

std::array<Real, 12> InstrumentPanel::compassSwing(const MagneticField& field) const {
    std::array<Real, 12> out{};
    for (int k = 0; k < 12; ++k) {
        const Real magnetic = k * kPi / 6;
        const Quat q = Quat::fromEuler(0, 0, magnetic + field.declination);
        out[static_cast<std::size_t>(k)] = wrapPi(compassReading(def_.compass, field, q, {0, 0, 1}));
    }
    return out;
}

void InstrumentPanel::updateCompass(Real dt, const SensorInputs& in) {
    const MagneticCompassDef& d = def_.compass;
    if (!d.installed) return;
    const Quat& q = in.attitude;
    const MagneticField field = in.nav ? in.nav->field : MagneticField{};
    // The card is a pendulum in liquid: its plane follows the apparent vertical
    // with a lag, but cannot tilt beyond its pivot limit relative to the bowl.
    const Vec3 fNed = q.rotate(in.specificForceBody);
    const Vec3 caseDown = q.rotate({0, 0, 1});
    Vec3 target = length(fNed) > 1.0 ? normalized(fNed * -1.0) : caseDown;
    const Real k = 1 - std::exp(-dt / std::max(d.verticalTimeConstant, 1e-3));
    compassVertical_ = normalized(compassVertical_ + (target - compassVertical_) * k);
    const Real tilt = std::acos(clamp(dot(compassVertical_, caseDown), -1.0, 1.0));
    if (tilt > d.tiltLimit) {
        const Vec3 axis = cross(compassVertical_, caseDown);
        if (length(axis) > 1e-9) compassVertical_ = normalized(rotateAbout(compassVertical_, normalized(axis), tilt - d.tiltLimit));
    }
    const Real heading = compassReading(d, field, q, compassVertical_);
    if (!initialised_) compassCard_.reset(heading);
    compassCard_.updateAngle(heading, dt, d.card);
    s_.magneticHeading = compassCard_.x;
}

// --------------------------------------------------- engine & systems ----

void InstrumentPanel::updateEngineAndSystems(Real dt, const SensorInputs& in) {
    const EngineGaugesDef& e = def_.engine;
    // Mechanical (cable-driven) tachometer.
    if (!initialised_) tachNeedle_.reset(in.engineRpm);
    tachNeedle_.update(in.engineRpm, dt, e.needle);
    s_.tachometerRpm = std::max(tachNeedle_.x, 0.0);
    // Electric gauges (senders + indicators on the gauge circuit).
    const bool p = in.gaugesPowered;
    const Real tau = 1.0;
    if (!initialised_) {
        oilPressNeedle_.reset(p ? in.oilPressure : 0.0);
        s_.oilTemperature = p ? in.oilTemperature : 0.0;
        s_.egt = p && e.exhaustGasTemperature ? in.egt : 0.0;
        s_.cht = p && e.cylinderHeadTemperature ? in.cht : 0.0;
        s_.fuelFlow = p && e.fuelFlow ? in.fuelFlow : 0.0;
    }
    oilPressNeedle_.update(p ? in.oilPressure : 0.0, dt, e.needle);
    s_.oilPressure = std::max(oilPressNeedle_.x, 0.0);
    s_.oilTemperature = firstOrder(s_.oilTemperature, p ? in.oilTemperature : 0.0, dt, tau);
    s_.egt = firstOrder(s_.egt, p && e.exhaustGasTemperature ? in.egt : 0.0, dt, 3.0);
    s_.cht = firstOrder(s_.cht, p && e.cylinderHeadTemperature ? in.cht : 0.0, dt, 3.0);
    s_.fuelFlow = firstOrder(s_.fuelFlow, p && e.fuelFlow ? in.fuelFlow : 0.0, dt, 1.0);
    for (int i = 0; i < 4; ++i) {
        const Real usable = i < in.tankCount ? std::max(in.fuelQuantity[i] - in.fuelUnusable[i], 0.0) : 0.0;
        s_.fuelGauge[i] = initialised_ ? firstOrder(s_.fuelGauge[i], p ? usable : 0.0, dt, e.fuelSenderTimeConstant)
                                       : (p ? usable : 0.0);
    }
    // Self-powered: ammeter shunt, suction gauge, voltmeter (shows the bus).
    s_.ammeter = -in.batteryCurrent;
    s_.voltmeter = in.busVoltage;
    s_.suction = in.suction;
    // Annunciator lamps, driven by their own switches.
    unsigned ann = 0;
    const AnnunciatorsDef& a = def_.annunciators;
    if (a.installed && in.annunciatorsPowered) {
        if (in.oilPressure < a.oilPressureBelow) ann |= kAnnOilPressure;
        if (in.suction < a.vacuumBelow) ann |= kAnnLowVacuum;
        if (in.busVoltage < a.voltsBelow) ann |= kAnnVolts;
        for (int i = 0; i < std::min(in.tankCount, 2); ++i)
            if (in.fuelQuantity[i] - in.fuelUnusable[i] < a.lowFuelBelow) ann |= i == 0 ? kAnnLowFuelLeft : kAnnLowFuelRight;
    }
    s_.annunciators = ann;
    s_.stallHorn = in.stallVaneTriggered;  // pneumatic reed: no power needed
    // OAT probe: static temperature plus partial ram rise, thermal lag.
    const Real mach = in.trueAirspeed / std::max(in.air.speedOfSound, 1.0);
    const Real oatTarget = in.air.temperature * (1 + def_.oat.recoveryFactor * 0.2 * mach * mach);
    oatProbe_ = initialised_ ? firstOrder(oatProbe_, oatTarget, dt, def_.oat.timeConstant) : oatTarget;
    s_.outsideAirTemperature = def_.oat.installed ? oatProbe_ : 0.0;
    s_.clock = def_.clock ? in.time : 0.0;
    s_.flapIndicator = in.flapAngle;
    s_.trimIndicator = in.elevatorTrim;
    s_.panelLights = in.panelLightsOn;
}

// ----------------------------------------------------------------- radios ----

void InstrumentPanel::updateRadios(const SensorInputs& in) {
    s_.avionicsOn = in.avionicsPowered;
    s_.nav1 = NavIndication{};
    if (in.avionicsPowered && def_.avionics.navComRadios > 0 && in.nav && in.nav->database) {
        s_.nav1 = evaluateNav(*in.nav->database, s_.navActiveKhz[0], s_.obs, in.position);
        if (!def_.avionics.glideslope) s_.nav1.glideslopeValid = false;
    }
    s_.altitudeReported = def_.avionics.transponder && def_.avionics.altitudeEncoder && in.avionicsPowered &&
                          s_.transponderMode == TransponderMode::Altitude;
}

// ------------------------------------------------------- pilot controls ----

void InstrumentPanel::setAltimeterSetting(Real pa) {
    s_.altimeterSetting = clamp(pa, def_.altimeter.settingMin, def_.altimeter.settingMax);
}

void InstrumentPanel::adjustAltimeterSetting(int hundredths) {
    const Real inHg = std::round(units::paToInHg(s_.altimeterSetting) * 100 + hundredths) / 100;
    setAltimeterSetting(units::inHgToPa(inHg));
}

void InstrumentPanel::setNavStandby(int r, Real khz) {
    if (r < 0 || r > 1) return;
    // NAV band 108.00-117.95 MHz in 50 kHz steps.
    s_.navStandbyKhz[r] = clamp(std::round(khz / 50) * 50, 108000.0, 117950.0);
}
void InstrumentPanel::swapNav(int r) {
    if (r >= 0 && r <= 1) std::swap(s_.navActiveKhz[r], s_.navStandbyKhz[r]);
}
void InstrumentPanel::tuneNavStandby(int r, int steps) {
    if (r < 0 || r > 1) return;
    Real f = s_.navStandbyKhz[r] + steps * 50.0;
    if (f > 117950) f = 108000 + (f - 117950 - 50);
    if (f < 108000) f = 117950 - (108000 - f - 50);
    setNavStandby(r, f);
}
void InstrumentPanel::setComStandby(int r, Real khz) {
    if (r < 0 || r > 1) return;
    const Real sp = def_.avionics.comSpacingKhz;
    s_.comStandbyKhz[r] = clamp(118000 + std::round((khz - 118000) / sp) * sp, 118000.0, 136975.0);
}
void InstrumentPanel::swapCom(int r) {
    if (r >= 0 && r <= 1) std::swap(s_.comActiveKhz[r], s_.comStandbyKhz[r]);
}
void InstrumentPanel::tuneComStandby(int r, int steps) {
    if (r < 0 || r > 1) return;
    Real f = s_.comStandbyKhz[r] + steps * def_.avionics.comSpacingKhz;
    if (f > 136975) f = 118000;
    if (f < 118000) f = 136975;
    setComStandby(r, f);
}
void InstrumentPanel::setSquawk(int code) {
    // Each digit 0-7 (octal); invalid codes are ignored.
    if (code < 0 || code > 7777) return;
    for (int c = code; c > 0; c /= 10)
        if (c % 10 > 7) return;
    s_.squawk = code;
}

// ------------------------------------------------------------- resets ----

void InstrumentPanel::resetToTruth(const SensorInputs& in) {
    initialised_ = false;
    aiSpin_ = dgSpin_ = tcSpin_ = 1;
    aiAxis_ = {0, 0, 1};
    s_.attitudeTumbled = false;
    dgAxisAzimuth_ = 0;
    dgCardOffset_ = 0;
    compassVertical_ = {0, 0, 1};
    update(0, in);
    syncHeadingIndicator(t_.magneticHeading);
    s_.turnCoordinatorDeflection = 0;
    tcGimbal_.reset(0);
}

void InstrumentPanel::resetCold(const SensorInputs& in) {
    initialised_ = false;
    aiSpin_ = dgSpin_ = tcSpin_ = 0;
    // A stopped attitude gyro rests against its stops: a tilted display.
    aiAxis_ = normalized(in.attitude.rotate(normalized(Vec3{-0.09, 0.26, 1})));
    dgAxisAzimuth_ = 0;
    dgCardOffset_ = 0.7;  // card wherever it was left
    compassVertical_ = in.attitude.rotate({0, 0, 1});
    tcGimbal_.reset(0);
    update(0, in);
}

}  // namespace avd
