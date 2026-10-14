// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
//
// References: subsonic impact pressure qc = p((1 + 0.2 M^2)^3.5 - 1) and the
// CAS relation (ICAO Doc 7488 / FAA Pilot's Handbook of Aeronautical
// Knowledge ch. 8); altimeter pressure-height relation from the ISA
// troposphere (44330.8 m, exponent 0.190263).
#include "avd/instruments.hpp"

namespace avd {

Real Instruments::impactPressure(Real p, Real mach) {
    // Subsonic isentropic relation. Above Mach 1 a pitot tube sits behind a
    // normal shock (Rayleigh formula); M1 aircraft never get there, but the
    // formula is continued so instruments stay finite (Milestone 7 adds it).
    const Real m = std::max(mach, 0.0);
    if (m <= 1.0) return p * (std::pow(1 + 0.2 * m * m, 3.5) - 1);
    const Real rayleigh = std::pow(1.2 * m * m, 3.5) * std::pow(6.0 / (7 * m * m - 1), 2.5);
    return p * (rayleigh - 1);
}

Real Instruments::casFromImpactPressure(Real qc) {
    constexpr Real a0 = 340.294, p0 = isa::kSeaLevelPressure;
    return a0 * std::sqrt(std::max(0.0, 5 * (std::pow(std::max(qc, 0.0) / p0 + 1, 2.0 / 7.0) - 1)));
}

Real Instruments::altitudeFromPressure(Real p, Real setting) {
    return 44330.77 * (1 - std::pow(std::max(p, 1.0) / setting, 0.190263));
}

void Instruments::reset(const InstrumentInputs& in) {
    initialised_ = false;
    update(0, in);
    r_.attitudeRoll = in.roll;
    r_.attitudePitch = in.pitch;
    r_.headingIndicator = in.heading;
}

void Instruments::update(Real dt, const InstrumentInputs& in) {
    const Real p = in.air.pressure;
    const Real mach = in.trueAirspeed / in.air.speedOfSound;
    const Real q = 0.5 * in.air.density * in.trueAirspeed * in.trueAirspeed;

    // Pitot-static plumbing.
    const Real qcTrue = impactPressure(p, mach);
    Real total = p + qcTrue;
    Real stat = p + (staticError_.empty() ? 0.0 : staticError_(in.alpha) * q);
    if (faults_.staticBlocked) {
        if (trappedStatic_ < 0) trappedStatic_ = stat;
        stat = trappedStatic_;
    } else {
        trappedStatic_ = -1;
    }
    if (faults_.pitotBlocked) {
        // Ram and drain blocked: total pressure trapped (airspeed acts as an altimeter).
        if (trappedTotal_ < 0) trappedTotal_ = total;
        total = trappedTotal_;
    } else {
        trappedTotal_ = -1;
    }
    r_.calibratedAirspeed = casFromImpactPressure(qcTrue);
    r_.indicatedAirspeed = casFromImpactPressure(total - stat);
    const Real alt = altitudeFromPressure(stat, kollsman_);
    if (!initialised_ || dt <= 0) {
        r_.verticalSpeed = 0;
        lastAltitude_ = alt;
    } else {
        const Real rate = (alt - lastAltitude_) / dt;
        r_.verticalSpeed += (rate - r_.verticalSpeed) * std::min(dt / vsiTau_, 1.0);
        lastAltitude_ = alt;
    }
    r_.indicatedAltitude = alt;

    // Vacuum gyros: follow truth when spinning; tumble/wander when slow.
    // (Estimated behaviour; real erection/precession dynamics are planned.)
    const Real follow = smoothstep(0.45, 0.8, in.vacuumGyroSpin);
    if (!initialised_) {
        r_.attitudeRoll = in.roll;
        r_.attitudePitch = in.pitch;
        r_.headingIndicator = in.heading;
    } else if (dt > 0) {
        const Real k = std::min(dt * 8.0 * follow, 1.0);
        r_.attitudeRoll += wrapPi(in.roll - r_.attitudeRoll) * k;
        r_.attitudePitch += (in.pitch - r_.attitudePitch) * k;
        r_.headingIndicator = wrapPi(r_.headingIndicator + wrapPi(in.heading - r_.headingIndicator) * k);
        const Real wander = (1 - follow) * dt;
        r_.attitudeRoll = wrapPi(r_.attitudeRoll + 0.15 * wander);
        r_.attitudePitch = clamp(r_.attitudePitch - 0.05 * wander, -kPi / 2, kPi / 2);
        r_.headingIndicator = wrapPi(r_.headingIndicator + 0.1 * wander);
    }
    r_.magneticHeading = wrapPi(in.heading - in.magneticVariation);

    // Turn coordinator: electric rate gyro canted 30 deg (senses yaw and roll rate).
    r_.turnRate = in.turnCoordinatorPowered
                      ? in.omegaBody.z * std::cos(kPi / 6) + in.omegaBody.x * std::sin(kPi / 6)
                      : 0.0;
    const Real fz = std::abs(in.specificForceBody.z) > 0.1 ? in.specificForceBody.z : -0.1;
    r_.slipBall = in.specificForceBody.y / -fz;
    r_.tachometerRpm = in.engineRpm;  // mechanical cable drive

    const bool powered = in.busVoltage >= 18.0;
    for (int i = 0; i < 4; ++i) {
        const Real target = powered && i < in.tankCount ? in.fuelQuantity[i] : 0.0;
        r_.fuelGauge[i] += (target - r_.fuelGauge[i]) * (initialised_ ? std::min(dt / 2.0, 1.0) : 1.0);
    }
    r_.oilPressure = powered ? in.oilPressure : 0;
    r_.oilTemperature = powered ? in.oilTemperature : 0;
    r_.egt = powered ? in.egt : 0;
    r_.cht = powered ? in.cht : 0;
    r_.fuelFlow = powered ? in.fuelFlow : 0;
    r_.voltmeter = in.busVoltage;
    r_.stallHorn = in.stallVaneTriggered;  // pneumatic reed, no power needed
    initialised_ = true;
}

}  // namespace avd
