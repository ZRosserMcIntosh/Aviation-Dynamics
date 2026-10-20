// Aviation Dynamics — ground handling: the combined-slip tyre, wheel spin and
// brakes (locking, sliding friction, anti-skid), the castoring nose wheel,
// parked stability and the crosswind take-off roll.
// SPDX-License-Identifier: MIT
#include <cstdio>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "avd/units.hpp"
#include "avd_test.hpp"
#include "scenarios.hpp"

using namespace avd;
using namespace avd::scenarios;
using namespace avd::units;

namespace {

const AircraftDefinition& ref() {
    static const AircraftDefinition d = loadReference();
    return d;
}
const AircraftDefinition& jet() {
    static const AircraftDefinition d = loadReference("top_g");
    return d;
}
// The Top G stopped by its tyres alone: no reversers or ground spoilers.
AircraftDefinition tyresOnlyJet() {
    AircraftDefinition d = jet();
    for (TurbofanDef& t : d.turbofans) t.reverser.present = false;
    d.controls.speedbrake.groundSpoilers = false;
    return d;
}
bool anyContains(const std::vector<std::string>& v, const char* s) {
    for (const auto& e : v)
        if (e.find(s) != std::string::npos) return true;
    return false;
}

// On the ground, engine running, parking brake off, rolling at `kt` along
// the body x axis plus `lateral` m/s to the right, the wheels turning at
// `spin` times their rolling speed (0: the instant of touchdown).
void setRolling(Aircraft& ac, const GroundQuery& g, const AtmosphereField& atm, Real kt, Real lateral, Real spin) {
    ac.setEngineRunning(true, 800);
    ac.placeOnGround(0, 0, 0, g, atm);
    ac.setCockpitControl("brakes.parking", 0);
    DynamicState s = ac.state();
    s.body.velocity = s.body.attitude.rotate(Vec3{ktToMs(kt), lateral, 0});
    for (std::size_t i = 0; i < ac.definition().gear.size(); ++i)
        s.wheelOmega[i] = spin * ktToMs(kt) / ac.definition().gear[i].wheelRadius;
    ac.setState(s);
}

// Direction of the nose-wheel contact point's velocity in body axes, from
// the body x axis (+ right): what a free castor lines up with.
Real noseVelocityDirection(const Aircraft& ac) {
    const DynamicState& s = ac.state();
    const GearDef& g = ac.definition().gear[0];
    const Vec3 r = g.attach + Vec3{0, 0, g.length} - ac.massProperties().cg;
    const Vec3 v = s.body.attitude.inverseRotate(s.body.velocity) + cross(s.body.omega, r);
    return std::atan2(v.y, std::max(v.x, 0.2));
}

// The reference airplane with brakes able to take its tyres past their peak
// (three times a C172's capacity): the tyre-model tests below want the
// brake-limited and tyre-limited regimes both reachable.
AircraftDefinition strongBrakes() {
    AircraftDefinition d = ref();
    for (GearDef& g : d.gear)
        if (g.brakeGroup >= 0) g.brakeTorque *= 3;
    return d;
}

// Wheel rim speed over the contact point's rolling speed (1 = rolling freely).
Real spinFraction(const GearTelemetry& gt, const GearDef& g) {
    return gt.wheelOmega * g.wheelRadius / std::max(gt.rollingSpeed, 1.0);
}

}  // namespace

TEST_CASE("ground handling: the gear's tyre, brake and castor fields load and validate") {
    const GearDef& n = ref().gear[0];
    const GearDef& m = ref().gear[1];
    CHECK_NEAR(n.castorAngle, degToRad(20), 1e-12);
    CHECK_NEAR(n.castorTrail, 0.08, 1e-12);
    CHECK_NEAR(n.steerLinkageStiffness, 2000, 1e-9);
    CHECK_NEAR(n.steerLinkageBreakout, 100, 1e-9);
    CHECK_NEAR(n.staticLoad, 2870, 1e-9);
    CHECK_NEAR(m.brakeTorque, 350, 1e-9);  // ~0.3 g full-pedal: a light aircraft's disc brakes
    CHECK_NEAR(m.wheelInertia, 0.18, 1e-12);
    CHECK_NEAR(m.staticLoad, 4220, 1e-9);
    CHECK_NEAR(m.slidingFrictionRatio, 0.75, 1e-12);
    CHECK_NEAR(m.corneringLoadSensitivity, 0.25, 1e-12);
    CHECK(m.antiSkidEfficiency == 0);  // a C172 has none
    CHECK(m.castorTrail == 0);
    CHECK_NEAR(jet().gear[1].antiSkidEfficiency, 0.88, 1e-12);
    CHECK_NEAR(jet().gear[1].brakeTorque, 59000, 1e-9);
    CHECK(jet().gear[0].castorTrail == 0);  // hydraulic steering: rigid
    AircraftDefinition d = ref();
    d.gear[1].slidingFrictionRatio = 1.5;
    CHECK(anyContains(d.validate(), "gear[1].slidingFrictionRatio"));
    d = ref();
    d.gear[1].antiSkidEfficiency = 1.2;
    CHECK(anyContains(d.validate(), "gear[1].antiSkidEfficiency"));
    d = ref();
    d.gear[0].castorTrail = -0.1;
    CHECK(anyContains(d.validate(), "gear[0].castorTrail"));
}

TEST_CASE("ground handling: braking and cornering share one friction ellipse") {
    // 15 kt with half a metre per second of sideways drift: the tyres corner
    // it away. With full brakes on top, the resultant tyre force never
    // exceeds mu N and the cornering force gives way to the braking.
    auto run = [](Real brake, Real& meanLateral, Real& worstDemand) {
        Aircraft ac(strongBrakes());
        SimpleWeather atm;
        FlatGround g(0);
        setRolling(ac, g, atm, 15, 0.5, 1.0);
        PilotInput in;
        in.brakeLeft = in.brakeRight = brake;
        meanLateral = worstDemand = 0;
        int n = 0;
        for (int i = 0; i < 240; ++i) {
            ac.step(kDefaultDt, in, g, atm);
            const AircraftTelemetry& t = ac.telemetry();
            for (int k = 0; k < 3; ++k) {
                const GearTelemetry& gt = t.gear[k];
                if (!gt.contact) continue;
                worstDemand = std::max(worstDemand, std::hypot(gt.longitudinalForce, gt.lateralForce) / (gt.peakFriction * gt.normalForce));
            }
            if (i >= 12 && i < 60) {  // 0.05-0.25 s, while the drift lasts
                meanLateral += std::abs(t.gear[1].lateralForce) + std::abs(t.gear[2].lateralForce);
                ++n;
            }
        }
        meanLateral /= n;
    };
    Real latFree, worstFree, latBrake, worstBrake;
    run(0, latFree, worstFree);
    run(1, latBrake, worstBrake);
    CHECK(worstFree <= 1 + 1e-9);
    CHECK(worstBrake <= 1 + 1e-9);
    CHECK(worstBrake > 0.7);           // the brakes take the tyres to their limit (and into the slide)
    CHECK(latFree > 500);              // the mains corner the drift away
    CHECK(latBrake < 0.5 * latFree);   // and lose most of it under the brakes
}

TEST_CASE("ground handling: without anti-skid a hard stop on a wet runway locks the wheels, which slide at mu_slide") {
    // `lateral`: a sideways drift (m/s) at brake application, the small
    // disturbance any real skid starts from (the ground-loop cases below;
    // before the induction model the idling engine's slipstream swirl
    // supplied one by itself).
    auto stop = [](const AircraftDefinition& d, bool pedals, Real lateral, Real& lockedFraction, Real& slidingRatio, Real& headingAtEnd) {
        Aircraft ac(d);
        SimpleWeather atm;
        SurfaceProperties s = surfaces::kAsphalt;
        s.waterDepth = 0.001;
        FlatGround g(0, s);
        setRolling(ac, g, atm, 40, lateral, 1.0);
        PilotInput in;
        in.brakeLeft = in.brakeRight = 1;
        const Vec3 x0 = ac.datumPosition();
        int fast = 0, locked = 0, sliding = 0;
        slidingRatio = 0;
        for (int i = 0; i < 240 * 60; ++i) {
            const AircraftTelemetry& t0 = ac.telemetry();
            in.yaw = pedals ? clamp(-3.0 * t0.heading - 0.8 * t0.omega.z, -1.0, 1.0) : 0.0;
            ac.step(kDefaultDt, in, g, atm);
            const AircraftTelemetry& t = ac.telemetry();
            if (t.groundSpeed < 0.3) break;
            if (t.groundSpeed < ktToMs(10)) continue;
            ++fast;
            const GearTelemetry& m = t.gear[1];
            if (m.held) {
                ++locked;
                slidingRatio += -m.longitudinalForce / (m.peakFriction * m.normalForce);
                ++sliding;
            }
        }
        lockedFraction = fast > 0 ? static_cast<Real>(locked) / fast : 0;
        if (sliding > 0) slidingRatio /= sliding;
        headingAtEnd = wrapPi(ac.telemetry().heading);
        return length(ac.datumPosition() - x0);
    };
    Real lockedFraction, slidingRatio, heading;
    // Brakes strong enough to lock the mains on the wet surface (a C172's own
    // 350 N.m would only just reach the peak); the sliding regime is the point.
    const AircraftDefinition strong = strongBrakes();
    const Real locked = stop(strong, true, 0, lockedFraction, slidingRatio, heading);
    CHECK(lockedFraction > 0.9);  // locked through the fast part of the roll
    CHECK_NEAR(slidingRatio, strong.gear[1].slidingFrictionRatio, 0.03);
    // The same airplane with an anti-skid system never locks and stops shorter.
    AircraftDefinition as = strong;
    for (GearDef& g : as.gear)
        if (g.brakeGroup >= 0) g.antiSkidEfficiency = 0.88;
    Real f2, s2, h2;
    const Real antiSkid = stop(as, true, 0, f2, s2, h2);
    std::printf("    wet stop from 40 kt: locked %.1f m (slide %.2f mu_peak), anti-skid %.1f m (locked fraction %.3f)\n", locked, slidingRatio, antiSkid, f2);
    CHECK(f2 < 0.02);
    CHECK(antiSkid < 0.95 * locked);
    // Locked mains have no cornering left: with the pedals free the skid
    // swings into a ground loop, where the rolling (anti-skid) tyres keep
    // the airplane straight.
    Real f3, s3, h3, f4, s4, h4;
    stop(strong, false, 1.0, f3, s3, h3);  // a 2 kt sideways drift when the brakes go on
    stop(as, false, 1.0, f4, s4, h4);
    CHECK(std::abs(h3) > degToRad(20));
    CHECK(std::abs(h4) < degToRad(6));
}

TEST_CASE("ground handling: the Top G's anti-skid holds the braking slip short of the peak and never locks a wheel") {
    auto stop = [](const AircraftDefinition& d, Real& forceRatio, Real& slip, Real& minSpin, bool& lockedLate) {
        Aircraft ac(d);
        SimpleWeather atm;
        FlatGround g(0);
        setRolling(ac, g, atm, 100, 0, 0);  // the wheels still to spin up: a touchdown
        PilotInput in;
        in.brakeLeft = in.brakeRight = 1;
        const Vec3 x0 = ac.datumPosition();
        forceRatio = slip = 0;
        minSpin = 1;
        lockedLate = false;
        int n = 0;
        for (int i = 0; i < 240 * 60; ++i) {
            ac.step(kDefaultDt, in, g, atm);
            const AircraftTelemetry& t = ac.telemetry();
            if (t.groundSpeed < 0.5) break;
            const Real tt = (i + 1) * kDefaultDt;
            for (int k = 1; k <= 2; ++k) {
                const GearTelemetry& m = t.gear[k];
                if (tt > 0.5 && t.groundSpeed > ktToMs(10)) {
                    minSpin = std::min(minSpin, spinFraction(m, d.gear[k]));
                    lockedLate = lockedLate || m.held;
                }
                if (tt > 1 && tt < 7) {
                    forceRatio += -m.longitudinalForce / (m.peakFriction * m.normalForce);
                    slip += m.slipRatio;
                    ++n;
                }
            }
        }
        forceRatio /= n;
        slip /= n;
        return length(ac.datumPosition() - x0);
    };
    Real ratio, slip, minSpin;
    bool lockedLate;
    const Real dist = stop(tyresOnlyJet(), ratio, slip, minSpin, lockedLate);
    CHECK(!lockedLate);
    CHECK(minSpin > 0.8);
    CHECK_RANGE(ratio, 0.80, 0.95);  // ~88 % of the peak force
    CHECK_RANGE(slip, 0.03, 0.12);   // on the rising side of the slip curve
    CHECK_RANGE(dist, 190, 300);     // 100 kt to a stop at 0.45-0.7 g on dry pavement
    // The same brakes without the anti-skid lock the wheels, which slide at
    // mu_slide: a longer stop.
    AircraftDefinition noAs = tyresOnlyJet();
    for (GearDef& g : noAs.gear) g.antiSkidEfficiency = 0;
    Real r2, s2, m2;
    bool l2;
    const Real distNo = stop(noAs, r2, s2, m2, l2);
    CHECK(l2);
    CHECK(distNo > 1.08 * dist);
}

TEST_CASE("ground handling: touchdown spins the wheels up within 0.3 s with an aft force that pitches the nose down") {
    auto run = [](Real spin, Real& spinUpTime, Real& minPitchRate, Real& minForce) {
        Aircraft ac(ref());
        SimpleWeather atm;
        FlatGround g(0);
        setRolling(ac, g, atm, 50, 0, spin);
        PilotInput in;
        spinUpTime = -1;
        minPitchRate = minForce = 0;
        for (int i = 0; i < 240; ++i) {
            ac.step(kDefaultDt, in, g, atm);
            const AircraftTelemetry& t = ac.telemetry();
            bool all = true;
            for (int k = 0; k < 3; ++k) all = all && spinFraction(t.gear[k], ref().gear[k]) > 0.95;
            if (all && spinUpTime < 0) spinUpTime = (i + 1) * kDefaultDt;
            if (i < 72) {
                minPitchRate = std::min(minPitchRate, t.omega.y);
                minForce = std::min(minForce, t.gear[1].longitudinalForce + t.gear[2].longitudinalForce);
            }
        }
    };
    Real tUp, q, fx, tUp2, q2, fx2;
    run(0, tUp, q, fx);
    run(1, tUp2, q2, fx2);
    CHECK_RANGE(tUp, 0.0, 0.3);
    CHECK(fx < -2000);                  // the aft spin-up force on the mains (sliding until they turn)
    CHECK(q < -0.05);                   // nose-down pitch rate (FRD: q < 0)
    // Wheels already turning at the rolling speed: only rolling resistance, no transient.
    CHECK(fx2 > -800);
    CHECK(q2 > 0.5 * q);
}

TEST_CASE("ground handling: parked on a 2 degree slope with the parking brake set it neither creeps nor jitters") {
    AirfieldGround ground(0);
    RunwayDef rw;
    rw.threshold = {-500, 0, 0};
    rw.headingTrue = 0;
    rw.length = 1000;
    rw.width = 60;
    rw.slope = std::tan(degToRad(2));
    ground.addRunway(rw);
    SimpleWeather atm;
    for (Real heading : {0.0, kPi, 0.5 * kPi}) {  // nose uphill, downhill, across the slope
        Aircraft ac(ref());
        ac.placeOnGround(0, 0, heading, ground, atm);
        CHECK(ac.parkingBrake());
        CHECK(std::abs(std::abs(wrapPi(ac.telemetry().pitch - (heading == 0.0 ? degToRad(2) : heading == kPi ? -degToRad(2) : 0.0)))) < degToRad(2.5));
        const Vec3 p0 = ac.datumPosition();
        PilotInput in;
        Real maxSpeed = 0, maxWheel = 0;
        for (int i = 0; i < 60 * 240; ++i) {
            ac.step(kDefaultDt, in, ground, atm);
            if (i < 10 * 240) continue;  // the tyres take ~1 mm of deflection to carry the slope load
            maxSpeed = std::max(maxSpeed, length(ac.state().body.velocity));
            for (std::size_t k = 0; k < 3; ++k) maxWheel = std::max(maxWheel, std::abs(ac.state().wheelOmega[k]));
        }
        CHECK(!ac.faulted());
        CHECK(length(ac.datumPosition() - p0) < 0.01);
        CHECK(maxSpeed < 1e-4);
        CHECK(maxWheel < 1e-6);
    }
    // Brakes off, engine off: the rolling threshold (0.04 of the weight)
    // holds it on a 1 degree slope; on 4 degrees it rolls away.
    auto rollsOn = [&](Real degrees) {
        AirfieldGround slope(0);
        RunwayDef r = rw;
        r.slope = std::tan(degToRad(degrees));
        slope.addRunway(r);
        Aircraft ac(ref());
        ac.placeOnGround(0, 0, kPi, slope, atm);  // nose downhill
        ac.setCockpitControl("brakes.parking", 0);
        const Vec3 p0 = ac.datumPosition();
        PilotInput in;
        for (int i = 0; i < 20 * 240; ++i) ac.step(kDefaultDt, in, slope, atm);
        return length(ac.datumPosition() - p0);
    };
    CHECK(rollsOn(1.0) < 0.05);
    CHECK(rollsOn(4.0) > 20.0);
}

TEST_CASE("ground handling: a 15 kt crosswind take-off roll tracks the centreline on the pedals and weathervanes without them") {
    auto roll = [](bool pedals, Real& maxLateral, Real& headingAtEnd, Real& speedAtEnd) {
        Aircraft ac(ref());
        SimpleWeather::Params wp;
        wp.windSpeed = ktToMs(15);
        wp.windFromDirection = degToRad(90);  // from the right of a northbound runway
        SimpleWeather atm(wp);
        AirfieldGround ground(0);
        RunwayDef rw;
        rw.threshold = {0, 0, 0};
        rw.headingTrue = 0;
        rw.length = 3000;
        rw.width = 45;
        ground.addRunway(rw);
        ac.setEngineRunning(true, 1000);
        ac.setFlapsImmediately(1);
        ac.placeOnGround(40, 0, 0, ground, atm);
        ac.setCockpitControl("brakes.parking", 0);
        const Real east0 = ac.datumPosition().y;
        PilotInput in;
        in.throttle = 1;
        in.roll = 0.5;  // aileron into the wind
        maxLateral = 0;
        for (int i = 0; i < 40 * 240; ++i) {
            const AircraftTelemetry& t = ac.telemetry();
            const Real y = ac.datumPosition().y - east0;
            in.yaw = pedals ? clamp(-3.0 * wrapPi(t.heading) - 0.8 * t.omega.z - 0.08 * y - 0.15 * t.velocityNed.y, -1.0, 1.0) : 0.0;
            ac.step(kDefaultDt, in, ground, atm);
            maxLateral = std::max(maxLateral, std::abs(ac.datumPosition().y - east0));
            if (ac.instruments().readings().indicatedAirspeed > ktToMs(45) || !ac.telemetry().onGround ||
                std::abs(wrapPi(ac.telemetry().heading)) > degToRad(25))
                break;
        }
        headingAtEnd = wrapPi(ac.telemetry().heading);
        speedAtEnd = ac.telemetry().groundSpeed;
    };
    Real latP, hdgP, vP, latF, hdgF, vF;
    roll(true, latP, hdgP, vP);
    roll(false, latF, hdgF, vF);
    CHECK(vP > ktToMs(35));              // reached the rotation range on the runway
    CHECK(latP < 3.0);                   // within 3 m of the centreline all the way
    CHECK(std::abs(hdgP) < degToRad(5));
    CHECK(hdgF > degToRad(8));           // free pedals: the nose swings into the wind (right)
    CHECK(latF > latP);
}

TEST_CASE("ground handling: the castoring nose wheel turns toward its velocity in a brake turn; a free castor lines up with it") {
    auto turn = [](const AircraftDefinition& d, Real yaw, Real brakeRight, Real throttle, Real& steer, Real& velocityDir, Real& yawRate) {
        Aircraft ac(d);
        SimpleWeather atm;
        FlatGround g(0);
        ac.setEngineRunning(true, 1000);
        ac.placeOnGround(0, 0, 0, g, atm);
        ac.setCockpitControl("brakes.parking", 0);
        PilotInput in;
        in.throttle = 0.25;  // ~1,500 rpm: walking pace in a few seconds (below ~20 % lever the idle thrust is under the rolling resistance)
        for (int i = 0; i < 4 * 240; ++i) ac.step(kDefaultDt, in, g, atm);
        in.yaw = yaw;
        in.brakeRight = brakeRight;
        in.throttle = throttle;
        steer = velocityDir = 0;
        int n = 0;
        for (int i = 0; i < 4 * 240; ++i) {
            ac.step(kDefaultDt, in, g, atm);
            if (i >= 3 * 240) {
                steer += ac.telemetry().gear[0].steerAngle;
                velocityDir += noseVelocityDirection(ac);
                ++n;
            }
        }
        steer /= n;
        velocityDir /= n;
        yawRate = ac.telemetry().omega.z;
    };
    // The C172's bungee linkage, pedals neutral, right brake: the tyre's
    // aligning moment on the trail turns the wheel toward the direction it
    // rolls in, against the linkage spring; a rigidly held wheel stays put.
    // Brake turns at ~1,700 rpm (lever 0.32 on the butterfly throttle schedule).
    Real s1, v1, r1;
    turn(strongBrakes(), 0, 0.6, 0.32, s1, v1, r1);
    CHECK(v1 > degToRad(2));
    CHECK_RANGE(s1, 0.3 * v1, v1);
    AircraftDefinition rigid = strongBrakes();
    rigid.gear[0].castorTrail = 0;
    Real s0, v0, r0;
    turn(rigid, 0, 0.6, 0.32, s0, v0, r0);
    CHECK(std::abs(s0) < 1e-9);
    // Full pedal, full right brake and power: the dragged tyre pushes the
    // bungee back past its breakout, so the wheel sits short of the 10 deg
    // commanded (the linkage yields instead of scrubbing the tyre).
    Real s2, v2, r2;
    turn(ref(), 1, 1, 1.0, s2, v2, r2);
    CHECK_RANGE(s2, degToRad(5), degToRad(9.5));
    // A free castor (no linkage) lines up with its velocity within 2 deg.
    AircraftDefinition freeCastor = ref();
    freeCastor.gear[0].steerLinkageStiffness = 0;
    freeCastor.gear[0].steerLinkageBreakout = 0;
    freeCastor.gear[0].maxSteerAngle = 0;
    freeCastor.gear[0].castorAngle = degToRad(60);
    Real s3, v3, r3;
    turn(freeCastor, 0, 0.6, 0.32, s3, v3, r3);
    CHECK(v3 > degToRad(2));
    CHECK_NEAR(s3, v3, degToRad(2));
}
