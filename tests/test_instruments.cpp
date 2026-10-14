// Aviation Dynamics — instrument validation: every instrument against the
// authoritative simulation state and documented instrument behaviour.
// SPDX-License-Identifier: MIT
//
// Tolerances are physical (instrument lag, position error, gyro erection),
// stated next to each check. References: Cessna 172S POH (markings,
// annunciators), FAA-H-8083-25B ch. 8 and FAA-H-8083-15B ch. 5 & 9.
#include <cstring>

#include "airports.hpp"
#include "avd/procedures.hpp"
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

constexpr Real kDt = kDefaultDt;
constexpr Real kStdRate = kPi / 60;

struct Flight {
    Aircraft ac{ref(), saoPauloWorldFrame()};
    SimpleWeather atm;
    FlatGround ground{0};
    TestPilot pilot;
    TrimResult trim;
    explicit Flight(Real kias = 100, Real alt = 1500, const SimpleWeather::Params* wp = nullptr) {
        if (wp) atm = SimpleWeather(*wp);
        ac.setNavEnvironment(&saoPauloNavEnvironment());
        startTrimmedFlight(ac, {kias, alt, 0}, atm, ground, &pilot, &trim);
        // 15 km north of MASP (not overhead the test VOR's cone of confusion).
        RigidBodyState s = ac.state().body;
        s.position.x += 15000;
        ac.setFlightState(s);
        ac.prepareContext(PilotInput{}, ground, atm);
        ac.resetInstrumentsToTruth();
        ac.instruments().setAltimeterSetting(atm.atmosphere().conditions().seaLevelPressure);
        fly(2);  // let needles settle on the trimmed state
    }
    void fly(Real seconds) {
        const int n = static_cast<int>(std::lround(seconds / kDt));
        for (int i = 0; i < n; ++i) ac.step(kDt, pilot.update(ac, kDt), ground, atm);
    }
    const CockpitState& cs() const { return ac.instruments().state(); }
    const InstrumentTruth& truth() const { return ac.instruments().truth(); }
};

Real stdRateBank(Real tas) { return std::atan(tas * kStdRate / kStandardGravity); }

struct Ground {
    Aircraft ac{ref(), saoPauloWorldFrame()};
    SimpleWeather atm;
    FlatGround g{0};
    Ground() {
        ac.setNavEnvironment(&saoPauloNavEnvironment());
        ac.placeOnGround(0, 0, 0, g, atm);
        ac.setColdAndDark();
    }
    void run(Real seconds, PilotInput in = {}) {
        for (int i = 0; i < static_cast<int>(seconds / kDt); ++i) ac.step(kDt, in, g, atm);
    }
};

}  // namespace

// =============================================================== unit level ===

TEST_CASE("instrument dynamics: critically damped needles do not overshoot") {
    SecondOrder n;
    NeedleDynamics d{8.0, 1.0};
    Real maxX = 0;
    for (int i = 0; i < 2000; ++i) {
        n.update(1.0, kDt, d);
        maxX = std::max(maxX, n.x);
    }
    CHECK(maxX <= 1.0 + 1e-9);
    CHECK_NEAR(n.x, 1.0, 1e-6);
    // Angle variant takes the short way across 359/000.
    SecondOrder a;
    a.reset(degToRad(359));
    a.x = wrapPi(a.x);
    for (int i = 0; i < 2000; ++i) a.updateAngle(degToRad(1), kDt, d);
    CHECK_NEAR(a.x, degToRad(1), 1e-6);
    // Step-size independent (sub-stepped internally).
    SecondOrder b1, b2;
    for (int i = 0; i < 240; ++i) b1.update(1, 1.0 / 240, d);
    for (int i = 0; i < 30; ++i) b2.update(1, 1.0 / 30, d);
    CHECK_NEAR(b1.x, b2.x, 0.01);
}

TEST_CASE("instrument relations: CAS, altimeter law, encoder") {
    CHECK_NEAR(msToKt(InstrumentPanel::casFromImpactPressure(InstrumentPanel::impactPressure(101325, ktToMs(100) / 340.294))), 100, 1e-6);
    CHECK_NEAR(InstrumentPanel::altitudeFromPressure(StandardAtmosphere::isaPressure(3000), 101325), 3000, 2);
    // 1 inHg of Kollsman change ~ 920-960 ft near sea level (standard pressure lapse).
    const Real p = StandardAtmosphere::isaPressure(0);
    const Real d = InstrumentPanel::altitudeFromPressure(p, inHgToPa(30.92)) - InstrumentPanel::altitudeFromPressure(p, inHgToPa(29.92));
    CHECK_RANGE(mToFt(d), 900, 980);
}

// ============================================================ ground/startup ==

TEST_CASE("ground: cold and dark panel reads what an unpowered airplane shows") {
    Ground gr;
    gr.run(1);
    const CockpitState& c = gr.ac.instruments().state();
    CHECK(c.indicatedAirspeed < ktToMs(1));           // calm air, stationary
    CHECK(std::abs(c.verticalSpeed) < fpmToMs(20));
    CHECK_NEAR(c.indicatedAltitude, gr.ac.altitudeMsl(), 3);  // 29.92 setting on an ISA day
    CHECK(c.turnCoordinatorFlag);
    CHECK(gr.ac.instruments().truth().attitudeGyroSpin < 0.01);
    CHECK(std::abs(c.attitudeRoll) > degToRad(5));     // stopped gyro resting against its stops
    CHECK_NEAR(c.fuelGauge[0], 0, 0);                  // electric gauges dead
    CHECK_NEAR(c.oilPressure, 0, 0);
    CHECK(c.annunciators == 0);                        // no power, no lamps
    CHECK_NEAR(c.tachometerRpm, 0, 1e-9);
    CHECK(!c.nav1.valid);
}

TEST_CASE("ground: master on lights the annunciators and powers the gauges") {
    Ground gr;
    gr.ac.setCockpitControl("electrical.battery", 1);
    gr.run(40);  // fuel senders are damped (5 s time constant)
    const CockpitState& c = gr.ac.instruments().state();
    CHECK(c.annunciators & kAnnOilPressure);  // engine stopped: no oil pressure
    CHECK(c.annunciators & kAnnLowVacuum);
    CHECK(!(c.annunciators & kAnnVolts));     // full battery > 24.5 V
    CHECK(!(c.annunciators & (kAnnLowFuelLeft | kAnnLowFuelRight)));
    // Fuel gauges settle on the usable quantity (100.3 L per tank).
    CHECK_NEAR(c.fuelGauge[0] * 1000, (0.106 - 0.00568) * 1000, 1.0);
    CHECK(c.ammeter < -0.5);                  // battery discharging into the loads
    gr.run(60);
    CHECK(!gr.ac.instruments().state().turnCoordinatorFlag);  // electric gyro up to speed
}

TEST_CASE("ground: engine start develops oil pressure, suction and charging; gyros erect") {
    Ground gr;
    ProcedureRunner run;
    run.start(quickStartPistonSingle(gr.ac));
    for (int i = 0; i < 30 * 240 && run.status() == ProcedureRunner::Status::Running; ++i) {
        PilotInput in;
        run.update(gr.ac, in, kDt);
        gr.ac.step(kDt, in, gr.g, gr.atm);
    }
    CHECK(run.status() == ProcedureRunner::Status::Done);
    PilotInput idle;
    idle.throttle = 0.12;
    gr.run(15, idle);
    const CockpitState& c = gr.ac.instruments().state();
    CHECK(c.oilPressure > 20 * 6894.757);     // POH: 20 psi minimum at idle
    CHECK(!(c.annunciators & kAnnOilPressure));
    CHECK(!(c.annunciators & kAnnLowVacuum));
    CHECK(c.ammeter > 0);                     // alternator recharging the battery
    CHECK_NEAR(c.tachometerRpm, gr.ac.telemetry().engines[0].rpm, 30);
    // The attitude gyro erects within a few minutes of taxi-time.
    gr.run(240, idle);
    const InstrumentTruth& t = gr.ac.instruments().truth();
    CHECK_NEAR(gr.ac.instruments().state().attitudeRoll, t.roll, degToRad(2));
    CHECK_NEAR(gr.ac.instruments().state().attitudePitch, t.pitch, degToRad(2));
    // Cold oil reads high pressure, inside the 115 psi red line.
    CHECK(gr.ac.instruments().state().oilPressure < 120 * 6894.757);
    // Power change: tachometer follows.
    PilotInput run2 = idle;
    run2.throttle = 0.5;
    gr.ac.setCockpitControl("brakes.parking", 1);
    gr.run(5, run2);
    CHECK(gr.ac.instruments().state().tachometerRpm > 1600);
}

// ==================================================================== takeoff ==

TEST_CASE("takeoff: airspeed develops smoothly and agrees with CAS; no needle jumps") {
    Ground gr;
    gr.ac.setEngineRunning(true, 1000);
    gr.ac.setCockpitControl("brakes.parking", 0);
    PilotInput in;
    in.throttle = 1;
    Real prev = 0, maxJump = 0;
    bool checked = false;
    for (int i = 0; i < 20 * 240; ++i) {
        gr.ac.step(kDt, in, gr.g, gr.atm);
        const Real ias = gr.ac.instruments().state().indicatedAirspeed;
        maxJump = std::max(maxJump, std::abs(ias - prev));
        prev = ias;
        const Real cas = gr.ac.instruments().truth().calibratedAirspeed;
        if (cas > ktToMs(40) && cas < ktToMs(45) && !checked) {
            // Small alpha on the roll: position error ~0; the needle lags ~0.3 s.
            CHECK_NEAR(ias, cas, ktToMs(2.5));
            checked = true;
        }
    }
    CHECK(checked);
    CHECK(maxJump < ktToMs(0.2));  // per 1/240 s step: no impossible jumps
}

// ===================================================================== flight ==

TEST_CASE("flight: straight and level -- every primary instrument agrees with truth") {
    Flight f(100, 1500);
    f.fly(30);
    const CockpitState& c = f.cs();
    const InstrumentTruth& t = f.truth();
    CHECK(std::abs(c.verticalSpeed) < fpmToMs(60));
    CHECK_NEAR(c.attitudePitch, t.pitch, degToRad(0.5));
    CHECK_NEAR(c.attitudeRoll, t.roll, degToRad(0.5));
    CHECK_NEAR(c.indicatedAltitude, t.pressureAltitude, 5);  // standard day, 29.92
    // IAS vs CAS: static-source position error at cruise alpha is ~1 kt (POH table).
    CHECK_NEAR(c.indicatedAirspeed, t.calibratedAirspeed, ktToMs(2));
    CHECK_NEAR(wrapPi(c.headingIndicator - t.magneticHeading), 0, degToRad(0.5));  // synced at spawn
    CHECK_NEAR(wrapPi(c.magneticHeading - t.magneticHeading), 0, degToRad(1.5));
    CHECK(std::abs(c.slipBall) < degToRad(1.5));
    CHECK(!c.turnCoordinatorFlag);
    CHECK(std::abs(c.turnRate) < degToRad(0.3));
}

TEST_CASE("flight: VSI lags then converges on the true climb rate") {
    Flight f(80, 1000);
    f.pilot.vertical = TestPilot::Vertical::Airspeed;
    f.pilot.targetIas = ktToMs(75);
    f.pilot.controls.throttle = 1.0;
    // Lag: when the true climb rate first passes 300 fpm, the needle trails it.
    Real early = 0, earlyTrue = 0;
    for (int i = 0; i < 15 * 240 && earlyTrue < fpmToMs(300); ++i) {
        f.ac.step(kDt, f.pilot.update(f.ac, kDt), f.ground, f.atm);
        early = f.cs().verticalSpeed;
        earlyTrue = f.truth().verticalSpeed;
    }
    CHECK(early < earlyTrue - fpmToMs(50));
    f.fly(25);
    CHECK(f.truth().verticalSpeed > fpmToMs(400));
    CHECK_NEAR(f.cs().verticalSpeed, f.truth().verticalSpeed, std::abs(f.truth().verticalSpeed) * 0.08 + fpmToMs(30));
    // Descent: idle.
    f.pilot.controls.throttle = 0;
    f.fly(30);
    CHECK(f.cs().verticalSpeed < -fpmToMs(300));
    CHECK_NEAR(f.cs().verticalSpeed, f.truth().verticalSpeed, std::abs(f.truth().verticalSpeed) * 0.1 + fpmToMs(40));
}

TEST_CASE("flight: standard-rate coordinated turn -- turn coordinator on the mark, ball centred") {
    Flight f(95, 1500);
    f.pilot.lateral = TestPilot::Lateral::Bank;
    f.pilot.targetBank = stdRateBank(f.ac.telemetry().trueAirspeed);
    f.pilot.controls.throttle = std::min(1.0, f.trim.throttle + 0.08);
    f.fly(30);
    const CockpitState& c = f.cs();
    // Actual heading rate:
    const Real h0 = f.truth().trueHeading;
    f.fly(10);
    const Real rate = wrapPi(f.truth().trueHeading - h0) / 10;
    CHECK_NEAR(rate, kStdRate, kStdRate * 0.08);
    // The TC miniature airplane sits on the standard-rate mark (calibrated at
    // ~16 deg bank; small difference at other banks is the instrument's own).
    CHECK_NEAR(c.turnRate, kStdRate, kStdRate * 0.15);
    CHECK(std::abs(c.slipBall) < degToRad(1.5));  // coordinated
    // Vacuum AI turn error: in a sustained coordinated turn the pendulous
    // erection works toward the apparent vertical (the cabin floor's normal),
    // so the displayed bank creeps below the true bank by a few degrees.
    CHECK(c.attitudeRoll <= f.truth().roll + degToRad(0.2));
    CHECK_NEAR(c.attitudeRoll, f.truth().roll, degToRad(3));
    // Heading indicator tracks the turn (no compass lag or turning error).
    CHECK_NEAR(wrapPi(c.headingIndicator - f.truth().magneticHeading), 0, degToRad(1.5));
}

TEST_CASE("flight: skid and slip move the ball the right way (ball follows apparent gravity)") {
    Flight f(90, 1500);
    f.pilot.coordinate = false;
    f.pilot.lateral = TestPilot::Lateral::WingsLevel;
    f.pilot.controls.yaw = 0.5;  // right rudder, wings held level: skid
    f.fly(6);
    CHECK(f.cs().slipBall < -degToRad(1.5));  // ball LEFT: "step on the ball" says left rudder
    CHECK_NEAR(f.cs().slipBall, f.truth().slipAngle, degToRad(0.7));
    f.pilot.controls.yaw = -0.5;  // left rudder: ball right
    f.fly(8);
    CHECK(f.cs().slipBall > degToRad(1.5));
}

TEST_CASE("flight: steep 45 deg turn -- attitude indicator follows; small erection error after") {
    Flight f(100, 1500);
    f.pilot.lateral = TestPilot::Lateral::Bank;
    f.pilot.targetBank = degToRad(45);
    f.pilot.controls.throttle = 1.0;
    f.fly(20);
    CHECK_NEAR(f.cs().attitudeRoll, f.truth().roll, degToRad(2));
    CHECK_NEAR(f.ac.telemetry().loadFactor, 1 / std::cos(degToRad(45)), 0.1);
    f.fly(20);
    f.pilot.lateral = TestPilot::Lateral::WingsLevel;
    f.pilot.controls.throttle = f.trim.throttle;
    f.fly(15);
    // Erection toward the apparent vertical during the turn leaves a small,
    // physically real error that then erects out.
    CHECK_NEAR(f.cs().attitudeRoll, f.truth().roll, degToRad(2.5));
    CHECK(!f.cs().attitudeTumbled);
}

TEST_CASE("flight: compass shows turning error in turns but agrees with the DG when straight") {
    Flight f(95, 1500);
    f.pilot.lateral = TestPilot::Lateral::Bank;
    f.pilot.targetBank = stdRateBank(f.ac.telemetry().trueAirspeed);
    f.pilot.controls.throttle = std::min(1.0, f.trim.throttle + 0.08);
    Real maxErr = 0;
    for (int i = 0; i < 120 * 240; ++i) {
        f.ac.step(kDt, f.pilot.update(f.ac, kDt), f.ground, f.atm);
        maxErr = std::max(maxErr, std::abs(wrapPi(f.cs().magneticHeading - f.truth().magneticHeading)));
    }
    CHECK(maxErr > degToRad(5));  // dip makes the card lead/lag in a turn
    f.pilot.lateral = TestPilot::Lateral::WingsLevel;
    f.fly(30);
    CHECK_NEAR(wrapPi(f.cs().magneticHeading - f.truth().magneticHeading), 0, degToRad(2));
    CHECK_NEAR(wrapPi(f.cs().headingIndicator - f.cs().magneticHeading), 0, degToRad(3));
}

TEST_CASE("flight: high altitude -- IAS, CAS, EAS, TAS and ground speed diverge correctly") {
    Flight f(90, ftToM(9000));
    const InstrumentTruth& t = f.truth();
    const Real rho = f.ac.telemetry().air.state.density;
    CHECK(t.trueAirspeed > f.cs().indicatedAirspeed * 1.1);
    CHECK_NEAR(t.trueAirspeed / t.calibratedAirspeed, std::sqrt(isa::kSeaLevelDensity / rho), 0.01);
    CHECK(t.equivalentAirspeed <= t.calibratedAirspeed + 1e-6);  // compressibility: EAS <= CAS
    CHECK_NEAR(t.groundSpeed, t.trueAirspeed, 0.5);              // calm air
}

TEST_CASE("atmosphere: Kollsman setting changes the indication, not the airplane") {
    Flight f(100, 1500);
    const Real alt0 = f.ac.altitudeMsl(), ind0 = f.cs().indicatedAltitude;
    f.ac.instruments().adjustAltimeterSetting(100);  // +1.00 inHg
    f.fly(3);
    CHECK_NEAR(f.ac.altitudeMsl(), alt0, 5);         // the aircraft did not climb
    CHECK_RANGE(mToFt(f.cs().indicatedAltitude - ind0), 850, 1000);
    // Clamped to the instrument's window (28.10-31.00 inHg).
    f.ac.instruments().adjustAltimeterSetting(1000);
    CHECK_NEAR(paToInHg(f.cs().altimeterSetting), 31.00, 1e-6);
    // The transponder encoder ignores the Kollsman setting (always 29.92).
    f.ac.instruments().setTransponderMode(TransponderMode::Altitude);
    f.fly(1);
    CHECK(f.cs().altitudeReported);
    CHECK_NEAR(f.cs().reportedAltitudeFt, mToFt(f.truth().pressureAltitude), 60);
    CHECK(f.cs().reportedAltitudeFt % 100 == 0);
}

TEST_CASE("atmosphere: on a hot day the altimeter reads low against true altitude") {
    SimpleWeather::Params hot;
    hot.conditions.temperatureDeviation = 20;
    Flight f(100, 2500, &hot);
    // Altimeter set to the (standard) sea-level pressure: reads pressure altitude.
    CHECK_NEAR(f.cs().indicatedAltitude, f.truth().pressureAltitude, 8);
    // Warm air column: true altitude above indicated by ~ 4 ft per 1000 ft per degC.
    const Real diff = f.truth().altitudeMsl - f.cs().indicatedAltitude;
    CHECK_RANGE(diff, 100, 220);
}

TEST_CASE("atmosphere: steady wind changes ground speed, not indicated airspeed") {
    SimpleWeather::Params wp;
    wp.windSpeed = 12;
    wp.windFromDirection = 0;
    wp.boundaryLayerExponent = 0;
    Flight calm(100, 1500), windy(100, 1500, &wp);
    windy.fly(20);
    calm.fly(10);
    // IAS reads the air, not the ground: it agrees with CAS in the wind...
    CHECK_NEAR(windy.cs().indicatedAirspeed, windy.truth().calibratedAirspeed, ktToMs(2));
    // ...while the 12 m/s headwind shows only in the ground speed.
    CHECK_NEAR(windy.truth().groundSpeed, windy.truth().trueAirspeed - 12, 1.0);
    // OAT probe shows ram rise of ~1.3 K at 100 kt.
    CHECK_RANGE(calm.cs().outsideAirTemperature - calm.ac.telemetry().air.state.temperature, 0.8, 2.0);
}

// =================================================================== failures ==

TEST_CASE("failure: pitot blocked with drain open -- airspeed falls to zero") {
    Flight f(100, 1500);
    f.ac.instruments().faults().pitotBlocked = true;
    f.fly(60);  // bleeds through the drain hole (8 s time constant); IAS ~ sqrt(qc)
    CHECK(f.cs().indicatedAirspeed < ktToMs(5));
    CHECK(f.truth().trueAirspeed > 40);  // still flying
}

TEST_CASE("failure: pitot and drain blocked -- airspeed behaves like an altimeter") {
    Flight f(90, 1000);
    f.ac.instruments().faults().pitotBlocked = true;
    f.ac.instruments().faults().pitotDrainBlocked = true;
    const Real ias0 = f.cs().indicatedAirspeed;
    f.pilot.vertical = TestPilot::Vertical::Airspeed;
    f.pilot.targetIas = ktToMs(75);  // the pilot is chasing a lying airspeed: use pitch
    f.pilot.vertical = TestPilot::Vertical::Pitch;
    f.pilot.targetPitch = degToRad(8);
    f.pilot.controls.throttle = 1;
    f.fly(30);
    CHECK(f.ac.altitudeMsl() > 1050);
    CHECK(f.cs().indicatedAirspeed > ias0 + ktToMs(3));  // reads HIGH in the climb
}

TEST_CASE("failure: static port blocked -- altimeter frozen, VSI zero, IAS reads low in a climb") {
    Flight f(80, 1000);
    f.ac.instruments().faults().staticBlocked = true;
    const Real alt0 = f.cs().indicatedAltitude;
    f.pilot.vertical = TestPilot::Vertical::Pitch;
    f.pilot.targetPitch = degToRad(7);
    f.pilot.controls.throttle = 1;
    f.fly(30);
    CHECK(f.ac.altitudeMsl() > 1050);
    CHECK_NEAR(f.cs().indicatedAltitude, alt0, 2);
    CHECK(std::abs(f.cs().verticalSpeed) < fpmToMs(30));
    CHECK(f.cs().indicatedAirspeed < f.truth().calibratedAirspeed - ktToMs(2));
}

TEST_CASE("failure: vacuum pump failure -- LOW VAC, vacuum gyros run down, electric TC unaffected") {
    Flight f(100, 1500);
    f.ac.vacuum().setPumpFailed(true);
    f.fly(2);
    CHECK(f.cs().annunciators & kAnnLowVacuum);
    CHECK(f.cs().suction < 1);
    // Usable for a while (rotor coast-down), then degrading.
    CHECK_NEAR(f.cs().attitudeRoll, f.truth().roll, degToRad(1));
    f.pilot.lateral = TestPilot::Lateral::Bank;  // keep flying by truth (the test pilot)
    f.pilot.targetBank = 0;
    for (int i = 0; i < 10; ++i) f.fly(60);
    CHECK(f.truth().attitudeGyroSpin < 0.1);
    CHECK(f.truth().headingGyroSpin < 0.1);
    CHECK(!f.cs().turnCoordinatorFlag);  // electric: still working
    // Fly a turn: the dead attitude indicator no longer shows the bank.
    f.pilot.targetBank = degToRad(25);
    f.fly(20);
    CHECK(std::abs(f.cs().attitudeRoll - f.truth().roll) > degToRad(8));
    CHECK(f.cs().turnRate > degToRad(1));  // the TC shows the turn
}

TEST_CASE("failure: electrical failure -- electric instruments and radios flag, vacuum ones keep working") {
    Flight f(100, 1500);
    f.ac.instruments().setNavStandby(0, 113700);
    f.ac.instruments().swapNav(0);
    f.fly(2);
    CHECK(f.cs().nav1.valid);
    f.ac.setCockpitControl("electrical.alternator", 0);
    f.ac.setCockpitControl("electrical.battery", 0);
    f.fly(30);
    CHECK(f.cs().turnCoordinatorFlag);
    CHECK(!f.cs().nav1.valid);
    CHECK(!f.cs().altitudeReported);
    CHECK_NEAR(f.cs().fuelGauge[0], 0, 0.01);
    CHECK(f.cs().annunciators == 0);  // annunciator panel unpowered
    CHECK_NEAR(f.cs().attitudeRoll, f.truth().roll, degToRad(1));       // vacuum AI fine
    CHECK(f.cs().tachometerRpm > 2000);                                  // mechanical tach
    CHECK(f.cs().indicatedAirspeed > ktToMs(90));                        // pitot-static fine
}

TEST_CASE("failure: alternator failure -- battery discharges, VOLTS annunciator as it runs down") {
    Flight f(100, 1500);
    f.ac.electrical().setAlternatorFailed(true);
    for (const char* l : {"landing_light", "taxi_light", "nav_lights", "strobes", "pitot_heat", "beacon"})
        f.ac.setCockpitControl(std::string("electrical.load.") + l, 1);
    f.fly(5);
    CHECK(f.cs().ammeter < -15);                // everything on the battery
    CHECK(!(f.cs().annunciators & kAnnVolts));  // a fresh battery still holds > 24.5 V
    f.fly(15 * 60);
    CHECK(f.cs().annunciators & kAnnVolts);     // bus below 24.5 V as it discharges
    CHECK(f.ac.electrical().batteryCharge() < 0.6);
}

TEST_CASE("failure: exceeding the attitude gyro's limits tumbles it; it re-erects over minutes") {
    Flight f(110, 2000);
    // A 120 deg bank exceeds the 110 deg roll limit.
    f.pilot.lateral = TestPilot::Lateral::Bank;
    f.pilot.maxBank = 2.5;
    f.pilot.targetBank = degToRad(125);
    bool tumbled = false;
    for (int i = 0; i < 8 * 240 && !tumbled; ++i) {
        f.ac.step(kDt, f.pilot.update(f.ac, kDt), f.ground, f.atm);
        tumbled = f.cs().attitudeTumbled;
    }
    CHECK(tumbled);
    f.pilot.targetBank = 0;
    f.pilot.vertical = TestPilot::Vertical::Altitude;
    f.pilot.targetAltitude = f.ac.altitudeMsl();
    f.pilot.controls.throttle = f.trim.throttle;
    f.fly(30);
    CHECK(std::abs(f.cs().attitudeRoll - f.truth().roll) + std::abs(f.cs().attitudePitch - f.truth().pitch) > degToRad(5));
    for (int i = 0; i < 20; ++i) f.fly(60);
    CHECK_NEAR(f.cs().attitudeRoll, f.truth().roll, degToRad(3));
}

// ================================================================= navigation ==

TEST_CASE("navigation: VOR radial, CDI sense, TO/FROM and flags") {
    const NavEnvironment& env = saoPauloNavEnvironment();
    const LocalFrame f = saoPauloWorldFrame();
    const Real var = env.field.declination;
    auto at = [&](Real radialMagDeg, Real distM, Real h) {
        const Real brg = degToRad(radialMagDeg) + var;
        Geodetic g = f.geodeticFromNed({distM * std::cos(brg), distM * std::sin(brg), 0});
        g.height = h;
        return g;
    };
    const Real obs090 = degToRad(90);
    NavIndication n = evaluateNav(*env.database, 113700, obs090, at(90, 20000, 1500));
    CHECK(n.valid);
    CHECK(n.toFrom == -1);
    CHECK_NEAR(n.cdi, 0, 0.01);
    CHECK_NEAR(n.radialMagnetic, obs090, degToRad(0.1));
    // On the 085 radial, outbound 090: course is to the right (south) -> needle right, half scale.
    n = evaluateNav(*env.database, 113700, obs090, at(85, 20000, 1500));
    CHECK_NEAR(n.cdi, 0.5, 0.02);
    // OBS 270 while on the 090 radial: TO the station.
    n = evaluateNav(*env.database, 113700, degToRad(270), at(90, 20000, 1500));
    CHECK(n.toFrom == 1);
    CHECK_NEAR(n.cdi, 0, 0.01);
    // Beyond line of sight at low altitude: flag. Wrong frequency: flag.
    CHECK(!evaluateNav(*env.database, 113700, obs090, at(90, 150000, 900)).valid);
    CHECK(!evaluateNav(*env.database, 112100, obs090, at(90, 20000, 1500)).valid);
    // Abeam (radial 180 with OBS 090): ambiguous, flag.
    CHECK(!evaluateNav(*env.database, 113700, obs090, at(180, 20000, 1500)).valid);
}

TEST_CASE("navigation: localizer and glideslope geometry on approach") {
    const NavEnvironment& env = saoPauloNavEnvironment();
    const LocalFrame f = saoPauloWorldFrame();
    const Airport a = campoDeMarte(f);
    const RunwayDef& r = a.runways[0];
    auto at = [&](Real along, Real across, Real heightAgl) {
        Geodetic g = f.geodeticFromNed(runwayPoint(r, along, across));
        g.height = a.elevation + 2 + heightAgl;
        return g;
    };
    const Real gsDist = 3300;  // from the GS antenna (300 m in from the threshold)
    const Real onPath = std::tan(degToRad(3)) * gsDist;
    NavIndication n = evaluateNav(*env.database, 110300, 0, at(-3000, 0, onPath));
    CHECK(n.valid && n.localizer);
    CHECK(n.glideslopeValid);
    CHECK_NEAR(n.cdi, 0, 0.05);
    CHECK_NEAR(n.glideslope, 0, 0.1);
    // Right of the centreline -> needle left; above the glidepath -> needle down.
    CHECK(evaluateNav(*env.database, 110300, 0, at(-3000, 150, onPath)).cdi < -0.4);
    CHECK(evaluateNav(*env.database, 110300, 0, at(-3000, 0, onPath + 40)).glideslope < -0.5);
    // Full scale at the threshold is ~700 ft wide (+-107 m).
    CHECK_NEAR(evaluateNav(*env.database, 110300, 0, at(0, 106.7, 15)).cdi, -1.0, 0.03);
}

TEST_CASE("navigation: an aircraft on the extended centreline reads a centred localizer") {
    // Regression: the aircraft once fed the receiver a single-radius latitude/
    // longitude (~0.7% off at 23.5 S), which read 0.12 of full scale (~33 m)
    // off the centreline 5 km out.
    const LocalFrame f = saoPauloWorldFrame();
    const Airport a = campoDeMarte(f);
    const RunwayDef& r = a.runways[0];
    Aircraft ac(ref(), f);
    ac.setNavEnvironment(&saoPauloNavEnvironment());
    ac.setEngineRunning(true, 2000);
    RigidBodyState s;
    s.position = runwayPoint(r, -3000);
    s.position.z = -(a.elevation + 3300 * std::tan(degToRad(3)));
    s.velocity = {ktToMs(65) * std::cos(r.headingTrue), ktToMs(65) * std::sin(r.headingTrue), 0};
    s.attitude = Quat::fromEuler(0, 0, r.headingTrue);
    ac.setFlightState(s);
    FlatGround ground(-a.elevation);
    SimpleWeather atm;
    ac.prepareContext(PilotInput{}, ground, atm);
    ac.resetInstrumentsToTruth();
    ac.instruments().setNavStandby(0, 110300);
    ac.instruments().swapNav(0);
    ac.prepareContext(PilotInput{}, ground, atm);
    ac.resetInstrumentsToTruth();
    const NavIndication& n = ac.instruments().state().nav1;
    CHECK(n.valid && n.localizer);
    CHECK_NEAR(n.cdi, 0, 0.02);
    CHECK_NEAR(n.glideslope, 0, 0.1);
}

TEST_CASE("navigation: navaid JSON loader requires provenance and valid frequencies") {
    NavDatabase db;
    auto e = db.loadJson(R"({"navaids": [{"type": "vor", "ident": "X", "frequency_mhz": 113.1,
        "latitude_deg": -23.5, "longitude_deg": -46.6, "source": "test"}]})");
    CHECK(e.empty());
    CHECK(db.navaids().size() == 1);
    NavDatabase bad;
    e = bad.loadJson(R"({"navaids": [{"type": "vor", "frequency_mhz": 99.9, "latitude_deg": 0, "longitude_deg": 0}]})");
    CHECK(e.size() == 2);  // no source, out-of-band frequency
}

TEST_CASE("radios: tuning steps, band wrap, swap and squawk validation") {
    Flight f(100, 1500);
    InstrumentPanel& p = f.ac.instruments();
    p.setComStandby(0, 118000);
    p.tuneComStandby(0, 1);
    CHECK_NEAR(f.cs().comStandbyKhz[0], 118025, 1e-9);
    p.tuneComStandby(0, -2);
    CHECK_NEAR(f.cs().comStandbyKhz[0], 136975, 1e-9);  // wraps
    p.setNavStandby(0, 113712);
    CHECK_NEAR(f.cs().navStandbyKhz[0], 113700, 1e-9);  // 50 kHz raster
    p.swapNav(0);
    CHECK_NEAR(f.cs().navActiveKhz[0], 113700, 1e-9);
    p.setSquawk(7000);
    CHECK(f.cs().squawk == 7000);
    p.setSquawk(7800);  // 8 is not an octal digit
    CHECK(f.cs().squawk == 7000);
    p.setObs(degToRad(370));
    CHECK_NEAR(f.cs().obs, degToRad(10), 1e-9);
}

TEST_CASE("navigation: flying a VOR radial in the aircraft moves the CDI through centre") {
    Flight f(100, 1500);
    InstrumentPanel& p = f.ac.instruments();
    p.setNavStandby(0, 113700);
    p.swapNav(0);
    // Aircraft is north of MASP heading north: set the OBS to its current radial.
    f.fly(1);
    const Real radial = f.cs().nav1.radialMagnetic;
    p.setObs(radial);
    f.fly(1);
    CHECK(f.cs().nav1.valid);
    CHECK_NEAR(f.cs().nav1.cdi, 0, 0.05);
    p.adjustObs(degToRad(5));
    f.fly(1);
    CHECK(std::abs(f.cs().nav1.cdi) > 0.4);
}

// ================================================================= regression ==

TEST_CASE("regression: heading instruments are continuous across 359/000") {
    Flight f(95, 1500);
    f.pilot.lateral = TestPilot::Lateral::Bank;
    f.pilot.targetBank = stdRateBank(f.ac.telemetry().trueAirspeed);
    f.pilot.controls.throttle = std::min(1.0, f.trim.throttle + 0.08);
    Real prevHi = f.cs().headingIndicator, prevC = f.cs().magneticHeading, maxStepHi = 0, maxStepC = 0;
    int wraps = 0;
    for (int i = 0; i < 130 * 240; ++i) {
        f.ac.step(kDt, f.pilot.update(f.ac, kDt), f.ground, f.atm);
        const Real hi = f.cs().headingIndicator, c = f.cs().magneticHeading;
        maxStepHi = std::max(maxStepHi, std::abs(wrapPi(hi - prevHi)));
        maxStepC = std::max(maxStepC, std::abs(wrapPi(c - prevC)));
        if (std::abs(hi - prevHi) > kPi) ++wraps;
        prevHi = hi;
        prevC = c;
    }
    CHECK(wraps >= 1);                       // went through north (a full 360 turn)
    CHECK(maxStepHi < degToRad(0.1));        // 3 deg/s * 1/240 s = 0.0125 deg
    CHECK(maxStepC < degToRad(0.5));
}

TEST_CASE("regression: no bank/pitch reversal -- right bank and nose up read right and up") {
    Flight f(100, 1500);
    f.pilot.lateral = TestPilot::Lateral::Bank;
    f.pilot.targetBank = degToRad(20);
    f.pilot.vertical = TestPilot::Vertical::Pitch;
    f.pilot.targetPitch = degToRad(8);
    f.pilot.controls.throttle = 1;
    f.fly(8);
    CHECK(f.cs().attitudeRoll > degToRad(15));
    CHECK(f.cs().attitudePitch > degToRad(5));
    CHECK(f.cs().turnRate > 0);  // right turn
}

TEST_CASE("regression: cockpit state does not depend on the host frame rate") {
    // Hosts accumulate wall-clock frames and run fixed steps; the result after
    // the same simulated time is bit-identical for 30 fps and 144 fps hosts.
    auto run = [](Real frame) {
        Flight f(100, 1500);
        f.pilot.lateral = TestPilot::Lateral::Bank;
        f.pilot.targetBank = 0.3;
        Real acc = 0;
        int steps = 0;
        const int total = 10 * 240;
        while (steps < total) {
            acc += frame;
            while (acc >= kDt && steps < total) {
                f.ac.step(kDt, f.pilot.update(f.ac, kDt), f.ground, f.atm);
                acc -= kDt;
                ++steps;
            }
            (void)f.cs();  // a renderer reading the snapshot changes nothing
        }
        return f.cs();
    };
    const CockpitState a = run(1.0 / 30), b = run(1.0 / 144);
    CHECK(a.indicatedAirspeed == b.indicatedAirspeed);
    CHECK(a.attitudeRoll == b.attitudeRoll);
    CHECK(a.headingIndicator == b.headingIndicator);
    CHECK(a.verticalSpeed == b.verticalSpeed);
    CHECK(a.slipBall == b.slipBall);
}

TEST_CASE("regression: no repeated control events for unchanged values") {
    Ground gr;
    gr.ac.drainEvents();
    for (int i = 0; i < 10; ++i) gr.ac.setCockpitControl("electrical.battery", 1);
    int n = 0;
    for (const auto& e : gr.ac.drainEvents()) n += e.type == FlightEventType::CockpitControlChanged;
    CHECK(n == 1);
}

TEST_CASE("panel configuration: markings come from data and invalid panels are rejected") {
    const InstrumentPanelDef& p = ref().panel;
    CHECK_NEAR(msToKt(p.airspeed.greenArc.low), 48, 1e-9);
    CHECK_NEAR(msToKt(p.airspeed.redLine), 163, 1e-9);
    CHECK_NEAR(p.engine.tachometerRedLine, 2700, 1e-9);
    CHECK(!p.avionics.gps && !p.avionics.adf && !p.avionics.autopilot);
    AircraftDefinition d = ref();
    d.panel.vsi.capillaryTimeConstant = 0;
    d.panel.avionics.comSpacingKhz = 12;
    const auto e = d.validate();
    CHECK(e.size() >= 2);
}

// Regression: a spawn in the air from a freshly built aircraft (the browser
// "cruise" reset) once seeded the VSI case and altimeter with the air at the
// previous position; the VSI then sat pegged for ~11 s.
TEST_CASE("spawn: airborne spawn seeds the pitot-static instruments at the new position") {
    Aircraft ac{ref(), saoPauloWorldFrame()};
    SimpleWeather atm;
    FlatGround ground{0};
    ac.setEngineRunning(true, 2300);
    RigidBodyState s;
    s.position = {0, 0, -1600};
    s.velocity = {units::ktToMs(100), 0, 0};
    s.attitude = Quat::fromEuler(0, units::degToRad(2), 0);
    ac.setFlightState(s);
    ac.prepareContext(PilotInput{}, ground, atm);
    ac.settleDownwash();
    ac.instruments().setAltimeterSetting(atm.atmosphere().conditions().seaLevelPressure);
    ac.resetInstrumentsToTruth();
    PilotInput in;
    in.throttle = 0.65;
    in.mixture = 1;
    for (int i = 0; i < static_cast<int>(1.0 / kDt); ++i) ac.step(kDt, in, ground, atm);
    const CockpitState& c = ac.instruments().state();
    const InstrumentTruth& t = ac.instruments().truth();
    CHECK_NEAR(units::mToFt(c.indicatedAltitude), units::mToFt(t.altitudeMsl), 40.0);
    CHECK_NEAR(units::msToFpm(c.verticalSpeed), units::msToFpm(t.verticalSpeed), 400.0);
}
