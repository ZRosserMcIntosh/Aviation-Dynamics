// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
#include "avd/telemetry.hpp"

#include "avd/units.hpp"

namespace avd {

void TelemetryRecorder::writeHeader() {
    out_ << "time_s,north_m,east_m,alt_msl_ft,agl_m,roll_deg,pitch_deg,heading_deg,"
            "tas_kt,ias_kt,gs_kt,vs_fpm,alpha_deg,beta_deg,mach,nz_g,mass_kg,cg_x_m,fuel_kg,"
            "rpm,map_inhg,fuel_flow_gph,power_hp,thrust_n,prop_eff,flap_deg,trim,elevator,aileron,rudder,"
            "on_ground,stall_warning,stalled_strips,bus_v,battery_soc\n";
}

void TelemetryRecorder::record(const Aircraft& ac) {
    const AircraftTelemetry& t = ac.telemetry();
    if (t.time - last_ < interval_ - 1e-9) return;
    last_ = t.time;
    using namespace units;
    const auto& e = t.engines[0];
    const Real gph = m3ToGal(e.engine.fuelFlow / ac.definition().fuel.density) * 3600;
    out_ << t.time << ',' << t.positionNed.x << ',' << t.positionNed.y << ',' << mToFt(t.altitudeMsl) << ','
         << t.heightAgl << ',' << radToDeg(t.roll) << ',' << radToDeg(t.pitch) << ',' << radToDeg(t.heading) << ','
         << msToKt(t.trueAirspeed) << ',' << msToKt(ac.instruments().readings().indicatedAirspeed) << ','
         << msToKt(t.groundSpeed) << ',' << msToFpm(t.verticalSpeed) << ',' << radToDeg(t.alpha) << ','
         << radToDeg(t.beta) << ',' << t.mach << ',' << t.loadFactor << ',' << t.mass << ',' << t.cg.x << ','
         << t.fuelMass << ',' << e.rpm << ',' << paToInHg(e.engine.manifoldPressure) << ',' << gph << ','
         << wToHp(e.engine.brakePower) << ',' << e.prop.thrust << ',' << e.prop.efficiency << ','
         << radToDeg(t.flapAngle) << ',' << t.elevatorTrim << ',' << t.controls.elevator << ',' << t.controls.aileron
         << ',' << t.controls.rudder << ',' << t.onGround << ',' << t.stallWarning << ',' << t.stalledStrips << ','
         << t.busVoltage << ',' << ac.electrical().batteryCharge() << '\n';
}

void InstrumentRecorder::writeHeader() {
    out_ << "time_s,tas_kt,cas_kt,eas_kt,gs_kt,ias_kt,alt_true_ft,alt_pressure_ft,alt_indicated_ft,kollsman_inhg,"
            "vs_true_fpm,vsi_fpm,pitch_true_deg,pitch_ai_deg,roll_true_deg,roll_ai_deg,ai_tumbled,"
            "hdg_true_deg,hdg_mag_deg,hdg_dg_deg,hdg_compass_deg,p_dps,q_dps,r_dps,turn_rate_tc_dps,tc_flag,"
            "slip_true_deg,ball_deg,static_meas_pa,total_meas_pa,ai_rotor,dg_rotor,tc_rotor,"
            "rpm_tach,oil_psi,oil_f,egt_f,fuel_gph,fuel_l_gal,fuel_r_gal,ammeter_a,bus_v,suction_inhg,"
            "annunciators,stall_horn,oat_c,nav_valid,nav_cdi,nav_tofrom,gs_valid,gs_dev,squawk,reported_alt_ft\n";
}

void InstrumentRecorder::record(const Aircraft& ac) {
    const InstrumentTruth& t = ac.instruments().truth();
    const CockpitState& c = ac.instruments().state();
    if (ac.time() - last_ < interval_ - 1e-9) return;
    last_ = ac.time();
    using namespace units;
    auto deg = [](Real r) { return radToDeg(r); };
    auto hdg = [](Real r) { return std::fmod(radToDeg(r) + 360.0, 360.0); };
    auto f = [](Real k) { return k > 0 ? (k - 273.15) * 9 / 5 + 32 : 0.0; };
    out_ << ac.time() << ',' << msToKt(t.trueAirspeed) << ',' << msToKt(t.calibratedAirspeed) << ','
         << msToKt(t.equivalentAirspeed) << ',' << msToKt(t.groundSpeed) << ',' << msToKt(c.indicatedAirspeed) << ','
         << mToFt(t.altitudeMsl) << ',' << mToFt(t.pressureAltitude) << ',' << mToFt(c.indicatedAltitude) << ','
         << paToInHg(c.altimeterSetting) << ',' << msToFpm(t.verticalSpeed) << ',' << msToFpm(c.verticalSpeed) << ','
         << deg(t.pitch) << ',' << deg(c.attitudePitch) << ',' << deg(t.roll) << ',' << deg(c.attitudeRoll) << ','
         << c.attitudeTumbled << ',' << hdg(t.trueHeading) << ',' << hdg(t.magneticHeading) << ','
         << hdg(c.headingIndicator) << ',' << hdg(c.magneticHeading) << ',' << deg(t.omegaBody.x) << ','
         << deg(t.omegaBody.y) << ',' << deg(t.omegaBody.z) << ',' << deg(c.turnRate) << ',' << c.turnCoordinatorFlag
         << ',' << deg(t.slipAngle) << ',' << deg(c.slipBall) << ',' << t.staticPressureMeasured << ','
         << t.totalPressureMeasured << ',' << t.attitudeGyroSpin << ',' << t.headingGyroSpin << ',' << t.turnGyroSpin
         << ',' << c.tachometerRpm << ',' << c.oilPressure / 6894.757 << ',' << f(c.oilTemperature) << ','
         << f(c.egt) << ',' << m3ToGal(c.fuelFlow / ac.definition().fuel.density) * 3600 << ','
         << m3ToGal(c.fuelGauge[0]) << ',' << m3ToGal(c.fuelGauge[1]) << ',' << c.ammeter << ',' << c.voltmeter << ','
         << paToInHg(c.suction) << ',' << c.annunciators << ',' << c.stallHorn << ','
         << kToCelsius(c.outsideAirTemperature) << ',' << c.nav1.valid << ',' << c.nav1.cdi << ',' << c.nav1.toFrom
         << ',' << c.nav1.glideslopeValid << ',' << c.nav1.glideslope << ',' << c.squawk << ','
         << (c.altitudeReported ? c.reportedAltitudeFt : -99999) << '\n';
}

}  // namespace avd
