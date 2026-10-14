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

}  // namespace avd
