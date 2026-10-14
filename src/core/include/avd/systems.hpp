// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
//
// Aircraft systems for Milestone 1/2: fuel, electrical, vacuum. Each system
// owns explicit component state (tank quantities, valve positions, line fuel,
// battery charge, bus voltage, pump suction). Cockpit switches change that
// state; dependencies are explicit function inputs, so "fuel valve closed ->
// line drains -> engine stops -> alternator drops off -> battery discharges ->
// flaps stop working" emerges from the connections rather than scripts.
// See docs/architecture/aircraft-systems.md.
#pragma once

#include <string>
#include <vector>

#include "avd/aircraft_def.hpp"
#include "avd/mass.hpp"

namespace avd {

class FuelSystem {
public:
    explicit FuelSystem(const FuelSystemDef& def);
    const FuelSystemDef& def() const { return def_; }

    // Switches / valves.
    int selector() const { return selector_; }
    void setSelector(int position);
    bool shutoffOpen() const { return shutoffOpen_; }
    void setShutoffOpen(bool open) { shutoffOpen_ = open; }
    bool auxPumpOn() const { return auxPump_; }
    void setAuxPump(bool on) { auxPump_ = on; }

    // Quantities (m^3).
    Real tankQuantity(int i) const { return quantity_[i]; }
    Real usable(int i) const { return std::max(0.0, quantity_[i] - def_.tanks[i].unusable); }
    Real totalQuantity() const;
    Real totalUsable() const;
    Real lineQuantity() const { return line_; }
    // Servicing / damage hooks. Quantities are clamped to tank capacity.
    void setTankQuantity(int i, Real m3);
    void setLeakRate(int i, Real m3PerS) { leak_[i] = std::max(0.0, m3PerS); }

    // Fuel available at the engine inlet for this step.
    bool fuelAtEngine() const { return line_ > 1e-3 * def_.lineVolume; }
    // Advance: the engine drew `demand` kg/s for `dt`. `feedPressure` is true
    // when the engine-driven pump turns or the aux pump is powered (or the
    // system is gravity-fed). Returns the mass actually delivered (kg).
    Real step(Real dt, Real demandKgPerS, bool feedPressure);

    // Mass items (one per tank) for the mass model.
    void appendMassItems(std::vector<MassItem>& items) const;
    // Engine demands fuel but none reaches the inlet (line empty).
    bool starving() const { return starving_; }

private:
    FuelSystemDef def_;
    std::vector<Real> quantity_, leak_;
    int selector_ = 0;
    bool shutoffOpen_ = true, auxPump_ = false, starving_ = false;
    Real line_ = 0;
};

struct ElectricalInputs {
    Real alternatorRpm = 0;   // engine rpm driving the alternator
    Real starterCurrent = 0;  // A demanded by the starter (if engaged)
    Real flapMotorCurrent = 0;
    Real auxPumpCurrent = 0;
};

class ElectricalSystem {
public:
    explicit ElectricalSystem(const ElectricalDef& def, Real alternatorMinRpm = 900);

    bool batterySwitch() const { return bat_; }
    bool alternatorSwitch() const { return alt_; }
    bool avionicsSwitch() const { return avionics_; }
    void setBatterySwitch(bool on) { bat_ = on; }
    void setAlternatorSwitch(bool on) { alt_ = on; }
    void setAvionicsSwitch(bool on) { avionics_ = on; }
    void setLoadSwitch(int i, bool on) { loadOn_[i] = on; }
    bool loadSwitch(int i) const { return loadOn_[i]; }
    int loadIndex(const std::string& name) const;
    void setAlternatorFailed(bool failed) { altFailed_ = failed; }

    void step(Real dt, const ElectricalInputs& in);

    Real busVoltage() const { return busVoltage_; }
    Real avionicsBusVoltage() const { return avionics_ ? busVoltage_ : 0; }
    Real batteryCharge() const { return charge_ / def_.batteryCapacity; }  // 0..1
    void setBatteryCharge(Real fraction);
    Real batteryCurrent() const { return batteryCurrent_; }  // + discharge
    bool alternatorOnline() const { return altOnline_; }
    bool loadPowered(int i) const;
    bool powered(Real minimumVoltage = 18) const { return busVoltage_ >= minimumVoltage; }

private:
    Real openCircuitVoltage() const;
    Real internalResistance() const;
    ElectricalDef def_;
    Real altMinRpm_;
    std::vector<bool> loadOn_;
    bool bat_ = false, alt_ = false, avionics_ = false, altFailed_ = false;
    bool altOnline_ = false;
    Real charge_ = 0;
    Real busVoltage_ = 0, batteryCurrent_ = 0;
};

class VacuumSystem {
public:
    explicit VacuumSystem(const VacuumDef& def) : def_(def) {}
    void step(Real dt, Real engineRpm);
    Real suction() const { return suction_; }       // Pa
    Real gyroSpin() const { return gyro_; }          // 0..1 of rated gyro speed
    void setGyroSpin(Real s) { gyro_ = clamp(s, 0.0, 1.0); }
    void setPumpFailed(bool f) { failed_ = f; }

private:
    VacuumDef def_;
    Real suction_ = 0, gyro_ = 0;
    bool failed_ = false;
};

}  // namespace avd
