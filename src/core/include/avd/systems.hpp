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

#include <array>
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

inline constexpr int kMaxGenerators = 4;

struct ElectricalInputs {
    // Drive speed of each generator relative to its cut-in speed (>= 1: at
    // speed; a piston alternator at rpm / alternatorMinRpm, a turbofan
    // starter-generator at N2 / cutInN2). `alternatorRpm` is the single
    // alternator's engine rpm (generator 0; kept for one-alternator callers).
    Real alternatorRpm = 0;
    std::array<Real, kMaxGenerators> generatorDrive{};
    Real starterCurrent = 0;  // A demanded by the starter (if engaged)
    Real flapMotorCurrent = 0;
    Real auxPumpCurrent = 0;  // the electric fuel pump while it runs (its breaker's declared current)
    Real hydraulicPumpCurrent = 0;  // electric hydraulic pumps while they run
};

// Main-bus electrical system: a lead-acid battery, one or more engine-driven
// generators in parallel, named loads with breakers, an avionics bus relay
// and an essential bus with load shedding.
//
// Load shed: the loads marked essential form the essential bus. With the
// definition's `autoLoadShed`, a bus controller sheds the rest once the
// battery alone has carried the bus for `loadShedDelay` (a G650-class jet
// after a dual generator loss keeps PFD 1, standby instruments, COM 1/NAV 1,
// engine indication, boost pumps and probe heat; the galley, lights and the
// other displays drop), which roughly doubles the battery's endurance; the
// shed clears when a generator comes back on line. The pilot can shed by
// hand (`setLoadShed`, cockpit "electrical.load_shed"), which is how a light
// aircraft's POH has it done after an alternator failure. A shed load draws
// no current and reads unpowered; its switch position is untouched.
//
// Battery (Estimated shape, Published plateau): under load a charged 24 V
// lead-acid battery sits on a plateau of ~24.4 V (12 x 2.03 V), not at its
// resting open-circuit 25.5 V; the resting value is a surface charge that a
// charger builds in a minute and a load strips in about as long. Its internal
// resistance rises steeply toward full discharge, and its acceptance of charge
// falls as it fills (CC/CV: the regulator's overpotential across a resistance
// that grows near full), so the ammeter settles to a few amps in cruise.
class ElectricalSystem {
public:
    explicit ElectricalSystem(const ElectricalDef& def, Real alternatorMinRpm = 900);
    const ElectricalDef& def() const { return def_; }

    bool batterySwitch() const { return bat_; }
    // The alternator / generator master: all generator switches on.
    bool alternatorSwitch() const;
    bool avionicsSwitch() const { return avionics_; }
    void setBatterySwitch(bool on) { bat_ = on; }
    void setAlternatorSwitch(bool on);   // every generator
    void setAvionicsSwitch(bool on) { avionics_ = on; }
    // The pilot's load shed (held until reset, whatever the generators do).
    void setLoadShed(bool on) { manualShed_ = on; }
    bool manualLoadShed() const { return manualShed_; }
    // The bus controller's automatic shed (the definition's autoLoadShed).
    bool autoLoadShed() const { return autoShed_; }
    // Non-essential loads shed, by the controller or the pilot.
    bool loadShed() const { return autoShed_ || manualShed_; }
    void setLoadSwitch(int i, bool on) { loadOn_[i] = on; }
    bool loadSwitch(int i) const { return loadOn_[i]; }
    int loadIndex(const std::string& name) const;
    void setAlternatorFailed(bool failed);  // every generator

    // Generators: one alternator unless the definition declares a list.
    int generatorCount() const { return static_cast<int>(gens_.size()); }
    const GeneratorDef& generator(int i) const { return gens_[static_cast<std::size_t>(i)]; }
    void setGeneratorSwitch(int i, bool on);
    bool generatorSwitch(int i) const { return genOn_[static_cast<std::size_t>(i)]; }
    void setGeneratorFailed(int i, bool failed);
    bool generatorFailed(int i) const { return genFailed_[static_cast<std::size_t>(i)]; }
    bool generatorOnline(int i) const { return genOnline_[static_cast<std::size_t>(i)]; }
    Real generatorCapacity() const;  // A, the generators on line

    void step(Real dt, const ElectricalInputs& in);

    Real busVoltage() const { return busVoltage_; }
    Real avionicsBusVoltage() const { return avionics_ ? busVoltage_ : 0; }
    Real batteryCharge() const { return charge_ / def_.batteryCapacity; }  // 0..1
    void setBatteryCharge(Real fraction);
    Real batteryCurrent() const { return batteryCurrent_; }  // + discharge
    bool batteryDepleted() const { return charge_ <= 0; }
    bool alternatorOnline() const;   // any generator on line
    bool loadPowered(int i) const;
    // Load i is cut by a shed (non-essential while loadShed()).
    bool loadShedded(int i) const;
    // Bus-powered devices work above the definition's bus minimum (default
    // 18 V on a 24 V system); pass a voltage to ask about another threshold.
    Real busMinimum() const { return def_.busMinimum(); }
    bool powered(Real minimumVoltage = -1) const {
        return busVoltage_ >= (minimumVoltage >= 0 ? minimumVoltage : def_.busMinimum());
    }
    // The breaker protecting the electric fuel pump ("fuel_pump"): its current
    // is drawn through ElectricalInputs::auxPumpCurrent only while the pump runs.
    int fuelPumpLoad() const { return pumpLoad_; }

private:
    Real openCircuitVoltage() const;
    Real internalResistance() const;
    Real chargeAcceptanceResistance() const;
    ElectricalDef def_;
    Real altMinRpm_;
    std::vector<GeneratorDef> gens_;
    std::vector<bool> loadOn_;
    std::vector<bool> genOn_, genFailed_, genOnline_;
    int pumpLoad_ = -1;
    bool bat_ = false, avionics_ = false;
    bool manualShed_ = false, autoShed_ = false;
    Real batteryOnlyTime_ = 0;  // s the battery alone has carried the bus
    Real charge_ = 0;
    Real surface_ = 1;   // surface charge, 0..1 (a fresh battery off the charger: 1)
    Real busVoltage_ = 0, batteryCurrent_ = 0;
};

class VacuumSystem {
public:
    explicit VacuumSystem(const VacuumDef& def) : def_(def) {}
    void step(Real dt, Real engineRpm);
    // Suction at the instrument manifold (Pa). Gyro rotors belong to the
    // instruments (avd/instruments.hpp), which spin from this suction.
    Real suction() const { return suction_; }
    void setPumpFailed(bool f) { failed_ = f; }
    bool pumpFailed() const { return failed_; }

private:
    VacuumDef def_;
    Real suction_ = 0;
    bool failed_ = false;
};

}  // namespace avd
