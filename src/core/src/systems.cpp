// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
#include "avd/systems.hpp"

#include <cmath>

namespace avd {

// ---------------------------------------------------------------- fuel ----

FuelSystem::FuelSystem(const FuelSystemDef& def)
    : def_(def), quantity_(def.tanks.size()), leak_(def.tanks.size(), 0.0), selector_(def.initialSelector) {
    for (std::size_t i = 0; i < def.tanks.size(); ++i)
        quantity_[i] = clamp(def.tanks[i].initial, 0.0, def.tanks[i].capacity);
    line_ = def.lineVolume;  // primed
}

void FuelSystem::setSelector(int p) {
    if (p >= 0 && p < static_cast<int>(def_.selector.size())) selector_ = p;
}

void FuelSystem::setTankQuantity(int i, Real m3) { quantity_[i] = clamp(m3, 0.0, def_.tanks[i].capacity); }

Real FuelSystem::totalQuantity() const {
    Real s = 0;
    for (Real q : quantity_) s += q;
    return s;
}

Real FuelSystem::totalUsable() const {
    Real s = 0;
    for (std::size_t i = 0; i < quantity_.size(); ++i) s += usable(static_cast<int>(i));
    return s;
}

Real FuelSystem::step(Real dt, Real demandKg, bool feedPressure) {
    // Leaks drain tanks regardless of valves (including unusable fuel).
    for (std::size_t i = 0; i < quantity_.size(); ++i)
        quantity_[i] = std::max(0.0, quantity_[i] - leak_[i] * dt);

    // 1. The engine draws from the line.
    const Real want = std::max(demandKg, 0.0) * dt / def_.density;
    const Real taken = std::min(want, line_);
    line_ -= taken;

    // 2. Refill the line from the selected tanks through the open valves,
    //    evenly from selected tanks that still hold usable fuel (equal gravity
    //    heads assumed; Estimated).
    if (shutoffOpen_ && !def_.selector.empty() && feedPressure) {
        const FuelSelectorPosition& pos = def_.selector[selector_];
        Real need = def_.lineVolume - line_;
        for (int pass = 0; pass < 2 && need > 1e-15; ++pass) {
            int open = 0;
            for (int t : pos.tanks)
                if (usable(t) > 0) ++open;
            if (open == 0) break;
            const Real share = need / open;
            for (int t : pos.tanks) {
                const Real d = std::min(share, usable(t));
                quantity_[t] -= d;
                line_ += d;
                need -= d;
            }
        }
    }
    starving_ = demandKg > 0 && !fuelAtEngine();
    return taken * def_.density;
}

void FuelSystem::appendMassItems(std::vector<MassItem>& items) const {
    for (std::size_t i = 0; i < quantity_.size(); ++i) {
        MassItem m;
        m.name = def_.tanks[i].name;
        m.mass = quantity_[i] * def_.density;
        m.position = def_.tanks[i].position;
        items.push_back(m);
    }
    if (line_ > 0) {
        // Line fuel is small; lump it at the first tank's position.
        MassItem m;
        m.name = "fuel line";
        m.mass = line_ * def_.density;
        m.position = def_.tanks.empty() ? Vec3{} : def_.tanks[0].position;
        items.push_back(m);
    }
}

// ----------------------------------------------------------- electrical ----

ElectricalSystem::ElectricalSystem(const ElectricalDef& def, Real alternatorMinRpm)
    : def_(def), altMinRpm_(alternatorMinRpm), gens_(def.generators), loadOn_(def.loads.size(), true), charge_(def.batteryCapacity) {
    if (gens_.empty()) {
        GeneratorDef g;
        g.name = "ALT";
        g.maxCurrent = def.alternatorMaxCurrent;
        gens_.push_back(g);
    }
    genOn_.assign(gens_.size(), false);
    genFailed_.assign(gens_.size(), false);
    genOnline_.assign(gens_.size(), false);
    for (std::size_t i = 0; i < def.loads.size(); ++i) loadOn_[i] = def.loads[i].initiallyOn;
    pumpLoad_ = loadIndex("fuel_pump");
}

int ElectricalSystem::loadIndex(const std::string& name) const {
    for (std::size_t i = 0; i < def_.loads.size(); ++i)
        if (def_.loads[i].name == name) return static_cast<int>(i);
    return -1;
}

bool ElectricalSystem::alternatorSwitch() const {
    for (bool on : genOn_)
        if (!on) return false;
    return true;
}

void ElectricalSystem::setAlternatorSwitch(bool on) {
    for (std::size_t i = 0; i < genOn_.size(); ++i) genOn_[i] = on;
}

void ElectricalSystem::setAlternatorFailed(bool failed) {
    for (std::size_t i = 0; i < genFailed_.size(); ++i) genFailed_[i] = failed;
}

void ElectricalSystem::setGeneratorSwitch(int i, bool on) {
    if (i >= 0 && i < generatorCount()) genOn_[static_cast<std::size_t>(i)] = on;
}

void ElectricalSystem::setGeneratorFailed(int i, bool failed) {
    if (i >= 0 && i < generatorCount()) genFailed_[static_cast<std::size_t>(i)] = failed;
}

bool ElectricalSystem::alternatorOnline() const {
    for (bool on : genOnline_)
        if (on) return true;
    return false;
}

Real ElectricalSystem::generatorCapacity() const {
    Real a = 0;
    for (std::size_t i = 0; i < gens_.size(); ++i)
        if (genOnline_[i]) a += gens_[i].maxCurrent;
    return a;
}

void ElectricalSystem::setBatteryCharge(Real f) {
    charge_ = clamp(f, 0.0, 1.0) * def_.batteryCapacity;
    surface_ = f >= 0.99 ? 1.0 : 0.0;  // a battery left at this state, off the charger
}

Real ElectricalSystem::openCircuitVoltage() const {
    // Lead-acid plateau under load: ~2.03 V/cell charged, ~1.92 V/cell at 2 %
    // (24.4 and 23.0 V for 12 cells), collapsing when empty; plus the surface
    // charge (up to +0.09 V/cell) the charger leaves and a load strips.
    const Real soc = batteryCharge();
    const Real vn = def_.batteryVoltage;
    const Real full = vn * 1.017, flat = vn * 0.96;
    const Real plateau = soc > 0.02 ? lerp(flat, full, (soc - 0.02) / 0.98) : lerp(0.0, flat, soc / 0.02);
    return plateau + vn * 0.045 * surface_ * std::min(1.0, soc / 0.1);
}

Real ElectricalSystem::internalResistance() const {
    // Lead-acid internal resistance rises steeply as the plates sulphate
    // toward full discharge (Estimated shape): x1 when full, ~x5 when flat.
    const Real d = 1 - batteryCharge();
    return def_.batteryResistance * (1 + 4 * d * d * d);
}

Real ElectricalSystem::chargeAcceptanceResistance() const {
    // The regulator's overpotential drives the charge through a resistance
    // that grows as the battery fills (CC/CV taper): from the internal
    // resistance when discharged to ~1.2 ohm (per 24 V) at full, where a 28 V
    // bus pushes ~2 A into a 24 V battery (Estimated; a C172S settles at a
    // few amps in cruise).
    const Real soc = batteryCharge();
    return def_.batteryResistance + 1.2 * (def_.batteryVoltage / 24.0) * std::pow(soc, 12.0);
}

bool ElectricalSystem::loadShedded(int i) const {
    if (i < 0 || i >= static_cast<int>(def_.loads.size())) return false;
    return loadShed() && !def_.loads[static_cast<std::size_t>(i)].essential;
}

bool ElectricalSystem::loadPowered(int i) const {
    if (i < 0 || i >= static_cast<int>(def_.loads.size()) || !loadOn_[i] || loadShedded(i)) return false;
    const ElectricalLoadDef& l = def_.loads[i];
    const Real v = l.onAvionicsBus ? avionicsBusVoltage() : busVoltage_;
    return v >= l.minimumVoltage;
}

void ElectricalSystem::step(Real dt, const ElectricalInputs& in) {
    // Bus load current at nominal voltage (constant-current loads). The fuel
    // pump's breaker carries the pump's own current only while it runs.
    // A shed non-essential load (bus controller or pilot) draws nothing.
    Real loadCurrent = in.starterCurrent + in.flapMotorCurrent + in.hydraulicPumpCurrent;
    if (pumpLoad_ < 0 || (loadOn_[static_cast<std::size_t>(pumpLoad_)] && !loadShedded(pumpLoad_))) loadCurrent += in.auxPumpCurrent;
    for (std::size_t i = 0; i < def_.loads.size(); ++i) {
        if (!loadOn_[i] || static_cast<int>(i) == pumpLoad_ || loadShedded(static_cast<int>(i))) continue;
        if (def_.loads[i].onAvionicsBus && !avionics_) continue;
        loadCurrent += def_.loads[i].current;
    }
    // Generators: each field excited from the bus through its switch (bus
    // voltage from the previous step, so the battery brings them on line),
    // on line when its own engine turns it fast enough. In parallel: one
    // regulated voltage, their ratings add.
    Real capacity = 0;
    for (std::size_t k = 0; k < gens_.size(); ++k) {
        Real drive = in.generatorDrive[k];
        if (k == 0 && altMinRpm_ > 0) drive = std::max(drive, in.alternatorRpm / altMinRpm_);
        genOnline_[k] = !genFailed_[k] && genOn_[k] && busVoltage_ > 8.0 && drive >= 1.0;
        if (genOnline_[k]) capacity += gens_[k].maxCurrent;
    }
    const bool online = alternatorOnline();
    const Real ocv = openCircuitVoltage();
    // Bus controller: the battery alone on the bus for the delay sheds the
    // non-essential loads; a generator back on line restores them. The
    // current above was drawn with the shed as it stood at step start.
    if (online || !bat_ || charge_ <= 0) {
        batteryOnlyTime_ = 0;
        autoShed_ = false;
    } else {
        batteryOnlyTime_ += dt;
        if (def_.autoLoadShed && batteryOnlyTime_ >= def_.loadShedDelay) autoShed_ = true;
    }

    if (online) {
        // Regulated bus; the battery charges with the taper of its state of
        // charge, limited to 1.5 C and by the generators' rating.
        const Real iMax = 1.5 * def_.batteryCapacity / 3600;
        const Real chargeI = bat_ ? clamp((def_.alternatorVoltage - ocv) / chargeAcceptanceResistance(), 0.0, iMax) : 0;
        const Real total = loadCurrent + chargeI;
        if (total <= capacity) {
            busVoltage_ = def_.alternatorVoltage;
            batteryCurrent_ = -chargeI;
        } else if (bat_) {
            // Overload: the battery supplies the excess at its terminal voltage.
            batteryCurrent_ = loadCurrent - capacity;
            busVoltage_ = std::max(ocv - batteryCurrent_ * internalResistance(), 0.0);
        } else {
            busVoltage_ = def_.alternatorVoltage * capacity / total;
            batteryCurrent_ = 0;
        }
    } else if (bat_) {
        batteryCurrent_ = loadCurrent;
        busVoltage_ = std::max(ocv - loadCurrent * internalResistance(), 0.0);
        if (charge_ <= 0) { busVoltage_ = 0; batteryCurrent_ = 0; }
    } else {
        busVoltage_ = 0;
        batteryCurrent_ = 0;
    }
    charge_ = clamp(charge_ - batteryCurrent_ * dt, 0.0, def_.batteryCapacity);
    // Surface charge: built by the charger (~1 min), stripped by a load
    // (~40 s), left alone it leaks away over ~10 min (Estimated).
    if (batteryCurrent_ < -0.5) surface_ += (1 - surface_) * std::min(dt / 60.0, 1.0);
    else if (batteryCurrent_ > 0.5) surface_ -= surface_ * std::min(dt / 40.0, 1.0);
    else surface_ -= surface_ * std::min(dt / 600.0, 1.0);
}

// --------------------------------------------------------------- vacuum ----

void VacuumSystem::step(Real, Real rpm) {
    if (!def_.present) return;
    // Engine-driven dry pump: suction rises with rpm to the regulator setting.
    suction_ = failed_ ? 0.0 : def_.regulatedSuction * clamp(rpm / def_.rpmForRegulated, 0.0, 1.0);
}

}  // namespace avd
