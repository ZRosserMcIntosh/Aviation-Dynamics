// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
#include "avd/systems.hpp"

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
    : def_(def), altMinRpm_(alternatorMinRpm), loadOn_(def.loads.size(), true), charge_(def.batteryCapacity) {}

int ElectricalSystem::loadIndex(const std::string& name) const {
    for (std::size_t i = 0; i < def_.loads.size(); ++i)
        if (def_.loads[i].name == name) return static_cast<int>(i);
    return -1;
}

void ElectricalSystem::setBatteryCharge(Real f) { charge_ = clamp(f, 0.0, 1.0) * def_.batteryCapacity; }

Real ElectricalSystem::openCircuitVoltage() const {
    // Lead-acid: ~2.13 V/cell full, ~1.95 V/cell flat; collapses when empty.
    const Real soc = batteryCharge();
    const Real full = def_.batteryVoltage * 1.065, flat = def_.batteryVoltage * 0.975;
    return soc > 0.02 ? lerp(flat, full, soc) : lerp(0.0, flat, soc / 0.02);
}

Real ElectricalSystem::internalResistance() const {
    // Lead-acid internal resistance rises steeply as the plates sulphate
    // toward full discharge (Estimated shape): x1 when full, ~x5 when flat.
    const Real d = 1 - batteryCharge();
    return def_.batteryResistance * (1 + 4 * d * d * d);
}

bool ElectricalSystem::loadPowered(int i) const {
    if (i < 0 || i >= static_cast<int>(def_.loads.size()) || !loadOn_[i]) return false;
    const ElectricalLoadDef& l = def_.loads[i];
    const Real v = l.onAvionicsBus ? avionicsBusVoltage() : busVoltage_;
    return v >= l.minimumVoltage;
}

void ElectricalSystem::step(Real dt, const ElectricalInputs& in) {
    // Bus load current at nominal voltage (constant-current loads).
    Real loadCurrent = in.starterCurrent + in.flapMotorCurrent + in.auxPumpCurrent;
    for (std::size_t i = 0; i < def_.loads.size(); ++i) {
        if (!loadOn_[i]) continue;
        if (def_.loads[i].onAvionicsBus && !avionics_) continue;
        loadCurrent += def_.loads[i].current;
    }
    // Alternator: field excited from the bus through the ALT switch; needs
    // bus voltage from the previous step (battery) to come online.
    const bool fieldExcited = alt_ && busVoltage_ > 8.0;
    altOnline_ = !altFailed_ && fieldExcited && in.alternatorRpm >= altMinRpm_;
    const Real ocv = openCircuitVoltage();

    if (altOnline_) {
        // Regulated bus; the battery charges with a current limited by its
        // internal resistance and the alternator rating.
        const Real chargeI = bat_ ? clamp((def_.alternatorVoltage - ocv) / (def_.batteryResistance * 4), 0.0, 20.0) : 0;
        const Real total = loadCurrent + chargeI;
        if (total <= def_.alternatorMaxCurrent) {
            busVoltage_ = def_.alternatorVoltage;
            batteryCurrent_ = -chargeI;
        } else if (bat_) {
            // Overload: the battery supplies the excess at its terminal voltage.
            batteryCurrent_ = loadCurrent - def_.alternatorMaxCurrent;
            busVoltage_ = std::max(ocv - batteryCurrent_ * internalResistance(), 0.0);
        } else {
            busVoltage_ = def_.alternatorVoltage * def_.alternatorMaxCurrent / total;
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
}

// --------------------------------------------------------------- vacuum ----

void VacuumSystem::step(Real dt, Real rpm) {
    if (!def_.present) return;
    suction_ = failed_ ? 0.0 : def_.regulatedSuction * clamp(rpm / def_.rpmForRegulated, 0.0, 1.0);
    // Gyro rotor speed relaxes toward the speed the suction can sustain.
    const Real target = clamp(suction_ / (0.8 * def_.regulatedSuction), 0.0, 1.0);
    const Real tau = target > gyro_ ? def_.gyroSpinUpTime : def_.gyroSpinDownTime;
    gyro_ += (target - gyro_) * std::min(dt / tau, 1.0);
}

}  // namespace avd
