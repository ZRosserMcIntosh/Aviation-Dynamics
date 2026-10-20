// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
//
// Definition validation. Definitions are untrusted input (mods, downloaded
// content): every number is checked for finiteness and physical sense before
// it can reach the simulation.
#include "avd/aircraft_def.hpp"

#include <sstream>

#include "avd/mass.hpp"

namespace avd {

const char* toString(ControlChannel c) {
    switch (c) {
        case ControlChannel::Aileron: return "aileron";
        case ControlChannel::Elevator: return "elevator";
        case ControlChannel::Rudder: return "rudder";
        case ControlChannel::Flap: return "flap";
        case ControlChannel::Spoiler: return "spoiler";
        case ControlChannel::None: return "none";
    }
    return "none";
}

const char* toString(Provenance p) {
    switch (p) {
        case Provenance::Published: return "published";
        case Provenance::Calibrated: return "calibrated";
        case Provenance::Estimated: return "estimated";
        case Provenance::Gameplay: return "gameplay";
    }
    return "estimated";
}

namespace {

class Checker {
public:
    std::vector<std::string> errors;

    void finite(const std::string& path, Real v) {
        if (!std::isfinite(v)) add(path, "must be a finite number");
    }
    void finite(const std::string& path, const Vec3& v) {
        if (!isFinite(v)) add(path, "must be finite");
    }
    void positive(const std::string& path, Real v) {
        if (!std::isfinite(v) || !(v > 0)) add(path, "must be > 0");
    }
    void nonNegative(const std::string& path, Real v) {
        if (!std::isfinite(v) || v < 0) add(path, "must be >= 0");
    }
    void range(const std::string& path, Real v, Real lo, Real hi) {
        if (!std::isfinite(v) || v < lo || v > hi) {
            std::ostringstream os;
            os << "must be in [" << lo << ", " << hi << "]";
            add(path, os.str());
        }
    }
    void table(const std::string& path, const Table1D& t, bool required = false) {
        if (t.empty()) {
            if (required) add(path, "table required");
            return;
        }
        if (auto e = t.validate(); !e.empty()) add(path, e);
    }
    void add(const std::string& path, const std::string& msg) { errors.push_back(path + ": " + msg); }
};

std::string idx(const std::string& base, std::size_t i) { return base + "[" + std::to_string(i) + "]"; }

void checkSection(Checker& c, const std::string& p, const SectionPolar& s) {
    c.range(p + ".liftSlope", s.liftSlope, 1.0, 8.0);
    c.range(p + ".zeroLiftAlpha", s.zeroLiftAlpha, -0.3, 0.3);
    c.range(p + ".clMax", s.clMax, 0.3, 4.0);
    c.range(p + ".clMin", s.clMin, -3.0, -0.1);
    c.range(p + ".cdMin", s.cdMin, 0.0, 0.2);
    c.range(p + ".cdQuadratic", s.cdQuadratic, 0.0, 1.0);
    c.finite(p + ".clAtCdMin", s.clAtCdMin);
    c.range(p + ".cm0", s.cm0, -0.5, 0.5);
    c.range(p + ".stallWidth", s.stallWidth, 0.005, 0.5);
    c.range(p + ".postStallClDrop", s.postStallClDrop, 0.0, 0.9);
    c.range(p + ".flatPlateCd", s.flatPlateCd, 0.5, 2.5);
}

}  // namespace

std::vector<std::string> AircraftDefinition::validate() const {
    Checker c;
    if (schemaVersion != 1) c.add("schemaVersion", "unsupported version " + std::to_string(schemaVersion) + " (expected 1)");
    if (id.empty()) c.add("id", "required");

    c.positive("emptyMass", emptyMass);
    c.finite("emptyCg", emptyCg);
    c.positive("emptyInertia.ixx", emptyInertiaDiag.x);
    c.positive("emptyInertia.iyy", emptyInertiaDiag.y);
    c.positive("emptyInertia.izz", emptyInertiaDiag.z);
    c.finite("emptyInertia.ixz", emptyIxz);
    {
        // Physically realisable inertia: triangle inequalities and positive definiteness.
        const Vec3& d = emptyInertiaDiag;
        if (d.x + d.y < d.z || d.y + d.z < d.x || d.x + d.z < d.y) c.add("emptyInertia", "violates the triangle inequality of principal moments");
        if (d.x * d.z - emptyIxz * emptyIxz <= 0) c.add("emptyInertia.ixz", "inertia tensor not positive definite");
    }
    c.positive("maxTakeoffMass", maxTakeoffMass);
    if (maxTakeoffMass < emptyMass) c.add("maxTakeoffMass", "must be >= emptyMass");
    c.nonNegative("maxLandingMass", maxLandingMass);
    c.nonNegative("maxRampMass", maxRampMass);
    if (maxRampMass > 0 && maxRampMass < maxTakeoffMass) c.add("maxRampMass", "must be >= maxTakeoffMass");
    c.nonNegative("maxZeroFuelMass", maxZeroFuelMass);
    if (maxZeroFuelMass > 0 && maxZeroFuelMass < emptyMass) c.add("maxZeroFuelMass", "must be >= emptyMass");
    for (std::size_t i = 0; i < stationGroupLimits.size(); ++i) {
        const auto p = idx("stationGroupLimits", i);
        c.positive(p + ".maxMass", stationGroupLimits[i].maxMass);
        if (stationGroupLimits[i].stations.empty()) c.add(p + ".stations", "at least one station");
        for (int s : stationGroupLimits[i].stations)
            if (s < 0 || s >= static_cast<int>(stations.size())) c.add(p + ".stations", "no station " + std::to_string(s));
    }
    for (std::size_t i = 0; i < stations.size(); ++i) {
        const auto p = idx("stations", i);
        c.finite(p + ".position", stations[i].position);
        c.nonNegative(p + ".maxMass", stations[i].maxMass);
        c.range(p + ".initial", stations[i].initial, 0, stations[i].maxMass);
    }
    c.table("cgForwardLimit", cgForwardLimitVsMass);
    c.table("cgAftLimit", cgAftLimitVsMass);

    c.positive("referenceArea", referenceArea);
    c.positive("referenceSpan", referenceSpan);
    c.positive("referenceChord", referenceChord);
    if (surfaces.empty()) c.add("surfaces", "at least one lifting surface required");
    if (surfaces.size() > 8) c.add("surfaces", "at most 8 lifting surfaces");
    for (std::size_t s = 0; s < surfaces.size(); ++s) {
        const LiftingSurfaceDef& L = surfaces[s];
        const auto p = idx("surfaces", s);
        if (L.panels.empty()) c.add(p + ".panels", "at least one panel required");
        checkSection(c, p + ".section", L.section);
        c.range(p + ".spanEfficiency", L.spanEfficiency, 0.3, 1.2);
        if (L.downwashSource >= static_cast<int>(s)) c.add(p + ".downwashSource", "must refer to an earlier surface");
        c.finite(p + ".downwashGradient", L.downwashGradient);
        c.range(p + ".downwashAtZeroLift", L.downwashAtZeroLift, -0.3, 0.3);
        c.range(p + ".slipstreamExposure", L.slipstreamExposure, 0, 1);
        if (L.slipstreamEngine >= static_cast<int>(engines.size())) c.add(p + ".slipstreamEngine", "no such engine");
        c.range(p + ".thicknessRatio", L.thicknessRatio, 0.01, 0.4);
        c.range(p + ".kornFactor", L.kornFactor, 0.7, 1.0);
        int strips = 0;
        for (std::size_t k = 0; k < L.panels.size(); ++k) {
            const PanelDef& P = L.panels[k];
            const auto pp = idx(p + ".panels", k);
            c.finite(pp + ".rootQuarterChord", P.rootQuarterChord);
            c.positive(pp + ".span", P.span);
            c.positive(pp + ".rootChord", P.rootChord);
            c.positive(pp + ".tipChord", P.tipChord);
            c.range(pp + ".sweep", P.sweep, -1.2, 1.2);
            c.range(pp + ".dihedral", P.dihedral, -1.5, 1.5);  // to ~86 deg: canted winglets
            c.range(pp + ".rootIncidence", P.rootIncidence, -0.5, 0.5);
            c.range(pp + ".tipIncidence", P.tipIncidence, -0.5, 0.5);
            c.range(pp + ".strips", P.strips, 1, 64);
            strips += P.strips * (P.mirrored ? 2 : 1);
            for (std::size_t m = 0; m < P.controls.size(); ++m) {
                const ControlSurfaceDef& C = P.controls[m];
                const auto pc = idx(pp + ".controls", m);
                if (C.channel == ControlChannel::None) c.add(pc + ".channel", "required");
                c.range(pc + ".spanStart", C.spanStart, 0, 1);
                c.range(pc + ".spanEnd", C.spanEnd, 0, 1);
                if (C.spanEnd <= C.spanStart) c.add(pc + ".spanEnd", "must be > spanStart");
                c.range(pc + ".chordFraction", C.chordFraction, 0.02, 0.6);
                c.range(pc + ".maxPositive", C.maxPositive, 0, 1.2);
                c.range(pc + ".maxNegative", C.maxNegative, 0, 1.2);
                c.table(pc + ".effectiveness", C.effectivenessVsDeflection);
                c.range(pc + ".clMaxIncrementRatio", C.clMaxIncrementRatio, 0, 1.5);
                c.range(pc + ".profileDragPerRad2", C.profileDragPerRad2, 0, 5);
                c.range(pc + ".chordExtension", C.chordExtension, 0, 0.6);
            }
        }
        if (strips > 512) c.add(p, "too many strips (max 512)");
    }
    for (std::size_t i = 0; i < fuselages.size(); ++i) {
        const auto p = idx("fuselages", i);
        c.finite(p + ".referencePoint", fuselages[i].referencePoint);
        c.nonNegative(p + ".volume", fuselages[i].volume);
        c.range(p + ".munkFactor", fuselages[i].munkFactor, 0, 1);
        c.nonNegative(p + ".crossflowArea", fuselages[i].crossflowArea);
        c.range(p + ".crossflowCd", fuselages[i].crossflowCd, 0, 2);
    }
    for (std::size_t i = 0; i < dragItems.size(); ++i) {
        c.finite(idx("dragItems", i) + ".position", dragItems[i].position);
        c.nonNegative(idx("dragItems", i) + ".dragArea", dragItems[i].dragArea);
        c.range(idx("dragItems", i) + ".criticalMach", dragItems[i].criticalMach, 0, 1.2);
        c.range(idx("dragItems", i) + ".bodyLength", dragItems[i].bodyLength, 0, 30);
        c.range(idx("dragItems", i) + ".bodySize", dragItems[i].bodySize, 0, 5);
        if (dragItems[i].bodySize > 0) c.finite(idx("dragItems", i) + ".bodyAxis", dragItems[i].bodyAxis);
    }

    if (engines.size() > 4) c.add("engines", "at most 4 engines");
    if (turbofans.size() > 4) c.add("turbofans", "at most 4 engines");
    if (!engines.empty() && !turbofans.empty()) c.add("turbofans", "piston engines and turbofans cannot be mixed");
    for (std::size_t i = 0; i < turbofans.size(); ++i) {
        const TurbofanDef& e = turbofans[i];
        const auto p = idx("turbofans", i);
        c.finite(p + ".position", e.position);
        if (!(length(e.thrustAxis) > 0.5)) c.add(p + ".thrustAxis", "must be a non-zero vector");
        c.positive(p + ".staticThrust", e.staticThrust);
        c.range(p + ".bypassRatio", e.bypassRatio, 0, 15);
        c.range(p + ".throttleRatio", e.throttleRatio, 0.9, 1.3);
        c.range(p + ".lapseMachCoefficient", e.lapseMachCoefficient, 0, 1);
        c.range(p + ".idleN1", e.idleN1, 0.1, 0.5);
        c.range(p + ".idleN2", e.idleN2, 0.3, 0.8);
        c.range(p + ".idleThrustFraction", e.idleThrustFraction, 0, 0.2);
        c.range(p + ".thrustExponent", e.thrustExponent, 0.5, 4);
        c.positive(p + ".accelerationTime", e.accelerationTime);
        c.positive(p + ".decelerationTime", e.decelerationTime);
        c.positive(p + ".startTime", e.startTime);
        c.positive(p + ".tsfcStatic", e.tsfcStatic);
        c.range(p + ".tsfcMachSlope", e.tsfcMachSlope, 0, 3);
        c.nonNegative(p + ".idleFuelFlow", e.idleFuelFlow);
        c.positive(p + ".ittIdle", e.ittIdle);
        c.positive(p + ".ittTakeoff", e.ittTakeoff);
        c.nonNegative(p + ".oilPressureIdle", e.oilPressureIdle);
        c.nonNegative(p + ".oilPressureMax", e.oilPressureMax);
        if (e.reverser.present) {
            c.range(p + ".reverser.effectiveness", e.reverser.effectiveness, 0, 0.8);
            c.positive(p + ".reverser.transitTime", e.reverser.transitTime);
            c.range(p + ".reverser.reverseN1", e.reverser.reverseN1, e.idleN1, 1);
            c.range(p + ".reverser.stowIas", e.reverser.stowIas, 0, 100);
            c.range(p + ".reverser.cascadeForward", e.reverser.cascadeForward, 0, 6);
            c.range(p + ".reverser.cowlRadius", e.reverser.cowlRadius, 0.1, 3);
        }
    }
    for (std::size_t i = 0; i < engines.size(); ++i) {
        const PistonEngineDef& e = engines[i];
        const auto p = idx("engines", i);
        c.finite(p + ".position", e.position);
        if (!(length(e.thrustAxis) > 0.5)) c.add(p + ".thrustAxis", "must be a non-zero vector");
        c.positive(p + ".ratedPower", e.ratedPower);
        c.range(p + ".ratedRpm", e.ratedRpm, 500, 20000);
        c.positive(p + ".displacement", e.displacement);
        c.range(p + ".volumetricEfficiency", e.volumetricEfficiency, 0.3, 1.3);
        c.nonNegative(p + ".frictionMepAtRated", e.frictionMepAtRated);
        c.nonNegative(p + ".frictionMepAtZero", e.frictionMepAtZero);
        c.range(p + ".idleManifoldRatio", e.idleManifoldRatio, 0.05, 0.9);
        c.range(p + ".intakeLossRatio", e.intakeLossRatio, 0, 0.3);
        c.positive(p + ".engineInertia", e.engineInertia);
        c.nonNegative(p + ".starterTorque", e.starterTorque);
        c.nonNegative(p + ".starterCurrent", e.starterCurrent);
        c.range(p + ".singleMagnetoPowerFactor", e.singleMagnetoPowerFactor, 0.5, 1);
        c.range(p + ".mixtureRichPhi", e.mixtureRichPhi, 0.8, 2.5);
        c.positive(p + ".fuelHeatingValue", e.fuelHeatingValue);
        c.range(p + ".stoichiometricFar", e.stoichiometricFar, 0.03, 0.12);
        c.table(p + ".powerVsPhi", e.powerVsPhi);
        c.range(p + ".pFactorGain", e.pFactorGain, 0, 1);
        c.range(p + ".slipstreamSwirlFactor", e.slipstreamSwirlFactor, 0, 3);
        if (e.fuelFeed != 0) c.add(p + ".fuelFeed", "only feed 0 is supported in schema v1");
        const PropellerDef& pr = e.propeller;
        c.range(p + ".propeller.diameter", pr.diameter, 0.3, 7);
        c.range(p + ".propeller.blades", pr.blades, 1, 8);
        c.range(p + ".propeller.pitch", pr.pitch, 0.1, 8);
        c.range(p + ".propeller.hubRadiusFraction", pr.hubRadiusFraction, 0.02, 0.5);
        c.table(p + ".propeller.chord", pr.chordVsRadius, true);
        for (Real v : pr.chordVsRadius.ys())
            if (!(v > 0 && v < pr.diameter)) { c.add(p + ".propeller.chord", "chords must be in (0, diameter)"); break; }
        checkSection(c, p + ".propeller.section", pr.section);
        c.positive(p + ".propeller.inertia", pr.inertia);
        c.range(p + ".propeller.bemElements", pr.bemElements, 4, 100);
    }

    c.range("fuel.density", fuel.density, 500, 1000);
    for (std::size_t i = 0; i < fuel.tanks.size(); ++i) {
        const FuelTankDef& t = fuel.tanks[i];
        const auto p = idx("fuel.tanks", i);
        c.finite(p + ".position", t.position);
        c.positive(p + ".capacity", t.capacity);
        c.range(p + ".unusable", t.unusable, 0, t.capacity);
        c.range(p + ".initial", t.initial, 0, t.capacity);
    }
    for (std::size_t i = 0; i < fuel.selector.size(); ++i)
        for (int t : fuel.selector[i].tanks)
            if (t < 0 || t >= static_cast<int>(fuel.tanks.size())) c.add(idx("fuel.selector", i) + ".tanks", "no such tank");
    if (!fuel.selector.empty()) c.range("fuel.initialSelector", fuel.initialSelector, 0, static_cast<Real>(fuel.selector.size() - 1));
    c.positive("fuel.lineVolume", fuel.lineVolume);

    c.range("electrical.batteryVoltage", electrical.batteryVoltage, 6, 60);
    c.positive("electrical.batteryCapacity", electrical.batteryCapacity);
    c.range("electrical.batteryResistance", electrical.batteryResistance, 1e-4, 1);
    c.range("electrical.alternatorVoltage", electrical.alternatorVoltage, 6, 60);
    c.nonNegative("electrical.alternatorMaxCurrent", electrical.alternatorMaxCurrent);
    for (std::size_t i = 0; i < electrical.loads.size(); ++i) {
        c.nonNegative(idx("electrical.loads", i) + ".current", electrical.loads[i].current);
        c.nonNegative(idx("electrical.loads", i) + ".minimumVoltage", electrical.loads[i].minimumVoltage);
    }

    if (gear.size() > 8) c.add("gear", "at most 8 gear legs");
    for (std::size_t i = 0; i < gear.size(); ++i) {
        const GearDef& g = gear[i];
        const auto p = idx("gear", i);
        c.finite(p + ".attach", g.attach);
        c.positive(p + ".length", g.length);
        c.positive(p + ".stiffness", g.stiffness);
        c.nonNegative(p + ".preload", g.preload);
        c.nonNegative(p + ".dampingCompression", g.dampingCompression);
        c.nonNegative(p + ".dampingRebound", g.dampingRebound);
        c.positive(p + ".maxCompression", g.maxCompression);
        c.positive(p + ".wheelRadius", g.wheelRadius);
        c.range(p + ".rollingResistance", g.rollingResistance, 0, 0.5);
        c.range(p + ".maxBrakeForceCoefficient", g.maxBrakeForceCoefficient, 0, 1.5);
        c.range(p + ".brakeGroup", g.brakeGroup, -1, 1);
        c.range(p + ".maxSteerAngle", g.maxSteerAngle, 0, 1.6);
        c.range(p + ".tillerSteerAngle", g.tillerSteerAngle, 0, 1.6);
        c.nonNegative(p + ".steerRate", g.steerRate);
        c.range(p + ".castorAngle", g.castorAngle, 0, 1.6);
        c.range(p + ".castorTrail", g.castorTrail, 0, 1);
        c.nonNegative(p + ".steerLinkageStiffness", g.steerLinkageStiffness);
        c.nonNegative(p + ".steerLinkageBreakout", g.steerLinkageBreakout);
        c.positive(p + ".corneringStiffness", g.corneringStiffness);
        c.range(p + ".corneringLoadSensitivity", g.corneringLoadSensitivity, 0, 0.45);
        c.nonNegative(p + ".staticLoad", g.staticLoad);
        c.positive(p + ".relaxationLength", g.relaxationLength);
        c.positive(p + ".longitudinalStiffness", g.longitudinalStiffness);
        c.range(p + ".tyrePressure", g.tyrePressure, 0, 3e6);
        c.nonNegative(p + ".wheelInertia", g.wheelInertia);
        c.nonNegative(p + ".brakeTorque", g.brakeTorque);
        c.range(p + ".slidingFrictionRatio", g.slidingFrictionRatio, 0.2, 1);
        c.range(p + ".antiSkidEfficiency", g.antiSkidEfficiency, 0, 1);
    }
    if (contacts.size() > 16) c.add("contacts", "at most 16 contact points");
    for (std::size_t i = 0; i < contacts.size(); ++i) {
        c.finite(idx("contacts", i) + ".position", contacts[i].position);
        c.positive(idx("contacts", i) + ".stiffness", contacts[i].stiffness);
        c.nonNegative(idx("contacts", i) + ".damping", contacts[i].damping);
        c.range(idx("contacts", i) + ".friction", contacts[i].friction, 0, 2);
    }
    if (controls.flapDetents.empty()) c.add("controls.flapDetents", "at least one detent (0) required");
    for (std::size_t i = 0; i < controls.flapDetents.size(); ++i) c.range(idx("controls.flapDetents", i), controls.flapDetents[i], -0.2, 1.2);
    c.positive("controls.flapRate", controls.flapRate);
    c.nonNegative("controls.flapMotorCurrent", controls.flapMotorCurrent);
    c.range("controls.elevatorTrimRange", controls.elevatorTrimRange, 0, 0.6);
    c.range("controls.stabilizerNoseUp", controls.stabilizerNoseUp, 0, 0.35);
    c.range("controls.stabilizerNoseDown", controls.stabilizerNoseDown, 0, 0.35);
    c.positive("controls.elevatorTrimRate", controls.elevatorTrimRate);
    c.range("controls.riggingAileron", controls.riggingAileron, -0.3, 0.3);
    c.range("controls.riggingRudder", controls.riggingRudder, -0.3, 0.3);
    c.range("controls.rudderLimiterIas", controls.rudderLimiterIas, 0, 300);
    c.positive("controls.gear.transitTime", controls.gear.transitTime);
    c.nonNegative("controls.gear.dragArea", controls.gear.dragArea);
    c.nonNegative("controls.gear.maxExtendedIas", controls.gear.maxExtendedIas);
    if (controls.speedbrake.present) {
        c.range("controls.speedbrake.detents", controls.speedbrake.detents, 2, 6);
        c.nonNegative("controls.speedbrake.dragArea", controls.speedbrake.dragArea);
        c.range("controls.speedbrake.liftLossFraction", controls.speedbrake.liftLossFraction, 0, 0.9);
        c.positive("controls.speedbrake.rate", controls.speedbrake.rate);
    }
    c.finite("pitotStatic.pitotPosition", pitotStatic.pitotPosition);
    c.finite("pitotStatic.staticPosition", pitotStatic.staticPosition);
    c.table("pitotStatic.staticErrorVsAlpha", pitotStatic.staticErrorVsAlpha);
    c.positive("pitotStatic.vsiTimeConstant", pitotStatic.vsiTimeConstant);
    if (stallWarning.present && (stallWarning.surface < 0 || stallWarning.surface >= static_cast<int>(surfaces.size())))
        c.add("stallWarning.surface", "no such surface");
    c.range("stallWarning.alphaMargin", stallWarning.alphaMargin, 0, 0.3);
    c.range("stallWarning.pusherMargin", stallWarning.pusherMargin, 0, stallWarning.alphaMargin);
    c.range("stallWarning.pusherElevator", stallWarning.pusherElevator, -1, 0);
    c.positive("vacuum.regulatedSuction", vacuum.regulatedSuction);
    c.positive("vacuum.rpmForRegulated", vacuum.rpmForRegulated);
    c.positive("vacuum.gyroSpinUpTime", vacuum.gyroSpinUpTime);
    c.positive("vacuum.gyroSpinDownTime", vacuum.gyroSpinDownTime);
    {
        const InstrumentPanelDef& p = panel;
        auto needle = [&](const std::string& path, const NeedleDynamics& n) {
            c.range(path + ".naturalFrequency", n.naturalFrequency, 0.1, 50);
            c.range(path + ".damping", n.damping, 0.05, 3);
        };
        needle("panel.airspeed.needle", p.airspeed.needle);
        needle("panel.altimeter.needle", p.altimeter.needle);
        needle("panel.vsi.needle", p.vsi.needle);
        needle("panel.turnCoordinator.gimbal", p.turnCoordinator.gimbal);
        needle("panel.inclinometer.ball", p.inclinometer.ball);
        needle("panel.compass.card", p.compass.card);
        needle("panel.engine.needle", p.engine.needle);
        c.range("panel.airspeed.pitotInsensitiveAngle", p.airspeed.pitotInsensitiveAngle, 0, 0.8);
        if (!(p.altimeter.settingMax > p.altimeter.settingMin)) c.add("panel.altimeter.settingMax", "must exceed settingMin");
        c.range("panel.vsi.capillaryTimeConstant", p.vsi.capillaryTimeConstant, 0.2, 30);
        c.positive("panel.vsi.dialRange", p.vsi.dialRange);
        c.range("panel.attitude.erectionRate", p.attitude.erectionRate, 0, 0.05);
        c.range("panel.attitude.pitchLimit", p.attitude.pitchLimit, 0.2, kPi / 2);
        c.range("panel.attitude.rollLimit", p.attitude.rollLimit, 0, kPi);
        c.positive("panel.attitude.spinUpTime", p.attitude.spinUpTime);
        c.positive("panel.attitude.spinDownTime", p.attitude.spinDownTime);
        c.range("panel.attitude.minimumUsefulSpin", p.attitude.minimumUsefulSpin, 0.05, 1);
        c.range("panel.heading.mechanicalDrift", p.heading.mechanicalDrift, -0.01, 0.01);
        if (p.heading.gimbalLimit != 0) c.range("panel.heading.gimbalLimit", p.heading.gimbalLimit, 0.2, kPi / 2);
        c.range("panel.heading.slavingRate", p.heading.slavingRate, 0, 0.5);
        c.positive("panel.heading.spinUpTime", p.heading.spinUpTime);
        c.positive("panel.heading.spinDownTime", p.heading.spinDownTime);
        c.range("panel.turnCoordinator.cant", p.turnCoordinator.cant, 0, 1.2);
        c.range("panel.turnCoordinator.standardRateDeflection", p.turnCoordinator.standardRateDeflection, 0.05, 1.2);
        c.range("panel.turnCoordinator.maxDeflection", p.turnCoordinator.maxDeflection, 0.05, 1.5);
        c.positive("panel.turnCoordinator.spinUpTime", p.turnCoordinator.spinUpTime);
        c.positive("panel.turnCoordinator.spinDownTime", p.turnCoordinator.spinDownTime);
        c.range("panel.inclinometer.tubeLimit", p.inclinometer.tubeLimit, 0.05, 1.0);
        c.range("panel.compass.tiltLimit", p.compass.tiltLimit, 0.05, 0.8);
        c.table("panel.compass.deviation", p.compass.deviation);
        c.positive("panel.engine.fuelSenderTimeConstant", p.engine.fuelSenderTimeConstant);
        c.range("panel.oat.recoveryFactor", p.oat.recoveryFactor, 0, 1);
        c.range("panel.avionics.navComRadios", p.avionics.navComRadios, 0, 2);
        if (p.avionics.comSpacingKhz != 25 && p.avionics.comSpacingKhz != 8.33)
            c.add("panel.avionics.comSpacingKhz", "must be 25 or 8.33");
    }
    c.nonNegative("limits.neverExceedIas", limits.neverExceedIas);
    c.range("limits.maxOperatingMach", limits.maxOperatingMach, 0, 3);
    c.table("limits.vfeVsFlap", limits.flapExtendedIasVsFlap);
    c.range("limits.limitLoadPositive", limits.limitLoadPositive, 1, 15);
    c.range("limits.limitLoadNegative", limits.limitLoadNegative, -15, 0);
    return c.errors;
}

}  // namespace avd
