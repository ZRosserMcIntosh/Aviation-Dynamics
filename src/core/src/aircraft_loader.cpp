// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
#include "avd/aircraft_loader.hpp"

#include <fstream>
#include <sstream>

#include "avd/json.hpp"
#include "avd/units.hpp"

namespace avd {

namespace {

using json::Value;
constexpr Real kDeg = kPi / 180;

// Schema reader for one JSON object: typed getters that record the keys they
// consume, then `finish()` rejects any key nobody asked for (typos).
class Obj {
public:
    Obj(const Value* v, std::string path, std::vector<std::string>& errors)
        : v_(v), path_(std::move(path)), err_(errors) {
        if (v_ && !v_->isObject()) {
            err(path_, "must be an object");
            v_ = nullptr;
        }
    }
    ~Obj() { finish(); }
    Obj(const Obj&) = delete;
    Obj& operator=(const Obj&) = delete;

    bool present() const { return v_ != nullptr; }
    bool has(const std::string& k) const { return v_ && v_->find(k); }
    const std::string& path() const { return path_; }

    void num(const std::string& k, Real& out, Real scale = 1.0, bool required = false) {
        const Value* x = get(k, required);
        if (!x) return;
        if (!x->isNumber()) { err(p(k), "must be a number"); return; }
        out = x->asNumber() * scale;
    }
    void integer(const std::string& k, int& out, bool required = false) {
        const Value* x = get(k, required);
        if (!x) return;
        if (!x->isNumber() || x->asNumber() != std::floor(x->asNumber()) || std::abs(x->asNumber()) > 1e6) {
            err(p(k), "must be an integer");
            return;
        }
        out = static_cast<int>(x->asNumber());
    }
    void boolean(const std::string& k, bool& out) {
        const Value* x = get(k, false);
        if (!x) return;
        if (!x->isBool()) { err(p(k), "must be true or false"); return; }
        out = x->asBool();
    }
    void str(const std::string& k, std::string& out, bool required = false) {
        const Value* x = get(k, required);
        if (!x) return;
        if (!x->isString()) { err(p(k), "must be a string"); return; }
        out = x->asString();
    }
    void vec3(const std::string& k, Vec3& out, Real scale = 1.0, bool required = false) {
        const Value* x = get(k, required);
        if (!x) return;
        if (!x->isArray() || x->items().size() != 3) { err(p(k), "must be an array of 3 numbers [x, y, z]"); return; }
        Real c[3];
        for (int i = 0; i < 3; ++i) {
            if (!x->items()[i].isNumber()) { err(p(k), "must contain numbers"); return; }
            c[i] = x->items()[i].asNumber() * scale;
        }
        out = {c[0], c[1], c[2]};
    }
    void numbers(const std::string& k, std::vector<Real>& out, Real scale = 1.0, bool required = false) {
        const Value* x = get(k, required);
        if (!x) return;
        if (!x->isArray()) { err(p(k), "must be an array of numbers"); return; }
        out.clear();
        for (const Value& e : x->items()) {
            if (!e.isNumber()) { err(p(k), "must contain only numbers"); return; }
            out.push_back(e.asNumber() * scale);
        }
    }
    // {"<xKey>": [...], "<yKey>": [...]} -> Table1D
    void table(const std::string& k, const std::string& xKey, const std::string& yKey, Table1D& out,
               Real xScale = 1.0, Real yScale = 1.0) {
        if (!has(k)) return;
        Obj t = child(k);
        std::vector<Real> xs, ys;
        t.numbers(xKey, xs, xScale, true);
        t.numbers(yKey, ys, yScale, true);
        if (xs.size() != ys.size()) { err(t.path(), "'" + xKey + "' and '" + yKey + "' must have equal length"); return; }
        out = Table1D(std::move(xs), std::move(ys));
    }
    Obj child(const std::string& k, bool required = false) { return Obj(get(k, required), p(k), err_); }
    // Array of objects: calls fn(Obj&, index).
    template <class F>
    void each(const std::string& k, F fn, bool required = false) {
        const Value* x = get(k, required);
        if (!x) return;
        if (!x->isArray()) { err(p(k), "must be an array"); return; }
        for (std::size_t i = 0; i < x->items().size(); ++i) {
            Obj o(&x->items()[i], p(k) + "[" + std::to_string(i) + "]", err_);
            fn(o, i);
        }
    }
    void err(const std::string& path, const std::string& m) { err_.push_back(path + ": " + m); }

private:
    std::string p(const std::string& k) const { return path_.empty() ? k : path_ + "." + k; }
    const Value* get(const std::string& k, bool required) {
        seen_.push_back(k);
        const Value* x = v_ ? v_->find(k) : nullptr;
        if (!x && required && v_) err(p(k), "required");
        if (x && x->isNull()) return nullptr;
        return x;
    }
    void finish() {
        if (!v_ || finished_) return;
        finished_ = true;
        for (const auto& m : v_->members()) {
            bool known = false;
            for (const auto& s : seen_) known = known || s == m.first;
            if (!known) err(p(m.first), "unknown key");
        }
    }
    const Value* v_;
    std::string path_;
    std::vector<std::string>& err_;
    std::vector<std::string> seen_;
    bool finished_ = false;
};

void readSection(Obj& o, SectionPolar& s) {
    o.num("lift_slope_per_rad", s.liftSlope);
    o.num("zero_lift_alpha_deg", s.zeroLiftAlpha, kDeg);
    o.num("cl_max", s.clMax);
    o.num("cl_min", s.clMin);
    o.num("cd_min", s.cdMin);
    o.num("cd_quadratic", s.cdQuadratic);
    o.num("cl_at_cd_min", s.clAtCdMin);
    o.num("cm0", s.cm0);
    o.num("stall_width_deg", s.stallWidth, kDeg);
    o.num("post_stall_cl_drop", s.postStallClDrop);
    o.num("flat_plate_cd", s.flatPlateCd);
}

ControlChannel channelFrom(const std::string& s, Obj& o) {
    if (s == "aileron") return ControlChannel::Aileron;
    if (s == "elevator") return ControlChannel::Elevator;
    if (s == "rudder") return ControlChannel::Rudder;
    if (s == "flap") return ControlChannel::Flap;
    if (s == "spoiler") return ControlChannel::Spoiler;
    o.err(o.path() + ".channel", "must be one of aileron, elevator, rudder, flap");
    return ControlChannel::None;
}

Provenance provenanceFrom(const std::string& s, Obj& o) {
    if (s == "published") return Provenance::Published;
    if (s == "calibrated") return Provenance::Calibrated;
    if (s == "estimated") return Provenance::Estimated;
    if (s == "gameplay") return Provenance::Gameplay;
    o.err(o.path() + ".kind", "must be one of published, calibrated, estimated, gameplay");
    return Provenance::Estimated;
}

constexpr Real kPsi = 6894.757;
Real fToK(Real f) { return (f - 32) * 5.0 / 9.0 + 273.15; }

void readRange(Obj& o, const std::string& key, Range& r, Real scale, Real offset = 0) {
    std::vector<Real> v;
    o.numbers(key, v);
    if (v.empty()) return;
    if (v.size() != 2) { o.err(o.path() + "." + key, "must be [low, high]"); return; }
    r = {v[0] * scale + offset, v[1] * scale + offset};
}

void readRangeF(Obj& o, const std::string& key, Range& r) {
    std::vector<Real> v;
    o.numbers(key, v);
    if (v.empty()) return;
    if (v.size() != 2) { o.err(o.path() + "." + key, "must be [low, high]"); return; }
    r = {fToK(v[0]), fToK(v[1])};
}

void readNeedle(Obj& o, const std::string& key, NeedleDynamics& n) {
    if (!o.has(key)) return;
    Obj c = o.child(key);
    c.num("natural_frequency_rad_s", n.naturalFrequency);
    c.num("damping_ratio", n.damping);
}

GyroPower gyroPowerFrom(const std::string& s, Obj& o) {
    if (s == "vacuum") return GyroPower::Vacuum;
    if (s == "electric") return GyroPower::Electric;
    o.err(o.path() + ".power", "must be vacuum or electric");
    return GyroPower::Vacuum;
}

void readPanel(Obj& o, InstrumentPanelDef& p) {
    const Real kt = units::kKnot;
    o.str("profile", p.profile);
    o.str("pitot_heat_load", p.pitotHeatLoad);
    o.str("panel_lights_load", p.panelLightsLoad);
    o.boolean("clock", p.clock);
    if (o.has("airspeed_indicator")) {
        Obj a = o.child("airspeed_indicator");
        readRange(a, "white_arc_kias", p.airspeed.whiteArc, kt);
        readRange(a, "green_arc_kias", p.airspeed.greenArc, kt);
        readRange(a, "yellow_arc_kias", p.airspeed.yellowArc, kt);
        a.num("red_line_kias", p.airspeed.redLine, kt);
        a.num("dial_min_kias", p.airspeed.dialMin, kt);
        a.num("dial_max_kias", p.airspeed.dialMax, kt);
        a.num("pitot_insensitive_angle_deg", p.airspeed.pitotInsensitiveAngle, kDeg);
        readNeedle(a, "needle", p.airspeed.needle);
    }
    if (o.has("altimeter")) {
        Obj a = o.child("altimeter");
        a.num("setting_min_inhg", p.altimeter.settingMin, units::kInHg);
        a.num("setting_max_inhg", p.altimeter.settingMax, units::kInHg);
        readNeedle(a, "needle", p.altimeter.needle);
    }
    if (o.has("vertical_speed_indicator")) {
        Obj a = o.child("vertical_speed_indicator");
        a.num("capillary_time_constant_s", p.vsi.capillaryTimeConstant);
        a.num("dial_range_fpm", p.vsi.dialRange, units::kFoot / 60);
        readNeedle(a, "needle", p.vsi.needle);
    }
    if (o.has("attitude_indicator")) {
        Obj a = o.child("attitude_indicator");
        std::string power = "vacuum";
        a.boolean("installed", p.attitude.installed);
        a.str("power", power);
        p.attitude.power = gyroPowerFrom(power, a);
        a.str("electrical_load", p.attitude.electricalLoad);
        a.num("erection_rate_deg_per_min", p.attitude.erectionRate, kDeg / 60);
        a.num("pitch_limit_deg", p.attitude.pitchLimit, kDeg);
        a.num("roll_limit_deg", p.attitude.rollLimit, kDeg);
        a.num("spin_up_time_constant_s", p.attitude.spinUpTime);
        a.num("spin_down_time_constant_s", p.attitude.spinDownTime);
        a.num("minimum_useful_spin", p.attitude.minimumUsefulSpin);
    }
    if (o.has("heading_indicator")) {
        Obj a = o.child("heading_indicator");
        std::string power = "vacuum";
        a.boolean("installed", p.heading.installed);
        a.str("power", power);
        p.heading.power = gyroPowerFrom(power, a);
        a.str("electrical_load", p.heading.electricalLoad);
        a.num("mechanical_drift_deg_per_hour", p.heading.mechanicalDrift, kDeg / 3600);
        a.num("gimbal_limit_deg", p.heading.gimbalLimit, kDeg);
        a.num("slaving_rate_deg_per_s", p.heading.slavingRate, kDeg);
        a.num("spin_up_time_constant_s", p.heading.spinUpTime);
        a.num("spin_down_time_constant_s", p.heading.spinDownTime);
    }
    if (o.has("turn_coordinator")) {
        Obj a = o.child("turn_coordinator");
        a.boolean("installed", p.turnCoordinator.installed);
        a.str("electrical_load", p.turnCoordinator.electricalLoad);
        a.num("cant_deg", p.turnCoordinator.cant, kDeg);
        a.num("standard_rate_deflection_deg", p.turnCoordinator.standardRateDeflection, kDeg);
        a.num("calibration_bank_deg", p.turnCoordinator.calibrationBank, kDeg);
        a.num("max_deflection_deg", p.turnCoordinator.maxDeflection, kDeg);
        a.num("spin_up_time_constant_s", p.turnCoordinator.spinUpTime);
        a.num("spin_down_time_constant_s", p.turnCoordinator.spinDownTime);
        readNeedle(a, "gimbal", p.turnCoordinator.gimbal);
    }
    if (o.has("inclinometer")) {
        Obj a = o.child("inclinometer");
        a.num("tube_limit_deg", p.inclinometer.tubeLimit, kDeg);
        readNeedle(a, "ball", p.inclinometer.ball);
    }
    if (o.has("pitot_probe")) {
        Obj a = o.child("pitot_probe");
        a.num("frontal_area_m2", p.pitotProbe.frontalArea);
        a.num("collection_efficiency", p.pitotProbe.collectionEfficiency);
        a.num("ice_to_block_inlet_kg", p.pitotProbe.iceToBlockInlet);
        a.num("ice_to_block_drain_kg", p.pitotProbe.iceToBlockDrain);
        a.num("heater_power_w", p.pitotProbe.heaterPower);
        a.num("heater_efficiency", p.pitotProbe.heaterEfficiency);
        a.num("heater_time_constant_s", p.pitotProbe.heaterTimeConstant);
        a.num("recovery_factor", p.pitotProbe.recoveryFactor);
    }
    if (o.has("magnetic_compass")) {
        Obj a = o.child("magnetic_compass");
        a.boolean("installed", p.compass.installed);
        a.num("tilt_limit_deg", p.compass.tiltLimit, kDeg);
        a.num("vertical_time_constant_s", p.compass.verticalTimeConstant);
        readNeedle(a, "card", p.compass.card);
        a.table("deviation", "heading_deg", "deviation_deg", p.compass.deviation, kDeg, kDeg);
        a.vec3("hard_iron_body", p.compass.hardIronBody);
    }
    if (o.has("engine_gauges")) {
        Obj a = o.child("engine_gauges");
        EngineGaugesDef& e = p.engine;
        readRange(a, "tachometer_green_rpm", e.tachometerGreen, 1);
        a.num("tachometer_red_line_rpm", e.tachometerRedLine);
        a.num("tachometer_max_rpm", e.tachometerMax);
        readRange(a, "oil_pressure_green_psi", e.oilPressureGreen, kPsi);
        a.num("oil_pressure_red_low_psi", e.oilPressureRedLow, kPsi);
        a.num("oil_pressure_red_high_psi", e.oilPressureRedHigh, kPsi);
        readRangeF(a, "oil_temperature_green_f", e.oilTemperatureGreen);
        Real redF = 0;
        a.num("oil_temperature_red_line_f", redF);
        if (redF > 0) e.oilTemperatureRedLine = fToK(redF);
        readRange(a, "suction_green_inhg", e.suctionGreen, units::kInHg);
        a.boolean("egt", e.exhaustGasTemperature);
        a.boolean("cht", e.cylinderHeadTemperature);
        a.boolean("fuel_flow", e.fuelFlow);
        a.boolean("ammeter", e.ammeter);
        a.boolean("voltmeter", e.voltmeter);
        a.str("electrical_load", e.gaugeLoad);
        a.num("fuel_sender_time_constant_s", e.fuelSenderTimeConstant);
        readNeedle(a, "needle", e.needle);
    }
    if (o.has("annunciators")) {
        Obj a = o.child("annunciators");
        a.boolean("installed", p.annunciators.installed);
        a.num("oil_pressure_below_psi", p.annunciators.oilPressureBelow, kPsi);
        a.num("vacuum_below_inhg", p.annunciators.vacuumBelow, units::kInHg);
        a.num("volts_below", p.annunciators.voltsBelow);
        a.num("low_fuel_below_gal", p.annunciators.lowFuelBelow, units::kUsGallon);
    }
    if (o.has("outside_air_temperature")) {
        Obj a = o.child("outside_air_temperature");
        a.boolean("installed", p.oat.installed);
        a.num("recovery_factor", p.oat.recoveryFactor);
        a.num("time_constant_s", p.oat.timeConstant);
    }
    if (o.has("avionics")) {
        Obj a = o.child("avionics");
        AvionicsDef& v = p.avionics;
        a.integer("nav_com_radios", v.navComRadios);
        a.boolean("glideslope", v.glideslope);
        a.boolean("transponder", v.transponder);
        a.boolean("altitude_encoder", v.altitudeEncoder);
        a.boolean("gps", v.gps);
        a.boolean("adf", v.adf);
        a.boolean("dme", v.dme);
        a.boolean("autopilot", v.autopilot);
        a.str("electrical_load", v.avionicsLoad);
        a.num("com_spacing_khz", v.comSpacingKhz);
    }
}

void readDefinition(Obj& r, AircraftDefinition& d) {
    r.integer("schema_version", d.schemaVersion, true);
    r.str("id", d.id, true);
    r.str("display_name", d.displayName);
    r.str("manufacturer", d.manufacturer);
    r.str("reference_aircraft", d.referenceAircraft);
    r.str("description", d.description);
    r.each("sources", [&](Obj& o, std::size_t) {
        SourceRef s;
        o.str("id", s.id, true);
        o.str("citation", s.citation, true);
        d.sources.push_back(s);
    });
    r.each("provenance", [&](Obj& o, std::size_t) {
        ProvenanceNote n;
        std::string kind;
        o.str("field", n.field, true);
        o.str("kind", kind, true);
        o.str("source", n.source);
        o.str("note", n.note);
        n.kind = provenanceFrom(kind, o);
        d.provenance.push_back(n);
    });

    {
        Obj m = r.child("mass", true);
        m.num("empty_kg", d.emptyMass, 1, true);
        m.vec3("empty_cg_m", d.emptyCg, 1, true);
        Obj in = m.child("empty_inertia_kgm2", true);
        in.num("ixx", d.emptyInertiaDiag.x, 1, true);
        in.num("iyy", d.emptyInertiaDiag.y, 1, true);
        in.num("izz", d.emptyInertiaDiag.z, 1, true);
        in.num("ixz", d.emptyIxz);
        m.boolean("empty_includes_unusable_fuel", d.emptyIncludesUnusableFuel);
        m.num("max_ramp_kg", d.maxRampMass);
        m.num("max_takeoff_kg", d.maxTakeoffMass, 1, true);
        m.num("max_landing_kg", d.maxLandingMass);
        m.num("max_zero_fuel_kg", d.maxZeroFuelMass);
        m.each("stations", [&](Obj& o, std::size_t) {
            PayloadStationDef s;
            o.str("name", s.name, true);
            o.vec3("position_m", s.position, 1, true);
            o.num("max_kg", s.maxMass, 1, true);
            o.num("initial_kg", s.initial);
            d.stations.push_back(s);
        });
        m.each("combined_limits", [&](Obj& o, std::size_t) {
            StationGroupLimitDef g;
            std::vector<Real> st;
            o.str("name", g.name, true);
            o.numbers("stations", st, 1, true);
            for (Real s : st) g.stations.push_back(static_cast<int>(s));
            o.num("max_kg", g.maxMass, 1, true);
            d.stationGroupLimits.push_back(g);
        });
        m.table("cg_forward_limit", "mass_kg", "x_m", d.cgForwardLimitVsMass);
        m.table("cg_aft_limit", "mass_kg", "x_m", d.cgAftLimitVsMass);
    }
    {
        Obj ref = r.child("reference", true);
        ref.num("area_m2", d.referenceArea, 1, true);
        ref.num("span_m", d.referenceSpan, 1, true);
        ref.num("chord_m", d.referenceChord, 1, true);
    }
    r.each("surfaces", [&](Obj& o, std::size_t) {
        LiftingSurfaceDef s;
        o.str("name", s.name, true);
        {
            Obj sec = o.child("section", true);
            readSection(sec, s.section);
        }
        o.num("span_efficiency", s.spanEfficiency);
        if (o.has("downwash")) {
            Obj dw = o.child("downwash");
            dw.integer("source", s.downwashSource, true);
            dw.num("gradient", s.downwashGradient);
            dw.num("at_zero_lift_deg", s.downwashAtZeroLift, kDeg);
        }
        if (o.has("slipstream")) {
            Obj ss = o.child("slipstream");
            ss.integer("engine", s.slipstreamEngine);
            ss.num("exposure", s.slipstreamExposure, 1, true);
        }
        o.boolean("ground_effect", s.groundEffect);
        o.boolean("lifting_line", s.liftingLine);
        o.boolean("trimmable_stabilizer", s.trimmableStabilizer);
        o.num("thickness_ratio", s.thicknessRatio);
        o.num("korn_factor", s.kornFactor);
        o.each("panels", [&](Obj& po, std::size_t) {
            PanelDef p;
            po.vec3("root_quarter_chord_m", p.rootQuarterChord, 1, true);
            po.num("span_m", p.span, 1, true);
            po.num("root_chord_m", p.rootChord, 1, true);
            po.num("tip_chord_m", p.tipChord, 1, true);
            po.num("sweep_deg", p.sweep, kDeg);
            po.num("dihedral_deg", p.dihedral, kDeg);
            po.num("root_incidence_deg", p.rootIncidence, kDeg);
            po.num("tip_incidence_deg", p.tipIncidence, kDeg);
            po.boolean("mirrored", p.mirrored);
            po.boolean("vertical", p.vertical);
            po.integer("strips", p.strips);
            po.each("controls", [&](Obj& co, std::size_t) {
                ControlSurfaceDef c;
                std::string ch;
                co.str("name", c.name, true);
                co.str("channel", ch, true);
                c.channel = channelFrom(ch, co);
                co.num("span_start", c.spanStart);
                co.num("span_end", c.spanEnd);
                co.num("chord_fraction", c.chordFraction, 1, true);
                co.num("max_positive_deg", c.maxPositive, kDeg, true);
                co.num("max_negative_deg", c.maxNegative, kDeg, true);
                co.boolean("antisymmetric", c.antisymmetric);
                co.table("effectiveness", "deflection_deg", "eta", c.effectivenessVsDeflection, kDeg);
                co.num("clmax_increment_ratio", c.clMaxIncrementRatio);
                co.num("profile_drag_per_rad2", c.profileDragPerRad2);
                co.num("chord_extension", c.chordExtension);
                p.controls.push_back(c);
            });
            s.panels.push_back(p);
        }, true);
        d.surfaces.push_back(s);
    }, true);
    r.each("fuselages", [&](Obj& o, std::size_t) {
        FuselageDef f;
        o.str("name", f.name);
        o.vec3("reference_point_m", f.referencePoint, 1, true);
        o.num("volume_m3", f.volume, 1, true);
        o.num("munk_factor", f.munkFactor);
        o.num("crossflow_area_m2", f.crossflowArea);
        o.num("crossflow_cd", f.crossflowCd);
        d.fuselages.push_back(f);
    });
    r.each("drag_items", [&](Obj& o, std::size_t) {
        DragItemDef di;
        o.str("name", di.name, true);
        o.vec3("position_m", di.position, 1, true);
        o.num("drag_area_m2", di.dragArea, 1, true);
        o.num("critical_mach", di.criticalMach);
        if (o.has("body")) {
            Obj b = o.child("body");
            b.vec3("axis", di.bodyAxis, 1, true);
            b.num("length_m", di.bodyLength, 1, true);
            b.num("size_m", di.bodySize, 1, true);
        }
        d.dragItems.push_back(di);
    });
    r.each("engines", [&](Obj& o, std::size_t) {
        PistonEngineDef e;
        std::string type = "piston";
        o.str("name", e.name);
        o.str("type", type);
        if (type != "piston") o.err(o.path() + ".type", "only \"piston\" is supported in schema v1");
        o.vec3("position_m", e.position, 1, true);
        o.vec3("thrust_axis", e.thrustAxis);
        o.boolean("clockwise_from_behind", e.clockwiseFromBehind);
        o.num("rated_power_kw", e.ratedPower, 1000, true);
        o.num("rated_rpm", e.ratedRpm, 1, true);
        o.num("displacement_l", e.displacement, 1e-3, true);
        o.num("volumetric_efficiency", e.volumetricEfficiency);
        o.num("friction_mep_rated_kpa", e.frictionMepAtRated, 1000);
        o.num("friction_mep_zero_kpa", e.frictionMepAtZero, 1000);
        o.num("idle_manifold_ratio", e.idleManifoldRatio);
        o.num("intake_loss_ratio", e.intakeLossRatio);
        o.num("engine_inertia_kgm2", e.engineInertia);
        o.num("starter_torque_nm", e.starterTorque);
        o.num("starter_current_a", e.starterCurrent);
        o.num("minimum_firing_rpm", e.minimumFiringRpm);
        o.num("single_magneto_power_factor", e.singleMagnetoPowerFactor);
        o.num("mixture_rich_phi", e.mixtureRichPhi);
        o.num("fuel_heating_value_mj_per_kg", e.fuelHeatingValue, 1e6);
        o.num("stoichiometric_far", e.stoichiometricFar);
        o.table("power_vs_phi", "phi", "relative_power", e.powerVsPhi);
        o.num("p_factor_gain", e.pFactorGain);
        o.num("slipstream_swirl_factor", e.slipstreamSwirlFactor);
        o.num("alternator_min_rpm", e.alternatorMinRpm);
        o.num("oil_pump_psi_per_rpm", e.oilPumpPressurePerRpm, 6894.757);
        o.num("oil_relief_psi", e.oilReliefPressure, 6894.757);
        o.num("oil_viscosity_temperature_scale_k", e.oilViscosityTemperatureScale);
        {
            Obj pr = o.child("propeller", true);
            std::string ptype = "fixed_pitch";
            pr.str("type", ptype);
            if (ptype != "fixed_pitch") pr.err(pr.path() + ".type", "only \"fixed_pitch\" is supported in schema v1");
            pr.num("diameter_m", e.propeller.diameter, 1, true);
            pr.integer("blades", e.propeller.blades, true);
            pr.num("pitch_m", e.propeller.pitch, 1, true);
            pr.num("hub_radius_fraction", e.propeller.hubRadiusFraction);
            pr.table("chord", "r_over_r", "chord_m", e.propeller.chordVsRadius);
            if (pr.has("thickness")) pr.table("thickness", "r_over_r", "t_over_c", e.propeller.thicknessVsRadius);
            pr.num("korn_factor", e.propeller.kornFactor);
            if (pr.has("section")) {
                Obj sec = pr.child("section");
                readSection(sec, e.propeller.section);
            }
            pr.num("inertia_kgm2", e.propeller.inertia, 1, true);
            pr.integer("bem_elements", e.propeller.bemElements);
        }
        d.engines.push_back(e);
    });
    r.each("turbofans", [&](Obj& o, std::size_t) {
        TurbofanDef e;
        o.str("name", e.name);
        o.vec3("position_m", e.position, 1, true);
        o.vec3("thrust_axis", e.thrustAxis);
        o.num("static_thrust_kn", e.staticThrust, 1000, true);
        o.num("bypass_ratio", e.bypassRatio);
        o.num("throttle_ratio", e.throttleRatio);
        o.num("lapse_mach_coefficient", e.lapseMachCoefficient);
        o.num("idle_n1", e.idleN1);
        o.num("idle_n2", e.idleN2);
        o.num("idle_thrust_fraction", e.idleThrustFraction);
        o.num("thrust_exponent", e.thrustExponent);
        o.num("acceleration_time_s", e.accelerationTime);
        o.num("deceleration_time_s", e.decelerationTime);
        o.num("start_time_s", e.startTime);
        // lb/(lbf h) -> kg/(N s)
        o.num("tsfc_static_lb_per_lbf_h", e.tsfcStatic, 0.45359237 / (4.4482216 * 3600));
        o.num("tsfc_mach_slope", e.tsfcMachSlope);
        o.num("idle_fuel_flow_kg_per_h", e.idleFuelFlow, 1.0 / 3600);
        o.num("itt_idle_k", e.ittIdle);
        o.num("itt_takeoff_k", e.ittTakeoff);
        o.num("oil_pressure_idle_psi", e.oilPressureIdle, 6894.757);
        o.num("oil_pressure_max_psi", e.oilPressureMax, 6894.757);
        o.integer("fuel_feed", e.fuelFeed);
        if (o.has("thrust_reverser")) {
            Obj rv = o.child("thrust_reverser");
            e.reverser.present = true;
            rv.num("effectiveness", e.reverser.effectiveness);
            rv.num("transit_time_s", e.reverser.transitTime);
            rv.num("reverse_n1", e.reverser.reverseN1);
            rv.num("stow_kias", e.reverser.stowIas, units::kKnot);
            rv.num("cascade_forward_of_nozzle_m", e.reverser.cascadeForward);
            rv.num("cowl_radius_m", e.reverser.cowlRadius);
        }
        d.turbofans.push_back(e);
    });
    if (r.has("fuel")) {
        Obj f = r.child("fuel");
        Real densityKgL = d.fuel.density / 1000;
        f.num("density_kg_per_l", densityKgL);
        d.fuel.density = densityKgL * 1000;
        f.num("line_volume_l", d.fuel.lineVolume, 1e-3);
        f.boolean("has_shutoff_valve", d.fuel.hasShutoffValve);
        f.each("tanks", [&](Obj& o, std::size_t) {
            FuelTankDef t;
            o.str("name", t.name, true);
            o.vec3("position_m", t.position, 1, true);
            o.num("capacity_l", t.capacity, 1e-3, true);
            o.num("unusable_l", t.unusable, 1e-3);
            o.num("initial_l", t.initial, 1e-3);
            d.fuel.tanks.push_back(t);
        });
        f.each("selector", [&](Obj& o, std::size_t) {
            FuelSelectorPosition p;
            std::vector<Real> tanks;
            o.str("name", p.name, true);
            o.numbers("tanks", tanks, 1, true);
            for (Real t : tanks) p.tanks.push_back(static_cast<int>(t));
            d.fuel.selector.push_back(p);
        });
        f.integer("initial_selector", d.fuel.initialSelector);
    }
    if (r.has("electrical")) {
        Obj e = r.child("electrical");
        e.num("battery_voltage_v", d.electrical.batteryVoltage);
        Real ah = d.electrical.batteryCapacity / 3600;
        e.num("battery_capacity_ah", ah);
        d.electrical.batteryCapacity = ah * 3600;
        e.num("battery_resistance_ohm", d.electrical.batteryResistance);
        e.num("alternator_voltage_v", d.electrical.alternatorVoltage);
        e.num("alternator_max_current_a", d.electrical.alternatorMaxCurrent);
        e.each("loads", [&](Obj& o, std::size_t) {
            ElectricalLoadDef l;
            o.str("name", l.name, true);
            o.str("label", l.label);
            o.boolean("initially_on", l.initiallyOn);
            o.num("current_a", l.current, 1, true);
            o.num("minimum_voltage_v", l.minimumVoltage);
            o.boolean("avionics_bus", l.onAvionicsBus);
            d.electrical.loads.push_back(l);
        });
    }
    r.each("gear", [&](Obj& o, std::size_t) {
        GearDef g;
        std::string brake = "none";
        o.str("name", g.name, true);
        o.vec3("attach_m", g.attach, 1, true);
        o.num("length_m", g.length, 1, true);
        o.num("stiffness_n_per_m", g.stiffness, 1, true);
        o.num("preload_n", g.preload);
        o.num("damping_compression_ns_per_m", g.dampingCompression);
        o.num("damping_rebound_ns_per_m", g.dampingRebound);
        o.num("max_compression_m", g.maxCompression);
        o.num("wheel_radius_m", g.wheelRadius);
        o.num("rolling_resistance", g.rollingResistance);
        o.num("brake_coefficient", g.maxBrakeForceCoefficient);
        o.str("brake_group", brake);
        if (brake == "left") g.brakeGroup = 0;
        else if (brake == "right") g.brakeGroup = 1;
        else if (brake == "none") g.brakeGroup = -1;
        else o.err(o.path() + ".brake_group", "must be left, right or none");
        o.num("max_steer_deg", g.maxSteerAngle, kDeg);
        o.num("tiller_steer_deg", g.tillerSteerAngle, kDeg);
        o.num("steer_rate_deg_per_s", g.steerRate, kDeg);
        o.num("cornering_stiffness_n_per_rad", g.corneringStiffness);
        o.num("relaxation_length_m", g.relaxationLength);
        o.num("longitudinal_stiffness_n_per_m", g.longitudinalStiffness);
        o.num("tyre_pressure_psi", g.tyrePressure, 6894.757);
        d.gear.push_back(g);
    });
    r.each("contacts", [&](Obj& o, std::size_t) {
        ContactPointDef c;
        o.str("name", c.name, true);
        o.vec3("position_m", c.position, 1, true);
        o.num("stiffness_n_per_m", c.stiffness);
        o.num("damping_ns_per_m", c.damping);
        o.num("friction", c.friction);
        d.contacts.push_back(c);
    });
    if (r.has("controls")) {
        Obj c = r.child("controls");
        c.numbers("flap_detents_deg", d.controls.flapDetents, kDeg);
        c.num("flap_rate_deg_per_s", d.controls.flapRate, kDeg);
        c.boolean("flaps_electric", d.controls.flapsElectric);
        c.num("flap_motor_current_a", d.controls.flapMotorCurrent);
        c.num("elevator_trim_range_deg", d.controls.elevatorTrimRange, kDeg);
        c.num("stabilizer_trim_nose_up_deg", d.controls.stabilizerNoseUp, kDeg);
        c.num("stabilizer_trim_nose_down_deg", d.controls.stabilizerNoseDown, kDeg);
        c.num("elevator_trim_rate", d.controls.elevatorTrimRate);
        c.num("rigging_aileron", d.controls.riggingAileron);
        c.num("rigging_rudder", d.controls.riggingRudder);
        c.num("rudder_limiter_kias", d.controls.rudderLimiterIas, units::kKnot);
        if (c.has("landing_gear")) {
            Obj g = c.child("landing_gear");
            g.boolean("retractable", d.controls.gear.retractable);
            g.num("transit_time_s", d.controls.gear.transitTime);
            g.num("drag_area_m2", d.controls.gear.dragArea);
            g.num("vle_kias", d.controls.gear.maxExtendedIas, units::kKnot);
        }
        if (c.has("speedbrake")) {
            Obj b = c.child("speedbrake");
            d.controls.speedbrake.present = true;
            b.integer("detents", d.controls.speedbrake.detents);
            b.num("drag_area_m2", d.controls.speedbrake.dragArea);
            b.num("lift_loss_fraction", d.controls.speedbrake.liftLossFraction);
            b.num("rate_per_s", d.controls.speedbrake.rate);
            b.boolean("ground_spoilers", d.controls.speedbrake.groundSpoilers);
            b.num("ground_arm_kt", d.controls.speedbrake.groundArmSpeed, units::kKnot);
            b.num("ground_stow_kt", d.controls.speedbrake.groundStowSpeed, units::kKnot);
            b.num("ground_rate_per_s", d.controls.speedbrake.groundRate);
        }
    }
    if (r.has("pitot_static")) {
        Obj p = r.child("pitot_static");
        p.vec3("pitot_position_m", d.pitotStatic.pitotPosition);
        p.vec3("static_position_m", d.pitotStatic.staticPosition);
        p.table("static_error", "alpha_deg", "dp_over_q", d.pitotStatic.staticErrorVsAlpha, kDeg);
        p.num("vsi_time_constant_s", d.pitotStatic.vsiTimeConstant);
    }
    if (r.has("stall_warning")) {
        Obj s = r.child("stall_warning");
        s.boolean("present", d.stallWarning.present);
        s.integer("surface", d.stallWarning.surface);
        s.integer("strip", d.stallWarning.strip);
        s.boolean("computed", d.stallWarning.computed);
        s.num("alpha_margin_deg", d.stallWarning.alphaMargin, kDeg);
        s.num("pusher_margin_deg", d.stallWarning.pusherMargin, kDeg);
        s.num("pusher_elevator", d.stallWarning.pusherElevator);
    } else {
        d.stallWarning.present = false;
    }
    if (r.has("vacuum")) {
        Obj v = r.child("vacuum");
        v.boolean("present", d.vacuum.present);
        v.num("regulated_suction_inhg", d.vacuum.regulatedSuction, units::kInHg);
        v.num("rpm_for_regulated", d.vacuum.rpmForRegulated);
        v.num("gyro_spin_up_s", d.vacuum.gyroSpinUpTime);
        v.num("gyro_spin_down_s", d.vacuum.gyroSpinDownTime);
    } else {
        d.vacuum.present = false;
    }
    if (r.has("panel")) {
        Obj pn = r.child("panel");
        readPanel(pn, d.panel);
    }
    if (r.has("limits")) {
        Obj l = r.child("limits");
        l.num("vne_kias", d.limits.neverExceedIas, units::kKnot);
        l.num("vno_kias", d.limits.maxStructuralCruiseIas, units::kKnot);
        l.num("va_kias", d.limits.maneuveringIas, units::kKnot);
        l.num("vfe_kias", d.limits.flapExtendedIas, units::kKnot);
        l.table("vfe_vs_flap", "flap_deg", "kias", d.limits.flapExtendedIasVsFlap, kDeg, units::kKnot);
        l.num("load_positive_g", d.limits.limitLoadPositive);
        l.num("load_negative_g", d.limits.limitLoadNegative);
        l.num("load_flaps_g", d.limits.limitLoadFlaps);
        l.num("mmo", d.limits.maxOperatingMach);
    }
}

}  // namespace

LoadResult loadAircraftFromJson(const std::string& text) {
    LoadResult res;
    const json::ParseResult pr = json::parse(text);
    if (!pr.ok()) {
        res.errors.push_back("JSON: " + pr.error);
        return res;
    }
    {
        Obj root(&pr.value, "", res.errors);
        readDefinition(root, res.definition);
    }
    if (res.errors.empty()) res.errors = res.definition.validate();
    return res;
}

LoadResult loadAircraftFromFile(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        LoadResult r;
        r.errors.push_back(path + ": cannot open file");
        return r;
    }
    std::ostringstream ss;
    ss << f.rdbuf();
    return loadAircraftFromJson(ss.str());
}

}  // namespace avd
