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
        m.num("max_takeoff_kg", d.maxTakeoffMass, 1, true);
        m.num("max_landing_kg", d.maxLandingMass);
        m.each("stations", [&](Obj& o, std::size_t) {
            PayloadStationDef s;
            o.str("name", s.name, true);
            o.vec3("position_m", s.position, 1, true);
            o.num("max_kg", s.maxMass, 1, true);
            o.num("initial_kg", s.initial);
            d.stations.push_back(s);
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
            if (pr.has("section")) {
                Obj sec = pr.child("section");
                readSection(sec, e.propeller.section);
            }
            pr.num("inertia_kgm2", e.propeller.inertia, 1, true);
            pr.integer("bem_elements", e.propeller.bemElements);
        }
        d.engines.push_back(e);
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
        o.num("cornering_stiffness_n_per_rad", g.corneringStiffness);
        o.num("relaxation_length_m", g.relaxationLength);
        o.num("longitudinal_stiffness_n_per_m", g.longitudinalStiffness);
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
        c.num("elevator_trim_rate", d.controls.elevatorTrimRate);
        c.num("rigging_aileron", d.controls.riggingAileron);
        c.num("rigging_rudder", d.controls.riggingRudder);
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
        s.num("alpha_margin_deg", d.stallWarning.alphaMargin, kDeg);
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
