// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
//
// References: FAA Instrument Flying Handbook (FAA-H-8083-15B) ch. 9 (VOR: 10 deg
// full-scale CDI, TO/FROM; ILS: localizer ~5 deg course width, glideslope
// 1.4 deg thick, i.e. +-0.7 deg full scale); FAA AIM 1-1-8 (standard service
// volumes); radio horizon d = 1.23 (sqrt(h1) + sqrt(h2)) NM, h in feet.
#include "avd/navigation.hpp"

#include "avd/json.hpp"
#include "avd/units.hpp"

namespace avd {

Vec3 MagneticField::directionNed() const {
    const Real ci = std::cos(inclination);
    return {ci * std::cos(declination), ci * std::sin(declination), std::sin(inclination)};
}

Real radioHorizonNm(Real a, Real b) {
    return 1.23 * (std::sqrt(std::max(units::mToFt(a), 0.0)) + std::sqrt(std::max(units::mToFt(b), 0.0)));
}

namespace {

constexpr Real kVorFullScale = 10.0 * kPi / 180;
constexpr Real kGsFullScale = 0.7 * kPi / 180;

bool inRange(const Navaid& n, const Vec3& rel, Real aircraftHeight) {
    const Real horiz = std::hypot(rel.x, rel.y);
    const Real los = radioHorizonNm(std::max(aircraftHeight - n.position.height, 0.0) + 10.0, 10.0);
    return horiz <= std::min(los, n.rangeNm) * units::kNauticalMile;
}

}  // namespace

NavIndication evaluateNav(const NavDatabase& db, Real freqKhz, Real obs, const Geodetic& ac) {
    NavIndication out;
    const Navaid* vorLoc = nullptr;
    const Navaid* gs = nullptr;
    Real bestRange = 1e18;
    for (const Navaid& n : db.navaids()) {
        if (std::abs(n.frequencyKhz - freqKhz) > 1.0) continue;
        const Vec3 rel = LocalFrame(n.position).nedFromGeodetic(ac);
        if (!inRange(n, rel, ac.height)) continue;
        const Real r = length(rel);
        if (n.type == Navaid::Type::Glideslope) {
            if (!gs || r < bestRange) gs = &n;
        } else if (!vorLoc || r < bestRange) {
            vorLoc = &n;
            bestRange = r;
        }
    }
    if (!vorLoc) return out;
    const Vec3 rel = LocalFrame(vorLoc->position).nedFromGeodetic(ac);
    const Real horiz = std::hypot(rel.x, rel.y);
    out.ident = vorLoc->ident;
    out.slantRange = length(rel);
    if (vorLoc->type == Navaid::Type::Vor) {
        // Radial: magnetic bearing from the station to the aircraft.
        const Real radialTrue = std::atan2(rel.y, rel.x);
        out.radialMagnetic = wrapPi(radialTrue - vorLoc->magneticVariation);
        const Real dev = wrapPi(out.radialMagnetic - obs);
        if (std::abs(dev) <= kPi / 2) {
            out.toFrom = -1;
            out.cdi = clamp(-dev / kVorFullScale, -1.0, 1.0);
        } else {
            out.toFrom = +1;
            out.cdi = clamp(wrapPi(dev - kPi) / kVorFullScale, -1.0, 1.0);
        }
        // Abeam (within ~10 deg of the TO/FROM boundary) and the cone of
        // confusion above the station: flag shown.
        const Real abeam = std::abs(std::abs(dev) - kPi / 2);
        const Real elevation = std::atan2(-rel.z, horiz);
        out.valid = abeam > 0.05 && elevation < 0.87;  // < ~50 deg above the horizon
        if (!out.valid) out.toFrom = 0;
        return out;
    }
    // Localizer: angle of the aircraft off the inbound course, seen from the
    // antenna at the far end of the runway (front course only).
    out.localizer = true;
    const Real bearing = std::atan2(rel.y, rel.x);  // antenna -> aircraft (true)
    const Real off = wrapPi(bearing - (vorLoc->course + kPi));
    out.valid = std::abs(off) < 0.61;  // within 35 deg of the course (service volume)
    out.cdi = out.valid ? clamp(off / (0.5 * vorLoc->courseWidth), -1.0, 1.0) : 0.0;
    out.toFrom = 0;
    if (gs && out.valid) {
        const Vec3 g = LocalFrame(gs->position).nedFromGeodetic(ac);
        const Real gh = std::hypot(g.x, g.y);
        // Coverage (ICAO Annex 10): 8 deg either side of the localizer
        // centreline, to 10 NM. Measured about the localizer course, not from
        // the glideslope antenna, which stands ~120 m off the centreline: from
        // there an aircraft close in and slightly off course looked outside it.
        if (std::abs(off) < 0.14 && gh < 10 * units::kNauticalMile && gh > 50) {
            const Real angle = std::atan2(-g.z, gh);
            out.glideslopeValid = true;
            out.glideslope = clamp((gs->glidepathAngle - angle) / kGsFullScale, -1.0, 1.0);
        }
    }
    return out;
}

std::vector<std::string> NavDatabase::loadJson(const std::string& text) {
    std::vector<std::string> err;
    const json::ParseResult r = json::parse(text);
    if (!r.ok()) return {"JSON: " + r.error};
    const json::Value* list = r.value.find("navaids");
    if (!list || !list->isArray()) return {"navaids: array required"};
    for (std::size_t i = 0; i < list->items().size(); ++i) {
        const json::Value& o = list->items()[i];
        const std::string p = "navaids[" + std::to_string(i) + "]";
        auto num = [&](const char* k, Real def, bool required = false) {
            const json::Value* v = o.find(k);
            if (!v) {
                if (required) err.push_back(p + "." + k + ": required");
                return def;
            }
            if (!v->isNumber()) {
                err.push_back(p + "." + k + ": must be a number");
                return def;
            }
            return v->asNumber();
        };
        auto str = [&](const char* k) {
            const json::Value* v = o.find(k);
            return v && v->isString() ? v->asString() : std::string();
        };
        Navaid n;
        const std::string type = str("type");
        if (type == "vor") n.type = Navaid::Type::Vor;
        else if (type == "localizer") n.type = Navaid::Type::Localizer;
        else if (type == "glideslope") n.type = Navaid::Type::Glideslope;
        else { err.push_back(p + ".type: must be vor, localizer or glideslope"); continue; }
        n.ident = str("ident");
        n.name = str("name");
        n.source = str("source");
        n.frequencyKhz = num("frequency_mhz", 0, true) * 1000;
        n.position = {units::degToRad(num("latitude_deg", 0, true)), units::degToRad(num("longitude_deg", 0, true)),
                      num("elevation_m", 0)};
        n.magneticVariation = units::degToRad(num("magnetic_variation_deg", 0));
        n.course = units::degToRad(num("course_true_deg", 0));
        n.courseWidth = units::degToRad(num("course_width_deg", 5));
        n.glidepathAngle = units::degToRad(num("glidepath_deg", 3));
        n.rangeNm = num("range_nm", 40);
        const json::Value* s = o.find("synthetic");
        n.synthetic = s && s->isBool() && s->asBool();
        if (n.source.empty()) err.push_back(p + ".source: required (provenance of the facility record)");
        if (!(n.frequencyKhz >= 108000 && n.frequencyKhz <= 117950)) err.push_back(p + ".frequency_mhz: must be 108.00-117.95");
        navaids_.push_back(n);
    }
    return err;
}

}  // namespace avd
