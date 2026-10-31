// Aviation Dynamics — scenarios.
// SPDX-License-Identifier: MIT
#include "airports.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iterator>

#include "avd/units.hpp"

namespace avd::scenarios {

namespace {
Geodetic deg(Real lat, Real lon, Real h = 0) { return {units::degToRad(lat), units::degToRad(lon), h}; }
}  // namespace

LocalFrame saoPauloWorldFrame() { return LocalFrame(deg(-23.5611, -46.6558, 0.0)); }

Vec3 saoPauloMapNed(Real latDeg, Real lonDeg) {
    Real e = 0, n = 0;
    wgs84::transverseMercator(deg(latDeg, lonDeg), units::degToRad(-45.0), 0.9996, e, n);
    return {n + 10000000.0 - kMapOriginNorth, e + 500000.0 - kMapOriginEast, 0.0};
}

namespace {

// Where a coordinate lies in `f`: on the map's projection in the São Paulo
// world (so runways, buildings and the map agree), on the tangent plane in
// any other frame.
Vec3 placeOn(const LocalFrame& f, Real latDeg, Real lonDeg) {
    const Geodetic o = saoPauloWorldFrame().origin();
    if (std::abs(f.origin().latitude - o.latitude) < 1e-12 && std::abs(f.origin().longitude - o.longitude) < 1e-12)
        return saoPauloMapNed(latDeg, lonDeg);
    Vec3 p = f.nedFromGeodetic(deg(latDeg, lonDeg, 0));
    p.z = 0;
    return p;
}

}  // namespace

Airport campoDeMarte(const LocalFrame& frame) {
    Airport a;
    a.icao = "SBMT";
    a.name = "Campo de Marte";
    a.elevation = 722.0;
    a.magneticVariation = units::degToRad(-21.5);  // ~21-22 deg W in 2026 (estimated)
    const Vec3 thr12 = placeOn(frame, -23.508333, -46.6425);
    const Vec3 thr30 = placeOn(frame, -23.510333, -46.631333);
    const Vec3 d = thr30 - thr12;
    const Real heading = std::atan2(d.y, d.x);
    const Vec3 dir{std::cos(heading), std::sin(heading), 0};
    RunwayDef r12;
    r12.ident = "12";
    r12.headingTrue = heading;
    r12.length = 1600;
    r12.width = 45;
    // Paved end of runway 12: 300 m before its displaced threshold.
    r12.threshold = thr12 - dir * 300.0;
    r12.threshold.z = -a.elevation;
    r12.displaced = 300;
    r12.surface = surfaces::kAsphalt;
    a.runways.push_back(r12);
    RunwayDef r30 = r12;
    r30.ident = "30";
    r30.headingTrue = wrapPi(heading + kPi);
    r30.threshold = r12.threshold + dir * r12.length;
    r30.threshold.z = -a.elevation;
    r30.displaced = 150;
    a.runways.push_back(r30);
    a.ilsKhz = 110300;
    a.ils.push_back({0, 110300, "ITMT", true, "synthetic test facility (Aviation Dynamics); SBMT has no ILS"});
    a.source = "SkyVector threshold coordinates (see airports.hpp)";
    return a;
}

Airport congonhas(const LocalFrame& frame) {
    Airport a;
    a.icao = "SBSP";
    a.name = "Congonhas";
    a.elevation = 802.0;
    a.magneticVariation = units::degToRad(-21.5);
    // The surveyed thresholds of 17R/35L. On the map's projection they lie
    // within 2 m of OpenStreetMap's centreline and on the imagery. (Placed on
    // a tangent plane at MASP they were ~140 m off the map, which had been
    // taken for an imagery offset and corrected by moving the runway.)
    const Vec3 thr35 = placeOn(frame, -23.6343, -46.6512);
    const Vec3 thr17 = placeOn(frame, -23.6210, -46.6604);
    const Vec3 d = thr17 - thr35;
    const Real heading = std::atan2(d.y, d.x);
    const Vec3 dir{std::cos(heading), std::sin(heading), 0};
    RunwayDef r35;
    r35.ident = "35L";
    r35.headingTrue = heading;
    r35.length = 1940;
    r35.width = 45;
    // Paved end of 35L: 61 m before its displaced threshold.
    r35.threshold = thr35 - dir * 61.0;
    r35.threshold.z = -a.elevation;
    r35.displaced = 61;
    r35.surface = surfaces::kAsphalt;
    a.runways.push_back(r35);
    RunwayDef r17 = r35;
    r17.ident = "17R";
    r17.headingTrue = wrapPi(heading + kPi);
    r17.threshold = r35.threshold + dir * r35.length;
    r17.threshold.z = -a.elevation;
    r17.displaced = 126;
    a.runways.push_back(r17);
    // On the runway as modelled (aligned with the map imagery).
    const char* src = "OurAirports airport-frequencies.csv (public domain), also Airportroutes and Metarcentral listings";
    a.ilsKhz = 109700;
    a.ils.push_back({0, 109700, "ISO", false, src});
    a.ils.push_back({1, 109300, "ISP", false, src});
    a.source = "SkyVector threshold coordinates (see airports.hpp)";
    return a;
}

namespace {

constexpr Real kFt = 0.3048;
constexpr const char* kOurAirports = "OurAirports runways.csv and airports.csv (public domain, retrieved October 2026)";

Airport fieldAt(const char* icao, const char* name, Real elevationFt) {
    Airport a;
    a.icao = icao;
    a.name = name;
    a.elevation = elevationFt * kFt;
    a.magneticVariation = units::degToRad(-21.5);  // regional estimate, as above
    a.source = kOurAirports;
    return a;
}

// A runway from OurAirports' records: the two ends' coordinates give its
// centre and direction, the published length (ft) its extent (the end
// coordinates are not always the paved ends: Guarulhos' are ~150 m short);
// displaced thresholds and width in ft; flat at the field elevation.
void addRunway(Airport& a, const LocalFrame& f, const char* id1, Real lat1, Real lon1, Real disp1Ft, const char* id2,
               Real lat2, Real lon2, Real disp2Ft, Real lengthFt, Real widthFt) {
    const Vec3 c1 = placeOn(f, lat1, lon1), c2 = placeOn(f, lat2, lon2);
    const Real heading = std::atan2(c2.y - c1.y, c2.x - c1.x), half = 0.5 * lengthFt * kFt;
    const Vec3 mid = (c1 + c2) * 0.5, dir{std::cos(heading), std::sin(heading), 0};
    Vec3 e1 = mid - dir * half, e2 = mid + dir * half;
    e1.z = e2.z = -a.elevation;
    RunwayDef r1;
    r1.ident = id1;
    r1.headingTrue = heading;
    r1.length = 2 * half;
    r1.width = widthFt * kFt;
    r1.threshold = e1;
    r1.displaced = disp1Ft * kFt;
    RunwayDef r2 = r1;
    r2.ident = id2;
    r2.headingTrue = wrapPi(r1.headingTrue + kPi);
    r2.threshold = e2;
    r2.displaced = disp2Ft * kFt;
    a.runways.push_back(r1);
    a.runways.push_back(r2);
}

}  // namespace

const std::vector<Airport>& saoPauloRegionAirports() {
    static const std::vector<Airport> list = [] {
        const LocalFrame f = saoPauloWorldFrame();
        std::vector<Airport> v{congonhas(f), campoDeMarte(f)};
        // Guarulhos: runways renumbered 09/27 -> 10/28 in September 2022.
        Airport gr = fieldAt("SBGR", "Guarulhos", 2461);
        addRunway(gr, f, "10L", -23.434059, -46.482548, 0, "28R", -23.425043, -46.449203, 0, 12139, 148);
        addRunway(gr, f, "10R", -23.438801, -46.487, 0, "28L", -23.4312, -46.4589, 0, 9843, 148);
        const char* grSrc = "OurAirports airport-frequencies.csv (public domain); DECEA chart ILS RWY 10L (AMDT 2209A1) for IUC";
        gr.ils = {{0, 110700, "IUC", false, grSrc}, {1, 111900, "IGS", false, grSrc},
                  {2, 111700, "IGH", false, grSrc}, {3, 111100, "IBC", false, grSrc}};
        gr.ilsKhz = 110700;
        v.push_back(gr);
        Airport kp = fieldAt("SBKP", "Viracopos", 2170);
        addRunway(kp, f, "15", -22.9985, -47.147, 0, "33", -23.0164, -47.122, 300, 10630, 148);
        kp.ils = {{0, 110300, "IKP", false, "Flight Plan Database listing for SBKP (single secondary source; unverified)"}};
        kp.ilsKhz = 110300;
        v.push_back(kp);
        Airport sj = fieldAt("SBSJ", "São José dos Campos", 2120);
        addRunway(sj, f, "16", -23.2197, -45.872, 0, "34", -23.2368, -45.8535, 0, 8780, 148);
        v.push_back(sj);
        Airport jd = fieldAt("SBJD", "Jundiaí", 2470);
        addRunway(jd, f, "18", -23.17495, -46.94665, 0, "36", -23.18658, -46.94127, 0, 4593, 98);
        v.push_back(jd);
        return v;
    }();
    return list;
}

Vec3 landingThreshold(const RunwayDef& r) { return runwayPoint(r, r.displaced); }

std::shared_ptr<const ElevationGrid> saoPauloTerrain() {
#ifndef AVD_DATA_DIR
    return nullptr;  // the browser build: the host hands the file over (avd_terrain)
#else
    static const std::shared_ptr<const ElevationGrid> grid = []() -> std::shared_ptr<const ElevationGrid> {
        std::ifstream f(std::string(AVD_DATA_DIR) + "/world/terrain_sp.avt", std::ios::binary);
        if (!f) return nullptr;
        const std::vector<unsigned char> bytes((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
        auto g = std::make_shared<ElevationGrid>();
        if (!g->parse(bytes.data(), bytes.size())) return nullptr;
        return g;
    }();
    return grid;
#endif
}

std::unique_ptr<RegionGround> makeRegionGround(const std::vector<Airport>& airports) {
    auto g = std::make_unique<RegionGround>();
    for (const Airport& a : airports) {
        RegionGround::Field fld;
        fld.down = -a.elevation;
        Vec3 c;
        int n = 0;
        for (std::size_t i = 0; i < a.runways.size(); i += 2) {
            fld.runways.push_back(a.runways[i]);  // one paved strip serves both ends
            c = c + runwayPoint(a.runways[i], 0.5 * a.runways[i].length);
            ++n;
        }
        if (n == 0) continue;
        c = c * (1.0 / n);
        fld.north = c.x;
        fld.east = c.y;
        Real r = 0;
        for (const RunwayDef& rw : a.runways) r = std::max(r, std::hypot(rw.threshold.x - c.x, rw.threshold.y - c.y));
        fld.radius = r + 1500;
        g->addField(fld);
    }
    return g;
}

std::vector<RouteWaypoint> approachRoute(const Airport& dest, int runway, const Vec3& from, int& faf,
                                         const RunwayDef* departure) {
    constexpr Real kNm = 1852, kGp = 0.0524, kTch = 15.24;
    const RunwayDef& rw = dest.runways[static_cast<std::size_t>(runway)];
    const Vec3 thr = landingThreshold(rw);
    const Real c = std::cos(rw.headingTrue), s = std::sin(rw.headingTrue);
    auto onFinal = [&](Real nm) { return Vec3{thr.x - c * nm * kNm, thr.y - s * nm * kNm, thr.z}; };
    auto pathAlt = [&](Real nm) { return dest.elevation + kTch + nm * kNm * std::tan(kGp); };
    std::vector<RouteWaypoint> r;
    Vec3 start = from;
    if (departure) {
        const Vec3 d0 = landingThreshold(*departure);
        const Real dc = std::cos(departure->headingTrue), ds = std::sin(departure->headingTrue);
        r.push_back({"RW" + departure->ident, d0, -1, true});
        start = {d0.x + dc * (departure->length - departure->displaced + 3 * kNm), d0.y + ds * (departure->length - departure->displaced + 3 * kNm), d0.z};
        r.push_back({"DP" + departure->ident, start, -1, false});
    }
    const Vec3 itf = onFinal(10);
    // Arriving from well off the final course: an initial fix 5 NM abeam the
    // intermediate fix, on the arrival's side. Not when already on the final
    // (within 1 NM of the extended centreline, inside the IF).
    const Real out = -((start.x - thr.x) * c + (start.y - thr.y) * s), lateral = -(start.x - thr.x) * s + (start.y - thr.y) * c;
    const bool onFinalAlready = std::abs(lateral) < kNm && out > -kNm && out < 10.5 * kNm;
    const Real inbound = std::atan2(itf.y - start.y, itf.x - start.x);
    if (!onFinalAlready && std::abs(wrapPi(inbound - rw.headingTrue)) > units::degToRad(60)) {
        const Real side = (-(start.x - thr.x) * s + (start.y - thr.y) * c) >= 0 ? 1.0 : -1.0;  // + right of the final
        r.push_back({"IA" + rw.ident, {itf.x - s * side * 5 * kNm, itf.y + c * side * 5 * kNm, thr.z}, pathAlt(10), false});
    }
    r.push_back({"IF" + rw.ident, itf, pathAlt(10), false});
    faf = static_cast<int>(r.size());
    r.push_back({"FF" + rw.ident, onFinal(5), pathAlt(5), false});
    r.push_back({"RW" + rw.ident, thr, dest.elevation + kTch, true});
    return r;
}

std::unique_ptr<AirfieldGround> makeAirfieldGround(const Airport& a, const LocalFrame&) {
    auto g = std::make_unique<AirfieldGround>(-a.elevation, surfaces::kGrass);
    // One paved strip covers both directions.
    if (!a.runways.empty()) g->addRunway(a.runways[0]);
    return g;
}

void addIlsNavaids(NavDatabase& d, const Airport& ap, const LocalFrame& f) {
    auto geo = [&](const Vec3& ned, Real h) {
        Geodetic g = f.geodeticFromNed(ned);
        g.height = h;
        return g;
    };
    for (const IlsDef& i : ap.ils) {
        if (i.runway < 0 || i.runway >= static_cast<int>(ap.runways.size())) continue;
        const RunwayDef& rw = ap.runways[static_cast<std::size_t>(i.runway)];
        Navaid loc;
        loc.type = Navaid::Type::Localizer;
        loc.ident = i.ident;
        loc.name = ap.icao + " ILS " + rw.ident + (i.synthetic ? " (synthetic)" : "");
        loc.frequencyKhz = i.khz;
        loc.position = geo(runwayPoint(rw, rw.length + 300), ap.elevation + 2);
        // The course in the antenna's own tangent frame: far from the world
        // origin its north is turned by the meridians' convergence (0.2 deg at
        // Viracopos), which would offset the beam.
        const Vec3 along = f.rotationTo(LocalFrame(loc.position)) * Vec3{std::cos(rw.headingTrue), std::sin(rw.headingTrue), 0};
        loc.course = std::atan2(along.y, along.x);
        // Width giving 700 ft (213 m) full-scale span at the landing threshold.
        loc.courseWidth = 2 * std::atan2(106.7, rw.length + 300 - rw.displaced);
        loc.rangeNm = 18;
        loc.synthetic = i.synthetic;
        loc.source = i.source;
        d.add(loc);
        Navaid gs = loc;
        gs.type = Navaid::Type::Glideslope;
        gs.ident = i.ident + " GS";
        gs.position = geo(runwayPoint(rw, rw.displaced + 300, 120), ap.elevation + 2);
        gs.glidepathAngle = units::degToRad(3.0);
        gs.rangeNm = 10;
        d.add(gs);
    }
}

Airport airportFromRecord(const json::Value& rec, const LocalFrame& f) {
    auto str = [&](const char* k) { const json::Value* v = rec.find(k); return v && v->isString() ? v->asString() : std::string(); };
    auto number = [](const json::Value* v, Real def) { return v && v->isNumber() ? v->asNumber() : def; };
    Airport a = fieldAt("", "", number(rec.find("elev"), 0));
    a.icao = str("icao");
    a.name = str("city").empty() ? str("name") : str("city");
    a.source = "OurAirports (public domain) via tools/navdata/brazil.py";
    const Real lat0 = number(rec.find("lat"), 0), lon0 = number(rec.find("lon"), 0);
    const json::Value* rws = rec.find("rw");
    if (!rws || !rws->isArray()) return a;
    for (const json::Value& r : rws->items()) {
        const json::Value* ends = r.find("ends");
        if (!ends || !ends->isArray() || ends->items().size() != 2) continue;
        const std::string id1 = ends->items()[0].asString(), id2 = ends->items()[1].asString();
        const Real len = number(r.find("len"), 0), wid = number(r.find("wid"), 98);
        if (len <= 0) continue;
        Real d1 = 0, d2 = 0;
        if (const json::Value* dp = r.find("disp"); dp && dp->isArray() && dp->items().size() == 2) {
            d1 = number(&dp->items()[0], 0);
            d2 = number(&dp->items()[1], 0);
        }
        // The runway's number (magnetic heading / 10, with the regional
        // variation) is the arbiter: OurAirports' listed true headings are
        // often magnetic or wrong, and a few end coordinates are misplaced.
        const Real fromNumber = std::atof(id1.c_str()) * 10 + units::radToDeg(a.magneticVariation);
        auto agrees = [&](Real hdgDeg) { return std::abs(units::radToDeg(wrapPi(units::degToRad(hdgDeg - fromNumber)))) < 20; };
        const json::Value *le = r.find("le"), *he = r.find("he");
        if (le && he && le->isArray() && he->isArray() && le->items().size() == 2 && he->items().size() == 2) {
            const Vec3 p1 = placeOn(f, le->items()[0].asNumber(), le->items()[1].asNumber());
            const Vec3 p2 = placeOn(f, he->items()[0].asNumber(), he->items()[1].asNumber());
            if (agrees(units::radToDeg(std::atan2(p2.y - p1.y, p2.x - p1.x)))) {
                addRunway(a, f, id1.c_str(), le->items()[0].asNumber(), le->items()[1].asNumber(), d1, id2.c_str(),
                          he->items()[0].asNumber(), he->items()[1].asNumber(), d2, len, wid);
                continue;
            }
        }
        // No (usable) end coordinates: centred on the reference point, on the
        // listed true heading if it agrees with the number, else the number's;
        // parallels 400 m apart, L to the left.
        const json::Value* listed = r.find("hdg");
        const Real hdg = listed && listed->isNumber() && agrees(listed->asNumber()) ? listed->asNumber() : fromNumber;
        const Real h = units::degToRad(hdg);
        Vec3 c = placeOn(f, lat0, lon0);
        const Real side = id1.find('L') != std::string::npos ? -200.0 : (id1.find('R') != std::string::npos ? 200.0 : 0.0);
        c = c + Vec3{-std::sin(h), std::cos(h), 0} * side;
        const Real half = 0.5 * len * kFt, mPerDegLat = 111320.0, mPerDegLon = 111320.0 * std::cos(units::degToRad(lat0));
        // Back to coordinates for addRunway (it reprojects them): a local
        // flat-Earth offset is exact enough over a runway length.
        const Vec3 e0 = placeOn(f, lat0, lon0);
        const Real dn = c.x - e0.x, de = c.y - e0.y;
        const Real latc = lat0 + dn / mPerDegLat, lonc = lon0 + de / mPerDegLon;
        const Real dLat = std::cos(h) * half / mPerDegLat, dLon = std::sin(h) * half / mPerDegLon;
        addRunway(a, f, id1.c_str(), latc - dLat, lonc - dLon, d1, id2.c_str(), latc + dLat, lonc + dLon, d2, len, wid);
    }
    if (const json::Value* ils = rec.find("ils"); ils && ils->isArray()) {
        for (const json::Value& i : ils->items()) {
            const json::Value* rwy = i.find("rwy");
            if (!rwy || !rwy->isString()) continue;
            for (std::size_t k = 0; k < a.runways.size(); ++k) {
                if (a.runways[k].ident != rwy->asString()) continue;
                IlsDef d;
                d.runway = static_cast<int>(k);
                d.khz = static_cast<int>(std::lround(number(i.find("mhz"), 0) * 1000));
                if (const json::Value* id = i.find("id"); id && id->isString()) d.ident = id->asString();
                if (const json::Value* src = i.find("src"); src && src->isString()) d.source = src->asString();
                if (d.khz >= 108100 && d.khz <= 111950) a.ils.push_back(d);
            }
        }
    }
    if (!a.ils.empty())
        for (const IlsDef& i : a.ils)
            if (i.runway == 0) a.ilsKhz = i.khz;
    return a;
}

const NavEnvironment& saoPauloNavEnvironment() {
    static NavDatabase db = [] {
        NavDatabase d;
        const LocalFrame f = saoPauloWorldFrame();
        const Real var = units::degToRad(-21.5);
        const std::string src = "synthetic test facility (Aviation Dynamics); not a real-world navaid";
        Navaid vor;
        vor.type = Navaid::Type::Vor;
        vor.ident = "TSP";
        vor.name = "Test VOR Paulista (synthetic)";
        vor.frequencyKhz = 113700;
        vor.position = f.origin();
        vor.position.height = 830;
        vor.magneticVariation = var;
        vor.rangeNm = 40;
        vor.synthetic = true;
        vor.source = src;
        d.add(vor);
        for (const Airport& ap : saoPauloRegionAirports()) addIlsNavaids(d, ap, f);
        return d;
    }();
    static NavEnvironment env = [] {
        NavEnvironment e;
        e.field.declination = units::degToRad(-21.5);
        e.field.inclination = units::degToRad(-38.0);
        e.database = &db;
        return e;
    }();
    return env;
}

Vec3 runwayPoint(const RunwayDef& r, Real along, Real across) {
    const Real c = std::cos(r.headingTrue), s = std::sin(r.headingTrue);
    return {r.threshold.x + along * c - across * s, r.threshold.y + along * s + across * c,
            r.threshold.z - r.slope * along};
}

void addSaoPauloWindEnvironment(SimpleWeatherParams& p, const Airport& a) {
    p.groundElevation = a.elevation;
    p.profile = WindProfile::LogLaw;
    p.surfaceRoughness = 0.03;  // the report: the airfield anemometer
    p.veer = units::degToRad(-20);
    if (a.runways.empty()) return;
    const RunwayDef& rw = a.runways[0];
    const Vec3 field = runwayPoint(rw, 0.5 * rw.length);
    p.roughnessPatches.push_back({field.x, field.y, 30000, 3000, 1.0});  // the city
    p.roughnessPatches.push_back({field.x, field.y, 1200, 400, 0.03});   // the airfield inside it
    Ridge cantareira;
    cantareira.north = field.x + 10000;
    cantareira.east = field.y;
    cantareira.axis = units::degToRad(80);
    cantareira.length = 25000;
    cantareira.height = 350;
    cantareira.halfWidth = 2500;
    cantareira.baseElevation = a.elevation + 30;
    p.ridges.push_back(cantareira);
}

}  // namespace avd::scenarios
