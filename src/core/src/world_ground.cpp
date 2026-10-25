// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
#include "avd/world_ground.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

#include "avd/json.hpp"
#include "avd/world_frame.hpp"

namespace avd {

namespace {

using Pt = std::pair<Real, Real>;

bool insidePolygon(const std::vector<Pt>& poly, Real x, Real y) {
    bool in = false;
    for (std::size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
        const Real xi = poly[i].first, yi = poly[i].second, xj = poly[j].first, yj = poly[j].second;
        if ((yi > y) != (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi) in = !in;
    }
    return in;
}

// Nearest point on the polygon boundary to (x, y): distance and that point.
Real nearestEdge(const std::vector<Pt>& poly, Real x, Real y, Real& cx, Real& cy) {
    Real best = std::numeric_limits<Real>::max();
    for (std::size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
        const Real ax = poly[j].first, ay = poly[j].second, bx = poly[i].first, by = poly[i].second;
        const Real ex = bx - ax, ey = by - ay, l2 = ex * ex + ey * ey;
        const Real t = l2 > 0 ? std::clamp(((x - ax) * ex + (y - ay) * ey) / l2, 0.0, 1.0) : 0.0;
        const Real px = ax + t * ex, py = ay + t * ey, d = std::hypot(x - px, y - py);
        if (d < best) {
            best = d;
            cx = px;
            cy = py;
        }
    }
    return best;
}

GroundHit hitFromEnu(Real x, Real y, Real z, const Vec3& normalEnu, Real distance, const SurfaceProperties& s) {
    GroundHit h;
    h.hit = true;
    h.distance = distance;
    h.point = world::nedFromEnu({x, y, z});
    h.normal = world::nedFromEnu(normalEnu);
    h.surface = s;
    return h;
}

}  // namespace

bool surfaceFromName(const std::string& name, SurfaceProperties& out) {
    if (name == "asphalt") out = surfaces::kAsphalt;
    else if (name == "concrete") out = surfaces::kConcrete;
    else if (name == "grass") out = surfaces::kGrass;
    else if (name == "dirt") out = surfaces::kDirt;
    else if (name == "water") out = surfaces::kWater;
    else if (name == "building") out = surfaces::kBuilding;
    else return false;
    return true;
}

void TiledWorldGround::addTile(WorldTile tile) {
    Tile t;
    t.data = std::move(tile);
    const WorldTile& d = t.data;
    t.x0 = d.i * d.size - 0.5 * d.size;
    t.y0 = d.j * d.size - 0.5 * d.size;
    const Real cell = d.size / Tile::kBuckets;
    for (std::size_t b = 0; b < d.buildings.size(); ++b) {
        Real minX = 1e300, minY = 1e300, maxX = -1e300, maxY = -1e300;
        for (const Pt& p : d.buildings[b].footprint) {
            minX = std::min(minX, p.first);
            maxX = std::max(maxX, p.first);
            minY = std::min(minY, p.second);
            maxY = std::max(maxY, p.second);
        }
        t.bbox.insert(t.bbox.end(), {minX, minY, maxX, maxY});
        const int i0 = std::clamp(static_cast<int>(std::floor((minX - t.x0) / cell)), 0, Tile::kBuckets - 1);
        const int i1 = std::clamp(static_cast<int>(std::floor((maxX - t.x0) / cell)), 0, Tile::kBuckets - 1);
        const int j0 = std::clamp(static_cast<int>(std::floor((minY - t.y0) / cell)), 0, Tile::kBuckets - 1);
        const int j1 = std::clamp(static_cast<int>(std::floor((maxY - t.y0) / cell)), 0, Tile::kBuckets - 1);
        for (int j = j0; j <= j1; ++j)
            for (int i = i0; i <= i1; ++i) t.buckets[j * Tile::kBuckets + i].push_back(static_cast<int>(b));
    }
    const std::pair<int, int> key{d.i, d.j};
    tiles_[key] = std::move(t);
}

const TiledWorldGround::Tile* TiledWorldGround::tileAt(Real east, Real north) const {
    if (tiles_.empty()) return nullptr;
    const Real size = tiles_.begin()->second.data.size;
    const auto it = tiles_.find({world::tileIndex(east, size), world::tileIndex(north, size)});
    return it == tiles_.end() ? nullptr : &it->second;
}

bool TiledWorldGround::terrainAt(Real east, Real north, Real& z, Vec3& normalEnu) const {
    const Tile* t = tileAt(east, north);
    if (!t) {
        z = outsideHeight_;
        normalEnu = {0, 0, 1};
        return false;
    }
    const WorldTile& d = t->data;
    const Real cell = d.size / (d.n - 1);
    const Real u = std::clamp((east - t->x0) / cell, 0.0, Real(d.n - 1));
    const Real v = std::clamp((north - t->y0) / cell, 0.0, Real(d.n - 1));
    const int i = std::min(static_cast<int>(u), d.n - 2), j = std::min(static_cast<int>(v), d.n - 2);
    const Real fu = u - i, fv = v - j;
    const auto H = [&](int a, int b) { return d.height[static_cast<std::size_t>(b * d.n + a)]; };
    const Real h00 = H(i, j), h10 = H(i + 1, j), h01 = H(i, j + 1), h11 = H(i + 1, j + 1);
    z = (h00 * (1 - fu) + h10 * fu) * (1 - fv) + (h01 * (1 - fu) + h11 * fu) * fv;
    const Real dzdx = ((h10 - h00) * (1 - fv) + (h11 - h01) * fv) / cell;
    const Real dzdy = ((h01 - h00) * (1 - fu) + (h11 - h10) * fu) / cell;
    normalEnu = normalized(Vec3{-dzdx, -dzdy, 1});
    return true;
}

Real TiledWorldGround::terrainHeight(Real east, Real north) const {
    Real z;
    Vec3 n;
    terrainAt(east, north, z, n);
    return z;
}

SurfaceProperties TiledWorldGround::surfaceAt(Real east, Real north) const {
    const Tile* t = tileAt(east, north);
    if (!t) return outside_;
    const WorldTile& d = t->data;
    if (d.surfaceN <= 0) return d.defaultSurface;
    const Real cell = d.size / d.surfaceN;
    const int i = std::clamp(static_cast<int>((east - t->x0) / cell), 0, d.surfaceN - 1);
    const int j = std::clamp(static_cast<int>((north - t->y0) / cell), 0, d.surfaceN - 1);
    return d.surface[static_cast<std::size_t>(j * d.surfaceN + i)];
}

template <class F>
void TiledWorldGround::forBuildingsAt(const Tile& t, Real east, Real north, F&& f) const {
    const Real cell = t.data.size / Tile::kBuckets;
    const int i = std::clamp(static_cast<int>(std::floor((east - t.x0) / cell)), 0, Tile::kBuckets - 1);
    const int j = std::clamp(static_cast<int>(std::floor((north - t.y0) / cell)), 0, Tile::kBuckets - 1);
    for (int b : t.buckets[j * Tile::kBuckets + i]) {
        const Real* bb = &t.bbox[static_cast<std::size_t>(4 * b)];
        if (east < bb[0] || east > bb[2] || north < bb[1] || north > bb[3]) continue;
        const WorldBuilding& w = t.data.buildings[static_cast<std::size_t>(b)];
        if (insidePolygon(w.footprint, east, north)) f(w);
    }
}

GroundHit TiledWorldGround::below(const Vec3& point) const {
    const Vec3 p = world::enuFromNed(point);
    Real z;
    Vec3 n;
    terrainAt(p.x, p.y, z, n);
    SurfaceProperties s = surfaceAt(p.x, p.y);
    if (const Tile* t = tileAt(p.x, p.y)) {
        forBuildingsAt(*t, p.x, p.y, [&](const WorldBuilding& w) {
            if (w.top <= p.z + 0.5 && w.top > z) {
                z = w.top;
                n = {0, 0, 1};
                s = surfaces::kBuilding;
            }
        });
    }
    return hitFromEnu(p.x, p.y, z, n, p.z - z, s);
}

bool TiledWorldGround::solidAt(const Vec3& point, GroundHit& contact) const {
    const Vec3 p = world::enuFromNed(point);
    const Tile* t = tileAt(p.x, p.y);
    if (!t) return false;
    bool found = false;
    forBuildingsAt(*t, p.x, p.y, [&](const WorldBuilding& w) {
        if (found || p.z >= w.top || p.z < w.base - 0.5) return;
        Real cx = 0, cy = 0;
        const Real wall = nearestEdge(w.footprint, p.x, p.y, cx, cy);
        const Real roof = w.top - p.z;
        if (roof <= wall) {
            contact = hitFromEnu(p.x, p.y, w.top, {0, 0, 1}, roof, surfaces::kBuilding);
        } else {
            const Real dx = cx - p.x, dy = cy - p.y, l = std::max(std::hypot(dx, dy), 1e-9);
            contact = hitFromEnu(cx, cy, p.z, {dx / l, dy / l, 0}, wall, surfaces::kBuilding);
        }
        found = true;
    });
    return found;
}

GroundHit TiledWorldGround::raycast(const Vec3& originNed, const Vec3& dirNed, Real maxDistance) const {
    const Vec3 o = world::enuFromNed(originNed), d = world::enuFromNed(dirNed);
    GroundHit best;
    Real bestT = maxDistance;
    // Terrain: march with a step that cannot jump over slopes up to 1:1, then
    // bisect the crossing.
    {
        const Real horiz = std::hypot(d.x, d.y);
        auto f = [&](Real t) { return o.z + d.z * t - terrainHeight(o.x + d.x * t, o.y + d.y * t); };
        Real t0 = 0, f0 = f(0);
        if (f0 >= 0) {
            for (int it = 0; it < 20000 && t0 < bestT; ++it) {
                const Real step = std::max(0.5, f0 / (std::max(-d.z, 0.0) + horiz + 1e-6));
                const Real t1 = std::min(t0 + step, bestT);
                const Real f1 = f(t1);
                if (f1 < 0) {
                    Real a = t0, b = t1;
                    for (int k = 0; k < 40; ++k) {
                        const Real m = 0.5 * (a + b);
                        (f(m) >= 0 ? a : b) = m;
                    }
                    const Real x = o.x + d.x * b, y = o.y + d.y * b;
                    Real z;
                    Vec3 n;
                    terrainAt(x, y, z, n);
                    best = hitFromEnu(x, y, z, n, b, surfaceAt(x, y));
                    bestT = b;
                    break;
                }
                if (t1 >= bestT) break;
                t0 = t1;
                f0 = f1;
            }
        }
    }
    // Buildings: exact ray/prism intersection (roof and walls).
    const Real ex = o.x + d.x * bestT, ey = o.y + d.y * bestT;
    const Real sMinX = std::min(o.x, ex), sMaxX = std::max(o.x, ex), sMinY = std::min(o.y, ey), sMaxY = std::max(o.y, ey);
    for (const auto& kv : tiles_) {
        const Tile& t = kv.second;
        const Real size = t.data.size;
        if (sMaxX < t.x0 || sMinX > t.x0 + size || sMaxY < t.y0 || sMinY > t.y0 + size) continue;
        for (std::size_t b = 0; b < t.data.buildings.size(); ++b) {
            const Real* bb = &t.bbox[4 * b];
            if (sMaxX < bb[0] || sMinX > bb[2] || sMaxY < bb[1] || sMinY > bb[3]) continue;
            const WorldBuilding& w = t.data.buildings[b];
            if (std::abs(d.z) > 1e-12) {
                const Real tr = (w.top - o.z) / d.z;
                if (tr >= 0 && tr < bestT && insidePolygon(w.footprint, o.x + d.x * tr, o.y + d.y * tr)) {
                    best = hitFromEnu(o.x + d.x * tr, o.y + d.y * tr, w.top, {0, 0, 1}, tr, surfaces::kBuilding);
                    bestT = tr;
                }
            }
            const auto& poly = w.footprint;
            for (std::size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
                const Real ax = poly[j].first, ay = poly[j].second, wx = poly[i].first - ax, wy = poly[i].second - ay;
                const Real den = d.x * wy - d.y * wx;
                if (std::abs(den) < 1e-12) continue;
                const Real tw = ((ax - o.x) * wy - (ay - o.y) * wx) / den;
                const Real u = ((ax - o.x) * d.y - (ay - o.y) * d.x) / den;
                if (tw < 0 || tw >= bestT || u < 0 || u > 1) continue;
                const Real z = o.z + d.z * tw;
                if (z < w.base || z > w.top) continue;
                Vec3 n = normalized(Vec3{wy, -wx, 0});
                if (n.x * d.x + n.y * d.y > 0) n = n * -1.0;  // face the ray
                best = hitFromEnu(o.x + d.x * tw, o.y + d.y * tw, z, n, tw, surfaces::kBuilding);
                bestT = tw;
            }
        }
    }
    if (!best.hit) {
        // Beyond the loaded tiles: the outside plane.
        if (d.z < -1e-12 && !tileAt(o.x, o.y)) {
            const Real tp = (outsideHeight_ - o.z) / d.z;
            if (tp >= 0 && tp <= maxDistance)
                best = hitFromEnu(o.x + d.x * tp, o.y + d.y * tp, outsideHeight_, {0, 0, 1}, tp, outside_);
        }
    }
    return best;
}

std::vector<std::string> TiledWorldGround::loadTileJson(const std::string& text) {
    std::vector<std::string> err;
    const json::ParseResult r = json::parse(text);
    if (!r.ok()) return {"JSON: " + r.error};
    const json::Value& o = r.value;
    auto str = [&](const json::Value& v, const char* k) {
        const json::Value* x = v.find(k);
        return x && x->isString() ? x->asString() : std::string();
    };
    auto num = [&](const json::Value& v, const std::string& path, const char* k, Real def, bool required) {
        const json::Value* x = v.find(k);
        if (!x) {
            if (required) err.push_back(path + k + ": required");
            return def;
        }
        if (!x->isNumber() || !std::isfinite(x->asNumber())) {
            err.push_back(path + k + ": must be a finite number");
            return def;
        }
        return Real(x->asNumber());
    };
    if (str(o, "format") != "avd-world-tile/1") return {"format: must be \"avd-world-tile/1\""};
    WorldTile t;
    t.name = str(o, "tile");
    t.source = str(o, "source");
    if (t.source.empty()) err.push_back("source: required (provenance of the tile data)");
    if (const json::Value* s = o.find("synthetic")) t.synthetic = s->isBool() && s->asBool();
    t.i = static_cast<int>(num(o, "", "i", 0, true));
    t.j = static_cast<int>(num(o, "", "j", 0, true));
    t.size = num(o, "", "size_m", 500, false);
    if (t.size <= 0) err.push_back("size_m: must be positive");
    if (!tiles_.empty() && std::abs(tiles_.begin()->second.data.size - t.size) > 1e-9)
        err.push_back("size_m: all tiles must share one size");
    const json::Value* terr = o.find("terrain");
    if (!terr || !terr->isObject()) {
        err.push_back("terrain: object required");
    } else {
        t.n = static_cast<int>(num(*terr, "terrain.", "n", 2, true));
        const json::Value* z = terr->find("z_m");
        if (t.n < 2) err.push_back("terrain.n: must be >= 2");
        else if (!z || !z->isArray() || z->items().size() != static_cast<std::size_t>(t.n * t.n))
            err.push_back("terrain.z_m: array of n*n heights required");
        else
            for (const json::Value& v : z->items()) {
                if (!v.isNumber() || !std::isfinite(v.asNumber())) {
                    err.push_back("terrain.z_m: heights must be finite numbers");
                    break;
                }
                t.height.push_back(v.asNumber());
            }
    }
    if (const std::string def = str(o, "default_surface"); !def.empty() && !surfaceFromName(def, t.defaultSurface))
        err.push_back("default_surface: unknown class '" + def + "'");
    if (const json::Value* s = o.find("surfaces")) {
        t.surfaceN = static_cast<int>(num(*s, "surfaces.", "n", 0, true));
        const json::Value* legend = s->find("legend");
        const json::Value* cells = s->find("cells");
        std::vector<SurfaceProperties> classes;
        if (legend && legend->isArray())
            for (const json::Value& v : legend->items()) {
                SurfaceProperties sp;
                if (!v.isString() || !surfaceFromName(v.asString(), sp)) err.push_back("surfaces.legend: unknown class");
                classes.push_back(sp);
            }
        if (t.surfaceN <= 0 || !cells || !cells->isArray() ||
            cells->items().size() != static_cast<std::size_t>(t.surfaceN * t.surfaceN))
            err.push_back("surfaces.cells: array of n*n legend indices required");
        else
            for (const json::Value& v : cells->items()) {
                const int k = v.isNumber() ? static_cast<int>(v.asNumber()) : -1;
                if (k < 0 || k >= static_cast<int>(classes.size())) {
                    err.push_back("surfaces.cells: index outside the legend");
                    break;
                }
                t.surface.push_back(classes[static_cast<std::size_t>(k)]);
            }
    }
    if (const json::Value* bl = o.find("buildings")) {
        if (!bl->isArray()) err.push_back("buildings: array required");
        else
            for (std::size_t bi = 0; bi < bl->items().size(); ++bi) {
                const json::Value& b = bl->items()[bi];
                const std::string path = "buildings[" + std::to_string(bi) + "].";
                WorldBuilding w;
                w.id = str(b, "id");
                const json::Value* fp = b.find("footprint");
                if (fp && fp->isArray())
                    for (const json::Value& v : fp->items())
                        if (v.isArray() && v.items().size() == 2 && v.items()[0].isNumber() && v.items()[1].isNumber())
                            w.footprint.emplace_back(v.items()[0].asNumber(), v.items()[1].asNumber());
                if (w.footprint.size() < 3) {
                    err.push_back(path + "footprint: at least 3 [east, north] points required");
                    continue;
                }
                const bool hasBase = b.find("base_m") != nullptr;
                w.base = num(b, path, "base_m", 0, false);
                if (!hasBase && err.empty() && !t.height.empty()) {
                    // Walls start at the lowest terrain under the footprint.
                    TiledWorldGround probe;
                    WorldTile only = t;
                    only.buildings.clear();
                    probe.addTile(only);
                    w.base = 1e300;
                    for (const Pt& p : w.footprint) w.base = std::min(w.base, probe.terrainHeight(p.first, p.second));
                }
                if (b.find("top_m")) w.top = num(b, path, "top_m", 0, true);
                else w.top = w.base + num(b, path, "height_m", 0, true);
                if (!(w.top > w.base)) err.push_back(path + "top must be above base");
                t.buildings.push_back(std::move(w));
            }
    }
    if (err.empty()) addTile(std::move(t));
    return err;
}

// ------------------------------------------------------------ BuildingLayer ----

std::int64_t BuildingLayer::cellKey(Real east, Real north) const {
    const auto i = static_cast<std::int64_t>(std::floor(east / cell_)), j = static_cast<std::int64_t>(std::floor(north / cell_));
    return (i << 32) ^ (j & 0xffffffff);
}

template <class F>
void BuildingLayer::forCells(const Entry& e, F&& f) const {
    for (Real y = std::floor(e.minY / cell_) * cell_; y <= e.maxY; y += cell_)
        for (Real x = std::floor(e.minX / cell_) * cell_; x <= e.maxX; x += cell_) f(cellKey(x + 0.5 * cell_, y + 0.5 * cell_));
}

// Groups are indexed (and unindexed) on their own: a host streams tiles of
// thousands of buildings while many others stay loaded.
void BuildingLayer::add(std::int64_t key, std::vector<WorldBuilding> buildings) {
    remove(key);
    const auto& list = groups_[key] = std::move(buildings);
    for (const WorldBuilding& b : list) {
        Entry e{&b, 1e300, 1e300, -1e300, -1e300, key};
        for (const Pt& p : b.footprint) {
            e.minX = std::min(e.minX, p.first); e.minY = std::min(e.minY, p.second);
            e.maxX = std::max(e.maxX, p.first); e.maxY = std::max(e.maxY, p.second);
        }
        forCells(e, [&](std::int64_t c) { grid_[c].push_back(e); });
    }
    count_ += list.size();
}

void BuildingLayer::remove(std::int64_t key) {
    const auto it = groups_.find(key);
    if (it == groups_.end()) return;
    for (const WorldBuilding& b : it->second) {
        Entry e{&b, 1e300, 1e300, -1e300, -1e300, key};
        for (const Pt& p : b.footprint) {
            e.minX = std::min(e.minX, p.first); e.minY = std::min(e.minY, p.second);
            e.maxX = std::max(e.maxX, p.first); e.maxY = std::max(e.maxY, p.second);
        }
        forCells(e, [&](std::int64_t c) {
            const auto cell = grid_.find(c);
            if (cell == grid_.end()) return;
            auto& v = cell->second;
            v.erase(std::remove_if(v.begin(), v.end(), [&](const Entry& x) { return x.group == key; }), v.end());
            if (v.empty()) grid_.erase(cell);
        });
    }
    count_ -= it->second.size();
    groups_.erase(it);
}

void BuildingLayer::clear() {
    groups_.clear();
    grid_.clear();
    count_ = 0;
}

template <class F>
void BuildingLayer::forBuildingsAt(Real east, Real north, F&& f) const {
    const auto it = grid_.find(cellKey(east, north));
    if (it == grid_.end()) return;
    for (const Entry& e : it->second)
        if (east >= e.minX && east <= e.maxX && north >= e.minY && north <= e.maxY && insidePolygon(e.b->footprint, east, north)) f(*e.b);
}

bool BuildingLayer::parseAvb1(const std::uint8_t* data, std::size_t size, Real x0, Real y0, std::vector<WorldBuilding>& out,
                              Real sink) const {
    std::size_t p = 0;
    auto take = [&](void* dst, std::size_t n) {
        if (p + n > size) return false;
        std::memcpy(dst, data + p, n);  // little-endian on every target we build for
        p += n;
        return true;
    };
    char magic[4];
    std::uint32_t count = 0;
    if (!take(magic, 4) || std::memcmp(magic, "AVB1", 4) != 0 || !take(&count, 4)) return false;
    out.reserve(out.size() + count);
    for (std::uint32_t k = 0; k < count; ++k) {
        std::uint16_t h = 0, mh = 0, n = 0;
        std::uint8_t kind = 0, rings = 0;
        if (!take(&h, 2) || !take(&mh, 2) || !take(&kind, 1) || !take(&rings, 1)) return false;
        WorldBuilding b;
        for (int r = 0; r < rings; ++r) {
            if (!take(&n, 2)) return false;
            for (int i = 0; i < n; ++i) {
                std::int16_t xy[2];
                if (!take(xy, 4)) return false;
                if (r == 0) b.footprint.emplace_back(x0 + xy[0] / 10.0, y0 + xy[1] / 10.0);  // the outline (courtyards are open air)
            }
        }
        if (b.footprint.size() < 3) continue;
        Real cx = 0, cy = 0;
        for (const Pt& q : b.footprint) { cx += q.first; cy += q.second; }
        cx /= static_cast<Real>(b.footprint.size());
        cy /= static_cast<Real>(b.footprint.size());
        // The inner ground under the centroid and the first corner (as the renderer stands it).
        Real g = 1e300;
        for (const Pt& q : {Pt{cx, cy}, b.footprint.front()}) {
            const GroundHit hit = inner_->below(world::nedFromEnu({q.first, q.second, 1e4}));
            if (hit.hit) g = std::min(g, -hit.point.z);
        }
        if (!(g < 1e299)) continue;
        b.base = g - sink + (mh > 0 ? mh / 10.0 + sink : 0.0);
        b.top = g + h / 10.0;
        (void)kind;
        if (b.top > b.base) out.push_back(std::move(b));
    }
    return true;
}

GroundHit BuildingLayer::below(const Vec3& point) const {
    GroundHit best = inner_->below(point);
    const Vec3 p = world::enuFromNed(point);
    forBuildingsAt(p.x, p.y, [&](const WorldBuilding& b) {
        // A roof at or below the point (0.5 m tolerance) that is higher than what we have.
        if (b.top > p.z + 0.5) return;
        if (best.hit && -best.point.z >= b.top) return;
        best = hitFromEnu(p.x, p.y, b.top, {0, 0, 1}, p.z - b.top, surfaces::kBuilding);
    });
    return best;
}

bool BuildingLayer::solidAt(const Vec3& point, GroundHit& contact) const {
    if (inner_->solidAt(point, contact)) return true;
    const Vec3 p = world::enuFromNed(point);
    bool found = false;
    forBuildingsAt(p.x, p.y, [&](const WorldBuilding& w) {
        if (found || p.z >= w.top || p.z < w.base - 0.5) return;
        Real cx = 0, cy = 0;
        const Real wall = nearestEdge(w.footprint, p.x, p.y, cx, cy);
        const Real roof = w.top - p.z;
        if (roof <= wall) {
            contact = hitFromEnu(p.x, p.y, w.top, {0, 0, 1}, roof, surfaces::kBuilding);
        } else {
            const Real dx = cx - p.x, dy = cy - p.y, l = std::max(std::hypot(dx, dy), 1e-9);
            contact = hitFromEnu(cx, cy, p.z, {dx / l, dy / l, 0}, wall, surfaces::kBuilding);
        }
        found = true;
    });
    return found;
}

GroundHit BuildingLayer::raycast(const Vec3& originNed, const Vec3& dirNed, Real maxDistance) const {
    GroundHit best = inner_->raycast(originNed, dirNed, maxDistance);
    Real bestT = best.hit ? best.distance : maxDistance;
    const Vec3 o = world::enuFromNed(originNed), d = world::enuFromNed(dirNed);
    // March the ray's horizontal track through the cells (half-cell steps), testing
    // each building met once: its roof and its walls.
    std::vector<const WorldBuilding*> seen;
    const Real horiz = std::hypot(d.x, d.y);
    const Real step = horiz > 1e-9 ? 0.5 * cell_ / horiz : bestT + 1;
    for (Real t = 0; t <= bestT + step; t += step) {
        const Real tt = std::min(t, bestT);
        const auto it = grid_.find(cellKey(o.x + d.x * tt, o.y + d.y * tt));
        if (it != grid_.end())
            for (const Entry& e : it->second) {
                if (std::find(seen.begin(), seen.end(), e.b) != seen.end()) continue;
                seen.push_back(e.b);
                const WorldBuilding& w = *e.b;
                // Roof plane.
                if (std::abs(d.z) > 1e-9) {
                    const Real tr = (w.top - o.z) / d.z;
                    if (tr >= 0 && tr < bestT && insidePolygon(w.footprint, o.x + d.x * tr, o.y + d.y * tr)) {
                        bestT = tr;
                        best = hitFromEnu(o.x + d.x * tr, o.y + d.y * tr, w.top, {0, 0, std::copysign(1.0, -d.z)}, tr, surfaces::kBuilding);
                    }
                }
                // Walls: the track against each edge, the height within base..top.
                for (std::size_t i = 0, j = w.footprint.size() - 1; i < w.footprint.size(); j = i++) {
                    const Real ax = w.footprint[j].first, ay = w.footprint[j].second;
                    const Real ex = w.footprint[i].first - ax, ey = w.footprint[i].second - ay;
                    const Real den = d.x * ey - d.y * ex;
                    if (std::abs(den) < 1e-12) continue;
                    const Real tw = ((ax - o.x) * ey - (ay - o.y) * ex) / den;
                    const Real s = ((ax - o.x) * d.y - (ay - o.y) * d.x) / den;
                    if (tw < 0 || tw >= bestT || s < 0 || s > 1) continue;
                    const Real z = o.z + d.z * tw;
                    if (z < w.base || z > w.top) continue;
                    const Real l = std::max(std::hypot(ex, ey), 1e-9);
                    Vec3 n{ey / l, -ex / l, 0};
                    if (n.x * d.x + n.y * d.y > 0) n = n * -1.0;
                    bestT = tw;
                    best = hitFromEnu(o.x + d.x * tw, o.y + d.y * tw, z, n, tw, surfaces::kBuilding);
                }
            }
        if (t >= bestT) break;
    }
    return best;
}

}  // namespace avd
