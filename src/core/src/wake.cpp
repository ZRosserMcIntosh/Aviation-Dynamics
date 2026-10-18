// Aviation Dynamics — wake vortices (see wake.hpp).
// SPDX-License-Identifier: MIT
#include "avd/wake.hpp"

#include <algorithm>
#include <cmath>

#include "avd/aero.hpp"
#include "avd/aircraft.hpp"
#include "avd/ground.hpp"

namespace avd {

namespace {

constexpr Real kLambOseen = 1.25643;  // r_c: radius of the peak swirl of a Lamb-Oseen vortex
constexpr Real kNoGround = 1e9;

// Lamb-Oseen factor for the swirl at radius^2 r2 of a core of radius^2 rc2.
Real coreFactor(Real r2, Real rc2) { return 1 - std::exp(-kLambOseen * r2 / std::max(rc2, 1e-12)); }

// Velocity at p of an infinite straight vortex through q along unit axis a.
Vec3 lineVelocity(const Vec3& p, const Vec3& q, const Vec3& a, Real gamma, Real rc2) {
    Vec3 d = p - q;
    d -= a * dot(d, a);
    const Real r2 = dot(d, d);
    if (r2 < 1e-12) return {};
    return cross(a, d) * (gamma / (kTwoPi * r2) * coreFactor(r2, rc2));
}

Vec3 mirror(const Vec3& p, Real groundZ) { return {p.x, p.y, 2 * groundZ - p.z}; }

// Rotation of v about unit axis a by angle t (Rodrigues).
Vec3 rotateAbout(const Vec3& v, const Vec3& a, Real t) {
    const Real c = std::cos(t), s = std::sin(t);
    return v * c + cross(a, v) * s + a * (dot(a, v) * (1 - c));
}

Real boxDistance(const Vec3& p, const Vec3& lo, const Vec3& hi) {
    const Real dx = std::max({lo.x - p.x, 0.0, p.x - hi.x});
    const Real dy = std::max({lo.y - p.y, 0.0, p.y - hi.y});
    const Real dz = std::max({lo.z - p.z, 0.0, p.z - hi.z});
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace

Vec3 vortexSegmentVelocity(const Vec3& p, const Vec3& a, const Vec3& b, Real gamma, Real rc) {
    // Katz & Plotkin's segment formula with the perpendicular distance
    // regularised by the Lamb-Oseen core.
    const Vec3 r0 = b - a, r1 = p - a, r2 = p - b;
    const Vec3 c = cross(r1, r2);
    const Real c2 = dot(c, c);
    const Real l2 = dot(r0, r0);
    if (l2 < 1e-12 || c2 < 1e-18 * l2) return {};
    const Real n1 = length(r1), n2 = length(r2);
    if (n1 < 1e-9 || n2 < 1e-9) return {};
    const Real h2 = c2 / l2;  // squared distance from the line
    const Real k = gamma / (4 * kPi * c2) * dot(r0, r1 / n1 - r2 / n2) * coreFactor(h2, rc * rc);
    return c * k;
}

Wake::Wake(const AeroModel& model, const WakeParams& p) : p_(p) { buildLayouts(model); }

void Wake::reset() {
    gates_.clear();
    for (Attached& a : attached_) a.edgeCount = 0;
    lastShed = -1e9;
    started_ = false;
}

// ----------------------------------------------------------------- layout --

void Wake::buildLayouts(const AeroModel& m) {
    layouts_.clear();
    Real mainArea = 0;
    int mainIndex = -1;
    for (int s = 0; s < m.surfaceCount(); ++s) {
        const int f = m.firstStrip(s), e = m.endStrip(s);
        if (e <= f) continue;
        Layout L;
        L.surface = s;
        // Strips in order along the lift axis (the bound vortex direction).
        const AeroModel::StripGeometry g0 = m.stripGeometry(f);
        const Vec3 lift = normalized(cross(g0.chordAxis, g0.normalAxis));
        std::vector<int> order;
        for (int i = f; i < e; ++i) order.push_back(i);
        std::sort(order.begin(), order.end(), [&](int a, int b) {
            return dot(m.stripGeometry(a).position, lift) < dot(m.stripGeometry(b).position, lift);
        });
        auto lowEnd = [&](int i) {
            const AeroModel::StripGeometry g = m.stripGeometry(i);
            const Vec3 h = g.spanAxis * (0.5 * g.width);
            return dot(h, lift) < 0 ? g.position + h : g.position - h;
        };
        auto highEnd = [&](int i) {
            const AeroModel::StripGeometry g = m.stripGeometry(i);
            const Vec3 h = g.spanAxis * (0.5 * g.width);
            return dot(h, lift) < 0 ? g.position - h : g.position + h;
        };
        // Group the strips into bins, with a bridge over the fuselage where
        // the two sides of a mirrored surface leave a gap (fuselage carry-over).
        const int n = static_cast<int>(order.size());
        const int perBin = std::max(1, (n + p_.maxBinsPerSurface - 2) / (p_.maxBinsPerSurface - 1));
        struct Bin {
            std::vector<int> strips;
        };
        std::vector<Bin> bins;
        for (int k = 0; k < n;) {
            Bin b;
            const int side = m.stripGeometry(order[k]).side;
            while (k < n && static_cast<int>(b.strips.size()) < perBin && m.stripGeometry(order[k]).side == side) b.strips.push_back(order[k++]);
            if (!bins.empty()) {
                const int prevLast = bins.back().strips.back();
                const AeroModel::StripGeometry pg = m.stripGeometry(prevLast), ng = m.stripGeometry(b.strips.front());
                const Real gap = dot(lowEnd(b.strips.front()) - highEnd(prevLast), lift);
                if (pg.side != ng.side && gap > 0.05 * (pg.width + ng.width)) bins.push_back(Bin{});  // bridge
            }
            bins.push_back(b);
        }
        bins.resize(std::min<std::size_t>(bins.size(), kMaxEdges - 1));
        L.binCount = static_cast<int>(bins.size());
        // Bin end points at the quarter chord and the trailing edge.
        auto te = [&](int i, const Vec3& qc) {
            const AeroModel::StripGeometry g = m.stripGeometry(i);
            return qc - g.chordAxis * (0.75 * g.chord);
        };
        std::vector<Vec3> loQ(bins.size()), hiQ(bins.size()), loT(bins.size()), hiT(bins.size());
        for (std::size_t b = 0; b < bins.size(); ++b) {
            if (bins[b].strips.empty()) continue;
            const int a = bins[b].strips.front(), z = bins[b].strips.back();
            loQ[b] = lowEnd(a);
            hiQ[b] = highEnd(z);
            loT[b] = te(a, loQ[b]);
            hiT[b] = te(z, hiQ[b]);
        }
        for (std::size_t b = 0; b < bins.size(); ++b) {
            if (!bins[b].strips.empty()) continue;  // bridge: between its neighbours
            loQ[b] = hiQ[b - 1];
            loT[b] = hiT[b - 1];
            hiQ[b] = loQ[b + 1];
            hiT[b] = loT[b + 1];
        }
        for (int k = 0; k <= L.binCount; ++k) {
            if (k == 0) {
                L.qc[0] = loQ[0];
                L.te[0] = loT[0];
            } else if (k == L.binCount) {
                L.qc[k] = hiQ[k - 1];
                L.te[k] = hiT[k - 1];
            } else {
                L.qc[k] = (hiQ[k - 1] + loQ[k]) * 0.5;
                L.te[k] = (hiT[k - 1] + loT[k]) * 0.5;
            }
        }
        Real chordArea = 0;
        for (int b = 0; b < L.binCount; ++b) {
            L.stripCount[b] = 0;
            for (int i : bins[static_cast<std::size_t>(b)].strips)
                if (L.stripCount[b] < 8) L.strips[b][L.stripCount[b]++] = i;
            for (int i : bins[static_cast<std::size_t>(b)].strips) {
                const AeroModel::StripGeometry g = m.stripGeometry(i);
                L.area += g.area;
                chordArea += g.chord * g.area;
            }
        }
        L.span = length(L.qc[L.binCount] - L.qc[0]);
        L.meanChord = L.area > 0 ? chordArea / L.area : 1;
        if (L.area > mainArea) {
            mainArea = L.area;
            mainIndex = static_cast<int>(layouts_.size());
        }
        layouts_.push_back(L);
    }
    if (mainIndex >= 0) {
        layouts_[static_cast<std::size_t>(mainIndex)].main = true;
        refSpan_ = std::max(layouts_[static_cast<std::size_t>(mainIndex)].span, 1.0);
    }
    attached_.assign(layouts_.size(), {});
    for (std::size_t i = 0; i < layouts_.size(); ++i) {
        attached_[i].layout = static_cast<int>(i);
        attached_[i].core = 0.25 * layouts_[i].meanChord;
    }
}

// ------------------------------------------------------------------ decay --

Real Wake::decay(const Gate& gt, Real age) const {
    // Holzapfel P2P, simplified: slow diffusion, then rapid decay from T2*.
    const Real ts = age / gt.t0, t2 = gt.t2 / gt.t0;
    Real d = std::exp(-0.03 * ts);
    if (ts > t2) d *= std::exp(-sq(ts - t2));
    return d;
}

Real Wake::rolled(const Gate& gt, Real age) const { return smoothstep(0, gt.rollTime, age); }

Real Wake::vortexCoreSq(const Gate& gt, const Vortex& v, Real age) const {
    (void)gt;
    return v.coreSq0 + 4 * kLambOseen * (p_.kinematicViscosity + p_.squire * std::abs(v.gamma)) * age;
}

Vec3 Wake::edgePosition(const Gate& gt, int e, Real age) const {
    const int o = gt.owner[static_cast<std::size_t>(e)];
    const Vec3& off = gt.offset[static_cast<std::size_t>(e)];
    if (o < 0) return gt.anchor + off;
    const Real k = 1 - rolled(gt, age);
    if (k <= 1e-6) return gt.v[static_cast<std::size_t>(o)].pos;
    return gt.v[static_cast<std::size_t>(o)].pos + rotateAbout(off, gt.axis, gt.spin[static_cast<std::size_t>(e)]) * k;
}

Real Wake::edgeCoreAt(const Gate& gt, int e, Real age) const {
    const int o = gt.owner[static_cast<std::size_t>(e)];
    if (o < 0) return gt.edgeCore;
    const Real rc = std::sqrt(vortexCoreSq(gt, gt.v[static_cast<std::size_t>(o)], age));
    return lerp(gt.edgeCore, rc, rolled(gt, age));
}

// --------------------------------------------------------------- shedding --

void Wake::update(const Aircraft& ac, const AtmosphereField& air, const GroundQuery* ground) {
    const Real now = ac.time();
    const DynamicState& st = ac.state();
    const Vec3 cgNed = st.body.position;
    // Set back in time, or moved further than it could fly: a new start.
    if (started_ && (now < time_ - 1e-9 || length(cgNed - lastPos_) > 50 + 2 * length(st.body.velocity) * (now - time_))) reset();
    lastPos_ = cgNed;
    const Vec3 cg = ac.massProperties().cg;
    const Quat& q = st.body.attitude;
    const AeroOutputs& ao = ac.aero();
    const AeroModel& m = ac.aeroModel();
    msl0_ = ac.altitudeMsl() + cgNed.z;
    groundZ_ = kNoGround;
    if (ground) {
        const GroundHit h = ground->below(cgNed);
        if (h.hit) groundZ_ = h.point.z;
    }
    // Bound vortices and the trailing edge now, in the world frame.
    auto world = [&](const Vec3& s) { return cgNed + q.rotate(s - cg); };
    for (std::size_t li = 0; li < layouts_.size(); ++li) {
        const Layout& L = layouts_[li];
        Attached& A = attached_[li];
        A.edgeCount = L.binCount + 1;
        for (int k = 0; k <= L.binCount; ++k) {
            A.qc[static_cast<std::size_t>(k)] = world(L.qc[static_cast<std::size_t>(k)]);
            A.te[static_cast<std::size_t>(k)] = world(L.te[static_cast<std::size_t>(k)]);
        }
        for (int b = 0; b < L.binCount; ++b) {
            Real gs = 0, ws = 0;
            for (int j = 0; j < L.stripCount[static_cast<std::size_t>(b)]; ++j) {
                const int i = L.strips[static_cast<std::size_t>(b)][static_cast<std::size_t>(j)];
                const StripTelemetry& t = ao.strips[static_cast<std::size_t>(i)];
                const AeroModel::StripGeometry g = m.stripGeometry(i);
                // Kutta-Joukowski on the (swept) bound vortex: the strip's
                // lift q S cl = rho |V x l| G with q on the flow normal to the
                // span, so G = V_perp cl (S / width) / 2 (S / width = c cos sweep).
                const Real G = t.q > 0 ? 0.5 * length(t.flowPerp) * t.cl * g.area / std::max(g.width, 1e-6) : 0;
                gs += G * g.width;
                ws += g.width;
            }
            A.g[static_cast<std::size_t>(b)] = ws > 0 ? gs / ws : 0;
        }
        for (int b = 0; b < L.binCount; ++b)  // fuselage bridges carry the mean of their neighbours
            if (L.stripCount[static_cast<std::size_t>(b)] == 0 && b > 0 && b + 1 < L.binCount)
                A.g[static_cast<std::size_t>(b)] = 0.5 * (A.g[static_cast<std::size_t>(b - 1)] + A.g[static_cast<std::size_t>(b + 1)]);
    }
    const Real speed = ao.airspeed;
    if (!started_) {
        time_ = now;
        started_ = true;
    }
    const Real interval = clamp(p_.shedSpacing * refSpan_ / std::max(speed, 1.0), p_.minShedInterval, p_.maxShedInterval);
    if (speed > 2 && now - lastShed >= interval - 1e-9) {
        time_ = now;
        shed(ac, speed);
        lastShed = now;
    }
    advanceTo(now, air, msl0_);
}

void Wake::shed(const Aircraft& ac, Real speed) {
    const Vec3 vAir = ac.state().body.velocity - ac.telemetry().air.wind;
    const Vec3 axis = length(vAir) > 0.5 ? normalized(vAir) * -1.0 : ac.state().body.attitude.rotate({-1, 0, 0});
    const std::int64_t index = nextIndex_++;
    for (std::size_t li = 0; li < layouts_.size(); ++li) {
        const Layout& L = layouts_[li];
        const Attached& A = attached_[li];
        Gate gt;
        gt.layout = static_cast<int>(li);
        gt.index = index;
        gt.time = time_;
        gt.lastUpdate = time_;
        gt.axis = axis;
        const Vec3 lat = A.te[static_cast<std::size_t>(L.binCount)] - A.te[0];
        gt.ey = normalized(lat - axis * dot(lat, axis));
        gt.ez = cross(axis, gt.ey);
        gt.edgeCount = A.edgeCount;
        gt.groundZ = groundZ_;
        gt.edgeCore = std::max(0.5 * L.span / std::max(L.binCount, 1), 0.05);
        for (int b = 0; b < L.binCount; ++b) gt.g[static_cast<std::size_t>(b)] = A.g[static_cast<std::size_t>(b)];
        // Trailing vorticity along the axis at each edge, then the runs of one
        // sign: each rolls up into one vortex at its centroid (Betz).
        std::array<Real, kMaxEdges> gam{};
        Real gamMax = 0;
        for (int e = 0; e < gt.edgeCount; ++e) {
            const Real before = e > 0 ? gt.g[static_cast<std::size_t>(e - 1)] : 0;
            const Real after = e < L.binCount ? gt.g[static_cast<std::size_t>(e)] : 0;
            gam[static_cast<std::size_t>(e)] = before - after;
            gamMax = std::max(gamMax, std::abs(gam[static_cast<std::size_t>(e)]));
        }
        Vec3 anchor;
        for (int e = 0; e < gt.edgeCount; ++e) anchor += A.te[static_cast<std::size_t>(e)];
        gt.anchor = anchor / std::max(gt.edgeCount, 1);
        const bool lifting = gamMax > 1e-3;
        if (lifting) {
            const Real tiny = 0.03 * gamMax;
            int runSign = 0, run = -1;
            std::array<Real, kMaxVortices> wsum{}, gsum{};
            std::array<Vec3, kMaxVortices> psum{};
            for (int e = 0; e < gt.edgeCount; ++e) {
                const Real gm = gam[static_cast<std::size_t>(e)];
                const int sgn = std::abs(gm) < tiny ? 0 : (gm > 0 ? 1 : -1);
                if (run < 0 || (sgn != 0 && sgn != runSign && runSign != 0)) {
                    if (run + 1 < kMaxVortices) ++run;
                    runSign = sgn;
                } else if (runSign == 0) {
                    runSign = sgn;
                }
                const std::size_t r = static_cast<std::size_t>(run);
                gt.owner[static_cast<std::size_t>(e)] = static_cast<std::int8_t>(run);
                wsum[r] += std::abs(gm);
                gsum[r] += gm;
                psum[r] += A.te[static_cast<std::size_t>(e)] * std::abs(gm);
            }
            gt.vortexCount = run + 1;
            for (int r = 0; r < gt.vortexCount; ++r) {
                Vortex& v = gt.v[static_cast<std::size_t>(r)];
                v.gamma = gsum[static_cast<std::size_t>(r)];
                if (wsum[static_cast<std::size_t>(r)] > 0) {
                    v.pos = psum[static_cast<std::size_t>(r)] / wsum[static_cast<std::size_t>(r)];
                } else {  // a run of negligible edges: its middle edge
                    Vec3 c;
                    int n = 0;
                    for (int e = 0; e < gt.edgeCount; ++e)
                        if (gt.owner[static_cast<std::size_t>(e)] == r) c += A.te[static_cast<std::size_t>(e)], ++n;
                    v.pos = c / std::max(n, 1);
                }
            }
            for (int e = 0; e < gt.edgeCount; ++e)
                gt.offset[static_cast<std::size_t>(e)] = A.te[static_cast<std::size_t>(e)] - gt.v[static_cast<std::size_t>(gt.owner[static_cast<std::size_t>(e)])].pos;
            // The main pair: the strongest vortex of each sign.
            int ip = -1, in = -1;
            for (int r = 0; r < gt.vortexCount; ++r) {
                const Real g = gt.v[static_cast<std::size_t>(r)].gamma;
                if (g > 0 && (ip < 0 || g > gt.v[static_cast<std::size_t>(ip)].gamma)) ip = r;
                if (g < 0 && (in < 0 || g < gt.v[static_cast<std::size_t>(in)].gamma)) in = r;
            }
            if (ip >= 0 && in >= 0) {
                gt.b0 = std::max(length(gt.v[static_cast<std::size_t>(ip)].pos - gt.v[static_cast<std::size_t>(in)].pos), 0.1 * L.span);
                gt.gamma0 = 0.5 * (gt.v[static_cast<std::size_t>(ip)].gamma - gt.v[static_cast<std::size_t>(in)].gamma);
            } else {
                gt.b0 = kPi / 4 * L.span;
                gt.gamma0 = gamMax;
            }
            for (int r = 0; r < gt.vortexCount; ++r) gt.v[static_cast<std::size_t>(r)].coreSq0 = sq(p_.rolledCore * gt.b0);
            // Roll-up over the Spreiter-Sacks distance 0.28 AR b / CL.
            Real lift = 0;
            for (int b = 0; b < L.binCount; ++b)
                lift += gt.g[static_cast<std::size_t>(b)] * length(A.qc[static_cast<std::size_t>(b + 1)] - A.qc[static_cast<std::size_t>(b)]);
            const Real cl = 2 * std::abs(lift) / (speed * std::max(L.area, 1e-3));
            const Real ar = sq(L.span) / std::max(L.area, 1e-3);
            gt.rollTime = clamp(0.28 * ar * L.span / std::max(cl, 0.05) / speed, 0.2, 4.0);
            // P2P time scale and the onset of rapid decay.
            gt.t0 = std::max(kTwoPi * sq(gt.b0) / std::max(gt.gamma0, 1e-3), 0.5);
            const Real w0 = gt.gamma0 / (kTwoPi * gt.b0);
            const Real epsStar = std::cbrt(std::max(p_.eddyDissipation, 1e-8) * gt.b0) / std::max(w0, 1e-6);
            gt.t2 = std::min(0.8 * std::pow(epsStar, -0.75), 6.0) * gt.t0;
        } else {
            for (int e = 0; e < gt.edgeCount; ++e) {
                gt.owner[static_cast<std::size_t>(e)] = -1;
                gt.offset[static_cast<std::size_t>(e)] = A.te[static_cast<std::size_t>(e)] - gt.anchor;
            }
            gt.b0 = kPi / 4 * L.span;
            gt.t0 = 1;
            gt.t2 = 2;
        }
        addGate(gt);
    }
}

void Wake::seedPair(Real time, const Vec3& centre, const Vec3& axis, const Vec3& lateral, Real b0, Real gamma, Real groundZ) {
    Gate gt;
    gt.layout = -1;
    gt.index = nextIndex_++;
    gt.time = gt.lastUpdate = time;
    gt.axis = normalized(axis);
    gt.ey = normalized(lateral - gt.axis * dot(lateral, gt.axis));
    gt.ez = cross(gt.axis, gt.ey);
    gt.groundZ = groundZ;
    gt.edgeCount = 2;
    gt.g[0] = gamma;
    gt.anchor = centre;
    gt.vortexCount = 2;
    gt.v[0].pos = centre - gt.ey * (0.5 * b0);
    gt.v[0].gamma = -gamma;
    gt.v[1].pos = centre + gt.ey * (0.5 * b0);
    gt.v[1].gamma = gamma;
    gt.owner[0] = 0;
    gt.owner[1] = 1;
    gt.b0 = b0;
    gt.gamma0 = std::abs(gamma);
    gt.edgeCore = p_.rolledCore * b0;
    for (int r = 0; r < 2; ++r) gt.v[static_cast<std::size_t>(r)].coreSq0 = sq(p_.rolledCore * b0);
    gt.rollTime = 1e-3;
    gt.t0 = std::max(kTwoPi * sq(b0) / std::max(gt.gamma0, 1e-3), 0.5);
    const Real w0 = gt.gamma0 / (kTwoPi * b0);
    const Real epsStar = std::cbrt(std::max(p_.eddyDissipation, 1e-8) * b0) / std::max(w0, 1e-6);
    gt.t2 = std::min(0.8 * std::pow(epsStar, -0.75), 6.0) * gt.t0;
    if (!started_) {
        time_ = time;
        started_ = true;
    }
    time_ = std::max(time_, time);
    refSpan_ = std::max(refSpan_, b0 / (kPi / 4));
    addGate(gt);
}

void Wake::addGate(Gate& gt) {
    refreshBox(gt);
    if (gates_.empty()) {
        lo_ = gt.lo;
        hi_ = gt.hi;
        cullRadius_ = 0;
    }
    lo_ = {std::min(lo_.x, gt.lo.x), std::min(lo_.y, gt.lo.y), std::min(lo_.z, gt.lo.z)};
    hi_ = {std::max(hi_.x, gt.hi.x), std::max(hi_.y, gt.hi.y), std::max(hi_.z, gt.hi.z)};
    cullRadius_ = std::max(cullRadius_, 10 * gt.b0);
    gates_.push_front(gt);
}

// -------------------------------------------------------------- transport --

void Wake::advanceTo(Real time, const AtmosphereField& air, Real altitudeOfZeroZ) {
    time_ = std::max(time_, time);
    msl0_ = altitudeOfZeroZ;
    for (Gate& gt : gates_) {
        const Real age = time_ - gt.time;
        const Real interval = age < 5 ? p_.updateInterval : age < 30 ? 0.2 : 0.5;
        const Real dt = time_ - gt.lastUpdate;
        if (dt >= interval - 1e-9) {
            advance(gt, dt, air, msl0_);
            gt.lastUpdate = time_;
        }
    }
    // Thin the older gates (whole rows), drop the decayed ones.
    gates_.erase(std::remove_if(gates_.begin(), gates_.end(),
                                [&](const Gate& gt) {
                                    const Real age = time_ - gt.time;
                                    if (age > p_.maxAge) return true;
                                    if (gt.layout >= 0) {
                                        if (age > 15 && gt.index % 16 != 0) return true;
                                        if (age > 2 && gt.index % 4 != 0) return true;
                                    }
                                    return age > gt.t0 && decay(gt, age) < 0.01;
                                }),
                 gates_.end());
    lo_ = {1e30, 1e30, 1e30};
    hi_ = {-1e30, -1e30, -1e30};
    cullRadius_ = 0;
    for (const Gate& gt : gates_) {
        lo_ = {std::min(lo_.x, gt.lo.x), std::min(lo_.y, gt.lo.y), std::min(lo_.z, gt.lo.z)};
        hi_ = {std::max(hi_.x, gt.hi.x), std::max(hi_.y, gt.hi.y), std::max(hi_.z, gt.hi.z)};
        cullRadius_ = std::max(cullRadius_, 10 * gt.b0);
    }
}

void Wake::advance(Gate& gt, Real dt, const AtmosphereField& air, Real altitudeOffset) {
    const Real age = time_ - gt.time;
    const Real d = decay(gt, age);
    const bool ground = gt.groundZ < kNoGround * 0.5;
    // One air sample per gate, at the pair's middle; the gradient carries it
    // across the pair (shear tilts and stretches it).
    Vec3 mid = gt.anchor;
    if (gt.vortexCount > 0) {
        mid = {};
        for (int r = 0; r < gt.vortexCount; ++r) mid += gt.v[static_cast<std::size_t>(r)].pos;
        mid = mid / gt.vortexCount;
    }
    const AirSample s = air.sample(mid, altitudeOffset - mid.z, time_);
    std::array<Vec3, kMaxVortices> vel{};
    for (int i = 0; i < gt.vortexCount; ++i) {
        const Vortex& vi = gt.v[static_cast<std::size_t>(i)];
        Vec3 u;
        for (int j = 0; j < gt.vortexCount; ++j) {
            const Vortex& vj = gt.v[static_cast<std::size_t>(j)];
            const Real rc2 = vortexCoreSq(gt, vj, age);
            if (j != i) u += lineVelocity(vi.pos, vj.pos, gt.axis, vj.gamma * d, rc2);
            if (ground) {
                const Vec3 ia{gt.axis.x, gt.axis.y, -gt.axis.z};
                u += lineVelocity(vi.pos, mirror(vj.pos, gt.groundZ), normalized(ia), -vj.gamma * d, rc2);
            }
        }
        u -= gt.axis * dot(u, gt.axis);
        vel[static_cast<std::size_t>(i)] = s.wind + s.windGradient * (vi.pos - mid) + u;
    }
    for (int i = 0; i < gt.vortexCount; ++i) {
        Vortex& v = gt.v[static_cast<std::size_t>(i)];
        v.pos += vel[static_cast<std::size_t>(i)] * dt;
        if (ground) v.pos.z = std::min(v.pos.z, gt.groundZ - 0.5 * std::sqrt(vortexCoreSq(gt, v, age)));
    }
    gt.anchor += (s.wind + s.windGradient * (gt.anchor - mid)) * dt;
    // The sheet spirals into its vortex as it rolls up.
    const Real k = 1 - rolled(gt, age);
    if (k > 1e-6) {
        for (int e = 0; e < gt.edgeCount; ++e) {
            const int o = gt.owner[static_cast<std::size_t>(e)];
            if (o < 0) continue;
            const Vortex& v = gt.v[static_cast<std::size_t>(o)];
            Vec3 off = gt.offset[static_cast<std::size_t>(e)];
            off -= gt.axis * dot(off, gt.axis);
            const Real r2 = dot(off, off) * k * k;
            const Real rc2 = vortexCoreSq(gt, v, age);
            const Real omega = v.gamma * d / kTwoPi * (r2 > 1e-9 ? coreFactor(r2, rc2) / r2 : kLambOseen / rc2);
            gt.spin[static_cast<std::size_t>(e)] += omega * dt;
        }
    }
    refreshBox(gt);
}

void Wake::refreshBox(Gate& gt) const {
    const Real age = time_ - gt.time;
    Vec3 lo{1e30, 1e30, 1e30}, hi{-1e30, -1e30, -1e30};
    Real pad = gt.edgeCore;
    for (int e = 0; e < gt.edgeCount; ++e) {
        const Vec3 p = edgePosition(gt, e, age);
        lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)};
        hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)};
    }
    for (int r = 0; r < gt.vortexCount; ++r) {
        const Vec3 p = gt.v[static_cast<std::size_t>(r)].pos;
        lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)};
        hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)};
        pad = std::max(pad, std::sqrt(vortexCoreSq(gt, gt.v[static_cast<std::size_t>(r)], age)));
    }
    gt.lo = lo - Vec3{pad, pad, pad};
    gt.hi = hi + Vec3{pad, pad, pad};
}

// ---------------------------------------------------------------- segments --

template <class Emit>
void Wake::forEachSegment(Real minAge, bool images, const Vec3* near, Emit&& emit) const {
    bool skip = false;  // the current row is too far from `near`
    auto out = [&](const Vec3& a, const Vec3& b, Real gamma, Real core, Real age, Real groundZ) {
        if (skip) return;
        if (std::abs(gamma) < 1e-4) return;
        if (dot(b - a, b - a) < 1e-8) return;
        emit(a, b, gamma, core, age);
        if (images && groundZ < kNoGround * 0.5) emit(mirror(a, groundZ), mirror(b, groundZ), -gamma, core, age);
    };
    // Per layout: the attached row (now) and its gates, newest first. Seeded
    // gates (layout -1) form their own sequence.
    const int nl = static_cast<int>(layouts_.size());
    for (int li = -1; li < nl; ++li) {
        // Collect this layout's gates in order.
        const Gate* prev = nullptr;
        Real prevAge = 0;
        std::array<Real, kMaxEdges> prevS{};
        std::array<Vec3, kMaxEdges> prevP{};
        std::array<Real, kMaxEdges> prevCore{};
        int edges = 0;
        bool havePrev = false;
        bool prevRolled = false;
        if (li >= 0) {
            const Attached& A = attached_[static_cast<std::size_t>(li)];
            edges = A.edgeCount;
            skip = near && edges > 0 && length(*near - A.qc[0]) > 12 * refSpan_;
            if (edges > 0 && minAge <= 0) {
                // Bound vortices along the quarter chord and the legs to the trailing edge.
                const Real gz = groundZ_;
                for (int j = 0; j + 1 < edges; ++j)
                    out(A.qc[static_cast<std::size_t>(j)], A.qc[static_cast<std::size_t>(j + 1)], A.g[static_cast<std::size_t>(j)], A.core, 0, gz);
                for (int e = 0; e < edges; ++e) {
                    const Real g = (e > 0 ? A.g[static_cast<std::size_t>(e - 1)] : 0) - (e + 1 < edges ? A.g[static_cast<std::size_t>(e)] : 0);
                    out(A.qc[static_cast<std::size_t>(e)], A.te[static_cast<std::size_t>(e)], g, A.core, 0, gz);
                }
            }
            if (edges > 0 && minAge <= 0) {
                for (int e = 0; e < edges; ++e) {
                    prevP[static_cast<std::size_t>(e)] = A.te[static_cast<std::size_t>(e)];
                    prevCore[static_cast<std::size_t>(e)] = A.core;
                }
                for (int j = 0; j + 1 < edges; ++j) prevS[static_cast<std::size_t>(j)] = A.g[static_cast<std::size_t>(j)];
                havePrev = true;
            }
        }
        for (const Gate& gt : gates_) {
            if (gt.layout != li) continue;
            const Real age = time_ - gt.time;
            if (age < minAge) continue;
            if (li < 0 && edges == 0) edges = gt.edgeCount;
            if (gt.edgeCount != edges) continue;
            const Real d = decay(gt, age);
            std::array<Real, kMaxEdges> S{};
            std::array<Vec3, kMaxEdges> P{};
            std::array<Real, kMaxEdges> C{};
            for (int j = 0; j + 1 < edges; ++j) S[static_cast<std::size_t>(j)] = gt.g[static_cast<std::size_t>(j)] * d;
            for (int e = 0; e < edges; ++e) {
                P[static_cast<std::size_t>(e)] = edgePosition(gt, e, age);
                C[static_cast<std::size_t>(e)] = edgeCoreAt(gt, e, age);
            }
            const bool isRolled = gt.vortexCount > 0 && rolled(gt, age) > 0.999;
            if (near) {
                Vec3 lo = gt.lo, hi = gt.hi;
                if (prev) {
                    lo = {std::min(lo.x, prev->lo.x), std::min(lo.y, prev->lo.y), std::min(lo.z, prev->lo.z)};
                    hi = {std::max(hi.x, prev->hi.x), std::max(hi.y, prev->hi.y), std::max(hi.z, prev->hi.z)};
                }
                skip = boxDistance(*near, lo, hi) > 10 * gt.b0 + (prev ? 0 : refSpan_);
            }
            if (havePrev) {
                // Trailing filaments from the newer row to this one, strength
                // of the newer ring; coincident ones (both rows rolled up) merged.
                const Real ageMid = 0.5 * (prevAge + age);
                if (prevRolled && isRolled && prev) {
                    std::array<Real, kMaxVortices * kMaxVortices> acc{};
                    for (int e = 0; e < edges; ++e) {
                        const Real g = (e > 0 ? prevS[static_cast<std::size_t>(e - 1)] : 0) - (e + 1 < edges ? prevS[static_cast<std::size_t>(e)] : 0);
                        acc[static_cast<std::size_t>(prev->owner[static_cast<std::size_t>(e)] * kMaxVortices + gt.owner[static_cast<std::size_t>(e)])] += g;
                    }
                    for (int a = 0; a < prev->vortexCount; ++a)
                        for (int b = 0; b < gt.vortexCount; ++b) {
                            const Real g = acc[static_cast<std::size_t>(a * kMaxVortices + b)];
                            if (g == 0) continue;
                            const Real core = 0.5 * (std::sqrt(vortexCoreSq(*prev, prev->v[static_cast<std::size_t>(a)], prevAge)) +
                                                     std::sqrt(vortexCoreSq(gt, gt.v[static_cast<std::size_t>(b)], age)));
                            out(prev->v[static_cast<std::size_t>(a)].pos, gt.v[static_cast<std::size_t>(b)].pos, g, core, ageMid, gt.groundZ);
                        }
                } else {
                    for (int e = 0; e < edges; ++e) {
                        const Real g = (e > 0 ? prevS[static_cast<std::size_t>(e - 1)] : 0) - (e + 1 < edges ? prevS[static_cast<std::size_t>(e)] : 0);
                        out(prevP[static_cast<std::size_t>(e)], P[static_cast<std::size_t>(e)], g,
                            0.5 * (prevCore[static_cast<std::size_t>(e)] + C[static_cast<std::size_t>(e)]), ageMid, gt.groundZ);
                    }
                }
                // Spanwise (shed) vorticity at this row: the change of the ring strengths.
                for (int j = 0; j + 1 < edges; ++j) {
                    const Real g = S[static_cast<std::size_t>(j)] - prevS[static_cast<std::size_t>(j)];
                    out(P[static_cast<std::size_t>(j)], P[static_cast<std::size_t>(j + 1)], g,
                        0.5 * (C[static_cast<std::size_t>(j)] + C[static_cast<std::size_t>(j + 1)]), age, gt.groundZ);
                }
            } else {
                // The front of the oldest-shown ring sequence (minAge > 0 or a
                // seeded pair): its spanwise edge closes the rings.
                for (int j = 0; j + 1 < edges; ++j)
                    out(P[static_cast<std::size_t>(j)], P[static_cast<std::size_t>(j + 1)], S[static_cast<std::size_t>(j)],
                        0.5 * (C[static_cast<std::size_t>(j)] + C[static_cast<std::size_t>(j + 1)]), age, gt.groundZ);
            }
            prev = &gt;
            prevAge = age;
            prevS = S;
            prevP = P;
            prevCore = C;
            prevRolled = isRolled;
            havePrev = true;
        }
        // The back of the oldest ring.
        if (havePrev && prev) {
            if (near) skip = boxDistance(*near, prev->lo, prev->hi) > 10 * prev->b0;
            for (int j = 0; j + 1 < edges; ++j)
                out(prevP[static_cast<std::size_t>(j + 1)], prevP[static_cast<std::size_t>(j)], prevS[static_cast<std::size_t>(j)],
                    0.5 * (prevCore[static_cast<std::size_t>(j)] + prevCore[static_cast<std::size_t>(j + 1)]), prevAge, prev->groundZ);
        }
    }
}

void Wake::segments(std::vector<WakeSegment>& out) const {
    out.clear();
    forEachSegment(0, true, nullptr, [&](const Vec3& a, const Vec3& b, Real g, Real core, Real age) { out.push_back({a, b, g, core, age}); });
}

Vec3 Wake::velocity(const Vec3& p, Real minAge) const {
    Vec3 u;
    const bool attachedNear = minAge <= 0 && !attached_.empty() && attached_[0].edgeCount > 0 &&
                              length(p - attached_[0].qc[0]) < 12 * refSpan_;
    if (!attachedNear && (gates_.empty() || boxDistance(p, lo_, hi_) > cullRadius_)) return u;
    forEachSegment(minAge, true, &p, [&](const Vec3& a, const Vec3& b, Real g, Real core, Real) {
        u += vortexSegmentVelocity(p, a, b, g, core);
    });
    return u;
}

Wake::PairInfo Wake::pair(Real age) const {
    PairInfo info;
    const Gate* best = nullptr;
    Real bestErr = 1e30;
    for (const Gate& gt : gates_) {
        if (gt.layout >= 0 && !layouts_[static_cast<std::size_t>(gt.layout)].main) continue;
        if (gt.vortexCount < 2) continue;
        const Real err = std::abs(time_ - gt.time - age);
        if (err < bestErr) {
            bestErr = err;
            best = &gt;
        }
    }
    if (!best) return info;
    const Gate& gt = *best;
    const Real a = time_ - gt.time;
    int ip = -1, in = -1;
    for (int r = 0; r < gt.vortexCount; ++r) {
        const Real g = gt.v[static_cast<std::size_t>(r)].gamma;
        if (g > 0 && (ip < 0 || g > gt.v[static_cast<std::size_t>(ip)].gamma)) ip = r;
        if (g < 0 && (in < 0 || g < gt.v[static_cast<std::size_t>(in)].gamma)) in = r;
    }
    if (ip < 0 || in < 0) return info;
    const Vortex& vp = gt.v[static_cast<std::size_t>(ip)];
    const Vortex& vn = gt.v[static_cast<std::size_t>(in)];
    const bool posRight = dot(vp.pos - vn.pos, gt.ey) > 0;
    info.valid = true;
    info.age = a;
    info.gamma0 = 0.5 * (vp.gamma - vn.gamma);
    info.gamma = info.gamma0 * decay(gt, a);
    info.spacing = length(vp.pos - vn.pos);
    info.core = std::sqrt(vortexCoreSq(gt, vp, a));
    info.rolled = rolled(gt, a);
    info.left = posRight ? vn.pos : vp.pos;
    info.right = posRight ? vp.pos : vn.pos;
    info.height = gt.groundZ < kNoGround * 0.5 ? gt.groundZ - 0.5 * (vp.pos.z + vn.pos.z) : kNoGround;
    return info;
}

// ----------------------------------------------------------------- field --

Vec3 WakeField::induced(const Vec3& p) const {
    Vec3 u;
    for (const Entry& e : wakes_) u += e.wake->velocity(p, e.minAge);
    return u;
}

AirSample WakeField::sample(const Vec3& positionNed, Real altitudeMsl, Real time, Real extent) const {
    AirSample s = base_->sample(positionNed, altitudeMsl, time, extent);
    if (wakes_.empty()) return s;
    const Real h = std::max(0.25 * extent, 0.25);
    const Vec3 e[3] = {{h, 0, 0}, {0, h, 0}, {0, 0, h}};
    Vec3 mean;
    for (int j = 0; j < 3; ++j) {
        const Vec3 up = induced(positionNed + e[j]), dn = induced(positionNed - e[j]);
        mean += up + dn;
        const Vec3 d = (up - dn) / (2 * h);
        s.windGradient.m[0][j] += d.x;
        s.windGradient.m[1][j] += d.y;
        s.windGradient.m[2][j] += d.z;
    }
    // The body-averaged wake velocity: the mean of the six probes and the centre.
    s.wind += (mean + induced(positionNed)) / 7.0;
    return s;
}

}  // namespace avd
