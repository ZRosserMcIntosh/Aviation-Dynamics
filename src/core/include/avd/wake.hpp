// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
//
// Wake: the vortex system an aircraft leaves in the air.
//
// The lift of every strip is a bound vortex (Kutta-Joukowski, G = V c cl / 2).
// Each lifting surface sheds its trailing-edge circulation into the air at
// regular intervals as a "gate" of vortex rings (the discrete form of a free
// vortex sheet, as in an unsteady vortex-lattice method). Gates stay where
// they were shed, in the world frame, and from there:
//
//   - roll up: the sheet's trailing vorticity winds into one vortex per
//     contiguous run of same-sign vorticity (Betz) at its centroid, over the
//     Spreiter-Sacks distance 0.28 AR b / CL, spiralling as it goes;
//   - move: with the local wind (boundary layer, turbulence, thermals,
//     microbursts all carry it) plus the velocity the vortices induce on each
//     other: a lifting pair of circulation G0 and spacing b0 sinks at
//     G0 / (2 pi b0) (b0 = pi b / 4 for elliptic loading);
//   - meet the ground: image vortices below a flat ground make a sinking pair
//     slow down and spread apart;
//   - grow and decay: Lamb-Oseen cores grow with Squire's eddy viscosity,
//     rc^2 = rc0^2 + 4 aL (nu + a1 |G|) t; the circulation decays in two
//     phases after Holzapfel's P2P model: slow diffusion, then rapid decay
//     from T2* = 0.8 eps*^-3/4 (in units of t0 = 2 pi b0^2 / G0), with
//     eps* = (eps b0)^(1/3) / w0.
//
// The velocity the wake induces anywhere follows from the regularised
// Biot-Savart law over the ring edges. Older gates are thinned with age, so a
// wake of a few minutes is a few hundred gates. Starting and stopping vortices
// (spanwise shed vorticity, when the lift changes) fall out of the rings.
//
// WakeField wraps an AtmosphereField with one or more wakes, so an aircraft
// flying into a wake (its own after a 360, or another's) feels it through the
// ordinary air sample. Deterministic; no allocation per call after warm-up.
#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <vector>

#include "avd/atmosphere.hpp"
#include "avd/math.hpp"

namespace avd {

class Aircraft;
class AeroModel;
class GroundQuery;

struct WakeParams {
    Real shedSpacing = 0.5;      // gate spacing, fraction of the wing span (time = this * span / speed)
    Real minShedInterval = 0.02; // s
    Real maxShedInterval = 0.25; // s
    Real maxAge = 240;           // s
    Real eddyDissipation = 1e-4; // m^2/s^3 ambient turbulence (1e-4 calm, 1e-2 moderate)
    Real squire = 1e-4;          // a1, Squire eddy-viscosity coefficient
    Real kinematicViscosity = 1.5e-5;  // m^2/s
    Real rolledCore = 0.04;      // initial rolled-up core radius, fraction of b0
    Real updateInterval = 1.0 / 30;    // s, advection step of young gates
    int maxBinsPerSurface = 20;  // trailing-edge bins per surface (strips are merged above this)
};

// One vortex filament segment from a to b; the circulation follows the
// right-hand rule about a -> b.
struct WakeSegment {
    Vec3 a, b;        // local NED (m)
    Real gamma = 0;   // m^2/s
    Real core = 0;    // Lamb-Oseen core radius (m)
    Real age = 0;     // s since shed (0: bound or attached)
};

class Wake {
public:
    explicit Wake(const AeroModel& model, const WakeParams& p = {});
    // A wake with no aircraft: only seeded vortex pairs (tests, scenarios).
    explicit Wake(const WakeParams& p = {}) : p_(p) {}

    const WakeParams& params() const { return p_; }
    void setEddyDissipation(Real eps) { p_.eddyDissipation = eps; }
    void reset();

    // After each aircraft step: records the bound vortices, sheds a gate when
    // due and advances the existing gates to the aircraft's time. `ground` may
    // be null (no ground effect).
    void update(const Aircraft& ac, const AtmosphereField& air, const GroundQuery* ground);
    // Advance the shed gates to `time` without shedding (altitudeOfZeroZ: the
    // MSL altitude of NED z = 0, for the air samples).
    void advanceTo(Real time, const AtmosphereField& air, Real altitudeOfZeroZ = 0);
    // Shed one fully rolled-up pair at `time` (seed a straight pair as a
    // series of these): centre and axis (pointing aft) in local NED, the
    // lateral direction toward the right vortex, spacing b0 and circulation
    // gamma (> 0: a lifting pair, sinking).
    void seedPair(Real time, const Vec3& centre, const Vec3& axis, const Vec3& lateral, Real b0, Real gamma, Real groundZ = 1e9);

    // Velocity the wake induces at a point (local NED, m/s), counting gates at
    // least `minAge` s old (minAge > 0 also leaves out the bound vortices and
    // the attached near wake: what the aircraft's own strip model already
    // accounts for). Includes the ground images.
    Vec3 velocity(const Vec3& p, Real minAge = 0) const;

    // Every vortex segment (bound, attached, shed and ground images), with
    // zero-length and negligible ones left out.
    void segments(std::vector<WakeSegment>& out) const;

    // Summary of the main lifting surface's wake (tests, displays).
    struct PairInfo {
        bool valid = false;
        Real age = 0;
        Real gamma = 0;       // tip-vortex circulation (m^2/s, magnitude)
        Real gamma0 = 0;      // at shedding
        Real spacing = 0;     // b0 (m)
        Real core = 0;        // m
        Real rolled = 0;      // roll-up fraction 0..1
        Vec3 left, right;     // vortex centres, local NED
        Real height = 0;      // above the ground (m), large without ground
    };
    // The main-wing gate closest to `age` seconds old.
    PairInfo pair(Real age) const;
    int gateCount() const { return static_cast<int>(gates_.size()); }
    Real time() const { return time_; }

private:
    static constexpr int kMaxEdges = 24;
    static constexpr int kMaxVortices = 6;

    // Trailing-edge discretisation of one surface (structural frame).
    struct Layout {
        int surface = 0;
        bool main = false;
        int binCount = 0;
        std::array<Vec3, kMaxEdges> qc{}, te{};  // edge points at the quarter chord and trailing edge
        // Strips of each bin (structural strip indices [first, end)), sorted
        // along the lift axis; a bin with first == end is a fuselage bridge
        // carrying the mean of its neighbours.
        std::array<int, kMaxEdges> first{}, end{};
        std::array<std::array<int, 8>, kMaxEdges> strips{};
        std::array<int, kMaxEdges> stripCount{};
        Real span = 0, area = 0, meanChord = 0;
    };
    struct Vortex {
        Vec3 pos;           // local NED
        Real gamma = 0;     // along the gate axis (aft), m^2/s at shedding
        Real coreSq0 = 0;   // initial core radius squared
    };
    struct Gate {
        int layout = 0;
        std::int64_t index = 0;
        Real time = 0;               // shed time
        Real lastUpdate = 0;
        Vec3 axis, ey, ez;           // aft along the flight path, lateral, normal (world)
        int edgeCount = 0;
        std::array<Real, kMaxEdges> g{};          // bin circulation at shedding (along the lift axis)
        std::array<Vec3, kMaxEdges> offset{};     // edge position at shedding minus its vortex
        std::array<Real, kMaxEdges> spin{};       // spiral angle accumulated (rad)
        std::array<std::int8_t, kMaxEdges> owner{};  // vortex index of each edge (-1: none)
        Vec3 anchor;                 // edges with no vortex ride the wind from here
        Real edgeCore = 0.1;         // near-wake (unrolled) core radius
        int vortexCount = 0;
        std::array<Vortex, kMaxVortices> v{};
        Real rollTime = 1;           // s to roll up
        Real b0 = 0, gamma0 = 0;     // main pair spacing and circulation at shedding
        Real t0 = 1, t2 = 5;         // P2P time scale and rapid-decay onset (s)
        Real groundZ = 1e9;          // NED z of the ground below (1e9: none)
        Vec3 lo, hi;                 // bounding box incl. cores (refreshed at each update)
    };
    // The bound and attached part (current aircraft state, world frame).
    struct Attached {
        int layout = 0;
        int edgeCount = 0;
        std::array<Vec3, kMaxEdges> qc{}, te{};
        std::array<Real, kMaxEdges> g{};
        Real core = 0.1;
    };

    void buildLayouts(const AeroModel& m);
    void shed(const Aircraft& ac, Real speed);
    void advance(Gate& gt, Real dt, const AtmosphereField& air, Real altitudeOffset);
    Real decay(const Gate& gt, Real age) const;
    Real rolled(const Gate& gt, Real age) const;
    Real vortexCoreSq(const Gate& gt, const Vortex& v, Real age) const;
    Vec3 edgePosition(const Gate& gt, int e, Real age) const;
    Real edgeCoreAt(const Gate& gt, int e, Real age) const;
    void refreshBox(Gate& gt) const;
    void addGate(Gate& gt);  // newest first, the bounds grown
    // Every segment (gates at least minAge old); with `near`, the rows too far
    // from that point to matter (10 b0) are skipped.
    template <class Emit>
    void forEachSegment(Real minAge, bool images, const Vec3* near, Emit&& emit) const;

    WakeParams p_;
    std::vector<Layout> layouts_;
    std::vector<Attached> attached_;
    std::deque<Gate> gates_;  // newest first
    Real time_ = 0, lastShed = -1e9;
    std::int64_t nextIndex_ = 0;
    Real refSpan_ = 1;
    Real groundZ_ = 1e9;
    Real msl0_ = 0;  // MSL altitude of NED z = 0 (from the last update)
    Vec3 lo_, hi_;          // bounding box of every gate
    Real cullRadius_ = 0;   // 10 b0 of the widest gate
    bool started_ = false;
    Vec3 lastPos_;  // CG at the last update (a jump: a new start)
};

// An air field with wakes in it.
class WakeField final : public AtmosphereField {
public:
    explicit WakeField(const AtmosphereField& base) : base_(&base) {}
    void setBase(const AtmosphereField& base) { base_ = &base; }
    // minAge: gates younger than this are left out (an aircraft's own near
    // wake, already in its strip model).
    void add(const Wake& w, Real minAge) { wakes_.push_back({&w, minAge}); }
    void clear() { wakes_.clear(); }

    AirSample sample(const Vec3& positionNed, Real altitudeMsl, Real time) const override {
        return sample(positionNed, altitudeMsl, time, 0);
    }
    // The wake's gradient is averaged over the body (central differences over
    // a quarter of the extent): a vortex core much smaller than the span rolls
    // the wing through its mean gradient, as the strips see it.
    AirSample sample(const Vec3& positionNed, Real altitudeMsl, Real time, Real extent) const override;

private:
    struct Entry {
        const Wake* wake;
        Real minAge;
    };
    Vec3 induced(const Vec3& p) const;
    const AtmosphereField* base_;
    std::vector<Entry> wakes_;
};

// Velocity induced at p by a straight vortex segment a -> b of circulation
// gamma with a Lamb-Oseen core of radius rc (regularised Biot-Savart).
Vec3 vortexSegmentVelocity(const Vec3& p, const Vec3& a, const Vec3& b, Real gamma, Real rc);

}  // namespace avd
