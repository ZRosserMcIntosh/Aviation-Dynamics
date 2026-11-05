// Aviation Dynamics — core.
// SPDX-License-Identifier: MIT
#include "avd/taws.hpp"

#include <cmath>
#include <cstring>

#include "avd/units.hpp"

namespace avd {

namespace {
using units::ftToM;
using units::mToFt;
using units::msToFpm;
constexpr Real kG = 9.80665;
constexpr Real kNm = units::kNauticalMile;
constexpr Real kDeg = kPi / 180;
constexpr Real kRaMax = 2500 * units::kFoot;  // the radio altimeter's range

unsigned bit(TawsAlert a) { return 1u << (static_cast<int>(a) - 1); }

struct VoiceInfo {
    TawsVoice voice;
    const char* text;
};
constexpr VoiceInfo kVoices[] = {
    {TawsVoice::None, ""},
    {TawsVoice::PullUp, "PULL UP"},
    {TawsVoice::TerrainAheadPullUp, "TERRAIN AHEAD, PULL UP"},
    {TawsVoice::TerrainTerrain, "TERRAIN, TERRAIN"},
    {TawsVoice::Minimums, "MINIMUMS"},
    {TawsVoice::CautionTerrain, "CAUTION TERRAIN, CAUTION TERRAIN"},
    {TawsVoice::TooLowTerrain, "TOO LOW, TERRAIN"},
    {TawsVoice::TooLowGear, "TOO LOW, GEAR"},
    {TawsVoice::TooLowFlaps, "TOO LOW, FLAPS"},
    {TawsVoice::SinkRate, "SINK RATE"},
    {TawsVoice::DontSink, "DON'T SINK"},
    {TawsVoice::Glideslope, "GLIDESLOPE"},
    {TawsVoice::BankAngle, "BANK ANGLE, BANK ANGLE"},
    {TawsVoice::Callout2500, "TWENTY FIVE HUNDRED"},
    {TawsVoice::Callout1000, "ONE THOUSAND"},
    {TawsVoice::Callout500, "FIVE HUNDRED"},
    {TawsVoice::Callout400, "FOUR HUNDRED"},
    {TawsVoice::Callout300, "THREE HUNDRED"},
    {TawsVoice::Callout200, "TWO HUNDRED"},
    {TawsVoice::Callout100, "ONE HUNDRED"},
    {TawsVoice::Callout50, "FIFTY"},
    {TawsVoice::Callout40, "FORTY"},
    {TawsVoice::Callout30, "THIRTY"},
    {TawsVoice::Callout20, "TWENTY"},
    {TawsVoice::Callout10, "TEN"},
};
static_assert(sizeof(kVoices) / sizeof(kVoices[0]) == static_cast<std::size_t>(TawsVoice::Count));

// How long the phrase takes to say (s): about 15 characters a second.
Real duration(TawsVoice v) { return 0.25 + 0.065 * static_cast<Real>(std::strlen(tawsVoiceText(v))); }

// The voice a repeating alert speaks.
TawsVoice voiceOf(TawsAlert a) {
    switch (a) {
        case TawsAlert::PullUpSinkRate:
        case TawsAlert::PullUpClosure: return TawsVoice::PullUp;
        case TawsAlert::TerrainAheadPullUp: return TawsVoice::TerrainAheadPullUp;
        case TawsAlert::TerrainTerrain: return TawsVoice::TerrainTerrain;
        case TawsAlert::CautionTerrain: return TawsVoice::CautionTerrain;
        case TawsAlert::TooLowTerrain: return TawsVoice::TooLowTerrain;
        case TawsAlert::TooLowGear: return TawsVoice::TooLowGear;
        case TawsAlert::TooLowFlaps: return TawsVoice::TooLowFlaps;
        case TawsAlert::SinkRate: return TawsVoice::SinkRate;
        case TawsAlert::DontSink: return TawsVoice::DontSink;
        case TawsAlert::Glideslope: return TawsVoice::Glideslope;
        default: return TawsVoice::None;  // one-shots: Minimums, AltitudeCallout, BankAngle
    }
}

// Linear interpolation of y over x through two points, unclamped.
Real line(Real x, Real x0, Real y0, Real x1, Real y1) { return y0 + (x - x0) * (y1 - y0) / (x1 - x0); }
}  // namespace

int tawsAlertLevel(TawsAlert a) {
    switch (a) {
        case TawsAlert::None: return 0;
        case TawsAlert::PullUpSinkRate:
        case TawsAlert::PullUpClosure:
        case TawsAlert::TerrainAheadPullUp:
        case TawsAlert::TerrainTerrain: return 3;
        case TawsAlert::Minimums:
        case TawsAlert::AltitudeCallout: return 1;
        default: return 2;
    }
}

const char* tawsAlertText(TawsAlert a) {
    switch (a) {
        case TawsAlert::PullUpSinkRate: return "PULL UP";
        case TawsAlert::PullUpClosure: return "PULL UP";
        case TawsAlert::TerrainAheadPullUp: return "TERRAIN AHEAD, PULL UP";
        case TawsAlert::TerrainTerrain: return "TERRAIN TERRAIN";
        case TawsAlert::Minimums: return "MINIMUMS";
        case TawsAlert::CautionTerrain: return "CAUTION TERRAIN";
        case TawsAlert::TooLowTerrain: return "TOO LOW TERRAIN";
        case TawsAlert::AltitudeCallout: return "ALTITUDE CALLOUT";
        case TawsAlert::TooLowGear: return "TOO LOW GEAR";
        case TawsAlert::TooLowFlaps: return "TOO LOW FLAPS";
        case TawsAlert::SinkRate: return "SINK RATE";
        case TawsAlert::DontSink: return "DON'T SINK";
        case TawsAlert::Glideslope: return "GLIDESLOPE";
        case TawsAlert::BankAngle: return "BANK ANGLE";
        default: return "";
    }
}

const char* tawsVoiceText(TawsVoice v) {
    const int i = static_cast<int>(v);
    return i >= 0 && i < static_cast<int>(TawsVoice::Count) ? kVoices[i].text : "";
}

TawsVoice tawsCalloutVoice(int feet) {
    switch (feet) {
        case 2500: return TawsVoice::Callout2500;
        case 1000: return TawsVoice::Callout1000;
        case 500: return TawsVoice::Callout500;
        case 400: return TawsVoice::Callout400;
        case 300: return TawsVoice::Callout300;
        case 200: return TawsVoice::Callout200;
        case 100: return TawsVoice::Callout100;
        case 50: return TawsVoice::Callout50;
        case 40: return TawsVoice::Callout40;
        case 30: return TawsVoice::Callout30;
        case 20: return TawsVoice::Callout20;
        case 10: return TawsVoice::Callout10;
        default: return TawsVoice::None;
    }
}

void TawsMonitor::setClass(TawsClass c) {
    class_ = c;
    if (c == TawsClass::A) callouts_ = {1000, 500, 100, 50, 40, 30, 20, 10};
    if (c == TawsClass::B) callouts_ = {500};
    reset();
}

void TawsMonitor::reset() {
    st_ = TawsStatus{};
    time_ = 0;
    started_ = false;
    wasOnGround_ = true;
    raWasValid_ = false;
    raPrev_ = altMax_ = raMaxTakeoff_ = 0;
    raMinApproach_ = 1e9;
    configuredAtStart_ = false;
    mode2Time_ = 0;
    mode2B_ = false;
    calloutArmed_.clear();
    dhArmed_ = true;
    minimumsAt_ = calloutAt_ = -1e9;
    bankStep_ = 0;
    fltaTimer_ = 0;
    fltaCaution_ = fltaWarning_ = false;
    warningUntil_ = cautionUntil_ = -1;
    voiceEnd_ = 0;
    voiceLevel_ = 0;
    due_.fill(0);
    prevActive_ = 0;
    pending_.clear();
}

bool TawsMonitor::nearRunway(const Vec3& p, Real margin, Real* heightAbove) const {
    bool any = false;
    for (const TawsRunway& r : runways_) {
        const Real c = std::cos(r.heading), s = std::sin(r.heading);
        const Real dn = p.x - r.threshold.x, de = p.y - r.threshold.y;
        const Real along = dn * c + de * s, lateral = -dn * s + de * c;
        // Its approach and departure paths: 2 NM before it to 2 NM past its end, 600 m either side.
        if (along < -2 * kNm - margin || along > r.length + 2 * kNm + margin || std::abs(lateral) > 600 + margin) continue;
        const Real h = r.threshold.z - p.z;
        if (heightAbove) *heightAbove = any ? std::min(*heightAbove, h) : h;
        any = true;
    }
    return any;
}

void TawsMonitor::oneShot(TawsVoice v, TawsAlert a) {
    // A newer callout replaces any still waiting (the older is stale by now).
    if (a == TawsAlert::AltitudeCallout)
        pending_.erase(std::remove_if(pending_.begin(), pending_.end(), [](const Pending& p) { return p.alert == TawsAlert::AltitudeCallout; }),
                       pending_.end());
    pending_.push_back({v, a, time_});
}

void TawsMonitor::update(const TawsInputs& in, const GroundQuery& ground) {
    if (class_ == TawsClass::Off) return;
    const Real dt = std::max(in.dt, 0.0);
    time_ += dt;
    if (!in.powered) {
        const unsigned count = st_.voiceCount;
        reset();
        st_.voiceCount = count;
        return;
    }
    const bool classA = class_ == TawsClass::A;

    // ---- Radio altimeter ----
    const Real ra = in.heightAgl - raZero_;
    const bool raValid = in.heightAgl < 1e8 && ra <= kRaMax;
    const bool prevValid = raWasValid_;
    const Real prevRa = raPrev_;
    if (raValid && raWasValid_ && dt > 0) {
        const Real rate = -(ra - raPrev_) / dt;
        st_.closureRate += (rate - st_.closureRate) * std::min(dt / 1.0, 1.0);
    } else {
        st_.closureRate = 0;
    }
    raPrev_ = ra;
    raWasValid_ = raValid;
    st_.radioAltitudeValid = raValid;
    st_.radioAltitude = raValid ? ra : 0;
    const Real raFt = mToFt(ra), prevFt = mToFt(prevRa);
    const Real alt = -in.position.z, vs = -in.velocity.z;
    const Real sinkFpm = -msToFpm(vs), casKt = units::msToKt(in.calibratedAirspeed);

    // ---- Phase of flight: the take-off phase from lift-off (or a go-around) ----
    if (!started_) {
        started_ = true;
        wasOnGround_ = in.onGround;
    }
    bool& takeoff = st_.takeoffPhase;
    const bool configured = in.gearDown && in.landingFlaps;
    if (wasOnGround_ && !in.onGround) {
        takeoff = true;
        altMax_ = alt;
        raMaxTakeoff_ = 0;
        configuredAtStart_ = configured;
    }
    wasOnGround_ = in.onGround;
    if (in.onGround) {
        takeoff = false;
        raMinApproach_ = 1e9;
    } else if (takeoff && (!raValid || raFt > 1500 || (configured && !configuredAtStart_))) {
        takeoff = false;  // climbed out, or configured to land again (a circuit)
        raMinApproach_ = 1e9;
    } else if (!takeoff && raValid) {
        // A go-around: climbing 100 ft back up after coming below 200 ft.
        raMinApproach_ = std::min(raMinApproach_, ra);
        if (raMinApproach_ < ftToM(200) && ra > raMinApproach_ + ftToM(100) && vs > 0) {
            takeoff = true;
            altMax_ = alt;
            raMaxTakeoff_ = ra;
            raMinApproach_ = 1e9;
            configuredAtStart_ = configured;
        }
    }
    if (takeoff && !configured) configuredAtStart_ = false;  // cleaned up: the next landing configuration ends it
    if (takeoff) {
        altMax_ = std::max(altMax_, alt);
        if (raValid) raMaxTakeoff_ = std::max(raMaxTakeoff_, ra);
    }
    st_.altitudeLoss = takeoff ? altMax_ - alt : 0;

    unsigned act = 0;
    auto set = [&](TawsAlert a) { act |= bit(a); };
    const bool air = !in.onGround;
    st_.glideslopeLoud = false;
    if (air && raValid) {
        // Mode 1: excessive descent rate.
        if (raFt >= 10 && raFt <= 2450) {
            if (sinkFpm > 998 && raFt < line(sinkFpm, 998, 10, 5007, 2450)) set(TawsAlert::SinkRate);
            if (sinkFpm > 1710 && raFt < line(sinkFpm, 1710, 10, 7125, 2450)) set(TawsAlert::PullUpSinkRate);
        }
        // Mode 2: excessive terrain closure rate.
        if (classA) {
            const Real c = msToFpm(st_.closureRate);
            bool inside = false;
            mode2B_ = in.landingFlaps;
            if (raFt >= 30 && c > 2000) {
                if (mode2B_) {
                    inside = raFt < (c < 3000 ? line(c, 2000, 30, 3000, 789) : 789.0);
                } else {
                    const Real upper = 1650 + 800 * clamp((casKt - 220) / 90, 0.0, 1.0);
                    inside = raFt < (c < 3450 ? line(c, 2000, 30, 3450, 1250) : std::min(upper, line(c, 3450, 1250, 5000, upper)));
                }
            }
            if (inside) {
                mode2Time_ += dt;
                set(!mode2B_ && mode2Time_ > 1.6 ? TawsAlert::PullUpClosure : TawsAlert::TerrainTerrain);
            } else {
                mode2Time_ = 0;
            }
        }
        // Mode 3: altitude loss after take-off.
        if (takeoff && raFt >= 30 && raFt <= 1500 && mToFt(st_.altitudeLoss) > line(raFt, 30, 10, 1500, 143)) set(TawsAlert::DontSink);
        // Mode 4: unsafe terrain clearance.
        if (classA && raFt >= 30) {
            if (takeoff) {
                if (!(in.gearDown && in.landingFlaps) && raFt < std::min(0.75 * mToFt(raMaxTakeoff_), 500.0)) set(TawsAlert::TooLowTerrain);
            } else if (!in.gearDown) {
                if (casKt < 190) {
                    if (raFt < 500) set(TawsAlert::TooLowGear);
                } else if (raFt < 500 + 500 * std::min((casKt - 190) / 60, 1.0)) {
                    set(TawsAlert::TooLowTerrain);
                }
            } else if (!in.landingFlaps) {
                if (casKt < 159) {
                    if (raFt < 245) set(TawsAlert::TooLowFlaps);
                } else if (raFt < 245 + 755 * std::min((casKt - 159) / 91, 1.0)) {
                    set(TawsAlert::TooLowTerrain);
                }
            }
        }
        // Mode 5: below the glideslope.
        if (classA && in.gearDown && in.glideslopeValid && !takeoff && raFt >= 30 && raFt <= 1000) {
            const Real dots = 2 * in.glideslope;  // + below the path
            const Real soft = raFt > 150 ? 1.3 : line(raFt, 150, 1.3, 30, 2.0);
            if (dots > soft) set(TawsAlert::Glideslope);
            if (raFt < 300 && dots >= 1.98) {
                set(TawsAlert::Glideslope);
                st_.glideslopeLoud = true;
            }
        }
        // Mode 6: the callouts on the way down, and the decision height.
        if (calloutArmed_.size() != callouts_.size()) calloutArmed_.assign(callouts_.size(), true);
        for (std::size_t i = 0; i < callouts_.size(); ++i) {
            const Real c = callouts_[i];
            if (raFt > 1.1 * c + 30) {
                calloutArmed_[i] = true;
            } else if (calloutArmed_[i] && prevValid && prevFt >= c && raFt < c) {
                calloutArmed_[i] = false;
                if (!classA || in.gearDown) {
                    oneShot(tawsCalloutVoice(callouts_[i]), TawsAlert::AltitudeCallout);
                    calloutAt_ = time_;
                }
            }
        }
        if (classA && dh_ > 0) {
            if (ra > 1.1 * dh_ + ftToM(30)) {
                dhArmed_ = true;
            } else if (dhArmed_ && prevValid && prevRa >= dh_ && ra < dh_) {
                dhArmed_ = false;
                oneShot(TawsVoice::Minimums, TawsAlert::Minimums);
                minimumsAt_ = time_;
            }
        }
    } else {
        mode2Time_ = 0;
    }
    // The advisories show for a moment after they are called.
    if (time_ - minimumsAt_ < 3) set(TawsAlert::Minimums);
    if (time_ - calloutAt_ < 1) set(TawsAlert::AltitudeCallout);
    // Bank angle (also above the radio altimeter's range).
    if (air) {
        const Real limit = raValid && raFt < 130 ? (raFt < 5 ? 1e9 : (10 + (raFt - 5) * 25 / 125) * kDeg) : 35 * kDeg;
        const Real bank = std::abs(in.roll);
        if (bank > limit) {
            set(TawsAlert::BankAngle);
            const int step = 1 + static_cast<int>((bank - limit) / (5 * kDeg));
            if (step > bankStep_) {
                bankStep_ = step;
                oneShot(TawsVoice::BankAngle, TawsAlert::BankAngle);
            }
        } else if (bank < limit - 2 * kDeg) {
            bankStep_ = 0;
        }
    }
    // Forward-looking terrain avoidance, once a second.
    if (air && !inhibit_) {
        fltaTimer_ -= dt;
        if (fltaTimer_ <= 0) {
            fltaTimer_ = 1.0;
            flta(in, ground);
            // Each alert holds for 3 s after the search last found it (no flicker between the searches).
            if (fltaWarning_) warningUntil_ = time_ + 3;
            if (fltaCaution_) cautionUntil_ = time_ + 3;
        }
        if (time_ < warningUntil_) set(TawsAlert::TerrainAheadPullUp);
        else if (time_ < cautionUntil_) set(TawsAlert::CautionTerrain);
    } else {
        fltaCaution_ = fltaWarning_ = false;
        warningUntil_ = cautionUntil_ = -1;
        fltaTimer_ = 0;
        st_.terrainTime = -1;
    }

    st_.active = act;
    st_.top = TawsAlert::None;
    for (int a = 1; a < static_cast<int>(TawsAlert::Count); ++a)
        if (act & bit(static_cast<TawsAlert>(a))) {
            st_.top = static_cast<TawsAlert>(a);
            break;
        }
    st_.level = tawsAlertLevel(st_.top);
    speak(dt);
}

void TawsMonitor::flta(const TawsInputs& in, const GroundQuery& ground) {
    fltaCaution_ = fltaWarning_ = false;
    st_.terrainTime = -1;
    const Vec3& p = in.position;
    const Vec3& v = in.velocity;
    const Real gs = std::hypot(v.x, v.y);
    // Phase and the required terrain clearance.
    Real hRunway = 0;
    const bool onRunwayPath = nearRunway(p, 0, &hRunway);
    Real nearest = 1e18, hNearest = 0;
    for (const TawsRunway& r : runways_) {
        const Real d = std::hypot(p.x - r.threshold.x, p.y - r.threshold.y);
        if (d < nearest) nearest = d, hNearest = r.threshold.z - p.z;
    }
    TawsPhase phase = TawsPhase::Enroute;
    if (st_.takeoffPhase) phase = TawsPhase::Departure;
    else if (nearest < 5 * kNm && hNearest < ftToM(1900)) phase = TawsPhase::Approach;
    else if (nearest < 15 * kNm || (runways_.empty() && in.gearDown)) phase = TawsPhase::Terminal;
    st_.phase = phase;
    const bool descending = -v.z < -units::fpmToMs(300);
    constexpr Real kRtc[4][2] = {{700, 500}, {350, 300}, {150, 100}, {100, 100}};  // ft: level, descending
    const Real rtc = ftToM(kRtc[static_cast<int>(phase)][descending ? 1 : 0]);
    st_.requiredClearance = rtc;
    // Quiet on a runway's approach and departure paths close to it, and too slow to matter.
    if ((onRunwayPath && hRunway < ftToM(1000)) || gs < 15) return;

    // The ground ahead along the track: the highest of the centre line and
    // either side, every 60 m (at most every second) for 60 s; runways left out.
    constexpr Real kLook = 60;
    const Real step = clamp(60 / gs, 0.25, 1.0);
    const int n = static_cast<int>(std::ceil(kLook / step));
    const Vec3 dir{v.x / gs, v.y / gs, 0}, left{-dir.y, dir.x, 0};
    const Real zTop = std::min(p.z, p.z + v.z * kLook) - 2000;  // probe from well above the path
    std::vector<Real> terrain(static_cast<std::size_t>(n) + 1, -1e9);
    for (int k = 1; k <= n; ++k) {
        const Real d = gs * k * step;
        const Vec3 c{p.x + dir.x * d, p.y + dir.y * d, zTop};
        if (nearRunway(c, 0, nullptr)) continue;
        const Real half = 30 + 0.05 * d;
        Real top = -1e9;
        for (int side = -1; side <= 1; ++side) {
            const GroundHit h = ground.below(c + left * (side * half));
            if (h.hit) top = std::max(top, -h.point.z);
        }
        terrain[static_cast<std::size_t>(k)] = top;
    }
    const Real alt0 = -p.z, vs = -v.z;
    for (int k = 1; k <= n; ++k)
        if (alt0 + vs * k * step - terrain[static_cast<std::size_t>(k)] < rtc) {
            st_.terrainTime = k * step;
            break;
        }
    // The escape: the present path for `delay` s, then a pull-up at
    // escapeLoad g to the better of the present climb and escapeClimb.
    const Real speed = std::max(length(v), 1.0);
    const Real gamma0 = std::atan2(vs, gs), gammaEscape = std::max(gamma0, escapeClimb_);
    const Real pitchRate = escapeLoad_ * kG / speed;
    auto fails = [&](Real delay, Real floor) {
        Real a = alt0, gam = gamma0;
        for (int k = 1; k <= n; ++k) {
            if (k * step > delay) gam = std::min(gam + pitchRate * step, gammaEscape);
            a += gs * std::tan(gam) * step;
            if (a - terrain[static_cast<std::size_t>(k)] < floor) return true;
        }
        return false;
    };
    fltaWarning_ = fails(15, 0.5 * rtc);
    fltaCaution_ = !fltaWarning_ && fails(30, rtc);
}

void TawsMonitor::speak(Real dt) {
    (void)dt;
    const unsigned act = st_.active;
    // A repeating alert speaks as it comes on.
    for (int a = 1; a < static_cast<int>(TawsAlert::Count); ++a) {
        const unsigned b = bit(static_cast<TawsAlert>(a));
        if ((act & b) && !(prevActive_ & b)) due_[static_cast<std::size_t>(a)] = time_;
    }
    prevActive_ = act;
    // Callouts more than 1.5 s late are dropped (the height has gone by).
    pending_.erase(std::remove_if(pending_.begin(), pending_.end(), [&](const Pending& q) { return time_ - q.time > 1.5; }), pending_.end());
    // The highest-priority phrase due; while a warning is on, nothing below it.
    int ceiling = static_cast<int>(TawsAlert::Count);
    for (int a = 1; a < ceiling; ++a)
        if ((act & bit(static_cast<TawsAlert>(a))) && tawsAlertLevel(static_cast<TawsAlert>(a)) == 3) {
            ceiling = a + 1;
            break;
        }
    TawsAlert best = TawsAlert::None;
    TawsVoice bestVoice = TawsVoice::None;
    int bestPending = -1;
    for (int a = 1; a < ceiling; ++a) {
        const TawsAlert al = static_cast<TawsAlert>(a);
        const TawsVoice v = voiceOf(al);
        if (v == TawsVoice::None || !(act & bit(al)) || due_[static_cast<std::size_t>(a)] > time_) continue;
        best = al, bestVoice = v;
        break;
    }
    for (std::size_t i = 0; i < pending_.size(); ++i)
        if (static_cast<int>(pending_[i].alert) < ceiling &&
            (best == TawsAlert::None || static_cast<int>(pending_[i].alert) < static_cast<int>(best))) {
            best = pending_[i].alert, bestVoice = pending_[i].voice, bestPending = static_cast<int>(i);
        }
    if (time_ < voiceEnd_) {
        // Only a warning cuts in on a lesser phrase.
        if (best == TawsAlert::None || tawsAlertLevel(best) < 3 || voiceLevel_ >= 3) return;
    }
    if (best == TawsAlert::None) {
        if (time_ >= voiceEnd_) st_.voice = TawsVoice::None;
        return;
    }
    st_.voice = bestVoice;
    ++st_.voiceCount;
    voiceEnd_ = time_ + duration(bestVoice);
    voiceLevel_ = tawsAlertLevel(best);
    if (bestPending >= 0) {
        pending_.erase(pending_.begin() + bestPending);
        return;
    }
    // When it may speak again (after the phrase ends).
    Real gap = 1.0;
    switch (best) {
        case TawsAlert::PullUpSinkRate:
        case TawsAlert::PullUpClosure: gap = 0.2; break;
        case TawsAlert::TerrainAheadPullUp: gap = 0.3; break;
        case TawsAlert::TerrainTerrain: gap = mode2B_ ? 1.0 : 1e9; break;  // 2A: once, then PULL UP
        case TawsAlert::CautionTerrain: gap = 6.0; break;
        case TawsAlert::Glideslope: gap = st_.glideslopeLoud ? 0.5 : 2.5; break;
        default: break;
    }
    due_[static_cast<std::size_t>(best)] = voiceEnd_ + gap;
}

}  // namespace avd
