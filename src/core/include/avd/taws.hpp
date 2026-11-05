// Aviation Dynamics — core.
// SPDX-License-Identifier: MIT
//
// Terrain awareness and warning (TAWS, TSO-C151c) with the classic ground
// proximity warning modes (GPWS, RTCA DO-161A): what the box in the avionics
// bay computes from the radio altimeter, the air data, the gear and flap
// positions, the glideslope receiver and a terrain database, and the voice it
// speaks. Envelopes after DO-161A and the published Honeywell MK V / MK VIII
// EGPWS ones (Product Specification 965-0976-xxx and the pilot's guides),
// read off their figures: approximate, and simplified where noted.
//
// - Mode 1, excessive descent rate: SINK RATE when the descent rate is past
//   a line from 998 fpm at 10 ft to 5,007 fpm at 2,450 ft of radio altitude,
//   PULL UP past 1,710 fpm at 10 ft to 7,125 fpm at 2,450 ft.
// - Mode 2, excessive terrain closure rate (the radio altitude's rate of
//   change, filtered over 1 s): 2A, flaps not in the landing position:
//   TERRAIN TERRAIN on entering a boundary from 2,000 fpm at 30 ft through
//   3,450 fpm at 1,250 ft to an upper limit at 5,000 fpm of 1,650 ft (rising
//   to 2,450 ft between 220 and 310 kt), then PULL UP while inside; 2B,
//   landing flaps: TERRAIN TERRAIN inside 2,000 fpm at 30 ft to 3,000 fpm at
//   789 ft, no higher.
// - Mode 3, altitude loss after take-off or a go-around, until 1,500 ft:
//   DON'T SINK when the barometric altitude lost since the highest point
//   passes 10 ft at 30 ft rising to 143 ft at 1,500 ft.
// - Mode 4, unsafe terrain clearance, outside the take-off phase: 4A, gear
//   not down: TOO LOW GEAR below 500 ft under 190 kt (TOO LOW TERRAIN above
//   it, the floor rising to 1,000 ft at 250 kt); 4B, gear down, flaps not in
//   the landing position: TOO LOW FLAPS below 245 ft under 159 kt (TOO LOW
//   TERRAIN above, the floor rising to 1,000 ft at 250 kt); 4C, in the
//   take-off phase without landing gear and flaps: TOO LOW TERRAIN when the
//   radio altitude falls below 75% of its highest since lift-off (at most
//   500 ft). All from 30 ft.
// - Mode 5, below the glideslope, gear down, an ILS glideslope received,
//   1,000 ft to 30 ft: GLIDESLOPE (soft) beyond 1.3 dots below (2.0 dots at
//   30 ft), loud below 300 ft at 2 dots (the needle pegged). A dot is 0.35 deg.
// - Mode 6, callouts on the way down (re-armed 10% and 30 ft above each):
//   the radio altitudes in `callouts`, MINIMUMS at the decision height; BANK
//   ANGLE beyond 35 deg (10 deg at 5 ft rising to 35 deg at 130 ft), again
//   at every 5 deg more.
// - Forward-looking terrain avoidance (FLTA): once a second the ground is
//   probed (GroundQuery::below, so the region's terrain grids and the
//   buildings as obstacles) along the track ahead for 60 s, on the centre
//   line and to either side (30 m + 5% of the distance), and two escape
//   manoeuvres are flown against it on paper: the present flight path for a
//   delay, then a pull-up (incremental `escapeLoad` g) to the better of the
//   present climb and `escapeClimb`. CAUTION TERRAIN if one begun in 30 s
//   would come within the required terrain clearance (RTC) of the ground;
//   TERRAIN AHEAD, PULL UP if one begun in 15 s would come within half of
//   it; each holds for 3 s after the search last found it. RTC (TSO-C151c's
//   FLTA test values) by phase, level / descending faster than 300 fpm: en
//   route 700 / 500 ft, terminal (within 15 NM of a runway) 350 / 300 ft,
//   approach (within 5 NM and below 1,900 ft above it) 150 / 100 ft,
//   departure (the take-off phase) 100 / 100 ft. The
//   runways (setRunways) are left out of the search, and FLTA is quiet over
//   their approach and departure paths (2 NM out, 600 m either side, below
//   1,000 ft above them). Straight track only (no turn prediction).
//
// Class A (the jets) has every mode; Class B (TSO-C151c's, light aircraft)
// Modes 1 and 3, the 500 ft callout, BANK ANGLE and FLTA. Priority, highest
// first (after TSO-C151c's alert prioritisation): the order of TawsAlert.
// Not modelled: the premature descent alert, the terrain display, windshear,
// Mode 2's terrain-clearance latch after leaving the envelope, steep-approach
// biasing, the glideslope cancel switch.
//
// The radio altimeter is the aircraft's height above the ground below
// (AircraftTelemetry::heightAgl: the region's terrain grids, the roofs of the
// buildings) less its installation zero, the reading with the wheels on the
// ground; it has no reading above 2,500 ft.
#pragma once

#include <algorithm>
#include <array>
#include <utility>
#include <vector>

#include "avd/ground.hpp"
#include "avd/math.hpp"

namespace avd {

enum class TawsClass { Off = 0, B = 1, A = 2 };

// The alerts, highest priority first. Bit (1 << (alert - 1)) in TawsStatus::active.
enum class TawsAlert : int {
    None = 0,
    PullUpSinkRate,      // Mode 1 PULL UP (warning)
    PullUpClosure,       // Mode 2A PULL UP (warning)
    TerrainAheadPullUp,  // FLTA warning
    TerrainTerrain,      // Mode 2 TERRAIN TERRAIN (warning)
    Minimums,            // Mode 6 decision height (advisory)
    CautionTerrain,      // FLTA caution
    TooLowTerrain,       // Mode 4 (caution)
    AltitudeCallout,     // Mode 6 (advisory)
    TooLowGear,          // Mode 4A (caution)
    TooLowFlaps,         // Mode 4B (caution)
    SinkRate,            // Mode 1 (caution)
    DontSink,            // Mode 3 (caution)
    Glideslope,          // Mode 5 (caution)
    BankAngle,           // Mode 6 (caution)
    Count
};
// 3 warning (red, PULL UP), 2 caution (amber), 1 advisory (callouts), 0 none.
int tawsAlertLevel(TawsAlert a);
const char* tawsAlertText(TawsAlert a);  // e.g. "TOO LOW GEAR"

// What the voice says.
enum class TawsVoice : int {
    None = 0, PullUp, TerrainAheadPullUp, TerrainTerrain, Minimums, CautionTerrain, TooLowTerrain,
    TooLowGear, TooLowFlaps, SinkRate, DontSink, Glideslope, BankAngle,
    Callout2500, Callout1000, Callout500, Callout400, Callout300, Callout200, Callout100,
    Callout50, Callout40, Callout30, Callout20, Callout10, Count
};
const char* tawsVoiceText(TawsVoice v);  // e.g. "TERRAIN AHEAD, PULL UP"
// The callout voice for a radio altitude in feet (None if there is none).
TawsVoice tawsCalloutVoice(int feet);

// A runway the terrain database knows (left out of the FLTA search).
struct TawsRunway {
    Vec3 threshold;    // NED of the runway end; its elevation is -threshold.z
    Real heading = 0;  // rad true, along the runway from `threshold`
    Real length = 0;   // m
};

// What the box reads each step.
struct TawsInputs {
    Real dt = 0;
    bool powered = true;
    bool onGround = false;       // weight on wheels
    Vec3 position, velocity;     // NED (m, m/s): the GPS's
    Real heightAgl = 1e9;        // the radio altimeter antenna's height above the ground (m; >= 1e8: no return)
    Real calibratedAirspeed = 0; // m/s
    Real roll = 0;               // rad
    bool gearDown = true;
    bool landingFlaps = false;
    bool glideslopeValid = false;
    Real glideslope = 0;         // needle, -1..1 of full scale (2 dots), + fly up (below the path)
};

enum class TawsPhase { Enroute = 0, Terminal = 1, Approach = 2, Departure = 3 };

struct TawsStatus {
    unsigned active = 0;               // bit (1 << (alert - 1)) per active TawsAlert
    TawsAlert top = TawsAlert::None;   // the highest-priority active alert
    int level = 0;                     // its level (tawsAlertLevel)
    TawsVoice voice = TawsVoice::None; // the phrase being spoken now
    unsigned voiceCount = 0;           // phrases begun: a new value means "say `voice`"
    bool radioAltitudeValid = false;
    Real radioAltitude = 0;            // m (the wheels' height above the ground)
    Real closureRate = 0;              // m/s, + the ground coming up
    Real altitudeLoss = 0;             // m since the highest point (Mode 3, take-off phase)
    bool takeoffPhase = false;
    bool glideslopeLoud = false;       // Mode 5's hard alert
    TawsPhase phase = TawsPhase::Enroute;
    Real terrainTime = -1;             // s until the present path comes within the RTC (FLTA; -1 none)
    Real requiredClearance = 0;        // m, the RTC in force
    bool isActive(TawsAlert a) const { return a != TawsAlert::None && (active & (1u << (static_cast<int>(a) - 1))) != 0; }
};

class TawsMonitor {
public:
    // Off clears the status; A and B set their callouts.
    void setClass(TawsClass c);
    TawsClass tawsClass() const { return class_; }
    // The radio altimeter's reading with the wheels on the ground (subtracted from heightAgl), m.
    void setRadioAltimeterZero(Real m) { raZero_ = m; }
    Real radioAltimeterZero() const { return raZero_; }
    // Decision height for MINIMUMS (radio altitude, m; 0 none). Default 200 ft.
    void setDecisionHeight(Real m) { dh_ = std::max(m, 0.0); }
    Real decisionHeight() const { return dh_; }
    // Radio altitudes called out on the way down (ft).
    void setCallouts(std::vector<int> feet) { callouts_ = std::move(feet); }
    const std::vector<int>& callouts() const { return callouts_; }
    // The terrain-inhibit switch: FLTA off (the GPWS modes stay).
    void setTerrainInhibit(bool on) { inhibit_ = on; }
    bool terrainInhibit() const { return inhibit_; }
    void setRunways(std::vector<TawsRunway> r) { runways_ = std::move(r); }
    const std::vector<TawsRunway>& runways() const { return runways_; }
    // The flap detent from which the flaps count as in the landing position
    // (-1: the last). The Aircraft reads it; the monitor takes `landingFlaps`.
    void setLandingFlapDetent(int d) { landingFlapDetent_ = d; }
    int landingFlapDetent() const { return landingFlapDetent_; }
    // Escape manoeuvre for FLTA: incremental load factor (g) and climb angle (rad).
    void setEscape(Real loadG, Real climb) { escapeLoad_ = loadG, escapeClimb_ = climb; }

    void update(const TawsInputs& in, const GroundQuery& ground);
    // A new flight (a spawn, a reset): the modes' memories start over (the configuration is kept).
    void reset();
    const TawsStatus& status() const { return st_; }

private:
    void flta(const TawsInputs& in, const GroundQuery& ground);
    void speak(Real dt);
    void oneShot(TawsVoice v, TawsAlert a);
    bool nearRunway(const Vec3& p, Real margin, Real* heightAbove) const;

    TawsClass class_ = TawsClass::Off;
    Real raZero_ = 0, dh_ = 60.96;
    std::vector<int> callouts_;
    bool inhibit_ = false;
    int landingFlapDetent_ = -1;
    std::vector<TawsRunway> runways_;
    Real escapeLoad_ = 0.5, escapeClimb_ = 0.105;  // 0.5 g, 6 deg

    TawsStatus st_;
    Real time_ = 0;
    bool started_ = false, wasOnGround_ = true, raWasValid_ = false;
    Real raPrev_ = 0;
    Real altMax_ = 0;          // Mode 3: highest altitude in the take-off phase (m, up)
    Real raMaxTakeoff_ = 0;    // Mode 4C
    Real raMinApproach_ = 1e9; // go-around detection
    bool configuredAtStart_ = false;  // gear and flaps for landing when the take-off phase began
    Real mode2Time_ = 0;       // s inside the Mode 2A envelope
    bool mode2B_ = false;
    std::vector<bool> calloutArmed_;
    bool dhArmed_ = true;
    Real minimumsAt_ = -1e9, calloutAt_ = -1e9;  // when they were called
    int bankStep_ = 0;         // 5 deg steps already announced past the limit
    Real fltaTimer_ = 0;
    bool fltaCaution_ = false, fltaWarning_ = false;   // the last search's findings
    Real warningUntil_ = -1, cautionUntil_ = -1;      // held 3 s after each
    // The voice.
    Real voiceEnd_ = 0;
    int voiceLevel_ = 0;
    std::array<Real, static_cast<int>(TawsAlert::Count)> due_{};  // next time each repeating alert may speak
    unsigned prevActive_ = 0;
    struct Pending {
        TawsVoice voice = TawsVoice::None;
        TawsAlert alert = TawsAlert::None;
        Real time = 0;
    };
    std::vector<Pending> pending_;
};

}  // namespace avd
