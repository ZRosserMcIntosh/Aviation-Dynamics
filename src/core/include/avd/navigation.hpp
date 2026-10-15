// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
//
// Navigation environment: the Earth's magnetic field and ground-based radio
// navigation facilities, plus the receiver geometry that turns an aircraft
// position into VOR / localizer / glideslope indications.
//
// No signal is fabricated: a NAV indicator only moves if a facility exists in
// the NavDatabase on the tuned frequency and is within line-of-sight range and
// its service volume. The repository ships only clearly labelled synthetic
// test facilities; real facility data must come from an authoritative source
// (e.g. the national AIP) and is loaded with the same JSON schema.
#pragma once

#include <string>
#include <vector>

#include "avd/geodesy.hpp"
#include "avd/math.hpp"

namespace avd {

// Locally uniform geomagnetic field (adequate over a city/region; a World
// Magnetic Model evaluation can replace it later).
struct MagneticField {
    Real declination = 0;   // rad, east positive (magnetic north east of true north)
    Real inclination = 0;   // rad, positive downward (northern hemisphere)
    Real intensity = 1;     // relative; only the direction matters for a compass
    // Field direction in local NED axes.
    Vec3 directionNed() const;
};

struct Navaid {
    enum class Type { Vor, Localizer, Glideslope };
    Type type = Type::Vor;
    std::string ident, name;
    Real frequencyKhz = 0;      // e.g. 113700 for 113.70 MHz
    Geodetic position;          // antenna (height = antenna elevation MSL, m)
    Real magneticVariation = 0; // rad, the VOR's slaved variation (east positive)
    Real course = 0;            // rad true: localizer inbound course
    Real courseWidth = 0.0873;  // rad, localizer full-scale (both sides) ~5 deg
    Real glidepathAngle = 0.0524;  // rad (3 deg)
    Real rangeNm = 40;          // designated operational coverage
    bool synthetic = false;     // true: test facility that does not exist in reality
    std::string source;         // provenance of the record
};

class NavDatabase {
public:
    void add(const Navaid& n) { navaids_.push_back(n); }
    const std::vector<Navaid>& navaids() const { return navaids_; }
    // JSON: {"navaids": [{"type": "vor"|"localizer"|"glideslope", ...}]}.
    // Returns error messages (empty on success).
    std::vector<std::string> loadJson(const std::string& text);

private:
    std::vector<Navaid> navaids_;
};

struct NavEnvironment {
    MagneticField field;
    const NavDatabase* database = nullptr;
};

// What a NAV receiver + VOR/LOC/GS indicator shows.
struct NavIndication {
    bool valid = false;         // NAV flag hidden (usable signal)
    bool localizer = false;
    Real cdi = 0;               // -1..1 of full scale; + = needle right (course is to the right)
    int toFrom = 0;             // +1 TO, -1 FROM, 0 OFF
    bool glideslopeValid = false;
    Real glideslope = 0;        // -1..1; + = needle up (glidepath above: fly up)
    std::string ident;
    // Truth for debugging/telemetry (not displayed by an analog receiver):
    Real radialMagnetic = 0;    // rad, radial from the VOR the aircraft is on
    Real slantRange = 0;        // m
};

// Evaluate the receiver for a tuned frequency and OBS setting. `aircraft` is
// geodetic (height MSL).
NavIndication evaluateNav(const NavDatabase& db, Real frequencyKhz, Real obsMagnetic, const Geodetic& aircraft);

// Radio line of sight (standard 4/3 Earth): 1.23 (sqrt(h1 ft) + sqrt(h2 ft)) NM.
Real radioHorizonNm(Real heightAMeters, Real heightBMeters);

}  // namespace avd
