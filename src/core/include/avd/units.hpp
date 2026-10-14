// Aviation Dynamics — engine-independent flight simulation core.
// SPDX-License-Identifier: MIT
//
// Aviation unit conversions. The core is SI internally; these are used only at
// boundaries (data files, instruments, reports). Exact definitions:
// ft = 0.3048 m, NM = 1852 m, lb = 0.45359237 kg, US gal = 3.785411784 L,
// inHg (0 degC) = 3386.389 Pa, hp (mechanical) = 745.69987 W.
#pragma once

#include "avd/math.hpp"

namespace avd::units {

inline constexpr Real kFoot = 0.3048;
inline constexpr Real kInch = 0.0254;
inline constexpr Real kNauticalMile = 1852.0;
inline constexpr Real kKnot = 1852.0 / 3600.0;
inline constexpr Real kPound = 0.45359237;
inline constexpr Real kPoundForce = kPound * kStandardGravity;
inline constexpr Real kUsGallon = 3.785411784e-3;  // m^3
inline constexpr Real kInHg = 3386.389;
inline constexpr Real kHectoPascal = 100.0;
inline constexpr Real kHorsepower = 745.69987158227022;
inline constexpr Real kSlug = kPoundForce / kFoot;
inline constexpr Real kSlugFt2 = kSlug * kFoot * kFoot;

constexpr Real ftToM(Real ft) { return ft * kFoot; }
constexpr Real mToFt(Real m) { return m / kFoot; }
constexpr Real inToM(Real in) { return in * kInch; }
constexpr Real ktToMs(Real kt) { return kt * kKnot; }
constexpr Real msToKt(Real ms) { return ms / kKnot; }
constexpr Real fpmToMs(Real fpm) { return fpm * kFoot / 60.0; }
constexpr Real msToFpm(Real ms) { return ms * 60.0 / kFoot; }
constexpr Real lbToKg(Real lb) { return lb * kPound; }
constexpr Real kgToLb(Real kg) { return kg / kPound; }
constexpr Real galToM3(Real gal) { return gal * kUsGallon; }
constexpr Real m3ToGal(Real m3) { return m3 / kUsGallon; }
constexpr Real hpToW(Real hp) { return hp * kHorsepower; }
constexpr Real wToHp(Real w) { return w / kHorsepower; }
constexpr Real inHgToPa(Real inHg) { return inHg * kInHg; }
constexpr Real paToInHg(Real pa) { return pa / kInHg; }
constexpr Real degToRad(Real d) { return d * (kPi / 180.0); }
constexpr Real radToDeg(Real r) { return r * (180.0 / kPi); }
constexpr Real rpmToRadS(Real rpm) { return rpm * (kTwoPi / 60.0); }
constexpr Real radSToRpm(Real w) { return w * (60.0 / kTwoPi); }
constexpr Real celsiusToK(Real c) { return c + 273.15; }
constexpr Real kToCelsius(Real k) { return k - 273.15; }

}  // namespace avd::units
