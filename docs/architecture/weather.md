# Weather: icing, rain and fronts

The air-motion side of the weather (boundary layer, turbulence, gusts,
microbursts, thermals, ridges, wind shifts) is in [wind.md](wind.md). This
page covers what the weather does besides moving the air: ice on the
airframe and in the carburettor, rain, and fronts that bring a new airmass.
All of it is opt-in: with no cloud water, rain or airmass front in
`SimpleWeather::Params`, nothing here changes a flight (every golden trace
and every earlier test is unchanged).

The physics is in `src/core/include/avd/icing.hpp` (`IcingModel`, one per
aircraft, advanced once per step) and the weather's side in
`avd/atmosphere.hpp` (`SimpleWeatherParams::rainRate`,
`cloudDropletDiameter`, `WindShift`'s airmass fields).

## Where the water is

`SimpleWeatherParams` has one stratiform cloud layer (`cloudBase`,
`cloudTop`, m MSL) with its liquid water (`cloudLiquidWater`, g/m³: stratus
0.1–0.5, cumulus to 1–2) and median volume droplet diameter
(`cloudDropletDiameter`, 20 µm by default, the 14 CFR 25 Appendix C
continuous-maximum value). Below 0 °C that water is supercooled: an icing
layer.

`rainRate` (mm/h) falls from the top of the layer to the ground
(everywhere without a layer). It is described by the Marshall–Palmer drop
spectrum: liquid water 0.089 R^0.84 g/m³, median volume diameter
0.895 R^0.21 mm (1.25 mm at 5 mm/h). In air below 0 °C rain is freezing
rain: millimetre drops that a leading edge catches all of.

`meteorologicalVisibility(sample)` gives the visibility (Koschmieder,
3.912 / extinction): 40 km of clear air, cloud droplets after Kunkel (1984,
about 80 m in a 0.3 g/m³ stratus), and rain by an empirical
0.25 R^0.63 km⁻¹ (about 3.4 km in 10 mm/h, 1.3 km in 50).

## Airframe ice

**Accretion.** A leading edge is a cylinder of twice its nose radius
(1.1 (t/c)² c). The share of the drops in its path that strike it, β₀, is
Langmuir & Blodgett's modified inertia parameter through the fit of Finstad,
Lozowski & Gates (1988): 0.38 for the C172's wing at 55 m/s in 20 µm drops,
0.66 for a 0.53 m NACA 0012 at 67 m/s (LEWICE gives about 0.65), 0.98 for
rain. The ice grows at the stagnation line at

    dk/dt = β₀ · LWC · V · n / ρ_ice

with n the freezing fraction on the surface's recovery temperature
T + 0.85 V²/(2 cp): all of the water freezes at −6 °C and colder (rime below
about −8 °C, 880 kg/m³), less of it toward 0 °C (glaze, 917 kg/m³), none
above. A jet at 250 KTAS is 6–7 K warmer than the air, so it does not ice in
−4 °C cloud. Each lifting surface carries its own ice (thickness and glaze
share), computed at its mean chord: the tail's smaller, thinner edge ices
about twice as fast for its chord as the wing.

In the C172 at 100 KTAS in a 0.4 g/m³ layer at −10 °C the wing gathers
0.55 mm/min (about 1.3 in/h: moderate icing in the FMH-1 rate classes
`icing::intensityClass` uses: trace, light to 1 in/h, moderate to 3, severe).

**Removal.** Above 0 °C the ice melts by forced convection at the leading
edge (Frössling's stagnation-line transfer, Nu = 1.14 Re^½ Pr^0.4), about
0.2 mm/min at +5 °C. Below 0 °C out of cloud it sublimates into the
unsaturated air (the Lewis analogy on the vapour density over ice), under
2 mm/h: slowly.

**Effect.** Each strip of a surface takes the effect of the ice height over
its own chord k/c on its section polar (`iceSectionEffect`), after Bragg,
Broeren & Blumenthal (2005):

| | clmax (and \|clmin\|) | cd₀ |
|---|---|---|
| roughness, k/c 2·10⁻⁴ | −22 % | +0.004 |
| rime shape, k/c 0.8 % | −39 % | +0.012 |
| glaze horns, k/c 2 % | −55 % | +0.032 |

The lift slope falls by a quarter of the clmax loss and the drag rises
faster with lift, so the stall angle falls by about (1 − f) clmax / a. The
lifting line then gives the airframe's behaviour: with 5 mm of rime on the
C172's wing and tail the trimmed CLmax falls 37 % (1.46 to 0.92), Vs1 rises
from 54 to 68 KCAS at gross weight, and the wing's stall angle falls
4.6°. With full flap it falls 48 %: the iced tail's clmin runs out before
the flapped wing stalls (tailplane-stall territory, NASA's Twin Otter
programme).

The C172's stall-warning reed trips at the clean wing's angle
(`StripTelemetry::iceStallShift`), so the iced wing stalls with little or no
warning. A jet's angle-of-attack computer goes to its icing schedule
(`stall_warning.icing_alpha_shift_deg`, 5° lower) while ice protection is on.

**Propeller and windshield.** The propeller's inboard blade (0.4 R) ices at
its helical speed (the outer blade is kept clear by kinetic heating); the
ice sheds by centrifugal force once it is about 2 % of the chord; thrust
falls and torque rises, at most about 13 % (Neel & Bright measured 10–19 %
loss of efficiency). The windshield gathers about half what the wing does
(heated windshields stay clear); 1 mm is opaque.

**Protection** (`ice_protection` in the aircraft JSON, one cockpit switch
`anti_ice.<name>` per switch name):

- *thermal* (bleed air): the leading edge warms up over 20 s with the bleed
  the cores supply (none below 45 % N2, all from 60 %), runs wet (no ice) and
  melts ice already there at 15 kW/m². Each lit turbofan loses 4 % of its
  thrust while it is on;
- *boots*: inflated every 60 s, shedding the ice over a 0.8 mm residual;
  ice builds again between cycles.

The jets (`tools/jetgen` and the Top G template): the Top G, CRJ700 and
B737-800 a bleed-air wing (their tails are unprotected), the CJ4 a bleed-air
wing and tail boots, the Learjet 45 a heated wing and stabiliser, the
Phenom 300 boots on the wing and tail; heated windshields and an ice
detector (the ICE annunciator) on all. The C172 (T-180) has none: it is not
approved for flight in known icing.

## Carburettor ice

A float carburettor (`engines[].carburettor`; absent, the engine is fuel
injected) cools its charge in the venturi as the fuel evaporates (70 % of it
before the plate: about 20 K at a full-rich FAR of 0.08) and at a partly
closed throttle plate (a fifth of the static temperature drop of the plate's
jet: about 10 K at idle, 4 K in cruise). Moist air cooled below its
saturation point there drops water; 0.8 % of it freezes on the throat and
plate while they are below 0 °C (below about −25 °C at the throat the water
comes as ice crystals that pass through). Cloud water and rain drawn in add
to it. The ice takes a share of the throttle's flow area (15 g would close
the wide-open throat; the same ice blocks a nearly closed plate's gap three
times as much), so MAP and rpm fall.

That reproduces the bands of the FAA's carburettor icing chart (AC 20-113,
SAIB CE-09-35): at 80 % humidity, icing at cruise power from about −10 °C
to +20 °C, at glide power to about +28 °C, none in dry air.

Carb heat (`engine<N>.carb_heat`, COLD / HOT) takes the intake air through
the exhaust muff: 50 K warmer at rated power, less at low power (use it
before reducing power). The warmer air is thinner: at full throttle on the
ground the C172 loses 120 rpm (the POH run-up's 75–150 rpm drop), and the
mixture is richer. It melts the ice; the meltwater goes through the
cylinders, a few seconds of rough running and lost power (`IcingStatus::roughness`),
then the rpm recovers. In the band the ice comes back once carb heat is off.

In a 15-minute cruise at +10 °C, 85 % humidity, the C172's MAP falls from
20.1 to 16.1 inHg and the rpm by 400; carb heat clears the ice within a
minute (the roughness peaks at 0.7) and the rpm comes back to within the
heated air's loss of where it started.

The T-180's reference, the C172S, has a fuel-injected IO-360-L2A, which has
no carburettor ice (an alternate-air door instead). The T-180 is given the
carburetted 172s' induction so that carburettor icing and its procedure
can be flown (see its provenance; delete the `carburettor` block for the
injected engine).

## Rain on the runway

`rainWaterFilmDepth(R)` (`avd/ground.hpp`) is the steady water film on a
paved runway, Gallaway et al. (1979), FHWA-RD-79-31, for a 45 m crowned
runway (22.5 m drainage path, 1.5 % cross slope, 0.5 mm macrotexture):
0.7 mm in 10 mm/h (wet: the 14 CFR 25.109 wet braking curve), 2.6 mm in
50 mm/h, over 3 mm above about 70 mm/h (standing water: hydroplaning above
9 √p kt). `RunwayWetting` builds it over about three minutes and lets it
drain and evaporate after the rain; its depth feeds
`SurfaceProperties::waterDepth` (the browser does this every `avd_advance`).

In flight, rain has no modelled aerodynamic effect: the documented drag rise
and lift loss of heavy rain (Haines & Luers 1983; Bezos et al., NASA TP-2932)
need rates of hundreds of mm/h to matter.

## Fronts

A `WindShift` (the moving wind shift of [wind.md](wind.md)) can bring a new
airmass: behind it the temperature deviation is `temperatureChange` K
different and the MSL pressure `pressureChange` Pa, across `airmassWidth`
(tanh), with a `pressureTrough` (sech²) on the line. The column over the
front's surface position takes the new conditions, integrated
hydrostatically with the new temperature (`SimpleWeather::conditionsAt`,
`qnhAt`), so the OAT, the static pressure at altitude and the local QNH all
change as the aircraft crosses it. An altimeter left on the setting from
ahead of the front reads wrong behind it: flying into lower pressure and
colder air it reads high on both counts (−5 hPa and −6 K at 4,500 ft: the
airplane 225 ft below what it shows). Behind a cold front proper the rise in
pressure and the cold column nearly cancel at low altitude.

Not modelled: the frontal slope (the airmass is a vertical column), the
front's own cloud and rain (set them with the cloud layer and `rainRate`),
and humidity changes across it.

## Tests

`tests/test_icing.cpp`:

- collection efficiencies (cylinder, NACA 0012 against LEWICE, rain), the
  C172's accretion rate against the hand formula and its intensity class,
  rime and glaze bands, kinetic heating, melting and sublimation;
- section and trimmed-airframe CLmax losses against the literature, the
  stall angle;
- the C172 in a supercooled layer (10 min at −10 °C, 0.4 g/m³: 4.5 mm on the
  wing, 6.7 mm on the tail, propeller thrust −14 %, 102 → 78 KCAS on the same
  power; Vs1 50 → 62 KCAS; the stall at 61 KCAS with under a second of
  warning, against 49 KCAS and 8 s clean): ice on wing, tail (faster), propeller and
  windshield, the speed lost on the same power, the stall speed up, the stall
  breaking higher with little warning, the ice melting in warm air;
- the CJ4's thermal wing and tail boots against the unprotected airframe,
  the detector, the bleed's 4 % thrust cost; no protection on the C172;
- the carburettor icing chart's bands; MAP and rpm falling in the band, not
  outside it; carb heat's rpm drop, the rough patch and the recovery, the ice
  coming back; the 120 rpm static drop;
- the Gallaway film and the runway wetting and drying, the visibility,
  freezing rain;
- a front's OAT, QNH and trough, and the altimeter error flying through one.

`tests/test_instruments.cpp`'s pitot-icing case now also slows on its
airframe ice (from 100 to about 80 KCAS in 5 minutes, same power).

`web/smoke_test.mjs`: the C172 at −5 °C in cloud picks up ice and its stall
speed rises; carburettor ice in humid air and carb heat clearing it; rain
wetting the runway; a cold front over the field; the CJ4's anti-ice.
