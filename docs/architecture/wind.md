# Wind and air motion

Aircraft sample the air at their position: `AtmosphereField::sample` returns
the air velocity (NED) and its spatial gradient. The aerodynamic model applies
the local air velocity, plus gradient × arm, to every lifting strip, the
propeller disc and the pitot. Nothing fakes turbulence with camera shake: a
gust is air moving across the airframe.

The building blocks are in `src/core/include/avd/wind.hpp`. `SimpleWeather`
(`atmosphere.hpp`) composes them.

## Mean wind: the boundary layer

The surface report is the wind at `referenceHeight` (10 m, the anemometer
height). The profile with height is one of three:

| Profile | Law | Use |
|---|---|---|
| `LogLaw` (default) | U(z) = U_ref ln(z/z0) / ln(z_ref/z0) | Neutral surface layer. Physically based. |
| `PowerLaw` | U(z) = U_ref (z/z_ref)^n | Engineering approximation (n = 1/7). |
| `Uniform` | U(z) = U_ref | Steady-wind tests. |

The **surface roughness** z0 sets the shear and, through the friction
velocity u* = κ U_ref / ln(z_ref/z0), the mechanical turbulence (below).
Typical values (Wieringa 1992, Davenport classification):

| Surface | z0 (m) |
|---|---|
| Open water | 0.0002 |
| Grass airfield (default) | 0.03 |
| Farmland | 0.1–0.25 |
| Suburbs | 0.5–1 |
| City centre | 1–2 |

The log law is held below max(0.5 m, 2 z0), where the flow is among the
roughness elements. The profile is constant above the boundary-layer top
(`boundaryLayerTop`, 600 m by default).

**Ekman turning.** Friction backs the surface wind against the free-stream
(gradient) wind. `veer` is the total turning from the surface to the top of
the layer, positive clockwise with height. It is about +20–30° over land in
the northern hemisphere, the opposite sign in the southern hemisphere (São
Paulo), and about 10° over water. The turning follows 1 − (1 − z/top)², so
most of it happens in the lower half of the layer.

**Winds aloft.** `windsAloft` is a list of reported levels (MSL altitude,
speed, direction). Above the boundary layer, the wind blends from the layer-top
wind to these levels. Speed and direction are interpolated by altitude, and the
direction turns the short way. Above the highest level the wind is constant.
A thin pair of levels makes a frontal or low-level-jet shear layer.

The vertical shear d(wind)/dz is part of the sampled gradient, so a climbing
or descending airplane feels it on its tail and wings.

## Turbulence

Turbulence is a frozen, world-space field: Fourier modes with a von Kármán
spectrum, divergence-free by construction, advected by the mean wind
(Taylor's hypothesis). Every aircraft sees the same air at the same place and
time. The run is deterministic for a seed.

**Statistics follow MIL-HDBK-1797 / MIL-F-8785C with height** (h in ft AGL,
lower bound 10 ft):

| Region | Length scales | Intensities |
|---|---|---|
| Low altitude, h < 1000 ft | L_w = h, L_u = L_v = h / (0.177 + 0.000823 h)^1.2 | σ_u/σ_w = σ_v/σ_w = 1 / (0.177 + 0.000823 h)^0.4 |
| 1000–2000 ft | linear blend | linear blend |
| Medium/high altitude, h > 2000 ft | L = 2500 ft, isotropic | probability-of-exceedance curves |

Near the ground the gusts are therefore anisotropic and short. Horizontal
gusts are about twice the vertical ones at 10 ft, and the vertical gust's
scale equals the height. Aloft the turbulence is isotropic and long. The
vertical component fades to zero over the bottom 3 m.

**Sources of turbulence.** They are combined root-sum-square:

1. **Mechanical**, on by default: σ_w = 1.25 u* in the low-altitude band
   (Panofsky & Dutton). It needs no setting. It comes from the actual wind over
   the actual roughness, so a 15 kt wind over the city is rougher than the same
   wind over the airfield. Over open terrain it agrees with MIL's σ_w = 0.1 W20
   to within about 10%. It fades out between 1000 and 2000 ft AGL.
2. **Severity class** (`severity`): light, moderate or severe. At low altitude
   W20 = 15 / 30 / 45 kt. Aloft, the 10⁻², 10⁻³ and 10⁻⁵ exceedance curves of
   MIL-F-8785C apply. This is clear-air or convective-weather turbulence at
   every height.
3. **Convective boundary layer** (when `convectiveVelocity` w* > 0):
   σ_w² = 1.8 w*² (z/zi)^(2/3) (1 − 0.8 z/zi)² (Lenschow et al. 1980), and
   σ_u = σ_v = 0.6 w*.
4. **`intensity`**: an extra RMS (m/s) with the same height dependence, for
   scenarios that need a specific level.

**Synthesis.** A bank of ten unit-intensity fields at integral scales
2, 4, …, 1024 m is built once. Each velocity component at a height blends the
two fields bracketing its length scale with cos/sin weights. That keeps the
variance exactly σ² between scales. Then it is scaled by its σ along the
turbulence axes (u along the surface wind, v across it, w vertical). The
gradient includes the height derivative of σ and L, so gust gradients stay
consistent near the ground. The tests check the gradient against finite
differences.

## Discrete events

These are analytic fields added on top of the mean wind and turbulence. Their
gradient comes from central differences. Each is a list in `SimpleWeather::Params`.

**Discrete gust** (`gusts`). A vertical plane front with a horizontal normal,
and a profile across it:

- `OneMinusCosine`: U = U_ds/2 (1 − cos(π s / H)) over 0 ≤ s ≤ 2H
  (FAR 25.341, with H the gust gradient distance);
- `RampAndHold`: a 1-cosine ramp over H to a sustained value, held for
  `holdLength`, then ramped back (MIL-F-8785C).

The amplitude is any NED vector: vertical, head or side gust. The front is
either fixed in space (`frontSpeed = 0`), moves at a given speed, or, by
default, is frozen in the air and carried by the mean wind.

*Check:* a 5 m/s vertical gust with H = 12.5 MAC gives a load factor within
the range set by Pratt's gust formula
Δn = K_g ρ V a U / (2 W/S), with K_g = 0.88 μ / (5.3 + μ). The sim reads
about 18% above Pratt, as it should: the lifting strips are quasi-steady.
There is no Küssner lift growth on gust entry, which K_g partly accounts for.
That is a known gap in the aerodynamics.

**Microburst** (`microbursts`). This is the Oseguera–Bowles analytic downburst
(NASA TM-100632, 1988). It is axisymmetric and satisfies continuity exactly:

    u = λR²/(2r) [1 − e^{−(r/R)²}] [e^{−z/z*} − e^{−z/ε}]
    w = −λ e^{−(r/R)²} [z* (1 − e^{−z/z*}) − ε (1 − e^{−z/ε})]

The parameters are:

- shaft radius R (1000 m);
- outflow decay height z* (600 m);
- surface friction layer ε (30 m);
- the peak radial outflow, which sets λ.

The peak outflow is at r = 1.12 R and z = z* ε ln(z*/ε)/(z* − ε), about 95 m
AGL with the defaults. Observed maxima are 50–100 m. The burst has a life
cycle (growth, mature and decay times) and can drift.

`windShearHazardFactor` is the FAA/TDWR hazard index
F = (dW_x/dt)/g − w/V. Above 0.1 the shear is hazardous.

*Checks:* the peak outflow and its location match the specification, the
sampled gradient is trace-free (incompressible), and a 15 m/s burst gives
F > 0.1 at 300 ft. A C172 crossing a 15 m/s burst at 1000 ft, holding wings
level with fixed power, behaves as encounters do. It gains about 5 kt in the
growing outflow headwind. Then the 9 m/s downdraft and the tailwind take
about 55 kt and 250 m, more sink than the airplane can out-climb.

**Shear layers.** Frontal and low-level-jet shear are expressed as
closely spaced `windsAloft` levels (see above).

## Vertical air

**Thermals** (`thermals`, active when `convectiveVelocity` w* > 0). The
mean updraft and the outer radius follow Allen (2006, NASA TM-2006-214250)
with height in a convective layer of depth zi:

    w_avg = w* (z/zi)^(1/3) (1 − 1.1 z/zi),   r2 = max(10, 0.102 (z/zi)^(1/3) (1 − 0.25 z/zi) zi)

Each thermal is an axisymmetric stream function. It has a core and a
compensating sink annulus with zero net mass flux:

    Ψ = A R²/2 (e^{−r²/4R²} − e^{−r²/R²}),   w = (1/r) ∂Ψ/∂r,   u_r = −(1/r) ∂Ψ/∂z

Continuity therefore holds exactly. That includes the inflow toward the base
of a thermal and the outflow under the inversion, which a "w only" model
lacks. The updraft reaches zero at r2. Its mean inside r2 equals Allen's
w_avg, and the peak is about 3 w_avg. The profile is rounded off over the
bottom 5% of the layer, so the convergence below the core stays finite.

The **random field** puts cores on a jittered grid at Allen's density,
N = 0.6 · area / (zi r2) at mid-layer, which is about 500 m spacing for
zi = 1500 m. Each cell produces thermals one after another. Each has a
random strength (0.6–1.4), a random position, and a sin² life cycle
(`lifetime`, 20 min by default). When one dies the next rises somewhere else
in the cell. The whole field drifts with the mean wind at mid-layer.
Explicit `thermals` (steady, or with their own life cycle) can be added, for
example a reliable thermal over a hangar roof or a ploughed field.
The **convective turbulence** of the layer (Lenschow σ, above) is switched on
with the thermals unless `convectiveTurbulence` is false.

*Checks:*

- the mean updraft inside r2 equals Allen's w_avg, w = 0 at r2, sink outside;
- the net flux over a 6 r2 disc is under 0.2% of the core flux;
- the gradient is trace-free;
- over a 60 km traverse the field has cores of at least w*, no net vertical
  flux, and lift over a minority of the sky;
- thermals are carried by the drift and they change over a lifetime;
- a C172 crossing a 3 m/s core climbs 1.3 m/s and then meets the sink ring.

**Ridges and mountain waves** (`ridges`). Queney's (1948) linear hydrostatic
solution for a Witch-of-Agnesi ridge h = h0 a²/(x² + a²):

    η = h0 a (a cos(lz) − x sin(lz)) / (x² + a²),   l = N/U
    w = U ∂η/∂x,   u′ = −U ∂η/∂z

Here x is across the ridge and measured downwind, z is height above the ridge
base, U is the cross-ridge wind at crest height, and N is the
Brunt–Väisälä frequency (`stability`, 0.01 s⁻¹ typical). One model gives:

- **ridge lift**: at the surface w = U dh/dx exactly, lift on the windward
  slope and sink in the lee;
- **mountain waves** aloft, with vertical wavelength 2πU/N and phase lines
  tilting upwind. Half a wavelength up, the lift sits over the lee slope.

The waves fade at the ridge ends (over one half-width), above `waveTop`
(where the tropopause absorbs them), and in cross-winds under about 3 m/s.
Flow that is too stable to cross the ridge (N h0/U > about 1, blocking) is
approximated by limiting the effective height to 0.8 U/N. The ridge is an
atmospheric object only. Its geometry should match the terrain of the world
tile (for São Paulo: the Serra da Cantareira north of SBMT, the Serra do Mar
to the south).

Not modelled yet: trapped lee waves and rotors, which need a two-layer
Scorer-parameter solution. Also absent is cloud suck under cumulus.

## Tests

`tests/test_wind.cpp` checks:

- the log-law ratios, roughness and friction velocity;
- the veer and the short-way interpolation of winds aloft;
- the shear gradient;
- the MIL-HDBK-1797 σ and L values by height, severity and convection;
- the realised RMS and the anisotropic autocorrelation of the field;
- gradient consistency;
- Taylor advection.

`tests/test_flight.cpp` checks that a 15 kt wind is bumpy at 500 ft and smooth
at 5000 ft.
