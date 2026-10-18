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
