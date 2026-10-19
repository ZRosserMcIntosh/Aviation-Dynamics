# Navigation: destinations, ILS and GPS

Pick a destination anywhere in Brazil and the aircraft does three
things:
- tunes NAV 1 to that runway's ILS and sets the course;
- loads a GPS route to it;
- flies on a ground where every field sits at its own elevation.

The code:
- `src/scenarios/airports.*`: airports, ILSs, routes and the region ground;
- `src/core/*/gps.*`: the navigator;
- `src/core/*/ground.*`: `RegionGround`;
- `web/avd_web.cpp`: the page API.

## Airports and their data

`saoPauloRegionAirports()` lists:

| Airport | Runways | ILS | Source |
|---|---|---|---|
| Congonhas SBSP | 17R/35L | 35L ISO 109.70, 17R ISP 109.30 | OurAirports frequencies (public domain); Airportroutes and Metarcentral listings agree |
| Campo de Marte SBMT | 12/30 | ITMT 110.30 on 12, **synthetic** (SBMT has no ILS) | — |
| Guarulhos SBGR | 10L/28R, 10R/28L | 10L IUC 110.70, 10R IGH 111.70, 28L IBC 111.10, 28R IGS 111.90 | OurAirports frequencies; DECEA chart ILS RWY 10L (AMDT 2209A1) for IUC |
| Viracopos SBKP | 15/33 | 15 IKP 110.30 | Flight Plan Database listing (single secondary source, unverified) |
| São José dos Campos SBSJ | 16/34 | none found: GPS approaches | — |
| Jundiaí SBJD | 18/36 | none: GPS approaches | — |

Runway data:
- Ends, widths, displaced thresholds and elevations come from OurAirports'
  `runways.csv` and `airports.csv` (public domain), retrieved October 2026.
- The ends' coordinates give each runway's centre and direction. Its extent
  is the published length: Guarulhos' end coordinates are about 150 m
  short of its 3,700 m.
- Runways are flat at the field elevation.

Data notes:
- **Guarulhos renumbered** its runways from 09/27 to 10/28 in September
  2022. Older listings name the same ILSs under the old numbers.
- **Congonhas** keeps its runway where the map imagery shows it, about
  112 m east of the survey. Its ILSs are built on that runway, so the
  beam lines up with the runway the pilot sees.
- **Shared frequency:** Viracopos' real IKP and Campo de Marte's synthetic
  ITMT both use 110.30, 85 km apart. The receiver takes the nearer station
  in range, as a real one would.
- **Not for real-world navigation.** Antenna sites are estimated, and
  waypoint names are generated, not the published fixes.

## Anywhere in Brazil (`data/world/airports_br.json`)

`tools/navdata/brazil.py` builds the list from OurAirports (public domain):
- **Coverage:** 1,150 Brazilian airports with at least one open paved
  runway of 3,000 ft or more.
- **Per airport:** codes, name, city, state, elevation, reference point,
  aliases, runways and ILSs.
- **Aliases:** OurAirports' keywords plus a short list of the names people
  use: Confins, Galeão, Santos Dumont, Pampulha.
- **Repairs:** a few records pair runway ends that aren't reciprocals
  (16/30). For those, the far end is derived from the near one.

Runways (`airportFromRecord`):
- **The runway number is the arbiter.** OurAirports' listed true headings
  are often magnetic or wrong (Florianópolis 14: listed 159°, coordinates
  120°), and a few end coordinates are misplaced (Tabatinga 07).
- **Placement:** end coordinates are used when they agree with the
  number to within 20°. Otherwise the runway is centred on the reference
  point, on the listed heading if that agrees with the number, else on the
  number's.
- **Parallels** without coordinates stand 400 m apart.
- **Variation:** the number's heading uses the regional 21.5° W estimate.
  Brazil's variation runs about 12–24° W.

ILSs:
- OurAirports has frequency records for only a few, and third-party
  listings disagree (one still shows Guarulhos' pre-2022 numbers).
- The list keeps those, plus a curated few: Galeão (two of three agree
  with a chart), Brasília and Viracopos (single secondary source, flagged).
- **Everywhere else:** RNAV (GPS) approaches.

The page loads one record into the core with
`avd_set_destination_record`. A region airport maps onto the region's own
data. Any other joins the ground (each field at its own elevation, blended
to the region) and the nav database after the region's airports, and the
route runs to it from the departure runway.

The flat frame holds across Brazil:
- **Altitude** is the frame height minus z everywhere, so a field reads its
  own elevation.
- **ILS geometry** is evaluated geodetically, with the course in the
  antenna's own frame. Galeão's and Brasília's beams are centred 360 and
  870 km away.
- **Directions:** local north in the frame turns by under 2° even 3,000 km
  out (Tabatinga).
- **Distances:** the tangent-plane projection shortens them by up to about
  3 % at 3,000 km.
- **Gravity:** Earth curvature in gravity's direction is ignored.

The page's search is lexical:
- accent- and case-insensitive, every word typed matching a word of the
  airport (as a prefix, inside it, or within one or two typos);
- exact codes first;
- bigger airports with scheduled service and an ILS ranked higher among
  equals.

## ILS geometry

- **Localizer:** antenna 300 m beyond the far end. Its course width puts
  full scale at ±106.7 m (700 ft wide) at the landing threshold.
- **Glideslope:** antenna 300 m past the landing threshold, 120 m to the
  side, 3°.
- **Coverage:** localizer ±35° to 18 NM; glideslope ±8° about the localizer
  centreline to 10 NM (ICAO Annex 10). The glideslope sector used to be
  measured from its own antenna. From 120 m off the centreline, that put an
  aircraft close in and slightly off course outside the sector.
- **Meridian convergence:** the localizer course is expressed in the
  antenna's own tangent frame. Far from the world origin (MASP), local north
  turns by the convergence of the meridians, 0.2° at Viracopos, which would
  otherwise offset the beam by 5 % of full scale.
- **Curvature:** the receiver works in true geometry, so the glidepath is an
  angle at the antenna over a curved Earth. On the path, an aircraft stands
  d²/2R higher than a flat 3° line would put it: 10 m at 6 NM, as real
  charts allow for.

Test: `tests/test_destinations.cpp` puts every ILS centred on its own 3° path
at 6 NM, still received close in, and silent on the reciprocal's final.

## The region ground

`RegionGround` has one field per airport:
- **Fields:** flat within the runways' extent plus 1.5 km, grass, with
  asphalt runways.
- **Between fields:** the elevations are blended by inverse-distance
  weighting (power 2) of the distance beyond each field's flat area. The
  result is continuous and gentle: under 2 % between Congonhas (802 m) and
  Guarulhos (750 m).
- **Not real terrain:** the Serra da Cantareira and the valleys are absent.

Altitude in the core is flat-Earth: the frame origin's height minus z, as
the aircraft, instruments and nav receiver all report it. A field 89 km away
therefore reads its own elevation on the altimeter. The web build draws the
same heights: a 1 km terrain mesh out to about 100 km, and the city map on a
500 m grid.

## GPS navigator (`avd/gps.hpp`)

A route is a list of waypoints. Each has an ident, a NED position, an
optional crossing altitude, and a fly-over flag (the runway).

**Sequencing:**
- The navigator advances when the leg is flown.
- For a fly-by waypoint it advances at the turn anticipation distance:
  r·tan(Δ/2), with r the radius of a 25° bank turn at the ground speed,
  capped at 5 km.
- `activate(i)` joins the route on leg i, for an aircraft already
  established on it. `directTo(i)` flies direct from the present position.

**Course deviation**, scaled as panel GPS navigators do (Garmin GNS/GTN):

| Phase | Full scale |
|---|---|
| En route | ±2 NM |
| Terminal (within 30 NM of the destination) | ±1 NM |
| Over the 2 NM before the FAF | narrows to ±0.3 NM |
| From the FAF | angular (2°), between 350 ft and 0.3 NM |

**Vertical path** (LPV / LNAV+V style): a 3° path from the FAF to 50 ft
over the threshold. It is valid in approach mode, and its full scale is
0.7° (as a glideslope), between 15 and 150 m.

**Approach route** (`approachRoute`):
1. **From the runway** (on the departure runway): the runway, then 3 NM
   straight out.
2. **Initial fix, when needed:** an IAF 5 NM abeam the IF. It is added
   when the arrival comes from more than 60° off the final course, and not
   when the aircraft is already on the final.
3. **The final:** the IF 10 NM out and the FAF 5 NM out on the extended
   centreline, at the 3° path's altitudes, then the landing threshold.

Test: the route from São José dos Campos to Guarulhos 10L gets an IAF. A
point-mass airplane flying the GPS course and path sequences every waypoint
in order and crosses the threshold within 150 m. On the final it stays
within 30 m laterally and 10 m vertically. The CDI and VDI sense and the
scaling are checked separately.

## Page API (`web/avd_web.cpp`)

- `avd_set_destination(airport, runway)` (-1 clears):
  - tunes NAV 1 to the runway's ILS (the test VOR goes to standby) and sets
    the OBS to its magnetic course;
  - builds the route, from the runway when on it, else from the present
    position.
- **Destinations:** `avd_destinations_text` (names, ends, ILSs);
  `avd_region` (runway geometry for drawing).
- **Ground:** `avd_ground_height(east, north)`.
- **GPS:**
  - `avd_route` / `avd_route_text`: waypoints, active leg, FAF;
  - `avd_gps`: valid, active, mode, DTK, XTK, full scale, CDI, distance and
    bearing to the waypoint, distance to go, vertical deviation and VDI.
- `avd_reset(4)`: the destination's IF, level on the final course (jets
  180 kt flaps 1, the 172 90 kt). `avd_reset(5)`: its final, like mode 2.
- The state's "above field" (`s[14]`) is now the height above the ground
  below, not above the home field.

## Limits

- No published procedures: no STARs, transitions, missed approaches,
  minimums or step-down fixes.
- No DME or marker beacons.
- Magnetic variation is one regional estimate (21.5° W).
- No terrain between the fields, and runways are flat.
- The world frame stays anchored in São Paulo. Brazil-wide distances are
  within 3 %; beyond Brazil, rebase the frame.
- Most airports have no ILS data here: they get GPS approaches.
- Runways without coordinates sit at the reference point.
