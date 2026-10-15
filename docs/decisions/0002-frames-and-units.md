# ADR 0002: Coordinate frames and units

Status: accepted (Milestone 0)

## Decision

- **SI everywhere inside the core.** Conversions live in `avd/units.hpp` and
  happen only at the boundaries: JSON fields with explicit unit suffixes
  (`_kg`, `_m`, `_kias`, `_inhg`), instrument displays, reports and the
  C ABI.
- **Body axes are FRD**, the aerospace convention used by the references
  (DATCOM, Etkin, Stevens & Lewis), so their equations transcribe without sign
  changes.
- **Integration happens in a local NED tangent frame** on the WGS-84
  ellipsoid (`LocalFrame`). It can be rebased (floating origin) without
  changing the physical state.
- **The world/game frame is ENU** (x east, y north, z up) with its origin at
  MASP, matching the São Paulo World tiles and the automotive engine.
  `world_frame.hpp` is the only place that converts, including the
  NED/FRD → ENU/FLU attitude used by renderers.
- **The structural frame** for aircraft data is the manufacturer's datum
  (for the C172, the front face of the firewall), with X forward, Y right,
  Z down in metres. The loader converts it to CG-relative positions.

## Consequences

- One transformation point for renderers means there is no ambiguity about
  signs in game code.
- Double precision with a rebasing origin keeps position error at the
  millimetre level across the metropolitan area.
- Units in JSON field names make data review possible without reading code.
