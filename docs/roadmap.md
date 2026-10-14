# Roadmap

Milestones follow the master brief: one aircraft done properly before many
done badly.

| Milestone | Scope | Status |
|---|---|---|
| 0 | Research, licensing decision, architecture | ✅ ADR 0001–0004 |
| 1 | Single-engine piston foundation: 6-DoF, atmosphere, mass, fuel, engine, propeller, systems, ground, the Tupi T-180, validation, headless demo | ✅ (two known discrepancies) |
| 2a | Analog cockpit instrumentation, navigation receivers, 2D browser cockpit | ✅ |
| 2b | Close the Milestone 1 gaps; world integration | **next** |
| 3 | Weather and environment: spatial weather, thermals, wake turbulence, icing | ⛔ |
| 4 | Turboprop and multi-engine: constant-speed propeller, feathering, Vmc | ⛔ |
| 5 | Jets: turbofan model, transonic and supersonic aerodynamics | ⛔ |
| 6 | Helicopters: blade-element rotor, ground resonance, vortex ring state | ⛔ |
| 7 | Damage, structural loads, fire, bailout | ⛔ |
| 8 | Multiplayer: authority model, replication, assisted-pilot labelling | ⛔ |
| 9 | Roster expansion | ⛔ |

## Next milestone: 2b

The goal is to make the one aircraft trustworthy enough to build the rest on.

1. **Take-off roll.** ✅ Swirl and tip compressibility added, pitch returned
   to the published value, rolling resistance corrected: −19.9% → −12.5%.
   Swirl alone was refuted as the cause. Still open: the lift-off and
   transition (the air segment is about 30% long). Next, review the scripted
   pilot's rotation and climb technique and the flaps-10 drag in ground
   effect against flight-test data.
2. **Landing braking technique.** ✅ A pilot braking model (delay, ramp,
   sustained effort) gives −18.7%, within tolerance. Originally planned as:
   add a pilot braking model (ramp, hold, anti-skid by feel), compare the
   landing roll at POH technique, and keep the
   tyre-limited result as a separate test.
3. **São Paulo World ground query.** Implement `GroundQuery` over the tiled
   terrain and building footprints. Report hard-point collisions with
   buildings as contact events. Run the SBMT circuit over real terrain.
4. **Trimmed approach spawn in the browser.** Expose the trim solver to WASM
   so "final approach" starts trimmed rather than hands-off.
5. **Cockpit follow-ups:**
   - compass deviation card;
   - pitot heat clearing a modelled ice accretion;
   - a recorded-flight replay in the web cockpit from `InstrumentRecorder`
     CSV.
6. **CI.** Keep every job green: Linux, macOS, Windows, sanitizers, WASM
   and the headless-browser cockpit test.

Exit criteria:

- every POH comparison is within its tolerance without a "known discrepancy"
  note;
- the SBMT circuit flies on real terrain;
- CI is green on all platforms.
