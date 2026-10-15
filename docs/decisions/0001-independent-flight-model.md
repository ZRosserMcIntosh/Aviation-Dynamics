# ADR 0001: An independent flight model, with JSBSim as a reference only

Status: accepted (Milestone 0)

## Context

The project is MIT licensed and will ship inside a browser game. It needs:

- a flight model that runs headless and in WebAssembly, and is deterministic;
- integration with an existing C++ automotive engine and a tiled São Paulo world;
- long-term growth to helicopters, jets, damage and multiplayer.

JSBSim is the obvious mature candidate. It is LGPL-2.1, data-driven
(XML), well validated, and has a published C172 model.

## Options

1. **Link JSBSim.** It is proven and its aircraft data already exists. On the
   other side:
   - LGPL obligations for a statically linked WASM bundle, where users must be
     able to relink against a modified library;
   - a large dependency built around its own property tree and XML stack;
   - its aerodynamics is coefficient-table driven, which fits poorly with the
     brief's emergent behaviour (strip-level stall, slipstream, asymmetric
     damage);
   - two simulation cores to reconcile with the automotive engine.
2. **Port JSBSim code.** This creates a derivative work under the LGPL, which
   is incompatible with the MIT goal. Rejected.
3. **Independent model, JSBSim as a published reference.** We write our own
   component-based model. JSBSim's documentation, its published C172 data
   and its output are used only to compare behaviour, never copied.

## Decision

Option 3. The core is written from textbook and public sources (Abbott &
von Doenhoff, DATCOM, Roskam, McCormick, FAA handbooks, the POH). No JSBSim
source, XML or tables enter the repository. A JSBSim run may be used as an
external comparison, and its results are cited as such.

## Consequences

- Each physical effect has to be built and validated by us. The validation
  suite (`docs/validation/`) becomes the main quality gate.
- Behaviour comes from components rather than from fitted coefficients. This
  makes stall, spin entry, asymmetric thrust and damage expressible later, but
  it means matching the POH is a calibration task. Calibrated parameters are
  labelled `calibrated` in the aircraft data.
- There are no licence constraints on WASM distribution.
