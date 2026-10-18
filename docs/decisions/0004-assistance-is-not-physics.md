# ADR 0004: Assistance acts on controls, never on physics

Status: accepted (Milestone 1)

## Context

The game must be enjoyable for players who have never flown, and it must still
be a real flight model. The usual shortcut is to make the physics forgiving in
easy modes: clamp bank, add damping, cap speed. That breaks the brief's
principles. It also produces two flight models that diverge over time, and in
multiplayer it lets an assisted aircraft outperform a real one.

## Decision

- `Aircraft::step` has one physics path. Nothing in it depends on the
  difficulty setting.
- `AssistLayer` sits between raw input and `PilotInput`. It can do only what a
  skilled pilot or a certified autopilot could do: move the controls. Examples
  are:
  - a wing leveller through the ailerons;
  - stall prevention that limits nose-up stick near the critical angle of
    attack;
  - auto-coordination through the rudder;
  - auto-trim through the trim actuator, including a straight-and-level
    hold that works like a two-axis autopilot (stick plus trim servo);
  - auto-mixture through the mixture lever.
- Checklists (`ProcedureRunner`, Quick Start) operate the same cockpit
  controls a player clicks. An assisted engine start therefore takes real time
  and draws real current, and it can fail for real reasons.
- `AssistReport` publishes which assists were active on each step, so the UI
  can show them and multiplayer can label assisted pilots.

## Consequences

- An Arcade player can still stall. Stall prevention only limits
  stick input, so a gust, a power change or a steep turn can still take the
  wing past the critical angle.
- The validation scenarios fly without assists, so the POH comparison
  measures the physics alone.
