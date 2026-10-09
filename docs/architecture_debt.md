# Architecture debt register

> Current scope: the post-demo stabilization phase. This document records debt that
> affects the next planned features. It is not a commitment to redesign the project
> for future 3D, multiplayer, logistics, or other transport types.

## Status

The first railway sandbox demo is functionally complete. Train movement, consists,
signals, dispatch plans, and persistence have been manually validated. The remaining
risk is mostly in system-level edge cases and transitional code left by rapid
iteration.

The project already has useful model and component regression tests. The gap is not
the absence of tests; it is the absence of reliable end-to-end GUI and long-running
scenario coverage.

## A0 work items

### A0.1 Save management — release prerequisite

**Status: implemented (2026-09-24).** `model/save_service.py` now owns named
GeoJSON save resolution, new/open/save-as flows, version migration
(`format_version=1`), and atomic replacement. `GameLoop.save_session()` routes
the existing `S` action through that service; `save_session_as(name)` provides
the save-as boundary for future UI wiring. Legacy documents without a version
field remain readable.

Wrap the current single-file GeoJSON persistence in a small save service.

Required outcomes:

- named save selection rather than a hard-coded `manual_track.geojson` path;
- new, open, and save-as flows;
- atomic writes and preservation of the previous save on failure;
- a format version and a clear migration entry point;
- compatibility with current GeoJSON saves;
- explicit separation of world data and runtime snapshots.

This is intentionally not a full campaign/save-slot system.

### A0.2 Transitional surface audit and cleanup

**Status: audit recorded (2026-09-24).** See
`docs/transitional_surface_audit.md`. No current player-facing path was found
safe to delete without changing an accepted workflow; the node-placement path,
`F` shortcut, and `Shift+right-click` path are explicitly classified as
debug/compatibility surfaces for a later removal decision.

Inventory every entry point as one of: player feature, debug tool, test hook, old
input compatibility, old-save compatibility, or obsolete code. Remove obsolete
player-facing paths after their replacement is verified, while retaining only the
compatibility needed to read existing saves.

Candidates include the legacy placement path, compatibility shortcuts, the temporary
plan-entry route, and old plan execution branches. Each removal must have a focused
manual check and a relevant regression script where practical.

### A0.3 Input and action boundary

**Status: completed (2026-09-25).** `controller/action_router.py`
now maps modal keyboard input to the same stable action ids already emitted by
`GameGUI`. The consist builder, train-information overlay, schedule overlay,
plan editor, POI editor, and workspace mode shortcuts use this shared route for
keyboard and widget actions; modal consumers still own their domain-specific
behavior. `controller/drag_state.py` defines the single-owner start/update/
release/cancel lifecycle required by the upcoming plan-list and consist drags.
World-space picking remains separate from screen-space widgets.

Keep the existing state-machine model, but make the boundary explicit:

```text
raw input -> context/overlay routing -> action id -> domain operation -> feedback
```

The first consumers are the plan list and consist/garage UI. World-space picking and
screen-space widgets remain separate. Drag start, update, release, and cancel must
have one ownership rule rather than being handled ad hoc by `GameLoop`.

### A0.4 Train-control authority

**Status: first slice implemented (2026-09-24).** `TrainControlAuthority` now
names the runtime owner as `plan`, `plan_paused`, `manual_takeover`, or `none`.
Normal right-click orders and manual K coupling remain available as a deliberate
temporary takeover/debug mode; they do not transfer or delete the wagon-owned
plan. `PlanDispatcher` owns the plan state and drops its execution lease when a
manual goal differs. This preserves the existing behavior while making the
ownership boundary observable and testable. A later UX pass may rename or hide
the takeover wording, but no product decision is currently required.

Define the relationship between normal plan execution and temporary driving before
building plan preview. Decide whether temporary driving is removed completely or
retained only as an explicitly named emergency/debug takeover, and specify how it
affects the active plan, pointer, signal waiting, coupling, and persistence.

## Deferred debt

The following are recorded but not part of A0:

- stable persistent Node/Edge identities and topology migration;
- a full decomposition of `GameLoop` and `renderer`;
- an event bus or replay framework;
- common vehicle/ECS abstractions for other transport types;
- a renderer rewrite for 3D;
- comprehensive end-to-end automation.

They become active only when a concrete feature creates a dependency on them.

## Exit criteria

A0 is complete when:

1. the demo can create/open/save a named save safely;
2. obsolete player-facing paths are removed or explicitly documented as debug tools;
3. UI actions have a single routing boundary used by both shortcuts and widgets;
4. train-control ownership is documented and implemented consistently;
5. the next feature can be developed without adding another parallel compatibility
   path.
