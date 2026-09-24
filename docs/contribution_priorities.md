# Contribution priorities

This page describes the kinds of help that are especially useful at the current
stage of Railway. It is intentionally short and may change more often than the
main contribution guide.

## Highest-value contributions now

### Manual testing and bug reports

Run the game and exercise complete flows, especially:

- Depot → consist builder → train generation;
- coupling, decoupling, reversal, and shunting;
- plan creation and plan execution;
- signal waiting and recovery;
- save and reload;
- UI overlays and keyboard isolation.

The project particularly needs reports that make pathfinding rejections easier to
understand: exact steps, train direction, target direction, current plan state,
and a screenshot or recording when possible.

### Regression tests

Add focused tests for bugs that can be reproduced without a window, and improve
test fixtures for pathfinding, dispatch, consists, topology protection, and save
compatibility. Automated tests do not replace manual verification, but they make
known failures much easier to prevent.

### Documentation and onboarding

Useful work includes improving the first-run instructions, explaining current
controls, documenting confusing error messages, and turning successful manual
scenarios into short reproducible examples.

### Small usability improvements

Small, well-scoped improvements to input routing, feedback, overlays, and error
messages are welcome. Please preserve the single-overlay state model and the
keyboard/GUI action-ID boundary.

### Core reliability fixes

Fixes to pathfinding, signal reservation, plan execution, persistence, and consist
operations are valuable when they are narrowly scoped and accompanied by tests and
manual verification.

## Good first contributions

- reproduce an existing issue and improve its report;
- add a regression test for a known edge case;
- improve a misleading player-facing message;
- clarify a document or add a small usage example;
- test the game on another platform;
- review a focused pull request or documentation change.

## Deliberately lower priority for now

The following are worthwhile long-term directions, but are not the best place to
start without prior discussion:

- the full preview-based dispatch-plan editor;
- large-scale 3D rendering or an engine migration;
- multiplayer architecture;
- CargoDist and a complete economy system;
- broad visual redesigns or large asset packs.

These areas are part of the long-term vision, but the project currently benefits
more from stabilizing and explaining the playable foundation.
