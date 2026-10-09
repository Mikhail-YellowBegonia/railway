# Next roadmap: post-demo stabilization and feature work

The prototype-to-demo roadmap is complete and archived. Work now starts with A0
architecture stabilization, followed by the six candidate feature directions below.

## A0 — architecture stabilization

1. Save management and release preparation.
2. Transitional-code inventory and removal of obsolete player-facing paths.
3. Shared input/action routing for keyboard, widgets, and drag interactions.
4. Explicit train-control authority; resolve the temporary-driving versus plan
   execution conflict.

The details and exit criteria live in `docs/architecture_debt.md` and
`docs/release/demo_release.md`.

## Candidate feature sequence

The following order is provisional until each item receives a detailed scope and
acceptance criteria from the project owner.

1. **Drag interaction** for the dispatch-plan list and garage/consist UI. This is the
   first reusable screen-space interaction layer.
2. **Plan preview** for spatially understandable shunting-plan authoring. It depends
   on the action/drag boundary and on train-control authority being explicit.
3. **Remove temporary driving**, or reduce it to a deliberately scoped takeover mode,
   once the plan authority decision is implemented.
4. **Map drawing rewrite**. Before changing topology or editing semantics, settle the
   persistent-reference and topology-migration requirements it actually needs.
5. **Speed limits** for curves, signs, and dispatch plans. Define source precedence
   and propagation before adding controller fields.
6. **Basic vehicle pack**. Keep vehicle content separate from vehicle capability and
   only add capability dependencies required by the defined pack.

## Planning rule

Do not introduce a new compatibility path when an existing boundary can be made
explicit instead. Large future-facing redesigns for 3D, multiplayer, logistics, or
other transport types are architectural review topics, not implicit prerequisites for
this roadmap.
