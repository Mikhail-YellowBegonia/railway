# Demo release checklist

The original prototype-to-demo roadmap is archived at
`docs/archive/roadmap-prototype-to-demo.md`. This checklist is the remaining release
work, not a new feature roadmap.

## Required before public demo declaration

- [x] Add save management around the current GeoJSON session format.
- [x] Verify named save, save-as, old-save migration, and failed-write behavior.
- [ ] Run the full regression entry point: `tools/run_tests.sh`.
- [ ] Perform a clean-directory install and launch from the README.
- [ ] Recheck the complete playable loop: build track, place signals/POIs, create a
      consist, spawn a train, create a plan, run it, couple/decouple, save, reload.
- [ ] Add a short screenshot or GIF showing the playable loop.
- [ ] Confirm repository hygiene and absence of local saves, caches, credentials, or
      personal absolute paths.
- [ ] Create the first demo tag after the above checks pass.

## Intentionally outside this release

Cargo and economy, realistic physics, LOD, multiplayer, 3D presentation, plan
persistence, and broad visual redesign remain future work.
