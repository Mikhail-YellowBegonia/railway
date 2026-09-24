# Contributing to Railway

Thank you for your interest in Railway. The project is still small, early, and
short on contributors, so practical help in many forms is genuinely valuable.
Code, tests, documentation, design feedback, bug reports, and careful manual
testing are all welcome.

For a short list of areas where help is especially useful right now, see
[Contribution priorities](docs/contribution_priorities.md). Those priorities are
kept separate from this guide and may change as the project develops.

## Before you start

Railway is an early prototype, not a finished game. Please prefer contributions
that improve the current playable foundation over work aimed only at distant
roadmap ideas. If a change would significantly alter the simulation model or
architecture, open a discussion first when practical.

The project roadmap and current design constraints are documented in:

- [the roadmap](docs/roadmap.md);
- [the current progress snapshot](docs/progress_snapshot.md);
- [the UI and interaction specification](docs/ui_ux.md);
- [the plan-layer roadmap](docs/plan_layer_roadmap.md).

## Development setup

Requirements:

- Python 3.11 or newer;
- [uv](https://docs.astral.sh/uv/);
- a platform supported by pygame-ce.

```bash
uv sync
uv run python main.py
```

The local file `manual_track.geojson` is a development save and is intentionally
ignored by Git. Do not add personal saves, absolute paths, screenshots of private
files, or generated debug output to a change unless they are explicitly needed.

## Tests and manual verification

Run the complete regression suite with:

```bash
./tools/run_tests.sh
```

The automated suite is useful but not comprehensive. Railway is an interactive
simulation, and important behavior must also be checked in the running game.
Manual testing is required for changes involving, in particular:

- coupling, decoupling, reversal, or shunting;
- dispatch plans and route execution;
- signals, reservations, or pathfinding;
- Depot train generation and consist editing;
- POIs, persistence, or topology changes;
- keyboard routing, overlays, or other UI state transitions.

For these changes, automated tests passing is not by itself an acceptance signal.
Please describe the manual scenario you checked and mention any remaining edge
cases. When a change touches coupling, decoupling, reversal, or plan execution,
please repeat a realistic GUI flow rather than testing only isolated functions.

## Project structure

- `model/` — railway, train, wagon, plan, signal, POI, and persistence models;
- `controller/` — editor state, input routing, plan editing, and game flow;
- `view/` — rendering and pygame-based screen-space controls;
- `tests/` — executable regression scripts;
- `docs/` — specifications, research notes, roadmap, and current status.

Please keep domain behavior in the model or controller layers rather than hiding
it in rendering code. GUI buttons and keyboard shortcuts should share stable action
IDs. Topology changes must go through the existing coordination and protection
paths, and plan edits must respect the established parked-train and fixed-route
rules.

## Reporting bugs

A useful bug report includes:

- operating system, Python version, and commit or branch;
- the map or save used;
- clear reproduction steps;
- expected and actual behavior;
- whether the issue reproduces consistently;
- terminal output, screenshots, or a short recording when useful.

For a pathfinding rejection, also describe the train's position and direction,
the target position and direction, whether the order was manual or plan-driven,
and whether signals, reservations, coupling, or decoupling were involved.

## Pull requests

Pull requests do not need to follow a heavy process. Please make it easy to
understand what changed and why. A good pull request normally includes:

- a short description of the problem and the solution;
- the main files or layers affected;
- tests that were run, including the full suite when practical;
- the manual scenario used for verification;
- known limitations or follow-up work.

Keep unrelated formatting or refactoring out of the same change when possible.
Documentation should be updated when behavior, controls, or project status changes.

## AI-assisted development

AI agents are explicitly welcome. Contributors may use them for research,
planning, implementation, testing, documentation, or review.

Diff review by a human is not required as a separate ritual, but the contributor
is responsible for ensuring that both the human author and the AI agent understand
the intended behavior, scope, risks, and verification results. Do not submit code
that nobody involved can explain or test.

## License

By contributing, you agree that your work can be distributed under the project's
[GPL-3.0-or-later license](LICENSE). You must have the right to submit the work.
