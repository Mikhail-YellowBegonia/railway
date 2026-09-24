# Railway

Railway is an early-stage railway sandbox game: a modern, 3D-oriented project
inspired by OpenTTD and other transport simulations. The current prototype uses
Python and pygame-ce as a small, testable foundation for a much larger game.

<!-- A gameplay GIF will be added here after the first recording session. -->

> The project is still in very early development. The current demo is playable,
> but it is not yet a complete game.

## What is here now

The prototype currently supports:

- CAD-like construction of straight, curved, biarc, and composite tracks;
- directional signals, path reservation, and signal-aware train movement;
- Platform and Depot POIs with custom names;
- a consist builder with powered control cars and ordinary coaches;
- multi-wagon consists that can be coupled, decoupled, reversed, saved, and restored;
- wagon-owned control state and dispatch plans;
- fixed-route plan execution, waiting, coupling, decoupling, and shunting operations;
- a keyboard-first interaction model with pygame-based screen-space controls.

The current demo is intentionally focused on the simulation core and basic railway
operations. Cargo logistics, economy, multiplayer, terrain, and a complete 3D
presentation are not implemented yet.

## Why this project exists

OpenTTD provides an excellent model for a playable railway sandbox. Railway aims to
build toward that breadth while exploring a more explicit and robust consist model.
Coupling, decoupling, shunting, control-car selection, and changing train makeup
are treated as first-class simulation problems rather than exceptional cases.

The long-term vision is:

1. a complete railway sandbox with construction, operations, and cargo transport;
2. reliable and expressive consist operations;
3. multiplayer support;
4. a modding system designed as part of the architecture;
5. a renderer fully separated from the simulation core, allowing a useful 3D
   presentation without making visual fidelity the project's primary goal.

## Core design principles

- **Wagons are domain objects.** Plans, control state, and other wagon-level data
  remain attached to the wagon so consists can change without losing ownership.
- **The world model is renderer-independent.** The project uses GeoJSON-inspired
  data structures, preserving 3D-capable coordinates while the current frontend
  remains a 2D implementation.
- **Simulation layers stay separate.** Planning, path reservation, movement,
  coupling, persistence, and rendering are developed as distinct layers.
- **Reliability comes before spectacle.** The current goal is a dependable and
  understandable simulation, not a polished AAA visual presentation.

## Development roadmap

### Early prototype

Python simulation core, pygame-based 2D tools, track construction, signalling,
train movement, flexible consists, and basic dispatch plans.

### Feature expansion

CargoDist-style logistics, economy, terrain and scenarios, additional vehicle types,
multiplayer foundations, and experiments with OpenStreetMap/OpenRailwayMap data.

### Playable version

An improved 3D presentation, richer onboarding, and—if needed—a new implementation
language or runtime for the game core.

### Long-term direction

A lightweight, open-source game framework or engine with a replaceable renderer,
mod support, and multiplayer capabilities.

The roadmap is exploratory. Later stages may change as the prototype reveals which
architectural decisions are worth keeping.

## Run the prototype

Requirements: Python 3.11+, [uv](https://docs.astral.sh/uv/), and a platform
supported by [pygame-ce](https://pyga.me/).

```bash
uv sync
uv run python main.py
```

The game starts from `test_track.geojson`. If `manual_track.geojson` exists, it is
loaded as the local development session instead. Press `S` to save that session;
the file is intentionally ignored by Git.

For the current controls and interaction rules, see
[docs/ui_ux.md](docs/ui_ux.md) and [docs/editor.md](docs/editor.md).

## Verification

The repository uses executable regression scripts:

```bash
./tools/run_tests.sh
```

The test suite is useful but not complete. Because this is an interactive simulation,
important changes should also be checked manually in the running game.

## Acknowledgements

- OpenTTD and other railway and transport simulation games are the project's main
  sources of inspiration. Many basic gameplay ideas and implementation approaches
  are intentionally studied and adapted from them.
- The OpenTTD PX-Patch developers provided important technical inspiration for
  coupling and decoupling design. Railway uses a different data and execution model,
  but that work has been an invaluable reference.
- The world representation is informed by GeoJSON and lightweight WebGIS conventions,
  extended toward a 3D-capable railway simulation.

## AI-assisted development

AI has been used extensively in this project for planning discussions, code
implementation, testing assistance, documentation, and review. Core design choices,
acceptance decisions, and reliability requirements remain under human responsibility.

## Get involved

Contributions and participation are very welcome—and genuinely needed. Whether you
want to report a bug, test a scenario, discuss simulation rules, improve the UI,
work on documentation, or contribute code, your help can make a real difference.

Contribution guidelines will be collected in `CONTRIBUTING.md`.

## License

Railway is licensed under the
[GNU General Public License v3.0 or later](LICENSE) (`GPL-3.0-or-later`).
