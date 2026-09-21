# Railway

Railway is a 2D railway sandbox built with Python and pygame-ce. It combines CAD-like
track construction with train consists, one-way path-based signalling, fixed-route
schedule plans, coupling, decoupling, and shunting operations.

The project is approaching its first usable demo. The core editor, signalling,
session persistence, schedule execution, and fixed-scenario shunting loop have been
implemented and manually tested.

## Current capabilities

- Build straight, curved, biarc, and composite track geometry with snapping.
- Place directional signals and run trains through reserved path blocks.
- Create, couple, decouple, reverse, save, and restore multi-wagon consists.
- Attach plans to control cars and freeze complete directed routes at edit time.
- Execute looping plans without runtime pathfinding.
- Add fixed-edge coupling, logical-position decoupling, waiting, and strict
  `simple_segment` reversal commands to a plan.
- Pause and resume plan autopilot without advancing its instruction pointer.

## Requirements

- Python 3.11 or newer
- [uv](https://docs.astral.sh/uv/)
- A platform supported by pygame-ce

## Run

```bash
uv sync
uv run python main.py
```

The game starts from `test_track.geojson`. If `manual_track.geojson` exists, it is
loaded as the current saved session instead.

## Essential controls

| Input | Action |
|---|---|
| `P` | Enter PLAY mode; with a parked train selected, enter or leave plan editing |
| Left click | Select a train, place a train, or add plan anchors while editing |
| Right click | Issue a manual destination order in PLAY mode |
| `B` / `D` / `H` | Track build / delete / signal mode |
| `S` | Save the network, signals, and trains to `manual_track.geojson` |
| `I` | Open the selected train's consist panel in PLAY mode |
| `K` | Couple or decouple at the highlighted coupler; add the corresponding plan command while editing |
| `R` | Reverse a parked train; add a reversal command while editing |
| `W` | Add a wait-for-coupling command while editing |
| `Enter` | Freeze the current plan route candidate |
| `Backspace` | Undo the current plan destination or last anchor |
| `Delete` / `Ctrl+Delete` | Delete the current plan item / clear the plan while editing |
| `Space` | Pause or resume plan autopilot; emergency-stop a train without a plan |
| `O` | Open the read-only schedule menu |
| `Esc` | Close the active overlay or cancel the current operation |

The complete editor and interaction reference is in [docs/editor.md](docs/editor.md).

## Suggested demo flow

1. Enter PLAY mode and place or select a train.
2. Use the consist panel to choose the active control car if needed.
3. Enter plan editing and click track nodes or edges to build and freeze route items.
4. Add coupling, decoupling, waiting, or reversal commands where required.
5. Leave plan editing and watch the train consume only the frozen routes.
6. Press `Space` to pause the plan completely and press it again to continue.

Plans currently live only in the running session; saving plan data is intentionally
outside the first demo scope.

## Tests

The repository uses executable regression scripts rather than a test runner:

```bash
for test_file in tests/test_*.py; do
  PYTHONPATH=. SDL_VIDEODRIVER=dummy uv run python "$test_file" || exit 1
done
```

## Documentation

- [Project roadmap](docs/roadmap.md)
- [Schedule-plan implementation roadmap](docs/plan_layer_roadmap.md)
- [Current progress snapshot](docs/progress_snapshot.md)
- [Editor specification](docs/editor.md)
- [Train control and signalling](docs/train_control.md)
- [Coupling and consist interaction](docs/consist_ui.md)
- [Session persistence](docs/session_persistence.md)

## Known limitations

- No collision simulation; trains may visually pass through one another outside the
  controlled coupling path.
- No physical reverse-driving primitive. Logical reversal changes the train's travel
  direction while preserving each wagon's physical orientation.
- Plans are not persisted yet.
- POIs, cargo, advanced vehicle physics, and LOD are outside the first demo scope.
- A declarative coupling selector is still required before the demo release. It will
  resolve a range/filter intent to a concrete wagon coupler and then reuse the current
  coupling execution path.

## License

Copyright (C) 2026 Mikhail.

Current and future versions are licensed under the GNU General Public License,
version 3 or (at your option) any later version (`GPL-3.0-or-later`). See
[LICENSE](LICENSE) for the complete terms.

The repository was previously published under the MIT License through commit
`d10c23e`. Rights already granted for those historical versions remain valid; the
GPL applies from the relicensing commit onward.
