# AGENTS.md

## Project
Python train driving sandbox game, MVC architecture.

## Essential commands
- **Run**: `uv run python main.py`
- **Add deps**: `uv add <package>`
- **Ad-hoc test**: `uv run python -c "..."` (no test framework yet)

## Architecture
```
model/     # No deps on view/controller. Pure data: Vec3, Node, Edge, RailNetwork
view/      # pygame-ce rendering: Camera, Renderer
controller/ # GameLoop, Editor (depends on model + view)
```

## Key conventions
- **World coords**: Y-up (meters). Camera flips Y for screen (Y-down).
- **GeoJSON**: LineString features, 2pts=straight, 3pts=arc [A,B,C]. Collinear/|BA|≠|BC| → rejected.
- **Edge.geometry**: `[]` for straight, `[B_midpoint]` for arc. Arc center/radius/angle auto-computed.
- **Node connectivity**: Default full-interconnect at each node. `connectivity_for()` returns groups.
- **Node colors by connection count**: 1=red, 2=white, 3=yellow, 4=green.
- **Round-trip**: `load_geojson()` → `write_geojson()` preserves arc geometry exactly (B point stored).

## Editor controls
| Key | Mode | Action |
|-----|------|--------|
| P   | Place node | Click empty space |
| C   | Connect | Click source node → click target (or empty to auto-create node) |
| D   | Delete | Click node or hovered edge |
| Middle-drag | Pan | |
| Scroll | Zoom | |
| Esc | Quit | |

## Input flow
Left-click → editor action. Middle-drag → camera pan (handled in GameLoop, bypasses Camera.handle_event). Scroll → camera zoom.

## Constraints
- Only straight-edge editing implemented; arc editing pending design doc.
- No lint/typecheck/test commands configured.
- `project.md` is the design doc — update it for architectural decisions.
