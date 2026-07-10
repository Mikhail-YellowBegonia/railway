# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Python railway sandbox game inspired by Transport Fever 2 + AutoCAD. MVC architecture.
Editor is the current focus — see `docs/editor.md` for the authoritative design spec
and step-by-step implementation status.

## Commands

- **Run game**: `uv run python main.py` (loads `test_track.geojson`)
- **Add deps**: `uv add <package>`
- **Ad-hoc tests**: `uv run python -c "..."` — there is no test framework; the
  established pattern is heredoc scripts that import editor/network modules,
  drive them programmatically, and assert. Examples appear throughout the
  conversation history when verifying each Step.

There are no lint, typecheck, or unit-test commands configured. Don't add them
without asking.

## Architecture

```
model/       Pure data + geometry, no view/controller deps
  vec3.py            Vec3 (3D arithmetic)
  rail_network.py    Node, Edge, RailNetwork (graph + connectivity + split_edge_at + turn_allowed)
  geom_utils.py      All geometry: projections, tangents, solve_case2_arc,
                     solve_biarc, merge predicates, split_arc_b_points
  geojson_loader.py  / geojson_writer.py — round-trip-safe arc serialization
  pathfinding.py     Edge-based Dijkstra, Path dataclass, turn_allowed integration
  spatial_index.py   Tile-based spatial index (340× speedup, 60fps保障)

view/        pygame-ce rendering only
  camera.py          World→screen mapping, pan/zoom
  renderer.py        Network drawing + preview (dashed straights/arcs/biarcs)

controller/  Wires it together
  game_loop.py       Event loop, key/mouse routing, modifier-key polling
  editor.py          Editor state machine (modes + build substates)
  snap.py            SnapSystem with PointSnapProvider + PathSnapProvider
  build_plan.py      ConstructionPlan / PreviewGeometry dataclasses
```

Dependency rule: `model` must stay pure (no view/controller imports). `controller`
imports from `model` and `view`. `view` only sees `model`.

## Editor state machine

Top-level modes: **IDLE** / **BUILD** / **DELETE**. BUILD has substates
**BUILD_IDLE** / **BUILD_ACTIVE** (after first click). All rejection paths preserve
current state — never auto-revert on bad input. Only "not buildable" gets explicit
visual feedback (red preview); other rejections are silent. See `docs/editor.md` §1, §4.0.

## Build cases

| Case | Trigger | Geometry |
|------|---------|----------|
| 1 | No T1 candidate | Free straight, `edge_geometry=[]` |
| 2 | T1 only (M1 has tangent, M2 doesn't) | Single arc, `[B]` |
| 3 | T1 + T2 candidates | Equal-radius biarc → **two consecutive Edges + middle Node** (Plan A) |

Tangent semantics: T1/T2 candidates point **away from the other end** of their
incident edge. The "best" candidate at commit time is the one with max dot product
against `(M2 - M1)` for T1, and enumerated per-candidate for T2 (biarc may want
reverse-direction T2 in C/S shapes — never pre-filter T2 by direction).

Force-straight (LSHIFT held): degenerates to "straight along T1, length controlled
by mouse". M2 is projected onto the (M1, T1) ray; **direction comes from existing
track**, not cursor.

## Edge geometry conventions

- `Edge.geometry == []` → straight; length = `|node_a - node_b|`.
- `Edge.geometry == [B]` → arc through `node_a → B → node_b`, where B is the
  intersection of the two endpoint tangents. `RailNetwork.add_edge` auto-computes
  `arc_center / arc_radius / arc_angle_rad / arc_start_dir / arc_normal`.
- Biarc never produces a single edge with two B points — it always splits into
  two consecutive Edges + an automatically-inserted middle Node.
- All arc math assumes the **XY plane**: `PLANE_NORMAL = +Z`, `arc_normal ∈ {+Z, -Z}`.

## Snap system

`SnapSystem.snap(world_pos, network, reference_pos)` tries Providers in order:
1. **PointSnapProvider** (threshold 0.3) — snaps to nearest Node, returns all
   incident-edge tangent candidates (endpoint=1, switch=N, isolated=0)
2. **PathSnapProvider** (threshold 0.3) — snaps to closest Edge, returns
   `[forward, reverse]` tangent candidates

`reference_pos` is M1 in BUILD_ACTIVE, None otherwise. Used by PathSnapProvider
to pick the "best" tangent for live preview display (does not affect the
candidate list).

## DELETE mode

Hover priority: **Node > Edge**.

- Click an edge → remove it; isolated endpoints get cleaned up.
- Click a `connection_count == 2` Node → attempt merge (silent if invalid).
  Strict checks: collinear straights (cos ≥ 1−1e-6) OR same-center / same-radius
  / same-direction / tangent-continuous arcs. Mixed straight+arc never merges.

The two invariants after any DELETE: **no orphan nodes, no headless edges**.

## Truncation (Step 6)

Path-snap on M1 or M2 splits the underlying Edge at commit time via
`RailNetwork.split_edge_at(edge_id, t)`. Arc splits use `split_arc_b_points`
(tangent-intersection inverse) to keep `arc_center / arc_radius / arc_normal`
identical across split→merge round trips. M1 and M2 snapping to the same edge
is rejected silently.

## GeoJSON round-trip

LineString features: 2 points = straight, 3 points `[A, B, C]` = arc with B as
the tangent-intersection. Constraints: A/B/C not collinear, `|BA| = |BC|`.
Arc metadata (center/radius/normal) is reconstructed deterministically from B
on load — round-trip is bit-stable for arc geometry within 1e-4.

## Input reference

| Key / mouse | Mode | Action |
|-------------|------|--------|
| `B` | any | Switch to BUILD mode |
| `D` | any | Switch to DELETE mode |
| `Esc` | BUILD_ACTIVE | Cancel current build, return to BUILD_IDLE |
| `Esc` | other | Switch to IDLE |
| Right click | BUILD_ACTIVE | Cancel current build (same as Esc) |
| `Q` | any | Quit program |
| `S` | any | Save network to manual_track.geojson (loaded on startup if exists) |
| `F` | any | Toggle pathfinding test mode (debug) |
| `I` | any | Toggle spatial index visualization (debug) |
| `LSHIFT` (held) | BUILD_ACTIVE | Force straight along T1 |
| `LALT` (held) | BUILD_ACTIVE | Force Case 2T single-tangent arc (M2 path-snap to straight edge) |
| Left/Right/Middle drag | IDLE | Pan camera |
| Middle drag | BUILD/DELETE | Pan camera |
| Left click | Pathfinding test | Select start/goal nodes (F mode) |
| Scroll | any | Zoom |

Modifier keys are polled per frame in `GameLoop._sync_modifiers`, not edge-triggered.

## Pathfinding (转向许可与寻路)

**Turn permission** (`RailNetwork.turn_allowed`): 几何自动推断,无需道岔配置。
判据为**前进半平面**——到达节点的行进方向 `d_in` 与离开方向 `d_out` 夹角
严格 < 90°(`d_in · d_out > 0`)。这样直通(0°)和缓分股(~32°)许可,发卡弯
(~148°)、正交(90°)、掉头(180°)禁止。初版用 cos(150°) 阈值,交叉渡线的
4 联通点会误判 148° 发卡弯为许可(dot=-0.847 通过 >=-0.866),改为前进半平面
后彻底修复(合法/非法两侧余量极大:32° vs 148°)。

**Pathfinding** (`model/pathfinding.py`): Edge-based Dijkstra。搜索状态 =
有向边 `(edge_id, dir)`,`dir ∈ {+1, -1}`(+1 沿 node_a→node_b,-1 反向)。
邻接由 `turn_allowed` 决定,代价/可通行走 `cost_fn`/`passable_fn` 钩子
(信号层以后注入约束,不返工)。`Path` dataclass = 有向边序列 + total_cost,
就是给运动学层的契约。`find_path_between_nodes` 是节点间寻路入口,枚举
所有出发有向边取最短。

**Debug 测试**(F 键叠加态): 左键依次点选两节点 → 自动求路 → 控制台打印
段数/总长/有向边序列;第三次点击重置。可视化:起点绿圈/终点红圈,路径橙色
加粗,每段中点顺序编号 1,2,3…。

## Working with `docs/editor.md`

The editor design doc is the source of truth for behavior. §3 covers the full
snap system (Point/Grid/Parallel/Path + length/angle), §7 lists the completed
scope (Steps 0–6, §10.1–§10.5, §12.1, snap features, and spatial index, all ✅).
§11 records the current stance: editor stays the focus, spatial index shipped
(340× speedup, satisfies 60fps), next direction is advanced parallel snap
(multi-segment along shortest path); Z-axis deferred until visual debugging
catches up. **Pseudocode blocks in the doc are non-normative — implement to the
behavior description, not the code samples.** When changing editor behavior,
update the relevant §3 / §4 / §5 sections; when adding new follow-up
requirements, append them as §12+ items.

**Keep docs in sync with code (learned the hard way).** Docs have drifted behind
the code before — snap features shipped while editor.md still marked them
"待研讨". Rule: when a feature is finalized (functionality frozen), update the
docs in the SAME change. Backfilling / reorganizing older sections is optional
and can be deferred, but at minimum the doc MUST point out the latest progress
(mark it ✅ in §7 and note it in §11) so editor.md never lies about what exists.

## Conventions used in this codebase

- Always respond to the user in Chinese (project convention).
- Match existing file style: type hints throughout, `from __future__ import annotations`,
  dataclasses for plain data, snake_case Python.
- Geometry tolerances live as module constants (`SNAP_THRESHOLD`, `MAX_ARC_RADIUS`,
  `MERGE_RADIUS_TOL`, `MERGE_DIR_DOT_MIN`, `BIARC_COLLINEAR_DOT_MIN`). Add new
  ones at the call site's module.
- Prefer immutable returns; `Vec3` arithmetic is non-mutating.
