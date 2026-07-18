# Vector-First Map Model — Implementation Plan (SQ-0375 pivot)

> **For agentic workers:** Use superpowers:subagent-driven-development task-by-task.

**Goal:** Replace tile-space geometry *construction* with a continuous model:
layout, sizing, and routing solved in real-valued 2D with no ASCII constraints,
then projected onto the char grid. The tile grid (`TilePlan`) remains the raster
output format; the rasterizer, theming, config switch, background-job plumbing,
and scroll mapping from the spike are kept.

**Architecture:** `mapper::model` builds a `MapModel` (rooms as real rects,
corridors as true-angle polylines, features as points) from the graph; a
projection step scales the model to chars (x doubled for terminal cell aspect)
and emits a `TilePlan`. Zooms become scales of one model.

## Global constraints

- `mapper` zero-dep (std only); f32 geometry is fine; determinism required
  (fixed iteration order, hash-seeded variation only — no RNG/time).
- All geometry work stays on background jobs (SQ-0378/0379 rule).
- Styleable: any new visual distinction gets ColorScheme+style.toml selectors.
- Classic renderer untouched; `map_renderer = "tiles"` keeps gating everything.
- Branch `tile-map-spike`; stage by explicit path only.

## Model spec

```rust
pub struct MapModel {
    pub rooms: Vec<ModelRoom>,      // id, center: Vec2, half: Vec2 (rect half-extents)
    pub paths: Vec<ModelPath>,      // per connector: points: Vec<Vec2>, kind, distorted, reciprocal, door_a/door_b: Vec2
    pub features: Vec<ModelFeature>,// room-anchored stairs/portals: room, kind, at: Vec2
}
```
Model units: 1.0 ≈ one "room diameter" scale-free unit; isotropic (projection
handles terminal aspect). `Vec2 = (f32, f32)`.

**Room build:** sizes from door-count minimums + content + id-hash jitter
(continuous, e.g. 1.0–2.2 wide); positions seeded from the layout engine's
cells, then a continuous refinement pass reusing `vpsc.rs`/`stress.rs` with
rectangle-aware separation (gap ≥ half_a + half_b + margin per axis) so sizes
genuinely push neighbors — run per layer, deterministic iteration caps.
Connected neighbors may end at zero gap (abutting → shared wall after
projection); unconnected get a minimum margin.

**Routing:** obstacle-avoiding polylines with true angles. Fine-lattice A*
(8-directional, resolution ~1/4 unit) over room rects inflated by clearance;
cost prefers straight and 45° continuation, deterministic tie-breaks; endpoint
segments leave/enter at the compass direction's true angle for ≥ clearance
before free routing (directional honesty). Post: collinear simplification,
parallel-path offsetting (paths sharing a channel get real-valued lateral
offsets), optional hash-seeded waviness for long paths (meander lives here as
polyline styling). Crossings allowed; recorded, not forbidden.

**Z-levels (2.5D):** `ModelRoom.level: i32`, derived by longest-path layering
over the Up/Down subgraph (same technique as `sort.rs` per-axis layering; drop
cycle-closing edges, mark distorted). Continuous x/y solved per level; a soft
cross-level constraint pulls a stair's endpoints toward the same x/y
(stairwell coherence — replaces `stack_updown_rooms`). Levels can auto-suggest
floors; the manual layer system stays as override.

**Districts (map-of-maps):** partition each level's subgraph into districts —
connected components, then dense non-planar clusters split out (maze
detection: edge/room ratio and dropped-constraint density thresholds). Each
district is built and routed independently (small solves, parallelizable);
the district graph (districts as super-nodes sized by their bounding boxes,
inter-district edges aggregated) is laid out with the same rectangle-aware
refinement; compose by translating district-local coords into global space;
inter-district paths route last against the composed obstacle set. District
kinds: `Standard`, `Maze` (abstraction: pivot renders a maze district as one
organic chamber outline with member-room pockets and only externally-
meaningful doors on its boundary; full internal detail is a later zoom
feature).

**Projection → TilePlan:** scale s (chars per unit vertically; 2s horizontal).
Rooms snap to integer rects (min interior 3×1, walls 1 char, rooms whose model
rects touch project to shared wall lines); doors punched where each path's
door point meets the wall; polylines rasterized: orthogonal runs → `─│` path
tiles, 45° runs → diagonal corridor tiles, other slopes → stair-step; glyph
resolution stays in the app rasterizer (masks + new diagonal glyph slots
`╱ ╲` styleable). Path-tile collisions → Bridge. Existing passage rendering
(WallKind::Path sides) applies where corridor spacing permits; where the
projection is too tight for side walls, degrade to bare path line (small
scales must still read).

## Tasks

**V1 — `mapper::model` room build** (new file `crates/mapper/src/model.rs`):
types above; `build_model(graph, layer) -> MapModel` with rooms+features only
(paths empty); continuous refinement via vpsc/stress reuse; unit tests: sizes
respect door minimums; refinement removes rect overlaps while preserving
relative order (N stays above, etc.); determinism; abutting connected pairs
end within epsilon of zero gap.

**V2 — polyline router** (new file `crates/mapper/src/vroute.rs`): pure
functions over `(&[RectF], endpoints, dir hints) -> Vec<Vec2>`; A* + simplify +
offset + waviness as above; unit tests: obstacle avoidance, 45° diagonal
honesty, determinism, offset separation of two parallel paths, waviness bend
count on long hauls, bounded search fallback (straight line) flagged.

**V1.5 — z-levels** (extends model.rs after V1): level derivation by Up/Down
layering; stairwell soft x/y alignment folded into refinement; tests: chain of
Up edges yields strictly increasing levels; cycle dropped + distorted; stair
endpoints within epsilon x/y when unconstrained; determinism.

**D1 — districts** (new `crates/mapper/src/district.rs`): partition (per-level
components + maze-cluster splitting), per-district build via V1, district-
graph composition layout (reuse rectangle refinement with district bounding
boxes), global translation, inter-district endpoint export for routing; maze
districts emit a chamber outline (convex-ish hull of member pockets) instead
of per-room rects. Tests: two components compose without overlap; maze
detection fires on a dense non-planar fixture and not on a grid fixture;
composition determinism; stable district assignment as rooms are added
(a new room joins an existing district rather than reshuffling membership).

**V3 — projection** (extends model.rs or new `project.rs`): `project(model,
scale) -> TilePlan` per the spec; reuse TilePlan/TileRoom/TileConn; add
diagonal corridor capability to `Tile`/rasterizer glyphs (`tile.path_d1/_d2`
defaults `╱ ╲`); tests: room snap minimums, shared-wall projection, door
placement on walls, diagonal run rasterization, bridge on crossing, TilePlan
invariants (bounds, no floor overlap), determinism.

**V4 — integration:** the render-job call site switches to the full vector
pipeline behind tiles mode: partition (D1) → per-district build (V1/V1.5) +
routes (V2) → compose (D1) → inter-district routes (V2) → project (V3); scale
from zoom (Boxes = base scale; Compact/Overview stay classic for now); update
tiles_dump example (add optional scale arg); refresh app snapshot; keep
stability walk test meaningful (rework assertions to model-space stability:
relative order + size stability, projection determinism); workspace green,
clippy clean; docs status notes.

> **[x] V4 DONE (2026-07-17):** `mapper::vector::realize_layer_vector`
> orchestrates partition → per-district build+routes (reciprocal dedup, door
> fans, waviness 0.35 past 4 units, maze waviness 0) → compose → global
> model → cross-district dashed stair links → `project_with_chambers` (maze
> outlines stamped as Path walls). Wired into the background render job and
> `tiles_dump` (optional scale arg, default `VECTOR_SCALE_BOXES = 5.5`).
> Walk test drives both pipelines. `offset_parallel` deliberately skipped
> (moves door endpoints — re-anchor first); follow-up alongside per-level
> projection offsets for multi-floor districts.

## Verification

Workspace `cargo test` + clippy + release build; `tiles_dump` on map.txt
layers 0/1/3 eyeballed (organic angled paths, no stub regressions, readable at
scale); user smoke in-game (confirm-class: visual).
