# Tile Map Renderer — Phase-1 Spike Implementation Plan

> **For agentic workers:** Use superpowers:subagent-driven-development to implement
> this plan task-by-task.

**Goal:** A tile-grid ("ASCII-art") map renderer behind a config switch, proving
shared walls, punched doors, walled corridors, and turn-to-turn stability on real
maps — plus the design for how mazes/overlapping paths work in the tile world.

**Architecture:** New `mapper::tiles` module realizes geometry (semantic `Tile`
grid) on top of the existing cell layout + lane router. New `app/render/tilemap.rs`
rasterizes tiles → themed glyphs. Config `map_renderer = "classic" | "tiles"`
selects exactly one renderer (never both).

**Tech stack:** Rust; `mapper` crate stays zero-dependency (std only); app uses
existing ratatui/style.toml infrastructure.

## Global Constraints

- `mapper` crate: **zero dependencies**, std only.
- **All map geometry work runs off the interpreter thread** (SQ-0378/0379 rule):
  tile realization happens inside the existing background render job; the draw
  pass only consumes cached results; the pulse runs while any map job is active.
- **Deterministic**: same graph → identical TileGrid. No wall-clock, no RNG;
  jitter only from hashing stable ids.
- **Styleable**: every new visual element gets a ColorScheme field + style.toml
  selector + render apply. No hard-coded styles.
- Old renderer stays fully functional; `map_renderer` defaults to `classic`.
- Commit per task on branch `tile-map-spike`; never `git add -A` (untracked
  files in the repo are not ours); stage explicitly by path.

---

## Design decisions locked for the spike

1. **Crossings are legal in tile space.** The classic renderer forbids connector
   overlaps, which is what drives `cleanup_overlaps` churn. Corridor crossings
   render as an explicit `Bridge` tile (over-corridor continuous, under-corridor
   interrupted). Consequence: the tiles path does NOT need the app-side overlap
   passes; positions come from the layout engine as-is.
2. **Reuse the lane router** (`mapper::route::route_lanes`) for corridor channel
   and lane assignment — it already handles reciprocal dedupe, T-junction merge
   stubs, and lane packing. What changes is realization: a lane becomes a walled
   corridor, not a 1-px polyline.
3. **Un-routable / distorted edges become paired stubs**, not magenta direct
   lines: a door + 1-tile corridor stub that fades out, on both rooms. (Maze
   plan §below expands this.)
4. **Diagonals**: corner-attached L-corridors for the spike (stair-stepping is a
   later visual experiment).
5. **Boxes zoom only.** In tiles mode, Compact/Overview still use the classic
   renderer. Portals: minimal — `<`/`>` stair features for Up/Down, `⊙`/`⊗`
   wall glyphs for In/Out; no dotted shafts.
6. **Names hidden** (spike): rooms show `#id` on the floor when
   `show_room_numbers` is on; the current room shows `@`.

---

## Geometry specification (normative for Task 1)

**Tile model** (`crates/mapper/src/tiles.rs`):

```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Tile {
    Void,
    Wall,
    Floor { room: RoomId },
    Corridor { conn: u16 },          // index into TilePlan::conns
    Door { conn: u16, kind: DoorKind },
    Bridge { over: u16, under: u16 },
    Feature { room: RoomId, kind: FeatureKind },
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DoorKind { TwoWay, OneWay(Direction), Stub(Direction) }
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FeatureKind { StairsUp, StairsDown, PortalIn, PortalOut }

pub struct TilePlan {
    pub w: usize,
    pub h: usize,
    pub tiles: Vec<Tile>,                     // row-major, w*h
    pub rooms: Vec<TileRoom>,                 // id, tile-space bounds (incl. walls), floor bounds
    pub conns: Vec<TileConn>,                 // origin, dest, dir, reciprocal, distorted
}
pub fn realize_layer(graph: &MapGraph, layer: LayerId) -> TilePlan;
```

**Sizing (S1).** Per room: `doors_h(side)` = count of connections attaching on
top/bottom, `doors_v(side)` = left/right (side assignment identical to the
router's `Side` logic; diagonals count toward their corner's two sides).
`jitter = fxhash-style hash of room id` (implement a tiny FNV-1a inline — no deps),
giving `jw ∈ {0,2}`, `jh ∈ {0,2}`.

```
floor_w = clamp(max(5 + jw, 2*doors_top+1, 2*doors_bottom+1), 5, 11)   // odd
floor_h = clamp(max(3 + jh, 2*doors_left+1, 2*doors_right+1), 3, 7)    // odd
room box = (floor_w + 2) × (floor_h + 2)                                // walls
```

**Tracks (S2).** `col_w(c) = max box width of rooms in logical col c` (min 5 if
somehow empty); `row_h(r)` likewise. Gutter after col `c`:
`gv(c) = 0` if the vertical channel V(c) carries no corridor lanes, else
`2k+1` for `k` lanes (floors at odd offsets `1,3,…,2k−1`, walls painted around).
X origin accumulation: `x(c+1) = x(c) + col_w(c) − (gv(c)==0 ? 1 : 0) + gv(c)`
— i.e. **zero-gutter neighbors overlap by one column: the shared wall**.
Rows identical with `gh(r)`.

**Placement + abutment (S3).** Each room's box is centered in its (col,row)
track cell. For every cardinal connection between rooms in *adjacent* tracks
with zero gutter between them: extend both boxes to the shared track boundary
column/row (grow floor, wall lands on the shared line). Punch `Door` at the
midpoint of the two floors' overlap span (fan multiple doors between the same
pair with the existing slot-offset ordering: 0, +1, −1, …). Adjacent but
unconnected rooms may also end up sharing the wall — no door. Reciprocal pair →
one `TwoWay` door; one-way → `OneWay(dir)`.

**Corridors (S4).** Every connection not realized as a shared-wall door gets a
corridor: door tile on each room wall (slot-offset on the correct side), a
perpendicular stub into the channel, lane runs from the route plan mapped to
tile coordinates (channel V(c) lane j → column `gutter_x(c) + 2j+1`; H similar),
L-bends at lane intersections. Paint pass afterwards: every `Void` tile
8-adjacent to a `Corridor` or `Door` tile becomes `Wall`. Where two corridor
floors cross perpendicular, write `Bridge` (deterministic over/under: lower
conn index goes over). Route-failure edges: `Door { kind: Stub(dir) }` +
one corridor tile on each endpoint, no connecting path.

**Portals (S5-minimal).** Up → `Feature(StairsUp)` on the floor tile nearest
the room's top-right interior corner; Down → StairsDown bottom-right; In/Out →
`PortalIn/Out` feature replacing a wall tile mid-right (mirrors today's
mid-slot). Skip Unknown (parity with classic).

---

## Task 1: `mapper::tiles` — realization core

**Files:** Create `crates/mapper/src/tiles.rs`; modify `crates/mapper/src/lib.rs`
(export). Tests inline `#[cfg(test)]` per crate convention.

**Steps:**
- [ ] Types + `realize_layer` skeleton returning empty plan; unit test scaffold.
- [ ] S1 sizing + S2 tracks (pure functions, tested directly: door-count → min
      sizes; determinism of jitter).
- [ ] S3 placement/abutment/doors. Tests: two rooms A—E⇄W—B adjacent ⇒ exactly
      one shared wall column, exactly one `TwoWay` door in it; one-way ⇒
      `OneWay`; unconnected neighbors ⇒ shared wall, zero doors.
- [ ] S4 corridors + wall paint + bridges. Tests: rooms two columns apart ⇒
      corridor with walls both sides, doors at both ends; two perpendicular
      corridors ⇒ ≥1 `Bridge`; route-failure edge ⇒ paired `Stub` doors.
- [ ] S5 portal features. Test: Up edge ⇒ `StairsUp` on origin floor.
- [ ] Invariant tests: determinism (`realize_layer` twice ⇒ identical);
      **every connection is represented** (door, corridor, or stub pair —
      count assertion over a 20-room seeded graph); no `Floor` of two rooms
      overlapping; grid bounds consistent.
- [ ] `cargo test -p mapper` green; commit.

## Task 2: app rasterizer + theme

**Files:** Create `crates/app/src/render/tilemap.rs`; modify
`crates/app/src/colors.rs` (ColorScheme fields), `crates/app/src/style.rs`
(selectors), `style.toml` (documented defaults), `crates/app/src/render/mod.rs`.

**Interfaces:** consumes `mapper::tiles::TilePlan`; produces
`render_tile_map(plan, state, area, buf)` and
`tile_room_screen_rects(plan, state, area) -> Vec<(RoomId, Rect)>`.

**Steps:**
- [ ] Theme = glyph+style table resolved from ColorScheme/SymbolSet: selectors
      `map.tile.wall`, `map.tile.floor`, `map.tile.corridor`, `map.tile.door`,
      `map.tile.bridge`, `map.tile.stairs`, `map.tile.player`, `map.tile.room-number`.
      Spike glyph defaults: wall `█`, floor `·` (dim), corridor `·`, door `∩`
      (one-way: `▸▾◂▴` by dir; stub: `?`), bridge `╪`, stairs `<` `>`,
      in/out `⊙` `⊗`, player `@`.
- [ ] Rasterize with scroll offset (1 tile = 1 cell; reuse pan/scroll state;
      clamp to plan bounds), `#id` centered on floor when `show_room_numbers`,
      `@` on current room floor center (id shifts up one row if both).
- [ ] Snapshot test: small 3-room graph → exact string frame (strip styles).
- [ ] `cargo test -p app` green, clippy clean; commit.

## Task 3: config switch + background integration + input

**Files:** Modify `crates/app/src/config.rs` (`map_renderer` enum, default
Classic), `crates/app/src/state.rs` (cached `tile_plan: Option<(u64, TilePlan)>`
keyed by `graph_gen` inside the render-job result), the render-job spawn/poll
path (`state.rs`/`loop_tick.rs` — extend `RenderJob` to also produce a
`TilePlan` when tiles mode is on), `crates/app/src/main.rs` (draw branch),
`crates/app/src/input.rs` + `slash::COMMANDS` registry (command
`toggle-map-renderer`, follow registry naming conventions on inspection),
config docs.

**Steps:**
- [ ] Config parse + enum + docs; command in the single `slash::COMMANDS`
      registry flipping the runtime setting.
- [ ] Extend the background render job: when tiles mode, job returns
      `(RenderMap, Some(TilePlan))`; coalescing/staleness/pulse semantics
      unchanged. Draw path: tiles mode + Boxes zoom → `render_tile_map` from
      cache (classic renderer as fallback while the first plan is in flight);
      other zooms and classic mode → existing path untouched.
- [ ] Mouse: in tiles mode use `tile_room_screen_rects` for `room_rects` so
      click/hover actions keep working.
- [ ] In tiles mode, skip scheduling `cleanup_overlaps`-motivated background
      work? **No** — leave scheduling untouched for the spike (positions still
      shared with classic mode); note as maze-plan follow-up.
- [ ] Workspace build + full `cargo test` green; commit.

## Task 4: smoke + docs

- [ ] Headless stability check: drive the mapper with a scripted Zork-like
      walk (reuse existing mapper test-graph helpers), realize after each of
      ~30 observations, assert no panic and that previously-realized room
      floor rects only change when their own row/col track grows (stability
      regression guard).
- [ ] Update `docs/ascii-art-map-concept.md` status note + README one-liner
      (experimental tile map view, config `map_renderer = "tiles"`).
- [ ] Commit.

---

## Maze / overlapping-paths plan (design, not built in this spike)

The maze layer is where the classic approach is weakest, because its core
constraint — *connector lines must never overlap* — is impossible to satisfy
for non-planar edge sets, so the pipeline thrashes (cleanup passes, distorted
magenta edges, dropped constraints). The tile world changes the rules:

1. **Crossings become first-class.** Perpendicular corridor crossings render
   as bridges (over-path continuous; under-path breaks with `╨`/`╥` ends or
   shading). Planarity is no longer required — only *local* legibility.
   Long-term this lets the tiles path drop `cleanup_overlaps` and
   `repair_directional_hints` entirely (they exist only to protect line-art).
2. **Stubs as honest teleports.** Edges that would need absurd routes (maze
   back-edges, `109 ? 16`-style unknowns) become paired stubs: a door +
   corridor fading into `░▒` darkness, hover/label shows the destination
   ("continues to Maze #201"), with the matching stub on the far room. This is
   what hand-drawn RexPaint maps do with numbered references — and it reads
   *better* for mazes ("twisty passages" shouldn't look tidy).
3. **Cave theme**: maze rooms as seeded blob chambers (erode/dilate the rect
   outline, doors and shared walls clamped intact); corridors jitter within
   their channel envelope into winding tunnels.
4. **Budgeted routing, not exhaustive**: route the planar-embeddable subset
   (the layout engine already identifies dropped/distorted edges); everything
   else is stubs by policy, not failure. Deterministic and cheap — no more
   multi-pass overlap hill-climbing for mazes.
5. Open question to revisit after the spike: should dense maze clusters
   optionally merge into one large cavern floor with internal pillar walls
   (chamber view), with room identity shown only on hover?

## Verification

- `cargo test` workspace green; `cargo clippy` clean; release build.
- Manual smoke (user): run Zork, `map_renderer = "tiles"`, walk the house/
  forest loop and the cellar; flip `toggle-map-renderer` live; confirm pulse
  during map jobs and no interpreter-thread stalls. (Track as `confirm` —
  visual/TTY rendering is exactly the untestable class.)
