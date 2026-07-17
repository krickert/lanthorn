# ASCII-Art Mapper Concept (SQ-0375)

> Design thinking for turning babelmap's live automap from "boxes connected by
> lines" into RexPaint-quality ASCII artwork, generated at runtime as the game
> is played. Companion to `mapping-rules-idea.md` (current-behavior snapshot)
> and `mapping_rules_concept.md` (layout rules draft). Nothing here is
> committed work — it is the evaluation SQ-0375 asked for.

---

## 1. What the inspiration actually does (and we don't)

Studying the four RexPaint gallery pieces and the Zorbus generator output,
the look comes from seven concrete ingredients:

1. **Rooms are places, not nodes.** Variable-sized floor *areas* filled with a
   floor texture (`·` dots, grass, rubble), not fixed 11×5 label boxes.
2. **Walls are material and shared.** Thick/textured wall runs (`█ ▓ ##`,
   double-line) that two adjacent rooms *share*. Adjacency is meaningful:
   connected neighbors touch.
3. **Doors pierce walls.** Where an exit exists between abutting rooms, the
   wall gets a door glyph (`∩`, `+`, or simply a gap). The door *is* the
   connection — no line-art needed for adjacent rooms.
4. **Corridors are skinny rooms.** A path between distant rooms is a
   1-tile-wide floor strip *with its own walls*, not a 1-px polyline.
5. **Interior life.** Item/feature glyphs on the floor (statue `*`, water `≈`,
   stairs `<` `>`), a player `@`, colored accents.
6. **Theme = material.** The same geometry reads as stone keep, cave, forest,
   or ship purely from the glyph palette + colors.
7. **Negative space.** Void (untouched background) between structures;
   outdoor maps surround areas with terrain instead of walls.

Our current render has none of these: uniform boxes, zero-thickness lane
polylines, connections drawn even between adjacent rooms, no interiors.

## 2. The core model shift: node-graph raster → tile grid

Everything above falls out of one change of representation. Instead of
rasterizing rooms-as-boxes + connectors-as-polylines straight into the ratatui
buffer, we generate an intermediate **tile grid**:

```rust
enum Tile {
    Void,                       // untouched background / outdoor terrain
    Wall { owner: WallKind },   // room wall, corridor wall, shared wall
    Floor { room: RoomId },     // interior floor of a room
    Corridor { conn: ConnId },  // corridor floor
    Door { conn: ConnId, kind: DoorKind }, // door/opening/one-way/secret-ish
    Feature(FeatureKind),       // stairs, portal, water, decoration
}
```

The tile grid is the *semantic* map. A separate, cheap pass turns tiles into
styled glyphs using the active **theme** (per layer). This buys us, for free:

- **Hit-testing becomes a lookup.** Mouse over cell → tile → room/item/door.
  Today we hit-test rectangles; tomorrow every glyph knows what it is.
- **Theming is trivially styleable** (style.toml selectors — standing rule):
  a theme is just a glyph+color table keyed by tile kind.
- **Overlays are compositional**: player marker, item glyphs, labels, and
  animation write *on top of* tiles without touching geometry.
- **The dump stays honest**: map.txt becomes the same tile→glyph pass with
  colors stripped.

This is the inverse of roguelike dungeon generation: Zorbus et al. generate
geometry first and derive the topology; we *know* the topology (from play)
and must realize geometry for it. That inverse problem is exactly what our
existing layout engine already half-solves — which drives the recommendation
below.

## 3. Generation pipeline (recommended: realize geometry on top of the existing layout)

Keep the entire logical-placement stack (incremental placement, chains,
SMACOF/VPSC relayout, layers, background jobs). It solves the genuinely hard
problem — stable, compass-faithful *relative* positions — and it is the part
we've already invested five phases in. Replace everything downstream of
"rooms have integer cells" with new stages:

```
(existing)  observe → place_incremental / relayout_auto → cells (col,row), chains, layers
(new) S1    Size      each room gets a tile footprint w×h
(new) S2    Tracks    logical col/row → tile-space tracks (variable widths, gutter 0 by default)
(new) S3    Abut      connected cardinal neighbors expand to share a wall; door punched in it
(new) S4    Carve     non-adjacent connections become walled corridors through gutters (lane router reused)
(new) S5    Portals   Up/Down → stair features; In/Out → door variants; cross-layer → highlighted stairs
(new) S6    Theme     tiles → glyphs+colors per layer theme (indoor / outdoor / cave / …)
(new) S7    Overlay   player @, items, labels, unexplored-exit stubs, animations
```

All of S1–S5 runs in the existing background tidy/render worker (SQ-0378/0379
architecture) keyed by `graph_gen`; S6–S7 are cheap per-frame passes.

### S1 — Sizing

Room footprint (floor area, excluding walls) driven by:
- **exit count per side** (each door on a side needs ≥1 wall tile of spacing:
  a side with k doors needs floor length ≥ 2k+1);
- **content**: item count, notes, portals (stairs need a floor tile);
- **importance**: mild growth with degree / visit count;
- **deterministic jitter**: hash(room id) → ±1 tile per axis so the map
  doesn't look stamped. Deterministic ⇒ stable across turns.

Names do **not** drive size: per the quest vision, names are hidden by
default (shown on shift/hover), so rooms can stay compact (floor from ~5×3
up to ~13×7). Quantize to odd sizes so doors can center.

### S2 — Tracks

Same idea as today's `boxes_axes` PosTable, in tile units: track width =
max footprint width of rooms in that logical column (+2 for walls), etc.
**Default gutter between tracks = 0** — this is what makes walls shareable,
and it matches the `mapping_rules_concept.md` rule "grid line spacing = 0;
spacing is added only where paths need to travel." Gutters open up (S4) only
on tracks that must carry corridors, exactly like today's lane-count-driven
`channel_width`.

### S3 — Abutment & shared walls

For each cardinal connection between rooms in adjacent tracks with no
corridor between them: extend both footprints to the shared track boundary;
the boundary line becomes one wall run owned by both; punch a `Door` tile
where the connection crosses (slot-offset for multiple doors, reusing the
existing `slot_offset` fan-out logic). Adjacent-but-unconnected rooms also
share the wall — just with no door (quest note: "Rooms directly next to each
other can share walls"). One-way passages get a directional door glyph
(e.g. `▸` on the floor just inside, or a themed one-way door).

### S4 — Corridor carving

Connections that span >1 track, diagonals, and lane-contended edges become
corridors: floor strip + walls, routed with the *existing* lane router
semantics (channels on odd doubled coords, lanes within channels). A lane
becomes a 3-tile-wide corridor (wall·floor·wall); parallel corridors share
their separating wall, so k lanes cost 2k+1 tiles, not 3k. Corridor meets
room wall → door. Reciprocal dedupe, T-junction merge stubs, and the
crossing rules carry over unchanged — they're topology logic, not raster
logic. Crossing corridors render as a bridge/junction tile.

Diagonals (Zork is full of them): route as stair-stepped corridors using the
half-diagonal glyphs (`🮠🮡🮢🮣`) we already ship for corner rounding, or as an
L-corridor leaving from the room *corner* (current corner-anchor behavior).
Stair-step reads more organically; needs a prototype to judge.

### S5 — Portals

- **Up/Down**: stair feature tiles on the room floor — themed (`<`/`>`
  roguelike-classic, `▲`/`▼`, or `≣` steps). Cleanly stacked partners can
  additionally share a dotted shaft as today.
- **In/Out**: door-variant glyph in the wall (`⊙`/`⊗` retained, or a themed
  archway).
- **Cross-layer** (existing layer badges): highlighted stairs + the layer tab
  strip we already have.

### S6 — Theming

A theme is a style.toml-declared table: wall glyph set (straight runs,
corners, junctions), floor texture (weighted glyph choices, seeded
deterministically per tile), door set, stair set, terrain accents, colors.
Layers already are the biome mechanism (map.txt example: Main = above
ground, Cellar = underground, Maze = cave) — so **theme is a per-layer
setting**, default from a global theme, overridable per layer.

Sketch of the initial three:

| | walls | floor | doors | flourish |
|---|---|---|---|---|
| **indoor / house** | `█▓` solid or `═║╔╗` double | sparse `·` | `∩` | furniture-ish glyphs from notes |
| **outdoor / forest** | *none* — areas bounded by tree/foliage runs `♣♠` | grass `. , '` | trail gaps | paths are open `░` trails, no walls |
| **cave / maze** | irregular `▒▓░` with seeded roughness | `·` + rubble `%` | gap or `+` | room outlines jittered into blobs |

Cave blobs: take the rectangular footprint, then erode/dilate the outline
with deterministic per-room noise (a few iterations of seeded cellular
automata *inside the footprint's bounding box only*, clamped to keep doors
and shared walls intact). Organic without instability.

### S7 — Overlays & interaction (the in-game "awesome" layer)

- **Player marker.** `@` (themed) standing on the current room's floor —
  replaces the heavy-outline current-room box. On movement, a short
  (~250ms, skippable, presentation-only) animation walks the glyph through
  the door and along the corridor path — we already know the exact tile
  path from S4. Pure render-thread work; no map-thread involvement.
- **Items.** `engine.introspect().room_objects(id)` (already used by the
  room panel) → up to N item glyphs placed deterministically on floor tiles.
  Mouse hover over an item glyph → popup with the item name (tile grid makes
  the hit-test a lookup). Themed glyph classes (weapon `†`, light `☼`,
  container `▯`, generic `•`) via a small name-keyword table, with a generic
  fallback — never guess hard.
- **Names.** Hidden by default. Reveal:
  - *Hover*: mouse over a room → floating name tooltip (works everywhere).
  - *Shift-hold*: overlay all names while Shift is held. ⚠ Plain terminals
    don't report key release; this needs the kitty keyboard protocol
    (crossterm `PushKeyboardEnhancementFlags::REPORT_EVENT_TYPES`), which
    works in kitty/WezTerm/foot/recent iTerm2 but not everywhere. Design:
    use shift-hold where the terminal supports it, otherwise a toggle key
    cycles labels (off → current-room → all). Verify protocol support
    matrix against crossterm docs before building (standing rule: verify
    external constants).
  - Labels draw in the room's floor if it fits, else in a floated box beside
    it (RexPaint pieces use numbered keys; we have hover instead).
- **Unexplored exits** (synergy with the tried-directions quest SQ-0384-ish):
  a door with `?`, or a corridor stub fading into `░▒` darkness — reads as
  "passage into the unknown", which is thematically perfect for IF.
- **Room numbers / align codes** stay available as debug overlays.

## 4. Alternatives considered

**B. Full re-solve in tile space** — treat room rectangles (positions *and*
sizes) as variables in one VPSC/stress solve over tile coordinates, with
overlap and adjacency constraints. Prettier global packing, but: a much
bigger rewrite, size-position coupling makes incremental stability hard
(rooms would breathe as the solve rebalances), and it discards working code.
Worth revisiting as a *tightening pass* (S2.5) later, not as the foundation.

**C. Roguelike-style stochastic carving** (Zorbus / Mike Anderson 1999):
generate organic dungeons by random room placement + corridor growth,
constrained to match the observed graph. Produces the most "authentic"
dungeon look, but it's fundamentally unstable turn-to-turn (a new room can
reshuffle everything) and topology fidelity becomes a search problem. Wrong
tool for a *live* automap. Could power a future "export pretty map" offline
mode where stability doesn't matter.

**A wins**: geometry realization on top of the existing stable layout.
Incremental, testable, zero-dep (tile grid + themes have no new
dependencies; mapper crate stays std-only — tile realization can live in
mapper, glyph/color theming in app).

## 5. Zoom & coexistence

- The tile map becomes the flagship **Boxes-replacement zoom** ("Art" view).
  Tile maps are larger than the box view (a 7×5-floor room + walls ≈ 9×7
  vs. 11×5 box but corridors cost less than today's gutters) — net similar
  footprint, denser look.
- **Compact** zoom can render the same tile grid scaled: room = solid floor
  color block with walls, no interiors (like the Zorbus previews).
- **Overview** stays as-is (1 glyph/room) or becomes the tile grid at 1:4.
- Keep the schematic box renderer behind a toggle during transition; retire
  it (or keep as "diagram mode") once Art view is trusted.

## 6. Stability & performance requirements

- **Deterministic**: same graph → identical map. All jitter/noise seeded by
  room id / tile coord. No wall-clock, no RNG state.
- **Incrementally stable**: a new room may widen its own tracks and open a
  gutter, but never reshuffles distant geometry (inherited from the layout
  engine's incremental regime; S1 sizes change only when a room's own
  exits/contents change).
- **All geometry work off the interpreter thread** (SQ-0378/0379 absolute
  rule): S1–S5 run in the existing background render/tidy jobs, pulse while
  active, abort-and-replace on rapid movement. S6–S7 are per-frame paints.
- Tile grid memory is trivial (Zork ≈ 200×120 tiles/layer).

## 7. Suggested phasing

1. **Tile core** (flagship risk-retirement): tile grid type + S1–S4 for the
   indoor theme only, flat colors, behind a view toggle. Prove shared walls,
   doors, corridors, stability on the Zork map.
2. **Themes**: style.toml theme tables, outdoor + cave (blob jitter),
   per-layer theme selection, portals/stairs (S5).
3. **Interaction**: @ marker, item glyphs + hover popups, name reveal
   (hover first, shift-hold where supported), unexplored-exit stubs.
4. **Motion & polish**: walk animation, current-room glow/pulse, Compact/
   Overview from tiles, map.txt dump from tiles, retire/park box renderer.

## 8. Open questions

1. Replace Boxes zoom outright, or add "Art" as a fourth view with the box
   renderer kept as a diagram mode?
2. Room size character: near-uniform with ±1 jitter (tidy, blueprint-like,
   insp2/insp4) vs. strongly content-driven (organic, insp3)? Affects S1.
3. Diagonal corridors: stair-stepped vs. corner-L? Needs a visual prototype.
4. Outdoor theme: are trails walled paths, open `░` routes, or implied by
   clearing adjacency? (Zork's forest wants open terrain.)
5. Item glyph classing: keyword table acceptable, or single generic glyph
   until a per-game override mechanism exists?
6. Where does tile realization live — `mapper` (std-only, testable, keeps
   "layout logic split across two crates" from getting worse) vs `app`?
   Leaning `mapper` for S1–S5, `app` for S6–S7.

---

## Status (2026-07-17)

The phase-1 spike is implemented on branch `tile-map-spike`, behind
`map_renderer = "tiles"` in the config and the `toggle-map-renderer` command
(Boxes zoom only; `classic` remains the default). It covers §7 phase 1 with the
indoor glyph set only: tile realization in `mapper::tiles` (S1–S5 — shared
walls, punched doors, walled corridors, bridges, minimal stairs/portals), a
themeable rasterizer in `app/render/tilemap.rs`, and a per-observation
stability regression test. Maze/overlapping-path handling, outdoor/cave
theming, and motion polish remain design-only — see § "Maze /
overlapping-paths plan" in `docs/superpowers/plans/2026-07-17-tile-map-spike.md`.
