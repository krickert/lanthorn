//! Tile-grid geometry realization (tile map spike, stage S1–S5): rooms become
//! variable-size walled floor areas, adjacent connected rooms share a wall with a
//! punched door, distant connections become meandering walled passages found by
//! a noise-biased A* through the free space between rooms (the hand-drawn Reed
//! Zork map is the reference look), and perpendicular corridor crossings become
//! explicit bridges.
//!
//! `realize_layer` sits ON TOP of the existing cell layout + lane router: room
//! cells come from `Room::pos`, and the router still supplies door sides/slots
//! and the per-channel lane counts that size S2's gutters — only the corridor
//! SHAPE comes from the meander search. The output `TilePlan` is a semantic
//! grid; rasterizing tiles into themed glyphs is the app's job. Deterministic
//! by construction: the only "random" input is FNV-1a hashing of stable ids
//! (size jitter, meander noise).

use std::collections::BTreeMap;

use crate::direction::{grid_offset, is_diagonal, opposite, Direction};
use crate::graph::{MapGraph, RoomId};
use crate::layer::LayerId;
use crate::route::{exit_point, route_lanes, RoutedConnector};
use crate::router::Side;

/// One semantic map tile. `Corridor`/`Door` carry an index into [`TilePlan::conns`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Tile {
    Void,
    Wall { kind: WallKind },
    Floor { room: RoomId },
    Corridor { conn: u16 },
    /// A 45° diagonal corridor step (vector projection); renders as `╱`/`╲`.
    CorridorDiag { conn: u16, slope: DiagSlope },
    Door { conn: u16, kind: DoorKind },
    /// A perpendicular corridor crossing: `over` runs continuous, `under` breaks.
    Bridge { over: u16, under: u16 },
    Feature { room: RoomId, kind: FeatureKind },
}

/// The slope of a [`Tile::CorridorDiag`] step, named for its glyph shape.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DiagSlope {
    /// `╲` — descending left-to-right (SE/NW travel).
    Down,
    /// `╱` — ascending left-to-right (NE/SW travel).
    Up,
}

/// What a wall tile belongs to, so the rasterizer can style room outlines and
/// corridor passage sides differently.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WallKind {
    /// A room box's structural ring.
    Room,
    /// A painted corridor side — the walls that make a passage.
    Path,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DoorKind {
    TwoWay,
    OneWay(Direction),
    /// Dead-end stub door for an edge with no drawable route (distorted fallback
    /// or a multi-edge merge); the direction is where the passage continues.
    Stub(Direction),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FeatureKind {
    StairsUp,
    StairsDown,
    PortalIn,
    PortalOut,
}

/// Tile-space rectangle spanning `x..x+w` × `y..y+h`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Rect {
    pub x: usize,
    pub y: usize,
    pub w: usize,
    pub h: usize,
}

impl Rect {
    pub fn right(&self) -> usize {
        self.x + self.w - 1
    }
    pub fn bottom(&self) -> usize {
        self.y + self.h - 1
    }
}

/// A realized room: its tile-space box (walls included) and interior floor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TileRoom {
    pub id: RoomId,
    pub bounds: Rect,
    pub floor: Rect,
}

/// One realized connection — the index space `Tile::Corridor`/`Tile::Door` point into.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TileConn {
    pub origin: RoomId,
    pub dest: RoomId,
    pub dir: Direction,
    pub reciprocal: bool,
    pub distorted: bool,
}

/// The realized tile grid for one layer (row-major, `w*h` tiles).
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct TilePlan {
    pub w: usize,
    pub h: usize,
    pub tiles: Vec<Tile>,
    pub rooms: Vec<TileRoom>,
    pub conns: Vec<TileConn>,
    /// Sorted `(logical column, tile x of that column track's left edge)` — the
    /// cell→tile mapping the renderer needs to translate cell-unit scroll state
    /// into a tile origin. Contiguous over the layer's occupied column range.
    pub col_x: Vec<(i32, i32)>,
    /// Row counterpart of [`Self::col_x`]: `(logical row, tile y of its track top)`.
    pub row_y: Vec<(i32, i32)>,
}

impl TilePlan {
    pub fn get(&self, x: usize, y: usize) -> Tile {
        self.tiles[y * self.w + x]
    }
}

// ── internals ────────────────────────────────────────────────────────────────

/// Void margin around the track region so stubs never need to write outside
/// the grid.
const MARGIN: i32 = 2;
/// Width/height of a track with no room in it (corridors pass through its
/// centre). Public so the renderer can extrapolate the cell→tile scroll mapping
/// beyond the ends of [`TilePlan::col_x`]/[`TilePlan::row_y`].
pub const MIN_TRACK: i32 = 5;

/// Working grid: tiles plus the corridor travel axis per tile (0 none, 1
/// horizontal, 2 vertical) so a perpendicular re-stamp can detect a crossing.
struct Grid {
    w: i32,
    h: i32,
    tiles: Vec<Tile>,
    axes: Vec<u8>,
}

impl Grid {
    fn new(w: i32, h: i32) -> Self {
        let n = (w * h) as usize;
        Grid { w, h, tiles: vec![Tile::Void; n], axes: vec![0; n] }
    }
    fn idx(&self, x: i32, y: i32) -> Option<usize> {
        (x >= 0 && x < self.w && y >= 0 && y < self.h).then(|| (y * self.w + x) as usize)
    }
    fn get(&self, x: i32, y: i32) -> Tile {
        self.idx(x, y).map_or(Tile::Void, |i| self.tiles[i])
    }
    fn set(&mut self, x: i32, y: i32, t: Tile) {
        if let Some(i) = self.idx(x, y) {
            self.tiles[i] = t;
        }
    }
}

/// FNV-1a over a room id — the deterministic jitter source (no RNG, no deps).
fn fnv1a(id: RoomId) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in id.to_le_bytes() {
        h ^= u64::from(b);
        h = h.wrapping_mul(0x100_0000_01b3);
    }
    h
}

/// The origin-room wall side(s) a departing connection occupies. Cardinals match
/// the router's `side_for`; a diagonal is corner-attached and counts toward both
/// of its corner's sides. The arrival sides of a one-way edge are
/// `departure_sides(opposite(dir))` — the same rule `oneway_entry_side` encodes
/// (a cardinal enters the side opposite its travel; a diagonal arrives on the
/// corner facing its origin).
fn departure_sides(dir: Direction) -> &'static [Side] {
    match dir {
        Direction::N => &[Side::Top],
        Direction::S => &[Side::Bottom],
        Direction::E => &[Side::Right],
        Direction::W => &[Side::Left],
        Direction::NE => &[Side::Top, Side::Right],
        Direction::NW => &[Side::Top, Side::Left],
        Direction::SE => &[Side::Bottom, Side::Right],
        Direction::SW => &[Side::Bottom, Side::Left],
        _ => &[],
    }
}

/// S1: per-room floor size `(floor_w, floor_h)` from door-count minimums plus
/// id-hash jitter. Every term is odd, so sizes stay odd and boxes centre exactly
/// in their (odd-sized) tracks.
fn floor_sizes(sub: &MapGraph) -> BTreeMap<RoomId, (i32, i32)> {
    let mut doors: BTreeMap<(RoomId, Side), i32> = BTreeMap::new();
    for c in sub.connections() {
        if grid_offset(c.dir).is_none() {
            continue;
        }
        for &s in departure_sides(c.dir) {
            *doors.entry((c.origin, s)).or_default() += 1;
        }
        // A one-way edge also lands a door at its destination; a reciprocal's far
        // door is already counted by the reverse edge's own departure.
        let has_reverse =
            sub.connections().iter().any(|r| r.origin == c.dest && r.dest == c.origin);
        if !has_reverse {
            for &s in departure_sides(opposite(c.dir)) {
                *doors.entry((c.dest, s)).or_default() += 1;
            }
        }
    }
    sub.rooms()
        .map(|r| {
            let d = |s: Side| doors.get(&(r.id, s)).copied().unwrap_or(0);
            let h = fnv1a(r.id);
            let jw = ((h & 1) * 2) as i32;
            let jh = (((h >> 1) & 1) * 2) as i32;
            // Top/bottom doors spread along the width; left/right along the height.
            let fw = (5 + jw).max(2 * d(Side::Top) + 1).max(2 * d(Side::Bottom) + 1).clamp(5, 11);
            let fh = (3 + jh).max(2 * d(Side::Left) + 1).max(2 * d(Side::Right) + 1).clamp(3, 7);
            (r.id, (fw, fh))
        })
        .collect()
}

/// S2 track tables for one axis: tile position + size per track, tile position
/// per non-empty gutter, and the total extent.
type AxisTables = (BTreeMap<i32, i32>, BTreeMap<i32, i32>, BTreeMap<i32, i32>, i32);

/// Accumulate track origins along one axis. A channel with `k` corridor lanes gets
/// a `2k+1`-wide gutter (lane floors at odd offsets); a channel with none gets NO
/// gutter and the neighbouring tracks overlap by one tile — the shared wall.
fn build_axis(
    room_lo: i32,
    room_hi: i32,
    sizes: &BTreeMap<i32, i32>,
    lanes: &BTreeMap<i32, u16>,
) -> AxisTables {
    let (mut lo, mut hi) = (room_lo, room_hi);
    for &c in lanes.keys() {
        // channel c is the gutter after track c: keep c within [lo-1, hi]
        lo = lo.min(c + 1);
        hi = hi.max(c);
    }
    let gutter_w = |c: i32| lanes.get(&c).map_or(0, |&k| 2 * i32::from(k) + 1);
    let mut track = BTreeMap::new();
    let mut size = BTreeMap::new();
    let mut gutter = BTreeMap::new();
    let mut pos = MARGIN;
    let lead = gutter_w(lo - 1);
    if lead > 0 {
        gutter.insert(lo - 1, pos);
        pos += lead;
    }
    for c in lo..=hi {
        let w = sizes.get(&c).copied().unwrap_or(MIN_TRACK);
        track.insert(c, pos);
        size.insert(c, w);
        pos += w;
        let gw = gutter_w(c);
        if gw > 0 {
            gutter.insert(c, pos);
            pos += gw;
        } else if c < hi {
            pos -= 1; // zero-gutter neighbours overlap by one tile: the shared wall
        }
    }
    (track, size, gutter, pos + MARGIN)
}

struct Tracks {
    x_track: BTreeMap<i32, i32>,
    col_w: BTreeMap<i32, i32>,
    gutter_x: BTreeMap<i32, i32>,
    y_track: BTreeMap<i32, i32>,
    row_h: BTreeMap<i32, i32>,
    gutter_y: BTreeMap<i32, i32>,
}

/// A room's mutable tile-space box during placement (walls included).
#[derive(Debug, Clone, Copy)]
struct RoomBox {
    id: RoomId,
    x0: i32,
    y0: i32,
    x1: i32,
    y1: i32,
}

/// The side of `a` facing `b` when the cells are cardinally adjacent.
fn facing_side(a: (i32, i32), b: (i32, i32)) -> Option<Side> {
    match (b.0 - a.0, b.1 - a.1) {
        (1, 0) => Some(Side::Right),
        (-1, 0) => Some(Side::Left),
        (0, 1) => Some(Side::Bottom),
        (0, -1) => Some(Side::Top),
        _ => None,
    }
}

/// S3: is this connector realized as a punched door in a shared wall? Yes when the
/// rooms are cardinally adjacent, the route actually passes straight through the
/// shared doorway (a 3-point polyline through the exit stub), and the channel
/// between the two tracks carries no corridor lanes (zero gutter — the boxes can
/// abut). Returns the origin's facing side.
fn shared_wall_side(
    c: &RoutedConnector,
    cells: &BTreeMap<RoomId, (i32, i32)>,
    tracks: &Tracks,
) -> Option<Side> {
    if c.merge {
        return None;
    }
    let a = cells.get(&c.origin).copied()?;
    let b = cells.get(&c.dest).copied()?;
    let side = facing_side(a, b)?;
    if c.points.len() != 3 || c.points[1] != exit_point(a, side) {
        return None;
    }
    let open = match side {
        Side::Right => tracks.gutter_x.contains_key(&a.0),
        Side::Left => tracks.gutter_x.contains_key(&b.0),
        Side::Bottom => tracks.gutter_y.contains_key(&a.1),
        Side::Top => tracks.gutter_y.contains_key(&b.1),
    };
    (!open).then_some(side)
}

/// The side of `a` whose box wall lies on the same line as `b`'s facing wall —
/// rooms that physically abut even though the router routed their edge some
/// other way (distorted geometry). Whether a straight doorway actually fits is
/// `realize_shared_door`'s overlap-span check.
fn abutting_side(a: &RoomBox, b: &RoomBox) -> Option<Side> {
    if a.x1 == b.x0 {
        return Some(Side::Right);
    }
    if b.x1 == a.x0 {
        return Some(Side::Left);
    }
    if a.y1 == b.y0 {
        return Some(Side::Bottom);
    }
    if b.y1 == a.y0 {
        return Some(Side::Top);
    }
    None
}

/// Slot index → signed offset along a wall: 0, +1, −1, +2, −2, … (matches the
/// classic renderer's `slot_offset` ordering).
fn slot_shift(slot: u16) -> i32 {
    let step = (i32::from(slot) + 1) / 2;
    if slot % 2 == 1 {
        step
    } else {
        -step
    }
}

/// The wall tile a connector endpoint wants, plus the outward step off it: the box
/// corner for a diagonal (corner-attached), else the side wall at the floor
/// midpoint shifted by the endpoint's slot.
fn door_want(
    bx: &RoomBox,
    corner: Option<Direction>,
    side: Side,
    slot: u16,
) -> ((i32, i32), (i32, i32)) {
    if let Some(d) = corner {
        let (dx, dy) = grid_offset(d).unwrap_or((0, 0));
        let x = if dx > 0 { bx.x1 } else { bx.x0 };
        let y = if dy > 0 { bx.y1 } else { bx.y0 };
        return ((x, y), (dx, dy));
    }
    let cx = (bx.x0 + 1 + bx.x1 - 1) / 2;
    let cy = (bx.y0 + 1 + bx.y1 - 1) / 2;
    match side {
        Side::Right => ((bx.x1, (cy + slot_shift(slot)).clamp(bx.y0 + 1, bx.y1 - 1)), (1, 0)),
        Side::Left => ((bx.x0, (cy + slot_shift(slot)).clamp(bx.y0 + 1, bx.y1 - 1)), (-1, 0)),
        Side::Bottom => (((cx + slot_shift(slot)).clamp(bx.x0 + 1, bx.x1 - 1), bx.y1), (0, 1)),
        Side::Top => (((cx + slot_shift(slot)).clamp(bx.x0 + 1, bx.x1 - 1), bx.y0), (0, -1)),
    }
}

/// Place `door` on the first candidate that is still a plain wall; failing that,
/// the first that is not already someone else's door; last resort, the first
/// candidate outright. Returns where it landed.
fn place_first(g: &mut Grid, cands: &[(i32, i32)], door: Tile) -> (i32, i32) {
    for &p in cands {
        if matches!(g.get(p.0, p.1), Tile::Wall { .. }) {
            g.set(p.0, p.1, door);
            return p;
        }
    }
    for &p in cands {
        if !matches!(g.get(p.0, p.1), Tile::Door { .. }) {
            g.set(p.0, p.1, door);
            return p;
        }
    }
    let p = cands[0];
    g.set(p.0, p.1, door);
    p
}

/// Punch `door` at `want`, probing 0, +1, −1, … along the wall (x when `horiz`)
/// within `span` when the spot is taken.
fn punch_door_along(
    g: &mut Grid,
    want: (i32, i32),
    horiz: bool,
    span: (i32, i32),
    door: Tile,
) -> (i32, i32) {
    let mut cands: Vec<(i32, i32)> = Vec::new();
    for s in 0..2 * (span.1 - span.0 + 1).max(1) {
        let off = slot_shift(s as u16);
        let p = if horiz { (want.0 + off, want.1) } else { (want.0, want.1 + off) };
        let v = if horiz { p.0 } else { p.1 };
        if (span.0..=span.1).contains(&v) {
            cands.push(p);
        }
    }
    if cands.is_empty() {
        cands.push(want);
    }
    place_first(g, &cands, door)
}

/// Punch a corner-attached door: the corner itself, falling back to the two wall
/// tiles beside it.
fn punch_corner_door(g: &mut Grid, bx: &RoomBox, want: (i32, i32), door: Tile) -> (i32, i32) {
    let ix = if want.0 == bx.x0 { want.0 + 1 } else { want.0 - 1 };
    let iy = if want.1 == bx.y0 { want.1 + 1 } else { want.1 - 1 };
    place_first(g, &[want, (want.0, iy), (ix, want.1)], door)
}

/// Punch one connector endpoint's door (corner or slotted side wall).
fn punch_endpoint(
    g: &mut Grid,
    bx: &RoomBox,
    corner: Option<Direction>,
    side: Side,
    slot: u16,
    door: Tile,
) -> (i32, i32) {
    let (want, _) = door_want(bx, corner, side, slot);
    if corner.is_some() {
        punch_corner_door(g, bx, want, door)
    } else {
        let horiz = matches!(side, Side::Top | Side::Bottom);
        let span = if horiz { (bx.x0 + 1, bx.x1 - 1) } else { (bx.y0 + 1, bx.y1 - 1) };
        punch_door_along(g, want, horiz, span, door)
    }
}

/// Stamp one corridor floor tile. Only Void is claimed; landing on a DIFFERENT
/// connection's corridor travelling the PERPENDICULAR axis writes a bridge
/// (deterministic over/under: the lower connection index goes over). Everything
/// else — rooms, doors, an existing bridge, a parallel corridor — is left alone.
fn stamp_corridor(g: &mut Grid, p: (i32, i32), conn: u16, horiz: bool) {
    let Some(i) = g.idx(p.0, p.1) else { return };
    let axis = if horiz { 1 } else { 2 };
    match g.tiles[i] {
        Tile::Void => {
            g.tiles[i] = Tile::Corridor { conn };
            g.axes[i] = axis;
        }
        Tile::Corridor { conn: other } if other != conn && g.axes[i] != axis => {
            g.tiles[i] = Tile::Bridge { over: other.min(conn), under: other.max(conn) };
        }
        _ => {}
    }
}

/// A dead-end stub: a `Stub` door plus a single corridor tile fading outward.
fn realize_stub(
    g: &mut Grid,
    bx: &RoomBox,
    corner: Option<Direction>,
    side: Side,
    slot: u16,
    conn: u16,
    dir: Direction,
) {
    let door = Tile::Door { conn, kind: DoorKind::Stub(dir) };
    let (_, out) = door_want(bx, corner, side, slot);
    let p = punch_endpoint(g, bx, corner, side, slot, door);
    stamp_corridor(g, (p.0 + out.0, p.1 + out.1), conn, out.1 == 0);
}

/// S3: punch a door through the shared wall of two abutting rooms, at the midpoint
/// of the floors' overlap span shifted by the connector's slot (the fan for
/// multiple doors between one pair). Returns false — nothing punched — when the
/// floors have no overlap span, so no straight doorway exists; the caller falls
/// back to a stub pair.
fn realize_shared_door(
    g: &mut Grid,
    c: &RoutedConnector,
    conn: u16,
    side: Side,
    ba: &RoomBox,
    bb: &RoomBox,
) -> bool {
    let kind = if c.reciprocal { DoorKind::TwoWay } else { DoorKind::OneWay(c.exit_dir) };
    let door = Tile::Door { conn, kind };
    match side {
        Side::Right | Side::Left => {
            let x = if side == Side::Right { ba.x1 } else { ba.x0 };
            let lo = (ba.y0 + 1).max(bb.y0 + 1);
            let hi = (ba.y1 - 1).min(bb.y1 - 1);
            if lo > hi {
                return false; // floors never overlap in y — no straight doorway exists
            }
            let want = (x, ((lo + hi) / 2 + slot_shift(c.exit_slot)).clamp(lo, hi));
            punch_door_along(g, want, false, (lo, hi), door);
        }
        Side::Top | Side::Bottom => {
            let y = if side == Side::Bottom { ba.y1 } else { ba.y0 };
            let lo = (ba.x0 + 1).max(bb.x0 + 1);
            let hi = (ba.x1 - 1).min(bb.x1 - 1);
            if lo > hi {
                return false;
            }
            let want = (((lo + hi) / 2 + slot_shift(c.exit_slot)).clamp(lo, hi), y);
            punch_door_along(g, want, true, (lo, hi), door);
        }
    }
    true
}

// ── Meandering corridor search (S4) ──────────────────────────────────────────
//
// Corridors are routed with tile-space A* rather than mapped off the lane
// router's polylines: the search wanders through whatever free space lies
// between the rooms, and deterministic per-tile noise plus a straight-run
// penalty give long hauls the irregular staircase bends of a hand-drawn map
// (the Reed Zork map is the reference look). Rooms can never be tunnelled
// through by construction — their tiles are simply not passable.

/// FNV-1a over three values — the per-(connection, tile) noise source that
/// makes corridors meander deterministically.
fn hash3(a: u64, b: u64, c: u64) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for v in [a, b, c] {
        for byte in v.to_le_bytes() {
            h ^= u64::from(byte);
            h = h.wrapping_mul(0x100_0000_01b3);
        }
    }
    h
}

/// Integer milli-cost tuning for the meander search (no floats: determinism by
/// construction). `COST_BASE` is one orthogonal step; `NOISE_AMP` bounds the
/// deterministic per-(conn,tile) surcharge that bends long hauls off the ruler
/// line; `COST_TURN` keeps short hops nearly direct; `COST_STRAIGHT` kicks in
/// once a run exceeds the connection's period (4..=6 tiles, hash-varied per
/// conn) so long corridors break into staircase bends; `COST_HUG` discourages
/// running beside a foreign corridor (parallel lines mush the glyph art) while
/// leaving perpendicular crossings cheap.
const COST_BASE: u32 = 1000;
const NOISE_AMP: u32 = 350;
const COST_TURN: u32 = 900;
const COST_STRAIGHT: u32 = 1200;
const COST_HUG: u32 = 700;
/// Surcharge for riding ALONG a foreign corridor tile (passages merging into a
/// shared tunnel stretch): expensive enough that corridors stay distinct where
/// space allows, cheap enough to squeeze through a door mouth the corridor
/// braid has otherwise choked off.
const COST_MERGE: u32 = 2500;
/// Straight-run lengths are capped at this in the search state (≥ max period+1).
const RUN_CAP: u32 = 7;
/// Abort the search (stub fallback) after this many state expansions.
const SEARCH_CAP: usize = 50_000;

/// Neighbor order is part of the tie-breaking and must stay fixed: E, W, S, N.
const DIRS: [(i32, i32); 4] = [(1, 0), (-1, 0), (0, 1), (0, -1)];

/// The corridor axis (1 horizontal, 2 vertical) of a `DIRS` index.
fn axis_of(dir: usize) -> u8 {
    if dir < 2 {
        1
    } else {
        2
    }
}

/// Deterministic A* from door `da` to door `db`: orthogonal moves; rooms
/// (walls, floors, features) and foreign doors are obstacles; existing bridges
/// are avoided entirely (a third corridor could not be drawn there). A foreign
/// corridor tile is passable two ways: crossed PERPENDICULAR and straight
/// through (the crossing becomes a Bridge when stamped), or ridden ALONG for a
/// `COST_MERGE` surcharge per tile — passages briefly merging reads fine in
/// passage rendering and keeps a door reachable when the corridor braid has
/// choked its mouth. Ties break on (f-cost, state index) so the result is
/// fully deterministic. Returns the tile path `da ..= db`, or None when no
/// route exists or the search cap blows.
fn meander_path(g: &Grid, conn: u16, da: (i32, i32), db: (i32, i32)) -> Option<Vec<(i32, i32)>> {
    use std::cmp::Reverse;
    use std::collections::BinaryHeap;
    if da == db {
        return Some(vec![da]);
    }
    let runs = RUN_CAP as usize + 1;
    let states = (g.w * g.h) as usize * 4 * runs;
    let mut best: Vec<u32> = vec![u32::MAX; states];
    let mut parent: Vec<u32> = vec![u32::MAX; states];
    const START: u32 = u32::MAX - 1;
    let w = g.w;
    let enc = |x: i32, y: i32, dir: usize, run: u32| {
        ((y * w + x) as usize * 4 + dir) * runs + run as usize
    };
    let period = 4 + (hash3(u64::from(conn), 0x6d65_616e, 0) % 3) as u32; // 4..=6
    let noise = |p: (i32, i32)| {
        (hash3(u64::from(conn), p.0 as u64, p.1 as u64) % u64::from(NOISE_AMP + 1)) as u32
    };
    let hug = |p: (i32, i32)| {
        DIRS.iter().any(|&(dx, dy)| {
            matches!(g.get(p.0 + dx, p.1 + dy), Tile::Corridor { conn: o } if o != conn)
        })
    };
    let passable = |p: (i32, i32)| {
        if p == db {
            return true;
        }
        match g.get(p.0, p.1) {
            Tile::Void => true,
            Tile::Corridor { conn: other } => other != conn,
            _ => false,
        }
    };
    // Riding a foreign corridor tile parallel to its travel axis = merging.
    let merges = |p: (i32, i32), dir: usize| {
        matches!(g.get(p.0, p.1), Tile::Corridor { conn: o } if o != conn)
            && g.idx(p.0, p.1).is_some_and(|i| g.axes[i] == axis_of(dir))
    };
    let hcost = |x: i32, y: i32| ((x - db.0).abs() + (y - db.1).abs()) as u32 * COST_BASE;
    let step_cost = |p: (i32, i32), dir: usize, turned: bool, run: u32| {
        let mut c = COST_BASE + noise(p);
        if turned {
            c += COST_TURN;
        }
        if run > period {
            c += COST_STRAIGHT;
        }
        if hug(p) {
            c += COST_HUG;
        }
        if merges(p, dir) {
            c += COST_MERGE;
        }
        c
    };

    let mut heap: BinaryHeap<Reverse<(u32, usize)>> = BinaryHeap::new();
    for (d, &(dx, dy)) in DIRS.iter().enumerate() {
        let p = (da.0 + dx, da.1 + dy);
        if g.idx(p.0, p.1).is_none() || !passable(p) {
            continue;
        }
        let cost = step_cost(p, d, false, 1);
        let s = enc(p.0, p.1, d, 1);
        if cost < best[s] {
            best[s] = cost;
            parent[s] = START;
            heap.push(Reverse((cost + hcost(p.0, p.1), s)));
        }
    }

    let mut expanded = 0usize;
    let mut goal = None;
    while let Some(Reverse((f, s))) = heap.pop() {
        let run = (s % runs) as u32;
        let dir = (s / runs) % 4;
        let cell = s / runs / 4;
        let (x, y) = ((cell % w as usize) as i32, (cell / w as usize) as i32);
        if f > best[s].saturating_add(hcost(x, y)) {
            continue; // stale heap entry
        }
        if (x, y) == db {
            goal = Some(s);
            break;
        }
        expanded += 1;
        if expanded > SEARCH_CAP {
            return None;
        }
        // On a tile we would CROSS (foreign corridor, perpendicular axis) the
        // exit must continue straight — that is what makes it a Bridge. A tile
        // we merge along (parallel axis) allows turns like any other.
        let on_crossing = matches!(g.get(x, y), Tile::Corridor { conn: o } if o != conn)
            && g.idx(x, y).is_some_and(|i| g.axes[i] != axis_of(dir));
        for (nd, &(dx, dy)) in DIRS.iter().enumerate() {
            if on_crossing && nd != dir {
                continue;
            }
            if (dx, dy) == (-DIRS[dir].0, -DIRS[dir].1) {
                continue; // no immediate reversal
            }
            let p = (x + dx, y + dy);
            if g.idx(p.0, p.1).is_none() || !passable(p) {
                continue;
            }
            let nrun = if nd == dir { (run + 1).min(RUN_CAP) } else { 1 };
            let cost = best[s] + step_cost(p, nd, nd != dir, nrun);
            let ns = enc(p.0, p.1, nd, nrun);
            if cost < best[ns] {
                best[ns] = cost;
                parent[ns] = s as u32;
                heap.push(Reverse((cost + hcost(p.0, p.1), ns)));
            }
        }
    }

    let mut s = goal?;
    let mut path = vec![db];
    while parent[s] != START {
        s = parent[s] as usize;
        let cell = s / runs / 4;
        path.push(((cell % w as usize) as i32, (cell / w as usize) as i32));
    }
    path.push(da);
    path.reverse();
    Some(path)
}

/// S4: realize one routed connector as door → meandering corridor → door. The
/// doors come from the router's sides/slots/corners as before; the corridor
/// shape comes from [`meander_path`]. On failure (no route, search cap) every
/// write is undone and `false` is returned — the caller falls back to an honest
/// stub pair.
fn realize_corridor(g: &mut Grid, c: &RoutedConnector, conn: u16, ba: &RoomBox, bb: &RoomBox) -> bool {
    let kind = if c.reciprocal { DoorKind::TwoWay } else { DoorKind::OneWay(c.exit_dir) };
    let door = Tile::Door { conn, kind };
    let exit_corner = is_diagonal(c.exit_dir).then_some(c.exit_dir);
    let (want_a, _) = door_want(ba, exit_corner, c.exit, c.exit_slot);
    let (want_b, _) = door_want(bb, c.entry_corner, c.entry, c.entry_slot);
    if want_a == want_b {
        // Diagonally-abutting boxes share the corner tile: the door IS the connection.
        punch_endpoint(g, ba, exit_corner, c.exit, c.exit_slot, door);
        return true;
    }
    // Snapshot before punching: on a failed route the doors must vanish again.
    let saved = (g.tiles.clone(), g.axes.clone());
    let da = punch_endpoint(g, ba, exit_corner, c.exit, c.exit_slot, door);
    let db = punch_endpoint(g, bb, c.entry_corner, c.entry, c.entry_slot, door);
    let Some(path) = meander_path(g, conn, da, db) else {
        (g.tiles, g.axes) = saved;
        return false;
    };
    for w in path.windows(2) {
        let (a, b) = (w[0], w[1]);
        if b != da && b != db {
            stamp_corridor(g, b, conn, a.1 == b.1);
        }
    }
    true
}

/// S5: Up/Down land on the floor tile nearest the room's top-right/bottom-right
/// interior corner (probing left past occupied tiles; several Up exits share one
/// stair).
fn stamp_stairs(g: &mut Grid, bx: &RoomBox, kind: FeatureKind) {
    let y = if kind == FeatureKind::StairsUp { bx.y0 + 1 } else { bx.y1 - 1 };
    let feat = Tile::Feature { room: bx.id, kind };
    let mut x = bx.x1 - 1;
    while x > bx.x0 {
        let t = g.get(x, y);
        if t == feat {
            return;
        }
        if matches!(t, Tile::Floor { .. }) {
            g.set(x, y, feat);
            return;
        }
        x -= 1;
    }
}

/// S5: In/Out replace a mid-right wall tile (the classic renderer's mid-slot),
/// probing up/down the wall when the midpoint is taken.
fn stamp_portal(g: &mut Grid, bx: &RoomBox, kind: FeatureKind) {
    let x = bx.x1;
    let cy = (bx.y0 + 1 + bx.y1 - 1) / 2;
    let feat = Tile::Feature { room: bx.id, kind };
    for s in 0..2 * (bx.y1 - bx.y0).max(1) {
        let y = cy + slot_shift(s as u16);
        if y <= bx.y0 || y >= bx.y1 {
            continue;
        }
        let t = g.get(x, y);
        if t == feat {
            return;
        }
        if t == (Tile::Wall { kind: WallKind::Room }) {
            g.set(x, y, feat);
            return;
        }
    }
}

/// S5 dispatch over the layer's non-planar edges (Unknown is skipped, matching
/// the classic renderer).
fn stamp_features(g: &mut Grid, sub: &MapGraph, boxes: &BTreeMap<RoomId, RoomBox>) {
    for c in sub.connections() {
        let kind = match c.dir {
            Direction::Up => FeatureKind::StairsUp,
            Direction::Down => FeatureKind::StairsDown,
            Direction::In => FeatureKind::PortalIn,
            Direction::Out => FeatureKind::PortalOut,
            _ => continue,
        };
        let Some(bx) = boxes.get(&c.origin) else { continue };
        match kind {
            FeatureKind::StairsUp | FeatureKind::StairsDown => stamp_stairs(g, bx, kind),
            FeatureKind::PortalIn | FeatureKind::PortalOut => stamp_portal(g, bx, kind),
        }
    }
}

/// Paint pass: every Void 8-adjacent to corridor floor (orthogonal or diagonal,
/// or a bridge) becomes a `Path` wall, so corridors read as walkable outlined
/// passages. Only Void is claimed — room walls, floors, and doors are never
/// overwritten, and a corridor running beside a room simply shares that room's
/// wall on that side. Two passages one tile apart share a single painted wall
/// between them. Shared by `realize_layer` and `project::project` (row-major
/// `w*h` tile slice).
pub(crate) fn paint_path_walls(w: i32, h: i32, tiles: &mut [Tile]) {
    let get = |tiles: &[Tile], x: i32, y: i32| -> Tile {
        if x >= 0 && x < w && y >= 0 && y < h {
            tiles[(y * w + x) as usize]
        } else {
            Tile::Void
        }
    };
    let mut to_wall: Vec<(i32, i32)> = Vec::new();
    for y in 0..h {
        for x in 0..w {
            if !matches!(get(tiles, x, y), Tile::Void) {
                continue;
            }
            'probe: for dy in -1..=1 {
                for dx in -1..=1 {
                    if dx == 0 && dy == 0 {
                        continue;
                    }
                    if matches!(
                        get(tiles, x + dx, y + dy),
                        Tile::Corridor { .. } | Tile::CorridorDiag { .. } | Tile::Bridge { .. }
                    ) {
                        to_wall.push((x, y));
                        break 'probe;
                    }
                }
            }
        }
    }
    for (x, y) in to_wall {
        tiles[(y * w + x) as usize] = Tile::Wall { kind: WallKind::Path };
    }
}

/// [`paint_path_walls`] over the working grid.
fn paint_walls(g: &mut Grid) {
    paint_path_walls(g.w, g.h, &mut g.tiles);
}

/// Realize one layer's sub-graph as a tile grid (stages S1–S5; see module docs).
/// Deterministic: the same graph always yields the identical `TilePlan`.
pub fn realize_layer(graph: &MapGraph, layer: LayerId) -> TilePlan {
    let sub = graph.layer_subgraph(layer);
    // Route only the 8-way compass edges; Up/Down/In/Out become features (S5)
    // and Unknown is skipped, so none of them may claim corridor lanes.
    let mut planar = sub.clone();
    let non_planar: Vec<(RoomId, Direction)> = planar
        .connections()
        .iter()
        .filter(|c| grid_offset(c.dir).is_none())
        .map(|c| (c.origin, c.dir))
        .collect();
    for (o, d) in non_planar {
        planar.remove_connection(o, d);
    }
    let plan = route_lanes(&planar);

    let cells: BTreeMap<RoomId, (i32, i32)> =
        sub.rooms().filter_map(|r| r.pos.map(|p| (r.id, p))).collect();
    if cells.is_empty() {
        return TilePlan::default();
    }

    // S1: room floor sizes; per-track box maxima.
    let sizes = floor_sizes(&sub);
    let mut col_box: BTreeMap<i32, i32> = BTreeMap::new();
    let mut row_box: BTreeMap<i32, i32> = BTreeMap::new();
    for (id, &(cx, cy)) in &cells {
        let (fw, fh) = sizes[id];
        let e = col_box.entry(cx).or_insert(0);
        *e = (*e).max(fw + 2);
        let e = row_box.entry(cy).or_insert(0);
        *e = (*e).max(fh + 2);
    }

    // S2: gutters sized from the route plan's lane counts; a diagonal's corner
    // needs one lane's worth of space in both its channels even though it
    // carries no lane (mirrors `RoutePlan::diag_corners`).
    let mut v_lanes = plan.v_lanes.clone();
    let mut h_lanes = plan.h_lanes.clone();
    for &(vc, hc) in &plan.diag_corners {
        v_lanes.entry(vc).or_insert(1);
        h_lanes.entry(hc).or_insert(1);
    }
    let min_c = cells.values().map(|p| p.0).min().unwrap_or(0);
    let max_c = cells.values().map(|p| p.0).max().unwrap_or(0);
    let min_r = cells.values().map(|p| p.1).min().unwrap_or(0);
    let max_r = cells.values().map(|p| p.1).max().unwrap_or(0);
    let (x_track, col_w, gutter_x, w) = build_axis(min_c, max_c, &col_box, &v_lanes);
    let (y_track, row_h, gutter_y, h) = build_axis(min_r, max_r, &row_box, &h_lanes);
    let tracks = Tracks { x_track, col_w, gutter_x, y_track, row_h, gutter_y };

    // S3: centre each box in its track cell...
    let mut boxes: BTreeMap<RoomId, RoomBox> = BTreeMap::new();
    for (&id, &(cx, cy)) in &cells {
        let (fw, fh) = sizes[&id];
        let (bw, bh) = (fw + 2, fh + 2);
        let x0 = tracks.x_track[&cx] + (tracks.col_w[&cx] - bw) / 2;
        let y0 = tracks.y_track[&cy] + (tracks.row_h[&cy] - bh) / 2;
        boxes.insert(id, RoomBox { id, x0, y0, x1: x0 + bw - 1, y1: y0 + bh - 1 });
    }
    // ...then extend connected zero-gutter neighbours to the shared track
    // boundary so their walls land on the same line.
    for c in &plan.connectors {
        if let Some(side) = shared_wall_side(c, &cells, &tracks) {
            let a = cells[&c.origin];
            let b = cells[&c.dest];
            match side {
                Side::Right => {
                    let bd = tracks.x_track[&b.0];
                    if let Some(bx) = boxes.get_mut(&c.origin) {
                        bx.x1 = bd;
                    }
                    if let Some(bx) = boxes.get_mut(&c.dest) {
                        bx.x0 = bd;
                    }
                }
                Side::Left => {
                    let bd = tracks.x_track[&a.0];
                    if let Some(bx) = boxes.get_mut(&c.origin) {
                        bx.x0 = bd;
                    }
                    if let Some(bx) = boxes.get_mut(&c.dest) {
                        bx.x1 = bd;
                    }
                }
                Side::Bottom => {
                    let bd = tracks.y_track[&b.1];
                    if let Some(bx) = boxes.get_mut(&c.origin) {
                        bx.y1 = bd;
                    }
                    if let Some(bx) = boxes.get_mut(&c.dest) {
                        bx.y0 = bd;
                    }
                }
                Side::Top => {
                    let bd = tracks.y_track[&a.1];
                    if let Some(bx) = boxes.get_mut(&c.origin) {
                        bx.y0 = bd;
                    }
                    if let Some(bx) = boxes.get_mut(&c.dest) {
                        bx.y1 = bd;
                    }
                }
            }
        }
    }

    // Stamp room boxes: wall ring + floor interior (shared walls simply coincide).
    let mut g = Grid::new(w, h);
    for bx in boxes.values() {
        for y in bx.y0..=bx.y1 {
            for x in bx.x0..=bx.x1 {
                let edge = x == bx.x0 || x == bx.x1 || y == bx.y0 || y == bx.y1;
                let t = if edge {
                    Tile::Wall { kind: WallKind::Room }
                } else {
                    Tile::Floor { room: bx.id }
                };
                g.set(x, y, t);
            }
        }
    }

    // S3/S4: one TileConn per routed connector; realize each as a shared-wall
    // door or a corridor — distorted edges included, their polylines are just as
    // drawable — with stubs only for merge connectors and routes that genuinely
    // cannot be stamped.
    let conns: Vec<TileConn> = plan
        .connectors
        .iter()
        .map(|c| TileConn {
            origin: c.origin,
            dest: c.dest,
            dir: c.exit_dir,
            reciprocal: c.reciprocal,
            distorted: c.distorted,
        })
        .collect();
    for (ci, c) in plan.connectors.iter().enumerate() {
        let conn = ci as u16;
        let Some(&ba) = boxes.get(&c.origin) else { continue };
        let exit_corner = is_diagonal(c.exit_dir).then_some(c.exit_dir);
        let bb = if c.merge { None } else { boxes.get(&c.dest).copied() };
        let Some(bb) = bb else {
            // A merge stub (single-sided by design) or an unplaced destination.
            realize_stub(&mut g, &ba, exit_corner, c.exit, c.exit_slot, conn, c.exit_dir);
            continue;
        };
        let realized = match shared_wall_side(c, &cells, &tracks) {
            Some(side) => realize_shared_door(&mut g, c, conn, side, &ba, &bb),
            // A failed corridor between boxes that physically abut (a distorted
            // edge's door opens straight into the neighbour) still gets a real
            // shared-wall door before stubs are even considered.
            None => {
                realize_corridor(&mut g, c, conn, &ba, &bb)
                    || abutting_side(&ba, &bb)
                        .is_some_and(|s| realize_shared_door(&mut g, c, conn, s, &ba, &bb))
            }
        };
        if !realized {
            // No stampable route — an honest stub pair on both rooms instead.
            realize_stub(&mut g, &ba, exit_corner, c.exit, c.exit_slot, conn, c.exit_dir);
            let dir = c.entry_dir.unwrap_or_else(|| opposite(c.exit_dir));
            realize_stub(&mut g, &bb, c.entry_corner, c.entry, c.entry_slot, conn, dir);
        }
    }

    stamp_features(&mut g, &sub, &boxes);
    paint_walls(&mut g);

    let rooms: Vec<TileRoom> = boxes
        .values()
        .map(|b| TileRoom {
            id: b.id,
            bounds: Rect {
                x: b.x0 as usize,
                y: b.y0 as usize,
                w: (b.x1 - b.x0 + 1) as usize,
                h: (b.y1 - b.y0 + 1) as usize,
            },
            floor: Rect {
                x: (b.x0 + 1) as usize,
                y: (b.y0 + 1) as usize,
                w: (b.x1 - b.x0 - 1) as usize,
                h: (b.y1 - b.y0 - 1) as usize,
            },
        })
        .collect();
    let col_x: Vec<(i32, i32)> = tracks.x_track.iter().map(|(&c, &x)| (c, x)).collect();
    let row_y: Vec<(i32, i32)> = tracks.y_track.iter().map(|(&r, &y)| (r, y)).collect();
    TilePlan { w: w as usize, h: h as usize, tiles: g.tiles, rooms, conns, col_x, row_y }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::direction::Direction;
    use crate::graph::MapGraph;
    use crate::layer::MAIN_LAYER;

    fn grid_room(g: &mut MapGraph, id: RoomId, pos: (i32, i32)) {
        g.upsert_room(id, format!("r{id}"));
        g.set_pos(id, pos);
    }

    fn door_tiles(p: &TilePlan) -> Vec<(usize, usize, u16, DoorKind)> {
        let mut v = Vec::new();
        for y in 0..p.h {
            for x in 0..p.w {
                if let Tile::Door { conn, kind } = p.get(x, y) {
                    v.push((x, y, conn, kind));
                }
            }
        }
        v
    }

    fn room(p: &TilePlan, id: RoomId) -> &TileRoom {
        p.rooms.iter().find(|r| r.id == id).unwrap()
    }

    fn rects_intersect(a: Rect, b: Rect) -> bool {
        a.x <= b.right() && b.x <= a.right() && a.y <= b.bottom() && b.y <= a.bottom()
    }

    /// ~20 rooms, mixed directions (reciprocal + one-way cardinals, diagonals,
    /// Up/Down, In/Out, Unknown, and shapes that force distortion), laid out by
    /// the real layout engine.
    fn big_graph() -> MapGraph {
        let mut g = MapGraph::new();
        for id in 1..=20u16 {
            g.upsert_room(id, format!("r{id}"));
        }
        let edges: &[(u16, Direction, u16)] = &[
            (1, Direction::E, 2),
            (2, Direction::W, 1),
            (2, Direction::E, 3),
            (3, Direction::E, 4),
            (1, Direction::S, 5),
            (5, Direction::N, 1),
            (5, Direction::E, 6),
            (6, Direction::W, 5),
            (3, Direction::S, 7),
            (7, Direction::N, 3),
            (4, Direction::SE, 8),
            (8, Direction::NW, 4),
            (5, Direction::S, 9),
            (9, Direction::E, 10),
            (10, Direction::E, 11),
            (11, Direction::W, 10),
            (11, Direction::E, 12),
            (12, Direction::W, 11),
            (9, Direction::S, 13),
            (13, Direction::E, 14),
            (14, Direction::N, 10),
            (11, Direction::S, 15),
            (15, Direction::N, 11),
            (15, Direction::E, 16),
            (16, Direction::W, 15),
            (13, Direction::S, 17),
            (17, Direction::N, 13),
            (17, Direction::E, 18),
            (18, Direction::W, 17),
            (18, Direction::E, 19),
            (19, Direction::N, 20),
            (20, Direction::S, 19),
            (6, Direction::Up, 7),
            (7, Direction::Down, 6),
            (7, Direction::NE, 2),
            (20, Direction::In, 1),
            (12, Direction::Out, 2),
            (15, Direction::Unknown, 16),
        ];
        for &(o, d, t) in edges {
            g.add_edge(o, d, t);
        }
        crate::layout::relayout_auto(&mut g);
        g
    }

    #[test]
    fn adjacent_reciprocal_pair_shares_one_wall_with_one_twoway_door() {
        let mut g = MapGraph::new();
        grid_room(&mut g, 1, (0, 0));
        grid_room(&mut g, 2, (1, 0));
        g.add_edge(1, Direction::E, 2);
        g.add_edge(2, Direction::W, 1);
        let p = realize_layer(&g, MAIN_LAYER);
        let (a, b) = (room(&p, 1), room(&p, 2));
        assert_eq!(a.bounds.right(), b.bounds.x, "boxes share exactly one wall column");
        let doors = door_tiles(&p);
        assert_eq!(doors.len(), 1, "one reciprocal pair → one door: {doors:?}");
        let (x, _, conn, kind) = doors[0];
        assert_eq!(x, a.bounds.right(), "the door sits in the shared wall");
        assert_eq!(kind, DoorKind::TwoWay);
        let tc = p.conns[conn as usize];
        assert_eq!((tc.origin, tc.dest), (1, 2));
    }

    #[test]
    fn one_way_edge_gets_a_one_way_door() {
        let mut g = MapGraph::new();
        grid_room(&mut g, 1, (0, 0));
        grid_room(&mut g, 2, (1, 0));
        g.add_edge(1, Direction::E, 2);
        let p = realize_layer(&g, MAIN_LAYER);
        let doors = door_tiles(&p);
        assert_eq!(doors.len(), 1);
        assert_eq!(doors[0].3, DoorKind::OneWay(Direction::E));
    }

    #[test]
    fn adjacent_unconnected_rooms_share_a_wall_with_no_door() {
        let mut g = MapGraph::new();
        grid_room(&mut g, 1, (0, 0));
        grid_room(&mut g, 2, (1, 0));
        let p = realize_layer(&g, MAIN_LAYER);
        let (a, b) = (room(&p, 1), room(&p, 2));
        assert_eq!(a.bounds.right(), b.bounds.x, "boxes still meet on one shared column");
        assert!(door_tiles(&p).is_empty(), "no connection → no door");
        for y in a.floor.y..=a.floor.bottom() {
            assert_eq!(
                p.get(a.bounds.right(), y),
                Tile::Wall { kind: WallKind::Room },
                "shared column stays solid wall"
            );
        }
    }

    /// True when `from` reaches `to` walking 4-adjacent Corridor/Door/Bridge tiles.
    fn doors_connected(p: &TilePlan, from: (usize, usize), to: (usize, usize)) -> bool {
        let passable = |x: usize, y: usize| {
            matches!(p.get(x, y), Tile::Corridor { .. } | Tile::Door { .. } | Tile::Bridge { .. })
        };
        let mut seen = vec![false; p.w * p.h];
        let mut stack = vec![from];
        seen[from.1 * p.w + from.0] = true;
        while let Some((x, y)) = stack.pop() {
            if (x, y) == to {
                return true;
            }
            let mut probe: Vec<(usize, usize)> = vec![(x + 1, y), (x, y + 1)];
            if x > 0 {
                probe.push((x - 1, y));
            }
            if y > 0 {
                probe.push((x, y - 1));
            }
            for (nx, ny) in probe {
                if nx < p.w && ny < p.h && !seen[ny * p.w + nx] && passable(nx, ny) {
                    seen[ny * p.w + nx] = true;
                    stack.push((nx, ny));
                }
            }
        }
        false
    }

    #[test]
    fn distant_rooms_get_a_connected_corridor_with_doors_at_both_ends() {
        let mut g = MapGraph::new();
        grid_room(&mut g, 1, (0, 0));
        grid_room(&mut g, 2, (2, 0));
        g.add_edge(1, Direction::E, 2);
        let p = realize_layer(&g, MAIN_LAYER);
        let corridor: Vec<(usize, usize)> = (0..p.h)
            .flat_map(|y| (0..p.w).map(move |x| (x, y)))
            .filter(|&(x, y)| matches!(p.get(x, y), Tile::Corridor { .. }))
            .collect();
        assert!(!corridor.is_empty(), "two columns apart → corridor tiles exist");
        let doors = door_tiles(&p);
        assert_eq!(doors.len(), 2, "a door on each room's wall");
        // The corridor is a connected orthogonal path from door to door.
        let (a, b) = ((doors[0].0, doors[0].1), (doors[1].0, doors[1].1));
        assert!(doors_connected(&p, a, b), "doors {a:?} and {b:?} are not connected");
        // The paint pass walls the passage in with Path walls: no corridor tile
        // touches bare Void, and Room walls only ever sit on a room's box ring.
        let in_a_room = |x: usize, y: usize| {
            p.rooms.iter().any(|r| {
                x >= r.bounds.x && x <= r.bounds.right() && y >= r.bounds.y && y <= r.bounds.bottom()
            })
        };
        for &(x, y) in &corridor {
            for dy in -1i32..=1 {
                for dx in -1i32..=1 {
                    let (nx, ny) = (x as i32 + dx, y as i32 + dy);
                    assert!(nx >= 0 && ny >= 0 && (nx as usize) < p.w && (ny as usize) < p.h);
                    assert_ne!(
                        p.get(nx as usize, ny as usize),
                        Tile::Void,
                        "corridor at ({x},{y}) leaks into void at ({nx},{ny})"
                    );
                }
            }
        }
        for y in 0..p.h {
            for x in 0..p.w {
                if p.get(x, y) == (Tile::Wall { kind: WallKind::Room }) {
                    assert!(in_a_room(x, y), "Room wall at ({x},{y}) outside every room box");
                }
            }
        }
    }

    #[test]
    fn distorted_edge_between_placed_rooms_is_a_real_connection_not_stubs() {
        // A layout-level distorted edge (wrong-side geometry: the edge says N but
        // the destination sits two columns east) still gets a routed polyline —
        // it must realize as a real corridor with plain doors, not a stub pair.
        let mut g = MapGraph::new();
        grid_room(&mut g, 1, (0, 0));
        grid_room(&mut g, 2, (2, 0));
        g.add_edge(1, Direction::N, 2);
        let idx = g.connections().iter().position(|c| c.origin == 1).unwrap();
        g.set_conn_distorted(idx, true);
        let p = realize_layer(&g, MAIN_LAYER);
        assert!(p.conns.iter().any(|c| c.distorted), "the edge stays flagged distorted");
        let doors = door_tiles(&p);
        assert!(!doors.is_empty(), "distorted edge still gets doors");
        assert!(
            doors.iter().all(|&(_, _, _, k)| !matches!(k, DoorKind::Stub(_))),
            "distorted edge must not degrade to stubs: {doors:?}"
        );
        let (a, b) = ((doors[0].0, doors[0].1), (doors[1].0, doors[1].1));
        assert!(doors_connected(&p, a, b), "distorted edge's doors must be connected");
    }

    /// Stamp a room box (wall ring + floor) into a raw test grid.
    fn stamp_box(g: &mut Grid, bx: &RoomBox) {
        for y in bx.y0..=bx.y1 {
            for x in bx.x0..=bx.x1 {
                let edge = x == bx.x0 || x == bx.x1 || y == bx.y0 || y == bx.y1;
                let t = if edge {
                    Tile::Wall { kind: WallKind::Room }
                } else {
                    Tile::Floor { room: bx.id }
                };
                g.set(x, y, t);
            }
        }
    }

    /// A straight-E reciprocal connector between rooms 1 and 2 for direct
    /// `realize_corridor` tests.
    fn east_connector() -> RoutedConnector {
        RoutedConnector {
            origin: 1,
            dest: 2,
            distorted: false,
            exit: Side::Right,
            entry: Side::Left,
            points: vec![(0, 0), (2, 0), (4, 0)],
            segs: vec![],
            exit_slot: 0,
            entry_slot: 0,
            reciprocal: true,
            exit_dir: Direction::E,
            entry_dir: Some(Direction::W),
            entry_corner: None,
            merge: false,
            secondary_exit: vec![],
            secondary_entry: vec![],
        }
    }

    /// Wrap a raw grid as a TilePlan so the connectivity helper can walk it.
    fn plan_of(g: &Grid) -> TilePlan {
        TilePlan {
            w: g.w as usize,
            h: g.h as usize,
            tiles: g.tiles.clone(),
            ..TilePlan::default()
        }
    }

    #[test]
    fn corridor_routes_around_an_obstructing_room() {
        // A third room's box sits squarely on the straight line between the two
        // doors; the meander search must route around it, never through it.
        let mut g = Grid::new(40, 13);
        let ba = RoomBox { id: 1, x0: 0, y0: 3, x1: 6, y1: 9 };
        let bb = RoomBox { id: 2, x0: 30, y0: 3, x1: 36, y1: 9 };
        let bc = RoomBox { id: 3, x0: 14, y0: 3, x1: 22, y1: 9 };
        for bx in [&ba, &bb, &bc] {
            stamp_box(&mut g, bx);
        }
        assert!(
            realize_corridor(&mut g, &east_connector(), 0, &ba, &bb),
            "open space above/below room 3 → a route exists"
        );
        // Room 3 is untouched and the two doors are connected around it.
        for y in bc.y0..=bc.y1 {
            for x in bc.x0..=bc.x1 {
                assert!(
                    matches!(g.get(x, y), Tile::Wall { .. } | Tile::Floor { room: 3 }),
                    "room 3 tile at ({x},{y}) was overwritten: {:?}",
                    g.get(x, y)
                );
            }
        }
        let p = plan_of(&g);
        let doors = door_tiles(&p);
        assert_eq!(doors.len(), 2, "doors punched on both rooms: {doors:?}");
        let (a, b) = ((doors[0].0, doors[0].1), (doors[1].0, doors[1].1));
        assert!(doors_connected(&p, a, b), "corridor must connect the doors around room 3");
    }

    #[test]
    fn walled_off_route_falls_back_cleanly() {
        // The blocker spans the grid's full height: no route can exist. The
        // corridor must refuse and leave the grid byte-identical (the caller
        // then emits the stub pair).
        let mut g = Grid::new(40, 13);
        let ba = RoomBox { id: 1, x0: 0, y0: 3, x1: 6, y1: 9 };
        let bb = RoomBox { id: 2, x0: 30, y0: 3, x1: 36, y1: 9 };
        let bc = RoomBox { id: 3, x0: 14, y0: 0, x1: 22, y1: 12 };
        for bx in [&ba, &bb, &bc] {
            stamp_box(&mut g, bx);
        }
        let before = g.tiles.clone();
        assert!(
            !realize_corridor(&mut g, &east_connector(), 0, &ba, &bb),
            "a fully walled-off destination must be refused"
        );
        assert_eq!(g.tiles, before, "a refused corridor must leave the grid untouched");
    }

    #[test]
    fn long_corridors_meander_with_multiple_bends() {
        // Two rooms ~20 tiles apart with nothing but empty space between them:
        // the corridor must wander (≥3 bends — no 20-tile ruler line), stay
        // door-to-door connected, and be identical across two realizations.
        let mut g = MapGraph::new();
        grid_room(&mut g, 1, (0, 0));
        grid_room(&mut g, 2, (4, 0));
        g.add_edge(1, Direction::E, 2);
        g.add_edge(2, Direction::W, 1);
        let p = realize_layer(&g, MAIN_LAYER);
        assert_eq!(p, realize_layer(&g, MAIN_LAYER), "the meander must be deterministic");
        let doors = door_tiles(&p);
        assert_eq!(doors.len(), 2);
        let (a, b) = ((doors[0].0, doors[0].1), (doors[1].0, doors[1].1));
        assert!(doors_connected(&p, a, b), "meandering corridor still connects its doors");
        let on_path = |x: usize, y: usize| {
            matches!(p.get(x, y), Tile::Corridor { .. } | Tile::Door { .. } | Tile::Bridge { .. })
        };
        let mut bends = 0;
        for y in 0..p.h {
            for x in 0..p.w {
                if !matches!(p.get(x, y), Tile::Corridor { .. }) {
                    continue;
                }
                let n = y > 0 && on_path(x, y - 1);
                let s = y + 1 < p.h && on_path(x, y + 1);
                let e = x + 1 < p.w && on_path(x + 1, y);
                let w = x > 0 && on_path(x - 1, y);
                if (n ^ s) && (e ^ w) {
                    bends += 1;
                }
            }
        }
        assert!(bends >= 3, "a ~20-tile corridor should bend at least 3 times, got {bends}");
    }

    #[test]
    fn merge_connector_keeps_its_single_sided_stub() {
        // An extra edge between an already-connected pair collapses into a merge
        // stub: a Stub door on the origin only, no second line to the destination.
        let mut g = MapGraph::new();
        grid_room(&mut g, 1, (0, 0));
        grid_room(&mut g, 2, (2, 0));
        g.add_edge(1, Direction::E, 2);
        g.add_edge(2, Direction::W, 1);
        g.add_edge(1, Direction::SE, 2);
        let p = realize_layer(&g, MAIN_LAYER);
        let stubs: Vec<_> = door_tiles(&p)
            .into_iter()
            .filter(|&(_, _, _, k)| matches!(k, DoorKind::Stub(_)))
            .collect();
        assert_eq!(stubs.len(), 1, "one merge edge → exactly one single-sided stub: {stubs:?}");
    }

    #[test]
    fn perpendicular_crossing_corridors_get_a_bridge() {
        let mut g = MapGraph::new();
        grid_room(&mut g, 1, (0, 0));
        grid_room(&mut g, 2, (2, 0));
        grid_room(&mut g, 3, (1, -1));
        grid_room(&mut g, 4, (1, 1));
        g.add_edge(1, Direction::E, 2);
        g.add_edge(3, Direction::S, 4);
        let p = realize_layer(&g, MAIN_LAYER);
        let bridges = p.tiles.iter().filter(|t| matches!(t, Tile::Bridge { .. })).count();
        assert!(bridges >= 1, "perpendicular corridors must cross on a bridge");
    }

    #[test]
    fn up_edge_stamps_stairs_on_the_origin_floor() {
        let mut g = MapGraph::new();
        grid_room(&mut g, 1, (0, 0));
        grid_room(&mut g, 2, (1, 0));
        g.add_edge(1, Direction::Up, 2);
        let p = realize_layer(&g, MAIN_LAYER);
        let f = room(&p, 1).floor;
        let found = (f.y..=f.bottom()).any(|y| {
            (f.x..=f.right())
                .any(|x| p.get(x, y) == Tile::Feature { room: 1, kind: FeatureKind::StairsUp })
        });
        assert!(found, "Up edge → StairsUp feature on room 1's floor");
    }

    #[test]
    fn door_count_minimums_grow_the_floor() {
        // Three one-way E arrivals land on room 1's LEFT wall → floor_h ≥ 2*3+1 = 7.
        let mut g = MapGraph::new();
        grid_room(&mut g, 1, (0, 0));
        grid_room(&mut g, 2, (-1, 0));
        grid_room(&mut g, 3, (-1, -1));
        grid_room(&mut g, 4, (-1, 1));
        g.add_edge(2, Direction::E, 1);
        g.add_edge(3, Direction::E, 1);
        g.add_edge(4, Direction::E, 1);
        let p = realize_layer(&g, MAIN_LAYER);
        assert_eq!(room(&p, 1).floor.h, 7, "three left-wall doors need a 7-tall floor");
    }

    #[test]
    fn realize_layer_is_deterministic() {
        let g = big_graph();
        let a = realize_layer(&g, MAIN_LAYER);
        let b = realize_layer(&g, MAIN_LAYER);
        assert_eq!(a, b, "two realizations of the same graph must be identical");
    }

    #[test]
    fn every_connection_is_represented() {
        let g = big_graph();
        let p = realize_layer(&g, MAIN_LAYER);
        let doors = door_tiles(&p);
        let feature =
            |room: RoomId, kind: FeatureKind| p.tiles.contains(&Tile::Feature { room, kind });
        for c in g.connections() {
            let pair = (c.origin.min(c.dest), c.origin.max(c.dest));
            let ok = match c.dir {
                Direction::Unknown => continue, // skipped by design (parity with classic)
                Direction::Up => feature(c.origin, FeatureKind::StairsUp),
                Direction::Down => feature(c.origin, FeatureKind::StairsDown),
                Direction::In => feature(c.origin, FeatureKind::PortalIn),
                Direction::Out => feature(c.origin, FeatureKind::PortalOut),
                _ => doors.iter().any(|&(_, _, conn, _)| {
                    let tc = p.conns[conn as usize];
                    (tc.origin.min(tc.dest), tc.origin.max(tc.dest)) == pair
                }),
            };
            assert!(ok, "connection {c:?} has no door, corridor, stub, or feature");
        }
        // Distorted edges are real connections now: routed like any other, so in
        // this graph (no blocked routes) none of their doors may be stubs.
        for &(_, _, conn, kind) in &doors {
            if p.conns[conn as usize].distorted {
                assert!(
                    !matches!(kind, DoorKind::Stub(_)),
                    "distorted edge {:?} degraded to a stub",
                    p.conns[conn as usize]
                );
            }
        }
    }

    /// Stability regression guard: a Zork-flavoured exploration walk (30
    /// observations) driven through the `Mapper` facade in Auto mode —
    /// N/S/E/W moves, two diagonals, Up and Down, and several loop closures
    /// revisiting known rooms. After every observation the MAIN_LAYER plan is
    /// realized and compared with the previous turn's:
    /// - realization never panics and is deterministic (realize twice, equal);
    /// - a room whose logical cell and incident-connection count are both
    ///   unchanged keeps its floor SIZE, unless its own row/col track changed
    ///   size or a gutter opened/closed on an adjacent channel (abutment
    ///   extends a floor to the shared track boundary, so size legitimately
    ///   depends on that much context — and nothing else);
    /// - for any two rooms that both stayed on their logical cells, their
    ///   floor-rect ORDER never flips — by x within a shared row track, by y
    ///   within a shared column track.
    ///
    /// Positions may still shift when tracks resize or gutters open — the goal
    /// is catching gross reshuffles, not pinning exact coordinates.
    #[test]
    fn exploration_walk_keeps_tile_plans_stable() {
        use crate::mapper::Mapper;
        use Direction::{Down, Up, E, N, NE, S, SW, W};
        let walk: &[(RoomId, &str, Option<Direction>)] = &[
            (1, "West of House", None),
            (2, "North of House", Some(N)),
            (3, "Behind House", Some(E)),
            (5, "Kitchen", Some(W)),
            (6, "Living Room", Some(W)),
            (8, "Cellar", Some(Down)),
            (6, "Living Room", Some(Up)),
            (5, "Kitchen", Some(E)),
            (7, "Attic", Some(Up)),
            (5, "Kitchen", Some(Down)),
            (3, "Behind House", Some(E)),
            (4, "South of House", Some(S)),
            (1, "West of House", Some(W)), // loop closure around the house
            (9, "Forest", Some(W)),
            (10, "Forest Path", Some(N)),
            (2, "North of House", Some(E)), // closure onto a mapped room
            (10, "Forest Path", Some(W)),
            (11, "Clearing", Some(NE)), // diagonal
            (12, "Canyon View", Some(E)),
            (13, "Rocky Ledge", Some(Down)),
            (14, "Canyon Bottom", Some(S)),
            (15, "End of Rainbow", Some(SW)), // diagonal
            (14, "Canyon Bottom", Some(NE)),
            (13, "Rocky Ledge", Some(N)),
            (12, "Canyon View", Some(Up)),
            (11, "Clearing", Some(W)),
            (16, "Forest 2", Some(N)),
            (10, "Forest Path", Some(S)), // geometrically-inconsistent closure
            (9, "Forest", Some(S)),
            (1, "West of House", Some(E)), // final closure home
        ];
        use std::collections::BTreeSet;
        /// The track/gutter context a room's realized floor size may
        /// legitimately depend on besides its own doors: per-column track
        /// width, per-row track height (from S1 base sizes), and which
        /// channels carry a gutter (abutment on/off). Mirrors
        /// `realize_layer`'s S1/S2 pre-computation.
        type SizeContext = (BTreeMap<i32, i32>, BTreeMap<i32, i32>, BTreeSet<i32>, BTreeSet<i32>);
        fn size_context(g: &MapGraph) -> SizeContext {
            let sub = g.layer_subgraph(MAIN_LAYER);
            let mut planar = sub.clone();
            let np: Vec<(RoomId, Direction)> = planar
                .connections()
                .iter()
                .filter(|c| grid_offset(c.dir).is_none())
                .map(|c| (c.origin, c.dir))
                .collect();
            for (o, d) in np {
                planar.remove_connection(o, d);
            }
            let route = route_lanes(&planar);
            let mut v: BTreeSet<i32> = route.v_lanes.keys().copied().collect();
            let mut h: BTreeSet<i32> = route.h_lanes.keys().copied().collect();
            for &(vc, hc) in &route.diag_corners {
                v.insert(vc);
                h.insert(hc);
            }
            let sizes = floor_sizes(&sub);
            let mut col: BTreeMap<i32, i32> = BTreeMap::new();
            let mut row: BTreeMap<i32, i32> = BTreeMap::new();
            for r in sub.rooms() {
                let Some((cx, cy)) = r.pos else { continue };
                let (fw, fh) = sizes[&r.id];
                let e = col.entry(cx).or_insert(0);
                *e = (*e).max(fw + 2);
                let e = row.entry(cy).or_insert(0);
                *e = (*e).max(fh + 2);
            }
            (col, row, v, h)
        }
        type Snapshot =
            (TilePlan, BTreeMap<RoomId, (i32, i32)>, BTreeMap<RoomId, usize>, SizeContext);
        let mut m = Mapper::default();
        let mut prev: Option<Snapshot> = None;
        for &(id, name, via) in walk {
            m.observe(id, name, via);
            let plan = realize_layer(&m.graph, MAIN_LAYER);
            assert_eq!(
                plan,
                realize_layer(&m.graph, MAIN_LAYER),
                "realization must be deterministic after entering {name}"
            );
            let cells: BTreeMap<RoomId, (i32, i32)> =
                m.graph.rooms().filter_map(|r| r.pos.map(|p| (r.id, p))).collect();
            let mut conns: BTreeMap<RoomId, usize> = BTreeMap::new();
            for c in m.graph.connections() {
                *conns.entry(c.origin).or_default() += 1;
                *conns.entry(c.dest).or_default() += 1;
            }
            let ctx = size_context(&m.graph);
            for &rid in cells.keys() {
                assert!(
                    plan.rooms.iter().any(|r| r.id == rid),
                    "placed room {rid} missing from the tile plan after entering {name}"
                );
            }
            if let Some((pp, pc, pn, px)) = &prev {
                let on_same_cell = |rid: RoomId| pc.get(&rid) == cells.get(&rid);
                let same_ctx = |rid: RoomId| {
                    let (cx, cy) = cells[&rid];
                    px.0.get(&cx) == ctx.0.get(&cx)
                        && px.1.get(&cy) == ctx.1.get(&cy)
                        && [cx - 1, cx].iter().all(|c| px.2.contains(c) == ctx.2.contains(c))
                        && [cy - 1, cy].iter().all(|c| px.3.contains(c) == ctx.3.contains(c))
                };
                for r in &plan.rooms {
                    let Some(pr) = pp.rooms.iter().find(|q| q.id == r.id) else { continue };
                    if on_same_cell(r.id) && pn.get(&r.id) == conns.get(&r.id) && same_ctx(r.id) {
                        assert_eq!(
                            (pr.floor.w, pr.floor.h),
                            (r.floor.w, r.floor.h),
                            "room {} floor resized with its cell, connections, track \
                             sizes, and adjacent gutters all unchanged after entering {name}",
                            r.id
                        );
                    }
                    for b in &plan.rooms {
                        if b.id <= r.id || !on_same_cell(r.id) || !on_same_cell(b.id) {
                            continue;
                        }
                        let Some(pb) = pp.rooms.iter().find(|q| q.id == b.id) else { continue };
                        let (ca, cb) = (cells[&r.id], cells[&b.id]);
                        if ca.1 == cb.1 {
                            assert_eq!(
                                pr.floor.x < pb.floor.x,
                                r.floor.x < b.floor.x,
                                "rooms {} and {} swapped x-order in row {} after entering {name}",
                                r.id,
                                b.id,
                                ca.1
                            );
                        }
                        if ca.0 == cb.0 {
                            assert_eq!(
                                pr.floor.y < pb.floor.y,
                                r.floor.y < b.floor.y,
                                "rooms {} and {} swapped y-order in column {} after entering {name}",
                                r.id,
                                b.id,
                                ca.0
                            );
                        }
                    }
                }
            }
            prev = Some((plan, cells, conns, ctx));
        }
    }

    #[test]
    fn floors_never_overlap_and_bounds_stay_inside_the_grid() {
        let g = big_graph();
        let p = realize_layer(&g, MAIN_LAYER);
        for (i, a) in p.rooms.iter().enumerate() {
            assert!(
                a.bounds.right() < p.w && a.bounds.bottom() < p.h,
                "room {} bounds {:?} exceed the {}x{} grid",
                a.id,
                a.bounds,
                p.w,
                p.h
            );
            for b in &p.rooms[i + 1..] {
                assert!(
                    !rects_intersect(a.floor, b.floor),
                    "floors of rooms {} and {} overlap: {:?} vs {:?}",
                    a.id,
                    b.id,
                    a.floor,
                    b.floor
                );
            }
        }
    }
}
