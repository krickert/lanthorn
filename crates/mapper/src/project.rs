//! Projection of the continuous [`MapModel`] onto the tile grid (vector-map
//! pivot, task V3): scale `s` chars per model unit vertically and `2s`
//! horizontally (terminal cell aspect), snap rooms to integer rects with
//! shared-wall handling, punch doors where each path's door point meets its
//! owning wall, and rasterize path polylines into orthogonal and 45°-diagonal
//! corridor tiles. The output reuses the spike's [`TilePlan`] wholesale, so the
//! app rasterizer, theming, background-job plumbing, and scroll mapping keep
//! working unchanged.
//!
//! Room snapping rules:
//! - integer rects from the scaled model rect edges (edge rounding makes
//!   model-abutting neighbours land on the same wall line by construction),
//!   walls 1 tile, interior at least 3×1;
//! - rooms whose model rects abut (gap < [`ABUT_EPS`] units) or whose projected
//!   rects overlap by ≤ 1 column beyond a shared wall snap to a SHARED wall
//!   line, moving the smaller room;
//! - deeper projected overlaps resolve by shrinking the smaller room toward the
//!   minimum size and then nudging it one tile per pass, in deterministic
//!   (id-sorted) pair order. Rooms are never dropped and nothing panics.
//!
//! Path rasterization classifies every polyline segment: orthogonal → Corridor
//! tiles, exact 45° (in TILE space, i.e. after aspect scaling) →
//! [`Tile::CorridorDiag`], any other slope → stair-step orthogonal
//! decomposition. Corridor-vs-corridor perpendicular collisions and any
//! diagonal collision become [`Tile::Bridge`]. The shared passage paint pass
//! from `tiles` walls the result in.

use std::collections::BTreeMap;

use crate::direction::{grid_offset, is_diagonal, opposite, Direction};
use crate::graph::RoomId;
use crate::model::{MapModel, ModelFeature, ModelPath, ModelRoom, PathKind, Vec2};
use crate::tiles::{
    paint_path_walls, DiagSlope, DoorKind, FeatureKind, Rect, Tile, TileConn, TilePlan, TileRoom,
    WallKind,
};

/// Void margin around the projected content.
const MARGIN: i32 = 2;
/// Minimum room box extents, walls included (interior 3×1).
const MIN_BOX_W: i32 = 5;
const MIN_BOX_H: i32 = 3;
/// Model-space gap below which two rooms count as abutting (shared wall).
const ABUT_EPS: f32 = 0.05;
/// Overlap-resolution sweep cap; each pass moves offenders at least one tile.
const RESOLVE_PASSES: usize = 32;

/// Corridor travel axes for bridge detection (0 = none in the axis grid).
const AXIS_H: u8 = 1;
const AXIS_V: u8 = 2;
const AXIS_DIAG_DOWN: u8 = 3;
const AXIS_DIAG_UP: u8 = 4;

/// A room's mutable tile-space box during placement (walls included).
#[derive(Debug, Clone, Copy)]
struct RoomBox {
    id: RoomId,
    x0: i32,
    y0: i32,
    x1: i32,
    y1: i32,
}

/// Working grid: tiles plus the corridor travel axis per tile.
struct Grid {
    w: i32,
    h: i32,
    tiles: Vec<Tile>,
    axes: Vec<u8>,
}

impl Grid {
    fn new(w: i32, h: i32) -> Self {
        let n = (w.max(0) * h.max(0)) as usize;
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

/// Probe offsets along a wall: 0, +1, −1, +2, −2, … (classic slot order).
fn probe_shift(k: i32) -> i32 {
    let step = (k + 1) / 2;
    if k % 2 == 1 {
        step
    } else {
        -step
    }
}

/// Model point → tile point: x doubled for terminal cell aspect.
fn proj(p: Vec2, s: f32) -> (i32, i32) {
    ((p.0 * 2.0 * s).round() as i32, (p.1 * s).round() as i32)
}

/// Grow `[lo, hi]` symmetrically (hi first) until it spans at least `min`.
fn grow_to_min(lo: &mut i32, hi: &mut i32, min: i32) {
    let mut grow_hi = true;
    while *hi - *lo + 1 < min {
        if grow_hi {
            *hi += 1;
        } else {
            *lo -= 1;
        }
        grow_hi = !grow_hi;
    }
}

/// Index (into `boxes`) of the pair's SMALLER room — the one adjustments move.
/// Ties break toward the higher id, so the lower id keeps its place.
fn mover(boxes: &[RoomBox], i: usize, j: usize) -> usize {
    let area = |b: &RoomBox| (b.x1 - b.x0 + 1) * (b.y1 - b.y0 + 1);
    let (ai, aj) = (area(&boxes[i]), area(&boxes[j]));
    if ai < aj || (ai == aj && boxes[i].id > boxes[j].id) {
        i
    } else {
        j
    }
}

/// Snap model-abutting pairs to a shared wall line: whenever two model rects
/// abut on an axis (gap < [`ABUT_EPS`]) and overlap on the other, translate the
/// smaller room's box so the facing walls coincide.
fn snap_shared_walls(rooms: &[ModelRoom], boxes: &mut [RoomBox]) {
    for i in 0..rooms.len() {
        for j in (i + 1)..rooms.len() {
            let (a, b) = (rooms[i], rooms[j]);
            let dx = (a.center.0 - b.center.0).abs();
            let dy = (a.center.1 - b.center.1).abs();
            let gapx = dx - (a.half.0 + b.half.0);
            let gapy = dy - (a.half.1 + b.half.1);
            let x_overlap = dx < a.half.0 + b.half.0 - 1e-4;
            let y_overlap = dy < a.half.1 + b.half.1 - 1e-4;
            if gapx.abs() < ABUT_EPS && y_overlap {
                let (li, ri) =
                    if a.center.0 <= b.center.0 { (i, j) } else { (j, i) };
                let delta = boxes[ri].x0 - boxes[li].x1;
                if delta != 0 {
                    let m = mover(boxes, i, j);
                    let d = if m == li { delta } else { -delta };
                    boxes[m].x0 += d;
                    boxes[m].x1 += d;
                }
            } else if gapy.abs() < ABUT_EPS && x_overlap {
                let (ti, bi) =
                    if a.center.1 <= b.center.1 { (i, j) } else { (j, i) };
                let delta = boxes[bi].y0 - boxes[ti].y1;
                if delta != 0 {
                    let m = mover(boxes, i, j);
                    let d = if m == ti { delta } else { -delta };
                    boxes[m].y0 += d;
                    boxes[m].y1 += d;
                }
            }
        }
    }
}

/// Shared tile columns/rows of two boxes on one axis (1 = exactly the shared
/// wall line; ≥ 2 = genuine intrusion needing resolution).
fn overlap(a0: i32, a1: i32, b0: i32, b1: i32) -> i32 {
    a1.min(b1) - a0.max(b0) + 1
}

/// Resolve projected box overlaps deeper than a shared wall line: per pair (in
/// id order) pick the axis of least penetration; ≤ 1 tile beyond the wall snaps
/// to a shared line, deeper first shrinks the smaller room toward its minimum
/// size and then nudges it one tile. Bounded sweeps; never drops a room.
fn resolve_overlaps(boxes: &mut [RoomBox]) {
    for _ in 0..RESOLVE_PASSES {
        let mut changed = false;
        for i in 0..boxes.len() {
            for j in (i + 1)..boxes.len() {
                let pen_x = overlap(boxes[i].x0, boxes[i].x1, boxes[j].x0, boxes[j].x1);
                let pen_y = overlap(boxes[i].y0, boxes[i].y1, boxes[j].y0, boxes[j].y1);
                if pen_x < 2 || pen_y < 2 {
                    continue;
                }
                changed = true;
                let m = mover(boxes, i, j);
                let o = if m == i { j } else { i };
                if pen_x <= pen_y {
                    let left = (boxes[m].x0 + boxes[m].x1, boxes[m].id)
                        < (boxes[o].x0 + boxes[o].x1, boxes[o].id);
                    let excess = pen_x - 1;
                    if excess <= 1 {
                        let d = if left { -excess } else { excess };
                        boxes[m].x0 += d;
                        boxes[m].x1 += d;
                    } else {
                        let slack = (boxes[m].x1 - boxes[m].x0 + 1 - MIN_BOX_W).max(0);
                        let cut = slack.min(excess - 1);
                        if left {
                            boxes[m].x1 -= cut;
                        } else {
                            boxes[m].x0 += cut;
                        }
                        if overlap(boxes[i].x0, boxes[i].x1, boxes[j].x0, boxes[j].x1) >= 2 {
                            let d = if left { -1 } else { 1 };
                            boxes[m].x0 += d;
                            boxes[m].x1 += d;
                        }
                    }
                } else {
                    let above = (boxes[m].y0 + boxes[m].y1, boxes[m].id)
                        < (boxes[o].y0 + boxes[o].y1, boxes[o].id);
                    let excess = pen_y - 1;
                    if excess <= 1 {
                        let d = if above { -excess } else { excess };
                        boxes[m].y0 += d;
                        boxes[m].y1 += d;
                    } else {
                        let slack = (boxes[m].y1 - boxes[m].y0 + 1 - MIN_BOX_H).max(0);
                        let cut = slack.min(excess - 1);
                        if above {
                            boxes[m].y1 -= cut;
                        } else {
                            boxes[m].y0 += cut;
                        }
                        if overlap(boxes[i].y0, boxes[i].y1, boxes[j].y0, boxes[j].y1) >= 2 {
                            let d = if above { -1 } else { 1 };
                            boxes[m].y0 += d;
                            boxes[m].y1 += d;
                        }
                    }
                }
            }
        }
        if !changed {
            break;
        }
    }
}

/// Door tile candidates (preferred first) plus the outward unit step.
type DoorSpec = (Vec<(i32, i32)>, (i32, i32));

/// Door tile candidates (preferred first) plus the outward step for one path
/// endpoint on `bx`: the box corner (falling back to the two wall tiles beside
/// it) for a diagonal, else the correct side's wall clamped around the
/// projected door point, probing along the non-corner span. `None` for
/// non-planar directions (those paths are not drawable).
fn door_candidates(bx: &RoomBox, dir: Direction, at: (i32, i32)) -> Option<DoorSpec> {
    let (dx, dy) = grid_offset(dir)?;
    if is_diagonal(dir) {
        let x = if dx > 0 { bx.x1 } else { bx.x0 };
        let y = if dy > 0 { bx.y1 } else { bx.y0 };
        let ix = if x == bx.x0 { x + 1 } else { x - 1 };
        let iy = if y == bx.y0 { y + 1 } else { y - 1 };
        return Some((vec![(x, y), (x, iy), (ix, y)], (dx, dy)));
    }
    let mut cands = Vec::new();
    if dy == 0 {
        // E/W: a vertical wall, door somewhere along its non-corner span.
        let x = if dx > 0 { bx.x1 } else { bx.x0 };
        let (lo, hi) = (bx.y0 + 1, bx.y1 - 1);
        let want = at.1.clamp(lo, hi);
        for k in 0..2 * (hi - lo + 1).max(1) {
            let y = want + probe_shift(k);
            if (lo..=hi).contains(&y) {
                cands.push((x, y));
            }
        }
    } else {
        let y = if dy > 0 { bx.y1 } else { bx.y0 };
        let (lo, hi) = (bx.x0 + 1, bx.x1 - 1);
        let want = at.0.clamp(lo, hi);
        for k in 0..2 * (hi - lo + 1).max(1) {
            let x = want + probe_shift(k);
            if (lo..=hi).contains(&x) {
                cands.push((x, y));
            }
        }
    }
    if cands.is_empty() {
        cands.push((bx.x0, bx.y0)); // degenerate box: never happens at min sizes
    }
    Some((cands, (dx, dy)))
}

/// Place `door` on the first candidate that is still plain wall; failing that,
/// the first that is not already someone else's door; last resort, the first
/// candidate outright. Returns where it landed.
fn punch(g: &mut Grid, cands: &[(i32, i32)], door: Tile) -> (i32, i32) {
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

/// The travel axis of a unit step (diagonal steps map to their glyph slope).
fn axis_for_step(dx: i32, dy: i32) -> u8 {
    if dx != 0 && dy != 0 {
        if dx.signum() == dy.signum() {
            AXIS_DIAG_DOWN
        } else {
            AXIS_DIAG_UP
        }
    } else if dy == 0 {
        AXIS_H
    } else {
        AXIS_V
    }
}

/// Stamp one corridor tile. Only Void is claimed; a collision with a DIFFERENT
/// connection becomes a [`Tile::Bridge`] when the axes are perpendicular or
/// either side is diagonal (deterministic over/under: lower index over, as in
/// `tiles`). Rooms, doors, and existing bridges are left alone.
fn stamp(g: &mut Grid, x: i32, y: i32, conn: u16, axis: u8) {
    let Some(i) = g.idx(x, y) else { return };
    let diag = axis >= AXIS_DIAG_DOWN;
    match g.tiles[i] {
        Tile::Void => {
            g.tiles[i] = if diag {
                let slope =
                    if axis == AXIS_DIAG_DOWN { DiagSlope::Down } else { DiagSlope::Up };
                Tile::CorridorDiag { conn, slope }
            } else {
                Tile::Corridor { conn }
            };
            g.axes[i] = axis;
        }
        Tile::Corridor { conn: o } if o != conn && (diag || g.axes[i] != axis) => {
            g.tiles[i] = Tile::Bridge { over: o.min(conn), under: o.max(conn) };
        }
        Tile::CorridorDiag { conn: o, .. } if o != conn => {
            g.tiles[i] = Tile::Bridge { over: o.min(conn), under: o.max(conn) };
        }
        _ => {}
    }
}

/// Rasterize one polyline segment from `p` (exclusive) to `q` (inclusive):
/// orthogonal runs stamp Corridor, exact tile-space 45° runs stamp
/// CorridorDiag, anything else stair-steps through a 4-connected Bresenham so
/// the corridor stays orthogonally connected.
fn raster_segment(g: &mut Grid, conn: u16, p: (i32, i32), q: (i32, i32)) {
    let (dx, dy) = (q.0 - p.0, q.1 - p.1);
    if dx == 0 && dy == 0 {
        return;
    }
    let (sx, sy) = (dx.signum(), dy.signum());
    let (mut x, mut y) = p;
    if dy == 0 {
        while x != q.0 {
            x += sx;
            stamp(g, x, y, conn, AXIS_H);
        }
    } else if dx == 0 {
        while y != q.1 {
            y += sy;
            stamp(g, x, y, conn, AXIS_V);
        }
    } else if dx.abs() == dy.abs() {
        let axis = axis_for_step(sx, sy);
        while x != q.0 {
            x += sx;
            y += sy;
            stamp(g, x, y, conn, axis);
        }
    } else {
        // Stair-step decomposition: each step moves one axis, choosing the move
        // that keeps the accumulated error closest to the ideal line.
        let (adx, ady) = (i64::from(dx.abs()), i64::from(dy.abs()));
        let mut err: i64 = 0;
        while (x, y) != q {
            let step_x = if x == q.0 {
                false
            } else if y == q.1 {
                true
            } else {
                (err - ady).abs() <= (err + adx).abs()
            };
            if step_x {
                x += sx;
                err -= ady;
                stamp(g, x, y, conn, AXIS_H);
            } else {
                y += sy;
                err += adx;
                stamp(g, x, y, conn, AXIS_V);
            }
        }
    }
}

/// A stub endpoint: a `Stub` door plus one corridor tile fading outward.
fn realize_stub(g: &mut Grid, bx: &RoomBox, dir: Direction, at: (i32, i32), conn: u16) {
    let Some((cands, out)) = door_candidates(bx, dir, at) else { return };
    let p = punch(g, &cands, Tile::Door { conn, kind: DoorKind::Stub(dir) });
    stamp(g, p.0 + out.0, p.1 + out.1, conn, axis_for_step(out.0, out.1));
}

/// The boxes' shared-wall relation for a cardinal path, if their walls coincide:
/// `Some((vertical_wall, span))` where the span is the facing floors' overlap.
fn shared_wall_span(ba: &RoomBox, bb: &RoomBox, dir: Direction) -> Option<(bool, i32, i32, i32)> {
    let (wall_ok, vertical, line) = match dir {
        Direction::E => (ba.x1 == bb.x0, true, ba.x1),
        Direction::W => (ba.x0 == bb.x1, true, ba.x0),
        Direction::S => (ba.y1 == bb.y0, false, ba.y1),
        Direction::N => (ba.y0 == bb.y1, false, ba.y0),
        _ => return None,
    };
    if !wall_ok {
        return None;
    }
    let (lo, hi) = if vertical {
        ((ba.y0 + 1).max(bb.y0 + 1), (ba.y1 - 1).min(bb.y1 - 1))
    } else {
        ((ba.x0 + 1).max(bb.x0 + 1), (ba.x1 - 1).min(bb.x1 - 1))
    };
    (lo <= hi).then_some((vertical, line, lo, hi))
}

/// Feature stamping, mirroring `tiles::stamp_features` placement rules: stairs
/// land on a floor tile (probing along the row when the spot is taken), portals
/// replace a tile of the nearest vertical wall (probing along the wall).
fn stamp_feature(g: &mut Grid, bx: &RoomBox, kind: FeatureKind, at: (i32, i32)) {
    let feat = Tile::Feature { room: bx.id, kind };
    match kind {
        FeatureKind::StairsUp | FeatureKind::StairsDown => {
            let (lo, hi) = (bx.x0 + 1, bx.x1 - 1);
            let y = at.1.clamp(bx.y0 + 1, bx.y1 - 1);
            let want = at.0.clamp(lo, hi);
            for k in 0..2 * (hi - lo + 1).max(1) {
                let x = want + probe_shift(k);
                if !(lo..=hi).contains(&x) {
                    continue;
                }
                let t = g.get(x, y);
                if t == feat {
                    return;
                }
                if matches!(t, Tile::Floor { .. }) {
                    g.set(x, y, feat);
                    return;
                }
            }
        }
        FeatureKind::PortalIn | FeatureKind::PortalOut => {
            let x = if (at.0 - bx.x0).abs() < (at.0 - bx.x1).abs() { bx.x0 } else { bx.x1 };
            let (lo, hi) = (bx.y0 + 1, bx.y1 - 1);
            let want = at.1.clamp(lo, hi);
            for k in 0..2 * (hi - lo + 1).max(1) {
                let y = want + probe_shift(k);
                if !(lo..=hi).contains(&y) {
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
    }
}

/// Realize one drawable (non-stub) path: a single door through a shared wall
/// when the boxes abut on the path's cardinal, a single corner door when
/// diagonal endpoints coincide, else door → rasterized polyline → door. Paths
/// routed in model space carry their polyline in `points`; an empty polyline
/// (the V1 model) falls back to a straight door-to-door segment.
fn realize_path(
    g: &mut Grid,
    path: &ModelPath,
    conn: u16,
    ba: &RoomBox,
    bb: &RoomBox,
    s: f32,
    off: (i32, i32),
) {
    let kind = if path.reciprocal { DoorKind::TwoWay } else { DoorKind::OneWay(path.dir) };
    let door = Tile::Door { conn, kind };
    let tr = |p: Vec2| {
        let (x, y) = proj(p, s);
        (x + off.0, y + off.1)
    };
    let pa = tr(path.door_a);
    let pb = tr(path.door_b);
    if let Some((vertical, line, lo, hi)) = shared_wall_span(ba, bb, path.dir) {
        // Abutting boxes: one door punched straight through the shared wall.
        let want = if vertical {
            (line, pa.1.clamp(lo, hi))
        } else {
            (pa.0.clamp(lo, hi), line)
        };
        let mut cands = Vec::new();
        for k in 0..2 * (hi - lo + 1).max(1) {
            let p = if vertical {
                (line, want.1 + probe_shift(k))
            } else {
                (want.0 + probe_shift(k), line)
            };
            if (lo..=hi).contains(if vertical { &p.1 } else { &p.0 }) {
                cands.push(p);
            }
        }
        punch(g, &cands, door);
        return;
    }
    let Some((ca, _)) = door_candidates(ba, path.dir, pa) else { return };
    let Some((cb, _)) = door_candidates(bb, opposite(path.dir), pb) else { return };
    if ca[0] == cb[0] {
        // Diagonally-abutting boxes share the corner tile: the door IS the path.
        punch(g, &ca, door);
        return;
    }
    let da = punch(g, &ca, door);
    let db = punch(g, &cb, door);
    let mut pts: Vec<(i32, i32)> = if path.points.len() >= 2 {
        path.points.iter().map(|&p| tr(p)).collect()
    } else {
        vec![da, db]
    };
    // Pin the polyline's ends to the punched door tiles so the corridor always
    // connects door-to-door even after clamping/probing moved a door.
    let n = pts.len();
    pts[0] = da;
    pts[n - 1] = db;
    for w in pts.windows(2) {
        raster_segment(g, conn, w[0], w[1]);
    }
}

/// Project the continuous model onto a [`TilePlan`] at `scale` chars per model
/// unit vertically (and `2 * scale` horizontally, correcting terminal cell
/// aspect). `graph_cells` supplies each room's logical layout cell purely for
/// the [`TilePlan::col_x`]/[`TilePlan::row_y`] scroll-mapping tables: in vector
/// mode there is no track lattice, so each logical column/row maps to the MEAN
/// tile x/y of its rooms' box origins — an APPROXIMATE mapping, good enough to
/// keep the existing cell-based scroll and recenter working. Deterministic: the
/// same model and cells always yield the identical plan; rooms are never
/// dropped and no input can panic the projection.
pub fn project(model: &MapModel, graph_cells: &BTreeMap<RoomId, (i32, i32)>, scale: f32) -> TilePlan {
    if model.rooms.is_empty() {
        return TilePlan::default();
    }
    let s = if scale.is_finite() { scale.max(0.05) } else { 0.05 };

    // ── rooms: scaled integer rects, min interior, shared-wall snapping ──
    let mut rooms: Vec<ModelRoom> = model.rooms.clone();
    rooms.sort_by_key(|r| r.id);
    let mut boxes: Vec<RoomBox> = rooms
        .iter()
        .map(|r| {
            let (mut x0, mut y0) =
                proj((r.center.0 - r.half.0, r.center.1 - r.half.1), s);
            let (mut x1, mut y1) =
                proj((r.center.0 + r.half.0, r.center.1 + r.half.1), s);
            grow_to_min(&mut x0, &mut x1, MIN_BOX_W);
            grow_to_min(&mut y0, &mut y1, MIN_BOX_H);
            RoomBox { id: r.id, x0, y0, x1, y1 }
        })
        .collect();
    snap_shared_walls(&rooms, &mut boxes);
    resolve_overlaps(&mut boxes);

    // ── translate so the min corner (boxes and path polylines) lands at MARGIN ──
    let mut min = (i32::MAX, i32::MAX);
    let mut max = (i32::MIN, i32::MIN);
    let mut fold = |x: i32, y: i32| {
        min = (min.0.min(x), min.1.min(y));
        max = (max.0.max(x), max.1.max(y));
    };
    for b in &boxes {
        fold(b.x0, b.y0);
        fold(b.x1, b.y1);
    }
    for p in &model.paths {
        for &q in &p.points {
            let (x, y) = proj(q, s);
            fold(x, y);
        }
    }
    let off = (MARGIN - min.0, MARGIN - min.1);
    for b in &mut boxes {
        b.x0 += off.0;
        b.x1 += off.0;
        b.y0 += off.1;
        b.y1 += off.1;
    }
    let (w, h) = (max.0 + off.0 + 1 + MARGIN, max.1 + off.1 + 1 + MARGIN);
    let mut g = Grid::new(w, h);

    // ── stamp room boxes: wall ring + floor (shared walls simply coincide) ──
    for b in &boxes {
        for y in b.y0..=b.y1 {
            for x in b.x0..=b.x1 {
                let edge = x == b.x0 || x == b.x1 || y == b.y0 || y == b.y1;
                let t = if edge {
                    Tile::Wall { kind: WallKind::Room }
                } else {
                    Tile::Floor { room: b.id }
                };
                g.set(x, y, t);
            }
        }
    }
    let by_id: BTreeMap<RoomId, usize> =
        boxes.iter().enumerate().map(|(i, b)| (b.id, i)).collect();

    // ── doors + corridors, one TileConn per model path ──
    let conns: Vec<TileConn> = model
        .paths
        .iter()
        .map(|p| TileConn {
            origin: p.origin,
            dest: p.dest,
            dir: p.dir,
            reciprocal: p.reciprocal,
            distorted: p.distorted,
        })
        .collect();
    for (ci, path) in model.paths.iter().enumerate().take(usize::from(u16::MAX)) {
        let conn = ci as u16;
        let Some(&ai) = by_id.get(&path.origin) else { continue };
        let ba = boxes[ai];
        let bb = by_id.get(&path.dest).map(|&bi| boxes[bi]);
        let tr = |p: Vec2| {
            let (x, y) = proj(p, s);
            (x + off.0, y + off.1)
        };
        match (path.kind, bb) {
            (PathKind::Stub, _) | (_, None) => {
                realize_stub(&mut g, &ba, path.dir, tr(path.door_a), conn);
                if let Some(bb) = bb {
                    realize_stub(&mut g, &bb, opposite(path.dir), tr(path.door_b), conn);
                }
            }
            (PathKind::Corridor, Some(bb)) => {
                realize_path(&mut g, path, conn, &ba, &bb, s, off);
            }
        }
    }

    // ── features, then the shared passage paint pass ──
    for ModelFeature { room, kind, at } in &model.features {
        if let Some(&i) = by_id.get(room) {
            let (x, y) = proj(*at, s);
            stamp_feature(&mut g, &boxes[i], *kind, (x + off.0, y + off.1));
        }
    }
    paint_path_walls(g.w, g.h, &mut g.tiles);

    // ── plan assembly ──
    let out_rooms: Vec<TileRoom> = boxes
        .iter()
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
    // Approximate cell→tile scroll tables: mean box origin per logical col/row.
    let mut col_acc: BTreeMap<i32, (i64, i64)> = BTreeMap::new();
    let mut row_acc: BTreeMap<i32, (i64, i64)> = BTreeMap::new();
    for b in &boxes {
        if let Some(&(cx, cy)) = graph_cells.get(&b.id) {
            let c = col_acc.entry(cx).or_insert((0, 0));
            c.0 += i64::from(b.x0);
            c.1 += 1;
            let r = row_acc.entry(cy).or_insert((0, 0));
            r.0 += i64::from(b.y0);
            r.1 += 1;
        }
    }
    let col_x: Vec<(i32, i32)> =
        col_acc.iter().map(|(&c, &(sum, n))| (c, (sum / n) as i32)).collect();
    let row_y: Vec<(i32, i32)> =
        row_acc.iter().map(|(&r, &(sum, n))| (r, (sum / n) as i32)).collect();

    TilePlan { w: w as usize, h: h as usize, tiles: g.tiles, rooms: out_rooms, conns, col_x, row_y }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Single construction point for model rooms, so a future `ModelRoom` field
    /// needs exactly one fix here. Levels are ignored by projection.
    fn room(id: RoomId, cx: f32, cy: f32, hx: f32, hy: f32) -> ModelRoom {
        ModelRoom { id, center: (cx, cy), half: (hx, hy), level: 0 }
    }

    #[allow(clippy::too_many_arguments)]
    fn path(
        origin: RoomId,
        dest: RoomId,
        dir: Direction,
        kind: PathKind,
        reciprocal: bool,
        door_a: Vec2,
        door_b: Vec2,
        points: Vec<Vec2>,
    ) -> ModelPath {
        ModelPath { origin, dest, dir, kind, distorted: false, reciprocal, door_a, door_b, points }
    }

    fn find(p: &TilePlan, id: RoomId) -> &TileRoom {
        p.rooms.iter().find(|r| r.id == id).unwrap()
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

    fn floors_disjoint(p: &TilePlan) {
        for i in 0..p.rooms.len() {
            for j in (i + 1)..p.rooms.len() {
                let (a, b) = (p.rooms[i].floor, p.rooms[j].floor);
                let hit = a.x <= b.right() && b.x <= a.right() && a.y <= b.bottom() && b.y <= a.bottom();
                assert!(
                    !hit,
                    "floors of rooms {} and {} overlap: {a:?} vs {b:?}",
                    p.rooms[i].id, p.rooms[j].id
                );
            }
        }
    }

    /// True when `from` reaches `to` walking 4-adjacent passage tiles.
    fn doors_connected(p: &TilePlan, from: (usize, usize), to: (usize, usize)) -> bool {
        let passable = |x: usize, y: usize| {
            matches!(
                p.get(x, y),
                Tile::Corridor { .. }
                    | Tile::CorridorDiag { .. }
                    | Tile::Door { .. }
                    | Tile::Bridge { .. }
            )
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

    fn cells(pairs: &[(RoomId, (i32, i32))]) -> BTreeMap<RoomId, (i32, i32)> {
        pairs.iter().copied().collect()
    }

    #[test]
    fn tiny_scale_honors_min_room_sizes() {
        let m = MapModel {
            rooms: vec![room(1, 0.0, 0.0, 0.5, 0.35)],
            paths: Vec::new(),
            features: Vec::new(),
            ..MapModel::default()
        };
        let p = project(&m, &BTreeMap::new(), 0.2);
        let r = find(&p, 1);
        assert!(r.bounds.w >= 5 && r.bounds.h >= 3, "box {:?} under minimum", r.bounds);
        assert!(r.floor.w >= 3 && r.floor.h >= 1, "floor {:?} under minimum", r.floor);
        assert!(r.bounds.right() < p.w && r.bounds.bottom() < p.h, "box out of plan bounds");
    }

    #[test]
    fn abutting_model_pair_projects_to_shared_wall_with_one_door() {
        let m = MapModel {
            rooms: vec![room(1, 0.0, 0.0, 0.5, 0.5), room(2, 1.0, 0.0, 0.5, 0.5)],
            paths: vec![path(
                1,
                2,
                Direction::E,
                PathKind::Corridor,
                true,
                (0.5, 0.0),
                (0.5, 0.0),
                Vec::new(),
            )],
            features: Vec::new(),
            ..MapModel::default()
        };
        let p = project(&m, &BTreeMap::new(), 3.0);
        let (a, b) = (find(&p, 1), find(&p, 2));
        assert_eq!(a.bounds.right(), b.bounds.x, "boxes share exactly one wall column");
        let doors = door_tiles(&p);
        assert_eq!(doors.len(), 1, "one reciprocal abutting pair → one door: {doors:?}");
        let (x, _, conn, kind) = doors[0];
        assert_eq!(x, a.bounds.right(), "the door sits in the shared wall");
        assert_eq!(kind, DoorKind::TwoWay);
        assert_eq!((p.conns[conn as usize].origin, p.conns[conn as usize].dest), (1, 2));
        assert!(p.tiles.iter().all(|t| !matches!(t, Tile::Corridor { .. })), "no corridor needed");
    }

    #[test]
    fn projected_overlap_resolves_without_dropping_rooms() {
        // Model rects clear each other by 0.1 units, but min-size expansion at
        // scale 1 makes the boxes collide several columns deep.
        let m = MapModel {
            rooms: vec![room(1, 0.0, 0.0, 0.3, 0.3), room(2, 0.7, 0.0, 0.3, 0.3)],
            paths: Vec::new(),
            features: Vec::new(),
            ..MapModel::default()
        };
        let p = project(&m, &BTreeMap::new(), 1.0);
        assert_eq!(p.rooms.len(), 2, "no room may be dropped");
        floors_disjoint(&p);
        let q = project(&m, &BTreeMap::new(), 1.0);
        assert_eq!(p, q, "overlap resolution must be deterministic");
    }

    #[test]
    fn empty_points_fall_back_to_a_connected_straight_corridor() {
        let m = MapModel {
            rooms: vec![room(1, 0.0, 0.0, 0.5, 0.5), room(2, 3.0, 0.0, 0.5, 0.5)],
            paths: vec![path(
                1,
                2,
                Direction::E,
                PathKind::Corridor,
                true,
                (0.5, 0.0),
                (2.5, 0.0),
                Vec::new(),
            )],
            features: Vec::new(),
            ..MapModel::default()
        };
        let p = project(&m, &BTreeMap::new(), 1.5);
        let doors = door_tiles(&p);
        assert_eq!(doors.len(), 2, "a door on each room's wall: {doors:?}");
        assert!(
            p.tiles.iter().any(|t| matches!(t, Tile::Corridor { conn: 0 })),
            "distant rooms → corridor tiles"
        );
        let (a, b) = ((doors[0].0, doors[0].1), (doors[1].0, doors[1].1));
        assert!(doors_connected(&p, a, b), "doors {a:?} and {b:?} are not connected");
        // The reinstated paint pass walls the corridor in.
        assert!(
            p.tiles.iter().any(|t| matches!(t, Tile::Wall { kind: WallKind::Path })),
            "passage paint pass ran"
        );
    }

    #[test]
    fn tile_space_45_degree_path_rasterizes_as_diag_tiles() {
        // Geometry chosen so the two corner door tiles differ by exactly (4, 4)
        // in TILE space (model dy = 2·dx compensates the 2× x aspect scale).
        let m = MapModel {
            rooms: vec![room(1, 0.0, 0.0, 0.5, 0.5), room(2, 4.0, 5.5, 1.0, 0.5)],
            paths: vec![path(
                1,
                2,
                Direction::SE,
                PathKind::Corridor,
                true,
                (0.5, 0.5),
                (3.0, 5.0),
                Vec::new(),
            )],
            features: Vec::new(),
            ..MapModel::default()
        };
        let p = project(&m, &BTreeMap::new(), 1.0);
        assert_eq!(door_tiles(&p).len(), 2, "corner doors on both rooms");
        let diags: Vec<(u16, DiagSlope)> = p
            .tiles
            .iter()
            .filter_map(|t| match t {
                Tile::CorridorDiag { conn, slope } => Some((*conn, *slope)),
                _ => None,
            })
            .collect();
        assert_eq!(diags.len(), 3, "a 45° run of 3 interior tiles: {diags:?}");
        assert!(
            diags.iter().all(|&(c, sl)| c == 0 && sl == DiagSlope::Down),
            "SE travel slopes down (╲): {diags:?}"
        );
    }

    #[test]
    fn crossing_corridors_become_a_bridge() {
        let m = MapModel {
            rooms: vec![
                room(1, 0.0, 2.0, 0.5, 0.5),
                room(2, 6.0, 2.0, 0.5, 0.5),
                room(3, 3.0, 0.0, 0.5, 0.5),
                room(4, 3.0, 4.0, 0.5, 0.5),
            ],
            paths: vec![
                path(1, 2, Direction::E, PathKind::Corridor, true, (0.5, 2.0), (5.5, 2.0), Vec::new()),
                path(3, 4, Direction::S, PathKind::Corridor, true, (3.0, 0.5), (3.0, 3.5), Vec::new()),
            ],
            features: Vec::new(),
            ..MapModel::default()
        };
        let p = project(&m, &BTreeMap::new(), 1.0);
        let bridges: Vec<Tile> =
            p.tiles.iter().copied().filter(|t| matches!(t, Tile::Bridge { .. })).collect();
        assert_eq!(
            bridges,
            vec![Tile::Bridge { over: 0, under: 1 }],
            "perpendicular crossing → one deterministic bridge"
        );
    }

    #[test]
    fn stub_path_yields_stub_doors_with_fade_tiles() {
        let m = MapModel {
            rooms: vec![room(1, 0.0, 0.0, 0.5, 0.5), room(2, 3.0, 0.0, 0.5, 0.5)],
            paths: vec![path(
                1,
                2,
                Direction::E,
                PathKind::Stub,
                false,
                (0.5, 0.0),
                (2.5, 0.0),
                Vec::new(),
            )],
            features: Vec::new(),
            ..MapModel::default()
        };
        let p = project(&m, &BTreeMap::new(), 1.5);
        let doors = door_tiles(&p);
        assert_eq!(doors.len(), 2, "stub doors on both rooms: {doors:?}");
        assert!(matches!(doors[0].3, DoorKind::Stub(_)));
        assert!(matches!(doors[1].3, DoorKind::Stub(_)));
        let fades = p.tiles.iter().filter(|t| matches!(t, Tile::Corridor { .. })).count();
        assert_eq!(fades, 2, "exactly one fade tile per stub door");
    }

    #[test]
    fn features_land_inside_their_room() {
        let m = MapModel {
            rooms: vec![room(1, 0.0, 0.0, 0.8, 0.6)],
            paths: Vec::new(),
            features: vec![
                ModelFeature { room: 1, kind: FeatureKind::StairsUp, at: (0.4, -0.3) },
                ModelFeature { room: 1, kind: FeatureKind::PortalIn, at: (0.8, -0.15) },
            ],
            ..MapModel::default()
        };
        let p = project(&m, &BTreeMap::new(), 2.0);
        let r = find(&p, 1);
        let mut stairs = None;
        let mut portal = None;
        for y in 0..p.h {
            for x in 0..p.w {
                match p.get(x, y) {
                    Tile::Feature { kind: FeatureKind::StairsUp, .. } => stairs = Some((x, y)),
                    Tile::Feature { kind: FeatureKind::PortalIn, .. } => portal = Some((x, y)),
                    _ => {}
                }
            }
        }
        let (sx, sy) = stairs.expect("stairs stamped");
        assert!(
            sx >= r.floor.x && sx <= r.floor.right() && sy >= r.floor.y && sy <= r.floor.bottom(),
            "stairs at ({sx},{sy}) outside floor {:?}",
            r.floor
        );
        let (px, py) = portal.expect("portal stamped");
        assert_eq!(px, r.bounds.right(), "portal replaces a right-wall tile");
        assert!(py > r.bounds.y && py < r.bounds.bottom());
    }

    /// A composite fixture exercising rooms, an abutting pair, a corridor, a
    /// diagonal, and features together.
    fn composite() -> MapModel {
        MapModel {
            rooms: vec![
                room(1, 0.0, 0.0, 0.5, 0.5),
                room(2, 1.0, 0.0, 0.5, 0.5),
                room(3, 4.0, 0.0, 0.5, 0.5),
                room(4, 4.0, 4.0, 1.0, 0.5),
            ],
            paths: vec![
                path(1, 2, Direction::E, PathKind::Corridor, true, (0.5, 0.0), (0.5, 0.0), Vec::new()),
                path(2, 3, Direction::E, PathKind::Corridor, true, (1.5, 0.0), (3.5, 0.0), Vec::new()),
                path(3, 4, Direction::S, PathKind::Corridor, false, (4.0, 0.5), (4.0, 3.5), Vec::new()),
            ],
            features: vec![ModelFeature { room: 3, kind: FeatureKind::StairsUp, at: (4.3, -0.3) }],
            ..MapModel::default()
        }
    }

    #[test]
    fn projection_is_deterministic() {
        let m = composite();
        let cells = cells(&[(1, (0, 0)), (2, (1, 0)), (3, (2, 0)), (4, (2, 1))]);
        assert_eq!(project(&m, &cells, 1.5), project(&m, &cells, 1.5));
    }

    #[test]
    fn tile_plan_invariants_hold() {
        let p = project(&composite(), &BTreeMap::new(), 1.5);
        assert_eq!(p.tiles.len(), p.w * p.h, "row-major w*h tile buffer");
        assert_eq!(p.rooms.len(), 4);
        assert_eq!(p.conns.len(), 3);
        floors_disjoint(&p);
        for r in &p.rooms {
            assert!(
                r.bounds.right() < p.w && r.bounds.bottom() < p.h,
                "room {} box {:?} exceeds plan {}x{}",
                r.id, r.bounds, p.w, p.h
            );
            // The floor really is the box interior.
            assert_eq!((r.floor.x, r.floor.y), (r.bounds.x + 1, r.bounds.y + 1));
            assert_eq!((r.floor.w, r.floor.h), (r.bounds.w - 2, r.bounds.h - 2));
        }
    }

    #[test]
    fn scroll_tables_are_mean_box_origins_per_cell() {
        let m = MapModel {
            rooms: vec![room(1, 0.0, 0.0, 0.5, 0.5), room(2, 3.0, 0.0, 0.5, 0.5)],
            paths: Vec::new(),
            features: Vec::new(),
            ..MapModel::default()
        };
        let cells = cells(&[(1, (0, 0)), (2, (1, 0))]);
        let p = project(&m, &cells, 1.5);
        let (a, b) = (find(&p, 1), find(&p, 2));
        assert_eq!(
            p.col_x,
            vec![(0, a.bounds.x as i32), (1, b.bounds.x as i32)],
            "one room per column → its own box origin"
        );
        let mean_y = (a.bounds.y as i32 + b.bounds.y as i32) / 2;
        assert_eq!(p.row_y, vec![(0, mean_y)], "shared row → mean of the box tops");
    }
}
