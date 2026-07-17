//! Vector-first realization orchestrator (task V4): the full continuous
//! pipeline behind tiles mode. Per layer:
//!
//! 1. [`district::partition`] splits the layer into districts (planar
//!    connected components, maze-classified);
//! 2. each district builds its continuous [`MapModel`] via
//!    [`model::build_model`] over the district's own sub-graph, then its
//!    intra-district connections are routed as true-angle polylines with
//!    [`vroute::route_path`] (reciprocal pairs collapsed to one path, door
//!    anchors fanned along room walls, other same-level rooms as obstacles,
//!    hash-seeded waviness on long hauls — zero waviness inside a maze);
//! 3. [`district::compose`] arranges the districts globally; each local model
//!    is translated by its composition offset into ONE global model;
//! 4. cross-district stair links (Up/Down — the only edges that can cross a
//!    district boundary, since planar edges define the districts) are routed
//!    against the GLOBAL obstacle set as dashed (`distorted`) trails; In/Out
//!    stay features only, Unknown is skipped, matching the classic renderer;
//! 5. [`project::project_with_chambers`] rasterizes the model into a
//!    [`TilePlan`], stamping each maze district's chamber outline as
//!    `Wall { kind: Path }` tiles (pocket-style maze abstraction — today the
//!    member rooms still render inside the outline; deepening the abstraction
//!    to boundary-doors-only is a later zoom feature).
//!
//! Collapsed rooms (a small vertical stub folded onto its anchor's plane by
//! `model::collapse_small_clusters`) keep their stair features and gain a
//! short direct dashed connection to their anchor (same-level Up/Down link,
//! `distorted = true`, zero waviness) — the "stairs into the attic drawn on
//! the ground floor plan" look.
//!
//! `vroute::offset_parallel` is deliberately NOT applied: its documented
//! caveat is that it moves path endpoints, which would detach doors from
//! their walls. Re-anchoring doors after offsetting is a follow-up.
//!
//! Deterministic throughout: BTree iteration, connection order, FNV-1a
//! seeding — the same graph always yields the identical plan.

use std::collections::{BTreeMap, BTreeSet};

use crate::direction::{grid_offset, is_diagonal, opposite, Direction};
use crate::district::{compose, partition, District, DistrictKind};
use crate::graph::{MapGraph, RoomId};
use crate::layer::{LayerId, MAIN_LAYER};
use crate::model::{
    build_model, feature_anchor, MapModel, ModelFeature, ModelPath, ModelRoom, PathKind, Vec2,
};
use crate::project::project_with_chambers;
use crate::router::Side;
use crate::tiles::{FeatureKind, TilePlan};
use crate::vroute::{route_path, RectF, RouteOpts};

/// Chars per model unit (vertically; horizontal is doubled by projection) for
/// the Boxes zoom — the only zoom the tile renderer serves. Sized so a typical
/// 1.0–1.4-unit-wide room projects to an 11–15-char box (eyeballed on the
/// Zork map via `tiles_dump`).
pub const VECTOR_SCALE_BOXES: f32 = 5.5;

/// Seeded meander strength for long intra-district corridors.
const PATH_WAVINESS: f32 = 0.35;
/// Straight-line door distance (model units) above which waviness turns on.
const WAVY_MIN_UNITS: f32 = 4.0;

/// FNV-1a over `(origin, dest, dir)` — the per-path meander seed.
fn seed_for(origin: RoomId, dest: RoomId, dir: Direction) -> u64 {
    let o = origin.to_le_bytes();
    let d = dest.to_le_bytes();
    let bytes = [o[0], o[1], d[0], d[1], dir as u8];
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in bytes {
        h ^= u64::from(b);
        h = h.wrapping_mul(0x100_0000_01b3);
    }
    h
}

/// Unit-ish compass vector for a planar direction (route_path normalizes).
fn unit(dir: Direction) -> Option<Vec2> {
    let (dx, dy) = grid_offset(dir)?;
    Some((dx as f32, dy as f32))
}

/// The wall side a cardinal departure occupies (diagonals are corner points).
fn cardinal_side(dir: Direction) -> Option<Side> {
    match dir {
        Direction::N => Some(Side::Top),
        Direction::S => Some(Side::Bottom),
        Direction::E => Some(Side::Right),
        Direction::W => Some(Side::Left),
        _ => None,
    }
}

/// Point on `r`'s rect boundary facing `dir`: side midpoint for a cardinal,
/// the corner for a diagonal (fanned anchors replace midpoints when a side
/// carries several doors — see [`door_anchors`]).
fn boundary_anchor(r: &ModelRoom, dir: Direction) -> Vec2 {
    let (cx, cy) = r.center;
    let (hx, hy) = r.half;
    match dir {
        Direction::N => (cx, cy - hy),
        Direction::S => (cx, cy + hy),
        Direction::E => (cx + hx, cy),
        Direction::W => (cx - hx, cy),
        Direction::NE => (cx + hx, cy - hy),
        Direction::NW => (cx - hx, cy - hy),
        Direction::SE => (cx + hx, cy + hy),
        Direction::SW => (cx - hx, cy + hy),
        _ => (cx, cy),
    }
}

/// The compass octant best matching a free vector (max dot product;
/// deterministic first-wins tie-break in fixed clockwise-from-N order).
fn octant_dir(v: Vec2) -> Direction {
    const ORDER: [Direction; 8] = [
        Direction::N,
        Direction::NE,
        Direction::E,
        Direction::SE,
        Direction::S,
        Direction::SW,
        Direction::W,
        Direction::NW,
    ];
    let mut best = Direction::N;
    let mut best_dot = f32::MIN;
    for d in ORDER {
        let (dx, dy) = grid_offset(d).expect("octant dirs are planar");
        let len = ((dx * dx + dy * dy) as f32).sqrt();
        let dot = (dx as f32 * v.0 + dy as f32 * v.1) / len;
        if dot > best_dot + 1e-6 {
            best_dot = dot;
            best = d;
        }
    }
    best
}

fn rectf(r: &ModelRoom) -> RectF {
    RectF { cx: r.center.0, cy: r.center.1, hw: r.half.0, hh: r.half.1 }
}

/// One deduped drawable connection: reciprocal pairs collapsed to a single
/// path. Mirrors the classic router's pairing rule (`route::back_edge_idx`):
/// any unconsumed reverse compass edge makes the pair reciprocal, preferring
/// the exact opposite direction.
struct Planned {
    origin: RoomId,
    dest: RoomId,
    dir: Direction,
    reciprocal: bool,
    distorted: bool,
}

fn plan_connections(sub: &MapGraph) -> Vec<Planned> {
    let conns = sub.connections();
    let mut consumed = vec![false; conns.len()];
    let mut out = Vec::new();
    for i in 0..conns.len() {
        if consumed[i] {
            continue;
        }
        let c = &conns[i];
        if grid_offset(c.dir).is_none() || c.origin == c.dest {
            continue; // Up/Down/In/Out are features, Unknown is skipped
        }
        consumed[i] = true;
        let is_back = |j: usize| {
            let p = &conns[j];
            !consumed[j]
                && p.origin == c.dest
                && p.dest == c.origin
                && grid_offset(p.dir).is_some()
        };
        let back = (0..conns.len())
            .find(|&j| is_back(j) && conns[j].dir == opposite(c.dir))
            .or_else(|| (0..conns.len()).find(|&j| is_back(j)));
        let mut distorted = c.distorted;
        if let Some(j) = back {
            consumed[j] = true;
            distorted |= conns[j].distorted;
        }
        out.push(Planned {
            origin: c.origin,
            dest: c.dest,
            dir: c.dir,
            reciprocal: back.is_some(),
            distorted,
        });
    }
    out
}

/// Door anchor per path endpoint: diagonals take their corner; cardinal doors
/// on the same `(room, side)` fan out along the side, ordered by the far
/// room's position along the side's axis (ties by far id, then path index) so
/// corridors leave in roughly the direction they travel.
fn door_anchors(
    rooms: &BTreeMap<RoomId, ModelRoom>,
    planned: &[Planned],
) -> Vec<(Vec2, Vec2)> {
    let mut doors: Vec<(Vec2, Vec2)> = vec![((0.0, 0.0), (0.0, 0.0)); planned.len()];
    // One fanned door endpoint: (path index, is the origin end, far room).
    type Endpoint = (usize, bool, RoomId);
    let mut groups: BTreeMap<(RoomId, Side), Vec<Endpoint>> = BTreeMap::new();
    for (i, p) in planned.iter().enumerate() {
        let ends = [(true, p.origin, p.dir, p.dest), (false, p.dest, opposite(p.dir), p.origin)];
        for (is_a, room, dir, other) in ends {
            let Some(r) = rooms.get(&room) else { continue };
            if is_diagonal(dir) {
                let a = boundary_anchor(r, dir);
                if is_a {
                    doors[i].0 = a;
                } else {
                    doors[i].1 = a;
                }
            } else if let Some(side) = cardinal_side(dir) {
                groups.entry((room, side)).or_default().push((i, is_a, other));
            }
        }
    }
    for ((room, side), mut eps) in groups {
        let r = rooms[&room];
        let horiz = matches!(side, Side::Top | Side::Bottom); // span runs along x
        eps.sort_by(|a, b| {
            let key = |other: RoomId| {
                rooms
                    .get(&other)
                    .map(|o| if horiz { o.center.0 } else { o.center.1 })
                    .unwrap_or(0.0)
            };
            key(a.2).total_cmp(&key(b.2)).then(a.2.cmp(&b.2)).then(a.0.cmp(&b.0))
        });
        let k = eps.len() as f32;
        for (n, (i, is_a, _)) in eps.into_iter().enumerate() {
            let t = (n as f32 + 1.0) / (k + 1.0);
            let p = if horiz {
                let y = if side == Side::Top { r.center.1 - r.half.1 } else { r.center.1 + r.half.1 };
                (r.center.0 - r.half.0 + 2.0 * r.half.0 * t, y)
            } else {
                let x = if side == Side::Left { r.center.0 - r.half.0 } else { r.center.0 + r.half.0 };
                (x, r.center.1 - r.half.1 + 2.0 * r.half.1 * t)
            };
            if is_a {
                doors[i].0 = p;
            } else {
                doors[i].1 = p;
            }
        }
    }
    doors
}

/// Route every deduped intra-district connection into `model.paths`.
/// Obstacles are the district's OTHER rooms on either endpoint's level (rooms
/// on unrelated levels overlap this plane legitimately and must not block);
/// long hauls meander unless the district is a maze; incomplete routes become
/// honest [`PathKind::Stub`]s.
fn route_intra(model: &mut MapModel, sub: &MapGraph, maze: bool) {
    let rooms: BTreeMap<RoomId, ModelRoom> = model.rooms.iter().map(|r| (r.id, *r)).collect();
    let planned = plan_connections(sub);
    let doors = door_anchors(&rooms, &planned);
    for (p, (da, db)) in planned.iter().zip(doors) {
        let Some(a) = rooms.get(&p.origin) else { continue };
        let mut mp = ModelPath {
            origin: p.origin,
            dest: p.dest,
            dir: p.dir,
            kind: PathKind::Stub,
            distorted: p.distorted,
            reciprocal: p.reciprocal,
            door_a: da,
            door_b: db,
            points: Vec::new(),
        };
        if let Some(b) = rooms.get(&p.dest) {
            let obstacles: Vec<RectF> = model
                .rooms
                .iter()
                .filter(|r| {
                    r.id != p.origin
                        && r.id != p.dest
                        && (r.level == a.level || r.level == b.level)
                })
                .map(rectf)
                .collect();
            let dist = ((db.0 - da.0).powi(2) + (db.1 - da.1).powi(2)).sqrt();
            let waviness =
                if maze || dist <= WAVY_MIN_UNITS { 0.0 } else { PATH_WAVINESS };
            let opts = RouteOpts { waviness, ..RouteOpts::default() };
            let routed = route_path(
                &obstacles,
                da,
                unit(p.dir),
                db,
                unit(p.dir),
                seed_for(p.origin, p.dest, p.dir),
                &opts,
            );
            mp.kind = if routed.complete { PathKind::Corridor } else { PathKind::Stub };
            mp.points = routed.points;
        }
        model.paths.push(mp);
    }
}

/// Dashed stair links for collapsed rooms: every same-level Up/Down pair with
/// at least one collapsed endpoint gets a short direct `distorted` connection
/// (zero waviness) so the folded attic/cellar reads as attached to its
/// anchor. Real cross-level stairs stay features-only.
fn collapsed_links(model: &mut MapModel, sub: &MapGraph) {
    if model.collapsed.is_empty() {
        return;
    }
    let rooms: BTreeMap<RoomId, ModelRoom> = model.rooms.iter().map(|r| (r.id, *r)).collect();
    let collapsed: BTreeSet<RoomId> = model.collapsed.iter().copied().collect();
    let mut seen: BTreeSet<(RoomId, RoomId)> = BTreeSet::new();
    for c in sub.connections() {
        if !matches!(c.dir, Direction::Up | Direction::Down) || c.origin == c.dest {
            continue;
        }
        if !seen.insert((c.origin.min(c.dest), c.origin.max(c.dest))) {
            continue; // the reciprocal stair edge folds into one link
        }
        let (Some(a), Some(b)) = (rooms.get(&c.origin), rooms.get(&c.dest)) else { continue };
        if a.level != b.level || (!collapsed.contains(&a.id) && !collapsed.contains(&b.id)) {
            continue;
        }
        let dir = octant_dir((b.center.0 - a.center.0, b.center.1 - a.center.1));
        let da = boundary_anchor(a, dir);
        let db = boundary_anchor(b, opposite(dir));
        let reciprocal = sub.connections().iter().any(|r| {
            r.origin == c.dest
                && r.dest == c.origin
                && matches!(r.dir, Direction::Up | Direction::Down)
        });
        let obstacles: Vec<RectF> = model
            .rooms
            .iter()
            .filter(|r| r.id != a.id && r.id != b.id && r.level == a.level)
            .map(rectf)
            .collect();
        let routed = route_path(
            &obstacles,
            da,
            None,
            db,
            None,
            seed_for(c.origin, c.dest, c.dir),
            &RouteOpts::default(),
        );
        // Stub-free by design: even a fallback route draws its straight dashed
        // trail — the pair sits snugly together after the collapse anyway.
        model.paths.push(ModelPath {
            origin: c.origin,
            dest: c.dest,
            dir,
            kind: PathKind::Corridor,
            distorted: true,
            reciprocal,
            door_a: da,
            door_b: db,
            points: routed.points,
        });
    }
}

/// Append `local` translated by `off` into `global`.
fn translate_into(global: &mut MapModel, local: &MapModel, off: Vec2) {
    let tr = |p: Vec2| (p.0 + off.0, p.1 + off.1);
    for r in &local.rooms {
        global.rooms.push(ModelRoom { center: tr(r.center), ..*r });
    }
    for p in &local.paths {
        let mut p = p.clone();
        p.door_a = tr(p.door_a);
        p.door_b = tr(p.door_b);
        for q in &mut p.points {
            *q = tr(*q);
        }
        global.paths.push(p);
    }
    for f in &local.features {
        global.features.push(ModelFeature { at: tr(f.at), ..*f });
    }
    global.distorted_levels.extend(local.distorted_levels.iter().copied());
    global.collapsed.extend(local.collapsed.iter().copied());
}

/// Route cross-district Up/Down links as dashed trails against the global
/// obstacle set (all rooms except the endpoints). These are the only edges
/// that can cross a district boundary — planar edges define the districts —
/// and they are how a stairs-only annex visually attaches to the map.
fn inter_links(global: &mut MapModel, sub: &MapGraph, owner: &BTreeMap<RoomId, u16>) {
    let rooms: BTreeMap<RoomId, ModelRoom> = global.rooms.iter().map(|r| (r.id, *r)).collect();
    let mut seen: BTreeSet<(RoomId, RoomId)> = BTreeSet::new();
    for c in sub.connections() {
        if !matches!(c.dir, Direction::Up | Direction::Down) || c.origin == c.dest {
            continue;
        }
        if owner.get(&c.origin) == owner.get(&c.dest) {
            continue; // intra-district stairs were handled per district
        }
        if !seen.insert((c.origin.min(c.dest), c.origin.max(c.dest))) {
            continue;
        }
        let (Some(a), Some(b)) = (rooms.get(&c.origin), rooms.get(&c.dest)) else { continue };
        let dir = octant_dir((b.center.0 - a.center.0, b.center.1 - a.center.1));
        let da = boundary_anchor(a, dir);
        let db = boundary_anchor(b, opposite(dir));
        let reciprocal = sub.connections().iter().any(|r| {
            r.origin == c.dest
                && r.dest == c.origin
                && matches!(r.dir, Direction::Up | Direction::Down)
        });
        let obstacles: Vec<RectF> = global
            .rooms
            .iter()
            .filter(|r| r.id != a.id && r.id != b.id)
            .map(rectf)
            .collect();
        let routed = route_path(
            &obstacles,
            da,
            None,
            db,
            None,
            seed_for(c.origin, c.dest, c.dir),
            &RouteOpts::default(),
        );
        global.paths.push(ModelPath {
            origin: c.origin,
            dest: c.dest,
            dir,
            kind: if routed.complete { PathKind::Corridor } else { PathKind::Stub },
            distorted: true,
            reciprocal,
            door_a: da,
            door_b: db,
            points: routed.points,
        });
    }
}

/// Per-district builds only see intra-district edges, so a cross-district
/// stair/portal loses its origin-room feature glyph. Re-add any missing
/// feature from the full layer sub-graph, then restore (room, kind) order.
fn augment_features(global: &mut MapModel, sub: &MapGraph) {
    let rooms: BTreeMap<RoomId, ModelRoom> = global.rooms.iter().map(|r| (r.id, *r)).collect();
    let have: BTreeSet<(RoomId, u8)> =
        global.features.iter().map(|f| (f.room, f.kind as u8)).collect();
    let mut add: BTreeSet<(RoomId, u8)> = BTreeSet::new();
    for c in sub.connections() {
        let kind = match c.dir {
            Direction::Up => FeatureKind::StairsUp,
            Direction::Down => FeatureKind::StairsDown,
            Direction::In => FeatureKind::PortalIn,
            Direction::Out => FeatureKind::PortalOut,
            _ => continue,
        };
        if rooms.contains_key(&c.origin) && !have.contains(&(c.origin, kind as u8)) {
            add.insert((c.origin, kind as u8));
        }
    }
    for (room, rank) in add {
        let kind = match rank {
            0 => FeatureKind::StairsUp,
            1 => FeatureKind::StairsDown,
            2 => FeatureKind::PortalIn,
            _ => FeatureKind::PortalOut,
        };
        let r = &rooms[&room];
        global.features.push(ModelFeature { room, kind, at: feature_anchor(kind, r.center, r.half) });
    }
    global.features.sort_by_key(|f| (f.room, f.kind as u8));
}

/// A fresh graph holding only the district's rooms (positions preserved) and
/// the edges internal to it, distortion flags carried over. Rooms land on
/// `MAIN_LAYER` so `build_model(dg, MAIN_LAYER)` sees the whole district.
fn district_subgraph(sub: &MapGraph, d: &District) -> MapGraph {
    let set: BTreeSet<RoomId> = d.rooms.iter().copied().collect();
    let mut out = MapGraph::new();
    for &id in &d.rooms {
        let Some(r) = sub.room(id) else { continue };
        out.upsert_room(id, r.name.clone());
        if let Some(p) = r.pos {
            out.set_pos(id, p);
        }
    }
    for c in sub.connections() {
        if set.contains(&c.origin) && set.contains(&c.dest) {
            out.add_edge(c.origin, c.dir, c.dest);
            if c.distorted {
                let idx = out.connections().len() - 1;
                out.set_conn_distorted(idx, true);
            }
        }
    }
    out
}

/// Realize one layer through the full vector pipeline (see module docs) at
/// `scale` chars per model unit. Deterministic; never panics; rooms are never
/// dropped (projection guarantees).
pub fn realize_layer_vector(graph: &MapGraph, layer: LayerId, scale: f32) -> TilePlan {
    let sub = graph.layer_subgraph(layer);
    let districts = partition(graph, layer);
    if districts.is_empty() {
        return TilePlan::default();
    }

    // Per-district: continuous build + intra-district routing.
    let mut built: Vec<(District, MapModel)> = Vec::with_capacity(districts.len());
    for d in districts {
        let dg = district_subgraph(&sub, &d);
        let mut m = build_model(&dg, MAIN_LAYER);
        route_intra(&mut m, &dg, d.kind == DistrictKind::Maze);
        collapsed_links(&mut m, &dg);
        built.push((d, m));
    }

    // Compose districts globally; cross-district pairs attract each other.
    let owner: BTreeMap<RoomId, u16> = built
        .iter()
        .flat_map(|(d, _)| d.rooms.iter().map(|&r| (r, d.id)))
        .collect();
    let mut inter: Vec<(RoomId, RoomId)> = Vec::new();
    let mut seen: BTreeSet<(RoomId, RoomId)> = BTreeSet::new();
    for c in sub.connections() {
        if c.origin == c.dest || owner.get(&c.origin) == owner.get(&c.dest) {
            continue;
        }
        let key = (c.origin.min(c.dest), c.origin.max(c.dest));
        if seen.insert(key) {
            inter.push(key);
        }
    }
    let comp = compose(&built, &inter);
    let offsets: BTreeMap<u16, Vec2> = comp.offsets.iter().copied().collect();

    // One global model: translate each district by its composition offset.
    let mut global = MapModel::default();
    for (d, m) in &built {
        let off = offsets.get(&d.id).copied().unwrap_or((0.0, 0.0));
        translate_into(&mut global, m, off);
    }
    inter_links(&mut global, &sub, &owner);
    augment_features(&mut global, &sub);

    let chambers: Vec<Vec<Vec2>> = comp.chamber.iter().map(|(_, poly)| poly.clone()).collect();
    let cells: BTreeMap<RoomId, (i32, i32)> =
        sub.rooms().filter_map(|r| r.pos.map(|p| (r.id, p))).collect();
    project_with_chambers(&global, &cells, scale, &chambers)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tiles::{DoorKind, Tile, WallKind};

    fn room_at(g: &mut MapGraph, id: RoomId, pos: (i32, i32)) {
        g.upsert_room(id, format!("R{id}"));
        g.set_pos(id, pos);
    }

    /// Six rooms, two districts: a 4-room block (1–4, mixed reciprocal /
    /// one-way / diagonal planar edges) and a 2-room annex (5–6) attached only
    /// by a reciprocal stairwell 4↔5.
    fn two_district_graph() -> MapGraph {
        let mut g = MapGraph::new();
        room_at(&mut g, 1, (0, 0));
        room_at(&mut g, 2, (1, 0));
        room_at(&mut g, 3, (0, 1));
        room_at(&mut g, 4, (1, 1));
        room_at(&mut g, 5, (10, 0));
        room_at(&mut g, 6, (11, 0));
        g.add_edge(1, Direction::E, 2);
        g.add_edge(2, Direction::W, 1);
        g.add_edge(1, Direction::S, 3);
        g.add_edge(3, Direction::N, 1);
        g.add_edge(2, Direction::S, 4); // one-way
        g.add_edge(1, Direction::SE, 4); // diagonal
        g.add_edge(4, Direction::Up, 5);
        g.add_edge(5, Direction::Down, 4);
        g.add_edge(5, Direction::E, 6);
        g.add_edge(6, Direction::W, 5);
        g
    }

    fn door_conns(p: &TilePlan) -> BTreeSet<u16> {
        p.tiles
            .iter()
            .filter_map(|t| match t {
                Tile::Door { conn, .. } => Some(*conn),
                _ => None,
            })
            .collect()
    }

    #[test]
    fn two_district_end_to_end() {
        let g = two_district_graph();
        let p = realize_layer_vector(&g, MAIN_LAYER, 4.0);
        assert_eq!(p.rooms.len(), 6, "every room survives the pipeline");
        assert_eq!(
            p,
            realize_layer_vector(&g, MAIN_LAYER, 4.0),
            "vector realization must be deterministic"
        );
        // Every deduped planar connection (4 in district A, 1 in district B)
        // plus the cross-district stair link is a conn with a door.
        assert_eq!(p.conns.len(), 6, "4 + 1 planar paths + 1 stair link: {:?}", p.conns);
        let doors = door_conns(&p);
        for ci in 0..p.conns.len() as u16 {
            assert!(
                doors.contains(&ci),
                "conn {ci} ({:?}) has no door tile",
                p.conns[ci as usize]
            );
        }
        // The stair link is the dashed (distorted) one, between 4 and 5.
        let stair: Vec<_> = p.conns.iter().filter(|c| c.distorted).collect();
        assert_eq!(stair.len(), 1);
        assert_eq!((stair[0].origin, stair[0].dest), (4, 5));
        // Stair features made it across the district boundary on both sides.
        let has_feature = |kind: FeatureKind, room: RoomId| {
            p.tiles.iter().any(|t| matches!(t, Tile::Feature { room: r, kind: k } if *r == room && *k == kind))
        };
        assert!(has_feature(FeatureKind::StairsUp, 4), "room 4 keeps its up-stairs glyph");
        assert!(has_feature(FeatureKind::StairsDown, 5), "room 5 keeps its down-stairs glyph");
    }

    /// A one-room attic reached by stairs from room 1 but planar-connected to
    /// room 2 (so it shares the district and the collapse machinery fires).
    fn collapsed_attic_graph() -> MapGraph {
        let mut g = MapGraph::new();
        room_at(&mut g, 1, (0, 0));
        room_at(&mut g, 2, (1, 0));
        room_at(&mut g, 3, (1, -1));
        g.add_edge(1, Direction::E, 2);
        g.add_edge(2, Direction::W, 1);
        g.add_edge(1, Direction::Up, 3);
        g.add_edge(3, Direction::Down, 1);
        g.add_edge(3, Direction::S, 2);
        g.add_edge(2, Direction::N, 3);
        g
    }

    #[test]
    fn collapsed_attic_gets_dashed_link_and_stairs() {
        let g = collapsed_attic_graph();
        let p = realize_layer_vector(&g, MAIN_LAYER, 4.0);
        assert_eq!(p.rooms.len(), 3);
        assert_eq!(p, realize_layer_vector(&g, MAIN_LAYER, 4.0), "deterministic");
        // The folded stair link renders as a dashed reciprocal connection with
        // real doors (never stubs) on both rooms.
        let (ci, link) = p
            .conns
            .iter()
            .enumerate()
            .find(|(_, c)| c.distorted)
            .expect("collapsed stair pair yields a dashed link");
        assert_eq!((link.origin, link.dest), (1, 3));
        assert!(link.reciprocal, "reciprocal stair pair folds to one two-way link");
        let link_doors: Vec<DoorKind> = p
            .tiles
            .iter()
            .filter_map(|t| match t {
                Tile::Door { conn, kind } if *conn == ci as u16 => Some(*kind),
                _ => None,
            })
            .collect();
        assert!(!link_doors.is_empty(), "the dashed link punches doors");
        assert!(
            link_doors.iter().all(|k| *k == DoorKind::TwoWay),
            "stub-free: {link_doors:?}"
        );
        let has_feature = |kind: FeatureKind, room: RoomId| {
            p.tiles.iter().any(|t| matches!(t, Tile::Feature { room: r, kind: k } if *r == room && *k == kind))
        };
        assert!(has_feature(FeatureKind::StairsUp, 1), "anchor keeps its stairs-up glyph");
        assert!(has_feature(FeatureKind::StairsDown, 3), "attic keeps its stairs-down glyph");
    }

    /// K5-density cluster (5 rooms, every pair connected, several distorted):
    /// classifies as a Maze district.
    fn maze_graph() -> MapGraph {
        let mut g = MapGraph::new();
        let cells = [(0, 0), (1, 0), (2, 0), (0, 1), (1, 1)];
        for (i, c) in cells.iter().enumerate() {
            room_at(&mut g, 1 + i as u16, *c);
        }
        let dirs = [
            (1u16, Direction::E, 2u16),
            (1, Direction::N, 3),
            (1, Direction::S, 4),
            (1, Direction::SE, 5),
            (2, Direction::N, 3),
            (2, Direction::W, 4),
            (2, Direction::S, 5),
            (3, Direction::SW, 4),
            (3, Direction::S, 5),
            (4, Direction::E, 5),
        ];
        for &(a, d, b) in &dirs {
            g.add_edge(a, d, b);
        }
        for i in 0..4 {
            g.set_conn_distorted(i, true);
        }
        g
    }

    #[test]
    fn maze_district_is_stamped_with_a_chamber_outline() {
        let g = maze_graph();
        let p = realize_layer_vector(&g, MAIN_LAYER, 4.0);
        assert_eq!(p.rooms.len(), 5, "chamber stamping must not destroy member rooms");
        assert_eq!(p, realize_layer_vector(&g, MAIN_LAYER, 4.0), "deterministic");
        // Chamber walls carry their own kind (trails are unwalled, so any
        // remaining Path-kind wall would be a paint-pass leak).
        let chamber_walls = p
            .tiles
            .iter()
            .filter(|t| matches!(t, Tile::Wall { kind: WallKind::Chamber }))
            .count();
        assert!(chamber_walls > 0, "maze chamber outline must be stamped");
        assert!(
            !p.tiles.iter().any(|t| matches!(t, Tile::Wall { kind: WallKind::Path })),
            "vector plans no longer paint passage walls"
        );
        // Member rooms keep their floors and structural walls.
        for r in &p.rooms {
            assert!(
                p.tiles
                    .iter()
                    .any(|t| matches!(t, Tile::Floor { room } if *room == r.id)),
                "room {} lost its floor",
                r.id
            );
        }
    }

    #[test]
    fn empty_graph_yields_empty_plan() {
        let g = MapGraph::new();
        assert_eq!(realize_layer_vector(&g, MAIN_LAYER, 4.0), TilePlan::default());
    }
}
