//! Continuous (vector-first) map model: rooms as real-valued rectangles,
//! corridors as true-angle polylines, features as points. Geometry is solved in
//! isotropic model units (1.0 ≈ one room diameter) with no ASCII constraints; a
//! later projection step scales the model to the char grid.
//!
//! V1 scope: `build_model` produces rooms + features only (`paths` stays empty
//! until the polyline router lands). Room sizes come from door-count minimums
//! plus id-hash jitter (the continuous analogue of `tiles::floor_sizes`);
//! positions are seeded from the cell layout (`Room::pos`) and refined by a
//! rectangle-aware constrained stress pass reusing `layout::vpsc` /
//! `layout::stress`: per-axis separation gaps of `half_a + half_b + margin`,
//! with margin 0 for cardinally connected pairs (they may abut → shared wall
//! after projection) and a minimum margin otherwise. Separation constraints
//! follow cell order, so the relative order the cell layout established (N
//! stays above, W stays left) is preserved by construction. Deterministic: BTree
//! iteration, FNV-1a id hashing only, fixed iteration count.

use std::collections::{BTreeMap, BTreeSet};

use crate::direction::{grid_offset, opposite, Direction};
use crate::graph::{MapGraph, RoomId};
use crate::layer::LayerId;
use crate::layout::stress;
use crate::layout::vpsc::Constraint;
use crate::router::Side;
use crate::tiles::FeatureKind;

/// A point / extent in model space. `+x` is east, `+y` is south (cell layout
/// convention); units are isotropic — projection handles terminal aspect.
pub type Vec2 = (f32, f32);

/// A room as a real-valued rectangle: `center ± half` spans it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ModelRoom {
    pub id: RoomId,
    pub center: Vec2,
    pub half: Vec2,
}

/// How a path is realized geometrically.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PathKind {
    /// A drawable corridor between two placed rooms.
    Corridor,
    /// No drawable route; endpoints keep directional stubs.
    Stub,
}

/// One realized connector: a polyline from `door_a` (on the origin's wall) to
/// `door_b` (on the destination's wall). Built by the polyline router (V2);
/// `build_model` leaves [`MapModel::paths`] empty for now.
#[derive(Debug, Clone, PartialEq)]
pub struct ModelPath {
    pub origin: RoomId,
    pub dest: RoomId,
    pub dir: Direction,
    pub kind: PathKind,
    pub distorted: bool,
    pub reciprocal: bool,
    pub door_a: Vec2,
    pub door_b: Vec2,
    pub points: Vec<Vec2>,
}

/// A room-anchored point feature (stairs / portals).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ModelFeature {
    pub room: RoomId,
    pub kind: FeatureKind,
    pub at: Vec2,
}

/// The continuous geometry model for one layer.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct MapModel {
    pub rooms: Vec<ModelRoom>,
    pub paths: Vec<ModelPath>,
    pub features: Vec<ModelFeature>,
}

// ── sizing constants (model units) ───────────────────────────────────────────

/// Base / clamp bounds for room width (east–west extent).
pub const ROOM_W_MIN: f32 = 1.0;
pub const ROOM_W_MAX: f32 = 2.2;
/// Base / clamp bounds for room height (north–south extent).
pub const ROOM_H_MIN: f32 = 0.7;
pub const ROOM_H_MAX: f32 = 1.6;
/// Wall length one door needs; a side with `d` doors demands `d * DOOR_SPAN`.
pub const DOOR_SPAN: f32 = 0.4;
/// Minimum per-axis gap between rooms with no cardinal connection.
pub const UNCONNECTED_MARGIN: f32 = 0.35;

/// Hash-bit size jitter amplitudes (kept below the door/clamp headroom).
const JITTER_W: f32 = 0.2;
const JITTER_H: f32 = 0.15;
/// Seed pitch per layout cell (x wider: widths run larger than heights).
const SEED_PITCH_X: f64 = 3.0;
const SEED_PITCH_Y: f64 = 2.0;
/// Fixed SMACOF iterations for the refinement pass (determinism + bounded cost).
const ITERS: usize = 40;

/// FNV-1a over a room id — the deterministic jitter source (no RNG, no deps).
fn fnv1a(id: RoomId) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in id.to_le_bytes() {
        h ^= u64::from(b);
        h = h.wrapping_mul(0x100_0000_01b3);
    }
    h
}

/// The origin-room wall side(s) a departing connection occupies (same rule as
/// `tiles::departure_sides`): cardinals take one side, a diagonal is
/// corner-attached and counts toward both of its corner's sides.
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

/// Per-room continuous size `(w, h)` from door-count minimums plus id-hash
/// jitter — the continuous analogue of `tiles::floor_sizes`. Top/bottom doors
/// spread along the width; left/right along the height.
fn room_sizes(sub: &MapGraph) -> BTreeMap<RoomId, (f32, f32)> {
    let mut doors: BTreeMap<(RoomId, Side), i32> = BTreeMap::new();
    for c in sub.connections() {
        if grid_offset(c.dir).is_none() {
            continue;
        }
        for &s in departure_sides(c.dir) {
            *doors.entry((c.origin, s)).or_default() += 1;
        }
        // A one-way edge also lands a door at its destination; a reciprocal's
        // far door is already counted by the reverse edge's own departure.
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
            let d = |s: Side| doors.get(&(r.id, s)).copied().unwrap_or(0) as f32;
            let h = fnv1a(r.id);
            let jw = if h & 1 == 1 { JITTER_W } else { 0.0 };
            let jh = if (h >> 1) & 1 == 1 { JITTER_H } else { 0.0 };
            let w = (ROOM_W_MIN + jw)
                .max(DOOR_SPAN * d(Side::Top))
                .max(DOOR_SPAN * d(Side::Bottom))
                .clamp(ROOM_W_MIN, ROOM_W_MAX);
            let hgt = (ROOM_H_MIN + jh)
                .max(DOOR_SPAN * d(Side::Left))
                .max(DOOR_SPAN * d(Side::Right))
                .clamp(ROOM_H_MIN, ROOM_H_MAX);
            (r.id, (w, hgt))
        })
        .collect()
}

/// Rectangle-aware refinement: constrained stress majorization over the placed
/// rooms. Every pair whose cells differ on an axis gets a VPSC separation
/// constraint on that axis, in cell order, with gap `half_a + half_b + margin`
/// — so sizes genuinely push neighbors, order is preserved, and no two rects
/// can overlap. Cardinally connected pairs get margin 0 (may abut); everything
/// else keeps [`UNCONNECTED_MARGIN`].
fn refine_positions(
    ids: &[RoomId],
    cells: &BTreeMap<RoomId, (i32, i32)>,
    sizes: &BTreeMap<RoomId, (f32, f32)>,
    sub: &MapGraph,
) -> Vec<(f64, f64)> {
    let n = ids.len();
    let index: BTreeMap<RoomId, usize> =
        ids.iter().enumerate().map(|(i, &id)| (id, i)).collect();

    let mut cardinal: BTreeSet<(RoomId, RoomId)> = BTreeSet::new();
    for c in sub.connections() {
        if matches!(c.dir, Direction::N | Direction::S | Direction::E | Direction::W) {
            cardinal.insert((c.origin.min(c.dest), c.origin.max(c.dest)));
        }
    }

    let mut xc: Vec<Constraint> = Vec::new();
    let mut yc: Vec<Constraint> = Vec::new();
    for i in 0..n {
        for j in (i + 1)..n {
            let (a, b) = (ids[i], ids[j]);
            let (ca, cb) = (cells[&a], cells[&b]);
            let (sa, sb) = (sizes[&a], sizes[&b]);
            let margin = if cardinal.contains(&(a.min(b), a.max(b))) {
                0.0
            } else {
                f64::from(UNCONNECTED_MARGIN)
            };
            let xgap = f64::from(sa.0 + sb.0) / 2.0 + margin;
            let ygap = f64::from(sa.1 + sb.1) / 2.0 + margin;
            match ca.0.cmp(&cb.0) {
                std::cmp::Ordering::Less => xc.push(Constraint { left: i, right: j, gap: xgap }),
                std::cmp::Ordering::Greater => xc.push(Constraint { left: j, right: i, gap: xgap }),
                std::cmp::Ordering::Equal => {}
            }
            match ca.1.cmp(&cb.1) {
                std::cmp::Ordering::Less => yc.push(Constraint { left: i, right: j, gap: ygap }),
                std::cmp::Ordering::Greater => yc.push(Constraint { left: j, right: i, gap: ygap }),
                std::cmp::Ordering::Equal => {}
            }
            // Degenerate duplicate cell (shouldn't happen): force x order by id.
            if ca == cb {
                xc.push(Constraint { left: i, right: j, gap: xgap });
            }
        }
    }

    // Graph-distance targets pull connected rooms together (hop = 1.0 unit);
    // the VPSC projection each iteration keeps the gaps above feasible.
    let mut adj: Vec<Vec<usize>> = vec![Vec::new(); n];
    for c in sub.connections() {
        if grid_offset(c.dir).is_none() {
            continue;
        }
        let (Some(&i), Some(&j)) = (index.get(&c.origin), index.get(&c.dest)) else { continue };
        if i != j {
            adj[i].push(j);
            adj[j].push(i);
        }
    }
    let dist = stress::all_pairs_dist(n, &adj);
    let seed: Vec<(f64, f64)> = ids
        .iter()
        .map(|id| {
            let (cx, cy) = cells[id];
            (f64::from(cx) * SEED_PITCH_X, f64::from(cy) * SEED_PITCH_Y)
        })
        .collect();
    stress::stress_layout(n, &dist, &xc, &yc, &seed, ITERS)
}

/// Feature anchor inside/on the room rect, mirroring `tiles::stamp_features`
/// placement: stairs sit in the top-right (up) / bottom-right (down) interior,
/// portals sit on the right wall (in above center, out below).
fn feature_anchor(kind: FeatureKind, center: Vec2, half: Vec2) -> Vec2 {
    let (cx, cy) = center;
    let (hx, hy) = half;
    match kind {
        FeatureKind::StairsUp => (cx + 0.55 * hx, cy - 0.55 * hy),
        FeatureKind::StairsDown => (cx + 0.55 * hx, cy + 0.55 * hy),
        FeatureKind::PortalIn => (cx + hx, cy - 0.25 * hy),
        FeatureKind::PortalOut => (cx + hx, cy + 0.25 * hy),
    }
}

fn feature_rank(kind: FeatureKind) -> u8 {
    match kind {
        FeatureKind::StairsUp => 0,
        FeatureKind::StairsDown => 1,
        FeatureKind::PortalIn => 2,
        FeatureKind::PortalOut => 3,
    }
}

/// Build the continuous model for one layer's sub-graph. Rooms and features
/// only for now; `paths` is left empty until the polyline router (V2).
/// Deterministic: the same graph always yields the identical `MapModel`.
pub fn build_model(graph: &MapGraph, layer: LayerId) -> MapModel {
    let sub = graph.layer_subgraph(layer);
    let cells: BTreeMap<RoomId, (i32, i32)> =
        sub.rooms().filter_map(|r| r.pos.map(|p| (r.id, p))).collect();
    if cells.is_empty() {
        return MapModel::default();
    }
    let ids: Vec<RoomId> = cells.keys().copied().collect();
    let sizes = room_sizes(&sub);
    let pos = refine_positions(&ids, &cells, &sizes, &sub);

    let rooms: Vec<ModelRoom> = ids
        .iter()
        .enumerate()
        .map(|(i, &id)| {
            let (w, h) = sizes[&id];
            ModelRoom {
                id,
                center: (pos[i].0 as f32, pos[i].1 as f32),
                half: (w / 2.0, h / 2.0),
            }
        })
        .collect();

    // Features: Up/Down/In/Out edges become room-anchored points (Unknown is
    // skipped, matching the classic renderer). Deduped and emitted in
    // (room, kind) order for determinism regardless of connection order.
    let mut wanted: BTreeSet<(RoomId, u8)> = BTreeSet::new();
    for c in sub.connections() {
        let kind = match c.dir {
            Direction::Up => FeatureKind::StairsUp,
            Direction::Down => FeatureKind::StairsDown,
            Direction::In => FeatureKind::PortalIn,
            Direction::Out => FeatureKind::PortalOut,
            _ => continue,
        };
        if cells.contains_key(&c.origin) {
            wanted.insert((c.origin, feature_rank(kind)));
        }
    }
    let by_id: BTreeMap<RoomId, &ModelRoom> = rooms.iter().map(|r| (r.id, r)).collect();
    let features: Vec<ModelFeature> = wanted
        .iter()
        .map(|&(room, rank)| {
            let kind = match rank {
                0 => FeatureKind::StairsUp,
                1 => FeatureKind::StairsDown,
                2 => FeatureKind::PortalIn,
                _ => FeatureKind::PortalOut,
            };
            let r = by_id[&room];
            ModelFeature { room, kind, at: feature_anchor(kind, r.center, r.half) }
        })
        .collect();

    MapModel { rooms, paths: Vec::new(), features }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::layer::MAIN_LAYER;

    const EPS: f32 = 1e-3;

    fn room_at(g: &mut MapGraph, id: RoomId, pos: (i32, i32)) {
        g.upsert_room(id, format!("R{id}"));
        g.set_pos(id, pos);
    }

    fn find(m: &MapModel, id: RoomId) -> &ModelRoom {
        m.rooms.iter().find(|r| r.id == id).unwrap()
    }

    /// 3×3 block with mixed connectivity: reciprocal cardinals, a diagonal, and
    /// a door-heavy hub, so refinement has real rectangles to push around.
    fn block_graph() -> MapGraph {
        let mut g = MapGraph::new();
        for (i, id) in (1u16..=9).enumerate() {
            room_at(&mut g, id, ((i % 3) as i32, (i / 3) as i32));
        }
        // Row 1: 1-2-3 reciprocal E/W.
        g.add_edge(1, Direction::E, 2);
        g.add_edge(2, Direction::W, 1);
        g.add_edge(2, Direction::E, 3);
        g.add_edge(3, Direction::W, 2);
        // Columns: 1-4-7 and 3-6-9 reciprocal N/S.
        g.add_edge(1, Direction::S, 4);
        g.add_edge(4, Direction::N, 1);
        g.add_edge(4, Direction::S, 7);
        g.add_edge(7, Direction::N, 4);
        g.add_edge(3, Direction::S, 6);
        g.add_edge(6, Direction::N, 3);
        // Hub 5: doors on all sides plus a diagonal.
        g.add_edge(5, Direction::N, 2);
        g.add_edge(5, Direction::E, 6);
        g.add_edge(5, Direction::W, 4);
        g.add_edge(5, Direction::S, 8);
        g.add_edge(5, Direction::SE, 9);
        // One-way long haul.
        g.add_edge(7, Direction::E, 9);
        g
    }

    #[test]
    fn door_minimums_respected() {
        let mut g = MapGraph::new();
        // Room 1 departs N, NE, NW: three top doors → width ≥ 3·DOOR_SPAN.
        // Room 5 departs E, NE, SE: three right doors → height ≥ 3·DOOR_SPAN.
        for (id, pos) in
            [(1u16, (0, 0)), (2, (0, -1)), (3, (1, -1)), (4, (-1, -1)), (5, (5, 0)), (6, (6, 0)), (7, (6, -1)), (8, (6, 1))]
        {
            room_at(&mut g, id, pos);
        }
        g.add_edge(1, Direction::N, 2);
        g.add_edge(1, Direction::NE, 3);
        g.add_edge(1, Direction::NW, 4);
        g.add_edge(5, Direction::E, 6);
        g.add_edge(5, Direction::NE, 7);
        g.add_edge(5, Direction::SE, 8);
        let m = build_model(&g, MAIN_LAYER);
        assert!(
            2.0 * find(&m, 1).half.0 >= 3.0 * DOOR_SPAN - EPS,
            "3 top doors demand width >= {}, got {}",
            3.0 * DOOR_SPAN,
            2.0 * find(&m, 1).half.0
        );
        assert!(
            2.0 * find(&m, 5).half.1 >= 3.0 * DOOR_SPAN - EPS,
            "3 right doors demand height >= {}, got {}",
            3.0 * DOOR_SPAN,
            2.0 * find(&m, 5).half.1
        );
    }

    #[test]
    fn refinement_leaves_no_overlaps() {
        let m = build_model(&block_graph(), MAIN_LAYER);
        for i in 0..m.rooms.len() {
            for j in (i + 1)..m.rooms.len() {
                let (a, b) = (&m.rooms[i], &m.rooms[j]);
                let dx = (a.center.0 - b.center.0).abs();
                let dy = (a.center.1 - b.center.1).abs();
                let x_sep = dx + EPS >= a.half.0 + b.half.0;
                let y_sep = dy + EPS >= a.half.1 + b.half.1;
                assert!(
                    x_sep || y_sep,
                    "rooms {} and {} overlap: dx={dx} dy={dy} halves {:?} {:?}",
                    a.id,
                    b.id,
                    a.half,
                    b.half
                );
            }
        }
    }

    #[test]
    fn relative_cell_order_is_preserved() {
        let g = block_graph();
        let m = build_model(&g, MAIN_LAYER);
        let cell = |id: RoomId| g.room(id).unwrap().pos.unwrap();
        for a in &m.rooms {
            for b in &m.rooms {
                let (ca, cb) = (cell(a.id), cell(b.id));
                if ca.0 < cb.0 {
                    assert!(
                        a.center.0 < b.center.0,
                        "room {} (cell x {}) must stay west of {} (cell x {})",
                        a.id,
                        ca.0,
                        b.id,
                        cb.0
                    );
                }
                if ca.1 < cb.1 {
                    assert!(
                        a.center.1 < b.center.1,
                        "room {} (cell y {}) must stay north of {} (cell y {})",
                        a.id,
                        ca.1,
                        b.id,
                        cb.1
                    );
                }
            }
        }
    }

    #[test]
    fn connected_ew_pair_abuts() {
        let mut g = MapGraph::new();
        room_at(&mut g, 1, (0, 0));
        room_at(&mut g, 2, (1, 0));
        g.add_edge(1, Direction::E, 2);
        g.add_edge(2, Direction::W, 1);
        let m = build_model(&g, MAIN_LAYER);
        let (a, b) = (find(&m, 1), find(&m, 2));
        let gap = (b.center.0 - b.half.0) - (a.center.0 + a.half.0);
        assert!(gap.abs() < EPS, "connected E/W pair must abut, gap = {gap}");
    }

    #[test]
    fn unconnected_neighbors_keep_margin() {
        let mut g = MapGraph::new();
        room_at(&mut g, 1, (0, 0));
        room_at(&mut g, 2, (1, 0));
        room_at(&mut g, 3, (2, 0));
        // 1 and 3 pull toward each other around unconnected 2 (no edges touch 2).
        g.add_edge(1, Direction::E, 3);
        g.add_edge(3, Direction::W, 1);
        let m = build_model(&g, MAIN_LAYER);
        let (a, b, c) = (find(&m, 1), find(&m, 2), find(&m, 3));
        let gap_ab = (b.center.0 - b.half.0) - (a.center.0 + a.half.0);
        let gap_bc = (c.center.0 - c.half.0) - (b.center.0 + b.half.0);
        assert!(gap_ab >= UNCONNECTED_MARGIN - EPS, "1|2 unconnected: gap {gap_ab}");
        assert!(gap_bc >= UNCONNECTED_MARGIN - EPS, "2|3 unconnected: gap {gap_bc}");
    }

    #[test]
    fn build_is_deterministic() {
        let g = block_graph();
        let a = build_model(&g, MAIN_LAYER);
        let b = build_model(&g, MAIN_LAYER);
        assert_eq!(a, b, "same graph must yield the identical model");
    }

    #[test]
    fn features_are_anchored_inside_their_room() {
        let mut g = block_graph();
        g.add_edge(1, Direction::Up, 2);
        g.add_edge(1, Direction::In, 3);
        g.add_edge(4, Direction::Down, 7);
        g.add_edge(4, Direction::Out, 5);
        let m = build_model(&g, MAIN_LAYER);
        let kinds: Vec<(RoomId, FeatureKind)> =
            m.features.iter().map(|f| (f.room, f.kind)).collect();
        assert_eq!(
            kinds,
            vec![
                (1, FeatureKind::StairsUp),
                (1, FeatureKind::PortalIn),
                (4, FeatureKind::StairsDown),
                (4, FeatureKind::PortalOut),
            ]
        );
        for f in &m.features {
            let r = find(&m, f.room);
            assert!(
                (f.at.0 - r.center.0).abs() <= r.half.0 + EPS
                    && (f.at.1 - r.center.1).abs() <= r.half.1 + EPS,
                "feature {:?} at {:?} outside room {} rect (center {:?} half {:?})",
                f.kind,
                f.at,
                f.room,
                r.center,
                r.half
            );
        }
    }

    #[test]
    fn paths_are_empty_and_rooms_sorted_by_id() {
        let m = build_model(&block_graph(), MAIN_LAYER);
        assert!(m.paths.is_empty(), "V1 builds no paths");
        let ids: Vec<RoomId> = m.rooms.iter().map(|r| r.id).collect();
        let mut sorted = ids.clone();
        sorted.sort_unstable();
        assert_eq!(ids, sorted);
        assert_eq!(ids.len(), 9);
    }

    #[test]
    fn empty_layer_yields_empty_model() {
        let g = MapGraph::new();
        assert_eq!(build_model(&g, MAIN_LAYER), MapModel::default());
    }
}
