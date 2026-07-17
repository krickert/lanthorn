//! Districts (map-of-maps): partition a layer's sub-graph into independently
//! buildable districts, detect maze-like clusters, and compose per-district
//! models into one global arrangement.
//!
//! [`partition`] finds connected components over the *planar* (compass) edges
//! of a layer, then classifies each component: a component is a [`DistrictKind::Maze`]
//! when it is large and dense/tangled enough (see [`is_maze`] thresholds) —
//! everything else is [`DistrictKind::Standard`]. District identity is the
//! lowest member `RoomId`, so membership and ids are stable as the map grows:
//! adding a room can only absorb it into the component it connects to.
//!
//! [`compose`] lays the districts out as super-nodes with the same
//! rectangle-aware constrained stress pass `model::build_model` uses for rooms
//! (`layout::vpsc` + `layout::stress`): half-extents are each district's model
//! bounding-box half plus [`DISTRICT_MARGIN`], seeds are the mean member-room
//! position (so the global arrangement follows the underlying layout), and
//! inter-district edges pull like connections. Maze districts additionally get
//! a chamber outline polygon. Deterministic throughout: BTree iteration, fixed
//! iteration count, no RNG.

use std::collections::{BTreeMap, BTreeSet};

use crate::direction::grid_offset;
use crate::graph::{MapGraph, RoomId};
use crate::layer::LayerId;
use crate::layout::vpsc::Constraint;
use crate::layout::{edge_is_satisfied, stress};
use crate::model::{MapModel, Vec2};

/// How a district is rendered / abstracted.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DistrictKind {
    /// Ordinary district: rooms and corridors drawn in full.
    Standard,
    /// Dense/tangled cluster: rendered as one chamber outline at low zoom.
    Maze,
}

/// One district: a connected component of a layer's planar sub-graph.
/// `id` is the lowest member [`RoomId`] (stable cluster identity); `rooms` is
/// sorted ascending.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct District {
    pub id: u16,
    pub kind: DistrictKind,
    pub rooms: Vec<RoomId>,
}

// ── maze heuristic thresholds ─────────────────────────────────────────────────

/// A component smaller than this is never a maze.
pub const MAZE_MIN_ROOMS: usize = 5;
/// Planar-edge (unordered connected pairs) to room ratio at/above which a
/// component reads as a maze (a grid tops out at 2·(n−√n)/n < 2; a 3×3 grid is
/// ≈1.33, K5 is 2.0).
pub const MAZE_EDGE_RATIO: f32 = 1.6;
/// Fraction of compass connections that are distorted/unsatisfied at/above
/// which a component reads as a maze even when sparse.
pub const MAZE_BAD_EDGE_FRACTION: f32 = 0.4;

/// Margin added to each district super-node half-extent during composition.
pub const DISTRICT_MARGIN: f32 = 0.5;
/// How far a maze chamber outline stands off its member room rects.
pub const CHAMBER_INFLATE: f32 = 0.3;

/// Fixed SMACOF iterations for the composition pass (mirrors `model::ITERS`).
const ITERS: usize = 40;

/// Pure maze-scoring heuristic over component statistics. `planar_pairs` is
/// the number of unordered room pairs joined by at least one compass edge;
/// `compass_edges` / `bad_edges` count directed compass connections, `bad`
/// being those distorted or geometrically unsatisfied. A cluster qualifies as
/// a maze when it has at least [`MAZE_MIN_ROOMS`] rooms and is either dense
/// (pair/room ratio ≥ [`MAZE_EDGE_RATIO`], i.e. decidedly non-planar-grid-like)
/// or tangled (≥ [`MAZE_BAD_EDGE_FRACTION`] of its compass edges bad).
pub fn is_maze(room_count: usize, planar_pairs: usize, compass_edges: usize, bad_edges: usize) -> bool {
    if room_count < MAZE_MIN_ROOMS {
        return false;
    }
    if planar_pairs as f32 / room_count as f32 >= MAZE_EDGE_RATIO {
        return true;
    }
    compass_edges > 0 && bad_edges as f32 / compass_edges as f32 >= MAZE_BAD_EDGE_FRACTION
}

/// Partition `layer`'s sub-graph into districts: connected components over the
/// planar (compass-direction) edges, each classified via [`is_maze`]. Rooms
/// with no planar edges become singleton Standard districts. Districts are
/// returned in ascending-id order; ids are the lowest member room id, so an
/// existing district keeps its id and membership when new rooms are appended
/// elsewhere (it only ever absorbs rooms that connect to it).
pub fn partition(graph: &MapGraph, layer: LayerId) -> Vec<District> {
    let sub = graph.layer_subgraph(layer);
    let mut adj: BTreeMap<RoomId, BTreeSet<RoomId>> =
        sub.rooms().map(|r| (r.id, BTreeSet::new())).collect();
    for c in sub.connections() {
        if grid_offset(c.dir).is_none() || c.origin == c.dest {
            continue;
        }
        if adj.contains_key(&c.origin) && adj.contains_key(&c.dest) {
            adj.get_mut(&c.origin).unwrap().insert(c.dest);
            adj.get_mut(&c.dest).unwrap().insert(c.origin);
        }
    }

    // BFS components in ascending room-id order → components arrive sorted by
    // their lowest member, which is also each district's id.
    let ids: Vec<RoomId> = adj.keys().copied().collect();
    let mut component: BTreeMap<RoomId, usize> = BTreeMap::new();
    let mut members: Vec<BTreeSet<RoomId>> = Vec::new();
    for &start in &ids {
        if component.contains_key(&start) {
            continue;
        }
        let idx = members.len();
        let mut comp = BTreeSet::new();
        let mut queue = vec![start];
        component.insert(start, idx);
        comp.insert(start);
        while let Some(id) = queue.pop() {
            for &next in &adj[&id] {
                if component.insert(next, idx).is_none() {
                    comp.insert(next);
                    queue.push(next);
                }
            }
        }
        members.push(comp);
    }

    // One pass over the connections accumulates per-component maze statistics.
    let mut pairs: Vec<BTreeSet<(RoomId, RoomId)>> = vec![BTreeSet::new(); members.len()];
    let mut compass = vec![0usize; members.len()];
    let mut bad = vec![0usize; members.len()];
    for c in sub.connections() {
        if grid_offset(c.dir).is_none() || c.origin == c.dest {
            continue;
        }
        let (Some(&i), Some(&j)) = (component.get(&c.origin), component.get(&c.dest)) else {
            continue;
        };
        if i != j {
            continue; // defensive: planar edges always join one component
        }
        pairs[i].insert((c.origin.min(c.dest), c.origin.max(c.dest)));
        compass[i] += 1;
        if c.distorted || !edge_is_satisfied(&sub, c) {
            bad[i] += 1;
        }
    }

    members
        .into_iter()
        .enumerate()
        .map(|(i, comp)| {
            let kind = if is_maze(comp.len(), pairs[i].len(), compass[i], bad[i]) {
                DistrictKind::Maze
            } else {
                DistrictKind::Standard
            };
            let rooms: Vec<RoomId> = comp.into_iter().collect();
            District { id: rooms[0], kind, rooms }
        })
        .collect()
}

// ── composition ───────────────────────────────────────────────────────────────

/// The global arrangement of composed districts. `offsets` maps district id →
/// translation to apply to that district's local model coordinates; `chamber`
/// carries, for each Maze district, its outline polygon in *global* (already
/// translated) coordinates. Both are in district input order.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Composition {
    pub offsets: Vec<(u16, (f32, f32))>,
    pub chamber: Vec<(u16, Vec<(f32, f32)>)>,
}

/// Local-space bounding box `(lo, hi)` of a district's model rooms; a
/// degenerate point box at the origin when the model is empty.
fn model_bbox(m: &MapModel) -> (Vec2, Vec2) {
    let mut lo = (f32::INFINITY, f32::INFINITY);
    let mut hi = (f32::NEG_INFINITY, f32::NEG_INFINITY);
    for r in &m.rooms {
        lo.0 = lo.0.min(r.center.0 - r.half.0);
        lo.1 = lo.1.min(r.center.1 - r.half.1);
        hi.0 = hi.0.max(r.center.0 + r.half.0);
        hi.1 = hi.1.max(r.center.1 + r.half.1);
    }
    if m.rooms.is_empty() {
        ((0.0, 0.0), (0.0, 0.0))
    } else {
        (lo, hi)
    }
}

/// Lay the district super-nodes out and translate each district's local
/// coordinates into global space. Mirrors `model::build_model`'s refinement:
/// every district pair gets per-axis VPSC separation constraints (gap = sum of
/// half-extents, which already include [`DISTRICT_MARGIN`] each) ordered by the
/// seed positions, and inter-district edges become unit-distance stress
/// attractions. Seeds are each district's mean member-room center — the
/// continuous analogue of the mean cell position, since model centers are
/// seeded from cells — so the global arrangement follows the underlying
/// layout. Deterministic for identical inputs.
pub fn compose(districts: &[(District, MapModel)], inter_edges: &[(RoomId, RoomId)]) -> Composition {
    let n = districts.len();
    if n == 0 {
        return Composition::default();
    }

    let boxes: Vec<(Vec2, Vec2)> = districts.iter().map(|(_, m)| model_bbox(m)).collect();
    let half: Vec<(f64, f64)> = boxes
        .iter()
        .map(|(lo, hi)| {
            (
                f64::from((hi.0 - lo.0) / 2.0 + DISTRICT_MARGIN),
                f64::from((hi.1 - lo.1) / 2.0 + DISTRICT_MARGIN),
            )
        })
        .collect();
    let seed: Vec<(f64, f64)> = districts
        .iter()
        .map(|(_, m)| {
            if m.rooms.is_empty() {
                return (0.0, 0.0);
            }
            let (sx, sy) = m.rooms.iter().fold((0.0f64, 0.0f64), |(sx, sy), r| {
                (sx + f64::from(r.center.0), sy + f64::from(r.center.1))
            });
            let k = m.rooms.len() as f64;
            (sx / k, sy / k)
        })
        .collect();

    // Separation constraints per pair, ordered by seed position per axis (the
    // super-node analogue of cell order); a full tie forces x order by index.
    let mut xc: Vec<Constraint> = Vec::new();
    let mut yc: Vec<Constraint> = Vec::new();
    for i in 0..n {
        for j in (i + 1)..n {
            let xgap = half[i].0 + half[j].0;
            let ygap = half[i].1 + half[j].1;
            let ox = seed[i].0.total_cmp(&seed[j].0);
            let oy = seed[i].1.total_cmp(&seed[j].1);
            match ox {
                std::cmp::Ordering::Less => xc.push(Constraint { left: i, right: j, gap: xgap }),
                std::cmp::Ordering::Greater => xc.push(Constraint { left: j, right: i, gap: xgap }),
                std::cmp::Ordering::Equal => {}
            }
            match oy {
                std::cmp::Ordering::Less => yc.push(Constraint { left: i, right: j, gap: ygap }),
                std::cmp::Ordering::Greater => yc.push(Constraint { left: j, right: i, gap: ygap }),
                std::cmp::Ordering::Equal => {}
            }
            if ox == std::cmp::Ordering::Equal && oy == std::cmp::Ordering::Equal {
                xc.push(Constraint { left: i, right: j, gap: xgap });
            }
        }
    }

    // Inter-district edges pull like connections (hop distance 1.0; the VPSC
    // projection keeps the separation gaps feasible each iteration).
    let owner: BTreeMap<RoomId, usize> = districts
        .iter()
        .enumerate()
        .flat_map(|(i, (d, _))| d.rooms.iter().map(move |&r| (r, i)))
        .collect();
    let mut adj: Vec<Vec<usize>> = vec![Vec::new(); n];
    for &(a, b) in inter_edges {
        let (Some(&i), Some(&j)) = (owner.get(&a), owner.get(&b)) else { continue };
        if i != j {
            adj[i].push(j);
            adj[j].push(i);
        }
    }
    let dist = stress::all_pairs_dist(n, &adj);
    let pos = stress::stress_layout(n, &dist, &xc, &yc, &seed, ITERS);

    let offsets: Vec<(u16, (f32, f32))> = districts
        .iter()
        .enumerate()
        .map(|(i, (d, _))| {
            let (lo, hi) = boxes[i];
            let center = ((lo.0 + hi.0) / 2.0, (lo.1 + hi.1) / 2.0);
            (d.id, (pos[i].0 as f32 - center.0, pos[i].1 as f32 - center.1))
        })
        .collect();

    let chamber: Vec<(u16, Vec<(f32, f32)>)> = districts
        .iter()
        .enumerate()
        .filter(|(_, (d, m))| d.kind == DistrictKind::Maze && !m.rooms.is_empty())
        .map(|(i, (d, m))| {
            let rects: Vec<(Vec2, Vec2)> = m.rooms.iter().map(|r| (r.center, r.half)).collect();
            (d.id, chamber_outline(&rects, offsets[i].1))
        })
        .collect();

    Composition { offsets, chamber }
}

// ── maze chamber outline ──────────────────────────────────────────────────────

/// Minimum notch depth worth cutting (below this the corner stays square).
const NOTCH_EPS: f32 = 0.05;

/// Chamber outline for a Maze district, in global coordinates (`offset`
/// already applied): the union bounding box of the member room rects (given as
/// `(center, half)` pairs) inflated by [`CHAMBER_INFLATE`], with an L-notch
/// cut at each bbox corner back to the inflated rect nearest that corner.
///
/// Documented choice: this is the "bounding-box-with-notches" pass rather than
/// a grid-scan boundary trace — the union of inflated rects is not guaranteed
/// connected, so a full trace needs multi-loop stitching that is
/// disproportionate here. Each corner's notch is the largest empty axis-
/// aligned rectangle anchored at that bbox corner: candidate widths come from
/// the rects' near edges, the height is the minimum clearance of the rects
/// intruding into that width, so a notch can never cut a member rect. Depths
/// are capped just below half the bbox extent, keeping the four notches
/// pairwise disjoint (the polygon stays simple). Emitted clockwise (screen
/// coords, +y south) starting at the top-left region.
fn chamber_outline(rooms: &[(Vec2, Vec2)], offset: Vec2) -> Vec<Vec2> {
    // Inflated member rects as (left, top, right, bottom).
    let rects: Vec<(f32, f32, f32, f32)> = rooms
        .iter()
        .map(|(center, half)| {
            (
                center.0 - half.0 - CHAMBER_INFLATE,
                center.1 - half.1 - CHAMBER_INFLATE,
                center.0 + half.0 + CHAMBER_INFLATE,
                center.1 + half.1 + CHAMBER_INFLATE,
            )
        })
        .collect();
    let l = rects.iter().map(|r| r.0).fold(f32::INFINITY, f32::min);
    let t = rects.iter().map(|r| r.1).fold(f32::INFINITY, f32::min);
    let r = rects.iter().map(|r| r.2).fold(f32::NEG_INFINITY, f32::max);
    let b = rects.iter().map(|r| r.3).fold(f32::NEG_INFINITY, f32::max);

    // Notch depth (nx, ny) at a corner: the largest empty rectangle anchored
    // there, in a corner-local frame where each rect is (dx, dy) = distance
    // from the corner to the rect's near edges. A notch of width nx is
    // intruded on by every rect with dx < nx; its height is their minimum dy.
    // Candidate widths are the rects' dx values; depths are capped just below
    // half the bbox extent so opposing notches never meet. Deterministic:
    // strict > keeps the first maximum in room order.
    let nxmax = ((r - l) / 2.0 - NOTCH_EPS).max(0.0);
    let nymax = ((b - t) / 2.0 - NOTCH_EPS).max(0.0);
    let notch = |ds: Vec<(f32, f32)>| -> (f32, f32) {
        let mut best = (0.0f32, 0.0f32);
        for &(dxk, _) in &ds {
            let nx = dxk.min(nxmax);
            if nx <= NOTCH_EPS {
                continue;
            }
            let mut ny = nymax;
            for &(dxj, dyj) in &ds {
                if dxj < nx {
                    ny = ny.min(dyj);
                }
            }
            if ny > NOTCH_EPS && nx * ny > best.0 * best.1 {
                best = (nx, ny);
            }
        }
        best
    };
    let tl = notch(rects.iter().map(|c| (c.0 - l, c.1 - t)).collect());
    let tr = notch(rects.iter().map(|c| (r - c.2, c.1 - t)).collect());
    let br = notch(rects.iter().map(|c| (r - c.2, b - c.3)).collect());
    let bl = notch(rects.iter().map(|c| (c.0 - l, b - c.3)).collect());

    let mut pts: Vec<Vec2> = Vec::new();
    let mut corner = |sq: Vec2, inner: Vec2, depth: (f32, f32), order_x_first: bool| {
        if depth.0 > NOTCH_EPS && depth.1 > NOTCH_EPS {
            // L-notch: entry point, inner corner, exit point (clockwise).
            if order_x_first {
                pts.push((sq.0, inner.1));
                pts.push(inner);
                pts.push((inner.0, sq.1));
            } else {
                pts.push((inner.0, sq.1));
                pts.push(inner);
                pts.push((sq.0, inner.1));
            }
        } else {
            pts.push(sq);
        }
    };
    corner((l, t), (l + tl.0, t + tl.1), tl, true);
    corner((r, t), (r - tr.0, t + tr.1), tr, false);
    corner((r, b), (r - br.0, b - br.1), br, true);
    corner((l, b), (l + bl.0, b - bl.1), bl, false);

    pts.iter().map(|p| (p.0 + offset.0, p.1 + offset.1)).collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::direction::Direction;
    use crate::layer::MAIN_LAYER;
    use crate::model::build_model;

    const EPS: f32 = 1e-3;

    fn room_at(g: &mut MapGraph, id: RoomId, pos: (i32, i32)) {
        g.upsert_room(id, format!("R{id}"));
        g.set_pos(id, pos);
    }

    /// 3×3 grid rooted at `base` (ids base..base+8), cells offset by `at`,
    /// single-direction compass edges consistent with the cells (all
    /// satisfied): 12 unordered pairs / 9 rooms ≈ 1.33 — planar.
    fn grid3(g: &mut MapGraph, base: RoomId, at: (i32, i32)) {
        for i in 0u16..9 {
            room_at(g, base + i, (at.0 + i32::from(i % 3), at.1 + i32::from(i / 3)));
        }
        for row in 0u16..3 {
            for col in 0u16..2 {
                g.add_edge(base + row * 3 + col, Direction::E, base + row * 3 + col + 1);
            }
        }
        for row in 0u16..2 {
            for col in 0u16..3 {
                g.add_edge(base + row * 3 + col, Direction::S, base + (row + 1) * 3 + col);
            }
        }
    }

    /// K5 on ids base..base+4: every pair connected (10 pairs / 5 rooms = 2.0),
    /// compass directions mostly inconsistent with the cells, several marked
    /// distorted — a dense non-planar tangle.
    fn k5(g: &mut MapGraph, base: RoomId, at: (i32, i32)) {
        let cells = [(0, 0), (1, 0), (2, 0), (0, 1), (1, 1)];
        for (i, c) in cells.iter().enumerate() {
            room_at(g, base + i as u16, (at.0 + c.0, at.1 + c.1));
        }
        let dirs = [
            (0u16, Direction::E, 1u16),
            (0, Direction::N, 2),
            (0, Direction::S, 3),
            (0, Direction::SE, 4),
            (1, Direction::N, 2),
            (1, Direction::W, 3),
            (1, Direction::S, 4),
            (2, Direction::SW, 3),
            (2, Direction::S, 4),
            (3, Direction::E, 4),
        ];
        let first = g.connections().len();
        for &(a, d, b) in &dirs {
            g.add_edge(base + a, d, base + b);
        }
        for i in 0..4 {
            g.set_conn_distorted(first + i, true);
        }
    }

    /// A fresh graph holding only `d`'s rooms (positions copied) and the edges
    /// internal to it — the per-district build input integration will use.
    fn district_graph(g: &MapGraph, d: &District) -> MapGraph {
        let set: BTreeSet<RoomId> = d.rooms.iter().copied().collect();
        let mut out = MapGraph::new();
        for &id in &d.rooms {
            let r = g.room(id).unwrap();
            out.upsert_room(id, r.name.clone());
            if let Some(p) = r.pos {
                out.set_pos(id, p);
            }
        }
        for c in g.connections() {
            if set.contains(&c.origin) && set.contains(&c.dest) {
                out.add_edge(c.origin, c.dir, c.dest);
            }
        }
        out
    }

    fn built(g: &MapGraph, districts: Vec<District>) -> Vec<(District, MapModel)> {
        districts
            .into_iter()
            .map(|d| {
                let m = build_model(&district_graph(g, &d), MAIN_LAYER);
                (d, m)
            })
            .collect()
    }

    /// Global-space bbox of a composed district.
    fn global_bbox(m: &MapModel, off: (f32, f32)) -> ((f32, f32), (f32, f32)) {
        let (lo, hi) = model_bbox(m);
        ((lo.0 + off.0, lo.1 + off.1), (hi.0 + off.0, hi.1 + off.1))
    }

    fn point_in_poly(p: (f32, f32), poly: &[(f32, f32)]) -> bool {
        let mut inside = false;
        let mut j = poly.len() - 1;
        for i in 0..poly.len() {
            let (xi, yi) = poly[i];
            let (xj, yj) = poly[j];
            if (yi > p.1) != (yj > p.1) && p.0 < (xj - xi) * (p.1 - yi) / (yj - yi) + xi {
                inside = !inside;
            }
            j = i;
        }
        inside
    }

    #[test]
    fn maze_heuristic_thresholds() {
        // Too small, no matter how dense.
        assert!(!is_maze(4, 8, 8, 8));
        // Dense: ratio 8/5 = 1.6 hits the threshold; 7/5 = 1.4 does not.
        assert!(is_maze(5, 8, 8, 0));
        assert!(!is_maze(5, 7, 10, 3));
        // Tangled: 4/10 = 40% bad hits; 3/10 = 30% does not.
        assert!(is_maze(5, 7, 10, 4));
        // No compass edges at all → never divides by zero, never a maze.
        assert!(!is_maze(6, 0, 0, 0));
    }

    #[test]
    fn grid_is_a_single_standard_district() {
        let mut g = MapGraph::new();
        grid3(&mut g, 1, (0, 0));
        let d = partition(&g, MAIN_LAYER);
        assert_eq!(d.len(), 1);
        assert_eq!(d[0].id, 1);
        assert_eq!(d[0].kind, DistrictKind::Standard);
        assert_eq!(d[0].rooms, (1u16..=9).collect::<Vec<_>>());
    }

    #[test]
    fn dense_nonplanar_cluster_is_a_maze() {
        let mut g = MapGraph::new();
        k5(&mut g, 1, (0, 0));
        let d = partition(&g, MAIN_LAYER);
        assert_eq!(d.len(), 1);
        assert_eq!(d[0].id, 1);
        assert_eq!(d[0].kind, DistrictKind::Maze, "K5-density must classify as Maze");
    }

    #[test]
    fn distorted_fraction_alone_triggers_maze() {
        // A sparse 5-room path (4 pairs / 5 rooms = 0.8) with 2 of 4 edges
        // distorted (50% ≥ 40%) is a maze; with 1 of 4 (25%) it is not.
        for (n_distorted, expect) in [(2usize, DistrictKind::Maze), (1, DistrictKind::Standard)] {
            let mut g = MapGraph::new();
            for i in 0u16..5 {
                room_at(&mut g, 1 + i, (i32::from(i), 0));
            }
            for i in 0u16..4 {
                g.add_edge(1 + i, Direction::E, 2 + i);
            }
            for i in 0..n_distorted {
                g.set_conn_distorted(i, true);
            }
            let d = partition(&g, MAIN_LAYER);
            assert_eq!(d.len(), 1);
            assert_eq!(d[0].kind, expect, "{n_distorted} of 4 distorted");
        }
    }

    #[test]
    fn isolated_room_is_a_singleton_standard_district() {
        let mut g = MapGraph::new();
        grid3(&mut g, 1, (0, 0));
        room_at(&mut g, 30, (10, 10));
        // A non-planar (Up) edge must not merge components.
        g.add_edge(30, Direction::Up, 1);
        let d = partition(&g, MAIN_LAYER);
        assert_eq!(d.len(), 2);
        assert_eq!((d[1].id, d[1].kind, d[1].rooms.clone()), (30, DistrictKind::Standard, vec![30]));
    }

    #[test]
    fn partition_only_sees_the_requested_layer() {
        let mut g = MapGraph::new();
        grid3(&mut g, 1, (0, 0));
        let attic = g.new_layer(None, "Attic".into());
        room_at(&mut g, 40, (0, 0));
        g.set_room_layer(40, attic);
        let main = partition(&g, MAIN_LAYER);
        assert_eq!(main.len(), 1);
        assert!(!main[0].rooms.contains(&40));
        let up = partition(&g, attic);
        assert_eq!(up.len(), 1);
        assert_eq!(up[0].rooms, vec![40]);
    }

    #[test]
    fn two_components_compose_without_overlap() {
        let mut g = MapGraph::new();
        grid3(&mut g, 1, (0, 0));
        grid3(&mut g, 11, (10, 0));
        let districts = partition(&g, MAIN_LAYER);
        assert_eq!(districts.len(), 2);
        assert!(districts.iter().all(|d| d.kind == DistrictKind::Standard));
        let built = built(&g, districts);
        let comp = compose(&built, &[(3, 11)]);
        assert_eq!(comp.offsets.len(), 2);
        assert!(comp.chamber.is_empty(), "no maze → no chamber outlines");
        let (alo, ahi) = global_bbox(&built[0].1, comp.offsets[0].1);
        let (blo, bhi) = global_bbox(&built[1].1, comp.offsets[1].1);
        let x_sep = blo.0 + EPS >= ahi.0 || alo.0 + EPS >= bhi.0;
        let y_sep = blo.1 + EPS >= ahi.1 || alo.1 + EPS >= bhi.1;
        assert!(x_sep || y_sep, "district bboxes overlap: a=({alo:?},{ahi:?}) b=({blo:?},{bhi:?})");
        // The margin-inflated super-nodes guarantee at least 2·DISTRICT_MARGIN
        // of clearance on the separating axis.
        if x_sep {
            let gap = (blo.0 - ahi.0).max(alo.0 - bhi.0);
            assert!(gap >= 2.0 * DISTRICT_MARGIN - EPS, "x clearance {gap}");
        }
    }

    #[test]
    fn composition_follows_the_underlying_layout_order() {
        // Grid A sits west of grid B in cell space; the composed arrangement
        // must preserve that east-west order (seeds come from the layout).
        let mut g = MapGraph::new();
        grid3(&mut g, 1, (0, 0));
        grid3(&mut g, 11, (10, 0));
        let built = built(&g, partition(&g, MAIN_LAYER));
        let comp = compose(&built, &[]);
        let (alo, ahi) = global_bbox(&built[0].1, comp.offsets[0].1);
        let (blo, _) = global_bbox(&built[1].1, comp.offsets[1].1);
        assert!(
            ahi.0 <= blo.0 + EPS,
            "district 1 must stay west of district 11: a=({alo:?},{ahi:?}) b_lo={blo:?}"
        );
    }

    #[test]
    fn district_ids_and_membership_stable_under_room_addition() {
        let mut before = MapGraph::new();
        grid3(&mut before, 1, (0, 0));
        k5(&mut before, 11, (10, 0));
        let a = partition(&before, MAIN_LAYER);

        // Snapshot two: one new room joins the grid district from the east.
        let mut after = before.clone();
        room_at(&mut after, 20, (3, 0));
        after.add_edge(3, Direction::E, 20);
        let b = partition(&after, MAIN_LAYER);

        let ids = |ds: &[District]| ds.iter().map(|d| d.id).collect::<Vec<_>>();
        assert_eq!(ids(&a), vec![1, 11]);
        assert_eq!(ids(&b), vec![1, 11], "existing district ids must survive the addition");
        let rooms = |ds: &[District], id: u16| {
            ds.iter().find(|d| d.id == id).unwrap().rooms.clone()
        };
        let mut grown = rooms(&a, 1);
        grown.push(20);
        assert_eq!(rooms(&b, 1), grown, "the grid district absorbs exactly the new room");
        assert_eq!(rooms(&b, 11), rooms(&a, 11), "the untouched maze district is unchanged");
        let kind = |ds: &[District], id: u16| ds.iter().find(|d| d.id == id).unwrap().kind;
        assert_eq!(kind(&a, 11), kind(&b, 11));
    }

    #[test]
    fn compose_is_deterministic() {
        let mut g = MapGraph::new();
        grid3(&mut g, 1, (0, 0));
        k5(&mut g, 11, (10, 0));
        let built = built(&g, partition(&g, MAIN_LAYER));
        let edges = [(3u16, 11u16), (9, 13)];
        let x = compose(&built, &edges);
        let y = compose(&built, &edges);
        assert_eq!(x, y, "same inputs must yield the identical composition");
    }

    #[test]
    fn maze_chamber_outline_contains_all_member_rects() {
        let mut g = MapGraph::new();
        grid3(&mut g, 1, (0, 0));
        k5(&mut g, 11, (10, 0));
        let built = built(&g, partition(&g, MAIN_LAYER));
        let comp = compose(&built, &[(3, 11)]);
        assert_eq!(comp.chamber.len(), 1, "exactly the maze district gets a chamber");
        let (id, poly) = &comp.chamber[0];
        assert_eq!(*id, 11);
        assert!(poly.len() >= 4, "outline must be a polygon, got {} points", poly.len());
        let off = comp.offsets.iter().find(|(d, _)| d == id).unwrap().1;
        let (_, maze_model) = built.iter().find(|(d, _)| d.id == *id).unwrap();
        for r in &maze_model.rooms {
            for (sx, sy) in [(-1.0f32, -1.0f32), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)] {
                let p = (
                    r.center.0 + sx * r.half.0 + off.0,
                    r.center.1 + sy * r.half.1 + off.1,
                );
                assert!(
                    point_in_poly(p, poly),
                    "room {} corner {p:?} escapes the chamber outline {poly:?}",
                    r.id
                );
            }
        }
    }

    #[test]
    fn single_room_chamber_is_its_inflated_rect() {
        let rooms = [((2.0, 3.0), (0.5, 0.4))];
        let poly = chamber_outline(&rooms, (1.0, -1.0));
        assert_eq!(
            poly,
            vec![
                (2.0 - 0.5 - CHAMBER_INFLATE + 1.0, 3.0 - 0.4 - CHAMBER_INFLATE - 1.0),
                (2.0 + 0.5 + CHAMBER_INFLATE + 1.0, 3.0 - 0.4 - CHAMBER_INFLATE - 1.0),
                (2.0 + 0.5 + CHAMBER_INFLATE + 1.0, 3.0 + 0.4 + CHAMBER_INFLATE - 1.0),
                (2.0 - 0.5 - CHAMBER_INFLATE + 1.0, 3.0 + 0.4 + CHAMBER_INFLATE - 1.0),
            ],
            "no notches on a single rect: plain inflated bbox, translated"
        );
    }

    #[test]
    fn chamber_outline_notches_an_empty_corner() {
        // Two rooms on a diagonal: the NE and SW bbox corners are empty and
        // must be notched (outline grows beyond 4 points), while both rects
        // stay inside.
        let rooms = [((0.0, 0.0), (0.5, 0.4)), ((4.0, 3.0), (0.5, 0.4))];
        let poly = chamber_outline(&rooms, (0.0, 0.0));
        assert!(poly.len() > 4, "empty corners must be notched, got {poly:?}");
        for ((cx, cy), (hx, hy)) in &rooms {
            for (sx, sy) in [(-1.0f32, -1.0f32), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)] {
                let p = (cx + sx * hx, cy + sy * hy);
                assert!(point_in_poly(p, &poly), "corner {p:?} outside {poly:?}");
            }
        }
    }

    #[test]
    fn empty_inputs_yield_empty_outputs() {
        assert!(partition(&MapGraph::new(), MAIN_LAYER).is_empty());
        assert_eq!(compose(&[], &[]), Composition::default());
    }
}
