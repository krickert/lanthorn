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
//! with margin 0 for ANY planar-connected pair — a cardinal pair may abut
//! (shared wall after projection), a diagonal pair may touch corners — and a
//! minimum margin otherwise. Stress ideals are size-aware (direct edges target
//! their abutment distance, multi-hop targets scale by the mean room extent),
//! and a centroid-pull compaction pass squeezes out remaining slack after
//! refinement (see [`compact_toward_centroid`]). Separation constraints
//! follow cell order, so the relative order the cell layout established (N
//! stays above, W stays left) is preserved by construction. Deterministic: BTree
//! iteration, FNV-1a id hashing only, fixed iteration count.
//!
//! V1.5 adds 2.5D z-levels: every room gets a `level` derived by longest-path
//! layering over the Up/Down subgraph (Up raises the level — see
//! [`room_levels`]), the continuous x/y refinement runs PER LEVEL (rooms on
//! different levels never constrain each other's x/y and may overlap — they are
//! different floors), and a stairwell-coherence nudge pulls each Up/Down edge's
//! endpoints toward the same x/y across levels (see [`stairwell_coherence`]).

use std::collections::{BTreeMap, BTreeSet, VecDeque};

use crate::direction::{grid_offset, opposite, Direction};
use crate::graph::{MapGraph, RoomId};
use crate::layer::LayerId;
use crate::layout::stress;
use crate::layout::vpsc::{self, Constraint};
use crate::router::Side;
use crate::tiles::FeatureKind;

/// A point / extent in model space. `+x` is east, `+y` is south (cell layout
/// convention); units are isotropic — projection handles terminal aspect.
pub type Vec2 = (f32, f32);

/// A room as a real-valued rectangle: `center ± half` spans it. `level` is the
/// 2.5D floor index (Up increases it; relative within a layer, not an absolute
/// storey number — each Up/Down component's lowest room sits at 0).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ModelRoom {
    pub id: RoomId,
    pub center: Vec2,
    pub half: Vec2,
    pub level: i32,
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
    /// Up/Down edges (as `(origin, dest)`) whose level ordering was dropped
    /// because it would close a cycle in the Up/Down subgraph. The level
    /// assignment distorts these connections' vertical semantics in the MODEL
    /// only; the input graph is never mutated.
    pub distorted_levels: Vec<(RoomId, RoomId)>,
    /// Rooms whose small vertical cluster was collapsed into its stair anchor's
    /// 2D plane (see [`collapse_small_clusters`]): they render on the anchor's
    /// level, and projection/rendering may treat their stair link specially
    /// (dashed connector + stairs glyphs). Sorted by id.
    pub collapsed: Vec<RoomId>,
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
/// Minimum per-axis gap between rooms with no planar (compass) connection.
/// Connected pairs — cardinal AND diagonal — get margin 0: a cardinal pair may
/// abut into a shared wall, a diagonal pair may close up to a corner touch.
pub const UNCONNECTED_MARGIN: f32 = 0.35;

/// Hash-bit size jitter amplitudes (kept below the door/clamp headroom).
const JITTER_W: f32 = 0.2;
const JITTER_H: f32 = 0.15;
/// Seed pitch per layout cell (x wider: widths run larger than heights).
const SEED_PITCH_X: f64 = 3.0;
const SEED_PITCH_Y: f64 = 2.0;
/// Fixed SMACOF iterations for the refinement pass (determinism + bounded cost).
const ITERS: usize = 40;
/// Slack added above the hard abutment distance in a direct edge's stress
/// ideal, so the target sits just OUTSIDE the VPSC boundary instead of pulling
/// against it every iteration; the compaction pass closes the slack afterward.
const IDEAL_SLACK: f64 = 0.1;
/// Fixed centroid-pull compaction rounds (see [`compact_toward_centroid`]).
const COMPACT_ITERS: usize = 4;
/// Fraction of the way each compaction round pulls a room toward the centroid
/// before re-projecting onto the separation constraints.
const COMPACT_STEP: f64 = 0.5;
/// Fixed nudge-and-reproject rounds for stairwell coherence (see
/// [`stairwell_coherence`]); geometric convergence makes a handful plenty.
const COHERENCE_ITERS: usize = 8;
/// Maximum room count for a vertical cluster to collapse into its stair
/// anchor's plane (see [`collapse_small_clusters`]).
pub const COLLAPSE_THRESHOLD: usize = 3;
/// Preferred soft y-gap between a collapsed room and its stair anchor beyond
/// their half-heights (model units); hard separation constraints still win.
const COLLAPSE_GAP: f64 = 0.2;

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

/// Depth-first reachability over an adjacency list (mirrors
/// `layout::sort::reaches`): is `target` reachable from `from`?
fn reaches(adj: &[Vec<usize>], from: usize, target: usize) -> bool {
    let mut stack = vec![from];
    let mut seen = BTreeSet::new();
    while let Some(v) = stack.pop() {
        if v == target {
            return true;
        }
        if !seen.insert(v) {
            continue;
        }
        stack.extend(adj[v].iter().copied());
    }
    false
}

/// Derive a 2.5D level per placed room from the Up/Down subgraph.
///
/// SIGN CONVENTION: **Up increases level** — an `origin -Up-> dest` edge orders
/// `level[dest] = level[origin] + 1` (and `Down` symmetrically lowers its
/// destination). Levels are longest-path coordinates, so each Up/Down
/// component's lowest room sits at level 0; they are relative floor indices
/// within the layer, not absolute storeys.
///
/// Technique mirrors `layout::sort::layer_axis`: ordering edges are accepted in
/// connection order, an edge that would close a cycle is dropped
/// (deterministically — same input, same drop) and reported as `(origin, dest)`
/// in the second return value; levels then come from Kahn's topological sort
/// with longest-path relaxation. The input graph is never mutated — a dropped
/// edge only marks the connection's vertical semantics as distorted in the
/// model output.
///
/// Rooms not touching any Up/Down edge inherit the level of their planar
/// cluster's anchor: a multi-source BFS from all leveled rooms across planar
/// (compass) edges, in dense-index order for determinism. Rooms unreachable
/// from any leveled room get level 0.
fn room_levels(
    ids: &[RoomId],
    sub: &MapGraph,
) -> (BTreeMap<RoomId, i32>, Vec<(RoomId, RoomId)>) {
    let n = ids.len();
    let index: BTreeMap<RoomId, usize> =
        ids.iter().enumerate().map(|(i, &id)| (id, i)).collect();

    // Accept Up/Down ordering edges that don't close a cycle.
    let mut adj: Vec<Vec<usize>> = vec![Vec::new(); n];
    let mut touched = vec![false; n];
    let mut dropped: Vec<(RoomId, RoomId)> = Vec::new();
    for c in sub.connections() {
        let (lo, hi) = match c.dir {
            Direction::Up => (c.origin, c.dest),
            Direction::Down => (c.dest, c.origin),
            _ => continue,
        };
        let (Some(&l), Some(&h)) = (index.get(&lo), index.get(&hi)) else { continue };
        if l == h {
            continue;
        }
        touched[l] = true;
        touched[h] = true;
        if reaches(&adj, h, l) {
            dropped.push((c.origin, c.dest)); // would close a cycle → drop
            continue;
        }
        adj[l].push(h);
    }

    // Longest path via Kahn's topological sort + relaxation (as `layer_axis`).
    let mut in_degree = vec![0usize; n];
    for successors in &adj {
        for &w in successors {
            in_degree[w] += 1;
        }
    }
    let mut queue: VecDeque<usize> = (0..n).filter(|&v| in_degree[v] == 0).collect();
    let mut coord = vec![0_i32; n];
    while let Some(v) = queue.pop_front() {
        for &w in &adj[v] {
            if coord[v] + 1 > coord[w] {
                coord[w] = coord[v] + 1;
            }
            in_degree[w] -= 1;
            if in_degree[w] == 0 {
                queue.push_back(w);
            }
        }
    }

    // Untouched rooms inherit across planar edges from the nearest leveled room
    // (multi-source BFS, deterministic order); still-unreached rooms stay 0.
    let mut planar: Vec<Vec<usize>> = vec![Vec::new(); n];
    for c in sub.connections() {
        if grid_offset(c.dir).is_none() {
            continue;
        }
        let (Some(&a), Some(&b)) = (index.get(&c.origin), index.get(&c.dest)) else { continue };
        if a != b {
            planar[a].push(b);
            planar[b].push(a);
        }
    }
    for nb in &mut planar {
        nb.sort_unstable();
        nb.dedup();
    }
    let mut level: Vec<Option<i32>> =
        (0..n).map(|v| touched[v].then_some(coord[v])).collect();
    let mut q: VecDeque<usize> = (0..n).filter(|&v| touched[v]).collect();
    while let Some(v) = q.pop_front() {
        let lv = level[v].unwrap();
        for &w in &planar[v] {
            if level[w].is_none() {
                level[w] = Some(lv);
                q.push_back(w);
            }
        }
    }

    let levels = ids
        .iter()
        .enumerate()
        .map(|(i, &id)| (id, level[i].unwrap_or(0)))
        .collect();
    (levels, dropped)
}

/// Collapse target for one collapsed room: its stair anchor across the folded
/// link, and whether it prefers sitting above (smaller y) that anchor.
struct CollapseAnchor {
    anchor: RoomId,
    above: bool,
}

/// Fold small vertical stubs back into the plane they hang off.
///
/// A one-room attic shouldn't spawn a whole level: a *cluster* (maximal set of
/// same-level rooms connected by any edge) collapses onto its neighbour level
/// when it has at most [`COLLAPSE_THRESHOLD`] rooms AND attaches to the rest of
/// the map through exactly ONE Up/Down link (a single stair anchor, counting a
/// reciprocal stair pair as one link). Collapsing reassigns the cluster's rooms
/// to the anchor room's level and records, for the cluster's stair-side room,
/// its anchor and the above/below bias (it was above iff its cluster's level
/// was higher). Chains fold transitively: fixpoint iteration re-clusters after
/// each round, so a ladder of one-room levels folds link by link. When a
/// candidate's anchor cluster is itself a candidate (mutually anchored pair),
/// exactly one side folds: the smaller cluster, or on equal size the
/// higher-level one (the attic folds onto the kitchen). Bigger clusters and
/// clusters bridged through two different stair links keep their own level.
/// Deterministic: connection order, min-root union-find, BTree iteration.
/// Returns the per-room collapse anchors and the set of all reassigned rooms.
fn collapse_small_clusters(
    ids: &[RoomId],
    sub: &MapGraph,
    levels: &mut BTreeMap<RoomId, i32>,
) -> (BTreeMap<RoomId, CollapseAnchor>, BTreeSet<RoomId>) {
    let n = ids.len();
    let index: BTreeMap<RoomId, usize> =
        ids.iter().enumerate().map(|(i, &id)| (id, i)).collect();
    let mut anchors: BTreeMap<RoomId, CollapseAnchor> = BTreeMap::new();
    let mut collapsed: BTreeSet<RoomId> = BTreeSet::new();

    fn find(parent: &mut [usize], mut v: usize) -> usize {
        while parent[v] != v {
            parent[v] = parent[parent[v]];
            v = parent[v];
        }
        v
    }

    loop {
        // Same-level clusters via min-root union-find over ALL edges.
        let mut parent: Vec<usize> = (0..n).collect();
        for c in sub.connections() {
            let (Some(&a), Some(&b)) = (index.get(&c.origin), index.get(&c.dest)) else {
                continue;
            };
            if a != b && levels[&ids[a]] == levels[&ids[b]] {
                let (ra, rb) = (find(&mut parent, a), find(&mut parent, b));
                if ra != rb {
                    parent[ra.max(rb)] = ra.min(rb);
                }
            }
        }
        let mut members: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
        for v in 0..n {
            let r = find(&mut parent, v);
            members.entry(r).or_default().push(v);
        }
        // External stair links per cluster, deduped by unordered room pair.
        let mut links: BTreeMap<usize, BTreeSet<(RoomId, RoomId)>> = BTreeMap::new();
        for c in sub.connections() {
            if !matches!(c.dir, Direction::Up | Direction::Down) || c.origin == c.dest {
                continue;
            }
            let (Some(&a), Some(&b)) = (index.get(&c.origin), index.get(&c.dest)) else {
                continue;
            };
            let (ra, rb) = (find(&mut parent, a), find(&mut parent, b));
            if ra == rb {
                continue;
            }
            let key = (c.origin.min(c.dest), c.origin.max(c.dest));
            links.entry(ra).or_default().insert(key);
            links.entry(rb).or_default().insert(key);
        }

        let candidates: BTreeSet<usize> = members
            .iter()
            .filter(|(r, m)| {
                m.len() <= COLLAPSE_THRESHOLD && links.get(r).map_or(0, BTreeSet::len) == 1
            })
            .map(|(&r, _)| r)
            .collect();

        let mut changed = false;
        for &root in &candidates {
            let &(p, q) = links[&root].first().expect("candidate has one link");
            let (a_in, b_out) =
                if find(&mut parent, index[&p]) == root { (p, q) } else { (q, p) };
            let anchor_root = find(&mut parent, index[&b_out]);
            let clevel = levels[&ids[members[&root][0]]];
            let alevel = levels[&b_out];
            // Mutually anchored candidate pair: exactly one side folds — the
            // smaller cluster, or on equal size the higher-level one.
            if candidates.contains(&anchor_root) {
                let (cs, asz) = (members[&root].len(), members[&anchor_root].len());
                if !(cs < asz || (cs == asz && clevel > alevel)) {
                    continue;
                }
            }
            for &v in &members[&root] {
                levels.insert(ids[v], alevel);
                collapsed.insert(ids[v]);
            }
            anchors.insert(a_in, CollapseAnchor { anchor: b_out, above: clevel > alevel });
            changed = true;
        }
        if !changed {
            return (anchors, collapsed);
        }
    }
}

/// One level's solve state: its rooms (sorted by id), their refined positions,
/// and the separation constraints used — kept so the stairwell-coherence nudge
/// can re-project onto the same feasible region.
struct LevelSolve {
    ids: Vec<RoomId>,
    pos: Vec<(f64, f64)>,
    xc: Vec<Constraint>,
    yc: Vec<Constraint>,
}

/// Rectangle-aware refinement for ONE level's rooms: constrained stress
/// majorization over the placed rooms. Every same-level pair whose cells differ
/// on an axis gets a VPSC separation constraint on that axis, in cell order,
/// with gap `half_a + half_b + margin` — so sizes genuinely push neighbors,
/// order is preserved, and no two same-level rects can overlap. Planar-connected
/// pairs (cardinal AND diagonal) get margin 0: a cardinal pair may abut, and a
/// diagonal pair — constrained on both axes by its diagonal cell order — may
/// close all the way up to a corner touch. (Relaxing one axis below the
/// half-sum for diagonal-only pairs was evaluated and rejected: letting the
/// pair overlap on an axis puts the arrival corner on the wrong side of the
/// departure corner, so the drawn corner-to-corner diagonal would point the
/// wrong way.) Everything else keeps [`UNCONNECTED_MARGIN`]. Rooms on OTHER
/// levels impose nothing here — they are different floors and may overlap this
/// one in x/y.
fn refine_level(
    ids: &[RoomId],
    cells: &BTreeMap<RoomId, (i32, i32)>,
    sizes: &BTreeMap<RoomId, (f32, f32)>,
    sub: &MapGraph,
    connected: &BTreeSet<(RoomId, RoomId)>,
) -> LevelSolve {
    let n = ids.len();
    let index: BTreeMap<RoomId, usize> =
        ids.iter().enumerate().map(|(i, &id)| (id, i)).collect();

    let mut xc: Vec<Constraint> = Vec::new();
    let mut yc: Vec<Constraint> = Vec::new();
    for i in 0..n {
        for j in (i + 1)..n {
            let (a, b) = (ids[i], ids[j]);
            let (ca, cb) = (cells[&a], cells[&b]);
            let (sa, sb) = (sizes[&a], sizes[&b]);
            let margin = if connected.contains(&(a.min(b), a.max(b))) {
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

    // Graph-distance targets pull connected rooms together; the VPSC projection
    // each iteration keeps the gaps above feasible. Ideals are size-aware so
    // stress stops fighting VPSC: a direct edge's ideal is the abutment
    // distance (half-size sums along the connecting axis; the corner-touch
    // center distance for a diagonal) plus [`IDEAL_SLACK`], and multi-hop
    // ideals scale hop counts by the mean room extent.
    let mut adj: Vec<Vec<usize>> = vec![Vec::new(); n];
    let mut edge_ideal: BTreeMap<(usize, usize), f64> = BTreeMap::new();
    for c in sub.connections() {
        let Some(off) = grid_offset(c.dir) else { continue };
        let (Some(&i), Some(&j)) = (index.get(&c.origin), index.get(&c.dest)) else { continue };
        if i == j {
            continue;
        }
        adj[i].push(j);
        adj[j].push(i);
        let (sa, sb) = (sizes[&ids[i]], sizes[&ids[j]]);
        let ex = f64::from(sa.0 + sb.0) / 2.0;
        let ey = f64::from(sa.1 + sb.1) / 2.0;
        let abut = match (off.0 != 0, off.1 != 0) {
            (true, false) => ex,
            (false, true) => ey,
            _ => ex.hypot(ey),
        };
        let d = abut + IDEAL_SLACK;
        edge_ideal
            .entry((i.min(j), i.max(j)))
            .and_modify(|v| *v = v.min(d))
            .or_insert(d);
    }
    let mut dist = stress::all_pairs_dist(n, &adj);
    let mean_ext = ids.iter().map(|id| f64::from(sizes[id].0 + sizes[id].1) / 2.0).sum::<f64>()
        / n as f64;
    for row in &mut dist {
        for d in row.iter_mut() {
            if d.is_finite() {
                *d *= mean_ext;
            }
        }
    }
    for (&(i, j), &d) in &edge_ideal {
        dist[i][j] = d;
        dist[j][i] = d;
    }
    let seed: Vec<(f64, f64)> = ids
        .iter()
        .map(|id| {
            let (cx, cy) = cells[id];
            (f64::from(cx) * SEED_PITCH_X, f64::from(cy) * SEED_PITCH_Y)
        })
        .collect();
    let pos = stress::stress_layout(n, &dist, &xc, &yc, &seed, ITERS);
    LevelSolve { ids: ids.to_vec(), pos, xc, yc }
}

/// Continuous compaction: iteratively pull every position toward the layout's
/// centroid as far as VPSC feasibility allows — the continuous analogue of
/// `tiles::compact_empty_lines`. Each of the [`COMPACT_ITERS`] rounds moves
/// every point [`COMPACT_STEP`] of the way toward the mean position, then
/// re-projects each axis onto the SAME separation constraints the refinement
/// used, so every ordering and minimum-gap guarantee survives while slack the
/// stress solve left behind (ideal targets sit [`IDEAL_SLACK`] outside the
/// boundary) is squeezed out. Shared with `district::compose` for the
/// super-node layout. Deterministic: fixed rounds, uniform weights.
pub(crate) fn compact_toward_centroid(
    pos: &mut [(f64, f64)],
    xc: &[Constraint],
    yc: &[Constraint],
) {
    let n = pos.len();
    if n <= 1 {
        return;
    }
    let w = vec![1.0; n];
    for _ in 0..COMPACT_ITERS {
        let cx = pos.iter().map(|p| p.0).sum::<f64>() / n as f64;
        let cy = pos.iter().map(|p| p.1).sum::<f64>() / n as f64;
        let dx: Vec<f64> = pos.iter().map(|p| p.0 + COMPACT_STEP * (cx - p.0)).collect();
        let dy: Vec<f64> = pos.iter().map(|p| p.1 + COMPACT_STEP * (cy - p.1)).collect();
        let nx = vpsc::solve_axis(&dx, &w, xc);
        let ny = vpsc::solve_axis(&dy, &w, yc);
        for (i, p) in pos.iter_mut().enumerate() {
            *p = (nx[i], ny[i]);
        }
    }
}

/// Stairwell coherence: pull each cross-level Up/Down edge's endpoints toward
/// the same x/y so stacked floors line their stairs up (replaces
/// `stack_updown_rooms`), and pull each collapsed room into a stair column
/// beside its anchor.
///
/// Mechanism: a post-VPSC iterative nudge, NOT extra SMACOF pairs — cross-level
/// stair endpoints sit in different per-level solves, so a shared stress solve
/// would reintroduce the cross-level coupling that per-level solving just
/// removed. Each round computes a desired position per room: a cross-level
/// stair endpoint wants the midpoint of its pair (averaged when a room serves
/// several stairs), a collapsed room wants its anchor's x and the anchor's y
/// offset by their half-heights plus [`COLLAPSE_GAP`] — above or below per its
/// bias (one-sided: the anchor itself is not dragged) — and every other room
/// wants to stay put. Each level then re-projects that desired vector onto its
/// OWN separation constraints via the same VPSC solve, so rooms move toward
/// their targets exactly as far as feasibility allows and every existing
/// separation guarantee is preserved. Deterministic: BTree iteration, fixed
/// rounds.
fn stairwell_coherence(
    solves: &mut BTreeMap<i32, LevelSolve>,
    sub: &MapGraph,
    levels: &BTreeMap<RoomId, i32>,
    sizes: &BTreeMap<RoomId, (f32, f32)>,
    anchors: &BTreeMap<RoomId, CollapseAnchor>,
) {
    let mut pairs: BTreeSet<(RoomId, RoomId)> = BTreeSet::new();
    for c in sub.connections() {
        if !matches!(c.dir, Direction::Up | Direction::Down) || c.origin == c.dest {
            continue;
        }
        // Only cross-level stairs get the midpoint pull; a same-level stair is
        // a collapsed link handled by the anchor bias below.
        let (Some(&la), Some(&lb)) = (levels.get(&c.origin), levels.get(&c.dest)) else {
            continue;
        };
        if la != lb {
            pairs.insert((c.origin.min(c.dest), c.origin.max(c.dest)));
        }
    }
    if pairs.is_empty() && anchors.is_empty() {
        return;
    }
    let locate: BTreeMap<RoomId, (i32, usize)> = solves
        .iter()
        .flat_map(|(&lv, s)| s.ids.iter().enumerate().map(move |(i, &id)| (id, (lv, i))))
        .collect();
    for _ in 0..COHERENCE_ITERS {
        // Midpoint target per stair endpoint, averaged over its pairs.
        let mut sum: BTreeMap<RoomId, (f64, f64, f64)> = BTreeMap::new();
        for &(a, b) in &pairs {
            let (la, ia) = locate[&a];
            let (lb, ib) = locate[&b];
            let pa = solves[&la].pos[ia];
            let pb = solves[&lb].pos[ib];
            let mid = ((pa.0 + pb.0) / 2.0, (pa.1 + pb.1) / 2.0);
            for id in [a, b] {
                let e = sum.entry(id).or_insert((0.0, 0.0, 0.0));
                e.0 += mid.0;
                e.1 += mid.1;
                e.2 += 1.0;
            }
        }
        // Collapsed rooms: pull into a stair column beside the anchor. A
        // collapsed room has no cross-level pair (its stair is same-level), so
        // this cleanly replaces any midpoint sum.
        for (&room, ca) in anchors {
            let (la, ia) = locate[&ca.anchor];
            let ap = solves[&la].pos[ia];
            let off = f64::from(sizes[&ca.anchor].1 + sizes[&room].1) / 2.0 + COLLAPSE_GAP;
            let ty = if ca.above { ap.1 - off } else { ap.1 + off };
            sum.insert(room, (ap.0, ty, 1.0));
        }
        for s in solves.values_mut() {
            let mut dx: Vec<f64> = s.pos.iter().map(|p| p.0).collect();
            let mut dy: Vec<f64> = s.pos.iter().map(|p| p.1).collect();
            for (i, id) in s.ids.iter().enumerate() {
                if let Some(&(sx, sy, cnt)) = sum.get(id) {
                    dx[i] = sx / cnt;
                    dy[i] = sy / cnt;
                }
            }
            let w = vec![1.0; s.ids.len()];
            let nx = vpsc::solve_axis(&dx, &w, &s.xc);
            let ny = vpsc::solve_axis(&dy, &w, &s.yc);
            for (i, p) in s.pos.iter_mut().enumerate() {
                *p = (nx[i], ny[i]);
            }
        }
    }
}

/// Feature anchor inside/on the room rect, mirroring `tiles::stamp_features`
/// placement: stairs sit in the top-right (up) / bottom-right (down) interior,
/// portals sit on the right wall (in above center, out below). Shared with the
/// vector orchestrator, which re-anchors cross-district features.
pub(crate) fn feature_anchor(kind: FeatureKind, center: Vec2, half: Vec2) -> Vec2 {
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

/// Standalone V1.5 collapse derivation over a FULL layer sub-graph: the
/// `(stair-side room, anchor room)` pair of every small vertical cluster that
/// [`collapse_small_clusters`] folds onto its anchor's plane. Sorted by the
/// collapsed room id (BTree iteration) — deterministic.
///
/// Used by `district::partition` to union a collapsed satellite into its
/// anchor's district BEFORE the per-district builds: districts are planar
/// components, so an attic reached only by stairs would otherwise become its
/// own district and the per-district [`build_model`] would never see anchor and
/// satellite together — the collapse placement could never fire. The
/// per-district build then RECOMPUTES the same decisions on its district
/// sub-graph; they coincide because the union puts the folded cluster, its
/// single stair link, and its anchor all inside one district.
pub(crate) fn collapse_anchor_pairs(graph: &MapGraph, layer: LayerId) -> Vec<(RoomId, RoomId)> {
    let sub = graph.layer_subgraph(layer);
    let cells: BTreeMap<RoomId, (i32, i32)> =
        sub.rooms().filter_map(|r| r.pos.map(|p| (r.id, p))).collect();
    if cells.is_empty() {
        return Vec::new();
    }
    let ids: Vec<RoomId> = cells.keys().copied().collect();
    let (mut levels, _) = room_levels(&ids, &sub);
    let (anchors, _) = collapse_small_clusters(&ids, &sub, &mut levels);
    anchors.iter().map(|(&room, ca)| (room, ca.anchor)).collect()
}

/// Build the continuous model for one layer's sub-graph. Rooms and features
/// only for now; `paths` is left empty until the polyline router (V2).
///
/// Stages: derive 2.5D levels from the Up/Down subgraph ([`room_levels`]),
/// fold small single-stair vertical stubs back into their anchor plane
/// ([`collapse_small_clusters`]), refine x/y PER LEVEL ([`refine_level`] —
/// rooms on different levels never constrain each other and may overlap),
/// compact each level toward its centroid ([`compact_toward_centroid`]), then
/// nudge stairwells coherent across levels ([`stairwell_coherence`]).
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
    let (mut levels, distorted_levels) = room_levels(&ids, &sub);
    let (anchors, collapsed) = collapse_small_clusters(&ids, &sub, &mut levels);

    // ANY planar connection — cardinal or diagonal — earns the zero margin.
    let mut connected: BTreeSet<(RoomId, RoomId)> = BTreeSet::new();
    for c in sub.connections() {
        if grid_offset(c.dir).is_some() && c.origin != c.dest {
            connected.insert((c.origin.min(c.dest), c.origin.max(c.dest)));
        }
    }
    let mut groups: BTreeMap<i32, Vec<RoomId>> = BTreeMap::new();
    for &id in &ids {
        groups.entry(levels[&id]).or_default().push(id);
    }
    let mut solves: BTreeMap<i32, LevelSolve> = groups
        .iter()
        .map(|(&lv, gids)| (lv, refine_level(gids, &cells, &sizes, &sub, &connected)))
        .collect();
    // Compaction runs BEFORE the coherence nudge: the nudge re-projects onto
    // the same constraints and is the last word on stairwell alignment and
    // collapsed-room placement, so compacting first cannot break either —
    // compacting after would drag aligned stair columns apart again.
    for s in solves.values_mut() {
        compact_toward_centroid(&mut s.pos, &s.xc, &s.yc);
    }
    stairwell_coherence(&mut solves, &sub, &levels, &sizes, &anchors);

    let pos: BTreeMap<RoomId, (f64, f64)> = solves
        .values()
        .flat_map(|s| s.ids.iter().copied().zip(s.pos.iter().copied()))
        .collect();
    let rooms: Vec<ModelRoom> = ids
        .iter()
        .map(|&id| {
            let (w, h) = sizes[&id];
            let p = pos[&id];
            ModelRoom {
                id,
                center: (p.0 as f32, p.1 as f32),
                half: (w / 2.0, h / 2.0),
                level: levels[&id],
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

    MapModel {
        rooms,
        paths: Vec::new(),
        features,
        distorted_levels,
        collapsed: collapsed.into_iter().collect(),
    }
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
    fn connected_diagonal_pair_touches_corners() {
        let mut g = MapGraph::new();
        room_at(&mut g, 1, (0, 0));
        room_at(&mut g, 2, (1, 1));
        g.add_edge(1, Direction::SE, 2);
        g.add_edge(2, Direction::NW, 1);
        let m = build_model(&g, MAIN_LAYER);
        let (a, b) = (find(&m, 1), find(&m, 2));
        assert!(
            b.center.0 > a.center.0 && b.center.1 > a.center.1,
            "SE neighbor must stay south-east: a {:?} b {:?}",
            a.center,
            b.center
        );
        let gx = (b.center.0 - b.half.0) - (a.center.0 + a.half.0);
        let gy = (b.center.1 - b.half.1) - (a.center.1 + a.half.1);
        assert!(gx.abs() < EPS, "diagonal pair closes to a corner touch in x: gap {gx}");
        assert!(gy.abs() < EPS, "diagonal pair closes to a corner touch in y: gap {gy}");
        // Tightness bound: the closest-corner Euclidean gap is (near) zero —
        // nothing like the old UNCONNECTED_MARGIN on both axes.
        assert!(gx.hypot(gy) < 2.0 * EPS, "corner gap {}", gx.hypot(gy));
    }

    #[test]
    fn compaction_shrinks_l_shape_bbox() {
        let mut g = MapGraph::new();
        room_at(&mut g, 1, (0, 0));
        room_at(&mut g, 2, (1, 0));
        room_at(&mut g, 3, (1, 1));
        g.add_edge(1, Direction::E, 2);
        g.add_edge(2, Direction::S, 3);
        let sub = g.layer_subgraph(MAIN_LAYER);
        let cells: BTreeMap<RoomId, (i32, i32)> =
            sub.rooms().filter_map(|r| r.pos.map(|p| (r.id, p))).collect();
        let ids: Vec<RoomId> = cells.keys().copied().collect();
        let sizes = room_sizes(&sub);
        let connected: BTreeSet<(RoomId, RoomId)> = [(1, 2), (2, 3)].into();
        let solve = refine_level(&ids, &cells, &sizes, &sub, &connected);
        let bbox_area = |s: &LevelSolve| {
            let (mut lo, mut hi) = ((f64::INFINITY, f64::INFINITY), (f64::NEG_INFINITY, f64::NEG_INFINITY));
            for (i, id) in s.ids.iter().enumerate() {
                let (hx, hy) = (f64::from(sizes[id].0) / 2.0, f64::from(sizes[id].1) / 2.0);
                lo.0 = lo.0.min(s.pos[i].0 - hx);
                lo.1 = lo.1.min(s.pos[i].1 - hy);
                hi.0 = hi.0.max(s.pos[i].0 + hx);
                hi.1 = hi.1.max(s.pos[i].1 + hy);
            }
            (hi.0 - lo.0) * (hi.1 - lo.1)
        };
        let before = bbox_area(&solve);
        let mut compacted = solve.pos.clone();
        compact_toward_centroid(&mut compacted, &solve.xc, &solve.yc);
        let mut again = solve.pos.clone();
        compact_toward_centroid(&mut again, &solve.xc, &solve.yc);
        assert_eq!(compacted, again, "compaction is deterministic");
        let after = bbox_area(&LevelSolve {
            ids: solve.ids.clone(),
            pos: compacted,
            xc: solve.xc.clone(),
            yc: solve.yc.clone(),
        });
        assert!(
            after <= before + 1e-9,
            "compaction must not grow the bounding box: {before} -> {after}"
        );
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

    // ── V1.5: z-levels, small-cluster collapse, stairwell coherence ─────────

    /// `n` stacked 4-room floors (rooms `f*4+1..=f*4+4` chained east on cell
    /// row `y = 5f`), no stairs — each floor is too big to collapse.
    fn floors(n: u16) -> MapGraph {
        let mut g = MapGraph::new();
        for f in 0..n {
            for i in 0..4u16 {
                let id = f * 4 + i + 1;
                room_at(&mut g, id, (i32::from(i), i32::from(f) * 5));
                if i > 0 {
                    g.add_edge(id - 1, Direction::E, id);
                }
            }
        }
        g
    }

    /// Two 4-room floors joined by one reciprocal stairwell at rooms 1 and 5.
    /// Each floor's cells share a row, so the stair endpoints are y-free and
    /// free to move west toward each other: "otherwise unconstrained".
    fn two_floor_graph() -> MapGraph {
        let mut g = floors(2);
        g.add_edge(1, Direction::Up, 5);
        g.add_edge(5, Direction::Down, 1);
        g
    }

    #[test]
    fn up_chain_levels_strictly_increase() {
        let mut g = floors(3);
        g.add_edge(1, Direction::Up, 5);
        g.add_edge(5, Direction::Up, 9);
        let m = build_model(&g, MAIN_LAYER);
        for id in 1..=4 {
            assert_eq!(find(&m, id).level, 0, "ground floor");
        }
        for id in 5..=8 {
            assert_eq!(find(&m, id).level, 1, "Up raises the level");
        }
        for id in 9..=12 {
            assert_eq!(find(&m, id).level, 2, "second Up raises it again");
        }
        assert!(m.distorted_levels.is_empty());
        assert!(m.collapsed.is_empty(), "4-room floors never collapse");
    }

    #[test]
    fn down_edge_lowers_the_destination() {
        let mut g = floors(2);
        g.add_edge(1, Direction::Down, 5);
        let m = build_model(&g, MAIN_LAYER);
        for id in 1..=4 {
            assert_eq!(find(&m, id).level, 1, "origin floor sits above");
        }
        for id in 5..=8 {
            assert_eq!(find(&m, id).level, 0, "Down lowers the destination");
        }
    }

    #[test]
    fn up_cycle_drops_closing_edge_and_reports_it() {
        let build = || {
            let mut g = MapGraph::new();
            room_at(&mut g, 1, (0, 0));
            room_at(&mut g, 2, (0, 1));
            room_at(&mut g, 3, (0, 2));
            g.add_edge(1, Direction::Up, 2);
            g.add_edge(2, Direction::Up, 3);
            g.add_edge(3, Direction::Up, 1);
            g
        };
        let m = build_model(&build(), MAIN_LAYER);
        assert_eq!(
            m.distorted_levels,
            vec![(3, 1)],
            "the cycle-closing edge (in connection order) is dropped and reported"
        );
        assert_eq!(find(&m, 1).level, 0);
        assert_eq!(find(&m, 2).level, 1);
        assert_eq!(find(&m, 3).level, 2);
        assert!(m.collapsed.is_empty(), "every room has two stair links — no single anchor");
        assert_eq!(build_model(&build(), MAIN_LAYER), m, "deterministic");
    }

    #[test]
    fn stair_endpoints_align_when_unconstrained() {
        let m = build_model(&two_floor_graph(), MAIN_LAYER);
        let (a, b) = (find(&m, 1), find(&m, 5));
        assert_eq!((a.level, b.level), (0, 1));
        assert!(m.collapsed.is_empty());
        assert!(
            (a.center.0 - b.center.0).abs() < 1e-2,
            "stairwell x coherence: {} vs {}",
            a.center.0,
            b.center.0
        );
        assert!(
            (a.center.1 - b.center.1).abs() < 1e-2,
            "stairwell y coherence: {} vs {}",
            a.center.1,
            b.center.1
        );
    }

    #[test]
    fn different_levels_may_overlap_in_xy() {
        let m = build_model(&two_floor_graph(), MAIN_LAYER);
        let (a, b) = (find(&m, 1), find(&m, 5));
        // The aligned stair endpoints are on different floors: their rects
        // overlap in x AND y, and that violates nothing.
        assert!((a.center.0 - b.center.0).abs() < a.half.0 + b.half.0);
        assert!((a.center.1 - b.center.1).abs() < a.half.1 + b.half.1);
    }

    #[test]
    fn planar_only_graph_is_all_level_zero() {
        let m = build_model(&block_graph(), MAIN_LAYER);
        assert!(m.rooms.iter().all(|r| r.level == 0));
        assert!(m.distorted_levels.is_empty());
        assert!(m.collapsed.is_empty());
    }

    #[test]
    fn planar_neighbours_inherit_their_cluster_level() {
        let mut g = floors(1);
        // Upper floor: stair room 5 plus planar mates 6..8 (4 rooms — no collapse);
        // only 5 touches a stair, the mates inherit its level across planar edges.
        for (id, cell) in [(5u16, (0, -2)), (6, (1, -2)), (7, (2, -2)), (8, (3, -2))] {
            room_at(&mut g, id, cell);
        }
        g.add_edge(5, Direction::E, 6);
        g.add_edge(6, Direction::E, 7);
        g.add_edge(7, Direction::E, 8);
        g.add_edge(1, Direction::Up, 5);
        room_at(&mut g, 9, (9, 9)); // no edges at all
        let m = build_model(&g, MAIN_LAYER);
        for id in 5..=8 {
            assert_eq!(find(&m, id).level, 1, "planar mates inherit the stair room's level");
        }
        assert_eq!(find(&m, 9).level, 0, "unreachable isolated room defaults to level 0");
        assert!(m.collapsed.is_empty());
    }

    #[test]
    fn one_room_attic_collapses_above_its_kitchen() {
        let mut g = MapGraph::new();
        // Ground floor: kitchen 1 heads a 4-room column (all on cell x 0, so
        // nothing constrains x and the attic can take the stair column exactly).
        for (id, cell) in [(1u16, (0, 0)), (2, (0, 1)), (3, (0, 2)), (4, (0, 3))] {
            room_at(&mut g, id, cell);
        }
        g.add_edge(1, Direction::S, 2);
        g.add_edge(2, Direction::S, 3);
        g.add_edge(3, Direction::S, 4);
        // One-room attic over the kitchen: single stair anchor → collapses.
        room_at(&mut g, 5, (0, -1));
        g.add_edge(1, Direction::Up, 5);
        g.add_edge(5, Direction::Down, 1);
        let m = build_model(&g, MAIN_LAYER);
        let (k, a) = (find(&m, 1), find(&m, 5));
        assert_eq!(a.level, 0, "attic folds onto the kitchen's level");
        assert_eq!(m.collapsed, vec![5]);
        assert!(
            (a.center.0 - k.center.0).abs() < 1e-2,
            "attic shares the stair column: x {} vs {}",
            a.center.0,
            k.center.0
        );
        let gap = (k.center.1 - k.half.1) - (a.center.1 + a.half.1);
        assert!(a.center.1 < k.center.1, "Up-collapsed room sits ABOVE its anchor");
        assert!(gap >= -EPS, "attic must not overlap the kitchen: gap {gap}");
        assert!(gap <= UNCONNECTED_MARGIN + 0.1, "and sits snugly against it: gap {gap}");
    }

    #[test]
    fn one_room_down_chain_folds_into_descending_column() {
        let mut g = MapGraph::new();
        for (id, cell) in [(1u16, (0, 0)), (2, (0, 1)), (3, (0, 2)), (4, (0, 3))] {
            room_at(&mut g, id, cell);
        }
        g.add_edge(1, Direction::Down, 2);
        g.add_edge(2, Direction::Down, 3);
        g.add_edge(3, Direction::Down, 4);
        let m = build_model(&g, MAIN_LAYER);
        let lv = find(&m, 3).level;
        for id in 1..=4 {
            assert_eq!(find(&m, id).level, lv, "ladder folds transitively onto one plane");
        }
        assert_eq!(m.collapsed, vec![1, 2, 4], "room 3 is the resident of the final plane");
        let y = |id: RoomId| find(&m, id).center.1;
        assert!(
            y(1) < y(2) && y(2) < y(3) && y(3) < y(4),
            "descending column preserved: {:?}",
            (y(1), y(2), y(3), y(4))
        );
        let x = |id: RoomId| find(&m, id).center.0;
        for id in 2..=4 {
            assert!((x(id) - x(1)).abs() < 1e-2, "column shares x");
        }
        assert_eq!(build_model(&g, MAIN_LAYER), m, "deterministic");
    }

    #[test]
    fn two_stair_anchors_prevent_collapse() {
        let mut g = floors(1);
        // 3-room upper cluster bridged to the ground through TWO stair rooms.
        for (id, cell) in [(5u16, (0, -2)), (6, (1, -2)), (7, (2, -2))] {
            room_at(&mut g, id, cell);
        }
        g.add_edge(5, Direction::E, 6);
        g.add_edge(6, Direction::E, 7);
        g.add_edge(1, Direction::Up, 5);
        g.add_edge(2, Direction::Up, 7);
        let m = build_model(&g, MAIN_LAYER);
        for id in 5..=7 {
            assert_eq!(find(&m, id).level, 1, "two stair anchors keep the cluster its own floor");
        }
        assert!(m.collapsed.is_empty());
    }

    #[test]
    fn large_cluster_keeps_its_own_level() {
        let mut g = floors(1);
        // A 12-room cellar under the 4-room ground floor, one Down stair.
        for i in 0..12u16 {
            room_at(&mut g, 20 + i, (i32::from(i), 4));
            if i > 0 {
                g.add_edge(19 + i, Direction::E, 20 + i);
            }
        }
        g.add_edge(1, Direction::Down, 20);
        let m = build_model(&g, MAIN_LAYER);
        for i in 0..12 {
            assert_eq!(find(&m, 20 + i).level, 0, "big cellar keeps its own floor");
        }
        for id in 1..=4 {
            assert_eq!(find(&m, id).level, 1);
        }
        assert!(m.collapsed.is_empty());
    }
}
