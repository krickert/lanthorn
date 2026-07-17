//! Continuous-space polyline router for the vector map model.
//!
//! Pure standalone geometry — no dependency on the rest of the crate. Routes a
//! path between two real-valued points around rectangular obstacles:
//!
//! - A* over a fine square lattice (resolution ~0.25 units) with 8-directional
//!   moves. Integer millicosts (orthogonal 1000, diagonal 1414) and
//!   binary-heap tie-breaking by `(cost, state index)` keep the search fully
//!   deterministic.
//! - Obstacles are rects inflated by `clearance`; endpoints may sit on (or
//!   inside) an inflated boundary — they are snapped to the nearest free
//!   lattice node before searching.
//! - When `from_dir` / `to_dir` are given, the first / last ~`clearance` of
//!   the path is forced along that unit direction (directional honesty: a NE
//!   exit truly leaves at 45°). `to_dir` is the direction of travel along the
//!   final segment *into* `to`.
//! - Cost shaping: a mild penalty for changing direction, EXCEPT that
//!   continuing straight beyond ~`5 / resolution` steps accrues a growing
//!   penalty modulated by an FNV-1a hash of `(seed, step)`, so long hauls
//!   develop irregular 45°/staircase character. `waviness` scales this;
//!   0 = shortest path.
//! - Post-processing: collinear and near-collinear simplification (points
//!   whose removal deviates by less than `resolution / 2` are dropped; the
//!   output point set is a subset of the searched points, so simplification
//!   never introduces a point inside an obstacle), then an optional
//!   [`offset_parallel`] pass that pushes near-parallel co-corridor paths
//!   apart laterally.

use std::cmp::Reverse;
use std::collections::BinaryHeap;

/// A 2D point / vector in model units.
pub type Vec2 = (f32, f32);

// ── Public types ─────────────────────────────────────────────────────────────

/// Axis-aligned rectangle: center plus half-extents.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RectF {
    pub cx: f32,
    pub cy: f32,
    pub hw: f32,
    pub hh: f32,
}

impl RectF {
    /// Is `p` strictly inside this rect inflated by `inflate` on every side?
    /// Points exactly on the inflated boundary count as free.
    fn contains_inflated(&self, p: Vec2, inflate: f32) -> bool {
        (p.0 - self.cx).abs() < self.hw + inflate - EPS
            && (p.1 - self.cy).abs() < self.hh + inflate - EPS
    }
}

/// Tuning knobs for [`route_path`].
#[derive(Debug, Clone)]
pub struct RouteOpts {
    /// Minimum distance kept from obstacle rects (rects are inflated by this).
    pub clearance: f32,
    /// Lattice spacing in model units. ~0.25 gives fine-grained routes.
    pub resolution: f32,
    /// Strength of the seeded long-haul meander. 0 = shortest path.
    pub waviness: f32,
    /// A* node-visit budget; exceeding it aborts to a straight fallback.
    pub max_visits: usize,
}

impl Default for RouteOpts {
    fn default() -> Self {
        RouteOpts {
            clearance: 0.25,
            resolution: 0.25,
            waviness: 0.0,
            max_visits: 100_000,
        }
    }
}

/// A routed polyline. `complete == false` means the search failed (blocked,
/// out of budget, or degenerate input) and `points` holds the straight
/// two-point fallback `[from, to]`; the caller may draw it as-is.
#[derive(Debug, Clone, PartialEq)]
pub struct RoutedPath {
    pub points: Vec<Vec2>,
    pub complete: bool,
}

// ── Cost constants (integer millicosts per lattice step) ─────────────────────

const EPS: f32 = 1e-4;
/// Cost of an orthogonal lattice step.
const COST_ORTH: i64 = 1000;
/// Cost of a diagonal lattice step (~1000 * sqrt(2)).
const COST_DIAG: i64 = 1414;
/// Mild penalty per 45° increment of direction change.
const TURN_PENALTY_PER_OCTANT: i64 = 250;
/// Straight runs longer than this many *model units* start accruing the
/// waviness penalty (converted to steps as `5.0 / resolution`).
const STRAIGHT_CAP_UNITS: f32 = 5.0;
/// Base waviness penalty in millicost per excess straight step, scaled by
/// `waviness`, by how far past the cap the run is, and by a hash in [0.5, 1.5).
const WAVE_BASE: f32 = 40.0;
/// Hard cap on lattice size (cells); larger searches fall back to straight.
const MAX_CELLS: usize = 2_000_000;

/// 8 lattice move directions, octant order (E, NE-ish going counter-clockwise
/// in +y-up terms; the actual orientation of y does not matter to the router).
const DIRS: [(i32, i32); 8] = [
    (1, 0),
    (1, 1),
    (0, 1),
    (-1, 1),
    (-1, 0),
    (-1, -1),
    (0, -1),
    (1, -1),
];
/// Pseudo-direction for the start state (no incoming direction).
const NO_DIR: usize = 8;

// ── Small vector helpers ─────────────────────────────────────────────────────

fn dist2(a: Vec2, b: Vec2) -> f32 {
    let dx = a.0 - b.0;
    let dy = a.1 - b.1;
    dx * dx + dy * dy
}

fn normalize(d: Vec2) -> Option<Vec2> {
    let len = (d.0 * d.0 + d.1 * d.1).sqrt();
    if len < EPS {
        None
    } else {
        Some((d.0 / len, d.1 / len))
    }
}

/// Distance from `p` to the segment `a..b`.
fn seg_dist(p: Vec2, a: Vec2, b: Vec2) -> f32 {
    let ab = (b.0 - a.0, b.1 - a.1);
    let len2 = ab.0 * ab.0 + ab.1 * ab.1;
    if len2 < EPS * EPS {
        return dist2(p, a).sqrt();
    }
    let t = (((p.0 - a.0) * ab.0 + (p.1 - a.1) * ab.1) / len2).clamp(0.0, 1.0);
    let q = (a.0 + ab.0 * t, a.1 + ab.1 * t);
    dist2(p, q).sqrt()
}

/// FNV-1a over the little-endian bytes of `(seed, step)`.
fn fnv1a(seed: u64, step: u64) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in seed.to_le_bytes().into_iter().chain(step.to_le_bytes()) {
        h ^= u64::from(b);
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    h
}

/// Deterministic hash-derived value in `[0, 1)`.
fn hash01(seed: u64, step: u64) -> f32 {
    (fnv1a(seed, step) >> 40) as f32 / (1u64 << 24) as f32
}

// ── Lattice ──────────────────────────────────────────────────────────────────

struct Lattice {
    ox: f32,
    oy: f32,
    nx: i32,
    ny: i32,
    res: f32,
    blocked: Vec<bool>,
}

impl Lattice {
    fn idx(&self, ix: i32, iy: i32) -> usize {
        iy as usize * self.nx as usize + ix as usize
    }

    fn world(&self, ix: i32, iy: i32) -> Vec2 {
        (self.ox + ix as f32 * self.res, self.oy + iy as f32 * self.res)
    }

    fn in_bounds(&self, ix: i32, iy: i32) -> bool {
        ix >= 0 && iy >= 0 && ix < self.nx && iy < self.ny
    }

    fn is_free(&self, ix: i32, iy: i32) -> bool {
        self.in_bounds(ix, iy) && !self.blocked[self.idx(ix, iy)]
    }

    /// Nearest free lattice node to `p`, searched in expanding Chebyshev
    /// rings (deterministic scan order; ties broken by scan order).
    fn snap_free(&self, p: Vec2) -> Option<(i32, i32)> {
        let ix0 = (((p.0 - self.ox) / self.res).round() as i32).clamp(0, self.nx - 1);
        let iy0 = (((p.1 - self.oy) / self.res).round() as i32).clamp(0, self.ny - 1);
        let max_r = ((2.0 / self.res).ceil() as i32).max(2);
        for r in 0..=max_r {
            let mut best: Option<(f32, i32, i32)> = None;
            for dy in -r..=r {
                for dx in -r..=r {
                    if dx.abs().max(dy.abs()) != r {
                        continue;
                    }
                    let (ix, iy) = (ix0 + dx, iy0 + dy);
                    if !self.is_free(ix, iy) {
                        continue;
                    }
                    let d2 = dist2(self.world(ix, iy), p);
                    if best.map(|(bd, _, _)| d2 < bd).unwrap_or(true) {
                        best = Some((d2, ix, iy));
                    }
                }
            }
            if let Some((_, ix, iy)) = best {
                return Some((ix, iy));
            }
        }
        None
    }
}

/// Build the search lattice: the bounding box of the points of interest plus
/// a routing margin, grown once to cover any inflated obstacle rect that
/// intersects it (single expansion pass; a chain of obstacles leading far
/// away can still bound the search, in which case the route falls back).
fn build_lattice(obstacles: &[RectF], interest: &[Vec2], opts: &RouteOpts) -> Option<Lattice> {
    let res = opts.resolution.max(1e-3);
    let margin = 2.0 + opts.clearance;
    let mut minx = f32::MAX;
    let mut miny = f32::MAX;
    let mut maxx = f32::MIN;
    let mut maxy = f32::MIN;
    for p in interest {
        minx = minx.min(p.0);
        miny = miny.min(p.1);
        maxx = maxx.max(p.0);
        maxy = maxy.max(p.1);
    }
    minx -= margin;
    miny -= margin;
    maxx += margin;
    maxy += margin;
    let inf = opts.clearance;
    for r in obstacles {
        let (rx0, rx1) = (r.cx - r.hw - inf, r.cx + r.hw + inf);
        let (ry0, ry1) = (r.cy - r.hh - inf, r.cy + r.hh + inf);
        if rx0 < maxx && rx1 > minx && ry0 < maxy && ry1 > miny {
            minx = minx.min(rx0 - margin);
            miny = miny.min(ry0 - margin);
            maxx = maxx.max(rx1 + margin);
            maxy = maxy.max(ry1 + margin);
        }
    }
    let ox = (minx / res).floor() * res;
    let oy = (miny / res).floor() * res;
    let nx = ((maxx - ox) / res).ceil() as i32 + 1;
    let ny = ((maxy - oy) / res).ceil() as i32 + 1;
    if nx <= 0 || ny <= 0 || nx as usize * ny as usize > MAX_CELLS {
        return None;
    }
    let mut blocked = vec![false; nx as usize * ny as usize];
    for iy in 0..ny {
        for ix in 0..nx {
            let p = (ox + ix as f32 * res, oy + iy as f32 * res);
            if obstacles.iter().any(|r| r.contains_inflated(p, inf)) {
                blocked[iy as usize * nx as usize + ix as usize] = true;
            }
        }
    }
    Some(Lattice {
        ox,
        oy,
        nx,
        ny,
        res,
        blocked,
    })
}

// ── A* search ────────────────────────────────────────────────────────────────

/// A* from `start` to `goal` over the lattice. State = (node, incoming
/// direction); 9 direction slots per node (8 moves + "none" for the start).
/// Returns the lattice polyline in world coordinates, or `None` when the
/// visit budget is exhausted or the goal is unreachable.
fn astar(
    lat: &Lattice,
    start: (i32, i32),
    goal: (i32, i32),
    seed: u64,
    opts: &RouteOpts,
) -> Option<Vec<Vec2>> {
    let nx = lat.nx as usize;
    let n_nodes = nx * lat.ny as usize;
    let n_states = n_nodes * 9;
    let mut gscore = vec![i64::MAX; n_states];
    let mut run_len = vec![0u32; n_states];
    let mut steps = vec![0u32; n_states];
    let mut came = vec![u32::MAX; n_states];

    let goal_node = lat.idx(goal.0, goal.1);
    let (gx, gy) = goal;
    // Octile-distance heuristic in base millicosts (admissible: shaping
    // penalties only ever add to the true cost).
    let heur = |node: usize| -> i64 {
        let ix = (node % nx) as i32;
        let iy = (node / nx) as i32;
        let dx = i64::from((ix - gx).abs());
        let dy = i64::from((iy - gy).abs());
        COST_ORTH * dx.max(dy) + (COST_DIAG - COST_ORTH) * dx.min(dy)
    };

    let start_key = lat.idx(start.0, start.1) * 9 + NO_DIR;
    gscore[start_key] = 0;
    let mut heap: BinaryHeap<Reverse<(i64, usize)>> = BinaryHeap::new();
    heap.push(Reverse((heur(start_key / 9), start_key)));

    let cap_steps = (STRAIGHT_CAP_UNITS / lat.res).round() as u32;
    let mut visits = 0usize;

    while let Some(Reverse((f, key))) = heap.pop() {
        let node = key / 9;
        let dir = key % 9;
        let g = gscore[key];
        if g == i64::MAX || f != g + heur(node) {
            continue; // stale heap entry
        }
        visits += 1;
        if visits > opts.max_visits {
            return None;
        }
        if node == goal_node {
            // Reconstruct: walk predecessors back to the start state.
            let mut rev: Vec<Vec2> = Vec::new();
            let mut k = key;
            loop {
                let n = k / 9;
                rev.push(lat.world((n % nx) as i32, (n / nx) as i32));
                if came[k] == u32::MAX {
                    break;
                }
                k = came[k] as usize;
            }
            rev.reverse();
            return Some(rev);
        }
        let ix = (node % nx) as i32;
        let iy = (node / nx) as i32;
        let cur_run = run_len[key];
        let cur_steps = steps[key];
        for (d, &(dx, dy)) in DIRS.iter().enumerate() {
            let (jx, jy) = (ix + dx, iy + dy);
            if !lat.is_free(jx, jy) {
                continue;
            }
            let mut cost = if dx != 0 && dy != 0 {
                COST_DIAG
            } else {
                COST_ORTH
            };
            let new_run;
            if dir == NO_DIR {
                new_run = 1;
            } else if d == dir {
                new_run = cur_run + 1;
            } else {
                new_run = 1;
                let diff = (d as i64 - dir as i64).abs();
                cost += TURN_PENALTY_PER_OCTANT * diff.min(8 - diff);
            }
            let new_steps = cur_steps + 1;
            if opts.waviness > 0.0 && new_run > cap_steps {
                let m = 0.5 + hash01(seed, u64::from(new_steps));
                cost += (opts.waviness * WAVE_BASE * (new_run - cap_steps) as f32 * m) as i64;
            }
            let nk = lat.idx(jx, jy) * 9 + d;
            let ng = g + cost;
            if ng < gscore[nk] {
                gscore[nk] = ng;
                run_len[nk] = new_run;
                steps[nk] = new_steps;
                came[nk] = key as u32;
                heap.push(Reverse((ng + heur(nk / 9), nk)));
            }
        }
    }
    None
}

// ── Post-processing ──────────────────────────────────────────────────────────

/// Greedy forward simplification: drop interior points whose removal keeps
/// every skipped point within `tol` of the replacement chord. Points listed
/// in `protected` are never dropped (used to preserve directional stubs).
/// The output is a subset of the input points.
fn simplify_points(pts: &[Vec2], tol: f32, protected: &[usize]) -> Vec<Vec2> {
    if pts.len() <= 2 {
        return pts.to_vec();
    }
    let last = pts.len() - 1;
    let mut out = vec![pts[0]];
    let mut i = 0usize;
    while i < last {
        let mut j = i + 1;
        let mut k = i + 2;
        while k <= last {
            if (i + 1..k).any(|m| protected.contains(&m)) {
                break;
            }
            if (i + 1..k).all(|m| seg_dist(pts[m], pts[i], pts[k]) <= tol) {
                j = k;
                k += 1;
            } else {
                break;
            }
        }
        out.push(pts[j]);
        i = j;
    }
    out
}

fn push_pt(pts: &mut Vec<Vec2>, p: Vec2) {
    let dup = matches!(pts.last(), Some(q) if dist2(*q, p) <= EPS * EPS);
    if !dup {
        pts.push(p);
    }
}

/// Is the straight segment `a..b` free of all inflated obstacles?
/// (Sampled at half-resolution spacing.)
fn segment_clear(obstacles: &[RectF], a: Vec2, b: Vec2, clearance: f32, res: f32) -> bool {
    let len = dist2(a, b).sqrt();
    let steps = ((len / (res * 0.5)).ceil() as usize).max(1);
    for s in 0..=steps {
        let t = s as f32 / steps as f32;
        let p = (a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t);
        if obstacles.iter().any(|r| r.contains_inflated(p, clearance)) {
            return false;
        }
    }
    true
}

// ── Public API ───────────────────────────────────────────────────────────────

/// Route an obstacle-avoiding polyline from `from` to `to`.
///
/// `from_dir` forces the departure to travel along that unit direction for
/// ~`clearance` before free routing begins; `to_dir` is the direction of
/// travel along the final segment into `to` (the approach covers the last
/// ~`clearance`). Forced stubs are honored even across obstacles.
///
/// On any failure (unreachable, lattice too large, visit budget exhausted)
/// the result is the straight two-point path with `complete == false`.
pub fn route_path(
    obstacles: &[RectF],
    from: Vec2,
    from_dir: Option<Vec2>,
    to: Vec2,
    to_dir: Option<Vec2>,
    seed: u64,
    opts: &RouteOpts,
) -> RoutedPath {
    let fallback = || RoutedPath {
        points: vec![from, to],
        complete: false,
    };
    if dist2(from, to) <= EPS * EPS {
        return RoutedPath {
            points: vec![from, to],
            complete: true,
        };
    }
    let res = opts.resolution.max(1e-3);
    let stub = opts.clearance.max(res);
    let fd = from_dir.and_then(normalize);
    let td = to_dir.and_then(normalize);

    // Fast path: no directional constraints, no waviness, straight shot clear.
    if fd.is_none()
        && td.is_none()
        && opts.waviness <= 0.0
        && segment_clear(obstacles, from, to, opts.clearance, res)
    {
        return RoutedPath {
            points: vec![from, to],
            complete: true,
        };
    }

    let start_anchor = fd.map_or(from, |d| (from.0 + d.0 * stub, from.1 + d.1 * stub));
    let goal_anchor = td.map_or(to, |d| (to.0 - d.0 * stub, to.1 - d.1 * stub));

    let Some(lat) = build_lattice(obstacles, &[from, to, start_anchor, goal_anchor], opts) else {
        return fallback();
    };
    let Some(start) = lat.snap_free(start_anchor) else {
        return fallback();
    };
    let Some(goal) = lat.snap_free(goal_anchor) else {
        return fallback();
    };
    let Some(mid) = astar(&lat, start, goal, seed, opts) else {
        return fallback();
    };

    let mut pts: Vec<Vec2> = Vec::with_capacity(mid.len() + 4);
    push_pt(&mut pts, from);
    if fd.is_some() {
        push_pt(&mut pts, start_anchor);
    }
    for p in mid {
        push_pt(&mut pts, p);
    }
    if td.is_some() {
        push_pt(&mut pts, goal_anchor);
    }
    push_pt(&mut pts, to);

    // Keep directional stub anchors through simplification so the true
    // departure/arrival angles survive.
    let mut protected: Vec<usize> = Vec::new();
    if fd.is_some() && pts.len() > 2 {
        protected.push(1);
    }
    if td.is_some() && pts.len() > 2 {
        protected.push(pts.len() - 2);
    }
    let pts = simplify_points(&pts, res * 0.5, &protected);

    RoutedPath {
        points: pts,
        complete: true,
    }
}

/// Push near-parallel paths sharing a corridor apart laterally by
/// `±separation / 2` each.
///
/// Heuristic pass, deterministic order (path pairs by index, segments by
/// position). For each pair of paths, the *first* pair of near-parallel
/// segments (within ~10° of parallel) whose perpendicular distance is below
/// `separation` and whose projections overlap is treated as a shared
/// corridor; the endpoints of those two segments are shifted apart along the
/// corridor normal (the lower-indexed path to the negative side when the
/// paths are coincident).
///
/// Known limitations:
/// - Only the matched segment of each path moves, which can introduce jogs
///   in longer polylines; only the first shared corridor per path pair is
///   handled.
/// - Path endpoints (door points) move too — callers that need pinned doors
///   should re-anchor the ends afterwards.
/// - Shifts from multiple pairs compound, and shifted paths are not
///   re-validated against obstacles or against each other.
pub fn offset_parallel(paths: &mut [RoutedPath], separation: f32) {
    if separation <= 0.0 || paths.len() < 2 {
        return;
    }
    let half = separation * 0.5;
    let mut deltas: Vec<Vec<Vec2>> = paths
        .iter()
        .map(|p| vec![(0.0, 0.0); p.points.len()])
        .collect();
    for a in 0..paths.len() {
        for b in a + 1..paths.len() {
            let Some((sa, sb, n, sign)) = find_corridor(&paths[a], &paths[b], separation) else {
                continue;
            };
            for i in [sa, sa + 1] {
                deltas[a][i].0 -= sign * n.0 * half;
                deltas[a][i].1 -= sign * n.1 * half;
            }
            for i in [sb, sb + 1] {
                deltas[b][i].0 += sign * n.0 * half;
                deltas[b][i].1 += sign * n.1 * half;
            }
        }
    }
    for (path, ds) in paths.iter_mut().zip(&deltas) {
        for (p, d) in path.points.iter_mut().zip(ds) {
            p.0 += d.0;
            p.1 += d.1;
        }
    }
}

/// First pair of near-parallel overlapping segments between two paths, if
/// any: `(segment index in a, segment index in b, unit normal of a's
/// segment, side sign of b relative to a)`.
fn find_corridor(a: &RoutedPath, b: &RoutedPath, separation: f32) -> Option<(usize, usize, Vec2, f32)> {
    for (sa, wa) in a.points.windows(2).enumerate() {
        let ua = normalize((wa[1].0 - wa[0].0, wa[1].1 - wa[0].1))?;
        for (sb, wb) in b.points.windows(2).enumerate() {
            let Some(ub) = normalize((wb[1].0 - wb[0].0, wb[1].1 - wb[0].1)) else {
                continue;
            };
            if (ua.0 * ub.0 + ua.1 * ub.1).abs() < 0.985 {
                continue; // not near-parallel
            }
            let n = (-ua.1, ua.0);
            let mid_a = ((wa[0].0 + wa[1].0) * 0.5, (wa[0].1 + wa[1].1) * 0.5);
            let mid_b = ((wb[0].0 + wb[1].0) * 0.5, (wb[0].1 + wb[1].1) * 0.5);
            let lateral = (mid_b.0 - mid_a.0) * n.0 + (mid_b.1 - mid_a.1) * n.1;
            if lateral.abs() >= separation {
                continue; // too far apart to share a corridor
            }
            // Longitudinal overlap of the two segments projected on ua.
            let proj = |p: Vec2| (p.0 - wa[0].0) * ua.0 + (p.1 - wa[0].1) * ua.1;
            let (a0, a1) = (proj(wa[0]), proj(wa[1]));
            let (b0, b1) = (proj(wb[0]), proj(wb[1]));
            let (alo, ahi) = (a0.min(a1), a0.max(a1));
            let (blo, bhi) = (b0.min(b1), b0.max(b1));
            if ahi.min(bhi) - alo.max(blo) <= EPS {
                continue; // no overlap along the corridor
            }
            let sign = if lateral < 0.0 { -1.0 } else { 1.0 };
            return Some((sa, sb, n, sign));
        }
    }
    None
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn opts(waviness: f32) -> RouteOpts {
        RouteOpts {
            waviness,
            ..RouteOpts::default()
        }
    }

    fn approx(a: f32, b: f32, eps: f32) -> bool {
        (a - b).abs() <= eps
    }

    /// Count direction changes (bends) in a polyline.
    fn bend_count(pts: &[Vec2]) -> usize {
        let mut bends = 0;
        for w in pts.windows(3) {
            let u = (w[1].0 - w[0].0, w[1].1 - w[0].1);
            let v = (w[2].0 - w[1].0, w[2].1 - w[1].1);
            let cross = u.0 * v.1 - u.1 * v.0;
            let dot = u.0 * v.0 + u.1 * v.1;
            if cross.abs() > 1e-3 || dot < 0.0 {
                bends += 1;
            }
        }
        bends
    }

    #[test]
    fn routes_around_blocking_rect() {
        let rect = RectF {
            cx: 5.0,
            cy: 0.0,
            hw: 1.0,
            hh: 1.0,
        };
        let o = opts(0.0);
        let path = route_path(&[rect], (0.0, 0.0), None, (10.0, 0.0), None, 7, &o);
        assert!(path.complete);
        assert_eq!(*path.points.first().unwrap(), (0.0, 0.0));
        assert_eq!(*path.points.last().unwrap(), (10.0, 0.0));
        assert!(path.points.len() > 2, "blocked straight line must bend");
        for p in &path.points {
            assert!(
                !rect.contains_inflated(*p, o.clearance),
                "point {p:?} inside inflated obstacle"
            );
        }
    }

    #[test]
    fn straight_when_unobstructed_and_no_waviness() {
        let path = route_path(&[], (0.0, 0.0), None, (10.0, 3.0), None, 1, &opts(0.0));
        assert!(path.complete);
        assert_eq!(path.points, vec![(0.0, 0.0), (10.0, 3.0)]);
    }

    #[test]
    fn diagonal_departure_is_honored() {
        let fd = (0.707, -0.707);
        let path = route_path(
            &[],
            (0.0, 0.0),
            Some(fd),
            (8.0, 0.0),
            None,
            3,
            &opts(0.0),
        );
        assert!(path.complete);
        assert!(path.points.len() >= 2);
        let p0 = path.points[0];
        let p1 = path.points[1];
        let slope = (p1.1 - p0.1) / (p1.0 - p0.0);
        assert!(
            approx(slope.abs(), 1.0, 1e-3),
            "first segment slope {slope} not 45 degrees"
        );
        assert!(slope < 0.0, "first segment must follow from_dir's sign");
        assert_eq!(*path.points.last().unwrap(), (8.0, 0.0));
    }

    #[test]
    fn arrival_direction_is_honored() {
        let td = (0.0, 1.0); // travelling +y into the endpoint
        let path = route_path(
            &[],
            (0.0, 0.0),
            None,
            (6.0, 4.0),
            Some(td),
            3,
            &opts(0.0),
        );
        assert!(path.complete);
        let n = path.points.len();
        let p0 = path.points[n - 2];
        let p1 = path.points[n - 1];
        assert!(approx(p1.0 - p0.0, 0.0, 1e-3), "final segment must be vertical");
        assert!(p1.1 > p0.1, "final segment must travel +y");
    }

    #[test]
    fn deterministic_output() {
        let obstacles = [
            RectF {
                cx: 4.0,
                cy: 1.0,
                hw: 1.5,
                hh: 0.8,
            },
            RectF {
                cx: 9.0,
                cy: -1.0,
                hw: 1.0,
                hh: 1.2,
            },
        ];
        let o = opts(0.7);
        let run = || {
            route_path(
                &obstacles,
                (0.0, 0.0),
                Some((1.0, 0.0)),
                (14.0, 0.5),
                Some((1.0, 0.0)),
                42,
                &o,
            )
        };
        let a = run();
        let b = run();
        assert_eq!(a, b);
    }

    #[test]
    fn waviness_bends_long_hauls_with_exact_endpoints() {
        let path = route_path(&[], (0.0, 0.0), None, (25.0, 0.0), None, 99, &opts(1.0));
        assert!(path.complete);
        assert_eq!(*path.points.first().unwrap(), (0.0, 0.0));
        assert_eq!(*path.points.last().unwrap(), (25.0, 0.0));
        let bends = bend_count(&path.points);
        assert!(bends >= 3, "expected >= 3 bends on a 25-unit haul, got {bends}");
    }

    #[test]
    fn zero_waviness_shortest_even_with_search() {
        // Force the full A* (via a from_dir aligned with the segment) and
        // check the result still simplifies down to a near-straight path.
        let path = route_path(
            &[],
            (0.0, 0.0),
            Some((1.0, 0.0)),
            (10.0, 0.0),
            None,
            5,
            &opts(0.0),
        );
        assert!(path.complete);
        assert!(
            bend_count(&path.points) == 0,
            "aligned departure over open ground should stay straight: {:?}",
            path.points
        );
    }

    #[test]
    fn max_visits_blowout_falls_back_straight() {
        let o = RouteOpts {
            max_visits: 2,
            ..RouteOpts::default()
        };
        let path = route_path(
            &[],
            (0.0, 0.0),
            Some((0.0, 1.0)),
            (10.0, 0.0),
            None,
            1,
            &o,
        );
        assert!(!path.complete);
        assert_eq!(path.points, vec![(0.0, 0.0), (10.0, 0.0)]);
    }

    #[test]
    fn unreachable_goal_falls_back_straight() {
        // Wall the goal off completely: the goal snaps inside the pocket and
        // the search exhausts without reaching it.
        let walls = [
            RectF {
                cx: 8.0,
                cy: 2.5,
                hw: 2.5,
                hh: 0.5,
            },
            RectF {
                cx: 8.0,
                cy: -2.5,
                hw: 2.5,
                hh: 0.5,
            },
            RectF {
                cx: 5.5,
                cy: 0.0,
                hw: 0.5,
                hh: 2.5,
            },
            RectF {
                cx: 10.5,
                cy: 0.0,
                hw: 0.5,
                hh: 2.5,
            },
        ];
        let path = route_path(&walls, (0.0, 0.0), None, (8.0, 0.0), None, 1, &opts(0.0));
        assert!(!path.complete);
        assert_eq!(path.points, vec![(0.0, 0.0), (8.0, 0.0)]);
    }

    #[test]
    fn offset_parallel_separates_coincident_paths() {
        let mk = || RoutedPath {
            points: vec![(0.0, 0.0), (10.0, 0.0)],
            complete: true,
        };
        let mut paths = [mk(), mk()];
        let sep = 0.5;
        offset_parallel(&mut paths, sep);
        let mid = |p: &RoutedPath| {
            (
                (p.points[0].0 + p.points[1].0) * 0.5,
                (p.points[0].1 + p.points[1].1) * 0.5,
            )
        };
        let (m0, m1) = (mid(&paths[0]), mid(&paths[1]));
        let d = dist2(m0, m1).sqrt();
        assert!(d >= sep * 0.9, "paths only {d} apart after offset");
        // Deterministic sides: both moved, in opposite lateral directions.
        assert!(approx(paths[0].points[0].1, -0.25, 1e-4));
        assert!(approx(paths[1].points[0].1, 0.25, 1e-4));
    }

    #[test]
    fn offset_parallel_leaves_distant_paths_alone() {
        let a = RoutedPath {
            points: vec![(0.0, 0.0), (10.0, 0.0)],
            complete: true,
        };
        let b = RoutedPath {
            points: vec![(0.0, 5.0), (10.0, 5.0)],
            complete: true,
        };
        let mut paths = [a.clone(), b.clone()];
        offset_parallel(&mut paths, 0.5);
        assert_eq!(paths[0], a);
        assert_eq!(paths[1], b);
    }

    #[test]
    fn collinear_chain_collapses_to_two_points() {
        let pts = vec![(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)];
        let out = simplify_points(&pts, 0.125, &[]);
        assert_eq!(out, vec![(0.0, 0.0), (3.0, 0.0)]);
    }

    #[test]
    fn near_collinear_point_dropped_within_tolerance() {
        let pts = vec![(0.0, 0.0), (1.0, 0.05), (2.0, 0.0)];
        let out = simplify_points(&pts, 0.125, &[]);
        assert_eq!(out, vec![(0.0, 0.0), (2.0, 0.0)]);
        // But a genuine bend survives.
        let pts = vec![(0.0, 0.0), (1.0, 0.5), (2.0, 0.0)];
        let out = simplify_points(&pts, 0.125, &[]);
        assert_eq!(out.len(), 3);
    }

    #[test]
    fn protected_points_survive_simplification() {
        let pts = vec![(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)];
        let out = simplify_points(&pts, 0.125, &[1]);
        assert_eq!(out, vec![(0.0, 0.0), (1.0, 0.0), (3.0, 0.0)]);
    }

    #[test]
    fn endpoint_on_obstacle_boundary_snaps_free() {
        // `from` sits exactly on the obstacle wall (a door): with clearance
        // inflation the point is inside the inflated rect, so it must snap to
        // a nearby free node and still produce a complete route.
        let rect = RectF {
            cx: 0.0,
            cy: 0.0,
            hw: 1.0,
            hh: 1.0,
        };
        let o = opts(0.0);
        let path = route_path(
            &[rect],
            (1.0, 0.0),
            Some((1.0, 0.0)),
            (6.0, 0.0),
            None,
            1,
            &o,
        );
        assert!(path.complete);
        assert_eq!(*path.points.first().unwrap(), (1.0, 0.0));
        assert_eq!(*path.points.last().unwrap(), (6.0, 0.0));
    }
}
