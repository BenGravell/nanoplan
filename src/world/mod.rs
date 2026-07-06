//! The interactive open world: an *infinite* procedurally generated street
//! network (a pure function of the seed), materialized Minecraft-style as a
//! 3×3 chunk window around the ego, routing to a user-placed goal, mixed IDM
//! traffic (cars, trucks, bikes, pedestrians), and the realtime closed-loop
//! stepper ([`LiveWorld`]) the viewer's "open world" mode drives — planner
//! and simulator run every tick, judo/treetop style, instead of precomputing
//! a rollout.

use std::collections::HashMap;

use web_time::Instant;

use crate::Rng;
use crate::planning::{Context, Planner, PlannerKind, bezier_idm::idm_accel};
use crate::scenarios::{Path, Road};
use crate::simulation::{Control, State, step};

/// Standard lane width; a street's half-width is `lanes × LANE_W_M`.
pub const LANE_W_M: f64 = 3.5;

const GRID_SPACING_M: f64 = 100.0;

/// Chunk side, in grid nodes; the active window is 3×3 chunks.
const CHUNK_NODES: i64 = 4;
const CHUNK_M: f64 = CHUNK_NODES as f64 * GRID_SPACING_M;

/// Spatial hysteresis: the ego must get this far past the center chunk's
/// bounds before the window recenters, so driving along a chunk line
/// doesn't thrash regeneration.
const RECENTER_HYST_M: f64 = 25.0;

/// Temporal hysteresis: an actor outside the active bounds survives this
/// long before despawning, so darting out of and back into a chunk doesn't
/// flicker traffic.
const DESPAWN_GRACE_S: f64 = 3.0;

/// How far outside the active window an actor may be before the despawn
/// grace clock starts.
const DESPAWN_MARGIN_M: f64 = 30.0;

/// Corner cut radius when a route turns at an intersection, so the lane
/// centerline stays drivable instead of kinking 90 degrees in place. Also
/// the taper length of lane gains/losses and junction lane connectors.
const CORNER_RADIUS_M: f64 = 12.0;

/// Corner cut radius of a slip-lane right turn: a wide curve that bypasses
/// the junction proper (see [`has_slip`]).
pub const SLIP_RADIUS_M: f64 = 22.0;

/// Length of a junction turn pocket (see [`has_pocket`]): over the last
/// stretch of the approach the roadway flares one lane wider on the right,
/// left-turners slide into the freed innermost lane, and through traffic
/// deflects around them.
pub const POCKET_M: f64 = 30.0;

/// Length of the pocket's flare taper, for drawing.
pub const POCKET_TAPER_M: f64 = 12.0;

/// How far short of a junction node crosswalks sit (see [`has_crosswalk`]).
pub const CROSSWALK_SETBACK_M: f64 = 15.0;

/// Extra route cost for starting out against the ego's current heading —
/// enough to prefer going around a couple of blocks over a U-turn (which
/// the planners track with an ugly off-lane loop), without ever making a
/// goal behind the ego unreachable.
const U_TURN_PENALTY_M: f64 = 400.0;

fn dist(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

fn mid(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0]
}

// --- procedural generation primitives -------------------------------------
//
// Everything below is a pure function of (seed, grid coordinates): any part
// of the infinite map can be queried at any time and always answers the
// same, which is what makes chunk unloading/reloading and window seams
// invisible.

/// SplitMix64-style hash of a grid coordinate pair under a salt.
fn hash(seed: u64, a: i64, b: i64, salt: u64) -> u64 {
    let mut z = seed
        ^ salt.wrapping_mul(0x9E3779B97F4A7C15)
        ^ (a as u64).wrapping_mul(0xBF58476D1CE4E5B9)
        ^ (b as u64).wrapping_mul(0x94D049BB133111EB);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D049BB133111EB);
    z ^ (z >> 31)
}

/// Hash folded to [0, 1).
fn unit(h: u64) -> f64 {
    (h >> 11) as f64 / (1u64 << 53) as f64
}

/// Classic 2D Perlin gradient noise, roughly in [-1, 1].
fn perlin(seed: u64, x: f64, y: f64) -> f64 {
    let (xi, yi) = (x.floor() as i64, y.floor() as i64);
    let (xf, yf) = (x - x.floor(), y - y.floor());
    let grad = |ix: i64, iy: i64, dx: f64, dy: f64| {
        let a = unit(hash(seed, ix, iy, 0x9E1)) * std::f64::consts::TAU;
        a.cos() * dx + a.sin() * dy
    };
    let fade = |t: f64| t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
    let lerp = |a: f64, b: f64, t: f64| a + (b - a) * t;
    let (u, v) = (fade(xf), fade(yf));
    lerp(
        lerp(grad(xi, yi, xf, yf), grad(xi + 1, yi, xf - 1.0, yf), u),
        lerp(
            grad(xi, yi + 1, xf, yf - 1.0),
            grad(xi + 1, yi + 1, xf - 1.0, yf - 1.0),
            u,
        ),
        v,
    )
}

/// Urbanization field in [0, 1]: dense downtown blocks where high, sparse
/// semi-rural streets where low. Drives street density, lane counts, actor
/// density, and the traffic mix.
fn urban(seed: u64, p: [f64; 2]) -> f64 {
    (0.5 + 0.5 * perlin(seed ^ 0x5EED_0B5C, p[0] / 350.0, p[1] / 350.0)).clamp(0.0, 1.0)
}

/// Intersection position for grid coordinate `c`: grid spacing plus a
/// deterministic jitter so the map doesn't read as graph paper.
fn node_pos(seed: u64, c: [i64; 2]) -> [f64; 2] {
    let j = 0.22 * GRID_SPACING_M;
    [
        c[0] as f64 * GRID_SPACING_M + (unit(hash(seed, c[0], c[1], 0xA11)) * 2.0 - 1.0) * j,
        c[1] as f64 * GRID_SPACING_M + (unit(hash(seed, c[0], c[1], 0xA22)) * 2.0 - 1.0) * j,
    ]
}

/// Order an adjacent grid pair canonically (lexicographically).
fn canon(a: [i64; 2], b: [i64; 2]) -> ([i64; 2], [i64; 2]) {
    if (a[0], a[1]) <= (b[0], b[1]) {
        (a, b)
    } else {
        (b, a)
    }
}

/// Is the grid line this edge lies on an arterial? Arterials are whole
/// rows/columns (~1 in 4), so multi-lane roads run for blocks instead of
/// appearing edge by edge.
fn arterial(seed: u64, a: [i64; 2], b: [i64; 2]) -> bool {
    if a[1] == b[1] {
        hash(seed, a[1], 0, 0xA57E).is_multiple_of(4)
    } else {
        hash(seed, a[0], 0, 0xA57F).is_multiple_of(4)
    }
}

/// Does the street between adjacent grid coordinates exist? Arterials
/// always do; every node also keeps one "parent" street (west or south,
/// hashed) so the network stays connected without a global check; the rest
/// survive a density draw that keeps more streets downtown than in the
/// sticks.
fn edge_exists(seed: u64, a: [i64; 2], b: [i64; 2]) -> bool {
    let (a, b) = canon(a, b);
    if arterial(seed, a, b) {
        return true;
    }
    // b = a + x̂ or a + ŷ; the edge is b's parent street when the hashed
    // parent direction (west vs south) points from b back to a
    let parent_west = hash(seed, b[0], b[1], 0xFA7) & 1 == 0;
    if parent_west == (b[0] > a[0]) {
        return true;
    }
    let salt = if b[0] > a[0] { 0xED60 } else { 0xED61 };
    let drop_p = 0.45 - 0.3 * urban(seed, mid(node_pos(seed, a), node_pos(seed, b)));
    unit(hash(seed, a[0], a[1], salt)) >= drop_p
}

/// Lanes per direction on this street: locals are 1 (sometimes 2 downtown),
/// arterials 2 — with occasional per-block promotions and demotions, so a
/// road widens and narrows along its length and lane counts shift across
/// junctions.
fn edge_lanes(seed: u64, a: [i64; 2], b: [i64; 2]) -> usize {
    let (a, b) = canon(a, b);
    let salt = if b[0] > a[0] { 0x1A90 } else { 0x1A91 };
    let h = hash(seed, a[0], a[1], salt);
    if arterial(seed, a, b) {
        match h % 8 {
            0 => 3, // gains a lane for a block
            1 => 1, // narrows: lane drop
            _ => 2,
        }
    } else if urban(seed, mid(node_pos(seed, a), node_pos(seed, b))) > 0.55 && h.is_multiple_of(3) {
        2
    } else {
        1
    }
}

/// Grid neighbours of `c` connected by an existing street (always at least
/// one: `c`'s own parent street).
fn neighbors(seed: u64, c: [i64; 2]) -> impl Iterator<Item = [i64; 2]> {
    [[1, 0], [-1, 0], [0, 1], [0, -1]]
        .into_iter()
        .map(move |d| [c[0] + d[0], c[1] + d[1]])
        .filter(move |&b| edge_exists(seed, c, b))
}

/// Chunk coordinates of a world position.
fn chunk_of(p: [f64; 2]) -> [i64; 2] {
    [
        (p[0] / CHUNK_M).floor() as i64,
        (p[1] / CHUNK_M).floor() as i64,
    ]
}

/// Extend a node walk with a random next street, avoiding an immediate
/// backtrack when the intersection offers any other way out.
fn random_next(seed: u64, walk: &[[i64; 2]], rng: &mut Rng) -> [i64; 2] {
    let at = *walk.last().unwrap();
    let back = walk.len().checked_sub(2).map(|i| walk[i]);
    let choices: Vec<[i64; 2]> = neighbors(seed, at).filter(|&v| Some(v) != back).collect();
    match choices.len() {
        0 => back.unwrap(),
        k => choices[(rng.uniform() * k as f64) as usize % k],
    }
}

// --- junction furniture ----------------------------------------------------
//
// Turn pockets, slip lanes, and crosswalks are all keyed on (junction grid
// coordinate, approach direction) — pure functions of the seed, like the
// network itself, so drivers, pedestrians, and the viewer's drawing always
// agree on where they are.

fn dir_idx(d: [i64; 2]) -> u64 {
    match d {
        [1, 0] => 0,
        [0, 1] => 1,
        [-1, 0] => 2,
        _ => 3,
    }
}

/// Does the approach to junction `j` from grid direction `d_in` widen into
/// a left-turn pocket? More likely downtown.
pub fn has_pocket(seed: u64, j: [i64; 2], d_in: [i64; 2]) -> bool {
    unit(hash(seed, j[0], j[1], 0xB0C0 + dir_idx(d_in)))
        < 0.25 + 0.5 * urban(seed, node_pos(seed, j))
}

/// Does the right turn out of junction `j`, approached from `d_in`, have a
/// slip lane — a wide corner-cutting curve past the junction proper?
pub fn has_slip(seed: u64, j: [i64; 2], d_in: [i64; 2]) -> bool {
    unit(hash(seed, j[0], j[1], 0x51B0 + dir_idx(d_in))) < 0.18
}

/// Is there a crosswalk across the approach to junction `j` from `d_in`
/// (at [`CROSSWALK_SETBACK_M`] before the node)? More likely downtown.
pub fn has_crosswalk(seed: u64, j: [i64; 2], d_in: [i64; 2]) -> bool {
    unit(hash(seed, j[0], j[1], 0xC205 + dir_idx(d_in)))
        < 0.2 + 0.6 * urban(seed, node_pos(seed, j))
}

/// Does pedestrian `id` cross at the crosswalk into `j` from `d_in`, if
/// one exists? Deterministic per (pedestrian, junction), so path rebuilds
/// mid-walk never flip an already-planned crossing under its feet.
fn ped_crosses(seed: u64, id: u64, j: [i64; 2], d_in: [i64; 2]) -> bool {
    has_crosswalk(seed, j, d_in) && unit(hash(id, j[0], j[1], 0xC407)) < 0.35
}

/// Grid coordinate of the node at position `p` — jitter is well under half
/// a grid step, so rounding recovers it exactly.
fn coord_near(p: [f64; 2]) -> [i64; 2] {
    [
        (p[0] / GRID_SPACING_M).round() as i64,
        (p[1] / GRID_SPACING_M).round() as i64,
    ]
}

/// Grid direction of travel from `a` to `b` (dominant axis).
fn grid_dir(a: [f64; 2], b: [f64; 2]) -> [i64; 2] {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    if dx.abs() >= dy.abs() {
        [dx.signum() as i64, 0]
    } else {
        [0, dy.signum() as i64]
    }
}

/// Turn direction at `v` coming from `a` and leaving toward `b`:
/// +1 left, -1 right, 0 near-straight.
fn turn_dir(a: [f64; 2], v: [f64; 2], b: [f64; 2]) -> i8 {
    let d0 = [v[0] - a[0], v[1] - a[1]];
    let d1 = [b[0] - v[0], b[1] - v[1]];
    let cross = d0[0] * d1[1] - d0[1] * d1[0];
    let lim = 0.35 * d0[0].hypot(d0[1]) * d1[0].hypot(d1[1]);
    (cross > lim) as i8 - (cross < -lim) as i8
}

/// Assemble the polyline a road user follows along `axis` (leg `i` runs
/// `axis[i]→axis[i+1]` with `lanes[i]` lanes per direction): keep right,
/// move to the innermost lane to approach a left turn (via the turn pocket
/// where one exists, while through traffic deflects around it and bikes and
/// sidewalks follow the flared curb), take slip-lane right turns on the
/// wide radius — and, for a pedestrian `ped = (id, side)` (`side` = the
/// sidewalk sign at `axis[0]`, +1 right of travel), cross to the other
/// sidewalk at the crosswalks [`ped_crosses`] picks.
fn corridor(
    seed: u64,
    axis: &[[f64; 2]],
    lanes: &[usize],
    kind: ActorKind,
    ped: Option<(u64, f64)>,
) -> Vec<[f64; 2]> {
    let mut side = ped.map_or(1.0, |(_, s)| s);
    let mut pts = vec![axis[0]];
    let (mut offs, mut radii) = (vec![], vec![CORNER_RADIUS_M]);
    for i in 0..lanes.len() {
        let (a, b) = (axis[i], axis[i + 1]);
        let (len, n) = (dist(a, b), lanes[i] as f64);
        // legs ending at a junction (not the walk/route endpoint) get the
        // junction furniture
        let junction = i + 2 < axis.len();
        let (d_in, j) = (grid_dir(a, b), coord_near(b));
        let turn = if junction {
            turn_dir(a, b, axis[i + 2])
        } else {
            0
        };
        let pocket = junction && len > 45.0 && has_pocket(seed, j, d_in);
        let split_at = |u_m: f64| {
            [
                a[0] + (b[0] - a[0]) * u_m / len,
                a[1] + (b[1] - a[1]) * u_m / len,
            ]
        };
        let base = match kind {
            ActorKind::Car | ActorKind::Truck => {
                if turn > 0 && !pocket && lanes[i] >= 2 {
                    0.5 * LANE_W_M // no pocket: approach in the inner lane
                } else {
                    (n - 0.5) * LANE_W_M
                }
            }
            ActorKind::Bike => n * LANE_W_M - 1.0,
            ActorKind::Pedestrian => side * (n * LANE_W_M + 1.8),
        };
        let mut cur = base;
        if pocket {
            let shifted = match kind {
                // left turn: slide into the pocket lane
                ActorKind::Car | ActorKind::Truck if turn > 0 => 0.5 * LANE_W_M,
                // the far sidewalk doesn't flare
                ActorKind::Pedestrian if side < 0.0 => base,
                // deflect around the pocket / follow the flared curb
                _ => base + LANE_W_M,
            };
            pts.push(split_at(len - POCKET_M));
            offs.push(cur);
            radii.push(CORNER_RADIUS_M);
            cur = shifted;
        }
        if let Some((id, _)) = ped
            && junction
            && ped_crosses(seed, id, j, d_in)
        {
            // cross at the crosswalk: a sharp offset flip to the opposite
            // sidewalk just before the junction
            pts.push(split_at(len - CROSSWALK_SETBACK_M));
            offs.push(cur);
            radii.push(2.0);
            side = -side;
            cur = side * (n * LANE_W_M + 1.8);
        }
        pts.push(b);
        offs.push(cur);
        radii.push(
            if junction && turn < 0 && kind != ActorKind::Pedestrian && has_slip(seed, j, d_in) {
                SLIP_RADIUS_M
            } else {
                CORNER_RADIUS_M
            },
        );
    }
    lane_polyline(&pts, &offs, &radii)
}

/// Turn a road-axis polyline into a drivable lane centerline: cut each
/// vertex with a quadratic Bezier of radius `radii[i]` (capped by the
/// adjacent leg lengths), then offset each point to the right of travel by
/// its leg's offset (`offsets[i]` for the leg `axis[i]→axis[i+1]`), blended
/// across corners — so lane gains and losses taper smoothly and junction
/// turns come out as lane connectors from the departure lane to the
/// arrival lane.
fn lane_polyline(axis: &[[f64; 2]], offsets: &[f64], radii: &[f64]) -> Vec<[f64; 2]> {
    let mut rounded: Vec<([f64; 2], f64)> = vec![(axis[0], offsets[0])];
    for i in 1..axis.len().saturating_sub(1) {
        let (a, v, b) = (axis[i - 1], axis[i], axis[i + 1]);
        let r = radii[i].min(0.4 * dist(a, v)).min(0.4 * dist(v, b));
        let pull = |from: [f64; 2]| {
            let d = dist(v, from).max(1e-9);
            [
                v[0] + (from[0] - v[0]) * r / d,
                v[1] + (from[1] - v[1]) * r / d,
            ]
        };
        let (p0, p2) = (pull(a), pull(b));
        for k in 0..=6 {
            let t = k as f64 / 6.0;
            let mt = 1.0 - t;
            rounded.push((
                [
                    mt * mt * p0[0] + 2.0 * mt * t * v[0] + t * t * p2[0],
                    mt * mt * p0[1] + 2.0 * mt * t * v[1] + t * t * p2[1],
                ],
                offsets[i - 1] + (offsets[i] - offsets[i - 1]) * t,
            ));
        }
    }
    if axis.len() > 1 {
        rounded.push((*axis.last().unwrap(), *offsets.last().unwrap()));
    }
    (0..rounded.len())
        .map(|i| {
            let (a, b) = (
                rounded[i.saturating_sub(1)].0,
                rounded[(i + 1).min(rounded.len() - 1)].0,
            );
            let d = dist(a, b).max(1e-9);
            let dir = [(b[0] - a[0]) / d, (b[1] - a[1]) / d];
            let (p, off) = rounded[i];
            // right normal of the direction of travel
            [p[0] + dir[1] * off, p[1] - dir[0] * off]
        })
        .collect()
}

/// The materialized active window of the infinite street network: the 3×3
/// chunks of grid nodes around the ego, instantiated for drawing, snapping,
/// and routing. Regenerating any window over the same seed yields the same
/// streets — the network is a pure function of the seed.
pub struct StreetMap {
    pub seed: u64,
    /// Center chunk of the 3×3 window.
    pub center: [i64; 2],
    /// Grid coordinate of each node — its stable identity across windows.
    pub coords: Vec<[i64; 2]>,
    /// Intersection positions.
    pub nodes: Vec<[f64; 2]>,
    /// Two-way streets between intersections, as node index pairs.
    pub edges: Vec<[usize; 2]>,
    /// Lanes per direction of each street.
    pub lanes: Vec<usize>,
    /// `adj[n]` = neighbours of node `n`.
    adj: Vec<Vec<usize>>,
}

impl StreetMap {
    pub fn window(seed: u64, center: [i64; 2]) -> Self {
        let lo = [(center[0] - 1) * CHUNK_NODES, (center[1] - 1) * CHUNK_NODES];
        let side = 3 * CHUNK_NODES;
        let mut coords = vec![];
        let mut index = HashMap::new();
        for gy in 0..side {
            for gx in 0..side {
                let c = [lo[0] + gx, lo[1] + gy];
                index.insert(c, coords.len());
                coords.push(c);
            }
        }
        let nodes: Vec<[f64; 2]> = coords.iter().map(|&c| node_pos(seed, c)).collect();
        let (mut edges, mut lanes) = (vec![], vec![]);
        for (i, &c) in coords.iter().enumerate() {
            for d in [[1, 0], [0, 1]] {
                let b = [c[0] + d[0], c[1] + d[1]];
                if let Some(&j) = index.get(&b)
                    && edge_exists(seed, c, b)
                {
                    edges.push([i, j]);
                    lanes.push(edge_lanes(seed, c, b));
                }
            }
        }
        let mut adj = vec![vec![]; nodes.len()];
        for &[a, b] in &edges {
            adj[a].push(b);
            adj[b].push(a);
        }
        StreetMap {
            seed,
            center,
            coords,
            nodes,
            edges,
            lanes,
            adj,
        }
    }

    /// Half-width of street `e`: lane count times the lane width.
    pub fn half_width(&self, e: usize) -> f64 {
        self.lanes[e] as f64 * LANE_W_M
    }

    /// Nearest point on the street network to `p`: (edge index, point).
    pub fn snap(&self, p: [f64; 2]) -> (usize, [f64; 2]) {
        let mut best = (0, p, f64::INFINITY);
        for (i, &[a, b]) in self.edges.iter().enumerate() {
            let (na, nb) = (self.nodes[a], self.nodes[b]);
            let (dx, dy) = (nb[0] - na[0], nb[1] - na[1]);
            let len2 = (dx * dx + dy * dy).max(1e-9);
            let u = (((p[0] - na[0]) * dx + (p[1] - na[1]) * dy) / len2).clamp(0.0, 1.0);
            let q = [na[0] + dx * u, na[1] + dy * u];
            let d = dist(p, q);
            if d < best.2 {
                best = (i, q, d);
            }
        }
        (best.0, best.1)
    }

    /// Shortest route from `from` (facing `yaw`) to `to`, as a lane
    /// centerline polyline with rounded corners — ready to be a [`Road`]
    /// centerline. Both endpoints are snapped to the network; routes that
    /// would start against the ego's heading pay [`U_TURN_PENALTY_M`], so
    /// the ego prefers going around the block. Multi-lane legs keep right,
    /// except approaching a left turn, where the route moves into the
    /// innermost (turning) lane.
    pub fn route(&self, from: [f64; 2], yaw: f64, to: [f64; 2]) -> Vec<[f64; 2]> {
        let (se, sp) = self.snap(from);
        let (ge, mut gp) = self.snap(to);
        let heading = [yaw.cos(), yaw.sin()];
        let seed_cost = |n: usize| {
            let d = [self.nodes[n][0] - sp[0], self.nodes[n][1] - sp[1]];
            let behind = d[0] * heading[0] + d[1] * heading[1] < 0.0;
            dist(sp, self.nodes[n]) + if behind { U_TURN_PENALTY_M } else { 0.0 }
        };
        // Dijkstra over intersections, seeded with both ends of the start
        // edge (the window is small, so the O(n^2) scan needs no heap)
        let n = self.nodes.len();
        let (mut cost, mut pred, mut done) =
            (vec![f64::INFINITY; n], vec![usize::MAX; n], vec![false; n]);
        for s in self.edges[se] {
            cost[s] = seed_cost(s);
        }
        for _ in 0..n {
            let Some(u) = (0..n)
                .filter(|&i| !done[i] && cost[i].is_finite())
                .min_by(|&a, &b| cost[a].total_cmp(&cost[b]))
            else {
                break;
            };
            done[u] = true;
            for &v in &self.adj[u] {
                let c = cost[u] + dist(self.nodes[u], self.nodes[v]);
                if c < cost[v] {
                    cost[v] = c;
                    pred[v] = u;
                }
            }
        }
        // arrive via whichever end of the goal edge is cheaper overall
        let mut end = self.edges[ge]
            .into_iter()
            .min_by(|&a, &b| {
                (cost[a] + dist(self.nodes[a], gp)).total_cmp(&(cost[b] + dist(self.nodes[b], gp)))
            })
            .unwrap();
        if !cost[end].is_finite() {
            // ponytail: a window seam can orphan a street from the rest of
            // the loaded grid; drive to the nearest reachable node instead
            end = (0..n)
                .filter(|&i| cost[i].is_finite())
                .min_by(|&a, &b| dist(self.nodes[a], gp).total_cmp(&dist(self.nodes[b], gp)))
                .unwrap();
            gp = self.nodes[end];
        }
        let mut chain = vec![end];
        while pred[*chain.last().unwrap()] != usize::MAX {
            chain.push(pred[*chain.last().unwrap()]);
        }
        chain.reverse();
        let via_nodes = cost[end] + dist(self.nodes[end], gp);
        // start and goal on the same street with nothing forward of them to
        // route through: drive it directly
        let mut axis: Vec<[f64; 2]> = if se == ge && seed_cost_direct(sp, gp, heading) < via_nodes {
            vec![sp, gp]
        } else {
            std::iter::once(sp)
                .chain(chain.into_iter().map(|i| self.nodes[i]))
                .chain(std::iter::once(gp))
                .collect()
        };
        // drop points that collapse onto their predecessor (start/goal
        // snapped right next to an intersection), always keeping the goal
        let raw = std::mem::take(&mut axis);
        axis.push(raw[0]);
        for (i, &p) in raw.iter().enumerate().skip(1) {
            if dist(*axis.last().unwrap(), p) >= 2.0 || axis.len() == 1 {
                axis.push(p);
            } else if i == raw.len() - 1 {
                *axis.last_mut().unwrap() = p;
            }
        }
        // lane count of each leg, looked up from the street it runs along
        let lanes: Vec<usize> = axis
            .windows(2)
            .map(|w| self.lanes[self.snap(mid(w[0], w[1])).0])
            .collect();
        corridor(self.seed, &axis, &lanes, ActorKind::Car, None)
    }
}

/// Direct same-street route cost, with the same U-turn penalty as
/// [`StreetMap::route`]'s Dijkstra seeds.
fn seed_cost_direct(sp: [f64; 2], gp: [f64; 2], heading: [f64; 2]) -> f64 {
    let d = [gp[0] - sp[0], gp[1] - sp[1]];
    let behind = d[0] * heading[0] + d[1] * heading[1] < 0.0;
    dist(sp, gp) + if behind { U_TURN_PENALTY_M } else { 0.0 }
}

/// What kind of road user a [`SmartActor`] is: sets its footprint, speed,
/// and where in the road corridor it travels.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActorKind {
    Car,
    Truck,
    Bike,
    Pedestrian,
}

impl ActorKind {
    /// Footprint (length, width) in meters.
    pub fn size_m(self) -> [f64; 2] {
        match self {
            ActorKind::Car => [4.6, 1.9],
            ActorKind::Truck => [9.5, 2.5],
            ActorKind::Bike => [1.8, 0.6],
            ActorKind::Pedestrian => [0.6, 0.6],
        }
    }

    fn speed_range(self) -> (f64, f64) {
        match self {
            ActorKind::Car => (5.0, 9.0),
            ActorKind::Truck => (4.0, 7.0),
            ActorKind::Bike => (2.5, 4.5),
            ActorKind::Pedestrian => (0.9, 1.6),
        }
    }
}

/// A traffic actor that wanders the street network: it follows its own
/// right-hand corridor (lane, curb, or sidewalk by [`ActorKind`]), holds
/// speed with the same IDM the Bezier planner uses, yields behind anything
/// ahead in its corridor, and brakes for anything about to be in front of
/// it (crossing traffic, the ego).
pub struct SmartActor {
    pub kind: ActorKind,
    pub state: State,
    /// Stable identity, for per-actor deterministic choices (crosswalks).
    id: u64,
    /// Chunk that spawned it; one spawn set lives per populated chunk.
    home: [i64; 2],
    /// World time it left the active bounds, if it has (despawn grace).
    out_since: Option<f64>,
    /// Node walk (grid coords) the path is built from; extended as consumed.
    walk: Vec<[i64; 2]>,
    path: Path,
    s: f64,
    target_speed: f64,
    /// Sidewalk sign (±1, right of travel) at the start of the current
    /// walk — pedestrians flip it by crossing at crosswalks.
    side: f64,
}

/// The deterministic traffic of one chunk: count and mix follow the
/// urbanization field (downtown: more of everything, especially pedestrians;
/// rural: sparse, more trucks). A pure function of (seed, chunk), so a
/// despawned chunk repopulates identically when revisited.
fn spawn_chunk(seed: u64, chunk: [i64; 2]) -> Vec<SmartActor> {
    let mut rng = Rng(hash(seed, chunk[0], chunk[1], 0x5FA1) | 1);
    let u = urban(
        seed,
        [
            (chunk[0] as f64 + 0.5) * CHUNK_M,
            (chunk[1] as f64 + 0.5) * CHUNK_M,
        ],
    );
    let count = 2 + (u * 5.0) as usize;
    (0..count)
        .map(|_| {
            let roll = rng.uniform();
            let (ped, bike, truck) = (0.08 + 0.30 * u, 0.12, 0.25 - 0.15 * u);
            let kind = if roll < ped {
                ActorKind::Pedestrian
            } else if roll < ped + bike {
                ActorKind::Bike
            } else if roll < ped + bike + truck {
                ActorKind::Truck
            } else {
                ActorKind::Car
            };
            SmartActor::spawn(seed, chunk, kind, &mut rng)
        })
        .collect()
}

impl SmartActor {
    fn spawn(seed: u64, chunk: [i64; 2], kind: ActorKind, rng: &mut Rng) -> Self {
        // a random street with an end in this chunk
        let c = [
            chunk[0] * CHUNK_NODES + (rng.uniform() * CHUNK_NODES as f64) as i64,
            chunk[1] * CHUNK_NODES + (rng.uniform() * CHUNK_NODES as f64) as i64,
        ];
        let nbs: Vec<_> = neighbors(seed, c).collect();
        let mut walk = vec![
            c,
            nbs[(rng.uniform() * nbs.len() as f64) as usize % nbs.len()],
        ];
        while walk.len() < 5 {
            let next = random_next(seed, &walk, rng);
            walk.push(next);
        }
        let (lo, hi) = kind.speed_range();
        let mut actor = SmartActor {
            kind,
            state: State::default(),
            id: rng.uniform().to_bits(),
            home: chunk,
            out_since: None,
            walk,
            path: Path::new(&[[0.0, 0.0], [1.0, 0.0]]),
            s: 0.0,
            target_speed: rng.range(lo, hi),
            side: 1.0,
        };
        actor.rebuild_path(seed);
        actor.s = rng.range(0.1, 0.4) * actor.path.length();
        let (p, yaw) = actor.path.pose_at(actor.s);
        actor.state = State {
            x: p[0],
            y: p[1],
            yaw,
            speed: rng.range(0.5, 1.0) * actor.target_speed,
        };
        actor
    }

    /// Rebuild the corridor path from the current walk.
    fn rebuild_path(&mut self, seed: u64) {
        let axis: Vec<[f64; 2]> = self.walk.iter().map(|&c| node_pos(seed, c)).collect();
        let lanes: Vec<usize> = self
            .walk
            .windows(2)
            .map(|w| edge_lanes(seed, w[0], w[1]))
            .collect();
        let ped = (self.kind == ActorKind::Pedestrian).then_some((self.id, self.side));
        self.path = Path::new(&corridor(seed, &axis, &lanes, self.kind, ped));
    }

    /// Nearest thing to follow: (gap, its speed along my direction) —
    /// either something ahead in my corridor, or anything close in front of
    /// my bumper regardless of lane (the intersection/crossing guard).
    fn lead(&self, others: &[(State, ActorKind)]) -> Option<(f64, f64)> {
        let me = self.state;
        let follow_gap = self.kind.size_m()[0] / 2.0 + 2.5;
        // bumper guard reach scales with speed: about a stride for a
        // walker, a car length and a half at driving speed
        let reach = 4.0 + 1.2 * me.speed;
        others
            .iter()
            .filter_map(|&(o, okind)| {
                // pedestrians have right of way: they queue behind other
                // pedestrians but never yield to vehicles (which brake for
                // them via their own bumper guard) — otherwise a crossing
                // ped and the car yielding to it deadlock at the crosswalk
                if self.kind == ActorKind::Pedestrian && okind != ActorKind::Pedestrian {
                    return None;
                }
                let (so, d) = self.path.project_near([o.x, o.y], self.s + 30.0, 55.0);
                let along = d.abs() < 2.5 && so > self.s + 0.5 && so - self.s < 80.0;
                let mut gap = along.then_some(so - self.s - follow_gap);
                // bumper guard: a narrow corridor straight ahead in my own
                // frame, so crossing traffic and corner-cutters get braked
                // for — but *not* oncoming traffic in the adjacent lane,
                // which a wide cone would catch, gridlocking whole streets
                let (dx, dy) = (o.x - me.x, o.y - me.y);
                let ahead = dx * me.yaw.cos() + dy * me.yaw.sin();
                let side = dy * me.yaw.cos() - dx * me.yaw.sin();
                if (0.5..reach).contains(&ahead) && side.abs() < 3.0 {
                    let g = (ahead - follow_gap).max(0.0);
                    gap = Some(gap.map_or(g, |x: f64| x.min(g)));
                }
                gap.map(|g| (g, o.speed * (o.yaw - me.yaw).cos()))
            })
            .min_by(|a, b| a.0.total_cmp(&b.0))
    }

    /// Advance one tick, reacting to a snapshot of every other road user.
    fn step(&mut self, seed: u64, others: &[(State, ActorKind)], dt: f64, rng: &mut Rng) {
        if self.s > self.path.length() - 60.0 {
            self.extend(seed, rng);
        }
        let accel = idm_accel(self.state.speed, self.target_speed, self.lead(others));
        self.state.speed = (self.state.speed + accel * dt).max(0.0);
        self.s += self.state.speed * dt;
        let (p, yaw) = self.path.pose_at(self.s);
        (self.state.x, self.state.y, self.state.yaw) = (p[0], p[1], yaw);
    }

    /// Grow the node walk ahead and drop the streets already driven, then
    /// rebuild the corridor path and re-find our place on it.
    fn extend(&mut self, seed: u64, rng: &mut Rng) {
        // keep the street we're on: find the walk segment nearest to us
        let pos = [self.state.x, self.state.y];
        let at = (0..self.walk.len() - 1)
            .min_by(|&i, &j| {
                let m = |k: usize| {
                    mid(
                        node_pos(seed, self.walk[k]),
                        node_pos(seed, self.walk[k + 1]),
                    )
                };
                dist(pos, m(i)).total_cmp(&dist(pos, m(j)))
            })
            .unwrap_or(0);
        // crossings already walked flip which sidewalk the rebuilt path
        // starts on (deterministic per junction, so the path never jumps
        // under the pedestrian's feet)
        if self.kind == ActorKind::Pedestrian {
            for k in 0..at {
                let (a, j) = (self.walk[k], self.walk[k + 1]);
                if ped_crosses(seed, self.id, j, [j[0] - a[0], j[1] - a[1]]) {
                    self.side = -self.side;
                }
            }
        }
        self.walk.drain(..at);
        while self.walk.len() < 6 {
            let next = random_next(seed, &self.walk, rng);
            self.walk.push(next);
        }
        self.rebuild_path(seed);
        self.s = self.path.project(pos).0;
    }
}

/// Ticks of planned future kept for the on-screen plan preview.
const PLAN_PREVIEW_TICKS: usize = 30;

/// Comfortable decel used to taper the target speed into the goal.
const GOAL_DECEL: f64 = 1.5;

/// The realtime interactive world: the active street window, the ego
/// (replanned and stepped every tick), the traffic, and the user's current
/// goal. The caller (the viewer's open-world mode) calls
/// [`tick`](LiveWorld::tick) at a fixed rate and
/// [`set_goal`](LiveWorld::set_goal) whenever the user clicks; with no goal
/// the ego brakes to a stop and waits.
pub struct LiveWorld {
    seed: u64,
    /// The 3×3-chunk window around the ego, recentered (with hysteresis) as
    /// it drives — so the drivable world is effectively infinite.
    pub map: StreetMap,
    pub ego: State,
    pub actors: Vec<SmartActor>,
    /// Where the ego is headed (snapped to the network), if anywhere.
    pub goal: Option<[f64; 2]>,
    /// The route to the goal as a planning road; `None` when goalless.
    pub road: Option<Road>,
    /// Planned ego states for the preview overlay (empty when goalless).
    pub plan: Vec<State>,
    /// Wall-clock cost of the most recent `plan()` call.
    pub last_plan_ms: f64,
    /// Cruise speed for the next route; live-tunable from the UI.
    pub target_speed: f64,
    pub dt: f64,
    route_path: Option<Path>,
    planner_kind: PlannerKind,
    planner: Box<dyn Planner>,
    rng: Rng,
    /// World clock, for the despawn grace timing.
    t: f64,
    /// Global cap on live actors (0 = no traffic).
    max_actors: usize,
}

impl LiveWorld {
    pub fn new(seed: u64, planner: PlannerKind, max_actors: usize, dt: f64) -> Self {
        let mut rng = Rng(seed.wrapping_mul(0x2545F4914F6CDD1D) | 1);
        // ego at rest mid-way along a random street near the origin, in the
        // rightmost lane
        let c = [
            (rng.uniform() * CHUNK_NODES as f64) as i64,
            (rng.uniform() * CHUNK_NODES as f64) as i64,
        ];
        let nbs: Vec<_> = neighbors(seed, c).collect();
        let b = nbs[(rng.uniform() * nbs.len() as f64) as usize % nbs.len()];
        let (pa, pb) = (node_pos(seed, c), node_pos(seed, b));
        let d = dist(pa, pb).max(1e-9);
        let dir = [(pb[0] - pa[0]) / d, (pb[1] - pa[1]) / d];
        let off = (edge_lanes(seed, c, b) as f64 - 0.5) * LANE_W_M;
        let ego = State {
            x: (pa[0] + pb[0]) / 2.0 + dir[1] * off,
            y: (pa[1] + pb[1]) / 2.0 - dir[0] * off,
            yaw: dir[1].atan2(dir[0]),
            speed: 0.0,
        };
        let mut world = LiveWorld {
            seed,
            map: StreetMap::window(seed, chunk_of([ego.x, ego.y])),
            ego,
            actors: vec![],
            goal: None,
            road: None,
            plan: vec![],
            last_plan_ms: 0.0,
            target_speed: 8.0,
            dt,
            route_path: None,
            planner_kind: planner,
            planner: planner.build(),
            rng,
            t: 0.0,
            max_actors,
        };
        world.populate();
        world
    }

    /// Spawn the deterministic traffic of any active chunk that has no
    /// actors of its own alive — newly loaded chunks (including the preload
    /// buffer ring) and chunks whose traffic despawned. Chunks whose actors
    /// are still alive (however far they've wandered) are left alone, so
    /// re-entering a chunk never double-spawns.
    fn populate(&mut self) {
        let [cx, cy] = self.map.center;
        for dy in -1..=1 {
            for dx in -1..=1 {
                let chunk = [cx + dx, cy + dy];
                if self.actors.iter().any(|a| a.home == chunk) {
                    continue;
                }
                for cand in spawn_chunk(self.seed, chunk) {
                    let p = [cand.state.x, cand.state.y];
                    let clear = self.actors.len() < self.max_actors
                        && dist(p, [self.ego.x, self.ego.y]) > 25.0
                        && self
                            .actors
                            .iter()
                            .all(|o| dist(p, [o.state.x, o.state.y]) > 10.0);
                    if clear {
                        self.actors.push(cand);
                    }
                }
            }
        }
    }

    /// Route from the ego's current pose to (the network point nearest) `p`
    /// and start driving there. A click essentially on top of the ego is
    /// ignored.
    pub fn set_goal(&mut self, p: [f64; 2]) {
        let line = self.map.route([self.ego.x, self.ego.y], self.ego.yaw, p);
        let path = Path::new(&line);
        if path.length() < 5.0 {
            return;
        }
        self.goal = Some(*line.last().unwrap());
        // Give the planner the road's actual half-width — the narrowest
        // street the route runs along — instead of the fixed 5.5 m default.
        // Most local streets are one lane (`LANE_W_M` = 3.5 m half-width),
        // well inside that default, so a route that reported 5.5 m let the
        // planner treat two-plus metres of sidewalk as drivable and plan
        // off the road surface. Taking the min keeps it on even the tightest
        // section. (A single scalar can't capture the route's rightmost-lane
        // offset; the symmetric `Road` model is a pre-existing simplification.)
        let half_width = line
            .iter()
            .map(|&q| self.map.half_width(self.map.snap(q).0))
            .fold(f64::INFINITY, f64::min);
        self.road = Some(Road {
            centerline: line,
            target_speed: self.target_speed,
            half_width,
            dt: self.dt,
        });
        self.route_path = Some(path);
    }

    /// Drop the goal; the ego brakes to a stop and waits for the next one.
    pub fn clear_goal(&mut self) {
        (self.goal, self.road, self.route_path) = (None, None, None);
        self.plan.clear();
    }

    /// Route distance left to the goal, if one is set.
    pub fn remaining_m(&self) -> Option<f64> {
        let path = self.route_path.as_ref()?;
        Some(path.length() - path.project([self.ego.x, self.ego.y]).0)
    }

    /// Swap the planner (fresh instance; warm starts don't carry across
    /// planner kinds). No-op if `kind` is already running.
    pub fn set_planner(&mut self, kind: PlannerKind) {
        if kind != self.planner_kind {
            self.planner_kind = kind;
            self.planner = kind.build();
        }
    }

    /// Advance the whole world one tick: recenter the chunk window if the
    /// ego has left it, spawn/despawn traffic at the edges, step every
    /// actor against a snapshot of the traffic (ego included), then replan
    /// and step the ego — or brake to a stop if there's no goal.
    pub fn tick(&mut self) {
        self.t += self.dt;
        // recenter once the ego is decisively outside the center chunk
        let [cx, cy] = self.map.center;
        let outside = self.ego.x < cx as f64 * CHUNK_M - RECENTER_HYST_M
            || self.ego.x > (cx + 1) as f64 * CHUNK_M + RECENTER_HYST_M
            || self.ego.y < cy as f64 * CHUNK_M - RECENTER_HYST_M
            || self.ego.y > (cy + 1) as f64 * CHUNK_M + RECENTER_HYST_M;
        if outside {
            self.map = StreetMap::window(self.seed, chunk_of([self.ego.x, self.ego.y]));
            self.populate();
        }
        // despawn actors that stay outside the active bounds past the grace
        let [cx, cy] = self.map.center;
        let lo = [
            (cx - 1) as f64 * CHUNK_M - DESPAWN_MARGIN_M,
            (cy - 1) as f64 * CHUNK_M - DESPAWN_MARGIN_M,
        ];
        let hi = [
            (cx + 2) as f64 * CHUNK_M + DESPAWN_MARGIN_M,
            (cy + 2) as f64 * CHUNK_M + DESPAWN_MARGIN_M,
        ];
        let t = self.t;
        self.actors.retain_mut(|a| {
            let inside = (lo[0]..hi[0]).contains(&a.state.x) && (lo[1]..hi[1]).contains(&a.state.y);
            if inside {
                a.out_since = None;
                true
            } else {
                t - *a.out_since.get_or_insert(t) < DESPAWN_GRACE_S
            }
        });
        let snapshot: Vec<(State, ActorKind)> = std::iter::once((self.ego, ActorKind::Car))
            .chain(self.actors.iter().map(|a| (a.state, a.kind)))
            .collect();
        for (i, actor) in self.actors.iter_mut().enumerate() {
            let others: Vec<(State, ActorKind)> = snapshot
                .iter()
                .enumerate()
                .filter_map(|(j, &s)| (j != i + 1).then_some(s))
                .collect();
            actor.step(self.seed, &others, self.dt, &mut self.rng);
        }
        let actor_states: Vec<State> = self.actors.iter().map(|a| a.state).collect();
        let Some(road) = &mut self.road else {
            // goalless: brake smoothly to a stop and wait for a click
            let accel = (-2.0f64).max(-self.ego.speed / self.dt);
            self.ego = step(
                self.ego,
                Control {
                    accel,
                    curvature: 0.0,
                },
                self.dt,
            );
            self.plan.clear();
            return;
        };
        // taper the target speed so the ego arrives stopped instead of
        // sailing through the goal at cruise speed
        let path = self.route_path.as_ref().unwrap();
        let remaining = path.length() - path.project([self.ego.x, self.ego.y]).0;
        road.target_speed = self
            .target_speed
            .min((2.0 * GOAL_DECEL * remaining).max(0.0).sqrt());
        let ctx = Context {
            road,
            actors: &actor_states,
            horizon: PLAN_PREVIEW_TICKS,
            latency: None,
            diagnostics: None,
        };
        let t0 = Instant::now();
        let controls = self.planner.plan(self.ego, &ctx);
        self.last_plan_ms = t0.elapsed().as_secs_f64() * 1e3;
        let mut s = self.ego;
        self.plan = controls
            .iter()
            .take(PLAN_PREVIEW_TICKS)
            .map(|&u| {
                s = step(s, u, self.dt);
                s
            })
            .collect();
        self.ego = step(
            self.ego,
            controls.first().copied().unwrap_or_default(),
            self.dt,
        );
        if remaining < 4.0 && self.ego.speed < 0.5 {
            (self.goal, self.road, self.route_path) = (None, None, None);
            self.plan.clear();
        }
    }
}

#[cfg(test)]
mod tests {
    use std::collections::HashSet;

    use super::*;

    #[test]
    fn windows_are_deterministic_and_seamless() {
        let (a, b) = (StreetMap::window(7, [0, 0]), StreetMap::window(7, [0, 0]));
        assert_eq!(a.nodes, b.nodes);
        assert_eq!(a.edges, b.edges);
        assert_eq!(a.lanes, b.lanes);
        // enough streets to be an interesting map
        assert!(a.edges.len() > a.nodes.len() / 2);
        // a shifted window agrees street for street where they overlap:
        // the map is a pure function of the seed, not of the window
        let shifted = StreetMap::window(7, [1, 0]);
        let streets = |m: &StreetMap| -> HashSet<([i64; 2], [i64; 2], usize)> {
            m.edges
                .iter()
                .zip(&m.lanes)
                .map(|(&[i, j], &n)| (m.coords[i], m.coords[j], n))
                .collect()
        };
        let in_overlap =
            |c: [i64; 2]| (0..8).contains(&c[0]) && (-CHUNK_NODES..2 * CHUNK_NODES).contains(&c[1]);
        let (sa, sb) = (streets(&a), streets(&shifted));
        let mut overlapping = 0;
        for e in &sb {
            if in_overlap(e.0) && in_overlap(e.1) {
                assert!(sa.contains(e), "seam mismatch on street {e:?}");
                overlapping += 1;
            }
        }
        assert!(overlapping > 20, "only {overlapping} streets in overlap");
    }

    #[test]
    fn roads_vary_in_width_and_traffic_in_kind() {
        let map = StreetMap::window(11, [0, 0]);
        assert!(map.lanes.iter().any(|&n| n == 1));
        assert!(map.lanes.iter().any(|&n| n >= 2));
        let kinds: Vec<ActorKind> = (-3..3)
            .flat_map(|i| spawn_chunk(11, [i, -i]))
            .map(|a| a.kind)
            .collect();
        assert!(kinds.contains(&ActorKind::Car));
        assert!(kinds.iter().any(|&k| k != ActorKind::Car));
    }

    #[test]
    fn routes_stay_on_the_road_and_reach_the_goal() {
        let map = StreetMap::window(3, [0, 0]);
        let (from, to) = ([-180.0, -190.0], [520.0, 610.0]);
        let line = map.route(from, 0.0, to);
        let (_, goal) = map.snap(to);
        assert!(dist(*line.last().unwrap(), goal) < 3.0 * LANE_W_M + 1e-6);
        // every point of the lane polyline stays within the local street's
        // half-width (plus corner-cut slack at junctions)
        for &p in &line {
            let (e, q) = map.snap(p);
            assert!(
                dist(p, q) <= map.half_width(e) + 0.6 * SLIP_RADIUS_M,
                "{p:?} is {} m off a {}-lane road",
                dist(p, q),
                map.lanes[e]
            );
        }
    }

    #[test]
    fn actors_cruise_alone_and_stop_behind_a_blocker() {
        let seed = 5;
        let mut rng = Rng(9);
        let mut free = spawn_chunk(seed, [0, 0]).remove(0);
        for _ in 0..600 {
            free.step(seed, &[], 0.1, &mut rng);
        }
        assert!(
            (free.state.speed - free.target_speed).abs() < 0.5,
            "speed {}",
            free.state.speed
        );

        // park a blocker (a pedestrian: every kind yields to those)
        // 30 m ahead on the actor's own corridor
        let mut actor = spawn_chunk(seed, [1, 1]).remove(0);
        let (p, yaw) = actor.path.pose_at(actor.s + 30.0);
        let blocker = State {
            x: p[0],
            y: p[1],
            yaw,
            speed: 0.0,
        };
        for _ in 0..600 {
            actor.step(seed, &[(blocker, ActorKind::Pedestrian)], 0.1, &mut rng);
        }
        assert!(actor.state.speed < 0.3, "speed {}", actor.state.speed);
        let gap = dist([actor.state.x, actor.state.y], [blocker.x, blocker.y]);
        assert!(gap > 2.0, "gap {gap}");
    }

    #[test]
    fn chunk_churn_keeps_actors_stable_then_prunes_them() {
        let mut w = LiveWorld::new(4, PlannerKind::Straight, 64, 0.1);
        assert!(!w.actors.is_empty());
        let n0 = w.actors.len();
        let start = w.ego;
        let homes: HashSet<[i64; 2]> = w.actors.iter().map(|a| a.home).collect();
        // dart two chunks over and straight back: the despawn grace keeps
        // every original actor alive, and home tracking prevents re-spawns
        w.ego.x += 2.0 * CHUNK_M;
        w.tick();
        w.ego = start;
        w.tick();
        assert_eq!(
            w.actors.iter().filter(|a| homes.contains(&a.home)).count(),
            n0,
            "chunk-line dithering flickered the original traffic"
        );
        // stay away past the grace period: far traffic is dropped
        w.ego.x += 2.0 * CHUNK_M;
        for _ in 0..50 {
            w.tick();
        }
        let [cx, cy] = w.map.center;
        for a in &w.actors {
            // grace lets a despawning actor coast a bit past the margin,
            // never chunks away
            assert!(
                a.state.x > (cx - 1) as f64 * CHUNK_M - 70.0
                    && a.state.x < (cx + 2) as f64 * CHUNK_M + 70.0
                    && a.state.y > (cy - 1) as f64 * CHUNK_M - 70.0
                    && a.state.y < (cy + 2) as f64 * CHUNK_M + 70.0,
                "actor left behind at ({}, {})",
                a.state.x,
                a.state.y
            );
        }
    }

    #[test]
    fn live_world_drives_to_a_clicked_goal_and_stops() {
        let mut w = LiveWorld::new(11, PlannerKind::BezierIdm, 0, 0.1);
        // a goal ~80 m straight ahead along the ego's own street
        let goal = [
            w.ego.x + 80.0 * w.ego.yaw.cos(),
            w.ego.y + 80.0 * w.ego.yaw.sin(),
        ];
        w.set_goal(goal);
        assert!(w.road.is_some());
        for _ in 0..600 {
            w.tick();
            if w.goal.is_none() {
                break;
            }
        }
        assert!(w.goal.is_none(), "never reached the goal");
        assert!(w.ego.speed < 0.5, "still moving at {}", w.ego.speed);
        let snapped = w.map.snap(goal).1;
        assert!(
            dist([w.ego.x, w.ego.y], snapped) < 12.0,
            "stopped {} m from the goal",
            dist([w.ego.x, w.ego.y], snapped)
        );
    }

    /// Signed lateral offset of `line` from the axis point at station `s_m`
    /// along the ray `a→dir` (positive = right of travel).
    fn lateral_near(line: &[[f64; 2]], a: [f64; 2], dir: [f64; 2], s_m: f64) -> f64 {
        let q = [a[0] + dir[0] * s_m, a[1] + dir[1] * s_m];
        // project's lateral is positive when the query is left of the path,
        // i.e. when the path runs right of the axis
        Path::new(line).project(q).1
    }

    /// Eastbound approach geometry into junction `j`: (axis start, unit
    /// direction, leg length).
    fn east_leg(seed: u64, j: [i64; 2]) -> ([f64; 2], [f64; 2], f64) {
        let (a, v) = (node_pos(seed, [j[0] - 1, j[1]]), node_pos(seed, j));
        let len = dist(a, v);
        (
            [a[0], a[1]],
            [(v[0] - a[0]) / len, (v[1] - a[1]) / len],
            len,
        )
    }

    #[test]
    fn turn_pockets_deflect_through_traffic() {
        let seed = 21;
        // an eastbound approach with a pocket, driven straight through
        let j = (0..99)
            .map(|x| [x, 3])
            .find(|&j| has_pocket(seed, j, [1, 0]))
            .unwrap();
        let axis = [
            node_pos(seed, [j[0] - 1, j[1]]),
            node_pos(seed, j),
            node_pos(seed, [j[0] + 1, j[1]]),
        ];
        let line = corridor(seed, &axis, &[1, 1], ActorKind::Car, None);
        let (a, dir, len) = east_leg(seed, j);
        // mid-block: the ordinary rightmost lane; entering the junction:
        // deflected one lane right, around the left-turn pocket
        let mid_off = lateral_near(&line, a, dir, 0.45 * len);
        // measured inside the pocket zone but before the through lanes
        // start merging back across the junction
        let end_off = lateral_near(&line, a, dir, len - 16.0);
        assert!((mid_off - 0.5 * LANE_W_M).abs() < 1.0, "mid {mid_off}");
        assert!((end_off - 1.5 * LANE_W_M).abs() < 1.2, "end {end_off}");
    }

    #[test]
    fn slip_lanes_widen_right_turns() {
        let seed = 21;
        let apex = |j: [i64; 2]| {
            // eastbound, turning right (south) at j
            let axis = [
                node_pos(seed, [j[0] - 1, j[1]]),
                node_pos(seed, j),
                node_pos(seed, [j[0], j[1] - 1]),
            ];
            let line = corridor(seed, &axis, &[1, 1], ActorKind::Car, None);
            let v = node_pos(seed, j);
            line.iter()
                .map(|&p| dist(p, v))
                .fold(f64::INFINITY, f64::min)
        };
        let find = |want: bool| {
            (0..99)
                .map(|x| [x, 5])
                .find(|&j| has_slip(seed, j, [1, 0]) == want)
                .unwrap()
        };
        // the slip-lane corner is cut visibly wider than a plain corner
        assert!(apex(find(true)) > apex(find(false)) + 2.0);
    }

    #[test]
    fn pedestrians_cross_at_crosswalks() {
        let seed = 21;
        // a junction with a crosswalk on its eastbound approach, and a
        // pedestrian id that chooses to cross there
        // (pocket-free, so the sidewalk holds its ordinary offset)
        let j = (0..99)
            .map(|x| [x, 7])
            .find(|&j| has_crosswalk(seed, j, [1, 0]) && !has_pocket(seed, j, [1, 0]))
            .unwrap();
        let id = (0..999)
            .find(|&id| ped_crosses(seed, id, j, [1, 0]))
            .unwrap();
        let axis = [
            node_pos(seed, [j[0] - 1, j[1]]),
            node_pos(seed, j),
            node_pos(seed, [j[0] + 1, j[1]]),
        ];
        let line = corridor(seed, &axis, &[1, 1], ActorKind::Pedestrian, Some((id, 1.0)));
        let (a, dir, len) = east_leg(seed, j);
        let sidewalk = LANE_W_M + 1.8;
        // right sidewalk before the crosswalk, left sidewalk after it
        let before = lateral_near(&line, a, dir, len - CROSSWALK_SETBACK_M - 12.0);
        let after = lateral_near(&line, a, dir, len - 4.0);
        assert!((before - sidewalk).abs() < 1.0, "before {before}");
        assert!((after + sidewalk).abs() < 1.0, "after {after}");
        // an id that doesn't cross stays on the right sidewalk
        let id2 = (0..999)
            .find(|&id| !ped_crosses(seed, id, j, [1, 0]))
            .unwrap();
        let stay = corridor(
            seed,
            &axis,
            &[1, 1],
            ActorKind::Pedestrian,
            Some((id2, 1.0)),
        );
        let kept = lateral_near(&stay, a, dir, len - 4.0);
        assert!(kept > 0.0, "kept {kept}");
    }

    #[test]
    fn traffic_keeps_flowing_through_junction_furniture() {
        // dense mixed traffic for a simulated minute: the pedestrian
        // right-of-way rule must prevent crosswalk deadlocks, so average
        // actor speed stays healthy instead of collapsing toward zero
        let mut w = LiveWorld::new(9, PlannerKind::Straight, 64, 0.1);
        assert!(w.actors.len() > 10);
        for _ in 0..500 {
            w.tick();
        }
        let mut moving = 0.0;
        for _ in 0..100 {
            w.tick();
            let mean: f64 =
                w.actors.iter().map(|a| a.state.speed).sum::<f64>() / w.actors.len() as f64;
            moving += mean / 100.0;
        }
        assert!(moving > 1.0, "traffic gridlocked: mean speed {moving}");
    }

    #[test]
    fn ego_drives_across_chunk_seams_indefinitely() {
        // chase a moving goal eastward through live traffic: the window
        // must recenter repeatedly and the closed loop must stay sane
        let mut w = LiveWorld::new(8, PlannerKind::BezierIdm, 64, 0.1);
        let (mut recenters, mut center) = (0, w.map.center);
        for _ in 0..4 {
            w.set_goal([w.ego.x + 300.0, w.ego.y + 60.0]);
            for _ in 0..1200 {
                w.tick();
                if w.map.center != center {
                    (recenters, center) = (recenters + 1, w.map.center);
                }
                if w.goal.is_none() {
                    break;
                }
            }
            assert!(
                w.ego.speed.is_finite() && w.ego.speed.abs() < 30.0,
                "closed loop diverged: speed {}",
                w.ego.speed
            );
        }
        assert!(recenters >= 2, "never crossed a chunk seam");
        assert!(!w.actors.is_empty(), "the world went empty");
    }

    #[test]
    fn goalless_ego_brakes_to_a_stop() {
        let mut w = LiveWorld::new(2, PlannerKind::Straight, 0, 0.1);
        w.ego.speed = 8.0;
        for _ in 0..60 {
            w.tick();
        }
        assert_eq!(w.ego.speed, 0.0);
        assert!(w.plan.is_empty());
    }
}
