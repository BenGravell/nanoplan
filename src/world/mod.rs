//! The interactive open world: a procedurally generated street map, routing
//! to a user-placed goal, basic IDM traffic actors, and the realtime
//! closed-loop stepper ([`LiveWorld`]) the viewer's "open world" mode drives
//! — planner and simulator run every tick, judo/treetop style, instead of
//! precomputing a rollout.

use web_time::Instant;

use crate::Rng;
use crate::planning::{Context, Planner, PlannerKind, bezier_idm::idm_accel};
use crate::scenarios::{Path, Road};
use crate::simulation::{Control, State, step};

/// Half-width of every street — the same drivable-area bound the metrics
/// and the shared cost function enforce around a route centerline.
pub const ROAD_HALF_WIDTH_M: f64 = 5.5;

/// Right-hand traffic: lane centers sit this far right of the road axis.
pub const LANE_OFFSET_M: f64 = 2.0;

/// Corner cut radius when a route turns at an intersection, so the lane
/// centerline stays drivable instead of kinking 90 degrees in place.
const CORNER_RADIUS_M: f64 = 12.0;

/// Extra route cost for starting out against the ego's current heading —
/// enough to prefer going around a couple of blocks over a U-turn (which
/// the planners track with an ugly off-lane loop), without ever making a
/// goal behind the ego unreachable.
const U_TURN_PENALTY_M: f64 = 400.0;

const CAR_LENGTH_M: f64 = 5.0;

fn dist(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

/// A procedurally generated street network: a jittered grid of
/// intersections with some streets removed, every road two-way with one
/// lane each direction. Deterministic in `seed`.
pub struct StreetMap {
    /// Intersection positions.
    pub nodes: Vec<[f64; 2]>,
    /// Two-way streets between intersections, as node index pairs.
    pub edges: Vec<[usize; 2]>,
    /// `adj[n]` = neighbours of node `n`.
    adj: Vec<Vec<usize>>,
}

const GRID_N: usize = 6;
const GRID_SPACING_M: f64 = 100.0;

impl StreetMap {
    pub fn generate(seed: u64) -> Self {
        let mut rng = Rng(seed.wrapping_mul(0x9E3779B97F4A7C15) | 1);
        let half = (GRID_N - 1) as f64 * GRID_SPACING_M / 2.0;
        let nodes: Vec<[f64; 2]> = (0..GRID_N * GRID_N)
            .map(|i| {
                let (gx, gy) = ((i % GRID_N) as f64, (i / GRID_N) as f64);
                let jitter = 0.22 * GRID_SPACING_M;
                [
                    gx * GRID_SPACING_M - half + rng.range(-jitter, jitter),
                    gy * GRID_SPACING_M - half + rng.range(-jitter, jitter),
                ]
            })
            .collect();
        // full grid edges, then randomly drop some — keeping the graph
        // connected — so blocks merge and the map reads as a street layout
        // rather than graph paper
        let mut edges: Vec<[usize; 2]> = vec![];
        for i in 0..GRID_N * GRID_N {
            if i % GRID_N + 1 < GRID_N {
                edges.push([i, i + 1]);
            }
            if i / GRID_N + 1 < GRID_N {
                edges.push([i, i + GRID_N]);
            }
        }
        let mut i = 0;
        while i < edges.len() {
            if rng.uniform() < 0.25 {
                let e = edges.remove(i);
                if !connected(nodes.len(), &edges) {
                    edges.insert(i, e);
                    i += 1;
                }
            } else {
                i += 1;
            }
        }
        let mut adj = vec![vec![]; nodes.len()];
        for &[a, b] in &edges {
            adj[a].push(b);
            adj[b].push(a);
        }
        StreetMap { nodes, edges, adj }
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

    /// Shortest route from `from` (facing `yaw`) to `to`, as a
    /// right-hand-lane centerline polyline with rounded corners — ready to
    /// be a [`Road`] centerline. Both endpoints are snapped to the network;
    /// routes that would start against the ego's heading pay
    /// [`U_TURN_PENALTY_M`], so the ego prefers going around the block.
    pub fn route(&self, from: [f64; 2], yaw: f64, to: [f64; 2]) -> Vec<[f64; 2]> {
        let (se, sp) = self.snap(from);
        let (ge, gp) = self.snap(to);
        let heading = [yaw.cos(), yaw.sin()];
        let seed_cost = |n: usize| {
            let d = [self.nodes[n][0] - sp[0], self.nodes[n][1] - sp[1]];
            let behind = d[0] * heading[0] + d[1] * heading[1] < 0.0;
            dist(sp, self.nodes[n]) + if behind { U_TURN_PENALTY_M } else { 0.0 }
        };
        // Dijkstra over intersections, seeded with both ends of the start
        // edge (the graph is tiny, so the O(n^2) scan needs no heap)
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
        let end = self.edges[ge]
            .into_iter()
            .min_by(|&a, &b| {
                (cost[a] + dist(self.nodes[a], gp)).total_cmp(&(cost[b] + dist(self.nodes[b], gp)))
            })
            .unwrap();
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
        lane_polyline(&axis)
    }

    /// Extend a node walk with a random next street, avoiding an immediate
    /// backtrack when the intersection offers any other way out.
    fn random_next(&self, walk: &[usize], rng: &mut Rng) -> usize {
        let at = *walk.last().unwrap();
        let back = walk.len().checked_sub(2).map(|i| walk[i]);
        let choices: Vec<usize> = self.adj[at]
            .iter()
            .copied()
            .filter(|&v| Some(v) != back)
            .collect();
        match choices.len() {
            0 => back.unwrap(),
            k => choices[(rng.uniform() * k as f64) as usize % k],
        }
    }
}

/// Direct same-street route cost, with the same U-turn penalty as
/// [`StreetMap::route`]'s Dijkstra seeds.
fn seed_cost_direct(sp: [f64; 2], gp: [f64; 2], heading: [f64; 2]) -> f64 {
    let d = [gp[0] - sp[0], gp[1] - sp[1]];
    let behind = d[0] * heading[0] + d[1] * heading[1] < 0.0;
    dist(sp, gp) + if behind { U_TURN_PENALTY_M } else { 0.0 }
}

fn connected(n: usize, edges: &[[usize; 2]]) -> bool {
    let mut adj = vec![vec![]; n];
    for &[a, b] in edges {
        adj[a].push(b);
        adj[b].push(a);
    }
    let mut seen = vec![false; n];
    let mut stack = vec![0];
    seen[0] = true;
    while let Some(u) = stack.pop() {
        for &v in &adj[u] {
            if !seen[v] {
                seen[v] = true;
                stack.push(v);
            }
        }
    }
    seen.into_iter().all(|s| s)
}

/// Turn a road-axis polyline into a drivable right-hand lane centerline:
/// cut each corner with a quadratic Bezier of radius [`CORNER_RADIUS_M`],
/// then offset the whole line [`LANE_OFFSET_M`] to the right of travel.
fn lane_polyline(axis: &[[f64; 2]]) -> Vec<[f64; 2]> {
    let mut rounded = vec![axis[0]];
    for i in 1..axis.len().saturating_sub(1) {
        let (a, v, b) = (axis[i - 1], axis[i], axis[i + 1]);
        let r = CORNER_RADIUS_M.min(0.4 * dist(a, v)).min(0.4 * dist(v, b));
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
            rounded.push([
                mt * mt * p0[0] + 2.0 * mt * t * v[0] + t * t * p2[0],
                mt * mt * p0[1] + 2.0 * mt * t * v[1] + t * t * p2[1],
            ]);
        }
    }
    if axis.len() > 1 {
        rounded.push(*axis.last().unwrap());
    }
    (0..rounded.len())
        .map(|i| {
            let (a, b) = (
                rounded[i.saturating_sub(1)],
                rounded[(i + 1).min(rounded.len() - 1)],
            );
            let d = dist(a, b).max(1e-9);
            let dir = [(b[0] - a[0]) / d, (b[1] - a[1]) / d];
            // right normal of the direction of travel
            [
                rounded[i][0] + dir[1] * LANE_OFFSET_M,
                rounded[i][1] - dir[0] * LANE_OFFSET_M,
            ]
        })
        .collect()
}

/// A traffic actor that wanders the street map: it follows its own
/// right-hand lane, holds speed with the same IDM the Bezier planner uses,
/// yields behind anything ahead in its lane, and brakes for anything about
/// to be in front of its bumper (crossing traffic, the ego).
pub struct SmartActor {
    /// Node walk the lane path is built from; extended as it's consumed.
    walk: Vec<usize>,
    path: Path,
    s: f64,
    target_speed: f64,
    pub state: State,
}

impl SmartActor {
    fn new(map: &StreetMap, rng: &mut Rng) -> Self {
        let e = map.edges[(rng.uniform() * map.edges.len() as f64) as usize % map.edges.len()];
        let mut walk = if rng.uniform() < 0.5 {
            vec![e[0], e[1]]
        } else {
            vec![e[1], e[0]]
        };
        while walk.len() < 5 {
            let next = map.random_next(&walk, rng);
            walk.push(next);
        }
        let path = Path::new(&lane_polyline(
            &walk.iter().map(|&i| map.nodes[i]).collect::<Vec<_>>(),
        ));
        let s = rng.range(0.1, 0.4) * path.length();
        let (p, yaw) = path.pose_at(s);
        let speed = rng.range(3.0, 6.0);
        SmartActor {
            walk,
            s,
            target_speed: rng.range(5.0, 9.0),
            state: State {
                x: p[0],
                y: p[1],
                yaw,
                speed,
            },
            path,
        }
    }

    /// Nearest thing to follow: (gap, its speed along my direction) —
    /// either a vehicle ahead in my lane, or anything close in front of the
    /// bumper regardless of lane (the intersection/crossing guard).
    fn lead(&self, others: &[State]) -> Option<(f64, f64)> {
        let me = self.state;
        others
            .iter()
            .filter_map(|o| {
                let (so, d) = self.path.project_near([o.x, o.y], self.s + 30.0, 55.0);
                let along = d.abs() < 2.5 && so > self.s + 0.5 && so - self.s < 80.0;
                let mut gap = along.then_some(so - self.s - CAR_LENGTH_M);
                // bumper guard: a narrow corridor straight ahead in my own
                // frame, so crossing traffic and corner-cutters get braked
                // for — but *not* oncoming cars in the adjacent lane, which
                // the old wide cone caught, gridlocking whole streets when
                // two opposing actors each braked for the other
                let (dx, dy) = (o.x - me.x, o.y - me.y);
                let ahead = dx * me.yaw.cos() + dy * me.yaw.sin();
                let side = dy * me.yaw.cos() - dx * me.yaw.sin();
                if (0.5..14.0).contains(&ahead) && side.abs() < 3.0 {
                    let g = (ahead - CAR_LENGTH_M).max(0.0);
                    gap = Some(gap.map_or(g, |x: f64| x.min(g)));
                }
                gap.map(|g| (g, o.speed * (o.yaw - me.yaw).cos()))
            })
            .min_by(|a, b| a.0.total_cmp(&b.0))
    }

    /// Advance one tick, reacting to a snapshot of every other vehicle.
    fn step(&mut self, map: &StreetMap, others: &[State], dt: f64, rng: &mut Rng) {
        if self.s > self.path.length() - 60.0 {
            self.extend(map, rng);
        }
        let accel = idm_accel(self.state.speed, self.target_speed, self.lead(others));
        self.state.speed = (self.state.speed + accel * dt).max(0.0);
        self.s += self.state.speed * dt;
        let (p, yaw) = self.path.pose_at(self.s);
        (self.state.x, self.state.y, self.state.yaw) = (p[0], p[1], yaw);
    }

    /// Grow the node walk ahead and drop the streets already driven, then
    /// rebuild the lane path and re-find our place on it.
    fn extend(&mut self, map: &StreetMap, rng: &mut Rng) {
        // keep the street we're on: find the walk segment nearest to us
        let pos = [self.state.x, self.state.y];
        let at = (0..self.walk.len() - 1)
            .min_by(|&i, &j| {
                let mid = |k: usize| {
                    let (a, b) = (map.nodes[self.walk[k]], map.nodes[self.walk[k + 1]]);
                    [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0]
                };
                dist(pos, mid(i)).total_cmp(&dist(pos, mid(j)))
            })
            .unwrap_or(0);
        self.walk.drain(..at);
        while self.walk.len() < 6 {
            let next = map.random_next(&self.walk, rng);
            self.walk.push(next);
        }
        self.path = Path::new(&lane_polyline(
            &self.walk.iter().map(|&i| map.nodes[i]).collect::<Vec<_>>(),
        ));
        self.s = self.path.project(pos).0;
    }
}

/// Ticks of planned future kept for the on-screen plan preview.
const PLAN_PREVIEW_TICKS: usize = 30;

/// Comfortable decel used to taper the target speed into the goal.
const GOAL_DECEL: f64 = 1.5;

/// The realtime interactive world: the street map, the ego (replanned and
/// stepped every tick), the traffic, and the user's current goal. The
/// caller (the viewer's open-world mode) calls [`tick`](LiveWorld::tick) at
/// a fixed rate and [`set_goal`](LiveWorld::set_goal) whenever the user
/// clicks; with no goal the ego brakes to a stop and waits.
pub struct LiveWorld {
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
}

impl LiveWorld {
    pub fn new(seed: u64, planner: PlannerKind, n_actors: usize, dt: f64) -> Self {
        let map = StreetMap::generate(seed);
        let mut rng = Rng(seed.wrapping_mul(0x2545F4914F6CDD1D) | 1);
        // ego at rest mid-way along a random street, in its right-hand lane
        let e = map.edges[(rng.uniform() * map.edges.len() as f64) as usize % map.edges.len()];
        let (a, b) = (map.nodes[e[0]], map.nodes[e[1]]);
        let d = dist(a, b).max(1e-9);
        let dir = [(b[0] - a[0]) / d, (b[1] - a[1]) / d];
        let ego = State {
            x: (a[0] + b[0]) / 2.0 + dir[1] * LANE_OFFSET_M,
            y: (a[1] + b[1]) / 2.0 - dir[0] * LANE_OFFSET_M,
            yaw: dir[1].atan2(dir[0]),
            speed: 0.0,
        };
        let mut actors: Vec<SmartActor> = vec![];
        while actors.len() < n_actors {
            let cand = SmartActor::new(&map, &mut rng);
            let clear = dist([cand.state.x, cand.state.y], [ego.x, ego.y]) > 30.0
                && actors
                    .iter()
                    .all(|o| dist([cand.state.x, cand.state.y], [o.state.x, o.state.y]) > 15.0);
            if clear {
                actors.push(cand);
            }
        }
        LiveWorld {
            map,
            ego,
            actors,
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
        self.road = Some(Road {
            centerline: line,
            target_speed: self.target_speed,
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

    /// Advance the whole world one tick: every actor reacts to a snapshot
    /// of the traffic (ego included), then the ego replans and steps —
    /// or brakes to a stop if there's no goal.
    pub fn tick(&mut self) {
        let snapshot: Vec<State> = std::iter::once(self.ego)
            .chain(self.actors.iter().map(|a| a.state))
            .collect();
        for (i, actor) in self.actors.iter_mut().enumerate() {
            let others: Vec<State> = snapshot
                .iter()
                .enumerate()
                .filter_map(|(j, &s)| (j != i + 1).then_some(s))
                .collect();
            actor.step(&self.map, &others, self.dt, &mut self.rng);
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
    use super::*;

    #[test]
    fn street_map_is_deterministic_and_connected() {
        let (a, b) = (StreetMap::generate(7), StreetMap::generate(7));
        assert_eq!(a.nodes, b.nodes);
        assert_eq!(a.edges, b.edges);
        assert!(connected(a.nodes.len(), &a.edges));
        // enough streets survived the pruning to be an interesting map
        assert!(a.edges.len() >= a.nodes.len() - 1);
    }

    #[test]
    fn routes_stay_on_the_road_and_reach_the_goal() {
        let map = StreetMap::generate(3);
        let (from, to) = ([-180.0, -190.0], [200.0, 180.0]);
        let line = map.route(from, 0.0, to);
        let (_, goal) = map.snap(to);
        assert!(dist(*line.last().unwrap(), goal) < 2.0 * LANE_OFFSET_M + 1e-6);
        // every point of the lane polyline is within the drivable half-width
        // of some street axis
        for &p in &line {
            let (_, q) = map.snap(p);
            assert!(
                dist(p, q) <= ROAD_HALF_WIDTH_M,
                "{p:?} is {} m off-road",
                dist(p, q)
            );
        }
    }

    #[test]
    fn smart_actor_cruises_alone_and_stops_behind_a_blocker() {
        let map = StreetMap::generate(5);
        let mut rng = Rng(9);
        let mut free = SmartActor::new(&map, &mut rng);
        for _ in 0..300 {
            free.step(&map, &[], 0.1, &mut rng);
        }
        assert!(
            (free.state.speed - free.target_speed).abs() < 0.5,
            "speed {}",
            free.state.speed
        );

        // park a blocker 30 m ahead in the actor's own lane
        let mut actor = SmartActor::new(&map, &mut rng);
        let (p, yaw) = actor.path.pose_at(actor.s + 30.0);
        let blocker = State {
            x: p[0],
            y: p[1],
            yaw,
            speed: 0.0,
        };
        for _ in 0..300 {
            actor.step(&map, &[blocker], 0.1, &mut rng);
        }
        assert!(actor.state.speed < 0.3, "speed {}", actor.state.speed);
        let gap = dist([actor.state.x, actor.state.y], [blocker.x, blocker.y]);
        assert!(gap > CAR_LENGTH_M * 0.8, "gap {gap}");
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
            dist([w.ego.x, w.ego.y], snapped) < 8.0,
            "stopped {} m from the goal",
            dist([w.ego.x, w.ego.y], snapped)
        );
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
