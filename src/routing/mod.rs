//! Route planning over the procedural street network.

use crate::simulation::Position;
use crate::world::{ActorKind, StreetMap, corridor, dist, mid};

/// Extra route cost for starting against the current heading.
const U_TURN_PENALTY_M: f64 = 400.0;

/// Find the shortest route and shape it into a drivable lane centerline.
pub fn route(
    map: &StreetMap,
    from: impl Into<Position>,
    yaw: f64,
    to: impl Into<Position>,
) -> Vec<[f64; 2]> {
    let (se, sp) = map.snap(from);
    let (ge, mut gp) = map.snap(to);
    let heading = [yaw.cos(), yaw.sin()];
    let seed_cost = |n: usize| {
        let d = [map.nodes[n][0] - sp[0], map.nodes[n][1] - sp[1]];
        let behind = d[0] * heading[0] + d[1] * heading[1] < 0.0;
        dist(sp, map.nodes[n]) + if behind { U_TURN_PENALTY_M } else { 0.0 }
    };

    // The window is small, so an O(n²) Dijkstra scan needs no heap.
    let n = map.nodes.len();
    let (mut cost, mut pred, mut done) =
        (vec![f64::INFINITY; n], vec![usize::MAX; n], vec![false; n]);
    for s in map.edges[se] {
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
        for &v in &map.adj[u] {
            let c = cost[u] + dist(map.nodes[u], map.nodes[v]);
            if c < cost[v] {
                cost[v] = c;
                pred[v] = u;
            }
        }
    }

    let mut end = map.edges[ge]
        .into_iter()
        .min_by(|&a, &b| {
            (cost[a] + dist(map.nodes[a], gp)).total_cmp(&(cost[b] + dist(map.nodes[b], gp)))
        })
        .unwrap();
    if !cost[end].is_finite() {
        // ponytail: a window seam can orphan a street; use the nearest
        // reachable node instead.
        end = (0..n)
            .filter(|&i| cost[i].is_finite())
            .min_by(|&a, &b| dist(map.nodes[a], gp).total_cmp(&dist(map.nodes[b], gp)))
            .unwrap();
        gp = map.nodes[end];
    }

    let mut chain = vec![end];
    while pred[*chain.last().unwrap()] != usize::MAX {
        chain.push(pred[*chain.last().unwrap()]);
    }
    chain.reverse();
    let via_nodes = cost[end] + dist(map.nodes[end], gp);
    let mut axis: Vec<[f64; 2]> = if se == ge && direct_cost(sp, gp, heading) < via_nodes {
        vec![sp, gp]
    } else {
        std::iter::once(sp)
            .chain(chain.into_iter().map(|i| map.nodes[i]))
            .chain(std::iter::once(gp))
            .collect()
    };

    let raw = std::mem::take(&mut axis);
    axis.push(raw[0]);
    for (i, &p) in raw.iter().enumerate().skip(1) {
        if dist(*axis.last().unwrap(), p) >= 2.0 || axis.len() == 1 {
            axis.push(p);
        } else if i == raw.len() - 1 {
            *axis.last_mut().unwrap() = p;
        }
    }
    let lanes = axis
        .windows(2)
        .map(|w| map.lanes[map.snap(mid(w[0], w[1])).0])
        .collect::<Vec<_>>();
    corridor(map.seed, &axis, &lanes, ActorKind::Car, None)
}

fn direct_cost(sp: [f64; 2], gp: [f64; 2], heading: [f64; 2]) -> f64 {
    let d = [gp[0] - sp[0], gp[1] - sp[1]];
    let behind = d[0] * heading[0] + d[1] * heading[1] < 0.0;
    dist(sp, gp) + if behind { U_TURN_PENALTY_M } else { 0.0 }
}
