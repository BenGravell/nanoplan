use crate::common::types::Position;

pub(crate) fn polygons_overlap(a: &[Position], b: &[Position]) -> bool {
    (0..a.len())
        .any(|i| (0..b.len()).any(|j| segments_intersect(a[i], a[(i + 1) % a.len()], b[j], b[(j + 1) % b.len()])))
        || point_in_polygon(a[0], b)
        || point_in_polygon(b[0], a)
}

fn point_in_polygon(point: Position, polygon: &[Position]) -> bool {
    let mut inside = false;
    for i in 0..polygon.len() {
        let (a, b) = (polygon[i], polygon[(i + 1) % polygon.len()]);
        if (a.y > point.y) != (b.y > point.y) && point.x < (b.x - a.x) * (point.y - a.y) / (b.y - a.y) + a.x {
            inside = !inside;
        }
    }
    inside
}

pub(crate) fn segments_intersect(a: Position, b: Position, c: Position, d: Position) -> bool {
    let cross = |p: Position, q: Position, r: Position| (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x);
    a.x.max(b.x) >= c.x.min(d.x)
        && c.x.max(d.x) >= a.x.min(b.x)
        && a.y.max(b.y) >= c.y.min(d.y)
        && c.y.max(d.y) >= a.y.min(b.y)
        && cross(a, b, c) * cross(a, b, d) <= 0.0
        && cross(c, d, a) * cross(c, d, b) <= 0.0
}
