pub(crate) fn polygons_overlap(a: &[[f64; 2]], b: &[[f64; 2]]) -> bool {
    (0..a.len()).any(|i| {
        (0..b.len())
            .any(|j| segments_intersect(a[i], a[(i + 1) % a.len()], b[j], b[(j + 1) % b.len()]))
    }) || point_in_polygon(a[0], b)
        || point_in_polygon(b[0], a)
}

fn point_in_polygon(point: [f64; 2], polygon: &[[f64; 2]]) -> bool {
    let mut inside = false;
    for i in 0..polygon.len() {
        let (a, b) = (polygon[i], polygon[(i + 1) % polygon.len()]);
        if (a[1] > point[1]) != (b[1] > point[1])
            && point[0] < (b[0] - a[0]) * (point[1] - a[1]) / (b[1] - a[1]) + a[0]
        {
            inside = !inside;
        }
    }
    inside
}

pub(crate) fn segments_intersect(a: [f64; 2], b: [f64; 2], c: [f64; 2], d: [f64; 2]) -> bool {
    let cross = |p: [f64; 2], q: [f64; 2], r: [f64; 2]| {
        (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    };
    a[0].max(b[0]) >= c[0].min(d[0])
        && c[0].max(d[0]) >= a[0].min(b[0])
        && a[1].max(b[1]) >= c[1].min(d[1])
        && c[1].max(d[1]) >= a[1].min(b[1])
        && cross(a, b, c) * cross(a, b, d) <= 0.0
        && cross(c, d, a) * cross(c, d, b) <= 0.0
}
