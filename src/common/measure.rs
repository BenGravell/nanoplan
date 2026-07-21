pub(crate) fn dist(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

pub(crate) fn dot<const N: usize>(a: [f64; N], b: [f64; N]) -> f64 {
    (0..N).map(|i| a[i] * b[i]).sum()
}
