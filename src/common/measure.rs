pub(crate) fn dot<const N: usize>(a: [f64; N], b: [f64; N]) -> f64 {
    (0..N).map(|i| a[i] * b[i]).sum()
}
