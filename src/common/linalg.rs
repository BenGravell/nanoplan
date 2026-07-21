pub(crate) fn mat_mul<const M: usize, const K: usize, const N: usize>(
    a: &[[f64; K]; M],
    b: &[[f64; N]; K],
) -> [[f64; N]; M] {
    let mut out = [[0.0; N]; M];
    for i in 0..M {
        for k in 0..K {
            let aik = a[i][k];
            for j in 0..N {
                out[i][j] += aik * b[k][j];
            }
        }
    }
    out
}

pub(crate) fn mat_vec<const M: usize, const N: usize>(a: &[[f64; N]; M], v: &[f64; N]) -> [f64; M] {
    std::array::from_fn(|i| (0..N).map(|j| a[i][j] * v[j]).sum())
}

pub(crate) fn transpose<const M: usize, const N: usize>(a: &[[f64; N]; M]) -> [[f64; M]; N] {
    std::array::from_fn(|i| std::array::from_fn(|j| a[j][i]))
}

pub(crate) fn vec_add<const N: usize>(a: [f64; N], b: [f64; N]) -> [f64; N] {
    std::array::from_fn(|i| a[i] + b[i])
}

pub(crate) fn mat_add<const M: usize, const N: usize>(
    a: [[f64; N]; M],
    b: [[f64; N]; M],
) -> [[f64; N]; M] {
    std::array::from_fn(|i| std::array::from_fn(|j| a[i][j] + b[i][j]))
}
