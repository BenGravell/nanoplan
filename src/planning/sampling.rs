//! Shared quasi-Monte-Carlo low-discrepancy sampling.
//!
//! The tree and sampling-MPC planners use the same deterministic Halton
//! coordinates. [`qmc_normals`] maps them through an inverse-normal CDF
//! for Gaussian perturbations. [`QuasiMonteCarlo`] keeps the sequence type
//! explicit at each call site.

/// Prime bases of the Halton sequence, one per dimension. Sized to cover
/// the highest-dimensional caller: the optimizers draw a `num_nodes * NU`
/// dimensional knot-noise vector, and `num_nodes` ranges up to a dozen, so
/// two dozen-plus primes leaves headroom (a request past this panics in
/// [`QuasiMonteCarlo::coordinate`] rather than silently reusing a base).
const PRIMES: [usize; 32] = [
    2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59, 61, 67, 71, 73, 79, 83, 89, 97, 101, 103, 107, 109,
    113, 127, 131,
];

/// The van der Corput radical-inverse sequence in `base`: reverse the
/// base-`base` digits of `index` and put them after the radix point. The
/// building block of a Halton sequence — pairing van der Corput sequences
/// in different (coprime) bases, one per dimension, gives a fully
/// deterministic, index-only low-discrepancy point set with no RNG state at
/// all. `index` should start at 1 — `index = 0` degenerates to `0.0` in
/// every base, which would stack a sample from every dimension onto one
/// corner.
pub(crate) fn van_der_corput(mut index: usize, base: usize) -> f64 {
    let mut result = 0.0;
    let mut fraction = 1.0 / base as f64;
    while index > 0 {
        result += fraction * (index % base) as f64;
        index /= base;
        fraction /= base as f64;
    }
    result
}

/// The one deterministic low-discrepancy construction shared across the
/// codebase — the compile-time seam that keeps every sampling planner on a
/// named QMC sequence (see the module doc). Its single method is an
/// associated function, not a `&self` method: a QMC coordinate is a pure
/// function of `(index, dim)` with no state to carry.
pub(crate) trait QuasiMonteCarlo {
    /// Coordinate `dim` (0-indexed) of low-discrepancy point `index`.
    /// `index` starts at 1 (index 0 sits on the origin in every supported
    /// sequence). Panics if `dim` exceeds the sequence's supported
    /// dimensionality rather than aliasing one.
    fn coordinate(index: usize, dim: usize) -> f64;
}

/// One-dimensional van der Corput as a QMC sequence.
pub(crate) struct VanDerCorput<const BASE: usize>;

impl<const BASE: usize> QuasiMonteCarlo for VanDerCorput<BASE> {
    fn coordinate(index: usize, dim: usize) -> f64 {
        assert_eq!(dim, 0, "van der Corput is one-dimensional");
        van_der_corput(index, BASE)
    }
}

const _: fn(usize, usize) -> f64 = <VanDerCorput<2> as QuasiMonteCarlo>::coordinate;

/// The Halton sequence: a van der Corput sequence per dimension in
/// successive prime bases. Zero-sized — there is nothing to construct, the
/// sequence is entirely a function of `(index, dim)`.
pub(crate) struct Halton;

impl QuasiMonteCarlo for Halton {
    fn coordinate(index: usize, dim: usize) -> f64 {
        van_der_corput(index, PRIMES[dim])
    }
}

/// Inverse standard-normal CDF (Acklam's rational approximation): map a
/// uniform QMC coordinate in `(0, 1)` to a standard-normal deviate. This is
/// what turns the shared Halton sequence into *low-discrepancy Gaussian*
/// noise for the optimizers' control-knot perturbations — the QMC stand-in
/// for judo's pseudo-random `np.random.randn`. Accurate to ~1e-9 in the
/// bulk, which is far finer than the sampling needs. `p` is assumed already
/// in `(0, 1)`; a Halton coordinate of a positive `index` always is.
pub(crate) fn inv_normal_cdf(p: f64) -> f64 {
    // coefficients of the rational approximation
    const A: [f64; 6] = [
        -3.969683028665376e+01,
        2.209460984245205e+02,
        -2.759285104469687e+02,
        1.38357751867269e+02,
        -3.066479806614716e+01,
        2.506628277459239e+00,
    ];
    const B: [f64; 5] = [
        -5.447609879822406e+01,
        1.615858368580409e+02,
        -1.556989798598866e+02,
        6.680131188771972e+01,
        -1.328068155288572e+01,
    ];
    const C: [f64; 6] = [
        -7.784894002430293e-03,
        -3.223964580411365e-01,
        -2.400758277161838e+00,
        -2.549732539343734e+00,
        4.374664141464968e+00,
        2.938163982698783e+00,
    ];
    const D: [f64; 4] = [
        7.784695709041462e-03,
        3.224671290700398e-01,
        2.445134137142996e+00,
        3.754408661907416e+00,
    ];
    const P_LOW: f64 = 0.02425;
    let p = p.clamp(1e-12, 1.0 - 1e-12);
    if p < P_LOW {
        // lower tail
        let q = (-2.0 * p.ln()).sqrt();
        (((((C[0] * q + C[1]) * q + C[2]) * q + C[3]) * q + C[4]) * q + C[5])
            / ((((D[0] * q + D[1]) * q + D[2]) * q + D[3]) * q + 1.0)
    } else if p <= 1.0 - P_LOW {
        // central region
        let q = p - 0.5;
        let r = q * q;
        (((((A[0] * r + A[1]) * r + A[2]) * r + A[3]) * r + A[4]) * r + A[5]) * q
            / (((((B[0] * r + B[1]) * r + B[2]) * r + B[3]) * r + B[4]) * r + 1.0)
    } else {
        // upper tail (by symmetry of the lower)
        let q = (-2.0 * (1.0 - p).ln()).sqrt();
        -(((((C[0] * q + C[1]) * q + C[2]) * q + C[3]) * q + C[4]) * q + C[5])
            / ((((D[0] * q + D[1]) * q + D[2]) * q + D[3]) * q + 1.0)
    }
}

/// `count` standard-normal noise vectors of dimension `dim`, drawn from the
/// shared QMC sequence: vector `k` is Halton point `base + k`, whose `dim`
/// coordinates are each mapped through [`inv_normal_cdf`]. Deterministic in
/// `base` — the QMC replacement for judo's
/// `np.random.randn(count, num_nodes, nu)`; the optimizer reshapes each
/// length-`dim` vector into its `(num_nodes, NU)` knot grid. Generic over
/// `Q` so the shared QMC interface (see the module doc) is named at this
/// call site.
///
/// **Mean-centered per dimension.** A pseudo-random `randn` is zero-mean
/// *in expectation*, but a low-discrepancy set of only a few dozen points
/// has a small, deterministic nonzero mean per dimension — and because it's
/// deterministic, that bias doesn't average out across ticks: it shows up
/// as a steady lateral drift, the ego settling ~1.5 m off-center instead of
/// on it (found exactly this way, by the sampling planners' centerline
/// tracking tests). Subtracting each dimension's empirical mean restores the
/// zero-mean property judo's RNG has in expectation, at the cost of one
/// degree of freedom (negligible at these counts).
pub(crate) fn qmc_normals<Q: QuasiMonteCarlo>(base: usize, count: usize, dim: usize) -> Vec<Vec<f64>> {
    let mut z: Vec<Vec<f64>> = (0..count)
        .map(|k| (0..dim).map(|d| inv_normal_cdf(Q::coordinate(base + k, d))).collect())
        .collect();
    if count > 0 {
        for d in 0..dim {
            let mean: f64 = z.iter().map(|zk| zk[d]).sum::<f64>() / count as f64;
            for zk in &mut z {
                zk[d] -= mean;
            }
        }
    }
    z
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn van_der_corput_base2_bit_reversal() {
        // 1 -> 0.1b, 2 -> 0.01b, 3 -> 0.11b in base-2 radical inverse
        assert_eq!(van_der_corput(1, 2), 0.5);
        assert_eq!(van_der_corput(2, 2), 0.25);
        assert_eq!(van_der_corput(3, 2), 0.75);
        assert_eq!(VanDerCorput::<2>::coordinate(3, 0), 0.75);
    }

    #[test]
    fn halton_coordinate_is_van_der_corput_in_the_dim_th_prime() {
        for i in 1..50 {
            assert_eq!(Halton::coordinate(i, 0), van_der_corput(i, 2));
            assert_eq!(Halton::coordinate(i, 1), van_der_corput(i, 3));
            assert_eq!(Halton::coordinate(i, 4), van_der_corput(i, 11));
        }
    }

    #[test]
    fn halton_coordinates_stay_in_the_unit_interval() {
        for i in 1..500 {
            for d in 0..8 {
                let v = Halton::coordinate(i, d);
                assert!((0.0..1.0).contains(&v), "coord {v} out of range");
            }
        }
    }

    #[test]
    fn inv_normal_cdf_is_symmetric_and_ordered() {
        assert!((inv_normal_cdf(0.5)).abs() < 1e-9);
        // standard quantiles
        assert!((inv_normal_cdf(0.975) - 1.959963985).abs() < 1e-6);
        assert!((inv_normal_cdf(0.025) + 1.959963985).abs() < 1e-6);
        // monotonic increasing
        let mut prev = f64::NEG_INFINITY;
        for k in 1..1000 {
            let v = inv_normal_cdf(k as f64 / 1000.0);
            assert!(v > prev, "not monotonic at {k}");
            prev = v;
        }
    }

    #[test]
    fn qmc_normals_are_deterministic_and_roughly_standard() {
        let a = qmc_normals::<Halton>(1, 256, 8);
        let b = qmc_normals::<Halton>(1, 256, 8);
        assert_eq!(a, b); // pure function of base
        assert_eq!(a.len(), 256);
        assert!(a.iter().all(|v| v.len() == 8));
        // sample mean per dimension should be near zero for a
        // low-discrepancy standard-normal set
        for d in 0..8 {
            let mean: f64 = a.iter().map(|v| v[d]).sum::<f64>() / a.len() as f64;
            assert!(mean.abs() < 0.3, "dim {d} mean {mean}");
        }
    }
}
