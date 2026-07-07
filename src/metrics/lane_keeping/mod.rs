//! Lane keeping: 1 while the ego holds the center of its lane, falling off as
//! it drifts to one side or straddles the line into the next lane. Smooth —
//! aggregated by average, so a brief, centered lane change barely dents it but
//! a *sustained* one-sided bias or a *prolonged* straddle does.
//!
//! Two distinct faults, both scored here against the ego's own reference lane
//! (the route centerline is that lane's center):
//! - **straddling** — the *instantaneous* offset sitting out near the lane
//!   line, i.e. the car half in each of two lanes;
//! - **bias** — a *persistent* one-sided offset, measured as the mean signed
//!   offset over a trailing window so a transient swerve (out and back,
//!   averaging toward center) is not mistaken for hugging one side.

use crate::metrics::TickCtx;

/// Half-width of a lane, in meters — half a standard ~3.5 m lane. The ego is
/// straddling the line into the neighbouring lane once its offset from the
/// centerline reaches this.
pub(crate) const LANE_HALF_WIDTH_M: f64 = 1.75;

/// Comfortable lane-keeping wander that draws no penalty: a driver never sits
/// exactly on the centerline. Offsets within this of center score a full 1.
pub(crate) const CENTER_TOLERANCE_M: f64 = 0.5;

/// Trailing window the one-sided-bias mean is taken over.
const BIAS_WINDOW_S: f64 = 2.0;

pub fn score(ctx: &TickCtx, i: usize) -> f64 {
    // sustained one-sided bias: mean *signed* offset over a trailing window,
    // so a brief lane change (offset out and back, crossing center) averages
    // toward zero while persistently hugging one side does not
    let window = ((BIAS_WINDOW_S / ctx.dt) as usize).max(1);
    let lo = i.saturating_sub(window);
    let mean = ctx.lateral[lo..=i].iter().sum::<f64>() / (i - lo + 1) as f64;

    // the worse of the sustained bias and the instantaneous straddle, each
    // measured past the comfortable centering tolerance
    let bias = (mean.abs() - CENTER_TOLERANCE_M).max(0.0);
    let straddle = (ctx.lateral[i].abs() - CENTER_TOLERANCE_M).max(0.0);
    let span = LANE_HALF_WIDTH_M - CENTER_TOLERANCE_M;
    (1.0 - bias.max(straddle) / span).clamp(0.0, 1.0)
}
