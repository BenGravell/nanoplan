//! Reusable library utilities. Keep these APIs and their tests even when the
//! application has no current callers; unused business logic outside this
//! module remains subject to the normal dead-code lint.
#![allow(
    dead_code,
    reason = "Reusable library APIs are retained independently of application usage"
)]

pub(crate) mod differencing;
pub(crate) mod interp;
pub(crate) mod kinematics;
pub(crate) mod linalg;
pub(crate) mod math;
pub(crate) mod measure;
pub(crate) mod rng;
pub(crate) mod types;
