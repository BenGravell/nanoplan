# Leeroy Jenkins

`leeroy_jenkins/mod.rs` — `LeeroyJenkinsPlanner`

Always commands maximum acceleration (`MAX_LON_ACCEL`) and zero steering (zero curvature) for the entire planning
horizon, regardless of speed, road geometry, or obstacles.

No seams beyond `total` — there is no `route`, `optimize`, or `extract` phase because there is no computation.

It exists as a reckless baseline against which to measure the other planners.
