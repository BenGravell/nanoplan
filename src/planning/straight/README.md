# Strawman

`straight/mod.rs` — `StraightPlanner`

```rust
fn plan(&mut self, _ego: State, ctx: &Context) -> Vec<Control> {
    vec![Control::default(); ctx.horizon]
}
```

Always drives straight ahead at whatever speed the ego already has (zero acceleration, zero curvature).

No seams beyond `total` — there's no `route`, `optimize`, or `extract` phase because there's no computation.

It exists to be the floor every other planner is measured against: whenever an obstacle is in the lane, it collides, and
the batch runner's mean score reliably shows this (~0.27 across a mixed synthetic batch, vs. 0.74-0.90 for the others).
