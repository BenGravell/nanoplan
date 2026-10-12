# Trajectory optimization

`mod.rs` owns `Refined<P>`, the optional statically composed post-process for any trajectory generator.
`ilqr.rs` owns the shared finite-difference solver and the standalone `IlqrPlanner`.
The planner registry chooses the composition once, when settings change.
Disabled mode uses the generator directly.
See [planning](../../README.md#optional-trajectory-optimization) for the UI control, validation, warm starts, and
benchmark.

For **Tree+iLQR**, select **Tree** and enable **Trajectory optimization (iLQR)**.
This constructs `Refined<GraphPlanner>` with `Connections::NearestZap`.
It uses the ordinary tree search budget, refines one generated trajectory with the shared bounded iteration allowance,
validates the result, and feeds an accepted improvement back into the generator's warm start.
There is no separate combined planner implementation or preset entry.

## iLQR

The solver derives from [treetop](https://github.com/BenGravell/treetop).
It alternates a backward pass over linearized dynamics and quadratic cost expansions with a forward line search.
Gradients, Hessians, and dynamics Jacobians use central finite differences of the shared scalar objective and vehicle
model.
The 73 cost probes at each stage reuse nine position samples in fixed-size stack arrays; predicted actors are reused
across probes.

Regularization increases when a backward pass fails its positive-definiteness check or a forward step is rejected.
Hard constraint violations receive finite depth-scaled penalties so the optimizer can find a direction out of them.
The terminal cost uses the actual trajectory length, and the returned controls are realized under actuation limits.
The post-process additionally requires full trajectory feasibility, strictly lower cost, and non-decreasing terminal
progress before accepting the result; otherwise the original generator output is retained.

The standalone iLQR planner starts from a lane-keeping guess or its shifted previous solution.
It has a larger iteration allowance than the post-process because it receives no sampled trajectory seed.

Tests cover finite-difference correctness, cached-probe equivalence and work bounds, refinement acceptance and fallback,
the Tree+iLQR composition's closed-loop driving and diagnostics.
