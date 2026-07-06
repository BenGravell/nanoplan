# nuPlan scenario definitions (vendored)

Vendored from [motional/nuplan-devkit](https://github.com/motional/nuplan-devkit)
(`master`, fetched 2026-07-02), Apache-2.0 (see [LICENSE.txt](LICENSE.txt)).

| File | What it defines |
|---|---|
| `nuplan_schema.md` | The nuPlan log database schema: ego poses, agent tracks, scenes with goal poses, `scenario_tag` (~70 scenario types), traffic light status. The source of truth for what a scenario is. |
| `vehicle_parameters.py` | Canonical ego vehicle (Chrysler Pacifica) geometry: wheelbase 3.089 m, width 2.297 m, length 5.176 m. Reference for the simulator's vehicle model. |
| `metrics_description.md` | Definitions and thresholds of the nuPlan planner quality metrics. Source of truth for [`src/metrics/`](../../src/metrics/README.md) (closed-loop score). |

These are definitions only — the nuPlan dataset itself (sqlite logs, maps)
is not vendored, and **exported nuPlan scenarios must stay local**: the
dataset is registration-gated and not redistributable, which is why the
scenario corpus this repo actually ships is CommonRoad-format instead
(see [`scenarios/commonroad/`](../commonroad/)).
[`tools/export_nuplan_scenarios.py`](../../tools/export_nuplan_scenarios.py)
converts a real nuPlan log you've licensed yourself into nanoplan's JSON
format — mainly to get `expert` trajectories for the cost-weight autotuner
([`src/tuning/README.md`](../../src/tuning/README.md)), which CommonRoad
scenarios don't carry.
