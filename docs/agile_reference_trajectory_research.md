# Multirotor reference interface

Public request surfaces:

- `AnalyticReference` — hold, circle, height circle, entry circle, figure-eight
- `ConstrainedReferenceRequest` — waypoint / region / gate constraints
- `SampledReference` — flat samples that already include required derivatives

The controller evaluates the active trajectory locally. Do not treat a 100 Hz
horizon topic as the main interface. There is no B-spline public API.

Launch:

```bash
roslaunch --files multirotor_reference_trajectory uav_multirotor_reference_trajectory.launch
roslaunch --files px4_multirotor_controller uav_nmpc_controller.launch
```

Design notes: `memory/now/agile-reference-trajectory.md` (`xgc2-dev-memory`).
