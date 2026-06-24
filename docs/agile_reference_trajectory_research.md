# Agile Reference Trajectory Research And Redesign

Date: 2026-06-25

This note records the research result for the `multirotor_reference_trajectory`
and `px4_multirotor_controller` reference interface redesign. It is intentionally
stricter than the current implementation: capabilities that are not implemented
yet are listed as gaps, not as supported behavior.

## Goal

Build a generic reference trajectory generation framework for aggressive
multirotor tracking and race-gate style tasks.

The controller side must not depend on a 100 Hz sampled horizon topic. The
reference node should publish compact active trajectory objects, and the
controller should evaluate the active trajectory locally over the NMPC horizon.

The public request interfaces remain three separate interfaces:

- `AnalyticReference`: analytic curves such as hold, circle, height circle,
  entry circle, and figure-eight.
- `ConstrainedReferenceRequest`: geometric, kinematic, yaw, and dynamic
  constraint requests for waypoint, region, and gate traversal planning.
- `SampledReference`: externally generated flat samples that already contain
  the derivatives required by the controller. Missing high-order derivatives are
  invalid; the controller must not patch them with finite differences.

No B-spline public interface is introduced for this product. FAST/EGO-style
B-spline systems are used only as design references for timing, feasibility, and
local sampling patterns.

## External Projects Reviewed

### AirSim Drone Racing Lab

Source:

- https://github.com/microsoft/AirSim-Drone-Racing-Lab
- https://github.com/microsoft/AirSim-NeurIPS2019-Drone-Racing

Key finding:

- Gates are scene objects with poses, commonly sorted by `GateN`.
- Crossing direction is derived from the gate orientation and used to form
  velocity constraints for spline flight APIs.
- Gate pose defines geometry and crossing axis. Desired crossing speed is a
  separate constraint, not an overloaded yaw or pose field.

Design impact:

- A gate request must separate gate geometry from traversal semantics.
- Gate orientation should define the local gate frame and crossing axis.
- Speed and direction constraints should be explicit fields.

### TOGT-Planner

Source:

- https://github.com/FSC-Lab/TOGT-Planner

Key finding:

- Race tracks encode ordered gates with shape type, pose, dimensions, margins,
  length, midpoints, and stationary flags.
- Gates are geometric constraints, not center points.
- The optimizer uses MINCO, L-BFGS, positive time variables, spatial variables,
  and dynamic penalties based on flatness-derived quantities.
- The useful pattern is `time variables + spatial variables + gate/corridor
  geometry + dynamic feasibility`, not fixed-time point interpolation.

Design impact:

- `segment_times` should be only an initial guess unless the request explicitly
  asks for fixed time.
- Gate traversal should be optimized through an aperture/region with crossing
  semantics.
- Dynamic feasibility should be part of planning cost and validation, not only a
  post-generation warning.

### rpg_time_optimal

Source:

- https://github.com/uzh-rpg/rpg_time_optimal

Key finding:

- It is a direct optimal-control style formulation with state and input
  constraints.
- It can encode start/end position, velocity, attitude, and body-rate
  constraints.
- Older track examples treat gates mostly as point/ring constraints; this loses
  rich aperture and crossing semantics.

Design impact:

- Arbitrary full attitude constraints are an OCP-level concept, not a clean
  generic MINCO waypoint interface.
- For a flatness-based polynomial reference, interior "full attitude through a
  gate" should not be accepted blindly. It must be converted to compatible
  yaw/acceleration/thrust constraints, or rejected as infeasible.

### ETH MAV Trajectory Generation

Source:

- https://github.com/ethz-asl/mav_trajectory_generation

Key finding:

- Position and yaw can be optimized as separate trajectory dimensions.
- Feasibility checks include thrust, velocity, roll/pitch rate, yaw rate, and
  yaw acceleration constraints.

Design impact:

- Yaw is a first-class trajectory component and should not be inferred from
  velocity whenever the request carries heading constraints.
- `ActivePolynomialReference` must carry yaw coefficients when yaw is planned.
- Controller-side flatness mapping should use analytic
  `p/v/a/jerk/snap/yaw/yaw_rate/yaw_accel`.

### Fast-Racing, FAST Planner, EGO Planner, Agilicious

Sources:

- https://github.com/ZJU-FAST-Lab/Fast-Racing
- https://github.com/uzh-rpg/agile_autonomy
- https://github.com/uzh-rpg/agilicious

Key finding:

- ROS-facing systems often publish compact curves and let a trajectory server
  sample position, velocity, acceleration, jerk, yaw, and yaw rate.
- Some stacks use B-splines internally and publish sampled `PositionCommand`
  style references.
- Local sampling may be progress-aware, not purely wall-clock based.

Design impact:

- The controller should consume active curve objects and sample the NMPC horizon
  locally.
- The reference node should not publish a high-rate sampled horizon as the main
  path.
- Progress-aware local sampling is a later controller-side option, but the
  current redesign should at least keep trajectory id, revision, start time, and
  duration explicit.

### TII Racing Messages

Source:

- https://github.com/tii-racing/drone-racing-msgs

Key finding:

- Gate geometry and sampled setpoint references are separated.
- A sampled setpoint carries pose, velocity, acceleration, jerk, heading,
  heading rate, and trajectory-relative time.

Design impact:

- Keep race geometry messages separate from controller reference messages.
- If an external planner publishes sampled references, each sample must carry
  enough derivative information for flatness mapping.

## Local Planner Sources Reviewed

### Planner MINCO Backends

Local source:

- Local planner implementations under `products/ros1/planner/planner`.

Relevant concepts:

- MINCO trajectory optimization.
- Positive time mapping.
- Spatial variables for constrained intermediate points.
- FIRI/H-polytope safe corridor generation.
- Dynamic penalties on velocity, body rate, tilt, and thrust through flatness.

Use in this product:

- Reuse the MINCO-style optimizer pattern and flatness feasibility checks.
- Borrow H-polytope/corridor concepts only when a request needs generic convex
  regions beyond point, sphere, box, and gate.
- Do not introduce a runtime dependency on the planner package.

### FAST/EGO Planner

Local source:

- `products/ros1/planner/planner/fast_planner`
- `products/ros1/planner/planner/ego_planner`

Relevant concepts:

- Segment timing from distance and velocity limits.
- Feasibility reparameterization.
- Explicit yaw trajectory in some FAST paths.
- Trajectory server pattern: compact trajectory in, high-rate command out.

Use in this product:

- Reuse timing and feasibility ideas.
- Do not add B-spline as a public reference interface for this controller
  product.

### FAPP

Local source:

- `external/dev/motion-planning/src/FAPP`

Relevant concepts:

- Compact MINCO trajectory broadcast.
- Dynamic-object and swarm-aware penalties.
- Velocity, acceleration, and jerk feasibility penalties.

Use in this product:

- Useful later for moving-object constraints.
- Not part of the initial controller/reference interface.

### CERLAB

Local source:

- `products/ros1/planner/planner/cerlab`

Relevant concepts:

- Box corridor trajectory solving.
- B-spline time optimization / time remapping.
- PX4 navigation integration examples.

Use in this product:

- Useful for post-processing and time remapping ideas.
- Not the primary gate constraint model.

## Current Implementation Reality

Current package:

- `multirotor_reference_trajectory`
- `px4_multirotor_controller`

Current implemented facts:

- `WaypointReferenceRequest.msg` has `POINT`, `SPHERE`, `BOX`, and `GATE`.
- It also has `segment_times`, start/end velocity, start/end acceleration,
  `desired_speed`, `time_weight`, dynamic limits, iteration limit, and tolerance.
- The runtime builds a waypoint problem, solves it on a planning worker thread,
  and publishes `ActivePolynomialReference`.
- The core has a MINCO/L-BFGS path with positive time variables and spatial
  variables for non-point interior constraints.
- The controller consumes active analytic, polynomial, and sampled references,
  then samples the NMPC horizon locally.

Current gaps:

- `CONSTRAINT_GATE` is not true semantic gate traversal. It is currently an
  oriented bounded region for an intermediate point.
- Gate orientation is used as a region frame. It is not a UAV attitude target.
- There is no explicit crossing direction constraint.
- There is no explicit gate normal speed or speed range constraint.
- There is no interior node velocity vector constraint.
- There is no interior yaw/yaw-rate/yaw-acceleration constraint.
- `desired_speed` mainly seeds missing segment durations. It is not a hard
  crossing-speed constraint.
- `ActivePolynomialReference` has `coeff_yaw`, but the waypoint solver currently
  does not generate a real yaw polynomial for constrained planning.
- When yaw coefficients are absent, yaw may be inferred from velocity. That is
  acceptable only for a declared velocity-heading policy, not for all gate tasks.
- Dynamic limit flags are not all treated as fatal by the controller.
- Arbitrary full attitude through an interior gate is not physically guaranteed
  to be compatible with differential flatness.

## Correct Interface Direction

The existing `WaypointReferenceRequest` should be replaced by
`ConstrainedReferenceRequest`. It should not become a giant union message. The
request should be decomposed into typed, repeated constraint arrays that share
node ids.

### Public Request Interfaces

The final public request interfaces should be:

- `AnalyticReference`
- `ConstrainedReferenceRequest`
- `SampledReference`

The final active controller interfaces should be:

- `ActiveAnalyticReference` or direct validated analytic activation
- `ActivePolynomialReference`
- `ActiveSampledReference`
- `ReferenceStatus`

The controller should consume only validated active references.

### ConstrainedReferenceRequest

Recommended fields:

- `std_msgs/Header header`
- `uint32 request_id`
- `uint32 trajectory_id`
- `uint32 revision`
- `time start_time`
- `uint32 flags`
- `ReferenceNode[] nodes`
- `SegmentTiming[] segment_timing`
- `RegionConstraint[] region_constraints`
- `VelocityConstraint[] velocity_constraints`
- `YawConstraint[] yaw_constraints`
- `BoundaryStateConstraint[] boundary_constraints`
- `DynamicLimits limits`
- `PlanningOptions options`

`segment_timing` is an initial guess by default. A separate flag can request
fixed timing, but time optimization remains the default for aggressive flight.

### ReferenceNode

Recommended fields:

- `uint32 node_id`
- `geometry_msgs/Point nominal_position`
- `float64 nominal_time`
- `uint32 flags`

Nodes are only anchors for constraints. They are not automatically hard point
waypoints unless a point constraint references them.

### RegionConstraint

Recommended fields:

- `uint32 node_id`
- `uint8 type`
- `geometry_msgs/Pose pose`
- `geometry_msgs/Vector3 size`
- `float64 margin`
- `uint8 crossing_axis`
- `uint8 crossing_direction`
- `uint32 flags`

Supported types:

- `POINT`
- `SPHERE`
- `BOX`
- `GATE`
- `POLYTOPE` only after H-polytope support is added.

Gate meaning:

- `pose.position` is the gate center.
- `pose.orientation` is the gate frame.
- `size` is aperture width, height, and thickness.
- `crossing_axis` declares which local axis is normal to the gate plane.
- `crossing_direction` declares positive, negative, or either direction.

Gate orientation must not be interpreted as UAV attitude.

### VelocityConstraint

Recommended fields:

- `uint32 node_id`
- `uint8 mode`
- `geometry_msgs/Vector3 direction`
- `float64 min_speed`
- `float64 max_speed`
- `float64 target_speed`
- `float64 weight`
- `uint32 flags`

Supported modes:

- `FREE`
- `VECTOR`
- `DIRECTION`
- `SPEED_RANGE`
- `GATE_NORMAL`

For race gates, the common case is `GATE_NORMAL` plus a speed range. The gate
geometry defines the local normal; velocity constraint defines the actual
crossing behavior.

### YawConstraint

Recommended fields:

- `uint32 node_id`
- `uint8 mode`
- `float64 yaw`
- `float64 yaw_rate`
- `float64 yaw_acceleration`
- `float64 weight`
- `uint32 flags`

Supported modes:

- `FREE`
- `FIXED`
- `VELOCITY_HEADING`
- `GATE_HEADING`
- `LOOKAHEAD_TARGET`

Yaw is independent of gate pose. If yaw is constrained, the solver must generate
`coeff_yaw`. If yaw is unconstrained and the request declares
`VELOCITY_HEADING`, yaw may be derived analytically from the velocity field with
singularity checks.

### BoundaryStateConstraint

Recommended fields:

- `uint8 boundary`
- `geometry_msgs/Vector3 position`
- `geometry_msgs/Vector3 velocity`
- `geometry_msgs/Vector3 acceleration`
- `float64 yaw`
- `float64 yaw_rate`
- `float64 yaw_acceleration`
- `uint32 mask`

Supported boundaries:

- `START`
- `END`

Interior arbitrary full-state constraints are not accepted as hard constraints
unless they can be mapped to flat outputs consistently.

### DynamicLimits

Recommended fields:

- `float64 max_velocity`
- `float64 max_acceleration`
- `float64 max_jerk`
- `float64 max_snap`
- `float64 min_thrust`
- `float64 max_thrust`
- `float64 max_tilt`
- `float64 max_body_rate`
- `float64 max_yaw_rate`
- `float64 max_yaw_acceleration`

These limits are used in both optimization penalties and final validation.
Fatal vs nonfatal treatment should be explicit in controller parameters.

### PlanningOptions

Recommended fields:

- `uint8 objective`
- `float64 time_weight`
- `float64 dynamic_penalty_weight`
- `float64 region_penalty_weight`
- `float64 velocity_penalty_weight`
- `float64 yaw_penalty_weight`
- `uint32 max_iterations`
- `float64 rel_cost_tol`
- `uint32 integral_resolution`
- `uint32 flags`

The first supported objective should be MINCO-S3 time/space optimization.
Fixed-time minimum-snap interpolation should not be the default path for race
gates.

## Core Optimizer Design

The core remains ROS-free and state-machine-free.

Recommended modules:

- `FlatOutput`
- `FullStateReference`
- `FlatnessMapper`
- `AnalyticEvaluator`
- `SampledEvaluator`
- `PiecewisePolynomialEvaluator`
- `ConstraintSet`
- `ShapeAdapter`
- `TimeParameterization`
- `YawTrajectorySolver`
- `MincoS3TrajectoryOptimizer`
- `TrajectoryValidator`

### Optimization Variables

Use:

- `tau`: unconstrained time variables mapped to positive segment durations.
- `xi`: spatial variables for region constraints.
- optional yaw variables or yaw polynomial boundary variables when yaw is
  constrained.

Do not treat `segment_times` as fixed unless the request explicitly asks for
fixed timing.

### Cost Terms

Minimum required cost terms:

- snap energy
- time cost
- region violation / region mapping cost
- velocity limit and node crossing-speed penalties
- acceleration, jerk, snap penalties
- thrust, tilt, body-rate penalties from flatness
- yaw and yaw-rate penalties when yaw is constrained

Hard equality constraints:

- start/end PVA when provided
- fixed point constraints
- fixed yaw boundary when provided

Soft or projected constraints:

- sphere/box/gate interior point
- gate crossing direction
- speed range
- dynamic limits

### Gate Traversal Semantics

For a gate node, the planner should enforce or penalize:

- position inside the aperture in the gate frame
- position near or inside the thickness slab
- velocity component along the requested crossing direction
- speed range along or near the gate normal
- optional yaw policy independent of gate pose

This is the minimum requirement for "fly through this gate at a requested speed
and heading policy".

### Full Attitude Constraints

For a quadrotor, flat outputs are position and yaw. Roll and pitch are induced
by acceleration, thrust, and yaw. Therefore:

- Start/end full attitude can be accepted only after checking whether it is
  compatible with flatness-derived acceleration and thrust.
- Interior full attitude through a gate is not a generic hard MINCO constraint.
- A request for arbitrary attitude at a gate should be rejected with a clear
  flag unless it can be converted into yaw plus physically feasible acceleration
  and thrust constraints.

This avoids pretending that a gate pose or quaternion directly means UAV pose.

## Reference Node State Machine

The reference node should keep the event-driven design:

- subscriber callbacks only create input events
- `SelfCheck`, `Ready`, `Planning`, `Active`, and `Fault` remain explicit states
- planning runs in a worker thread and posts completion events back to the main
  state-machine queue
- active trajectory publication runs through the output consumer queue
- no optimizer blocks the 100 Hz main loop

Required refinements:

- Planning jobs must carry `request_id`, `trajectory_id`, and `revision`.
- Newer revisions cancel or supersede old worker results.
- A failed plan must not publish an active reference.
- `start_time` is the intended activation time, not process start time.
- `Active` should publish latched active trajectory objects plus lightweight
  status, not a 100 Hz horizon.

## Controller Contract

The PX4 NMPC controller should:

- subscribe only to validated active references
- maintain an `ActiveTrajectoryCache`
- sample the active trajectory over its prediction horizon every control cycle
- use analytic flatness mapping for full-state reference recovery
- reject references with missing derivatives, nonfinite values, low thrust,
  yaw singularities, unsupported hard constraints, expired time domains, or
  fatal dynamic flags
- avoid finite-difference reconstruction of body rate or angular acceleration

The controller should not solve MINCO. It only evaluates the active reference
and constructs NMPC `x_ref/u_ref`.

## Implementation Plan

### Step 1: Replace Request Messages

- Add `ConstrainedReferenceRequest` and typed constraint messages.
- Remove the old `WaypointReferenceRequest` main path.
- Keep the three public entry families only:
  `AnalyticReference`, `ConstrainedReferenceRequest`, `SampledReference`.
- Update launch/config/docs to use the new request names.

### Step 2: Refactor Core Problem Model

- Replace the current flat `WaypointProblem` with `ConstraintSet`.
- Add explicit region, velocity, yaw, boundary, dynamic-limit, and planner-option
  structs.
- Keep all core structs ROS-free.

### Step 3: Implement Gate And Velocity Constraints

- Implement gate geometry mapping in gate-local frame.
- Add crossing-axis and crossing-direction handling.
- Add speed-range and gate-normal velocity penalties.
- Add tests proving that gate pose, crossing direction, and speed constraints
  affect the optimized trajectory.

### Step 4: Implement Yaw Planning

- Generate yaw polynomial coefficients when yaw constraints exist.
- Support `FIXED`, `VELOCITY_HEADING`, and `GATE_HEADING` first.
- Validate yaw, yaw rate, and yaw acceleration.
- Ensure `ActivePolynomialReference.coeff_yaw` is populated for planned yaw.

### Step 5: Integrate Dynamic Feasibility

- Keep flatness-based thrust, tilt, and body-rate penalties in optimization.
- Add final validation flags for velocity, acceleration, jerk, snap, thrust,
  tilt, body rate, yaw rate, and yaw acceleration.
- Decide fatal flag policy explicitly in controller config.

### Step 6: Tighten SampledReference

- Require position, velocity, acceleration, jerk, snap, yaw, yaw rate, and yaw
  acceleration for aggressive sampled references.
- Reject missing or nonfinite derivative fields.
- Keep sampled references for replay/debug/external planners, not as the main
  online horizon transport.

### Step 7: Update Controller Consumption

- Update active cache for the new message names and fatal flag policy.
- Ensure polynomial yaw is used when present.
- Ensure velocity-heading yaw is declared, not implicit.
- Add tests for stale, expired, missing yaw, missing derivative, and fatal
  dynamic flags.

## Test Plan

Core tests:

- analytic circle, height circle, and figure-eight produce derivatives through
  snap
- flatness mapping rejects low thrust, nonfinite inputs, and yaw singularities
- MINCO-S3 trajectory satisfies start/end PVA and segment continuity
- region constraints map POINT, SPHERE, BOX, and GATE correctly
- gate crossing direction changes the optimized velocity sign
- gate speed range changes optimized segment timing or velocity penalty
- yaw fixed and velocity-heading modes generate valid yaw coefficients
- impossible full-attitude constraints are rejected instead of silently accepted

State-machine tests:

- `SelfCheck -> Ready -> Planning -> Active`
- planning failure enters `Fault` or returns to `Ready` according to policy
- planning worker does not block the main loop
- newer revision suppresses older worker result
- reset clears pending plans and active references

Controller tests:

- Custom1 waits for a legal active trajectory
- controller rejects expired references
- controller rejects prediction horizon outside trajectory time domain
- controller rejects missing high-order derivative sampled references
- controller rejects fatal dynamic flags
- controller samples polynomial yaw instead of deriving yaw when coeffs exist

Simulation tests:

- FS150 analytic height circle tracking remains green
- POINT waypoint request generates active polynomial and tracks
- GATE request with crossing direction and speed range generates active
  polynomial and tracks
- invalid gate orientation or infeasible speed is rejected before Custom1 NMPC
  tracking starts
- tracking logs include trajectory id, revision, reference type, flags, tracking
  error, thrust, omega, alpha, and NMPC status

## Immediate Design Decisions

- Replace `WaypointReferenceRequest` rather than extending it with more optional
  fields.
- Keep gate geometry and traversal constraints separate.
- Treat yaw as an independent trajectory dimension.
- Treat arbitrary full attitude as a validation problem unless it is
  flatness-compatible.
- Keep MINCO solving in the reference node worker thread.
- Keep NMPC sampling in the controller.
- Do not add B-spline as a public interface in this product.
- Do not publish a 100 Hz horizon as the main controller interface.
