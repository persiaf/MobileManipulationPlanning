# Hierarchical Strategy Design

This project exposes three planning strategies over the same MATLAB scene,
candidate library, collision checker, reachability model, and global path
planner.

## Bottom-Up

Bottom-Up freezes the original RCH-MT-MMP decomposition: task regions are
handled independently, local docking poses are selected first, and global route
ordering happens after all docks have been chosen. It is the regression
baseline. The default weights in `cfg.strategy.bottom_up.weights` match the
previous hard-coded values.

## Top-Down

Top-Down first builds a feasible anchor shortlist for each region. The
shortlist keeps high-coverage, high-reachability, low-local-cost, side-diverse
and unique-task-covering candidates when possible. Region-level costs are then
computed as the minimum feasible grid-planner path cost over filtered anchor
pairs, not from table centers or a single fixed region representative.

After the region order is selected, local dock scoring uses normalized coverage
gain, reachability, local cost, side-switch cost, and route insertion delta.
When the next exact dock is not fixed yet, the route term uses the cheapest
transition from the current candidate to the next region anchor set.

Coverage repair may add docks when the initial budget misses tasks, but it
never marks impossible tasks as complete.

## Bidirectional

Bidirectional starts from the complete Bottom-Up solution and applies
conservative local neighborhoods:

- redundant dock removal;
- single dock replacement inside a region;
- two-to-one dock replacement;
- route reinsertion for a fixed dock set.

Each move is accepted only when the objective improves and predicted coverage
does not decrease. Rejected moves are rolled back by retaining the previous
plan. Iteration diagnostics record accepted/rejected operations, removed docks,
pose replacements, route changes, and rejection reasons.

The objective combines uncovered tasks, dock count, route length, estimated
execution time, and local docking cost. The uncovered-task weight is kept high
so efficiency gains cannot be obtained by silently sacrificing task coverage.

## Shared Boundary

`build_candidate_library` calls `generate_docking_candidates` once per region.
All strategies use the same `candidate_library.scene_signature` and
`candidate_library.total_candidates` in a trial.

## Diagnostics

`plan.diagnostics` uses a stable schema filled by
`planning_strategy/common/build_plan_diagnostics.m`. Comparisons against
Bottom-Up are produced by
`planning_strategy/common/compare_strategy_plans.m`, using configurable pose
tolerances:

- `cfg.strategy.diagnostics.position_tolerance_m`;
- `cfg.strategy.diagnostics.yaw_tolerance_rad`.

Route-order diagnostics use `normalize_visit_order.m` before comparison. The
normal form is a local dock order in `1:num_docks`; start and optional return
nodes are not counted as docks. `build_route_edge_set.m` then reports directed
and undirected dock-to-dock route edges, plus separate start-edge and
return-edge diagnostics. The legacy `route_edge_difference_count` is defined as
the undirected dock-to-dock route-edge symmetric difference.

The execution-time model reports navigation, docking, manipulation, arm
transition, side-switch, turning, lift, and replanning components. Lift and
replanning are zero placeholders unless real controller logs are explicitly
connected. The total is asserted to equal the component sum.

Path-length diagnostics use explicit meter-suffixed fields:

- `task_path_length_m`: start to first task dock plus all task-dock-to-task-dock segments;
- `return_path_length_m`: final task dock to the optional return target;
- `total_path_length_m`: task path plus return path.

## Limits

The reachability score is geometric and does not prove full IK feasibility.
The MATLAB planner exports service targets, not low-level robot commands.
Estimated execution time is a model estimate, not measured robot time.
The strategy comparison has not been validated on Ubuntu 20.04, ROS Noetic, or
a real mobile dual-arm platform.
