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

Top-Down first selects a feasible candidate anchor for each region, builds a
region-level path-cost matrix with the existing grid planner, orders the task
regions, and then selects local docks with entry and exit route context. Table
centers are not used as navigation targets.

Coverage repair may add docks when the initial budget misses tasks, but it
never marks impossible tasks as complete.

## Bidirectional

Bidirectional starts from the complete Bottom-Up solution, computes a global
route, and records route feedback. The first implementation is conservative:
it does not delete or replace docks unless a future refinement can prove that
coverage is not reduced and the objective improves.

## Shared Boundary

`build_candidate_library` calls `generate_docking_candidates` once per region.
All strategies use the same `candidate_library.scene_signature` and
`candidate_library.total_candidates` in a trial.

## Limits

The reachability score is geometric and does not prove full IK feasibility.
The MATLAB planner exports service targets, not low-level robot commands.
Estimated execution time is a model estimate, not measured robot time.
