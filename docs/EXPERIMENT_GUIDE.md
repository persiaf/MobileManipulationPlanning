# Experiment Guide

## Stage 1

Baseline uses scene_config, robot_config, and planner_config. Cafeteria has basic3 and complex12. navigation_docking_only is a navigation/docking proxy; full_manipulation is the formal manipulation mode when supported. Inspect results/tables/cafeteria_* and validation logs.

## Stage 2

L1/L2/L3 are large-scale profiles. Sparse routing is planner_cfg.routing.mode='sparse_knn' with planner_cfg.routing.sparse_knn.k. Frozen P3/P4 state files validate downstream behavior; they are not full upstream reruns.

## Stage 3

P5 applies ADD_LOCAL_OBSTACLE, INVALIDATE_DOCK, and ADD_TASKS. Flow: static state -> event -> local_replan -> result row. By default exp_dynamic_local_replanning_p5 uses frozen L2 state and an estimated full-replan reference. Check current CSVs before claiming independent footprint-waypoint exports.

Strategies are bottom_up, top_down, and bidirectional.
