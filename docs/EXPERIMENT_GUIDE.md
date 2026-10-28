# Experiment Guide

## Stage 1

Baseline uses scene_config, robot_config, and planner_config. All maintained experiments use the same robot_config parameters as main_mobile_manipulation_planning.m. C0-C3, L1-L3, V1-V3 and the four-table scene vary layout/task scale without changing the robot. Restaurant and hardware-matched scenes and the alternate physical-robot profile have been removed; see main_robot_unification.md.

## Stage 2

L1/L2/L3 are large-scale profiles. Sparse routing is planner_cfg.routing.mode='sparse_knn' with planner_cfg.routing.sparse_knn.k. Frozen P3/P4 state files validate downstream behavior; they are not full upstream reruns.

## Stage 3

P5 applies ADD_LOCAL_OBSTACLE, INVALIDATE_DOCK, and ADD_TASKS. The current exp_dynamic_local_replanning_p5 delegates to run_protocol_dynamic_comparison: generate a fresh MAIN4 state, create one changed scene, and actually run LOCAL_REPLAN and FULL_REPLAN against it. MAIN4 is an explicitly labeled four-table subset of the main task distribution, not L2. Estimated references are rejected. Both timers include internal route validation; the same extra paired certification is outside both timers and recorded separately. Pair MAT files retain the initial, changed and both resulting planning states.

## Repaired evidence suite

Run `run_repair_evidence_suite()` after the repair tests. It performs a discarded warm-up and then runs three seeds (2026-2028), 15 one-factor sampling/weight variants on an explicit four-table C2 subset, all full C0-C3 profiles, main-scene references and paired dynamic events. The same robot, threshold 0.30, nominal model, return condition and timing scope apply. Full-sized complex profiles retain their own layout/task distributions, so comparisons between C0-C3 are whole-scenario comparisons, not isolated causal tests of rotation.

Independent reachability uses a separately implemented geometric scorer and a bounded denser pose search for library-uncovered tasks. It supplies geometric witnesses, not IK or physical execution proof; finite search failures remain unresolved. No tasks are removed from the denominator. See [repair report](geometry_protocol_evidence_repair.md) for checks, raw results and limitations. Old frozen-state routing replays remain explicitly labeled downstream checks and reject stale implementation identities.

Strategies are bottom_up, top_down, and bidirectional.
