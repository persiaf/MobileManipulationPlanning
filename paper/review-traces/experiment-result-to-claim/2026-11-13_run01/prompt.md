# Result-to-claim evaluation request

Evaluate `results/joint_attribution_20261113/` against the user requirement: preserve arm reachability, task coverage evidence and all existing scenes; improve joint initialization/search allocation; determine whether gains actually require changing docking poses rather than insufficient fixed-set search.

The protocol compares F (original docking set, exact ATSP order), I (F initialization, old search schedule), N (raw route initialization, new schedule), and predeclared J (F initialization, new schedule). It also solves the old joint output's fixed docking set as a post-hoc attribution diagnostic. All 27 scene/task combinations and all four methods are retained.

Run the independent exported-data audit once; inspect frozen hashes, support/cardinality, sampled base geometry, route cost, moves, bound trace arithmetic and derived claims. Do not rerun IK, navigation or optimization. Then judge bounded docking attribution separately from broad allocation/initialization superiority. Preserve negative ablations; do not select a different primary method after seeing results.

Inputs reviewed: frozen protocol; production ATSP, new budgeted planner and runner; paired and aggregate results; `independent_audit.json`; analysis report and figure scripts. The reviewer also visually inspected the certification and initialization/budget ablation figures.
