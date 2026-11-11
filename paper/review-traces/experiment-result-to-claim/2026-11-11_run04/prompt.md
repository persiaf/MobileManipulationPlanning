# Independent result-to-claim review

Role: independent paper architect/reviewer, `/root/robustness_audit`.

User objective: after saving the current state to GitHub, retain the same arm model and compare staged planning, fixed-set reordering and joint optimization across task distributions and table layouts; assess whether navigation gains persist while retaining task coverage and minimum docking cardinality. Figures should retain the main-scene style.

Predeclared design: `paper/TASK_NAVIGATION_ROBUSTNESS_EXPERIMENT_PLAN_20261111.md`; 3 layouts × 3 task distributions, L0_D0…L2_D2. Every scenario retains 16 tables, 36 tasks, original per-table target counts, robot/TCP/mount/IK settings, map dimensions and start. Each layout has one D0-reference candidate subpool shared across its distributions and task modes. Primary controls are `minimum_seed`, `reorder_only`, `joint_local`; original `staged` is auxiliary. Both comparisons require all three primary methods to pass full-task verification, base navigation and the same regional minimum dock count. J ties use absolute 1e-7. The algorithm and budgets remained fixed during execution.

Evidence: `results/task_navigation_robustness_20261111/`, including the 108-row batch CSV, per-case support matrices, DP certificates, assignments, actual paths, accepted moves and NN recovery matrices. Inputs were independently checked before execution. L0_D0 was independently checked once upon completion and reproduced the preceding main-scene numerical results. Final review uses one final result audit, not repeated experiments or test sweeps.

A post-hoc graph diagnostic was separately registered in `POSTHOC_REORDER_DIAGNOSTIC_PLAN.md` after a 3-pass allocation concern emerged. It uses existing directed D, the same fixed selected set and seed, and best-improvement 2-opt until a complete nonimproving pass or 10000 neighbors. Its existing CSV is checked without rerunning that search or requesting new IK/A*. It does not alter primary controls, mission counts or pair denominators.

Evaluate: (1) whether navigation improvement is stable, (2) which comparisons and conditions support that statement, (3) what explains negative cases, (4) what the graph diagnostic establishes, and (5) the next useful method change. Keep the judgment focused on evidence that advances the argument. Do not upgrade majority wins to universal superiority, round the small negative case to a tie, or count graph-only output as a newly executed mission.

No `paper/PAPER_CLAIM_AUDIT.json` is available, so the paper-facing judgment remains provisional.
