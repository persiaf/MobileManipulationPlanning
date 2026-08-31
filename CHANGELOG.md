# Changelog

## 2026-08-31

- Added `docs/current_code_structure.md` as the current source-tree and file-responsibility inventory.
- Added GitHub maintenance rules covering branch/version separation, README and schema synchronization, release evidence, generated-file exclusions, and post-push verification.
- Recorded the current local-versus-`origin/main` version boundary; uncommitted work is not considered published GitHub content.

## 2026-07-24

- Fixed strategy route diagnostics by normalizing final visit orders to local dock indices before comparing route edges.
- Added explicit directed, undirected, start-edge, return-edge, and visit-order difference metrics.
- Standardized strategy summary path fields as `task_path_length_m`, `return_path_length_m`, and `total_path_length_m`, with consistency assertions.
- Regenerated the single-scene strategy comparison; Bidirectional now correctly reports zero route-edge difference when it retains the Bottom-Up solution.
- Simplified README by merging duplicated validation/strategy-result sections and clarifying Bidirectional accepted no neighborhood move in the current single-scene run.
- Added execution-time component diagnostics for navigation, docking, manipulation, arm transition, side switch, turning, lift, and replanning.
- Added strategy plan-difference diagnostics relative to Bottom-Up.
- Upgraded Top-Down region abstraction from one representative anchor to candidate anchor shortlists with side diversity and unique-task coverage preservation.
- Updated Top-Down region transition cost to use feasible anchor-pair path costs and route insertion context during local dock selection.
- Added conservative Bidirectional neighborhoods for redundant removal, replacement, two-to-one replacement, and route reinsertion with rollback on non-improving or coverage-reducing moves.
- Extended the hierarchical strategy comparison experiment with time-breakdown and plan-difference CSV/figure outputs.
- Added focused MATLAB tests for execution-time decomposition, plan-difference metrics, and region-anchor shortlist preservation.
- Documented that estimated execution time is model-estimated and that Ubuntu 20.04, ROS Noetic, and real-platform validation remain unverified.

## 2026-07-20

- Removed the optional LNN scoring/training stack, dedicated experiments, models, caches, and generated results.
- Restored a single deterministic heuristic docking cost path.
- Separated predicted, commanded, executed, and verified task states.
- Added versioned JSON/CSV integration contracts, calibration loaders, discrete lift service configurations, real-data experiment templates, and MATLAB tests.
- Removed unused functions, stale archives, editor/interpreter caches, partial result files, and the disconnected Python prototype.
