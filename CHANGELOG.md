# Changelog

## [0.3.0] — 2026-10-28

- Unified maintained scenes on the unchanged main robot; removed restaurant, hardware-matched and alternate physical-model branches, parameters and dedicated outputs.
- Corrected rotated scene geometry, footprint axes and occupied-cell checks, candidate optional arguments, task/table membership, exact navigation endpoints and SE(2) route provenance.
- Added source-bound frozen-scene loading and a fixed threshold/return/timing protocol; separated proxy, geometric, simulated, IK and measured-execution evidence.
- Replaced estimated dynamic references with actual paired replans; synchronized added-task services/state/candidates and committed compact, reload-validated pair snapshots before CSV output.
- Completed 78 core experiment rows and 21 independent weight cases; preserved coverage/routing failures and unknown IK/physical execution. Added 118 passing targeted regression checks and separate full-suite release validation.
- Added root VERSION, portable README instructions, tagged-release notes and authoritative evidence. Ignored interrupted/corrupt runs and host-local pointers; paper-format version and integration schema remain unchanged.
- Recorded the incomplete all-tests attempt after 41 completed files; corrected the outdated A* provenance-label assertion and passed both focused D* rechecks. No full-suite pass is claimed.

Release scope, validation and known limits: [docs/releases/v0.3.0.md](docs/releases/v0.3.0.md).

## 2026-10-27 — Single main-scene robot

- Kept all numeric values in config/robot_config.m and the main staged entry unchanged.
- Removed restaurant scenes/proxies, hardware-matched scenes, the alternate physical geometry, and their dedicated tests and outputs.
- Unified the registry, GUI, config resolution and planning entry points around robot_config; conflicting overrides now fail explicitly.
- Added cross-profile/configuration regression checks. Scope, removed-file hashes and validation are documented in docs/main_robot_unification.md.

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
