# MobileManipulationPlanning

Research release **[v0.3.0](https://github.com/persiaf/MobileManipulationPlanning/tree/v0.3.0)** (2026-10-28), maintained on [`codex/v0.3.0-main-robot-consistency`](https://github.com/persiaf/MobileManipulationPlanning/tree/codex/v0.3.0-main-robot-consistency). The default `main` branch remains the earlier baseline; this release does not merge the paper-format branch. See [release notes](docs/releases/v0.3.0.md) and [CHANGELOG](CHANGELOG.md).

MATLAB research code for task-driven docking and navigation planning for a mobile dual-arm robot. Main chain: scene -> tasks -> docking candidates -> dock selection -> route -> footprint validation. MATLAB performs offline/simulated planning, not low-level robot control.

An opt-in [task-conditioned arm workspace experiment](docs/TASK_CONDITIONED_ARM_WORKSPACE_20261110.md) adds pinned VX300s kinematics, downward grasping and sampled wiping-path IK. Run `addpath('pipeline'); init_paths; report = demo_task_conditioned_workspace;`. The historical default remains unchanged; the optional arm mounts are unmeasured and arm collisions/contact forces are not checked.

The [controlled two-table comparison](results/task_constraint_comparison_20261111/EXPERIMENT_PROTOCOL.md) freezes the arm, ten targets, candidate poses and navigation rules across position-only, downward-grasp and wiping tasks. Run `result = exp_task_constraint_comparison;` after path initialization, then `python experiments/audit_task_constraint_comparison.py` and `python visualization/task_constraint_comparison/run_all.py`. CSV evidence, vector figures and [analysis](results/task_constraint_comparison_20261111/RESULTS_ANALYSIS.md) are retained separately from the historical baseline outputs.

## Research stages

| Stage | Scope | Primary entry | Boundary |
|---|---|---|---|
| Stage 1 | Main-scene baseline | main_mobile_manipulation_planning.m | One robot from config/robot_config.m |
| Stage 2 | Large-scale L1/L2/L3 and sparse routing | experiments/exp_large_scale_*.m | Frozen-state validation is not a full rerun |
| Stage 3 | P5 dynamic D1/D2/D3 | experiments/exp_dynamic_local_replanning_p5.m | Fresh paired local/full replans; identical changed scene |

## Requirements and quick start

MATLAB R2022a or newer is the repository recommendation; an exact version/toolbox pin is not explicitly pinned. ROS Toolbox is not required.

    cd('/path/to/MobileManipulationPlanning');
    addpath(fullfile(pwd,'pipeline'));
    init_paths;
    addpath(fullfile(pwd,'experiment_manager'));
    result = run_selected_experiment('baseline','headless',true,'render_outputs',false,'save_record',true);

Original staged pipeline: run('main_mobile_manipulation_planning.m'). GUI: addpath(genpath('experiment_manager')); app = launch_mobile_manipulation_experiment_manager();

## Registered profiles

| Profile | Purpose | Status |
|---|---|---|
| baseline | Default scene | Maintained |
| strategy_comparison | Compare strategies | Maintained |
| L1/L2/L3 | Large-scale studies | Experiment/frozen-state workflow |
| D1/D2/D3 | ADD_LOCAL_OBSTACLE, INVALIDATE_DOCK, ADD_TASKS | P5 validation workflow |

All maintained scenes use the exact robot parameters from config/robot_config.m, as used by main_mobile_manipulation_planning.m. Restaurant scenes, the alternate physical-robot geometry, and hardware-matched scenes were removed on 2026-10-27. See docs/main_robot_unification.md for parameters, removal scope, and historical-result boundaries.

Geometry, protocol and evidence repairs are documented in [the repair report](docs/geometry_protocol_evidence_repair.md). Formal comparisons fix the main threshold (0.30), nominal geometry and return to start. Reports distinguish proxy support, geometric coverage, simulated geometric completion, IK validation and measured completion; missing IK/measured evidence is `NaN`.

    init_paths;
    evidence = run_repair_evidence_suite();

The evidence suite runs sequentially after a discarded warm-up: three main-scene references, 45 sampling/weight cases, 12 full C0-C3 cases and nine paired dynamic events. It preserves blocked and failed cases, inputs, per-task independent geometric audits and implementation signatures. Historical outputs are not overwritten.

The completed repair evidence has 78 core rows plus a separately warmed 21-row MAIN4 weight supplement; all 118 relevant tests passed. C0/C1 completed all three seeds, C2 retained coverage failures, C3 completed two seeds, and the original main scene retained routing failures despite full geometric coverage. See the report for the supported conclusions, actual paired dynamic results and source-snapshot revisions.

Release validation also reran both D* fallback checks successfully. The all-tests attempt ended before a final summary after 41 test files; **a full-suite pass is not claimed**. Its incomplete-run record and the corrected fallback-provenance recheck are linked in the release notes.

The [tracked evidence](results/repair_20261027/evidence_20261027_221823/) includes CSVs, per-task audits, compact MAT snapshots, source manifests and verification reports. Interrupted/corrupt output, caches and host-specific pointers are excluded from Git and retained locally. To reproduce the summaries without rerunning MATLAB experiments:

```sh
python results/repair_20261027/analyze_repair_evidence.py --evidence-dir results/repair_20261027/evidence_20261027_221823
python results/repair_20261027/analyze_weight_completion_supplement.py --evidence-dir results/repair_20261027/evidence_20261027_221823
```

To rerun the 118 targeted checks in MATLAB, use `run('results/repair_20261027/run_repair_validation.m')`. To run the separate 21-case weight supplement in a new MATLAB process, add `results/repair_20261027` to the path and call `run_weight_completion_supplement`; existing output is preserved by refusing to overwrite it. Full-suite release validation is recorded separately from the targeted checks.

## Algorithms and configuration

Task strategies: bottom_up, top_down, bidirectional. Global planner dispatch: convex_astar, astar, jps, gbfs, theta_star, lazy_theta_star. Routing: dense and sparse_knn. P5 local replanning is event-driven and separate from global planner selection.

Effective configuration order: BASE -> SCENE PROFILE -> EXPERIMENT OVERRIDE -> USER/GUI OVERRIDE. Base files are config/scene_config.m, config/robot_config.m, config/planner_config.m. Robot overrides must match robot_config.m; configuration and planning entry points reject conflicting robot parameters. Scene size, layout, task distribution, and planner settings can vary independently. The legacy_validated name is an identical compatibility alias, with no separate geometry.

## Outputs, visualization, tests

Managed records: results/managed_experiments/<experiment_id>/. Tables: results/tables/. Figures: results/figures/. Canonical visualization settings: visualization/visualization_config.m. Sparse graph lines mean topological connectivity; final route means planner waypoint path.

    addpath(fullfile(pwd,'pipeline'));
    init_paths;
    addpath(fullfile(pwd,'experiment_manager'));
    results = run_all_tests();

See docs/QUICK_START.md, docs/EXPERIMENT_GUIDE.md, docs/CODE_ARCHITECTURE.md, docs/CONFIGURATION_GUIDE.md, docs/DEVELOPMENT_HISTORY.md, docs/TROUBLESHOOTING.md, docs/FILE_INDEX.md.

Repository maintenance and version boundaries are described in [current_code_structure.md](docs/current_code_structure.md). Project version `0.3.0` is recorded in [VERSION](VERSION); the mission-plan schema remains `1.0`, evidence metrics use `2.0`, and `rch-mt-mmp-matlab` remains the integration planner identifier. The independent `paper_tac/VERSION` is unchanged.
