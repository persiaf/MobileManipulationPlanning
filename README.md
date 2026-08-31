# MobileManipulationPlanning

MATLAB implementation of the RCH-MT-MMP hierarchical planner for mobile dual-arm manipulation. The repository keeps the paper simulation pipeline intact while adding a maintainable data boundary for a Windows planning workstation and an Ubuntu 20.04/ROS Noetic execution computer.

The MATLAB side performs offline scene construction, docking-pose selection, route ordering, path planning, and mission export. It does not directly control a robot and does not require ROS Toolbox.

## Planning pipeline

```text
simulation or measured scene input
  -> task-region construction
  -> docking candidate generation
  -> geometric dual-arm reachability scoring
  -> multi-docking task coverage
  -> global visit ordering and path planning
  -> mission_plan.json
  -> ROS execution process (separate repository/process)
  -> execution_log.json or execution_log.csv
  -> MATLAB analysis
```

Geometric reachability is a screening model. It is not a claim of collision-aware inverse-kinematics or coordinated bimanual feasibility. The adapter in `manipulator/evaluate_ik_feasibility.m` intentionally raises an error until a real robot adapter is supplied.

## Quick start

MATLAB R2022a or newer is recommended.

```matlab
run('main_mobile_manipulation_planning.m')
```

The main entry point resolves the project root, initializes paths, creates the configured simulation scene, selects service poses, plans the global route, and runs the simulation execution model.

Configuration is split into:

- `config/scene_config.m`: map, tables, tasks, and docking sampling.
- `config/robot_config.m`: chassis and geometric arm-workspace parameters.
- `config/planner_config.m`: heuristic costs, global planner, execution timing, experiments, and visualization.
- `config/real_platform_config.m`: frame, tolerances, discrete lift heights, schema version, and default interface paths.

Key strategy and timing options in `planner_config.m`:

- `cfg.strategy.name`: `bottom_up`, `top_down`, or `bidirectional`.
- `cfg.strategy.diagnostics.position_tolerance_m`: pose-match tolerance for strategy diagnostics.
- `cfg.strategy.diagnostics.yaw_tolerance_rad`: yaw-match tolerance for strategy diagnostics.
- `cfg.strategy.top_down.max_anchors_per_region`: maximum candidate anchors kept per task region.
- `cfg.strategy.top_down.max_anchor_pair_queries`: nearest anchor-pair path queries used for region transition costs.
- `cfg.strategy.top_down.preserve_side_diversity`: preserve candidates from different table sides when possible.
- `cfg.strategy.top_down.preserve_unique_task_cover`: preserve unique task-covering candidates when possible.
- `cfg.strategy.bidirectional.enable_redundant_removal`: try coverage-preserving dock removal.
- `cfg.strategy.bidirectional.enable_single_replacement`: try one-for-one dock replacement.
- `cfg.strategy.bidirectional.enable_two_to_one_replacement`: try replacing two docks with one dock.
- `cfg.strategy.bidirectional.enable_route_reinsertion`: try route reinsertion over the selected dock set.
- `cfg.execution_time.*`: unified aliases for the simulated execution-time model. These values are model parameters, not measured robot execution data.

## Planning Strategies

The planner supports three framework-level strategies through `cfg.strategy.name`:

- `bottom_up`: the original RCH-MT-MMP baseline. It selects local docks first and then orders the global route.
- `top_down`: orders task regions using feasible candidate-anchor shortlists and grid-planner path costs, then selects route-aware local docks with coverage repair.
- `bidirectional`: starts from the Bottom-Up feasible solution and tries conservative neighborhood refinements without reducing predicted coverage.

In the current single-scene experiment, the conservative bidirectional refinement accepted no neighborhood modification and therefore retained the Bottom-Up solution. This should not be described as an observed efficiency improvement.

Run the single-scene comparison:

```matlab
run('experiments/exp_compare_hierarchical_strategies.m')
```

All three strategies can share one `candidate_library`, so a comparison trial uses the same scene, candidate poses, collision checks, reachability model, and global path planner.

## Execution-state semantics

The planner maintains four distinct task sets:

- `predicted_tasks`: stage-two geometric coverage predictions.
- `commanded_tasks`: tasks explicitly assigned to a visited service pose.
- `executed_tasks`: tasks reported as attempted by the executor.
- `verified_tasks`: tasks accepted by the simulation verifier, or by real sensor feedback after a real execution adapter is implemented.

The execution stage never searches the entire scene for extra tasks. Completion statistics use verified tasks, and report predicted coverage, commanded-task, and verified-completion rates separately.

## Real-platform interface

The stable cross-platform boundary is UTF-8 JSON/CSV:

- `real_scene.json`: versioned measured scene, map metadata, tables, objects, and task definitions.
- `mission_plan.json`: ordered service configurations generated by MATLAB.
- `execution_log.json` or `.csv`: measured execution feedback returned by Linux.

Schemas live in `integration/schemas/`. See `docs/data_contracts.md` and `docs/real_platform_integration.md` before exchanging data with Linux.

Example export:

```matlab
real_cfg = real_platform_config();
service = build_service_config(1, pose_info_list{1}, ...
    task_sequence_summary{1}, real_cfg);
service = expand_lift_candidates(service, real_cfg);

mission = export_mission_plan_json( ...
    fullfile('results', 'real', 'mission_plan.json'), ...
    scene, 1:numel(service), service, real_cfg);
```

`lift.enabled=false` preserves the existing base-pose planning behavior and produces one service configuration per selected pose. When enabled, the interface expands the same base pose over configured discrete heights; it does not perform continuous base/lift/arm co-optimization.

## Tests

```matlab
run('tests/run_all_tests.m')
```

The suite covers path initialization, a small planning smoke test, candidate generation, explicit task-command consistency, mission-plan JSON round trips, coordinate round trips, hierarchical strategy consistency, execution-time decomposition, strategy plan-difference metrics, region-anchor shortlist preservation, and removal of obsolete learning-related references.

## Repository layout

```text
config/             simulation, robot, planner, and real-platform configuration
environment/        scene/grid builders and measured scene/map import
collision/          footprint collision and service-pose validation
manipulator/        geometric single/dual/bimanual reachability and IK adapter boundary
docking/            reachability field, candidates, multi-pose selection, service configs
global_planner/     dynamic planner dispatch, wrappers, route ordering, path stitching
pipeline/           five-stage simulation pipeline
execution/          timing, task-state semantics, and simulated execution
integration/        JSON/CSV import/export, validation, and schemas
calibration/        loaders for measured calibration artifacts
experiments/        simulation studies and real-data analysis templates
tests/              MATLAB automated tests
docs/               architecture, contracts, integration notes, paper notes, and media
results/figures/     retained publication figures
results/tables/      retained publication tables
results/raw/         minimal reproducibility data
results/real/        generated mission plans and compact real-run outputs
results/logs/        local execution logs (ignored by Git)
```

## Global planners

`global_planner/run_path_planner.m` dispatches algorithms dynamically. Wrapper files must not be removed based only on direct-call search results. The default benchmark set is A*, JPS, GBFS, and Theta*. Extended deterministic and stochastic planners remain selectable for controlled comparisons. See `docs/planner_algorithms.md`.

## Current Results

Path-length fields use explicit units:

- `task_path_length_m`: start to the first task dock plus all task-dock-to-task-dock path segments.
- `return_path_length_m`: final task dock to the optional return target. It is zero when return-to-start is disabled.
- `total_path_length_m`: `task_path_length_m + return_path_length_m`.

The current single-scene strategy comparison was regenerated on 2026-07-24.

| Strategy | Verified completion | Docks | Task path | Return path | Total path | Estimated mission time | Main diagnostic |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| bottom_up | 100% | 27 | 57.34 m | 5.95 m | 63.29 m | 621.72 s | Baseline reference |
| top_down | 100% | 27 | 63.67 m | 3.41 m | 67.08 m | 728.57 s | Different order and six different dock poses; higher turning time |
| bidirectional | 100% | 27 | 57.34 m | 5.95 m | 63.29 m | 621.72 s | Accepted no neighborhood moves and retained Bottom-Up solution |

Route-difference diagnostics are computed after normalizing final visit orders to local dock indices. The legacy `route_edge_difference_count` is the undirected dock-to-dock route-edge symmetric difference.

| Comparison target | Region-order diff | Visit-order diff | Pose diff | Task-assignment diff | Directed edge diff | Undirected edge diff | Accepted bidirectional moves |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| bottom_up vs bottom_up | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| top_down vs bottom_up | 15 | 22 | 6 | 26 | 44 | 28 | 0 |
| bidirectional vs bottom_up | 0 | 0 | 0 | 0 | 0 | 0 | 0 |

Source:

- `results/tables/strategy_comparison_summary.csv`
- `results/tables/strategy_time_breakdown.csv`
- `results/tables/strategy_plan_differences.csv`

Main outputs:

- `results/raw/strategy_comparison_records.mat`
- `results/tables/strategy_comparison_summary.csv`
- `results/tables/strategy_time_breakdown.csv`
- `results/tables/strategy_plan_differences.csv`
- `results/figures/fig_strategy_route_comparison.png`
- `results/figures/fig_strategy_route_comparison.pdf`
- `results/figures/fig_strategy_tradeoff.png`
- `results/figures/fig_strategy_tradeoff.pdf`
- `results/figures/fig_strategy_time_breakdown.png`
- `results/figures/fig_strategy_time_breakdown.pdf`
- `results/figures/fig_strategy_plan_differences.png`
- `results/figures/fig_strategy_plan_differences.pdf`

## Limitations

Geometric reachability is not full IK. MATLAB performs offline planning and exports service targets, not low-level robot commands. Estimated mission time is simulated/model-estimated time based on `total_path_length_m`, turning, docking, manipulation, and arm-transition terms; it is not measured robot execution time.

The source tree and MATLAB simulation interfaces are testable on Windows. The repository does not claim that ROS connectivity, base control, arm IK, ACT policy execution, or real-platform success rates have been validated on Ubuntu 20.04, ROS Noetic, or a real mobile dual-arm platform.

Store large measured datasets outside the repository. Import them through relative paths recorded in run metadata; do not embed machine-specific Windows or Linux absolute paths in exchanged JSON.

## Documentation

- `docs/current_code_structure.md`: current source-tree inventory, file responsibilities, data flow, and maintenance rules.
- `docs/hierarchical_strategy_design.md`: detailed Bottom-Up, Top-Down, and Bidirectional design notes.
- `docs/strategy_experiment_protocol.md`: strategy comparison protocol, metrics, and interpretation rules.
- `docs/planner_algorithms.md`: global planner wrappers and benchmark algorithms.
- `docs/data_contracts.md`: JSON/CSV exchange contracts.
- `docs/real_platform_integration.md`: real-platform integration boundary.

## README maintenance checklist

When changing code, keep this README synchronized with:

- new or renamed run scripts in `experiments/`;
- new configuration fields in `config/*.m`;
- new ablation modes, strategy names, or benchmark algorithms;
- new result files under `results/raw`, `results/tables`, or `results/figures`;
- updated experimental numbers, always with the exact source CSV/MAT file and generation date;
- removed functions, scripts, modes, or stale output paths.

## GitHub Maintenance and Version Status

The canonical GitHub repository is `https://github.com/persiaf/MobileManipulationPlanning`. The repository currently has two related version lines:

- planning code: the MATLAB project on the strategy-development branch, with interface `schema_version = 1.0` and `planner_version = rch-mt-mmp-matlab`;
- paper package: `paper_tac/VERSION` is `0.2.0-format-revision`, maintained on branch `paper/tac-format-v0.2.0`.

At the time of this update, the local working tree contains uncommitted planning and experiment changes after the `origin/main` snapshot (`1a03b74`, tagged `v0.2.0-hierarchical-strategies`). Those changes must not be described as published on GitHub until they are reviewed, committed, and pushed. The current paper-format commit is `1448082` (`paper-tac-v0.2.0-format`); it is also local branch state and should be checked against the remote branch before release.

Before updating the GitHub project README or creating a release:

1. Run `run('tests/run_all_tests.m')` in MATLAB and record the result.
2. Re-run the experiment scripts that support numbers shown in this README; update the date and source CSV/MAT paths together with every changed number.
3. Check that `README.md`, `CHANGELOG.md`, `docs/current_code_structure.md`, `paper_tac/VERSION`, and the three JSON schemas describe the same code and interface version.
4. Review `git status`, commit source/docs/results as a coherent change, and push the intended branch. Do not push temporary logs, caches, editor files, large sensor data, or generated build intermediates.
5. After pushing, verify the GitHub default-branch README, branch names, tags, release notes, and repository description. If the GitHub description or topics change, record the change in the changelog or release notes.
6. Keep the planning-code release and paper-format release separate; do not change the root project version merely because the LaTeX paper format version changes.

The GitHub README is a published interface: it must only report committed and reproducible behavior. In particular, retain the explicit limitations that geometric reachability is not full IK and that ROS/Ubuntu/real-platform execution remains unverified unless new evidence and tests are added.
