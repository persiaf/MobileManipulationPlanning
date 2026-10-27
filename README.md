# MobileManipulationPlanning

MATLAB research code for task-driven docking and navigation planning for a mobile dual-arm robot. Main chain: scene -> tasks -> docking candidates -> dock selection -> route -> footprint validation. MATLAB performs offline/simulated planning, not low-level robot control.

## Research stages

| Stage | Scope | Primary entry | Boundary |
|---|---|---|---|
| Stage 1 | Baseline and cafeteria | main_mobile_manipulation_planning.m; main_cafeteria_mobile_manipulation_planning.m | Use current validation CSV/logs |
| Stage 2 | Large-scale L1/L2/L3 and sparse routing | experiments/exp_large_scale_*.m | Frozen-state validation is not a full rerun |
| Stage 3 | P5 dynamic D1/D2/D3 | experiments/exp_dynamic_local_replanning_p5.m | Reuses frozen P3/P4 state by default |

## Requirements and quick start

MATLAB R2022a or newer is the repository recommendation; an exact version/toolbox pin is not explicitly pinned. ROS Toolbox is not required.

    cd('D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning');
    init_paths;
    result = run_selected_experiment('baseline','headless',true,'render_outputs',false,'save_record',true);

Original staged pipeline: run('main_mobile_manipulation_planning.m'). Cafeteria: main_cafeteria_mobile_manipulation_planning('complex12',2026,'navigation_docking_only'). GUI: addpath(genpath('experiment_manager')); app = launch_mobile_manipulation_experiment_manager();

## Registered profiles

| Profile | Purpose | Status |
|---|---|---|
| baseline | Default scene | Maintained |
| cafeteria_basic3 | 3-task cafeteria | Maintained |
| cafeteria_complex12 | 12-task cafeteria | Maintained |
| hardware_matched_lab | Hardware-matched lab | Maintained |
| strategy_comparison | Compare strategies | Maintained |
| L1/L2/L3 | Large-scale studies | Experiment/frozen-state workflow |
| D1/D2/D3 | ADD_LOCAL_OBSTACLE, INVALIDATE_DOCK, ADD_TASKS | P5 validation workflow |

navigation_docking_only validates navigation/docking proxies only; it does not prove physical arm reachability. full_manipulation invokes formal manipulation reachability when supported.

## Algorithms and configuration

Task strategies: bottom_up, top_down, bidirectional. Global planner dispatch: convex_astar, astar, jps, gbfs, theta_star, lazy_theta_star. Routing: dense and sparse_knn. P5 local replanning is event-driven and separate from global planner selection.

Effective configuration order: BASE -> PROFILE -> EXPERIMENT OVERRIDE -> USER/GUI OVERRIDE. Base files are config/scene_config.m, config/robot_config.m, config/planner_config.m. Do not casually change geometry, reachability, candidate generation, footprint semantics, or historical profiles.

## Outputs, visualization, tests

Managed records: results/managed_experiments/<experiment_id>/. Tables: results/tables/. Figures: results/figures/. Canonical visualization settings: visualization/visualization_config.m. Sparse graph lines mean topological connectivity; final route means planner waypoint path.

    init_paths;
    run('tests/run_all_tests.m');

See docs/QUICK_START.md, docs/EXPERIMENT_GUIDE.md, docs/CODE_ARCHITECTURE.md, docs/CONFIGURATION_GUIDE.md, docs/DEVELOPMENT_HISTORY.md, docs/TROUBLESHOOTING.md, docs/FILE_INDEX.md.
