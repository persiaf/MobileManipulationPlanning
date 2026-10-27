# Quick Start

From MATLAB:

    cd('D:/file_used/西柚/实验室_任务/硕士论文相关/小论文/MobileManipulationPlanning');
    init_paths;

Baseline/headless:

    result = run_selected_experiment('baseline','headless',true,'render_outputs',false);

Cafeteria:

    result = main_cafeteria_mobile_manipulation_planning('complex12',2026,'navigation_docking_only');

GUI:

    addpath(genpath('experiment_manager')); app = launch_mobile_manipulation_experiment_manager();

Runner options are headless, save_record, render_outputs, output_dir, and builder_args.
 