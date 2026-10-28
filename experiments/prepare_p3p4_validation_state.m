function state_file = prepare_p3p4_validation_state(scale, seed, strategy, state_file)
%PREPARE_P3P4_VALIDATION_STATE Save one scene/candidate/docking state.
%
% The saved state lets dense/sparse routing modes reuse the same selected
% docking set in separate short-lived MATLAB processes.

    if nargin < 3 || isempty(strategy), strategy = 'bottom_up'; end
    root = fileparts(fileparts(mfilename('fullpath')));
    init_paths(root);
    cleanup_obj = configure_headless_experiment(); %#ok<NASGU>
    if nargin < 4 || isempty(state_file)
        out_dir = fullfile(root, 'results', 'cache', 'repaired_p3p4_states');
        if ~isfolder(out_dir), mkdir(out_dir); end
        state_file = fullfile(out_dir, sprintf('p3p4_state_%s_seed%d_%s.mat', ...
            lower(scale), seed, lower(strategy)));
    end

    scene = build_scene_from_cfg(large_scale_profile_config(scale, seed));
    robot_cfg = robot_config();
    planner_cfg = apply_experiment_protocol(p3p4_base_config(strategy, false));
    task_table_map = assign_tasks_to_tables(scene);
    regions = build_task_regions(scene, task_table_map);
    candidate_library = build_candidate_library(regions, scene, robot_cfg, planner_cfg);
    plan = run_planning_strategy(candidate_library, regions, scene, robot_cfg, planner_cfg);
    covered_by_library = library_covered_tasks(candidate_library);
    candidate_uncovered_tasks = setdiff(1:scene.num_tasks, covered_by_library, 'stable');
    model_infeasible_tasks = []; % No independent physical-infeasibility certificate.
    validation_identity = demo_state_cache_identity(scene,robot_cfg,planner_cfg,scale,seed);
    region_metadata = summarize_region_metadata(scene, candidate_library, pose_cluster_regions(plan, scene));

    save(state_file, 'scale', 'seed', 'strategy', 'scene', 'robot_cfg', ...
        'planner_cfg', 'task_table_map', 'regions', 'candidate_library', ...
        'plan', 'model_infeasible_tasks', 'candidate_uncovered_tasks', ...
        'region_metadata','validation_identity','-v7.3');
    fprintf('P3P4_STATE_READY file=%s scale=%s seed=%d strategy=%s tasks=%d docks=%d\n', ...
        state_file, scale, seed, strategy, scene.num_tasks, plan.num_docking_poses);
end

function cfg = p3p4_base_config(strategy, cache_on)
    cfg = planner_config();
    cfg.strategy.name = strategy;
    cfg.strategy.bidirectional.use_shared_path_cost_cache = cache_on;
    cfg.routing.mode = 'dense';
    cfg.candidate_pair_prefilter.enable = false;
    cfg.runtime.disable_visualization = true;
    cfg.runtime.disable_video = true;
    cfg.runtime.quiet = true;
    cfg.runtime.quiet_stage2 = true;
    cfg.benchmark.enable = false;
    cfg.visualization.show_scene = false;
    cfg.visualization.show_docking = false;
    cfg.visualization.show_global_path = false;
end

function ids = pose_cluster_regions(plan, scene)
    ids = ones(plan.num_docking_poses, 1);
    if ~isfield(scene.scene_cfg, 'table_region_id'), return; end
    table_region_id = scene.scene_cfg.table_region_id(:);
    for i = 1:numel(plan.pose_info_list)
        info = plan.pose_info_list{i};
        if isstruct(info) && isfield(info, 'table_id') && ...
                info.table_id >= 1 && info.table_id <= numel(table_region_id)
            ids(i) = table_region_id(info.table_id);
        end
    end
end

function tasks = library_covered_tasks(library)
    tasks = [];
    for ri = 1:numel(library.regions)
        for ci = 1:numel(library.regions(ri).candidates)
            tasks = union(tasks, library.regions(ri).candidates(ci).covered_tasks, 'stable');
        end
    end
end
