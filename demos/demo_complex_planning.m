function state = demo_complex_planning(profile_id, seed)
%DEMO_COMPLEX_PLANNING Show a sparse Bottom-Up planning overlay.

    if nargin < 1 || isempty(profile_id), profile_id = 'V2'; end
    if nargin < 2 || isempty(seed), seed = 2026; end
    profile_id = upper(char(profile_id));

    root = fileparts(fileparts(mfilename('fullpath')));
    init_paths(root);
    override.planner.strategy.name = 'bottom_up';
    override.planner.routing.mode = 'sparse_knn';
    override.planner.routing.sparse_knn.k = 4;
    override.planner.candidate_pair_prefilter.enable = false;
    effective = resolve_effective_config('profile', profile_id, 'seed', seed, ...
        'experiment_override', override);
    print_effective_config(effective);
    vis_cfg = effective.visualization;
    state = load_demo_state(root, profile_id, seed, effective.planner);
    visualize_planning_overlay(state.scene, state.plan, state.stats, vis_cfg, ...
        'title', sprintf('%s Sparse Bottom-Up Planning Overlay', profile_id), ...
        'filename', sprintf('%s_planning', profile_id));
end

function state = load_demo_state(root, profile_id, seed, planner_cfg)
    scale_id = profile_scale(profile_id);
    state_file = fullfile(root, 'results', 'tables', ...
        sprintf('p3p4_state_%s_seed%d_bottom_up.mat', lower(scale_id), seed));
    if isfile(state_file)
        data = load(state_file);
        state.scene = data.scene;
        state.plan = data.plan;
        state.robot_cfg = data.robot_cfg;
        state.planner_cfg = planner_cfg;
        task_table_map = data.task_table_map;
    else
        state.scene = build_complex_visual_scene(profile_id, seed);
        state.robot_cfg = robot_config();
        state.planner_cfg = planner_cfg;
        task_table_map = assign_tasks_to_tables(state.scene);
        regions = build_task_regions(state.scene, task_table_map);
        library = build_candidate_library(regions, state.scene, state.robot_cfg, planner_cfg);
        state.plan = run_planning_strategy(library, regions, state.scene, state.robot_cfg, planner_cfg);
    end
    [~, global_path, ~, route_order, ~, path_len, stats] = stage3_global_planning( ...
        state.scene, state.plan.selected_poses, state.plan.task_sequence_summary, task_table_map, ...
        state.robot_cfg, planner_cfg, state.plan.pose_info_list, struct(), state.plan);
    stats.global_path = global_path;
    stats.path_len = path_len;
    stats.route_order = route_order;
    state.stats = stats;
end

function scale_id = profile_scale(profile_id)
    switch upper(profile_id)
        case 'V1'
            scale_id = 'L1';
        case 'V2'
            scale_id = 'L2';
        case 'V3'
            scale_id = 'L3';
        otherwise
            error('demo_complex_planning:UnknownProfile', ...
                'Expected V1, V2, or V3; received "%s".', profile_id);
    end
end
