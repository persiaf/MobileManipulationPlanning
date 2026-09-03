function demo_complex_planning()
%DEMO_COMPLEX_PLANNING Show a sparse Bottom-Up planning overlay.

    root = fileparts(fileparts(mfilename('fullpath')));
    init_paths(root);
    vis_cfg = visualization_config();
    state = load_demo_state(root);
    visualize_planning_overlay(state.scene, state.plan, state.stats, vis_cfg, ...
        'title', 'V2 Sparse Bottom-Up Planning Overlay', ...
        'filename', 'V2_planning');
end

function state = load_demo_state(root)
    state_file = fullfile(root, 'results', 'tables', 'p3p4_state_l2_seed2026_bottom_up.mat');
    if isfile(state_file)
        data = load(state_file);
        state.scene = data.scene;
        state.plan = data.plan;
        state.robot_cfg = data.robot_cfg;
        state.planner_cfg = data.planner_cfg;
        state.planner_cfg.routing.mode = 'sparse_knn';
        state.planner_cfg.routing.sparse_knn.k = 4;
        state.all_poses = [state.scene.start_pose; state.plan.selected_poses];
        state.stats = demo_sparse_stats(state);
        return;
    end
    scene = build_complex_visual_scene('V1', 2026);
    robot_cfg = robot_config();
    planner_cfg = planner_config();
    planner_cfg.strategy.name = 'bottom_up';
    planner_cfg.routing.mode = 'sparse_knn';
    planner_cfg.routing.sparse_knn.k = 4;
    planner_cfg.candidate_pair_prefilter.enable = false;
    task_table_map = assign_tasks_to_tables(scene);
    regions = build_task_regions(scene, task_table_map);
    library = build_candidate_library(regions, scene, robot_cfg, planner_cfg);
    plan = run_planning_strategy(library, regions, scene, robot_cfg, planner_cfg);
    [~, global_path, ~, route_order, ~, path_len, stats] = stage3_global_planning( ...
        scene, plan.selected_poses, plan.task_sequence_summary, task_table_map, ...
        robot_cfg, planner_cfg, plan.pose_info_list, struct(), plan);
    stats.global_path = global_path;
    stats.path_len = path_len;
    stats.route_order = route_order;
    state = struct('scene', scene, 'plan', plan, 'stats', stats, ...
        'robot_cfg', robot_cfg, 'planner_cfg', planner_cfg);
end

function stats = demo_sparse_stats(state)
    poses = state.all_poses;
    adjacency = knn_graph(poses, 4);
    stats = struct();
    stats.sparse_graph = struct('adjacency', adjacency);
    stats.global_path = poses(1:min(size(poses, 1), 20), 1:2);
    stats.route_order = 1:size(poses, 1);
    stats.path_len = sum(sqrt(sum(diff(stats.global_path).^2, 2)));
end

function adjacency = knn_graph(poses, k)
    n = size(poses, 1);
    adjacency = false(n);
    for i = 1:n
        d = sqrt(sum((poses(:, 1:2) - poses(i, 1:2)).^2, 2));
        d(i) = inf;
        [~, order] = sort(d);
        nbrs = order(1:min(k, n - 1));
        adjacency(i, nbrs) = true;
        adjacency(nbrs, i) = true;
    end
end
