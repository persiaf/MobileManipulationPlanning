function state = p5_fixture_state()
%P5_FIXTURE_STATE Small deterministic state for local replanning tests.

    scene = struct();
    scene.start_pose = [0, 0, 0];
    scene.num_tasks = 6;
    scene.scene_cfg.table_positions = [1, 0; 2, 0; 1, 1; 2, 1; 4, 4; 5, 4];
    scene.scene_cfg.table_region_id = [1; 1; 2; 2; 3; 3];
    scene.scene_cfg.task_offset_z = 0.5;
    for i = 1:6
        scene.tasks(i).id = i;
        scene.tasks(i).x = scene.scene_cfg.table_positions(i, 1);
        scene.tasks(i).y = scene.scene_cfg.table_positions(i, 2);
        scene.tasks(i).z = 0.5;
        scene.tasks(i).status = 0;
        scene.tasks(i).table_id = i;
    end
    scene.region_metadata = struct('region_count', 3, 'region_ids', [1; 2; 3], ...
        'table_region_id', [1; 1; 2; 2; 3; 3], 'task_region_id', [1; 1; 2; 2; 3; 3], ...
        'tasks_per_region', [2; 2; 2], 'selected_docks_per_region', [2; 2; 2]);

    plan = struct();
    plan.selected_poses = [1, 0, 0; 2, 0, 0; 1, 1, 0; 2, 1, 0; 4, 4, 0; 5, 4, 0];
    plan.pose_to_region_map = [1; 1; 2; 2; 3; 3];
    plan.covered_tasks_predicted = 1:6;
    plan.task_sequence_summary = { ...
        struct('covered_tasks', 1, 'table_id', 1, 'dock_index', 1), ...
        struct('covered_tasks', 2, 'table_id', 2, 'dock_index', 2), ...
        struct('covered_tasks', 3, 'table_id', 3, 'dock_index', 3), ...
        struct('covered_tasks', 4, 'table_id', 4, 'dock_index', 4), ...
        struct('covered_tasks', 5, 'table_id', 5, 'dock_index', 5), ...
        struct('covered_tasks', 6, 'table_id', 6, 'dock_index', 6)};
    plan.pose_info_list = plan.task_sequence_summary;
    plan.timing.strategy_total_time_s = 1;

    all_poses = [scene.start_pose; plan.selected_poses];
    adjacency = true(7);
    adjacency(1:8:end) = false;
    cost = zeros(7);
    for i = 1:7
        for j = 1:7
            if i == j
                cost(i, j) = 0;
            else
                cost(i, j) = norm(all_poses(i, 1:2) - all_poses(j, 1:2));
            end
        end
    end

    state = struct();
    state.scale = 'fixture';
    state.seed = 1;
    state.scene = scene;
    state.plan = plan;
    state.all_poses = all_poses;
    state.candidate_library.total_candidates = 12;
    for r = 1:3
        state.candidate_library.regions(r).candidates = repmat( ...
            struct('pose', [r, r, 0], 'covered_tasks', r), 1, 3);
    end
    state.stats.path_len = 4;
    state.stats.route_order = [1, 2, 3, 4, 5, 6, 7, 1];
    state.stats.sparse_graph.adjacency = adjacency;
    state.stats.sparse_graph.cost_matrix = cost;
    state.stats.sparse_graph.region_ids = [0; plan.pose_to_region_map(:)];
    state.stats.sparse_graph.next_hop = ones(7);
    state.stats.sparse_graph.connectivity.ok = true;
    state.stats.timing.stage3_time_s = 1;
    state.stats.timing.expensive_planner_calls = 10;
end
