function state = create_c2_5_1_dstar_fixture()
    scene = struct();
    scene.start_pose = [0.5, 0.5, 0];
    scene.num_tasks = 1;
    scene.grid_map = zeros(100, 100);
    scene.map_info = struct('resolution', 0.1, 'height_m', 10, 'width_m', 10, 'rows', 100, 'cols', 100);
    scene.scene_cfg.table_positions = [8.5, 8.5];
    scene.scene_cfg.table_region_id = 1;
    scene.scene_cfg.table_orientations = 0;
    scene.scene_cfg.table.length = 0.8; scene.scene_cfg.table.width = 0.5;
    scene.scene_cfg.task_offset_z = 0.5;
    scene.tasks = struct('id', 1, 'x', 8.5, 'y', 8.5, 'z', 0.5, 'status', 0, 'table_id', 1);
    scene.region_metadata = struct('region_count', 1, 'region_ids', 1, 'table_region_id', 1, ...
        'task_region_id', 1, 'tasks_per_region', 1, 'selected_docks_per_region', 1);
    state = struct('scale', 'fixture', 'seed', 1, 'scene', scene);
    state.plan = struct('selected_poses', [8.5, 8.5, 0], 'pose_to_region_map', 1, ...
        'covered_tasks_predicted', 1, 'task_sequence_summary', {{struct('covered_tasks', 1, 'table_id', 1, 'dock_index', 1)}}, ...
        'pose_info_list', {{struct('covered_tasks', 1, 'table_id', 1, 'dock_index', 1)}});
    state.robot_cfg = struct('footprint', struct('length_m', 0.423, 'width_m', 0.366, 'model', 'rectangle'));
    state.all_poses = [scene.start_pose; state.plan.selected_poses];
    state.candidate_library.total_candidates = 1;
    state.planner_cfg = struct('global_planner', 'dstar_lite', 'incremental_replanning_enabled', true, ...
        'footprint_aware_navigation', true, 'robot_cfg', state.robot_cfg, ...
        'astar', struct('allow_diagonal', true, 'heuristic_weight', 1));
    route = [1, 2, 1];
    graph = build_sparse_docking_graph(state.all_poses, state.scene.grid_map, state.scene.map_info, ...
        state.planner_cfg, [0; 1]);
    state.stats.sparse_graph = graph;
    [state.stats.global_path, state.stats.segment_stats] = stitch_sparse_graph_path(state.all_poses, route, graph);
    state.rc2_plan = struct('selected_docks', state.plan.selected_poses, 'route_order', route, ...
        'global_path', state.stats.global_path, 'path_segments', state.stats.segment_stats);
    state.task_state = struct('predicted_tasks', 1, 'commanded_tasks', 1, 'executed_tasks', 1, 'verified_tasks', 1);
end

