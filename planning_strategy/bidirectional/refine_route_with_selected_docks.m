function [preferred_visit_order, route_length_m] = refine_route_with_selected_docks(selected_poses, scene, planner_cfg)
%REFINE_ROUTE_WITH_SELECTED_DOCKS Compute a global order for selected docks.

    if isempty(selected_poses)
        preferred_visit_order = 1;
        route_length_m = 0;
        return;
    end
    path_cost_audit('set_source', 'BIDIRECTIONAL_REFINEMENT');
    all_poses = [scene.start_pose; selected_poses];
    dist_matrix = compute_path_cost_matrix(all_poses, scene.grid_map, scene.map_info, planner_cfg);
    enable_2opt = isfield(planner_cfg, 'enable_2opt') && planner_cfg.enable_2opt;
    preferred_visit_order = nearest_neighbor_order(dist_matrix, 1, enable_2opt);
    route = preferred_visit_order;
    if isfield(planner_cfg, 'execution') && isfield(planner_cfg.execution, 'return_to_start') && ...
            planner_cfg.execution.return_to_start
        route = [route, 1];
    end
    path = stitch_global_path(all_poses, route, scene.grid_map, scene.map_info, planner_cfg);
    route_length_m = compute_path_length(path);
end
