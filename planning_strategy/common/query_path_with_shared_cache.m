function [path, cost, success, cache_hit] = query_path_with_shared_cache( ...
        grid_map, start_pose, goal_pose, map_info, planner_cfg)
%QUERY_PATH_WITH_SHARED_CACHE Use an attached run-local cache when present.

    context = get_context(planner_cfg);
    if isempty(context)
        path_cost_audit('record', start_pose, goal_pose, planner_cfg.global_planner, true);
        path = run_path_planner(grid_map, start_pose, goal_pose, map_info, planner_cfg);
        cost = path_length(path); success = ~isempty(path) && size(path, 1) >= 2;
        if ~success,cost=inf;end
        cache_hit = false;
        return;
    end
    [path, cost, success, cache_hit] = context.query( ...
        grid_map, start_pose, goal_pose, map_info, planner_cfg);
    path_cost_audit('record', start_pose, goal_pose, planner_cfg.global_planner, ~cache_hit);
end

function context = get_context(planner_cfg)
    context = [];
    if isfield(planner_cfg, 'runtime') && isfield(planner_cfg.runtime, 'shared_path_cost_context') && ...
            isa(planner_cfg.runtime.shared_path_cost_context, 'SharedPathCostContext')
        context = planner_cfg.runtime.shared_path_cost_context;
    end
end
