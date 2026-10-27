function [cost, cache] = get_cached_path_cost(cache, start_pose, goal_pose, scene, planner_cfg)
%GET_CACHED_PATH_COST Return cached grid-planner path length between poses.

    current_signature = build_path_cache_signature(scene.grid_map, scene.map_info, planner_cfg);
    if ~strcmp(cache.map_signature, current_signature)
        error('get_cached_path_cost:StaleCache', ...
            'Path-cost cache cannot be reused after map or path semantics change.');
    end

    key = make_pose_key(cache.map_signature, start_pose, goal_pose, planner_cfg);
    if isKey(cache.costs, key)
        path_cost_audit('record', start_pose, goal_pose, planner_cfg.global_planner, false);
        cache.hits = cache.hits + 1;
        cost = cache.costs(key);
        return;
    end

    path_cost_audit('record', start_pose, goal_pose, planner_cfg.global_planner, true);
    cache.misses = cache.misses + 1;
    run_opts = struct('suppress_warnings', true);
    path = run_path_planner(scene.grid_map, start_pose, goal_pose, scene.map_info, ...
        planner_cfg, planner_cfg.global_planner, run_opts);
    if isempty(path)
        cost = inf;
    else
        cost = compute_path_length(path);
    end
    cache.costs(key) = cost;
end

function key = make_pose_key(map_sig, start_pose, goal_pose, planner_cfg)
    q = 1000;
    a = round(start_pose(:)' * q);
    b = round(goal_pose(:)' * q);
    key = sprintf('%s|%s|%d_%d_%d|%d_%d_%d', ...
        map_sig, lower(planner_cfg.global_planner), a(1), a(2), a(3), b(1), b(2), b(3));
end
