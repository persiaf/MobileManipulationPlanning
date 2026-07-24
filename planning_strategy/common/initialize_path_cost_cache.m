function cache = initialize_path_cost_cache(scene, planner_cfg)
%INITIALIZE_PATH_COST_CACHE Create a small in-memory path-cost cache.

    cache = struct();
    cache.map_signature = sprintf('%dx%d_%.4f_%s_%.3f', ...
        size(scene.grid_map, 1), size(scene.grid_map, 2), ...
        scene.map_info.resolution, planner_cfg.global_planner, ...
        getfield_default(planner_cfg, 'inflation_radius_m', 0));
    cache.costs = containers.Map('KeyType', 'char', 'ValueType', 'double');
    cache.hits = 0;
    cache.misses = 0;
end

function v = getfield_default(s, field_name, default_value)
    if isstruct(s) && isfield(s, field_name)
        v = s.(field_name);
    else
        v = default_value;
    end
end
