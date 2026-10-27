function cache = initialize_path_cost_cache(scene, planner_cfg)
%INITIALIZE_PATH_COST_CACHE Create a small in-memory path-cost cache.

    cache = struct();
    cache.map_signature = build_path_cache_signature(scene.grid_map, scene.map_info, planner_cfg);
    cache.costs = containers.Map('KeyType', 'char', 'ValueType', 'double');
    cache.hits = 0;
    cache.misses = 0;
end
