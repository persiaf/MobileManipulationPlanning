classdef SharedPathCostContext < handle
    %SHAREDPATHCOSTCONTEXT Scene-local directed path and cost cache.

    properties (SetAccess = private)
        scene_signature
        planner_name
        planner_signature
        costs
        paths
        successes
        query_count = 0
        hit_count = 0
        miss_count = 0
        expensive_planner_calls = 0
        source_queries
        source_hits
        source_misses
        source_expensive_calls
        validated_sources
        current_source = 'OTHER'
    end

    methods
        function obj = SharedPathCostContext(scene, planner_cfg)
            obj.scene_signature = scene_signature(scene, planner_cfg);
            obj.planner_name = lower(planner_cfg.global_planner);
            obj.planner_signature = planner_semantic_signature(planner_cfg);
            obj.costs = containers.Map('KeyType', 'char', 'ValueType', 'double');
            obj.paths = containers.Map('KeyType', 'char', 'ValueType', 'any');
            obj.successes = containers.Map('KeyType', 'char', 'ValueType', 'logical');
            obj.source_queries = containers.Map('KeyType', 'char', 'ValueType', 'double');
            obj.source_hits = containers.Map('KeyType', 'char', 'ValueType', 'double');
            obj.source_misses = containers.Map('KeyType', 'char', 'ValueType', 'double');
            obj.source_expensive_calls = containers.Map('KeyType', 'char', 'ValueType', 'double');
            obj.validated_sources = containers.Map('KeyType', 'char', 'ValueType', 'logical');
        end

        function setSource(obj, source)
            obj.current_source = char(source);
        end

        function [path, cost, success, hit] = query(obj, grid_map, start_pose, goal_pose, map_info, planner_cfg)
            obj.assertCompatible(grid_map, map_info, planner_cfg);
            key = obj.makeKey(start_pose, goal_pose, planner_cfg);
            obj.query_count = obj.query_count + 1;
            obj.increment(obj.source_queries, obj.current_source);
            if isKey(obj.costs, key)
                hit = true; obj.hit_count = obj.hit_count + 1;
                obj.increment(obj.source_hits, obj.current_source);
                path = obj.paths(key); cost = obj.costs(key); success = obj.successes(key);
                return;
            end
            hit = false; obj.miss_count = obj.miss_count + 1;
            obj.expensive_planner_calls = obj.expensive_planner_calls + 1;
            obj.increment(obj.source_misses, obj.current_source);
            obj.increment(obj.source_expensive_calls, obj.current_source);
            path = run_path_planner(grid_map, start_pose, goal_pose, map_info, planner_cfg);
            cost = path_length(path); success = ~isempty(path) && size(path, 1) >= 2;
            if ~success,cost=inf;end
            obj.paths(key) = path; obj.costs(key) = cost; obj.successes(key) = success;
        end

        function d = diagnostics(obj)
            d = struct('cache_query_count', obj.query_count, 'cache_hit_count', obj.hit_count, ...
                'cache_miss_count', obj.miss_count, 'expensive_planner_calls', obj.expensive_planner_calls, ...
                'strategy_cache_hits', source_total(obj.source_hits, ...
                    {'BIDIRECTIONAL_BASE','BIDIRECTIONAL_REFINEMENT'}), ...
                'refinement_cache_hits', source_total(obj.source_hits, {'BIDIRECTIONAL_REFINEMENT'}), ...
                'stage3_cache_hits', source_total(obj.source_hits, {'BIDIRECTIONAL_STAGE3'}), ...
                'final_route_cache_hits', source_total(obj.source_hits, {'FINAL_ROUTE'}), ...
                'entry_count', obj.costs.Count, 'memory_estimate_bytes', obj.memoryEstimate());
        end
    end

    methods (Access = private)
        function key = makeKey(obj, start_pose, goal_pose, planner_cfg)
            q = 1000; a = round(start_pose(:)' * q); b = round(goal_pose(:)' * q);
            key = sprintf('%s|%s|%d_%d_%d>%d_%d_%d', obj.scene_signature, ...
                lower(planner_cfg.global_planner), a(1), a(2), a(3), b(1), b(2), b(3));
        end

        function assertCompatible(obj, grid_map, map_info, planner_cfg)
            if ~strcmpi(obj.planner_name, planner_cfg.global_planner)
                error('SharedPathCostContext:PlannerChanged', ...
                    'A shared path-cost context cannot be reused with another planner.');
            end
            if ~strcmp(obj.planner_signature, planner_semantic_signature(planner_cfg))
                error('SharedPathCostContext:PlannerSemanticsChanged', ...
                    'A shared path-cost context cannot be reused with changed planner semantics.');
            end
            actual = grid_signature(grid_map, map_info, planner_cfg);
            if ~strcmp(obj.scene_signature, actual)
                error('SharedPathCostContext:SceneChanged', ...
                    'A shared path-cost context cannot be reused with another scene, map, or footprint.');
            end
            if ~isKey(obj.validated_sources, obj.current_source)
                obj.validated_sources(obj.current_source) = true;
            end
        end

        function increment(~, counter_map, key)
            if isKey(counter_map, key)
                next_value = counter_map(key) + 1;
            else
                next_value = 1;
            end
            counter_map(key) = next_value; %#ok<NASGU>
        end

        function bytes = memoryEstimate(obj)
            bytes = 0;
            keys = obj.paths.keys;
            for i = 1:numel(keys)
                path = obj.paths(keys{i});
                bytes = bytes + numel(path) * 8 + numel(keys{i}) * 2 + 17;
            end
        end
    end
end

function signature = planner_semantic_signature(planner_cfg)
    fields = {'astar','convex_astar','dijkstra','jps','theta_star','lazy_theta_star', ...
        'hybrid_astar','rrt','rrt_star','rrt_connect','informed_rrt','aco','pso','ga'};
    semantic = struct('global_planner', planner_cfg.global_planner);
    for i = 1:numel(fields)
        if isfield(planner_cfg, fields{i}), semantic.(fields{i}) = planner_cfg.(fields{i}); end
    end
    signature = build_path_cache_signature(zeros(0,0), ...
        struct('resolution', 1), struct('global_planner', 'semantic', ...
        'footprint_aware_navigation', false, 'robot_cfg', semantic, ...
        'semantic_options', semantic));
end

function signature = scene_signature(scene, planner_cfg)
    signature = grid_signature(scene.grid_map, scene.map_info, planner_cfg);
end

function signature = grid_signature(grid_map, map_info, planner_cfg)
    signature = build_path_cache_signature(grid_map, map_info, planner_cfg);
end

function total = source_total(map, names)
    total = 0;
    for i = 1:numel(names), if isKey(map, names{i}), total = total + map(names{i}); end, end
end
