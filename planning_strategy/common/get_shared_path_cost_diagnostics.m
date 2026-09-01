function diagnostics = get_shared_path_cost_diagnostics(planner_cfg)
%GET_SHARED_PATH_COST_DIAGNOSTICS Snapshot an attached cache context.
    diagnostics = empty_diagnostics();
    if isfield(planner_cfg, 'runtime') && isfield(planner_cfg.runtime, 'shared_path_cost_context') && ...
            isa(planner_cfg.runtime.shared_path_cost_context, 'SharedPathCostContext')
        diagnostics = planner_cfg.runtime.shared_path_cost_context.diagnostics();
    end
end

function d = empty_diagnostics()
    d = struct('cache_query_count',0,'cache_hit_count',0,'cache_miss_count',0, ...
        'expensive_planner_calls',0,'strategy_cache_hits',0,'refinement_cache_hits',0, ...
        'stage3_cache_hits',0,'final_route_cache_hits',0,'entry_count',0,'memory_estimate_bytes',0);
end
