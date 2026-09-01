function set_shared_path_cost_source(planner_cfg, source)
%SET_SHARED_PATH_COST_SOURCE Attribute subsequent shared-cache queries.
    if isfield(planner_cfg, 'runtime') && isfield(planner_cfg.runtime, 'shared_path_cost_context') && ...
            isa(planner_cfg.runtime.shared_path_cost_context, 'SharedPathCostContext')
        planner_cfg.runtime.shared_path_cost_context.setSource(source);
    end
end
