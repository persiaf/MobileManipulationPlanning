function objective = compute_strategy_objective(plan, route_metrics, planner_cfg)
%COMPUTE_STRATEGY_OBJECTIVE Compute a transparent scalar comparison score.

    if nargin < 2 || isempty(route_metrics), route_metrics = struct(); end
    w = struct('uncovered', 1000, 'docks', 1, 'path', 1, ...
        'local_cost', 1, 'execution_time', 0.1);
    if isfield(planner_cfg, 'strategy') && isfield(planner_cfg.strategy, 'bidirectional') && ...
            isfield(planner_cfg.strategy.bidirectional, 'objective_weights')
        w = merge_struct(w, planner_cfg.strategy.bidirectional.objective_weights);
    end
    objective = w.uncovered * numel(plan.uncovered_tasks_predicted) + ...
        w.docks * plan.num_docking_poses + ...
        w.path * getfield_default(route_metrics, 'route_length_m', 0) + ...
        w.local_cost * plan.local_cost_total + ...
        w.execution_time * getfield_default(route_metrics, 'estimated_execution_time_s', 0);
end

function out = merge_struct(out, in)
    names = fieldnames(in);
    for i = 1:numel(names)
        out.(names{i}) = in.(names{i});
    end
end

function v = getfield_default(s, field_name, default_value)
    if isstruct(s) && isfield(s, field_name)
        v = s.(field_name);
    else
        v = default_value;
    end
end
