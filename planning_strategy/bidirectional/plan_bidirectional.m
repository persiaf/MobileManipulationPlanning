function plan = plan_bidirectional(candidate_library, regions, scene, robot_cfg, planner_cfg)
%PLAN_BIDIRECTIONAL Conservative global-local feedback baseline.

    t_strategy = tic;
    base_plan = plan_bottom_up(candidate_library, regions, scene, robot_cfg, planner_cfg);
    t_refine = tic;
    [preferred_visit_order, route_length_m] = refine_route_with_selected_docks( ...
        base_plan.selected_poses, scene, planner_cfg);
    timing = base_plan.timing;
    timing.route_refinement_time_s = toc(t_refine);
    timing.strategy_total_time_s = toc(t_strategy);

    plan = base_plan;
    plan.strategy_name = 'bidirectional';
    plan.preferred_visit_order = preferred_visit_order;
    plan.timing = timing;
    plan.diagnostics.iterations = struct('iteration', 0, ...
        'objective_before', base_plan.objective_value, ...
        'objective_after', base_plan.objective_value, ...
        'completion_rate', base_plan.metrics.predicted_completion_rate, ...
        'num_docks', base_plan.num_docking_poses, ...
        'route_length', route_length_m, ...
        'estimated_execution_time', 0, ...
        'num_replacements', 0, ...
        'num_removed_docks', 0);
    plan.diagnostics.num_iterations = 0;
    plan.diagnostics.route_length_m = route_length_m;
    plan.objective_value = compute_strategy_objective(plan, ...
        struct('route_length_m', route_length_m, 'estimated_execution_time_s', 0), planner_cfg);
    plan.metrics.route_context_queries = getfield_default(plan.metrics, 'route_context_queries', 0);
end

function v = getfield_default(s, field_name, default_value)
    if isstruct(s) && isfield(s, field_name)
        v = s.(field_name);
    else
        v = default_value;
    end
end
