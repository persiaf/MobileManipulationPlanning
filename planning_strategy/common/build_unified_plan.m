function plan = build_unified_plan(strategy_name, selected_poses, pose_to_region_map, ...
    task_sequence_summary, pose_info_list, region_order, preferred_visit_order, ...
    candidate_library, scene, planner_cfg, timing, diagnostics)
%BUILD_UNIFIED_PLAN Normalize strategy outputs into one struct.

    if nargin < 11 || isempty(timing), timing = struct(); end
    if nargin < 12 || isempty(diagnostics), diagnostics = struct(); end

    covered = [];
    local_cost_total = 0;
    for i = 1:numel(task_sequence_summary)
        if isfield(task_sequence_summary{i}, 'covered_tasks')
            covered = union(covered, task_sequence_summary{i}.covered_tasks, 'stable');
        end
    end
    for i = 1:numel(pose_info_list)
        info = pose_info_list{i};
        if isstruct(info) && isfield(info, 'cost_breakdown') && ...
                isfield(info.cost_breakdown, 'total')
            local_cost_total = local_cost_total + info.cost_breakdown.total;
        end
    end

    uncovered = setdiff(1:scene.num_tasks, covered, 'stable');
    diagnostics = build_plan_diagnostics(diagnostics);
    diagnostics.candidate_evaluations = candidate_library.total_candidates;
    plan.strategy_name = strategy_name;
    plan.selected_poses = selected_poses;
    plan.pose_to_region_map = pose_to_region_map;
    plan.task_sequence_summary = task_sequence_summary;
    plan.pose_info_list = pose_info_list;
    plan.region_order = region_order;
    plan.preferred_visit_order = preferred_visit_order;
    plan.covered_tasks_predicted = covered;
    plan.uncovered_tasks_predicted = uncovered;
    if isempty(uncovered)
        plan.status = 'READY';
        plan.reason = '';
    else
        plan.status = 'BLOCKED';
        plan.reason = 'PARTIAL_TASK_COVERAGE';
    end
    plan.num_docking_poses = size(selected_poses, 1);
    plan.local_cost_total = local_cost_total;
    plan.objective = struct();
    plan.objective.stage2 = compute_strategy_objective(plan, [], planner_cfg);
    plan.objective_value_stage2 = plan.objective.stage2;
    plan.objective_value = plan.objective.stage2;
    plan.metrics = struct();
    plan.metrics.predicted_completion_rate = 100 * numel(covered) / max(scene.num_tasks, 1);
    plan.metrics.num_docking_poses = plan.num_docking_poses;
    plan.metrics.local_cost_total = local_cost_total;
    plan.metrics.coverage_repair_count = getfield_default(diagnostics, 'coverage_repair_count', 0);
    plan.metrics.candidate_evaluations = candidate_library.total_candidates;
    plan.metrics.route_context_queries = getfield_default(diagnostics, 'route_context_queries', 0);
    plan.metrics.cache_hits = getfield_default(diagnostics, 'cache_hits', 0);
    plan.metrics.cache_misses = getfield_default(diagnostics, 'cache_misses', 0);
    plan.timing = complete_timing(timing, candidate_library);
    plan.diagnostics = diagnostics;
    plan.service_configs = collect_service_configs(task_sequence_summary);
end

function timing = complete_timing(timing, candidate_library)
    timing.candidate_generation_time_s = getfield_default(timing, ...
        'candidate_generation_time_s', candidate_library.total_generation_time_s);
    timing.region_order_time_s = getfield_default(timing, 'region_order_time_s', 0);
    timing.local_selection_time_s = getfield_default(timing, 'local_selection_time_s', 0);
    timing.route_refinement_time_s = getfield_default(timing, 'route_refinement_time_s', 0);
    timing.strategy_total_time_s = getfield_default(timing, 'strategy_total_time_s', ...
        timing.candidate_generation_time_s + timing.region_order_time_s + ...
        timing.local_selection_time_s + timing.route_refinement_time_s);
end

function service_configs = collect_service_configs(task_sequence_summary)
    service_configs = struct([]);
    for i = 1:numel(task_sequence_summary)
        if isfield(task_sequence_summary{i}, 'service_configs') && ...
                ~isempty(task_sequence_summary{i}.service_configs)
            sc = task_sequence_summary{i}.service_configs;
            service_configs = [service_configs, sc(:)']; %#ok<AGROW>
        end
    end
end

function v = getfield_default(s, field_name, default_value)
    if isstruct(s) && isfield(s, field_name)
        v = s.(field_name);
    else
        v = default_value;
    end
end
