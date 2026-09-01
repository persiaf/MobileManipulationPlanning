function plan = plan_bidirectional(candidate_library, regions, scene, robot_cfg, planner_cfg)
%PLAN_BIDIRECTIONAL Bottom-up seed plus conservative neighborhood search.
% The search never accepts lower predicted coverage when
% require_non_decreasing_coverage=true. It reuses the shared candidate
% library and only changes selected docking poses/order.

    %#ok<INUSD>
    if use_shared_cache(planner_cfg)
        planner_cfg.runtime.shared_path_cost_context = SharedPathCostContext(scene, planner_cfg);
    end
    t_strategy = tic;
    base_plan = plan_bottom_up(candidate_library, regions, scene, robot_cfg, planner_cfg);
    current = refresh_bidirectional_plan(base_plan, candidate_library, scene, planner_cfg, struct());
    diagnostics = build_plan_diagnostics(current.diagnostics);
    diagnostics.iteration_log = struct([]);

    max_iter = get_bidir_field(planner_cfg, 'max_iterations', 5);
    tol = get_bidir_field(planner_cfg, 'improvement_tolerance', 1e-3);
    for iter = 1:max_iter
        before = current;
        before_obj = current.objective_value;
        [candidate, operation_type, rejection_reason] = best_neighbor(current, candidate_library, regions, scene, planner_cfg);
        if isempty(operation_type)
            diagnostics.rejected_iterations = diagnostics.rejected_iterations + 1;
            diagnostics.iteration_log = append_log(diagnostics.iteration_log, ...
                make_log(iter, 'none', before, before, false, 'no_feasible_neighbor'));
            break;
        end
        candidate = refresh_bidirectional_plan(candidate, candidate_library, scene, planner_cfg, diagnostics);
        improved = candidate.objective_value < before_obj - tol;
        coverage_ok = coverage_acceptance_ok(before, candidate, planner_cfg);
        if improved && coverage_ok
            current = candidate;
            diagnostics.accepted_iterations = diagnostics.accepted_iterations + 1;
            diagnostics = update_operation_counts(diagnostics, operation_type, before, current);
            diagnostics.iteration_log = append_log(diagnostics.iteration_log, ...
                make_log(iter, operation_type, before, current, true, ''));
        else
            diagnostics.rejected_iterations = diagnostics.rejected_iterations + 1;
            if ~coverage_ok
                rejection_reason = 'coverage_decreased';
            elseif ~improved
                rejection_reason = 'objective_not_improved';
            end
            diagnostics.iteration_log = append_log(diagnostics.iteration_log, ...
                make_log(iter, operation_type, before, candidate, false, rejection_reason));
            break;
        end
    end

    t_refine = tic;
    current = refresh_bidirectional_plan(current, candidate_library, scene, planner_cfg, diagnostics);
    current.timing.route_refinement_time_s = current.timing.route_refinement_time_s + toc(t_refine);
    current.timing.strategy_total_time_s = toc(t_strategy);
    current.strategy_name = 'bidirectional';
    current.diagnostics = build_plan_diagnostics(diagnostics);
    current.metrics.coverage_repair_count = current.diagnostics.coverage_repair_count;
    current.metrics.route_context_queries = current.diagnostics.route_context_queries;
    current.metrics.cache_hits = current.diagnostics.cache_hits;
    current.metrics.cache_misses = current.diagnostics.cache_misses;
    if isfield(planner_cfg.runtime, 'shared_path_cost_context')
        current.shared_path_cost_context = planner_cfg.runtime.shared_path_cost_context;
        current.shared_path_cost_diagnostics = get_shared_path_cost_diagnostics(planner_cfg);
    end
    plan = current;
    validate_plan_consistency(plan, scene);
end

function tf = use_shared_cache(planner_cfg)
    tf = false;
    if isfield(planner_cfg, 'strategy') && isfield(planner_cfg.strategy, 'bidirectional') && ...
            isfield(planner_cfg.strategy.bidirectional, 'use_shared_path_cost_cache')
        tf = logical(planner_cfg.strategy.bidirectional.use_shared_path_cost_cache);
    end
end

function logs = append_log(logs, entry)
    if isempty(logs)
        logs = entry;
        return;
    end
    names = union(fieldnames(logs), fieldnames(entry), 'stable');
    logs = align_struct_fields(logs, names);
    entry = align_struct_fields(entry, names);
    logs(end + 1) = entry;
end

function s = align_struct_fields(s, names)
    for i = 1:numel(names)
        if ~isfield(s, names{i})
            [s.(names{i})] = deal([]);
        end
    end
    s = orderfields(s, names);
end

function [best, operation_type, rejection_reason] = best_neighbor(plan, candidate_library, regions, scene, planner_cfg)
    best = [];
    operation_type = '';
    rejection_reason = '';
    operations = {};
    if get_bidir_field(planner_cfg, 'enable_redundant_removal', true)
        operations{end + 1} = @try_redundant_removal; %#ok<AGROW>
    end
    if get_bidir_field(planner_cfg, 'enable_single_replacement', true)
        operations{end + 1} = @try_single_replacement; %#ok<AGROW>
    end
    if get_bidir_field(planner_cfg, 'enable_two_to_one_replacement', true)
        operations{end + 1} = @try_two_to_one_replacement; %#ok<AGROW>
    end
    if get_bidir_field(planner_cfg, 'enable_route_reinsertion', true)
        operations{end + 1} = @try_route_reinsertion; %#ok<AGROW>
    end
    for oi = 1:numel(operations)
        [candidate, op_name, reason] = operations{oi}(plan, candidate_library, regions, scene, planner_cfg);
        if isempty(candidate)
            rejection_reason = reason;
            continue;
        end
        best = candidate;
        operation_type = op_name;
        rejection_reason = reason;
        return;
    end
end

function [candidate, op_name, reason] = try_redundant_removal(plan, candidate_library, regions, scene, planner_cfg)
    %#ok<INUSD>
    candidate = [];
    op_name = 'remove_redundant_dock';
    reason = 'no_redundant_dock';
    original = plan.covered_tasks_predicted;
    for i = 1:plan.num_docking_poses
        trial = remove_dock(plan, i);
        if isequal(sort(trial.covered_tasks_predicted), sort(original))
            candidate = trial;
            return;
        end
    end
end

function [candidate, op_name, reason] = try_single_replacement(plan, candidate_library, regions, scene, planner_cfg)
    candidate = [];
    op_name = 'single_replacement';
    reason = 'no_single_replacement';
    shortlist_size = get_common_shortlist(planner_cfg);
    for i = 1:plan.num_docking_poses
        ri = plan.pose_to_region_map(i);
        old_tasks = summary_tasks(plan.task_sequence_summary{i});
        candidates = candidate_library.regions(ri).candidates;
        order = candidate_order(candidates, shortlist_size);
        for idx = order
            new_tasks = intersect(candidates(idx).covered_tasks, regions(ri).task_ids);
            if numel(new_tasks) < numel(old_tasks)
                continue;
            end
            trial = replace_dock(plan, i, candidates(idx), regions(ri), new_tasks);
            if coverage_acceptance_ok(plan, trial, planner_cfg)
                candidate = trial;
                return;
            end
        end
    end
end

function [candidate, op_name, reason] = try_two_to_one_replacement(plan, candidate_library, regions, scene, planner_cfg)
    candidate = [];
    op_name = 'two_to_one_replacement';
    reason = 'no_two_to_one_replacement';
    shortlist_size = get_common_shortlist(planner_cfg);
    for i = 1:plan.num_docking_poses
        for j = i+1:plan.num_docking_poses
            if plan.pose_to_region_map(i) ~= plan.pose_to_region_map(j)
                continue;
            end
            ri = plan.pose_to_region_map(i);
            target_tasks = union(summary_tasks(plan.task_sequence_summary{i}), ...
                summary_tasks(plan.task_sequence_summary{j}), 'stable');
            candidates = candidate_library.regions(ri).candidates;
            order = candidate_order(candidates, shortlist_size);
            for idx = order
                new_tasks = intersect(candidates(idx).covered_tasks, regions(ri).task_ids);
                if isempty(setdiff(target_tasks, new_tasks))
                    trial = replace_dock(plan, i, candidates(idx), regions(ri), new_tasks);
                    trial = remove_dock(trial, j);
                    if coverage_acceptance_ok(plan, trial, planner_cfg)
                        candidate = trial;
                        return;
                    end
                end
            end
        end
    end
end

function [candidate, op_name, reason] = try_route_reinsertion(plan, candidate_library, regions, scene, planner_cfg)
    %#ok<INUSD>
    candidate = plan;
    op_name = 'route_reinsertion';
    reason = '';
    old_order = plan.preferred_visit_order;
    [new_order, ~] = refine_route_with_selected_docks(plan.selected_poses, scene, planner_cfg);
    candidate.preferred_visit_order = new_order;
    if isequal(old_order, new_order)
        candidate = [];
        reason = 'route_unchanged';
    end
end

function plan = refresh_bidirectional_plan(plan, candidate_library, scene, planner_cfg, diagnostics)
    if nargin < 5 || isempty(diagnostics)
        diagnostics = struct();
    end
    diagnostics = build_plan_diagnostics(diagnostics);
    covered = [];
    local_cost_total = 0;
    for i = 1:numel(plan.task_sequence_summary)
        covered = union(covered, summary_tasks(plan.task_sequence_summary{i}), 'stable');
    end
    for i = 1:numel(plan.pose_info_list)
        info = plan.pose_info_list{i};
        if isstruct(info) && isfield(info, 'cost_breakdown') && isfield(info.cost_breakdown, 'total')
            local_cost_total = local_cost_total + info.cost_breakdown.total;
        end
    end
    [preferred_visit_order, route_length_m] = refine_route_with_selected_docks(plan.selected_poses, scene, planner_cfg);
    plan.strategy_name = 'bidirectional';
    plan.preferred_visit_order = preferred_visit_order;
    plan.covered_tasks_predicted = covered;
    plan.uncovered_tasks_predicted = setdiff(1:scene.num_tasks, covered, 'stable');
    plan.num_docking_poses = size(plan.selected_poses, 1);
    plan.local_cost_total = local_cost_total;
    plan.objective_value = compute_strategy_objective(plan, ...
        struct('route_length_m', route_length_m, 'estimated_execution_time_s', 0), planner_cfg);
    plan.metrics.predicted_completion_rate = 100 * numel(covered) / max(scene.num_tasks, 1);
    plan.metrics.num_docking_poses = plan.num_docking_poses;
    plan.metrics.local_cost_total = local_cost_total;
    plan.metrics.coverage_repair_count = diagnostics.coverage_repair_count;
    plan.metrics.candidate_evaluations = candidate_library.total_candidates;
    plan.metrics.route_context_queries = diagnostics.route_context_queries;
    plan.metrics.cache_hits = diagnostics.cache_hits;
    plan.metrics.cache_misses = diagnostics.cache_misses;
    plan.diagnostics = diagnostics;
    plan.diagnostics.route_length_m = route_length_m;
end

function trial = remove_dock(plan, idx)
    trial = plan;
    trial.selected_poses(idx, :) = [];
    trial.pose_to_region_map(idx) = [];
    trial.task_sequence_summary(idx) = [];
    trial.pose_info_list(idx) = [];
end

function trial = replace_dock(plan, idx, candidate, region, new_tasks)
    trial = plan;
    info = make_pose_info(candidate, region, new_tasks);
    trial.selected_poses(idx, :) = info.pose;
    trial.task_sequence_summary{idx}.pose = info.pose;
    trial.task_sequence_summary{idx}.covered_tasks = new_tasks;
    trial.task_sequence_summary{idx}.covered_count = numel(new_tasks);
    trial.task_sequence_summary{idx}.arm_assign_map = info.arm_assign_map;
    if isfield(candidate, 'side_id')
        trial.task_sequence_summary{idx}.side_id = candidate.side_id;
    end
    trial.pose_info_list{idx} = info;
end

function info = make_pose_info(candidate, region, new_tasks)
    arm_map = containers.Map('KeyType', 'double', 'ValueType', 'char');
    for ti = 1:numel(new_tasks)
        tid = new_tasks(ti);
        local_idx = find(region.task_ids == tid, 1);
        if ~isempty(local_idx) && isfield(candidate, 'arm_assignment') && local_idx <= numel(candidate.arm_assignment)
            arm_map(tid) = candidate.arm_assignment{local_idx};
        end
    end
    info.pose = candidate.pose;
    info.covered_tasks = new_tasks;
    info.covered_count = numel(new_tasks);
    info.arm_assign_map = arm_map;
    info.table_id = region.table_id;
    info.side_id = get_candidate_side(candidate);
    info.cost_breakdown = candidate.cost_breakdown;
    info.remaining_after = 0;
end

function tf = coverage_acceptance_ok(before, after, planner_cfg)
    require_ok = get_bidir_field(planner_cfg, 'require_non_decreasing_coverage', true);
    if require_ok
        tf = numel(after.covered_tasks_predicted) >= numel(before.covered_tasks_predicted) && ...
            isempty(setdiff(before.covered_tasks_predicted, after.covered_tasks_predicted));
    else
        tf = true;
    end
end

function diagnostics = update_operation_counts(diagnostics, operation_type, before, after)
    switch operation_type
        case 'remove_redundant_dock'
            diagnostics.num_removed_docks = diagnostics.num_removed_docks + ...
                max(0, before.num_docking_poses - after.num_docking_poses);
        case 'single_replacement'
            diagnostics.num_pose_replacements = diagnostics.num_pose_replacements + 1;
        case 'two_to_one_replacement'
            diagnostics.num_pose_replacements = diagnostics.num_pose_replacements + 1;
            diagnostics.num_removed_docks = diagnostics.num_removed_docks + 1;
        case 'route_reinsertion'
            diagnostics.num_route_changes = diagnostics.num_route_changes + 1;
    end
end

function log_entry = make_log(iter, operation_type, before, after, accepted, rejection_reason)
    log_entry.iteration = iter;
    log_entry.operation_type = operation_type;
    log_entry.objective_before = before.objective_value;
    log_entry.objective_after = after.objective_value;
    log_entry.completion_before = before.metrics.predicted_completion_rate;
    log_entry.completion_after = after.metrics.predicted_completion_rate;
    log_entry.docks_before = before.num_docking_poses;
    log_entry.docks_after = after.num_docking_poses;
    log_entry.path_before_m = safe_diag(before, 'route_length_m');
    log_entry.path_after_m = safe_diag(after, 'route_length_m');
    log_entry.execution_before_s = 0;
    log_entry.execution_after_s = 0;
    log_entry.accepted = logical(accepted);
    log_entry.rejection_reason = rejection_reason;
end

function order = candidate_order(candidates, max_n)
    if isempty(candidates)
        order = [];
        return;
    end
    coverage = arrayfun(@(c) numel(c.covered_tasks), candidates);
    costs = [candidates.total_cost];
    score = coverage(:) * 1000 - costs(:);
    [~, order] = sort(score, 'descend');
    order = order(1:min(max_n, numel(order)));
    order = reshape(order, 1, []);
end

function tasks = summary_tasks(summary_entry)
    tasks = [];
    if isstruct(summary_entry) && isfield(summary_entry, 'covered_tasks')
        tasks = summary_entry.covered_tasks;
    end
end

function side_id = get_candidate_side(candidate)
    if isfield(candidate, 'side_id') && ~isempty(candidate.side_id)
        side_id = candidate.side_id;
    else
        side_id = 0;
    end
end

function n = get_common_shortlist(planner_cfg)
    n = 25;
    if isfield(planner_cfg, 'strategy') && isfield(planner_cfg.strategy, 'common') && ...
            isfield(planner_cfg.strategy.common, 'route_shortlist_size')
        n = planner_cfg.strategy.common.route_shortlist_size;
    end
end

function v = get_bidir_field(planner_cfg, field_name, default_value)
    v = default_value;
    if isfield(planner_cfg, 'strategy') && isfield(planner_cfg.strategy, 'bidirectional') && ...
            isfield(planner_cfg.strategy.bidirectional, field_name)
        v = planner_cfg.strategy.bidirectional.(field_name);
    end
end

function v = safe_diag(plan, field_name)
    if isstruct(plan) && isfield(plan, 'diagnostics') && isfield(plan.diagnostics, field_name)
        v = plan.diagnostics.(field_name);
    else
        v = 0;
    end
end
