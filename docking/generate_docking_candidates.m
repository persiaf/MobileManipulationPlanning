function [candidates, diagnostics] = generate_docking_candidates(region, scene, robot_cfg, planner_cfg)
%GENERATE_DOCKING_CANDIDATES Build and score deterministic docking poses.

    scene_cfg = scene.scene_cfg;
    map_info = scene.map_info;
    tau_r = planner_cfg.reachability.threshold;
    table_center = scene_cfg.table_positions(region.table_id, :);
    table_theta = scene_cfg.table_orientations(region.table_id);

    task_ids = region.task_ids;
    num_tasks = numel(task_ids);
    task_positions = zeros(num_tasks, 2);
    for ti = 1:num_tasks
        tid = task_ids(ti);
        task_positions(ti, :) = [scene.tasks(tid).x, scene.tasks(tid).y];
    end
    task_centroid = mean(task_positions, 1);

    reach_field = compute_reachability_field(table_center, table_theta, ...
        scene_cfg, robot_cfg, map_info, task_centroid, task_positions);
    candidate_poses = reach_field.candidates;
    diagnostics = initialize_diagnostics(task_ids, candidate_poses);

    candidates = struct('pose', {}, 'table_id', {}, 'side_id', {}, ...
        'covered_tasks', {}, 'reach_scores', {}, 'arm_assignment', {}, ...
        'nominal_covered_tasks', {}, 'nominal_reach_scores', {}, ...
        'robust_covered_tasks', {}, 'robust_reachability_summary', {}, ...
        'coverage_ratio', {}, 'cost_breakdown', {}, 'total_cost', {});

    reachability_mode = 'nominal';
    if isfield(planner_cfg, 'reachability') && isfield(planner_cfg.reachability, 'mode')
        reachability_mode = lower(char(planner_cfg.reachability.mode));
    end
    if ~ismember(reachability_mode, {'nominal', 'robust'})
        error('generate_docking_candidates:UnknownReachabilityMode', ...
            'Unknown reachability mode "%s".', reachability_mode);
    end
    robust_q_min = getfield_default(planner_cfg.reachability, ...
        'robust_success_fraction_min', 0.80);
    error_model = getfield_default(planner_cfg, 'docking_error_model', struct());

    for ci = 1:size(candidate_poses, 1)
        pose = candidate_poses(ci, :);
        if check_footprint_collision(pose, scene.grid_map, map_info, robot_cfg)
            diagnostics.collision_rejected_candidate_count = ...
                diagnostics.collision_rejected_candidate_count + 1;
            continue;
        end
        diagnostics.collision_valid_candidate_count = ...
            diagnostics.collision_valid_candidate_count + 1;
        diagnostics.collision_valid_support_count = ...
            diagnostics.collision_valid_support_count + 1;

        if isfield(reach_field, 'side_ids') && ci <= numel(reach_field.side_ids)
            side_id = reach_field.side_ids(ci);
        else
            side_id = determine_table_side(pose, table_center, table_theta, scene_cfg);
        end

        reach_scores = zeros(num_tasks, 3);
        robust_summary = repmat(struct('minimum_score', NaN, 'mean_score', NaN, ...
            'success_fraction', NaN, 'sample_count', 0, 'threshold', tau_r, ...
            'geometric_only', true, 'error_model_source', 'unspecified', ...
            'error_model_measured', false), 1, num_tasks);
        arm_assign = cell(num_tasks, 1);
        nominal_covered = [];
        robust_covered = [];
        for ti = 1:num_tasks
            tid = task_ids(ti);
            target = [scene.tasks(tid).x, scene.tasks(tid).y, scene.tasks(tid).z];
            diagnostics.candidate_task_pair_count = ...
                diagnostics.candidate_task_pair_count + 1;
            if candidate_prefilter_enabled(planner_cfg)
                [quick_reject, ~] = candidate_reachability_quick_filter( ...
                    pose, target, robot_cfg);
                if quick_reject
                    diagnostics.quick_rejected_pair_count = ...
                        diagnostics.quick_rejected_pair_count + 1;
                    arm_assign{ti} = 'quick_reject';
                    continue;
                end
            end
            diagnostics.formal_reachability_call_count = ...
                diagnostics.formal_reachability_call_count + 1;
            [ls, rs, cs, arm] = evaluate_dual_arm_reachability(pose, target, robot_cfg, tau_r);
            reach_scores(ti, :) = [ls, rs, cs];
            arm_assign{ti} = arm;
            if cs > 0
                nominal_covered(end + 1) = tid; %#ok<AGROW>
                diagnostics.reachable_support_count(ti) = ...
                    diagnostics.reachable_support_count(ti) + 1;
            end
            if strcmp(reachability_mode, 'robust')
                robust_summary(ti) = evaluate_robust_reachability( ...
                    pose, target, robot_cfg, tau_r, error_model);
                if robust_summary(ti).success_fraction >= robust_q_min
                    robust_covered(end + 1) = tid; %#ok<AGROW>
                end
            end
        end

        if strcmp(reachability_mode, 'robust')
            covered = robust_covered;
        else
            covered = nominal_covered;
        end

        idx = numel(candidates) + 1;
        candidates(idx).pose = pose;
        candidates(idx).table_id = region.table_id;
        candidates(idx).side_id = side_id;
        candidates(idx).covered_tasks = covered;
        candidates(idx).reach_scores = reach_scores;
        if strcmp(reachability_mode, 'robust')
            candidates(idx).reach_scores(:, 3) = [robust_summary.success_fraction]';
        end
        candidates(idx).arm_assignment = arm_assign;
        candidates(idx).nominal_covered_tasks = nominal_covered;
        candidates(idx).nominal_reach_scores = reach_scores;
        candidates(idx).robust_covered_tasks = robust_covered;
        candidates(idx).robust_reachability_summary = robust_summary;
        candidates(idx).coverage_ratio = numel(covered) / max(num_tasks, 1);
        candidates(idx).cost_breakdown = struct();
        candidates(idx).total_cost = inf;
    end
    diagnostics.reachability_valid_candidate_count = sum(arrayfun( ...
        @(c) ~isempty(c.covered_tasks), candidates));
    diagnostics.reachability_rejected_candidate_count = ...
        diagnostics.collision_valid_candidate_count - diagnostics.reachability_valid_candidate_count;
    diagnostics.library_candidate_count = numel(candidates);
    diagnostics.library_support_count = zeros(num_tasks, 1);
    for ci = 1:numel(candidates)
        for ti = 1:num_tasks
            diagnostics.library_support_count(ti) = diagnostics.library_support_count(ti) + ...
                ismember(task_ids(ti), candidates(ci).covered_tasks);
        end
    end

    for ci = 1:numel(candidates)
        cost_struct = evaluate_docking_candidate(candidates(ci), region, scene, ...
            robot_cfg, planner_cfg, task_positions, task_centroid);
        candidates(ci).cost_breakdown = cost_struct;
        candidates(ci).total_cost = cost_struct.total;
    end
    candidates = annotate_heuristic_ranks(candidates);
end

function value = getfield_default(s, name, default_value)
    if isstruct(s) && isfield(s, name), value = s.(name); else, value = default_value; end
end

function tf = candidate_prefilter_enabled(planner_cfg)
    tf = isfield(planner_cfg, 'candidate_pair_prefilter') && ...
        isfield(planner_cfg.candidate_pair_prefilter, 'enable') && ...
        planner_cfg.candidate_pair_prefilter.enable;
end

function diagnostics = initialize_diagnostics(task_ids, candidate_poses)
    n = numel(task_ids);
    diagnostics = struct();
    diagnostics.task_ids = task_ids(:);
    diagnostics.raw_candidate_count = size(candidate_poses, 1);
    diagnostics.collision_rejected_candidate_count = 0;
    diagnostics.collision_valid_candidate_count = 0;
    diagnostics.reachability_rejected_candidate_count = 0;
    diagnostics.reachability_valid_candidate_count = 0;
    diagnostics.library_candidate_count = 0;
    diagnostics.raw_support_count = repmat(size(candidate_poses, 1), n, 1);
    diagnostics.collision_valid_support_count = zeros(n, 1);
    diagnostics.reachable_support_count = zeros(n, 1);
    diagnostics.library_support_count = zeros(n, 1);
    diagnostics.candidate_task_pair_count = 0;
    diagnostics.quick_rejected_pair_count = 0;
    diagnostics.formal_reachability_call_count = 0;
end

function candidates = annotate_heuristic_ranks(candidates)
    if isempty(candidates)
        return;
    end
    costs = [candidates.total_cost]';
    [best_cost, ~] = min(costs);
    [~, order] = sort(costs, 'ascend');
    ranks = zeros(numel(candidates), 1);
    ranks(order) = 1:numel(candidates);
    denom = max(numel(candidates) - 1, 1);
    for i = 1:numel(candidates)
        candidates(i).selected_candidate_rank_by_heuristic = ranks(i);
        candidates(i).candidate_rank_percentile_in_region = (ranks(i) - 1) / denom;
        candidates(i).candidate_cost_gap_to_best_heuristic = max(0, costs(i) - best_cost);
    end
end

function side_id = determine_table_side(pose, table_center, table_theta, scene_cfg)
    local = rot2(-table_theta) * (pose(1:2) - table_center)';
    dl = abs(local(1)) - scene_cfg.table.length / 2;
    dw = abs(local(2)) - scene_cfg.table.width / 2;
    if dw >= dl
        side_id = 1 + (local(2) <= 0);
    else
        side_id = 3 + (local(1) > 0);
    end
end
