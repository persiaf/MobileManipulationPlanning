function [selected_poses, pose_to_region_map, task_sequence_summary, pose_info_list, diagnostics, cache] = ...
    select_route_aware_multi_docks(candidate_library, regions, scene, robot_cfg, ...
    planner_cfg, region_order, region_anchors, cache)
%SELECT_ROUTE_AWARE_MULTI_DOCKS Choose docks using region order context.

    %#ok<INUSD>
    selected_poses = [];
    pose_to_region_map = [];
    task_sequence_summary = {};
    pose_info_list = {};
    diagnostics.coverage_repair_count = 0;
    diagnostics.route_context_queries = 0;
    diagnostics.uncovered_task_ids = [];

    prev_pose = scene.start_pose;
    prev_side = [];
    for pos = 1:numel(region_order)
        ri = region_order(pos);
        region = regions(ri);
        candidates = candidate_library.regions(ri).candidates;
        if isempty(candidates)
            diagnostics.uncovered_task_ids = union(diagnostics.uncovered_task_ids, region.task_ids, 'stable');
            continue;
        end

        next_anchor = [];
        if pos < numel(region_order)
            next_anchor = get_anchor_poses(region_anchors, region_order(pos + 1));
        elseif isfield(planner_cfg, 'execution') && isfield(planner_cfg.execution, 'return_to_start') && ...
                planner_cfg.execution.return_to_start
            next_anchor = scene.start_pose;
        end

        budget = estimate_docking_budget(candidates, region, planner_cfg);
        [region_poses, pose_info, prev_pose, prev_side, diagnostics, cache] = ...
            select_region_docks(candidates, region, scene, planner_cfg, budget, ...
            prev_pose, prev_side, next_anchor, false, diagnostics, cache, region.task_ids);

        remaining = setdiff(region.task_ids, collect_pose_tasks(pose_info), 'stable');
        if should_repair(planner_cfg) && ~isempty(remaining)
            max_repair = get_topdown_field(planner_cfg, 'max_repair_docks_per_region', 2);
            [repair_poses, repair_info, prev_pose, prev_side, diagnostics, cache] = ...
                select_region_docks(candidates, region, scene, planner_cfg, max_repair, ...
                prev_pose, prev_side, next_anchor, true, diagnostics, cache, remaining);
            if ~isempty(repair_poses)
                region_poses = [region_poses; repair_poses]; %#ok<AGROW>
                pose_info = [pose_info, repair_info]; %#ok<AGROW>
                diagnostics.coverage_repair_count = diagnostics.coverage_repair_count + numel(repair_info);
            end
        end

        diagnostics.uncovered_task_ids = union(diagnostics.uncovered_task_ids, ...
            setdiff(region.task_ids, collect_pose_tasks(pose_info), 'stable'), 'stable');
        [selected_poses, pose_to_region_map, task_sequence_summary, pose_info_list] = ...
            append_region_plan(selected_poses, pose_to_region_map, task_sequence_summary, ...
            pose_info_list, ri, region, region_poses, pose_info, planner_cfg);
    end

    if isempty(selected_poses)
        error('select_route_aware_multi_docks:NoFeasibleDockingPose', ...
            'Top-down strategy selected no docking poses.');
    end
end

function [region_poses, pose_info, prev_pose, prev_side, diagnostics, cache] = ...
    select_region_docks(candidates, region, scene, planner_cfg, max_iter, prev_pose, ...
    prev_side, next_anchor, repair_mode, diagnostics, cache, remaining_tasks)

    region_poses = [];
    pose_info = struct('pose', {}, 'covered_tasks', {}, 'covered_count', {}, ...
        'arm_assign_map', {}, 'table_id', {}, 'side_id', {}, ...
        'cost_breakdown', {}, 'remaining_after', {});
    available = true(1, numel(candidates));
    for iter = 1:max_iter
        if isempty(remaining_tasks), break; end
        shortlist = build_shortlist(candidates, remaining_tasks, planner_cfg);
        shortlist = shortlist(available(shortlist));
        if isempty(shortlist), break; end
        [best_idx, new_tasks, diagnostics, cache] = pick_best_candidate(candidates, shortlist, ...
            region, scene, planner_cfg, remaining_tasks, prev_pose, prev_side, next_anchor, ...
            repair_mode, diagnostics, cache);
        if best_idx == 0 || isempty(new_tasks), break; end
        info = make_pose_info(candidates(best_idx), region, new_tasks, remaining_tasks);
        region_poses = [region_poses; info.pose]; %#ok<AGROW>
        pose_info(end + 1) = info; %#ok<AGROW>
        remaining_tasks = setdiff(remaining_tasks, new_tasks, 'stable');
        available(best_idx) = false;
        prev_pose = info.pose;
        prev_side = info.side_id;
    end
end

function shortlist = build_shortlist(candidates, remaining_tasks, planner_cfg)
    n = numel(candidates);
    score = -inf(1, n);
    for i = 1:n
        gain = numel(intersect(candidates(i).covered_tasks, remaining_tasks));
        if gain > 0
            score(i) = gain * 100 + mean_candidate_reach(candidates(i)) - candidates(i).total_cost;
        end
    end
    [~, order] = sort(score, 'descend');
    order = order(isfinite(score(order)));
    top_m = get_common_shortlist(planner_cfg);
    shortlist = order(1:min(top_m, numel(order)));
    for tid = remaining_tasks(:)'
        coverers = find(arrayfun(@(c) ismember(tid, c.covered_tasks), candidates));
        if ~isempty(coverers) && isempty(intersect(shortlist, coverers))
            [~, best_local] = max(score(coverers));
            shortlist(end + 1) = coverers(best_local); %#ok<AGROW>
        end
    end
    shortlist = unique(shortlist, 'stable');
end

function [best_idx, best_tasks, diagnostics, cache] = pick_best_candidate(candidates, shortlist, ...
    region, scene, planner_cfg, remaining_tasks, prev_pose, prev_side, next_anchor, repair_mode, diagnostics, cache)

    w = topdown_weights(planner_cfg, repair_mode);
    raw_costs = [candidates(shortlist).total_cost];
    coverage_gains = zeros(1, numel(shortlist));
    reach_values = zeros(1, numel(shortlist));
    side_switch_costs = zeros(1, numel(shortlist));
    route_costs = zeros(1, numel(shortlist));
    base_next_cost = 0;
    if ~isempty(next_anchor) && all(isfinite(next_anchor(:)))
        [base_next_cost, cache] = min_path_cost_to_set(cache, prev_pose, next_anchor, scene, planner_cfg);
    end
    for k = 1:numel(shortlist)
        ci = shortlist(k);
        new_tasks = intersect(candidates(ci).covered_tasks, remaining_tasks, 'stable');
        coverage_gains(k) = numel(new_tasks);
        reach_values(k) = mean_new_task_reach(candidates(ci), region, new_tasks);
        if ~isempty(prev_side) && isfield(candidates(ci), 'side_id') && candidates(ci).side_id ~= prev_side
            side_switch_costs(k) = planner_cfg.candidate.side_switch_penalty;
        end
        [entry_cost, cache] = get_cached_path_cost(cache, prev_pose, candidates(ci).pose, scene, planner_cfg);
        exit_cost = 0;
        if ~isempty(next_anchor) && all(isfinite(next_anchor(:)))
            [exit_cost, cache] = min_path_cost_to_set(cache, candidates(ci).pose, next_anchor, scene, planner_cfg);
        end
        route_costs(k) = entry_cost + exit_cost - base_next_cost;
        diagnostics.route_context_queries = diagnostics.route_context_queries + 2;
    end
    local_norm = normalize01(raw_costs);
    route_norm = normalize01(route_costs);
    coverage_norm = normalize01(coverage_gains);
    reach_norm = normalize01(reach_values);
    side_norm = normalize01(side_switch_costs);
    best_idx = 0;
    best_score = -inf;
    best_tasks = [];
    for k = 1:numel(shortlist)
        ci = shortlist(k);
        new_tasks = intersect(candidates(ci).covered_tasks, remaining_tasks, 'stable');
        if isempty(new_tasks), continue; end
        score = w.coverage_gain * coverage_norm(k) + ...
            w.reachability * reach_norm(k) - ...
            w.local_cost * local_norm(k) - ...
            w.route_context * route_norm(k) - ...
            w.side_switch * side_norm(k);
        if score > best_score
            best_score = score;
            best_idx = ci;
            best_tasks = new_tasks;
        end
    end
end

function poses = get_anchor_poses(region_anchors, ri)
    if isstruct(region_anchors)
        poses = region_anchors(ri).poses;
    else
        poses = region_anchors(ri, :);
    end
end

function [best_cost, cache] = min_path_cost_to_set(cache, from_pose, to_poses, scene, planner_cfg)
    best_cost = inf;
    for i = 1:size(to_poses, 1)
        [c, cache] = get_cached_path_cost(cache, from_pose, to_poses(i, :), scene, planner_cfg);
        best_cost = min(best_cost, c);
    end
    if ~isfinite(best_cost)
        best_cost = 0;
    end
end

function info = make_pose_info(candidate, region, new_tasks, remaining_tasks)
    arm_map = containers.Map('KeyType', 'double', 'ValueType', 'char');
    for ti = 1:numel(new_tasks)
        tid = new_tasks(ti);
        local_idx = find(region.task_ids == tid, 1);
        if ~isempty(local_idx) && local_idx <= numel(candidate.arm_assignment)
            arm_map(tid) = candidate.arm_assignment{local_idx};
        end
    end
    info.pose = candidate.pose;
    info.covered_tasks = new_tasks;
    info.covered_count = numel(new_tasks);
    info.arm_assign_map = arm_map;
    info.table_id = region.table_id;
    info.side_id = candidate.side_id;
    info.cost_breakdown = candidate.cost_breakdown;
    info.remaining_after = max(numel(remaining_tasks) - numel(new_tasks), 0);
end

function tasks = collect_pose_tasks(pose_info)
    tasks = [];
    for i = 1:numel(pose_info)
        tasks = union(tasks, pose_info(i).covered_tasks, 'stable');
    end
end

function tf = should_repair(planner_cfg)
    tf = true;
    if isfield(planner_cfg, 'strategy') && isfield(planner_cfg.strategy, 'top_down') && ...
            isfield(planner_cfg.strategy.top_down, 'enable_coverage_repair')
        tf = planner_cfg.strategy.top_down.enable_coverage_repair;
    end
end

function n = get_common_shortlist(planner_cfg)
    n = 25;
    if isfield(planner_cfg, 'strategy') && isfield(planner_cfg.strategy, 'common') && ...
            isfield(planner_cfg.strategy.common, 'route_shortlist_size')
        n = planner_cfg.strategy.common.route_shortlist_size;
    end
end

function v = get_topdown_field(planner_cfg, field_name, default_value)
    v = default_value;
    if isfield(planner_cfg, 'strategy') && isfield(planner_cfg.strategy, 'top_down') && ...
            isfield(planner_cfg.strategy.top_down, field_name)
        v = planner_cfg.strategy.top_down.(field_name);
    end
end

function w = topdown_weights(planner_cfg, repair_mode)
    w = planner_cfg.strategy.top_down.weights;
    if repair_mode
        w.coverage_gain = max(w.coverage_gain, 0.60);
        w.route_context = min(w.route_context, 0.10);
    end
end

function x = normalize01(v)
    v = v(:)';
    finite = isfinite(v);
    x = ones(size(v));
    if any(finite)
        lo = min(v(finite));
        hi = max(v(finite));
        if hi > lo
            x(finite) = (v(finite) - lo) / (hi - lo);
        else
            x(finite) = 0;
        end
    end
end

function r = mean_candidate_reach(candidate)
    if isfield(candidate, 'reach_scores') && ~isempty(candidate.reach_scores)
        r = mean(candidate.reach_scores(:, 3));
    else
        r = 0;
    end
end

function r = mean_new_task_reach(candidate, region, new_tasks)
    vals = zeros(1, numel(new_tasks));
    used = false(1, numel(new_tasks));
    for i = 1:numel(new_tasks)
        local_idx = find(region.task_ids == new_tasks(i), 1);
        if ~isempty(local_idx) && local_idx <= size(candidate.reach_scores, 1)
            vals(i) = candidate.reach_scores(local_idx, 3);
            used(i) = true;
        end
    end
    if any(used)
        r = mean(vals(used));
    else
        r = 0;
    end
end
