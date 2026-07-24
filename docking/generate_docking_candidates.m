function candidates = generate_docking_candidates(region, scene, robot_cfg, planner_cfg)
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

    candidates = struct('pose', {}, 'table_id', {}, 'side_id', {}, ...
        'covered_tasks', {}, 'reach_scores', {}, 'arm_assignment', {}, ...
        'coverage_ratio', {}, 'cost_breakdown', {}, 'total_cost', {});

    for ci = 1:size(candidate_poses, 1)
        pose = candidate_poses(ci, :);
        if check_footprint_collision(pose, scene.grid_map, map_info, robot_cfg)
            continue;
        end

        if isfield(reach_field, 'side_ids') && ci <= numel(reach_field.side_ids)
            side_id = reach_field.side_ids(ci);
        else
            side_id = determine_table_side(pose, table_center, table_theta, scene_cfg);
        end

        reach_scores = zeros(num_tasks, 3);
        arm_assign = cell(num_tasks, 1);
        covered = [];
        for ti = 1:num_tasks
            tid = task_ids(ti);
            target = [scene.tasks(tid).x, scene.tasks(tid).y, scene.tasks(tid).z];
            [ls, rs, cs, arm] = evaluate_dual_arm_reachability(pose, target, robot_cfg, tau_r);
            reach_scores(ti, :) = [ls, rs, cs];
            arm_assign{ti} = arm;
            if cs > 0
                covered(end + 1) = tid; %#ok<AGROW>
            end
        end

        idx = numel(candidates) + 1;
        candidates(idx).pose = pose;
        candidates(idx).table_id = region.table_id;
        candidates(idx).side_id = side_id;
        candidates(idx).covered_tasks = covered;
        candidates(idx).reach_scores = reach_scores;
        candidates(idx).arm_assignment = arm_assign;
        candidates(idx).coverage_ratio = numel(covered) / max(num_tasks, 1);
        candidates(idx).cost_breakdown = struct();
        candidates(idx).total_cost = inf;
    end

    for ci = 1:numel(candidates)
        cost_struct = evaluate_docking_candidate(candidates(ci), region, scene, ...
            robot_cfg, planner_cfg, task_positions, task_centroid);
        candidates(ci).cost_breakdown = cost_struct;
        candidates(ci).total_cost = cost_struct.total;
    end
    candidates = annotate_heuristic_ranks(candidates);
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
