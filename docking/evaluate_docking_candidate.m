function cost = evaluate_docking_candidate(candidate, region, scene, robot_cfg, planner_cfg, task_positions, task_centroid)
%EVALUATE_DOCKING_CANDIDATE Evaluate the deterministic heuristic cost.

    if nargin < 7
        task_ids = region.task_ids;
        task_positions = zeros(numel(task_ids), 2);
        for ti = 1:numel(task_ids)
            tid = task_ids(ti);
            task_positions(ti, :) = [scene.tasks(tid).x, scene.tasks(tid).y];
        end
        task_centroid = mean(task_positions, 1);
    end

    weights = planner_cfg.cost_weights;
    w = [weights.w_reach, weights.w_coverage, weights.w_orientation, ...
         weights.w_safety, weights.w_approach, weights.w_maneuver];
    if sum(w) > 0
        w = w ./ sum(w);
    end

    pose = candidate.pose;
    num_tasks = numel(region.task_ids);
    gamma = mean(candidate.reach_scores(:, 3));
    if isempty(gamma) || ~isfinite(gamma)
        gamma = 0;
    end
    reachability_cost = 1 - gamma;

    covered_count = numel(candidate.covered_tasks);
    coverage_ratio = covered_count / max(num_tasks, 1);
    coverage_cost = 1 - coverage_ratio;

    vec_to_tasks = task_centroid - pose(1:2);
    ideal_theta = atan2(vec_to_tasks(2), vec_to_tasks(1));
    orient_diff = abs(wrapToPiLocal(pose(3) - ideal_theta));
    orientation_cost = orient_diff / pi;

    dist_to_obs_m = compute_footprint_clearance(pose, scene.grid_map, scene.map_info, robot_cfg);
    nominal_clearance = planner_cfg.candidate.safety_distance_m;
    safety_cost = max(0, 1 - dist_to_obs_m / max(nominal_clearance, eps));

    table_center = scene.scene_cfg.table_positions(region.table_id, :);
    entry_point = (task_centroid + table_center) / 2;
    approach_dist = norm(pose(1:2) - entry_point);
    approach_dist_max = max(norm(table_center - entry_point) + 1.5, 1.0);
    approach_cost = min(1, approach_dist / approach_dist_max);

    maneuver_cost = orientation_cost;
    if isfield(candidate, 'side_id') && isfield(region, 'preferred_side') && ...
            ~isempty(region.preferred_side) && candidate.side_id ~= region.preferred_side
        maneuver_cost = maneuver_cost + planner_cfg.candidate.side_switch_penalty;
    end
    maneuver_cost = min(1, maneuver_cost);

    total = w(1) * reachability_cost + w(2) * coverage_cost + ...
            w(3) * orientation_cost + w(4) * safety_cost + ...
            w(5) * approach_cost + w(6) * maneuver_cost;

    cost = struct( ...
        'total', total, ...
        'reachability', reachability_cost, ...
        'coverage', coverage_cost, ...
        'orientation', orientation_cost, ...
        'safety', safety_cost, ...
        'approach', approach_cost, ...
        'maneuver', maneuver_cost, ...
        'Gamma', gamma, ...
        'coverage_ratio', coverage_ratio, ...
        'covered_count', covered_count, ...
        'total_tasks', num_tasks, ...
        'orientation_error', orient_diff, ...
        'approach_distance_m', approach_dist, ...
        'min_obstacle_dist_m', dist_to_obs_m);
end
