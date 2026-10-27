function library = build_cafeteria_navigation_library(regions, scene, robot_cfg, planner_cfg)
%BUILD_CAFETERIA_NAVIGATION_LIBRARY Reuse docking geometry with 2-D proxy support.

    empty = struct('region_id', {}, 'table_id', {}, 'region', {}, 'candidates', {}, ...
        'num_candidates', {}, 'task_ids', {}, 'generation_time_s', {}, ...
        'generation_diagnostics', {});
    library = struct('regions', empty, 'total_candidates', 0, ...
        'baseline_generation_time_s', 0, 'total_generation_time_s', 0, ...
        'service_support_mode', 'cafeteria_2d_proxy');
    t = tic;
    for ri = 1:numel(regions)
        [raw, diag] = generate_docking_candidates(regions(ri), scene, robot_cfg, planner_cfg);
        for ci = 1:numel(raw)
            raw(ci).service_support_mode = 'cafeteria_2d_proxy';
            raw(ci).physical_reachability = 'not_evaluated';
            raw(ci).supported_task_ids = [];
            raw(ci).proxy_service_score = 0;
            raw(ci).proxy_neutral_value = 0;
        end
        proxy = raw([]);
        support_counts = zeros(numel(regions(ri).task_ids), 1);
        for ci = 1:numel(raw)
            c = raw(ci);
            c.service_support_mode = 'cafeteria_2d_proxy';
            c.physical_reachability = 'not_evaluated';
            c.supported_task_ids = [];
            c.proxy_service_score = 0;
            % Bottom-Up expects a numeric reach term. Zero is an explicit
            % neutral proxy value, not a manipulation-reachability result.
            c.reach_scores = zeros(numel(regions(ri).task_ids), 3);
            c.proxy_neutral_value = 0;
            c.arm_assignment = repmat({'not_evaluated'}, numel(regions(ri).task_ids), 1);
            for ti = 1:numel(regions(ri).task_ids)
                task = scene.tasks(regions(ri).task_ids(ti));
                [ok, score] = evaluate_cafeteria_service_support(c, task, scene);
                if ok
                    c.supported_task_ids(end+1) = task.id; %#ok<AGROW>
                    support_counts(ti) = support_counts(ti) + 1;
                    c.proxy_service_score = max(c.proxy_service_score, score);
                end
            end
            c.covered_tasks = c.supported_task_ids;
            c.coverage_ratio = numel(c.covered_tasks) / max(numel(regions(ri).task_ids), 1);
            c.cost_breakdown = proxy_cost(c, regions(ri), scene, robot_cfg);
            c.total_cost = c.cost_breakdown.total;
            if ~isempty(c.covered_tasks), proxy(end+1) = c; end %#ok<AGROW>
        end
        diag.proxy_supported_candidate_count = numel(proxy);
        diag.proxy_support_count = support_counts;
        entry = struct('region_id', ri, 'table_id', regions(ri).table_id, ...
            'region', regions(ri), 'candidates', proxy, 'num_candidates', numel(proxy), ...
            'task_ids', regions(ri).task_ids, 'generation_time_s', 0, ...
            'generation_diagnostics', diag);
        library.regions(end+1) = entry; %#ok<AGROW>
        library.total_candidates = library.total_candidates + numel(proxy);
    end
    library.baseline_generation_time_s = toc(t);
    library.total_generation_time_s = library.baseline_generation_time_s;
end

function cost = proxy_cost(candidate, region, scene, robot_cfg)
    pose = candidate.pose;
    center = region.center;
    clearance = compute_footprint_clearance(pose, scene.grid_map, scene.map_info, robot_cfg);
    orient = abs(wrapToPiLocal(pose(3) - atan2(center(2)-pose(2), center(1)-pose(1)))) / pi;
    approach = min(1, norm(pose(1:2)-center) / 2);
    cost = struct('total', 0.45*(1-candidate.coverage_ratio) + 0.25*orient + ...
        0.20*approach + 0.10*max(0,1-clearance/0.25), ...
        'reachability', 0, 'coverage', 1-candidate.coverage_ratio, ...
        'orientation', orient, 'safety', max(0,1-clearance/0.25), ...
        'approach', approach, 'maneuver', orient, 'Gamma', NaN, ...
        'coverage_ratio', candidate.coverage_ratio, ...
        'covered_count', numel(candidate.covered_tasks), ...
        'total_tasks', numel(region.task_ids), 'orientation_error', orient*pi, ...
        'approach_distance_m', norm(pose(1:2)-center), 'min_obstacle_dist_m', clearance);
end
