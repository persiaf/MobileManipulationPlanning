function [library, diagnostics] = augment_candidate_library( ...
        library, regions, scene, robot_cfg, planner_cfg)
%AUGMENT_CANDIDATE_LIBRARY Add task-driven candidates without removing L0.

    t_start = tic;
    cfg = get_config(planner_cfg);
    task_cells = arrayfun(@(r) r.task_ids(:), regions, 'UniformOutput', false);
    scoped_tasks = unique(vertcat(task_cells{:}), 'stable')';
    covered_before = library_covered_tasks(library);
    uncovered_before = setdiff(scoped_tasks, covered_before, 'stable');
    diagnostics = initialize_diagnostics(library, scoped_tasks, uncovered_before, cfg);

    if ~cfg.enable || isempty(uncovered_before)
        diagnostics.supplementation_time_s = toc(t_start);
        return;
    end

    pool = repmat(empty_pool_entry(), 0, 1);
    for ri = 1:numel(regions)
        region_uncovered = intersect(regions(ri).task_ids, uncovered_before, 'stable');
        if isempty(region_uncovered), continue; end
        raw_poses = generate_region_supplementary_poses( ...
            regions(ri), region_uncovered, scene, robot_cfg, cfg);
        diagnostics.raw_supplementary_count = diagnostics.raw_supplementary_count + size(raw_poses, 1);
        existing_poses = candidate_poses(library.regions(ri).candidates);

        task_positions = region_task_positions(regions(ri), scene);
        task_centroid = mean(task_positions, 1);
        for pi = 1:size(raw_poses, 1)
            pose = raw_poses(pi, 1:3); side_id = raw_poses(pi, 4);
            if check_footprint_collision(pose, scene.grid_map, scene.map_info, robot_cfg)
                continue;
            end
            if pose_exists(pose, existing_poses, cfg), continue; end
            diagnostics.collision_valid_supplementary_count = ...
                diagnostics.collision_valid_supplementary_count + 1;
            candidate = evaluate_pose(pose, side_id, regions(ri), scene, ...
                robot_cfg, planner_cfg, task_positions, task_centroid, library.regions(ri).candidates);
            if isempty(candidate.covered_tasks), continue; end
            diagnostics.reachable_supplementary_count = ...
                diagnostics.reachable_supplementary_count + 1;
            gain = intersect(candidate.covered_tasks, uncovered_before, 'stable');
            if isempty(gain), continue; end
            diagnostics.coverage_positive_supplementary_count = ...
                diagnostics.coverage_positive_supplementary_count + 1;
            entry = empty_pool_entry(); entry.region_id = ri;
            entry.candidate = candidate; entry.initial_gain = gain;
            pool(end + 1, 1) = entry; %#ok<AGROW>
            existing_poses(end + 1, :) = pose; %#ok<AGROW>
        end
    end

    [selected, remaining] = greedy_select(pool, uncovered_before, cfg, numel(regions));
    selected_per_region = zeros(1, numel(regions));
    for si = 1:numel(selected)
        ri = selected(si).region_id;
        library.regions(ri).candidates(end + 1) = selected(si).candidate;
        library.regions(ri).num_candidates = numel(library.regions(ri).candidates);
        library.regions(ri).candidates = annotate_ranks(library.regions(ri).candidates);
        selected_per_region(ri) = selected_per_region(ri) + 1;
        library.total_candidates = library.total_candidates + 1;
    end

    covered_after = library_covered_tasks(library);
    diagnostics.selected_supplementary_count = numel(selected);
    diagnostics.candidate_count_after = library.total_candidates;
    diagnostics.uncovered_tasks_after = numel(setdiff(scoped_tasks, covered_after));
    diagnostics.library_coverage_after = numel(intersect(scoped_tasks, covered_after)) / max(numel(scoped_tasks), 1);
    diagnostics.selected_per_region = selected_per_region;
    diagnostics.selected_region_ids = [selected.region_id];
    diagnostics.selected_poses = selected_poses(selected);
    diagnostics.remaining_task_ids = remaining;
    diagnostics.task = build_task_diagnostics( ...
        library, uncovered_before, diagnostics.support_count_before);
    diagnostics.recovery_rate = mean([diagnostics.task.recovered]);
    diagnostics.supplementation_time_s = toc(t_start);
end

function cfg = get_config(planner_cfg)
    defaults = struct('enable', false, 'variant', 'joint', ...
        'normal_offset_step_factors', [0, 0.25, 0.5, 0.75], ...
        'tangential_offset_step_factors', [-0.5, 0, 0.5], ...
        'baseline_orientation_step_factors', [-2, -1, 0, 1, 2], ...
        'refined_orientation_step_factors', [-2, -1.5, -1, -0.5, 0, 0.5, 1, 1.5, 2], ...
        'orientation_step_rad', pi / 12, ...
        'max_supplementary_candidates_per_task', 500, ...
        'max_supplementary_candidates_per_region', 40, ...
        'max_supplementary_candidates_total', 400, ...
        'position_tolerance_m', 0.01, 'heading_tolerance_rad', deg2rad(2));
    cfg = defaults;
    if isfield(planner_cfg, 'candidate_augmentation')
        names = fieldnames(defaults);
        for i = 1:numel(names)
            if isfield(planner_cfg.candidate_augmentation, names{i})
                cfg.(names{i}) = planner_cfg.candidate_augmentation.(names{i});
            end
        end
    end
    valid = {'position_only','orientation_only','joint'};
    if ~ismember(lower(cfg.variant), valid)
        error('augment_candidate_library:UnknownVariant', ...
            'Unknown candidate augmentation variant: %s.', cfg.variant);
    end
end

function poses = generate_region_supplementary_poses(region, task_ids, scene, robot_cfg, cfg)
    table_id = region.table_id;
    center = scene.scene_cfg.table_positions(table_id, :);
    table_theta = scene.scene_cfg.table_orientations(table_id);
    table = scene.scene_cfg.table;
    d_min = scene.scene_cfg.docking_min_distance_m;
    d_step = scene.scene_cfg.docking_distance_step_m;
    lat_step = scene.scene_cfg.docking_lateral_step_m;
    variant = lower(cfg.variant);
    poses = zeros(0, 4);

    if strcmp(variant, 'orientation_only')
        task_positions = region_task_positions(region, scene);
        field = compute_reachability_field(center, table_theta, scene.scene_cfg, ...
            robot_cfg, scene.map_info, mean(task_positions, 1), task_positions);
        baseline = [field.candidates, field.side_ids(:)];
        positions = unique_position_rows(baseline, cfg.position_tolerance_m);
    else
        positions = zeros(0, 4);
    end

    for ui = 1:numel(task_ids)
        tid = task_ids(ui);
        target = [scene.tasks(tid).x, scene.tasks(tid).y];
        target_local = rot2(-table_theta) * (target - center)';
        if ~strcmp(variant, 'orientation_only')
            positions = task_aligned_positions(target_local, center, table_theta, ...
                table, d_min, d_step, lat_step, cfg);
        end
        if strcmp(variant, 'position_only')
            angle_factors = cfg.baseline_orientation_step_factors;
        else
            angle_factors = cfg.refined_orientation_step_factors;
        end
        task_poses = zeros(0, 4);
        for pi = 1:size(positions, 1)
            world = positions(pi, 1:2); side_id = positions(pi, 4);
            nominal = atan2(target(2) - world(2), target(1) - world(1));
            for ai = 1:numel(angle_factors)
                theta = wrapToPiLocal(nominal + angle_factors(ai) * cfg.orientation_step_rad);
                task_poses(end + 1, :) = [world, theta, side_id]; %#ok<AGROW>
            end
        end
        task_poses = unique_pose_rows(task_poses, cfg);
        limit = min(size(task_poses, 1), cfg.max_supplementary_candidates_per_task);
        poses = [poses; task_poses(1:limit, :)]; %#ok<AGROW>
    end
    poses = unique_pose_rows(poses, cfg);
end

function positions = task_aligned_positions(target_local, center, table_theta, ...
        table, d_min, d_step, lat_step, cfg)
    positions = zeros(0, 4); R = rot2(table_theta);
    normals = d_min + cfg.normal_offset_step_factors * d_step;
    for side_id = 1:4
        if side_id <= 2, task_tangent = target_local(1); else, task_tangent = target_local(2); end
        tangents = task_tangent + cfg.tangential_offset_step_factors * lat_step;
        for d = normals
            for tangent = tangents
                switch side_id
                    case 1, local = [tangent; table.width / 2 + d];
                    case 2, local = [tangent; -table.width / 2 - d];
                    case 3, local = [-table.length / 2 - d; tangent];
                    otherwise, local = [table.length / 2 + d; tangent];
                end
                world = center(:) + R * local;
                positions(end + 1, :) = [world', 0, side_id]; %#ok<AGROW>
            end
        end
    end
end

function candidate = evaluate_pose(pose, side_id, region, scene, robot_cfg, ...
        planner_cfg, task_positions, task_centroid, baseline_candidates)
    task_ids = region.task_ids; n = numel(task_ids);
    if isempty(baseline_candidates)
        candidate = struct('pose',pose,'table_id',region.table_id,'side_id',side_id, ...
            'covered_tasks',[],'reach_scores',zeros(n,3),'arm_assignment',{cell(n,1)}, ...
            'nominal_covered_tasks',[],'nominal_reach_scores',zeros(n,3), ...
            'robust_covered_tasks',[],'robust_reachability_summary', ...
            repmat(struct('minimum_score',NaN,'mean_score',NaN,'success_fraction',NaN, ...
                'sample_count',0,'threshold',planner_cfg.reachability.threshold, ...
                'geometric_only',true,'error_model_source','unspecified', ...
                'error_model_measured',false),1,n), ...
            'coverage_ratio',0,'cost_breakdown',struct(),'total_cost',inf, ...
            'selected_candidate_rank_by_heuristic',0, ...
            'candidate_rank_percentile_in_region',0, ...
            'candidate_cost_gap_to_best_heuristic',0);
    else
        candidate = baseline_candidates(1);
        candidate.pose = pose; candidate.table_id = region.table_id;
        candidate.side_id = side_id; candidate.covered_tasks = [];
        candidate.reach_scores = zeros(n, 3); candidate.arm_assignment = cell(n, 1);
        candidate.nominal_covered_tasks = [];
        candidate.nominal_reach_scores = zeros(n, 3);
        candidate.robust_covered_tasks = [];
        candidate.robust_reachability_summary = repmat(struct( ...
            'minimum_score',NaN,'mean_score',NaN,'success_fraction',NaN, ...
            'sample_count',0,'threshold',planner_cfg.reachability.threshold, ...
            'geometric_only',true,'error_model_source','unspecified', ...
            'error_model_measured',false),1,n);
    end
    for ti = 1:n
        tid = task_ids(ti);
        target = [scene.tasks(tid).x, scene.tasks(tid).y, scene.tasks(tid).z];
        [ls, rs, cs, arm] = evaluate_dual_arm_reachability( ...
            pose, target, robot_cfg, planner_cfg.reachability.threshold);
        candidate.reach_scores(ti, :) = [ls, rs, cs];
        candidate.nominal_reach_scores(ti, :) = [ls, rs, cs];
        candidate.arm_assignment{ti} = arm;
        if cs > 0, candidate.nominal_covered_tasks(end + 1) = tid; end %#ok<AGROW>
        mode = getfield_default(planner_cfg.reachability, 'mode', 'nominal');
        if strcmpi(mode, 'robust')
            model = getfield_default(planner_cfg, 'docking_error_model', struct());
            candidate.robust_reachability_summary(ti) = evaluate_robust_reachability( ...
                pose, target, robot_cfg, planner_cfg.reachability.threshold, model);
            if candidate.robust_reachability_summary(ti).success_fraction >= ...
                    getfield_default(planner_cfg.reachability,'robust_success_fraction_min',1.0)
                candidate.robust_covered_tasks(end + 1) = tid; %#ok<AGROW>
            end
        end
    end
    mode = getfield_default(planner_cfg.reachability, 'mode', 'nominal');
    if strcmpi(mode, 'robust')
        candidate.covered_tasks = candidate.robust_covered_tasks;
        candidate.reach_scores(:,3) = [candidate.robust_reachability_summary.success_fraction]';
    else
        candidate.covered_tasks = candidate.nominal_covered_tasks;
    end
    candidate.coverage_ratio = numel(candidate.covered_tasks) / max(n, 1);
    candidate.cost_breakdown = evaluate_docking_candidate(candidate, region, scene, ...
        robot_cfg, planner_cfg, task_positions, task_centroid);
    candidate.total_cost = candidate.cost_breakdown.total;
end

function value = getfield_default(s, name, default_value)
    if isstruct(s) && isfield(s, name), value = s.(name); else, value = default_value; end
end

function [selected, remaining] = greedy_select(pool, uncovered, cfg, num_regions)
    selected = repmat(empty_pool_entry(), 0, 1);
    remaining = uncovered; available = true(1, numel(pool));
    region_counts = zeros(1, num_regions);
    while ~isempty(remaining) && numel(selected) < cfg.max_supplementary_candidates_total
        best = 0; best_gain = -1; best_cost = inf;
        for i = 1:numel(pool)
            ri = pool(i).region_id;
            if ~available(i) || region_counts(ri) >= cfg.max_supplementary_candidates_per_region
                continue;
            end
            gain = numel(intersect(pool(i).candidate.covered_tasks, remaining));
            cost = pool(i).candidate.total_cost;
            if gain > best_gain || (gain == best_gain && cost < best_cost)
                best = i; best_gain = gain; best_cost = cost;
            end
        end
        if best == 0 || best_gain <= 0, break; end
        selected(end + 1, 1) = pool(best); %#ok<AGROW>
        remaining = setdiff(remaining, pool(best).candidate.covered_tasks, 'stable');
        region_counts(pool(best).region_id) = region_counts(pool(best).region_id) + 1;
        available(best) = false;
    end
end

function diagnostics = initialize_diagnostics(library, scoped_tasks, uncovered, cfg)
    before_counts = task_support_counts(library, uncovered);
    diagnostics = struct('enabled',logical(cfg.enable),'variant',cfg.variant, ...
        'baseline_candidate_count',library.total_candidates, ...
        'uncovered_tasks_before',numel(uncovered), ...
        'raw_supplementary_count',0,'collision_valid_supplementary_count',0, ...
        'reachable_supplementary_count',0,'coverage_positive_supplementary_count',0, ...
        'selected_supplementary_count',0,'candidate_count_after',library.total_candidates, ...
        'uncovered_tasks_after',numel(uncovered), ...
        'library_coverage_before',(numel(scoped_tasks)-numel(uncovered))/max(numel(scoped_tasks),1), ...
        'library_coverage_after',(numel(scoped_tasks)-numel(uncovered))/max(numel(scoped_tasks),1), ...
        'supplementation_time_s',0,'recovery_rate',double(isempty(uncovered)), ...
        'support_count_before',before_counts,'selected_per_region',zeros(1,numel(library.regions)), ...
        'selected_region_ids',[],'selected_poses',zeros(0,3), ...
        'remaining_task_ids',uncovered,'task',repmat(empty_task_diag(),0,1));
    if ~isempty(uncovered)
        diagnostics.task = build_task_diagnostics(library, uncovered, before_counts);
    end
end

function task = build_task_diagnostics(library, task_ids, before_counts)
    after_counts = task_support_counts(library, task_ids);
    task = repmat(empty_task_diag(), numel(task_ids), 1);
    for i = 1:numel(task_ids)
        task(i).task_id = task_ids(i);
        task(i).region_id = find_task_region(library, task_ids(i));
        task(i).support_count_before = before_counts(i);
        task(i).support_count_after = after_counts(i);
        task(i).supplemental_support_count = max(0, after_counts(i) - before_counts(i));
        task(i).recovered = before_counts(i) == 0 && after_counts(i) > 0;
    end
end

function counts = task_support_counts(library, task_ids)
    counts = zeros(numel(task_ids), 1);
    for ri = 1:numel(library.regions)
        candidates = library.regions(ri).candidates;
        for ci = 1:numel(candidates)
            for ti = 1:numel(task_ids)
                counts(ti) = counts(ti) + ismember(task_ids(ti), candidates(ci).covered_tasks);
            end
        end
    end
end

function ri = find_task_region(library, task_id)
    ri = 0;
    for i = 1:numel(library.regions)
        if ismember(task_id, library.regions(i).task_ids), ri = i; return; end
    end
end

function tasks = library_covered_tasks(library)
    tasks = [];
    for ri = 1:numel(library.regions)
        for ci = 1:numel(library.regions(ri).candidates)
            tasks = union(tasks, library.regions(ri).candidates(ci).covered_tasks, 'stable');
        end
    end
end

function poses = candidate_poses(candidates)
    poses = zeros(0, 3);
    if ~isempty(candidates), poses = reshape([candidates.pose],3,[])'; end
end

function tf = pose_exists(pose, poses, cfg)
    if isempty(poses), tf = false; return; end
    pos_close = vecnorm(poses(:,1:2)-pose(1:2),2,2) < cfg.position_tolerance_m;
    yaw_close = abs(arrayfun(@(x) wrapToPiLocal(x-pose(3)), poses(:,3))) < cfg.heading_tolerance_rad;
    tf = any(pos_close & yaw_close);
end

function rows = unique_pose_rows(rows, cfg)
    out = zeros(0, 4);
    for i = 1:size(rows,1)
        if ~pose_exists(rows(i,1:3), out(:,1:3), cfg)
            out(end+1,:) = rows(i,:); %#ok<AGROW>
        end
    end
    rows = out;
end

function rows = unique_position_rows(rows, tolerance)
    out = zeros(0, 4);
    for i = 1:size(rows,1)
        if isempty(out) || ~any(vecnorm(out(:,1:2)-rows(i,1:2),2,2)<tolerance & out(:,4)==rows(i,4))
            out(end+1,:) = rows(i,:); %#ok<AGROW>
        end
    end
    rows = out;
end

function positions = region_task_positions(region, scene)
    positions = zeros(numel(region.task_ids), 2);
    for i = 1:numel(region.task_ids)
        tid = region.task_ids(i); positions(i,:) = [scene.tasks(tid).x, scene.tasks(tid).y];
    end
end

function candidates = annotate_ranks(candidates)
    costs = [candidates.total_cost]'; [best,~] = min(costs);
    [~,order] = sort(costs,'ascend'); ranks = zeros(numel(candidates),1);
    ranks(order) = 1:numel(candidates); denom = max(numel(candidates)-1,1);
    for i = 1:numel(candidates)
        candidates(i).selected_candidate_rank_by_heuristic = ranks(i);
        candidates(i).candidate_rank_percentile_in_region = (ranks(i)-1)/denom;
        candidates(i).candidate_cost_gap_to_best_heuristic = max(0,costs(i)-best);
    end
end

function poses = selected_poses(selected)
    poses = zeros(numel(selected),3);
    for i = 1:numel(selected), poses(i,:) = selected(i).candidate.pose; end
end

function entry = empty_pool_entry()
    entry = struct('region_id',0,'candidate',[],'initial_gain',[]);
end

function row = empty_task_diag()
    row = struct('task_id',0,'region_id',0,'support_count_before',0, ...
        'support_count_after',0,'supplemental_support_count',0,'recovered',false);
end
