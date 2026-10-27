function [changed_state, event] = apply_dynamic_event(planning_state, event_type, event_id)
%APPLY_DYNAMIC_EVENT Create a deterministic local discrete P5 change event.
%
% The event is determined only by the frozen planning state, event_type, and
% event_id. This function does not rerun stage1/stage2/stage3.

    planning_state = normalize_planning_state(planning_state);
    if nargin < 3 || isempty(event_id), event_id = 1; end
    event_type = upper(string(event_type));
    changed_state = planning_state;
    event = struct();
    event.type = char(event_type);
    event.event_id = event_id;
    event.seed = get_state_seed(planning_state);
    event.radius_m = 0.28;
    event.added_task_ids = [];
    event.target_dock_index = NaN;
    event.target_region = NaN;
    event.center = [NaN, NaN];
    event.target_task_id = NaN;
    event.invalidated_coverage_pairs = zeros(0, 2);
    event.reason = '';

    switch event_type
        case "ADD_LOCAL_OBSTACLE"
            [edge, center] = choose_route_edge(planning_state, event_id);
            event.target_edge = edge;
            event.center = center;
            event.radius_m = 0.32;
            changed_state.scene = add_circular_grid_obstacle(changed_state.scene, ...
                event.center, event.radius_m);
        case "INVALIDATE_DOCK"
            dock_idx = choose_dock(planning_state, event_id);
            event.target_dock_index = dock_idx;
            event.center = planning_state.plan.selected_poses(dock_idx, 1:2);
            event.radius_m = 0.35;
            event.target_region = dock_region(planning_state, dock_idx);
            changed_state.scene = add_circular_grid_obstacle(changed_state.scene, ...
                event.center, event.radius_m);
        case "ADD_TASKS"
            [tasks, region_id] = deterministic_added_tasks(planning_state, event_id);
            changed_state.scene.tasks = [changed_state.scene.tasks, tasks];
            changed_state.scene.num_tasks = numel(changed_state.scene.tasks);
            changed_state.scene.region_metadata = update_task_region_metadata( ...
                changed_state.scene.region_metadata, tasks, region_id);
            event.added_task_ids = [tasks.id];
            event.target_region = region_id;
            event.center = mean([[tasks.x]', [tasks.y]'], 1);
        case "TASK_COVERAGE_FAILURE"
            [task_id, dock_idx] = choose_coverage_pair(planning_state, event_id);
            event.target_task_id = task_id;
            event.target_dock_index = dock_idx;
            event.target_region = dock_region(planning_state, dock_idx);
            event.invalidated_coverage_pairs = [task_id, dock_idx];
            event.reason = 'deterministic_external_coverage_failure';
            event.center = planning_state.plan.selected_poses(dock_idx, 1:2);
        otherwise
            error('apply_dynamic_event:UnknownEvent', 'Unknown event type "%s".', event_type);
    end
end

function [task_id, dock_idx] = choose_coverage_pair(state, event_id)
    pairs = zeros(0, 2);
    for d = 1:numel(state.plan.pose_info_list)
        info = state.plan.pose_info_list{d};
        if isstruct(info) && isfield(info, 'covered_tasks')
            for tid = reshape(info.covered_tasks, 1, [])
                if ~is_task_verified(state, tid)
                    pairs(end + 1, :) = [tid, d]; %#ok<AGROW>
                end
            end
        end
    end
    if isempty(pairs), error('apply_dynamic_event:NoPendingCoverage', 'No pending coverage pair exists.'); end
    pair = pairs(1 + mod(event_id - 1, size(pairs, 1)), :);
    task_id = pair(1); dock_idx = pair(2);
end

function tf = is_task_verified(state, task_id)
    tf = isfield(state, 'task_state') && isstruct(state.task_state) && ...
        isfield(state.task_state, 'verified_tasks') && ...
        any(state.task_state.verified_tasks == task_id);
end

function scene = add_circular_grid_obstacle(scene, center, radius_m)
    map_info = scene.map_info;
    res = map_info.resolution;
    [ci, cj] = world2grid(center(1), center(2), res, map_info.height_m);
    r_cells = max(1, ceil(radius_m / res));
    rows = size(scene.grid_map, 1);
    cols = size(scene.grid_map, 2);
    for i = max(1, ci - r_cells):min(rows, ci + r_cells)
        for j = max(1, cj - r_cells):min(cols, cj + r_cells)
            [wx, wy] = grid2world(i, j, res, map_info.height_m);
            if norm([wx, wy] - center) <= radius_m
                scene.grid_map(i, j) = 1;
            end
        end
    end
end

function seed = get_state_seed(state)
    seed = NaN;
    if isfield(state, 'seed'), seed = state.seed; return; end
    if isfield(state, 'scene') && isfield(state.scene, 'scene_cfg') && ...
            isfield(state.scene.scene_cfg, 'random_seed')
        seed = state.scene.scene_cfg.random_seed;
    end
end

function [edge, center] = choose_route_edge(state, event_id)
    order = state.stats.route_order;
    edges = [order(1:end-1)', order(2:end)'];
    edges = edges(all(edges > 0, 2), :);
    idx = 1 + mod(event_id - 1, size(edges, 1));
    edge = edges(idx, :);
    p = state.all_poses(edge, 1:2);
    center = mean(p, 1);
end

function dock_idx = choose_dock(state, event_id)
    n = size(state.plan.selected_poses, 1);
    regions = reshape(state.plan.pose_to_region_map, [], 1);
    ids = unique(regions, 'stable');
    counts = arrayfun(@(r) sum(regions == r), ids);
    [~, ri] = max(counts);
    candidates = find(regions == ids(ri));
    dock_idx = candidates(1 + mod(event_id - 1, numel(candidates)));
    dock_idx = min(max(dock_idx, 1), n);
end

function rid = dock_region(state, dock_idx)
    rid = 1;
    if isfield(state.plan, 'pose_to_region_map') && numel(state.plan.pose_to_region_map) >= dock_idx
        rid = state.plan.pose_to_region_map(dock_idx);
    end
end

function [tasks, region_id] = deterministic_added_tasks(state, event_id)
    region_ids = state.scene.region_metadata.region_ids(:)';
    region_id = region_ids(1 + mod(event_id - 1, numel(region_ids)));
    table_regions = state.scene.scene_cfg.table_region_id(:);
    tables = find(table_regions == region_id);
    table_id = tables(1 + mod(event_id - 1, numel(tables)));
    center = state.scene.scene_cfg.table_positions(table_id, :);
    offsets = [0.18, 0.12; -0.18, 0.12; 0.18, -0.12; -0.18, -0.12];
    first_id = state.scene.num_tasks + 1;
    tasks = repmat(state.scene.tasks(1), 1, size(offsets, 1));
    for i = 1:size(offsets, 1)
        tasks(i).id = first_id + i - 1;
        tasks(i).x = center(1) + offsets(i, 1);
        tasks(i).y = center(2) + offsets(i, 2);
        tasks(i).z = state.scene.scene_cfg.task_offset_z;
        tasks(i).status = 0;
        tasks(i).table_id = table_id;
    end
end

function metadata = update_task_region_metadata(metadata, tasks, region_id)
    metadata.task_region_id = [metadata.task_region_id(:); repmat(region_id, numel(tasks), 1)];
    idx = find(metadata.region_ids == region_id, 1);
    if ~isempty(idx)
        metadata.tasks_per_region(idx) = metadata.tasks_per_region(idx) + numel(tasks);
    end
end
