function scene = apply_static_obstacles_to_scene(scene, obstacles)
%APPLY_STATIC_OBSTACLES_TO_SCENE Add deterministic block obstacles to grid map.
%
% Obstacles use the same occupancy-grid representation as P5 local changes:
% grid_map == 1 means occupied. The function returns a modified scene copy and
% records valid obstacles in scene.static_obstacles.

    if nargin < 2 || isempty(obstacles)
        scene.static_obstacles = struct('id', {}, 'type', {}, 'center', {}, 'size', {}, 'theta', {});
        return;
    end
    valid = struct('id', {}, 'type', {}, 'center', {}, 'size', {}, 'theta', {});
    for i = 1:numel(obstacles)
        obs = normalize_obstacle(obstacles(i), i);
        if obstacle_overlaps_scene(scene, obs)
            warning('apply_static_obstacles_to_scene:SkippedInvalidObstacle', ...
                'Skipping obstacle %s because it overlaps start/table/task geometry.', obs.id);
            continue;
        end
        scene.grid_map = rasterize_block(scene.grid_map, scene.map_info, obs);
        valid(end + 1) = obs; %#ok<AGROW>
    end
    scene.static_obstacles = valid;
end

function obs = normalize_obstacle(obs, idx)
    if ~isfield(obs, 'id') || isempty(obs.id), obs.id = sprintf('O%d', idx); end
    if ~isfield(obs, 'type') || isempty(obs.type), obs.type = 'block'; end
    if ~isfield(obs, 'theta') || isempty(obs.theta), obs.theta = 0; end
end

function tf = obstacle_overlaps_scene(scene, obs)
    c = obs.center(:)';
    half = obs.size(:)' / 2;
    start_clearance = norm(scene.start_pose(1:2) - c);
    tf = start_clearance < max(half) + 0.05;
    if tf, return; end
    for i = 1:size(scene.scene_cfg.table_positions, 1)
        if point_in_or_near_block(scene.scene_cfg.table_positions(i, :), obs, ...
                0.0)
            tf = true; return;
        end
    end
    for i = 1:scene.num_tasks
        if point_in_or_near_block([scene.tasks(i).x, scene.tasks(i).y], obs, 0.08)
            tf = true; return;
        end
    end
end

function tf = point_in_or_near_block(p, obs, margin)
    R = rot2(-obs.theta);
    local = R * (p(:) - obs.center(:));
    half = obs.size(:) / 2 + margin;
    tf = all(abs(local) <= half);
end

function grid = rasterize_block(grid, map_info, obs)
    res = map_info.resolution;
    rows = size(grid, 1);
    cols = size(grid, 2);
    half_diag = norm(obs.size(:)) / 2;
    [ci, cj] = world2grid(obs.center(1), obs.center(2), res, map_info.height_m);
    span = ceil(half_diag / res) + 2;
    for i = max(1, ci - span):min(rows, ci + span)
        for j = max(1, cj - span):min(cols, cj + span)
            [wx, wy] = grid2world(i, j, res, map_info.height_m);
            if point_in_or_near_block([wx, wy], obs, 0)
                grid(i, j) = 1;
            end
        end
    end
end
