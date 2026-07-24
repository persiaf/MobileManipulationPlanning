function path = run_convex_astar_wrapper(grid_map, start_pose, goal_pose, map_info, planner_cfg, run_opts)
%RUN_CONVEX_ASTAR_WRAPPER Convex-corner based A* global planner.
%   This wrapper only runs when cfg.global_planner='convex_astar'. It returns
%   the same world-coordinate path format as other global planner wrappers.
%   run_opts: 可选结构体, 字段 suppress_warnings (logical), 抑制回退 fprintf

    if nargin < 6
        run_opts = struct();
    end

    [si, sj, gi, gj, traversable] = prepare_start_goal(grid_map, start_pose, goal_pose, map_info);

    if ~isfield(planner_cfg, 'convex_astar') || ~planner_cfg.convex_astar.enable
        path = fallback_convex_astar(grid_map, start_pose, goal_pose, map_info, planner_cfg, ...
                                     'convex_astar disabled', run_opts);
        return;
    end

    corners = find_convex_corners(traversable);
    corners = unique([corners; gi, gj], 'rows', 'stable');

    corner_count = size(corners, 1);
    max_by_ratio = floor(numel(traversable) * planner_cfg.convex_astar.max_corner_ratio);
    max_corner_count = min(max_by_ratio, planner_cfg.convex_astar.max_corner_count);
    if corner_count > max_corner_count
        path = fallback_convex_astar(grid_map, start_pose, goal_pose, map_info, planner_cfg, ...
            sprintf('convex corners = %d exceeds limit %d', corner_count, max_corner_count), run_opts);
        return;
    end

    if planner_cfg.convex_astar.debug
        fprintf('[convex_astar] convex corners = %d\n', corner_count);
    end

    nodes = [[si, sj]; corners];
    nodes = unique(nodes, 'rows', 'stable');
    start_idx = find(nodes(:,1) == si & nodes(:,2) == sj, 1);
    goal_idx = find(nodes(:,1) == gi & nodes(:,2) == gj, 1);

    n_nodes = size(nodes, 1);
    g_cost = inf(n_nodes, 1);
    f_cost = inf(n_nodes, 1);
    parent = zeros(n_nodes, 1);
    parent_segment = cell(n_nodes, 1);
    closed = false(n_nodes, 1);

    g_cost(start_idx) = 0;
    f_cost(start_idx) = convex_heuristic(nodes(start_idx,:), nodes(goal_idx,:), planner_cfg);
    open_list = [start_idx, f_cost(start_idx)];

    found = false;
    max_iter = max(1, n_nodes * n_nodes);
    iter = 0;

    while ~isempty(open_list) && iter < max_iter
        iter = iter + 1;
        [~, min_idx] = min(open_list(:, 2));
        current_idx = open_list(min_idx, 1);
        open_list(min_idx, :) = [];

        if closed(current_idx)
            continue;
        end
        closed(current_idx) = true;

        if current_idx == goal_idx
            found = true;
            break;
        end

        current = nodes(current_idx, :);
        for next_idx = 2:n_nodes
            if next_idx == current_idx || closed(next_idx)
                continue;
            end

            [is_neighbor, segment_cells] = convex_neighbor_relation(current, nodes(next_idx, :), traversable);
            if ~is_neighbor
                continue;
            end

            g_new = g_cost(current_idx) + convex_segment_cost(segment_cells);
            if g_new < g_cost(next_idx)
                parent(next_idx) = current_idx;
                parent_segment{next_idx} = segment_cells;
                g_cost(next_idx) = g_new;
                f_cost(next_idx) = g_new + convex_heuristic(nodes(next_idx,:), nodes(goal_idx,:), planner_cfg);
                open_list = [open_list; next_idx, f_cost(next_idx)]; %#ok<AGROW>
            end
        end
    end

    if ~found
        path = fallback_convex_astar(grid_map, start_pose, goal_pose, map_info, planner_cfg, ...
                                     'search failed', run_opts);
        return;
    end

    path_grid = reconstruct_convex_path(start_idx, goal_idx, parent, parent_segment);
    if isempty(path_grid)
        path = fallback_convex_astar(grid_map, start_pose, goal_pose, map_info, planner_cfg, ...
                                     'path reconstruction failed', run_opts);
        return;
    end

    path = convert_path_grid2world(path_grid, map_info);
end

function h = convex_heuristic(p, goal, planner_cfg)
    dx = abs(p(1) - goal(1));
    dy = abs(p(2) - goal(2));
    if isfield(planner_cfg.convex_astar, 'heuristic') && strcmpi(planner_cfg.convex_astar.heuristic, 'euclidean')
        h = hypot(dx, dy);
    else
        h = max(dx, dy) + (sqrt(2) - 1) * min(dx, dy);
    end
end

function c = convex_segment_cost(segment_cells)
    if size(segment_cells, 1) < 2
        c = 0;
        return;
    end
    d = diff(segment_cells, 1, 1);
    c = sum(sqrt(sum(d.^2, 2)));
end

function path_grid = reconstruct_convex_path(start_idx, goal_idx, parent, parent_segment)
    idx_chain = goal_idx;
    cur = goal_idx;
    guard = 0;
    while cur ~= start_idx && guard < numel(parent)
        guard = guard + 1;
        cur = parent(cur);
        if cur == 0
            path_grid = [];
            return;
        end
        idx_chain = [cur; idx_chain]; %#ok<AGROW>
    end

    path_grid = [];
    for k = 2:length(idx_chain)
        seg = parent_segment{idx_chain(k)};
        if isempty(seg)
            path_grid = [];
            return;
        end
        if isempty(path_grid)
            path_grid = seg;
        else
            path_grid = [path_grid; seg(2:end, :)]; %#ok<AGROW>
        end
    end
end

function path = fallback_convex_astar(grid_map, start_pose, goal_pose, map_info, planner_cfg, reason, run_opts)
    if ~isfield(planner_cfg, 'convex_astar') || ...
       ~isfield(planner_cfg.convex_astar, 'enable_fallback') || ...
       ~planner_cfg.convex_astar.enable_fallback
        if nargin < 7 || ~isfield(run_opts, 'suppress_warnings') || ~run_opts.suppress_warnings
            warning('[convex_astar] %s, no fallback enabled.', reason);
        end
        path = [];
        return;
    end

    fallback = 'astar';
    if isfield(planner_cfg.convex_astar, 'fallback_planner')
        fallback = lower(planner_cfg.convex_astar.fallback_planner);
    end

    if nargin < 7 || ~isfield(run_opts, 'suppress_warnings') || ~run_opts.suppress_warnings
        fprintf('[convex_astar] %s, fallback to %s\n', reason, fallback);
    end
    switch fallback
        case 'jps'
            path = run_jps_wrapper(grid_map, start_pose, goal_pose, map_info, planner_cfg);
        otherwise
            path = run_astar_wrapper(grid_map, start_pose, goal_pose, map_info, planner_cfg);
    end
end
