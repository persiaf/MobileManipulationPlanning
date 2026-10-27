function [path, state, diagnostics] = run_dstar_lite_wrapper(grid_map, start_pose, goal_pose, map_info, planner_cfg, opts)
    state = [];
    diagnostics = struct('incremental_reuse', false, 'reinitialized', false, ...
        'changed_cell_count', 0, 'affected_edge_count', 0, 'updated_vertex_count', 0, ...
        'expanded_node_count', 0, 'queue_operation_count', 0, 'search_time_ms', 0, ...
        'fallback_used', false, 'fallback_reason', '', 'path_found', false, ...
        'path_safe', false, 'path_length', NaN, 'state_version', 0);
    if nargin >= 6 && isstruct(opts) && (isfield(opts, 'dstar_state') || isfield(opts, 'incremental_mode'))
        [path, state, diagnostics] = stateful_dstar_lite(grid_map, start_pose, goal_pose, map_info, planner_cfg, opts);
        return;
    end
    rows = map_info.rows;
    cols = map_info.cols;

    [si, sj, gi, gj, traversable] = prepare_start_goal(grid_map, start_pose, goal_pose, map_info);

    g = inf(rows, cols);
    rhs = inf(rows, cols);
    parent = zeros(rows, cols, 2);

    rhs(gi, gj) = 0;

    open_key1 = inf(rows, cols);
    open_key2 = inf(rows, cols);
    open_list = [];
    open_map = false(rows, cols);

    open_key1(gi, gj) = heuristic(gi, gj, si, sj);
    open_key2(gi, gj) = 0;
    open_list = sub2ind([rows, cols], gi, gj);
    open_map(gi, gj) = true;

    moves = [-1, 0, 1; 0, 1, 1; 1, 0, 1; 0, -1, 1;
             -1,-1, sqrt(2); -1, 1, sqrt(2); 1, -1, sqrt(2); 1, 1, sqrt(2)];

    max_iter = rows * cols;
    iter = 0;
    found = false;

    while ~isempty(open_list) && iter < max_iter
        iter = iter + 1;

        [~, min_idx] = min(open_key1(open_list) + open_key2(open_list));
        li = open_list(min_idx);
        [ui, uj] = ind2sub([rows, cols], li);
        open_list(min_idx) = [];
        open_map(ui, uj) = false;

        if g(ui, uj) > rhs(ui, uj)
            g(ui, uj) = rhs(ui, uj);
            if ui == si && uj == sj
                found = true;
            end
        else
            g(ui, uj) = inf;
            for d = 1:size(moves, 1)
                vi = ui + moves(d, 1); vj = uj + moves(d, 2);
                if vi < 1 || vi > rows || vj < 1 || vj > cols || ~traversable(vi, vj)
                    continue;
                end
                if moves(d, 1) ~= 0 && moves(d, 2) ~= 0
                    if ~traversable(ui, vj) || ~traversable(vi, uj)
                        continue;
                    end
                end
                if vi == gi && vj == gj
                    continue;
                end
                [rhs(vi, vj), parent(vi, vj, :)] = compute_rhs_val(vi, vj, g, gi, gj, traversable, moves, rows, cols);
                [g, rhs, open_list, open_map, open_key1, open_key2] = ...
                    update_state(vi, vj, g, rhs, open_list, open_map, open_key1, open_key2, si, sj);
            end
        end

        for d = 1:size(moves, 1)
            vi = ui + moves(d, 1); vj = uj + moves(d, 2);
            if vi < 1 || vi > rows || vj < 1 || vj > cols || ~traversable(vi, vj)
                continue;
            end
            if moves(d, 1) ~= 0 && moves(d, 2) ~= 0
                if ~traversable(ui, vj) || ~traversable(vi, uj)
                    continue;
                end
            end
            if vi == gi && vj == gj
                continue;
            end
            [rhs(vi, vj), parent(vi, vj, :)] = compute_rhs_val(vi, vj, g, gi, gj, traversable, moves, rows, cols);
            [g, rhs, open_list, open_map, open_key1, open_key2] = ...
                update_state(vi, vj, g, rhs, open_list, open_map, open_key1, open_key2, si, sj);
        end
    end

    if ~found
        path = [start_pose; goal_pose];
        return;
    end
    path = reconstruct_path_grid(si, sj, gi, gj, parent, map_info.resolution, map_info.height_m);
end

function h = heuristic(i1, j1, i2, j2)
    h = sqrt((i1 - i2)^2 + (j1 - j2)^2);
end

function [rhs_val, best_pred] = compute_rhs_val(vi, vj, g, gi, gj, traversable, moves, rows, cols)
    if vi == gi && vj == gj
        rhs_val = 0;
        best_pred = [0, 0];
        return;
    end
    best = inf;
    best_pred = [0, 0];
    for d = 1:size(moves, 1)
        pi = vi - moves(d, 1); pj = vj - moves(d, 2);
        if pi < 1 || pi > rows || pj < 1 || pj > cols || ~traversable(pi, pj)
            continue;
        end
        if moves(d, 1) ~= 0 && moves(d, 2) ~= 0
            if ~traversable(vi, pj) || ~traversable(pi, vj)
                continue;
            end
        end
        val = g(pi, pj) + moves(d, 3);
        if val < best
            best = val;
            best_pred = [pi, pj];
        end
    end
    rhs_val = best;
end

function [g, rhs, open_list, open_map, open_key1, open_key2] = update_state(vi, vj, g, rhs, open_list, open_map, open_key1, open_key2, si, sj)
    lin_idx = sub2ind(size(g), vi, vj);
    if ~open_map(vi, vj)
        if g(vi, vj) ~= rhs(vi, vj)
            open_map(vi, vj) = true;
            open_list = [open_list; lin_idx];
            open_key1(vi, vj) = min(g(vi, vj), rhs(vi, vj)) + heuristic(vi, vj, si, sj);
            open_key2(vi, vj) = min(g(vi, vj), rhs(vi, vj));
        end
    end
end

function [path, state, diagnostics] = stateful_dstar_lite(grid_map, start_pose, goal_pose, map_info, planner_cfg, opts)
%STATEFUL_DSTAR_LITE Persist g/rhs and the open queue between local updates.
    t0 = tic;
    [si, sj, gi, gj, traversable] = prepare_start_goal(grid_map, start_pose, goal_pose, map_info);
    previous = field_or(opts, 'dstar_state', []);
    compatible = isstruct(previous) && isfield(previous, 'g') && isequal(size(previous.g), size(traversable)) && ...
        isfield(previous, 'goal') && isequal(previous.goal, [gi, gj]) && isfield(previous, 'map_metadata') && ...
        previous.map_metadata.rows == map_info.rows && previous.map_metadata.cols == map_info.cols && ...
        abs(previous.map_metadata.resolution - map_info.resolution) < 1e-12 && ...
        abs(previous.map_metadata.height_m - map_info.height_m) < 1e-12 && ...
        field_or(previous, 'validated_path_safe', true) && ...
        isfield(previous, 'config_signature') && strcmp(previous.config_signature, stateful_config_signature(planner_cfg));
    reused = compatible;
    if reused
        state = previous;
        changed = field_or(opts, 'changed_cells', []);
        [r, c] = find(state.traversable ~= traversable);
        changed = unique([changed; r, c], 'rows');
        state.key_modifier = state.key_modifier + heuristic(state.start(1), state.start(2), si, sj);
        state.start = [si, sj]; state.previous_start = state.start; state.traversable = traversable;
        for k = 1:size(changed, 1), state = stateful_update_vertex_neighborhood(state, changed(k, :)); end
    else
        changed = zeros(0, 2);
        state = stateful_initialize(traversable, map_info, planner_cfg, [si, sj], [gi, gj]);
    end
    [state, search] = stateful_compute_shortest(state);
    [path, found, search_cost_grid] = stateful_extract_path(state, start_pose, goal_pose, map_info);
    path_safe = found && stateful_path_safe(path, state.traversable, map_info);
    state.validated_path_safe = path_safe;
    path_length_m = stateful_path_length(path);
    state.state_version = state.state_version + 1;
    state.map_signature = stateful_map_signature(traversable, map_info);
    state.signatures.map = state.map_signature;
    state.last_search = search;
    diagnostics = struct('incremental_reuse', reused, 'reinitialized', ~reused, ...
        'changed_cell_count', size(changed, 1), 'affected_edge_count', state.last_affected_edge_count, ...
        'updated_vertex_count', search.updated_vertex_count, 'expanded_node_count', search.expanded_node_count, ...
        'queue_operation_count', search.queue_operation_count, 'search_time_ms', toc(t0) * 1000, ...
        'fallback_used', false, 'fallback_reason', '', 'path_found', found, 'path_safe', path_safe, ...
        'search_cost_grid', search_cost_grid, 'path_length_m', path_length_m, ...
        'path_length', path_length_m, 'state_version', state.state_version);
end

function state = stateful_initialize(traversable, map_info, planner_cfg, start, goal)
    state = struct('g', inf(size(traversable)), 'rhs', inf(size(traversable)), ...
        'open_list', zeros(0, 1), 'open_keys', zeros(0, 2), 'key_modifier', 0, ...
        'start', start, 'goal', goal, 'previous_start', start, 'traversable', traversable, ...
        'map_metadata', struct('rows', map_info.rows, 'cols', map_info.cols, ...
            'resolution', map_info.resolution, 'height_m', map_info.height_m), ...
        'footprint_config', field_or(planner_cfg, 'robot_cfg', struct()), ...
        'planner_options', planner_cfg, 'state_version', 0, 'last_affected_edge_count', 0, ...
        'config_signature', stateful_config_signature(planner_cfg), 'signatures', struct(), 'last_search', struct());
    state.map_signature = stateful_map_signature(traversable, map_info);
    state.signatures.map = state.map_signature;
    state.rhs(goal(1), goal(2)) = 0;
    state = stateful_insert_open(state, goal);
end

function [state, search] = stateful_compute_shortest(state)
    t0 = tic; expanded = 0; queue_ops = 0; updated = 0; limit = max(1, numel(state.g) * 20);
    while ~isempty(state.open_list) && expanded < limit
        [top, ~] = stateful_peek_open(state);
        if ~stateful_key_less(top, stateful_calculate_key(state, state.start)) && ...
                state.rhs(state.start(1), state.start(2)) == state.g(state.start(1), state.start(2)), break; end
        [state, u, old_key] = stateful_pop_open(state); queue_ops = queue_ops + 1;
        if stateful_key_less(old_key, stateful_calculate_key(state, u))
            state = stateful_insert_open(state, u); queue_ops = queue_ops + 1;
        elseif state.g(u(1), u(2)) > state.rhs(u(1), u(2))
            state.g(u(1), u(2)) = state.rhs(u(1), u(2));
            pred = stateful_neighbors(u, size(state.g));
            for i = 1:size(pred, 1), state = stateful_update_vertex(state, pred(i, :)); updated = updated + 1; end
        else
            state.g(u(1), u(2)) = inf; state = stateful_update_vertex(state, u); updated = updated + 1;
            pred = stateful_neighbors(u, size(state.g));
            for i = 1:size(pred, 1), state = stateful_update_vertex(state, pred(i, :)); updated = updated + 1; end
        end
        expanded = expanded + 1;
    end
    search = struct('expanded_node_count', expanded, 'queue_operation_count', queue_ops, ...
        'updated_vertex_count', updated, 'search_time_ms', toc(t0) * 1000);
end

function state = stateful_update_vertex_neighborhood(state, cell_rc)
    affected = stateful_neighbors(cell_rc, size(state.g));
    state.last_affected_edge_count = state.last_affected_edge_count + size(affected, 1);
    state = stateful_update_vertex(state, cell_rc);
    for i = 1:size(affected, 1), state = stateful_update_vertex(state, affected(i, :)); end
end

function state = stateful_update_vertex(state, u)
    if ~isequal(u, state.goal), state.rhs(u(1), u(2)) = stateful_rhs(state, u); end
    state = stateful_remove_open(state, u);
    if state.g(u(1), u(2)) ~= state.rhs(u(1), u(2)), state = stateful_insert_open(state, u); end
end

function value = stateful_rhs(state, u)
    if ~state.traversable(u(1), u(2)), value = inf; return; end
    value = inf; next = stateful_neighbors(u, size(state.g));
    for i = 1:size(next, 1)
        c = stateful_edge_cost(u, next(i, :), state.traversable);
        if isfinite(c), value = min(value, c + state.g(next(i, 1), next(i, 2))); end
    end
end

function [path, found, cost] = stateful_extract_path(state, start_pose, goal_pose, map_info)
    cost = state.g(state.start(1), state.start(2)); found = isfinite(cost);
    if ~found, path = zeros(0, 3); return; end
    cells = state.start; current = state.start;
    for k = 1:(numel(state.g) + 1)
        if isequal(current, state.goal), break; end
        next = stateful_neighbors(current, size(state.g)); best = inf; choice = [];
        for i = 1:size(next, 1)
            c = stateful_edge_cost(current, next(i, :), state.traversable);
            if isfinite(c) && c + state.g(next(i, 1), next(i, 2)) < best, best = c + state.g(next(i, 1), next(i, 2)); choice = next(i, :); end
        end
        if isempty(choice), found = false; path = zeros(0, 3); cost = inf; return; end
        cells(end + 1, :) = choice; current = choice;
    end
    found = isequal(current, state.goal);
    if ~found, path = zeros(0, 3); cost = inf; return; end
    path = zeros(size(cells, 1), 3);
    for i = 1:size(cells, 1), [path(i, 1), path(i, 2)] = grid2world(cells(i, 1), cells(i, 2), map_info.resolution, map_info.height_m); end
    path(1, :) = start_pose; path(end, :) = goal_pose;
end

function state = stateful_insert_open(state, u)
    state = stateful_remove_open(state, u);
    state.open_list(end + 1, 1) = sub2ind(size(state.g), u(1), u(2));
    state.open_keys(end + 1, :) = stateful_calculate_key(state, u);
end
function state = stateful_remove_open(state, u)
    state.open_list = state.open_list(:);
    if isempty(state.open_list), state.open_keys = zeros(0, 2); return; end
    if size(state.open_keys, 1) ~= numel(state.open_list), error('run_dstar_lite_wrapper:QueueInvariant', 'D* Lite open queue shape is inconsistent.'); end
    lin = sub2ind(size(state.g), u(1), u(2)); keep = state.open_list ~= lin;
    state.open_list = state.open_list(keep); state.open_keys = state.open_keys(keep, :);
end
function [key, idx] = stateful_peek_open(state)
    idx = 1;
    for i = 2:size(state.open_keys, 1), if stateful_key_less(state.open_keys(i, :), state.open_keys(idx, :)), idx = i; end, end
    key = state.open_keys(idx, :);
end
function [state, u, key] = stateful_pop_open(state)
    [key, idx] = stateful_peek_open(state); lin = state.open_list(idx); [i, j] = ind2sub(size(state.g), lin); u = [i, j];
    state.open_list(idx) = []; state.open_keys(idx, :) = [];
end
function key = stateful_calculate_key(state, u)
    m = min(state.g(u(1), u(2)), state.rhs(u(1), u(2)));
    key = [m + stateful_heuristic(u(1), u(2), state.start(1), state.start(2)) + state.key_modifier, m];
end
function tf = stateful_key_less(a, b), tf = a(1) < b(1) - 1e-12 || (abs(a(1) - b(1)) <= 1e-12 && a(2) < b(2) - 1e-12); end
function n = stateful_neighbors(u, sz), m = [-1,0;0,1;1,0;0,-1;-1,-1;-1,1;1,-1;1,1]; n = u + m; n = n(n(:,1)>=1 & n(:,1)<=sz(1) & n(:,2)>=1 & n(:,2)<=sz(2), :); end
function c = stateful_edge_cost(u, v, map), if ~map(u(1),u(2)) || ~map(v(1),v(2)), c=inf; return; end; if u(1)~=v(1) && u(2)~=v(2) && (~map(u(1),v(2)) || ~map(v(1),u(2))), c=inf; return; end; c=hypot(double(u(1)-v(1)),double(u(2)-v(2))); end
function h = stateful_heuristic(i1,j1,i2,j2), h=hypot(double(i1-i2),double(j1-j2)); end
function sig = stateful_map_signature(map, info)
    payload = struct('map', logical(map), 'rows', info.rows, 'cols', info.cols, ...
        'resolution', info.resolution, 'height_m', info.height_m);
    sig = build_struct_signature(payload);
end
function sig = stateful_config_signature(cfg)
    payload = struct('global_planner', field_or(cfg, 'global_planner', 'dstar_lite'), ...
        'astar', field_or(cfg, 'astar', struct()), ...
        'footprint_aware_navigation', field_or(cfg, 'footprint_aware_navigation', false), ...
        'robot_cfg', field_or(cfg, 'robot_cfg', field_or(cfg, 'robot', struct())));
    sig = build_struct_signature(payload);
end
function ok = stateful_path_safe(path, map, info)
    ok = ~isempty(path) && size(path, 1) >= 2;
    if ~ok, return; end
    cells = zeros(size(path, 1), 2);
    for i = 1:size(path, 1)
        [cells(i, 1), cells(i, 2)] = world2grid(path(i, 1), path(i, 2), info.resolution, info.height_m);
        ok = ok && cells(i, 1) >= 1 && cells(i, 1) <= size(map, 1) && cells(i, 2) >= 1 && cells(i, 2) <= size(map, 2) && map(cells(i, 1), cells(i, 2));
    end
    if ~ok, return; end
    delta = abs(diff(cells, 1, 1)); ok = all(max(delta, [], 2) <= 1 & sum(delta, 2) >= 1);
end
function value = stateful_path_length(path)
    if size(path, 1) < 2, value = inf; else, value = sum(sqrt(sum(diff(path(:, 1:2), 1, 1).^2, 2))); end
end
function value = field_or(s,name,fallback), if isstruct(s) && isfield(s,name), value=s.(name); else, value=fallback; end, end
