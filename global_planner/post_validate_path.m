function path = post_validate_path(grid_path, traversable)
%POST_VALIDATE_PATH Enforce table/wall avoidance for every path segment.
%   Some planners output sparse waypoints. A line drawn between sparse points
%   may cross table cells even if each waypoint itself is free. This function
%   checks every segment; if a segment is invalid, it replaces that segment by
%   an 8-neighbour A* grid path on the same traversability map.

    if isempty(grid_path)
        path = grid_path;
        return;
    end

    grid_path = round(grid_path(:, 1:2));
    rows = size(traversable, 1);
    cols = size(traversable, 2);

    % Snap all waypoints to legal cells. This is important because converting
    % world coordinates back to grid indices may land on a dilated table cell.
    for k = 1:size(grid_path, 1)
        grid_path(k, :) = nearest_free_cell(grid_path(k, :), traversable, rows, cols);
    end

    path = grid_path(1, :);
    for k = 2:size(grid_path, 1)
        p1 = path(end, :);
        p2 = grid_path(k, :);

        if isequal(p1, p2)
            continue;
        end

        if collision_free(p1, p2, traversable)
            path = [path; p2]; %#ok<AGROW>
        else
            seg = astar_grid_segment(p1, p2, traversable, rows, cols);
            if size(seg, 1) >= 2
                path = [path; seg(2:end, :)]; %#ok<AGROW>
            else
                warning('post_validate_path:repairFailed', ...
                    '路径段 [%d,%d] -> [%d,%d] 无法修复，已保留前一安全点。', ...
                    p1(1), p1(2), p2(1), p2(2));
            end
        end
    end

    % Remove consecutive duplicate grid cells.
    if size(path, 1) > 1
        keep = [true; any(diff(path, 1, 1) ~= 0, 2)];
        path = path(keep, :);
    end
end

function pt = nearest_free_cell(pt, traversable, rows, cols)
    pt = round(pt(:)');
    pt(1) = min(max(pt(1), 1), rows);
    pt(2) = min(max(pt(2), 1), cols);
    if traversable(pt(1), pt(2))
        return;
    end

    max_r = max(rows, cols);
    for r = 1:max_r
        best = [];
        best_d = inf;
        for di = -r:r
            for dj = -r:r
                if max(abs(di), abs(dj)) ~= r
                    continue;
                end
                ni = pt(1) + di;
                nj = pt(2) + dj;
                if ni >= 1 && ni <= rows && nj >= 1 && nj <= cols && traversable(ni, nj)
                    d = di^2 + dj^2;
                    if d < best_d
                        best_d = d;
                        best = [ni, nj]; %#ok<AGROW>
                    end
                end
            end
        end
        if ~isempty(best)
            pt = best;
            return;
        end
    end
end

function path = astar_grid_segment(start_cell, goal_cell, traversable, rows, cols)
    si = start_cell(1); sj = start_cell(2);
    gi = goal_cell(1);  gj = goal_cell(2);

    if ~traversable(si, sj) || ~traversable(gi, gj)
        path = [];
        return;
    end

    moves = [-1, 0, 1; 0, 1, 1; 1, 0, 1; 0, -1, 1; ...
             -1,-1, sqrt(2); -1, 1, sqrt(2); 1,-1, sqrt(2); 1, 1, sqrt(2)];

    g = inf(rows, cols);
    closed = false(rows, cols);
    parent = zeros(rows, cols, 2);
    g(si, sj) = 0;
    parent(si, sj, :) = [si, sj];
    open = [si, sj, hypot(si - gi, sj - gj)];

    found = false;
    while ~isempty(open)
        [~, idx] = min(open(:, 3));
        cur = open(idx, 1:2);
        open(idx, :) = [];
        ci = cur(1); cj = cur(2);
        if closed(ci, cj)
            continue;
        end
        closed(ci, cj) = true;

        if ci == gi && cj == gj
            found = true;
            break;
        end

        for m = 1:size(moves, 1)
            ni = ci + moves(m, 1);
            nj = cj + moves(m, 2);
            if ni < 1 || ni > rows || nj < 1 || nj > cols || ~traversable(ni, nj) || closed(ni, nj)
                continue;
            end
            if moves(m, 1) ~= 0 && moves(m, 2) ~= 0
                if ~traversable(ci, nj) || ~traversable(ni, cj)
                    continue;
                end
            end
            ng = g(ci, cj) + moves(m, 3);
            if ng < g(ni, nj)
                g(ni, nj) = ng;
                parent(ni, nj, :) = [ci, cj];
                f = ng + hypot(ni - gi, nj - gj);
                open = [open; ni, nj, f]; %#ok<AGROW>
            end
        end
    end

    if ~found
        path = [];
        return;
    end

    path = [gi, gj];
    ci = gi; cj = gj;
    guard = 0;
    while ~(ci == si && cj == sj) && guard < rows * cols
        guard = guard + 1;
        pi = parent(ci, cj, 1);
        pj = parent(ci, cj, 2);
        if pi == 0 && pj == 0
            path = [];
            return;
        end
        path = [pi, pj; path]; %#ok<AGROW>
        ci = pi; cj = pj;
    end
end
