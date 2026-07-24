function free = collision_free(p1, p2, traversable)
%COLLISION_FREE Strict grid segment validation.
%   p1, p2 are grid indices [row, col]. traversable=true means the cell can
%   be occupied by the mobile base. All non-traversable cells, including
%   table projections (grid_map==2) and walls (grid_map==1), are forbidden.

    p1 = round(p1(:)');
    p2 = round(p2(:)');
    rows_t = size(traversable, 1);
    cols_t = size(traversable, 2);

    if any(p1 < 1) || p1(1) > rows_t || p1(2) > cols_t || ...
       any(p2 < 1) || p2(1) > rows_t || p2(2) > cols_t
        free = false;
        return;
    end

    if ~traversable(p1(1), p1(2)) || ~traversable(p2(1), p2(2))
        free = false;
        return;
    end

    di = p2(1) - p1(1);
    dj = p2(2) - p1(2);
    steps = max(abs(di), abs(dj));
    if steps == 0
        free = true;
        return;
    end

    % Dense sampling at sub-cell resolution. This prevents any-angle planners
    % such as Theta*, Lazy Theta*, RRT, PSO and GA from drawing a straight
    % segment through table cells.
    n_check = max(ceil(steps * 20), 1);
    last_i = NaN; last_j = NaN;
    for k = 0:n_check
        t = k / n_check;
        i = round(p1(1) + t * di);
        j = round(p1(2) + t * dj);

        if i < 1 || i > rows_t || j < 1 || j > cols_t || ~traversable(i, j)
            free = false;
            return;
        end

        % Additional corner-cut guard: if a sampled point jumps diagonally
        % from the previous sampled cell, both side-adjacent cells must be free.
        if ~isnan(last_i) && abs(i - last_i) == 1 && abs(j - last_j) == 1
            if ~traversable(last_i, j) || ~traversable(i, last_j)
                free = false;
                return;
            end
        end
        last_i = i; last_j = j;
    end
    free = true;
end
