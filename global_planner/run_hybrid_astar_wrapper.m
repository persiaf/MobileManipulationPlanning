function path = run_hybrid_astar_wrapper(grid_map, start_pose, goal_pose, map_info, planner_cfg)
    rows = map_info.rows;
    cols = map_info.cols;

    [si, sj, gi, gj, traversable] = prepare_start_goal(grid_map, start_pose, goal_pose, map_info);

    n_theta = 12;
    theta_vals = linspace(0, 2*pi*(1 - 1/n_theta), n_theta);
    g_cost = inf(rows, cols, n_theta);
    parent = zeros(rows, cols, n_theta, 4);

    start_theta = start_pose(3);
    [~, st_idx] = min(abs(theta_vals - wrapToPiLocal(start_theta)));
    g_cost(si, sj, st_idx) = 0;

    open_list = [];
    h_s = sqrt((si - gi)^2 + (sj - gj)^2);
    open_list = [open_list; si, sj, st_idx, h_s];

    headings_visited = cell(rows, cols);

    step_size = 2;
    wheel_base = 0.3;
    max_steer = pi / 4;
    steer_angles = [-max_steer, -max_steer/2, 0, max_steer/2, max_steer];

    max_iter = rows * cols * 3;
    iter = 0;
    found = false;
    goal_idx = 1;

    while ~isempty(open_list) && iter < max_iter
        iter = iter + 1;

        [~, min_idx] = min(open_list(:, 4));
        ci = open_list(min_idx, 1);
        cj = open_list(min_idx, 2);
        ck = open_list(min_idx, 3);
        open_list(min_idx, :) = [];

        dist_to_goal = sqrt((ci - gi)^2 + (cj - gj)^2);
        if dist_to_goal < 3
            found = true;
            goal_idx = ck;
            break;
        end

        c_theta = theta_vals(ck);

        if isempty(headings_visited{ci, cj})
            headings_visited{ci, cj} = [c_theta];
        else
            visited_here = false;
            for hv = 1:length(headings_visited{ci, cj})
                if abs(wrapToPiLocal(c_theta - headings_visited{ci, cj}(hv))) < 0.15
                    visited_here = true;
                    break;
                end
            end
            if visited_here
                continue;
            end
            headings_visited{ci, cj} = [headings_visited{ci, cj}, c_theta];
        end

        for s = 1:length(steer_angles)
            steer = steer_angles(s);
            d_theta = steer * step_size;

            for d = 1:step_size
                new_theta = wrapToPiLocal(c_theta + steer * d);

                dx = cos(new_theta);
                dy = sin(new_theta);

                ni = round(ci + dy * d);
                nj = round(cj + dx * d);

                if ni < 1 || ni > rows || nj < 1 || nj > cols
                    continue;
                end
                if ~traversable(ni, nj)
                    break;
                end

                [~, nk] = min(abs(theta_vals - new_theta));
                new_g = g_cost(ci, cj, ck) + 1;

                if new_g < g_cost(ni, nj, nk)
                    g_cost(ni, nj, nk) = new_g;
                    parent(ni, nj, nk, :) = [ci, cj, ck, steer];

                    h = sqrt((ni - gi)^2 + (nj - gj)^2);
                    f = new_g + 1.5 * h;
                    open_list = [open_list; ni, nj, nk, f];
                end
            end
        end
    end

    if ~found
        path = [start_pose; goal_pose];
        return;
    end

    grid_path = [gi, gj];
    ci = gi; cj = gj; ck = goal_idx;
    prev_ci = 0; prev_cj = 0;

    max_steps = rows * cols;
    for step = 1:max_steps
        par = squeeze(parent(ci, cj, ck, :));
        if all(par == 0)
            break;
        end

        pari = par(1); parj = par(2); park = par(3);
        if pari ~= prev_ci || parj ~= prev_cj
            grid_path = [pari, parj; grid_path];
            prev_ci = pari; prev_cj = parj;
        end

        ci = pari; cj = parj; ck = park;

        if ci == si && cj == sj
            break;
        end
    end

    if grid_path(1,1) ~= si || grid_path(1,2) ~= sj
        grid_path = [si, sj; grid_path];
    end

    path = convert_path_grid2world(grid_path, map_info);
end

function h = heuristic_ha(i1, j1, i2, j2)
    h = sqrt((i1 - i2)^2 + (j1 - j2)^2);
end