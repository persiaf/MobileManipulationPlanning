function [grid_map, map_info] = build_grid_map(scene_cfg)
    cols = ceil(scene_cfg.map_width_m  / scene_cfg.grid_resolution_m);
    rows = ceil(scene_cfg.map_height_m / scene_cfg.grid_resolution_m);

    grid_map = zeros(rows, cols);

    border_pix = ceil(scene_cfg.border_thickness_m / scene_cfg.grid_resolution_m);

    grid_map(1:border_pix, :) = 1;
    grid_map(end-border_pix+1:end, :) = 1;
    grid_map(:, 1:border_pix) = 1;
    grid_map(:, end-border_pix+1:end) = 1;

    for t = 1:size(scene_cfg.table_positions, 1)
        cx = scene_cfg.table_positions(t, 1);
        cy = scene_cfg.table_positions(t, 2);
        theta = scene_cfg.table_orientations(t);

        [gi, gj] = world2grid(cx, cy, scene_cfg.grid_resolution_m, scene_cfg.map_height_m);
        h_pix = ceil(scene_cfg.table.width  / scene_cfg.grid_resolution_m);
        w_pix = ceil(scene_cfg.table.length / scene_cfg.grid_resolution_m);

        mask = rectMask(rows, cols, gi, gj, h_pix, w_pix, theta);
        grid_map(mask) = 2;

        leg_radius_pix = ceil(0.025 / scene_cfg.grid_resolution_m);
        R = rot2(theta);
        L2 = scene_cfg.table.length / 2;
        W2 = scene_cfg.table.width  / 2;
        leg_corners = [-L2, -W2;  L2, -W2;  L2,  W2; -L2,  W2];
        for leg = 1:4
            leg_local = leg_corners(leg, :)';
            leg_world = R * leg_local;
            lx = cx + leg_world(1);
            ly = cy + leg_world(2);
            [li, lj] = world2grid(lx, ly, scene_cfg.grid_resolution_m, scene_cfg.map_height_m);
            for di = -leg_radius_pix:leg_radius_pix
                for dj = -leg_radius_pix:leg_radius_pix
                    ni = li + di;
                    nj = lj + dj;
                    if ni >= 1 && ni <= rows && nj >= 1 && nj <= cols
                        if sqrt(di^2 + dj^2) <= leg_radius_pix
                            grid_map(ni, nj) = 1;
                        end
                    end
                end
            end
        end
    end

    map_info.rows = rows;
    map_info.cols = cols;
    map_info.resolution = scene_cfg.grid_resolution_m;
    map_info.width_m  = scene_cfg.map_width_m;
    map_info.height_m = scene_cfg.map_height_m;

    obstacle_mask = (grid_map > 0);
    dilation_cells = max(1, ceil(scene_cfg.safety_margin_m / scene_cfg.grid_resolution_m));
    for pass = 1:dilation_cells
        expanded = obstacle_mask;
        for i = 2:rows-1
            for j = 2:cols-1
                if ~obstacle_mask(i, j)
                    if any(any(obstacle_mask(i-1:i+1, j-1:j+1)))
                        expanded(i, j) = true;
                    end
                end
            end
        end
        obstacle_mask = expanded;
    end
    grid_map(obstacle_mask & (grid_map == 0)) = 1;
end