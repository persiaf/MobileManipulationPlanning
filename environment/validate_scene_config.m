function all_ok = validate_scene_config(scene_cfg, grid_map, map_info)
    all_ok = true;
    fprintf('\n--- 场景合法性检查 ---\n');

    num_tables = size(scene_cfg.table_positions, 1);
    L = scene_cfg.table.length;
    W = scene_cfg.table.width;

    all_ok = all_ok & check_table_bounds(scene_cfg);
    all_ok = all_ok & check_table_overlap(scene_cfg);
    all_ok = all_ok & check_table_start_conflict(scene_cfg);
    all_ok = all_ok & check_start_free(grid_map, map_info, scene_cfg.start_pose);

    if all_ok
        fprintf('  场景配置检查通过！\n');
    end
end


function ok = check_table_bounds(scene_cfg)
    ok = true;
    L = scene_cfg.table.length;
    W = scene_cfg.table.width;
    margin = scene_cfg.table_wall_margin_m;

    for t = 1:size(scene_cfg.table_positions, 1)
        cx = scene_cfg.table_positions(t, 1);
        cy = scene_cfg.table_positions(t, 2);
        theta = scene_cfg.table_orientations(min(t, end));

        R = rot2(theta);
        half_L = L / 2;
        half_W = W / 2;
        corners_local = [-half_L, -half_W; half_L, -half_W; half_L, half_W; -half_L, half_W];
        for c = 1:4
            cl = corners_local(c, :)';
            cw = R * cl;
            wx = cx + cw(1);
            wy = cy + cw(2);
            if wx < margin || wx > scene_cfg.map_width_m - margin || ...
               wy < margin || wy > scene_cfg.map_height_m - margin
                fprintf('  ERROR: 桌子 %d 超出地图边界或距墙太近 (角落 (%.2f, %.2f), 边距要求 >= %.2f m)\n', ...
                        t, wx, wy, margin);
                ok = false;
                break;
            end
        end
    end
end


function ok = check_table_overlap(scene_cfg)
    ok = true;
    min_spacing = scene_cfg.table_min_spacing_m;
    num = size(scene_cfg.table_positions, 1);

    for i = 1:num
        for j = i+1:num
            d = norm(scene_cfg.table_positions(i, :) - scene_cfg.table_positions(j, :));
            if d < min_spacing
                fprintf('  ERROR: 桌子 %d 与桌子 %d 距离过近 (%.2f m < %.2f m)\n', ...
                        i, j, d, min_spacing);
                ok = false;
            end
        end
    end
end


function ok = check_table_start_conflict(scene_cfg)
    ok = true;
    sx = scene_cfg.start_pose(1);
    sy = scene_cfg.start_pose(2);
    margin = scene_cfg.table_start_margin_m;

    for t = 1:size(scene_cfg.table_positions, 1)
        cx = scene_cfg.table_positions(t, 1);
        cy = scene_cfg.table_positions(t, 2);
        d = sqrt((sx - cx)^2 + (sy - cy)^2);
        if d < margin
            fprintf('  ERROR: 桌子 %d 与起点距离过近 (%.2f m < %.2f m)\n', ...
                    t, d, margin);
            ok = false;
        end
    end
end


function ok = check_start_free(grid_map, map_info, start_pose)
    ok = true;
    [gi, gj] = world2grid(start_pose(1), start_pose(2), ...
                          map_info.resolution, map_info.height_m);
    if gi < 1 || gi > size(grid_map, 1) || gj < 1 || gj > size(grid_map, 2)
        fprintf('  ERROR: 起点超出地图范围\n');
        ok = false;
        return;
    end
    if grid_map(gi, gj) ~= 0
        fprintf('  ERROR: 起点位于不可通行区域 (grid_map=%d)\n', grid_map(gi, gj));
        ok = false;
    end
end