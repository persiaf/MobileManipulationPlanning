function h_fig = draw_comprehensive_overview(scene, optimal_poses, global_path, ...
                                               all_poses, visit_order, executed_tasks, ...
                                               task_table_map, stats, pose_info, visual_cfg)
    if nargin < 10
        visual_cfg = planner_config().visualization;
    end
    if nargin < 9
        pose_info = [];
    end
    if nargin < 8
        stats = struct();
    end

    h_fig = figure('Name', '综合规划总图', 'Position', [50, 50, 1200, 850]);
    clf;

    ax_main = axes('Position', [0.06, 0.10, 0.68, 0.85]);
    hold(ax_main, 'on');
    res = scene.map_info.resolution;
    h_m  = scene.map_info.height_m;
    axes(ax_main);
    plot_grid(scene.grid_map, res, h_m);
    axis(ax_main, 'equal');
    xlim(ax_main, [0, scene.map_info.width_m]);
    ylim(ax_main, [0, scene.map_info.height_m]);
    xlabel(ax_main, 'X (m)', 'FontSize', 11);
    ylabel(ax_main, 'Y (m)', 'FontSize', 11);

    scene_cfg = scene.scene_cfg;
    for t = 1:size(scene_cfg.table_positions, 1)
        cx = scene_cfg.table_positions(t, 1);
        cy = scene_cfg.table_positions(t, 2);
        L  = scene_cfg.table.length;
        W  = scene_cfg.table.width;
        theta = scene_cfg.table_orientations(min(t, end));

        R = rot2(theta);
        corners_l = [-L/2, -W/2; L/2, -W/2; L/2, W/2; -L/2, W/2]';
        corners_w = R * corners_l;
        corners_w(1,:) = corners_w(1,:) + cx;
        corners_w(2,:) = corners_w(2,:) + cy;

        corners_w = round(corners_w / res) * res;

        plot(ax_main, [corners_w(1,:), corners_w(1,1)], ...
              [corners_w(2,:), corners_w(2,1)], ...
              '-', 'Color', [0.85 0.15 0.15], 'LineWidth', 2);

        plot(ax_main, corners_w(1,:), corners_w(2,:), ...
             's', 'MarkerSize', 10, 'MarkerFaceColor', [0.8 0.15 0.15], ...
             'MarkerEdgeColor', 'k', 'LineWidth', 1.5);

        text(ax_main, cx, cy, sprintf('T%d', t), ...
             'HorizontalAlignment', 'center', 'FontSize', 9, ...
             'FontWeight', 'bold', 'Color', 'w', ...
             'BackgroundColor', [0.45 0 0 0.7], 'Margin', 1);
    end

    for ti = 1:scene.num_tasks
        if scene.tasks(ti).status == 1
            plot(ax_main, scene.tasks(ti).x, scene.tasks(ti).y, 's', ...
                 'MarkerSize', 7, 'MarkerEdgeColor', [0.15 0.15 0.15], ...
                 'MarkerFaceColor', [0.5 0.5 0.5], 'LineWidth', 1);
        else
            plot(ax_main, scene.tasks(ti).x, scene.tasks(ti).y, 'o', ...
                 'MarkerSize', 7, 'MarkerEdgeColor', [0 0.5 0], ...
                 'MarkerFaceColor', [0.3 1 0.3], 'LineWidth', 1);
        end
    end

    plot(ax_main, scene.start_pose(1), scene.start_pose(2), 'p', ...
         'MarkerSize', 16, 'MarkerEdgeColor', 'k', ...
         'MarkerFaceColor', [0.2 0.4 1], 'LineWidth', 2);
    text(ax_main, scene.start_pose(1)+0.12, scene.start_pose(2)+0.05, '起点', ...
         'FontSize', 9, 'FontWeight', 'bold', 'Color', [0.1 0.2 0.5], ...
         'BackgroundColor', [1 1 1 0.8], 'Margin', 1);

    robot_cfg = robot_config();
    route_order_for_plot = visit_order;
    if isfield(stats, 'route_order') && ~isempty(stats.route_order)
        route_order_for_plot = stats.route_order;
    end

    if ~isempty(route_order_for_plot) && ~isempty(all_poses)
        if size(all_poses, 1) == size(optimal_poses, 1) + 1
            % 推荐路径：all_poses 为原始 [start; optimal_poses]，按访问顺序索引。
            dock_order = route_order_for_plot(route_order_for_plot > 1);
            ordered_poses = all_poses(dock_order, :);
            ordered_pose_indices = dock_order - 1;
        else
            % 兼容旧调用：all_poses 可能已经是 ordered_route_poses。
            ordered_poses = all_poses(2:end, :);
            if ~isempty(ordered_poses) && norm(ordered_poses(end,1:2) - scene.start_pose(1:2)) < 1e-6
                ordered_poses = ordered_poses(1:end-1, :);
            end
            ordered_pose_indices = 1:size(ordered_poses, 1);
        end
    else
        ordered_poses = optimal_poses;
        ordered_pose_indices = 1:size(ordered_poses, 1);
    end
    n_poses = size(ordered_poses, 1);
    pose_colors = lines(max(n_poses, 1));

    arm_left_color  = [0.85 0.35 0.15];
    arm_right_color = [0.15 0.35 0.85];

    for i = 1:n_poses
        pose = ordered_poses(i, :);
        x = pose(1);
        y = pose(2);
        theta = pose(3);

        L = robot_cfg.chassis.length;
        W = robot_cfg.chassis.width;
        c_l = [-L/2, -W/2; L/2, -W/2; L/2, W/2; -L/2, W/2]';
        R_mat = rot2(theta);
        c_w = R_mat * c_l;
        c_w(1,:) = c_w(1,:) + x;
        c_w(2,:) = c_w(2,:) + y;

        patch(ax_main, c_w(1,:), c_w(2,:), pose_colors(i,:), ...
              'FaceAlpha', 0.35, 'EdgeColor', pose_colors(i,:)*0.6, 'LineWidth', 1.2);

        arrow_len = L * 0.4;
        quiver(ax_main, x, y, arrow_len*cos(theta), arrow_len*sin(theta), 0, ...
               'Color', pose_colors(i,:)*0.6, 'LineWidth', 2, ...
               'MaxHeadSize', 0.7, 'AutoScale', 'off');

        spacing = robot_cfg.dual_arm.base_spacing;
        mount_x = robot_cfg.arm_base_mount_x;
        lx = x + mount_x * cos(theta) + spacing/2 * cos(theta + pi/2);
        ly = y + mount_x * sin(theta) + spacing/2 * sin(theta + pi/2);
        rx = x + mount_x * cos(theta) - spacing/2 * cos(theta + pi/2);
        ry = y + mount_x * sin(theta) - spacing/2 * sin(theta + pi/2);

        plot(ax_main, lx, ly, 'o', 'MarkerSize', 7, ...
             'MarkerEdgeColor', [0.9 0.3 0.1], 'MarkerFaceColor', [1 0.5 0.2]);
        plot(ax_main, rx, ry, 'o', 'MarkerSize', 7, ...
             'MarkerEdgeColor', [0.9 0.3 0.1], 'MarkerFaceColor', [1 0.5 0.2]);

        r_reach = robot_cfg.single_arm.reach_radius;
        t_circ = linspace(0, 2*pi, 80);
        if visual_cfg.show_arm_workspace
            plot(ax_main, lx + r_reach*cos(t_circ), ly + r_reach*sin(t_circ), '-', ...
                 'Color', [pose_colors(i,:) 0.25], 'LineWidth', 1);
            plot(ax_main, rx + r_reach*cos(t_circ), ry + r_reach*sin(t_circ), '-', ...
                 'Color', [pose_colors(i,:) 0.25], 'LineWidth', 1);
        end

        if visual_cfg.show_dense_debug_info
            table_str = '';
            side_str  = '';
            cov_str   = '';
            pose_info_idx = ordered_pose_indices(i);
            if ~isempty(pose_info) && pose_info_idx <= length(pose_info) && ~isempty(pose_info{pose_info_idx})
                info = pose_info{pose_info_idx};
                if isfield(info, 'table_id')
                    table_str = sprintf('T%d-', info.table_id);
                end
                if isfield(info, 'side_id')
                    side_str = sprintf('侧%d', info.side_id);
                end
                if isfield(info, 'covered_count')
                    cov_str = sprintf(' %dT', info.covered_count);
                end
            end
            tag = sprintf('D%d%s%s%s', i, table_str, side_str, cov_str);
        else
            tag = sprintf('D%d', i);
        end
        text(ax_main, x + 0.1, y + 0.06, tag, 'FontSize', 9, ...
             'FontWeight', 'bold', 'Color', pose_colors(i,:)*0.7, ...
             'BackgroundColor', [1 1 1 0.8], 'Margin', 1);

        if visual_cfg.show_arm_task_links && ...
                ~isempty(pose_info) && ordered_pose_indices(i) <= length(pose_info) && ~isempty(pose_info{ordered_pose_indices(i)})
            info = pose_info{ordered_pose_indices(i)};
            if isfield(info, 'covered_tasks') && ~isempty(info.covered_tasks)
                for ti_idx = 1:length(info.covered_tasks)
                    tid = info.covered_tasks(ti_idx);
                    if tid > scene.num_tasks
                        continue;
                    end

                    task_x = scene.tasks(tid).x;
                    task_y = scene.tasks(tid).y;

                    arm_side = 'none';
                    if isfield(info, 'arm_assign_map') && ~isempty(info.arm_assign_map)
                        if isa(info.arm_assign_map, 'containers.Map') && isKey(info.arm_assign_map, tid)
                            arm_side = info.arm_assign_map(tid);
                        elseif isa(info.arm_assign_map, 'containers.Map')
                            keys_list = info.arm_assign_map.keys;
                            for kidx = 1:length(keys_list)
                                if keys_list{kidx} == tid
                                    arm_side = info.arm_assign_map(keys_list{kidx});
                                    break;
                                end
                            end
                        end
                    end

                    if strcmp(arm_side, 'left')
                        link_color = arm_left_color;
                        link_style = '-';
                    elseif strcmp(arm_side, 'right')
                        link_color = arm_right_color;
                        link_style = '-';
                    else
                        link_color = [0.5 0.5 0.5];
                        link_style = ':';
                    end

                    plot(ax_main, [x, task_x], [y, task_y], ...
                         link_style, 'Color', link_color, 'LineWidth', 1.0);

                    if strcmp(arm_side, 'left')
                        text(ax_main, task_x + 0.04, task_y + 0.04, 'L', ...
                             'FontSize', 5, 'Color', arm_left_color, ...
                             'FontWeight', 'bold', ...
                             'BackgroundColor', [1 1 1 0.6], 'Margin', 0.3);
                    elseif strcmp(arm_side, 'right')
                        text(ax_main, task_x + 0.04, task_y + 0.04, 'R', ...
                             'FontSize', 5, 'Color', arm_right_color, ...
                             'FontWeight', 'bold', ...
                             'BackgroundColor', [1 1 1 0.6], 'Margin', 0.3);
                    end
                end
            end
        end
    end

    path_dist = 0;
    if ~isempty(global_path) && size(global_path, 1) > 1
        path_dist = compute_path_length(global_path);

        plot(ax_main, global_path(:,1), global_path(:,2), '-', ...
             'Color', [0.1 0.1 0.6], 'LineWidth', 2.5);

        plot(ax_main, global_path(:,1), global_path(:,2), '-', ...
             'Color', [0.3 0.5 1], 'LineWidth', 1);

        n_arrows = min(10, size(global_path, 1)-1);
        step = max(1, floor(size(global_path, 1) / (n_arrows + 1)));
        for a = step:step:size(global_path, 1)-1
            du = global_path(a+1,1) - global_path(a,1);
            dv = global_path(a+1,2) - global_path(a,2);
            L_uv = sqrt(du^2 + dv^2);
            if L_uv > 1e-6
                quiver(ax_main, global_path(a,1), global_path(a,2), ...
                       du/L_uv*0.1, dv/L_uv*0.1, 0, ...
                       'Color', [0.15 0.15 0.5], 'LineWidth', 1.2, ...
                       'MaxHeadSize', 0.7, 'AutoScale', 'off');
            end
        end

        plot(ax_main, global_path(end,1), global_path(end,2), 'h', ...
             'MarkerSize', 16, 'MarkerEdgeColor', 'k', ...
             'MarkerFaceColor', [0.8 0.2 0.2], 'LineWidth', 2);
    end

    return_str = '否';
    if isfield(stats, 'return_to_start') && stats.return_to_start
        return_str = '是';
    end
    total_time_s = 0;
    if isfield(stats, 'experiment_stats') && isfield(stats.experiment_stats, 'total') && ...
            isfield(stats.experiment_stats.total, 'estimated_total_time_s')
        total_time_s = stats.experiment_stats.total.estimated_total_time_s;
    end
    complete_rate = 0;
    if isfield(stats, 'rate')
        complete_rate = stats.rate;
    end
    title(ax_main, sprintf('RCH-MT-MMP | 路径 %.2f m | 返回起点: %s | 完成率 %.1f%% | 总时间 %.1f s', ...
          path_dist, return_str, complete_rate, total_time_s), ...
          'FontSize', 13, 'FontWeight', 'bold');

    ax_legend = axes('Position', [0.76, 0.10, 0.22, 0.85]);
    axis(ax_legend, 'off');
    hold(ax_legend, 'on');

    legend_str = {};
    legend_str{end+1} = '【 图例说明 】';
    legend_str{end+1} = '';
    legend_str{end+1} = '■ 黑色区域 — 障碍物';
    legend_str{end+1} = '■ 深红区域 — 桌面投影';
    legend_str{end+1} = '-- 红实线 — 桌子边缘/桌腿';
    legend_str{end+1} = '● 绿色 — 待执行任务';
    legend_str{end+1} = '■ 灰色 — 已完成任务';
    legend_str{end+1} = '★ 蓝色 — 起点';
    legend_str{end+1} = '◆ 红色 — 路径终点';
    legend_str{end+1} = '';
    legend_str{end+1} = '— 蓝线 — 全局路径';
    legend_str{end+1} = '→ 箭头 — 行驶方向';
    legend_str{end+1} = '';
    if visual_cfg.show_arm_task_links
        legend_str{end+1} = '— 橙线 — 左臂覆盖 (L)';
        legend_str{end+1} = '— 蓝线 — 右臂覆盖 (R)';
        legend_str{end+1} = '';
    end

    legend_str{end+1} = '【 场景概况 】';
    if isfield(stats, 'num_tables')
        legend_str{end+1} = sprintf('桌子数量: %d', stats.num_tables);
        if isfield(stats, 'table_pos')
            for t = 1:min(stats.num_tables, 6)
                legend_str{end+1} = sprintf(' T%d: (%.1f, %.1f)', ...
                    t, stats.table_pos(t,1), stats.table_pos(t,2));
            end
            if stats.num_tables > 6
                legend_str{end+1} = sprintf(' ... 共%d张桌子', stats.num_tables);
            end
        end
    end
    legend_str{end+1} = '';

    legend_str{end+1} = '【 执行统计 】';
    legend_str{end+1} = sprintf('任务总数: %d', scene.num_tasks);
    if ~isempty(executed_tasks)
        legend_str{end+1} = sprintf('已完成: %d', length(executed_tasks));
        legend_str{end+1} = sprintf('完成率: %.1f%%', 100*length(executed_tasks)/scene.num_tasks);
        rem = setdiff(1:scene.num_tasks, executed_tasks);
        if ~isempty(rem)
            legend_str{end+1} = sprintf('未完成: %d [ID: %s]', length(rem), num2str(rem));
        end
    end
    legend_str{end+1} = sprintf('停靠位姿: %d个', n_poses);
    if exist('path_dist', 'var')
        legend_str{end+1} = sprintf('路径总长: %.2f m', path_dist);
    end

    legend_str{end+1} = '';
    if n_poses > 0
        legend_str{end+1} = '【 停靠位姿列表 】';
        for i = 1:n_poses
            theta_deg = round(rad2deg(ordered_poses(i, 3)));
            table_str = '';
            side_str  = '';
            cov_str   = '';
            pose_info_idx = ordered_pose_indices(i);
            if ~isempty(pose_info) && pose_info_idx <= length(pose_info) && ~isempty(pose_info{pose_info_idx})
                info = pose_info{pose_info_idx};
                if isfield(info, 'table_id')
                    table_str = sprintf('T%d', info.table_id);
                end
                if isfield(info, 'side_id')
                    side_str = sprintf('侧%d', info.side_id);
                end
                if isfield(info, 'covered_count')
                    cov_str = sprintf('%dT', info.covered_count);
                end
            else
                if ~isempty(task_table_map) && i <= length(task_table_map)
                    table_str = sprintf('T%d', i);
                    cov_str = sprintf('%dT', length(task_table_map{i}));
                end
            end
            legend_str{end+1} = sprintf('D%d: %s %s %s %.1f°', ...
                                        i, table_str, side_str, cov_str, theta_deg);
        end
    end

    legend_str{end+1} = '';
    legend_str{end+1} = '【 访问顺序 】';
    if ~isempty(visit_order)
        visit_str = '起点';
        for vi = 1:length(visit_order)
            if visit_order(vi) == 1
                continue;
            end
            d_idx = visit_order(vi) - 1;
            if ~isempty(pose_info) && d_idx <= length(pose_info) && ~isempty(pose_info{d_idx})
                info = pose_info{d_idx};
                if isfield(info, 'table_id')
                    visit_str = [visit_str, ' → T' num2str(info.table_id) '-D' num2str(d_idx)];
                else
                    visit_str = [visit_str, ' → D' num2str(d_idx)];
                end
            else
                visit_str = [visit_str, ' → D' num2str(d_idx)];
            end
        end
        legend_str{end+1} = visit_str;
    end

    for li = 1:length(legend_str)
        y_pos = 1 - (li - 1) * 0.025;
        if y_pos < 0.02
            break;
        end
        if li == 1
            text(ax_legend, 0, y_pos, legend_str{li}, 'FontSize', 11, ...
                 'FontWeight', 'bold', 'Color', [0.15 0.15 0.4], ...
                 'Units', 'normalized');
        elseif startsWith(legend_str{li}, '【')
            text(ax_legend, 0, y_pos, legend_str{li}, 'FontSize', 9, ...
                 'FontWeight', 'bold', 'Color', [0.2 0.2 0.5], ...
                 'Units', 'normalized');
        else
            text(ax_legend, 0.02, y_pos, legend_str{li}, 'FontSize', 8, ...
                 'Color', [0.25 0.25 0.35], 'Units', 'normalized');
        end
    end

    drawnow;
end