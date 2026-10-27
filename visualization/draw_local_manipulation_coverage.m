function h_fig = draw_local_manipulation_coverage(scene, pose_info, pose_idx, visual_cfg)
    if nargin < 4
        visual_cfg = legacy_visualization_config();
    end
    if nargin < 3 || isempty(pose_idx)
        pose_idx = 1;
    end

    h_fig = figure('Name', sprintf('本地操作覆盖 — 停靠位姿 D%d', pose_idx), ...
                   'Position', [100, 100, 900, 700]);
    clf;
    hold on;

    res = scene.map_info.resolution;
    h_m = scene.map_info.height_m;
    plot_grid(scene.grid_map, res, h_m);
    axis equal;
    xlim([0, scene.map_info.width_m]);
    ylim([0, scene.map_info.height_m]);
    xlabel('X (m)', 'FontSize', 11);
    ylabel('Y (m)', 'FontSize', 11);

    scene_cfg = scene.scene_cfg;
    for t = 1:size(scene_cfg.table_positions, 1)
        cx = scene_cfg.table_positions(t, 1);
        cy = scene_cfg.table_positions(t, 2);
        L  = scene_cfg.table.length;
        W  = scene_cfg.table.width;
        theta = scene_cfg.table_orientations(min(t, end));

        corners_w = oriented_rectangle_corners([cx, cy], theta, L, W)';
        corners_w = round(corners_w / res) * res;

        plot([corners_w(1,:), corners_w(1,1)], ...
             [corners_w(2,:), corners_w(2,1)], ...
             '-', 'Color', [0.85 0.15 0.15], 'LineWidth', 2);
        text(cx, cy, sprintf('T%d', t), ...
             'HorizontalAlignment', 'center', 'FontSize', 9, ...
             'FontWeight', 'bold', 'Color', 'w', ...
             'BackgroundColor', [0.45 0 0 0.7], 'Margin', 1);
    end

    for ti = 1:scene.num_tasks
        if scene.tasks(ti).status == 1
            plot(scene.tasks(ti).x, scene.tasks(ti).y, 's', ...
                 'MarkerSize', 9, 'MarkerEdgeColor', [0.15 0.15 0.15], ...
                 'MarkerFaceColor', [0.5 0.5 0.5], 'LineWidth', 1);
        else
            plot(scene.tasks(ti).x, scene.tasks(ti).y, 'o', ...
                 'MarkerSize', 9, 'MarkerEdgeColor', [0 0.5 0], ...
                 'MarkerFaceColor', [0.3 1 0.3], 'LineWidth', 1);
            text(scene.tasks(ti).x + 0.04, scene.tasks(ti).y + 0.04, ...
                 num2str(ti), 'FontSize', 6, 'Color', [0 0.4 0], ...
                 'FontWeight', 'bold', 'BackgroundColor', [1 1 1 0.7], 'Margin', 0.5);
        end
    end

    robot_cfg = robot_config();
    arm_left_color  = [0.85 0.35 0.15];
    arm_right_color = [0.15 0.35 0.85];

    if pose_idx > length(pose_info) || isempty(pose_info{pose_idx})
        warning('pose_info{%d} 为空, 无法显示详细覆盖。', pose_idx);
        title(sprintf('停靠位姿 D%d — 无可用的覆盖信息', pose_idx));
        return;
    end

    info = pose_info{pose_idx};
    best_poses_dummy = zeros(pose_idx, 3);
    best_poses_dummy(pose_idx, :) = [info.x, info.y, info.theta];
    draw_robot(best_poses_dummy(pose_idx, :), robot_cfg, h_fig, sprintf('D%d', pose_idx));

    pose = best_poses_dummy(pose_idx, :);
    x = pose(1); y = pose(2); theta = pose(3);

    spacing = robot_cfg.dual_arm.base_spacing;
    mount_x = robot_cfg.arm_base_mount_x;
    lx = x + mount_x * cos(theta) + spacing/2 * cos(theta + pi/2);
    ly = y + mount_x * sin(theta) + spacing/2 * sin(theta + pi/2);
    rx = x + mount_x * cos(theta) - spacing/2 * cos(theta + pi/2);
    ry = y + mount_x * sin(theta) - spacing/2 * sin(theta + pi/2);

    plot(lx, ly, 'o', 'MarkerSize', 8, ...
         'MarkerEdgeColor', [0.9 0.3 0.1], 'MarkerFaceColor', [1 0.5 0.2]);
    plot(rx, ry, 'o', 'MarkerSize', 8, ...
         'MarkerEdgeColor', [0.9 0.3 0.1], 'MarkerFaceColor', [1 0.5 0.2]);
    text(lx + 0.06, ly + 0.04, '左臂', 'FontSize', 7, 'Color', arm_left_color, ...
         'FontWeight', 'bold');
    text(rx + 0.06, ry + 0.04, '右臂', 'FontSize', 7, 'Color', arm_right_color, ...
         'FontWeight', 'bold');

    r_reach = robot_cfg.single_arm.reach_radius;
    t_circ = linspace(0, 2*pi, 80);
    plot(lx + r_reach*cos(t_circ), ly + r_reach*sin(t_circ), '--', ...
         'Color', [arm_left_color 0.5], 'LineWidth', 1.2);
    plot(rx + r_reach*cos(t_circ), ry + r_reach*sin(t_circ), '--', ...
         'Color', [arm_right_color 0.5], 'LineWidth', 1.2);

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
                    vals_list = info.arm_assign_map.values;
                    for kidx = 1:length(keys_list)
                        if keys_list{kidx} == tid
                            arm_side = vals_list{kidx};
                            break;
                        end
                    end
                end
            end

            if strcmp(arm_side, 'left')
                link_color = arm_left_color; link_style = '-'; link_width = 1.5;
            elseif strcmp(arm_side, 'right')
                link_color = arm_right_color; link_style = '-'; link_width = 1.5;
            else
                link_color = [0.5 0.5 0.5]; link_style = ':'; link_width = 1.0;
            end

            plot([x, task_x], [y, task_y], ...
                 link_style, 'Color', link_color, 'LineWidth', link_width);

            if strcmp(arm_side, 'left')
                text(task_x + 0.05, task_y + 0.05, 'L', 'FontSize', 7, ...
                     'Color', arm_left_color, 'FontWeight', 'bold', ...
                     'BackgroundColor', [1 1 1 0.8], 'Margin', 0.5);
            elseif strcmp(arm_side, 'right')
                text(task_x + 0.05, task_y + 0.05, 'R', 'FontSize', 7, ...
                     'Color', arm_right_color, 'FontWeight', 'bold', ...
                     'BackgroundColor', [1 1 1 0.8], 'Margin', 0.5);
            end
        end
    end

    table_str = ''; side_str = ''; cov_str = '';
    if isfield(info, 'table_id')
        table_str = sprintf(' 服务桌子 T%d', info.table_id);
    end
    if isfield(info, 'side_id')
        side_str = sprintf(' 侧%d', info.side_id);
    end
    if isfield(info, 'covered_count')
        cov_str = sprintf(' 覆盖%d个任务', info.covered_count);
    end
    title(sprintf('停靠位姿 D%d —%s%s%s', pose_idx, table_str, side_str, cov_str), ...
          'FontSize', 12, 'FontWeight', 'bold');

    hold off;
end
