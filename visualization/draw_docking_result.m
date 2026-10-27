function h_legend = draw_docking_result(best_poses, scene, fig_handle, visit_order, pose_info, visual_cfg)
    if nargin < 6
        visual_cfg = legacy_visualization_config();
    end
    if nargin < 5
        pose_info = [];
    end
    if nargin < 4
        visit_order = [];
    end
    if nargin < 3 || isempty(fig_handle)
        fig_handle = gcf;
    end
    figure(fig_handle);
    hold on;

    robot_cfg = robot_config();
    n_poses = size(best_poses, 1);
    pose_colors = lines(max(n_poses, 1));

    arm_left_color  = [0.8 0.3 0.1];
    arm_right_color = [0.1 0.3 0.8];

    for i = 1:n_poses
        pose = best_poses(i, :);

        if ~isempty(visit_order)
            vis_idx = find(visit_order == i + 1, 1);
            if isempty(vis_idx)
                tag = sprintf('D%d', i);
            else
                tag = sprintf('D%d[%d]', i, vis_idx);
            end
        else
            tag = sprintf('D%d', i);
        end

        draw_robot(pose, robot_cfg, fig_handle, tag);

        plot(pose(1), pose(2), 'o', 'Color', pose_colors(i,:), ...
             'MarkerSize', 14, 'LineWidth', 2.5, ...
             'MarkerFaceColor', pose_colors(i,:) * 0.6 + 0.4);
    end

    if ~isempty(pose_info)
        for i = 1:min(n_poses, length(pose_info))
            info = pose_info{i};
            if isempty(info)
                continue;
            end

            pose = best_poses(i, :);
            table_id_str = '';
            if isfield(info, 'table_id')
                table_id_str = sprintf('T%d-', info.table_id);
            end

            if visual_cfg.show_dense_debug_info
                covered_str = '';
                if isfield(info, 'covered_count')
                    covered_str = sprintf(' %d任务', info.covered_count);
                end
                dock_label = sprintf('%sD%d%s', table_id_str, i, covered_str);
            else
                dock_label = sprintf('D%d', i);
            end
            text(pose(1) - 0.15, pose(2) - 0.18, dock_label, ...
                 'FontSize', 8, 'Color', pose_colors(i,:), ...
                 'BackgroundColor', [1 1 1 0.85], 'Margin', 1);

            if visual_cfg.show_arm_task_links && ...
                    isfield(info, 'covered_tasks') && ~isempty(info.covered_tasks)
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
                        link_color = arm_left_color;
                        link_style = '-';
                        link_width = 1.2;
                    elseif strcmp(arm_side, 'right')
                        link_color = arm_right_color;
                        link_style = '-';
                        link_width = 1.2;
                    else
                        link_color = [0.5 0.5 0.5];
                        link_style = ':';
                        link_width = 1.0;
                    end

                    plot([pose(1), task_x], [pose(2), task_y], ...
                         link_style, 'Color', link_color, 'LineWidth', link_width);

                    if strcmp(arm_side, 'left')
                        text(task_x + 0.04, task_y + 0.04, 'L', ...
                             'FontSize', 6, 'Color', arm_left_color, ...
                             'FontWeight', 'bold', ...
                             'BackgroundColor', [1 1 1 0.7], 'Margin', 0.5);
                    elseif strcmp(arm_side, 'right')
                        text(task_x + 0.04, task_y + 0.04, 'R', ...
                             'FontSize', 6, 'Color', arm_right_color, ...
                             'FontWeight', 'bold', ...
                             'BackgroundColor', [1 1 1 0.7], 'Margin', 0.5);
                    end
                end
            end
        end
    end

    for i = 1:n_poses
        pose = best_poses(i, :);
        spacing = robot_cfg.dual_arm.base_spacing;
        mount_x = robot_cfg.arm_base_mount_x;
        theta = pose(3);
        lx = pose(1) + mount_x * cos(theta) + spacing/2 * cos(theta + pi/2);
        ly = pose(2) + mount_x * sin(theta) + spacing/2 * sin(theta + pi/2);
        rx = pose(1) + mount_x * cos(theta) - spacing/2 * cos(theta + pi/2);
        ry = pose(2) + mount_x * sin(theta) - spacing/2 * sin(theta + pi/2);

        r_reach = robot_cfg.single_arm.reach_radius;
        t_circ = linspace(0, 2*pi, 80);
        if visual_cfg.show_arm_workspace && i == 1
            plot(lx + r_reach*cos(t_circ), ly + r_reach*sin(t_circ), '--', ...
                 'Color', [1 0.5 0.2 0.4], 'LineWidth', 0.8);
            plot(rx + r_reach*cos(t_circ), ry + r_reach*sin(t_circ), '--', ...
                 'Color', [1 0.5 0.2 0.4], 'LineWidth', 0.8);
        end
    end
end
