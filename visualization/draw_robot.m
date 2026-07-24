function h_group = draw_robot(pose, robot_cfg, fig_handle, tag)
    if nargin < 4
        tag = '';
    end
    if nargin < 3 || isempty(fig_handle)
        fig_handle = gcf;
    end
    figure(fig_handle);
    hold on;

    x = pose(1);
    y = pose(2);
    theta = pose(3);

    L = robot_cfg.chassis.length;
    W = robot_cfg.chassis.width;

    corners = [-L/2, -W/2; L/2, -W/2; L/2, W/2; -L/2, W/2];
    R_mat = rot2(theta);
    corners_w = zeros(4, 2);
    for c = 1:4
        corners_w(c, :) = (R_mat * corners(c, :)')' + [x, y];
    end
    patch(corners_w(:,1), corners_w(:,2), [0.15 0.35 0.65], ...
          'FaceAlpha', 0.55, 'EdgeColor', [0.05 0.15 0.4], 'LineWidth', 2);

    arrow_len = L * 0.45;
    dx = arrow_len * cos(theta);
    dy = arrow_len * sin(theta);
    quiver(x, y, dx, dy, 0, 'Color', [0.1 0.2 0.55], ...
           'LineWidth', 2.5, 'MaxHeadSize', 0.8, 'AutoScale', 'off');

    spacing = robot_cfg.dual_arm.base_spacing;
    mount_x = robot_cfg.arm_base_mount_x;

    lx = x + mount_x * cos(theta) + spacing/2 * cos(theta + pi/2);
    ly = y + mount_x * sin(theta) + spacing/2 * sin(theta + pi/2);
    rx = x + mount_x * cos(theta) - spacing/2 * cos(theta + pi/2);
    ry = y + mount_x * sin(theta) - spacing/2 * sin(theta + pi/2);

    plot(lx, ly, 'o', 'MarkerSize', 8, 'MarkerEdgeColor', [0.9 0.3 0.1], ...
         'MarkerFaceColor', [1 0.5 0.2], 'LineWidth', 1.5);
    plot(rx, ry, 'o', 'MarkerSize', 8, 'MarkerEdgeColor', [0.9 0.3 0.1], ...
         'MarkerFaceColor', [1 0.5 0.2], 'LineWidth', 1.5);

    reach_r = robot_cfg.single_arm.reach_radius;
    t_circle = linspace(0, 2*pi, 80);
    cx_l = lx + reach_r * cos(t_circle);
    cy_l = ly + reach_r * sin(t_circle);
    cx_r = rx + reach_r * cos(t_circle);
    cy_r = ry + reach_r * sin(t_circle);

    plot(cx_l, cy_l, '--', 'Color', [1 0.55 0.25 0.6], 'LineWidth', 1);
    plot(cx_r, cy_r, '--', 'Color', [1 0.55 0.25 0.6], 'LineWidth', 1);

    front_angles = linspace(theta - pi/3, theta + pi/3, 40);
    fw_depth = robot_cfg.dual_arm.collab_forward_depth;
    fw_width = robot_cfg.dual_arm.collab_lateral_width / 2;
    fw_base_x = (lx + rx) / 2;
    fw_base_y = (ly + ry) / 2;
    fw_x = [fw_base_x; fw_base_x + fw_depth * cos(theta)];
    fw_y = [fw_base_y; fw_base_y + fw_depth * sin(theta)];
    for sign = [-1, 1]
        fw_x = [fw_x; fw_base_x + fw_depth * cos(theta) + sign * fw_width * cos(theta + pi/2)];
        fw_y = [fw_y; fw_base_y + fw_depth * sin(theta) + sign * fw_width * sin(theta + pi/2)];
    end
    fw_x = [fw_x; fw_base_x];
    fw_y = [fw_y; fw_base_y];
    patch(fw_x, fw_y, [1 0.7 0.2], 'FaceAlpha', 0.12, 'EdgeColor', 'none');

    if ~isempty(tag)
        text(x + 0.08, y + 0.08, tag, 'FontSize', 10, 'FontWeight', 'bold', ...
             'Color', [0.1 0.2 0.5], 'BackgroundColor', [1 1 1 0.7], 'Margin', 1);
    end
end