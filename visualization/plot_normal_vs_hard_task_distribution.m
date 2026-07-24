function plot_normal_vs_hard_task_distribution(out_dir, seed)
%PLOT_NORMAL_VS_HARD_TASK_DISTRIBUTION Compare normal and hard task layouts.
%
%   Output:
%     fig_normal_vs_hard_task_distribution.png/.fig/.pdf

    if nargin < 1 || isempty(out_dir)
        out_dir = fullfile(fileparts(fileparts(mfilename('fullpath'))), ...
                           'results', 'figures');
    end
    if nargin < 2 || isempty(seed)
        seed = 2026;
    end
    if ~exist(out_dir, 'dir'), mkdir(out_dir); end

    try
        cfg_normal = create_scaled_scene_config('small', seed);
        cfg_normal.task.difficulty = 'normal';
        scene_normal = build_scene_from_cfg(cfg_normal);

        cfg_hard = create_scaled_scene_config('small', seed);
        cfg_hard.task.difficulty = 'hard_mixed';
        scene_hard = build_scene_from_cfg(cfg_hard);
    catch ME
        warning('plot_normal_vs_hard_task_distribution:failed', ...
            'Task distribution figure skipped: %s', ME.message);
        return;
    end

    fig = figure('Name', 'Normal vs Hard Task Distribution', ...
                 'Visible', 'off', 'Color', 'w', 'Position', [80, 80, 1050, 500]);
    tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    ax1 = nexttile;
    draw_task_scene(ax1, scene_normal, [0.20 0.70 0.30], 'Normal random');

    ax2 = nexttile;
    draw_task_scene(ax2, scene_hard, [0.85 0.25 0.20], 'Hard mixed');

    export_pub_figure(fig, fullfile(out_dir, 'fig_normal_vs_hard_task_distribution'));
    close(fig);
end

function draw_task_scene(ax, scene, task_color, title_text)
    hold(ax, 'on');
    axis(ax, 'equal');
    xlim(ax, [0, scene.map_info.width_m]);
    ylim(ax, [0, scene.map_info.height_m]);

    plot_tables(ax, scene);

    x = arrayfun(@(s) s.x, scene.tasks);
    y = arrayfun(@(s) s.y, scene.tasks);
    plot(ax, x, y, 'o', 'MarkerSize', 6.0, ...
        'MarkerFaceColor', task_color, 'MarkerEdgeColor', [0.15 0.15 0.15], ...
        'LineStyle', 'none');

    plot(ax, scene.start_pose(1), scene.start_pose(2), 'p', ...
        'MarkerSize', 12, 'MarkerFaceColor', [0.95 0.20 0.15], ...
        'MarkerEdgeColor', 'k', 'LineWidth', 1.0);

    xlabel(ax, 'X (m)', 'Interpreter', 'none');
    ylabel(ax, 'Y (m)', 'Interpreter', 'none');
    title(ax, sprintf('%s (%d tasks)', title_text, scene.num_tasks), 'Interpreter', 'none');
    apply_publication_style(ax);
end

function plot_tables(ax, scene)
    cfg = scene.scene_cfg;
    for t = 1:size(cfg.table_positions, 1)
        cx = cfg.table_positions(t, 1);
        cy = cfg.table_positions(t, 2);
        theta = cfg.table_orientations(min(t, end));
        R = rot2(theta);
        L = cfg.table.length;
        W = cfg.table.width;
        corners = [-L/2, -W/2; L/2, -W/2; L/2, W/2; -L/2, W/2]';
        world = R * corners;
        world(1,:) = world(1,:) + cx;
        world(2,:) = world(2,:) + cy;
        patch(ax, world(1,:), world(2,:), [0.88 0.88 0.88], ...
            'EdgeColor', [0.32 0.32 0.32], 'LineWidth', 0.9);
    end
end
