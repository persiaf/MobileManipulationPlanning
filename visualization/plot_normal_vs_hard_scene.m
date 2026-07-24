function plot_normal_vs_hard_scene(out_dir, seed)
%PLOT_NORMAL_VS_HARD_SCENE normal random 与 hard_mixed 任务分布场景对比图。
%   生成 small 场景的 normal 和 hard_mixed 两个子图并排对比。
%
%   输出:
%     fig_normal_vs_hard_scene.png/.fig

    if nargin < 1 || isempty(out_dir)
        out_dir = fullfile(fileparts(fileparts(mfilename('fullpath'))), ...
                           'results', 'figures');
    end
    if nargin < 2 || isempty(seed)
        seed = 2026;
    end
    if ~exist(out_dir, 'dir'), mkdir(out_dir); end

    % ============ 构建两个场景 ============
    scale = 'small';

    % --- Normal ---
    try
        scene_cfg_n = create_scaled_scene_config(scale, seed);
        scene_cfg_n.task.difficulty = 'normal';
        scene_n = build_scene_from_cfg(scene_cfg_n);
    catch ME
        warning('plot_normal_vs_hard_scene: 无法构建 normal 场景: %s', ME.message);
        return;
    end

    % --- Hard Mixed ---
    try
        scene_cfg_h = create_scaled_scene_config(scale, seed);
        scene_cfg_h.task.difficulty = 'hard_mixed';
        scene_h = build_scene_from_cfg(scene_cfg_h);
    catch ME
        warning('plot_normal_vs_hard_scene: 无法构建 hard_mixed 场景: %s', ME.message);
        return;
    end

    % ============ 绘图 ============
    fig = figure('Name', 'Normal vs Hard Scene', 'Visible', 'off');
    set(fig, 'Position', [100, 100, 1200, 520]);

    fig_pos = fig.Position;
    tile_w = fig_pos(3) / 2;
    tile_h = fig_pos(4);

    % --- 左：Normal ---
    ax1 = axes('Parent', fig, 'Units', 'pixels', ...
               'Position', [50, 60, tile_w - 70, tile_h - 100]);
    hold(ax1, 'on');

    draw_scene_background(scene_n);
    xlim(ax1, [0, scene_n.map_info.width_m]);
    ylim(ax1, [0, scene_n.map_info.height_m]);
    xlabel(ax1, 'X (m)', 'FontSize', 12, 'FontName', 'Times New Roman');
    ylabel(ax1, 'Y (m)', 'FontSize', 12, 'FontName', 'Times New Roman');

    for ti = 1:scene_n.num_tasks
        plot(ax1, scene_n.tasks(ti).x, scene_n.tasks(ti).y, 'o', ...
             'MarkerSize', 10, 'MarkerEdgeColor', [0 0.4 0], ...
             'MarkerFaceColor', [0.2 1 0.2], 'LineWidth', 1.2);
    end
    plot(ax1, scene_n.start_pose(1), scene_n.start_pose(2), 'p', ...
         'MarkerSize', 18, 'MarkerEdgeColor', 'k', ...
         'MarkerFaceColor', '#ff0000', 'LineWidth', 1.5);
    title(ax1, sprintf('Normal Random  (Tables: %d, Tasks: %d)', ...
          size(scene_n.scene_cfg.table_positions, 1), scene_n.num_tasks), ...
          'FontSize', 13, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
    axis(ax1, 'equal');
    grid(ax1, 'on');
    box(ax1, 'on');

    % --- 右：Hard Mixed ---
    ax2 = axes('Parent', fig, 'Units', 'pixels', ...
               'Position', [tile_w + 20, 60, tile_w - 70, tile_h - 100]);
    hold(ax2, 'on');

    draw_scene_background(scene_h);
    xlim(ax2, [0, scene_h.map_info.width_m]);
    ylim(ax2, [0, scene_h.map_info.height_m]);
    xlabel(ax2, 'X (m)', 'FontSize', 12, 'FontName', 'Times New Roman');
    ylabel(ax2, 'Y (m)', 'FontSize', 12, 'FontName', 'Times New Roman');

    for ti = 1:scene_h.num_tasks
        plot(ax2, scene_h.tasks(ti).x, scene_h.tasks(ti).y, '^', ...
             'MarkerSize', 10, 'MarkerEdgeColor', [0.7 0.1 0.1], ...
             'MarkerFaceColor', [1 0.3 0.3], 'LineWidth', 1.2);
    end
    plot(ax2, scene_h.start_pose(1), scene_h.start_pose(2), 'p', ...
         'MarkerSize', 18, 'MarkerEdgeColor', 'k', ...
         'MarkerFaceColor', '#ff0000', 'LineWidth', 1.5);
    title(ax2, sprintf('Hard Mixed  (Tables: %d, Tasks: %d)', ...
          size(scene_h.scene_cfg.table_positions, 1), scene_h.num_tasks), ...
          'FontSize', 13, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
    axis(ax2, 'equal');
    grid(ax2, 'on');
    box(ax2, 'on');

    export_pub_figure(fig, fullfile(out_dir, 'fig_normal_vs_hard_scene'));
    close(fig);
end