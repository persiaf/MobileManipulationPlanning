function fig = visualize_cafeteria_scene(scene, vis_cfg, varargin)
%VISUALIZE_CAFETERIA_SCENE Draw fixed tables, A4 tasks, aisle, and plan data.

    if nargin < 2 || isempty(vis_cfg), vis_cfg = visualization_config(); end
    opts = parse_options(varargin{:});
    fig = figure('Name', opts.title, 'Color', 'w', 'Visible', vis_cfg.figure_visible, ...
        'Position', [80, 80, 1050, 820]);
    ax = axes(fig); hold(ax, 'on'); axis(ax, 'equal'); box(ax, 'on'); grid(ax, 'on');
    xlim(ax, [0, scene.map_info.width_m]); ylim(ax, [0, scene.map_info.height_m]);
    xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)');

    aisle = scene.scene_cfg.cafeteria.main_aisle_x;
    patch(ax, aisle([1 2 2 1]), [0.1 0.1 scene.map_info.height_m-0.1 scene.map_info.height_m-0.1], ...
        [0.78 0.88 0.94], 'FaceAlpha', 0.20, 'EdgeColor', [0.35 0.55 0.65], ...
        'LineStyle', '--', 'DisplayName', 'Main aisle');
    text(ax, mean(aisle), scene.map_info.height_m - 0.28, 'Main aisle', ...
        'HorizontalAlignment', 'center', 'Color', [0.15 0.35 0.45]);

    cfg = scene.scene_cfg;
    for t = 1:cfg.num_tables
        corners = oriented_rectangle_corners(cfg.table_positions(t, :), ...
            cfg.table_orientations(t), cfg.table.length, cfg.table.width);
        patch(ax, corners(:, 1), corners(:, 2), [0.55 0.38 0.22], ...
            'FaceAlpha', 0.72, 'EdgeColor', [0.25 0.16 0.08], 'LineWidth', 1.2, ...
            'DisplayName', legend_once(t, 'Table'));
        text(ax, cfg.table_positions(t, 1), cfg.table_positions(t, 2), sprintf('T%d', t), ...
            'HorizontalAlignment', 'center', 'Color', 'w', 'FontWeight', 'bold');
    end

    marker_size = cfg.task_marker.size;
    for i = 1:scene.num_tasks
        task = scene.tasks(i);
        corners = oriented_rectangle_corners([task.x, task.y], ...
            cfg.table_orientations(task.table_id), marker_size(1), marker_size(2));
        color = [0.12 0.62 0.28];
        if ~isempty(opts.operability) && ~opts.operability(i).model_feasible
            color = [0.85 0.20 0.16];
        end
        patch(ax, corners(:, 1), corners(:, 2), color, 'FaceAlpha', 0.90, ...
            'EdgeColor', [0.05 0.18 0.08], 'LineWidth', 1.0, ...
            'DisplayName', legend_once(i, 'A4 task'));
        text(ax, task.x, task.y, sprintf('A%d', i), ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
            'Color', 'w', 'FontSize', 7, 'FontWeight', 'bold');
    end

    plot(ax, scene.start_pose(1), scene.start_pose(2), 'p', 'MarkerSize', 15, ...
        'MarkerFaceColor', [0.92 0.15 0.12], 'MarkerEdgeColor', 'k', 'DisplayName', 'Start');
    if ~isempty(opts.plan) && isfield(opts.plan, 'selected_poses') && ~isempty(opts.plan.selected_poses)
        draw_selected_docks(ax, opts.plan.selected_poses);
        for di = 1:size(opts.plan.selected_poses,1)
            p = opts.plan.selected_poses(di,:);
            text(ax,p(1),p(2)+0.22,sprintf('D%d',di),'Color',[0.45 0.20 0], ...
                'FontWeight','bold','HorizontalAlignment','center');
            s=opts.plan.task_sequence_summary{di};
            for ti=s.covered_tasks
                task=scene.tasks(ti);
                plot(ax,[p(1),task.x],[p(2),task.y],':','Color',[0.45 0.35 0.15], ...
                    'HandleVisibility','off');
            end
        end
    end
    if ~isempty(opts.path) && size(opts.path, 1) >= 2
        draw_collision_free_route(ax, opts.path);
    end
    title(ax, opts.title, 'Interpreter', 'none');
    if vis_cfg.show_legend, build_scene_legend(ax); end
    save_visualization_figure(fig, vis_cfg, opts.filename);
end

function opts = parse_options(varargin)
    opts = struct('title', 'CAFETERIA 1x3 / 2x3', 'filename', '', ...
        'operability', [], 'plan', [], 'path', []);
    for i = 1:2:numel(varargin), opts.(varargin{i}) = varargin{i + 1}; end
end

function name = legend_once(i, name)
    if i > 1, name = ''; end
end
