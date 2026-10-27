function fig = visualize_complex_scene(scene, vis_cfg, varargin)
%VISUALIZE_COMPLEX_SCENE Unified scene-only visualizer for V1/V2/V3 profiles.

    if nargin < 2 || isempty(vis_cfg), vis_cfg = visualization_config(); end
    opts = parse_opts(varargin{:});
    fig = figure('Name', opts.title, 'Color', 'w', 'Visible', vis_cfg.figure_visible, ...
        'Position', [80, 80, 1100, 900]);
    ax = axes(fig); hold(ax, 'on');
    setup_axes(ax, scene, vis_cfg);
    if vis_cfg.show_map
        draw_map(ax, scene);
    end
    if vis_cfg.show_static_obstacles
        draw_static_obstacles(ax, scene);
    end
    if vis_cfg.show_tables
        draw_tables(ax, scene, vis_cfg);
    end
    if vis_cfg.show_tasks
        draw_tasks(ax, scene);
    end
    if vis_cfg.show_start
        draw_start(ax, scene);
    end
    title(ax, opts.title, 'FontSize', vis_cfg.font_size + 2, 'Interpreter', 'none');
    maybe_legend(ax, vis_cfg);
    save_visualization_figure(fig, vis_cfg, opts.filename);
end

function opts = parse_opts(varargin)
    opts = struct('title', 'Complex Scene', 'filename', '');
    for i = 1:2:numel(varargin)
        opts.(varargin{i}) = varargin{i + 1};
    end
end

function setup_axes(ax, scene, vis_cfg)
    axis(ax, 'equal');
    xlim(ax, [0, scene.map_info.width_m]);
    ylim(ax, [0, scene.map_info.height_m]);
    xlabel(ax, 'X (m)', 'FontSize', vis_cfg.font_size);
    ylabel(ax, 'Y (m)', 'FontSize', vis_cfg.font_size);
    if vis_cfg.show_grid, grid(ax, 'on'); else, grid(ax, 'off'); end
    box(ax, 'on');
end

function draw_map(ax, scene)
    [ii, jj] = find(scene.grid_map > 0);
    if isempty(ii), return; end
    res = scene.map_info.resolution;
    xy = zeros(numel(ii), 2);
    for k = 1:numel(ii)
        [xy(k, 1), xy(k, 2)] = grid2world(ii(k), jj(k), res, scene.map_info.height_m);
    end
    plot(ax, xy(:, 1), xy(:, 2), '.', 'Color', [0.88 0.88 0.88], ...
        'MarkerSize', 2, 'HandleVisibility', 'off');
end

function draw_static_obstacles(ax, scene)
    if ~isfield(scene, 'static_obstacles') || isempty(scene.static_obstacles), return; end
    for i = 1:numel(scene.static_obstacles)
        obs = scene.static_obstacles(i);
        c = oriented_rectangle_corners(obs.center, obs.theta, obs.size(1), obs.size(2));
        patch(ax, c(:, 1), c(:, 2), [0.25 0.25 0.25], ...
            'FaceAlpha', 0.65, 'EdgeColor', [0.05 0.05 0.05], ...
            'LineWidth', 1.0, 'DisplayName', legend_once(i, 'Static obstacle'));
    end
end

function draw_tables(ax, scene, vis_cfg)
    cfg = scene.scene_cfg;
    regions = ones(size(cfg.table_positions, 1), 1);
    if isfield(cfg, 'table_region_id')
        regions = cfg.table_region_id(:);
    end
    colors = lines(max(numel(unique(regions)), 1));
    unique_regions = unique(regions, 'stable');
    for t = 1:size(cfg.table_positions, 1)
        rid = regions(t);
        color = colors(find(unique_regions == rid, 1), :);
        corners = oriented_rectangle_corners(cfg.table_positions(t, :), cfg.table_orientations(t), ...
            cfg.table.length, cfg.table.width);
        patch(ax, corners(:, 1), corners(:, 2), color, 'FaceAlpha', 0.32, ...
            'EdgeColor', color * 0.55, 'LineWidth', 1.2, ...
            'DisplayName', legend_once(t, 'Table'));
        if vis_cfg.show_table_orientation
            draw_heading(ax, cfg.table_positions(t, :), cfg.table_orientations(t), 0.35, color);
        end
    end
    if vis_cfg.show_region_labels
        for r = reshape(unique_regions, 1, [])
            pts = cfg.table_positions(regions == r, :);
            label_pos = mean(pts, 1);
            text(ax, label_pos(1), label_pos(2), sprintf('R%d', r), ...
                'FontWeight', 'bold', 'Color', [0.05 0.05 0.05], ...
                'HorizontalAlignment', 'center', 'BackgroundColor', 'w', ...
                'Margin', 1, 'FontSize', vis_cfg.font_size);
        end
    end
end

function draw_tasks(ax, scene)
    x = [scene.tasks.x]';
    y = [scene.tasks.y]';
    plot(ax, x, y, 'o', 'MarkerSize', 4, 'MarkerFaceColor', [0.1 0.55 0.2], ...
        'MarkerEdgeColor', 'none', 'DisplayName', 'Task');
end

function draw_start(ax, scene)
    plot(ax, scene.start_pose(1), scene.start_pose(2), 'p', 'MarkerSize', 16, ...
        'MarkerFaceColor', [0.9 0.1 0.1], 'MarkerEdgeColor', 'k', ...
        'DisplayName', 'Start');
end

function draw_heading(ax, center, theta, len, color)
    quiver(ax, center(1), center(2), len * cos(theta), len * sin(theta), 0, ...
        'Color', color * 0.6, 'LineWidth', 1.0, 'MaxHeadSize', 1.5, ...
        'HandleVisibility', 'off');
end

function name = legend_once(i, name)
    if i > 1, name = ''; end
end

function maybe_legend(ax, vis_cfg)
    if vis_cfg.show_legend
        build_scene_legend(ax);
    end
end
