function fig = visualize_planning_overlay(scene, plan, stats, vis_cfg, varargin)
%VISUALIZE_PLANNING_OVERLAY Show selected docks, sparse graph, and final route.

    if nargin < 4 || isempty(vis_cfg), vis_cfg = visualization_config(); end
    opts = parse_opts(varargin{:});
    fig = visualize_complex_scene(scene, vis_cfg, 'title', opts.title, 'filename', '');
    ax = gca(fig); hold(ax, 'on');
    all_poses = [scene.start_pose; plan.selected_poses];
    if vis_cfg.show_sparse_graph && isstruct(stats) && isfield(stats, 'sparse_graph') && ...
            ~isempty(stats.sparse_graph)
        draw_sparse_graph(ax, all_poses, stats.sparse_graph.adjacency);
    end
    if vis_cfg.show_selected_docks
        draw_docks(ax, plan.selected_poses);
    end
    if vis_cfg.show_final_route && isstruct(stats) && isfield(stats, 'global_path') && ...
            size(stats.global_path, 1) >= 2 && size(stats.global_path, 2) >= 2
        plot(ax, stats.global_path(:, 1), stats.global_path(:, 2), '-', ...
            'Color', [0.0 0.2 0.9], 'LineWidth', 2.8, 'DisplayName', 'final route');
    end
    title(ax, opts.title, 'FontSize', vis_cfg.font_size + 2, 'Interpreter', 'none');
    if vis_cfg.show_legend, legend(ax, 'Location', 'eastoutside'); end
    save_figure_if_requested(fig, vis_cfg, opts.filename);
end

function draw_sparse_graph(ax, poses, adjacency)
    [ii, jj] = find(triu(adjacency, 1));
    for k = 1:numel(ii)
        plot(ax, poses([ii(k), jj(k)], 1), poses([ii(k), jj(k)], 2), '-', ...
            'Color', [0.45 0.45 0.45 0.35], 'LineWidth', 0.4, ...
            'HandleVisibility', 'off');
    end
    plot(ax, NaN, NaN, '-', 'Color', [0.45 0.45 0.45], ...
        'LineWidth', 0.8, 'DisplayName', 'sparse graph');
end

function draw_docks(ax, poses)
    plot(ax, poses(:, 1), poses(:, 2), '^', 'MarkerSize', 7, ...
        'MarkerFaceColor', [1.0 0.65 0.1], 'MarkerEdgeColor', [0.45 0.25 0], ...
        'DisplayName', 'selected dock');
    for i = 1:size(poses, 1)
        quiver(ax, poses(i, 1), poses(i, 2), 0.25*cos(poses(i, 3)), ...
            0.25*sin(poses(i, 3)), 0, 'Color', [0.55 0.25 0], ...
            'LineWidth', 0.8, 'MaxHeadSize', 1.0, 'HandleVisibility', 'off');
    end
end

function opts = parse_opts(varargin)
    opts = struct('title', 'Static Planning Overlay', 'filename', '');
    for i = 1:2:numel(varargin)
        opts.(varargin{i}) = varargin{i + 1};
    end
end

function save_figure_if_requested(fig, vis_cfg, filename)
    if isempty(filename), return; end
    out_dir = vis_cfg.output_dir;
    if ~isfolder(out_dir), mkdir(out_dir); end
    png_path = fullfile(out_dir, [filename '.png']);
    if vis_cfg.save_png
        if exist('exportgraphics', 'file') == 2
            exportgraphics(fig, png_path, 'Resolution', vis_cfg.dpi);
        else
            print(fig, png_path, '-dpng', sprintf('-r%d', vis_cfg.dpi));
        end
    end
    if vis_cfg.save_fig
        savefig(fig, fullfile(out_dir, [filename '.fig']));
    end
end
