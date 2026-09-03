function figs = visualize_dynamic_replan(planning_state_before, change_event, local_result, vis_cfg, varargin)
%VISUALIZE_DYNAMIC_REPLAN Show before and after local replanning context.

    if nargin < 4 || isempty(vis_cfg), vis_cfg = visualization_config(); end
    opts = parse_opts(varargin{:});
    [changed_state, event] = apply_dynamic_event(planning_state_before, ...
        change_event.type, change_event.event_id);
    impact = analyze_change_impact(planning_state_before, event);

    before_stats = planning_state_before.stats;
    if ~isfield(before_stats, 'global_path')
        before_stats.global_path = [];
    end
    figs.before = visualize_planning_overlay(planning_state_before.scene, ...
        planning_state_before.plan, before_stats, vis_cfg, ...
        'title', [opts.title ' — Before Change'], 'filename', [opts.prefix '_before']);

    figs.after = visualize_planning_overlay(changed_state.scene, ...
        planning_state_before.plan, before_stats, vis_cfg, ...
        'title', [opts.title ' — After Local Replan'], 'filename', '');
    ax = gca(figs.after); hold(ax, 'on');
    draw_event(ax, event);
    draw_invalid_edges(ax, planning_state_before.all_poses, impact.invalid_sparse_edges);
    draw_affected_regions(ax, changed_state.scene, impact.affected_regions);
    text(ax, 0.02, 0.98, sprintf('Local A*: %d | fallback: %d', ...
        getfield_default(local_result, 'local_astar_calls', 0), ...
        getfield_default(local_result, 'fallback', false)), ...
        'Units', 'normalized', 'VerticalAlignment', 'top', ...
        'BackgroundColor', 'w', 'Margin', 2, 'FontWeight', 'bold');
    save_figure_if_requested(figs.after, vis_cfg, [opts.prefix '_after']);
end

function draw_event(ax, event)
    if all(isfinite(event.center))
        rectangle(ax, 'Position', [event.center(1)-event.radius_m, ...
            event.center(2)-event.radius_m, 2*event.radius_m, 2*event.radius_m], ...
            'Curvature', [1 1], 'EdgeColor', [0.85 0.05 0.05], ...
            'LineWidth', 2.0, 'LineStyle', '-');
        plot(ax, NaN, NaN, '-', 'Color', [0.85 0.05 0.05], ...
            'LineWidth', 2.0, 'DisplayName', 'new local obstacle');
    end
end

function draw_invalid_edges(ax, poses, edges)
    for i = 1:size(edges, 1)
        p = poses(edges(i, :), :);
        plot(ax, p(:, 1), p(:, 2), '-', 'Color', [0.9 0.0 0.0], ...
            'LineWidth', 2.0, 'DisplayName', legend_once(i, 'invalidated edge'));
    end
end

function draw_affected_regions(ax, scene, affected_regions)
    if isempty(affected_regions), return; end
    cfg = scene.scene_cfg;
    for i = 1:numel(affected_regions)
        rid = affected_regions(i);
        pts = cfg.table_positions(cfg.table_region_id == rid, :);
        if isempty(pts), continue; end
        c = mean(pts, 1);
        plot(ax, c(1), c(2), 'o', 'MarkerSize', 22, 'LineWidth', 2.0, ...
            'MarkerEdgeColor', [0.9 0.05 0.05], 'MarkerFaceColor', 'none', ...
            'DisplayName', legend_once(i, 'affected region'));
    end
end

function opts = parse_opts(varargin)
    opts = struct('title', 'Dynamic Local Replanning', 'prefix', 'V3_dynamic');
    for i = 1:2:numel(varargin)
        opts.(varargin{i}) = varargin{i + 1};
    end
end

function v = getfield_default(s, name, default_value)
    if isstruct(s) && isfield(s, name)
        v = s.(name);
    else
        v = default_value;
    end
end

function name = legend_once(i, name)
    if i > 1, name = ''; end
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
