function outputs = render_experiment_outputs(scene, robot_cfg, plan, stats, vis_cfg, varargin)
%RENDER_EXPERIMENT_OUTPUTS Unified IEEE-style renderer for Stages 1-3.
%   outputs = render_experiment_outputs(scene, robot_cfg, plan, stats,
%   vis_cfg, 'candidate_library', library, 'replanning', replanning)
%   writes the canonical scene/candidate/docking/planning/replanning PNGs.

    if nargin < 5 || isempty(vis_cfg), vis_cfg = visualization_config(); end
    if nargin < 2 || isempty(robot_cfg), robot_cfg = robot_config(); end %#ok<NASGU>
    if nargin < 4 || isempty(stats), stats = struct(); end
    if nargin < 3 || isempty(plan), plan = struct(); end
    opts = parse_options(varargin{:});
    vis_cfg = normalize_config(vis_cfg);
    if ~isfolder(vis_cfg.output_dir), mkdir(vis_cfg.output_dir); end
    outputs = struct('scene','', 'candidate','', 'docking','', ...
        'planning','', 'replanning','');

    if opts.stage >= 1 && ~isempty(scene)
        fig = visualize_complex_scene(scene, vis_cfg, 'title', 'Scene', ...
            'filename', vis_cfg.output_names.scene);
        style_axes(fig, vis_cfg);
        save_visualization_figure(fig, vis_cfg, vis_cfg.output_names.scene);
        outputs.scene = fullfile(vis_cfg.output_dir, [vis_cfg.output_names.scene '.png']);
        close_if_requested(fig, vis_cfg);
    end

    if opts.stage >= 1 && ~isempty(opts.candidate_library)
        fig = visualize_complex_scene(scene, vis_cfg, 'title', 'Docking Candidates', ...
            'filename', '');
        ax = findobj(fig, 'Type', 'axes'); ax = ax(1); hold(ax, 'on');
        draw_candidate_library(ax, opts.candidate_library, vis_cfg);
        if vis_cfg.show_legend, legend(ax, 'Location', 'eastoutside'); end
        save_visualization_figure(fig, vis_cfg, vis_cfg.output_names.candidate);
        style_axes(fig, vis_cfg);
        save_visualization_figure(fig, vis_cfg, vis_cfg.output_names.candidate);
        outputs.candidate = fullfile(vis_cfg.output_dir, [vis_cfg.output_names.candidate '.png']);
        close_if_requested(fig, vis_cfg);
    end

    if opts.stage >= 2 && isfield(plan, 'selected_poses') && ~isempty(plan.selected_poses)
        fig = visualize_planning_overlay(scene, plan, struct(), vis_cfg, ...
            'title', 'Selected Docking Poses', 'filename', vis_cfg.output_names.docking);
        style_axes(fig, vis_cfg);
        save_visualization_figure(fig, vis_cfg, vis_cfg.output_names.docking);
        outputs.docking = fullfile(vis_cfg.output_dir, [vis_cfg.output_names.docking '.png']);
        close_if_requested(fig, vis_cfg);
    end

    if opts.stage >= 3 && has_path(stats)
        stats.global_path = stats.global_path;
        fig = visualize_planning_overlay(scene, plan, stats, vis_cfg, ...
            'title', 'Global Planning', 'filename', vis_cfg.output_names.planning);
        style_axes(fig, vis_cfg);
        save_visualization_figure(fig, vis_cfg, vis_cfg.output_names.planning);
        outputs.planning = fullfile(vis_cfg.output_dir, [vis_cfg.output_names.planning '.png']);
        close_if_requested(fig, vis_cfg);
    end

    if ~isempty(opts.replanning) && isstruct(opts.replanning) && ...
            isfield(opts.replanning, 'planning_state_before')
        r = opts.replanning;
        local_result = get_field(r, 'local_result', struct());
        event = get_field(r, 'change_event', struct('type','UNKNOWN','event_id',1));
        figs = visualize_dynamic_replan(r.planning_state_before, event, ...
            local_result, vis_cfg, 'title', 'Local Replanning', 'prefix', vis_cfg.output_names.replanning);
        if isfield(figs, 'after'), style_axes(figs.after, vis_cfg); end
        if isfield(figs, 'after')
            save_visualization_figure(figs.after, vis_cfg, vis_cfg.output_names.replanning);
        end
        outputs.replanning = fullfile(vis_cfg.output_dir, [vis_cfg.output_names.replanning '.png']);
        if isfield(figs, 'before'), close_if_requested(figs.before, vis_cfg); end
        if isfield(figs, 'after'), close_if_requested(figs.after, vis_cfg); end
    elseif opts.stage >= 3 && has_path(stats)
        fig = visualize_planning_overlay(scene, plan, stats, vis_cfg, ...
            'title', 'Replanning State', 'filename', vis_cfg.output_names.replanning);
        style_axes(fig, vis_cfg);
        save_visualization_figure(fig, vis_cfg, vis_cfg.output_names.replanning);
        outputs.replanning = fullfile(vis_cfg.output_dir, [vis_cfg.output_names.replanning '.png']);
        close_if_requested(fig, vis_cfg);
    end
end

function opts = parse_options(varargin)
    opts = struct('stage', 3, 'candidate_library', [], 'replanning', []);
    for i = 1:2:numel(varargin)
        if i + 1 <= numel(varargin), opts.(varargin{i}) = varargin{i + 1}; end
    end
end

function cfg = normalize_config(cfg)
    canonical = visualization_config();
    cfg = merge_struct_recursive(canonical, cfg);
end

function draw_candidate_library(ax, library, cfg)
    if ~isfield(library, 'regions'), return; end
    stride = max(1, round(cfg.candidate_stride));
    first = true;
    for r = 1:numel(library.regions)
        candidates = library.regions(r).candidates;
        for k = 1:stride:numel(candidates)
            p = candidates(k).pose;
            if first
                plot(ax, p(1), p(2), '.', 'Color', cfg.style.candidate, ...
                    'MarkerSize', 8, 'DisplayName', 'Candidate');
                first = false;
            else
                plot(ax, p(1), p(2), '.', 'Color', cfg.style.candidate, ...
                    'MarkerSize', 8, 'HandleVisibility', 'off');
            end
        end
    end
end

function tf = has_path(stats)
    tf = isstruct(stats) && isfield(stats, 'global_path') && ...
        size(stats.global_path, 1) >= 2;
end

function value = get_field(s, name, fallback)
    if isstruct(s) && isfield(s, name), value = s.(name); else, value = fallback; end
end

function style_axes(fig, cfg)
    ax = findobj(fig, 'Type', 'axes');
    for i = 1:numel(ax)
        set(ax(i), 'FontName', cfg.style.font_name, 'FontSize', cfg.style.font_size, ...
            'LineWidth', cfg.style.axis_line_width, 'Box', 'on', 'Color', cfg.style.background);
    end
end

function close_if_requested(fig, cfg)
    if isfield(cfg, 'close_figures_after_save') && cfg.close_figures_after_save
        close(fig);
    end
end
