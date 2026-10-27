function fig = visualize_planning_overlay(scene, plan, stats, vis_cfg, varargin)
%VISUALIZE_PLANNING_OVERLAY Show docks, topology, and stored planner waypoints.

    if nargin < 4 || isempty(vis_cfg), vis_cfg = visualization_config(); end
    opts = parse_opts(varargin{:});
    fig = visualize_complex_scene(scene, vis_cfg, 'title', opts.title, 'filename', '');
    ax = gca(fig); hold(ax, 'on');
    all_poses = [scene.start_pose; plan.selected_poses];
    if vis_cfg.show_sparse_graph && isstruct(stats) && isfield(stats, 'sparse_graph') && ...
            ~isempty(stats.sparse_graph)
        draw_sparse_graph_connectivity(ax, all_poses, stats.sparse_graph.adjacency);
    end
    if vis_cfg.show_selected_docks
        draw_selected_docks(ax, plan.selected_poses);
    end
    if vis_cfg.show_final_route && isstruct(stats) && isfield(stats, 'global_path') && ...
            size(stats.global_path, 1) >= 2 && size(stats.global_path, 2) >= 2
        draw_collision_free_route(ax, stats.global_path);
    end
    title(ax, opts.title, 'FontSize', vis_cfg.font_size + 2, 'Interpreter', 'none');
    if vis_cfg.show_legend, build_scene_legend(ax); end
    save_visualization_figure(fig, vis_cfg, opts.filename);
end

function opts = parse_opts(varargin)
    opts = struct('title', 'Static Planning Overlay', 'filename', '');
    for i = 1:2:numel(varargin)
        opts.(varargin{i}) = varargin{i + 1};
    end
end
