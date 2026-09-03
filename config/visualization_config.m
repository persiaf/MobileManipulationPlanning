function cfg = visualization_config()
%VISUALIZATION_CONFIG Unified options for complex scene figures.

    cfg.show_map = true;
    cfg.show_regions = true;
    cfg.show_region_labels = true;
    cfg.show_tables = true;
    cfg.show_table_orientation = true;
    cfg.show_tasks = true;
    cfg.show_infeasible_tasks = true;
    cfg.show_candidates = false;
    cfg.candidate_stride = 20;
    cfg.show_selected_docks = true;
    cfg.show_sparse_graph = true;
    cfg.show_final_route = true;
    cfg.show_static_obstacles = true;
    cfg.show_start = true;
    cfg.show_legend = true;
    cfg.show_grid = true;
    cfg.save_png = true;
    cfg.save_fig = false;
    cfg.output_dir = fullfile('results', 'figures', 'complex_scene');
    cfg.figure_visible = 'off';
    cfg.dpi = 300;
    cfg.font_size = 11;
end
