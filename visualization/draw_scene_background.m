function draw_scene_background(scene)
    res = scene.map_info.resolution;
    h_m = scene.map_info.height_m;
    plot_grid(scene.grid_map, res, h_m);

    scene_cfg = scene.scene_cfg;
    for t = 1:size(scene_cfg.table_positions, 1)
        cx = scene_cfg.table_positions(t, 1);
        cy = scene_cfg.table_positions(t, 2);
        L  = scene_cfg.table.length;
        W  = scene_cfg.table.width;
        th = scene_cfg.table_orientations(t);
        cw = oriented_rectangle_corners([cx, cy], th, L, W)';
        cw = round(cw / res) * res;

        plot([cw(1,:) cw(1,1)], [cw(2,:) cw(2,1)], ...
             '--', 'Color', [0.9 0.15 0.15], 'LineWidth', 2);
        plot(cw(1,:), cw(2,:), 's', 'MarkerSize', 8, ...
             'MarkerFaceColor', [0.8 0.15 0.15], ...
             'MarkerEdgeColor', 'k', 'LineWidth', 1);
        text(cx, cy, sprintf('Table %d', t), 'HorizontalAlignment', 'center', ...
             'FontSize', 8, 'FontWeight', 'bold', 'Color', 'w', ...
             'BackgroundColor', [0.5 0.05 0.05 0.7], 'Margin', 1);
    end
end
