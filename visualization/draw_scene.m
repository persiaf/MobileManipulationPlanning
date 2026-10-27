function draw_scene(scene, fig_handle)
    if nargin < 2
        fig_handle = figure('Name', 'Scene', 'Position', [80, 80, 1000, 780]);
    else
        figure(fig_handle);
        clf;
    end

    hold on;

    draw_scene_background(scene);

    xlim([0, scene.map_info.width_m]);
    ylim([0, scene.map_info.height_m]);
    xlabel('X (m)', 'FontSize', 11);
    ylabel('Y (m)', 'FontSize', 11);

    for ti = 1:scene.num_tasks
        if scene.tasks(ti).status == 0
            plot(scene.tasks(ti).x, scene.tasks(ti).y, 'o', ...
                 'MarkerSize', 10, 'MarkerEdgeColor', [0 0.4 0], ...
                 'MarkerFaceColor', [0.2 1 0.2], 'LineWidth', 1.2);
        else
            plot(scene.tasks(ti).x, scene.tasks(ti).y, 's', ...
                 'MarkerSize', 10, 'MarkerEdgeColor', [0.15 0.15 0.15], ...
                 'MarkerFaceColor', [0.45 0.45 0.45], 'LineWidth', 1.2);
        end
        task_label = ti;
        if isfield(scene, 'scene_cfg') && isfield(scene.scene_cfg, 'task_ids') && numel(scene.scene_cfg.task_ids) >= ti
            task_label = scene.scene_cfg.task_ids(ti);
        end
        text(scene.tasks(ti).x, scene.tasks(ti).y, sprintf('Task %d', task_label), ...
            'HorizontalAlignment','left','VerticalAlignment','bottom','FontSize',8, ...
            'Color',[0.05 0.25 0.05],'FontWeight','bold');
    end

    h_start = plot(scene.start_pose(1), scene.start_pose(2), 'p', ...
                   'MarkerSize', 18, 'MarkerEdgeColor', 'k', ...
                   'MarkerFaceColor', '#ff0000', 'LineWidth', 1.5);

    if scene.num_tasks > 0
        completed = sum([scene.tasks(:).status]);
        total = scene.num_tasks;
        scene_cfg = scene.scene_cfg;
        title(sprintf('Task Scene  |  Tables: %d  |  Tasks: %d/%d', ...
              size(scene_cfg.table_positions, 1), completed, total), ...
              'FontSize', 13);
    end

    hold off;
end
