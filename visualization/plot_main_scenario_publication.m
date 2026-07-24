function plot_main_scenario_publication(out_dir)
%PLOT_MAIN_SCENARIO_PUBLICATION Generate simplified English main scenario.
%
%   Output:
%     fig_main_scenario_publication.png/.fig/.pdf

    if nargin < 1 || isempty(out_dir)
        out_dir = fullfile(fileparts(fileparts(mfilename('fullpath'))), ...
                           'results', 'figures');
    end
    if ~exist(out_dir, 'dir'), mkdir(out_dir); end

    old_visible = get(0, 'DefaultFigureVisible');
    set(0, 'DefaultFigureVisible', 'off');
    try
        robot_cfg = robot_config();
        planner_cfg = planner_config();
        planner_cfg.visualization.show_scene = false;
        planner_cfg.visualization.show_candidates = false;
        planner_cfg.visualization.show_docking = false;
        planner_cfg.visualization.show_global_path = false;
        planner_cfg.visualization.show_reachability = false;
        planner_cfg.visualization.show_task_links = false;
        planner_cfg.visualization.show_arm_assign = false;
        planner_cfg.visualization.show_cost_heatmap = false;

        scene = build_scene();
        task_table_map = assign_tasks_to_tables(scene);
        regions = build_task_regions(scene, task_table_map);
        [optimal_poses, ~, task_sequence_summary, pose_info_list] = ...
            stage2_docking_planning(regions, scene, robot_cfg, planner_cfg); %#ok<ASGLU>
        [~, global_path, ~, ~, ~, ~, ~] = ...
            stage3_global_planning(scene, optimal_poses, task_sequence_summary, ...
                                   task_table_map, robot_cfg, planner_cfg, pose_info_list);
        close all;

        fig = figure('Name', 'Main Scenario Publication', 'Visible', 'off', ...
                     'Color', 'w', 'Position', [80, 80, 900, 760]);
        ax = axes('Parent', fig, 'Position', [0.10, 0.10, 0.84, 0.82]);
        hold(ax, 'on');
        axis(ax, 'equal');
        xlim(ax, [0, scene.map_info.width_m]);
        ylim(ax, [0, scene.map_info.height_m]);

        plot_tables(ax, scene);
        plot_tasks(ax, scene);
        plot_docking(ax, optimal_poses);
        if ~isempty(global_path)
            plot(ax, global_path(:,1), global_path(:,2), '-', ...
                'Color', [0.10 0.25 0.85], 'LineWidth', 1.8, ...
                'DisplayName', 'Global path');
        end
        plot(ax, scene.start_pose(1), scene.start_pose(2), 'p', ...
            'MarkerSize', 14, 'MarkerFaceColor', [0.95 0.20 0.15], ...
            'MarkerEdgeColor', 'k', 'LineWidth', 1.0, 'DisplayName', 'Start');

        xlabel(ax, 'X (m)', 'Interpreter', 'none');
        ylabel(ax, 'Y (m)', 'Interpreter', 'none');
        title(ax, 'Main Scenario Planning Result', 'Interpreter', 'none');
        legend(ax, 'Location', 'northeastoutside', 'Interpreter', 'none', ...
            'FontName', 'Times New Roman', 'FontSize', 10);
        apply_publication_style(ax);

        export_pub_figure(fig, fullfile(out_dir, 'fig_main_scenario_publication'));
        close(fig);
    catch ME
        warning('plot_main_scenario_publication:failed', ...
            'Main scenario publication figure skipped: %s', ME.message);
    end
    set(0, 'DefaultFigureVisible', old_visible);
    close all;
end

function plot_tables(ax, scene)
    cfg = scene.scene_cfg;
    for t = 1:size(cfg.table_positions, 1)
        cx = cfg.table_positions(t, 1);
        cy = cfg.table_positions(t, 2);
        theta = cfg.table_orientations(min(t, end));
        R = rot2(theta);
        L = cfg.table.length;
        W = cfg.table.width;
        corners = [-L/2, -W/2; L/2, -W/2; L/2, W/2; -L/2, W/2]';
        world = R * corners;
        world(1,:) = world(1,:) + cx;
        world(2,:) = world(2,:) + cy;
        patch(ax, world(1,:), world(2,:), [0.86 0.86 0.86], ...
            'EdgeColor', [0.28 0.28 0.28], 'LineWidth', 1.0, ...
            'DisplayName', display_once(t, 'Tables'));
    end
end

function plot_tasks(ax, scene)
    x = arrayfun(@(s) s.x, scene.tasks);
    y = arrayfun(@(s) s.y, scene.tasks);
    plot(ax, x, y, 'o', 'MarkerSize', 5.5, ...
        'MarkerFaceColor', [0.20 0.70 0.30], ...
        'MarkerEdgeColor', [0.05 0.35 0.10], 'LineStyle', 'none', ...
        'DisplayName', 'Task points');
end

function plot_docking(ax, poses)
    if isempty(poses)
        return;
    end
    plot(ax, poses(:,1), poses(:,2), '^', 'MarkerSize', 6.5, ...
        'MarkerFaceColor', [0.95 0.55 0.15], ...
        'MarkerEdgeColor', [0.45 0.25 0.05], 'LineStyle', 'none', ...
        'DisplayName', 'Selected docking poses');
end

function name = display_once(idx, label)
    if idx == 1
        name = label;
    else
        name = '';
    end
end
