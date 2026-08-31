function records = exp_compare_hierarchical_strategies()
%EXP_COMPARE_HIERARCHICAL_STRATEGIES Fair single-scene comparison of strategies.

    project_root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(project_root, 'pipeline'));
    init_paths(project_root);
    cd(project_root);

    [robot_cfg, ~, planner_cfg, scene, task_table_map, regions] = stage1_build_scene();
    planner_cfg.runtime.disable_visualization = true;
    planner_cfg.runtime.disable_video = true;
    planner_cfg.benchmark.enable = false;
    planner_cfg.visualization.show_scene = false;
    planner_cfg.visualization.show_docking = false;
    planner_cfg.visualization.show_global_path = false;
    planner_cfg.strategy.common.route_shortlist_size = min(15, planner_cfg.strategy.common.route_shortlist_size);

    candidate_library = build_candidate_library(regions, scene, robot_cfg, planner_cfg);
    strategies = {'bottom_up', 'top_down', 'bidirectional'};
    records = struct([]);
    outputs = struct([]);

    for si = 1:numel(strategies)
        cfg_i = planner_cfg;
        cfg_i.strategy.name = strategies{si};
        timing = struct('scene_time_s', 0, ...
            'candidate_generation_time_s', candidate_library.total_generation_time_s);
        t_strategy = tic;
        plan = run_planning_strategy(candidate_library, regions, scene, robot_cfg, cfg_i);
        timing.docking_time_s = toc(t_strategy);
        if isfield(plan, 'timing')
            timing.strategy_total_time_s = plan.timing.strategy_total_time_s;
            timing.region_order_time_s = plan.timing.region_order_time_s;
            timing.local_selection_time_s = plan.timing.local_selection_time_s;
            timing.route_refinement_time_s = plan.timing.route_refinement_time_s;
        end
        scene_i = scene;
        [scene_i, global_path, all_poses, visit_order, executed_tasks, total_path_len, stats] = ...
            stage3_global_planning(scene_i, plan.selected_poses, plan.task_sequence_summary, ...
            task_table_map, robot_cfg, cfg_i, plan.pose_info_list, timing, plan);
        plan.final_visit_order = visit_order;
        plan.visit_order = visit_order;
        plan.route_order = stats.route_order;
        stats.plan = plan;
        record = build_result_record(scene_i, plan.selected_poses, executed_tasks, ...
            global_path, total_path_len, stats, cfg_i, plan.pose_info_list, []);
        record.strategy = strategies{si};
        record.scene_id = candidate_library.scene_signature;
        record.status = 'OK';
        record.error_message = '';
        records = append_record(records, record); %#ok<AGROW>
        outputs(si).strategy = strategies{si}; %#ok<AGROW>
        outputs(si).plan = plan;
        outputs(si).path = global_path;
        outputs(si).all_poses = all_poses;
        outputs(si).visit_order = visit_order;
        outputs(si).stats = stats;
        outputs(si).record = record;
    end

    ensure_result_dirs();
    save(fullfile('results', 'raw', 'strategy_comparison_records.mat'), ...
        'records', 'outputs', 'candidate_library');
    write_strategy_summary(records, fullfile('results', 'tables', 'strategy_comparison_summary.csv'));
    write_strategy_time_breakdown(records, fullfile('results', 'tables', 'strategy_time_breakdown.csv'));
    plan_differences = write_strategy_plan_differences(outputs, records, planner_cfg, ...
        fullfile('results', 'tables', 'strategy_plan_differences.csv')); %#ok<NASGU>
    plot_strategy_route_comparison(scene, outputs, fullfile('results', 'figures', 'fig_strategy_route_comparison'));
    plot_strategy_tradeoff(records, fullfile('results', 'figures', 'fig_strategy_tradeoff'));
    plot_strategy_time_breakdown(records, fullfile('results', 'figures', 'fig_strategy_time_breakdown'));
    plot_strategy_plan_differences(plan_differences, fullfile('results', 'figures', 'fig_strategy_plan_differences'));
end

function out = append_record(records, record)
    if isempty(records)
        out = record;
    else
        out = [records; record]; %#ok<AGROW>
    end
end

function ensure_result_dirs()
    dirs = {fullfile('results', 'raw'), fullfile('results', 'tables'), fullfile('results', 'figures')};
    for i = 1:numel(dirs)
        if ~isfolder(dirs{i}), mkdir(dirs{i}); end
    end
end

function write_strategy_summary(records, output_file)
    rows = struct([]);
    for i = 1:numel(records)
        rows(i).strategy = records(i).strategy; %#ok<AGROW>
        rows(i).scene_id = records(i).scene_id;
        rows(i).seed = records(i).seed;
        rows(i).difficulty = records(i).difficulty;
        rows(i).num_tasks = records(i).num_tasks;
        rows(i).predicted_completion_rate = records(i).predicted_completion_rate;
        rows(i).verified_completion_rate = records(i).verified_completion_rate;
        rows(i).uncovered_tasks = records(i).uncovered_tasks;
        rows(i).num_docking_poses = records(i).num_docking_poses;
        rows(i).task_path_length_m = records(i).task_path_length_m;
        rows(i).return_path_length_m = records(i).return_path_length_m;
        rows(i).total_path_length_m = records(i).total_path_length_m;
        rows(i).num_turns = records(i).num_turns;
        rows(i).planning_time_s = records(i).planning_time_total;
        rows(i).candidate_generation_time = records(i).candidate_generation_time_s;
        rows(i).region_order_time = records(i).region_order_time_s;
        rows(i).local_selection_time = records(i).local_selection_time_s;
        rows(i).global_path_time = records(i).time_global;
        rows(i).estimated_execution_time_s = records(i).estimated_execution_time_s;
        rows(i).estimated_total_time_s = records(i).estimated_total_time_s;
        rows(i).coverage_repair_count = records(i).coverage_repair_count;
        rows(i).accepted_iterations = records(i).accepted_iterations;
        rows(i).num_pose_replacements = records(i).num_pose_replacements;
        rows(i).num_removed_docks = records(i).num_removed_docks;
        rows(i).num_route_changes = records(i).num_route_changes;
        rows(i).strategy_objective_value = records(i).strategy_objective_value;
        rows(i).status = records(i).status;
        rows(i).error_message = records(i).error_message;
    end
    writetable(struct2table(rows), output_file);
end

function write_strategy_time_breakdown(records, output_file)
    rows = struct([]);
    for i = 1:numel(records)
        rows(i).strategy = records(i).strategy; %#ok<AGROW>
        rows(i).task_navigation_time_s = records(i).task_navigation_time_s;
        rows(i).return_navigation_time_s = records(i).return_navigation_time_s;
        rows(i).navigation_time_s = records(i).navigation_time_s;
        rows(i).docking_time_s = records(i).docking_time_s;
        rows(i).manipulation_time_s = records(i).manipulation_time_s;
        rows(i).arm_transition_time_s = records(i).arm_transition_time_s;
        rows(i).side_switch_time_s = records(i).side_switch_time_s;
        rows(i).turning_time_s = records(i).turning_time_s;
        rows(i).lift_time_s = records(i).lift_time_s;
        rows(i).replanning_time_s = records(i).replanning_time_s;
        rows(i).mission_total_time_s = records(i).mission_total_time_s;
        rows(i).total_estimated_time_s = records(i).estimated_execution_time_s;
        rows(i).num_tasks = records(i).num_tasks;
        rows(i).num_docks = records(i).num_docking_poses;
        rows(i).num_side_switches = records(i).num_side_switches;
        rows(i).num_arm_switches = records(i).num_arm_switches;
        rows(i).total_abs_yaw_change_rad = records(i).total_abs_yaw_change_rad;
    end
    writetable(struct2table(rows), output_file);
end

function comparisons = write_strategy_plan_differences(outputs, records, planner_cfg, output_file)
    comparisons = struct([]);
    ref_idx = find(strcmp({outputs.strategy}, 'bottom_up'), 1);
    if isempty(ref_idx)
        warning('exp_compare_hierarchical_strategies:no_bottom_up', ...
            'No bottom_up reference found; skipping plan-difference table.');
        return;
    end
    ref_plan = outputs(ref_idx).plan;
    ref_record = records(ref_idx);
    for i = 1:numel(outputs)
        cmp = compare_strategy_plans(ref_plan, outputs(i).plan, ref_record, records(i), planner_cfg);
        cmp.coverage_repair_count = records(i).coverage_repair_count;
        cmp.accepted_bidirectional_moves = records(i).accepted_iterations;
        cmp.num_pose_replacements = records(i).num_pose_replacements;
        cmp.num_removed_docks = records(i).num_removed_docks;
        cmp.num_route_changes = records(i).num_route_changes;
        comparisons = append_record(comparisons, cmp); %#ok<AGROW>
    end
    writetable(struct2table(comparisons), output_file);
end

function plot_strategy_route_comparison(scene, outputs, output_base)
    h = figure('Visible', 'off', 'Position', [80, 80, 1200, 400]);
    for i = 1:numel(outputs)
        subplot(1, numel(outputs), i);
        draw_scene(scene, h);
        hold on;
        path = outputs(i).path;
        if ~isempty(path)
            plot(path(:, 1), path(:, 2), 'LineWidth', 1.5);
        end
        poses = outputs(i).plan.selected_poses;
        plot(poses(:, 1), poses(:, 2), 'ko', 'MarkerFaceColor', 'y', 'MarkerSize', 4);
        title(strrep(outputs(i).strategy, '_', '-'));
        axis equal;
    end
    saveas(h, [output_base, '.png']);
    saveas(h, [output_base, '.pdf']);
    close(h);
end

function plot_strategy_tradeoff(records, output_base)
    h = figure('Visible', 'off', 'Position', [80, 80, 900, 500]);
    names = string({records.strategy});
    values = [ ...
        [records.verified_completion_rate]; ...
        [records.num_docking_poses]; ...
        [records.total_path_length]; ...
        [records.planning_time_total]; ...
        [records.estimated_execution_time_s] ...
    ]';
    bar(values);
    set(gca, 'XTickLabel', names);
    legend({'completion %', 'docks', 'path m', 'planning s', 'exec est s'}, 'Location', 'best');
    grid on;
    title('Hierarchical strategy tradeoff');
    saveas(h, [output_base, '.png']);
    saveas(h, [output_base, '.pdf']);
    close(h);
end

function plot_strategy_time_breakdown(records, output_base)
    h = figure('Visible', 'off', 'Position', [80, 80, 900, 520]);
    names = string({records.strategy});
    values = [[records.navigation_time_s]', [records.docking_time_s]', ...
        [records.manipulation_time_s]', [records.arm_transition_time_s]', ...
        [records.side_switch_time_s]', [records.turning_time_s]'];
    bar(values, 'stacked');
    set(gca, 'XTickLabel', names, 'TickLabelInterpreter', 'none');
    ylabel('Estimated execution time (s)', 'Interpreter', 'none');
    legend({'navigation', 'docking', 'manipulation', 'arm transition', ...
        'side switch', 'turning'}, 'Location', 'best');
    grid on;
    title('Strategy time breakdown', 'Interpreter', 'none');
    saveas(h, [output_base, '.png']);
    saveas(h, [output_base, '.pdf']);
    close(h);
end

function plot_strategy_plan_differences(comparisons, output_base)
    if isempty(comparisons)
        return;
    end
    h = figure('Visible', 'off', 'Position', [80, 80, 950, 520]);
    names = string({comparisons.strategy});
    values = [[comparisons.pose_difference_count]', ...
        [comparisons.task_assignment_difference_count]', ...
        [comparisons.route_edge_difference_count]', ...
        [comparisons.coverage_repair_count]', ...
        [comparisons.accepted_bidirectional_moves]'];
    bar(values);
    set(gca, 'XTickLabel', names, 'TickLabelInterpreter', 'none');
    ylabel('Count', 'Interpreter', 'none');
    legend({'pose diff', 'task assignment diff', 'route edge diff', ...
        'coverage repair', 'accepted bidir moves'}, 'Location', 'best');
    grid on;
    title('Plan differences relative to Bottom-Up', 'Interpreter', 'none');
    saveas(h, [output_base, '.png']);
    saveas(h, [output_base, '.pdf']);
    close(h);
end
