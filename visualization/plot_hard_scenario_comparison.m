function plot_hard_scenario_comparison(csv_file, out_dir)
%PLOT_HARD_SCENARIO_COMPARISON Draw 2x2 hard scenario summary.
%
%   Output:
%     fig_hard_scenario_summary.png/.fig/.pdf

    if nargin < 2 || isempty(out_dir)
        out_dir = fullfile(fileparts(fileparts(mfilename('fullpath'))), ...
                           'results', 'figures');
    end
    if ~exist(out_dir, 'dir'), mkdir(out_dir); end

    if exist(csv_file, 'file') ~= 2
        warning('plot_hard_scenario_comparison:missing_csv', ...
            'CSV not found, skip hard scenario figure: %s', csv_file);
        return;
    end

    T = readtable(csv_file, 'Delimiter', ',');
    required = {'difficulty', 'completion_rate_mean', 'completion_rate_std', ...
                'uncovered_tasks_mean', 'uncovered_tasks_std', ...
                'total_path_length_mean', 'total_path_length_std', ...
                'num_docking_poses_mean', 'num_docking_poses_std'};
    if ~check_required_columns(T, required, 'plot_hard_scenario_comparison')
        return;
    end

    order = {'normal', 'hard_corner', 'hard_two_side', 'hard_reach_boundary', 'hard_mixed'};
    labels = {'Normal', 'Corner', 'Two-side', 'Reach-boundary', 'Mixed-hard'};
    T = reorder_table(T, order);
    if isempty(T)
        warning('plot_hard_scenario_comparison:no_matching_rows', ...
            'No expected difficulty rows found in %s', csv_file);
        return;
    end
    labels = labels(1:height(T));
    x = 1:height(T);

    fig = figure('Name', 'Hard Scenario Summary', 'Visible', 'off', ...
                 'Color', 'w', 'Position', [80, 80, 1180, 760]);
    tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    ax1 = nexttile;
    bar_with_error(ax1, x, T.completion_rate_mean, T.completion_rate_std, ...
        [0.20 0.45 0.75]);
    ylim(ax1, [max(0, min(T.completion_rate_mean - T.completion_rate_std) - 4), 106]);
    ylabel(ax1, 'Completion rate (%)', 'Interpreter', 'none');
    title(ax1, '(a) Completion rate', 'Interpreter', 'none');
    if all(abs(T.completion_rate_mean - 100) < 1e-6)
        text(ax1, mean(x), 102.5, 'All settings reached 100% completion', ...
            'HorizontalAlignment', 'center', 'FontName', 'Times New Roman', ...
            'FontSize', 10, 'Interpreter', 'none');
    end
    style_categorical_axis(ax1, x, labels);

    ax2 = nexttile;
    bar_with_error(ax2, x, T.uncovered_tasks_mean, T.uncovered_tasks_std, ...
        [0.86 0.34 0.28]);
    ylabel(ax2, 'Uncovered tasks', 'Interpreter', 'none');
    title(ax2, '(b) Uncovered tasks', 'Interpreter', 'none');
    ylim(ax2, [0, max([1, max(T.uncovered_tasks_mean + T.uncovered_tasks_std) * 1.25])]);
    style_categorical_axis(ax2, x, labels);

    ax3 = nexttile;
    bar_with_error(ax3, x, T.total_path_length_mean, T.total_path_length_std, ...
        [0.40 0.66 0.38]);
    ylabel(ax3, 'Total path length (m)', 'Interpreter', 'none');
    title(ax3, '(c) Total path length', 'Interpreter', 'none');
    ylim(ax3, [0, max(T.total_path_length_mean + T.total_path_length_std) * 1.18]);
    style_categorical_axis(ax3, x, labels);

    ax4 = nexttile;
    bar_with_error(ax4, x, T.num_docking_poses_mean, T.num_docking_poses_std, ...
        [0.78 0.50 0.18]);
    ylabel(ax4, 'Docking poses', 'Interpreter', 'none');
    title(ax4, '(d) Docking poses', 'Interpreter', 'none');
    ylim(ax4, [0, max(T.num_docking_poses_mean + T.num_docking_poses_std) * 1.18]);
    style_categorical_axis(ax4, x, labels);

    export_pub_figure(fig, fullfile(out_dir, 'fig_hard_scenario_summary'));
    close(fig);
end

function bar_with_error(ax, x, y, e, color)
    bar(ax, x, y, 0.62, 'FaceColor', color, 'EdgeColor', 'none');
    hold(ax, 'on');
    errorbar(ax, x, y, e, 'k.', 'LineWidth', 1.1, 'CapSize', 8);
end

function style_categorical_axis(ax, x, labels)
    set(ax, 'XTick', x, 'XTickLabel', labels);
    xtickangle(ax, 25);
    apply_publication_style(ax);
end

function T_out = reorder_table(T, order)
    T_out = T([],:);
    raw = cellstr(string(T.difficulty));
    for i = 1:numel(order)
        idx = find(strcmpi(raw, order{i}), 1);
        if ~isempty(idx)
            T_out = [T_out; T(idx,:)]; %#ok<AGROW>
        end
    end
end

function ok = check_required_columns(T, required, caller_name)
    names = T.Properties.VariableNames;
    missing = required(~ismember(required, names));
    ok = isempty(missing);
    if ~ok
        warning([caller_name, ':missing_columns'], '%s missing required column(s): %s', ...
            caller_name, strjoin(missing, ', '));
    end
end
