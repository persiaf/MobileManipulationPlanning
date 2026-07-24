function plot_hard_ablation_comparison(csv_file, out_dir)
%PLOT_HARD_ABLATION_COMPARISON Draw hard-mixed ablation comparison.
%
%   Output:
%     fig_hard_ablation_comparison.png/.fig/.pdf

    if nargin < 2 || isempty(out_dir)
        out_dir = fullfile(fileparts(fileparts(mfilename('fullpath'))), ...
                           'results', 'figures');
    end
    if ~exist(out_dir, 'dir'), mkdir(out_dir); end

    if exist(csv_file, 'file') ~= 2
        warning('plot_hard_ablation_comparison:missing_csv', ...
            'CSV not found, skip hard ablation figure: %s', csv_file);
        return;
    end

    T = readtable(csv_file, 'Delimiter', ',');
    required = {'method', 'completion_rate_mean', 'uncovered_tasks_mean', ...
                'num_docking_poses_mean', 'total_path_length_mean'};
    if ~check_required_columns(T, required, 'plot_hard_ablation_comparison')
        return;
    end

    labels = cell(height(T), 1);
    for i = 1:height(T)
        labels{i} = format_method_label(T.method{i});
    end
    y = 1:height(T);

    fig = figure('Name', 'Hard Ablation Comparison', ...
                 'Visible', 'off', 'Color', 'w', 'Position', [80, 80, 1080, 720]);
    tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    horizontal_panel(nexttile, y, labels, T.completion_rate_mean, ...
        optional_col(T, 'completion_rate_std'), 'Completion rate (%)', ...
        '(a) Completion rate', [0.20 0.45 0.75], 108);

    horizontal_panel(nexttile, y, labels, T.uncovered_tasks_mean, ...
        optional_col(T, 'uncovered_tasks_std'), 'Uncovered tasks', ...
        '(b) Uncovered tasks', [0.86 0.34 0.28], []);

    horizontal_panel(nexttile, y, labels, T.num_docking_poses_mean, ...
        optional_col(T, 'num_docking_poses_std'), 'Docking poses', ...
        '(c) Docking poses', [0.40 0.66 0.38], []);

    horizontal_panel(nexttile, y, labels, T.total_path_length_mean, ...
        optional_col(T, 'total_path_length_std'), 'Total path length (m)', ...
        '(d) Total path length', [0.78 0.50 0.18], []);

    export_pub_figure(fig, fullfile(out_dir, 'fig_hard_ablation_comparison'));
    close(fig);
end

function horizontal_panel(ax, y, labels, values, err, x_label, title_text, color, fixed_xmax)
    barh(ax, y, values, 0.62, 'FaceColor', color, 'EdgeColor', 'none');
    hold(ax, 'on');
    if any(err ~= 0)
        draw_horizontal_error(ax, values, y, err);
    end
    set(ax, 'YTick', y, 'YTickLabel', labels, 'YDir', 'reverse');
    if isempty(fixed_xmax)
        xmax = max(values + err);
        if xmax <= 0, xmax = 1; end
        xlim(ax, [0, xmax * 1.18]);
    else
        xlim(ax, [0, fixed_xmax]);
    end
    xlabel(ax, x_label, 'Interpreter', 'none');
    title(ax, title_text, 'Interpreter', 'none');
    apply_publication_style(ax);
end

function draw_horizontal_error(ax, values, y, err)
    cap = 0.10;
    for i = 1:numel(values)
        if err(i) <= 0
            continue;
        end
        x1 = values(i) - err(i);
        x2 = values(i) + err(i);
        plot(ax, [x1, x2], [y(i), y(i)], 'k-', 'LineWidth', 1.0);
        plot(ax, [x1, x1], [y(i) - cap, y(i) + cap], 'k-', 'LineWidth', 1.0);
        plot(ax, [x2, x2], [y(i) - cap, y(i) + cap], 'k-', 'LineWidth', 1.0);
    end
end

function values = optional_col(T, col_name)
    if any(strcmp(T.Properties.VariableNames, col_name))
        values = T.(col_name);
    else
        values = zeros(height(T), 1);
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
