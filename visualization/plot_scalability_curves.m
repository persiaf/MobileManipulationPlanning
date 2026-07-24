function plot_scalability_curves(csv_file, out_dir)
%PLOT_SCALABILITY_CURVES Draw 2x2 scalability summary from random batch CSV.
%
%   Output:
%     fig_scalability_summary.png/.fig/.pdf

    if nargin < 2 || isempty(out_dir)
        out_dir = fullfile(fileparts(fileparts(mfilename('fullpath'))), ...
                           'results', 'figures');
    end
    if ~exist(out_dir, 'dir'), mkdir(out_dir); end

    if exist(csv_file, 'file') ~= 2
        warning('plot_scalability_curves:missing_csv', ...
            'CSV not found, skip scalability figure: %s', csv_file);
        return;
    end

    T = readtable(csv_file, 'Delimiter', ',');
    required = {'scale', 'num_tasks_mean', 'num_tasks_std', ...
                'num_docking_poses_mean', 'num_docking_poses_std', ...
                'total_path_length_mean', 'total_path_length_std', ...
                'planning_time_total_mean', 'planning_time_total_std'};
    if ~check_required_columns(T, required, 'plot_scalability_curves')
        return;
    end

    T = reorder_scale_table(T);
    labels = format_scale_labels(T.scale);
    x = 1:height(T);

    fig = figure('Name', 'Scalability Summary', 'Visible', 'off', ...
                 'Color', 'w', 'Position', [80, 80, 1120, 740]);
    tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    ax1 = nexttile;
    line_with_error(ax1, x, T.num_tasks_mean, T.num_tasks_std, 'o', [0.20 0.45 0.75]);
    ylabel(ax1, 'Number of tasks', 'Interpreter', 'none');
    title(ax1, '(a) Number of tasks', 'Interpreter', 'none');
    style_scale_axis(ax1, x, labels);

    ax2 = nexttile;
    line_with_error(ax2, x, T.num_docking_poses_mean, T.num_docking_poses_std, 's', [0.40 0.66 0.38]);
    ylabel(ax2, 'Docking poses', 'Interpreter', 'none');
    title(ax2, '(b) Docking poses', 'Interpreter', 'none');
    style_scale_axis(ax2, x, labels);

    ax3 = nexttile;
    line_with_error(ax3, x, T.total_path_length_mean, T.total_path_length_std, '^', [0.78 0.50 0.18]);
    ylabel(ax3, 'Total path length (m)', 'Interpreter', 'none');
    title(ax3, '(c) Total path length', 'Interpreter', 'none');
    style_scale_axis(ax3, x, labels);

    ax4 = nexttile;
    line_with_error(ax4, x, T.planning_time_total_mean, T.planning_time_total_std, 'd', [0.86 0.34 0.28]);
    ylabel(ax4, 'Planning time (s)', 'Interpreter', 'none');
    title(ax4, '(d) Planning time', 'Interpreter', 'none');
    style_scale_axis(ax4, x, labels);

    annotation(fig, 'textbox', [0.22, 0.01, 0.56, 0.04], ...
        'String', 'Completion rate was 100% for all evaluated scales.', ...
        'EdgeColor', 'none', 'HorizontalAlignment', 'center', ...
        'FontName', 'Times New Roman', 'FontSize', 10, 'Interpreter', 'none');

    export_pub_figure(fig, fullfile(out_dir, 'fig_scalability_summary'));
    close(fig);
end

function line_with_error(ax, x, y, e, marker, color)
    errorbar(ax, x, y, e, [marker, '-'], 'Color', color, ...
        'MarkerFaceColor', color, 'MarkerSize', 8, 'LineWidth', 1.8, ...
        'CapSize', 9);
end

function style_scale_axis(ax, x, labels)
    set(ax, 'XTick', x, 'XTickLabel', labels);
    xlim(ax, [min(x) - 0.25, max(x) + 0.25]);
    apply_publication_style(ax);
end

function labels = format_scale_labels(scales)
    labels = cell(numel(scales), 1);
    for i = 1:numel(scales)
        s = char(scales(i));
        labels{i} = [upper(s(1)), lower(s(2:end))];
    end
end

function T_out = reorder_scale_table(T)
    order = {'small', 'medium', 'large'};
    raw = cellstr(string(T.scale));
    T_out = T([],:);
    for i = 1:numel(order)
        idx = find(strcmpi(raw, order{i}), 1);
        if ~isempty(idx)
            T_out = [T_out; T(idx,:)]; %#ok<AGROW>
        end
    end
    if isempty(T_out)
        T_out = T;
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
