function plot_ablation_bar(csv_file, out_dir)
%PLOT_ABLATION_BAR Draw SCI-ready ablation summary figures from CSV.
%
%   Outputs:
%     fig_ablation_completion_uncovered.png/.fig/.pdf
%     fig_ablation_path_docking.png/.fig/.pdf

    if nargin < 2 || isempty(out_dir)
        out_dir = fullfile(fileparts(fileparts(mfilename('fullpath'))), ...
                           'results', 'figures');
    end
    if ~exist(out_dir, 'dir'), mkdir(out_dir); end

    if exist(csv_file, 'file') ~= 2
        warning('plot_ablation_bar:missing_csv', 'CSV not found, skip ablation figures: %s', csv_file);
        return;
    end

    T = readtable(csv_file, 'Delimiter', ',');
    required = {'method', 'completion_rate', 'uncovered_tasks', ...
                'total_path_length', 'num_docking_poses'};
    if ~check_required_columns(T, required, 'plot_ablation_bar')
        return;
    end

    labels = cell(height(T), 1);
    for i = 1:height(T)
        labels{i} = format_method_label(T.method{i});
    end
    y = 1:height(T);

    completion = T.completion_rate;
    uncovered = T.uncovered_tasks;
    path_len = T.total_path_length;
    docks = T.num_docking_poses;

    fig1 = figure('Name', 'Ablation Completion and Uncovered', ...
                  'Visible', 'off', 'Color', 'w', 'Position', [80, 80, 1050, 560]);
    tiledlayout(fig1, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    ax1 = nexttile;
    barh(ax1, y, completion, 0.62, 'FaceColor', [0.20 0.45 0.75]);
    set(ax1, 'YTick', y, 'YTickLabel', labels, 'YDir', 'reverse');
    xlim(ax1, [0, 108]);
    xlabel(ax1, 'Completion rate (%)', 'Interpreter', 'none');
    title(ax1, '(a) Completion rate', 'Interpreter', 'none');
    for i = 1:numel(y)
        text(ax1, min(completion(i) + 1.4, 104), y(i), sprintf('%.1f%%', completion(i)), ...
            'VerticalAlignment', 'middle', 'FontName', 'Times New Roman', ...
            'FontSize', 10, 'Interpreter', 'none');
    end
    apply_publication_style(ax1);

    ax2 = nexttile;
    barh(ax2, y, uncovered, 0.62, 'FaceColor', [0.86 0.34 0.28]);
    set(ax2, 'YTick', y, 'YTickLabel', labels, 'YDir', 'reverse');
    xmax = max([max(uncovered) * 1.25, 1]);
    xlim(ax2, [0, xmax]);
    xlabel(ax2, 'Uncovered tasks', 'Interpreter', 'none');
    title(ax2, '(b) Uncovered tasks', 'Interpreter', 'none');
    for i = 1:numel(y)
        text(ax2, uncovered(i) + xmax * 0.025, y(i), sprintf('%d', round(uncovered(i))), ...
            'VerticalAlignment', 'middle', 'FontName', 'Times New Roman', ...
            'FontSize', 10, 'Interpreter', 'none');
    end
    apply_publication_style(ax2);

    export_pub_figure(fig1, fullfile(out_dir, 'fig_ablation_completion_uncovered'));
    close(fig1);

    fig2 = figure('Name', 'Ablation Path and Docking', ...
                  'Visible', 'off', 'Color', 'w', 'Position', [80, 80, 1120, 520]);
    tiledlayout(fig2, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    ax3 = nexttile;
    bar(ax3, 1:numel(labels), path_len, 0.62, 'FaceColor', [0.20 0.45 0.75]);
    set(ax3, 'XTick', 1:numel(labels), 'XTickLabel', labels);
    xtickangle(ax3, 30);
    ylabel(ax3, 'Total path length (m)', 'Interpreter', 'none');
    title(ax3, '(a) Total path length', 'Interpreter', 'none');
    ylim(ax3, [0, max(path_len) * 1.18]);
    apply_publication_style(ax3);

    ax4 = nexttile;
    bar(ax4, 1:numel(labels), docks, 0.62, 'FaceColor', [0.40 0.66 0.38]);
    set(ax4, 'XTick', 1:numel(labels), 'XTickLabel', labels);
    xtickangle(ax4, 30);
    ylabel(ax4, 'Number of docking poses', 'Interpreter', 'none');
    title(ax4, '(b) Number of docking poses', 'Interpreter', 'none');
    ylim(ax4, [0, max(docks) * 1.18]);
    apply_publication_style(ax4);

    export_pub_figure(fig2, fullfile(out_dir, 'fig_ablation_path_docking'));
    close(fig2);
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
