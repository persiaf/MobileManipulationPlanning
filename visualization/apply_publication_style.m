function apply_publication_style(ax, opts)
%APPLY_PUBLICATION_STYLE Apply consistent SCI-style axes formatting.
%
%   apply_publication_style(gca)
%   apply_publication_style(gca, struct('FontSize', 11))

    if nargin < 1 || isempty(ax)
        ax = gca;
    end
    if nargin < 2
        opts = struct();
    end

    font_name = get_opt(opts, 'FontName', 'Times New Roman');
    font_size = get_opt(opts, 'FontSize', 11);
    line_width = get_opt(opts, 'LineWidth', 1.0);

    set(ax, ...
        'FontName', font_name, ...
        'FontSize', font_size, ...
        'LineWidth', line_width, ...
        'TickLabelInterpreter', 'none', ...
        'Box', 'on', ...
        'XGrid', 'on', ...
        'YGrid', 'on', ...
        'GridAlpha', 0.18);

    ax.Title.Interpreter = 'none';
    ax.XLabel.Interpreter = 'none';
    ax.YLabel.Interpreter = 'none';
end

function value = get_opt(opts, field_name, default_value)
    if isstruct(opts) && isfield(opts, field_name)
        value = opts.(field_name);
    else
        value = default_value;
    end
end
