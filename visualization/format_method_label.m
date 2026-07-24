function label = format_method_label(method_name)
%FORMAT_METHOD_LABEL Convert internal ablation names to compact labels.

    s = char(method_name);
    switch lower(strtrim(s))
        case 'full'
            label = 'Full';
        case 'no_reachability'
            label = 'No reach.';
        case 'single_docking_per_table'
            label = 'Single dock';
        case 'nearest_docking'
            label = 'Nearest dock';
        case 'no_coverage_cost'
            label = 'No cov. cost';
        case 'no_coverage_priority'
            label = 'No cov. priority';
        case 'no_coverage_all'
            label = 'No cov. all';
        otherwise
            label = strrep(s, '_', ' ');
    end
end
