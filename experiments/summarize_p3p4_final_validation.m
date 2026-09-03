function summary = summarize_p3p4_final_validation(input_csv, output_csv)
%SUMMARIZE_P3P4_FINAL_VALIDATION Summarize resource-bounded validation rows.

    root = fileparts(fileparts(mfilename('fullpath')));
    init_paths(root);
    if nargin < 1 || isempty(input_csv)
        input_csv = fullfile(root, 'results', 'tables', 'p3p4_final_validation_raw.csv');
    end
    if nargin < 2 || isempty(output_csv)
        output_csv = fullfile(root, 'results', 'tables', 'p3p4_final_validation_summary.csv');
    end
    raw = readtable(input_csv, 'TextType', 'string');
    summary = groupsummary(raw, {'scale','strategy','mode','routing_mode','k'}, ...
        {'mean'}, {'task_count','feasible_tasks','selected_docks','compression_ratio', ...
        'dense_pair_count','sparse_edge_count','actual_astar_calls','coverage', ...
        'feasible_task_coverage','verified_completion','total_path_length', ...
        'T_path_cost','T_stage3','T_total','cache_hit_count','expensive_planner_calls'});
    writetable(summary, output_csv);
end
