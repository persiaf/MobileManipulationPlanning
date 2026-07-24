function records = exp_strategy_random_batch(quick_mode)
%EXP_STRATEGY_RANDOM_BATCH Paired random comparison scaffold.

    if nargin < 1
        quick_mode = true;
    end
    records = exp_compare_hierarchical_strategies();
    if quick_mode
        save(fullfile('results', 'raw', 'strategy_random_batch_partial.mat'), 'records');
    else
        save(fullfile('results', 'raw', 'strategy_random_batch_records.mat'), 'records');
    end
end
