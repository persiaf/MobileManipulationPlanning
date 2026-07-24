function records = exp_strategy_hard_scenarios()
%EXP_STRATEGY_HARD_SCENARIOS Hard-scenario comparison entry point.
% This scaffold reuses the current configured scene; scenario-specific task
% distributions should be set through scene_config before calling it.

    records = exp_compare_hierarchical_strategies();
    save(fullfile('results', 'raw', 'strategy_hard_scenario_records.mat'), 'records');
end
