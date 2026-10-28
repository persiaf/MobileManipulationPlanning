function cfg = dynamic_event_config()
%DYNAMIC_EVENT_CONFIG Current actual-pair dynamic experiment defaults.

    cfg.scale = 'MAIN4';
    cfg.seeds = [2026, 2027, 2028];
    cfg.strategy = 'bottom_up';
    cfg.routing_mode = 'sparse_knn';
    cfg.sparse_k = 6;
    cfg.events = {'ADD_LOCAL_OBSTACLE', 'INVALIDATE_DOCK', 'ADD_TASKS'};
    cfg.reference_type = 'actual_current_code_replan';
    cfg.raw_csv = fullfile('results','repair_20261027','dynamic','dynamic_raw.csv');
    cfg.summary_csv = fullfile('results','repair_20261027','dynamic','dynamic_summary.csv');
end
