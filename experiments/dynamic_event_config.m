function cfg = dynamic_event_config()
%DYNAMIC_EVENT_CONFIG Minimal deterministic P5-Fast experiment matrix.

    cfg.scale = 'L2';
    cfg.seeds = [2026, 2027];
    cfg.strategy = 'bottom_up';
    cfg.routing_mode = 'sparse_knn';
    cfg.sparse_k = 4;
    cfg.events = {'ADD_LOCAL_OBSTACLE', 'INVALIDATE_DOCK', 'ADD_TASKS'};
    cfg.full_replan_timeout_s = 300;
    cfg.raw_csv = fullfile('results', 'tables', 'dynamic_local_replanning_p5_raw.csv');
    cfg.summary_csv = fullfile('results', 'tables', 'dynamic_local_replanning_p5_summary.csv');
end
