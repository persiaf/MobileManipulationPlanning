function plan = plan_top_down(candidate_library, regions, scene, robot_cfg, planner_cfg)
%PLAN_TOP_DOWN Region-order-first baseline with route-aware local selection.

    validate_candidate_library(candidate_library, regions);
    t_strategy = tic;
    cache = initialize_path_cost_cache(scene, planner_cfg);

    t_order = tic;
    region_anchors = build_region_anchors(candidate_library, planner_cfg);
    path_cost_audit('set_source', 'TOP_DOWN_REGION_ORDER');
    [region_order, region_cost_matrix, cache] = plan_region_order(scene, region_anchors, planner_cfg, cache);
    timing.region_order_time_s = toc(t_order);

    t_local = tic;
    path_cost_audit('set_source', 'TOP_DOWN_ANCHOR');
    [selected_poses, pose_to_region_map, task_sequence_summary, pose_info_list, diagnostics, cache] = ...
        select_route_aware_multi_docks(candidate_library, regions, scene, robot_cfg, ...
        planner_cfg, region_order, region_anchors, cache);
    timing.local_selection_time_s = toc(t_local);
    timing.candidate_generation_time_s = candidate_library.total_generation_time_s;
    timing.strategy_total_time_s = toc(t_strategy);
    diagnostics.region_cost_matrix = region_cost_matrix;
    diagnostics.cache_hits = cache.hits;
    diagnostics.cache_misses = cache.misses;

    preferred_visit_order = build_preferred_visit_order(pose_to_region_map, region_order);
    plan = build_unified_plan('top_down', selected_poses, pose_to_region_map, ...
        task_sequence_summary, pose_info_list, region_order, preferred_visit_order, ...
        candidate_library, scene, planner_cfg, timing, diagnostics);
    validate_plan_consistency(plan, scene);
end

function preferred = build_preferred_visit_order(pose_to_region_map, region_order)
    preferred = 1;
    for ri = region_order(:)'
        pose_idx = find(pose_to_region_map(:)' == ri);
        preferred = [preferred, pose_idx + 1]; %#ok<AGROW>
    end
end
