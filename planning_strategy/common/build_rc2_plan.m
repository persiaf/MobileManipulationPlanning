function rc2_plan = build_rc2_plan(plan, visit_order, route_order, global_path, ...
        path_segments, sparse_graph, scene, robot_cfg, planner_cfg)
%BUILD_RC2_PLAN Freeze the RC2 planning output contract.
    rc2_plan = struct();
    rc2_plan.status = getfield_default(plan, 'status', 'UNKNOWN');
    rc2_plan.reason = getfield_default(plan, 'reason', '');
    rc2_plan.selected_docks = getfield_default(plan, 'selected_poses', []);
    rc2_plan.task_assignment = getfield_default(plan, 'task_sequence_summary', {});
    rc2_plan.covered_tasks = getfield_default(plan, 'covered_tasks_predicted', []);
    rc2_plan.uncovered_tasks = getfield_default(plan, 'uncovered_tasks_predicted', []);
    rc2_plan.visit_order = visit_order;
    rc2_plan.route_order = route_order;
    rc2_plan.global_path = global_path;
    rc2_plan.path_segments = path_segments;
    rc2_plan.candidate_library = getfield_default(plan, 'candidate_library', []);
    rc2_plan.reachability_summary = getfield_default(plan, 'reachability_summary', struct());
    rc2_plan.robust_reachability_summary = getfield_default(plan, ...
        'robust_reachability_summary', struct());
    rc2_plan.sparse_graph = sparse_graph;
    rc2_plan.objective = getfield_default(plan, 'objective', struct());
    rc2_plan.reachability_mode = getfield_default(plan, 'reachability_mode', 'nominal');
    rc2_plan.scene_signature = build_config_signature(scene);
    rc2_plan.robot_signature = build_config_signature(robot_cfg);
    rc2_plan.planner_signature = build_config_signature(planner_signature_payload(planner_cfg));
end

function payload = planner_signature_payload(cfg)
    payload = struct();
    names = {'global_planner','routing','footprint_aware_navigation','reachability', ...
        'docking_error_model','strategy','cost_weights'};
    for i = 1:numel(names)
        if isfield(cfg, names{i}), payload.(names{i}) = cfg.(names{i}); end
    end
    if isfield(cfg, 'robot_cfg') && isfield(cfg.robot_cfg, 'footprint')
        payload.robot_footprint = cfg.robot_cfg.footprint;
    end
end

function signature = build_config_signature(value)
    signature = build_struct_signature(value);
end

function value = getfield_default(s, name, default_value)
    if isstruct(s) && isfield(s, name), value = s.(name); else, value = default_value; end
end
