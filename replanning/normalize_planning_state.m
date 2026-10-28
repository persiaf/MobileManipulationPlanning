function state = normalize_planning_state(state)
%NORMALIZE_PLANNING_STATE Provide read-only legacy aliases for RC2 state.
%   The original rc2_plan is retained untouched.  Aliases are reconstructed
%   only when an older consumer requires plan/stats/all_poses fields.

    if isfield(state, 'robot_cfg') && isstruct(state.robot_cfg) && ...
            isfield(state.robot_cfg, 'chassis')
        state.robot_cfg = validate_main_robot_config(state.robot_cfg);
        if isfield(state, 'planner_cfg') && isstruct(state.planner_cfg)
            if isfield(state.planner_cfg, 'robot_cfg')
                validate_main_robot_config(state.planner_cfg.robot_cfg);
            end
            state.planner_cfg.robot_cfg = state.robot_cfg;
        end
    end
    if ~isfield(state, 'rc2_plan') || ~isstruct(state.rc2_plan)
        return;
    end
    rc2 = state.rc2_plan;
    if ~isfield(state, 'plan') || ~isstruct(state.plan)
        state.plan = struct();
        state.plan.status = field_or(rc2, 'status', 'UNKNOWN');
        state.plan.reason = field_or(rc2, 'reason', '');
        state.plan.selected_poses = field_or(rc2, 'selected_docks', []);
        state.plan.task_sequence_summary = field_or(rc2, 'task_assignment', {});
        state.plan.covered_tasks_predicted = field_or(rc2, 'covered_tasks', []);
        state.plan.uncovered_tasks_predicted = field_or(rc2, 'uncovered_tasks', []);
        state.plan.pose_info_list = state.plan.task_sequence_summary;
        state.plan.candidate_library = field_or(rc2, 'candidate_library', []);
        state.plan.reachability_summary = field_or(rc2, 'reachability_summary', struct());
        state.plan.robust_reachability_summary = field_or(rc2, 'robust_reachability_summary', struct());
        state.plan.reachability_mode = field_or(rc2, 'reachability_mode', 'nominal');
        state.plan.pose_to_region_map = assignment_regions(state.plan.task_sequence_summary, ...
            size(state.plan.selected_poses, 1));
    end
    if ~isfield(state, 'all_poses') && isfield(state, 'scene') && isfield(state.scene, 'start_pose')
        state.all_poses = [state.scene.start_pose; state.plan.selected_poses];
    end
    if ~isfield(state, 'stats') || ~isstruct(state.stats), state.stats = struct(); end
    if ~isfield(state.stats, 'route_order')
        state.stats.route_order = field_or(rc2, 'route_order', field_or(rc2, 'visit_order', []));
    end
    if ~isfield(state.stats, 'sparse_graph')
        state.stats.sparse_graph = field_or(rc2, 'sparse_graph', field_or(state, 'sparse_graph', struct()));
    end
    if ~isfield(state.stats, 'path_len')
        state.stats.path_len = compute_path_length(field_or(rc2, 'global_path', []));
    end
    if ~isfield(state.stats, 'global_path')
        state.stats.global_path = field_or(rc2, 'global_path', []);
    end
    if ~isfield(state.stats, 'segment_stats')
        state.stats.segment_stats = field_or(rc2, 'path_segments', struct([]));
    end
    if ~isfield(state, 'candidate_library')
        state.candidate_library = field_or(rc2, 'candidate_library', []);
    end
end

function regions = assignment_regions(assignments, n)
    regions = ones(n, 1);
    for k = 1:min(n, numel(assignments))
        if isstruct(assignments{k}) && isfield(assignments{k}, 'region_id')
            regions(k) = assignments{k}.region_id;
        elseif isstruct(assignments{k}) && isfield(assignments{k}, 'table_id')
            regions(k) = assignments{k}.table_id;
        end
    end
end

function value = field_or(s, name, fallback)
    if isstruct(s) && isfield(s, name), value = s.(name); else, value = fallback; end
end
