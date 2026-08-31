function comparison = compare_strategy_plans(reference_plan, plan, reference_record, record, planner_cfg)
%COMPARE_STRATEGY_PLANS Compare one strategy plan against a reference plan.
% Pose matching uses configurable position/yaw tolerances from
% cfg.strategy.diagnostics. Counts are diagnostic only and do not modify plans.

    if nargin < 3, reference_record = struct(); end
    if nargin < 4, record = struct(); end
    if nargin < 5, planner_cfg = struct(); end
    [pos_tol, yaw_tol] = tolerances(planner_cfg);

    comparison.reference_strategy = safe_text(reference_plan, 'strategy_name', 'reference');
    comparison.strategy = safe_text(plan, 'strategy_name', 'strategy');
    comparison.region_order_hamming = hamming_count( ...
        safe_num_array(reference_plan, 'region_order'), safe_num_array(plan, 'region_order'));
    ref_order = normalize_plan_order(reference_plan);
    plan_order = normalize_plan_order(plan);
    comparison.visit_order_hamming = hamming_count(ref_order.dock_order, plan_order.dock_order);
    comparison.route_order_difference_count = comparison.visit_order_hamming;
    comparison.route_order_valid = ref_order.is_valid && plan_order.is_valid;
    comparison.route_order_error_message = join_errors(ref_order, plan_order);
    [match_count, diff_count] = pose_difference( ...
        safe_pose_array(reference_plan), safe_pose_array(plan), pos_tol, yaw_tol);
    comparison.pose_match_count = match_count;
    comparison.pose_difference_count = diff_count;
    comparison.task_assignment_difference_count = task_assignment_difference( ...
        safe_summary(reference_plan), safe_summary(plan));
    [directed_diff, undirected_diff, start_diff, return_diff] = route_edge_differences(ref_order, plan_order);
    comparison.directed_route_edge_difference_count = directed_diff;
    comparison.undirected_route_edge_difference_count = undirected_diff;
    comparison.start_edge_difference_count = start_diff;
    comparison.return_edge_difference_count = return_diff;
    comparison.route_edge_difference_count = undirected_diff;
    comparison.num_docks_difference = safe_num(plan, 'num_docking_poses') - ...
        safe_num(reference_plan, 'num_docking_poses');
    comparison.path_length_difference_m = safe_path_length_m(record) - ...
        safe_path_length_m(reference_record);
    comparison.execution_time_difference_s = safe_record_num(record, 'estimated_execution_time_s') - ...
        safe_record_num(reference_record, 'estimated_execution_time_s');
end

function normalized = normalize_plan_order(plan)
    num_docks = safe_num(plan, 'num_docking_poses');
    if num_docks == 0
        num_docks = size(safe_pose_array(plan), 1);
    end
    order = first_order_field(plan, {'final_visit_order', 'visit_order', ...
        'route_order', 'preferred_visit_order'});
    normalized = normalize_visit_order(order, num_docks, struct('node_indexing', 'auto'));
end

function order = first_order_field(plan, names)
    order = [];
    for i = 1:numel(names)
        if isstruct(plan) && isfield(plan, names{i}) && ~isempty(plan.(names{i}))
            order = plan.(names{i});
            return;
        end
    end
end

function [directed_diff, undirected_diff, start_diff, return_diff] = route_edge_differences(ref_order, plan_order)
    if ~ref_order.is_valid || ~plan_order.is_valid
        directed_diff = NaN;
        undirected_diff = NaN;
        start_diff = NaN;
        return_diff = NaN;
        return;
    end
    dock_opts = struct('include_start_edge', false, 'include_return_edge', false);
    full_opts = struct('include_start_edge', true, 'include_return_edge', true);
    ref_dock = build_route_edge_set(ref_order, dock_opts);
    plan_dock = build_route_edge_set(plan_order, dock_opts);
    ref_full = build_route_edge_set(ref_order, full_opts);
    plan_full = build_route_edge_set(plan_order, full_opts);
    directed_diff = size(setxor(ref_dock.directed_edges, plan_dock.directed_edges, 'rows'), 1);
    undirected_diff = size(setxor(ref_dock.undirected_edges, plan_dock.undirected_edges, 'rows'), 1);
    start_diff = size(setxor(ref_full.start_edges, plan_full.start_edges, 'rows'), 1);
    return_diff = size(setxor(ref_full.return_edges, plan_full.return_edges, 'rows'), 1);
end

function msg = join_errors(ref_order, plan_order)
    parts = {};
    if ~ref_order.is_valid
        parts{end + 1} = ['reference: ', ref_order.error_message]; %#ok<AGROW>
    end
    if ~plan_order.is_valid
        parts{end + 1} = ['plan: ', plan_order.error_message]; %#ok<AGROW>
    end
    msg = strjoin(parts, ' | ');
end

function [pos_tol, yaw_tol] = tolerances(planner_cfg)
    pos_tol = 0.05;
    yaw_tol = deg2rad(5);
    if isfield(planner_cfg, 'strategy') && isfield(planner_cfg.strategy, 'diagnostics')
        d = planner_cfg.strategy.diagnostics;
        if isfield(d, 'position_tolerance_m'), pos_tol = d.position_tolerance_m; end
        if isfield(d, 'yaw_tolerance_rad'), yaw_tol = d.yaw_tolerance_rad; end
    end
end

function n = hamming_count(a, b)
    m = max(numel(a), numel(b));
    a(end + 1:m) = NaN;
    b(end + 1:m) = NaN;
    n = sum(~isequaln_vec(a, b));
end

function tf = isequaln_vec(a, b)
    tf = (a == b) | (isnan(a) & isnan(b));
end

function [matches, differences] = pose_difference(a, b, pos_tol, yaw_tol)
    used = false(size(b, 1), 1);
    matches = 0;
    for i = 1:size(a, 1)
        for j = 1:size(b, 1)
            if used(j), continue; end
            if norm(a(i, 1:2) - b(j, 1:2)) <= pos_tol && ...
                    abs(wrapToPiLocal(a(i, 3) - b(j, 3))) <= yaw_tol
                used(j) = true;
                matches = matches + 1;
                break;
            end
        end
    end
    differences = max(size(a, 1), size(b, 1)) - matches;
end

function n = task_assignment_difference(a, b)
    m = max(numel(a), numel(b));
    n = 0;
    for i = 1:m
        ta = summary_tasks(a, i);
        tb = summary_tasks(b, i);
        if ~isequal(ta, tb)
            n = n + 1;
        end
    end
end

function tasks = summary_tasks(summary, idx)
    tasks = [];
    if idx <= numel(summary) && isstruct(summary{idx}) && isfield(summary{idx}, 'covered_tasks')
        tasks = reshape(sort(summary{idx}.covered_tasks(:)), 1, []);
    end
end

function summary = safe_summary(plan)
    if isstruct(plan) && isfield(plan, 'task_sequence_summary')
        summary = plan.task_sequence_summary;
    else
        summary = {};
    end
end

function poses = safe_pose_array(plan)
    if isstruct(plan) && isfield(plan, 'selected_poses') && ~isempty(plan.selected_poses)
        poses = plan.selected_poses;
    else
        poses = zeros(0, 3);
    end
end

function a = safe_num_array(s, field_name)
    if isstruct(s) && isfield(s, field_name) && ~isempty(s.(field_name))
        a = reshape(double(s.(field_name)), 1, []);
    else
        a = [];
    end
end

function v = safe_num(s, field_name)
    if isstruct(s) && isfield(s, field_name) && isnumeric(s.(field_name)) && ~isempty(s.(field_name))
        v = double(s.(field_name)(1));
    else
        v = 0;
    end
end

function v = safe_record_num(s, field_name)
    v = safe_num(s, field_name);
end

function v = safe_path_length_m(s)
    v = safe_num(s, 'total_path_length_m');
    if v == 0
        v = safe_num(s, 'total_path_length');
    end
end

function txt = safe_text(s, field_name, default_value)
    if isstruct(s) && isfield(s, field_name) && ~isempty(s.(field_name))
        txt = char(string(s.(field_name)));
    else
        txt = default_value;
    end
end
