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
    comparison.visit_order_hamming = hamming_count( ...
        safe_num_array(reference_plan, 'preferred_visit_order'), safe_num_array(plan, 'preferred_visit_order'));
    [match_count, diff_count] = pose_difference( ...
        safe_pose_array(reference_plan), safe_pose_array(plan), pos_tol, yaw_tol);
    comparison.pose_match_count = match_count;
    comparison.pose_difference_count = diff_count;
    comparison.task_assignment_difference_count = task_assignment_difference( ...
        safe_summary(reference_plan), safe_summary(plan));
    comparison.route_edge_difference_count = route_edge_difference( ...
        safe_num_array(reference_plan, 'preferred_visit_order'), safe_num_array(plan, 'preferred_visit_order'));
    comparison.num_docks_difference = safe_num(plan, 'num_docking_poses') - ...
        safe_num(reference_plan, 'num_docking_poses');
    comparison.path_length_difference_m = safe_record_num(record, 'total_path_length') - ...
        safe_record_num(reference_record, 'total_path_length');
    comparison.execution_time_difference_s = safe_record_num(record, 'estimated_execution_time_s') - ...
        safe_record_num(reference_record, 'estimated_execution_time_s');
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

function n = route_edge_difference(a, b)
    ea = route_edges(a);
    eb = route_edges(b);
    n = size(setxor(ea, eb, 'rows'), 1);
end

function edges = route_edges(order)
    if numel(order) < 2
        edges = zeros(0, 2);
    else
        edges = [order(1:end-1)', order(2:end)'];
    end
end

function tasks = summary_tasks(summary, idx)
    tasks = [];
    if idx <= numel(summary) && isstruct(summary{idx}) && isfield(summary{idx}, 'covered_tasks')
        tasks = sort(summary{idx}.covered_tasks(:)');
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

function txt = safe_text(s, field_name, default_value)
    if isstruct(s) && isfield(s, field_name) && ~isempty(s.(field_name))
        txt = char(string(s.(field_name)));
    else
        txt = default_value;
    end
end
