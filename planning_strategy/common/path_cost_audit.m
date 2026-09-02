function output = path_cost_audit(action, varargin)
%PATH_COST_AUDIT Optional query logger used by diagnostic experiments only.
% This function observes path-cost calls; it never serves or caches a cost.

    persistent state
    if isempty(state), state = new_state(); end
    output = [];
    switch lower(action)
        case 'reset'
            state = new_state();
            state.enabled = true;
            if ~isempty(varargin), state.scene_id = char(varargin{1}); end
        case 'reset_count_only'
            state = new_state();
            state.enabled = true;
            state.store_entries = false;
            if ~isempty(varargin), state.scene_id = char(varargin{1}); end
        case 'disable'
            state = new_state();
        case 'set_context'
            state.strategy = char(varargin{1});
            state.source = char(varargin{2});
            if numel(varargin) >= 3, state.scene_id = char(varargin{3}); end
        case 'set_source'
            state.source = char(varargin{1});
        case 'record'
            if ~state.enabled, return; end
            start_pose = varargin{1}; goal_pose = varargin{2};
            planner = char(varargin{3}); computed = logical(varargin{4});
            state.sequence = state.sequence + 1;
            state.expensive_count = state.expensive_count + double(computed);
            if ~state.store_entries, return; end
            entry = struct('sequence', state.sequence, 'scene_id', state.scene_id, ...
                'strategy', state.strategy, 'query_source', state.source, ...
                'planner', lower(planner), 'start_x', start_pose(1), ...
                'start_y', start_pose(2), 'start_theta', pose_theta(start_pose), ...
                'goal_x', goal_pose(1), 'goal_y', goal_pose(2), ...
                'goal_theta', pose_theta(goal_pose), 'directed_key', ...
                make_key(state.scene_id, planner, start_pose, goal_pose), ...
                'expensive_computation', computed);
            state.entries(end + 1, 1) = entry;
        case 'snapshot'
            output = state.entries;
        case 'expensive_count'
            output = state.expensive_count;
        case 'context'
            output = struct('strategy', state.strategy, 'source', state.source, ...
                'scene_id', state.scene_id, 'enabled', state.enabled);
        otherwise
            error('path_cost_audit:UnknownAction', 'Unknown action %s.', action);
    end
end

function state = new_state()
    empty = struct('sequence', {}, 'scene_id', {}, 'strategy', {}, ...
        'query_source', {}, 'planner', {}, 'start_x', {}, 'start_y', {}, ...
        'start_theta', {}, 'goal_x', {}, 'goal_y', {}, 'goal_theta', {}, ...
        'directed_key', {}, 'expensive_computation', {});
    state = struct('enabled', false, 'store_entries', true, 'scene_id', '', ...
        'strategy', '', 'source', 'OTHER', 'sequence', 0, ...
        'expensive_count', 0, 'entries', empty);
end

function key = make_key(scene_id, planner, start_pose, goal_pose)
    q = 1000;
    a = round([start_pose(1:2), pose_theta(start_pose)] * q);
    b = round([goal_pose(1:2), pose_theta(goal_pose)] * q);
    key = sprintf('%s|%s|%d_%d_%d>%d_%d_%d', scene_id, lower(planner), ...
        a(1), a(2), a(3), b(1), b(2), b(3));
end

function theta = pose_theta(pose)
    if numel(pose) >= 3, theta = pose(3); else, theta = 0; end
end
