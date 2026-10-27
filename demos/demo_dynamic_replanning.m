function result = demo_dynamic_replanning(event_type, seed)
%DEMO_DYNAMIC_REPLANNING Show the P5 D1 before/after local repair view.

    if nargin < 1 || isempty(event_type), event_type = 'ADD_LOCAL_OBSTACLE'; end
    if nargin < 2 || isempty(seed), seed = 2026; end

    root = fileparts(fileparts(mfilename('fullpath')));
    init_paths(root);
    vis_cfg = visualization_config();
    state = p5_demo_state(root, seed);
    [changed_state, event] = apply_dynamic_event(state, event_type, 1);
    impact = analyze_change_impact(state, event);
    local_result = local_replan(state, changed_state, event, impact);
    figures = visualize_dynamic_replan(state, event, local_result, vis_cfg, ...
        'title', 'V3 Dynamic Local Replanning', 'prefix', 'V3_dynamic');
    result = struct('state_before', state, 'state_after', changed_state, ...
        'event', event, 'impact', impact, 'local_result', local_result, ...
        'figures', figures);
end

function state = p5_demo_state(root, seed)
    state_file = fullfile(root, 'results', 'tables', ...
        sprintf('p3p4_state_l2_seed%d_bottom_up.mat', seed));
    if isfile(state_file)
        data = load(state_file);
        state.scale = 'V3';
        state.seed = seed;
        state.scene = data.scene;
        state.robot_cfg = data.robot_cfg;
        state.planner_cfg = data.planner_cfg;
        state.candidate_library = data.candidate_library;
        state.plan = data.plan;
        state.all_poses = [state.scene.start_pose; state.plan.selected_poses];
        state.stats = lightweight_stats(state);
    else
        state = p5_fixture_state();
    end
end

function stats = lightweight_stats(state)
    adjacency = knn_graph(state.all_poses, 4);
    cost = euclidean_cost_matrix(state.all_poses, adjacency);
    stats = struct();
    stats.sparse_graph = struct('adjacency', adjacency, ...
        'region_ids', [0; state.plan.pose_to_region_map(:)], ...
        'cost_matrix', cost);
    stats.route_order = 1:size(state.all_poses, 1);
    stats.global_path = state.all_poses(:, 1:2);
    stats.path_len = sum(sqrt(sum(diff(stats.global_path).^2, 2)));
end

function cost = euclidean_cost_matrix(poses, adjacency)
    n = size(poses, 1);
    cost = inf(n);
    cost(1:n+1:end) = 0;
    for i = 1:n
        for j = i+1:n
            if adjacency(i, j)
                d = norm(poses(i, 1:2) - poses(j, 1:2));
                cost(i, j) = d;
                cost(j, i) = d;
            end
        end
    end
end

function adjacency = knn_graph(poses, k)
    n = size(poses, 1);
    adjacency = false(n);
    for i = 1:n
        d = sqrt(sum((poses(:, 1:2) - poses(i, 1:2)).^2, 2));
        d(i) = inf;
        [~, order] = sort(d);
        nbrs = order(1:min(k, n - 1));
        adjacency(i, nbrs) = true;
        adjacency(nbrs, i) = true;
    end
end
