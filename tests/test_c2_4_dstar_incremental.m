function tests = test_c2_4_dstar_incremental
%TEST_C2_4_DSTAR_INCREMENTAL Verify persisted D* Lite search state.
    tests = functiontests(localfunctions);
end

function testInitialSearchIsNotReuse(testCase)
    [map, info, cfg, start_pose, goal_pose] = fixture();
    [path, ~, state, d] = run_path_planner(map, start_pose, goal_pose, info, cfg, 'dstar_lite', opts([]));
    verifyTrue(testCase, dstar_field(d, 'path_found'));
    verifyFalse(testCase, dstar_field(d, 'incremental_reuse'));
    verifyTrue(testCase, ~isempty(path));
    verifyGreaterThan(testCase, state.state_version, 0);
end

function testLocalObstacleReusesState(testCase)
    [map, info, cfg, start_pose, goal_pose] = fixture();
    [~, ~, state, d1] = run_path_planner(map, start_pose, goal_pose, info, cfg, 'dstar_lite', opts([]));
    changed = [10, 10]; map(10, 10) = 1;
    [path, ~, state2, d2] = run_path_planner(map, start_pose, goal_pose, info, cfg, 'dstar_lite', opts(state, changed));
    verifyTrue(testCase, d1.path_found && d2.path_found);
    verifyTrue(testCase, d2.incremental_reuse);
    verifyFalse(testCase, d2.reinitialized);
    verifyEqual(testCase, d2.changed_cell_count, 1);
    verifyGreaterThanOrEqual(testCase, state2.state_version, state.state_version + 1);
    verifyTrue(testCase, path_is_free(path, map, info));
end

function testObstacleRemovalReusesState(testCase)
    [map, info, cfg, start_pose, goal_pose] = fixture(); map(10, 10) = 1;
    [~, ~, state, ~] = run_path_planner(map, start_pose, goal_pose, info, cfg, 'dstar_lite', opts([]));
    map(10, 10) = 0;
    [~, ~, ~, d] = run_path_planner(map, start_pose, goal_pose, info, cfg, 'dstar_lite', opts(state, [10, 10]));
    verifyTrue(testCase, d.path_found && d.incremental_reuse);
end

function testConsecutiveUpdatesUseNewestState(testCase)
    [map, info, cfg, start_pose, goal_pose] = fixture();
    [~, ~, state1, ~] = run_path_planner(map, start_pose, goal_pose, info, cfg, 'dstar_lite', opts([]));
    map(10, 10) = 1; [~, ~, state2, d2] = run_path_planner(map, start_pose, goal_pose, info, cfg, 'dstar_lite', opts(state1, [10, 10]));
    map(11, 11) = 1; [path, ~, ~, d3] = run_path_planner(map, start_pose, goal_pose, info, cfg, 'dstar_lite', opts(state2, [11, 11]));
    verifyTrue(testCase, d2.incremental_reuse && d3.incremental_reuse);
    verifyEqual(testCase, d3.state_version, state2.state_version + 1);
    verifyTrue(testCase, path_is_free(path, map, info));
end

function testNoMapChangeIsReuseNoop(testCase)
    [map, info, cfg, start_pose, goal_pose] = fixture();
    [~, ~, state, ~] = run_path_planner(map, start_pose, goal_pose, info, cfg, 'dstar_lite', opts([]));
    [path, ~, ~, d] = run_path_planner(map, start_pose, goal_pose, info, cfg, 'dstar_lite', opts(state, []));
    verifyTrue(testCase, d.incremental_reuse && d.path_found);
    verifyEqual(testCase, d.changed_cell_count, 0);
    verifyTrue(testCase, path_is_free(path, map, info));
end

function testGoalChangeReinitializes(testCase)
    [map, info, cfg, start_pose, goal_pose] = fixture();
    [~, ~, state, ~] = run_path_planner(map, start_pose, goal_pose, info, cfg, 'dstar_lite', opts([]));
    new_goal = [17.5, 3.5, 0];
    [path, ~, ~, d] = run_path_planner(map, start_pose, new_goal, info, cfg, 'dstar_lite', opts(state, []));
    verifyTrue(testCase, d.reinitialized && ~d.incremental_reuse && d.path_found);
    verifyTrue(testCase, ~isempty(path));
end

function testUnreachableReturnsFailure(testCase)
    [map, info, cfg, start_pose, goal_pose] = fixture(); map(10, :) = 1;
    [path, ~, ~, d] = run_path_planner(map, start_pose, goal_pose, info, cfg, 'dstar_lite', opts([]));
    verifyFalse(testCase, d.path_found);
    verifyFalse(testCase, d.path_safe);
    verifyLessThanOrEqual(testCase, size(path, 1), 2);
end

function testIncrementalCostMatchesFreshSearch(testCase)
    [map, info, cfg, start_pose, goal_pose] = fixture();
    [~, ~, state, ~] = run_path_planner(map, start_pose, goal_pose, info, cfg, 'dstar_lite', opts([]));
    map(10, 10) = 1;
    [~, ~, ~, d_inc] = run_path_planner(map, start_pose, goal_pose, info, cfg, 'dstar_lite', opts(state, [10, 10]));
    [~, ~, ~, d_full] = run_path_planner(map, start_pose, goal_pose, info, cfg, 'dstar_lite', opts([]));
    verifyEqual(testCase, d_inc.path_length, d_full.path_length, 'AbsTol', 1e-9);
end

function testStateContainsPersistentSearchFields(testCase)
    [map, info, cfg, start_pose, goal_pose] = fixture();
    [~, ~, state, ~] = run_path_planner(map, start_pose, goal_pose, info, cfg, 'dstar_lite', opts([]));
    verifyTrue(testCase, all(isfield(state, {'g','rhs','open_list','open_keys','key_modifier','previous_start','occupancy_map'})) || ...
        all(isfield(state, {'g','rhs','open_list','open_keys','key_modifier','previous_start','traversable'})));
    verifyTrue(testCase, isfield(state, 'map_signature') && isfield(state, 'state_version'));
end

function testLegacyFiveInputWrapperStillWorks(testCase)
    [map, info, cfg, start_pose, goal_pose] = fixture();
    path = run_dstar_lite_wrapper(map, start_pose, goal_pose, info, cfg);
    verifyGreaterThanOrEqual(testCase, size(path, 1), 2);
end

function testDiagnosticsExposeMeasuredQueueWork(testCase)
    [map, info, cfg, start_pose, goal_pose] = fixture();
    [~, ~, ~, d] = run_path_planner(map, start_pose, goal_pose, info, cfg, 'dstar_lite', opts([]));
    verifyGreaterThan(testCase, d.expanded_node_count, 0);
    verifyGreaterThan(testCase, d.queue_operation_count, 0);
    verifyGreaterThanOrEqual(testCase, d.search_time_ms, 0);
end

function testMapSignatureAndVersionAdvance(testCase)
    [map, info, cfg, start_pose, goal_pose] = fixture();
    [~, ~, state1, ~] = run_path_planner(map, start_pose, goal_pose, info, cfg, 'dstar_lite', opts([]));
    map(10, 10) = 1;
    [~, ~, state2, d] = run_path_planner(map, start_pose, goal_pose, info, cfg, 'dstar_lite', opts(state1, [10, 10]));
    verifyNotEqual(testCase, state1.map_signature, state2.map_signature);
    verifyEqual(testCase, state2.state_version, state1.state_version + 1);
    verifyEqual(testCase, d.state_version, state2.state_version);
end

function testDifferentGoalsDoNotReuseState(testCase)
    [map, info, cfg, start_pose, goal_pose] = fixture();
    [~, ~, state, ~] = run_path_planner(map, start_pose, goal_pose, info, cfg, 'dstar_lite', opts([]));
    [~, ~, ~, d] = run_path_planner(map, start_pose, [17.5, 3.5, 0], info, cfg, 'dstar_lite', opts(state, []));
    verifyTrue(testCase, d.reinitialized);
    verifyFalse(testCase, d.incremental_reuse);
end

function testFallbackIsNotReportedAsIncrementalSuccess(testCase)
    [map, info, cfg, start_pose, goal_pose] = fixture(); map(10, :) = 1;
    [~, ~, ~, d] = run_path_planner(map, start_pose, goal_pose, info, cfg, 'dstar_lite', opts([]));
    verifyFalse(testCase, d.path_found);
    verifyFalse(testCase, d.path_safe);
    verifyFalse(testCase, d.incremental_reuse);
end

function testFootprintInflatedIncrementalPathIsSafe(testCase)
    [map, info, cfg, start_pose, goal_pose] = fixture();
    cfg.footprint_aware_navigation = true; cfg.robot_cfg = robot_config();
    [~, ~, state, d1] = run_path_planner(map, start_pose, goal_pose, info, cfg, 'dstar_lite', opts([]));
    map(10, 10) = 1;
    [path, ~, ~, d2] = run_path_planner(map, start_pose, goal_pose, info, cfg, 'dstar_lite', opts(state, [10, 10]));
    verifyTrue(testCase, d1.path_found && d2.path_found && d2.path_safe);
    verifyTrue(testCase, path_is_free(path, build_footprint_inflated_map(map, cfg.robot_cfg.footprint, info), info));
end

function [map, info, cfg, start_pose, goal_pose] = fixture()
    map = zeros(20, 20); info = struct('rows', 20, 'cols', 20, 'resolution', 1, 'height_m', 20);
    cfg = struct('global_planner', 'dstar_lite', 'incremental_replanning_enabled', true, ...
        'footprint_aware_navigation', false, 'astar', struct('allow_diagonal', true, 'heuristic_weight', 1));
    start_pose = [1.5, 18.5, 0]; goal_pose = [17.5, 2.5, 0];
end

function o = opts(state, changed)
    if nargin < 2, changed = []; end
    o = struct('incremental_mode', true, 'dstar_state', state, 'changed_cells', changed);
end

function value = dstar_field(d, name)
    value = isstruct(d) && isfield(d, name) && logical(d.(name));
end

function ok = path_is_free(path, map, info)
    ok = ~isempty(path);
    for i = 1:size(path, 1)
        [r, c] = world2grid(path(i, 1), path(i, 2), info.resolution, info.height_m);
        ok = ok && r >= 1 && r <= size(map, 1) && c >= 1 && c <= size(map, 2) && map(r, c) == 0;
    end
end
