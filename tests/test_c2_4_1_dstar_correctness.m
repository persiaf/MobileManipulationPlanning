function tests = test_c2_4_1_dstar_correctness
%TEST_C2_4_1_DSTAR_CORRECTNESS End-to-end consecutive local replanning.
    tests = functiontests(localfunctions);
end

function testConsecutiveLocalReplanReusesEdgeStates(testCase)
    state = c2_fixture_dstar();
    frozen = state.rc2_plan;
    [changed1, event1] = apply_dynamic_event(state, 'ADD_LOCAL_OBSTACLE', 5);
    impact1 = analyze_change_impact(state, event1);
    [result1, next1, diag1] = local_replan(state, changed1, event1, impact1);
    verifyTrue(testCase, any(strcmp(result1.status, {'SUCCESS','FALLBACK_REQUIRED'})));
    verifyTrue(testCase, isfield(diag1, 'incremental_dstar'));
    if strcmp(result1.status, 'SUCCESS'), verifyTrue(testCase, isfield(next1.stats, 'path_search_states')); end
    verifyEqual(testCase, next1.rc2_plan, frozen);
    if isfield(next1.stats, 'path_search_states')
        verifyGreaterThan(testCase, numel(next1.stats.path_search_states.states), 0);
    end

    [changed2, event2] = apply_dynamic_event(next1, 'ADD_LOCAL_OBSTACLE', 6);
    impact2 = analyze_change_impact(next1, event2);
    [result2, next2, diag2] = local_replan(next1, changed2, event2, impact2);
    verifyTrue(testCase, any(strcmp(result2.status, {'SUCCESS','FALLBACK_REQUIRED'})));
    if isfield(diag2, 'incremental_dstar')
        verifyTrue(testCase, ~diag2.incremental_dstar.fallback_used || ~isempty(diag2.incremental_dstar.fallback_reason));
    end
    verifyNotEmpty(testCase, next2.path_repair_version);
    verifyEqual(testCase, next2.rc2_plan, frozen);
    verifyEqual(testCase, next2.task_state, next1.task_state);
    if isfield(next2, 'global_path') && ~isempty(next2.global_path)
        verifyEqual(testCase, next2.stats.path_len, compute_path_length(next2.global_path), 'AbsTol', 1e-12);
    end
end

function testMovingStartAndMapChangeKeepStateCompatible(testCase)
    state = c2_fixture_dstar();
    [changed1, event1] = apply_dynamic_event(state, 'ADD_LOCAL_OBSTACLE', 5);
    impact1 = analyze_change_impact(state, event1);
    [~, next1] = local_replan(state, changed1, event1, impact1);
    if isfield(next1.stats, 'global_path') && ~isempty(next1.stats.global_path)
        next1.scene.start_pose = next1.stats.global_path(min(3, size(next1.stats.global_path, 1)), :);
    end
    next1.all_poses(1, :) = next1.scene.start_pose;
    [changed2, event2] = apply_dynamic_event(next1, 'ADD_LOCAL_OBSTACLE', 6);
    impact2 = analyze_change_impact(next1, event2);
    [result, next2, diagnostics] = local_replan(next1, changed2, event2, impact2);
    verifyTrue(testCase, any(strcmp(result.status, {'SUCCESS','FALLBACK_REQUIRED'})));
    verifyTrue(testCase, isfield(diagnostics, 'incremental_dstar'));
    if strcmp(result.status, 'FALLBACK_REQUIRED'), verifyTrue(testCase, ~isempty(result.fallback_reason)); end
end

function testKmAccumulatesConsecutiveStartMotion(testCase)
    [map, info, cfg, ~, goal] = direct_fixture(1.0);
    a = [1.5, 10.5, 0]; b = [2.5, 9.5, 0]; c = [3.5, 8.5, 0];
    [~, s1, d1] = run_dstar_lite_wrapper(map, a, goal, info, cfg, direct_opts([]));
    [~, s2, d2] = run_dstar_lite_wrapper(map, b, goal, info, cfg, direct_opts(s1));
    [~, s3, d3] = run_dstar_lite_wrapper(map, c, goal, info, cfg, direct_opts(s2));
    verifyFalse(testCase, d1.incremental_reuse);
    verifyTrue(testCase, d2.incremental_reuse && d3.incremental_reuse);
    verifyEqual(testCase, s2.key_modifier, hypot(1, 1), 'AbsTol', 1e-12);
    verifyEqual(testCase, s3.key_modifier, 2 * hypot(1, 1), 'AbsTol', 1e-12);
    verifyEqual(testCase, s3.previous_start, [4, 4]);
end

function testGridCostAndWorldLengthUseSeparateUnits(testCase)
    [map, info, cfg, start, goal] = direct_fixture(0.05);
    [path, ~, d] = run_dstar_lite_wrapper(map, start, goal, info, cfg, direct_opts([]));
    verifyEqual(testCase, d.path_length_m, sum(sqrt(sum(diff(path(:, 1:2), 1, 1).^2, 2))), 'AbsTol', 1e-12);
    verifyEqual(testCase, d.path_length_m, d.search_cost_grid * info.resolution, 'AbsTol', 1e-10);
end

function testFootprintConfigChangeReinitializes(testCase)
    [map, info, cfg, start, goal] = direct_fixture(1.0);
    [~, state, ~] = run_dstar_lite_wrapper(map, start, goal, info, cfg, direct_opts([]));
    cfg.robot_cfg.footprint.width_m = cfg.robot_cfg.footprint.width_m + 0.1;
    [~, ~, d] = run_dstar_lite_wrapper(map, start, goal, info, cfg, direct_opts(state));
    verifyTrue(testCase, d.reinitialized && ~d.incremental_reuse);
end

function testEqualOccupancyDifferentLayoutChangesSignature(testCase)
    [map, info, cfg, start, goal] = direct_fixture(1.0);
    map(4, 4) = 1; map(5, 5) = 1;
    [~, state, ~] = run_dstar_lite_wrapper(map, start, goal, info, cfg, direct_opts([]));
    map(4, 4) = 0; map(5, 5) = 0; map(4, 5) = 1; map(5, 4) = 1;
    [~, ~, d] = run_dstar_lite_wrapper(map, start, goal, info, cfg, direct_opts(state));
    verifyTrue(testCase, d.incremental_reuse);
    verifyEqual(testCase, d.changed_cell_count, 4);
end

function [map, info, cfg, start, goal] = direct_fixture(resolution)
    map = zeros(12, 12); info = struct('rows', 12, 'cols', 12, 'resolution', resolution, 'height_m', 12 * resolution);
    cfg = struct('global_planner', 'dstar_lite', 'incremental_replanning_enabled', true, ...
        'footprint_aware_navigation', false, 'robot_cfg', robot_config(), ...
        'astar', struct('allow_diagonal', true, 'heuristic_weight', 1));
    start = [0.5 * resolution, 10.5 * resolution, 0]; goal = [10.5 * resolution, 1.5 * resolution, 0];
end

function o = direct_opts(state)
    o = struct('incremental_mode', true, 'dstar_state', state, 'changed_cells', []);
end

function state = c2_fixture_dstar()
    state = p5_fixture_state('incremental_navigation');
    state.task_state=struct('predicted_tasks',state.plan.covered_tasks_predicted, ...
        'commanded_tasks',1,'executed_tasks',1,'verified_tasks',1);
end
