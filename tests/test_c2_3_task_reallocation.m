function tests = test_c2_3_task_reallocation
%TEST_C2_3_TASK_REALLOCATION Deterministic coverage-failure repair checks.
    tests = functiontests(localfunctions);
end

function testCoverageFailureEventCarriesStablePair(testCase)
    state = c2_fixture();
    [changed, event] = apply_dynamic_event(state, 'TASK_COVERAGE_FAILURE', 1);
    verifyEqual(testCase, event.type, 'TASK_COVERAGE_FAILURE');
    verifyEqual(testCase, event.invalidated_coverage_pairs, ...
        [event.target_task_id, event.target_dock_index]);
    impact = analyze_change_impact(state, event);
    verifyEqual(testCase, impact.affected_tasks, event.target_task_id);
    verifyEqual(testCase, impact.affected_docks, event.target_dock_index);
    verifyEqual(testCase, changed.plan.selected_poses, state.plan.selected_poses);
end

function testAlreadyVerifiedTaskIsNoop(testCase)
    state = c2_fixture();
    [changed, event] = apply_dynamic_event(state, 'TASK_COVERAGE_FAILURE', 1);
    state.task_state.verified_tasks = event.target_task_id;
    impact = analyze_change_impact(state, event);
    [result, next_state] = local_replan(state, changed, event, impact);
    verifyEqual(testCase, result.status, 'NOOP');
    verifyEqual(testCase, result.fallback, false);
    verifyEqual(testCase, next_state.task_state.verified_tasks, event.target_task_id);
end

function testInvalidCoveragePairFallsBack(testCase)
    state = c2_fixture();
    [changed, event] = apply_dynamic_event(state, 'TASK_COVERAGE_FAILURE', 1);
    event.invalidated_coverage_pairs = [event.target_task_id, event.target_dock_index + 100];
    impact = analyze_change_impact(state, event);
    [result, next_state] = local_replan(state, changed, event, impact);
    verifyEqual(testCase, result.status, 'FALLBACK_REQUIRED');
    verifyEqual(testCase, result.fallback_reason, 'INVALID_COVERAGE_PAIR');
    verifyEqual(testCase, next_state.rc2_plan, state.rc2_plan);
end

function testUnknownTaskFallsBack(testCase)
    state = c2_fixture();
    event = struct('type', 'TASK_COVERAGE_FAILURE', 'event_id', 9, ...
        'target_task_id', 9999, 'target_dock_index', 1, 'target_region', 1, ...
        'invalidated_coverage_pairs', [9999 1], 'reason', 'test');
    impact = analyze_change_impact(state, event);
    [result, ~] = local_replan(state, state, event, impact);
    verifyEqual(testCase, result.status, 'FALLBACK_REQUIRED');
    verifyEqual(testCase, result.fallback_reason, 'INVALID_TASK_ID');
end

function testOldSingleOutputInterface(testCase)
    state = c2_fixture();
    [changed, event] = apply_dynamic_event(state, 'TASK_COVERAGE_FAILURE', 1);
    impact = analyze_change_impact(state, event);
    result = local_replan(state, changed, event, impact);
    verifyTrue(testCase, isfield(result, 'status'));
end

function testPendingTaskCanBeReassignedWhenExistingDockReachesIt(testCase)
    state = c2_fixture();
    task_id = state.plan.task_sequence_summary{1}.covered_tasks(1);
    event = struct('type', 'TASK_COVERAGE_FAILURE', 'event_id', 100, ...
        'target_task_id', task_id, 'target_dock_index', 1, 'target_region', 1, ...
        'invalidated_coverage_pairs', [task_id 1], 'reason', 'test');
    impact = analyze_change_impact(state, event);
    [result, next_state, diagnostics] = local_replan(state, state, event, impact);
    verifyEqual(testCase, result.status, 'SUCCESS');
    verifyTrue(testCase, any(strcmp(diagnostics.reassignment_level, {'EXISTING_DOCK','NEW_DOCK'})));
    verifyTrue(testCase, any(next_state.plan.covered_tasks_predicted == task_id));
    verifyEqual(testCase, result.final_path_length, compute_path_length(result.global_path), 'AbsTol', 1e-12);
    verifyTrue(testCase, result.local_astar_calls >= 0);
    verifyTrue(testCase, diagnostics.path_validation.valid);
    verifyTrue(testCase, diagnostics.path_validation.footprint_collision_free);
end

function state = c2_fixture()
    [scene, robot_cfg, planner_cfg, regions] = create_test_fixture();
    library = build_candidate_library(regions, scene, robot_cfg, planner_cfg);
    plan = run_planning_strategy(library, regions, scene, robot_cfg, planner_cfg);
    plan.candidate_library = library;
    [scene, ~, ~, ~, ~, ~, stats] = stage3_global_planning(scene, ...
        plan.selected_poses, plan.task_sequence_summary, {}, robot_cfg, ...
        planner_cfg, plan.pose_info_list, struct(), plan);
    state = stats.planning_state;
    state.task_state = struct('predicted_tasks', plan.covered_tasks_predicted, ...
        'commanded_tasks', [], 'executed_tasks', [], 'verified_tasks', []);
    for i = 1:numel(scene.tasks), scene.tasks(i).status = 0; end
    state.scene = scene;
    state = normalize_planning_state(state);
end
