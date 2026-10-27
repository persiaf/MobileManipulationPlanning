function tests = test_c2_2_dock_repair
%TEST_C2_2_DOCK_REPAIR Deterministic single-dock replacement checks.
    tests = functiontests(localfunctions);
end

function testInvalidDockGetsRealReplacementAndPath(testCase)
    state = c2_fixture();
    frozen = state.rc2_plan;
    [changed, event] = apply_dynamic_event(state, 'INVALIDATE_DOCK', 1);
    changed = invalidate_dock_cell(state, changed, event);
    impact = analyze_change_impact(state, event);
    [result, next_state, diagnostics] = local_replan(state, changed, event, impact);
    verifyEqual(testCase, result.status, 'SUCCESS');
    verifyEqual(testCase, result.recomputed_docks, 1);
    verifyNotEmpty(testCase, result.global_path);
    verifyTrue(testCase, diagnostics.path_validation.valid);
    verifyTrue(testCase, diagnostics.path_validation.footprint_collision_free);
    verifyNotEqual(testCase, next_state.current_plan.selected_poses, state.plan.selected_poses);
    verifyEqual(testCase, next_state.rc2_plan, frozen);
    verifyEqual(testCase, result.final_path_length, compute_path_length(result.global_path), 'AbsTol', 1e-12);
    dock_idx = event_dock_index(event);
    original_tasks = state.plan.task_sequence_summary{dock_idx}.covered_tasks;
    verifyEqual(testCase, sort(next_state.plan.task_sequence_summary{dock_idx}.covered_tasks), ...
        sort(original_tasks));
    other = setdiff(1:size(state.plan.selected_poses, 1), dock_idx);
    for k = other
        verifyEqual(testCase, next_state.plan.task_sequence_summary{k}, ...
            state.plan.task_sequence_summary{k});
    end
    verifyEqual(testCase, next_state.task_state, state.task_state);
end

function testCandidateChoiceIsDeterministic(testCase)
    state = c2_fixture();
    [changed, event] = apply_dynamic_event(state, 'INVALIDATE_DOCK', 1);
    changed = invalidate_dock_cell(state, changed, event);
    impact = analyze_change_impact(state, event);
    [first, first_state] = local_replan(state, changed, event, impact);
    [second, second_state] = local_replan(state, changed, event, impact);
    verifyEqual(testCase, first.status, 'SUCCESS');
    verifyEqual(testCase, second.status, 'SUCCESS');
    verifyEqual(testCase, first_state.current_plan.selected_poses, ...
        second_state.current_plan.selected_poses);
    verifyEqual(testCase, first.diagnostics.selected_candidate_index, ...
        second.diagnostics.selected_candidate_index);
end

function testEmptyBackupPoolFallsBack(testCase)
    state = c2_fixture();
    state.candidate_library.regions(1).candidates = [];
    [changed, event] = apply_dynamic_event(state, 'INVALIDATE_DOCK', 1);
    impact = analyze_change_impact(state, event);
    [result, next_state] = local_replan(state, changed, event, impact);
    verifyEqual(testCase, result.status, 'FALLBACK_REQUIRED');
    verifyEqual(testCase, result.fallback_reason, 'NO_VALID_BACKUP_DOCK');
    verifyEqual(testCase, next_state.plan.selected_poses, state.plan.selected_poses);
end

function testCandidatesCollidingWithChangedMapAreRejected(testCase)
    state = c2_fixture();
    candidates = state.candidate_library.regions(1).candidates;
    for k = 1:min(3, numel(candidates))
        [i, j] = world2grid(candidates(k).pose(1), candidates(k).pose(2), ...
            state.scene.map_info.resolution, state.scene.map_info.height_m);
        state.scene.grid_map(i, j) = 1;
    end
    state.candidate_library.regions(1).candidates = candidates(1:min(3, numel(candidates)));
    [changed, event] = apply_dynamic_event(state, 'INVALIDATE_DOCK', 1);
    changed.scene = state.scene;
    impact = analyze_change_impact(state, event);
    [result, ~, diagnostics] = local_replan(state, changed, event, impact);
    verifyEqual(testCase, result.status, 'FALLBACK_REQUIRED');
    verifyTrue(testCase, any(contains(diagnostics.candidate_rejections, 'unsafe_candidate')));
end

function testReturnedStateCanDriveAnotherEvent(testCase)
    state = c2_fixture();
    [changed, event] = apply_dynamic_event(state, 'INVALIDATE_DOCK', 1);
    changed = invalidate_dock_cell(state, changed, event);
    impact = analyze_change_impact(state, event);
    [first, next_state] = local_replan(state, changed, event, impact);
    assumeEqual(testCase, first.status, 'SUCCESS');
    [changed2, event2] = apply_dynamic_event(next_state, 'INVALIDATE_DOCK', 2);
    impact2 = analyze_change_impact(next_state, event2);
    [second, state2] = local_replan(next_state, changed2, event2, impact2);
    verifyTrue(testCase, any(strcmp(second.status, {'SUCCESS','FALLBACK_REQUIRED'})));
    verifyTrue(testCase, isfield(state2, 'rc2_plan'));
    verifyEqual(testCase, state2.rc2_plan, state.rc2_plan);
end

function testCompletedAffectedTaskIsNotReassigned(testCase)
    state = c2_fixture();
    state.task_state = struct('verified_tasks', 1);
    [changed, event] = apply_dynamic_event(state, 'INVALIDATE_DOCK', 1);
    impact = analyze_change_impact(state, event);
    [result, next_state, diagnostics] = local_replan(state, changed, event, impact);
    verifyEqual(testCase, result.status, 'FALLBACK_REQUIRED');
    verifyEqual(testCase, result.fallback_reason, 'INVALID_DOCK_HAS_NO_PENDING_TASKS');
    verifyEmpty(testCase, diagnostics.pending_affected_tasks);
    verifyEqual(testCase, next_state.task_state.verified_tasks, 1);
end

function testNoCompleteBackupCoverageFallsBack(testCase)
    state = c2_fixture();
    state.candidate_library.regions(1).candidates(1).covered_tasks = [];
    state.candidate_library.regions(1).candidates(2).covered_tasks = [];
    state.candidate_library.regions(1).candidates(3).covered_tasks = [];
    [changed, event] = apply_dynamic_event(state, 'INVALIDATE_DOCK', 1);
    impact = analyze_change_impact(state, event);
    [result, next_state] = local_replan(state, changed, event, impact);
    verifyEqual(testCase, result.status, 'FALLBACK_REQUIRED');
    verifyTrue(testCase, result.fallback);
    verifyEqual(testCase, result.fallback_reason, 'INCOMPLETE_AFFECTED_TASK_COVERAGE');
    verifyEqual(testCase, next_state.plan.selected_poses, state.plan.selected_poses);
end

function testMultipleInvalidDocksIsExplicitlyUnsupported(testCase)
    state = c2_fixture();
    [changed, event] = apply_dynamic_event(state, 'INVALIDATE_DOCK', 1);
    changed = invalidate_dock_cell(state, changed, event);
    event.target_dock_index = [1 2];
    impact = analyze_change_impact(state, event);
    [result, next_state] = local_replan(state, changed, event, impact);
    verifyEqual(testCase, result.status, 'FALLBACK_REQUIRED');
    verifyEqual(testCase, result.fallback_reason, 'MULTIPLE_INVALID_DOCKS_UNSUPPORTED');
    verifyEqual(testCase, next_state.plan.selected_poses, state.plan.selected_poses);
end

function changed = invalidate_dock_cell(state, changed, event)
    changed.scene = state.scene;
    [i, j] = world2grid(event.center(1), event.center(2), ...
        state.scene.map_info.resolution, state.scene.map_info.height_m);
    changed.scene.grid_map(i, j) = 1;
end

function idx = event_dock_index(event)
    idx = event.target_dock_index;
end

function testOldSingleOutputInterfaceStillWorks(testCase)
    state = c2_fixture();
    [changed, event] = apply_dynamic_event(state, 'INVALIDATE_DOCK', 1);
    changed = invalidate_dock_cell(state, changed, event);
    impact = analyze_change_impact(state, event);
    result = local_replan(state, changed, event, impact);
    verifyTrue(testCase, isfield(result, 'status'));
    verifyEqual(testCase, result.status, 'SUCCESS');
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
