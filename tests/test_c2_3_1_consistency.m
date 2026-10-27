function tests = test_c2_3_1_consistency
%TEST_C2_3_1_CONSISTENCY Verify task/dock/state consistency after C2-3.
    tests = functiontests(localfunctions);
end

function testNewDockMovesTaskExactlyOnce(testCase)
    state = c2_fixture();
    task_id = state.plan.task_sequence_summary{1}.covered_tasks(1);
    event = coverage_event(task_id, 1, 1);
    impact = analyze_change_impact(state, event);
    [result, next_state, diagnostics] = local_replan(state, state, event, impact);
    verifyEqual(testCase, result.status, 'SUCCESS');
    verifyEqual(testCase, diagnostics.reassignment_level, 'NEW_DOCK');
    owners = owners_of_task(next_state.plan.task_sequence_summary, task_id);
    verifyEqual(testCase, numel(owners), 1);
    verifyEqual(testCase, owners, numel(next_state.plan.task_sequence_summary));
    verifyEmpty(testCase, next_state.plan.task_sequence_summary{1}.covered_tasks(next_state.plan.task_sequence_summary{1}.covered_tasks == task_id));
    verifyEqual(testCase, next_state.plan.pose_to_region_map(end), 1);
    verifyTrue(testCase, diagnostics.path_validation.valid);
    verifyTrue(testCase, diagnostics.path_validation.footprint_collision_free);
    verifyEqual(testCase, next_state.rc2_plan, state.rc2_plan);
end

function testExistingDockMergesArmAssignmentAndAction(testCase)
    state = c2_fixture();
    task_id = state.plan.task_sequence_summary{2}.covered_tasks(1);
    dock = 2;
    state.plan.task_sequence_summary{1}.covered_tasks(end + 1) = task_id;
    state.plan.pose_info_list{1}.covered_tasks(end + 1) = task_id;
    old_map = state.plan.task_sequence_summary{dock}.arm_assign_map;
    event = coverage_event(task_id, 1, 1);
    impact = analyze_change_impact(state, event);
    [result, next_state, diagnostics] = local_replan(state, state, event, impact);
    verifyEqual(testCase, result.status, 'SUCCESS');
    verifyEqual(testCase, diagnostics.reassignment_level, 'EXISTING_DOCK');
    verifyEqual(testCase, next_state.plan.pose_to_region_map, state.plan.pose_to_region_map);
    new_map = next_state.plan.task_sequence_summary{dock}.arm_assign_map;
    old_keys = old_map.keys;
    for i = 1:numel(old_keys)
        verifyEqual(testCase, new_map(old_keys{i}), old_map(old_keys{i}));
    end
    verifyTrue(testCase, isfield(next_state.plan.task_sequence_summary{dock}, 'task_actions'));
    action_ids = [next_state.plan.task_sequence_summary{dock}.task_actions.task_id];
    verifyTrue(testCase, any(action_ids == task_id));
end

function testPersistentPairsSurviveTwoEvents(testCase)
    state = c2_fixture();
    task1 = state.plan.task_sequence_summary{1}.covered_tasks(1);
    first_event = coverage_event(task1, 1, 1);
    impact1 = analyze_change_impact(state, first_event);
    [first, state1] = local_replan(state, state, first_event, impact1);
    assumeEqual(testCase, first.status, 'SUCCESS');
    task2 = state1.plan.task_sequence_summary{2}.covered_tasks(1);
    second_event = coverage_event(task2, 2, 1);
    impact2 = analyze_change_impact(state1, second_event);
    [second, state2] = local_replan(state1, state1, second_event, impact2);
    assumeEqual(testCase, second.status, 'SUCCESS');
    verifyTrue(testCase, any(all(state2.invalidated_coverage_pairs == [task1 1], 2)));
    verifyTrue(testCase, any(all(state2.invalidated_coverage_pairs == [task2 2], 2)));
end

function testFailureDoesNotCommitPlan(testCase)
    state = c2_fixture();
    frozen = state.rc2_plan;
    task_id = state.plan.task_sequence_summary{1}.covered_tasks(1);
    state.candidate_library.regions(1).candidates = [];
    event = coverage_event(task_id, 1, 1);
    impact = analyze_change_impact(state, event);
    [result, next_state] = local_replan(state, state, event, impact);
    verifyEqual(testCase, result.status, 'FALLBACK_REQUIRED');
    verifyEqual(testCase, next_state.rc2_plan, frozen);
    verifyEqual(testCase, next_state.plan.selected_poses, state.plan.selected_poses);
end

function event = coverage_event(task_id, dock_idx, region_id)
    event = struct('type', 'TASK_COVERAGE_FAILURE', 'event_id', 901, ...
        'target_task_id', task_id, 'target_dock_index', dock_idx, ...
        'target_region', region_id, 'invalidated_coverage_pairs', [task_id dock_idx], ...
        'reason', 'c2_3_1_test');
end

function docks = owners_of_task(summaries, task_id)
    docks = [];
    for i = 1:numel(summaries)
        if isfield(summaries{i}, 'covered_tasks') && any(summaries{i}.covered_tasks == task_id)
            docks(end + 1) = i; %#ok<AGROW>
        end
    end
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
