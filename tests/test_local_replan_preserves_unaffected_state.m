function tests = test_local_replan_preserves_unaffected_state
    tests = functiontests(localfunctions);
end

function testDockInvalidationPreservesUnaffectedDocks(testCase)
    state = p5_fixture_state();
    [changed, event] = apply_dynamic_event(state, 'INVALIDATE_DOCK', 1);
    impact = analyze_change_impact(state, event);
    [result,next,diagnostics] = local_replan(state, changed, event, impact);
    verifyEqual(testCase, result.status, 'SUCCESS');
    affected=event.target_dock_index;
    unchanged=setdiff(1:size(state.plan.selected_poses,1),affected);
    verifyEqual(testCase,size(next.plan.selected_poses),size(state.plan.selected_poses));
    verifyEqual(testCase,next.plan.selected_poses(unchanged,:),state.plan.selected_poses(unchanged,:));
    verifyGreaterThan(testCase,norm(next.plan.selected_poses(affected,1:2)- ...
        state.plan.selected_poses(affected,1:2)),1e-6);
    for d=unchanged
        verifyEqual(testCase,next.plan.task_sequence_summary{d}.covered_tasks, ...
            state.plan.task_sequence_summary{d}.covered_tasks);
    end
    verifyGreaterThan(testCase,numel(unchanged)/size(state.plan.selected_poses,1),0.5);
    verifyEqual(testCase,next.rc2_plan,state.rc2_plan);
    verifyEqual(testCase,next.task_state,state.task_state);
    verifyTrue(testCase,diagnostics.path_validation.valid);
end
