function tests = test_local_replan_route_feasibility
    tests = functiontests(localfunctions);
end

function testPathObstacleRepairAvoidsInvalidEdge(testCase)
    state = p5_fixture_state();
    [changed, event] = apply_dynamic_event(state, 'ADD_LOCAL_OBSTACLE', 5);
    impact = analyze_change_impact(state, event);
    [result,next,diagnostics] = local_replan(state, changed, event, impact);
    verifyFalse(testCase, result.fallback);
    verifyTrue(testCase, result.sparse_graph_connected);
    verifyTrue(testCase, result.no_collision_invalid_route);
    verifyGreaterThanOrEqual(testCase, result.reused_edge_ratio, 0.5);
    verifyTrue(testCase,diagnostics.path_validation.valid);
    verifyEqual(testCase,next.plan.selected_poses,state.plan.selected_poses);
end

function testAddedTasksCoveredByLocalSupplement(testCase)
    state = p5_fixture_state();
    [changed, event] = apply_dynamic_event(state, 'ADD_TASKS', 1);
    impact = analyze_change_impact(state, event);
    original_assignments=assignment_snapshot(state.plan);
    original_library=state.candidate_library;
    [result,next,diagnostics] = local_replan(state, changed, event, impact);
    verifyEqual(testCase,result.status,'SUCCESS');
    verifyEqual(testCase, result.added_tasks, 4);
    verifyEqual(testCase, result.model_feasible_added_tasks, 4);
    verifyEqual(testCase, result.raw_coverage, 100);
    % No independent infeasibility denominator is available in this fixture.
    verifyTrue(testCase,isnan(result.feasible_task_coverage));
    verifyLessThan(testCase, result.region_locality_ratio, 1);
    assigned=[];
    for d=1:numel(next.plan.task_sequence_summary)
        summary=next.plan.task_sequence_summary{d};
        assigned=[assigned,reshape(summary.covered_tasks,1,[])]; %#ok<AGROW>
        verifyEqual(testCase,sort(service_task_ids(summary.service_configs)), ...
            sort(reshape(summary.covered_tasks,1,[])));
        for task_id=reshape(intersect(summary.covered_tasks,event.added_task_ids),1,[])
            task=next.scene.tasks([next.scene.tasks.id]==task_id);
            [~,~,score]=evaluate_dual_arm_reachability(next.plan.selected_poses(d,:), ...
                [task.x task.y task.z],next.robot_cfg,next.planner_cfg.reachability.threshold);
            verifyGreaterThan(testCase,score,0);
        end
    end
    for task_id=event.added_task_ids,verifyEqual(testCase,sum(assigned==task_id),1);end
    verifyEqual(testCase,sort(service_task_ids(next.plan.service_configs)),sort(assigned));
    verifyTrue(testCase,all(ismember(event.added_task_ids,next.task_state.predicted_tasks)));
    verifyEqual(testCase,next.task_state.commanded_tasks,state.task_state.commanded_tasks);
    verifyEqual(testCase,next.task_state.executed_tasks,state.task_state.executed_tasks);
    verifyEqual(testCase,next.task_state.verified_tasks,state.task_state.verified_tasks);
    expected_map=assign_tasks_to_tables(next.scene);
    verifyEqual(testCase,next.task_table_map,expected_map);
    expected_regions=build_task_regions(next.scene,expected_map);
    verifyEqual(testCase,next.regions,expected_regions);
    affected_tables=unique([next.scene.tasks(ismember([next.scene.tasks.id],event.added_task_ids)).table_id]);
    for table_id=reshape(affected_tables,1,[])
        entry=next.candidate_library.regions([next.candidate_library.regions.table_id]==table_id);
        expected_ids=[next.scene.tasks([next.scene.tasks.table_id]==table_id).id];
        verifyEqual(testCase,sort(reshape(entry.task_ids,1,[])),sort(expected_ids));
        verifyEqual(testCase,sort(reshape(entry.region.task_ids,1,[])),sort(expected_ids));
    end
    expected_signature=build_candidate_library_signature(next.regions,next.scene,next.robot_cfg,next.planner_cfg);
    verifyEqual(testCase,next.candidate_library.scene_signature,expected_signature);
    verifyEqual(testCase,next.plan.candidate_library.scene_signature,expected_signature);
    verifyEqual(testCase,assignment_snapshot(state.plan),original_assignments);
    verifyEqual(testCase,state.candidate_library,original_library);
    verifyEqual(testCase,next.rc2_plan,state.rc2_plan);
    verifyTrue(testCase,diagnostics.path_validation.valid);
    verifyEmpty(testCase,next.task_state.verified_tasks);
end

function ids=service_task_ids(services)
    ids=[];
    for k=1:numel(services),ids=[ids,reshape(services(k).task_ids,1,[])];end %#ok<AGROW>
end

function snapshot=assignment_snapshot(plan)
    snapshot=struct('poses',plan.selected_poses,'covered',plan.covered_tasks_predicted, ...
        'summary_tasks',{{}},'arm_keys',{{}},'arm_values',{{}}, ...
        'services',service_task_ids(plan.service_configs));
    for d=1:numel(plan.task_sequence_summary)
        summary=plan.task_sequence_summary{d}; map=summary.arm_assign_map;
        snapshot.summary_tasks{d}=reshape(summary.covered_tasks,1,[]);
        ordered=sort(cell2mat(keys(map)));
        snapshot.arm_keys{d}=ordered;
        snapshot.arm_values{d}=values(map,num2cell(ordered));
    end
end
