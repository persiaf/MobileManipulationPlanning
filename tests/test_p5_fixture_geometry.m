function tests = test_p5_fixture_geometry
%TEST_P5_FIXTURE_GEOMETRY The P5 scaffold must carry real main-model evidence.
    tests=functiontests(localfunctions);
end

function setupOnce(testCase)
    testCase.TestData.state=p5_fixture_state();
end

function testMainRobotAndTableTasksAreValid(testCase)
    state=testCase.TestData.state;
    verifyEqual(testCase,state.robot_cfg,robot_config());
    verifyEqual(testCase,state.planner_cfg.robot_cfg,state.robot_cfg);
    planner=planner_config();
    verifyEqual(testCase,state.planner_cfg.reachability.threshold,planner.reachability.threshold);
    verifyFalse(testCase,check_footprint_collision(state.scene.start_pose, ...
        state.scene.grid_map,state.scene.map_info,state.robot_cfg));
    verifyEqual(testCase,[state.scene.tasks.id],1:state.scene.num_tasks);
    cfg=state.scene.scene_cfg;
    for task=state.scene.tasks
        local=rot2(-cfg.table_orientations(task.table_id))* ...
            ([task.x task.y]-cfg.table_positions(task.table_id,:))';
        verifyLessThanOrEqual(testCase,abs(local(1)),cfg.table.length/2);
        verifyLessThanOrEqual(testCase,abs(local(2)),cfg.table.width/2);
    end
end

function testCandidateCoverageIsMeasuredAtItsActualPose(testCase)
    state=testCase.TestData.state;
    for r=1:numel(state.candidate_library.regions)
        candidates=state.candidate_library.regions(r).candidates;
        verifyGreaterThanOrEqual(testCase,numel(candidates),2);
        verifyEqual(testCase,sort([candidates.side_id]),[1 2]);
        for candidate=candidates
            verifyFalse(testCase,check_footprint_collision(candidate.pose, ...
                state.scene.grid_map,state.scene.map_info,state.robot_cfg));
            for task_id=candidate.covered_tasks
                task=state.scene.tasks([state.scene.tasks.id]==task_id);
                [~,~,score]=evaluate_dual_arm_reachability(candidate.pose,[task.x task.y task.z], ...
                    state.robot_cfg,state.planner_cfg.reachability.threshold);
                verifyGreaterThan(testCase,score,0);
            end
        end
    end
end

function testRoutesHaveExactDocksAndNoSimulatedPhysicalCompletion(testCase)
    state=testCase.TestData.state;
    validation=validate_replanning_state(state,state);
    verifyTrue(testCase,validation.valid,validation.reason);
    verifyEmpty(testCase,state.task_state.commanded_tasks);
    verifyEmpty(testCase,state.task_state.executed_tasks);
    verifyEmpty(testCase,state.task_state.verified_tasks);
    order=state.stats.route_order;
    for k=1:numel(state.stats.segment_stats)
        path=state.stats.segment_stats(k).path;
        verifyEqual(testCase,path(1,:),state.all_poses(order(k),:),'AbsTol',1e-9);
        verifyEqual(testCase,path(end,:),state.all_poses(order(k+1),:),'AbsTol',1e-9);
    end
end
