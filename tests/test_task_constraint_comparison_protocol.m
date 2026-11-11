function tests=test_task_constraint_comparison_protocol
%TEST_TASK_CONSTRAINT_COMPARISON_PROTOCOL Freeze comparison inputs, not IK outcomes.
    tests=functiontests(localfunctions);
end

function setupOnce(testCase)
    testCase.TestData.frozen=task_constraint_comparison_setup();
end

function testSetupRepeatsTheSameGeometryAndModel(testCase)
    first=testCase.TestData.frozen;
    second=task_constraint_comparison_setup();
    verifyEqual(testCase,first.scene.grid_map,second.scene.grid_map);
    verifyEqual(testCase,first.scene.scene_cfg,second.scene.scene_cfg);
    verifyEqual(testCase,first.scene.tasks,second.scene.tasks);
    verifyEqual(testCase,first.pool,second.pool);
    verifyEqual(testCase,first.robot,second.robot);
    verifyEqual(testCase,first.planner,second.planner);
    verifyEqual(testCase,first.scene.num_tasks,10);
    verifyEqual(testCase,first.raw_candidate_count,144);
    verifyGreaterThanOrEqual(testCase,numel(first.pool),80);
    verifyLessThanOrEqual(testCase,numel(first.pool),160);
    verifyEqual(testCase,numel(unique([first.pool.candidate_id])),numel(first.pool));
end

function testEveryFrozenCandidateHasSafeFootprint(testCase)
    setup=testCase.TestData.frozen;
    for k=1:numel(setup.pool)
        verifyFalse(testCase,check_footprint_collision(setup.pool(k).pose, ...
            setup.scene.grid_map,setup.scene.map_info,setup.robot), ...
            sprintf('Candidate %d is not footprint-safe.',setup.pool(k).candidate_id));
    end
end

function testWipingPathsRemainInsideTheirDeclaredTables(testCase)
    setup=testCase.TestData.frozen;cfg=setup.scene.scene_cfg;
    for k=1:numel(setup.scene.tasks)
        task=comparison_manipulation_task(setup.scene.tasks(k),'table_wipe');
        resolved=resolve_manipulation_task(task);
        points=resolved.samples_world;
        local=(rot2(-cfg.table_orientations(task.table_id))* ...
            (points(:,1:2)-cfg.table_positions(task.table_id,:))')';
        verifyLessThanOrEqual(testCase,max(abs(local(:,1))),cfg.table.length/2+1e-12);
        verifyLessThanOrEqual(testCase,max(abs(local(:,2))),cfg.table.width/2+1e-12);
        verifyEqual(testCase,points(:,3),repmat(cfg.table.height,size(points,1),1), ...
            'AbsTol',1e-12);
        verifyEqual(testCase,norm(points(end,:)-points(1,:)),setup.wipe_length_m, ...
            'AbsTol',1e-12);
    end
end

function testTaskProfilesKeepAnchorsAndTCPIdentical(testCase)
    setup=testCase.TestData.frozen;
    for k=1:numel(setup.scene.tasks)
        anchor=setup.scene.tasks(k);
        expected=[anchor.x,anchor.y,anchor.z];tcp=[];
        for mode=setup.mode_order
            task=comparison_manipulation_task(anchor,mode{1});
            resolved=resolve_manipulation_task(task);
            verifyEqual(testCase,[task.x,task.y,task.z],expected);
            verifyEqual(testCase,task.id,anchor.id);
            verifyEqual(testCase,task.table_id,anchor.table_id);
            if isempty(tcp),tcp=resolved.T_ee_tcp;end
            verifyEqual(testCase,resolved.T_ee_tcp,tcp);
        end
    end
end

function testMatchedPathChangesOnlyOrientationFreedom(testCase)
    setup=testCase.TestData.frozen;expected_modes={'none','axis','full'};
    for k=1:numel(setup.scene.tasks)
        reference=[];
        for mi=1:numel(setup.mode_order)
            task=comparison_manipulation_task(setup.scene.tasks(k),setup.mode_order{mi},true);
            cfg=resolve_manipulation_task(task);
            verifyEqual(testCase,cfg.orientation_mode,expected_modes{mi});
            verifyEqual(testCase,cfg.approach_distance_m,0);
            verifyEqual(testCase,cfg.orientation_tolerance_rad,deg2rad(5),'AbsTol',1e-12);
            rest=rmfield(cfg,'orientation_mode');
            if isempty(reference),reference=rest;end
            verifyEqual(testCase,rest,reference);
        end
    end
end

function testConstraintModesHaveDifferentCacheSignatures(testCase)
    setup=testCase.TestData.frozen;scene0=setup.scene;
    regions=build_task_regions(scene0,assign_tasks_to_tables(scene0));
    signatures=cell(1,numel(setup.mode_order));
    for mi=1:numel(setup.mode_order)
        scene=scene0;
        for k=1:numel(scene.tasks)
            scene.tasks(k)=comparison_manipulation_task(scene.tasks(k),setup.mode_order{mi});
        end
        signatures{mi}=build_candidate_library_signature(regions,scene,setup.robot,setup.planner);
        verifyEqual(testCase,scene.grid_map,scene0.grid_map);
        verifyEqual(testCase,[[scene.tasks.x]',[scene.tasks.y]',[scene.tasks.z]'], ...
            [[scene0.tasks.x]',[scene0.tasks.y]',[scene0.tasks.z]']);
    end
    verifyEqual(testCase,numel(unique(signatures)),numel(setup.mode_order));
end

function testNavigationAndSelectionProtocolIsFrozen(testCase)
    setup=testCase.TestData.frozen;cfg=setup.planner;
    verifyEqual(testCase,cfg.robot_cfg,setup.robot);
    verifyEqual(testCase,cfg.global_planner,'astar');
    verifyEqual(testCase,cfg.routing.mode,'dense');
    verifyTrue(testCase,cfg.execution.return_to_start);
    verifyFalse(testCase,cfg.enable_2opt);
    verifyFalse(testCase,cfg.candidate_augmentation.enable);
    verifyEqual(testCase,cfg.reachability.mode,'nominal');
    verifyEqual(testCase,setup.robot.manipulator.mode,'task_pose_ik');
    verifyFalse(testCase,setup.robot.manipulator.hardware_calibrated);
end
