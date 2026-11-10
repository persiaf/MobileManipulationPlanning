function tests=test_task_conditioned_manipulation
    tests=functiontests(localfunctions);
end

function testProfilesExpressDifferentTaskRequirements(testCase)
    grasp=manipulation_task_profile('top_down_grasp');
    wipe=manipulation_task_profile('table_wipe');
    verifyEqual(testCase,grasp.orientation_mode,'axis');
    verifyEqual(testCase,wipe.orientation_mode,'full');
    verifyEqual(testCase,grasp.axis_direction_world,[0;0;-1]);
    verifyEqual(testCase,wipe.rotation_world*wipe.tool_axis,[0;0;-1]);
    verifyGreaterThan(testCase,grasp.approach_distance_m,0);
end

function testWipingRequiresPathAndContactPlane(testCase)
    task=struct('x',0.5,'y',0,'z',0.5,'task_type','table_wipe');
    verifyError(testCase,@()resolve_manipulation_task(task), ...
        'resolve_manipulation_task:InvalidConstraint');
    task.manipulation.waypoints_world=[0.5,0,0.5;0.6,0,0.52];
    verifyError(testCase,@()resolve_manipulation_task(task), ...
        'resolve_manipulation_task:InvalidConstraint');
    task.manipulation.waypoints_world=[0.5,0,0.5;0.6,0,0.5];
    cfg=resolve_manipulation_task(task);
    verifyGreaterThan(testCase,size(cfg.samples_world,1),2);
    verifyLessThanOrEqual(testCase,max(vecnorm(diff(cfg.samples_world),2,2)), ...
        cfg.sample_spacing_m+1e-12);
end

function testFKTargetUsesMountAndToolFrames(testCase)
    [robot,task]=reachable_fixture();
    robot.manipulator.left_mount_T_base_arm(1:3,1:3)= ...
        [0,-1,0;1,0,0;0,0,1];
    tool=eye(4); tool(1:3,4)=[0.03;0;0];
    task.manipulation.T_ee_tcp=tool;
    q=[0,-0.6,0.4,0,pi/2+0.2,0];
    model=robot.manipulator.model;
    model.tool_transform=model.tool_transform*tool;
    T=robot.manipulator.left_mount_T_base_arm*arm_forward_kinematics(q,model);
    task.x=T(1,4);task.y=T(2,4);task.z=T(3,4);
    task.manipulation.rotation_world=T(1:3,1:3);
    [ls,rs,score,arm,details]=evaluate_task_reachability([0,0,0],task,robot);
    verifyEqual(testCase,[ls,rs,score],[1,0,1]);
    verifyEqual(testCase,arm,'left');
    verifyTrue(testCase,details.ik_checked);
    verifyFalse(testCase,details.collision_checked);
    actual=robot.manipulator.left_mount_T_base_arm* ...
        arm_forward_kinematics(details.left.q_path(end,:),model);
    verifyLessThanOrEqual(testCase,norm(actual(1:3,4)-T(1:3,4)),0.003);
    % A global base rotation/translation must preserve the local IK problem.
    pose=[0.2,0.3,0.7]; R=[cos(.7),-sin(.7),0;sin(.7),cos(.7),0;0,0,1];
    point=R*T(1:3,4)+[pose(1);pose(2);0];
    task.x=point(1);task.y=point(2);task.z=point(3);
    task.manipulation.rotation_world=R*T(1:3,1:3);
    [~,~,score]=evaluate_task_reachability(pose,task,robot);
    verifyEqual(testCase,score,1);
end

function testReachableCenterCannotAcceptUnreachableWipe(testCase)
    [robot,task]=reachable_fixture();
    [~,~,score]=evaluate_task_reachability([0,0,0],task,robot);
    verifyEqual(testCase,score,1);
    task.task_type='table_wipe';
    task.manipulation.waypoints_world=[task.x,task.y,task.z;task.x+2,task.y,task.z];
    [~,~,score,arm,details]=evaluate_task_reachability([0,0,0],task,robot);
    verifyEqual(testCase,score,0);
    verifyEqual(testCase,arm,'none');
    verifyGreaterThan(testCase,details.left.failed_sample,1);
end

function testSimulationReportsIKEvidenceOnly(testCase)
    [robot,task]=reachable_fixture();
    scene=struct('tasks',task,'num_tasks',1);
    [~,verified,checks,evidence]=simulate_planned_task_execution( ...
        scene,1,[0,0,0],robot,planner_config());
    verifyEqual(testCase,verified,1);
    verifyTrue(testCase,checks.ik_checked);
    verifyTrue(testCase,checks.ik_feasible);
    verifyTrue(testCase,evidence.ik_evidence_available);
    verifyFalse(testCase,evidence.measured_evidence_available);
    verifyFalse(testCase,checks.collision_checked);
    verifyTrue(testCase,isnan(checks.measured_completed));
end

function testActualFKMustSatisfyContactHeight(testCase)
    [robot,task]=reachable_fixture();
    robot.manipulator.model.ik.seeds=[0,-0.6,0.4,0,pi/2+0.2,0];
    robot.manipulator.model.ik.max_seeds=1;
    task.task_type='table_wipe';
    task.z=task.z-0.02;
    task.manipulation.position_tolerance_m=0.03;
    task.manipulation.waypoints_world=[task.x,task.y,task.z;task.x+0.001,task.y,task.z];
    [~,~,score,~,details]=evaluate_task_reachability([0,0,0],task,robot);
    verifyEqual(testCase,score,0);
    verifyEqual(testCase,details.left.reason,'contact_height_exceeded');
end

function testWipingToolFrameMustMatchSurfaceNormal(testCase)
    task=struct('x',0.5,'y',0,'z',0.5,'task_type','table_wipe');
    task.manipulation=struct('tool_axis',[0;0;1], ...
        'waypoints_world',[0.5,0,0.5;0.6,0,0.5]);
    verifyError(testCase,@()resolve_manipulation_task(task), ...
        'resolve_manipulation_task:InvalidConstraint');
    task.manipulation.rotation_world=diag([1,-1,-1]);
    cfg=resolve_manipulation_task(task);
    verifyEqual(testCase,cfg.rotation_world*cfg.tool_axis,[0;0;-1]);
end

function testInvalidFrameAndUnknownConstraintsFailClearly(testCase)
    [robot,task]=reachable_fixture();
    task.manipulation.T_ee_tcp=diag([-1,1,1,1]);
    verifyError(testCase,@()evaluate_task_reachability([0,0,0],task,robot), ...
        'resolve_manipulation_task:InvalidConstraint');
    task.manipulation=struct('invented_option',true);
    verifyError(testCase,@()resolve_manipulation_task(task), ...
        'resolve_manipulation_task:InvalidConstraint');
end

function testJointDiscontinuityRejectsPath(testCase)
    [robot,task]=reachable_fixture();
    task.manipulation.waypoints_world=[task.x,task.y,task.z;task.x+0.06,task.y,task.z];
    task.manipulation.sample_spacing_m=0.01;
    task.manipulation.max_joint_step_rad=1e-8;
    [~,~,score,~,details]=evaluate_task_reachability([0,0,0],task,robot);
    verifyEqual(testCase,score,0);
    verifyEqual(testCase,details.left.reason,'joint_step_exceeded');
end

function testRobustReachCannotSwitchArmsAcrossOffsets(testCase)
    [robot,task]=reachable_fixture();
    task.y=0;
    task.manipulation.arm_side='either';
    robot.manipulator.left_mount_T_base_arm(2,4)=0.5;
    robot.manipulator.right_mount_T_base_arm(2,4)=-0.5;
    [a,b,c]=evaluate_task_reachability([0,0.5,0],task,robot);
    verifyEqual(testCase,[a,b,c],[0,1,1]);
    [a,b,c]=evaluate_task_reachability([0,-0.5,0],task,robot);
    verifyEqual(testCase,[a,b,c],[1,0,1]);
    error_model=struct('position_offsets_m',[0,0.5;0,-0.5],'yaw_offsets_rad',0);
    result=evaluate_robust_reachability([0,0,0],[task.x,task.y,task.z], ...
        robot,0.3,error_model,task);
    verifyEqual(testCase,result.minimum_score,0);
    verifyEqual(testCase,result.success_fraction,0.5);
    verifyFalse(testCase,result.geometric_only);
end

function testTaskIKFlowsThroughDockingAndNavigation(testCase)
    [scene,robot,planner]=create_test_fixture();
    robot.manipulator=task_manipulation_config(robot);
    planner.robot_cfg=robot;
    scene.tasks=scene.tasks(1);
    scene.num_tasks=1;
    scene.tasks(1).task_type='top_down_grasp';
    % The VX300s needs its own candidate distances, rather than the original
    % small-arm scene's very close 0.25--0.40 m docking band.
    scene.scene_cfg.docking_min_distance_m=0.55;
    scene.scene_cfg.docking_max_distance_m=0.55;
    scene.scene_cfg.docking_distance_step_m=0.2;
    scene.scene_cfg.docking_lateral_step_m=0.9;
    scene.scene_cfg.docking_orientation_step_rad=pi/3;
    task_table_map=assign_tasks_to_tables(scene);
    regions=build_task_regions(scene,task_table_map);
    [poses,~,summary,info,plan]=stage2_docking_planning(regions,scene,robot,planner);
    verifyNotEmpty(testCase,poses);
    [~,path,~,~,~,~,stats]=stage3_global_planning(scene,poses,summary, ...
        task_table_map,robot,planner,info,struct(),plan);
    verifyNotEmpty(testCase,path);
    verifyEqual(testCase,stats.evidence.source,'ik_validation');
    verifyEqual(testCase,stats.evidence.ik_feasible_tasks,1);
    verifyFalse(testCase,stats.evidence.measured_evidence_available);
end

function [robot,task]=reachable_fixture()
    robot=robot_config();
    robot.manipulator=task_manipulation_config(robot);
    q=[0,-0.6,0.4,0,pi/2+0.2,0];
    T=robot.manipulator.left_mount_T_base_arm*arm_forward_kinematics(q,robot.manipulator.model);
    task=struct('x',T(1,4),'y',T(2,4),'z',T(3,4),'task_type','position_only');
    task.manipulation=struct('orientation_mode','full','rotation_world',T(1:3,1:3), ...
        'arm_side','left');
end
