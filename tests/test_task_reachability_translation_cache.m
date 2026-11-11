function tests=test_task_reachability_translation_cache
    tests=functiontests(localfunctions);
end

function testTranslationReusesPositiveAndUpdatesAbsoluteConstraints(testCase)
    [robot,task]=fixture();
    [~,~,score,~,first,cache]=evaluate_task_reachability_cached([0,0,0],task,robot,.3,[]);
    verifyEqual(testCase,score,1);delta=[2.8,2.5,0];
    moved=translate_task(task,delta);
    [ls,rs,score,arm,second,cache]=evaluate_task_reachability_cached(delta,moved,robot,.3,cache);
    verifyEqual(testCase,[ls,rs,score],[1,0,1]);verifyEqual(testCase,arm,'left');
    verifyEqual(testCase,cache.stats.positive_hits,1);verifyEqual(testCase,cache.stats.misses,1);
    verifyEqual(testCase,second.constraints.samples_world,first.constraints.samples_world+delta,'AbsTol',1e-12);
    verifyEqual(testCase,second.constraints.contact_plane_point_world, ...
        first.constraints.contact_plane_point_world+delta','AbsTol',1e-12);
    verifyLessThanOrEqual(testCase,max(second.left.position_errors_m),second.constraints.position_tolerance_m);
end

function testFullPathShapeAndOrientationDoNotAlias(testCase)
    [robot,task]=fixture();
    [~,~,~,~,~,cache]=evaluate_task_reachability_cached([0,0,0],task,robot,.3,[]);
    xyz=[task.x,task.y,task.z];task.manipulation.waypoints_world=[xyz;xyz];
    [~,~,~,~,details,cache]=evaluate_task_reachability_cached([0,0,0],task,robot,.3,cache);
    verifyEqual(testCase,cache.stats.misses,2);verifyEqual(testCase,size(details.left.q_path,1),2);
    a=.05;Rx=[1,0,0;0,cos(a),-sin(a);0,sin(a),cos(a)];
    task.manipulation.rotation_world=task.manipulation.rotation_world*Rx;
    [~,~,~,~,~,cache]=evaluate_task_reachability_cached([0,0,0],task,robot,.3,cache);
    verifyEqual(testCase,cache.stats.misses,3);verifyEqual(testCase,cache.stats.positive_hits,0);
end

function testToolModelAndBudgetChangesMiss(testCase)
    [robot,task]=fixture();
    [~,~,~,~,~,cache]=evaluate_task_reachability_cached([0,0,0],task,robot,.3,[]);
    task.manipulation.T_ee_tcp=eye(4);task.manipulation.T_ee_tcp(1,4)=.001;
    [~,~,~,~,~,cache]=evaluate_task_reachability_cached([0,0,0],task,robot,.3,cache);
    robot.manipulator.model.joint_limits(6,2)=2.5;
    [~,~,~,~,~,cache]=evaluate_task_reachability_cached([0,0,0],task,robot,.3,cache);
    robot.manipulator.model.ik.max_iterations=121;
    [~,~,~,~,~,cache]=evaluate_task_reachability_cached([0,0,0],task,robot,.3,cache);
    verifyEqual(testCase,cache.stats.misses,4);verifyEqual(testCase,cache.stats.positive_hits,0);
end

function testNegativeRequiresExactRawLocalRequest(testCase)
    [robot,task]=fixture();task.x=4;
    [~,~,score,~,~,cache]=evaluate_task_reachability_cached([0,0,0],task,robot,.3,[]);
    verifyEqual(testCase,score,0);
    [~,~,~,~,~,cache]=evaluate_task_reachability_cached([0,0,0],task,robot,.3,cache);
    verifyEqual(testCase,cache.stats.exact_negative_hits,1);
    task.x=task.x+4e-12;
    [~,~,~,~,~,cache]=evaluate_task_reachability_cached([0,0,0],task,robot,.3,cache);
    verifyEqual(testCase,cache.stats.exact_negative_hits,1);
    verifyEqual(testCase,cache.stats.misses,2);
end

function testPositiveDoesNotSmuggleApproximateOtherArmFailure(testCase)
    [robot,task]=fixture();task.manipulation.arm_side='either';
    robot.manipulator.right_mount_T_base_arm(1,4)=10;
    [ls,rs,~,~,~,cache]=evaluate_task_reachability_cached([0,0,0],task,robot,.3,[]);
    verifyEqual(testCase,[ls,rs],[1,0]);
    task.y=task.y+4e-12;
    [ls,rs,score,~,~,cache]=evaluate_task_reachability_cached([0,0,0],task,robot,.3,cache);
    verifyEqual(testCase,[ls,rs,score],[1,0,1]);
    verifyEqual(testCase,cache.stats.positive_hits,1);
    verifyEqual(testCase,cache.stats.arm_direct_checks,1);
    verifyEqual(testCase,cache.stats.exact_negative_hits,0);
end

function testContactFailureIsNotStoredAsReusableNegative(testCase)
    [robot,task]=fixture();task.task_type='table_wipe';
    task.z=task.z-.02;
    task.manipulation.position_tolerance_m=.03;
    xyz=[task.x,task.y,task.z];task.manipulation.waypoints_world=[xyz;xyz+[.001,0,0]];
    [~,~,score,~,details,cache]=evaluate_task_reachability_cached([0,0,0],task,robot,.3,[]);
    verifyEqual(testCase,score,0);verifyEqual(testCase,details.left.reason,'contact_height_exceeded');
    [~,~,~,~,~,cache]=evaluate_task_reachability_cached([0,0,0],task,robot,.3,cache);
    verifyEqual(testCase,cache.stats.exact_negative_hits,0);verifyEqual(testCase,cache.stats.misses,2);
end

function testInvalidCachedJointLimitWitnessFallsBack(testCase)
    [robot,task]=fixture();
    [~,~,~,~,~,cache]=evaluate_task_reachability_cached([0,0,0],task,robot,.3,[]);
    keys=cache.buckets.keys;bucket=cache.buckets(keys{1});
    bucket{1}.details.left.q_path(1,1)=robot.manipulator.model.joint_limits(1,2)+.01;
    cache.buckets(keys{1})=bucket;
    [~,~,score,~,details,cache]=evaluate_task_reachability_cached([0,0,0],task,robot,.3,cache);
    verifyEqual(testCase,score,1);verifyEqual(testCase,cache.stats.witness_fallbacks,1);
    verifyEqual(testCase,cache.last_event,'witness_fallback');
    verifyLessThanOrEqual(testCase,details.left.q_path(:,1),robot.manipulator.model.joint_limits(1,2));
end

function testContactHeightWitnessCheckedBeyondPositionTolerance(testCase)
    [robot,task,q]=fixture();task.task_type='table_wipe';
    xyz=[task.x,task.y,task.z];task.manipulation.waypoints_world=[xyz;xyz+[.001,0,0]];
    task.manipulation.position_tolerance_m=.03;
    [~,~,score,~,~,cache]=evaluate_task_reachability_cached([0,0,0],task,robot,.3,[]);
    verifyEqual(testCase,score,1);
    bad=q;bad(2)=bad(2)+.05;bad(3)=bad(3)-.05;
    T=robot.manipulator.left_mount_T_base_arm*arm_forward_kinematics(bad,robot.manipulator.model);
    verifyLessThan(testCase,norm(T(1:3,4)-xyz'),.03);
    verifyGreaterThan(testCase,abs(T(3,4)-task.z),.003);
    keys=cache.buckets.keys;bucket=cache.buckets(keys{1});
    bucket{1}.details.left.q_path=repmat(bad,size(bucket{1}.details.left.q_path,1),1);
    cache.buckets(keys{1})=bucket;
    [~,~,score,~,~,cache]=evaluate_task_reachability_cached([0,0,0],task,robot,.3,cache);
    verifyEqual(testCase,score,1);verifyEqual(testCase,cache.stats.witness_fallbacks,1);
end

function testJointStepWitnessFallsBackEvenWhenFKPoseUnchanged(testCase)
    [robot,task]=fixture();task.manipulation.orientation_mode='none';
    xyz=[task.x,task.y,task.z];task.manipulation.waypoints_world=[xyz;xyz];
    [~,~,score,~,~,cache]=evaluate_task_reachability_cached([0,0,0],task,robot,.3,[]);
    verifyEqual(testCase,score,1);
    keys=cache.buckets.keys;bucket=cache.buckets(keys{1});
    bucket{1}.details.left.q_path(2,6)=bucket{1}.details.left.q_path(2,6)+1;
    cache.buckets(keys{1})=bucket;
    [~,~,score,~,~,cache]=evaluate_task_reachability_cached([0,0,0],task,robot,.3,cache);
    verifyEqual(testCase,score,1);verifyEqual(testCase,cache.stats.witness_fallbacks,1);
end

function testPeriodicDirectBinaryCheckIsRecorded(testCase)
    [robot,task]=fixture();
    [~,~,~,~,~,cache]=evaluate_task_reachability_cached([0,0,0],task,robot,.3,[]);
    cache.stats.positive_hits=99;
    [~,~,~,~,~,cache]=evaluate_task_reachability_cached([0,0,0],task,robot,.3,cache);
    verifyEqual(testCase,cache.stats.direct_checks,1);verifyEqual(testCase,cache.stats.direct_disagreements,0);
    verifyEqual(testCase,cache.stats.request_count,2);
end

function [robot,task,q]=fixture()
    robot=robot_config();robot.manipulator=task_manipulation_config(robot);
    q=[0,-.6,.4,0,pi/2+.2,0];
    robot.manipulator.model.ik.seeds=q;robot.manipulator.model.ik.max_seeds=1;
    T=robot.manipulator.left_mount_T_base_arm*arm_forward_kinematics(q,robot.manipulator.model);
    task=struct('x',T(1,4),'y',T(2,4),'z',T(3,4),'task_type','position_only');
    task.manipulation=struct('orientation_mode','full','rotation_world',T(1:3,1:3),'arm_side','left');
end

function task=translate_task(task,delta)
    task.x=task.x+delta(1);task.y=task.y+delta(2);task.z=task.z+delta(3);
    if isfield(task.manipulation,'waypoints_world')
        task.manipulation.waypoints_world=task.manipulation.waypoints_world+delta;
    end
end
