function tests = test_candidate_parameter_repair
    tests = functiontests(localfunctions);
end

function testFiveAndSixArgumentDefaults(testCase)
    [cfg, robot, info] = fixture();
    a = compute_reachability_field([2 2],0,cfg,robot,info);
    verifyEqual(testCase, a.task_centroid, [2 2]);
    b = compute_reachability_field([2 2],0,cfg,robot,info,[2.1 2]);
    verifyEqual(testCase, b.task_centroid, [2.1 2]);
end

function testSeventhArgumentChangesTaskFacingHeadings(testCase)
    [cfg, robot, info] = fixture();
    tasks = [1.55 1.88; 2.45 2.12];
    a = compute_reachability_field([2 2],0,cfg,robot,info,[2 2],tasks);
    b = compute_reachability_field([2 2],0,cfg,robot,info,[2 2],[]);
    verifyGreaterThan(testCase, size(a.candidates,1), size(b.candidates,1));
    xy = [2.0 2.45];
    rows = a.candidates(vecnorm(a.candidates(:,1:2)-xy,2,2)<1e-9,:);
    expected = atan2(tasks(1,2)-xy(2),tasks(1,1)-xy(1));
    verifyTrue(testCase, any(abs(arrayfun(@wrapToPiLocal,rows(:,3)-expected))<1e-9));
end

function testOrientationSamplingIsExplicitAndValidated(testCase)
    [cfg, robot, info] = fixture();
    a = compute_reachability_field([2 2],0,cfg,robot,info,[2 2]);
    cfg.docking_orientation_step_rad = pi/24;
    b = compute_reachability_field([2 2],0,cfg,robot,info,[2 2]);
    verifyGreaterThan(testCase, size(b.candidates,1), size(a.candidates,1));
    cfg.docking_orientation_step_rad = 0;
    verifyError(testCase, @() compute_reachability_field([2 2],0,cfg,robot,info), ...
        'compute_reachability_field:InvalidOrientationStep');
end

function [cfg, robot, info] = fixture()
    cfg = scene_config();
    cfg.docking_min_distance_m = 0.25;
    cfg.docking_max_distance_m = 0.25;
    cfg.docking_lateral_step_m = 0.15;
    robot = robot_config();
    info = struct('width_m',4,'height_m',4,'resolution',0.05,'rows',80,'cols',80);
end
