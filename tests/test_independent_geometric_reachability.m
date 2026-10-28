function tests = test_independent_geometric_reachability
    tests = functiontests(localfunctions);
end

function testKnownCentralTargetAndHonestScope(testCase)
    robot = robot_config();
    pose = [0,0,0];
    target = [robot.arm_base_mount_x+0.15,robot.dual_arm.base_spacing/2, ...
        robot.arm_base_mount_z+(robot.single_arm.reach_z_min+robot.single_arm.reach_z_max)/2];
    a = independent_geometric_reachability(pose,target,robot,0.30);
    verifyTrue(testCase,a.left.hard_envelope_feasible);
    verifyTrue(testCase,a.left.score_accepted);
    verifyEqual(testCase,a.left.arm_local_xyz,[0.15,0,0.096],'AbsTol',1e-12);
    verifyFalse(testCase,a.ik_verified);
    verifyTrue(testCase,all(isnan(a.ik_feasible)));
    verifyFalse(testCase,a.collision_checked);
    verifyEqual(testCase,a.validation_kind,'geometric_model_cross_check');
end

function testHardEnvelopeAndScoreAcceptanceAreSeparate(testCase)
    robot = robot_config();
    target = [robot.arm_base_mount_x+0.27,robot.dual_arm.base_spacing/2, ...
        robot.arm_base_mount_z+robot.single_arm.reach_z_max-0.001];
    a = independent_geometric_reachability([0,0,0],target,robot,0.99);
    verifyTrue(testCase,a.left.hard_envelope_feasible);
    verifyFalse(testCase,a.left.score_accepted);
    verifyEqual(testCase,a.left.reason,{'score_below_acceptance'});
end

function testIndependentKnownHardBoundaries(testCase)
    robot = robot_config(); pose = [0,0,0];
    x0 = robot.arm_base_mount_x; y0 = robot.dual_arm.base_spacing/2;
    zmid = robot.arm_base_mount_z+0.096;
    targets = [x0+0.009,y0,zmid; x0+0.279,y0,zmid; ...
        x0-0.198,y0,zmid; x0+0.15,y0,robot.arm_base_mount_z+0.335; ...
        x0+0.15,y0,robot.arm_base_mount_z-0.143];
    a = independent_geometric_reachability(pose,targets,robot,0.30);
    verifyFalse(testCase,any(a.left.hard_envelope_feasible));
    verifyEqual(testCase,a.left.reason,{'inner_radial';'outer_radial'; ...
        'backward';'upper_height';'lower_height'});
end

function testRotationTranslationInvariantLocalGeometry(testCase)
    robot = robot_config();
    local = [0.15,0.04,0.12];
    headings = [0;pi/2;-pi/2;pi/4;-pi/4;pi];
    poses = [repmat([2,-1],numel(headings),1),headings];
    xb = robot.arm_base_mount_x+local(1); yb = robot.dual_arm.base_spacing/2+local(2);
    targets = [poses(:,1)+xb*cos(headings)-yb*sin(headings), ...
        poses(:,2)+xb*sin(headings)+yb*cos(headings), ...
        repmat(robot.arm_base_mount_z+local(3),numel(headings),1)];
    a = independent_geometric_reachability(poses,targets,robot,0.30);
    verifyEqual(testCase,a.left.arm_local_xyz,repmat(local,numel(headings),1),'AbsTol',1e-12);
    verifyEqual(testCase,a.left.accepted_score,repmat(a.left.accepted_score(1),numel(headings),1),'AbsTol',1e-12);
end

function testRandomCrossCheckAgainstProduction(testCase)
    robot = robot_config();
    previous = rng; cleanup = onCleanup(@() rng(previous)); %#ok<NASGU>
    rng(20261027,'twister'); n = 750;
    poses = [4*rand(n,2)-2,2*pi*rand(n,1)-pi];
    local = [0.8*rand(n,2)-0.4,0.65*rand(n,1)-0.20];
    targets = [poses(:,1)+local(:,1).*cos(poses(:,3))-local(:,2).*sin(poses(:,3)), ...
        poses(:,2)+local(:,1).*sin(poses(:,3))+local(:,2).*cos(poses(:,3)), ...
        robot.arm_base_mount_z+local(:,3)];
    thresholds = [0,0.25,0.30,0.75];
    for threshold = thresholds
        a = independent_geometric_reachability(poses,targets,robot,threshold);
        scores = zeros(n,3); assignments = cell(n,1);
        for k = 1:n
            [scores(k,1),scores(k,2),scores(k,3),assignments{k}] = ...
                evaluate_dual_arm_reachability(poses(k,:),targets(k,:),robot,threshold);
        end
        verifyEqual(testCase,[a.left.accepted_score,a.right.accepted_score,a.combined_score],scores,'AbsTol',2e-12);
        verifyEqual(testCase,a.arm_assignment,assignments);
    end
end

function testInvalidInputAndBroadcast(testCase)
    robot = robot_config();
    a = independent_geometric_reachability(zeros(3,3),[0.4,0.1,0.5],robot,0.3);
    verifyEqual(testCase,a.pair_count,3);
    verifyError(testCase,@() independent_geometric_reachability(zeros(2,3),zeros(3,3),robot,0.3), ...
        'independent_geometric_reachability:RowCount');
    verifyError(testCase,@() independent_geometric_reachability([0,0,NaN],[0,0,0],robot,0.3), ...
        'independent_geometric_reachability:Input');
    verifyError(testCase,@() independent_geometric_reachability([0,0,0],[0,0,0],robot,1.1), ...
        'independent_geometric_reachability:Threshold');
end

function testOracleDoesNotCallProductionEvaluator(testCase)
    source = fileread(which('independent_geometric_reachability'));
    verifyEmpty(testCase,regexp(source,'evaluate_(single|dual)_arm_reachability\s*\(','once'));
    verifyEmpty(testCase,regexp(source,'rot2\s*\(','once'));
end
