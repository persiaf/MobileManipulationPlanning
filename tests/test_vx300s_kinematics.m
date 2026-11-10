function tests = test_vx300s_kinematics
    tests = functiontests(localfunctions);
end

function testVendorZeroPoseAndJointOrder(testCase)
    model = vx300s_kinematic_model();
    [T,J] = arm_forward_kinematics(zeros(6,1),model);
    % Independent vendor POE home transform (mr_descriptions.py, vx300s).
    expected = [eye(3),[0.536494;0;0.42705];0,0,0,1];
    verifyEqual(testCase,T,expected,'AbsTol',1e-12);
    verifyEqual(testCase,model.joint_names, ...
        {'waist','shoulder','elbow','forearm_roll','wrist_angle','wrist_rotate'});
    verifyEqual(testCase,J(4:6,:),[0,0,0,1,0,1;0,1,1,0,1,0;1,0,0,0,0,0], ...
        'AbsTol',1e-12);
    verifyEqual(testCase,model.provenance.commit,'0bb2b0e6d0e619bff02cf74dbd5af5681dcf80c9');
    verifyEqual(testCase,rad2deg(model.joint_limits(2,:)),[-106,72],'AbsTol',1e-12);
end

function testWaistKnownQuarterTurn(testCase)
    T = arm_forward_kinematics([pi/2,0,0,0,0,0],vx300s_kinematic_model());
    verifyEqual(testCase,T(1:3,4),[0;0.536494;0.42705],'AbsTol',1e-12);
    verifyEqual(testCase,T(1:3,1:3),[0,-1,0;1,0,0;0,0,1],'AbsTol',1e-12);
end

function testChainAgainstIndependentVendorPOE(testCase)
    % Vendor screw axes in space coordinates; expm is independent of the
    % production chain's translated-joint Rodrigues implementation.
    S = [0,0,1,0,0,0;0,1,0,-0.12705,0,0; ...
        0,1,0,-0.42705,0,0.05955;1,0,0,0,0.42705,0; ...
        0,1,0,-0.42705,0,0.35955;1,0,0,0,0.42705,0]';
    M = [eye(3),[0.536494;0;0.42705];0,0,0,1];
    qs = [0.4,-0.6,0.8,0.3,-0.5,0.7;-1.2,0.3,-0.9,-1.1,1.2,-0.4];
    for row=1:size(qs,1)
        expected=eye(4);
        for k=1:6
            w=S(1:3,k); twist=zeros(4);
            twist(1:3,1:3)=[0,-w(3),w(2);w(3),0,-w(1);-w(2),w(1),0];
            twist(1:3,4)=S(4:6,k);
            expected=expected*expm(twist*qs(row,k));
        end
        expected=expected*M;
        actual=arm_forward_kinematics(qs(row,:),vx300s_kinematic_model());
        verifyEqual(testCase,actual,expected,'AbsTol',1e-12);
    end
end

function testJacobianFiniteDifferencesAndToolOffset(testCase)
    model = vx300s_kinematic_model();
    model.tool_transform(1:3,4) = [0.15;0.03;0.02];
    q = [0.4;-0.6;0.8;0.3;-0.5;0.7]; h=1e-7;
    [T,J] = arm_forward_kinematics(q,model);
    for k=1:6
        dq=zeros(6,1);dq(k)=h;
        Tp=arm_forward_kinematics(q+dq,model);
        Tm=arm_forward_kinematics(q-dq,model);
        dR=(Tp(1:3,1:3)-Tm(1:3,1:3))/(2*h)*T(1:3,1:3)';
        numerical=[(Tp(1:3,4)-Tm(1:3,4))/(2*h);dR(3,2);dR(1,3);dR(2,1)];
        verifyEqual(testCase,J(:,k),numerical,'AbsTol',2e-8);
    end
end

function testFullPoseIKFromMultipleKnownReachablePoses(testCase)
    model = vx300s_kinematic_model();
    qs = [0,-0.7,0.9,0,0.4,0;0.7,-0.5,-0.6,0.5,1.0,-0.4; ...
        -0.8,-1.0,1.1,-0.7,0.5,0.8];
    for k=1:size(qs,1)
        T = arm_forward_kinematics(qs(k,:),model);
        target = struct('position',T(1:3,4),'rotation',T(1:3,1:3), ...
            'orientation_mode','full');
        result = solve_arm_task_ik(target,model,struct());
        verifyTrue(testCase,result.feasible);
        verifyLessThanOrEqual(testCase,result.position_error_m,model.ik.position_tolerance_m);
        verifyLessThanOrEqual(testCase,result.orientation_error_rad,model.ik.orientation_tolerance_rad);
        verifyGreaterThanOrEqual(testCase,result.q,model.joint_limits(:,1)-1e-12);
        verifyLessThanOrEqual(testCase,result.q,model.joint_limits(:,2)+1e-12);
    end
end

function testAxisConstraintAndWarmStart(testCase)
    model = vx300s_kinematic_model();
    q = [0.3;-0.8;0.9;0.2;0.5;-0.2];
    T = arm_forward_kinematics(q,model);
    target = struct('position',T(1:3,4),'orientation_mode','axis', ...
        'tool_axis',[1;0;0],'axis_direction',T(1:3,1));
    first = solve_arm_task_ik(target,model,struct('initial_guess',q));
    verifyTrue(testCase,first.feasible); verifyEqual(testCase,first.iterations,0);
    verifyEqual(testCase,first.q,q,'AbsTol',1e-12);
    target.position = target.position + [0.003;0;0];
    next = solve_arm_task_ik(target,model,struct('initial_guess',first.q,'max_seeds',1));
    verifyTrue(testCase,next.feasible);
    verifyLessThan(testCase,norm(next.q-first.q),0.15);
end

function testAntiparallelAxisCannotPassAsZeroError(testCase)
    model = vx300s_kinematic_model();
    T = arm_forward_kinematics(zeros(6,1),model);
    target = struct('position',T(1:3,4),'orientation_mode','axis', ...
        'tool_axis',[1;0;0],'axis_direction',[-1;0;0]);
    r = solve_arm_task_ik(target,model,struct('initial_guess',zeros(6,1), ...
        'max_seeds',1,'max_iterations',0));
    verifyFalse(testCase,r.feasible);
    verifyEqual(testCase,r.orientation_error_rad,pi,'AbsTol',1e-12);
end

function testFullRotationPiCannotPassAsZeroError(testCase)
    model = vx300s_kinematic_model();
    T = arm_forward_kinematics(zeros(6,1),model);
    target = struct('position',T(1:3,4),'orientation_mode','full', ...
        'rotation',diag([1,-1,-1]));
    r = solve_arm_task_ik(target,model,struct('initial_guess',zeros(6,1), ...
        'max_seeds',1,'max_iterations',0));
    verifyFalse(testCase,r.feasible);
    verifyEqual(testCase,r.orientation_error_rad,pi,'AbsTol',1e-12);
end

function testUnreachableAndDeterministicWithoutRngChanges(testCase)
    model = vx300s_kinematic_model(); previous = rng;
    target = struct('position',[3;0;0],'orientation_mode','none');
    a = solve_arm_task_ik(target,model,struct());
    b = solve_arm_task_ik(target,model,struct());
    verifyFalse(testCase,a.feasible);verifyEqual(testCase,a,b);
    verifyEqual(testCase,rng,previous);
end
