function [left_score,right_score,combined_score,arm_assignment,details] = ...
        evaluate_task_reachability(pose,task,robot_cfg,threshold)
%EVALUATE_TASK_REACHABILITY Legacy xyz or task-conditioned sampled-path IK.
%   Success means a numerical IK sequence was found within joint/pose limits.
%   It does not certify collisions, between-sample motion or contact forces.
    if nargin<4 || isempty(threshold), threshold=0.3; end
    validateattributes(pose,{'numeric'},{'real','finite','vector','numel',3});
    mode='legacy_xyz';
    if isfield(robot_cfg,'manipulator') && isfield(robot_cfg.manipulator,'mode')
        mode=char(robot_cfg.manipulator.mode);
    end
    details=struct('mode',mode,'ik_checked',false,'ik_feasible',NaN, ...
        'collision_checked',false,'measured_completed',NaN,'left',[],'right',[]);
    if strcmp(mode,'legacy_xyz')
        explicit_task=isfield(task,'task_type') && ...
            any(strcmpi(task.task_type,{'top_down_grasp','table_wipe'}));
        explicit_options=isfield(task,'manipulation') && ~isempty(task.manipulation) && ...
            (~isstruct(task.manipulation) || ~isscalar(task.manipulation) || ...
            ~isempty(fieldnames(task.manipulation)));
        if explicit_task || explicit_options
            error('evaluate_task_reachability:TaskModeRequired', ...
                'Task constraints require robot.manipulator.mode=task_pose_ik.');
        end
        [left_score,right_score,combined_score,arm_assignment]= ...
            evaluate_dual_arm_reachability(pose,[task.x,task.y,task.z],robot_cfg,threshold);
        return;
    end
    if ~strcmp(mode,'task_pose_ik')
        error('evaluate_task_reachability:UnknownMode','Unknown mode "%s".',mode);
    end
    if ~isfield(robot_cfg.manipulator,'model')
        error('evaluate_task_reachability:MissingModel','Configure a complete kinematic model.');
    end
    cfg=resolve_manipulation_task(task);
    details.ik_checked=true;
    details.constraints=cfg;
    details.left=evaluate_arm(pose,cfg,robot_cfg.manipulator,'left');
    details.right=evaluate_arm(pose,cfg,robot_cfg.manipulator,'right');
    left_score=double(details.left.feasible);
    right_score=double(details.right.feasible);
    combined_score=max(left_score,right_score);
    details.ik_feasible=combined_score>0;
    if combined_score==0
        arm_assignment='none';
    elseif left_score>=right_score
        arm_assignment='left';
    else
        arm_assignment='right';
    end
end

function out=evaluate_arm(pose,cfg,manipulator,side)
    model=manipulator.model;
    n=numel(model.joint_names);
    out=struct('feasible',false,'reason','ik_not_found','failed_sample',0, ...
        'sample_count',size(cfg.samples_world,1),'q_path',zeros(0,n), ...
        'position_errors_m',[],'orientation_errors_rad',[], ...
        'collision_checked',false,'continuous_motion_certified',false);
    if ~strcmp(cfg.arm_side,'either') && ~strcmp(cfg.arm_side,side)
        out.reason='arm_not_allowed'; return;
    end
    mount_name=[side '_mount_T_base_arm'];
    if ~isfield(manipulator,mount_name)
        error('evaluate_task_reachability:MissingMount','Missing %s.',mount_name);
    end
    mount=manipulator.(mount_name);
    if ~isnumeric(mount) || ~isreal(mount) || ~isequal(size(mount),[4,4]) || any(~isfinite(mount(:))) || ...
            norm(mount(4,:)-[0,0,0,1])>1e-9 || ...
            norm(mount(1:3,1:3)'*mount(1:3,1:3)-eye(3),'fro')>1e-8 || ...
            abs(det(mount(1:3,1:3))-1)>1e-8
        error('evaluate_task_reachability:InvalidMount','Mount must be a rigid transform.');
    end
    R=[cos(pose(3)),-sin(pose(3)),0;sin(pose(3)),cos(pose(3)),0;0,0,1];
    T_world_base=[R,[pose(1);pose(2);0];0,0,0,1];
    T_world_arm=T_world_base*mount;
    R_arm_world=T_world_arm(1:3,1:3)';
    model.tool_transform=model.tool_transform*cfg.T_ee_tcp;
    target=struct('position',[],'rotation',R_arm_world*cfg.rotation_world, ...
        'orientation_mode',cfg.orientation_mode,'tool_axis',cfg.tool_axis, ...
        'axis_direction',R_arm_world*cfg.axis_direction_world);
    options=struct('position_tolerance_m',cfg.position_tolerance_m, ...
        'orientation_tolerance_rad',cfg.orientation_tolerance_rad);
    for k=1:size(cfg.samples_world,1)
        target.position=R_arm_world*(cfg.samples_world(k,:)'-T_world_arm(1:3,4));
        if k>1
            options.initial_guess=out.q_path(end,:);
            options.max_seeds=1;
        end
        result=solve_arm_task_ik(target,model,options);
        if ~result.feasible
            out.failed_sample=k; return;
        end
        if strcmp(cfg.task_type,'table_wipe')
            actual=T_world_arm*arm_forward_kinematics(result.q,model);
            height=dot(actual(1:3,4)-cfg.contact_plane_point_world,cfg.surface_normal_world);
            if abs(height)>cfg.contact_height_tolerance_m
                out.reason='contact_height_exceeded';out.failed_sample=k;return;
            end
            approach=actual(1:3,1:3)*cfg.tool_axis;
            tilt=acos(max(-1,min(1,dot(approach,-cfg.surface_normal_world))));
            if tilt>cfg.orientation_tolerance_rad
                out.reason='contact_normal_exceeded';out.failed_sample=k;return;
            end
        end
        if k>1 && max(abs(result.q(:)'-out.q_path(end,:)))>cfg.max_joint_step_rad
            out.reason='joint_step_exceeded'; out.failed_sample=k; return;
        end
        out.q_path(end+1,:)=result.q(:)';
        out.position_errors_m(end+1,1)=result.position_error_m;
        out.orientation_errors_rad(end+1,1)=result.orientation_error_rad;
    end
    out.feasible=true;
    out.reason='sampled_path_ik_feasible';
end
