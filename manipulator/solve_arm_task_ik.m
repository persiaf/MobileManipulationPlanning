function result = solve_arm_task_ik(target, model, options)
%SOLVE_ARM_TASK_IK Bounded deterministic multistart damped numerical IK.
% target.position is arm-local. orientation_mode: none, axis, or full.
% Axis constraints align R*target.tool_axis with target.axis_direction;
% rotation about that direction remains free. Full uses target.rotation.
% A failed finite search is not a proof that no IK solution exists.

    if nargin < 3, options = struct(); end
    settings = model.ik;
    names = fieldnames(options);
    for k = 1:numel(names), settings.(names{k}) = options.(names{k}); end
    validate_settings(settings);
    validateattributes(target.position,{'numeric'}, ...
        {'real','finite','vector','numel',3},mfilename,'target.position');
    target.position = target.position(:);
    if ~isfield(target,'orientation_mode'), target.orientation_mode = 'none'; end
    mode = char(target.orientation_mode);
    if ~ismember(mode,{'none','axis','full'})
        error('solve_arm_task_ik:OrientationMode','Unknown orientation_mode.');
    end
    if strcmp(mode,'axis')
        target.tool_axis = unit_vector(target.tool_axis,'tool_axis');
        target.axis_direction = unit_vector(target.axis_direction,'axis_direction');
    elseif strcmp(mode,'full')
        validateattributes(target.rotation,{'numeric'}, ...
            {'real','finite','size',[3,3]},mfilename,'target.rotation');
        if norm(target.rotation'*target.rotation-eye(3),'fro') > 1e-8 || ...
                abs(det(target.rotation)-1) > 1e-8
            error('solve_arm_task_ik:Rotation','target.rotation must be in SO(3).');
        end
    end
    lower = model.joint_limits(:,1); upper = model.joint_limits(:,2);
    seeds = settings.seeds;
    bearing = atan2(target.position(2),target.position(1));
    seeds(:,1) = atan2(sin(seeds(:,1)+bearing),cos(seeds(:,1)+bearing));
    if isfield(settings,'initial_guess') && ~isempty(settings.initial_guess)
        validateattributes(settings.initial_guess,{'numeric'}, ...
            {'real','finite','vector','numel',numel(lower)},mfilename,'initial_guess');
        seeds = [settings.initial_guess(:)';seeds];
    end
    seeds = min(max(seeds,lower'),upper');
    seeds = unique(seeds,'rows','stable');
    seed_count = min(size(seeds,1),settings.max_seeds);
    result = struct('feasible',false,'q',seeds(1,:)', ...
        'position_error_m',Inf,'orientation_error_rad',Inf, ...
        'iterations',0,'seeds_tried',0);
    best_cost = Inf;
    % Triangle inequality bound is conservative for this chain and any tool.
    outer_bound = sum(sqrt(sum(model.joint_origin_xyz.^2,2))) + ...
        norm(model.tool_transform(1:3,4));
    if norm(target.position) > outer_bound + settings.position_tolerance_m
        T = arm_forward_kinematics(result.q,model);
        [~,result.position_error_m,result.orientation_error_rad] = ...
            task_error(T,target,mode,settings.orientation_weight_m);
        return;
    end
    for seed = 1:seed_count
        q = seeds(seed,:)';
        damping = settings.damping;
        result.seeds_tried = seed;
        for iteration = 0:settings.max_iterations
            [T,J] = arm_forward_kinematics(q,model);
            [e,pos_error,angle_error] = task_error(T,target,mode,settings.orientation_weight_m);
            cost = e'*e;
            if cost < best_cost
                best_cost = cost;
                result.q = q;
                result.position_error_m = pos_error;
                result.orientation_error_rad = angle_error;
            end
            if pos_error <= settings.position_tolerance_m && ...
                    angle_error <= settings.orientation_tolerance_rad
                result.feasible = true; result.q = q;
                result.position_error_m = pos_error;
                result.orientation_error_rad = angle_error;
                return;
            end
            if iteration == settings.max_iterations, break; end
            if strcmp(mode,'none')
                A = J(1:3,:);
            elseif strcmp(mode,'axis')
                a = T(1:3,1:3)*target.tool_axis;
                A = [J(1:3,:); settings.orientation_weight_m*(eye(3)-a*a')*J(4:6,:)];
            else
                A = [J(1:3,:);settings.orientation_weight_m*J(4:6,:)];
            end
            step = A'*((A*A'+damping^2*eye(size(A,1)))\e);
            step = step*min(1,settings.max_step_rad/max(norm(step,Inf),eps));
            accepted = false;
            for scale = [1,0.5,0.25,0.125,0.0625]
                trial = min(max(q+scale*step,lower),upper);
                trial_T = arm_forward_kinematics(trial,model);
                trial_e = task_error(trial_T,target,mode,settings.orientation_weight_m);
                if trial_e'*trial_e < cost-1e-14
                    q = trial;
                    damping = max(settings.damping/10,damping*0.7);
                    accepted = true;
                    break;
                end
            end
            result.iterations = result.iterations + 1;
            if ~accepted
                damping = min(1,damping*3);
                if damping >= 1, break; end
            end
        end
    end
end

function [e,position_error,orientation_error] = task_error(T,target,mode,weight)
    dp = target.position-T(1:3,4);
    position_error = norm(dp);
    orientation_error = 0;
    if strcmp(mode,'none'), e = dp; return; end
    if strcmp(mode,'axis')
        a = T(1:3,1:3)*target.tool_axis;
        b = target.axis_direction;
        c = max(-1,min(1,a'*b));
        v = cross(a,b); s = norm(v);
        orientation_error = atan2(s,c);
        if s > 1e-10
            dr = v*(orientation_error/s);
        elseif c < 0
            % Antiparallel axes have pi error, not a zero cross-product error.
            [~,least] = min(abs(a)); reference = zeros(3,1); reference(least)=1;
            v = cross(a,reference); dr = pi*v/norm(v);
        else
            dr = zeros(3,1);
        end
    else
        dr = rotation_log(target.rotation*T(1:3,1:3)');
        orientation_error = norm(dr);
    end
    e = [dp;weight*dr];
end

function v = rotation_log(R)
    c = max(-1,min(1,(trace(R)-1)/2));
    angle = acos(c);
    vee = [R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)];
    if angle < 1e-7
        v = vee/2;
    elseif pi-angle < 1e-5
        % Select the largest diagonal to retain axis signs near pi.
        B = (R+R'+2*eye(3))/4;
        [~,idx] = max(diag(B));
        axis = B(:,idx)/sqrt(max(B(idx,idx),eps));
        axis = axis/norm(axis);
        if axis'*vee < 0, axis = -axis; end
        v = angle*axis;
    else
        v = angle*vee/(2*sin(angle));
    end
end

function v = unit_vector(v,name)
    validateattributes(v,{'numeric'},{'real','finite','vector','numel',3},mfilename,name);
    v = v(:);
    if norm(v) < eps, error('solve_arm_task_ik:Axis','Axis must be nonzero.'); end
    v = v/norm(v);
end

function validate_settings(s)
    for f = {'position_tolerance_m','orientation_tolerance_rad','damping', ...
            'orientation_weight_m','max_step_rad'}
        validateattributes(s.(f{1}),{'numeric'},{'real','finite','scalar','positive'},mfilename,f{1});
    end
    validateattributes(s.max_iterations,{'numeric'}, ...
        {'real','finite','scalar','integer','nonnegative'},mfilename,'max_iterations');
    validateattributes(s.max_seeds,{'numeric'}, ...
        {'real','finite','scalar','integer','positive'},mfilename,'max_seeds');
    validateattributes(s.seeds,{'numeric'},{'real','finite','2d','ncols',6,'nonempty'},mfilename,'seeds');
end
