function cfg = resolve_manipulation_task(task)
%RESOLVE_MANIPULATION_TASK Validate constraints and sample a Cartesian path.
%   task.manipulation overrides the named profile. Explicit world-frame
%   waypoints are mandatory for wiping; a center point is insufficient.
    task_type = 'position_only';
    if isfield(task,'task_type') && ~isempty(task.task_type)
        task_type = task.task_type;
    end
    cfg = manipulation_task_profile(task_type);
    if isfield(task,'manipulation') && ~isempty(task.manipulation)
        overrides = task.manipulation;
        if ~isstruct(overrides) || ~isscalar(overrides)
            invalid('manipulation must be a scalar struct.');
        end
        names = fieldnames(overrides);
        for k = 1:numel(names)
            if ~isfield(cfg,names{k}) || strcmp(names{k},'task_type')
                invalid(['Unknown manipulation option: ' names{k}]);
            end
            cfg.(names{k}) = overrides.(names{k});
        end
    end
    xyz = [task.x,task.y,task.z];
    if ~isnumeric(xyz) || ~isreal(xyz) || numel(xyz)~=3 || any(~isfinite(xyz))
        invalid('Task xyz must be finite.');
    end
    cfg.contact_plane_point_world=xyz(:);
    if ~any(strcmp(cfg.orientation_mode,{'none','axis','full'}))
        invalid('orientation_mode must be none, axis or full.');
    end
    if ~any(strcmp(cfg.arm_side,{'either','left','right'}))
        invalid('arm_side must be either, left or right.');
    end
    cfg.tool_axis = unit_vector(cfg.tool_axis,'tool_axis');
    cfg.axis_direction_world = unit_vector(cfg.axis_direction_world,'axis_direction_world');
    cfg.surface_normal_world = unit_vector(cfg.surface_normal_world,'surface_normal_world');
    validate_transform(cfg.T_ee_tcp);
    R = cfg.rotation_world;
    if ~isnumeric(R) || ~isreal(R) || ~isequal(size(R),[3,3]) || any(~isfinite(R(:))) || ...
            norm(R'*R-eye(3),'fro')>1e-8 || abs(det(R)-1)>1e-8
        invalid('rotation_world must be a proper 3-by-3 rotation.');
    end
    positive = {'position_tolerance_m','orientation_tolerance_rad', ...
        'sample_spacing_m','max_joint_step_rad','contact_height_tolerance_m'};
    for k=1:numel(positive)
        v=cfg.(positive{k});
        if ~isnumeric(v) || ~isreal(v) || ~isscalar(v) || ~isfinite(v) || v<=0
            invalid([positive{k} ' must be finite and positive.']);
        end
    end
    if cfg.orientation_tolerance_rad>=pi || cfg.position_tolerance_m>0.05
        invalid('Use orientation tolerance < pi and position tolerance <= 0.05 m.');
    end
    if ~isnumeric(cfg.approach_distance_m) || ~isreal(cfg.approach_distance_m) || ...
            ~isscalar(cfg.approach_distance_m) || ~isfinite(cfg.approach_distance_m) || cfg.approach_distance_m<0
        invalid('approach_distance_m must be finite and nonnegative.');
    end
    points=cfg.waypoints_world;
    if ~isnumeric(points) || ~isreal(points) || size(points,2)~=3 || any(~isfinite(points(:)))
        invalid('waypoints_world must be a finite N-by-3 matrix.');
    end
    if strcmp(cfg.task_type,'table_wipe')
        if strcmp(cfg.orientation_mode,'none')
            invalid('table_wipe requires an axis or full orientation constraint.');
        end
        direction=cfg.axis_direction_world;
        if strcmp(cfg.orientation_mode,'full'),direction=R*cfg.tool_axis;end
        if acos(max(-1,min(1,dot(direction,-cfg.surface_normal_world))))>cfg.orientation_tolerance_rad
            invalid('Wiping tool approach axis must oppose surface_normal_world; configure rotation_world for the tool TCP.');
        end
        if size(points,1)<2 || ~any(vecnorm(diff(points),2,2)>eps)
            invalid('table_wipe needs at least two distinct ordered waypoints_world.');
        end
        heights=(points-xyz)*cfg.surface_normal_world;
        if any(abs(heights)>cfg.contact_height_tolerance_m)
            invalid('Wiping path leaves the contact plane through task xyz.');
        end
    elseif isempty(points)
        points=xyz;
    end
    if cfg.approach_distance_m>0
        direction=cfg.axis_direction_world;
        if strcmp(cfg.orientation_mode,'full'), direction=R*cfg.tool_axis; end
        points=[points(1,:)-cfg.approach_distance_m*direction'; ...
            points; points(end,:)-cfg.approach_distance_m*direction'];
    end
    samples=points(1,:);
    for k=2:size(points,1)
        steps=max(1,ceil(norm(points(k,:)-points(k-1,:))/cfg.sample_spacing_m));
        if size(samples,1)+steps>1000, invalid('Path exceeds the 1000-sample budget.'); end
        alpha=(1:steps)'/steps;
        samples=[samples; points(k-1,:)+alpha*(points(k,:)-points(k-1,:))]; %#ok<AGROW>
    end
    cfg.samples_world=samples;
end

function v=unit_vector(v,name)
    if ~isnumeric(v) || ~isreal(v) || numel(v)~=3 || any(~isfinite(v(:))) || norm(v)<eps
        invalid([name ' must be a finite nonzero three-vector.']);
    end
    v=v(:)/norm(v);
end

function validate_transform(T)
    if ~isnumeric(T) || ~isreal(T) || ~isequal(size(T),[4,4]) || any(~isfinite(T(:))) || ...
            norm(T(4,:)-[0,0,0,1])>1e-9 || norm(T(1:3,1:3)'*T(1:3,1:3)-eye(3),'fro')>1e-8 || ...
            abs(det(T(1:3,1:3))-1)>1e-8
        invalid('T_ee_tcp must be a proper homogeneous transform.');
    end
end

function invalid(message)
    error('resolve_manipulation_task:InvalidConstraint','%s',message);
end
