function [ls,rs,cs,arm,details,cache] = ...
        evaluate_task_reachability_cached(pose,task,robot,tau,cache)
%EVALUATE_TASK_REACHABILITY_CACHED Translation buckets with exact FK witnesses.
% Quantization only finds possible entries; it never accepts a pose or an IK
% failure. Positive witnesses are checked against the unquantized new local
% request. Failed arms are reused only for exact raw requests and pure search
% failure reasons. An unknown failed arm is recomputed using the old evaluator.
% Map data are process-local; export cache.stats rather than serializing Map.

    if nargin<4 || isempty(tau),tau=0.3;end
    if nargin<5 || isempty(cache),cache=initialize_cache();end
    if ~isstruct(cache) || ~isfield(cache,'revision') || ...
            ~strcmp(cache.revision,'task_local_translation_witness_v1') || ...
            ~isfield(cache,'buckets') || ~isa(cache.buckets,'containers.Map')
        error('evaluate_task_reachability_cached:Cache','Invalid cache or policy revision.');
    end
    cache.stats.request_count=cache.stats.request_count+1;
    cache.last_event='miss';
    if ~isfield(robot,'manipulator') || ~isfield(robot.manipulator,'mode') || ...
            ~strcmp(robot.manipulator.mode,'task_pose_ik')
        [ls,rs,cs,arm,details]=evaluate_task_reachability(pose,task,robot,tau);
        cache.stats.misses=cache.stats.misses+1;
        return;
    end
    validateattributes(pose,{'numeric'},{'real','finite','vector','numel',3});
    cfg=resolve_manipulation_task(task);
    request=local_request(pose,cfg,robot.manipulator,tau);
    approximate=request;
    approximate.left=quantized(request.left);
    approximate.right=quantized(request.right);
    bytes=getByteStreamFromArray(approximate);
    digest=java.security.MessageDigest.getInstance('SHA-256');
    digest.update(bytes);
    key=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2)',1,[]));
    bucket={};
    if isKey(cache.buckets,key),bucket=cache.buckets(key);end
    fallback=false;
    for entry_index=1:numel(bucket)
        entry=bucket{entry_index};
        % Also compare the full bucket descriptor: a digest collision alone
        % cannot cross models, mounts, budgets or constraint definitions.
        if ~isequaln(approximate,entry.approximate),continue;end
        exact=isequaln(request,entry.request);
        if entry.cs==0
            if exact && pure_negative(entry.details)
                ls=entry.ls;rs=entry.rs;cs=entry.cs;arm=entry.arm;
                details=entry.details;details.constraints=cfg;
                cache.stats.exact_negative_hits=cache.stats.exact_negative_hits+1;
                cache.last_event='exact_negative_hit';
                cache=periodic_direct_check(cache,pose,task,robot,tau,[ls,rs,cs]);
                return;
            end
            continue;
        end
        details=entry.details;details.constraints=cfg;
        for side_cell={'left','right'}
            side=side_cell{1};out=details.(side);
            if out.feasible
                [ok,out]=validate_witness(out,request.(side),cfg,robot.manipulator.model);
                if ~ok,fallback=true;break;end
                details.(side)=out;
            end
        end
        if fallback
            cache.stats.witness_fallbacks=cache.stats.witness_fallbacks+1;
            cache.last_event='witness_fallback';
            break;
        end
        % Never inherit an approximate failure of the other arm merely
        % because one validated positive witness already proves combined=1.
        for side_cell={'left','right'}
            side=side_cell{1};out=details.(side);
            if out.feasible,continue;end
            if ~strcmp(cfg.arm_side,'either') && ~strcmp(cfg.arm_side,side)
                details.(side)=blocked_arm(out,size(cfg.samples_world,1), ...
                    numel(robot.manipulator.model.joint_names));
            elseif ~(exact && allowed_negative(out.reason))
                single_task=task;
                if ~isfield(single_task,'manipulation') || isempty(single_task.manipulation)
                    single_task.manipulation=struct();
                end
                single_task.manipulation.arm_side=side;
                [~,~,~,~,direct]=evaluate_task_reachability(pose,single_task,robot,tau);
                details.(side)=direct.(side);
                cache.stats.arm_direct_checks=cache.stats.arm_direct_checks+1;
            end
        end
        [ls,rs,cs,arm,details]=scores(details);
        cache.stats.positive_hits=cache.stats.positive_hits+1;
        cache.last_event='positive_hit';
        cache=update_maxima(cache,details);
        bucket=insert_entry(bucket,request,approximate,ls,rs,cs,arm,details);
        cache.buckets(key)=bucket;
        cache=periodic_direct_check(cache,pose,task,robot,tau,[ls,rs,cs]);
        return;
    end
    [ls,rs,cs,arm,details]=evaluate_task_reachability(pose,task,robot,tau);
    cache.stats.misses=cache.stats.misses+1;
    cache=update_maxima(cache,details);
    bucket=insert_entry(bucket,request,approximate,ls,rs,cs,arm,details);
    cache.buckets(key)=bucket;
end

function cache=initialize_cache()
    cache.revision='task_local_translation_witness_v1';
    cache.quantization_magnitude=1e-10;
    cache.buckets=containers.Map('KeyType','char','ValueType','any');
    names={'request_count','positive_hits','exact_negative_hits','misses', ...
        'witness_fallbacks','max_position_error_m','max_orientation_error_rad', ...
        'direct_checks','direct_disagreements','arm_direct_checks', ...
        'witness_direct_search_failures'};
    for k=1:numel(names),cache.stats.(names{k})=0;end
    cache.last_event='initialized';
end

function request=local_request(pose,cfg,manipulator,tau)
    request.revision='task_local_translation_witness_v1';
    request.mode='task_pose_ik';request.threshold=tau;
    request.manipulator=manipulator; % Exact model, mounts, tool and IK budget.
    world_fields={'samples_world','waypoints_world','rotation_world', ...
        'axis_direction_world','surface_normal_world','contact_plane_point_world'};
    request.constraints=rmfield(cfg,world_fields);
    R=[cos(pose(3)),-sin(pose(3)),0;sin(pose(3)),cos(pose(3)),0;0,0,1];
    base=[R,[pose(1);pose(2);0];0,0,0,1];
    for side_cell={'left','right'}
        side=side_cell{1};name=[side '_mount_T_base_arm'];
        if ~isfield(manipulator,name)
            error('evaluate_task_reachability:MissingMount','Missing %s.',name);
        end
        mount=manipulator.(name);
        if ~isnumeric(mount) || ~isreal(mount) || ~isequal(size(mount),[4,4]) || ...
                any(~isfinite(mount(:))) || norm(mount(4,:)-[0,0,0,1])>1e-9 || ...
                norm(mount(1:3,1:3)'*mount(1:3,1:3)-eye(3),'fro')>1e-8 || ...
                abs(det(mount(1:3,1:3))-1)>1e-8
            error('evaluate_task_reachability:InvalidMount','Mount must be a rigid transform.');
        end
        frame=base*mount;Ra=frame(1:3,1:3)';origin=frame(1:3,4);
        local.R_arm_world=Ra;
        local.samples=(Ra*(cfg.samples_world'-origin))';
        local.sample_dimensions=size(cfg.samples_world);
        local.waypoints=(Ra*(cfg.waypoints_world'-origin))';
        local.waypoint_dimensions=size(cfg.waypoints_world);
        local.rotation=Ra*cfg.rotation_world;
        local.axis_direction=Ra*cfg.axis_direction_world;
        local.contact_plane_point=Ra*(cfg.contact_plane_point_world-origin);
        local.surface_normal=Ra*cfg.surface_normal_world;
        request.(side)=local;
    end
end

function out=quantized(in)
    out=in;names=fieldnames(in);
    for k=1:numel(names)
        value=in.(names{k});
        if isnumeric(value)
            value=round(value/1e-10)*1e-10;
            value(value==0)=0; % Normalize signed zero only for bucket lookup.
        end
        out.(names{k})=value;
    end
end

function [ok,out]=validate_witness(out,local,cfg,model)
    ok=false;q=out.q_path;n=size(local.samples,1);joints=numel(model.joint_names);
    if ~isnumeric(q) || ~isreal(q) || ~isequal(size(q),[n,joints]) || any(~isfinite(q(:)))
        return;
    end
    if any(any(q<model.joint_limits(:,1)')) || any(any(q>model.joint_limits(:,2)')) || ...
            (n>1 && any(any(abs(diff(q,1,1))>cfg.max_joint_step_rad)))
        return;
    end
    model.tool_transform=model.tool_transform*cfg.T_ee_tcp;
    positions=zeros(n,1);angles=zeros(n,1);
    for k=1:n
        actual=arm_forward_kinematics(q(k,:),model);
        positions(k)=norm(actual(1:3,4)-local.samples(k,:)');
        if strcmp(cfg.orientation_mode,'axis')
            a=actual(1:3,1:3)*cfg.tool_axis;b=local.axis_direction;
            angles(k)=atan2(norm(cross(a,b)),max(-1,min(1,dot(a,b))));
        elseif strcmp(cfg.orientation_mode,'full')
            relative=local.rotation*actual(1:3,1:3)';
            angles(k)=acos(max(-1,min(1,(trace(relative)-1)/2)));
        end
        if positions(k)>cfg.position_tolerance_m || angles(k)>cfg.orientation_tolerance_rad
            return;
        end
        if strcmp(cfg.task_type,'table_wipe')
            height=dot(actual(1:3,4)-local.contact_plane_point,local.surface_normal);
            approach=actual(1:3,1:3)*cfg.tool_axis;
            normal_angle=atan2(norm(cross(approach,-local.surface_normal)), ...
                max(-1,min(1,dot(approach,-local.surface_normal))));
            if abs(height)>cfg.contact_height_tolerance_m || normal_angle>cfg.orientation_tolerance_rad
                return;
            end
        end
    end
    out.position_errors_m=positions;out.orientation_errors_rad=angles;
    out.sample_count=n;out.failed_sample=0;
    out.collision_checked=false;out.continuous_motion_certified=false;
    ok=true;
end

function out=blocked_arm(out,samples,joints)
    out.feasible=false;out.reason='arm_not_allowed';out.failed_sample=0;
    out.sample_count=samples;out.q_path=zeros(0,joints);
    out.position_errors_m=[];out.orientation_errors_rad=[];
end

function answer=allowed_negative(reason)
    answer=any(strcmp(reason,{'ik_not_found','joint_step_exceeded','arm_not_allowed'}));
end

function answer=pure_negative(details)
    answer=~details.left.feasible && ~details.right.feasible && ...
        allowed_negative(details.left.reason) && allowed_negative(details.right.reason);
end

function [ls,rs,cs,arm,details]=scores(details)
    ls=double(details.left.feasible);rs=double(details.right.feasible);cs=max(ls,rs);
    details.ik_feasible=cs>0;
    if cs==0,arm='none';elseif ls>=rs,arm='left';else,arm='right';end
end

function bucket=insert_entry(bucket,request,approximate,ls,rs,cs,arm,details)
    entry=struct('request',request,'approximate',approximate, ...
        'ls',ls,'rs',rs,'cs',cs,'arm',arm,'details',details);
    for k=1:numel(bucket)
        if isequaln(bucket{k}.request,request),bucket{k}=entry;return;end
    end
    bucket{end+1}=entry;
end

function cache=update_maxima(cache,details)
    if ~isfield(details,'ik_checked') || ~details.ik_checked,return;end
    for side_cell={'left','right'}
        out=details.(side_cell{1});
        if out.feasible
            if ~isempty(out.position_errors_m)
                cache.stats.max_position_error_m=max(cache.stats.max_position_error_m,max(out.position_errors_m));
            end
            if ~isempty(out.orientation_errors_rad)
                cache.stats.max_orientation_error_rad=max(cache.stats.max_orientation_error_rad,max(out.orientation_errors_rad));
            end
        end
    end
end

function cache=periodic_direct_check(cache,pose,task,robot,tau,cached)
    hits=cache.stats.positive_hits+cache.stats.exact_negative_hits;
    if mod(hits,100)~=0 || cache.stats.direct_checks>=30,return;end
    [ls,rs,cs]=evaluate_task_reachability(pose,task,robot,tau);
    cache.stats.direct_checks=cache.stats.direct_checks+1;
    if ~isequal(cached>0,[ls,rs,cs]>0)
        cache.stats.direct_disagreements=cache.stats.direct_disagreements+1;
        if cached(3)>0 && cs==0
            % A validated FK witness remains constructive evidence even if a
            % fresh finite local IK search fails. This is a search mismatch,
            % not a failed physical constraint check.
            cache.stats.witness_direct_search_failures=cache.stats.witness_direct_search_failures+1;
        end
    end
end
