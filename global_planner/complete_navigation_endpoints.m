function [path, report] = complete_navigation_endpoints(core_path, start_pose, goal_pose, raw_map, map_info, planner_cfg, fixed_heading)
%COMPLETE_NAVIGATION_ENDPOINTS Attach validated exact-pose endpoint motions.
%   Interior grid legs use tangent yaw and in-place turns at grid vertices.
%   Endpoint connectors translate with the requested yaw (holonomic model),
%   and rotate only at the conservatively inflated grid endpoint. Direct,
%   x-first and y-first short connectors are tried against the original map.
%   An optional fixed_heading keeps the grid core at that yaw, for searches
%   on a rectangular configuration-space slice that cannot rotate freely.
    start_pose=as_pose(start_pose);goal_pose=as_pose(goal_pose);
    path=zeros(0,3);
    report=struct('valid',false,'reason','NO_CORE_PATH','requested_poses_valid',false, ...
        'requested_endpoints_matched',false,'footprint_checked',false, ...
        'start_position_error_m',NaN,'goal_position_error_m',NaN, ...
        'start_yaw_error_rad',NaN,'goal_yaw_error_rad',NaN, ...
        'start_connector',struct(),'goal_connector',struct(),'trajectory_validation',struct());
    [ok,~,r]=validate_navigation_trajectory(start_pose,raw_map,map_info,planner_cfg);
    report.footprint_checked=r.footprint_checked;
    if ~ok,report.reason=['START_' r.reason];return;end
    [ok,~,r]=validate_navigation_trajectory(goal_pose,raw_map,map_info,planner_cfg);
    if ~ok,report.reason=['GOAL_' r.reason];return;end
    report.requested_poses_valid=true;
    if isempty(core_path),return;end
    if size(core_path,2)<2 || any(~isfinite(core_path(:)))
        report.reason='INVALID_CORE_PATH';return;
    end
    xy=core_path(:,1:2);
    keep=[true;any(abs(diff(xy,1,1))>1e-12,2)];xy=xy(keep,:);
    max_snap=connector_bound(planner_cfg,map_info.resolution);
    start_indices=find(sqrt(sum((xy-start_pose(1:2)).^2,2))<=max_snap+1e-12)';
    goal_indices=fliplr(find(sqrt(sum((xy-goal_pose(1:2)).^2,2))<=max_snap+1e-12)');
    if isempty(start_indices),report.reason='START_GRID_SNAP_TOO_FAR';return;end
    if isempty(goal_indices),report.reason='GOAL_GRID_SNAP_TOO_FAR';return;end
    % A grid endpoint near a dilated-cell boundary is not automatically safe
    % for every yaw. Trim short terminal portions only when exact raw-map
    % connectors certify a safer existing grid anchor further inside.
    found=false;
    for a=start_indices
        for b=goal_indices
            if b<a,continue;end
            if nargin >= 7 && ~isempty(fixed_heading)
                points=remove_collinear(xy(a:b,:));
                interior=[points,repmat(fixed_heading,size(points,1),1)];
            else
                interior=grid_motion(xy(a:b,:),start_pose(3));
            end
            [prefix,detail]=connector(start_pose,interior(1,:),true,max_snap,raw_map,map_info,planner_cfg);
            report.start_connector=detail;
            if ~detail.valid,report.reason=['START_' detail.reason];continue;end
            [suffix,detail]=connector(interior(end,:),goal_pose,false,max_snap,raw_map,map_info,planner_cfg);
            report.goal_connector=detail;
            if ~detail.valid,report.reason=['GOAL_' detail.reason];continue;end
            coarse=[prefix;interior;suffix];
            if size(coarse,1)<2,coarse=[start_pose;goal_pose];end
            [ok,dense,validation]=validate_navigation_trajectory(coarse,raw_map,map_info,planner_cfg);
            report.trajectory_validation=validation;
            if ~ok,report.reason=['FINAL_' validation.reason];continue;end
            report.core_start_index=a;report.core_goal_index=b;
            found=true;break;
        end
        if found,break;end
    end
    if ~found,return;end
    dense(1,:)=start_pose;dense(end,:)=goal_pose;
    report.start_position_error_m=norm(dense(1,1:2)-start_pose(1:2));
    report.goal_position_error_m=norm(dense(end,1:2)-goal_pose(1:2));
    report.start_yaw_error_rad=abs(wrapToPiLocal(dense(1,3)-start_pose(3)));
    report.goal_yaw_error_rad=abs(wrapToPiLocal(dense(end,3)-goal_pose(3)));
    report.requested_endpoints_matched=max([report.start_position_error_m,report.goal_position_error_m, ...
        report.start_yaw_error_rad,report.goal_yaw_error_rad])<=1e-12;
    report.valid=report.requested_endpoints_matched;
    report.reason='';path=dense;
end

function interior=grid_motion(xy,initial_yaw)
    xy=remove_collinear(xy);
    if size(xy,1)==1,interior=[xy,initial_yaw];return;end
    heading=atan2(xy(2,2)-xy(1,2),xy(2,1)-xy(1,1));
    interior=[xy(1,:),heading];
    for k=2:size(xy,1)
        heading=atan2(xy(k,2)-xy(k-1,2),xy(k,1)-xy(k-1,1));
        interior=[interior;xy(k-1,:),heading;xy(k,:),heading]; %#ok<AGROW>
    end
end

function [path,detail]=connector(a,b,is_start,max_snap,map,info,cfg)
    path=zeros(0,3);
    detail=struct('valid',false,'reason','NO_SAFE_CONNECTOR','variant','', ...
        'snap_distance_m',norm(b(1:2)-a(1:2)),'maximum_snap_distance_m',max_snap, ...
        'path_length_m',NaN,'sample_count',0);
    if detail.snap_distance_m>max_snap+1e-12,detail.reason='GRID_SNAP_TOO_FAR';return;end
    variants={'direct','x_then_y','y_then_x'};
    for k=1:numel(variants)
        if is_start,yaw=a(3);else,yaw=b(3);end
        points=[a(1:2);b(1:2)];
        if k==2,points=[a(1:2);b(1),a(2);b(1:2)];
        elseif k==3,points=[a(1:2);a(1),b(2);b(1:2)];end
        translation=[points,repmat(yaw,size(points,1),1)];
        if is_start,candidate=[translation;b];else,candidate=[a;translation];end
        [valid,dense,r]=validate_navigation_trajectory(candidate,map,info,cfg);
        if valid
            path=candidate;detail.valid=true;detail.reason='';detail.variant=variants{k};
            detail.path_length_m=compute_path_length(dense);detail.sample_count=r.sample_count;
            return;
        end
    end
end

function d=connector_bound(cfg,res)
    d=2*res;
    enabled=~isfield(cfg,'footprint_aware_navigation') || cfg.footprint_aware_navigation;
    if ~enabled,return;end
    if isfield(cfg,'robot_cfg'),robot=cfg.robot_cfg;else,robot=cfg.robot;end
    if isfield(robot,'footprint')
        fp=robot.footprint;
        if isfield(fp,'length_m'),L=fp.length_m;else,L=fp.length;end
        if isfield(fp,'width_m'),W=fp.width_m;else,W=fp.width;end
    else,L=robot.chassis.length;W=robot.chassis.width;end
    d=max(d,hypot(L,W)/2+2*res);
end

function p=as_pose(p)
    p=p(:)';
    if numel(p)<2 || numel(p)>3 || any(~isfinite(p))
        error('complete_navigation_endpoints:Pose','Endpoints must be finite [x y] or [x y theta].');
    end
    if numel(p)==2,p(3)=0;end
end

function xy=remove_collinear(xy)
    if size(xy,1)<3,return;end
    keep=true(size(xy,1),1);
    for k=2:size(xy,1)-1
        a=xy(k,:)-xy(k-1,:);b=xy(k+1,:)-xy(k,:);
        if abs(a(1)*b(2)-a(2)*b(1))<=1e-12 && dot(a,b)>0,keep(k)=false;end
    end
    xy=xy(keep,:);
end
