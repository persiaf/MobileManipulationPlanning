function [path, report] = run_fixed_heading_astar(raw_map, start_pose, goal_pose, map_info, planner_cfg)
%RUN_FIXED_HEADING_ASTAR Recover narrow passages for a rectangular base.
%   A circumscribed disk can disconnect passages that the actual rectangle
%   can traverse. Try a small deterministic set of fixed-yaw configuration
%   spaces, then certify every translation, rotation and exact endpoint on
%   the raw map. This uses the same holonomic motion model as normal routing;
%   it is deliberately not a complete SE(2) or nonholonomic search.

    path=zeros(0,3);
    report=struct('valid',false,'reason','NO_FIXED_HEADING_PATH', ...
        'requested_poses_valid',true,'heading_rad',NaN,'attempt_count',0);
    if isfield(planner_cfg,'robot_cfg'),robot=planner_cfg.robot_cfg;
    else,robot=planner_cfg.robot;end
    origin=[0 0];
    if isfield(map_info,'origin_xy'),origin=reshape(map_info.origin_xy,1,2);end
    local_start=start_pose;local_goal=goal_pose;
    local_start(1:2)=start_pose(1:2)-origin;
    local_goal(1:2)=goal_pose(1:2)-origin;
    % Rectangular collision geometry is pi-periodic. Keep a nearby yaw
    % representation so connector interpolation takes the shortest turn.
    headings=[goal_pose(3),start_pose(3),0,pi/2];
    tried=[];
    for yaw=headings
        if any(abs(mod(yaw-tried+pi/2,pi)-pi/2)<1e-10),continue;end
        tried(end+1)=yaw; %#ok<AGROW>
        report.attempt_count=report.attempt_count+1;
        layer=build_footprint_inflated_map(raw_map,robot.footprint,map_info,yaw);
        core=run_astar_wrapper(layer,local_start,local_goal,map_info,planner_cfg);
        if ~grid_core_safe(core,layer,map_info),continue;end
        core(:,1:2)=core(:,1:2)+origin;
        [candidate,detail]=complete_navigation_endpoints( ...
            core,start_pose,goal_pose,raw_map,map_info,planner_cfg,yaw);
        if ~detail.valid,continue;end
        path=candidate;report=detail;
        report.heading_rad=yaw;report.attempt_count=numel(tried);
        return;
    end
end

function valid=grid_core_safe(core,layer,info)
    valid=false;
    if isempty(core) || any(~isfinite(core(:))),return;end
    [i,j]=world2grid(core(:,1),core(:,2),info.resolution,info.height_m);
    cells=[i j];free=layer==0;
    for k=1:size(cells,1)
        if ~collision_free(cells(max(1,k-1),:),cells(k,:),free),return;end
    end
    valid=true;
end
