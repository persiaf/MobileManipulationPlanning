function [valid, dense, report] = validate_navigation_trajectory(path, raw_map, map_info, planner_cfg)
%VALIDATE_NAVIGATION_TRAJECTORY Sample SE(2) motion against the original map.
%   Translations are sampled at <= resolution/3 and yaw at <= 5 degrees.
%   This is a geometric holonomic validation, not a dynamics/controller test.
%   In point-only unit mode, exact points and intervening grid segments are
%   checked; footprint mode checks the configured rectangle at each sample.
    report = struct('valid',false,'reason','EMPTY_PATH','footprint_checked',false, ...
        'sample_count',0,'bad_sample_index',0,'translation_step_m',map_info.resolution/3, ...
        'angular_step_rad',pi/36);
    valid=false; dense=zeros(0,3);
    if isempty(path),return;end
    if size(path,2)<2 || any(~isfinite(path(:)))
        report.reason='INVALID_PATH_VALUES';return;
    end
    if size(path,2)<3,path=[path(:,1:2),zeros(size(path,1),1)];else,path=path(:,1:3);end
    use_footprint=true;
    if isfield(planner_cfg,'footprint_aware_navigation'),use_footprint=logical(planner_cfg.footprint_aware_navigation);end
    report.footprint_checked=use_footprint;
    if use_footprint,robot=robot_for_collision(planner_cfg);end
    dense=path(1,:);
    for k=2:size(path,1)
        delta_xy=path(k,1:2)-path(k-1,1:2);
        delta_yaw=wrapToPiLocal(path(k,3)-path(k-1,3));
        n=max([1,ceil(norm(delta_xy)/report.translation_step_m),ceil(abs(delta_yaw)/report.angular_step_rad)]);
        t=(1:n)'/n;
        samples=[path(k-1,1:2)+t*delta_xy,path(k-1,3)+t*delta_yaw];
        samples(end,:)=path(k,:); % Preserve requested yaw representation too.
        dense=[dense;samples]; %#ok<AGROW>
    end
    origin=[0 0];if isfield(map_info,'origin_xy'),origin=map_info.origin_xy;end
    traversable=raw_map==0;
    previous_cell=[];
    for k=1:size(dense,1)
        report.sample_count=k;
        [i,j]=world2grid(dense(k,1),dense(k,2),map_info.resolution,map_info.height_m,origin);
        inside=i>=1 && i<=size(raw_map,1) && j>=1 && j<=size(raw_map,2);
        if use_footprint
            collision=check_footprint_collision(dense(k,:),raw_map,map_info,robot);
        else
            collision=~inside || raw_map(i,j)~=0;
        end
        if collision
            report.bad_sample_index=k;report.reason='POSE_COLLISION';return;
        end
        if ~isempty(previous_cell) && ~collision_free(previous_cell,[i j],traversable)
            report.bad_sample_index=k;report.reason='CENTER_SEGMENT_COLLISION';return;
        end
        previous_cell=[i j];
    end
    valid=true;report.valid=true;report.reason='';
end

function robot=robot_for_collision(cfg)
    if isfield(cfg,'robot_cfg'),robot=cfg.robot_cfg;
    elseif isfield(cfg,'robot'),robot=cfg.robot;
    else,error('validate_navigation_trajectory:MissingRobotConfig','Effective robot config is required.');end
    % Match the footprint used by navigation inflation, including synthetic
    % low-level fixtures that intentionally contain no full robot model.
    if isfield(robot,'footprint')
        fp=robot.footprint;
        if isfield(fp,'length_m'),robot.chassis.length=fp.length_m;
        elseif isfield(fp,'length'),robot.chassis.length=fp.length;end
        if isfield(fp,'width_m'),robot.chassis.width=fp.width_m;
        elseif isfield(fp,'width'),robot.chassis.width=fp.width;end
    end
end
