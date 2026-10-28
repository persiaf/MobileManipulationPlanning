function report = validate_planned_route(scene, path, poses, order, segments, robot)
%VALIDATE_PLANNED_ROUTE Certify sampled geometry and actual docking endpoints.
    report = struct('valid',false,'path_exists',size(path,1)>=2, ...
        'map_collision_free',false,'footprint_checked',true, ...
        'footprint_collision_free',false,'segments_continuous',false, ...
        'endpoints_match',false,'aggregate_matches_segments',false,'endpoint_max_position_error_m',NaN, ...
        'endpoint_max_yaw_error_rad',NaN,'reason','empty route', ...
        'scope','SE2 resampled footprint on safety-dilated scene map; exact dock endpoints and joined-segment provenance');
    if size(path,1)<2, return; end
    if size(path,2)~=3 || any(~isfinite(path(:))) || isempty(order) || ...
            any(order<1 | order>size(poses,1) | order~=fix(order))
        report.reason='invalid route values or pose indices'; return;
    end
    nav_cfg=struct('robot_cfg',robot,'footprint_aware_navigation',true);
    [safe,dense,nav_report]=validate_navigation_trajectory(path,scene.grid_map,scene.map_info,nav_cfg);
    report.navigation_validation=nav_report;
    [report.map_collision_free,~] = validate_world_path_collision(scene.grid_map,dense,scene.map_info,false);
    report.footprint_collision_free=safe;
    errors=[]; yaw_errors=[]; continuous=true;
    if numel(segments)~=numel(order)-1 || ~isfield(segments,'path')
        report.reason='missing route segment waypoint provenance'; return;
    end
    joined=[];
    for k=1:numel(segments)
        p=segments(k).path;
        if isempty(p) || size(p,2)~=3 || any(~isfinite(p(:)))
            report.reason='empty or malformed route segment'; return;
        end
        if isempty(joined),joined=p;else,joined=[joined;p(2:end,:)];end %#ok<AGROW>
        a=poses(order(k),:); b=poses(order(k+1),:);
        errors=[errors,norm(p(1,1:2)-a(1:2)),norm(p(end,1:2)-b(1:2))]; %#ok<AGROW>
        yaw_errors=[yaw_errors,abs(wrapToPiLocal(p(1,3)-a(3))),abs(wrapToPiLocal(p(end,3)-b(3)))]; %#ok<AGROW>
        if k>1
            previous=segments(k-1).path(end,:);
            continuous=continuous && norm(previous(1:2)-p(1,1:2))<1e-7 && ...
                abs(wrapToPiLocal(previous(3)-p(1,3)))<1e-7;
        end
    end
    report.aggregate_matches_segments=isequal(size(path),size(joined)) && ...
        all(abs(path(:,1:2)-joined(:,1:2))<1e-7,'all') && ...
        all(abs(wrapToPiLocal(path(:,3)-joined(:,3)))<1e-7);
    report.endpoint_max_position_error_m=max(errors);
    report.endpoint_max_yaw_error_rad=max(yaw_errors);
    report.endpoints_match=all(errors<1e-7) && all(yaw_errors<1e-7);
    report.segments_continuous=continuous;
    report.valid=report.map_collision_free && report.footprint_collision_free && ...
        report.endpoints_match && continuous && report.aggregate_matches_segments;
    report.reason='';
    if ~report.valid, report.reason='route failed SE2 geometry, continuity, exact docking endpoints or joined-segment provenance validation'; end
end
