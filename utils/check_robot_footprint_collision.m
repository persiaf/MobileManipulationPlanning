function [collision_flag, nearest_obstacle_distance, collision_type] = ...
    check_robot_footprint_collision(robot_pose, footprint, obstacle_map)
%CHECK_ROBOT_FOOTPRINT_COLLISION Validate a pose against a static obstacle map.
%   Supports the project rectangular chassis model and a conservative circle
%   approximation.  OBSTACLE_MAP may be a scene struct or a struct with
%   grid_map/map_info fields.  The function is intentionally read-only.

    arguments
        robot_pose (1,:) double
        footprint
        obstacle_map
    end
    if numel(robot_pose) < 3
        error('check_robot_footprint_collision:Pose', ...
            'robot_pose must be [x y theta].');
    end
    [grid_map, map_info] = unpack_map(obstacle_map);
    fp = normalize_footprint(footprint);
    pose = robot_pose(1:3);

    switch fp.type
        case 'rectangle'
            [collision_flag, nearest_obstacle_distance] = ...
                rectangle_check(pose, fp.length_m, fp.width_m, grid_map, map_info);
        case 'circle'
            [collision_flag, nearest_obstacle_distance] = ...
                circle_check(pose, fp.radius_m, grid_map, map_info);
        otherwise
            error('check_robot_footprint_collision:Footprint', ...
                'Unsupported footprint type: %s', fp.type);
    end
    if collision_flag
        collision_type = 'obstacle';
    elseif nearest_obstacle_distance <= 0
        collision_type = 'boundary';
    else
        collision_type = 'none';
    end
end

function [grid_map, map_info] = unpack_map(obstacle_map)
    if isfield(obstacle_map, 'grid_map')
        grid_map = obstacle_map.grid_map;
        map_info = obstacle_map.map_info;
    elseif isfield(obstacle_map, 'occupancy')
        grid_map = obstacle_map.occupancy;
        map_info = obstacle_map.map_info;
    else
        error('check_robot_footprint_collision:Map', ...
            'obstacle_map must contain grid_map/map_info or occupancy/map_info.');
    end
    required = {'resolution','height_m','rows','cols'};
    for i = 1:numel(required)
        if ~isfield(map_info, required{i})
            error('check_robot_footprint_collision:MapInfo', ...
                'map_info.%s is required.', required{i});
        end
    end
end

function fp = normalize_footprint(footprint)
    if isnumeric(footprint) && numel(footprint) >= 2
        fp.type = 'rectangle'; fp.length_m = footprint(1); fp.width_m = footprint(2);
        return;
    end
    if ischar(footprint) || isstring(footprint)
        error('check_robot_footprint_collision:Footprint', ...
            'A footprint struct or [length width] is required.');
    end
    if ~isstruct(footprint), error('check_robot_footprint_collision:Footprint', 'Invalid footprint.'); end
    fp.type = lower(char(get_field(footprint, 'type', 'rectangle')));
    switch fp.type
        case 'rectangle'
            fp.length_m = get_field(footprint, 'length_m', get_field(footprint, 'length', NaN));
            fp.width_m = get_field(footprint, 'width_m', get_field(footprint, 'width', NaN));
            if ~all(isfinite([fp.length_m fp.width_m]))
                error('check_robot_footprint_collision:Footprint', 'Rectangle length/width are required.');
            end
        case 'circle'
            fp.radius_m = get_field(footprint, 'radius_m', get_field(footprint, 'radius', NaN));
            if ~isfinite(fp.radius_m), error('check_robot_footprint_collision:Footprint', 'Circle radius is required.'); end
        otherwise
            error('check_robot_footprint_collision:Footprint', 'Unknown footprint type %s.', fp.type);
    end
end

function value = get_field(s, name, fallback)
    if isfield(s, name), value = s.(name); else, value = fallback; end
end

function [flag, distance_m] = rectangle_check(pose, length_m, width_m, map, info)
    cfg.chassis.length = length_m; cfg.chassis.width = width_m;
    flag = check_footprint_collision(pose, map, info, cfg);
    distance_m = footprint_clearance(pose, length_m, width_m, map, info);
end

function [flag, distance_m] = circle_check(pose, radius_m, map, info)
    [ci, cj] = world2grid(pose(1), pose(2), info.resolution, info.height_m, origin(info));
    r = ceil(radius_m / info.resolution) + 1;
    ii = max(1,ci-r):min(info.rows,ci+r); jj = max(1,cj-r):min(info.cols,cj+r);
    flag = false; distance_m = inf;
    for i = ii
        for j = jj
            d = hypot(i-ci,j-cj) * info.resolution;
            distance_m = min(distance_m, d-radius_m);
            if map(i,j) > 0 && d <= radius_m + info.resolution/2, flag = true; end
        end
    end
    if isinf(distance_m), distance_m = radius_m; end
end

function d = footprint_clearance(pose, length_m, width_m, map, info)
    [ci,cj] = world2grid(pose(1),pose(2),info.resolution,info.height_m,origin(info));
    r = ceil(hypot(length_m,width_m)/2/info.resolution)+2;
    ii=max(1,ci-r):min(info.rows,ci+r); jj=max(1,cj-r):min(info.cols,cj+r);
    d=inf; c=cos(pose(3)); s=sin(pose(3));
    for i=ii
        for j=jj
            if map(i,j)<=0, continue; end
            dx=(j-cj)*info.resolution; dy=(i-ci)*info.resolution;
            along=dx*c+dy*s; lateral=-dx*s+dy*c;
            edge=max(abs(along)-length_m/2,0)^2 + max(abs(lateral)-width_m/2,0)^2;
            d=min(d,sqrt(edge));
        end
    end
    if isinf(d), d=inf; end
end

function o = origin(info)
    if isfield(info,'origin_xy'), o=info.origin_xy; else, o=[0 0]; end
end
