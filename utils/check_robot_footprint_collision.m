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
            [collision_flag, nearest_obstacle_distance, collision_type] = ...
                rectangle_check(pose, fp.length_m, fp.width_m, grid_map, map_info);
        case 'circle'
            [collision_flag, nearest_obstacle_distance, collision_type] = ...
                circle_check(pose, fp.radius_m, grid_map, map_info);
        otherwise
            error('check_robot_footprint_collision:Footprint', ...
                'Unsupported footprint type: %s', fp.type);
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
    fp.type = lower(char(get_field(footprint, 'type', get_field(footprint,'model','rectangle'))));
    switch fp.type
        case 'rectangle'
            fp.length_m = get_field(footprint, 'length_m', get_field(footprint, 'length', NaN));
            fp.width_m = get_field(footprint, 'width_m', get_field(footprint, 'width', NaN));
            if ~all(isfinite([fp.length_m fp.width_m])) || any([fp.length_m fp.width_m] <= 0)
                error('check_robot_footprint_collision:Footprint', 'Rectangle length/width are required.');
            end
        case 'circle'
            fp.radius_m = get_field(footprint, 'radius_m', get_field(footprint, 'radius', NaN));
            if ~isfinite(fp.radius_m) || fp.radius_m <= 0, error('check_robot_footprint_collision:Footprint', 'Positive circle radius is required.'); end
        otherwise
            error('check_robot_footprint_collision:Footprint', 'Unknown footprint type %s.', fp.type);
    end
end

function value = get_field(s, name, fallback)
    if isfield(s, name), value = s.(name); else, value = fallback; end
end

function [flag, distance_m, kind] = rectangle_check(pose, length_m, width_m, map, info)
    cfg.chassis.length = length_m; cfg.chassis.width = width_m;
    [flag, distance_m, kind] = check_footprint_collision(pose, map, info, cfg);
end

function [flag, distance_m, kind] = circle_check(pose, radius_m, map, info)
    o = origin(info); res = info.resolution;
    width_m = size(map,2)*res;
    if isfield(info,'width_m'), width_m = info.width_m; end
    distance_m = min([pose(1)-o(1),o(1)+width_m-pose(1), ...
        pose(2)-o(2),o(2)+info.height_m-pose(2)]) - radius_m;
    flag = distance_m <= 1e-12; kind = 'none';
    if flag, distance_m = 0; kind = 'boundary'; return; end
    [ii,jj] = find(map>0);
    if isempty(ii), return; end
    x=o(1)+(jj-0.5)*res; y=o(2)+info.height_m-(ii-0.5)*res;
    gap=hypot(max(abs(x-pose(1))-res/2,0),max(abs(y-pose(2))-res/2,0))-radius_m;
    distance_m=max(0,min(distance_m,min(gap)));
    if any(gap <= 1e-12), flag=true; distance_m=0; kind='obstacle'; end
end

function o = origin(info)
    if isfield(info,'origin_xy'), o=info.origin_xy; else, o=[0 0]; end
end
