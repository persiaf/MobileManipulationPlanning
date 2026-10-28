function [is_collision, clearance_m, collision_type] = check_footprint_collision(pose, grid_map, map_info, robot_cfg)
%CHECK_FOOTPRINT_COLLISION Exact 2-D rectangle versus occupied cell squares.
%   Robot +x is its length axis; +y is its width axis. World yaw is CCW.
%   Touching an occupied cell or map boundary counts as collision. Optional
%   clearance is the true edge separation, including the map boundary; it is
%   zero on collision. One-output calls only inspect the footprint vicinity.

    if numel(pose) < 3 || any(~isfinite(pose(1:3)))
        error('check_footprint_collision:Pose', 'pose must contain finite [x y theta].');
    end
    res = map_info.resolution;
    half_l = robot_cfg.chassis.length / 2;
    half_w = robot_cfg.chassis.width / 2;
    if ~all(isfinite([res half_l half_w])) || any([res half_l half_w] <= 0)
        error('check_footprint_collision:Geometry', 'Resolution and chassis dimensions must be positive.');
    end
    [rows, cols] = size(grid_map);
    origin = [0 0];
    if isfield(map_info, 'origin_xy'), origin = map_info.origin_xy; end
    width_m = cols * res;
    if isfield(map_info, 'width_m'), width_m = map_info.width_m; end
    height_m = map_info.height_m;
    c = cos(pose(3)); s = sin(pose(3));
    extent_x = half_l * abs(c) + half_w * abs(s);
    extent_y = half_l * abs(s) + half_w * abs(c);
    bounds = [pose(1)-extent_x, pose(1)+extent_x, ...
              pose(2)-extent_y, pose(2)+extent_y];
    boundary_gap = min([bounds(1)-origin(1), origin(1)+width_m-bounds(2), ...
        bounds(3)-origin(2), origin(2)+height_m-bounds(4)]);
    clearance_m = max(0, boundary_gap);
    is_collision = boundary_gap <= 1e-12;
    collision_type = 'none';
    if is_collision
        collision_type = 'boundary';
        return;
    end

    [cx, cy] = occupied_centres(grid_map, bounds, res, height_m, origin, rows, cols);
    dx = cx - pose(1); dy = cy - pose(2); h = res / 2;
    tolerance = 1e-12;
    % Separating-axis theorem: the two map axes and two robot axes suffice.
    hit = abs(dx) <= extent_x+h+tolerance & abs(dy) <= extent_y+h+tolerance & ...
        abs(dx*c+dy*s) <= half_l+h*(abs(c)+abs(s))+tolerance & ...
        abs(-dx*s+dy*c) <= half_w+h*(abs(c)+abs(s))+tolerance;
    if any(hit)
        is_collision = true; clearance_m = 0; collision_type = 'obstacle';
        return;
    end
    if nargout < 2, return; end

    % Cells outside this expanded box cannot be nearer than the boundary.
    search_bounds = bounds + [-clearance_m clearance_m -clearance_m clearance_m];
    [cx, cy] = occupied_centres(grid_map, search_bounds, res, height_m, origin, rows, cols);
    if isempty(cx), return; end
    % For disjoint convex polygons the minimum distance is vertex-to-edge.
    % Check robot vertices against cell edges, then cell vertices against the
    % oriented robot edges. This also accounts for the finite cell square.
    signs = [-1 -1; 1 -1; 1 1; -1 1];
    for k = 1:4
        vx = pose(1) + signs(k,1)*half_l*c - signs(k,2)*half_w*s;
        vy = pose(2) + signs(k,1)*half_l*s + signs(k,2)*half_w*c;
        d = hypot(max(abs(vx-cx)-h,0), max(abs(vy-cy)-h,0));
        clearance_m = min(clearance_m, min(d));
        dx = cx+signs(k,1)*h-pose(1); dy = cy+signs(k,2)*h-pose(2);
        d = hypot(max(abs(dx*c+dy*s)-half_l,0), ...
                  max(abs(-dx*s+dy*c)-half_w,0));
        clearance_m = min(clearance_m, min(d));
    end
end

function [x, y] = occupied_centres(map, bounds, res, height_m, origin, rows, cols)
    % Include touching neighbours and let the exact geometry reject extras.
    j0 = max(1, floor((bounds(1)-origin(1))/res));
    j1 = min(cols, ceil((bounds(2)-origin(1))/res)+1);
    i0 = max(1, floor((origin(2)+height_m-bounds(4))/res));
    i1 = min(rows, ceil((origin(2)+height_m-bounds(3))/res)+1);
    if i0 > i1 || j0 > j1, x = zeros(0,1); y = zeros(0,1); return; end
    [i, j] = find(map(i0:i1,j0:j1) > 0);
    x = origin(1) + (j+j0-1.5)*res;
    y = origin(2) + height_m - (i+i0-1.5)*res;
end
