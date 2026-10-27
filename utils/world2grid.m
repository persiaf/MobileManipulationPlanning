function [gi, gj] = world2grid(wx, wy, resolution, map_height_m, origin_xy)
    if nargin < 5 || isempty(origin_xy)
        origin_xy = [0, 0];
    end
    local_x = wx - origin_xy(1);
    local_y = wy - origin_xy(2);
    scaled_x = snap_near_integer(local_x / resolution);
    scaled_y = snap_near_integer((map_height_m - local_y) / resolution);
    gj = floor(scaled_x) + 1;
    gi = floor(scaled_y) + 1;
end

function value = snap_near_integer(value)
    nearest = round(value);
    tolerance = 4 .* eps(max(abs(value), 1));
    snap = isfinite(value) & abs(value - nearest) <= tolerance;
    value(snap) = nearest(snap);
end
