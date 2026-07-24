function [gi, gj] = world2grid(wx, wy, resolution, map_height_m, origin_xy)
    if nargin < 5 || isempty(origin_xy)
        origin_xy = [0, 0];
    end
    local_x = wx - origin_xy(1);
    local_y = wy - origin_xy(2);
    gj = floor(local_x / resolution) + 1;
    gi = floor((map_height_m - local_y) / resolution) + 1;
end
