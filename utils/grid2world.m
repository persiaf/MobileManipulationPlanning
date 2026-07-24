function [wx, wy] = grid2world(gi, gj, resolution, map_height_m, origin_xy)
    if nargin < 5 || isempty(origin_xy)
        origin_xy = [0, 0];
    end
    wx = origin_xy(1) + (gj - 0.5) * resolution;
    wy = origin_xy(2) + map_height_m - (gi - 0.5) * resolution;
end
