function mask = rectMask(rows, cols, i0, j0, h, w, theta)
%RECTMASK Rasterize a rectangle whose angle is measured in world coordinates.
%   Columns increase along world +x; rows increase along world -y.
    if nargin < 7
        theta = 0;
    end
    [j, i] = meshgrid(1:cols, 1:rows);
    dx = j - j0;
    dy = i0 - i;
    dxr =  dx * cos(theta) + dy * sin(theta);
    dyr = -dx * sin(theta) + dy * cos(theta);
    mask = abs(dxr) <= w/2 + 1e-12 & abs(dyr) <= h/2 + 1e-12;
end
