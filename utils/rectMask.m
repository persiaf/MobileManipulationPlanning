function mask = rectMask(rows, cols, i0, j0, h, w, theta)
    if nargin < 7
        theta = 0;
    end
    mask = false(rows, cols);
    for i = 1:rows
        for j = 1:cols
            dx = j - j0;
            dy = i - i0;
            dxr =  dx * cos(theta) + dy * sin(theta);
            dyr = -dx * sin(theta) + dy * cos(theta);
            if abs(dxr) <= w/2 && abs(dyr) <= h/2
                mask(i, j) = true;
            end
        end
    end
end