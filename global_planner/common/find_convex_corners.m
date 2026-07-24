function corners = find_convex_corners(traversable)
%FIND_CONVEX_CORNERS Extract traversable convex corner cells around obstacles.
%   traversable=true means the mobile base can pass. A cell is a convex
%   corner when it is diagonal to an obstacle and the two side-adjacent cells
%   in that diagonal direction are also free.

    [rows, cols] = size(traversable);
    corner_mask = false(rows, cols);

    diag_dirs = [-1, -1; -1, 1; 1, -1; 1, 1];
    for i = 2:rows-1
        for j = 2:cols-1
            if traversable(i, j)
                continue;
            end

            for d = 1:size(diag_dirs, 1)
                di = diag_dirs(d, 1);
                dj = diag_dirs(d, 2);
                ci = i + di;
                cj = j + dj;

                if ci < 1 || ci > rows || cj < 1 || cj > cols
                    continue;
                end

                if traversable(ci, cj) && traversable(i + di, j) && traversable(i, j + dj)
                    corner_mask(ci, cj) = true;
                end
            end
        end
    end

    [ci, cj] = find(corner_mask);
    corners = [ci, cj];
end
