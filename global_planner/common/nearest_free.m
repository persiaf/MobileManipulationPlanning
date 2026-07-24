function [fi, fj] = nearest_free(grid_map, si, sj)
    rows = size(grid_map, 1);
    cols = size(grid_map, 2);
    for r = 1:max(rows, cols)
        for di = -r:r
            for dj = -r:r
                ni = si + di;
                nj = sj + dj;
                if ni >= 1 && ni <= rows && nj >= 1 && nj <= cols && grid_map(ni, nj) == 0
                    fi = ni;
                    fj = nj;
                    return;
                end
            end
        end
    end
    fi = si;
    fj = sj;
end