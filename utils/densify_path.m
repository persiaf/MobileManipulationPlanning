function dense_path = densify_path(path, step_m)
    if nargin < 2
        step_m = 0.05;
    end
    if isempty(path) || size(path, 1) == 0
        if size(path, 2) >= 3
            dense_path = path;
        elseif size(path, 2) == 2
            dense_path = [path, zeros(size(path, 1), 1)];
        else
            dense_path = zeros(0, 3);
        end
        return;
    end
    if size(path, 1) == 1
        if size(path, 2) >= 3
            dense_path = path(1, 1:3);
        else
            dense_path = [path(1, 1:2), 0];
        end
        return;
    end

    if size(path, 2) >= 3
        accum = path(1, 1:3);
    else
        accum = [path(1, 1:2), 0];
    end

    for k = 2:size(path, 1)
        d = norm(path(k, 1:2) - path(k-1, 1:2));
        n_insert = floor(d / step_m);
        if n_insert > 0
            dx = path(k, 1) - path(k-1, 1);
            dy = path(k, 2) - path(k-1, 2);
            ith = atan2(dy, dx);
            for t = 1:n_insert
                alpha = t / (n_insert + 1);
                ix = path(k-1, 1) + alpha * dx;
                iy = path(k-1, 2) + alpha * dy;
                accum = [accum; ix, iy, ith];
            end
        end
        if size(path, 2) >= 3
            accum = [accum; path(k, 1:3)];
        else
            accum = [accum; path(k, 1:2), 0];
        end
    end

    dense_path = accum;
end