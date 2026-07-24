function pose = interpolate_path_by_distance(path, s)
    if isempty(path)
        pose = [0, 0, 0];
        return;
    end
    if size(path, 1) == 1
        if size(path, 2) >= 3
            pose = path(1, 1:3);
        else
            pose = [path(1, 1:2), 0];
        end
        return;
    end

    if s <= 0
        if size(path, 2) >= 3
            pose = path(1, 1:3);
        else
            pose = [path(1, 1:2), 0];
        end
        return;
    end

    cumdist = 0;
    for k = 2:size(path, 1)
        seg_len = norm(path(k, 1:2) - path(k-1, 1:2));
        if s <= cumdist + seg_len
            if seg_len < 1e-9
                if size(path, 2) >= 3
                    pose = path(k, 1:3);
                else
                    pose = [path(k, 1:2), 0];
                end
                return;
            end
            alpha = (s - cumdist) / seg_len;
            x = path(k-1, 1) + alpha * (path(k, 1) - path(k-1, 1));
            y = path(k-1, 2) + alpha * (path(k, 2) - path(k-1, 2));
            theta = atan2(path(k, 2) - path(k-1, 2), path(k, 1) - path(k-1, 1));
            pose = [x, y, theta];
            return;
        end
        cumdist = cumdist + seg_len;
    end

    if size(path, 2) >= 3
        pose = path(end, 1:3);
    else
        dx = path(end, 1) - path(end-1, 1);
        dy = path(end, 2) - path(end-1, 2);
        pose = [path(end, 1:2), atan2(dy, dx)];
    end
end