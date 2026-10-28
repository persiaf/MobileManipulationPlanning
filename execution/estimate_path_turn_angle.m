function [total_turn_angle_rad, num_turns] = estimate_path_turn_angle(path)
    total_turn_angle_rad = 0;
    num_turns = 0;

    if isempty(path) || size(path, 1) < 2
        return;
    end
    if size(path,2)>=3
        changes=abs(atan2(sin(diff(path(:,3))),cos(diff(path(:,3)))));
        total_turn_angle_rad=sum(changes);
        num_turns=sum(changes>1e-3);
        return;
    end

    pts = path(:, 1:2);
    for k = 2:size(pts, 1)-1
        v1 = pts(k, :) - pts(k-1, :);
        v2 = pts(k+1, :) - pts(k, :);
        if norm(v1) < 1e-9 || norm(v2) < 1e-9
            continue;
        end

        a1 = atan2(v1(2), v1(1));
        a2 = atan2(v2(2), v2(1));
        dtheta = atan2(sin(a2-a1), cos(a2-a1));
        if abs(dtheta) > 1e-3
            total_turn_angle_rad = total_turn_angle_rad + abs(dtheta);
            num_turns = num_turns + 1;
        end
    end
end
