function reach_field = compute_reachability_field(table_center, table_theta, ...
        scene_cfg, robot_cfg, map_info, task_centroid, task_positions)
    if nargin < 7 || isempty(task_centroid)
        task_centroid = table_center;
    end
    if nargin < 8
        task_positions = [];
    end

    L = scene_cfg.table.length;
    W = scene_cfg.table.width;

    d_min = scene_cfg.docking_min_distance_m;
    d_max = scene_cfg.docking_max_distance_m;
    d_step = scene_cfg.docking_distance_step_m;
    lat_step = scene_cfg.docking_lateral_step_m;

    distances = d_min : d_step : d_max;

    half_L = L / 2;
    half_W = W / 2;
    lateral_range = half_L + 0.3;
    lateral_samples = -lateral_range : lat_step : lateral_range;

    R_mat = rot2(table_theta);

    % raw_positions = [wx, wy, side_id, normal_theta]
    raw_positions = [];

    for di = 1:length(distances)
        d = distances(di);
        for li = 1:length(lateral_samples)
            lat = lateral_samples(li);
            for side = 1:4
                switch side
                    case 1       % 桌子 +Y 侧
                        local_x = lat;
                        local_y =  half_W + d;
                        normal_local = [0; -1];
                    case 2       % 桌子 -Y 侧
                        local_x = lat;
                        local_y = -half_W - d;
                        normal_local = [0; 1];
                    case 3       % 桌子 -X 侧
                        local_x = -half_L - d;
                        local_y = lat;
                        normal_local = [1; 0];
                    case 4       % 桌子 +X 侧
                        local_x =  half_L + d;
                        local_y = lat;
                        normal_local = [-1; 0];
                end

                if side == 1 || side == 2
                    if abs(lat) > half_L
                        continue;
                    end
                else
                    % 侧边停靠时沿桌宽方向允许覆盖整个宽度，并略向外扩一点，
                    % 避免桌角任务因候选点太少而不可达。
                    if abs(lat) > half_W + 0.20
                        continue;
                    end
                end

                world_pos = R_mat * [local_x; local_y];
                wx = table_center(1) + world_pos(1);
                wy = table_center(2) + world_pos(2);

                if wx < 0.1 || wx > map_info.width_m - 0.1 || ...
                   wy < 0.1 || wy > map_info.height_m - 0.1
                    continue;
                end

                normal_world = R_mat * normal_local;
                normal_theta = atan2(normal_world(2), normal_world(1));
                raw_positions = [raw_positions; wx, wy, side, normal_theta];
            end
        end
    end

    % 候选朝向不再只看任务质心。对每个候选位置，同时考虑：
    % 1) 面向任务质心；2) 面向每个具体任务点；3) 面向桌子中心/侧边法向。
    % 这样可避免桌角任务被“质心朝向”误杀。
    angle_offsets = [-pi/6, -pi/12, 0, pi/12, pi/6];
    candidates = zeros(size(raw_positions, 1) * max(1, (2 + size(task_positions, 1))) * length(angle_offsets), 4);
    idx = 0;

    for k = 1:size(raw_positions, 1)
        wx = raw_positions(k, 1);
        wy = raw_positions(k, 2);
        side_id = raw_positions(k, 3);
        side_theta = raw_positions(k, 4);

        targets = [task_centroid; task_positions; table_center];
        base_angles = side_theta;
        for ti = 1:size(targets, 1)
            dx = targets(ti, 1) - wx;
            dy = targets(ti, 2) - wy;
            if hypot(dx, dy) > 1e-6
                base_angles = [base_angles, atan2(dy, dx)];
            end
        end
        base_angles = unique_angle_list(base_angles, deg2rad(4));

        for ai = 1:length(base_angles)
            for oi = 1:length(angle_offsets)
                theta_var = wrapToPiLocal(base_angles(ai) + angle_offsets(oi));
                idx = idx + 1;
                candidates(idx, :) = [wx, wy, theta_var, side_id];
            end
        end
    end

    candidates = candidates(1:idx, :);
    candidates = unique_pose_rows(candidates, 0.01, deg2rad(2));

    reach_field.candidates = candidates(:, 1:3);
    reach_field.side_ids = candidates(:, 4);
    reach_field.table_center = table_center;
    reach_field.table_theta  = table_theta;
    reach_field.task_centroid = task_centroid;
end

function out = unique_angle_list(angles, tol)
    if isempty(angles)
        out = [];
        return;
    end
    angles = arrayfun(@wrapToPiLocal, angles);
    out = [];
    for i = 1:length(angles)
        a = angles(i);
        keep = true;
        for j = 1:length(out)
            if abs(wrapToPiLocal(a - out(j))) < tol
                keep = false;
                break;
            end
        end
        if keep
            out = [out, a];
        end
    end
end

function out = unique_pose_rows(rows_in, pos_tol, ang_tol)
    out = [];
    for i = 1:size(rows_in, 1)
        r = rows_in(i, :);
        keep = true;
        for j = 1:size(out, 1)
            if norm(r(1:2) - out(j, 1:2)) < pos_tol && ...
               abs(wrapToPiLocal(r(3) - out(j, 3))) < ang_tol && ...
               r(4) == out(j, 4)
                keep = false;
                break;
            end
        end
        if keep
            out = [out; r];
        end
    end
end
