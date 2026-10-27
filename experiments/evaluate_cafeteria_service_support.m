function [supported, score, table_side, tangential_distance_m] = evaluate_cafeteria_service_support(candidate, task, scene)
%EVALUATE_CAFETERIA_SERVICE_SUPPORT Cafeteria-only 2-D docking proxy.

    cfg = scene.scene_cfg;
    table_id = task.table_id;
    center = cfg.table_positions(table_id, :);
    theta = cfg.table_orientations(table_id);
    task_local = rot2(-theta) * ([task.x, task.y] - center)';
    if abs(task_local(1)) / (cfg.table.length / 2) > abs(task_local(2)) / (cfg.table.width / 2)
        table_side = 3 + (task_local(1) > 0); % WEST=3, EAST=4
        tangent_task = task_local(2);
    else
        table_side = 1 + (task_local(2) < 0); % NORTH=1, SOUTH=2
        tangent_task = task_local(1);
    end
    candidate_local = rot2(-theta) * (candidate.pose(1:2) - center)';
    if candidate.side_id <= 2
        tangent_candidate = candidate_local(1);
    else
        tangent_candidate = candidate_local(2);
    end
    tangential_distance_m = abs(tangent_candidate - tangent_task);
    side_match = candidate.side_id == table_side;
    yaw_to_center = atan2(center(2) - candidate.pose(2), center(1) - candidate.pose(1));
    faces_table = abs(wrapToPiLocal(candidate.pose(3) - yaw_to_center)) <= pi/2;
    tolerance = cfg.cafeteria.service_tangential_tolerance_m;
    supported = side_match && faces_table && tangential_distance_m <= tolerance;
    score = double(supported) * max(0, 1 - tangential_distance_m / max(tolerance, eps));
end
