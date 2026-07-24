function [global_path, segment_stats] = stitch_global_path(poses, route_order, grid_map, map_info, planner_cfg)
    global_path = [];
    segment_stats = struct('from_id', {}, 'to_id', {}, 'plan_time_s', {}, ...
                           'path_length_m', {}, 'num_points', {}, 'is_return_segment', {});

    for k = 1:length(route_order) - 1
        from_idx = route_order(k);
        to_idx   = route_order(k + 1);

        t_seg = tic;
        segment = run_path_planner(grid_map, poses(from_idx, :), ...
                                    poses(to_idx, :), map_info, planner_cfg);
        plan_time_s = toc(t_seg);

        segment_stats(k).from_id = from_idx;
        segment_stats(k).to_id = to_idx;
        segment_stats(k).plan_time_s = plan_time_s;
        segment_stats(k).path_length_m = compute_path_length(segment);
        segment_stats(k).num_points = size(segment, 1);
        segment_stats(k).is_return_segment = (k == length(route_order) - 1) && (to_idx == 1);

        if isempty(global_path)
            global_path = segment;
        elseif ~isempty(segment)
            global_path = [global_path; segment(2:end, :)]; %#ok<AGROW>
        end
    end

    if ~isempty(global_path)
        global_path = densify_path(global_path, map_info.resolution);
    end
end
