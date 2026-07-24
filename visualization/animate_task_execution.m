function animate_task_execution(scene, global_path, all_poses, visit_order, ...
                                  optimal_poses, task_sequence_summary, executed_tasks, ...
                                  robot_cfg, planner_cfg, stats)
    if nargin < 10 || isempty(stats)
        stats = struct();
    end
    if nargin < 9 || isempty(planner_cfg)
        planner_cfg = planner_config();
    end
    if nargin < 8 || isempty(robot_cfg)
        robot_cfg = robot_config();
    end

    if isempty(global_path) || size(global_path, 1) < 2
        warning('全局路径为空或点数不足，跳过动画生成。');
        return;
    end

    global_path = densify_path(global_path, scene.map_info.resolution);

    hFig = figure('Name', '任务执行动画', 'Position', [60, 60, 1100, 820], 'Color', 'w');
    clf;
    hold on;
    draw_scene_background(scene);
    xlim([0 scene.map_info.width_m]);
    ylim([0 scene.map_info.height_m]);
    xlabel('X (m)', 'FontSize', 11);
    ylabel('Y (m)', 'FontSize', 11);
    axis equal;
    grid on;
    box on;

    media_dir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'docs', 'media');
    if ~isfolder(media_dir)
        mkdir(media_dir);
    end
    video_file = fullfile(media_dir, 'task_execution_demo.mp4');
    vid_writer = VideoWriter(video_file, 'MPEG-4');
    vid_writer.FrameRate = 30;
    vid_writer.Quality = 95;
    open(vid_writer);

    robot_speed = 0.6;
    frame_rate = vid_writer.FrameRate;
    dt = 1 / frame_rate;

    if isfield(stats, 'route_order') && ~isempty(stats.route_order)
        route_order = stats.route_order;
    else
        route_order = visit_order;
    end
    return_to_start = isfield(stats, 'return_to_start') && stats.return_to_start;
    return_path_len_m = 0;
    if isfield(stats, 'exec_stats') && isfield(stats.exec_stats, 'return_path_len_m')
        return_path_len_m = stats.exec_stats.return_path_len_m;
    end

    ordered_poses = all_poses(route_order, :);
    total_pts = size(global_path, 1);
    n_poses = size(ordered_poses, 1);
    n_segments = n_poses - 1;
    n_task_stops = sum(route_order(2:end) > 1);
    segment_colors = lines(max(n_segments, 1));

    task_x = [scene.tasks.x];
    task_y = [scene.tasks.y];

    cum_dist = zeros(total_pts, 1);
    for k = 2:total_pts
        cum_dist(k) = cum_dist(k-1) + norm(global_path(k,1:2) - global_path(k-1,1:2));
    end
    total_path_len = cum_dist(end);
    pose_reached_at = compute_pose_indices_on_path(global_path, ordered_poses);
    pose_reached_s  = cum_dist(pose_reached_at);

    exec_plan = build_execution_plan(n_segments, route_order, pose_reached_at, ...
                                     pose_reached_s, task_sequence_summary);

    h_path_lines = gobjects(n_segments, 1);
    h_path_dashed = gobjects(n_segments, 1);
    for s = 1:n_segments
        h_path_lines(s) = plot(NaN, NaN, '-', 'Visible', 'off');
        h_path_dashed(s) = plot(NaN, NaN, '--', 'Visible', 'off');
    end

    h_pose_static = gobjects(n_poses, 1);
    for pp = 1:n_poses
        h_pose_static(pp) = pose_marker_h(ordered_poses(pp,:), pp, segment_colors, n_segments);
    end

    h_task_static = gobjects(scene.num_tasks, 1);
    for ti = 1:scene.num_tasks
        h_task_static(ti) = plot(task_x(ti), task_y(ti), 'o', ...
             'MarkerSize', 8, 'MarkerEdgeColor', [0 0.4 0], ...
             'MarkerFaceColor', [0.2 1 0.2], 'LineWidth', 1);
    end

    [h_robot_chassis, h_robot_dir, h_robot_arm_l, h_robot_arm_r] = ...
        draw_robot_at(ordered_poses(1,:), robot_cfg);

    h_dyn_text = text(0, 0, '', 'Visible', 'off');
    h_pose_labels = gobjects(n_poses, 1);

    h_title = title('');

    s_now = 0;
    last_reached_pose = 1;
    last_paused_pose = 1;
    last_nav_theta = ordered_poses(1, 3);
    completed_so_far = [];
    first_frame_written = false;
    paused_at_current_pose = false;
    continue_after_pause = false;

    while true
        s_clamped = min(s_now, total_path_len);
        p_idx = find(cum_dist >= s_clamped, 1, 'first');
        if isempty(p_idx)
            p_idx = total_pts;
        end

        robot_pose_now = interpolate_path_by_distance(global_path, s_clamped);
        robot_xy_now = robot_pose_now(1:2);
        theta_now = robot_pose_now(3);
        last_nav_theta = theta_now;

        while last_reached_pose < n_poses && ...
              s_clamped >= pose_reached_s(last_reached_pose + 1) - scene.map_info.resolution
            last_reached_pose = last_reached_pose + 1;
            ri = route_order(last_reached_pose);
            if ri > 1 && last_reached_pose <= length(exec_plan)
                plan = exec_plan(last_reached_pose - 1);
                new_tasks = plan.covered_tasks;
                completed_so_far = unique([completed_so_far, new_tasks(:)']);
            end
            continue_after_pause = true;
        end

        if continue_after_pause || (last_reached_pose > last_paused_pose && ~paused_at_current_pose)
        continue_after_pause = false;
        for p = last_paused_pose + 1:last_reached_pose
            dock_pose = ordered_poses(p, :);
            theta_to = dock_pose(3);

            rot_frames = round(frame_rate * 0.6);
            for rf = 1:rot_frames
                paused_at_current_pose = true;

                for s = 1:n_segments
                    set(h_path_lines(s), 'XData', NaN, 'YData', NaN);
                    set(h_path_dashed(s), 'XData', NaN, 'YData', NaN);
                end

                for s = 1:p-2
                    seg_s = pose_reached_at(s);
                    seg_e = pose_reached_at(s + 1);
                    if seg_s <= 0 || seg_e <= seg_s
                        continue;
                    end
                    set(h_path_lines(s), 'XData', global_path(seg_s:seg_e, 1), ...
                         'YData', global_path(seg_s:seg_e, 2), ...
                         'Color', segment_colors(s,:), 'LineStyle', '-', ...
                         'LineWidth', 2.5, 'Visible', 'on');
                end

                if p - 1 >= 1 && p - 1 <= n_segments
                    seg_s = pose_reached_at(p - 1);
                    seg_e = pose_reached_at(p);
                    if seg_s > 0 && seg_e > seg_s
                        set(h_path_lines(p-1), 'XData', global_path(seg_s:seg_e, 1), ...
                             'YData', global_path(seg_s:seg_e, 2), ...
                             'Color', segment_colors(p-1,:), 'LineStyle', '-', ...
                             'LineWidth', 2.5, 'Visible', 'on');
                    end
                end

                alpha = rf / rot_frames;
                theta_rot = last_nav_theta + alpha * wrapToPiLocal(theta_to - last_nav_theta);
                pose_rot = [dock_pose(1), dock_pose(2), theta_rot];
                update_robot_handles(h_robot_chassis, h_robot_dir, ...
                                     h_robot_arm_l, h_robot_arm_r, pose_rot, robot_cfg);

                plan = exec_plan(p - 1);
                if plan.to_pose_idx == 1
                    dyn_str = sprintf('返回起点 (%.1f m)', return_path_len_m);
                    title_str = '任务完成，返回起点...';
                else
                    dyn_str = sprintf('执行: 桌%d 停%d (%d任务)', ...
                                      plan.table_id, plan.dock_index, length(plan.covered_tasks));
                    title_str = sprintf('到达停靠位姿 %d/%d | 姿态调整中...', p - 1, n_segments);
                end
                set(h_dyn_text, 'Position', [dock_pose(1)+0.15, dock_pose(2)+0.18, 0], ...
                     'String', dyn_str, ...
                     'FontSize', 9, 'Color', segment_colors(min(p-1,n_segments),:), ...
                     'BackgroundColor', [1 1 1 0.9], 'Margin', 3, ...
                     'FontWeight', 'bold', 'Visible', 'on');

                set(h_title, 'String', title_str, 'FontSize', 13, 'FontWeight', 'bold');

                drawnow limitrate;
                frame = getframe(hFig);
                writeVideo(vid_writer, frame);
                first_frame_written = true;
            end

            for tf = 1:round(frame_rate * 1.2)
                paused_at_current_pose = true;

                for s = 1:n_segments
                    set(h_path_lines(s), 'XData', NaN, 'YData', NaN);
                    set(h_path_dashed(s), 'XData', NaN, 'YData', NaN);
                end

                for s = 1:p-2
                    seg_s = pose_reached_at(s);
                    seg_e = pose_reached_at(s + 1);
                    if seg_s <= 0 || seg_e <= seg_s
                        continue;
                    end
                    set(h_path_lines(s), 'XData', global_path(seg_s:seg_e, 1), ...
                         'YData', global_path(seg_s:seg_e, 2), ...
                         'Color', segment_colors(s,:), 'LineStyle', '-', ...
                         'LineWidth', 2.5, 'Visible', 'on');
                end

                if p - 1 >= 1 && p - 1 <= n_segments
                    seg_s = pose_reached_at(p - 1);
                    seg_e = pose_reached_at(p);
                    if seg_s > 0 && seg_e > seg_s
                        set(h_path_lines(p-1), 'XData', global_path(seg_s:seg_e, 1), ...
                             'YData', global_path(seg_s:seg_e, 2), ...
                             'Color', segment_colors(p-1,:), 'LineStyle', '-', ...
                             'LineWidth', 2.5, 'Visible', 'on');
                    end
                end

                update_robot_handles(h_robot_chassis, h_robot_dir, ...
                                     h_robot_arm_l, h_robot_arm_r, dock_pose, robot_cfg);

                plan = exec_plan(p - 1);
                if plan.to_pose_idx == 1
                    dyn_str = sprintf('返回起点 (%.1f m)', return_path_len_m);
                    title_str = '任务完成，返回起点...';
                else
                    dyn_str = sprintf('执行: 桌%d 停%d (%d任务)', ...
                                      plan.table_id, plan.dock_index, length(plan.covered_tasks));
                    title_str = sprintf('到达停靠位姿 %d/%d | 机械臂执行任务...', p - 1, n_segments);
                end
                set(h_dyn_text, 'Position', [dock_pose(1)+0.15, dock_pose(2)+0.18, 0], ...
                     'String', dyn_str, ...
                     'FontSize', 9, 'Color', segment_colors(min(p-1,n_segments),:), ...
                     'BackgroundColor', [1 1 1 0.9], 'Margin', 3, ...
                     'FontWeight', 'bold', 'Visible', 'on');

                set(h_title, 'String', title_str, 'FontSize', 13, 'FontWeight', 'bold');

                drawnow limitrate;
                frame = getframe(hFig);
                writeVideo(vid_writer, frame);
                first_frame_written = true;
            end

            last_nav_theta = theta_to;
        end
        last_paused_pose = last_reached_pose;
        paused_at_current_pose = false;
    end

        for s = 1:n_segments
            set(h_path_lines(s), 'XData', NaN, 'YData', NaN);
            set(h_path_dashed(s), 'XData', NaN, 'YData', NaN);
        end

        for s = 1:last_reached_pose - 2
            seg_s = pose_reached_at(s);
            seg_e = pose_reached_at(s + 1);
            if seg_s <= 0 || seg_e <= seg_s
                continue;
            end
            set(h_path_lines(s), 'XData', global_path(seg_s:seg_e, 1), ...
                 'YData', global_path(seg_s:seg_e, 2), ...
                 'Color', segment_colors(s,:), 'LineStyle', '-', ...
                 'LineWidth', 2.5, 'Visible', 'on');
        end

        cur_seg = last_reached_pose - 1;
        if cur_seg >= 1 && cur_seg <= n_segments && pose_reached_at(cur_seg) > 0
            seg_s = pose_reached_at(cur_seg);
            if cur_seg < n_segments && pose_reached_at(cur_seg + 1) > 0
                seg_e = pose_reached_at(cur_seg + 1);
            else
                seg_e = total_pts;
            end
            if seg_s > 0 && seg_e > seg_s
                div = min(p_idx, seg_e);
                if div >= seg_s + 1
                    set(h_path_lines(cur_seg), 'XData', global_path(seg_s:div, 1), ...
                         'YData', global_path(seg_s:div, 2), ...
                         'Color', segment_colors(cur_seg,:), 'LineStyle', '-', ...
                         'LineWidth', 2.5, 'Visible', 'on');
                end
                if div < seg_e
                    set(h_path_dashed(cur_seg), 'XData', global_path(div:seg_e, 1), ...
                         'YData', global_path(div:seg_e, 2), ...
                         'Color', [segment_colors(cur_seg,:) 0.3], 'LineStyle', '--', ...
                         'LineWidth', 1, 'Visible', 'on');
                end
            end
        end

        update_robot_handles(h_robot_chassis, h_robot_dir, h_robot_arm_l, ...
                             h_robot_arm_r, [robot_xy_now, theta_now], robot_cfg);

        for ti = 1:scene.num_tasks
            if ismember(ti, completed_so_far)
                set(h_task_static(ti), 'Marker', 's', 'MarkerSize', 10, ...
                     'MarkerEdgeColor', [0.15 0.15 0.15], ...
                     'MarkerFaceColor', [0.45 0.45 0.45], 'LineWidth', 1.2);
            else
                set(h_task_static(ti), 'Marker', 'o', 'MarkerSize', 8, ...
                     'MarkerEdgeColor', [0 0.4 0], ...
                     'MarkerFaceColor', [0.2 1 0.2], 'LineWidth', 1);
            end
        end

        for pp = 1:n_poses
            if isgraphics(h_pose_labels(pp))
                delete(h_pose_labels(pp));
            end
        end
        for pp = 2:n_poses
            if pp <= last_reached_pose && pp - 1 <= length(exec_plan)
                plan = exec_plan(pp - 1);
                if plan.to_pose_idx == 1
                    str = '返回起点';
                else
                    str = sprintf('桌%d(停%d)', plan.table_id, plan.dock_index);
                end
                h_pose_labels(pp) = text(ordered_poses(pp,1) + 0.10, ...
                     ordered_poses(pp,2) + 0.10, str, ...
                     'FontSize', 7, 'Color', segment_colors(min(pp-1,n_segments),:), ...
                     'BackgroundColor', [1 1 1 0.85], 'Margin', 1);
            end
        end

        dist_traveled = min(s_now, total_path_len);
        progress_pct = 100 * dist_traveled / max(total_path_len, 0.01);

        current_task_stops = sum(route_order(2:min(last_reached_pose, length(route_order))) > 1);
        if return_to_start && last_reached_pose == n_poses && route_order(end) == 1
            progress_title = sprintf(['移动操作机器人任务执行 | 进度: %.1f%% | 路径: %.1f/%.1f m | ' ...
                '返回起点中 | 停靠: %d/%d | 完成: %d/%d任务'], ...
                progress_pct, dist_traveled, total_path_len, ...
                current_task_stops, n_task_stops, length(completed_so_far), scene.num_tasks);
        else
            progress_title = sprintf(['移动操作机器人任务执行 | 进度: %.1f%% | 路径: %.1f/%.1f m | ' ...
                '停靠: %d/%d | 完成: %d/%d任务'], ...
                progress_pct, dist_traveled, total_path_len, ...
                current_task_stops, n_task_stops, length(completed_so_far), scene.num_tasks);
        end

        set(h_title, 'String', progress_title, 'FontSize', 12, 'FontWeight', 'bold');

        drawnow limitrate;
        frame = getframe(hFig);
        writeVideo(vid_writer, frame);
        first_frame_written = true;

        if s_now >= total_path_len
            for s = 1:n_segments
                set(h_path_lines(s), 'XData', NaN, 'YData', NaN);
                set(h_path_dashed(s), 'XData', NaN, 'YData', NaN);
            end
            for s = 1:n_segments
                seg_s = pose_reached_at(s);
                seg_e = pose_reached_at(s + 1);
                if seg_s <= 0 || seg_e <= seg_s
                    continue;
                end
                set(h_path_lines(s), 'XData', global_path(seg_s:seg_e, 1), ...
                     'YData', global_path(seg_s:seg_e, 2), ...
                     'Color', segment_colors(s,:), 'LineStyle', '-', ...
                     'LineWidth', 2.5, 'Visible', 'on');
            end

            for hold_fr = 1:frame_rate * 2
                update_robot_handles(h_robot_chassis, h_robot_dir, h_robot_arm_l, ...
                                     h_robot_arm_r, ordered_poses(end,:), robot_cfg);

                for ti = 1:scene.num_tasks
                    if ismember(ti, completed_so_far)
                        set(h_task_static(ti), 'Marker', 's', 'MarkerSize', 10, ...
                             'MarkerEdgeColor', [0.15 0.15 0.15], ...
                             'MarkerFaceColor', [0.45 0.45 0.45], 'LineWidth', 1.2);
                    end
                end

                if return_to_start && route_order(end) == 1
                    final_title = sprintf('任务执行完成并返回起点! | 总路径: %.1f m | 返航: %.1f m | 任务: %d/%d (%.0f%%)', ...
                        total_path_len, return_path_len_m, length(completed_so_far), scene.num_tasks, ...
                        100*length(completed_so_far)/scene.num_tasks);
                else
                    final_title = sprintf('任务执行完成! | 总路径: %.1f m | 任务: %d/%d (%.0f%%)', ...
                        total_path_len, length(completed_so_far), scene.num_tasks, ...
                        100*length(completed_so_far)/scene.num_tasks);
                end
                set(h_title, 'String', final_title, 'FontSize', 14, 'FontWeight', 'bold');

                drawnow limitrate;
                frame = getframe(hFig);
                writeVideo(vid_writer, frame);
                first_frame_written = true;
            end
            break;
        end

        s_now = s_now + robot_speed * dt;
    end

    if ~first_frame_written
        frame = getframe(hFig);
        writeVideo(vid_writer, frame);
    end

    close(vid_writer);
    close(hFig);
    fprintf('\n  动画视频已保存: %s\n', video_file);
    fprintf('  文件大小: %.1f MB\n', dir(video_file).bytes / 1e6);
end

function exec_plan = build_execution_plan(n_segments, visit_order, pose_reached_at, ...
                                          pose_reached_s, task_sequence_summary)
    exec_plan = struct();
    for k = 1:n_segments
        exec_plan(k).segment_id = k;
        exec_plan(k).path_start_idx = pose_reached_at(k);
        exec_plan(k).path_end_idx   = pose_reached_at(k + 1);
        exec_plan(k).start_s = pose_reached_s(k);
        exec_plan(k).end_s   = pose_reached_s(k + 1);
        exec_plan(k).from_pose_idx = visit_order(k);
        exec_plan(k).to_pose_idx   = visit_order(k + 1);

        ri = visit_order(k + 1);
        if ri > 1
            s_info = task_sequence_summary{ri - 1};
            exec_plan(k).table_id = s_info.table_id;
            exec_plan(k).dock_index = s_info.dock_index;
            if isfield(s_info, 'covered_tasks') && ~isempty(s_info.covered_tasks)
                exec_plan(k).covered_tasks = s_info.covered_tasks(:)';
            else
                exec_plan(k).covered_tasks = s_info.task_ids(:)';
            end
        else
            exec_plan(k).table_id = 0;
            exec_plan(k).dock_index = 0;
            exec_plan(k).covered_tasks = [];
        end
    end
end

function pose_idx = compute_pose_indices_on_path(global_path, ordered_poses)
    n = size(ordered_poses, 1);
    pose_idx = ones(n, 1);
    search_start = 1;
    for p = 1:n
        dx = global_path(search_start:end, 1) - ordered_poses(p, 1);
        dy = global_path(search_start:end, 2) - ordered_poses(p, 2);
        d = sqrt(dx.^2 + dy.^2);
        [~, rel] = min(d);
        idx = search_start + rel - 1;
        pose_idx(p) = idx;
        search_start = max(idx, search_start);
    end
    pose_idx(1) = 1;
    pose_idx(end) = size(global_path, 1);
    for p = 2:n
        pose_idx(p) = max(pose_idx(p), pose_idx(p-1));
    end
end

function update_robot_handles(h_chassis, h_dir, h_arm_l, h_arm_r, pose, robot_cfg)
    x = pose(1); y = pose(2); theta = pose(3);
    L = robot_cfg.chassis.length;
    W = robot_cfg.chassis.width;

    corners = [-L/2 -W/2; L/2 -W/2; L/2 W/2; -L/2 W/2];
    R = rot2(theta);
    cw = zeros(4, 2);
    for c = 1:4
        cw(c,:) = (R * corners(c,:)')' + [x y];
    end
    if isgraphics(h_chassis)
        set(h_chassis, 'XData', cw(:,1), 'YData', cw(:,2));
        set(h_chassis, 'Visible', 'on');
    end

    alen = L * 0.45;
    if isgraphics(h_dir)
        set(h_dir, 'XData', x, 'YData', y, 'UData', alen*cos(theta), 'VData', alen*sin(theta));
        set(h_dir, 'Visible', 'on');
    end

    sp = robot_cfg.dual_arm.base_spacing;
    mx = robot_cfg.arm_base_mount_x;
    lx = x + mx*cos(theta) + sp/2*cos(theta+pi/2);
    ly = y + mx*sin(theta) + sp/2*sin(theta+pi/2);
    rx = x + mx*cos(theta) - sp/2*cos(theta+pi/2);
    ry = y + mx*sin(theta) - sp/2*sin(theta+pi/2);

    if isgraphics(h_arm_l)
        set(h_arm_l, 'XData', lx, 'YData', ly, 'Visible', 'on');
    end
    if isgraphics(h_arm_r)
        set(h_arm_r, 'XData', rx, 'YData', ry, 'Visible', 'on');
    end
end

function [h_chassis, h_dir, h_arm_l, h_arm_r] = draw_robot_at(pose, robot_cfg)
    x = pose(1); y = pose(2); theta = pose(3);
    L = robot_cfg.chassis.length;
    W = robot_cfg.chassis.width;

    corners = [-L/2 -W/2; L/2 -W/2; L/2 W/2; -L/2 W/2];
    R = rot2(theta);
    cw = zeros(4, 2);
    for c = 1:4
        cw(c,:) = (R * corners(c,:)')' + [x y];
    end
    h_chassis = patch(cw(:,1), cw(:,2), [0.15 0.35 0.65], ...
          'FaceAlpha', 0.55, 'EdgeColor', [0.05 0.15 0.4], 'LineWidth', 2.5);

    alen = L * 0.45;
    h_dir = quiver(x, y, alen*cos(theta), alen*sin(theta), 0, ...
           'Color', [0.1 0.2 0.55], 'LineWidth', 2.8, ...
           'MaxHeadSize', 0.9, 'AutoScale', 'off');

    sp = robot_cfg.dual_arm.base_spacing;
    mx = robot_cfg.arm_base_mount_x;
    lx = x + mx*cos(theta) + sp/2*cos(theta+pi/2);
    ly = y + mx*sin(theta) + sp/2*sin(theta+pi/2);
    rx = x + mx*cos(theta) - sp/2*cos(theta+pi/2);
    ry = y + mx*sin(theta) - sp/2*sin(theta+pi/2);

    h_arm_l = plot(lx, ly, 'o', 'MarkerSize', 7, 'MarkerEdgeColor', [0.9 0.3 0.1], ...
         'MarkerFaceColor', [1 0.5 0.2], 'LineWidth', 1.5);
    h_arm_r = plot(rx, ry, 'o', 'MarkerSize', 7, 'MarkerEdgeColor', [0.9 0.3 0.1], ...
         'MarkerFaceColor', [1 0.5 0.2], 'LineWidth', 1.5);
end

function h = pose_marker_h(p, idx, segment_colors, n_segments)
    if idx == 1
        h = plot(p(1), p(2), 'p', 'MarkerSize', 18, ...
             'MarkerEdgeColor', 'k', 'MarkerFaceColor', '#ff0000', 'LineWidth', 1.5);
    else
        h = plot(p(1), p(2), 'o', 'MarkerSize', 12, ...
             'MarkerEdgeColor', 'k', ...
             'MarkerFaceColor', segment_colors(min(idx-1, n_segments),:), 'LineWidth', 1.5);
    end
end

function a = wrapToPiLocal(a)
    a = mod(a + pi, 2*pi) - pi;
end
