function draw_global_path(global_path, fig_handle, waypoint_poses, visit_labels)
    if nargin < 4
        visit_labels = {};
    end
    if nargin < 3
        waypoint_poses = [];
    end
    if nargin < 2 || isempty(fig_handle)
        fig_handle = figure('Name', 'Global Path', 'Position', [200, 80, 1000, 780]);
    end
    figure(fig_handle);
    hold on;

    if size(global_path, 1) < 2
        return;
    end

    n_seg = 1;
    if ~isempty(waypoint_poses) && size(waypoint_poses, 1) > 1
        n_seg = size(waypoint_poses, 1) - 1;
    end

    seg_colors = lines(max(n_seg, 1));
    total_dist = 0;

    if ~isempty(waypoint_poses) && size(waypoint_poses, 1) > 1
        seg_start = 1;
        for s = 1:n_seg
            to_wp = waypoint_poses(s+1, :);
            reach_tol = max(2 * 0.05, 0.10);
            seg_end = seg_start;
            for k = seg_start:size(global_path, 1)
                if sqrt((global_path(k,1)-to_wp(1))^2 + ...
                        (global_path(k,2)-to_wp(2))^2) < reach_tol
                    seg_end = k;
                    break;
                end
                seg_end = k;
            end

            seg_x = global_path(seg_start:seg_end, 1);
            seg_y = global_path(seg_start:seg_end, 2);
            plot(seg_x, seg_y, '-', 'Color', seg_colors(s,:), 'LineWidth', 2.5);

            seg_dist = sum(sqrt(diff(seg_x).^2 + diff(seg_y).^2));
            total_dist = total_dist + seg_dist;

            mid = round((seg_start + seg_end) / 2);
            if mid > 0 && mid <= size(global_path, 1)
                text(global_path(mid, 1), global_path(mid, 2) + 0.08, ...
                     sprintf('%.2fm', seg_dist), 'FontSize', 7, ...
                     'Color', seg_colors(s,:)*0.7, ...
                     'HorizontalAlignment', 'center');
            end

            seg_start = min(seg_end + 1, size(global_path, 1));
        end

        n_arrows = min(10, size(global_path, 1));
        arrow_step = max(1, floor(size(global_path, 1) / (n_arrows + 1)));
        for a = arrow_step:arrow_step:size(global_path, 1)-1
            dx_a = global_path(a+1, 1) - global_path(a, 1);
            dy_a = global_path(a+1, 2) - global_path(a, 2);
            len_a = sqrt(dx_a^2 + dy_a^2);
            if len_a > 1e-6
                dx_a = dx_a / len_a * 0.12;
                dy_a = dy_a / len_a * 0.12;
                quiver(global_path(a, 1), global_path(a, 2), dx_a, dy_a, 0, ...
                       'Color', [0.3 0.3 0.3], 'LineWidth', 1, ...
                       'MaxHeadSize', 0.6, 'AutoScale', 'off');
            end
        end

        if ~isempty(visit_labels)
            for s = 1:min(length(visit_labels), size(waypoint_poses, 1))
                wp = waypoint_poses(s, :);
                text(wp(1) + 0.1, wp(2) - 0.1, visit_labels{s}, ...
                     'FontSize', 8, 'FontWeight', 'bold', ...
                     'Color', [0.2 0.2 0.5], ...
                     'BackgroundColor', [1 1 1 0.85], 'Margin', 1);
            end
        end
    else
        plot(global_path(:,1), global_path(:,2), 'b-', 'LineWidth', 2.5);
        total_dist = sum(sqrt(diff(global_path(:,1)).^2 + diff(global_path(:,2)).^2));
    end

    plot(global_path(1,1), global_path(1,2), 'p', ...
         'MarkerSize', 18, 'MarkerEdgeColor', 'k', ...
         'MarkerFaceColor', '#ff0000', 'LineWidth', 1.5);
    plot(global_path(end,1), global_path(end,2), 'h', ...
         'MarkerSize', 18, 'MarkerEdgeColor', 'k', ...
         'MarkerFaceColor', '#0000ff', 'LineWidth', 1.5);

    xlabel('X (m)', 'FontSize', 11);
    ylabel('Y (m)', 'FontSize', 11);
    title(sprintf('Global Path — Total: %.2f m | Docking Points: %d', ...
          total_dist, n_seg+1), 'FontSize', 13);
    grid on;
    hold off;
end