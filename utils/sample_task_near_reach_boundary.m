function local_pts = sample_task_near_reach_boundary(L, W, n_tasks, margin)
%SAMPLE_TASK_NEAR_REACH_BOUNDARY 在桌子四条边（含四角）生成任务点，测试可达边界。
%
%   任务点沿桌子四条边（+y, -y, -x, +x）均匀分布，覆盖全周。
%   这种分布使部分任务点在任何单一停靠位姿下都接近可达半径极限。
%
%   local_pts = sample_task_near_reach_boundary(L, W, n_tasks, margin)
%
%   输入:
%       L       - 桌子长度 (m)
%       W       - 桌子宽度 (m)
%       n_tasks - 需要生成的任务点数量
%       margin  - 边距 (m)，默认 0.04
%
%   输出:
%       local_pts - [n_tasks x 2] 桌子局部坐标系下的任务点

    if nargin < 4 || isempty(margin)
        margin = 0.04;
    end

    half_L = L / 2 - margin;
    half_W = W / 2 - margin;

    total_edges = 4;
    pts_per_edge = ceil(n_tasks / total_edges);

    % +y 边（长边）
    x_top = linspace(-half_L, half_L, pts_per_edge)';
    top = [x_top, half_W * ones(pts_per_edge, 1)];

    % -y 边（长边）
    x_bot = linspace(-half_L, half_L, pts_per_edge)';
    bot = [x_bot, -half_W * ones(pts_per_edge, 1)];

    % -x 边（短边）
    y_left = linspace(-half_W, half_W, max(2, ceil(pts_per_edge / 2)))';
    left = [-half_L * ones(length(y_left), 1), y_left];

    % +x 边（短边）
    y_right = linspace(-half_W, half_W, max(2, ceil(pts_per_edge / 2)))';
    right = [half_L * ones(length(y_right), 1), y_right];

    all_pts = [top; bot; left; right];

    % 去重（角点会被两条边重复生成）
    all_pts = unique(all_pts, 'rows');

    % 加微小抖动
    jitter = (rand(size(all_pts)) - 0.5) * 0.02;
    all_pts = all_pts + jitter;
    all_pts(:, 1) = max(-half_L, min(half_L, all_pts(:, 1)));
    all_pts(:, 2) = max(-half_W, min(half_W, all_pts(:, 2)));

    if size(all_pts, 1) > n_tasks
        sel = randperm(size(all_pts, 1), n_tasks);
        all_pts = all_pts(sel, :);
    end

    local_pts = all_pts;
end