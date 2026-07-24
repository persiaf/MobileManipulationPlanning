function local_pts = sample_task_on_opposite_edges(L, W, n_tasks, margin)
%SAMPLE_TASK_ON_OPPOSITE_EDGES 在桌子相对两侧长边生成任务点。
%
%   任务点分布在 +y 和 -y 两条长边上，使单个停靠位姿难以覆盖所有任务。
%
%   local_pts = sample_task_on_opposite_edges(L, W, n_tasks, margin)
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

    n_per_side = ceil(n_tasks / 2);
    % 沿桌子长度方向均匀分布
    x_positions = linspace(-half_L, half_L, n_per_side)';

    % +y 侧（上侧长边）
    side_plus  = [x_positions,  half_W * ones(n_per_side, 1)];
    % -y 侧（下侧长边）
    side_minus = [x_positions, -half_W * ones(n_per_side, 1)];

    all_pts = [side_plus; side_minus];

    % 加微小抖动
    jitter = (rand(size(all_pts)) - 0.5) * 0.02;
    all_pts = all_pts + jitter;
    all_pts(:, 1) = max(-half_L, min(half_L, all_pts(:, 1)));
    all_pts(:, 2) = max(-half_W, min(half_W, all_pts(:, 2)));

    % 若点数超量随机抽取
    if size(all_pts, 1) > n_tasks
        sel = randperm(size(all_pts, 1), n_tasks);
        all_pts = all_pts(sel, :);
    end

    local_pts = all_pts;
end