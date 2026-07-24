function local_pts = sample_task_near_table_corner(L, W, n_tasks, margin)
%SAMPLE_TASK_NEAR_TABLE_CORNER 生成靠近桌子四角的任务点。
%
%   local_pts = sample_task_near_table_corner(L, W, n_tasks, margin)
%
%   输入:
%       L       - 桌子长度 (m)
%       W       - 桌子宽度 (m)
%       n_tasks - 需要生成的任务点数量
%       margin  - 角点安全边距 (m)，默认 0.06，防止任务点落到桌面外
%
%   输出:
%       local_pts - [n_tasks x 2] 桌子局部坐标系下的任务点 [local_x, local_y]

    if nargin < 4 || isempty(margin)
        margin = 0.06;
    end

    half_L = L / 2 - margin;
    half_W = W / 2 - margin;

    % 四个角点位置
    corners = [-half_L, -half_W;
               -half_L,  half_W;
                half_L, -half_W;
                half_L,  half_W];

    % 随机从四个角中选取（允许重复，模拟多个任务靠近同一角）
    indices = randi(4, n_tasks, 1);
    local_pts = corners(indices, :);

    % 添加微小随机抖动（±0.01 m），避免完全重合
    jitter = (rand(n_tasks, 2) - 0.5) * 0.02;
    local_pts = local_pts + jitter;
    % 钳制在合法范围内
    local_pts(:, 1) = max(-half_L, min(half_L, local_pts(:, 1)));
    local_pts(:, 2) = max(-half_W, min(half_W, local_pts(:, 2)));
end