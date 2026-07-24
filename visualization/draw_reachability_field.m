function draw_reachability_field(candidates, region, scene, fig_handle)
    if nargin < 4
        fig_handle = gcf;
    end
    figure(fig_handle);
    hold on;

    if isempty(candidates)
        return;
    end

    if ~isfield(candidates(1), 'cost_breakdown') || isempty(candidates(1).cost_breakdown)
        robot_cfg = robot_config();
        planner_cfg = planner_config();
        task_ids = region.task_ids;
        num_tasks = length(task_ids);
        task_positions = zeros(num_tasks, 2);
        for ti = 1:num_tasks
            tid = task_ids(ti);
            task_positions(ti, :) = [scene.tasks(tid).x, scene.tasks(tid).y];
        end
        task_centroid = mean(task_positions, 1);
        for ci = 1:length(candidates)
            cost = evaluate_docking_candidate(candidates(ci), region, scene, robot_cfg, planner_cfg, task_positions, task_centroid);
            candidates(ci).cost_breakdown = cost;
            candidates(ci).total_cost = cost.total;
        end
    end

    for ci = 1:length(candidates)
        pose = candidates(ci).pose;
        cost_total = candidates(ci).total_cost;
        score = 1.0 / (1.0 + max(cost_total, 0.001));
        color = [1 - score, score, 0];
        plot(pose(1), pose(2), '.', 'Color', color, 'MarkerSize', 8);
    end
    hold off;
end