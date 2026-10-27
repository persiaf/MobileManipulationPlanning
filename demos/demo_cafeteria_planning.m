function result = demo_cafeteria_planning(task_mode, seed, planning_mode)
%DEMO_CAFETERIA_PLANNING Run the gated fixed cafeteria experiment.

    if nargin < 1 || isempty(task_mode), task_mode = 'complex12'; end
    if isnumeric(task_mode)
        seed = task_mode;
        task_mode = 'complex12';
    elseif nargin < 2 || isempty(seed)
        seed = 2026;
    end
    if nargin < 3 || isempty(planning_mode), planning_mode = 'navigation_docking_only'; end
    result = run_cafeteria_planning(task_mode, seed, true, 'off', planning_mode);
end
