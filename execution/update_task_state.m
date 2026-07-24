function scene = update_task_state(scene, completed_task_ids)
    for i = 1:length(completed_task_ids)
        tid = completed_task_ids(i);
        scene.tasks(tid).status = 1;
    end
end