function write_repair_case_artifacts(state,row,audit_rows,out_dir,key)
%WRITE_REPAIR_CASE_ARTIFACTS Small replayable inputs/results, no old overwrite.
    if ~isfolder(out_dir),mkdir(out_dir);end
    record=struct('row',row);
    if isfield(state,'scene'),record.scene=state.scene;end
    if isfield(state,'robot_cfg'),record.robot_cfg=state.robot_cfg;end
    if isfield(state,'planner_cfg'),record.planner_cfg=state.planner_cfg;end
    if isfield(state,'plan')
        record.selected_poses=state.plan.selected_poses;
        record.covered_task_ids=state.plan.covered_tasks_predicted;
        record.task_assignment=state.plan.task_sequence_summary;
    end
    if isfield(state,'stats')
        record.global_path=state.stats.global_path;
        record.path_validation=state.stats.path_validation;
        record.evidence=state.stats.evidence;
    end
    save(fullfile(out_dir,[key '.mat']),'record','-v7.3');
    if ~isempty(audit_rows)
        writetable(struct2table(audit_rows,'AsArray',true),fullfile(out_dir,[key '_task_audit.csv']));
    end
end
