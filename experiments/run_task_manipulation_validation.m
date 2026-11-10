function summary=run_task_manipulation_validation(output_dir)
%RUN_TASK_MANIPULATION_VALIDATION Targeted regression checks and demo evidence.
    if nargin<1 || isempty(output_dir)
        output_dir=fullfile('results','task_conditioned_workspace');
    end
    if ~isfolder(output_dir),mkdir(output_dir);end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(genpath(fullfile(root,'experiment_manager')));
    names={'test_vx300s_kinematics.m', ...
        'test_task_conditioned_manipulation.m', ...
        'test_task_reachability_cache_and_legacy.m', ...
        'test_evidence_metric_contract.m', ...
        'test_planned_execution_consistency.m', ...
        'test_round_b_model_contracts.m', ...
        'test_main_smoke.m','test_main_robot_unification.m', ...
        'test_fixed_heading_navigation_recovery.m'};
    files=cellfun(@(n)fullfile(root,'tests',n),names,'UniformOutput',false);
    results=runtests(files);
    cases=table({results.Name}',[results.Passed]',[results.Failed]', ...
        [results.Incomplete]',[results.Duration]', ...
        'VariableNames',{'name','passed','failed','incomplete','duration_s'});
    writetable(cases,fullfile(output_dir,'validation_cases.csv'));
    summary=struct('date','2026-11-10','matlab_version',version, ...
        'scope','targeted_tests_not_full_repository_suite','test_files',{names}, ...
        'test_count',numel(results),'passed',sum([results.Passed]), ...
        'failed',sum([results.Failed]),'incomplete',sum([results.Incomplete]));
    write_summary(summary,output_dir);
    assert(all([results.Passed]),'run_task_manipulation_validation:Failed', ...
        'Targeted validation failed; inspect validation_cases.csv.');
    report=demo_task_conditioned_workspace(output_dir);
    summary.workspace_sample_count=report.sample_count;
    summary.workspace_supported_counts=report.supported_counts;
    summary.model_revision=report.model_revision;
    summary.collision_checked=false;
    summary.hardware_calibrated=false;
    write_summary(summary,output_dir);
    fprintf('Task manipulation checks: %d/%d passed.\n',summary.passed,summary.test_count);
end

function write_summary(summary,output_dir)
    fid=fopen(fullfile(output_dir,'validation_summary.json'),'w','n','UTF-8');
    if fid<0,error('run_task_manipulation_validation:OpenFailed','Cannot write summary.');end
    cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
    fprintf(fid,'%s\n',jsonencode(summary,'PrettyPrint',true));
end
