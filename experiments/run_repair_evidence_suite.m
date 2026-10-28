function result=run_repair_evidence_suite(output_root)
%RUN_REPAIR_EVIDENCE_SUITE Sequential pilot studies after the repair gates.
% A discarded warm-up precedes timing. Do not run other MATLAB experiments
% concurrently if comparing these wall-clock durations.
    root=fileparts(fileparts(mfilename('fullpath'))); init_paths(root);
    if nargin<1
        output_root=fullfile(root,'results','repair_20261027',['evidence_' datestr(now,'yyyymmdd_HHMMSS')]);
    end
    if ~isfolder(output_root),mkdir(output_root);end
    started=tic;
    result=struct('matlab_version',version,'seeds',[2026 2027 2028], ...
        'started_at',datestr(now,30),'execution_order',{{'warmup','main','sensitivity','complex','dynamic'}}, ...
        'timing_note','sequential same MATLAB process; discarded warmup; pilot timings without confidence intervals', ...
        'complete',false);
    [warmup,~,~]=run_protocol_planning_case(repair_study_scene_config('MAIN4',2025), ...
        planner_config(),struct('run_audit',false,'compute_identity',false));
    result.warmup=warmup;
    save_manifest(output_root,result);
    rows=struct([]);
    for seed=result.seeds
        [row,state,audit]=run_protocol_planning_case(repair_study_scene_config('MAIN',seed),planner_config());
        if isempty(rows),rows=row;else,rows(end+1)=row;end %#ok<AGROW>
        out=fullfile(output_root,'main'); if ~isfolder(out),mkdir(out);end
        writetable(struct2table(rows,'AsArray',true),fullfile(out,'main_raw.csv'));
        write_repair_case_artifacts(state,row,audit,fullfile(out,'cases'),sprintf('MAIN_seed%d',seed));
        fprintf('MAIN seed=%d status=%s coverage=%.3f T=%.3f\n',seed,row.status,row.geometric_coverage_percent,row.planning_time_s);
    end
    result.main_rows=numel(rows); save_manifest(output_root,result);
    raw=exp_repair_sampling_weight_sensitivity(struct('output_dir',fullfile(output_root,'sensitivity')));
    result.sensitivity_rows=height(raw); save_manifest(output_root,result);
    raw=exp_repair_complex_scenes(struct('output_dir',fullfile(output_root,'complex')));
    result.complex_rows=height(raw); save_manifest(output_root,result);
    raw=run_protocol_dynamic_comparison(struct('output_dir',fullfile(output_root,'dynamic')));
    result.dynamic_rows=height(raw); result.complete=true;
    result.elapsed_wall_s=toc(started); result.finished_at=datestr(now,30);
    save_manifest(output_root,result);
    fprintf('REPAIR_EVIDENCE_SUITE_COMPLETE output=%s elapsed=%.3fs\n',output_root,result.elapsed_wall_s);
end

function save_manifest(output_root,result)
    fid=fopen(fullfile(output_root,'run_manifest.json'),'w','n','UTF-8');
    if fid<0,error('run_repair_evidence_suite:Manifest','Cannot write run manifest.');end
    clean=onCleanup(@()fclose(fid)); %#ok<NASGU>
    fprintf(fid,'%s',jsonencode(result,'PrettyPrint',true));
end
