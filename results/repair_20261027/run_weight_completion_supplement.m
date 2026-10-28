function [raw, result] = run_weight_completion_supplement()
%RUN_WEIGHT_COMPLETION_SUPPLEMENT MAIN4 one-factor weight pilot, 21 cases.
% Run in a NEW single-thread MATLAB process after the main suite finishes:
%   addpath(fullfile(pwd,'results','repair_20261027'));
%   [raw,result] = run_weight_completion_supplement();
% This result-only driver does not change the frozen production sources.
% A discarded 2025 warm-up precedes three seeds, each with seven settings.
% Timings belong to this independent batch and must not be pooled with the
% earlier main-suite batch. All planning failures remain in the CSV.

    driver_dir=fileparts(mfilename('fullpath'));
    root=fileparts(fileparts(driver_dir));
    addpath(fullfile(root,'pipeline')); init_paths(root);
    pointer=fullfile(driver_dir,'current_evidence_path.txt');
    if isfile(pointer)
        evidence_root=strtrim(fileread(pointer));
    else
        evidence_root=fullfile(driver_dir,'evidence_20261027_221823');
    end
    if ~isfolder(evidence_root)
        error('weight_completion_supplement:MissingEvidence','The pointed evidence directory does not exist.');
    end
    out=fullfile(evidence_root,'weight_completion_supplement');
    csv_path=fullfile(out,'weight_completion_raw.csv');
    if isfile(csv_path)
        error('weight_completion_supplement:ExistingResults', ...
            'Existing supplement CSV retained. Archive that folder before starting a new independent batch.');
    end
    if ~isfolder(out),mkdir(out);end
    copyfile([mfilename('fullpath') '.m'],fullfile(out,'run_weight_completion_supplement.m'));

    reference_path=fullfile(evidence_root,'dynamic','base','seed2026.mat');
    if ~isfile(reference_path)
        error('weight_completion_supplement:MissingSourceReference','Missing frozen main-case source reference.');
    end
    reference=load(reference_path,'record');
    expected_source=reference.record.row.implementation_signature;
    if isempty(expected_source)
        error('weight_completion_supplement:MissingSourceIdentity','The frozen reference has no implementation identity.');
    end

    seeds=[2026 2027 2028];
    variants={'baseline','cost_reach_half','cost_reach_double', ...
        'cost_coverage_half','cost_coverage_double', ...
        'selection_coverage_half','selection_coverage_double'};
    batch_id=['MAIN4_WEIGHT_COMPLETION_' datestr(now,'yyyymmdd_HHMMSS')];
    result=struct('study','MAIN4_weight_completion_supplement', ...
        'batch_id',batch_id,'matlab_version',version,'started_at',datestr(now,30), ...
        'seeds',seeds,'variants',{variants},'expected_rows',numel(seeds)*numel(variants), ...
        'completed_rows',0,'complete',false,'scene_id','MAIN4', ...
        'source_implementation_signature',expected_source, ...
        'source_reference','dynamic/base/seed2026.mat', ...
        'source_snapshot','../io_revision_source_snapshot.zip', ...
        'invocation_requirement','new MATLAB process, -singleCompThread, no concurrent experiment', ...
        'timing_comparison_group',batch_id, ...
        'timing_note','sequential independent batch after discarded warm-up; do not pool absolute timings with the earlier main-suite process', ...
        'scope_note','four-table main task-distribution subset; three-seed descriptive pilot, not a full main-scene or physical-robot validation');
    started=tic;
    save_manifest(out,result);
    [warmup,~,~]=run_protocol_planning_case(repair_study_scene_config('MAIN4',2025), ...
        planner_config(),struct('run_audit',false,'compute_identity',true,'variant','discarded_warmup'));
    result.warmup=warmup;
    save_manifest(out,result);
    assert(strcmp(warmup.implementation_signature,expected_source), ...
        'weight_completion_supplement:SourceDrift','Production sources differ from the frozen main batch.');

    rows=struct([]);
    for seed=seeds
        reference_geometry='';
        for vi=1:numel(variants)
            name=variants{vi};
            cfg=planner_config();
            switch name
                case 'baseline'
                case 'cost_reach_half',cfg.cost_weights.w_reach=cfg.cost_weights.w_reach/2;
                case 'cost_reach_double',cfg.cost_weights.w_reach=cfg.cost_weights.w_reach*2;
                case 'cost_coverage_half',cfg.cost_weights.w_coverage=cfg.cost_weights.w_coverage/2;
                case 'cost_coverage_double',cfg.cost_weights.w_coverage=cfg.cost_weights.w_coverage*2;
                case 'selection_coverage_half',cfg.strategy.bottom_up.weights.coverage_gain=cfg.strategy.bottom_up.weights.coverage_gain/2;
                case 'selection_coverage_double',cfg.strategy.bottom_up.weights.coverage_gain=cfg.strategy.bottom_up.weights.coverage_gain*2;
            end
            [row,state,audit]=run_protocol_planning_case(repair_study_scene_config('MAIN4',seed),cfg, ...
                struct('variant',name,'run_audit',true,'run_routing',true,'compute_identity',true));
            row.supplement_batch_id=batch_id;
            row.timing_comparison_group=batch_id;
            row.paired_geometry_checked=~isempty(row.task_geometry_signature);
            row.paired_geometry_matches=false;
            if row.paired_geometry_checked
                if isempty(reference_geometry),reference_geometry=row.task_geometry_signature;end
                row.paired_geometry_matches=strcmp(row.task_geometry_signature,reference_geometry);
            end
            row.frozen_source_matches=strcmp(row.implementation_signature,expected_source);
            if row.paired_geometry_checked && ~row.paired_geometry_matches
                row.status='ERROR_PAIRING';
                row.failure_reason='The same-seed table/task geometry changed across weight settings.';
            end
            if ~isempty(row.implementation_signature) && ~row.frozen_source_matches
                row.status='ERROR_SOURCE_DRIFT';
                row.failure_reason='Production implementation signature differs from the frozen main batch.';
            end
            if isempty(rows),rows=row;else,rows(end+1)=row;end %#ok<AGROW>
            raw=struct2table(rows,'AsArray',true);
            % Persist the result before the paired-input assertions so an
            % invalid configuration is retained instead of disappearing.
            writetable(raw,csv_path);
            write_repair_case_artifacts(state,row,audit,fullfile(out,'cases'),sprintf('seed%d_%s',seed,name));
            result.completed_rows=numel(rows);
            result.status_counts=count_statuses(rows);
            result.pairing_unchecked_rows=sum(~[rows.paired_geometry_checked]);
            result.elapsed_wall_s=toc(started);
            save_manifest(out,result);
            fprintf('WEIGHT_COMPLETION seed=%d variant=%s status=%s coverage=%.3f docks=%d path=%.3f T=%.3f\n', ...
                seed,name,row.status,row.geometric_coverage_percent,row.selected_docks,row.path_length_m,row.planning_time_s);
            assert(~row.paired_geometry_checked || row.paired_geometry_matches, ...
                'weight_completion_supplement:GeometryDrift','Paired weight settings changed task/table geometry.');
            assert(isempty(row.implementation_signature) || row.frozen_source_matches, ...
                'weight_completion_supplement:SourceDrift','Production implementation changed during the supplement.');
        end
    end
    result.complete=numel(rows)==result.expected_rows;
    result.finished_at=datestr(now,30); result.elapsed_wall_s=toc(started);
    save_manifest(out,result);
    fprintf('WEIGHT_COMPLETION_SUPPLEMENT_COMPLETE rows=%d output=%s elapsed=%.3fs\n', ...
        numel(rows),out,result.elapsed_wall_s);
end

function counts=count_statuses(rows)
    labels=unique({rows.status},'stable');
    counts=repmat(struct('status','','count',0),1,numel(labels));
    for k=1:numel(labels)
        counts(k).status=labels{k}; counts(k).count=sum(strcmp({rows.status},labels{k}));
    end
end

function save_manifest(out,result)
    fid=fopen(fullfile(out,'weight_completion_manifest.json'),'w','n','UTF-8');
    if fid<0,error('weight_completion_supplement:Manifest','Cannot write supplement manifest.');end
    clean=onCleanup(@()fclose(fid)); %#ok<NASGU>
    fprintf(fid,'%s',jsonencode(result,'PrettyPrint',true));
end
