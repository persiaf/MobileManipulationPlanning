function raw = exp_repair_complex_scenes(options)
%EXP_REPAIR_COMPLEX_SCENES Fresh full-size C0-C3 under the same protocol.
    if nargin<1,options=struct();end
    root=fileparts(fileparts(mfilename('fullpath'))); init_paths(root);
    out=field_or(options,'output_dir',fullfile(root,'results','repair_20261027','complex'));
    if ~isfolder(out),mkdir(out);end
    seeds=field_or(options,'seeds',[2026 2027 2028]);
    profiles=field_or(options,'profiles',{'C0','C1','C2','C3'}); rows=struct([]);
    for seed=seeds
        for pi=1:numel(profiles)
            id=profiles{pi}; cfg=planner_config(); scene_cfg=repair_study_scene_config(id,seed);
            [row,state,audit]=run_protocol_planning_case(scene_cfg,cfg,struct('run_audit',true,'variant','fixed_protocol'));
            if isempty(rows),rows=row;else,rows(end+1)=row;end %#ok<AGROW>
            raw=struct2table(rows,'AsArray',true); writetable(raw,fullfile(out,'complex_raw.csv'));
            write_repair_case_artifacts(state,row,audit,fullfile(out,'cases'),sprintf('%s_seed%d',id,seed));
            fprintf('COMPLEX profile=%s seed=%d status=%s coverage=%.3f oracle=%.3f T=%.3f\n', ...
                id,seed,row.status,row.geometric_coverage_percent,row.independent_coverage_percent,row.planning_time_s);
        end
    end
end

function v=field_or(s,n,d)
    if isstruct(s)&&isfield(s,n),v=s.(n);else,v=d;end
end
