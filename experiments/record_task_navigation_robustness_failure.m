function record_task_navigation_robustness_failure(folder,definition,phase,err)
%RECORD_TASK_NAVIGATION_ROBUSTNESS_FAILURE Retain pre-artifact errors as failures.
    if isstruct(err),reason=err.message;else,reason=[err.identifier ': ' err.message];end
    evidence=struct('case_id',definition.case_id,'phase',phase,'failure_reason',reason, ...
        'synthetic_summary_rows_are_failure_markers',true,'mission_success',false);
    write_json(fullfile(folder,'scenario_failure.json'),evidence);
    modes={'position_only','top_down_grasp','table_wipe'};methods={'staged','minimum_seed','reorder_only','joint_local'};
    rows=cell(0,25);
    for mi=1:3
        for method=methods
            rows(end+1,:)={modes{mi},method{1},0,definition.task_count,0,NaN,NaN,NaN,NaN, ...
                false,false,0,'CASE_PREPARATION_FAILED',reason,NaN,NaN,NaN,NaN,0,0,0,0,phase,false,false}; %#ok<AGROW>
        end
    end
    names={'mode','method','covered_tasks','target_count','docks','minimum_docks','route_length_m','navigation_cost', ...
        'selection_time_s','route_valid','mission_valid','verified_tasks','status','failure_reason','raw_route_length_m', ...
        'raw_navigation_cost','input_seed_cost','minimum_seed_cost','oracle_queries','new_lazy_path_queries', ...
        'external_oracle_hits','accepted_moves','termination_reason','local_optimum_certified','global_optimal'};
    writetable(cell2table(rows,'VariableNames',names),fullfile(folder,'summary.csv'));
    write_json(fullfile(folder,'run_state.json'),struct('status','COMPLETE','completed_cases',12, ...
        'valid_missions',0,'preparation_failed',true));
end

function write_json(path,value)
    fid=fopen(path,'w','n','UTF-8');assert(fid>=0);cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
    fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));
end
