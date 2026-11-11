function run_task_navigation_robustness_layout(output_dir,layout_id)
%RUN_TASK_NAVIGATION_ROBUSTNESS_LAYOUT One independent layout worker.
% Three layout processes may run concurrently; each writes its own case dirs.
    if nargin<1,output_dir=fullfile('results','task_navigation_robustness_20261111');end
    if nargin<2,layout_id=0;end
    maxNumCompThreads(1);
    loaded=load(fullfile(output_dir,'batch_setup.mat'),'prepared');p=loaded.prepared;
    indices=find([p.definitions.layout_id]==layout_id);assert(numel(indices)==3);
    state=struct('status','RUNNING','layout_id',layout_id,'phase','libraries', ...
        'completed_libraries',0,'completed_scenarios',0,'case_id','','mode','','error','');
    state_path=fullfile(output_dir,sprintf('layout_state_%d.json',layout_id));write_state(state_path,state);
    modes=p.metadata.task_modes;proof_rows={};failed_cases=false(size(p.definitions));
    try
        for mi=1:numel(modes)
            cache=[];
            for index=reshape(indices,1,[])
                def=p.definitions(index);state.case_id=def.case_id;state.mode=modes{mi};write_state(state_path,state);
                folder=fullfile(output_dir,'cases',def.case_id);
                if failed_cases(index),continue;end
                try
                    [~,~,proof,cache]=build_main_task_navigation_library(p.setups{index},modes{mi},folder,cache);
                catch err
                    record_task_navigation_robustness_failure(folder,def,['library_' modes{mi}],err);
                    failed_cases(index)=true;continue;
                end
                proof_rows{end+1}=struct('case_id',def.case_id,'mode',modes{mi},'proof',proof); %#ok<AGROW>
                write_state(fullfile(folder,['ik_build_proof_' modes{mi} '.json']),proof);
                state.completed_libraries=state.completed_libraries+1;write_state(state_path,state);
                write_state(fullfile(output_dir,sprintf('layout_ik_proofs_%d.json',layout_id)),proof_rows);
            end
        end
        clear cache;
        state.phase='planning';state.mode='';write_state(state_path,state);
        for index=reshape(indices,1,[])
            def=p.definitions(index);state.case_id=def.case_id;write_state(state_path,state);
            folder=fullfile(output_dir,'cases',def.case_id);complete=false;
            if isfile(fullfile(folder,'run_state.json'))
                previous=jsondecode(fileread(fullfile(folder,'run_state.json')));
                complete=strcmp(previous.status,'COMPLETE');
            end
            if ~complete
                options=struct('setup',p.setups{index},'write_checkpoints',false,'retain_full_mat',false);
                try
                    exp_main_task_navigation(folder,options);
                catch err
                    record_task_navigation_robustness_failure(folder,def,'scenario_planning',err);
                end
            end
            state.completed_scenarios=state.completed_scenarios+1;write_state(state_path,state);
        end
        state.status='COMPLETE';state.phase='complete';write_state(state_path,state);
    catch err
        state.status='ERROR';state.error=[err.identifier ': ' err.message];write_state(state_path,state);rethrow(err);
    end
end

function write_state(path,value)
    parent=fileparts(path);temporary=[tempname(parent) '.json'];
    fid=fopen(temporary,'w','n','UTF-8');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));fclose(fid);
    [ok,message]=movefile(temporary,path,'f');assert(ok,message);
end
