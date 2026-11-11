function [library,scene,proof]=build_main_task_navigation_library(setup,mode,output_dir)
%BUILD_MAIN_TASK_NAVIGATION_LIBRARY Score every frozen same-table pair.
% Translation-cache hits retain FK-checked witnesses; negative hits require
% identical raw requests. Export only plain counters, never the cache Map.
% A completed MAT is reused only after model/task/pool/policy/source signature
% validation. A mismatch fails and requires a fresh output directory.
    if ~any(strcmp(mode,{'position_only','top_down_grasp','table_wipe'}))
        error('build_main_task_navigation_library:Mode','Unknown task mode.');
    end
    if nargin<3||isempty(output_dir)
        error('build_main_task_navigation_library:OutputDirectory','Provide a dedicated experiment output directory.');
    end
    mode=char(mode);output_dir=char(output_dir);call_started=tic;
    scene=setup.scene;scene.tasks=normalize_task_fields(scene.tasks);
    for ti=1:scene.num_tasks,scene.tasks(ti)=comparison_manipulation_task(scene.tasks(ti),mode);end
    robot=setup.robot;planner=setup.planner;regions=setup.regions;pool=setup.pool;
    if ~isequal([pool.candidate_id],1:numel(pool))
        error('build_main_task_navigation_library:PoolOrder','candidate_id must equal its 1-based pool index.');
    end
    flattened=[];task_ids=[];
    for ri=1:numel(regions)
        flattened=[flattened,find([pool.table_id]==regions(ri).table_id)]; %#ok<AGROW>
        task_ids=[task_ids,reshape(regions(ri).task_ids,1,[])]; %#ok<AGROW>
    end
    if ~isequal(flattened,1:numel(pool))||~isequal(sort(task_ids),1:scene.num_tasks)
        error('build_main_task_navigation_library:PoolOrder','Regions must partition tasks and retain the frozen pool flattening order.');
    end
    revision='main_task_navigation_library_v1';policy='task_local_translation_witness_v1';
    source_names={'evaluate_task_reachability_cached','evaluate_task_reachability', ...
        'resolve_manipulation_task','solve_arm_task_ik','arm_forward_kinematics', ...
        'manipulation_task_profile','comparison_manipulation_task'};
    source_payload=struct();
    for si=1:numel(source_names)
        name=source_names{si};path=which(name);
        if isempty(path),error('build_main_task_navigation_library:MissingEvaluator','Missing %s.',name);end
        source_payload.(name)=build_struct_signature(fileread(path));
    end
    scene_signature=build_candidate_library_signature(regions,scene,robot,planner);
    signature_payload=struct('builder_revision',revision,'cache_policy_revision',policy, ...
        'scene_signature',scene_signature,'pool',pool,'scorer_sources',source_payload);
    signature=build_struct_signature(signature_payload);
    common_payload=signature_payload;
    common_payload.scene_signature=build_candidate_library_signature(regions,setup.scene,robot,planner);
    common_signature=build_struct_signature(common_payload);
    if ~isfolder(output_dir),mkdir(output_dir);end
    directory_manifest=fullfile(output_dir,'main_library_geometry_signature.json');
    if isfile(directory_manifest)
        previous=jsondecode(fileread(directory_manifest));
        if ~isfield(previous,'signature')||~strcmp(previous.signature,common_signature)
            error('build_main_task_navigation_library:DirectorySignatureMismatch', ...
                'This directory contains another geometry/model/cache policy. Use a fresh output directory.');
        end
    else
        write_json(directory_manifest,struct('signature',common_signature,'builder_revision',revision, ...
            'cache_policy_revision',policy,'base_scene_signature',common_payload.scene_signature, ...
            'candidate_count',numel(pool),'scorer_sources',source_payload));
    end
    mat_file=fullfile(output_dir,['candidate_library_' mode '.mat']);
    pair_file=fullfile(output_dir,['pair_support_' mode '.csv']);
    candidate_file=fullfile(output_dir,['candidate_support_' mode '.csv']);
    proof_file=fullfile(output_dir,['ik_cache_' mode '.json']);
    if isfile(mat_file)
        stored=load(mat_file,'library','scene','proof','signature');
        if ~all(isfield(stored,{'library','scene','proof','signature'}))||~strcmp(stored.signature,signature)
            error('build_main_task_navigation_library:StaleCache', ...
                'Cached library does not match this pool/task/model/policy. Use a fresh output directory.');
        end
        validate_candidate_library(stored.library,regions,scene,robot,planner);
        if ~isequaln(stored.scene.tasks,scene.tasks)||~isequaln(stored.scene.grid_map,scene.grid_map)|| ...
                ~isequaln(stored.scene.map_info,scene.map_info)||~isequaln(stored.scene.start_pose,scene.start_pose)
            error('build_main_task_navigation_library:StaleCache','Cached scene differs from this request.');
        end
        if ~isfile(pair_file)||~isfile(candidate_file)||~isfile(proof_file)
            error('build_main_task_navigation_library:MissingArtifacts','Completed library is missing CSV/JSON evidence; restore it or use a fresh output directory.');
        end
        library=stored.library;scene=stored.scene;proof=stored.proof;
        validate_pool(library,pool);
        proof.reused_from_disk=true;proof.current_call_elapsed_time_s=toc(call_started);
        write_json(proof_file,proof);
        fprintf('MAIN %s: reused signed library, %d candidates, %d pairs\n',mode,numel(pool),proof.pair_count);
        return;
    end
    cache=[];pair_rows=cell(0,13);candidate_rows=cell(0,9);
    library=struct('regions',struct([]),'total_candidates',numel(pool),'scene_signature',scene_signature);
    scoring=tic;support_per_task=zeros(1,scene.num_tasks);pair_count=0;supported_pairs=0;
    expected_pairs=0;
    for ri=1:numel(regions)
        expected_pairs=expected_pairs+sum([pool.table_id]==regions(ri).table_id)*numel(regions(ri).task_ids);
    end
    for ri=1:numel(regions)
        region_clock=tic;indices=find([pool.table_id]==regions(ri).table_id);
        tids=reshape(regions(ri).task_ids,1,[]);candidates=struct([]);
        fprintf('MAIN %s table %d/%d: %d frozen candidates x %d tasks\n', ...
            mode,ri,numel(regions),numel(indices),numel(tids));
        for ci=1:numel(indices)
            entry=pool(indices(ci));reach=zeros(numel(tids),3);arms=cell(numel(tids),1);
            for ti=1:numel(tids)
                tid=tids(ti);before=cache_stats(cache);
                [ls,rs,cs,arm,d,cache]=evaluate_task_reachability_cached(entry.pose,scene.tasks(tid), ...
                    robot,planner.reachability.threshold,cache);
                if ~isfield(cache,'revision')||~strcmp(cache.revision,policy)
                    error('build_main_task_navigation_library:CachePolicy','Evaluator cache policy differs from the signed policy.');
                end
                if ~isscalar(cs)||~isfinite(cs)||cs<0|| ...
                        (cs>0&&~any(strcmp(arm,{'left','right'})))
                    error('build_main_task_navigation_library:EvaluatorResult','Invalid task reachability result.');
                end
                kind=cache_event(cache,before);reach(ti,:)=[ls,rs,cs];arms{ti}=arm;
                pair_count=pair_count+1;supported_pairs=supported_pairs+double(cs>0);
                support_per_task(tid)=support_per_task(tid)+double(cs>0);
                [pe,oe,step]=quality(d,arm);
                pair_rows(end+1,:)={mode,entry.candidate_id,tid,cs>0,arm, ...
                    side_field(d,'left','reason','not_evaluated'),side_field(d,'right','reason','not_evaluated'), ...
                    side_field(d,'left','failed_sample',0),side_field(d,'right','failed_sample',0),pe,oe,step,kind}; %#ok<AGROW>
            end
            covered=tids(reach(:,3)>0);
            c=struct('pose',entry.pose,'table_id',entry.table_id,'side_id',entry.side_id, ...
                'covered_tasks',covered,'reach_scores',reach,'arm_assignment',{arms}, ...
                'nominal_covered_tasks',covered,'nominal_reach_scores',reach,'robust_covered_tasks',[], ...
                'robust_reachability_summary',repmat(struct('minimum_score',NaN,'mean_score',NaN, ...
                    'success_fraction',NaN,'sample_count',0,'threshold',planner.reachability.threshold, ...
                    'geometric_only',false,'error_model_source','not_evaluated','error_model_measured',false),1,numel(tids)), ...
                'coverage_ratio',numel(covered)/max(numel(tids),1),'cost_breakdown',struct(), ...
                'total_cost',Inf,'candidate_id',entry.candidate_id,'dense_candidate_id',entry.dense_candidate_id);
            c.cost_breakdown=evaluate_docking_candidate(c,regions(ri),scene,robot,planner);
            c.total_cost=c.cost_breakdown.total;
            if isempty(candidates),candidates=c;else,candidates(end+1)=c;end %#ok<AGROW>
            candidate_rows(end+1,:)={mode,entry.candidate_id,entry.table_id,entry.pose(1),entry.pose(2), ...
                entry.pose(3),numel(covered),numel(tids),entry.dense_candidate_id}; %#ok<AGROW>
            if mod(ci,12)==0||ci==numel(indices)
                s=cache_stats(cache);
                fprintf('  %s table %d: %d/%d candidates, pairs %d/%d, hits +%d/-%d\n', ...
                    mode,regions(ri).table_id,ci,numel(indices),pair_count,expected_pairs, ...
                    field_or(s,'positive_hits',0),field_or(s,'exact_negative_hits',0));
            end
        end
        region_entry=struct('region_id',ri,'table_id',regions(ri).table_id,'region',regions(ri), ...
            'candidates',candidates,'num_candidates',numel(candidates),'task_ids',tids, ...
            'generation_time_s',toc(region_clock),'generation_diagnostics',struct('all_frozen_pairs_evaluated',true));
        if isempty(library.regions),library.regions=region_entry;else,library.regions(end+1)=region_entry;end
        write_tables();
    end
    assert(pair_count==expected_pairs,'Frozen pair denominator changed.');
    library.total_generation_time_s=toc(scoring);library.baseline_generation_time_s=library.total_generation_time_s;
    library.augmentation_diagnostics=struct('enabled',false,'reason','frozen_geometric_subpool');
    validate_candidate_library(library,regions,scene,robot,planner);validate_pool(library,pool);
    proof=cache_stats(cache);
    proof.mode=mode;proof.cache_policy_revision=policy;proof.builder_revision=revision;
    proof.elapsed_time_s=library.total_generation_time_s;proof.current_call_elapsed_time_s=toc(call_started);
    proof.pair_count=pair_count;proof.expected_pair_count=expected_pairs;proof.supported_pairs=supported_pairs;
    proof.candidate_count=numel(pool);proof.target_count=scene.num_tasks;
    proof.union_supported_count=sum(support_per_task>0);proof.full_union=all(support_per_task>0);
    proof.unsupported_task_ids=find(support_per_task==0);proof.per_task_support_counts=support_per_task;
    proof.all_pairs_preserved=true;proof.reused_from_disk=false;proof.signature=signature;
    proof.scene_signature=scene_signature;proof.geometry_directory_signature=common_signature;
    proof.scorer_sources=source_payload;proof.arm_collision_checked=false;proof.independent_ik_validation=false;
    proof.evidence_scope='finite-search task IK plus per-hit FK witness checks; not independent IK, continuous motion or physical execution';
    write_json(proof_file,proof);
    save(mat_file,'library','scene','proof','signature','signature_payload','-v7');
    fprintf('MAIN %s LIBRARY COMPLETE: %d/%d pairs, %d/%d tasks supported, %.2fs\n', ...
        mode,supported_pairs,pair_count,proof.union_supported_count,scene.num_tasks,proof.elapsed_time_s);

    function write_tables()
        writetable(cell2table(pair_rows,'VariableNames',{'mode','candidate_id','task_id','feasible','assigned_arm', ...
            'reason_left','reason_right','failed_sample_left','failed_sample_right', ...
            'max_position_error_m','max_orientation_error_rad','max_joint_step_rad','cache_kind'}),pair_file);
        writetable(cell2table(candidate_rows,'VariableNames',{'mode','candidate_id','table_id','x_m','y_m', ...
            'yaw_rad','support_count','task_count','dense_candidate_id'}),candidate_file);
    end
end

function validate_pool(library,pool)
    at=0;
    for ri=1:numel(library.regions)
        cs=library.regions(ri).candidates;
        for ci=1:numel(cs)
            at=at+1;
            if at>numel(pool)||~isequal(cs(ci).pose,pool(at).pose)|| ...
                    cs(ci).candidate_id~=pool(at).candidate_id|| ...
                    cs(ci).dense_candidate_id~=pool(at).dense_candidate_id||cs(ci).side_id~=pool(at).side_id
                error('build_main_task_navigation_library:PoolOrder','Scored library does not retain the frozen pool order.');
            end
        end
    end
    if at~=numel(pool),error('build_main_task_navigation_library:PoolOrder','A frozen candidate was dropped.');end
end
function stats=cache_stats(cache)
    stats=struct();if isstruct(cache)&&isfield(cache,'stats'),stats=cache.stats;end
end
function event=cache_event(cache,before)
    if isstruct(cache)&&isfield(cache,'last_event'),event=char(cache.last_event);return;end
    after=cache_stats(cache);event='miss';
    if field_or(after,'positive_hits',0)>field_or(before,'positive_hits',0),event='positive_hit';
    elseif field_or(after,'exact_negative_hits',0)>field_or(before,'exact_negative_hits',0),event='exact_negative_hit';
    elseif field_or(after,'witness_fallbacks',0)>field_or(before,'witness_fallbacks',0),event='witness_fallback';end
end
function value=side_field(details,side,name,fallback)
    value=fallback;if isstruct(details)&&isfield(details,side)&&isstruct(details.(side))
        value=field_or(details.(side),name,fallback);
    end
end
function [pe,oe,step]=quality(details,arm)
    pe=NaN;oe=NaN;step=NaN;
    if ~any(strcmp(arm,{'left','right'}))||~isfield(details,arm),return;end
    d=details.(arm);p=field_or(d,'position_errors_m',[]);o=field_or(d,'orientation_errors_rad',[]);
    if ~isempty(p),pe=max(p);end
    if ~isempty(o),oe=max(o);end
    q=field_or(d,'q_path',[]);step=0;
    if size(q,1)>1,step=max(abs(diff(q)),[],'all');end
end
function value=field_or(s,name,fallback)
    if isstruct(s)&&isfield(s,name),value=s.(name);else,value=fallback;end
end
function write_json(path,data)
    fid=fopen(path,'w','n','UTF-8');
    if fid<0,error('build_main_task_navigation_library:WriteFailed','Cannot write %s.',path);end
    cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
    fprintf(fid,'%s\n',jsonencode(data,'PrettyPrint',true));
end
