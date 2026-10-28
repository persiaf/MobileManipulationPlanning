function info=save_repair_dynamic_pair(path,payload)
%SAVE_REPAIR_DYNAMIC_PAIR Commit a compact, plain-data evidence snapshot.
% The snapshot is auditable input/output evidence, not a live planning state.
% Replaying requires regenerating candidates and run-local path caches.
% Temporary MAT-v7 data are reloaded before the same-directory rename.
    folder=fileparts(path); if isempty(folder),folder=pwd;end
    if ~isfolder(folder),mkdir(folder);end
    omitted=struct('path',{},'original_class',{},'reason',{});
    record=payload;
    for name={'state','changed','next','full_state'}
        if isfield(record,name{1}),record.(name{1})=compact_state(record.(name{1}));end
    end
    if isfield(record,'local_result')
        record.local_result=drop_fields(record.local_result, ...
            {'next_state','updated_sparse_graph','global_path','path_segments'});
        if isfield(record.local_result,'diagnostics')
            record.local_result.diagnostics=drop_fields(record.local_result.diagnostics, ...
                {'rebuilt_graph','next_state','candidate_library'});
        end
    end
    record=plain(record,'pair');
    record.snapshot_schema='dynamic_plain_evidence_v1';
    record.snapshot_replayable_live_state=false;
    record.snapshot_replay_requirement='Regenerate candidates and run-local caches from saved scene, robot_cfg and planner_cfg before planning.';
    record.snapshot_omitted_fields={'candidate_library','stats.planning_state','stats.rc2_plan', ...
        'plan.shared_path_cost_context','sparse_graph.path_search_states','duplicate result paths and next_state'};
    record.snapshot_omitted_objects=omitted;
    temporary=[tempname(folder) '.mat'];
    cleanup=onCleanup(@()remove_temporary(temporary)); %#ok<NASGU>
    save(temporary,'-struct','record','-v7');
    checked=load(temporary);
    if ~isequal(sort(fieldnames(checked)),sort(fieldnames(record))) || ...
            ~strcmp(checked.snapshot_schema,'dynamic_plain_evidence_v1')
        error('save_repair_dynamic_pair:ReloadFailed','Temporary evidence snapshot failed reload validation.');
    end
    for name={'event','local_row','full_row'}
        if isfield(record,name{1}) && ~isequaln(checked.(name{1}),record.(name{1}))
            error('save_repair_dynamic_pair:ReloadMismatch','Reload mismatch in %s.',name{1});
        end
    end
    assert_plain(checked,'pair');
    [ok,message]=movefile(temporary,path,'f');
    if ~ok,error('save_repair_dynamic_pair:CommitFailed','%s',message);end
    entry=dir(path);
    info=struct('path',path,'bytes',entry.bytes,'schema',record.snapshot_schema, ...
        'reload_validated',true,'omitted_object_count',numel(omitted));

    function value=plain(value,location)
        if isa(value,'containers.Map')
            map_keys=keys(value); map_values=values(value,map_keys);
            value=struct('encoding','containers.Map_as_pairs','key_type',value.KeyType, ...
                'value_type',value.ValueType,'keys',{map_keys},'values',{map_values});
            value=plain(value,location);
        elseif isstruct(value)
            names=fieldnames(value);
            for vi=1:numel(value)
                for fi=1:numel(names)
                    name=names{fi};value(vi).(name)=plain(value(vi).(name),[location '.' name]);
                end
            end
        elseif iscell(value)
            for vi=1:numel(value),value{vi}=plain(value{vi},sprintf('%s{%d}',location,vi));end
        elseif isstring(value)
            if isscalar(value),value=char(value);else,value=cellstr(value);end
        elseif isnumeric(value) || islogical(value) || ischar(value)
            return;
        else
            original_class=class(value);
            reason='Non-plain object omitted explicitly; inspect original class before using this evidence.';
            if isa(value,'SharedPathCostContext'),reason='Run-local mutable path-cost cache omitted.';end
            omitted(end+1)=struct('path',location,'original_class',original_class,'reason',reason); %#ok<AGROW>
            value=struct('snapshot_object_omitted',true,'original_class',original_class,'reason',reason);
        end
    end
end

function s=compact_state(s)
    names={'scene','robot_cfg','planner_cfg','all_poses','task_state','path_validation', ...
        'regions','task_table_map','scale','seed','invalidated_coverage_pairs'};
    out=pick_fields(s,names);
    if isfield(s,'plan')
        out.plan=pick_fields(s.plan,{'status','reason','selected_poses','task_sequence_summary', ...
            'covered_tasks_predicted','uncovered_tasks_predicted','service_configs','pose_to_region_map', ...
            'num_docking_poses','strategy_name','objective','objective_value','reachability_mode'});
    end
    if isfield(s,'stats')
        out.stats=pick_fields(s.stats,{'route_order','visit_order','global_path','segment_stats', ...
            'path_len','path_validation','timing','evidence','task_state'});
        if isfield(s.stats,'sparse_graph')
            out.stats.sparse_graph=pick_fields(s.stats.sparse_graph,{'mode','options','region_ids', ...
                'geometric_adjacency','adjacency','cost_matrix','shortest_cost_matrix','next_hop', ...
                'direct_paths','connectivity','geometric_connectivity','stats'});
        end
    end
    s=out;
end

function out=pick_fields(s,names)
    out=struct();
    for k=1:numel(names),if isfield(s,names{k}),out.(names{k})=s.(names{k});end,end
end

function s=drop_fields(s,names)
    if ~isstruct(s),return;end
    keep=intersect(fieldnames(s),names,'stable');
    if ~isempty(keep),s=rmfield(s,keep);end
end

function assert_plain(value,location)
    if isstruct(value)
        names=fieldnames(value);
        for vi=1:numel(value),for fi=1:numel(names),assert_plain(value(vi).(names{fi}),[location '.' names{fi}]);end,end
    elseif iscell(value)
        for vi=1:numel(value),assert_plain(value{vi},location);end
    elseif ~(isnumeric(value) || islogical(value) || ischar(value))
        error('save_repair_dynamic_pair:NonPlainReload','Unexpected %s at %s.',class(value),location);
    end
end

function remove_temporary(path)
    if isfile(path),delete(path);end
end
