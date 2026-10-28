function tests=test_protocol_dynamic_comparison
    tests=functiontests(localfunctions);
end

function setupOnce(testCase)
    [scene,~,cfg]=create_test_fixture();
    out=tempname; mkdir(out);
    options=struct('output_dir',out,'seeds',2026,'scene_id','ONE_TABLE_TEST_FIXTURE', ...
        'scene_cfg',scene.scene_cfg,'planner_cfg',cfg,'run_base_audit',false, ...
        'events',{{'ADD_LOCAL_OBSTACLE'}});
    raw=run_protocol_dynamic_comparison(options);
    testCase.TestData.output_dir=out;
    testCase.TestData.raw=raw;
    testCase.TestData.pair=load(fullfile(out,'seed2026_ADD_LOCAL_OBSTACLE_pair.mat'));
end

function testPairedTimersExcludeIdenticalExternalCertification(testCase)
    raw=testCase.TestData.raw; p=testCase.TestData.pair;
    verifyEqual(testCase,height(raw),2);
    verifyEqual(testCase,raw.mode,{'LOCAL_REPLAN';'FULL_REPLAN'});
    verifyEqual(testCase,p.local_row.timing_scope,p.full_row.timing_scope);
    verifyEqual(testCase,p.local_row.certification_scope,p.full_row.certification_scope);
    verifyTrue(testCase,contains(p.local_row.excluded_time_scope,'external_certification'));
    verifyEqual(testCase,p.local_row.replanning_time_s,p.local_row.wall_time_s);
    verifyEqual(testCase,p.full_row.wall_time_s-p.full_row.replanning_time_s, ...
        p.full_row.execution_sim_time_s_excluded,'AbsTol',1e-12);
    verifyGreaterThanOrEqual(testCase,p.local_row.certification_time_s,0);
    verifyGreaterThanOrEqual(testCase,p.full_row.certification_time_s,0);
    verifyTrue(testCase,p.full_row.certification_checked);
    verifyTrue(testCase,p.full_certification.valid,p.full_certification.reason);
    if any(strcmp(p.local_result.status,{'SUCCESS','NOOP'}))
        verifyTrue(testCase,p.local_row.certification_checked);
        verifyEqual(testCase,p.local_row.path_safe,p.local_certification.valid);
    else
        verifyFalse(testCase,p.local_row.certification_checked);
    end
    verifyTrue(testCase,all(isnan(raw.actual_completion_percent)));
    verifyTrue(testCase,all(isnan(raw.ik_feasibility_percent)));
end

function testQueryRequestsAndPairedPlanningStatesRemainAuditable(testCase)
    p=testCase.TestData.pair;
    verifyTrue(testCase,all(isfield(p,{'state','changed','next','full_case','full_state', ...
        'local_result','impact','local_certification','full_certification'})));
    verifyEqual(testCase,p.local_row.path_query_scope,p.full_row.path_query_scope);
    verifyTrue(testCase,contains(p.local_row.path_query_scope,'all_attempted_candidate_graphs'));
    verifyTrue(testCase,p.local_result.path_query_count_known);
    verifyTrue(testCase,p.local_row.path_queries_known);
    verifyEqual(testCase,p.local_row.path_queries,p.local_result.path_query_count_total);
    verifyEqual(testCase,p.full_row.path_queries,p.full_state.stats.sparse_graph.stats.actual_path_queries);
    verifyTrue(testCase,p.full_row.path_queries_known);
    verifyEqual(testCase,p.full_state.scene.grid_map,p.changed.scene.grid_map);
    verifyEqual(testCase,p.next.scene.grid_map,p.changed.scene.grid_map);
    verifyFalse(testCase,isequal(p.state.scene.grid_map,p.changed.scene.grid_map));
    verifyEqual(testCase,p.full_state.robot_cfg,p.state.robot_cfg);
    verifyEqual(testCase,[[p.full_state.scene.tasks.x]' [p.full_state.scene.tasks.y]' [p.full_state.scene.tasks.z]'], ...
        [[p.changed.scene.tasks.x]' [p.changed.scene.tasks.y]' [p.changed.scene.tasks.z]']);
    verifyTrue(testCase,isfile(fullfile(testCase.TestData.output_dir,'dynamic_raw.csv')));
end

function testCompactPairReloadsWithoutLiveCaches(testCase)
    p=testCase.TestData.pair;
    verifyEqual(testCase,p.snapshot_schema,'dynamic_plain_evidence_v1');
    verifyFalse(testCase,p.snapshot_replayable_live_state);
    verifyTrue(testCase,contains(p.snapshot_replay_requirement,'Regenerate candidates'));
    verifyFalse(testCase,isfield(p.state.stats,'planning_state'));
    verifyFalse(testCase,isfield(p.state,'candidate_library'));
    verifyFalse(testCase,isfield(p.local_result,'next_state'));
    summary=p.state.plan.task_sequence_summary{1};
    verifyEqual(testCase,summary.arm_assign_map.encoding,'containers.Map_as_pairs');
    verifyEqual(testCase,numel(summary.arm_assign_map.keys),numel(summary.arm_assign_map.values));
    verifyTrue(testCase,is_plain(p));
    entry=dir(fullfile(testCase.TestData.output_dir,'seed2026_ADD_LOCAL_OBSTACLE_pair.mat'));
    verifyLessThan(testCase,entry.bytes,5e6);
end

function testUnknownObjectsAreExplicitlyMarked(testCase)
    path=[tempname '.mat'];
    payload=struct('event',struct('type','TEST'),'local_row',struct('status','TEST'), ...
        'full_row',struct('status','TEST'),'unclassified_object',@sin);
    info=save_repair_dynamic_pair(path,payload); loaded=load(path);
    verifyTrue(testCase,info.reload_validated);
    verifyTrue(testCase,loaded.unclassified_object.snapshot_object_omitted);
    verifyEqual(testCase,loaded.unclassified_object.original_class,'function_handle');
    verifyEqual(testCase,loaded.snapshot_omitted_objects.path,'pair.unclassified_object');
    verifyTrue(testCase,is_plain(loaded));
end

function ok=is_plain(value)
    ok=true;
    if isstruct(value)
        names=fieldnames(value);
        for vi=1:numel(value),for fi=1:numel(names),ok=ok && is_plain(value(vi).(names{fi}));end,end
    elseif iscell(value)
        for vi=1:numel(value),ok=ok && is_plain(value{vi});end
    else
        ok=isnumeric(value) || islogical(value) || ischar(value);
    end
end
