function tests=test_task_navigation_robustness_protocol
% Fixed factorial geometry/task fairness and process-local cache IO only.
% Does not score the nine complete IK libraries.
    tests=functiontests(localfunctions);
end

function setupOnce(testCase)
    cases=task_navigation_robustness_cases();refs=cell(1,3);setups=cell(1,9);
    project_root=fileparts(fileparts(mfilename('fullpath')));
    prior=fullfile(project_root,'results','main_joint_task_navigation_20261111','main_setup.mat');
    if isfile(prior),data=load(prior,'setup');refs{1}=task_navigation_robustness_setup(cases(1),data.setup);
    else,refs{1}=task_navigation_robustness_setup(cases(1));end
    for li=1:3
        at=(li-1)*3+1;
        if li>1,refs{li}=task_navigation_robustness_setup(cases(at));end
        setups{at}=refs{li};
        setups{at+1}=task_navigation_robustness_setup(cases(at+1),refs{li});
        setups{at+2}=task_navigation_robustness_setup(cases(at+2),refs{li});
    end
    testCase.TestData.cases=cases;testCase.TestData.references=refs;testCase.TestData.setups=setups;
end

function testCompletePredeclaredFactorialPreservesTaskCounts(testCase)
    cases=testCase.TestData.cases;verifyEqual(testCase,numel(cases),9);
    verifyEqual(testCase,numel(unique({cases.case_id})),9);
    verifyEqual(testCase,unique([[cases.layout_id]',[cases.distribution_id]'],'rows'), ...
        [0 0;0 1;0 2;1 0;1 1;1 2;2 0;2 1;2 2]);
    for k=1:9
        s=testCase.TestData.setups{k};c=cases(k);
        verifyEqual(testCase,s.scene.num_tasks,36);verifyEqual(testCase,c.task_count,36);
        verifyEqual(testCase,c.per_table_task_counts,cases(1).per_table_task_counts);
        verifyEqual(testCase,[s.scene.tasks.id],1:36);
        verifyEqual(testCase,reshape([s.scene.tasks.table_id],[],1),c.task_table_ids);
        verifyEqual(testCase,s.source_scene_cfg,c.scene_cfg);
    end
end

function testNoArgumentMainSetupAndOriginalReferenceRemainIdentical(testCase)
    fresh=main_task_navigation_setup();reference=testCase.TestData.references{1};
    verifyEqual(testCase,reference.source_scene_cfg,scene_config());
    verifyEqual(testCase,reference.scene,fresh.scene);
    verifyEqual(testCase,reference.pool,fresh.pool);
    verifyEqual(testCase,reference.dense_mapping,fresh.dense_mapping);
    verifyEqual(testCase,reference.robot,fresh.robot);
    verifyEqual(testCase,numel(reference.pool),906);
    verifyEqual(testCase,height(reference.dense_mapping),26660);
end

function testEachLayoutHasOneSharedPoolAcrossAllDistributions(testCase)
    for li=1:3
        reference=testCase.TestData.references{li};
        for di=1:3
            s=testCase.TestData.setups{(li-1)*3+di};
            verifyEqual(testCase,s.pool,reference.pool);
            verifyEqual(testCase,s.dense_mapping,reference.dense_mapping);
            verifyEqual(testCase,s.table_counts,reference.table_counts);
            verifyEqual(testCase,s.reference_pool_signature,reference.reference_pool_signature);
            verifyEqual(testCase,s.reference_scene_cfg,reference.reference_scene_cfg);
            verifyEqual(testCase,s.robot,reference.robot);
            verifyEqual(testCase,s.planner,reference.planner);
        end
    end
end

function testAllLocalTaskAnchorsAndWipeSamplesStayOnTheirTables(testCase)
    tcp=[];
    for k=1:9
        s=testCase.TestData.setups{k};cfg=s.scene.scene_cfg;c=s.case_definition;
        verifyTrue(testCase,s.wipe_paths_inside_table);
        for ti=1:36
            anchor=s.scene.tasks(ti);t=anchor.table_id;
            local=(rot2(-cfg.table_orientations(t))* ...
                ([anchor.x,anchor.y]-cfg.table_positions(t,:))')';
            verifyEqual(testCase,local,c.task_local_xy(ti,:),'AbsTol',1e-12);
            for mode=s.mode_order
                task=comparison_manipulation_task(anchor,mode{1});r=resolve_manipulation_task(task);
                verifyEqual(testCase,[task.x,task.y,task.z],[anchor.x,anchor.y,anchor.z]);
                if isempty(tcp),tcp=r.T_ee_tcp;end
                verifyEqual(testCase,r.T_ee_tcp,tcp);
                if ~strcmp(mode{1},'table_wipe'),continue;end
                local_path=(rot2(-cfg.table_orientations(t))* ...
                    (r.samples_world(:,1:2)-cfg.table_positions(t,:))')';
                verifyLessThanOrEqual(testCase,max(abs(local_path(:,1))),cfg.table.length/2+1e-12);
                verifyLessThanOrEqual(testCase,max(abs(local_path(:,2))),cfg.table.width/2+1e-12);
                verifyEqual(testCase,norm(diff(r.waypoints_world)),.12,'AbsTol',1e-12);
            end
        end
    end
end

function testReferencePoolHasValidFootprintsAndUnchangedBudget(testCase)
    for li=1:3
        s=testCase.TestData.references{li};pool=s.pool;
        verifyEqual(testCase,[pool.candidate_id],1:numel(pool));
        for t=1:16
            entries=pool([pool.table_id]==t);verifyLessThanOrEqual(testCase,numel(entries),64);
            for side=1:4
                side_entries=entries([entries.side_id]==side);if isempty(side_entries),continue;end
                P=reshape([side_entries.pose],3,[])';
                [xy,~,groups]=unique(round(P(:,1:2)*1e10)/1e10,'rows');
                verifyLessThanOrEqual(testCase,size(xy,1),8);
                verifyLessThanOrEqual(testCase,max(accumarray(groups,1)),2);
            end
        end
        for pi=1:numel(pool)
            verifyFalse(testCase,check_footprint_collision(pool(pi).pose,s.scene.grid_map,s.scene.map_info,s.robot));
        end
    end
end

function testWrongLayoutReferenceAndOffTableWipeAreRejected(testCase)
    cases=testCase.TestData.cases;refs=testCase.TestData.references;
    verifyError(testCase,@()task_navigation_robustness_setup(cases(4),refs{1}), ...
        'task_navigation_robustness_setup:Reference');
    bad=cases(2);t=bad.scene_cfg.manual_tasks(1,1);
    bad.scene_cfg.manual_tasks(1,2)=bad.scene_cfg.table_positions(t,1)+.58;
    verifyError(testCase,@()task_navigation_robustness_setup(bad,refs{1}), ...
        'task_navigation_robustness_setup:WipeOutsideTable');
end

function testExistingSignedMainLibraryIsCompatibleAndUntouched(testCase)
    source=fullfile(fileparts(fileparts(mfilename('fullpath'))),'results','main_joint_task_navigation_20261111');
    assumeTrue(testCase,isfile(fullfile(source,'candidate_library_position_only.mat')));
    folder=tempname;mkdir(folder);cleanup=onCleanup(@()rmdir(folder,'s')); %#ok<NASGU>
    names={'main_library_geometry_signature.json','candidate_library_position_only.mat', ...
        'pair_support_position_only.csv','candidate_support_position_only.csv','ik_cache_position_only.json'};
    for k=1:numel(names),copyfile(fullfile(source,names{k}),fullfile(folder,names{k}));end
    [library,scene,proof,cache]=build_main_task_navigation_library( ...
        testCase.TestData.references{1},'position_only',folder);
    verifyTrue(testCase,proof.reused_from_disk);verifyEmpty(testCase,cache);
    verifyEqual(testCase,library.total_candidates,906);verifyEqual(testCase,scene.num_tasks,36);
    verifyEqual(testCase,proof.cache_scope,'signed_library_replay_without_new_witness_validation');
    verifyEqual(testCase,proof.supported_pairs,1420);
end

function testWarmWitnessReuseResetsCountersAndNeverSerializesMap(testCase)
    mini=tiny_setup(testCase.TestData.references{1});a=tempname;b=tempname;mkdir(a);mkdir(b);
    ca=onCleanup(@()rmdir(a,'s'));cb=onCleanup(@()rmdir(b,'s')); %#ok<NASGU>
    [first,~,cold,cache]=build_main_task_navigation_library(mini,'position_only',a);
    verifyEqual(testCase,cold.request_count,1);verifyEqual(testCase,cold.warm_bucket_count,0);
    verifyEqual(testCase,cold.supported_pairs,1);verifyEqual(testCase,cache.stats.request_count,1);
    [second,~,warm,cache]=build_main_task_navigation_library(mini,'position_only',b,cache);
    verifyEqual(testCase,second.regions(1).candidates.covered_tasks,first.regions(1).candidates.covered_tasks);
    verifyEqual(testCase,warm.request_count,1);verifyEqual(testCase,warm.positive_hits,1);
    verifyEqual(testCase,warm.misses,0);verifyEqual(testCase,warm.direct_checks,0);
    verifyGreaterThan(testCase,warm.warm_bucket_count,0);
    verifyTrue(testCase,warm.cache_counters_reset_per_library);
    verifyEqual(testCase,cache.stats.request_count,1);
    info=whos('-file',fullfile(b,'candidate_library_position_only.mat'));
    verifyFalse(testCase,ismember('cache',{info.name}));
    cache.stats.request_count=999;before=cache;
    [~,~,reused,after]=build_main_task_navigation_library(mini,'position_only',b,cache);
    verifyTrue(testCase,reused.reused_from_disk);
    verifyFalse(testCase,reused.cache_counters_reset_per_library);
    verifyEqual(testCase,reused.request_count,1);
    verifyEqual(testCase,after.stats,before.stats);
    verifyEqual(testCase,after.buckets.Count,before.buckets.Count);
end

function setup=tiny_setup(source)
    setup=source;task=source.scene.tasks(source.regions(1).task_ids(1));task.id=1;
    setup.scene.tasks=task;setup.scene.num_tasks=1;
    setup.regions=source.regions(1);setup.regions.task_ids=1;
    setup.pool=source.pool(1);setup.pool.candidate_id=1;
end
