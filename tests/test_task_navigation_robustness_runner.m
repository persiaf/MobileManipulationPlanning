function tests=test_task_navigation_robustness_runner
    tests=functiontests(localfunctions);
end

function setupOnce(testCase)
    root=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(root,'pipeline'));init_paths(root);
end

function testInjectedSetupRetainsModesAndSkipsFullSnapshots(testCase)
    cfg=scene_config();cfg.table_positions=[2,2];cfg.table_orientations=0;cfg.num_tables=1;
    cfg.task_mode='manual';cfg.manual_tasks=[1,2,2.12,.5];
    setup=main_task_navigation_setup(cfg);P=reshape([setup.pool.pose],3,[])';
    [~,index]=min(vecnorm(P-[2,2.5,-pi/2],2,2));chosen=setup.pool(index);
    chosen.candidate_id=1;setup.pool=chosen;
    setup.dense_mapping.selected(:)=false;setup.dense_mapping.candidate_id(:)=0;
    row=setup.dense_mapping.dense_candidate_id==chosen.dense_candidate_id;
    setup.dense_mapping.selected(row)=true;setup.dense_mapping.candidate_id(row)=1;
    setup.table_counts.retained_candidates(:)=1;
    folder=tempname;mkdir(folder);cleanup=onCleanup(@()rmdir(folder,'s')); %#ok<NASGU>
    result=exp_main_task_navigation(folder,struct('setup',setup,'write_checkpoints',false,'retain_full_mat',false));
    summary=readtable(fullfile(folder,'summary.csv'),'TextType','string');
    verifyEqual(testCase,height(summary),12);verifyEqual(testCase,size(result.case_results),[3,4]);
    verifyEqual(testCase,unique(summary.target_count),1);
    verifyFalse(testCase,isfile(fullfile(folder,'main_joint_result.mat')));
    verifyEmpty(testCase,dir(fullfile(folder,'checkpoint_*.mat')));
    for mode={'position_only','top_down_grasp','table_wipe'}
        verifyTrue(testCase,isfile(fullfile(folder,['effective_tasks_' mode{1} '.csv'])));
    end
    state=jsondecode(fileread(fullfile(folder,'run_state.json')));verifyEqual(testCase,state.status,'COMPLETE');
end

function testCollectorCombinesMissingAndTextFailureReasons(testCase)
    folder=tempname;mkdir(folder);cleanup=onCleanup(@()rmdir(folder,'s')); %#ok<NASGU>
    rows=cell(9,5);
    for i=1:9
        id=sprintf('case%d',i);relative=fullfile('cases',id);child=fullfile(folder,relative);mkdir(child);
        record_task_navigation_robustness_failure(child,struct('case_id',id,'task_count',36), ...
            'fixture',MException('RobustnessTest:Expected','Expected fixture'));
        if i==1
            current=readtable(fullfile(child,'summary.csv'),'TextType','string');
            current.failure_reason=strings(12,1);writetable(current,fullfile(child,'summary.csv'));
        end
        rows(i,:)={id,floor((i-1)/3),mod(i-1,3),2026,relative};
    end
    writetable(cell2table(rows,'VariableNames',{'case_id','layout_id','distribution_id','seed','relative_dir'}), ...
        fullfile(folder,'scenario_manifest.csv'));
    combined=collect_task_navigation_robustness(folder);
    verifyEqual(testCase,height(combined),108);verifyClass(testCase,combined.failure_reason,'string');
    state=jsondecode(fileread(fullfile(folder,'run_state.json')));verifyEqual(testCase,state.status,'COMPLETE');
    verifyEqual(testCase,state.completed_scenarios,9);verifyEqual(testCase,state.completed_rows,108);
end

function testPreparationFailureRetainsAllPlannedRowsAndNaNCosts(testCase)
    folder=tempname;mkdir(folder);cleanup=onCleanup(@()rmdir(folder,'s')); %#ok<NASGU>
    definition=struct('case_id','test_failure','task_count',36);
    err=MException('RobustnessTest:Expected','Expected preparation failure fixture');
    record_task_navigation_robustness_failure(folder,definition,'library_fixture',err);
    summary=readtable(fullfile(folder,'summary.csv'),'TextType','string');
    verifyEqual(testCase,height(summary),12);verifyTrue(testCase,all(isnan(summary.navigation_cost)));
    verifyTrue(testCase,all(isnan(summary.route_length_m)));verifyFalse(testCase,any(summary.mission_valid));
    verifyEqual(testCase,unique(summary.target_count),36);
    state=jsondecode(fileread(fullfile(folder,'run_state.json')));verifyTrue(testCase,state.preparation_failed);
    verifyEqual(testCase,state.valid_missions,0);
end
