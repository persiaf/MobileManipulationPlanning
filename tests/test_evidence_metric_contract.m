function tests = test_evidence_metric_contract
    tests = functiontests(localfunctions);
end

function testSimulationDoesNotBecomePhysicalEvidence(testCase)
    status = separate_execution_status([1,2],[1,2],[1,2],1);
    e = build_execution_evidence_metrics(status,2);
    verifyEqual(testCase,status.executed_tasks,[1,2]);
    verifyEqual(testCase,status.verified_tasks,1);
    verifyEqual(testCase,e.geometric_coverage_rate,100);
    verifyEqual(testCase,e.simulated_geometric_completion_rate,50);
    verifyTrue(testCase,isnan(e.ik_feasibility_rate));
    verifyTrue(testCase,isnan(e.actual_completion_rate));
    verifyTrue(testCase,isnan(e.proxy_coverage_rate));
    verifyFalse(testCase,e.actual_execution_measured);
    verifyEqual(testCase,status.legacy_verified_semantics,'simulated_geometric_acceptance');
end

function testExplicitMeasuredSourceHasSeparateRate(testCase)
    status = separate_execution_status([], [1,2], [1,2], 1, ...
        struct('source','measured_execution'));
    e = build_execution_evidence_metrics(status,2);
    verifyEqual(testCase,e.actual_completion_rate,50);
    verifyTrue(testCase,e.measured_evidence_available);
    verifyTrue(testCase,isnan(e.simulated_geometric_completion_rate));
    verifyTrue(testCase,isnan(e.ik_feasibility_rate));
end

function testProxyAndIKAreIndependentKinds(testCase)
    proxy = build_execution_evidence_metrics(separate_execution_status([1,2],[],[],[], ...
        struct('source','proxy_support')),2);
    verifyEqual(testCase,proxy.proxy_coverage_rate,100);
    verifyTrue(testCase,isnan(proxy.geometric_coverage_rate));
    verifyTrue(testCase,isnan(proxy.actual_completion_rate));
    ik = build_execution_evidence_metrics(separate_execution_status([],[],[1,2],1, ...
        struct('source','ik_validation')),2);
    verifyEqual(testCase,ik.ik_checked_coverage_rate,100);
    verifyEqual(testCase,ik.ik_feasibility_rate,50);
    verifyTrue(testCase,isnan(ik.simulated_geometric_completion_rate));
    verifyTrue(testCase,isnan(ik.actual_completion_rate));
end

function testSimulationOutputsAreExplicitlyLabeled(testCase)
    robot = robot_config(); planner = planner_config();
    scene.num_tasks = 1;
    scene.tasks = struct('id',1,'x',robot.arm_base_mount_x+0.15, ...
        'y',robot.dual_arm.base_spacing/2,'z',robot.arm_base_mount_z+0.096);
    [executed,verified,rows,e] = simulate_planned_task_execution(scene,1,[0,0,0],robot,planner);
    verifyEqual(testCase,executed,1); verifyEqual(testCase,verified,1);
    verifyEqual(testCase,rows.evidence_source,'geometric_simulation');
    verifyTrue(testCase,rows.geometric_accepted);
    verifyFalse(testCase,rows.ik_checked);
    verifyTrue(testCase,isnan(rows.measured_completed));
    verifyFalse(testCase,e.measured_evidence_available);
end

function testMissingStatusNeverInventsVerification(testCase)
    [scene,poses,path,exec,cfg] = fixture(); %#ok<ASGLU>
    stats = build_experiment_stats(scene,[1,2],poses,path,exec,struct(), ...
        struct([]),{},[]);
    verifyEqual(testCase,stats.task.verified_tasks,0);
    verifyEqual(testCase,stats.task.executed_tasks,2);
    verifyTrue(testCase,isnan(stats.task.verified_completion_rate));
    verifyTrue(testCase,isnan(stats.task.actual_completion_rate));
    verifyTrue(testCase,isnan(stats.planning.total_planning_time_s));
    verifyFalse(testCase,stats.planning.total_time_available);
end

function testPartialOrUnsafeMissionIsNotSuccess(testCase)
    [scene,poses,path,exec,cfg] = fixture();
    stats = struct('exec_stats',exec,'path_validation',struct('valid',true), ...
        'execution_status',separate_execution_status([1,2],[1,2],[1,2],1));
    partial = build_result_record(scene,poses,[1,2],path,1,stats,cfg,{},[]);
    verifyFalse(testCase,partial.success);
    verifyEqual(testCase,partial.completed_tasks,1);
    verifyEqual(testCase,partial.completion_rate,50);
    stats.execution_status = separate_execution_status([1,2],[1,2],[1,2],[1,2]);
    good = build_result_record(scene,poses,[1,2],path,1,stats,cfg,{},[]);
    verifyTrue(testCase,good.success);
    verifyTrue(testCase,isnan(good.actual_completion_rate));
    stats.path_validation.valid = false;
    bad = build_result_record(scene,poses,[1,2],path,1,stats,cfg,{},[]);
    verifyFalse(testCase,bad.success);
    verifyTrue(testCase,bad.path_validation_available);
    verifyFalse(testCase,bad.path_safe);
end

function testLegacyExecutedArgumentDoesNotMeanCompleted(testCase)
    [scene,poses,path,exec,cfg] = fixture();
    record = build_result_record(scene,poses,[1,2],path,1,struct('exec_stats',exec),cfg,{},[]);
    verifyFalse(testCase,record.success);
    verifyEqual(testCase,record.completed_tasks,0);
    verifyEqual(testCase,record.executed_task_count,2);
    verifyTrue(testCase,isnan(record.completion_rate));
    verifyTrue(testCase,isnan(record.verified_completion_rate));
    verifyTrue(testCase,isnan(record.predicted_completion_rate));
end

function testMissingTimeUnknownAndMeasuredZeroPreserved(testCase)
    [scene,poses,path,exec,cfg] = fixture();
    status = separate_execution_status([1,2],[1,2],[1,2],[1,2]);
    stats = struct('exec_stats',exec,'execution_status',status);
    unknown = build_result_record(scene,poses,[1,2],path,1,stats,cfg,{},[]);
    verifyTrue(testCase,isnan(unknown.planning_time_total));
    verifyFalse(testCase,unknown.planning_time_available);
    stats.timing = struct('total_planning_time_s',0,'timing_scope','end_to_end', ...
        'timing_source','wall_clock');
    measured = build_result_record(scene,poses,[1,2],path,1,stats,cfg,{},[]);
    verifyEqual(testCase,measured.planning_time_total,0);
    verifyTrue(testCase,measured.planning_time_available);
    verifyEqual(testCase,measured.timing_scope,'end_to_end');
end

function testCanonicalAlignmentKeepsFlatEvidence(testCase)
    [scene,poses,path,exec,cfg] = fixture();
    status = separate_execution_status([1,2],[1,2],[1,2],[1,2]);
    record = build_result_record(scene,poses,[1,2],path,1, ...
        struct('exec_stats',exec,'execution_status',status),cfg,{},[]);
    template = create_empty_result_record();
    aligned = align_record_fields(record,template);
    verifyEqual(testCase,fieldnames(aligned),fieldnames(template));
    verifyEqual(testCase,aligned.evidence_source,'geometric_simulation');
    verifyEqual(testCase,aligned.simulated_geometric_completion_rate,100);
    verifyTrue(testCase,isnan(aligned.actual_completion_rate));
    verifyTrue(testCase,isnan(template.planning_time_total));
    file = [tempname '.csv']; cleanup = onCleanup(@() delete(file)); %#ok<NASGU>
    verifyWarningFree(testCase,@() writetable(struct2table([aligned;template]),file));
end

function [scene,poses,path,exec,cfg] = fixture()
    scene = struct('num_tasks',2,'tasks',struct('id',{1,2}), ...
        'scene_cfg',struct('num_tables',1,'random_seed',2026));
    poses = [1,0,0]; path = [0,0;1,0]; cfg = planner_config();
    exec = estimate_execution_time(path,[1,2],[0,0,0;poses], ...
        {struct('covered_tasks',[1,2])},struct('path_length_m',{},'is_return_segment',{}),cfg);
end
