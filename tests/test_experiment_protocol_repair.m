function tests = test_experiment_protocol_repair
    tests = functiontests(localfunctions);
end

function setupOnce(~)
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'experiment_manager'));
end

function testReturnAndScoreShareMainBasis(testCase)
    cfg=planner_config();
    verifyEqual(testCase,cfg.benchmark.include_return_to_start,cfg.execution.return_to_start);
    verifyEqual(testCase,cfg.benchmark.order_mode,'fixed_order');
    cfg=apply_experiment_protocol(cfg);
    validate_experiment_protocol(cfg);
    verifyEqual(testCase,cfg.robot_cfg,robot_config());
    cfg.reachability.threshold=0.25;
    verifyError(testCase,@()validate_experiment_protocol(cfg),'validate_experiment_protocol:ConflictingSetting');
    cfg.reachability.threshold=NaN;
    verifyError(testCase,@()validate_experiment_protocol(cfg),'validate_experiment_protocol:ConflictingSetting');
end

function testReturnDriftIsRejected(testCase)
    cfg=apply_experiment_protocol(planner_config()); cfg.execution.return_to_start=false;
    verifyError(testCase,@()validate_experiment_protocol(cfg),'validate_experiment_protocol:ConflictingSetting');
    override.planner.reachability.threshold=0.25;
    verifyError(testCase,@()resolve_effective_config('user_override',override), ...
        'validate_experiment_protocol:ConflictingSetting');
    verifyError(testCase,@()experiment_config_builder('baseline','planner_override', ...
        struct('execution',struct('return_to_start',false))), ...
        'validate_experiment_protocol:ConflictingSetting');
end

function testAssignmentAwareManipulationTiming(testCase)
    cfg=planner_config(); cfg.execution_time.manipulation_time_per_task_s=6;
    summary=struct('covered_tasks',[1 2],'task_actions', ...
        struct('task_id',{1,2},'assigned_arm',{'left','left'}));
    stats=estimate_execution_time([1 1 0;2 1 0],[1 2],zeros(2,3),{summary},struct([]),cfg);
    verifyEqual(testCase,stats.manipulation_time_s,12);
    summary.task_actions(2).assigned_arm='right';
    stats=estimate_execution_time([1 1 0;2 1 0],[1 2],zeros(2,3),{summary},struct([]),cfg);
    verifyEqual(testCase,stats.manipulation_time_s,6);
end

function testFailedRoutingRetainsTimingAndIndependentAudit(testCase)
    [scene,~,cfg]=create_test_fixture();
    scene.start_pose=[0.05 0.05 0];
    [row,~,audit]=run_protocol_planning_case(scene.scene_cfg,cfg, ...
        struct('scene',scene,'compute_identity',false,'run_audit',true));
    verifyEqual(testCase,row.status,'BLOCKED_ROUTING');
    verifyEqual(testCase,row.audit_status,'PASS');
    verifyEqual(testCase,numel(audit),scene.num_tasks);
    verifyGreaterThan(testCase,row.routing_time_s,0);
    verifyEqual(testCase,row.planning_time_s,row.scene_time_s+row.candidate_time_s+ ...
        row.selection_time_s+row.routing_time_s,'AbsTol',1e-9);
    verifyEqual(testCase,row.execution_sim_time_s,0);
    verifyFalse(testCase,row.path_safe);
end

function testCurrentPlanningCaseMeasuresAndLabelsEvidence(testCase)
    [scene,~,cfg]=create_test_fixture();
    [row,state,~]=run_protocol_planning_case(scene.scene_cfg,cfg, ...
        struct('scene',scene,'compute_identity',false,'run_audit',true));
    verifyEqual(testCase,row.status,'SUCCESS',row.failure_reason);
    verifyTrue(testCase,row.path_safe);
    verifyEqual(testCase,row.geometric_coverage_percent,100);
    verifyEqual(testCase,row.independent_mismatch_count,0);
    verifyTrue(testCase,isnan(row.actual_completion_percent));
    verifyTrue(testCase,isnan(row.ik_feasibility_percent));
    verifyEqual(testCase,row.planning_time_s,row.scene_time_s+row.candidate_time_s+ ...
        row.selection_time_s+row.routing_time_s,'AbsTol',1e-9);
    verifyTrue(testCase,state.stats.timing.excludes_execution_simulation);
    verifyTrue(testCase,state.stats.protocol.enforce);
end
