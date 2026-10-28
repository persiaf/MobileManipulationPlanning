function tests = test_independent_reachability_audit
    tests = functiontests(localfunctions);
end

function testLibraryAgreementAndNoPhysicalClaim(testCase)
    [scene,robot,planner,regions] = create_test_fixture();
    library = build_candidate_library(regions,scene,robot,planner);
    [summary,rows] = audit_independent_reachability(scene,robot,planner,library,coarse_options());
    verifyEqual(testCase,numel(rows),scene.num_tasks);
    verifyEqual(testCase,summary.mismatch_count,0);
    verifyEqual(testCase,summary.candidate_pair_mismatch_count,0);
    verifyEqual(testCase,summary.candidate_coverage_percent,100);
    verifyEqual(testCase,summary.audited_geometric_coverage_percent,100);
    verifyEqual(testCase,summary.dense_sample_count,0);
    verifyTrue(testCase,isnan(summary.ik_feasibility_percent));
    verifyTrue(testCase,isnan(summary.actual_completion_percent));
    verifyFalse(testCase,summary.ik_verified);
end

function testThinnedLibraryExposesSamplingGap(testCase)
    [scene,robot,planner,regions] = create_test_fixture();
    library = build_candidate_library(regions,scene,robot,planner);
    candidates = library.regions(1).candidates;
    library.regions(1).candidates = candidates(~arrayfun(@(c) ismember(1,c.covered_tasks),candidates));
    [summary,rows] = audit_independent_reachability(scene,robot,planner,library,coarse_options());
    first = rows([rows.task_id]==1);
    verifyFalse(testCase,first.production_library_covered);
    verifyFalse(testCase,first.independent_library_covered);
    verifyTrue(testCase,first.independent_dense_support_found);
    verifyEqual(testCase,first.classification,'candidate_sampling_gap');
    verifyGreaterThanOrEqual(testCase,summary.sampling_gap_count,1);
    verifyTrue(testCase,first.audited_geometric_supported);
    verifyFalse(testCase,check_footprint_collision([first.support_x_m,first.support_y_m,first.support_yaw_rad], ...
        scene.grid_map,scene.map_info,robot));
end

function testVerticalCertificateNeedsNoFiniteSearch(testCase)
    [scene,robot,planner] = single_task();
    scene.tasks.z = robot.arm_base_mount_z+robot.single_arm.reach_z_max+0.01;
    [summary,rows] = audit_independent_reachability(scene,robot,planner,empty_library(),coarse_options());
    verifyEqual(testCase,rows.classification,'hard_vertical_certificate');
    verifyEqual(testCase,summary.hard_vertical_certificate_count,1);
    verifyEqual(testCase,summary.unresolved_count,0);
    verifyEqual(testCase,summary.dense_sample_count,0);
    verifyEqual(testCase,summary.audited_geometric_coverage_percent,0);
end

function testFiniteSearchFailureIsUnresolved(testCase)
    [scene,robot,planner] = single_task();
    scene.tasks.x = 2; scene.tasks.y = 2;
    options = coarse_options(); options.normal_offsets_m = scene.scene_cfg.docking_max_distance_m;
    [summary,rows] = audit_independent_reachability(scene,robot,planner,empty_library(),options);
    verifyEqual(testCase,rows.classification,'not_found_in_audited_samples');
    verifyEqual(testCase,summary.unresolved_count,1);
    verifyEqual(testCase,summary.hard_vertical_certificate_count,0);
    verifyFalse(testCase,rows.audited_geometric_supported);
end

function testForgedCoverageCreatesMismatch(testCase)
    [scene,robot,planner] = single_task();
    entry = struct('table_id',1,'task_ids',1,'candidates',struct('pose',[0.5,0.5,0],'covered_tasks',1));
    library = struct('regions',entry);
    [summary,rows] = audit_independent_reachability(scene,robot,planner,library,coarse_options());
    verifyEqual(testCase,summary.mismatch_count,1);
    verifyEqual(testCase,rows.classification,'library_coverage_mismatch');
    verifyFalse(testCase,rows.independent_library_covered);
end

function testDeterministicAllTaskAccountingAndExplicitCap(testCase)
    [scene,robot,planner] = single_task();
    options = coarse_options();
    [sa,a] = audit_independent_reachability(scene,robot,planner,empty_library(),options);
    [sb,b] = audit_independent_reachability(scene,robot,planner,empty_library(),options);
    verifyEqual(testCase,rmfield(a,'audit_time_s'),rmfield(b,'audit_time_s'));
    verifyEqual(testCase,rmfield(sa,'audit_time_s'),rmfield(sb,'audit_time_s'));
    options.max_samples_per_task = 1;
    verifyError(testCase,@() audit_independent_reachability(scene,robot,planner,empty_library(),options), ...
        'audit_independent_reachability:SampleCap');
    verifyError(testCase,@() audit_independent_reachability(scene,robot,planner,empty_library(),struct('max_tasks',1)), ...
        'audit_independent_reachability:UnknownOption');
end

function [scene,robot,planner] = single_task()
    [scene,robot,planner] = create_test_fixture();
    scene.tasks = scene.tasks(1); scene.num_tasks = 1;
end

function library = empty_library()
    library = struct('regions',struct('table_id',{},'task_ids',{},'candidates',{}));
end

function options = coarse_options()
    options = struct('tangential_step_m',0.05,'normal_step_m',0.05, ...
        'heading_step_deg',10,'heading_span_deg',90);
end
