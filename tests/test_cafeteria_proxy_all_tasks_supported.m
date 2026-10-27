function tests = test_cafeteria_proxy_all_tasks_supported
    tests = functiontests(localfunctions);
end

function testEveryComplex12TaskHasProxySupport(testCase)
    [scene, robot_cfg, planner_cfg] = cafeteria_proxy_fixture();
    task_table_map = assign_tasks_to_tables(scene);
    regions = build_task_regions(scene, task_table_map);
    library = build_cafeteria_navigation_library(regions, scene, robot_cfg, planner_cfg);

    verifyEqual(testCase, library.service_support_mode, 'cafeteria_2d_proxy');
    % complex12: 9 tables, 8 of them carry tasks (build_task_regions returns one
    % region per active table); the 4 spatial layout regions live in the metadata.
    verifyEqual(testCase, numel(regions), 8);
    verifyEqual(testCase, scene.region_metadata.region_count, 4);

    support = zeros(1, scene.num_tasks);
    for ri = 1:numel(library.regions)
        for ci = 1:numel(library.regions(ri).candidates)
            ids = library.regions(ri).candidates(ci).supported_task_ids;
            for k = 1:numel(ids)
                support(ids(k)) = support(ids(k)) + 1;
            end
        end
    end
    verifyEqual(testCase, sum(support > 0), scene.num_tasks, ...
        'Every complex12 task must have at least one supporting proxy candidate.');
    verifyGreaterThan(testCase, min(support), 0);
    verifyGreaterThan(testCase, library.total_candidates, 0);
end

function testProxyCandidatesNeverClaimArmReachability(testCase)
    [scene, robot_cfg, planner_cfg] = cafeteria_proxy_fixture();
    task_table_map = assign_tasks_to_tables(scene);
    regions = build_task_regions(scene, task_table_map);
    library = build_cafeteria_navigation_library(regions, scene, robot_cfg, planner_cfg);

    n_candidates = 0;
    for ri = 1:numel(library.regions)
        for ci = 1:numel(library.regions(ri).candidates)
            c = library.regions(ri).candidates(ci);
            n_candidates = n_candidates + 1;
            verifyEqual(testCase, c.service_support_mode, 'cafeteria_2d_proxy');
            verifyEqual(testCase, c.physical_reachability, 'not_evaluated');
            verifyEqual(testCase, c.reach_scores, zeros(size(c.reach_scores)), 'AbsTol', 0);
            verifyTrue(testCase, all(strcmp(c.arm_assignment, 'not_evaluated')));
        end
    end
    verifyGreaterThan(testCase, n_candidates, 0);
end

function [scene, robot_cfg, planner_cfg] = cafeteria_proxy_fixture()
    override = cafeteria_experiment_override();
    override.planner.runtime.headless = true;
    override.planner.runtime.disable_visualization = true;
    override.planner.runtime.disable_video = true;
    profile_override.scene = cafeteria_scene_profile_config('complex12', 2026);
    profile_override.scene.cafeteria.planning_mode = 'navigation_docking_only';
    effective = resolve_effective_config('profile', 'CAFETERIA_1X3_2X3', ...
        'seed', 2026, 'robot_profile', 'physical_robot', ...
        'profile_override', profile_override, 'experiment_override', override);
    scene = build_scene_from_cfg(effective.scene);
    robot_cfg = effective.robot;
    planner_cfg = effective.planner;
end