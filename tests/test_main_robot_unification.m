function tests = test_main_robot_unification
%TEST_MAIN_ROBOT_UNIFICATION One robot across scenes, overrides and runners.
    tests = functiontests(localfunctions);
end

function testCanonicalGeometryAndLegacyAlias(testCase)
    robot = robot_config();
    verifyEqual(testCase, robot.chassis.length, 0.423);
    verifyEqual(testCase, robot.chassis.width, 0.366);
    verifyEqual(testCase, robot.dual_arm.base_spacing, 0.250);
    verifyEqual(testCase, robot.arm_base_mount_x, 0.2115, 'AbsTol', 1e-12);
    verifyEqual(testCase, robot.arm_base_mount_z, 0.1833, 'AbsTol', 1e-12);
    verifyEqual(testCase, robot.single_arm.reach_radius, 0.278);
    verifyEqual(testCase, robot_geometry_profile(), robot);
    verifyEqual(testCase, robot_geometry_profile('legacy_validated'), robot);
    verifyError(testCase, @() robot_geometry_profile('physical_robot'), ...
        'robot_geometry_profile:UnknownProfile');
end

function testEveryRetainedNamedSceneUsesMainRobot(testCase)
    names = {'','C0','C1','C2','C3','L1','L2','L3','V1','V2','V3'};
    for k = 1:numel(names)
        effective = resolve_effective_config('profile', names{k}, 'seed', 2026);
        verifyEqual(testCase, effective.robot, robot_config());
        verifyEqual(testCase, effective.planner.robot_cfg, effective.robot);
        verifyEqual(testCase, effective.scene.table.length, 1.20);
        verifyEqual(testCase, effective.scene.table.width, 0.40);
        verifyEqual(testCase, effective.scene.table.height, 0.50);
    end
    scene = rc2_four_table_scene_config();
    base = scene_config();
    verifyEqual(testCase, scene.table, base.table);
end

function testRegistryAndRemovedScenes(testCase)
    entries = experiment_registry();
    verifyEqual(testCase, {entries.id}, {'baseline','strategy_comparison'});
    for k = 1:numel(entries)
        cfg = experiment_config_builder(entries(k).id);
        verifyEqual(testCase, cfg.robot_cfg, robot_config());
        verifyEqual(testCase, cfg.planner_cfg.robot_cfg, cfg.robot_cfg);
    end
    removed = {'cafeteria_basic3','cafeteria_complex12','hardware_matched_lab'};
    for k = 1:numel(removed)
        verifyError(testCase, @() experiment_registry(removed{k}), ...
            'experiment_registry:UnknownExperiment');
    end
    verifyError(testCase, @() resolve_effective_config('profile','CAFETERIA_1X3_2X3'), ...
        'resolve_effective_config:UnknownProfile');
    verifyError(testCase, @() resolve_effective_config('robot_profile','physical_robot'), ...
        'robot_geometry_profile:UnknownProfile');
end

function testAllResolverLayersRejectConflictingGeometry(testCase)
    names = {'profile_override','experiment_override','user_override'};
    override.robot.arm_base_mount_z = 0.40;
    for k = 1:numel(names)
        verifyError(testCase, @() resolve_effective_config(names{k}, override), ...
            'validate_main_robot_config:ConflictingParameter');
    end
    nested.planner.robot_cfg.footprint.width_m = 0.50;
    verifyError(testCase, @() resolve_effective_config('user_override', nested), ...
        'validate_main_robot_config:ConflictingParameter');
end

function testBuilderRejectsLegacyAndSemanticOverrideRoutes(testCase)
    verifyError(testCase, @() experiment_config_builder('baseline', ...
        'robot_override', struct('arm_base_mount_z',0.40)), ...
        'validate_main_robot_config:ConflictingParameter');
    keys = {'robot_cfg','robot'};
    for k = 1:numel(keys)
        override = struct(); override.(keys{k}).dual_arm.base_spacing = 0.40;
        verifyError(testCase, @() experiment_config_builder('baseline','override',override), ...
            'validate_main_robot_config:ConflictingParameter');
    end
    nested.robot_cfg.arm_base_mount_x = 0.06;
    verifyError(testCase, @() experiment_config_builder('baseline','planner_override',nested), ...
        'validate_main_robot_config:ConflictingParameter');
end

function testEntryPointsRejectAlternateFrozenRobot(testCase)
    cfg = experiment_config_builder('baseline');
    cfg.robot_cfg.arm_base_mount_z = 0.40;
    verifyError(testCase, @() run_selected_experiment(cfg,'headless',true, ...
        'render_outputs',false), 'validate_main_robot_config:ConflictingParameter');
    effective = resolve_effective_config();
    effective.robot.arm_base_mount_x = 0.06;
    verifyError(testCase, @() stage1_build_scene(effective), ...
        'validate_main_robot_config:ConflictingParameter');
    robot = robot_config(); robot.dual_arm.base_spacing = 0.40;
    verifyError(testCase, @() stage3_global_planning([],[],[],[],robot, ...
        planner_config(),{},struct(),struct('status','READY')), ...
        'validate_main_robot_config:ConflictingParameter');
end

function testOldSnapshotsGetOnlyTheCanonicalDerivedFootprint(testCase)
    canonical = robot_config();
    old = rmfield(canonical, 'footprint');
    verifyEqual(testCase, validate_main_robot_config(old), canonical);
    state = struct('robot_cfg',old,'planner_cfg',struct());
    normalized = normalize_planning_state(state);
    verifyEqual(testCase, normalized.robot_cfg, canonical);
    verifyEqual(testCase, normalized.planner_cfg.robot_cfg, canonical);
    verifyFalse(testCase, isfield(state.robot_cfg,'footprint'));
    old.arm_base_mount_z = 0.40;
    verifyError(testCase, @() validate_main_robot_config(old), ...
        'validate_main_robot_config:ConflictingParameter');
    state.robot_cfg = old;
    verifyError(testCase, @() normalize_planning_state(state), ...
        'validate_main_robot_config:ConflictingParameter');
end
