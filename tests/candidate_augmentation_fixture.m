function [baseline, damaged, regions, scene, robot_cfg, planner_cfg] = ...
        candidate_augmentation_fixture()
%CANDIDATE_AUGMENTATION_FIXTURE Deterministic small library with one gap.

    persistent cache
    if isempty(cache)
        scene = build_scene_from_cfg(large_scale_profile_config('s4', 1));
        robot_cfg = robot_config(); planner_cfg = planner_config();
        regions = build_task_regions(scene, assign_tasks_to_tables(scene));
        regions = regions(1);
        baseline = build_candidate_library(regions, scene, robot_cfg, planner_cfg);
        damaged = baseline;
        planner_cfg.candidate_augmentation.enable = true;
        planner_cfg.candidate_augmentation.variant = 'joint';
        cache = struct('baseline',baseline,'damaged',damaged,'regions',regions, ...
            'scene',scene,'robot_cfg',robot_cfg,'planner_cfg',planner_cfg);
    end
    baseline = cache.baseline; damaged = cache.damaged; regions = cache.regions;
    scene = cache.scene; robot_cfg = cache.robot_cfg; planner_cfg = cache.planner_cfg;
end
