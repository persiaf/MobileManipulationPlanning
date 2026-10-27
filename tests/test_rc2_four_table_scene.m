function report = test_rc2_four_table_scene()
%TEST_RC2_FOUR_TABLE_SCENE Focused structural and all-region regression test.
    root=fileparts(fileparts(mfilename('fullpath'))); addpath(genpath(root));
    cfg=rc2_four_table_scene_config(); scene=build_scene_from_cfg(cfg);
    assert(size(cfg.table_positions,1)==4); assert(numel(scene.tasks)==8);
    ids=[scene.tasks.id]; assert(numel(unique(ids))==8 && isequal(ids,1:8));
    assert(all(arrayfun(@(t)sum([scene.tasks.table_id]==t)==2,1:4)));
    [robot,~,planner,~,~,regions]=stage1_build_scene(resolve_effective_config('user_override',struct('scene',cfg,'planner',struct('runtime',struct('disable_visualization',true)))));
    library=build_candidate_library(regions,scene,robot,planner); %#ok<NASGU>
    report=struct('table_count',4,'task_count',8,'unique_task_ids',true,'tasks_per_table',2,'region_count',numel(regions));
end
