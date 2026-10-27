function [state,data]=create_c2_5_1_fixture()
%CREATE_C2_5_1_FIXTURE Existing two-task model, without simulated execution.
    [scene,robot,planner,regions]=create_test_fixture();
    t=tic; library=build_candidate_library(regions,scene,robot,planner); candidate_time=toc(t);
    t=tic; plan=run_planning_strategy(library,regions,scene,robot,planner); selection_time=toc(t);
    plan.candidate_library=library;
    [~,~,~,~,~,~,stats]=stage3_global_planning(scene,plan.selected_poses, ...
        plan.task_sequence_summary,{},robot,planner,plan.pose_info_list,struct(),plan);
    state=normalize_planning_state(stats.planning_state);
    % Demonstration starts before execution. RC2 freeze remains unchanged.
    state.scene=scene; state.tasks=scene.tasks;
    state.task_state=struct('predicted_tasks',plan.covered_tasks_predicted, ...
        'commanded_tasks',[],'executed_tasks',[],'verified_tasks',[]);
    state=complete_replanning_route(state);
    data=struct('scene',scene,'robot',robot,'planner',planner,'regions',regions, ...
        'library',library,'plan',plan,'stats',stats, ...
        'candidate_time_s',candidate_time,'selection_time_s',selection_time);
end
