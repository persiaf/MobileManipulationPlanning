function tests=test_c2_5_1_safety_integration
    tests=functiontests(localfunctions);
end
function setupOnce(t)
    t.TestData.state=create_c2_5_1_fixture();
end
function testKeepSafe(t)
    s=t.TestData.state; [c,e]=apply_dynamic_event(s,'ADD_LOCAL_OBSTACLE',1);
    c.scene=s.scene; e.center=[3.8 3.8];
    [i,j]=world2grid(3.8,3.8,s.scene.map_info.resolution,s.scene.map_info.height_m);
    c.scene.grid_map(i,j)=1;
    [r,n,d]=execute_replanning_action(s,c,e,[], 'KEEP');
    verifyEqual(t,r.status,'NOOP',r.fallback_reason);
    verifyTrue(t,r.path_safe&&d.committed&&n.execution_allowed);
end
function testKeepUnsafeUnchangedMap(t)
    s=t.TestData.state; s.scene.grid_map(:)=1;
    e=struct('type','ADD_LOCAL_OBSTACLE');
    [mask,~]=evaluate_replanning_action_mask(s,s,e,struct());
    verifyFalse(t,mask(1));
end
function testKeepMissingFootprint(t)
    s=t.TestData.state; s.robot_cfg=rmfield(s.robot_cfg,'footprint');
    [mask,~]=evaluate_replanning_action_mask(s,s,struct('type','ADD_LOCAL_OBSTACLE'),struct());
    verifyFalse(t,any(mask));
end
function testIncompleteEventCannotKeepSafePlan(t)
    s=t.TestData.state;
    [mask,~]=evaluate_replanning_action_mask(s,s,struct('type','ADD_LOCAL_OBSTACLE'),[]);
    verifyFalse(t,any(mask));
end
function testKeepDuplicateCoverage(t)
    s=t.TestData.state; s.plan.task_sequence_summary{2}.covered_tasks=[1 2];
    [mask,~]=evaluate_replanning_action_mask(s,s,struct('type','ADD_LOCAL_OBSTACLE'),struct());
    verifyFalse(t,mask(1));
end
function testPathSuccess(t)
    s=t.TestData.state; [c,e]=apply_dynamic_event(s,'ADD_LOCAL_OBSTACLE',1);
    [r,n,d]=execute_replanning_action(s,c,e,[], 'REPLAN_PATH');
    verifyEqual(t,r.status,'SUCCESS',r.fallback_reason);
    verifyTrue(t,d.committed&&r.path_safe&&n.execution_allowed);
    verifyNotEqual(t,n.stats.global_path,s.stats.global_path);
    verifyEqual(t,n.rc2_plan,s.rc2_plan);
end
function testDockSuccess(t)
    s=t.TestData.state; [c,e]=dock_event(s);
    [r,n,d]=execute_replanning_action(s,c,e,[], 'SWITCH_DOCK');
    verifyEqual(t,r.status,'SUCCESS',r.fallback_reason);
    verifyTrue(t,d.committed&&r.path_safe);
    verifyNotEqual(t,n.plan.selected_poses,s.plan.selected_poses);
    verifyEqual(t,n.rc2_plan,s.rc2_plan);
end
function testTaskSuccess(t)
    s=t.TestData.state; [c,e]=apply_dynamic_event(s,'TASK_COVERAGE_FAILURE',1);
    [r,n,d]=execute_replanning_action(s,c,e,[], 'REALLOCATE_TASK');
    verifyEqual(t,r.status,'SUCCESS',r.fallback_reason);
    verifyTrue(t,d.committed&&r.task_coverage_valid&&r.path_safe);
    verifyFalse(t,ismember(e.target_task_id,n.plan.task_sequence_summary{e.target_dock_index}.covered_tasks));
    verifyEqual(t,n.task_state,s.task_state); verifyEqual(t,n.rc2_plan,s.rc2_plan);
end
function testFailurePreservesEnvironment(t)
    s=t.TestData.state; s.candidate_library.regions(1).candidates=[];
    [c,e]=dock_event(s);
    [r,n,d]=execute_replanning_action(s,c,e,[], 'SWITCH_DOCK');
    verifyEqual(t,r.status,'FALLBACK_REQUIRED');
    verifyEqual(t,r.fallback_reason,'NO_VALID_BACKUP_DOCK');
    verifyFalse(t,d.committed||n.execution_allowed);
    verifyEqual(t,n.scene,c.scene); verifyEqual(t,n.plan,s.plan);
end
function testRejectUnsafeCandidateAtomically(t)
    s=t.TestData.state;
    % Unaffected task corruption survives low-level repair; the high-level
    % validator must reject the otherwise successful new path transaction.
    s.plan.pose_to_region_map(2)=999;
    [c,e]=apply_dynamic_event(s,'ADD_LOCAL_OBSTACLE',1);
    [r,n,d]=execute_replanning_action(s,c,e,[], 'REPLAN_PATH');
    verifyEqual(t,d.low_level_result.status,'SUCCESS');
    verifyEqual(t,r.status,'FALLBACK_REQUIRED');
    verifyFalse(t,d.committed||n.execution_allowed);
    verifyEqual(t,n.stats.global_path,s.stats.global_path); verifyEqual(t,n.scene,c.scene);
end
function testUnsupportedAndUnknown(t)
    s=t.TestData.state;
    [r,n]=execute_replanning_action(s,s,struct('type','ADD_TASKS'),[], 'KEEP');
    verifyEqual(t,r.status,'UNSUPPORTED_EVENT'); verifyFalse(t,n.execution_allowed);
    r=execute_replanning_action(s,s,struct('type','UNKNOWN'),[], 'KEEP');
    verifyEqual(t,r.status,'NO_LEGAL_ACTION');
end
function testIllegalAction(t)
    s=t.TestData.state; [c,e]=apply_dynamic_event(s,'ADD_LOCAL_OBSTACLE',1);
    [r,n]=execute_replanning_action(s,c,e,[], 'REALLOCATE_TASK');
    verifyEqual(t,r.status,'ACTION_REJECTED'); verifyEqual(t,n.plan,s.plan);
    verifyFalse(t,n.execution_allowed);
end
function testCompletedAndUnknownTaskIDs(t)
    s=t.TestData.state; [c,e]=apply_dynamic_event(s,'TASK_COVERAGE_FAILURE',1);
    s.task_state.verified_tasks=e.target_task_id; c.task_state=s.task_state;
    [mask,~]=evaluate_replanning_action_mask(s,c,e,[]); verifyFalse(t,mask(4));
    e.target_task_id=101;
    [mask,~]=evaluate_replanning_action_mask(s,c,e,[]); verifyFalse(t,mask(4));
end
function testContinuousDifferentEvents(t)
    s=t.TestData.state; [c,e]=apply_dynamic_event(s,'TASK_COVERAGE_FAILURE',1);
    [r,n]=execute_replanning_action(s,c,e,[], 'REALLOCATE_TASK');
    assertEqual(t,r.status,'SUCCESS',r.fallback_reason);
    [c2,e2]=apply_dynamic_event(n,'ADD_LOCAL_OBSTACLE',1);
    [r2,n2,d]=execute_replanning_action(n,c2,e2,[], 'REPLAN_PATH');
    verifyEqual(t,r2.status,'SUCCESS',r2.fallback_reason);
    verifyTrue(t,d.committed); verifyEqual(t,n2.rc2_plan,s.rc2_plan);
    verifyTrue(t,ismember(e.invalidated_coverage_pairs,n2.invalidated_coverage_pairs,'rows'));
end
function [c,e]=dock_event(s)
    [c,e]=apply_dynamic_event(s,'INVALIDATE_DOCK',1);
    % Existing C2-2 deterministic fixture: one occupied cell at the old dock.
    c.scene=s.scene; [i,j]=world2grid(e.center(1),e.center(2),s.scene.map_info.resolution,s.scene.map_info.height_m);
    c.scene.grid_map(i,j)=1;
end

function testUnifiedDstarTwoEvents(t)
    s=create_c2_5_1_dstar_fixture(); s=normalize_planning_state(s);
    versions=[]; keys={};
    for k=1:2
        [c,e]=apply_dynamic_event(s,'ADD_LOCAL_OBSTACLE',5);
        q=s.stats.sparse_graph.direct_paths{1,2};
        e.center=q(floor(size(q,1)/2),1:2);
        if k==1,e.center=e.center+[0.25 0];e.radius_m=0.15;else,e.radius_m=0.25;end
        c.scene=s.scene;
        for i=1:c.scene.map_info.rows
            for j=1:c.scene.map_info.cols
                [x,y]=grid2world(i,j,c.scene.map_info.resolution,c.scene.map_info.height_m);
                if hypot(x-e.center(1),y-e.center(2))<=e.radius_m,c.scene.grid_map(i,j)=1;end
            end
        end
        [r,n,d]=execute_replanning_action(s,c,e,[],'REPLAN_PATH');
        assertEqual(t,r.status,'SUCCESS',r.fallback_reason);
        edge=d.low_level.incremental_dstar.edge_results(1);
        verifyFalse(t,edge.fallback_used);verifyTrue(t,r.path_safe);
        versions(end+1)=edge.state_version;keys{end+1}=edge.key; %#ok<AGROW>
        if k==2
            verifyTrue(t,edge.state_loaded&&edge.incremental_reuse);
            verifyFalse(t,edge.reinitialized);verifyEqual(t,keys{1},keys{2});
            verifyGreaterThan(t,versions(2),versions(1));
        end
        s=n;
    end
end

function testNonContiguousTaskExecution(t)
    s=create_c2_5_1_fixture(); newids=[101 105];
    for k=1:numel(s.scene.tasks),s.scene.tasks(k).id=newids(k);end
    s.tasks=s.scene.tasks;s.task_state.predicted_tasks=newids;
    for d=1:numel(s.plan.task_sequence_summary)
        for name={'task_sequence_summary','pose_info_list'}
            a=s.plan.(name{1}){d};old=a.covered_tasks;a.covered_tasks=newids(old);
            m=containers.Map('KeyType','double','ValueType','char');
            for tid=old,m(newids(tid))=a.arm_assign_map(tid);end
            a.arm_assign_map=m;
            if isfield(a,'task_actions')
                for k=1:numel(a.task_actions),a.task_actions(k).task_id=newids(a.task_actions(k).task_id);end
            end
            s.plan.(name{1}){d}=a;
        end
    end
    s.plan.covered_tasks_predicted=newids;s=complete_replanning_route(s);
    [c,e]=apply_dynamic_event(s,'TASK_COVERAGE_FAILURE',1);
    [r,n]=execute_replanning_action(s,c,e,[],'REALLOCATE_TASK');
    verifyEqual(t,r.status,'SUCCESS',r.fallback_reason);
    verifyEqual(t,[n.scene.tasks.id],newids);
    verifyTrue(t,ismember([101 1],n.invalidated_coverage_pairs,'rows'));
end
