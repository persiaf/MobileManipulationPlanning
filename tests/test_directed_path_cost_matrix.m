function tests=test_directed_path_cost_matrix
    tests=functiontests(localfunctions);
end

function setupOnce(testCase)
    scene=build_scene_from_cfg(scene_config());cfg=planner_config();cfg.global_planner='astar';
    % Exact doubles from the original table offsets, with no checkpoint or
    % generated result dependency. B's y is deliberately 1.5-(.2+.4).
    A=[6,9.5,-pi/2];B=[5.85,1.5-(.2+.4),pi/2];
    testCase.TestData.scene=scene;testCase.TestData.cfg=cfg;testCase.TestData.poses=[B;A];
end

function testOptInKeepsDirectedCostsAndRejectsFailedFiniteEntries(testCase)
    costs=[0 1 Inf;7 0 9;NaN 4 0];success=true(3);success(2,3)=false;
    [scene,cfg,poses,context]=cached_oracle(costs,success);
    cfg.routing.directed_costs=true;
    D=compute_path_cost_matrix(poses,scene.grid_map,scene.map_info,cfg);
    lambda=cfg.edge_cost.lambda_theta;
    verifyEqual(testCase,D(1,2),1+lambda*.4,'AbsTol',1e-12);
    verifyEqual(testCase,D(2,1),7+lambda*.4,'AbsTol',1e-12);
    verifyTrue(testCase,isinf(D(2,3))); % ok=false dominates finite cached d.
    verifyTrue(testCase,isinf(D(1,3)));verifyTrue(testCase,isinf(D(3,1)));
    verifyEqual(testCase,D(3,2),4+lambda*.6,'AbsTol',1e-12);
    verifyEqual(testCase,context.query_count,6);
    verifyEqual(testCase,diag(D),zeros(3,1));
end

function testAbsentAndFalseFlagsRetainTheLegacyMirroredMatrix(testCase)
    costs=[0 1 5;7 0 9;2 4 0];
    [scene,cfg,poses,context]=cached_oracle(costs,true(3));
    D=compute_path_cost_matrix(poses,scene.grid_map,scene.map_info,cfg);
    verifyEqual(testCase,D,D');verifyEqual(testCase,context.query_count,3);
    cfg.routing.directed_costs=false;
    explicit=compute_path_cost_matrix(poses,scene.grid_map,scene.map_info,cfg);
    verifyEqual(testCase,explicit,D);
    verifyEqual(testCase,D(2,1),1+cfg.edge_cost.lambda_theta*.4,'AbsTol',1e-12);
end

function testActualMainSceneDoesNotInventTheFailedReverseEdge(testCase)
    scene=testCase.TestData.scene;cfg=testCase.TestData.cfg;P=testCase.TestData.poses;
    legacy=compute_path_cost_matrix(P,scene.grid_map,scene.map_info,cfg);
    cfg.routing.directed_costs=true;
    directed=compute_path_cost_matrix(P,scene.grid_map,scene.map_info,cfg);
    verifyTrue(testCase,isfinite(directed(1,2)));
    verifyTrue(testCase,isinf(directed(2,1)));
    verifyEqual(testCase,directed(1,2),legacy(1,2),'AbsTol',1e-10);
    verifyEqual(testCase,legacy(2,1),legacy(1,2));
end

function testBothEndpointsAndReversedWitnessRemainCollisionValid(testCase)
    scene=testCase.TestData.scene;cfg=testCase.TestData.cfg;P=testCase.TestData.poses;
    for k=1:2
        verifyTrue(testCase,validate_navigation_trajectory(P(k,:),scene.grid_map,scene.map_info,cfg));
    end
    [path,info]=run_path_planner(scene.grid_map,P(1,:),P(2,:),scene.map_info,cfg, ...
        'astar',struct('suppress_warnings',true));
    verifyTrue(testCase,info.path_valid);verifyNotEmpty(testCase,path);
    reversed=flipud(path);
    verifyTrue(testCase,validate_navigation_trajectory(reversed,scene.grid_map,scene.map_info,cfg));
    verifyEqual(testCase,reversed(1,:),P(2,:),'AbsTol',1e-12);
    verifyEqual(testCase,reversed(end,:),P(1,:),'AbsTol',1e-12);
end

function [scene,cfg,poses,context]=cached_oracle(costs,success)
    cfg=planner_config();cfg.global_planner='astar';poses=[.5 .5 0;.8 .5 .4;1.1 .5 -.2];
    scene=struct('grid_map',zeros(30),'map_info',struct('rows',30,'cols',30, ...
        'resolution',.1,'width_m',3,'height_m',3));
    context=SharedPathCostContext(scene,cfg);q=1000;
    cost_map=context.costs;path_map=context.paths;success_map=context.successes;
    for i=1:3
        for j=1:3
            if i==j,continue;end
            a=round(poses(i,:)*q);b=round(poses(j,:)*q);
            key=sprintf('%s|astar|%d_%d_%d>%d_%d_%d',context.scene_signature, ...
                a(1),a(2),a(3),b(1),b(2),b(3));
            cost_map(key)=costs(i,j);path_map(key)=[poses(i,:);poses(j,:)];success_map(key)=success(i,j);
        end
    end
    cfg.runtime.shared_path_cost_context=context;
end
