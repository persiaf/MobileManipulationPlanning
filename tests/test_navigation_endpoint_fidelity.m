function tests=test_navigation_endpoint_fidelity
%TEST_NAVIGATION_ENDPOINT_FIDELITY Exact-pose paths validated on the raw map.
    tests=functiontests(localfunctions);
end

function testSuccessfulPathRetainsExactFullRequestedPoses(testCase)
    [map,info,cfg]=fixture();a=[0.701 0.754 0.70];b=[3.13 2.77 -1.10];
    [path,provenance]=run_path_planner(map,a,b,info,cfg);
    verifyNotEmpty(testCase,path);
    verifyEqual(testCase,path(1,:),a,'AbsTol',1e-12);
    verifyEqual(testCase,path(end,:),b,'AbsTol',1e-12);
    verifyTrue(testCase,provenance.path_valid);
    verifyTrue(testCase,provenance.endpoint_validation.requested_endpoints_matched);
    verifyLessThanOrEqual(testCase,max(vecnorm(diff(path(:,1:2)),2,2)),info.resolution/3+1e-12);
    verifyLessThanOrEqual(testCase,max(abs(arrayfun(@wrapToPiLocal,diff(path(:,3))))),pi/36+1e-12);
    verifyTrue(testCase,validate_navigation_trajectory(path,map,info,cfg));
end

function testNominalCollisionCannotBeHiddenByGridSnapping(testCase)
    [map,info,cfg]=fixture();a=[0.7 0.7 0];b=[2.03 2.03 0];
    [i,j]=world2grid(b(1),b(2),info.resolution,info.height_m);map(i,j)=1;
    [path,p]=run_path_planner(map,a,b,info,cfg);
    verifyEmpty(testCase,path);verifyFalse(testCase,p.path_valid);
    verifyFalse(testCase,p.endpoint_validation.requested_poses_valid);
    verifyEqual(testCase,p.failure_reason,'GOAL_POSE_COLLISION');
end

function testCloseWallDockUsesSafeRotationAnchor(testCase)
    [map,info,cfg]=fixture();map(:,21)=1;
    a=[0.7 2 0];b=[1.76 2 pi/2];
    verifyFalse(testCase,check_footprint_collision(b,map,info,robot_config()));
    [path,p]=run_path_planner(map,a,b,info,cfg);
    verifyNotEmpty(testCase,path);
    verifyEqual(testCase,path(end,:),b,'AbsTol',1e-12);
    verifyTrue(testCase,p.path_valid);
    verifyTrue(testCase,validate_navigation_trajectory(path,map,info,cfg));
end

function testLongEndpointSnapIsExplicitFailure(testCase)
    [map,info,cfg]=fixture();a=[0.7 0.7 0];b=[3 3 0];
    [path,p]=complete_navigation_endpoints([1.5 1.5 0;2 2 0],a,b,map,info,cfg);
    verifyEmpty(testCase,path);verifyFalse(testCase,p.valid);
    verifyEqual(testCase,p.reason,'START_GRID_SNAP_TOO_FAR');
end

function testAxisOrderConnectorCanAvoidBlockedDiagonalCorner(testCase)
    [map,info,cfg]=fixture();
    cfg.robot_cfg.footprint.length_m=0.02;cfg.robot_cfg.footprint.width_m=0.02;
    [i,j]=world2grid(1.15,1.05,info.resolution,info.height_m);map(i,j)=1;
    a=[1.05 1.05 0];b=[1.55 1.15 0];
    [path,p]=complete_navigation_endpoints([1.15 1.15 0;1.55 1.15 0],a,b,map,info,cfg);
    verifyNotEmpty(testCase,path);verifyTrue(testCase,p.valid);
    verifyEqual(testCase,p.start_connector.variant,'y_then_x');
    verifyTrue(testCase,validate_navigation_trajectory(path,map,info,cfg));
end

function testPointOnlyModePreservesWorldOriginAndExactEndpoints(testCase)
    [map,info,cfg]=fixture();info.origin_xy=[5 -3];
    cfg.footprint_aware_navigation=false;cfg=rmfield(cfg,'robot_cfg');
    a=[5.31 -2.45 pi];b=[8.42 0.27 -2];
    [path,p]=run_path_planner(map,a,b,info,cfg);
    verifyNotEmpty(testCase,path);
    verifyEqual(testCase,path(1,:),a,'AbsTol',1e-12);
    verifyEqual(testCase,path(end,:),b,'AbsTol',1e-12);
    verifyFalse(testCase,p.endpoint_validation.footprint_checked);
    verifyTrue(testCase,validate_navigation_trajectory(path,map,info,cfg));
end

function testSameCellAndPureRotationStillHaveExactYaw(testCase)
    [map,info,cfg]=fixture();a=[1.07 1.03 -pi+0.1];b=[a(1:2) pi-0.1];
    [path,p]=run_path_planner(map,a,b,info,cfg);
    verifyGreaterThanOrEqual(testCase,size(path,1),2);
    verifyEqual(testCase,path(1,:),a,'AbsTol',1e-12);
    verifyEqual(testCase,path(end,:),b,'AbsTol',1e-12);
    verifyTrue(testCase,p.path_valid);
end

function testDstarStateRemainsReusableOnlyForItsValidatedRoute(testCase)
    [map,info,cfg]=fixture();cfg.footprint_aware_navigation=false;
    a=[0.55 1.45 0];b=[3.15 0.55 0];
    opts=struct('incremental_mode',true,'dstar_state',[],'changed_cells',[],'suppress_warnings',true);
    [path,p,s1,d1]=run_path_planner(map,a,b,info,cfg,'dstar_lite',opts);
    verifyNotEmpty(testCase,path);verifyTrue(testCase,d1.search_state_valid);
    verifyFalse(testCase,p.fallback_used);
    opts.dstar_state=s1;a=[0.65 1.45 0.2];
    [path,~,s2,d2]=run_path_planner(map,a,b,info,cfg,'dstar_lite',opts);
    verifyTrue(testCase,d2.incremental_reuse);verifyTrue(testCase,d2.search_state_valid);
    verifyTrue(testCase,isstruct(s2) && isfield(s2,'g'));
    verifyEqual(testCase,path(1,:),a,'AbsTol',1e-12);
    [i,j]=world2grid(b(1),b(2),info.resolution,info.height_m);map(i,j)=1;
    opts.dstar_state=s2;
    [path,~,s3,d3]=run_path_planner(map,a,b,info,cfg,'dstar_lite',opts);
    verifyEmpty(testCase,path);verifyEmpty(testCase,s3);verifyFalse(testCase,d3.search_state_valid);
end

function testUnknownPlannerReportsActualFallback(testCase)
    [map,info,cfg]=fixture();
    [path,p,~,d]=run_path_planner(map,[0.7 0.7 0],[3 3 0],info,cfg, ...
        'not_a_planner',struct('suppress_warnings',true));
    verifyNotEmpty(testCase,path);verifyTrue(testCase,p.fallback_used);
    verifyEqual(testCase,p.fallback_count,1);verifyEqual(testCase,p.actual_planner,'astar');
    verifyEqual(testCase,d.actual_planner,'astar');
end

function testFreeInflatedGridCentresAreSafeForEverySampledYaw(testCase)
    [map,info,cfg]=fixture();map(20,20)=1;
    [inflated,meta]=build_footprint_inflated_map(map,cfg.robot_cfg.footprint,info);
    verifyEqual(testCase,meta.radius_m,hypot(0.423,0.366)/2,'AbsTol',1e-12);
    verifyGreaterThan(testCase,meta.search_radius_m,meta.radius_m);
    for i=15:25
        for j=15:25
            if inflated(i,j),continue;end
            [x,y]=grid2world(i,j,info.resolution,info.height_m);
            for yaw=0:pi/12:2*pi
                verifyFalse(testCase,check_footprint_collision([x y yaw],map,info,cfg.robot_cfg));
            end
        end
    end
    empty=zeros(size(map));inflated=build_footprint_inflated_map(empty,cfg.robot_cfg.footprint,info);
    verifyTrue(testCase,all(inflated(1,:)>0));
    verifyTrue(testCase,all(inflated(:,1)>0));
end

function [map,info,cfg]=fixture()
    map=zeros(40);info=struct('rows',40,'cols',40,'resolution',0.1,'height_m',4,'width_m',4);
    cfg=planner_config();cfg.global_planner='astar';cfg.benchmark.enable=false;
end
