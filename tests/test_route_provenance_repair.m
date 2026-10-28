function tests=test_route_provenance_repair
%TEST_ROUTE_PROVENANCE_REPAIR Bind route safety to actual segment trajectories.
    tests=functiontests(localfunctions);
end

function testUnrelatedAggregatePathCannotBorrowValidSegmentEndpoints(testCase)
    [scene,robot]=fixture();
    poses=[1 1 0;3 1 0];segments=struct('path',poses);
    unrelated=[1 3 0;3 3 0];
    report=validate_planned_route(scene,unrelated,poses,[1 2],segments,robot);
    verifyFalse(testCase,report.valid);
end

function testSafeEndpointsCannotHideUnsafeIntermediateRotation(testCase)
    [scene,robot]=fixture();scene.grid_map(:,21)=1;
    turn=[1.76 2 0;1.76 2 pi];segments=struct('path',turn);
    verifyFalse(testCase,check_footprint_collision(turn(1,:),scene.grid_map,scene.map_info,robot));
    verifyFalse(testCase,check_footprint_collision(turn(2,:),scene.grid_map,scene.map_info,robot));
    verifyTrue(testCase,check_footprint_collision([1.76 2 pi/4],scene.grid_map,scene.map_info,robot));
    report=validate_planned_route(scene,turn,turn,[1 2],segments,robot);
    verifyFalse(testCase,report.valid);
end

function testExactSafeSegmentConcatenationPasses(testCase)
    [scene,robot]=fixture();cfg=planner_config();cfg.global_planner='astar';
    poses=[0.7 0.7 0;2.1 1.2 0.7;3.1 2.7 -0.6];
    a=run_path_planner(scene.grid_map,poses(1,:),poses(2,:),scene.map_info,cfg);
    b=run_path_planner(scene.grid_map,poses(2,:),poses(3,:),scene.map_info,cfg);
    verifyNotEmpty(testCase,a);verifyNotEmpty(testCase,b);
    segments=struct('path',{a,b});path=[a;b(2:end,:)];
    report=validate_planned_route(scene,path,poses,[1 2 3],segments,robot);
    verifyTrue(testCase,report.valid);
    verifyLessThanOrEqual(testCase,report.endpoint_max_position_error_m,1e-12);
    verifyLessThanOrEqual(testCase,report.endpoint_max_yaw_error_rad,1e-12);
end

function [scene,robot]=fixture()
    scene=struct('grid_map',zeros(40),'map_info',struct('rows',40,'cols',40, ...
        'resolution',0.1,'height_m',4,'width_m',4));
    robot=robot_config();
end
