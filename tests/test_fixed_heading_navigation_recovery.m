function tests=test_fixed_heading_navigation_recovery
    tests=functiontests(localfunctions);
end

function testFixedHeadingLayersMatchRectangleCollision(testCase)
    robot=robot_config();map=zeros(50);
    map(18:32,23:27)=2;map(9,12)=1;
    info=struct('rows',50,'cols',50,'resolution',0.05, ...
        'height_m',2.5,'width_m',2.5,'origin_xy',[4 -2]);
    for yaw=[0,pi/2,pi/6,-pi/3]
        layer=build_footprint_inflated_map(map,robot.footprint,info,yaw);
        for i=1:3:50
            for j=1:3:50
                [x,y]=grid2world(i,j,info.resolution,info.height_m,info.origin_xy);
                hit=check_footprint_collision([x y yaw],map,info,robot);
                verifyEqual(testCase,layer(i,j)>0,hit);
            end
        end
    end
end

function testRectangleCrossesPassageBlockedByCircumscribedDisk(testCase)
    [map,info,cfg,a,b]=corridor_fixture(26:35);
    cfg.navigation.fixed_heading_fallback=false;
    [path,p]=run_path_planner(map,a,b,info,cfg,'astar',struct('suppress_warnings',true));
    verifyEmpty(testCase,path);verifyFalse(testCase,p.path_valid);
    cfg.navigation.fixed_heading_fallback=true;
    [path,p,~,d]=run_path_planner(map,a,b,info,cfg,'astar',struct('suppress_warnings',true));
    verifyNotEmpty(testCase,path);verifyTrue(testCase,p.path_valid);
    verifyTrue(testCase,p.fallback_used);
    verifyEqual(testCase,p.actual_planner,'fixed_heading_astar');
    verifyEqual(testCase,d.actual_planner,'fixed_heading_astar');
    verifyEqual(testCase,path(1,:),a,'AbsTol',1e-12);
    verifyEqual(testCase,path(end,:),b,'AbsTol',1e-12);
    verifyTrue(testCase,validate_navigation_trajectory(path,map,info,cfg));
    verifyEqual(testCase,path(:,3),zeros(size(path,1),1),'AbsTol',1e-12);
end

function testRecoveryDoesNotPassPhysicallyBlockedGap(testCase)
    [map,info,cfg,a,b]=corridor_fixture(28:33);
    [path,p]=run_path_planner(map,a,b,info,cfg,'astar',struct('suppress_warnings',true));
    verifyEmpty(testCase,path);verifyFalse(testCase,p.path_valid);
    verifyNotEmpty(testCase,p.failure_reason);
end

function testUpperMainSceneDockKeepsExactPoseAndSafeFootprint(testCase)
    cfg=scene_config();scene=build_scene_from_cfg(cfg);planner=planner_config();
    a=scene.start_pose;b=[3.85 9.5 -pi/2];
    [path,p]=run_path_planner(scene.grid_map,a,b,scene.map_info,planner, ...
        'convex_astar',struct('suppress_warnings',true));
    verifyNotEmpty(testCase,path);verifyTrue(testCase,p.path_valid);
    verifyEqual(testCase,p.actual_planner,'fixed_heading_astar');
    verifyEqual(testCase,path(1,:),a,'AbsTol',1e-12);
    verifyEqual(testCase,path(end,:),b,'AbsTol',1e-12);
    verifyTrue(testCase,validate_navigation_trajectory(path,scene.grid_map,scene.map_info,planner));
end

function testRecoverySettingInvalidatesPathCache(testCase)
    [map,info,cfg]=corridor_fixture(26:35);
    enabled=build_path_cache_signature(map,info,cfg);
    cfg.navigation.fixed_heading_fallback=false;
    verifyNotEqual(testCase,build_path_cache_signature(map,info,cfg),enabled);
end

function [map,info,cfg,a,b]=corridor_fixture(gap_rows)
    map=zeros(60);map(:,27:34)=1;map(gap_rows,27:34)=0;
    info=struct('rows',60,'cols',60,'resolution',0.05,'height_m',3,'width_m',3);
    cfg=planner_config();cfg.global_planner='astar';
    a=[0.6 1.475 0];b=[2.4 1.475 0];
end
