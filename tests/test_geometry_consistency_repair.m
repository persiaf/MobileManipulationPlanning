function tests = test_geometry_consistency_repair
%TEST_GEOMETRY_CONSISTENCY_REPAIR Independent world-polygon regression checks.
    tests = functiontests(localfunctions);
end

function testRasterAngleUsesWorldPositiveY(testCase)
    mask = rectMask(101,101,51,51,8,24,pi/4);
    verifyTrue(testCase, mask(45,57));  % world (+x,+y), along the table
    verifyFalse(testCase, mask(57,57)); % mirrored world (+x,-y)
    [j,i] = meshgrid(1:101,1:101);
    for theta = [0,pi/6,pi/4,pi/2,-pi/6]
        corners = rectangle_vertices([0 0 theta],24,8);
        expected = inpolygon(j-51,51-i,corners(:,1),corners(:,2));
        % Classify closed polygon edges independently of floating rounding
        % in inpolygon's special handling of axis-aligned rotated edges.
        edge_distance=inf(size(j));
        for k=1:4
            a=corners(k,:); b=corners(mod(k,4)+1,:); v=b-a;
            t=max(0,min(1,((j-51-a(1))*v(1)+(51-i-a(2))*v(2))/dot(v,v)));
            edge_distance=min(edge_distance,hypot(j-51-a(1)-t*v(1),51-i-a(2)-t*v(2)));
        end
        expected=expected | edge_distance<=1e-10;
        verifyEqual(testCase,rectMask(101,101,51,51,8,24,theta),expected, ...
            sprintf('world yaw %.6f',theta));
    end
end

function testMainRobotLengthAndWidthAxes(testCase)
    [map,info] = empty_map(2,0.005,[0 0]); robot = robot_config();
    pose = [1.0025 1.0025 0];
    map = place_cell(map,info,pose(1)+0.20,pose(2));
    verifyTrue(testCase,check_footprint_collision(pose,map,info,robot));
    verifyFalse(testCase,check_footprint_collision([pose(1:2),pi/2],map,info,robot));
    map(:)=0; map=place_cell(map,info,pose(1),pose(2)+0.20);
    verifyFalse(testCase,check_footprint_collision(pose,map,info,robot));
    verifyTrue(testCase,check_footprint_collision([pose(1:2),pi/2],map,info,robot));
end

function testRotatedCellIntersectionsMatchIndependentPolygonEdges(testCase)
    [map,info] = empty_map(3,0.05,[-1.3 2.2]); robot=robot_config();
    for theta = [0,pi/12,pi/6,pi/4,pi/2,-pi/6,2*pi/3]
        pose = [info.origin_xy+[1.537 1.487],theta];
        body=rectangle_vertices(pose,robot.chassis.length,robot.chassis.width);
        for i=23:2:37
            for j=23:2:37
                map(:)=0; map(i,j)=1;
                cell_poly=cell_vertices(i,j,info);
                expected=polygons_intersect(body,cell_poly);
                actual=check_footprint_collision(pose,map,info,robot);
                verifyEqual(testCase,actual,expected, ...
                    sprintf('yaw=%.4f row=%d col=%d',theta,i,j));
            end
        end
    end
end

function testRotatedCornerIsNotOmittedByScanBounds(testCase)
    [map,info] = empty_map(2,0.005,[0 0]); robot=robot_config();
    pose=[1.0025 1.0025 pi/4];
    corner=rectangle_vertices(pose,robot.chassis.length,robot.chassis.width);
    [~,k]=max(corner(:,1)); p=0.995*corner(k,:)+0.005*pose(1:2);
    map=place_cell(map,info,p(1),p(2));
    verifyTrue(testCase,check_footprint_collision(pose,map,info,robot));
end

function testFootprintOutsideEmptyMapIsCollision(testCase)
    [map,info]=empty_map(3,0.05,[4 -2]); robot=robot_config();
    pose=[4.10 -0.5 0];
    [flag,gap,kind]=check_footprint_collision(pose,map,info,robot);
    verifyTrue(testCase,flag); verifyEqual(testCase,gap,0);
    verifyEqual(testCase,kind,'boundary');
    [flag,gap,kind]=check_robot_footprint_collision(pose,robot.footprint, ...
        struct('grid_map',map,'map_info',info));
    verifyTrue(testCase,flag); verifyEqual(testCase,gap,0);
    verifyEqual(testCase,kind,'boundary');
end

function testClearanceMeasuresBodyEdgeToCellEdge(testCase)
    [map,info]=empty_map(4,0.05,[0 0]); robot=robot_config(); pose=[1 1 0];
    map=place_cell(map,info,1.525,1.025);
    gap=compute_footprint_clearance(pose,map,info,robot);
    verifyEqual(testCase,gap,1.50-(1+robot.chassis.length/2),'AbsTol',1e-12);
    shifted=compute_footprint_clearance([1.01 1 0],map,info,robot);
    verifyEqual(testCase,gap-shifted,0.01,'AbsTol',1e-12);
    verifyFalse(testCase,check_footprint_collision(pose,map,info,robot));
end

function testRotatedClearanceMatchesIndependentVertexEdgeDistance(testCase)
    [map,info]=empty_map(4,0.05,[0 0]); robot=robot_config();
    pose=[1.012 1.019 pi/6]; map=place_cell(map,info,1.525,1.325);
    [i,j]=find(map); cell_poly=cell_vertices(i,j,info);
    body=rectangle_vertices(pose,robot.chassis.length,robot.chassis.width);
    expected=polygon_distance(body,cell_poly);
    verifyEqual(testCase,compute_footprint_clearance(pose,map,info,robot), ...
        expected,'AbsTol',1e-12);
end

function testOriginTranslationPreservesCollisionAndClearance(testCase)
    [map,info]=empty_map(4,0.05,[0 0]); robot=robot_config();
    pose=[1.012 1.019 -pi/6]; map=place_cell(map,info,1.525,1.325);
    [flag,gap]=check_footprint_collision(pose,map,info,robot);
    translated=info; translated.origin_xy=[5.7 -8.3];
    [flag2,gap2]=check_footprint_collision([pose(1:2)+translated.origin_xy,pose(3)], ...
        map,translated,robot);
    verifyEqual(testCase,flag2,flag); verifyEqual(testCase,gap2,gap,'AbsTol',1e-12);
end

function testCircleIgnoresFreeCellsAndAccountsForCellSquare(testCase)
    [map,info]=empty_map(4,0.05,[0 0]);
    map=place_cell(map,info,1.525,1.025);
    scene=struct('grid_map',map,'map_info',info);
    footprint=struct('type','circle','radius_m',0.10);
    [flag,gap,kind]=check_robot_footprint_collision([1 1 0],footprint,scene);
    verifyFalse(testCase,flag); verifyEqual(testCase,gap,0.40,'AbsTol',1e-12);
    verifyEqual(testCase,kind,'none');
    [flag,gap]=check_robot_footprint_collision([1.4 1 0],footprint,scene);
    verifyTrue(testCase,flag); verifyEqual(testCase,gap,0);
end

function [map,info]=empty_map(size_m,res,origin)
    n=round(size_m/res); map=zeros(n);
    info=struct('rows',n,'cols',n,'resolution',res,'width_m',size_m, ...
        'height_m',size_m,'origin_xy',origin);
end

function map=place_cell(map,info,x,y)
    [i,j]=world2grid(x,y,info.resolution,info.height_m,info.origin_xy);
    map(i,j)=1;
end

function p=rectangle_vertices(pose,L,W)
    local=[-L/2 -W/2;L/2 -W/2;L/2 W/2;-L/2 W/2];
    R=[cos(pose(3)) -sin(pose(3));sin(pose(3)) cos(pose(3))];
    p=local*R'+pose(1:2);
end

function p=cell_vertices(i,j,info)
    x=info.origin_xy(1)+(j-1)*info.resolution;
    y=info.origin_xy(2)+info.height_m-i*info.resolution;
    p=[x y;x+info.resolution y;x+info.resolution y+info.resolution;x y+info.resolution];
end

function yes=polygons_intersect(a,b)
    yes=any(inpolygon(a(:,1),a(:,2),b(:,1),b(:,2))) || ...
        any(inpolygon(b(:,1),b(:,2),a(:,1),a(:,2)));
    if yes,return;end
    for i=1:4
        p=a(i,:); q=a(mod(i,4)+1,:);
        for j=1:4
            r=b(j,:); s=b(mod(j,4)+1,:);
            d=cross2(q-p,s-r);
            if abs(d)<1e-12,continue;end
            t=cross2(r-p,s-r)/d; u=cross2(r-p,q-p)/d;
            if t>=0 && t<=1 && u>=0 && u<=1,yes=true;return;end
        end
    end
end

function z=cross2(a,b),z=a(1)*b(2)-a(2)*b(1);end

function d=polygon_distance(a,b)
    d=inf;
    for i=1:4
        for j=1:4
            d=min(d,point_segment_distance(a(i,:),b(j,:),b(mod(j,4)+1,:)));
            d=min(d,point_segment_distance(b(i,:),a(j,:),a(mod(j,4)+1,:)));
        end
    end
end

function d=point_segment_distance(p,a,b)
    t=max(0,min(1,dot(p-a,b-a)/dot(b-a,b-a)));
    d=norm(p-(a+t*(b-a)));
end
