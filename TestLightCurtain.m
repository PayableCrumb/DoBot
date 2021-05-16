%% Enviroment

% Table
[f,v,data] = plyread('table.ply','tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;

table_h = trisurf(f,v(:,1),v(:,2),v(:,3)...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
hold on


axis equal;

% fence
%[f,v,data] = plyread('fence.ply','tri');
%vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;

%fence_h = trisurf(f,v(:,1),v(:,2),v(:,3)...
%    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

%hold on

% Base
surf([-1,-1;1,1],[-1,1;-1,1],[0.01,0.01;0.01,0.01],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
hold on


% Box
%[f,v,data] = plyread('Redbox.ply','tri');                                           %Red Box
%vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;

%Redbox_h = trisurf(f,v(:,1),v(:,2),v(:,3)...
%    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

%[f,v,data] = plyread('Greenbox.ply','tri');                                         %Green Box
%vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;

%Greenbox_h = trisurf(f,v(:,1),v(:,2),v(:,3)...
%    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');


% Dobot
robot = Dobot;

% Blocks

% Red = RedBlock(transl(0.25,0.055,0.75));  % real z = 0.72
% Green = GreenBlock(transl(0.25,-0.055,0.75));

% Plot light curtain
[Y,Z] = meshgrid(-1.5:0.2:1.5,0:0.1:0.7);
sizeMat = size(Y);
X = repmat(0.75,sizeMat(1),sizeMat(2));
oneSideOfCube_h = surf(X,Y,Z,'FaceColor','red');
cubePoints = [X(:),Y(:),Z(:)];

% Plot sphere to intersect
x = 1.5;
sphereCenter = [x,1,0.2];
radius = 0.2;
[X,Y,Z] = sphere(20);
X = X * radius + sphereCenter(1);
Y = Y * radius + sphereCenter(2);
Z = Z * radius + sphereCenter(3);
spherePc_h = surf(X,Y,Z);

%% 
steps = 100;
q1 = [-2.3562 pi/4 pi/4 0 0];
q2 = [2.3562 pi/4 pi/4 0 0];
qMatrix1 = jtraj(q1,q2,steps);

for i = 1:steps
    x = x - 0.01;
    sphereCenter = [x,1,0.2];
    radius = 0.2;
    [X,Y,Z] = sphere(20);
    X = X * radius + sphereCenter(1);
    Y = Y * radius + sphereCenter(2);
    Z = Z * radius + sphereCenter(3);
    delete (spherePc_h);
    spherePc_h = surf(X,Y,Z);
    
    robot.model.animate(qMatrix1(i,:));
    drawnow();
    
    algebraicDist = GetAlgebraicDist(cubePoints, sphereCenter, radius);
    pointsInside = find(algebraicDist < 1);
    if(pointsInside >= 1)
       disp('UNSAFE: Robot stopped')
       break
    end
end

% input('');
% delete (spherePc_h);
% disp('Remove Object');
% 
% currentPose = robot.model.getpos;
% qMatrix2 = jtraj(currentPose,q2,steps);
% 
% for i = 1:steps
%     robot.model.animate(qMatrix2(i,:));
%     drawnow();
% end


%% Function
% GetAlgebraicDist
% determine the algebraic distance given a set of points and the center
% point and radii of an elipsoid
% *Inputs:* 
%
% _points_ (many*(2||3||6) double) x,y,z cartesian point
%
% _centerPoint_ (1 * 3 double) xc,yc,zc of an ellipsoid
%
% _radii_ (1 * 3 double) a,b,c of an ellipsoid
%
% *Returns:* 
%
% _algebraicDist_ (many*1 double) algebraic distance for the ellipsoid

function algebraicDist = GetAlgebraicDist(points, centerPoint, radii)

algebraicDist = ((points(:,1)-centerPoint(1))/radii).^2 ...
              + ((points(:,2)-centerPoint(2))/radii).^2 ...
              + ((points(:,3)-centerPoint(3))/radii).^2;
end
