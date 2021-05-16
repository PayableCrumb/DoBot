%% Plot Dobot
robot = Dobot;
hold on;

% Base
surf([-1,-1;1,1],[-1,1;-1,1],[0.01,0.01;0.01,0.01],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
hold on

% Table
[f,v,data] = plyread('table.ply','tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;

table_h = trisurf(f,v(:,1),v(:,2),v(:,3)...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
hold on

%% Create Cube(obstacle)
centerpnt = [0.25,0.2,0.865];
side = 0.1;
plotOptions.plotFaces = true;
[vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);
axis equal
 

%% Get the transform of every joint (i.e. start and end of every link)
n = robot.model.n;
tr = zeros(4,4,n);
tr(:,:,1) = robot.model.base;
L = robot.model.links;
for i = 1 : robot.model.n
   tr(:,:,i+1) = tr(:,:,i) * trotz(robot.q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
end

% Go through until there are no step sizes larger than 1 degree
q1 = robot.model.getpos;
q2 = [pi/2,pi/4,pi/4,0,0];
steps = 2;
while ~isempty(find(1 < abs(diff(rad2deg(jtraj(q1,q2,steps)))),1))
    steps = steps + 1;
end
qMatrix = jtraj(q1,q2,steps);

% 
result = true(steps,1);
for i = 1: steps
    result(i) = IsCollision(robot,qMatrix(i,:),faces,vertex,faceNormals,false);
    if result(i) >= 1
      stepStop = i; 
      qMatrix(i,:); 
      break;
    end
    robot.model.animate(qMatrix(i,:));
    drawnow();
end

for i = stepStop:-1:stepStop-10
    robot.model.animate(qMatrix(i,:));
    stop = qMatrix(i,:);
    drawnow();
end

stopPose1 = robot.model.fkine(stop);
Pose1 = stopPose1(1:3,4);

% RMRC

deltaT = 0.05;                                        % Discrete time step
x = zeros(3,50);
s = lspb(0,1,50);                                 % Create interpolation scalar
for i = 1:50
    x(1,i) = Pose1(1)*(1-s(i)) + s(i)*Pose1(1);
    x(2,i) = Pose1(2)*(1-s(i)) + s(i)*Pose1(2);
    x(3,i) = Pose1(3)*(1-s(i)) + s(i)*0.92; 
    x(4,i) = 0;
    x(5,i) = 0;
end

qMatrix1 = nan(50,5);

q0 = robot.model.getpos;
T = robot.model.fkine(q0);
qMatrix1(1,:) = robot.model.ikcon(T,q0);                 % Solve for joint angles

for i = 1:50-1
    xdot = (x(:,i+1) - x(:,i))/deltaT;                             % Calculate velocity at discrete time step
    J = robot.model.jacob0(qMatrix1(i,:));            % Get the Jacobian at the current state
    J = J(1:5,1:5);                          
    qdot = inv(J)*xdot;                             % Solve velocitities via RMRC
    qMatrix1(i+1,:) =  qMatrix1(i,:) + deltaT*qdot';      % Update next joint state
    robot.model.animate(qMatrix1(i,:));
    drawnow();
end

q1 = robot.model.getpos;
qMove = [0.9758    0.4017    0.5149    0.6542   -0.4063];

qMatrix  = jtraj(q1,qMove,50);

for i = 1:50
    robot.model.animate(qMatrix(i,:));
    drawnow();
end

stopPose2 = robot.model.fkine(stop);
Pose2 = stopPose2(1:3,4);

% RMRC

deltaT = 0.05;                                        % Discrete time step
x = zeros(3,50);
s = lspb(0,1,50);                                 % Create interpolation scalar
for i = 1:50
    x(1,i) = Pose2(1)*(1-s(i)) + s(i)*Pose2(1);
    x(2,i) = Pose2(2)*(1-s(i)) + s(i)*Pose2(2);
    x(3,i) = Pose2(3)*(1-s(i)) + s(i)*0.7; 
    x(4,i) = 0;
    x(5,i) = 0;
end

qMatrix2 = nan(50,5);

q0 = robot.model.getpos;
T = robot.model.fkine(q0);
qMatrix2(1,:) = robot.model.ikcon(T,q0);                 % Solve for joint angles

for i = 1:50-1
    xdot = (x(:,i+1) - x(:,i))/deltaT;                             % Calculate velocity at discrete time step
    J = robot.model.jacob0(qMatrix2(i,:));            % Get the Jacobian at the current state
    J = J(1:5,1:5);                          
    qdot = inv(J)*xdot;                             % Solve velocitities via RMRC
    qMatrix2(i+1,:) =  qMatrix2(i,:) + deltaT*qdot';      % Update next joint state
    robot.model.animate(qMatrix2(i,:));
    drawnow();
end


%% IsCollision
% This is based upon the output of questions 2.5 and 2.6
% Given a robot model (robot), and trajectory (i.e. joint state vector) (qMatrix)
% and triangle obstacles in the environment (faces,vertex,faceNormals)
function result = IsCollision(robot,qMatrix,faces,vertex,faceNormals,returnOnceFound)
if nargin < 6
    returnOnceFound = true;
end
result = false;

for qIndex = 1:size(qMatrix,1)
    % Get the transform of every joint (i.e. start and end of every link)
    tr = GetLinkPoses(qMatrix(qIndex,:), robot);

    % Go through each link and also each triangle face
    for i = 1 : size(tr,3)-1    
        for faceIndex = 1:size(faces,1)
            vertOnPlane = vertex(faces(faceIndex,1)',:);
            [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
            if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
%                 plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                disp('Collision Detected');
                result = true;
                if returnOnceFound
                    return
                end
            end
        end    
    end
end
end

%% GetLinkPoses
% q - robot joint angles
% robot -  seriallink robot model
% transforms - list of transforms
function [ transforms ] = GetLinkPoses( q, robot)

links = robot.model.links;
transforms = zeros(4, 4, length(links) + 1);
transforms(:,:,1) = robot.model.base;

for i = 1:length(links)
    L = links(1,i);
    
    current_transform = transforms(:,:, i);
    
    current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
    transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
    transforms(:,:,i + 1) = current_transform;
end
end

%% IsIntersectionPointInsideTriangle
% Given a point which is known to be on the same plane as the triangle
% determine if the point is 
% inside (result == 1) or 
% outside a triangle (result ==0 )
function result = IsIntersectionPointInsideTriangle(intersectP,triangleVerts)

u = triangleVerts(2,:) - triangleVerts(1,:);
v = triangleVerts(3,:) - triangleVerts(1,:);

uu = dot(u,u);
uv = dot(u,v);
vv = dot(v,v);

w = intersectP - triangleVerts(1,:);
wu = dot(w,u);
wv = dot(w,v);

D = uv * uv - uu * vv;

% Get and test parametric coords (s and t)
s = (uv * wv - vv * wu) / D;
if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
    result = 0;
    return;
end

t = (uv * wu - uu * wv) / D;
if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
    result = 0;
    return;
end

result = 1;                      % intersectP is in Triangle
end
