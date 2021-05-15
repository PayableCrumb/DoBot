%% Plot Dobot
robot = Dobot();
hold on;

Cube = false;

%% Create Cube(obstacle)
if Cube == true
    centerpnt = [0.25,0.2,0.865];
    side = 0.1;
    plotOptions.plotFaces = true;
    [vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);
    axis equal
    disp('There is an object in the path');
end

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
    if Cube == true
        result(i) = IsCollision(robot,qMatrix(i,:),faces,vertex,faceNormals,false);
        if result(i) >= 1
            qCollide = qMatrix(i,:); 
        break;
        end
    end
    robot.model.animate(qMatrix(i,:));
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
                plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
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