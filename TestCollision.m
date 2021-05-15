robot = Dobot();
hold on;

%% Create Cube(obstacle)
% [Y,Z] = meshgrid(-0.75:0.05:0.75,-0.75:0.05:0.75);
% sizeMat = size(Y);
% X = repmat(0.75,sizeMat(1),sizeMat(2));
% oneSideOfCube_h = surf(X,Y,Z);
% 
% % Combine one surface as a point cloud
% cubePoints = [X(:),Y(:),Z(:)];
% 
% % Make a cube by rotating the single side by 0,90,180,270, and around y to make the top and bottom faces
% cubePoints = [ cubePoints ...
%              ; cubePoints * rotz(pi/2)...
%              ; cubePoints * rotz(pi) ...
%              ; cubePoints * rotz(3*pi/2) ...
%              ; cubePoints * roty(pi/2) ...
%              ; cubePoints * roty(-pi/2)];         
%          
% % Plot the cube's point cloud         
% cubeAtOigin_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'r.');
% cubePoints = cubePoints + repmat([2,0,-0.5],size(cubePoints,1),1);
% cube_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'b.');
% axis equal

centerpnt = [0.25,0.2,0];
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


