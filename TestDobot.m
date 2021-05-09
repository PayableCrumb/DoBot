%% Enviroment

% Table
[f,v,data] = plyread('table.ply','tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;

table_h = trisurf(f,v(:,1),v(:,2),v(:,3)...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
hold on


axis equal;

% Base
surf([-1,-1;1,1],[-1,1;-1,1],[0.01,0.01;0.01,0.01],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
hold on


% Box
[f,v,data] = plyread('Redbox.ply','tri');                                           %Red Box
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;

Redbox_h = trisurf(f,v(:,1),v(:,2),v(:,3)...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

[f,v,data] = plyread('Greenbox.ply','tri');                                         %Green Box
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;

Greenbox_h = trisurf(f,v(:,1),v(:,2),v(:,3)...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');


% Dobot
robot1 = Dobot;
qHome1 = robot1.model.getpos;

% Blocks

Red = RedBlock(transl(-0.05,0,0.745)*trotx(pi));
Green = GreenBlock(transl(-0.1,0,0.745)*trotx(pi));


%% Moving 1

q1 = robot1.model.getpos;
q2 = robot1.model.ikcon(Red.RedBlockPose*transl(0,0,-0.04));
qMatrix = jtraj(q1,q2,50);


for i = 1:50
    robot1.model.animate(qMatrix(i,:));                                    % Moving the robot near the RedBlock
    drawnow();
end

move1 = transl(-0.35,-0.25,0.85)*trotx(pi);
        q1 = robot1.model.getpos;
        q2 = robot1.model.ikcon(move1,q1);
        qMatrix = jtraj(q1,q2,50);
        
for i = 1:50                                                       %% Plot the moving of robot 1 to build wall
            robot1.model.animate(qMatrix(i,:));
            newPose1 = robot1.model.fkine(qMatrix(i,:));
            Red.move(newPose1);            
            drawnow();
end    

% RMRC

deltaT = 0.05;                                        % Discrete time step
x = zeros(3,50);
s = lspb(0,1,50);                                 % Create interpolation scalar
for i = 1:50
    x(1,i) = -0.35*(1-s(i)) + s(i)*-0.35;
    x(2,i) = -0.25*(1-s(i)) + s(i)*-0.25;
    x(3,i) = 0.85*(1-s(i)) + s(i)*0.785; 
    x(4,i) = 0;
    x(5,i) = 0;
end

qMatrix1 = nan(50,5);

q0 = robot1.model.getpos;
T = robot1.model.fkine(q0);
qMatrix1(1,:) = robot1.model.ikcon(T,q0);                 % Solve for joint angles

for i = 1:50-1
    xdot = (x(:,i+1) - x(:,i))/deltaT;                             % Calculate velocity at discrete time step
    J = robot1.model.jacob0(qMatrix1(i,:));            % Get the Jacobian at the current state
    J = J(1:5,1:5);                          
    qdot = inv(J)*xdot;                             % Solve velocitities via RMRC
    qMatrix1(i+1,:) =  qMatrix1(i,:) + deltaT*qdot';      % Update next joint state
    robot1.model.animate(qMatrix1(i,:));
    newPose1 = robot1.model.fkine(qMatrix1(i,:));
    Red.move(newPose1);
    drawnow();
end

for i = 1:49
    robot1.model.animate(qMatrix1(50-i,:));
    drawnow();
end

%% Moving 2

q1 = robot1.model.getpos;
q2 = robot1.model.ikcon(Green.GreenBlockPose*transl(0,0,-0.04));
qMatrix2 = jtraj(q1,q2,50);


for i = 1:50
    robot1.model.animate(qMatrix2(i,:));                                    % Moving the robot near the RedBlock
    drawnow();
end

move2 = transl(-0.26,-0.26,0.85)*trotx(pi);
        q1 = robot1.model.getpos;
        q2 = robot1.model.ikcon(move2,q1);
        qMatrix2 = jtraj(q1,q2,50);
        
for i = 1:50                                                       %% Plot the moving of robot 1 to build wall
            robot1.model.animate(qMatrix2(i,:));
            newPose1 = robot1.model.fkine(qMatrix2(i,:));
            Green.move(newPose1);            
            drawnow();
end    


% RMRC

deltaT = 0.05;                                        % Discrete time step
x = zeros(3,50);
s = lspb(0,1,50);                                 % Create interpolation scalar
for i = 1:50
    x(1,i) = -0.26*(1-s(i)) + s(i)*-0.26;
    x(2,i) = -0.25*(1-s(i)) + s(i)*-0.25;
    x(3,i) = 0.85*(1-s(i)) + s(i)*0.785; 
    x(4,i) = 0;
    x(5,i) = 0;
end

qMatrix3 = nan(50,5);

q0 = robot1.model.getpos;
T = robot1.model.fkine(q0);
qMatrix3(1,:) = robot1.model.ikcon(T,q0);                 % Solve for joint angles

for i = 1:50-1
    xdot = (x(:,i+1) - x(:,i))/deltaT;                             % Calculate velocity at discrete time step
    J = robot1.model.jacob0(qMatrix3(i,:));            % Get the Jacobian at the current state
    J = J(1:5,1:5);                          
    qdot = inv(J)*xdot;                             % Solve velocitities via RMRC
    qMatrix3(i+1,:) =  qMatrix3(i,:) + deltaT*qdot';      % Update next joint state
    robot1.model.animate(qMatrix3(i,:));
    newPose1 = robot1.model.fkine(qMatrix3(i,:));
    Green.move(newPose1);
    drawnow();
end

for i = 1:49
    robot1.model.animate(qMatrix3(50-i,:));
    drawnow();
end
% Home

q1 = robot1.model.getpos;
q2 = qHome1;

qMatrix4 = jtraj(q1,q2,50);

for i = 1:50                                                                % Moving the robot to original pose
    robot1.model.animate(qMatrix4(i,:));
    drawnow();
end
