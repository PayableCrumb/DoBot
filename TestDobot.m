%% Enviroment

% Table
[f,v,data] = plyread('table.ply','tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;

table_h = trisurf(f,v(:,1),v(:,2),v(:,3)...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
hold on


axis equal;

% fence
[f,v,data] = plyread('fence.ply','tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;

fence_h = trisurf(f,v(:,1),v(:,2),v(:,3)...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

hold on

% Base
surf([-2,-2;2,2],[-2,2;-2,2],[0.01,0.01;0.01,0.01],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
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

Red = RedBlock(transl(0.25,0.055,0.75));  % real z = 0.72
Green = GreenBlock(transl(0.25,-0.055,0.75));


% Moving 1

q1 = robot1.model.getpos;
q2 = robot1.model.ikcon(Red.RedBlockPose);
qMatrix = jtraj(q1,q2,50);

for i = 1:50
    robot1.model.animate(qMatrix(i,:));                                    % Moving the robot near the RedBlock
    drawnow();
end

move1 = transl(0.05,-0.22,0.95);
        q1 = robot1.model.getpos;
        q2 = robot1.model.ikcon(move1,q1);
        qMatrix5 = jtraj(q1,q2,50);
        
for i = 1:50                                                       %% Plot the moving of robot 1 to build wall
            robot1.model.animate(qMatrix5(i,:));
            newPose1 = robot1.model.fkine(qMatrix5(i,:));
            Red.move(newPose1);            
            drawnow();
end   

% RMRC 1

deltaT = 0.05;                                        % Discrete time step
x = zeros(3,50);
s = lspb(0,1,50);                                 % Create interpolation scalar
for i = 1:50
    x(1,i) = 0.05*(1-s(i)) + s(i)*0.05;
    x(2,i) = -0.22*(1-s(i)) + s(i)*-0.22;
    x(3,i) = 0.95*(1-s(i)) + s(i)*0.75; 
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

% Moving 2

q1 = robot1.model.getpos;
q2 = robot1.model.ikcon(Green.GreenBlockPose);
qMatrix2 = jtraj(q1,q2,50);


for i = 1:50
    robot1.model.animate(qMatrix2(i,:));                                    % Moving the robot near the RedBlock
    drawnow();
end

move2 = transl(-0.04,-0.22,0.95);
        q1 = robot1.model.getpos;
        q2 = robot1.model.ikcon(move2,q1);
        qMatrix6 = jtraj(q1,q2,50);
        
for i = 1:50                                                       %% Plot the moving of robot 1 to build wall
            robot1.model.animate(qMatrix6(i,:));
            newPose1 = robot1.model.fkine(qMatrix6(i,:));
            Green.move(newPose1);            
            drawnow();
end    

% RMRC 2

deltaT = 0.05;                                        % Discrete time step
x = zeros(3,50);
s = lspb(0,1,50);                                 % Create interpolation scalar
for i = 1:50
    x(1,i) = -0.04*(1-s(i)) + s(i)*-0.04;
    x(2,i) = -0.22*(1-s(i)) + s(i)*-0.22;
    x(3,i) = 0.95*(1-s(i)) + s(i)*0.75; 
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

%% Real Dobot Run
% Moving 1
jointTarget = [0,pi/4,pi/4,0]; % Remember that the Dobot has 4 joints by default.

[targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
trajectoryPoint.Positions = jointTarget;
targetJointTrajMsg.Points = trajectoryPoint;

send(targetJointTrajPub,targetJointTrajMsg);

pause(2);
% 
jointTarget = [qMatrix(50,1:3),0]; % Remember that the Dobot has 4 joints by default.

[targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
trajectoryPoint.Positions = jointTarget;
targetJointTrajMsg.Points = trajectoryPoint;

send(targetJointTrajPub,targetJointTrajMsg);

pause(3);

% Turn on the tool
[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [1]; % Send 1 for on and 0 for off 
send(toolStatePub,toolStateMsg);

jointTarget = [qMatrix5(50,1:3),0]; % Remember that the Dobot has 4 joints by default.

[targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
trajectoryPoint.Positions = jointTarget;
targetJointTrajMsg.Points = trajectoryPoint;

send(targetJointTrajPub,targetJointTrajMsg);

pause(2);

jointTarget = [qMatrix1(50,1:3),0]; % Remember that the Dobot has 4 joints by default.

[targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
trajectoryPoint.Positions = jointTarget;
targetJointTrajMsg.Points = trajectoryPoint;

send(targetJointTrajPub,targetJointTrajMsg);

pause(3.5);

% Turn off the tool
[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [0]; % Send 1 for on and 0 for off 
send(toolStatePub,toolStateMsg);

jointTarget = [qMatrix1(1,1:3),0]; % Remember that the Dobot has 4 joints by default.

[targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
trajectoryPoint.Positions = jointTarget;
targetJointTrajMsg.Points = trajectoryPoint;

send(targetJointTrajPub,targetJointTrajMsg);

% Moving 2
pause(2);

jointTarget = [qMatrix2(50,1:3),0]; % Remember that the Dobot has 4 joints by default.

[targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
trajectoryPoint.Positions = jointTarget;
targetJointTrajMsg.Points = trajectoryPoint;

send(targetJointTrajPub,targetJointTrajMsg);

pause(3);

% Turn on the tool
[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [1]; % Send 1 for on and 0 for off 
send(toolStatePub,toolStateMsg);

jointTarget = [qMatrix6(50,1:3),0]; % Remember that the Dobot has 4 joints by default.

[targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
trajectoryPoint.Positions = jointTarget;
targetJointTrajMsg.Points = trajectoryPoint;

send(targetJointTrajPub,targetJointTrajMsg);

pause(2);

jointTarget = [qMatrix3(50,1:3),0]; % Remember that the Dobot has 4 joints by default.

[targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
trajectoryPoint.Positions = jointTarget;
targetJointTrajMsg.Points = trajectoryPoint;

send(targetJointTrajPub,targetJointTrajMsg);

pause(3.5);

% Turn on the tool
[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [0]; % Send 1 for on and 0 for off 
send(toolStatePub,toolStateMsg);

jointTarget = [qMatrix3(1,1:3),0]; % Remember that the Dobot has 4 joints by default.

[targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
trajectoryPoint.Positions = jointTarget;
targetJointTrajMsg.Points = trajectoryPoint;

send(targetJointTrajPub,targetJointTrajMsg);

pause(2);

jointTarget = [qMatrix4(50,1:3),0]; % Remember that the Dobot has 4 joints by default.

[targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
trajectoryPoint.Positions = jointTarget;
targetJointTrajMsg.Points = trajectoryPoint;

send(targetJointTrajPub,targetJointTrajMsg);
