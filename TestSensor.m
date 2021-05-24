% Made by Sensor and Control for mechatronics System Group 4
% Tien Dat Luu, Phan Nhan Truong and Le Bao Nguyen

% Calibration results

% Focal length (pixels):   [  908.46337890625     906.6737670898438 ]
% Principal point (pixels):[  644.8220825195312      370.8726501464844  ]
% Radial distortion:       [    0.1883 +/- 0.0052       -0.5543 +/- 0.0274        0.4529 +/- 0.0481  ]

clc;
clf;

rosinit % Turn on ROS 

% The information from April tag, we reference from source: https://au.mathworks.com/help/vision/ref/readapriltag.html 

mess = rossubscriber('/camera/color/image_raw/compressed');
pause(1);
late = mess.LatestMessage;
raw_img = readImage(late);
imshow(raw_img);

% Choose tagFamily
tagFamily = ["tag36h11"];

% Find location of tag in the image
[id,loc,detectedFamily] = readAprilTag(raw_img,tagFamily);

for idx = 1:length(id)
        % Display the ID and tag family
        disp("Detected Tag ID, Family: " + id(idx) + ", " ...
            + detectedFamily{idx});
 
        % Insert markers to indicate the locations
        markerRadius = 8;
        numCorners = size(loc,1);
        markerPosition = [loc(:,:,idx)];
        for a = 1:4
            I = insertText(raw_img, markerPosition(a,:), loc(a,1));
        end
end

% Create XY into the center of image

loc(:,1,1) = loc(:,1,1)-640;
loc(:,2,1) = loc(:,2,1)-360;

loc(:,1,2) = loc(:,1,2)-640;
loc(:,2,2) = loc(:,2,2)-360;

u1 = sum(loc(:,1,1))/4;
v1 = sum(loc(:,2,1))/4;

u2 = sum(loc(:,1,2))/4;
v2 = sum(loc(:,2,2))/4;


% Create the center marker of the image
centerMarker = [640 , 360; 320 ,360; 960 , 360; 640 , 180; 640 , 540];
raw_img = insertMarker(raw_img,centerMarker);

imshow(raw_img);

% Calibration measurement
% Focal Length
fx = 908.46337890625 ;
fy = 906.6737670898438;

% Principal Points
px = 644.8220825195312;
py = 370.8726501464844;
% Depth
z = 585;


% Apply the calibration to calculate XY (m)

x1 = (((u1*z)-px)/fx)/1000;
y1 = (((v1*z)-py)/fy)/1000;

x2 = (((u2*z)-px)/fx)/1000;
y2 = (((v2*z)-py)/fy)/1000;


% Robot position (x = 0.29m , y = 0 compare to camera and end-effecter x = 0.03 )

x = 0.065;
y = -0.23;
z = 0.07;

zRobot = -0.0445;

x1Robot = 0.29 - y1 -0.03;
y1Robot = -x1;

x2Robot = 0.29 - y2 - 0.03;
y2Robot = -x2;

%% Dobot moving
% For the control Dobot, we use the package from this website: https://github.com/gapaul/dobot_magician_driver
% Block 1
% Moving the block 1
endEffectorPosition = [x1Robot,y1Robot,zRobot + 0.03];
endEffectorRotation = [0,0,0];

[targetEndEffectorPub,targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');

targetEndEffectorMsg.Position.X = endEffectorPosition(1);
targetEndEffectorMsg.Position.Y = endEffectorPosition(2);
targetEndEffectorMsg.Position.Z = endEffectorPosition(3);

qua = eul2quat(endEffectorRotation);
targetEndEffectorMsg.Orientation.W = qua(1);
targetEndEffectorMsg.Orientation.X = qua(2);
targetEndEffectorMsg.Orientation.Y = qua(3);
targetEndEffectorMsg.Orientation.Z = qua(4);

send(targetEndEffectorPub,targetEndEffectorMsg);

pause(4);


endEffectorPosition = [x1Robot,y1Robot,zRobot];
endEffectorRotation = [0,0,0];

[targetEndEffectorPub,targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');

targetEndEffectorMsg.Position.X = endEffectorPosition(1);
targetEndEffectorMsg.Position.Y = endEffectorPosition(2);
targetEndEffectorMsg.Position.Z = endEffectorPosition(3);

qua = eul2quat(endEffectorRotation);
targetEndEffectorMsg.Orientation.W = qua(1);
targetEndEffectorMsg.Orientation.X = qua(2);
targetEndEffectorMsg.Orientation.Y = qua(3);
targetEndEffectorMsg.Orientation.Z = qua(4);

send(targetEndEffectorPub,targetEndEffectorMsg);

pause(4);

% Turn on suction cup
[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [1]; % Send 1 for on and 0 for off 
send(toolStatePub,toolStateMsg);

% Moving the box place
endEffectorPosition = [x,y,z];
endEffectorRotation = [0,0,0];

[targetEndEffectorPub,targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');

targetEndEffectorMsg.Position.X = endEffectorPosition(1);
targetEndEffectorMsg.Position.Y = endEffectorPosition(2);
targetEndEffectorMsg.Position.Z = endEffectorPosition(3);

qua = eul2quat(endEffectorRotation);
targetEndEffectorMsg.Orientation.W = qua(1);
targetEndEffectorMsg.Orientation.X = qua(2);
targetEndEffectorMsg.Orientation.Y = qua(3);
targetEndEffectorMsg.Orientation.Z = qua(4);

send(targetEndEffectorPub,targetEndEffectorMsg);

pause(4);

% Turn off suction cup
[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [0]; % Send 1 for on and 0 for off 
send(toolStatePub,toolStateMsg);


% Block 2 
% Moving the block 2
endEffectorPosition = [x2Robot,y2Robot,zRobot + 0.03];
endEffectorRotation = [0,0,0];

[targetEndEffectorPub,targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');

targetEndEffectorMsg.Position.X = endEffectorPosition(1);
targetEndEffectorMsg.Position.Y = endEffectorPosition(2);
targetEndEffectorMsg.Position.Z = endEffectorPosition(3);

qua = eul2quat(endEffectorRotation);
targetEndEffectorMsg.Orientation.W = qua(1);
targetEndEffectorMsg.Orientation.X = qua(2);
targetEndEffectorMsg.Orientation.Y = qua(3);
targetEndEffectorMsg.Orientation.Z = qua(4);

send(targetEndEffectorPub,targetEndEffectorMsg);

pause(2);


endEffectorPosition = [x2Robot,y2Robot,zRobot];
endEffectorRotation = [0,0,0];

[targetEndEffectorPub,targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');

targetEndEffectorMsg.Position.X = endEffectorPosition(1);
targetEndEffectorMsg.Position.Y = endEffectorPosition(2);
targetEndEffectorMsg.Position.Z = endEffectorPosition(3);

qua = eul2quat(endEffectorRotation);
targetEndEffectorMsg.Orientation.W = qua(1);
targetEndEffectorMsg.Orientation.X = qua(2);
targetEndEffectorMsg.Orientation.Y = qua(3);
targetEndEffectorMsg.Orientation.Z = qua(4);

send(targetEndEffectorPub,targetEndEffectorMsg);

pause(4);

% Turn on suction cup
[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [1]; % Send 1 for on and 0 for off 
send(toolStatePub,toolStateMsg);

% Moving the box place
endEffectorPosition = [x,y,z];
endEffectorRotation = [0,0,0];

[targetEndEffectorPub,targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');

targetEndEffectorMsg.Position.X = endEffectorPosition(1);
targetEndEffectorMsg.Position.Y = endEffectorPosition(2);
targetEndEffectorMsg.Position.Z = endEffectorPosition(3);

qua = eul2quat(endEffectorRotation);
targetEndEffectorMsg.Orientation.W = qua(1);
targetEndEffectorMsg.Orientation.X = qua(2);
targetEndEffectorMsg.Orientation.Y = qua(3);
targetEndEffectorMsg.Orientation.Z = qua(4);

send(targetEndEffectorPub,targetEndEffectorMsg);

pause(4);

% Turn off suction cup
[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [0]; % Send 1 for on and 0 for off 
send(toolStatePub,toolStateMsg);

% Moving the home pose
jointTarget = [0,pi/4,pi/4,0]; % Remember that the Dobot has 4 joints by default.

[targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
trajectoryPoint.Positions = jointTarget;
targetJointTrajMsg.Points = trajectoryPoint;

send(targetJointTrajPub,targetJointTrajMsg);


rosshutdown % Turn off ROS