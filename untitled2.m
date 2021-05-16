function varargout = GUIDE(varargin)
% GUIDE MATLAB code for GUIDE.fig
%      GUIDE, by itself, creates a new GUIDE or raises the existing
%      singleton*.
%
%      H = GUIDE returns the handle to a new GUIDE or the handle to
%      the existing singleton*.
%
%      GUIDE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUIDE.M with the given input arguments.
%
%      GUIDE('Property','Value',...) creates a new GUIDE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before GUIDE_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to GUIDE_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help GUIDE

% Last Modified by GUIDE v2.5 16-May-2021 22:10:02

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @GUIDE_OpeningFcn, ...
                   'gui_OutputFcn',  @GUIDE_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before GUIDE is made visible.
function GUIDE_OpeningFcn(hObject, eventdata, handles, varargin)
% Choose default command line output for GUIDE
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes GUIDE wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = GUIDE_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in btn_eStop.
function btn_eStop_Callback(hObject, eventdata, handles)
if get(handles.btn_eStop,'userdata')==1
set(handles.btn_eStop,'userdata',0);
else
set(handles.btn_eStop,'userdata',1);
end



% --- Executes on button press in btn_Load.
function btn_Load_Callback(hObject, eventdata, handles)
% hObject    handle to btn_Load (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cla(handles.axes1,'reset');
set(handles.btn_eStop, 'userdata', 1);
handles = guidata(hObject);
i = 1;
% your iterative computation here;
a = get(handles.btn_eStop, 'userdata')
handles.output = hObject;

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
% save('Dobot.m');
% Blocks

Red = RedBlock(transl(0.25,0.055,0.75));  % real z = 0.72
Green = GreenBlock(transl(0.25,-0.055,0.75));
disp('on')
handles.output=hObject;
handles.robot1.model = robot1.model;
handles.Red=Red;
handles.Green=Green;
handles.qHome1=qHome1
guidata(hObject,handles);



% --- Executes on button press in btn_Start.
function btn_Start_Callback(hObject, eventdata, handles)
% hObject    handle to btn_Start (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Moving 1

q1 = handles.robot1.model.getpos;
q2 = handles.robot1.model.ikcon(handles.Red.RedBlockPose);
qMatrix = jtraj(q1,q2,50);

for i = 1:50
    handles.robot1.model.animate(qMatrix(i,:));                                    % Moving the robot near the RedBlock
    drawnow();
    if get(handles.btn_eStop,'userdata')==0
     uiwait
 end
end

move1 = transl(0.05,-0.22,0.95);
        q1 = handles.robot1.model.getpos;
        q2 = handles.robot1.model.ikcon(move1,q1);
        qMatrix5 = jtraj(q1,q2,50);
        
for i = 1:50                                                       %% Plot the moving of robot 1 to build wall
            handles.robot1.model.animate(qMatrix5(i,:));
            newPose1 = handles.robot1.model.fkine(qMatrix5(i,:));
            handles.Red.move(newPose1);            
            drawnow();
            if get(handles.btn_eStop,'userdata')==0
            uiwait
           end
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

q0 = handles.robot1.model.getpos;
T = handles.robot1.model.fkine(q0);
qMatrix1(1,:) = handles.robot1.model.ikcon(T,q0);                 % Solve for joint angles

for i = 1:50-1
    xdot = (x(:,i+1) - x(:,i))/deltaT;                             % Calculate velocity at discrete time step
    J = handles.robot1.model.jacob0(qMatrix1(i,:));            % Get the Jacobian at the current state
    J = J(1:5,1:5);                          
    qdot = inv(J)*xdot;                             % Solve velocitities via RMRC
    qMatrix1(i+1,:) =  qMatrix1(i,:) + deltaT*qdot';      % Update next joint state
    handles.robot1.model.animate(qMatrix1(i,:));
    newPose1 = handles.robot1.model.fkine(qMatrix1(i,:));
    handles.Red.move(newPose1);
    drawnow();
  if get(handles.btn_eStop,'userdata')==0
            uiwait
           end
end

for i = 1:49
    handles.robot1.model.animate(qMatrix1(50-i,:));
    drawnow();
   if get(handles.btn_eStop,'userdata')==0
            uiwait
           end
end

% Moving 2

q1 = handles.robot1.model.getpos;
q2 = handles.robot1.model.ikcon(handles.Green.GreenBlockPose);
qMatrix2 = jtraj(q1,q2,50);


for i = 1:50
    handles.robot1.model.animate(qMatrix2(i,:));                                    % Moving the robot near the RedBlock
    drawnow();
    if get(handles.btn_eStop,'userdata')==0
            uiwait
           end
end

move2 = transl(-0.04,-0.22,0.95);
        q1 = handles.robot1.model.getpos;
        q2 = handles.robot1.model.ikcon(move2,q1);
        qMatrix6 = jtraj(q1,q2,50);
        
for i = 1:50                                                       %% Plot the moving of robot 1 to build wall
            handles.robot1.model.animate(qMatrix6(i,:));
            newPose1 = handles.robot1.model.fkine(qMatrix6(i,:));
            handles.Green.move(newPose1);            
            drawnow();
           if get(handles.btn_eStop,'userdata')==0
            uiwait
           end
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

q0 = handles.robot1.model.getpos;
T = handles.robot1.model.fkine(q0);
qMatrix3(1,:) = handles.robot1.model.ikcon(T,q0);                 % Solve for joint angles

for i = 1:50-1
    xdot = (x(:,i+1) - x(:,i))/deltaT;                             % Calculate velocity at discrete time step
    J = handles.robot1.model.jacob0(qMatrix3(i,:));            % Get the Jacobian at the current state
    J = J(1:5,1:5);                          
    qdot = inv(J)*xdot;                             % Solve velocitities via RMRC
    qMatrix3(i+1,:) =  qMatrix3(i,:) + deltaT*qdot';      % Update next joint state
    handles.robot1.model.animate(qMatrix3(i,:));
    newPose1 = handles.robot1.model.fkine(qMatrix3(i,:));
    handles.Green.move(newPose1);
   if get(handles.btn_eStop,'userdata')==0
            uiwait
           end
    drawnow();
end

for i = 1:49
    handles.robot1.model.animate(qMatrix3(50-i,:));
    drawnow();
    if get(handles.btn_eStop,'userdata')==0
            uiwait
           end
end

% Home

q1 = handles.robot1.model.getpos;
q2 = handles.qHome1;


qMatrix4 = jtraj(q1,q2,50);

for i = 1:50                                                                % Moving the robot to original pose
    handles.robot1.model.animate(qMatrix4(i,:));
    drawnow();
   if get(handles.btn_eStop,'userdata')==0
            uiwait
           end
end

% %% Real Dobot Run
% % Moving 1
% jointTarget = [0,pi/4,pi/4,0]; % Remember that the Dobot has 4 joints by default.
% 
% [targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
% trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
% trajectoryPoint.Positions = jointTarget;
% targetJointTrajMsg.Points = trajectoryPoint;
% 
% send(targetJointTrajPub,targetJointTrajMsg);
% 
% pause(2);
% % 
% jointTarget = [qMatrix(50,1:3),0]; % Remember that the Dobot has 4 joints by default.
% 
% [targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
% trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
% trajectoryPoint.Positions = jointTarget;
% targetJointTrajMsg.Points = trajectoryPoint;
% 
% send(targetJointTrajPub,targetJointTrajMsg);
% 
% pause(3);
% 
% % Turn on the tool
% [toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
% toolStateMsg.Data = [1]; % Send 1 for on and 0 for off 
% send(toolStatePub,toolStateMsg);
% 
% jointTarget = [qMatrix5(50,1:3),0]; % Remember that the Dobot has 4 joints by default.
% 
% [targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
% trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
% trajectoryPoint.Positions = jointTarget;
% targetJointTrajMsg.Points = trajectoryPoint;
% 
% send(targetJointTrajPub,targetJointTrajMsg);
% 
% pause(2);
% 
% jointTarget = [qMatrix1(50,1:3),0]; % Remember that the Dobot has 4 joints by default.
% 
% [targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
% trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
% trajectoryPoint.Positions = jointTarget;
% targetJointTrajMsg.Points = trajectoryPoint;
% 
% send(targetJointTrajPub,targetJointTrajMsg);
% 
% pause(3.5);
% 
% % Turn off the tool
% [toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
% toolStateMsg.Data = [0]; % Send 1 for on and 0 for off 
% send(toolStatePub,toolStateMsg);
% 
% jointTarget = [qMatrix1(1,1:3),0]; % Remember that the Dobot has 4 joints by default.
% 
% [targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
% trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
% trajectoryPoint.Positions = jointTarget;
% targetJointTrajMsg.Points = trajectoryPoint;
% 
% send(targetJointTrajPub,targetJointTrajMsg);
% 
% % Moving 2
% pause(2);
% 
% jointTarget = [qMatrix2(50,1:3),0]; % Remember that the Dobot has 4 joints by default.
% 
% [targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
% trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
% trajectoryPoint.Positions = jointTarget;
% targetJointTrajMsg.Points = trajectoryPoint;
% 
% send(targetJointTrajPub,targetJointTrajMsg);
% 
% pause(3);
% 
% % Turn on the tool
% [toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
% toolStateMsg.Data = [1]; % Send 1 for on and 0 for off 
% send(toolStatePub,toolStateMsg);
% 
% jointTarget = [qMatrix6(50,1:3),0]; % Remember that the Dobot has 4 joints by default.
% 
% [targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
% trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
% trajectoryPoint.Positions = jointTarget;
% targetJointTrajMsg.Points = trajectoryPoint;
% 
% send(targetJointTrajPub,targetJointTrajMsg);
% 
% pause(2);
% 
% jointTarget = [qMatrix3(50,1:3),0]; % Remember that the Dobot has 4 joints by default.
% 
% [targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
% trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
% trajectoryPoint.Positions = jointTarget;
% targetJointTrajMsg.Points = trajectoryPoint;
% 
% send(targetJointTrajPub,targetJointTrajMsg);
% 
% pause(3.5);
% 
% % Turn on the tool
% [toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
% toolStateMsg.Data = [0]; % Send 1 for on and 0 for off 
% send(toolStatePub,toolStateMsg);
% 
% jointTarget = [qMatrix3(1,1:3),0]; % Remember that the Dobot has 4 joints by default.
% 
% [targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
% trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
% trajectoryPoint.Positions = jointTarget;
% targetJointTrajMsg.Points = trajectoryPoint;
% 
% send(targetJointTrajPub,targetJointTrajMsg);
% 
% pause(2);
% 
% jointTarget = [qMatrix4(50,1:3),0]; % Remember that the Dobot has 4 joints by default.
% 
% [targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
% trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
% trajectoryPoint.Positions = jointTarget;
% targetJointTrajMsg.Points = trajectoryPoint;
% 
% send(targetJointTrajPub,targetJointTrajMsg);
% 

handles.output=hObject;
guidata(hObject,handles);


% --- Executes on button press in btn_Continue.
function btn_Continue_Callback(hObject, eventdata, handles)
if get(handles.btn_eStop,'userdata')==0
set(handles.btn_eStop,'userdata',1);
uiresume
end


% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in btn_Collision.
function btn_Collision_Callback(hObject, eventdata, handles)
handles = guidata(hObject);
disp('Collision');
cla(handles.axes1,'reset');
%axes(handles.axes2);
TestCollision
guidata(hObject,handles);



% --- Executes on button press in Light.
function Light_Callback(hObject, eventdata, handles)
handles = guidata(hObject);
disp('Light Curtain');
cla(handles.axes1,'reset');
%axes(handles.axes3);
TestLightCurtain
guidata(hObject,handles);


% --- Executes on button press in btn_Control.
function btn_Control_Callback(hObject, eventdata, handles)
handles = guidata(hObject);
disp('Light Curtain');
cla(handles.axes1,'reset');
%axes(handles.axes3);
untitled
guidata(hObject,handles);
