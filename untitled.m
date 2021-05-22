function varargout = untitled(varargin)
% UNTITLED MATLAB code for untitled.fig
%      UNTITLED, by itself, creates a new UNTITLED or raises the existing
%      singleton*.
%
%      H = UNTITLED returns the handle to a new UNTITLED or the handle to
%      the existing singleton*.
%
%      UNTITLED('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in UNTITLED.M with the given input arguments.
%
%      UNTITLED('Property','Value',...) creates a new UNTITLED or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before untitled_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to untitled_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help untitled

% Last Modified by GUIDE v2.5 16-May-2021 19:26:51

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @untitled_OpeningFcn, ...
                   'gui_OutputFcn',  @untitled_OutputFcn, ...
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


function varargout = untitled_OutputFcn(hObject, eventdata, handles) 
varargout{1} = handles.output;

function untitled_OpeningFcn(hObject, eventdata, handles, varargin)
handles = guidata(hObject);
save('Dobot.mat');
workspace = [-1 1 -1 1 -0.3 1]; 
opt.deg = true;
opt.orientation = {'rpy', 'eul', 'approach'};
opt.callback = []; 
[opt,args] = tb_optparse(opt, varargin);

handles.orientation = opt.orientation;
handles.callback = opt.callback;
handles.opt = opt;
axes(handles.robot1);
robot = Dobot;
handles.output = hObject;

qlim = robot.model.qlim;

if any(isinf(qlim))
        error('RTB:teach:badarg', 'Must define joint coordinate limits for prismatic axes, set qlim properties for prismatic Links');
    end
    
    if isempty(args)
        q = [];
    else
        q = args{1};
    end
    % set up scale factor, from actual limits in radians/metres to display units
    
    qscale = ones(robot.model.n,1);
    
    for j=1:robot.model.n
          L1=robot.model.links(1);
          L2=robot.model.links(2);
          L3=robot.model.links(3);
          L4=robot.model.links(4);
          L5=robot.model.links(5);
        if opt.deg && L1.isrevolute
            qscale(1) = 180/pi;
        end
        if opt.deg && L2.isrevolute
            qscale(2) = 180/pi;
        end
        if opt.deg && L3.isrevolute
            qscale(3) = 180/pi;
        end
        if opt.deg && L4.isrevolute
            qscale(4) = 180/pi;
        end
        if opt.deg && L5.isrevolute
            qscale(5) = 180/pi;
        end
    end
    handles.qscale = qscale;
    handles.robot.model = robot.model;

    
    
        c = findobj(0,'Tag',robot.model.name);  % check all figure
        ax = get(c(1), 'Parent'); % get first axis holding the robot
    handles.fig = get(ax, 'Parent');  % get the figure that holds the axis
    
    % shrink the current axes to make room
    %   [l b w h]
   
    
    handles.curax = ax;
    
 %---- get the current robot state
  
    if isempty(q)
        % check to see if there are any graphical robots of this name
        rhandles = findobj('Tag', robot.model.name);
        
        % find the graphical element of this name
        if isempty(rhandles)
            error('RTB:teach:badarg', 'No graphical robot of this name found');
        end
        % get the info from its Userdata
        info = get(rhandles(1), 'UserData');
        if ~isempty(info.q)
            q = info.q;
        end     
    else
    robot.model.plot(q);   
    end
    handles.q = q;
    T6 = robot.model.fkine(q);
    
    handles.qscale = qscale;
    handles.robot.model = robot.model;
   
guidata(hObject, handles);

% --- Executes on button press in btn_Apply.
function btn_Apply_Callback(hObject, eventdata, handles)
handles = guidata(hObject);

global PX
global PY
global PZ
T = [ 1 0 0 handles.PX;
      0 1 0 handles.PY;
      0 0 1 handles.PZ;
      0 0 0 1];
J = handles.robot.model.getpos;
JJ = handles.robot.model.ikcon(T,J);
qMatrix = jtraj(J,JJ,50);

%animate(handles.robot.model,qMatrix)
for i = 1:50
    handles.robot.model.animate(qMatrix(i,:));                                    % Moving the robot near the RedBlock
    drawnow();
end

% handles.edit1.String = num2str(floor(JJ(1)));
% handles.edit2.String = num2str(floor(JJ(2)));
% handles.edit3.String = num2str(floor(JJ(3)));
% handles.edit4.String = num2str(floor(JJ(4)));

set(handles.edit1, 'string',num2str(round(rad2deg(JJ(1)))));
set(handles.edit2, 'string',num2str(round(rad2deg(JJ(2)))));
set(handles.edit3, 'string',num2str(round(rad2deg(JJ(3)))));
set(handles.edit4, 'string',num2str(round(rad2deg(JJ(4)))));


set(handles.slider1, 'value', rad2deg(JJ(1)));
set(handles.slider2, 'value', rad2deg(JJ(2)));
set(handles.slider3, 'value', rad2deg(JJ(3)));
set(handles.slider4, 'value', rad2deg(JJ(4)));
guidata(hObject, handles);

function edit1_Callback(hObject, eventdata, handles)
handles = guidata(hObject);

value1 = str2double(get(handles.edit1, 'String'))/ handles.qscale(1);
            set(handles.slider1, 'value', value1*handles.qscale(1));

 h = findobj('Tag',handles.robot.model.name);
    
    if isempty(h)
        error('RTB:teach:badarg', 'No graphical robot of this name found');
    end
    
    info = get(h(1), 'Userdata');
%     
       info.q(1) = value1;
%     
     set(h(1), 'UserData', info);
     
animate(handles.robot.model,info.q);
    
    T6 = handles.robot.model.fkine(info.q); 
switch handles.orientation
        case 'approach'
            orient = T6(:,3);    % approach vector
        case 'eul'
            orient = tr2eul(T6, 'setopt', handles.opt);
        case'rpy'
            orient = tr2rpy(T6, 'setopt', handles.opt);
    end
set(handles.X, 'String', sprintf('%.3f', T6(1,4)));
set(handles.Y, 'String', sprintf('%.3f', T6(2,4)));
set(handles.Z, 'String', sprintf('%.3f', T6(3,4)));
set(handles.R, 'String', sprintf('%.3f', orient(1)));
set(handles.P, 'String', sprintf('%.3f', orient(2)));
set(handles.YY, 'String', sprintf('%.3f', orient(3)));

 guidata(hObject,handles)           


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit2_Callback(hObject, eventdata, handles)
handles = guidata(hObject);
global value2
value2 = str2double(get(handles.edit2, 'String'))/ handles.qscale(2);
            set(handles.slider2, 'value', value2*handles.qscale(2));
   
 h = findobj('Tag',handles.robot.model.name);
    
    if isempty(h)
        error('RTB:teach:badarg', 'No graphical robot of this name found');
    end
    
    info = get(h(1), 'Userdata');
%     
       info.q(2) = value2;
%     
     set(h(1), 'UserData', info);
     
animate(handles.robot.model,info.q);
    
    T6 = handles.robot.model.fkine(info.q); 
switch handles.orientation
        case 'approach'
            orient = T6(:,3);    % approach vector
        case 'eul'
            orient = tr2eul(T6, 'setopt', handles.opt);
        case'rpy'
            orient = tr2rpy(T6, 'setopt', handles.opt);
    end
set(handles.X, 'String', sprintf('%.3f', T6(1,4)));
set(handles.Y, 'String', sprintf('%.3f', T6(2,4)));
set(handles.Z, 'String', sprintf('%.3f', T6(3,4)));
set(handles.R, 'String', sprintf('%.3f', orient(1)));
set(handles.P, 'String', sprintf('%.3f', orient(2)));
set(handles.YY, 'String', sprintf('%.3f', orient(3)));

 guidata(hObject,handles)  


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit3_Callback(hObject, eventdata, handles)
handles = guidata(hObject);
global value3
value1 = str2double(get(handles.edit3, 'String'))/ handles.qscale(3);
            set(handles.slider3, 'value', value3*handles.qscale(3));
   
 h = findobj('Tag',handles.robot.model.name);
    
    if isempty(h)
        error('RTB:teach:badarg', 'No graphical robot of this name found');
    end
    
    info = get(h(1), 'Userdata');
%     
       info.q(3) = value3;
%     
     set(h(1), 'UserData', info);
     
animate(handles.robot.model,info.q);
    
    T6 = handles.robot.model.fkine(info.q); 

switch handles.orientation
        case 'approach'
            orient = T6(:,3);    % approach vector
        case 'eul'
            orient = tr2eul(T6, 'setopt', handles.opt);
        case'rpy'
            orient = tr2rpy(T6, 'setopt', handles.opt);
    end
set(handles.X, 'String', sprintf('%.3f', T6(1,4)));
set(handles.Y, 'String', sprintf('%.3f', T6(2,4)));
set(handles.Z, 'String', sprintf('%.3f', T6(3,4)));
set(handles.R, 'String', sprintf('%.3f', orient(1)));
set(handles.P, 'String', sprintf('%.3f', orient(2)));
set(handles.YY, 'String', sprintf('%.3f', orient(3)));
 guidata(hObject,handles)     


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit4_Callback(hObject, eventdata, handles)
handles = guidata(hObject);
global value4
value1 = str2double(get(handles.edit4, 'String'))/ handles.qscale(4);
            set(handles.slider4, 'value', value4*handles.qscale(4));
   
 h = findobj('Tag',handles.robot.model.name);
    
    if isempty(h)
        error('RTB:teach:badarg', 'No graphical robot of this name found');
    end
    
    info = get(h(1), 'Userdata');
%     
       info.q(4) = value4;
%     
     set(h(1), 'UserData', info);
     
animate(handles.robot.model,info.q);
    
    T6 = handles.robot.model.fkine(info.q); 
switch handles.orientation
        case 'approach'
            orient = T6(:,3);    % approach vector
        case 'eul'
            orient = tr2eul(T6, 'setopt', handles.opt);
        case'rpy'
            orient = tr2rpy(T6, 'setopt', handles.opt);
    end
set(handles.X, 'String', sprintf('%.3f', T6(1,4)));
set(handles.Y, 'String', sprintf('%.3f', T6(2,4)));
set(handles.Z, 'String', sprintf('%.3f', T6(3,4)));
set(handles.R, 'String', sprintf('%.3f', orient(1)));
set(handles.P, 'String', sprintf('%.3f', orient(2)));
set(handles.YY, 'String', sprintf('%.3f', orient(3)));

 guidata(hObject,handles)     


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles = guidata(hObject);
global value1
value1 = get(handles.slider1, 'value')/handles.qscale(1);
            set(handles.edit1, 'string',num2str(round(value1*handles.qscale(1))));
            h = findobj('Tag',handles.robot.model.name);
    
    if isempty(h)
        error('RTB:teach:badarg', 'No graphical robot of this name found');
    end
    
    info = get(h(1), 'Userdata');
%     
       info.q(1) = value1;
%     
     set(h(1), 'UserData', info);
     
animate(handles.robot.model,info.q);
    
    T6 = handles.robot.model.fkine(info.q); 
switch handles.orientation
        case 'approach'
            orient = T6(:,3);    % approach vector
        case 'eul'
            orient = tr2eul(T6, 'setopt', handles.opt);
        case'rpy'
            orient = tr2rpy(T6, 'setopt', handles.opt);
    end
set(handles.X, 'String', sprintf('%.3f', T6(1,4)));
set(handles.Y, 'String', sprintf('%.3f', T6(2,4)));
set(handles.Z, 'String', sprintf('%.3f', T6(3,4)));
set(handles.R, 'String', sprintf('%.3f', orient(1)));
set(handles.P, 'String', sprintf('%.3f', orient(2)));
set(handles.YY, 'String', sprintf('%.3f', orient(3)));
            guidata(hObject,handles)
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
handles = guidata(hObject);
global value2
value2 = get(handles.slider2, 'value')/handles.qscale(2);
            set(handles.edit2, 'string',num2str(round(value2*handles.qscale(2))));
            
            h = findobj('Tag',handles.robot.model.name);
    
    if isempty(h)
        error('RTB:teach:badarg', 'No graphical robot of this name found');
    end
    
    info = get(h(1), 'Userdata');
%     
       info.q(2) = value2;
%     
     set(h(1), 'UserData', info);
     
animate(handles.robot.model,info.q);
    
    T6 = handles.robot.model.fkine(info.q); 
switch handles.orientation
        case 'approach'
            orient = T6(:,3);    % approach vector
        case 'eul'
            orient = tr2eul(T6, 'setopt', handles.opt);
        case'rpy'
            orient = tr2rpy(T6, 'setopt', handles.opt);
    end
set(handles.X, 'String', sprintf('%.3f', T6(1,4)));
set(handles.Y, 'String', sprintf('%.3f', T6(2,4)));
set(handles.Z, 'String', sprintf('%.3f', T6(3,4)));
set(handles.R, 'String', sprintf('%.3f', orient(1)));
set(handles.P, 'String', sprintf('%.3f', orient(2)));
set(handles.YY, 'String', sprintf('%.3f', orient(3)));
            guidata(hObject,handles)

% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider3_Callback(hObject, eventdata, handles)
handles = guidata(hObject);
global value3
value3 = get(handles.slider3, 'value')/handles.qscale(3);
            set(handles.edit3, 'string',num2str(round(value3*handles.qscale(3))));
            h = findobj('Tag',handles.robot.model.name);
    
    if isempty(h)
        error('RTB:teach:badarg', 'No graphical robot of this name found');
    end
    
    info = get(h(1), 'Userdata');
%     
       info.q(3) = value3;
%     
     set(h(1), 'UserData', info);
     
animate(handles.robot.model,info.q);
    
    T6 = handles.robot.model.fkine(info.q); 
switch handles.orientation
        case 'approach'
            orient = T6(:,3);    % approach vector
        case 'eul'
            orient = tr2eul(T6, 'setopt', handles.opt);
        case'rpy'
            orient = tr2rpy(T6, 'setopt', handles.opt);
    end
set(handles.X, 'String', sprintf('%.3f', T6(1,4)));
set(handles.Y, 'String', sprintf('%.3f', T6(2,4)));
set(handles.Z, 'String', sprintf('%.3f', T6(3,4)));
set(handles.R, 'String', sprintf('%.3f', orient(1)));
set(handles.P, 'String', sprintf('%.3f', orient(2)));
set(handles.YY, 'String', sprintf('%.3f', orient(3)));
            guidata(hObject,handles)


% --- Executes during object creation, after setting all properties.
function slider3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider4_Callback(hObject, eventdata, handles)
handles = guidata(hObject);
global value4
value4 = get(handles.slider4, 'value')/handles.qscale(4);
            set(handles.edit4, 'string',num2str(round(value4*handles.qscale(4))));
            h = findobj('Tag',handles.robot.model.name);
    
    if isempty(h)
        error('RTB:teach:badarg', 'No graphical robot of this name found');
    end
    
    info = get(h(1), 'Userdata');
%     
       info.q(4) = value4;
%     
     set(h(1), 'UserData', info);
     
animate(handles.robot.model,info.q);
    
    T6 = handles.robot.model.fkine(info.q); 
switch handles.orientation
        case 'approach'
            orient = T6(:,3);    % approach vector
        case 'eul'
            orient = tr2eul(T6, 'setopt', handles.opt);
        case'rpy'
            orient = tr2rpy(T6, 'setopt', handles.opt);
    end
set(handles.X, 'String', sprintf('%.3f', T6(1,4)));
set(handles.Y, 'String', sprintf('%.3f', T6(2,4)));
set(handles.Z, 'String', sprintf('%.3f', T6(3,4)));
set(handles.R, 'String', sprintf('%.3f', orient(1)));
set(handles.P, 'String', sprintf('%.3f', orient(2)));
set(handles.YY, 'String', sprintf('%.3f', orient(3)));
            guidata(hObject,handles)


% --- Executes during object creation, after setting all properties.
function slider4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function P_Callback(hObject, eventdata, handles)
% hObject    handle to P (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of P as text
%        str2double(get(hObject,'String')) returns contents of P as a double


% --- Executes during object creation, after setting all properties.
function P_CreateFcn(hObject, eventdata, handles)
% hObject    handle to P (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function R_Callback(hObject, eventdata, handles)
% hObject    handle to R (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of R as text
%        str2double(get(hObject,'String')) returns contents of R as a double


% --- Executes during object creation, after setting all properties.
function R_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Z_Callback(hObject, eventdata, handles)
handles = guidata(hObject);
global PZ
PZ = str2double(get(handles.Z, 'String'));
handles.PZ=PZ
guidata(hObject,handles);

function Z_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Y_Callback(hObject, eventdata, handles)
handles = guidata(hObject);
global PY
PY = str2double(get(handles.Y, 'String'));
handles.PY=PY
guidata(hObject,handles);

function Y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yatch (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function X_Callback(hObject, eventdata, handles)
handles = guidata(hObject);
global PX
PX = str2double(get(handles.X, 'String'));
handles.PX=PX
guidata(hObject,handles);

function X_CreateFcn(hObject, eventdata, handles)
% hObject    handle to X (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider6_Callback(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --------------------------------------------------------------------
function Untitled_1_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
function quit_callback(robot, handles)
    set(handles.fig, 'ResizeFcn', '');
    delete(handles.panel);
    set(handles.curax, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1])



function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double


% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider5_Callback(hObject, eventdata, handles)
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function YY_Callback(hObject, eventdata, handles)
% hObject    handle to YY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of YY as text
%        str2double(get(hObject,'String')) returns contents of YY as a double


% --- Executes during object creation, after setting all properties.
function YY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to YY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
