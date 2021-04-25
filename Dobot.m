classdef Dobot < handle
    properties
        %> Robot model
        model;
        workspace = [-2 2 -2 2 -2 2];           
    end
    
    methods%% Class for UR3 robot simulation
function self = Dobot
    
self.GetDobotRobot();
% robot = 
self.PlotAndColourRobot();%robot,workspace);
end

%% GetDobotRobot
% Given a name (optional), create and return a UR3 robot model
function GetDobotRobot(self)

L1 = Link('d',0.03,'a',0,'alpha',pi/2,'qlim',[-pi/2 pi/2])
 
L2 = Link('d',0,'a',0.135,'alpha',0,'qlim',[deg2rad(0),deg2rad(85)])

L3 = Link('d',0,'a',0.147,'alpha',0,'qlim',[deg2rad(0),deg2rad(105)])

L4 = Link('d',0,'a',0.1,'alpha',0,'qlim',[-pi/2 pi/2])

L2.offset=deg2rad(-5)
L3.offset=deg2rad(-105)
myRobot = SerialLink([L1 L2 L3 L4], 'name', 'Puma560')



      
q = zeros(1,4)

myRobot.plot(q)


    self.model = SerialLink([L1 L2 L3 L4],'name','Dobot');
    self.model.base = self.model.base * transl(-0.5,0,0);
end
%% PlotAndColourRobot
% Given a robot index, add the glyphs (vertices and faces) and
% colour them in if data is available 
function PlotAndColourRobot(self)%robot,workspace)
    for linkIndex = 0:self.model.n
           [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['DobotLink',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
        self.model.faces{linkIndex+1} = faceData;
        self.model.points{linkIndex+1} = vertexData;
    end

    % Display robot
    self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
    if isempty(findobj(get(gca,'Children'),'Type','Light'))
        camlight
    end  
    self.model.delay = 0;

    % Try to correctly colour the arm (if colours are in ply file data)
    for linkIndex = 0:self.model.n
        handles = findobj('Tag', self.model.name);
        h = get(handles,'UserData');
        try 
            h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                          , plyData{linkIndex+1}.vertex.green ...
                                                          , plyData{linkIndex+1}.vertex.blue]/255;
            h.link(linkIndex+1).Children.FaceColor = 'interp';
        catch ME_1
            disp(ME_1);
            continue;
        end
    end
end        
    end
end
