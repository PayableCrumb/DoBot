classdef Dobot < handle
    properties
        %> robot model
        model;
        workspace = [-1 1 -1 1 -0.3 1]; 
        
        %> Joint 2: Actual vs Suggested real joint limitsactual joint limits less than 0 cause problems with IMU (officially     qlimDegs = [-5,85], suggest it is better to be [5,80] )
        actualRealQ2lim = deg2rad([-5,85]);
        suggestedRealQ2lim = deg2rad([5,80]);
        %> Joint 3: Actual vs Suggested real joint limitsactual joint limits less than 0 (or more than 90) cause problems with IMU (officially q2limDegs = -10 to 95 )
        actualRealQ3lim = deg2rad([-10,95]);
        suggestedRealQ3lim = deg2rad([5,85]);
    end

    methods%% Class for Dobot simulation
function self = Dobot
    self.GetDobot();
    self.PlotAndColourRobot();
%     self.PlotLimits();
end
%% Dobot model
function GetDobot(self)
    L1 = Link('d',0.08,  'a',0,       'alpha',-pi/2);
    L2 = Link('d',0,     'a',0.135,   'alpha',0);
    L3 = Link('d',0,     'a',0.147,   'alpha',0);
    L4 = Link('d',0,     'a',0.08,    'alpha',-pi/2);
    L5 = Link('d',0,     'a',0,       'alpha',0);
    
%     L1.qlim = [-360 360]*pi/180;
%     L2.qlim = [-90 90]*pi/180;
%     L3.qlim = [-160 160]*pi/180;
%     L4.qlim = [-360 360]*pi/180;
%     L5.qlim = [-360 360]*pi/180;
%     L6.qlim = [-360 360]*pi/180;
% 
%     L2.offset = -pi/4;
%     L3.offset = pi/2;
      
    self.model = SerialLink([L1 L2 L3 L4 L5],'name','Dobot');
    self.model.base = eye(4);
%      self.model.plot(zeros(1,5));
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
%% create joint limits for Dobot
function PlotLimits(self)
    titleStr = {'Model limits given ACTUAL real robot limits','Model limits given SUGGESTED real robot limits'};
    qlimActualAndSuggested = {self.model.qlim,self.model.qlim};
    qlimActualAndSuggested{1}(2,:) = self.actualRealQ2lim;
    qlimActualAndSuggested{1}(3,:) = self.actualRealQ3lim;
    qlimActualAndSuggested{2}(2,:) = self.suggestedRealQ2lim;
    qlimActualAndSuggested{2}(3,:) = self.suggestedRealQ3lim;            

    fig_h = figure;
    for limitsIndex = 1:size(titleStr,2)
        clf(fig_h);
        data = [];
        lowerLimit = [];
        upperLimit = [];
    
        qlim = qlimActualAndSuggested{ limitsIndex };
        for q2 = qlim(2,1):0.01:qlim(2,2)
            for theta3 = qlim(3,1):0.01:qlim(3,2)+0.01
                q3 = pi/2 -q2 + theta3;
                data = [data;q2,q3]; %#ok<AGROW>
                if theta3 <= qlim(3,1)
                    lowerLimit = [lowerLimit;q2,q3]; %#ok<AGROW>
                elseif qlim(3,2) <= theta3
                    upperLimit = [upperLimit;q2,q3]; %#ok<AGROW>
                end
            end
        end
    plot(rad2deg(data(:,1)),rad2deg(data(:,2)),'g.');
    hold on;
    plot(rad2deg(lowerLimit(:,1)),rad2deg(lowerLimit(:,2)),'b');
    plot(rad2deg(upperLimit(:,1)),rad2deg(upperLimit(:,2)),'r');
    title(titleStr{limitsIndex})
    set(gca,'fontSize',15)
    xlabel('q2')
    ylabel('q3')
    grid on;
        axis([-10,90,-15,200]);
        drawnow();
        img = getframe(gcf);
        imwrite(img.cdata, [titleStr{limitsIndex}, '.jpg']);
    end
    close(fig_h);
end
    end
end