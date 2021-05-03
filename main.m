%% Enviroment

% Fence
[f,v,data] = plyread('fence.ply','tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;

fence_h = trisurf(f,v(:,1),v(:,2),v(:,3)...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

hold on

% Table
[f,v,data] = plyread('table.ply','tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;

table_h = trisurf(f,v(:,1),v(:,2),v(:,3)...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
hold on

% Worker
[f,v,data] = plyread('worker.ply','tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;

worker_h = trisurf(f,v(:,1),v(:,2),v(:,3)...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

axis([-5,5,-5,5,0,5]);
axis equal;

% Fire Extinguisher
[f,v,data] = plyread('FireExtinguisher.ply','tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;

FireExtinguisher_h = trisurf(f,v(:,1),v(:,2),v(:,3)...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

% Button
[f,v,data] = plyread('button.ply','tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;

button_h = trisurf(f,v(:,1),v(:,2),v(:,3)...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

% Base
surf([-5,-5;5,5],[-5,5;-5,5],[0.01,0.01;0.01,0.01],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
hold on


% Box
[f,v,data] = plyread('Redbox.ply','tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;

Redbox_h = trisurf(f,v(:,1),v(:,2),v(:,3)...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

[f,v,data] = plyread('Greenbox.ply','tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;

Greenbox_h = trisurf(f,v(:,1),v(:,2),v(:,3)...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');


% Dobot
robot = Dobot;

% RedBlock

Red = RedBlock(transl(-0.25,0,0.75));
Green = GreenBlock(transl(-0.3,0,0.75));