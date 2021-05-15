classdef GreenBlock < handle
    properties
        GreenBlockPose;
        updatedPoints;
        GreenBlockVertexCount;
        GreenBlockVerts;
        midPoint;
        GreenBlock_h;

    end
   methods
       function self = GreenBlock(GreenBlockPose)
                self.GreenBlockPose = GreenBlockPose;
                self.location(self.GreenBlockPose);
                
       end
       function location(self,GreenBlockPose)
        % After saving in blender then load the triangle mesh
        [f,v,data] = plyread('GreenBlock.ply','tri');

        % Get vertex count
        self.GreenBlockVertexCount = size(v,1);

        % Move center point to origin
        self.midPoint = sum(v)/self.GreenBlockVertexCount;
        self.GreenBlockVerts = v - repmat(self.midPoint,self.GreenBlockVertexCount,1);


        % Scale the colours to be 0-to-1 (they are originally 0-to-255
        vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

        % Then plot the trisurf
        self.GreenBlock_h = trisurf(f,self.GreenBlockVerts(:,1),self.GreenBlockVerts(:,2), self.GreenBlockVerts(:,3) ...
            ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

        self.GreenBlockPose = GreenBlockPose;
        self.updatedPoints = [self.GreenBlockPose * [self.GreenBlockVerts,ones(self.GreenBlockVertexCount,1)]']';  

        % Now update the Vertices
        self.GreenBlock_h.Vertices = self.updatedPoints(:,1:3);

       end
       
       function move(self,NewPose)
      
               
                self.GreenBlockPose = NewPose;
                % Transform the vertices
                self.updatedPoints = [self.GreenBlockPose * [self.GreenBlockVerts,ones(self.GreenBlockVertexCount,1)]']';
    
                % Update the mesh vertices in the patch handle
                self.GreenBlock_h.Vertices = self.updatedPoints(:,1:3);
                drawnow();   
       end
   end  
end
