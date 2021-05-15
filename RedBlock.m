classdef RedBlock < handle
    properties
        RedBlockPose;
        updatedPoints;
        RedBlockVertexCount;
        RedBlockVerts;
        midPoint;
        RedBlock_h;

    end
   methods
       function self = RedBlock(RedBlockPose)
                self.RedBlockPose = RedBlockPose;
                self.location(self.RedBlockPose);
                
       end
       function location(self,RedBlockPose)
        % After saving in blender then load the triangle mesh
        [f,v,data] = plyread('RedBlock.ply','tri');

        % Get vertex count
        self.RedBlockVertexCount = size(v,1);

        % Move center point to origin
        self.midPoint = sum(v)/self.RedBlockVertexCount;
        self.RedBlockVerts = v - repmat(self.midPoint,self.RedBlockVertexCount,1);


        % Scale the colours to be 0-to-1 (they are originally 0-to-255
        vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

        % Then plot the trisurf
        self.RedBlock_h = trisurf(f,self.RedBlockVerts(:,1),self.RedBlockVerts(:,2), self.RedBlockVerts(:,3) ...
            ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

        self.RedBlockPose = RedBlockPose;
        self.updatedPoints = [self.RedBlockPose * [self.RedBlockVerts,ones(self.RedBlockVertexCount,1)]']';  

        % Now update the Vertices
        self.RedBlock_h.Vertices = self.updatedPoints(:,1:3);

       end
       
       function move(self,NewPose)
      
               
                self.RedBlockPose = NewPose;
                % Transform the vertices
                self.updatedPoints = [self.RedBlockPose * [self.RedBlockVerts,ones(self.RedBlockVertexCount,1)]']';
    
                % Update the mesh vertices in the patch handle
                self.RedBlock_h.Vertices = self.updatedPoints(:,1:3);
                drawnow();   
       end
   end  
end
