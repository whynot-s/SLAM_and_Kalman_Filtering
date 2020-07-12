% The landmark vertex:
% x(1) - x
% x(2) - y

% Note that the oplus method has to be used to account for angular
% discontinuities

classdef LandmarkVertex < g2o.core.BaseVertex
   
    properties(Access = public) %need to be public property
    %put extra properties here.
    landmarkId
    end
    
    methods(Access = public)
        function this = LandmarkVertex(landmarkId) %no values needed to instantiate
            this=this@g2o.core.BaseVertex(2);
            this.landmarkId = landmarkId;
        end
        
        
        function oplus(this, update)
        %probably won't be used because the landmark does not have
        %orientation ...
            % Add the update
            this.x = this.x + update;
            
            % Wrap the angle to [-pi,pi]
            %this.x(3) = g2o.stuff.normalize_theta(this.x(3)); %unneeded as
            %there is no orientation data for the landmarks
        end
    end
end