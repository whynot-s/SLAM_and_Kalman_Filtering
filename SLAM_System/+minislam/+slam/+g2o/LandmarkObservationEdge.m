classdef LandmarkObservationEdge < g2o.core.BaseUnaryEdge %this is adapted from the GPSMeasurementEdge code on Github

    methods(Access = public)
        
        function this = LandmarkObservationEdge()
            this = this@g2o.core.BaseUnaryEdge(2); %I believe that this is the size of the measurement vector.
%             disp(this.edgeVertices{1})
%             disp(this.owningGraph)
        end
        
        function computeError(this)
            %disp("in compute error, edge vertices and owning graph")
            x = this.edgeVertices{1}.estimate(); %the vertex that it will be attached to is the landmark, therefore
            x(2) = g2o.stuff.normalize_theta(x(2));
            %x will only be two-long i believe
            this.errorZ = x - this.z; %the current measurement is this.z and throughout multiple measurements, x will change
        end
        
        function linearizeOplus(this)
            this.J{1} = ...
                [-1 0 ; %changed this because it's only a two long vector, don't need to disregard the last value.
                0 -1 ];
        end        
    end
end